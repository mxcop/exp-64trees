#include "precomp.h"
#include "cwbvh.h"

/* 2-wide BVH node. */
struct Bvh2Node {
	/* Minimum bounds */
	float3 min_bounds = 0.0f;
	/* Left child node, or first primitive index */
	uint32_t left_first = 0u;
	/* Maximum bounds */
	float3 max_bounds = 0.0f;
	/* Primitive count */
	uint32_t prim_count = 0u;

	/* Returns true if this node is a leaf node. */
	bool is_leaf() const { return prim_count > 0; }
	/* Returns the surface area of this node. */
	float surface_area() const { 
		const float extent_x = max_bounds.x - min_bounds.x;
		const float extent_y = max_bounds.y - min_bounds.y;
		const float extent_z = max_bounds.z - min_bounds.z;
		return extent_x * extent_y + extent_y * extent_z + extent_z * extent_x;
	}
};

constexpr int BVH_BINS = 8u;
constexpr float BVH_FAR = 1e30f;

inline float half_area(const float3& v) { return v.x < -BVH_FAR ? 0 : (v.x * v.y + v.y * v.z + v.z * v.x); }

void CwBvh::build(const Aabb* input_prims, const uint32_t input_count) {
	
	/* Calculate the number of nodes we need and allocate space for them */
	const uint32_t nodes_needed = input_count * 2u;
	Bvh2Node* bvh2_nodes = new Bvh2Node[nodes_needed] {};
	uint32_t node_ptr = 2u; /* Skip the 2nd node for better cache-line alignment */
	indices = new uint32_t[input_count] {};
	for (uint32_t i = 0u; i < input_count; ++i) indices[i] = i;
	prims = new Aabb[input_count] {};
	memcpy(prims, input_prims, input_count * sizeof(Aabb));

	/* Setup the root node for the 2-wide tree */
	Bvh2Node& bvh2_root = bvh2_nodes[0];
	bvh2_root.left_first = 0u; 
	bvh2_root.prim_count = input_count;
	bvh2_root.min_bounds = BVH_FAR;
	bvh2_root.max_bounds = -BVH_FAR;
	for (uint32_t i = 0u; i < input_count; ++i) {
		bvh2_root.min_bounds.x = fminf(bvh2_root.min_bounds.x, prims[i].min.x);
		bvh2_root.min_bounds.y = fminf(bvh2_root.min_bounds.y, prims[i].min.y);
		bvh2_root.min_bounds.z = fminf(bvh2_root.min_bounds.z, prims[i].min.z);
		bvh2_root.max_bounds.x = fmaxf(bvh2_root.max_bounds.x, prims[i].max.x);
		bvh2_root.max_bounds.y = fmaxf(bvh2_root.max_bounds.y, prims[i].max.y);
		bvh2_root.max_bounds.z = fmaxf(bvh2_root.max_bounds.z, prims[i].max.z);
	}

	/* Build a 2-wide tree */
	uint32_t tasks[256] {}, task_count = 0u;
	float3 min_extent = (bvh2_root.max_bounds - bvh2_root.min_bounds) * 1e-20f;
	float3 best_lmin {}, best_lmax {}, best_rmin {}, best_rmax {};

	for (;;) {
		for (;;) {
			Bvh2Node& node = bvh2_nodes[node_ptr];

			/* Initialize the bins */
			float3 bin_min[3][BVH_BINS]{}, bin_max[3][BVH_BINS]{};
			for (uint32_t a = 0; a < 3u; ++a) {
				for (uint32_t i = 0; i < BVH_BINS; ++i) {
					bin_min[a][i] = float3(BVH_FAR);
					bin_max[a][i] = float3(-BVH_FAR);
				}
			}

			/* Scatter the primitives into the bins */
			uint32_t bin_cnt[3][BVH_BINS]{};
			const float3 rpd3 = float3(float3(BVH_BINS) / (node.max_bounds - node.min_bounds));
			const float3 nmin3 = node.min_bounds;
			for (uint32_t i = 0u; i < node.prim_count; ++i) {
				const Aabb& prim = prims[indices[node.left_first + i]];
				int3 bi = int3(((prim.min + prim.max) * 0.5f - nmin3) * rpd3);
				bi.x = clamp(bi.x, 0, BVH_BINS - 1);
				bi.y = clamp(bi.y, 0, BVH_BINS - 1);
				bi.z = clamp(bi.z, 0, BVH_BINS - 1);
				bin_min[0][bi.x] = fminf(bin_min[0][bi.x], prim.min);
				bin_max[0][bi.x] = fmaxf(bin_max[0][bi.x], prim.max), bin_cnt[0][bi.x]++;
				bin_min[1][bi.y] = fminf(bin_min[1][bi.y], prim.min);
				bin_max[1][bi.y] = fmaxf(bin_max[1][bi.y], prim.max), bin_cnt[1][bi.y]++;
				bin_min[2][bi.z] = fminf(bin_min[2][bi.z], prim.min);
				bin_max[2][bi.z] = fmaxf(bin_max[2][bi.z], prim.max), bin_cnt[2][bi.z]++;
			}

			/* Calculate the cost for each split */
			float split_cost = BVH_FAR;
			uint32_t best_axis = 0u, best_split = 0u;
			for (int32_t a = 0; a < 3; ++a) {
				/* Skip if the node is too small */
				if ((node.max_bounds[a] - node.min_bounds[a]) <= min_extent[a]) continue;

				/* Evaluate the cost of each bin configuration */
				float3 lb_min[BVH_BINS - 1]{}, rb_min[BVH_BINS - 1]{}, l1{ BVH_FAR }, l2{ -BVH_FAR };
				float3 lb_max[BVH_BINS - 1]{}, rb_max[BVH_BINS - 1]{}, r1{ BVH_FAR }, r2{ -BVH_FAR };
				float l_cost[BVH_BINS - 1]{}, r_cost[BVH_BINS - 1]{};
				for (uint32_t ln = 0u, rn = 0u, i = 0u; i < BVH_BINS - 1; ++i) {
					lb_min[i] = l1 = fminf(l1, bin_min[a][i]);
					rb_min[BVH_BINS - 2 - i] = r1 = fminf(r1, bin_min[a][BVH_BINS - 1 - i]);
					lb_max[i] = l2 = fmaxf(l2, bin_max[a][i]);
					rb_max[BVH_BINS - 2 - i] = r2 = fmaxf(r2, bin_max[a][BVH_BINS - 1 - i]);
					ln += bin_cnt[a][i], rn += bin_cnt[a][BVH_BINS - 1 - i];
					l_cost[i] = ln == 0 ? BVH_FAR : (half_area(l2 - l1) * (float)ln);
					r_cost[BVH_BINS - 2 - i] = rn == 0 ? BVH_FAR : (half_area(r2 - r1) * (float)rn);
				}

				/* Find the split with the lowest cost */
				for (uint32_t i = 0u; i < BVH_BINS - 1; ++i) {
					const float cost = l_cost[i] + r_cost[i];
					if (cost >= split_cost) continue;

					split_cost = cost, best_axis = a, best_split = i;
					best_lmin = lb_min[i], best_rmin = rb_min[i], best_lmax = lb_max[i], best_rmax = rb_max[i];
				}
			}

			/* Calculate the actual split cost */
			split_cost = 1.0f + split_cost / node.surface_area();

			/* Break if the split is not worth it */
			if (split_cost >= node.prim_count) {
				break;
			}

			/* Sort the primitve indices */
			uint32_t j = node.left_first + node.prim_count, src = node.left_first;
			const float rpd = rpd3[best_axis], nmin = nmin3[best_axis];
			for (uint32_t i = 0u; i < node.prim_count; ++i) {
				const Aabb& prim = prims[indices[src]];
				int32_t bi = (uint32_t)(((prim.min[best_axis] + prim.max[best_axis]) * 0.5f - nmin) * rpd);
				bi = clamp(bi, 0, BVH_BINS - 1);
				if ((uint32_t)bi <= best_split) src++;
				else swap(indices[src], indices[--j]);
			}

			/* Create the two child nodes */
			const uint32_t left_cnt = src - node.left_first, right_cnt = node.prim_count - left_cnt;
			if (left_cnt == 0u || right_cnt == 0u) break;
			const uint32_t n = node_ptr;
			node_ptr += 2u;
			bvh2_nodes[n].min_bounds = best_lmin, bvh2_nodes[n].max_bounds = best_lmax;
			bvh2_nodes[n].left_first = node.left_first, bvh2_nodes[n].prim_count = left_cnt;
			bvh2_nodes[n + 1].min_bounds = best_rmin, bvh2_nodes[n + 1].max_bounds = best_rmax;
			bvh2_nodes[n + 1].left_first = j, bvh2_nodes[n + 1].prim_count = right_cnt;
			node.left_first = n, node.prim_count = 0u;

			/* Add one of the two child nodes onto the build stack */
			tasks[task_count++] = n + 1, node_ptr = n;
		}

		/* Fetch task from the build stack */
		if (task_count == 0) break; else node_ptr = tasks[--task_count];
	}

	/* Convert the 2-wide tree to 8-wide */


	/* Quantize the 8-wide tree into a compressed wide tree */


}

inline float2 intersect_aabb(const float3 origin, const float3 invDir, const float3 bbMin, const float3 bbMax) {
	float3 t0 = (bbMin - origin) * invDir;
	float3 t1 = (bbMax - origin) * invDir;

	const float3 temp = t0;
	t0 = fminf(temp, t1), t1 = fmaxf(temp, t1);

	const float tmin = fmaxf(fmaxf(fmaxf(t0.x, t0.y), t0.z), 0.0f);
	const float tmax = fminf(fminf(t1.x, t1.y), t1.z);

	return float2(tmin, tmax);
}

CwBvh::~CwBvh() {
	delete[] nodes;
	delete[] prims;
}
