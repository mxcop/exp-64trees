#include "precomp.h"
#include "cwbvh.h"

constexpr int BVH_BINS = 4u;
constexpr float BVH_FAR = 1e30f;

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

/* 8-wide BVH node. */
struct Bvh8Node {
	/* Minimum bounds */
	float3 min_bounds = 0.0f;
	/* Left child node, or first primitive index */
	uint32_t first_prim = 0u;
	/* Maximum bounds */
	float3 max_bounds = 0.0f;
	/* Primitive count */
	uint32_t prim_count = 0u;
	/* Child nodes */
	uint32_t children[8] {};
	uint32_t child_count = 0u;

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

/* Compressed 8-wide BVH node. */
struct CwBvhNode {
	/* Minimum bounds of this node. */
	float min_bounds_x = 0.0f;
	float min_bounds_y = 0.0f;
	float min_bounds_z = 0.0f;

	/* Logarithm of the extent of this node. */
	int8_t log_extent_x = 0;
	int8_t log_extent_y = 0;
	int8_t log_extent_z = 0;
	uint8_t intersection_mask = 0u;

	/* Index of the first child node. (children are stored contiguously) */
	uint32_t child_base_index = 0u;
	/* Index of the first primitive. (primitives are stored contiguously) */
	uint32_t prim_base_index = 0u;

	/* Child node metadata. */
	uint8_t meta[8] {};
	/* Quantized child node bounding boxes. (40 bytes) */
	uint8_t lo_x[8] {};
	uint8_t lo_y[8] {};
	uint8_t lo_z[8] {};
	uint8_t hi_x[8] {};
	uint8_t hi_y[8] {};
	uint8_t hi_z[8] {};
};

inline float half_area(const float3& v) { return v.x < -BVH_FAR ? 0 : (v.x * v.y + v.y * v.z + v.z * v.x); }

void CwBvh::build(const Aabb* input_prims, const uint32_t input_count) {
	/* Calculate how many nodes we might need */
	const uint32_t nodes_needed = input_count * 2u;

	/* Allocate BVH2 nodes */
	bvh2_nodes = new Bvh2Node[nodes_needed] {};
	
	/* Copy the input primitives */
	prims = new Aabb[input_count] {};
	memcpy(prims, input_prims, input_count * sizeof(Aabb));
	bvh2_indices = new uint32_t[input_count] {};
	cwbvh_indices = new uint32_t[input_count] {};
	for (uint32_t i = 0u; i < input_count; ++i) bvh2_indices[i] = i;
	memcpy(cwbvh_indices, bvh2_indices, input_count * sizeof(uint32_t));

	/* Build the 2-wide BVH */
	bvh2_node_count = 2u; /* Skip the 2nd node for better cache-line alignment */
	build_bvh2(prims, input_count);

	/* Allocate CWBVH nodes */
	cwbvh_nodes = new CwBvhNode[bvh2_node_count] {};
	cwbvh_node_count = 0u;

	/* Convert 2-wide BVH to CWBVH */
	bvh2_to_cwbvh();
}

void CwBvh::print() const {
	

#if 1
	{
		uint32_t stack[32]{};
		uint32_t stack_ptr = 0u;
		uint32_t node_index = 0u;
	
		for (;;) {
			const Bvh8Node& node = bvh8_nodes[node_index];

			printf("node %d: (%d, %d)\n", node_index, node.child_count, node.prim_count);

			for (uint32_t i = 0; i < 8; i++) {
				if (node.children[i] == 0u) continue; /* Empty node */

				const Bvh8Node& child = bvh8_nodes[node.children[i]];
				const bool is_leaf = child.is_leaf();

				if (is_leaf) {
					//const uint32_t prim_bits = node.qbounds[i].meta >> 5;
					//const uint32_t prim_count = prim_bits == 0b001 ? 1 : prim_bits == 0b011 ? 2 : 3;
					printf("[%d] leaf %d..%d\n", i, child.first_prim, child.first_prim + child.prim_count);
				} else {
					const uint32_t child_index = node.children[i];
					printf("[%d] node %d\n", i, child_index);
					stack[stack_ptr++] = child_index;
				}
			}

			if (stack_ptr == 0u) {
				break;
			} else {
				node_index = stack[--stack_ptr];
			}
		}
	}
#endif
	printf("\n");

	uint32_t stack[32]{};
	uint32_t stack_ptr = 0u;
	uint32_t node_index = 0u;
	for (;;) {
		const CwBvhNode& node = cwbvh_nodes[node_index];

		printf("node %d:\n", node_index);

		for (uint32_t i = 0; i < 8; i++) {
			if (node.meta[i] == 0u) continue; /* Empty node */

			const bool is_leaf = (node.intersection_mask & (1u << i)) == 0;

			if (is_leaf) {
				// Leaf node - test primitives (following tinybvh)
				const uint32_t primBits = node.meta[i] >> 5;
				const uint32_t primCount = primBits == 0b001 ? 1 :
					primBits == 0b011 ? 2 : 3;
				const uint32_t leafIdx = node.meta[i] & 0x1f;
				const uint32_t primBase = node.prim_base_index;

				// Calculate actual primitive offset
				uint32_t primOffset = 0;
				for (uint32_t j = 0; j < i; j++) {
					if ((node.intersection_mask & (1u << j)) == 0 &&
						node.meta[j] != 0) {
						const uint32_t pb = node.meta[j] >> 5;
						primOffset += pb == 0b001 ? 1 : pb == 0b011 ? 2 : 3;
					}
				}

				//const uint32_t prim_bits = node.qbounds[i].meta >> 5;
				//const uint32_t prim_count = prim_bits == 0b001 ? 1 : prim_bits == 0b011 ? 2 : 3;
				printf("[%d] leaf %d..%d\n", i, primBase + primOffset, primBase + primOffset + primCount);
			} else {
				const uint32_t child_index = node.child_base_index + ((node.meta[i] & 0b11111u) - 24u);
				printf("[%d] node %d\n", i, child_index);
				stack[stack_ptr++] = child_index;
			}
		}

		if (stack_ptr == 0u) {
			break;
		}
		else {
			node_index = stack[--stack_ptr];
		}
	}
}

void CwBvh::build_bvh2(const Aabb* input_prims, const uint32_t input_count) {
	/* Setup the root node for the BVH */
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

	/* Build the BVH */
	uint32_t build_tasks[256]{}, task_count = 0u, node_ptr = 0u;
	const float3 min_extent = (bvh2_root.max_bounds - bvh2_root.min_bounds) * 1e-20f;
	float3 best_lmin{}, best_lmax{}, best_rmin{}, best_rmax{};
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
				const Aabb& prim = prims[bvh2_indices[node.left_first + i]];
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

			/* Break if the split is not worth it (only if 3 or less primitives) */
			if (node.prim_count < 4 && split_cost >= node.prim_count) {
				break;
			}

			/* Sort the primitve indices */
			uint32_t j = node.left_first + node.prim_count, src = node.left_first;
			const float rpd = rpd3[best_axis], nmin = nmin3[best_axis];
			for (uint32_t i = 0u; i < node.prim_count; ++i) {
				const Aabb& prim = prims[bvh2_indices[src]];
				int32_t bi = (uint32_t)(((prim.min[best_axis] + prim.max[best_axis]) * 0.5f - nmin) * rpd);
				bi = clamp(bi, 0, BVH_BINS - 1);
				if ((uint32_t)bi <= best_split) src++;
				else swap(bvh2_indices[src], bvh2_indices[--j]);
			}

			/* Create the two child nodes */
			const uint32_t left_cnt = src - node.left_first, right_cnt = node.prim_count - left_cnt;
			if (left_cnt == 0u || right_cnt == 0u) break;
			const uint32_t n = bvh2_node_count;
			bvh2_node_count += 2u;
			bvh2_nodes[n].min_bounds = best_lmin, bvh2_nodes[n].max_bounds = best_lmax;
			bvh2_nodes[n].left_first = node.left_first, bvh2_nodes[n].prim_count = left_cnt;
			bvh2_nodes[n + 1].min_bounds = best_rmin, bvh2_nodes[n + 1].max_bounds = best_rmax;
			bvh2_nodes[n + 1].left_first = j, bvh2_nodes[n + 1].prim_count = right_cnt;
			node.left_first = n, node.prim_count = 0u;

			/* Add one of the two child nodes onto the build stack */
			build_tasks[task_count++] = n + 1, node_ptr = n;
		}

		/* Fetch task from the build stack */
		if (task_count == 0) break; else node_ptr = build_tasks[--task_count];
	}
}

void CwBvh::bvh2_to_cwbvh() {
	/* Create a temporary BVH8 */
	bvh8_nodes = new Bvh8Node[std::max(2u, bvh2_node_count)] {};

	/* Mirror the BVH2 nodes */
	for (uint32_t i = 0u; i < bvh2_node_count; i++) {
		if (i == 1u) continue; /* Skip node number 1, it is invalid. */
	
		Bvh2Node& orig = bvh2_nodes[i];
		Bvh8Node& node = bvh8_nodes[i];
		node.min_bounds = orig.min_bounds, node.max_bounds = orig.max_bounds;
		if (orig.is_leaf()) node.prim_count = orig.prim_count, node.first_prim = orig.left_first;
		else node.children[0] = orig.left_first, node.children[1] = orig.left_first + 1u, node.child_count = 2u;
	}
	
	{ /* Collapse the BVH to use all 8 children */
		uint32_t stack[128] {};
		uint32_t stack_ptr = 0u, node_index = 0u;
		for (;;) {
			Bvh8Node& node = bvh8_nodes[node_index];

			for (;node.child_count < 8u;) {
				/* Try to find a child to collapse */
				uint32_t best_child = 256u;
				float best_child_sa = 0.0f;
				for (uint32_t i = 0u; i < node.child_count; ++i) {
					const Bvh8Node& child = bvh8_nodes[node.children[i]];
					if ((child.is_leaf() == false) && (node.child_count - 1u + child.child_count) <= 8u) {
						const float surface_area = child.surface_area();
						if (surface_area > best_child_sa) best_child = i, best_child_sa = surface_area;
					}
				}

				/* Did not find a suitable child to collapse */
				if (best_child == 256u) break;

				/* Collapse the child node */
				const Bvh8Node& child = bvh8_nodes[node.children[best_child]];
				for (uint32_t i = 1u; i < child.child_count; ++i) {
					node.children[node.child_count++] = child.children[i];
				}
				node.children[best_child] = child.children[0];
			}

			/* After collapsing this node, continue with its children */
			for (uint32_t i = 0u; i < node.child_count; ++i) {
				const uint32_t child_index = node.children[i];
				const Bvh8Node& child = bvh8_nodes[child_index];
				if (child.is_leaf() == false) stack[stack_ptr++] = child_index;
			}

			/* Traverse down the stack */
			if (stack_ptr == 0u) break;
			node_index = stack[--stack_ptr];
		}
	}

	/* Handle the special case where the root node is a leaf node */
	Bvh8Node& root_node = bvh8_nodes[0];
	if (root_node.is_leaf()) {
		bvh8_nodes[1] = root_node;
		root_node.child_count = 1u;
		root_node.children[0] = 1u;
		root_node.prim_count = 0u;
	}

	/* Compress the BVH8 into the CWBVH format */
	/* <https://research.nvidia.com/sites/default/files/publications/ylitie2017hpg-paper.pdf> */
    uint32_t src_stack[256] {}, dst_stack[256] {};
    uint32_t stack_ptr = 1u, node_ptr = 1u, prim_ptr = 0u;
    for (; stack_ptr > 0u;) {
        /* Get the source and destination nodes */
        const Bvh8Node& src = bvh8_nodes[src_stack[--stack_ptr]];
        CwBvhNode& dst = cwbvh_nodes[dst_stack[stack_ptr]];
		const float3 node_center = (src.min_bounds + src.max_bounds) * 0.5f;

		/* Octant-based child ordering for traversal coherence */
		/* Calculate edge weights for bipartite matching */
		float edge_w[8][8] {};
		uint32_t assignment[8] {};
		bool slot_empty[8] {};
		for (uint32_t s = 0u; s < 8u; ++s) {
			assignment[s] = 256u;
			slot_empty[s] = true;

			const float3 octant_dir(
				(((s >> 2u) & 1u) == 1u) ? -1.0f : 1.0f,
				(((s >> 1u) & 1u) == 1u) ? -1.0f : 1.0f,
				(((s >> 0u) & 1u) == 1u) ? -1.0f : 1.0f
			);
			for (uint32_t i = 0u; i < 8u; ++i) {
				if (src.children[i] == 0u) {
					edge_w[s][i] = BVH_FAR;
					continue;
				}

				/* Calculate the cost for this child/octant combination */
				const Bvh8Node& child = bvh8_nodes[src.children[i]];
				const float3 child_center = (child.min_bounds + child.max_bounds) * 0.5f;
				edge_w[s][i] = dot(child_center - node_center, octant_dir);
			}
		}

		/* Greedy bipartite matching, assigning children to octants */
		for (;;) {
			float min_cost = BVH_FAR;
			uint32_t min_entry_x = 256u, min_entry_y = 256u;
			for (uint32_t s = 0u; s < 8u; ++s) {
				for (uint32_t i = 0u; i < 8u; ++i) {
					if (assignment[i] == 256u && slot_empty[s] && edge_w[s][i] < min_cost) {
						min_cost = edge_w[s][i], min_entry_x = s, min_entry_y = i;
					}
				}
			}
			if (min_entry_x == 256u && min_entry_y == 256u) break;
			slot_empty[min_entry_x] = false, assignment[min_entry_y] = min_entry_x;
		}

		/* Fill in remaining slots */
		for (uint32_t i = 0u; i < 8u; ++i) {
			if (assignment[i] != 256u) continue;
			for (uint32_t s = 0u; s < 8u; ++s) {
                if (slot_empty[s] == false) continue;
				slot_empty[s] = false, assignment[i] = s;
				break;
			}
		}

		/* Assign the children to their new slots */
		uint32_t children[8] {};
		for (uint32_t i = 0u; i < 8u; ++i) children[assignment[i]] = src.children[i];

		/* Child bounds quantization parameters */
        dst.log_extent_x = (int8_t)ceilf(log2f((src.max_bounds.x - src.min_bounds.x) * (1.0f / 255.0f)));
        dst.log_extent_y = (int8_t)ceilf(log2f((src.max_bounds.y - src.min_bounds.y) * (1.0f / 255.0f)));
        dst.log_extent_z = (int8_t)ceilf(log2f((src.max_bounds.z - src.min_bounds.z) * (1.0f / 255.0f)));
		const float rq_x = 1.0f / exp2f((float)dst.log_extent_x);
		const float rq_y = 1.0f / exp2f((float)dst.log_extent_y);
		const float rq_z = 1.0f / exp2f((float)dst.log_extent_z);
		dst.min_bounds_x = src.min_bounds.x;
		dst.min_bounds_y = src.min_bounds.y;
		dst.min_bounds_z = src.min_bounds.z;

		/* Quantize the child nodes */
        uint32_t leaf_prim_count = 0u;
        dst.child_base_index = 0xFFFFFFFFu;
        dst.prim_base_index = 0xFFFFFFFFu;
        dst.intersection_mask = 0u;
		for (uint32_t i = 0u; i < 8u; ++i) {
			/* Clear empty slots */
			if (children[i] == 0u) {
				dst.meta[i] = 0u;
				dst.lo_x[i] = dst.lo_y[i] = dst.lo_z[i] = 0u;
				dst.hi_x[i] = dst.hi_y[i] = dst.hi_z[i] = 0u;
				continue;
			}

			/* Calculate quantized bounding box */
			Bvh8Node& child = bvh8_nodes[children[i]];
			dst.lo_x[i] = (uint8_t)floorf((child.min_bounds.x - src.min_bounds.x) * rq_x);
			dst.lo_y[i] = (uint8_t)floorf((child.min_bounds.y - src.min_bounds.y) * rq_y);
			dst.lo_z[i] = (uint8_t)floorf((child.min_bounds.z - src.min_bounds.z) * rq_z);
			dst.hi_x[i] = (uint8_t)ceilf((child.max_bounds.x - src.min_bounds.x) * rq_x);
			dst.hi_y[i] = (uint8_t)ceilf((child.max_bounds.y - src.min_bounds.y) * rq_y);
			dst.hi_z[i] = (uint8_t)ceilf((child.max_bounds.z - src.min_bounds.z) * rq_z);

			if (child.is_leaf()) {
				/* Unary encode primitive count */
                if (dst.prim_base_index == 0xFFFFFFFFu) dst.prim_base_index = prim_ptr;
				const uint32_t unaryEncodedTriCount = child.prim_count == 1u ? 0b001u : child.prim_count == 2u ? 0b011u : 0b111u;

				/* Set the child metadata */
                dst.meta[i] = (uint8_t)((unaryEncodedTriCount << 5u) | leaf_prim_count);

				/* Copy over primitive indices */
                leaf_prim_count += child.prim_count;
				for (uint32_t j = 0u; j < child.prim_count; ++j) {
					cwbvh_indices[prim_ptr++] = bvh2_indices[child.first_prim + j];
				}
			} else {
                const uint32_t new_index = node_ptr++;
                if (dst.child_base_index == 0xFFFFFFFFu) dst.child_base_index = new_index;
                dst.intersection_mask |= (1u << i);

				/* Set the child metadata */
				dst.meta[i] = (1u << 5u) | (24u + (uint8_t)i);

				/* Push the child onto the stack */
                src_stack[stack_ptr] = children[i];
                dst_stack[stack_ptr++] = new_index;
			}
		}
	}

	//delete[] bvh8_nodes;
}

inline float intersect_aabb(const Ray ray, const float3 box_min, const float3 box_max) {
	const float3 t_to_min = (box_min - ray.O) * ray.rD;
	const float3 t_to_max = (box_max - ray.O) * ray.rD;
	const float3 t_min = fminf(t_to_min, t_to_max);
	const float3 t_max = fmaxf(t_to_min, t_to_max);
	const float t_near = fmaxf(fmaxf(fmaxf(t_min.x, t_min.y), t_min.z), 0);
	const float t_far = fminf(fminf(fminf(t_max.x, t_max.y), t_max.z), ray.t);

	return t_near > t_far ? BVH_FAR : t_near;
}

LeafHit CwBvh::trace_bvh2(const Ray& ray) const {
	Bvh2Node* node = &bvh2_nodes[0], *stack[32] {};
	uint32_t stack_ptr = 0u;
	float closest_dist = BVH_FAR;
	uint16_t steps = 0u;

	for (;;) {
		if (node->is_leaf()) {
			for (uint32_t i = 0u; i < node->prim_count; ++i) {
				const Aabb& prim = prims[bvh2_indices[node->left_first + i]];
				const float dist = intersect_aabb(ray, prim.min, prim.max);
				steps += 1;
				if (dist < closest_dist) {
					closest_dist = dist;
				}
			}
			if (stack_ptr == 0) break; else node = stack[--stack_ptr];
			continue;
		}
		Bvh2Node* child1 = &bvh2_nodes[node->left_first], *child2 = &bvh2_nodes[node->left_first + 1];
		float dist1 = intersect_aabb(ray, child1->min_bounds, child1->max_bounds);
		float dist2 = intersect_aabb(ray, child2->min_bounds, child2->max_bounds);
		steps += 2;

		if (dist1 > dist2) { swap(dist1, dist2); swap(child1, child2); }
		if (dist1 == BVH_FAR) {
			if (stack_ptr == 0) break; else node = stack[--stack_ptr];
		} else {
			node = child1; /* continue with the nearest */
			if (dist2 != BVH_FAR) stack[stack_ptr++] = child2; /* push far child */
		}
	}

	LeafHit hit {};
	hit.t = closest_dist;
	hit.steps = steps;
	return hit;
}

#define STACK_POP() { ngroup = traversalStack[--stackPtr]; }
#define STACK_PUSH() { traversalStack[stackPtr++] = ngroup; }
inline uint32_t __bfind(uint32_t x) {
	return 31 - __lzcnt(x);
}
inline uint32_t __popc(uint32_t x) {
	return __popcnt(x);
}
static uint32_t as_uint(const float v) { return *(uint32_t*)&v; }
inline uint32_t extract_byte(const uint32_t i, const uint32_t n) { return (i >> (n * 8)) & 0xFF; }
inline uint32_t sign_extend_s8x4(const uint32_t i)
{
	// asm("prmt.b32 %0, %1, 0x0, 0x0000BA98;" : "=r"(v) : "r"(i)); // BA98: 1011`1010`1001`1000
	// with the given parameters, prmt will extend the sign to all bits in a byte.
	uint32_t b0 = (i & 0b10000000000000000000000000000000) ? 0xff000000 : 0;
	uint32_t b1 = (i & 0b00000000100000000000000000000000) ? 0x00ff0000 : 0;
	uint32_t b2 = (i & 0b00000000000000001000000000000000) ? 0x0000ff00 : 0;
	uint32_t b3 = (i & 0b00000000000000000000000010000000) ? 0x000000ff : 0;
	return b0 + b1 + b2 + b3; // probably can do better than this.
}
LeafHit CwBvh::trace_cwbvh(const Ray& ray) const {
	LeafHit hit(BIG_F32);
	uint2 traversalStack[128];
	uint32_t hitAddr = 0, stackPtr = 0;
	const float4* blasNodes = (float4*)cwbvh_nodes;
	float tmin = 0, tmax = 1e30f;
	const uint32_t octinv = (7 - ((ray.D.x < 0 ? 4 : 0) | (ray.D.y < 0 ? 2 : 0) | (ray.D.z < 0 ? 1 : 0))) * 0x1010101;
	uint2 ngroup = uint2(0, 0b10000000000000000000000000000000), tgroup = uint2(0);
	do
	{
		hit.steps += 1;
		if (ngroup.y > 0x00FFFFFF)
		{
			const uint32_t hits = ngroup.y, imask = ngroup.y;
			const uint32_t child_bit_index = __bfind(hits), child_node_base_index = ngroup.x;
			ngroup.y &= ~(1 << child_bit_index);
			if (ngroup.y > 0x00FFFFFF) { STACK_PUSH( /* nodeGroup */); }
			{
				const uint32_t slot_index = (child_bit_index - 24) ^ (octinv & 255);
				const uint32_t relative_index = __popc(imask & ~(0xFFFFFFFF << slot_index));
				const uint32_t child_node_index = child_node_base_index + relative_index;
				const float4 n0 = blasNodes[child_node_index * 5 + 0], n1 = blasNodes[child_node_index * 5 + 1];
				const float4 n2 = blasNodes[child_node_index * 5 + 2], n3 = blasNodes[child_node_index * 5 + 3];
				const float4 n4 = blasNodes[child_node_index * 5 + 4], p = n0;
				int3 e;
				e.x = (int32_t) * ((int8_t*)&n0.w + 0), e.y = (int32_t) * ((int8_t*)&n0.w + 1), e.z = (int32_t) * ((int8_t*)&n0.w + 2);
				ngroup.x = as_uint(n1.x), tgroup.x = as_uint(n1.y), tgroup.y = 0;
				uint32_t hitmask = 0;
				const uint32_t vx = (e.x + 127) << 23u; const float adjusted_idirx = *(float*)&vx * ray.rD.x;
				const uint32_t vy = (e.y + 127) << 23u; const float adjusted_idiry = *(float*)&vy * ray.rD.y;
				const uint32_t vz = (e.z + 127) << 23u; const float adjusted_idirz = *(float*)&vz * ray.rD.z;
				const float origx = -(ray.O.x - p.x) * ray.rD.x;
				const float origy = -(ray.O.y - p.y) * ray.rD.y;
				const float origz = -(ray.O.z - p.z) * ray.rD.z;
				{	// First 4
					const uint32_t meta4 = *(uint32_t*)&n1.z, is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
					const uint32_t inner_mask4 = sign_extend_s8x4(is_inner4 << 3);
					const uint32_t bit_index4 = (meta4 ^ (octinv & inner_mask4)) & 0x1F1F1F1F;
					const uint32_t child_bits4 = (meta4 >> 5) & 0x07070707;
					uint32_t swizzledLox = (ray.rD.x < 0) ? *(uint32_t*)&n3.z : *(uint32_t*)&n2.x, swizzledHix = (ray.rD.x < 0) ? *(uint32_t*)&n2.x : *(uint32_t*)&n3.z;
					uint32_t swizzledLoy = (ray.rD.y < 0) ? *(uint32_t*)&n4.x : *(uint32_t*)&n2.z, swizzledHiy = (ray.rD.y < 0) ? *(uint32_t*)&n2.z : *(uint32_t*)&n4.x;
					uint32_t swizzledLoz = (ray.rD.z < 0) ? *(uint32_t*)&n4.z : *(uint32_t*)&n3.x, swizzledHiz = (ray.rD.z < 0) ? *(uint32_t*)&n3.x : *(uint32_t*)&n4.z;
					float tminx[4], tminy[4], tminz[4], tmaxx[4], tmaxy[4], tmaxz[4];
					tminx[0] = ((swizzledLox >> 0) & 0xFF) * adjusted_idirx + origx, tminx[1] = ((swizzledLox >> 8) & 0xFF) * adjusted_idirx + origx, tminx[2] = ((swizzledLox >> 16) & 0xFF) * adjusted_idirx + origx;
					tminx[3] = ((swizzledLox >> 24) & 0xFF) * adjusted_idirx + origx, tminy[0] = ((swizzledLoy >> 0) & 0xFF) * adjusted_idiry + origy, tminy[1] = ((swizzledLoy >> 8) & 0xFF) * adjusted_idiry + origy;
					tminy[2] = ((swizzledLoy >> 16) & 0xFF) * adjusted_idiry + origy, tminy[3] = ((swizzledLoy >> 24) & 0xFF) * adjusted_idiry + origy, tminz[0] = ((swizzledLoz >> 0) & 0xFF) * adjusted_idirz + origz;
					tminz[1] = ((swizzledLoz >> 8) & 0xFF) * adjusted_idirz + origz, tminz[2] = ((swizzledLoz >> 16) & 0xFF) * adjusted_idirz + origz, tminz[3] = ((swizzledLoz >> 24) & 0xFF) * adjusted_idirz + origz;
					tmaxx[0] = ((swizzledHix >> 0) & 0xFF) * adjusted_idirx + origx, tmaxx[1] = ((swizzledHix >> 8) & 0xFF) * adjusted_idirx + origx, tmaxx[2] = ((swizzledHix >> 16) & 0xFF) * adjusted_idirx + origx;
					tmaxx[3] = ((swizzledHix >> 24) & 0xFF) * adjusted_idirx + origx, tmaxy[0] = ((swizzledHiy >> 0) & 0xFF) * adjusted_idiry + origy, tmaxy[1] = ((swizzledHiy >> 8) & 0xFF) * adjusted_idiry + origy;
					tmaxy[2] = ((swizzledHiy >> 16) & 0xFF) * adjusted_idiry + origy, tmaxy[3] = ((swizzledHiy >> 24) & 0xFF) * adjusted_idiry + origy, tmaxz[0] = ((swizzledHiz >> 0) & 0xFF) * adjusted_idirz + origz;
					tmaxz[1] = ((swizzledHiz >> 8) & 0xFF) * adjusted_idirz + origz, tmaxz[2] = ((swizzledHiz >> 16) & 0xFF) * adjusted_idirz + origz, tmaxz[3] = ((swizzledHiz >> 24) & 0xFF) * adjusted_idirz + origz;
					for (int32_t i = 0; i < 4; i++)
					{
						// Use VMIN, VMAX to compute the slabs
						const float cmin = fmaxf(fmaxf(fmaxf(tminx[i], tminy[i]), tminz[i]), tmin);
						const float cmax = fminf(fminf(fminf(tmaxx[i], tmaxy[i]), tmaxz[i]), tmax);
						if (cmin <= cmax) hitmask |= extract_byte(child_bits4, i) << extract_byte(bit_index4, i);
					}
				}
				{	// Second 4
					const uint32_t meta4 = *(uint32_t*)&n1.w, is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
					const uint32_t inner_mask4 = sign_extend_s8x4(is_inner4 << 3);
					const uint32_t bit_index4 = (meta4 ^ (octinv & inner_mask4)) & 0x1F1F1F1F;
					const uint32_t child_bits4 = (meta4 >> 5) & 0x07070707;
					uint32_t swizzledLox = (ray.rD.x < 0) ? *(uint32_t*)&n3.w : *(uint32_t*)&n2.y, swizzledHix = (ray.rD.x < 0) ? *(uint32_t*)&n2.y : *(uint32_t*)&n3.w;
					uint32_t swizzledLoy = (ray.rD.y < 0) ? *(uint32_t*)&n4.y : *(uint32_t*)&n2.w, swizzledHiy = (ray.rD.y < 0) ? *(uint32_t*)&n2.w : *(uint32_t*)&n4.y;
					uint32_t swizzledLoz = (ray.rD.z < 0) ? *(uint32_t*)&n4.w : *(uint32_t*)&n3.y, swizzledHiz = (ray.rD.z < 0) ? *(uint32_t*)&n3.y : *(uint32_t*)&n4.w;
					float tminx[4], tminy[4], tminz[4], tmaxx[4], tmaxy[4], tmaxz[4];
					tminx[0] = ((swizzledLox >> 0) & 0xFF) * adjusted_idirx + origx, tminx[1] = ((swizzledLox >> 8) & 0xFF) * adjusted_idirx + origx, tminx[2] = ((swizzledLox >> 16) & 0xFF) * adjusted_idirx + origx;
					tminx[3] = ((swizzledLox >> 24) & 0xFF) * adjusted_idirx + origx, tminy[0] = ((swizzledLoy >> 0) & 0xFF) * adjusted_idiry + origy, tminy[1] = ((swizzledLoy >> 8) & 0xFF) * adjusted_idiry + origy;
					tminy[2] = ((swizzledLoy >> 16) & 0xFF) * adjusted_idiry + origy, tminy[3] = ((swizzledLoy >> 24) & 0xFF) * adjusted_idiry + origy, tminz[0] = ((swizzledLoz >> 0) & 0xFF) * adjusted_idirz + origz;
					tminz[1] = ((swizzledLoz >> 8) & 0xFF) * adjusted_idirz + origz, tminz[2] = ((swizzledLoz >> 16) & 0xFF) * adjusted_idirz + origz, tminz[3] = ((swizzledLoz >> 24) & 0xFF) * adjusted_idirz + origz;
					tmaxx[0] = ((swizzledHix >> 0) & 0xFF) * adjusted_idirx + origx, tmaxx[1] = ((swizzledHix >> 8) & 0xFF) * adjusted_idirx + origx, tmaxx[2] = ((swizzledHix >> 16) & 0xFF) * adjusted_idirx + origx;
					tmaxx[3] = ((swizzledHix >> 24) & 0xFF) * adjusted_idirx + origx, tmaxy[0] = ((swizzledHiy >> 0) & 0xFF) * adjusted_idiry + origy, tmaxy[1] = ((swizzledHiy >> 8) & 0xFF) * adjusted_idiry + origy;
					tmaxy[2] = ((swizzledHiy >> 16) & 0xFF) * adjusted_idiry + origy, tmaxy[3] = ((swizzledHiy >> 24) & 0xFF) * adjusted_idiry + origy, tmaxz[0] = ((swizzledHiz >> 0) & 0xFF) * adjusted_idirz + origz;
					tmaxz[1] = ((swizzledHiz >> 8) & 0xFF) * adjusted_idirz + origz, tmaxz[2] = ((swizzledHiz >> 16) & 0xFF) * adjusted_idirz + origz, tmaxz[3] = ((swizzledHiz >> 24) & 0xFF) * adjusted_idirz + origz;
					for (int32_t i = 0; i < 4; i++)
					{
						const float cmin = fmaxf(fmaxf(fmaxf(tminx[i], tminy[i]), tminz[i]), tmin);
						const float cmax = fminf(fminf(fminf(tmaxx[i], tmaxy[i]), tmaxz[i]), tmax);
						if (cmin <= cmax) hitmask |= extract_byte(child_bits4, i) << extract_byte(bit_index4, i);
					}
				}
				ngroup.y = (hitmask & 0xFF000000) | (as_uint(n0.w) >> 24), tgroup.y = hitmask & 0x00FFFFFF;
			}
		}
		else tgroup = ngroup, ngroup = uint2(0);
		while (tgroup.y != 0)
		{
			uint32_t primIndex = __bfind(tgroup.y);
			tgroup.y -= (1u << primIndex);
			int32_t primAddr = tgroup.x + primIndex;
			const Aabb& prim = prims[cwbvh_indices[primAddr]];

			const float t = intersect_aabb(ray, prim.min, prim.max);

            if (t == BIG_F32) continue; /* miss */
            if (t < 0.0f || t > tmax) continue;
            tmax = t;
		}
		if (ngroup.y > 0x00FFFFFF) continue;
		if (stackPtr > 0) { STACK_POP( /* nodeGroup */); }
		else
		{
			hit.t = tmax;
			break;
		}
	} while (true);
	return hit;
}

CwBvh::~CwBvh() {
	//delete[] nodes;
	//delete[] prims;
}
