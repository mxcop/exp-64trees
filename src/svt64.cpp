#include "precomp.h"
#include "svt64.h"

#include "immintrin.h"

/* Log with base. */
inline uint32_t log_base(const uint32_t x, const uint32_t b)
{
	return log2(x) / log2(b);
}

Svt64::Node::Node(const bool is_leaf, const uint32_t ptr, const uint64_t mask)
{
	/* Only set the 31 least significant bits. */
	child_ptr = ptr & 0x7FFFFFFFu;
	child_mask = mask;

	/* Most significant bit is used to indicate a leaf node. */
	if (is_leaf) child_ptr |= 0x80000000u;
}

/* Recursive tree subdivide function. */
Svt64::Node Svt64::subdivide(const RawVoxels& raw_data, int scale, int3 index)
{
	/* Create a leaf node */
	if (scale == 2) {
		Node leaf_node = Node(true, voxel_count, 0x00);

		/* Check if the node is outside the voxel grid bounds */
		if (index.x + 3u >= raw_data.w || index.y + 3u >= raw_data.h || index.z + 3u >= raw_data.d) return leaf_node;
		if (index.x < 0 || index.y < 0 || index.z < 0) return leaf_node;

		const int wh = raw_data.h * raw_data.w;

		for (int i = 0; i < 64; ++i) {
			/* Fetch the voxel data */
			const int voxel_x = index.x + ((i >> 0) & 3);
			const int voxel_y = index.y + ((i >> 4) & 3);
			const int voxel_z = index.z + ((i >> 2) & 3);
			const int voxel_offset = voxel_z * wh + voxel_y * raw_data.w + voxel_x;
			const uint8_t voxel = raw_data.raw_data[voxel_offset];

			if (voxel != 0x00) {
				voxels[voxel_count++] = voxel;
				leaf_node.child_mask |= (1ull << i);
			}
		}

		return leaf_node;
	}

	/* Descend */
	scale -= 2;

	/* Collect child nodes */
	Node child_nodes[64]{};
	uint64_t child_mask = 0x00;
	uint32_t child_count = 0u;

	for (int i = 0; i < 64; ++i) {
		/* Subdivide the child node */
		const int3 child_index = int3(i >> 0 & 3, i >> 4 & 3, i >> 2 & 3);
		const Node child = subdivide(raw_data, scale, index + (child_index << scale));

		/* If the child is not empty */
		if (child.child_mask != 0u) {
			child_mask |= (1ull << i);
			child_nodes[child_count++] = child;
		}
	}

	/* Create a node */
	const Node node = Node(false, node_count, child_mask);

	/* Add child nodes into the nodes array */
	memcpy(nodes + node_count, child_nodes, child_count * sizeof(Node));
	node_count += child_count;
	return node;
}

void Svt64::build(const RawVoxels& raw_data)
{
	/* Delete old data */
	if (nodes != nullptr) delete[] nodes;
	if (voxels != nullptr) delete[] voxels;

	/* Allocate space for new tree */
	nodes = (Node*)malloc(1ull << 24);
	node_count = 1u;
	voxels = (uint8_t*)malloc(1ull << 24);
	voxel_count = 0u;

	/* Required tree depth */
	const uint32_t req_depth = log_base(max(max(raw_data.w, raw_data.h), raw_data.d), 4);
	printf("required depth: %u\n", req_depth);

	/* Begin the recursive build */
	nodes[0] = subdivide(raw_data, req_depth * 2, int3(0));

	//printf("size: (%u, %u, %u), nodes: %u, voxels: %u\n", raw_data.w, raw_data.h, raw_data.d, node_count, voxel_count);

	const uint64_t raw_size = raw_data.w * raw_data.h * raw_data.d;
	const uint64_t tree_size = node_count * sizeof(Node) + voxel_count;
	const float memory_percent = ((float)tree_size / (float)raw_size) * 100.0f;
	printf("raw memory: %.2fMB, tree memory: %.2fMB (%.2f%%)\n", (float)raw_size / 1000000, (float)tree_size / 1000000, memory_percent);
}

inline uint get_node_cell_index(float3 pos, int scale_exp) {
	uint3 cell_pos = uint3((uint32_t&)pos.x >> scale_exp & 3, (uint32_t&)pos.y >> scale_exp & 3, (uint32_t&)pos.z >> scale_exp & 3);
	return cell_pos.x + cell_pos.y * 16u + cell_pos.z * 4u;
}

inline float3 floor_scale(float3 pos, int scale_exp) {
	uint32_t mask = ~0u << scale_exp;
	const uint3 masked = uint3(((uint32_t&)pos.x & mask), (uint32_t&)pos.y & mask, (uint32_t&)pos.z & mask);
	return float3((float&)masked.x, (float&)masked.y, (float&)masked.z);
}

// Reverses `pos` from range [1.0, 2.0) to (2.0, 1.0] if `dir > 0`.
inline float3 mirror_pos(const float3 pos, const float3 dir) {
	float3 mirrored{};
	mirrored.x = dir.x > 0.0f ? (3.0f - pos.x) : pos.x;
	mirrored.y = dir.y > 0.0f ? (3.0f - pos.y) : pos.y;
	mirrored.z = dir.z > 0.0f ? (3.0f - pos.z) : pos.z;
	return mirrored;
}

// Count number of set bits in variable range [0..width]
inline uint popcnt_var64(uint64_t mask, uint width) {
	return __popcnt64(mask & ((1ull << width) - 1));
}

float2 intersect_aabb(float3 origin, float3 invDir, float3 bbMin, float3 bbMax) {
	float3 t0 = (bbMin - origin) * invDir;
	float3 t1 = (bbMax - origin) * invDir;

	float3 temp = t0;
	t0 = fminf(temp, t1), t1 = fmaxf(temp, t1);

	float tmin = fmaxf(fmaxf(t0.x, t0.y), t0.z);
	float tmax = fminf(fminf(t1.x, t1.y), t1.z);

	return float2(tmin, tmax);
}

/* Credit: <https://dubiousconst282.github.io/2024/10/03/voxel-ray-tracing/> */
float Svt64::trace(const Ray& ray) const
{
	/* Traversal state */
	uint stack[11]{};
	int scale_exp = 21; /* 23 mantissa bits - 2 */
	uint node_index = 0u; /* root node */
	Node node = nodes[node_index];

	const float2 hit = intersect_aabb(ray.O, ray.rD, float3(1.0f), float3(2.0f));
	if (hit.y < hit.x) return 1e30f;

	const float3 origin = mirror_pos(ray.O + ray.D * hit.x, ray.D);
	const float3 dir = ray.D;

	/* Mirror coordinates to simplify cell intersections */
	uint mirror_mask = 0x00;
	if (dir.x > 0.0f) mirror_mask |= 3u << 0;
	if (dir.y > 0.0f) mirror_mask |= 3u << 4;
	if (dir.z > 0.0f) mirror_mask |= 3u << 2;

	/* Safety clamp */
	float3 pos = clamp(origin, 1.0f, 1.9999999f);
	float3 inv_dir = 1.0 / -fabs(dir);

	float3 side_dist{};
	int i = 0;

	for (i = 0; i < 256; i++) {
		uint child_index = get_node_cell_index(pos, scale_exp) ^ mirror_mask;

		/* Descend down the tree */
		while ((node.child_mask >> child_index & 1) != 0 && !node.is_leaf()) {
			/* Push the current node on the stack (at `scale_exp / 2`) */
			stack[scale_exp >> 1] = node_index;

			/* Fetch the child node */
			node_index = node.child_ptr + popcnt_var64(node.child_mask, child_index);
			node = nodes[node_index];

			/* Decrease the scale & get the next child index */
			scale_exp -= 2;
			child_index = get_node_cell_index(pos, scale_exp) ^ mirror_mask;
		}

		/* If this node is a leaf, check if we hit a voxel */
		if (node.is_leaf() && (node.child_mask >> child_index & 1) != 0) break;

		// 2³ steps
		int adv_scale_exp = scale_exp;
		if ((node.child_mask >> (child_index & 0b101010) & 0x00330033) == 0) adv_scale_exp++;

		// Compute next pos by intersecting with max cell sides
		const float3 cell_min = floor_scale(pos, adv_scale_exp);

		side_dist = (cell_min - origin) * inv_dir;
		float tmax = fminf(fminf(side_dist.x, side_dist.y), side_dist.z);

		const int3 cell_min_i = int3((int&)cell_min.x, (int&)cell_min.y, (int&)cell_min.z);

		int3 neighbor_max = cell_min_i; // +select(side_dist == tmax, -1, (1 << adv_scale_exp) - 1);
		neighbor_max.x += side_dist.x == tmax ? -1 : (1 << adv_scale_exp) - 1;
		neighbor_max.y += side_dist.y == tmax ? -1 : (1 << adv_scale_exp) - 1;
		neighbor_max.z += side_dist.z == tmax ? -1 : (1 << adv_scale_exp) - 1;

		pos = fminf(origin - fabs(dir) * tmax, (float3&)neighbor_max);

		// Find common ancestor based on left-most carry bit
		// We only care about changes in the exponent and high bits of
		// each cell position (10'10'10'...), so the odd bits are masked.
		uint3 diff_pos = uint3((uint&)pos.x ^ (uint&)cell_min.x, (uint&)pos.y ^ (uint&)cell_min.y, (uint&)pos.z ^ (uint&)cell_min.z); // asuint(pos) ^ asuint(cell_min);
		int diff_exp = 31 - _lzcnt_u32((diff_pos.x | diff_pos.y | diff_pos.z) & 0xFFAAAAAA); // 31 - lzcnt, or findMSB in GLSL

		if (diff_exp > scale_exp) {
			scale_exp = diff_exp;
			if (diff_exp > 21) break;  // going out of root?

			node_index = stack[scale_exp >> 1];
			node = nodes[node_index];
		}
	}

	return (float)i + 1;

	if (node.is_leaf() && scale_exp <= 21) {
		pos = mirror_pos(pos, dir);

		return hit.x + length(pos - origin);
	}
	return 1e30f;
}

Svt64::~Svt64()
{
	if (nodes != nullptr) delete[] nodes;
	if (voxels != nullptr) delete[] voxels;
}
