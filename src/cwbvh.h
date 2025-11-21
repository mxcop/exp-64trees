#pragma once

#include <cstdint>

struct LeafHit {
	float t = 1e30f;

	LeafHit() = default;
	LeafHit(float t) : t(t) {};
};

/* Axis Aligned Bounding Box. */
struct Aabb {
	float3 min = 1e30f;
	float3 max = -1e30f;
};

/* 8-wide Compressed Bounding Volume Hierarchy. */
class CwBvh {
	struct Node {
		/* Minimum bounds of this node. */
		float min_bounds_x = 0.0f;
		float min_bounds_y = 0.0f;
		float min_bounds_z = 0.0f;

		/* Logarithm of the extent of this node. */
		uint8_t log_extent_x = 0u;
		uint8_t log_extent_y = 0u;
		uint8_t log_extent_z = 0u;
		uint8_t intersection_mask = 0u;

		/* Index of the first child node. (children are stored contiguously) */
		uint32_t child_base_index = 0u;
		/* Index of the first primitive. (primitives are stored contiguously) */
		uint32_t prim_base_index = 0u;

		/* Quantized child node bounding boxes. (48 bytes) */
		struct QuantizedBounds {
			uint8_t meta = 0u;
			uint8_t lo_x = 0u;
			uint8_t lo_y = 0u;
			uint8_t lo_z = 0u;
			uint8_t hi_x = 0u;
			uint8_t hi_y = 0u;
			uint8_t hi_z = 0u;
		} qbounds[8] {};

		Node() = default;
	};

	/* List of tree nodes. */
	Node* nodes = nullptr;
	uint32_t node_count = 0u;
	/* List of primitive data. */
	uint32_t* indices = nullptr;
	Aabb* prims = nullptr;
	uint32_t prim_count = 0u;

	/* Recursive tree subdivide function. */
	Node subdivide(const RawVoxels& raw_data, int scale, int3 index);

public:
	CwBvh() = default;
	~CwBvh();

	/* Build the BVH. */
	void build(const Aabb* input_prims, const uint32_t input_count);

	LeafHit trace(const Ray& ray) const;

	void set_voxel(uint32_t x, uint32_t y, uint32_t z);
};
