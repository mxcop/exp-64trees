#pragma once

#include <cstdint>

struct LeafHit {
	float t = 1e30f;
	uint16_t steps = 0u;

	LeafHit() = default;
	LeafHit(float t) : t(t) {};
};

/* Axis Aligned Bounding Box. */
struct Aabb {
	float3 min = 1e30f;
	float3 max = -1e30f;
};

struct Bvh2Node;
struct Bvh8Node;
struct CwBvhNode;

/* 8-wide Compressed Bounding Volume Hierarchy. */
class CwBvh {
	/* Primitive data */
	Aabb* prims = nullptr;
	uint32_t prim_count = 0u;

	/* 2-wide BVH */
	Bvh2Node* bvh2_nodes = nullptr;
	uint32_t* bvh2_indices = nullptr;
	uint32_t bvh2_node_count = 0u;

	Bvh8Node* bvh8_nodes = nullptr;

	/* CWBVH */
	CwBvhNode* cwbvh_nodes = nullptr;
	uint32_t* cwbvh_indices = nullptr;
	uint32_t cwbvh_node_count = 0u;

	void build_bvh2(const Aabb* input_prims, const uint32_t input_count);

	void bvh2_to_cwbvh();

public:
	CwBvh() = default;
	~CwBvh();

	/* Build the BVH. */
	void build(const Aabb* input_prims, const uint32_t input_count);

	void print() const;

	LeafHit trace_bvh2(const Ray& ray) const;

	LeafHit trace_cwbvh(const Ray& ray) const;

	void set_voxel(uint32_t x, uint32_t y, uint32_t z);
};
