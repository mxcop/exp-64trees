#pragma once

#include <cstdint>

#include "voxel-data.h"

struct VoxelHit {
	float t = 1e30f;
	float3 normal = 0.0f;
	Voxel material{};
	uint16_t steps = 0u;

	VoxelHit() = default;
	VoxelHit(float t, float3 normal, Voxel mat, uint16_t steps) : t(t), normal(normal), material(mat), steps(steps) {};
};

/* 64-wide Sparse Voxel Tree. */
class Svt64 {
	struct Node {
		/* The most significant bit indicates if this node is a leaf containing voxels. */
		/* The 31 least significant bits are an absolute offset into an array of child nodes / voxels. */
		uint32_t child_ptr = 0u;
		/* Indicates which child nodes / voxels are present using 1 bit per child. */
		uint64_t child_mask = 0u;

		Node() = default;
		Node(const bool is_leaf, const uint32_t ptr, const uint64_t mask);

		/* Is this node a leaf node? */
		inline bool is_leaf() const { return (child_ptr >> 31u) == 1u; };

		/* Absolute offset into an array of child nodes / voxels. */
		inline uint32_t abs_ptr() const {
			return child_ptr & 0x7FFFFFFFu;
		};
	};

	/* List of tree nodes. */
	Node* nodes = nullptr;
	uint32_t node_count = 0u;
	/* List of voxel data. */
	Voxel* voxels = nullptr;
	uint32_t voxel_count = 0u;
	uint32_t req_depth = 0u;

	/* Recursive tree subdivide function. */
	Node subdivide(const RawVoxels& raw_data, int scale, int3 index);

public:
	Svt64() = default;
	~Svt64();

	/* Build the Sparse Voxel Tree. */
	void build(const RawVoxels& raw_data);

	/* Get the current memory usage of the 64tree. */
	uint64_t memory_usage() const;

	VoxelHit trace(const Ray& ray) const;

	void set_voxel(uint32_t x, uint32_t y, uint32_t z);
};
