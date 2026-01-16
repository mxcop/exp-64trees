#pragma once

#include <cstdint>

#include "voxel-data.h"
#include "vbc64.h"

/* Compressed 64-wide Sparse Voxel Tree. */
class Cvt64 {
#pragma pack(push, 4)
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
#pragma pack(pop)

	struct Block {
		Vbc64 albedo_r{};
		Vbc64 albedo_g{};
		Vbc64 albedo_b{};
		uint32_t dword_offset = 0u;
	};

	/* List of tree nodes. */
	Node* nodes = nullptr;
	uint32_t node_count = 0u;
	/* List of voxel data. */
	Block* blocks = nullptr;
	uint32_t block_count = 0u;
	uint32_t* voxels = nullptr;
	uint32_t dword_count = 0u;
	uint32_t req_depth = 0u;
	/* Number of wasted bytes. */
	uint32_t wasted_count = 0u;

	/* Recursive tree subdivide function. */
	Node subdivide(const RawVoxels& raw_data, int scale, int3 index);

public:
	Cvt64() = default;
	~Cvt64();

	/* Build the Sparse Voxel Tree. */
	void build(const RawVoxels& raw_data);

	/* Get the current memory usage of the 64tree. */
	uint64_t memory_usage() const;
	/* Get the current wasted memory of the 64tree. */
	uint64_t wasted_memory() const;

	VoxelHit trace(const Ray& ray) const;

	void set_voxel(uint32_t x, uint32_t y, uint32_t z) {};
};
