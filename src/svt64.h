#pragma once

#include <cstdint>

/* Raw voxel data. */
struct RawVoxels {
	uint8_t* raw_data = nullptr; 
	int w = 0u, h = 0u, d = 0u;

	RawVoxels(uint8_t* raw_data, int w, int h, int d) : raw_data(raw_data), w(w), h(h), d(d) {};
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
	uint8_t* voxels = nullptr;
	uint32_t voxel_count = 0u;

	/* Recursive tree subdivide function. */
	Node subdivide(const RawVoxels& raw_data, int scale, int3 index);

public:
	Svt64() = default;
	~Svt64();

	/* Build the Sparse Voxel Tree. */
	void build(const RawVoxels& raw_data);

	float trace(const Ray& ray) const;
};
