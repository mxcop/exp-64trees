#pragma once

#include <cstdint>
#include <vector>

#include "voxel-data.h"

/* 8x8x8 Sparse Voxel Brickmap */
class Svb8 {
	struct Brick {
		uint32_t handle = 0xFFFFFFFFu;
		inline bool is_empty() const { return handle == 0xFFFFFFFFu; };
	};

	/* Grid of bricks. */
	Brick* map = nullptr;
	uint32_t width = 0u, height = 0u, depth = 0u; /* Extent in bricks. */
	/* List of voxel data. */
	Voxel* voxels = nullptr;
	uint32_t voxel_count = 0u;
	uint32_t voxel_capacity = 0u;

	/* Allocate a new brick, returns the handle of the new brick. */
	uint32_t alloc_brick();

public:
	Svb8() = default;
	~Svb8() = default;

	/* Initialize the brickmap. (extent is in bricks) */
	void init(const uint32_t w, const uint32_t h, const uint32_t d);

	/* Get the current memory usage of the brickmap. */
	uint64_t memory_usage() const;

	/* Clear the brickmap. */
	void clear();

	/* Set the voxel at the given position in the brickmap. */
	void set_voxel(const uint32_t x, const uint32_t y, const uint32_t z, const Voxel& voxel);

	/* Get the voxel from the given position in the brickmap. */
	const Voxel& get_voxel(const uint32_t x, const uint32_t y, const uint32_t z) const;
};
