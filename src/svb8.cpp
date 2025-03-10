#include "precomp.h"
#include "svb8.h"

/* Size of a single brick in voxels. */
constexpr uint32_t BRICK_SIZE = 8u * 8u * 8u;

/* Approximately 0.5mB (256 bricks) */
constexpr uint32_t MIN_VOXEL_MEMORY = BRICK_SIZE * 256u;

uint32_t Svb8::alloc_brick() {
	/* Check if we're out of voxel memory */
	if ((voxel_count + BRICK_SIZE) > voxel_capacity) {
		/* Copy the voxel data into a new memory block with twice the capacity */
		Voxel* new_voxels = new Voxel[voxel_capacity * 2u]{};
		memcpy(new_voxels, voxels, voxel_capacity * sizeof(Voxel));

		/* Free the old voxel memory block */
		delete[] voxels;

		/* Save the new voxel memory block */
		voxels = new_voxels;
		voxel_capacity = voxel_capacity * 2u;
	}

	/* Increment the voxel count */
	const uint32_t handle = voxel_count;
	voxel_count += BRICK_SIZE;

	return handle;
}

void Svb8::init(const uint32_t w, const uint32_t h, const uint32_t d) {
	/* Set the extent of the brickmap */
	width = w, height = h, depth = d;

	/* Free the map if it was allocated */
	if (map != nullptr) {
		delete[] map;
		map = nullptr;
	}

	/* Allocate a new brickmap */
	map = new Brick[w * h * d]{};

	/* Free the voxels if it was allocated */
	if (voxels != nullptr) {
		delete[] voxels;
		voxels = nullptr;
	}

	/* Clear the voxels list, keep enough space for 256 bricks (0.5mB) */
	voxels = new Voxel[MIN_VOXEL_MEMORY]{};
	voxel_count = 0u;
	voxel_capacity = MIN_VOXEL_MEMORY;
}

uint64_t Svb8::memory_usage() const {
	const uint64_t voxel_memory = sizeof(Voxel) * voxel_capacity;
	const uint64_t brick_memory = sizeof(Brick) * width * height * depth;
	return voxel_memory + brick_memory;
}

void Svb8::clear() {
	/* Set all bricks to empty */
	memset(map, 0xFFFFFFFFu, sizeof(Brick) * width * height * depth);

	/* Free the voxels if it was allocated */
	if (voxels != nullptr) {
		delete[] voxels;
		voxels = nullptr;
	}

	/* Clear the voxels list, keep enough space for 256 bricks (0.5mB) */
	voxels = new Voxel[MIN_VOXEL_MEMORY]{};
	voxel_count = 0u;
	voxel_capacity = MIN_VOXEL_MEMORY;
}

void Svb8::set_voxel(const uint32_t x, const uint32_t y, const uint32_t z, const Voxel& voxel) {
	/* Find the brick to edit */
	const uint32_t bx = x >> 3u;
	const uint32_t by = y >> 3u;
	const uint32_t bz = z >> 3u;
	Brick& brick = map[bx + by * width + bz * width * height];

	/* If the brick is empty, allocate it */
	if (brick.is_empty()) {
		if (voxel.is_empty()) return;
		brick.handle = alloc_brick();
	}

	/* Get the local coordinate of the voxel within its brick */
	const uint32_t vx = x & 0b111u;
	const uint32_t vy = y & 0b111u;
	const uint32_t vz = z & 0b111u;

	/* Get the address where the bricks voxels are stored */
	Voxel* brick_data = voxels + brick.handle;

	/* Set the voxel */
	brick_data[vx + vy * 8u + vz * 8u * 8u] = voxel;
}

const Voxel& Svb8::get_voxel(const uint32_t x, const uint32_t y, const uint32_t z) const {
	/* Find the brick to edit */
	const uint32_t bx = x >> 3u;
	const uint32_t by = y >> 3u;
	const uint32_t bz = z >> 3u;
	const Brick& brick = map[x + y * width + z * width * height];

	/* Get the local coordinate of the voxel within its brick */
	const uint32_t vx = x & 0b111u;
	const uint32_t vy = y & 0b111u;
	const uint32_t vz = z & 0b111u;

	/* Get the address where the bricks voxels are stored */
	const Voxel* brick_data = voxels + brick.handle;

	/* Get the voxel */
	return brick_data[vx + vy * 8u + vz * 8u * 8u];
}
