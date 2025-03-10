#pragma once

#include <cstdint>
#include <string_view>

struct Voxel {
	uint32_t albedo_r : 8;
	uint32_t albedo_g : 8;
	uint32_t albedo_b : 8;
	uint32_t albedo_a : 8;
};

constexpr Voxel VOXEL_EMPTY = Voxel();

/* Make sure 1 voxel is always 4 bytes. */
static_assert(sizeof(Voxel) == 4llu);

/* Raw voxel data. */
struct RawVoxels {
	Voxel* raw_data = nullptr;
	uint32_t w = 0u, h = 0u, d = 0u;

	RawVoxels() = default;

	/** @brief Load raw voxel data from a `.vox` file. */
	static RawVoxels from_file(const std::string_view path);

	/** @brief Destroy the raw voxel data. */
	void destroy() { delete[] raw_data; };

private:
	RawVoxels(Voxel* raw_data, const uint32_t w, const uint32_t h, const uint32_t d) : raw_data(raw_data), w(w), h(h), d(d) {};
};
