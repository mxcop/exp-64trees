#include "precomp.h"
#include "voxel-data.h"

#include <vector>
#include <string>
#include <fstream>

#define OGT_VOX_IMPLEMENTATION
#include <ogt_vox.h>

/* Read binary data from a file. */
std::vector<std::byte> read_binary_file(const std::string& path);

RawVoxels RawVoxels::from_file(const std::string_view path) {
	/* Load the binary file data */
	const std::vector<std::byte> binary = read_binary_file(std::string(path));
	if (binary.empty() == true) return RawVoxels();

	/* Parse the vox file data */
	const ogt_vox_scene* scene = ogt_vox_read_scene((uint8_t*)binary.data(), binary.size());
	if (scene == nullptr) return RawVoxels();

	/* Logging */
	printf("loaded model with size: %u, %u, %u\n", scene->models[0]->size_x, scene->models[0]->size_y, scene->models[0]->size_z);

	/* Read out model info */
	const uint32_t width = scene->models[0]->size_x;
	const uint32_t height = scene->models[0]->size_y;
	const uint32_t depth = scene->models[0]->size_z;
	const uint32_t voxel_cnt = width * height * depth;

	/* Allocate the raw data */
	Voxel* raw_data = new Voxel[voxel_cnt]{};

	for (uint32_t z = 0; z < height; z++) {
		for (uint32_t y = 0; y < depth; y++) {
			for (uint32_t x = 0; x < width; x++) {
				/* .vox model axes are weird */
				const uint32_t write_index = z * depth * width + y * width + x;
				const uint32_t read_index = y * height * width + (height - z - 1) * width + x;

				const ogt_vox_rgba& mat = scene->palette.color[scene->models[0]->voxel_data[read_index]];
				raw_data[write_index].albedo_r = mat.r;
				raw_data[write_index].albedo_g = mat.g;
				raw_data[write_index].albedo_b = mat.b;
				raw_data[write_index].albedo_a = mat.a;
			}
		}
	}

	/* Free the parsed data */
	ogt_vox_destroy_scene(scene);

	return RawVoxels(raw_data, width, height, depth);
}

std::vector<std::byte> read_binary_file(const std::string& path) {
	std::ifstream file(path, std::ios::binary | std::ios::ate);
	if (!file.is_open()) return {};
	const std::streamsize size = file.tellg();
	file.seekg(0, std::ios::beg);
	std::vector<std::byte> buffer(size);
	file.read(reinterpret_cast<char*>(buffer.data()), size);
	return buffer;
}
