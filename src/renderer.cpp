#include "precomp.h"

#include "svt64.h"
#include "svb8.h"

constexpr uint32_t GRID_SIZE = 64u;
constexpr uint32_t CELL_COUNT = GRID_SIZE * GRID_SIZE * GRID_SIZE;

/* Called once on init. */
void Renderer::Init() {
	voxel_data = RawVoxels::from_file("assets/dragon.vox");

	brickmap = new Svb8();
	brickmap->init(voxel_data.w / 8u, voxel_data.h / 8u, voxel_data.d / 8u);

	for (uint32_t z = 0u; z < voxel_data.d; ++z) {
		for (uint32_t y = 0u; y < voxel_data.h; ++y) {
			for (uint32_t x = 0u; x < voxel_data.w; ++x) {
				brickmap->set_voxel(x, y, z, voxel_data.raw_data[x + y * voxel_data.w + z * voxel_data.w * voxel_data.h]);
			}
		}
	}

	const uint64_t raw_memory = (uint64_t)voxel_data.w * voxel_data.h * voxel_data.d * sizeof(Voxel);
	const uint64_t bm_memory = brickmap->memory_usage();
	printf("[raw] memory usage: %.2fMB\n", (float)raw_memory / 1000000);
	printf("[brickmap] memory usage: %.2fMB (%.2f%%)\n", (float)bm_memory / 1000000, (float)bm_memory / (float)raw_memory * 100.0f);

	tree = new Svt64();
	tree->build(voxel_data);
	const uint64_t svt64_memory = tree->memory_usage();
	printf("[64tree] memory usage: %.2fMB (%.2f%%)\n", (float)svt64_memory / 1000000, (float)svt64_memory / (float)raw_memory * 100.0f);
}

inline float3 heat_color(float t) {
	t = clamp(t, 0.0f, 1.0f);
	static const float3 c0 = float3(-0.002136485053939582, -0.000749655052795221, -0.005386127855323933);
	static const float3 c1 = float3(0.2516605407371642, 0.6775232436837668, 2.494026599312351);
	static const float3 c2 = float3(8.353717279216625, -3.577719514958484, 0.3144679030132573);
	static const float3 c3 = float3(-27.66873308576866, 14.26473078096533, -13.64921318813922);
	static const float3 c4 = float3(52.17613981234068, -27.94360607168351, 12.94416944238394);
	static const float3 c5 = float3(-50.76852536473588, 29.04658282127291, 4.23415299384598);
	static const float3 c6 = float3(18.65570506591883, -11.48977351997711, -5.601961508734096);
	return c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * (c5 + t * c6)))));
}

/* Trace a ray. */
float3 Renderer::Trace(Ray& ray)
{
	const VoxelHit hit = tree->trace(ray);

	switch (display_mode) {
	case DisplayMode::eAlbedo: {
		if (hit.t < 1e30f) {

			float irradiance = 1.0f;
			//float irradiance = 0.0f;
			//const float3 hit_point = ray.O + ray.D * (hit.t - 0.001f);

			//for (int i = 0; i < 8; ++i) {
			//	const float3 bounce_dir = normalize(hit.normal + random_unit_vector(seed));
			//	const VoxelHit bounce_hit = tree->trace(Ray(hit_point, bounce_dir));
			//	irradiance += bounce_hit.t < 1e30f ? 0.0f : 1.0f;
			//}
			//irradiance *= (1.0f / 8.0f);

			const Voxel v = hit.material;
			return float3((float)v.albedo_r / 255.0f, (float)v.albedo_g / 255.0f, (float)v.albedo_b / 255.0f) * irradiance;
		}
		break;
	}
	case DisplayMode::eDepth:
		return float3(1.0f - hit.t * 0.2f);
	case DisplayMode::eNormal:
		return hit.normal * 0.5f + 0.5f;
	case DisplayMode::eSteps:
		return heat_color((float)hit.steps / 32.0f);
	}

	return float3(0.05f, 0.05f, 0.05f);
}

/* Called once per frame. */
void Renderer::Tick(float deltaTime)
{
	//{ /* Build and time tree */
	//	Timer t; 
	//	tree->build(voxel_data);
	//	static float avg = 10, alpha = 1;
	//	avg = (1 - alpha) * avg + alpha * t.elapsed() * 1000;
	//	if (alpha > 0.05f) alpha *= 0.5f;
	//	printf("tree build time: %5.2fms\n", avg);
	//}

	//{ /* Build and time brickmap */
	//	Timer t;
	//	brickmap->init(voxel_data.w / 8u, voxel_data.h / 8u, voxel_data.d / 8u);

	//	//for (uint32_t bz = 0u; bz < voxel_data.d / 8; ++bz) {
	//	//	for (uint32_t by = 0u; by < voxel_data.h / 8; ++by) {
	//	//		for (uint32_t bx = 0u; bx < voxel_data.w / 8; ++bx) {

	//	//			for (uint32_t vz = 0u; vz < 8; ++vz) {
	//	//				for (uint32_t vy = 0u; vy < 8; ++vy) {
	//	//					for (uint32_t vx = 0u; vx < 8; ++vx) {
	//	//						const uint32_t z = bz * 8 + vz;
	//	//						const uint32_t y = by * 8 + vy;
	//	//						const uint32_t x = bx * 8 + vx;
	//	//						brickmap->set_voxel(x, y, z, voxel_data.raw_data[x + y * voxel_data.w + z * voxel_data.w * voxel_data.h]);
	//	//					}
	//	//				}
	//	//			}

	//	//		}
	//	//	}
	//	//}
	//	for (uint32_t z = 0u; z < voxel_data.d; ++z) {
	//		for (uint32_t y = 0u; y < voxel_data.h; ++y) {
	//			for (uint32_t x = 0u; x < voxel_data.w; ++x) {
	//				brickmap->set_voxel(x, y, z, voxel_data.raw_data[x + y * voxel_data.w + z * voxel_data.w * voxel_data.h]);
	//			}
	//		}
	//	}
	//	static float avg = 10, alpha = 1;
	//	avg = (1 - alpha) * avg + alpha * t.elapsed() * 1000;
	//	if (alpha > 0.05f) alpha *= 0.5f;
	//	printf("brickmap build time: %5.2fms\n", avg);
	//}

	Timer t; /* frame timer. */

#pragma omp parallel for schedule(dynamic)
	for (int y = 0; y < SCRHEIGHT; y++)
	{
		// trace a primary ray for each pixel on the line
		for (int x = 0; x < SCRWIDTH; x++)
		{
			float4 pixel = float4(Trace(camera.GetPrimaryRay((float)x, (float)y)), 0);
			// translate accumulator contents to rgb32 pixels
			screen->pixels[x + y * SCRWIDTH] = RGBF32_to_RGB8(&pixel);
			// accumulator[x + y * SCRWIDTH] = pixel;
		}
	}

	/* Log performance. */
	static float avg = 10, alpha = 1;
	avg = (1 - alpha) * avg + alpha * t.elapsed() * 1000;
	if (alpha > 0.05f) alpha *= 0.5f;
	float fps = 1000.0f / avg, rps = (SCRWIDTH * SCRHEIGHT) / avg;
	// printf("%5.2fms (%.1ffps) - %.1fMrays/s\n", avg, fps, rps / 1000);

	/* User input */
	camera.HandleInput(deltaTime);
}

/* ImGUI tick. */
void Renderer::UI() {
	if (lmb) {
		const Ray ray = camera.GetPrimaryRay((float)mousePos.x, (float)mousePos.y);
		const VoxelHit hit = tree->trace(ray);

		if (hit.t != 1e30f) {
			const float3 hit_pos = ray.O + ray.D * (hit.t - 0.001f);
			const uint3 hit_index = uint3((hit_pos - 1.0f) * float3(voxel_data.w, voxel_data.h, voxel_data.d));
			// printf("hit pos: %u, %u, %u\n", hit_index.x, hit_index.y, hit_index.z);
			//Timer t; /* frame timer. */
			tree->set_voxel(hit_index.x, hit_index.y, hit_index.z);
			//static float avg = 10, alpha = 1;
			//avg = (1 - alpha) * avg + alpha * t.elapsed() * 1000000;
			//if (alpha > 0.05f) alpha *= 0.5f;
			//printf("set voxel time: %5.2fus\n", avg);
		}
	}

	if (ImGui::Begin("Debug")) {
		const char* modes[] = { "Albedo", "Depth", "Normal", "Steps"};
		ImGui::Combo("Display Mode", (int*)&display_mode, modes, IM_ARRAYSIZE(modes));

		ImGui::Separator();
		const uint64_t raw_memory = (uint64_t)voxel_data.w * voxel_data.h * voxel_data.d * sizeof(Voxel);
		const uint64_t svt64_memory = tree->memory_usage();
		const uint64_t svt64_wasted = tree->wasted_memory();
		ImGui::Text("[raw] memory usage: %.2fMB\n", (float)raw_memory / 1000000);
		ImGui::Text("[64tree] memory usage: %.2fMB (%.2f%%)\n", (float)svt64_memory / 1000000, (float)svt64_memory / (float)raw_memory * 100.0f);
		ImGui::Text("[64tree] wasted memory: %.2fMB (%.2f%%)\n", (float)svt64_wasted / 1000000, (float)svt64_wasted / (float)svt64_memory * 100.0f);

		if (ImGui::Button("Defragment")) {
			Timer t;
			tree->defrag();
			printf("defrag time: %5.2fus\n", t.elapsed() * 1000000);
		}
	}
	ImGui::End();
}

void Renderer::Shutdown()
{
	delete tree;
}