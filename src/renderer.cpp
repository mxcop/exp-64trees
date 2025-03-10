#include "precomp.h"

#include "svt64.h"

constexpr uint32_t GRID_SIZE = 64u;
constexpr uint32_t CELL_COUNT = GRID_SIZE * GRID_SIZE * GRID_SIZE;

/* Called once on init. */
void Renderer::Init() {
	voxel_data = RawVoxels::from_file("assets/dragon.vox");

	tree = new Svt64();

	//Timer t;
	tree->build(voxel_data);
	//const float ms = t.elapsed() * 1000.0f;

	//printf("tree build time: %5.2fms\n", ms);
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
	//	tree->build(*voxel_data);
	//	static float avg = 10, alpha = 1;
	//	avg = (1 - alpha) * avg + alpha * t.elapsed() * 1000;
	//	if (alpha > 0.05f) alpha *= 0.5f;
	//	printf("tree build time: %5.2fms\n", avg);
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
void Renderer::UI()
{
	if (ImGui::Begin("Debug")) {
		const char* modes[] = { "Albedo", "Depth", "Normal", "Steps"};
		ImGui::Combo("Display Mode", (int*)&display_mode, modes, IM_ARRAYSIZE(modes));
	}
	ImGui::End();
}

void Renderer::Shutdown()
{
	delete tree;
}
