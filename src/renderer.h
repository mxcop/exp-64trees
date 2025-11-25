#pragma once

class Svb8;
class Svt64;
class CwBvh;
class Bvh2;

#include "voxel-data.h"

namespace Tmpl8
{

enum class DisplayMode : int {
	eAlbedo,
	eDepth,
	eNormal,
	eSteps
};

class Renderer : public TheApp
{
public:
	// game flow methods
	void Init();
	float3 Trace( Ray& ray );
	void Tick( float deltaTime );
	void UI();
	void Shutdown();
	// input handling
	void MouseUp(int button) { if (button == 0) lmb = false; }
	void MouseDown( int button ) { if (button == 0) lmb = true; }
	void MouseMove( int x, int y ) { mousePos.x = x, mousePos.y = y; }
	void MouseWheel( float y ) { /* implement if you want to handle the mouse wheel */ }
	void KeyUp( int key ) { /* implement if you want to handle keys */ }
	void KeyDown( int key ) { /* implement if you want to handle keys */ }
	// data members
	uint seed = 1923674u;
	int2 mousePos;
	RawVoxels voxel_data{};
	Svb8* brickmap = nullptr;
	Svt64* tree = nullptr;
	CwBvh* cwbvh = nullptr;
    Bvh2* bvh2 = nullptr;
	Camera camera;
	DisplayMode display_mode = DisplayMode::eAlbedo;
	bool animating = true;
	bool lmb = false;
	float anim_time = 0;
	float frame_time = 0.0f;
	bool use_cwbvh = false;
};

} // namespace Tmpl8