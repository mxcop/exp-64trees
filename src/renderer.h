#pragma once

class Svb8;
class Svt64;

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
	void MouseUp( int button ) { /* implement if you want to detect mouse button presses */ }
	void MouseDown( int button );
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
	Camera camera;
	DisplayMode display_mode = DisplayMode::eAlbedo;
	bool animating = true;
	float anim_time = 0;
};

} // namespace Tmpl8