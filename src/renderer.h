#pragma once

class Svt64;
struct RawVoxels;

#include "ogt_vox.h"

namespace Tmpl8
{

enum class DisplayMode : int {
	eAlbedo,
	eDepth,
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
	void MouseDown( int button ) { /* implement if you want to detect mouse button presses */ }
	void MouseMove( int x, int y ) { mousePos.x = x, mousePos.y = y; }
	void MouseWheel( float y ) { /* implement if you want to handle the mouse wheel */ }
	void KeyUp( int key ) { /* implement if you want to handle keys */ }
	void KeyDown( int key ) { /* implement if you want to handle keys */ }
	// data members
	int2 mousePos;
	RawVoxels* voxel_data = nullptr;
	ogt_vox_palette palette{};
	Svt64* tree = nullptr;
	Camera camera;
	DisplayMode display_mode = DisplayMode::eAlbedo;
	bool animating = true;
	float anim_time = 0;
};

} // namespace Tmpl8