// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "gfx/draw_overlay.h"
#include "gfx/edges.h"
#include "gfx/highlight_outline.h"
#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct FrameInput
{
	Mat4 view;
	Mat4 viewInv;
	Mat4 proj;
	Mat4 projInv;
	b3Vec3 cameraPosition;

	// World-space position of the relative render frame's origin (the draw
	// origin the host shifts to the camera each frame). Shapes render relative
	// to it, so the ground grid adds it back to recover absolute world XZ and
	// stay locked to the world instead of sliding with the camera. Used full
	// precision for the origin axes, which fade out far from the world origin.
	b3Vec3 drawOrigin;

	// Draw origin XZ wrapped to the grid period (10 cells) in double before it
	// narrows. The grid pattern repeats over this period, so the wrapped offset
	// reproduces the same lines while staying small enough for a float to
	// resolve a 1 m cell out at planetary distance.
	b3Vec2 gridWrap;

	// seconds since InitRenderer
	float time;

	// 0 = lit, 1 = view-space depth grayscale, 2 = CSM cascade index,
	// 3 = view-space normal (RGB, [-1,1] mapped to [0,1]),
	// 4 = raw GTAO ambient-occlusion (grayscale, post-upsample).
	int debugMode;

	// cascade passes still clear to depth=1.0
	bool disableShadows;

	// depth pre-pass + trace + upsample skipped
	// the full-res AO target is cleared to 1.0 (no occlusion) instead
	bool disableAmbientOcclusion;

	// Simulation up axis. The camera folds the sim->display map into the
	// view transform, but the procedural sky reconstructs directions in sim
	// space, so it needs the up axis to orient the Preetham model.
	bool zUp;
} FrameInput;

// Directional light. The renderer owns one. Defaults are set in InitRenderer:
// dirToSun normalized from (0.5, 0.8, 0.4), color (1.0, 0.95, 0.85),
// ambient 0.10. Apps can override via SetSun before any draw call.
// dirToSun is normalized internally, caller need not pre-normalize.
typedef struct Sun
{
	b3Vec3 dirToSun; // world-space direction TO the sun
	b3Vec3 color;	 // RGB intensity, multiplied by N.L for direct light
	float ambient;	 // ambient strength added to direct light (0..1 typical)
	float strength;  // multiplies color [0,1]
} Sun;

void InitRenderer( const sg_environment* env );

// Sun settings
void SetSun( Sun sun );
Sun GetSun( void );

// Exposure stop applied by the AgX tone map pass (final image multiplied
// by pow(2, ev) before tone map).
void SetExposure( float ev );
float GetExposure( void );

// AgX-look saturation applied by the tone map pass after the AgX sigmoid.
// 1.0 (default) is AgX's stock Standard look. >1.0 counteracts the
// path-to-white desaturation for punchier debug colors, <1.0 mutes them.
void SetToneMapSaturation( float saturation );
float GetToneMapSaturation( void );

// Preetham turbidity, fed to both the sky pass and the IBL cube map rebuild.
void SetSkyTurbidity( float t );
float GetSkyTurbidity( void );

// Toggle image-based lighting (IBL). Ambient comes from the procedural sky
// (diffuse SH + prefiltered specular). When disabled, lit shapes fall back
// to flat ambient, base_color * sun.ambient. Does not affect the sky background.
void SetIblEnabled( bool enabled );
bool GetIblEnabled( void );

void SetEdgeOverlayParams( const EdgeOverlayParams* settings );
EdgeOverlayParams GetEdgeOverlayParams( void );

// Body highlighting parameters
void SetHighlightParams( const HighlightParams* params );
HighlightParams GetHighlightParams( void );

typedef struct
{
	int cubeCount;
	int sphereCount;
	int capsuleCount;
	int cubeCountXp;
	int sphereCountXp;
	int capsuleCountXp;
	int geomInstanceCount; // sum across all draw spans (opaque)
	int geomInstanceCountXp;
	int geomSpanCount;
	int lineCount;
	int pointCount;
	int drawCallCount; // approximate, one per non-empty shape stream + spans
	float frameTimeMs; // last RenderFrame cost
} RenderStats;

RenderStats GetRenderStats( void );

// Last-frame camera latch. Populated at the top of RenderFrame /
// RenderFrameOffscreen so post-scene consumers (e.g. the ImGui-backed
// in-world text overlay) can project world positions to screen pixels using
// the same matrices the scene was rasterized with. viewportW/H are in pixels,
// matching the framebuffer the scene rendered to.
typedef struct CameraState
{
	Mat4 view;
	Mat4 viewInv;
	Mat4 proj;
	Mat4 projInv;
	int viewportW;
	int viewportH;
} CameraState;
CameraState GetCameraState( void );

// Cube proxy buffer shared by sphere and capsule impostors
sg_buffer GetCubeProxyVertexBuffer( void );
sg_buffer GetCubeProxyIndexBuffer( void );
int GetCubeProxyIndexCount( void );

void RenderFrame( const sg_swapchain* swapChain, const FrameInput* frame );

// Render the scene into a caller-owned offscreen attachment set. The
// renderer is host-agnostic. This is the entry point a non-sokol_app
// host (e.g. a GLFW-based Box3D sample) would call. `attachments` describes
// the tonemap output target (single-sample, sRGB-encoded BGRA8/RGBA8), the
// renderer owns its own MSAA HDR scene target internally. `outputFormat` is
// the color format of attachments.colors[0], used to key the tonemap pipeline
// cache. width/height are the attachment image dimensions, fed into
// ub_frame.viewport.
void RenderFrameOffscreen( const sg_attachments* attachments, sg_pixel_format outputFormat, int width, int height,
							  const FrameInput* frame );
void ShutdownRenderer( void );

int GetRenderErrorCount( void );

// Reset the per-frame arena and clear all instance counts. Safe to call
// every frame even if no submissions follow.
void ResetFrameArena( void );

// Append one cube instance into the frame arena. transform places and orients
// the cube. scale multiplies the unit cube proxy (which spans [-0.5, 0.5]), so
// scale is the cube's full side lengths. baseColor is linear RGBA (alpha < 1.0
// routes to the transparent arena, alpha == 1.0 to the opaque arena). metallic
// and roughness feed the PBR BRDF (clamp to [0,1] at the call site). shadowCast
// picks whether this instance contributes to the shadow caster pass. Logs and
// drops on overflow.
void AppendCube( b3Transform transform, b3Vec3 scale, Vec4 baseColor, float metallic, float roughness, TransparentShadowCast shadowCast );

// Append one sphere instance. transform's rotation drives the surface pattern
// so spin is visible, its translation is the sphere center. radius is the only
// scale, given separately so the bounding-cube proxy matches the analytic
// silhouette. baseColor, metallic, roughness feed the PBR BRDF, the impostor
// pattern modulates baseColor before lighting. shadowCast as in AppendCube.
// Logs and drops on overflow.
void AppendSphere( b3Transform transform, float radius, Vec4 baseColor, float metallic, float roughness,
					  TransparentShadowCast shadowCast );

// Append one capsule instance. transform's rotation places the capsule's local
// +X axis (the long axis) in world space and drives the surface pattern so
// all three rotational axes are visible, its translation is the capsule
// center. halfLength is the distance from center to either cap center along
// local +X. radius is the cap radius. baseColor, metallic, roughness feed
// the PBR BRDF, the impostor pattern modulates baseColor before lighting.
// shadowCast as in AppendCube. Logs and drops on overflow.
void AppendCapsule( b3Transform transform, float halfLength, float radius, Vec4 baseColor, float metallic, float roughness,
					   TransparentShadowCast shadowCast );

#ifdef __cplusplus
} // extern "C"
#endif
