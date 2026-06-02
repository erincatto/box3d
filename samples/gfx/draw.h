// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "gfx/draw_overlay.h"
#include "gfx/utility.h"

// Neutral-dielectric material defaults for the short-form shape calls.
#define DEFAULT_METALLIC 0.0f
#define DEFAULT_ROUGHNESS 0.5f

// Thickness/size defaults for the short-form overlay calls
#define DEFAULT_LINE_THICKNESS_PX 1.5f
#define DEFAULT_POINT_SIZE_PX 4.0f

// Arrow head length as a fraction of the shaft length
#define DEFAULT_ARROW_HEAD_FRAC 0.15f

#ifdef __cplusplus
extern "C"
{
#endif

// Cube. transform places and orients the cube. scale is the cube's full side
// lengths (it multiplies the unit cube proxy, which spans [-0.5, 0.5]).
void DrawCube( b3Transform transform, b3Vec3 scale, Vec4 baseColor );
void DrawCubeEx( b3Transform transform, b3Vec3 scale, Vec4 baseColor, float metallic, float roughness,
				 TransparentShadowCast shadowCast );

// Sphere impostor. transform supplies the orientation (rotation drives the
// surface pattern so spin is visible) and the center (translation).
void DrawSphere( b3Transform transform, float radius, Vec4 baseColor );
void DrawSphereEx( b3Transform transform, float radius, Vec4 baseColor, float metallic, float roughness,
				   TransparentShadowCast shadowCast );

// Capsule impostor. transform supplies the orientation (the capsule's local +X
// axis becomes the world long axis, rotation about local +X drives the
// cylinder/cap surface pattern so roll is visible) and the center
// (translation). halfLength is the distance from center to either cap
// center along local +X. radius is the cap radius.
void DrawCapsule( b3Transform transform, float halfLength, float radius, Vec4 baseColor );
void DrawCapsuleEx( b3Transform transform, float halfLength, float radius, Vec4 baseColor, float metallic, float roughness,
					TransparentShadowCast shadowCast );

// All overlay submissions land in the renderer's per-frame overlay arenas
// and are drawn into the MSAA HDR scene target after opaque + sky, before
// the AgX tone map pass. Colors are straight (non-premultiplied) linear RGBA.

void DrawLine( b3Vec3 a, b3Vec3 b, Vec4 color );
void DrawLineEx( b3Vec3 a, b3Vec3 b, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode );

void DrawPoint( b3Vec3 p, Vec4 color );
void DrawPointEx( b3Vec3 p, Vec4 color, float size, OverlayThicknessUnit sizeUnit, OverlayOcclusionMode occlusionMode );

// Single-arrow shaft from a to b with two short angled head segments at b.
// headLengthFrac is the head length as a fraction of the shaft length
void DrawArrow( b3Vec3 a, b3Vec3 b, Vec4 color );
void DrawArrowEx( b3Vec3 a, b3Vec3 b, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				  OverlayOcclusionMode occlusionMode, float headLengthFrac );

// Three orthogonal lines through center, total length size per axis, uniform color.
void DrawCross( b3Vec3 center, float size, Vec4 color );
void DrawCrossEx( b3Vec3 center, float size, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				  OverlayOcclusionMode occlusionMode );

// Wireframe AABB (12 edges) between world-space min and max corners.
void DrawAabb( b3Vec3 min, b3Vec3 max, Vec4 color );
void DrawAabbEx( b3Vec3 min, b3Vec3 max, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode );

// Draw RGB axes
void DrawAxes( b3Transform transform, float size );
void DrawAxesEx( b3Transform transform, float size, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode );

// Planar overlay grid centered at `center`, lying in the plane defined by
// `normal`. `halfExtent` is the half-width per in-plane axis, `divisions` the
// cell count per axis. Emits 2 * (divisions + 1) lines along an orthonormal
// basis derived from the normal, so it is not pinned to a Y-up ground plane.
void DrawGrid( b3Vec3 center, b3Vec3 normal, float halfExtent, int divisions, Vec4 color );

// Wireframe triangle: three overlay lines a -> b -> c -> a.
void DrawTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, Vec4 color );

void DrawWireSphere( b3Transform transform, const b3Sphere* sphere, int segments, Vec4 color );
void DrawWireCapsule( b3Transform transform, const b3Capsule* capsule, int segments, Vec4 color );

#ifdef __cplusplus
} // extern "C"
#endif
