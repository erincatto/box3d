// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "gfx/draw_overlay.h"
#include "gfx/utility.h"

#include "box3d/collision.h"

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

// Solid shape wrappers over the impostor primitives. They take Box3D shape
// types and fold the shape-local frame into the transform, so samples draw
// straight from collision data.
void DrawSolidSphere( b3Transform transform, b3Sphere sphere, Vec4 color );
void DrawSolidCapsule( b3Transform transform, b3Capsule capsule, Vec4 color );

// Convex hull wireframe. Each undirected edge once, in world space.
void DrawHull( b3Transform transform, const b3HullData* hull, Vec4 color );

// Plane as a unit wireframe quad through the point, plus a short normal and a dot.
void DrawPlane( b3Vec3 normal, b3Vec3 point, Vec4 color );

// All overlay submissions land in the renderer's per-frame overlay arenas
// and are drawn into the MSAA HDR scene target after opaque + sky, before
// the AgX tone map pass. Colors are straight (non-premultiplied) linear RGBA.

void DrawLine( b3Vec3 a, b3Vec3 b, Vec4 color );
void DrawLineEx( b3Vec3 a, b3Vec3 b, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode );

// Debug point, size in pixels. Dimmed rather than hidden when occluded so
// contact and witness points stay visible through geometry.
void DrawPoint( b3Vec3 p, float size, Vec4 color );
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

// AABB wireframe expanded outward by `extension` on every side.
void DrawBounds( b3AABB bounds, float extension, Vec4 color );

// Draw RGB axes
void DrawAxes( b3Transform transform, float size );
void DrawAxesEx( b3Transform transform, float size, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode );

// Planar overlay grid centered at `center`, lying in the plane defined by
// `normal`. `halfExtent` is the half-width per in-plane axis, `divisions` the
// cell count per axis. Emits 2 * (divisions + 1) lines along an orthonormal
// basis derived from the normal, so it is not pinned to a Y-up ground plane.
void DrawGrid( b3Vec3 center, b3Vec3 normal, float halfExtent, int divisions, Vec4 color );

// Gray ground grid in the XZ plane, `size` meters half-extent with 1 m cells.
void DrawGroundGrid( int size );

// Wireframe triangle: three overlay lines a -> b -> c -> a.
void DrawTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, Vec4 color );

void DrawWireSphere( b3Transform transform, const b3Sphere* sphere, int segments, Vec4 color );
void DrawWireCapsule( b3Transform transform, const b3Capsule* capsule, int segments, Vec4 color );

// printf-style positioned text label in the draw-origin-relative frame. The renderer projects
// with its latched camera.
void DrawString3D( b3Vec3 point, Vec4 color, const char* format, ... );

// World space draws. Set the draw origin to the camera focus each frame (SetDrawOrigin); these
// take absolute world positions and subtract the origin in double before handing float
// coordinates to the relative-frame draws above. Identity in float mode (b3Pos aliases b3Vec3,
// origin zero), so samples can call them unconditionally.
void SetDrawOrigin( b3Pos origin );

void DrawWorldCube( b3WorldTransform transform, b3Vec3 scale, Vec4 color );
void DrawWorldSphere( b3WorldTransform transform, float radius, Vec4 color );
void DrawWorldCapsule( b3WorldTransform transform, float halfLength, float radius, Vec4 color );
void DrawWorldHull( b3WorldTransform transform, const b3HullData* hull, Vec4 color );
void DrawWorldLine( b3Pos a, b3Pos b, Vec4 color );
void DrawWorldPoint( b3Pos p, float size, Vec4 color );
void DrawWorldArrow( b3Pos a, b3Pos b, Vec4 color );
void DrawWorldAxes( b3WorldTransform transform, float size );
void DrawWorldAabb( b3AABB bounds, Vec4 color );
void DrawWorldString( b3Pos point, Vec4 color, const char* format, ... );

#ifdef __cplusplus
} // extern "C"
#endif
