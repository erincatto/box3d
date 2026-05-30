// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

// Debug-draw compatibility layer for the samples.
//
// The samples were written against the old scene renderer, whose draw calls
// took a Scene* (and DrawWorldString a Camera*) plus a packed b3HexColor.
// render3d's gfx/draw.h is a flat C API with scalar args and linear Vec4
// colors, and several of its names collide with the old ones (DrawSphere,
// DrawLine, ...). A C++ overload can't share a name with an extern "C"
// function, so a sample translation unit can't see gfx/draw.h and these at
// once. This header keeps the exact old spelling and forwards through gfx via
// draw_bridge, so a sample ports by swapping includes and input constants, not
// by rewriting its draw calls. The Scene*/Camera* arguments are vestigial: the
// renderer owns that state now, so they are accepted and ignored.

#pragma once

#include "gfx/utility.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "box3d/types.h"

struct Scene;
class Camera;

// Packed 0xRRGGBB (sRGB) to linear RGBA, matching the renderer's shape path.
Vec4 MakeColor( b3HexColor hexColor );

// Shape draws. transform places the shape; shape-local centers fold in.
void DrawSphere( Scene* scene, const b3Transform& transform, const b3Sphere& sphere, b3HexColor color );
void DrawCapsule( Scene* scene, const b3Transform& transform, const b3Capsule& capsule, b3HexColor color );
void DrawHull( Scene* scene, const b3Transform& transform, const b3Hull* hull, b3HexColor color, bool drawNormals );
void DrawBox( Scene* scene, b3Vec3 extents, const b3Transform& transform, b3HexColor color );

// Overlay primitives.
void DrawPoint( Scene* scene, b3Vec3 point, float size, b3HexColor color );
void DrawLine( Scene* scene, b3Vec3 point1, b3Vec3 point2, b3HexColor color );
void DrawArrow( Scene* scene, b3Vec3 point1, b3Vec3 point2, float size, b3HexColor color );
void DrawCross( Scene* scene, b3Vec3 center, float radius, b3HexColor color );
void DrawBounds( Scene* scene, b3AABB bounds, float extension, b3HexColor color );
void DrawTransform( Scene* scene, const b3Transform& transform, float length );

// Wireframe triangle (the old DrawFace was filled; debug-vis tradeoff).
void DrawFace( Scene* scene, b3Vec3 vertex1, b3Vec3 vertex2, b3Vec3 vertex3, b3HexColor color );

// Ground grid in the XZ plane, `size` half-extent in meters with 1 m cells.
void DrawGrid( Scene* scene, int size );

// World-space text label. The renderer projects with its latched camera; the
// Camera* is accepted for source compatibility and ignored.
void DrawWorldString( Camera* camera, b3Vec3 point, b3HexColor color, const char* format, ... );

// The old renderer toggled a shader-side ground grid per scene. The new
// renderer has no equivalent, so this is a no-op kept so call sites compile.
void EnableGrid( Scene* scene, bool enable );
