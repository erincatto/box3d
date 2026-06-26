// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Per-frame append API + the render pass that rasterizes a small set
// "highlighted" shapes into the renderer-owned R8 mask target. The pass
// runs once per frame, after the GTAO depth prepass, before tonemap, the
// outline post-pass then reads the mask and composites silhouette colors
// over the resolved scene.
//
// Covers spheres, capsules, and any registered geometry (hull, mesh,
// height field). The geometry path is occlusion unaware, so a large
// selected shape outlines its whole projected footprint.
//
// Encodes one of three states per pixel as the kind value passed in:
// 1 = hover -> R8 UNORM 0.5
// 2 = select -> R8 UNORM 1.0
// 0 -> unhighlighted, never written
//
// Single-threaded, render-thread only. Submission counts capped per
// shape class, overflows log and drop.

#pragma once

#include "gfx/geometry_registry.h"
#include "gfx/utility.h"
#include "sokol_gfx.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum HighlightKind
{
	HIGHLIGHT_KIND_NONE = 0,
	HIGHLIGHT_KIND_HOVER = 1,
	HIGHLIGHT_KIND_SELECT = 2,
} HighlightKind;

void HighlightMaskInit( void );
void HighlightMaskShutdown( void );

// Reset per-frame counts. Called from ResetFrameArena.
void HighlightMaskBeginFrame( void );

// Append a highlighted sphere. transform supplies the world-space center via
// its translation, rotation is ignored (a sphere is rotationally invariant
// for silhouette purposes). Logs + drops on overflow.
void AppendHighlightSphere( b3Transform transform, float radius, HighlightKind kind );

// Append a highlighted capsule. transform follows the same convention as
// AppendCapsule (rotation places local +X along the long axis,
// translation is the capsule center). halfLength and radius are
// world-space scalars. Logs + drops on overflow.
void AppendHighlightCapsule( b3Transform transform, float halfLength, float radius, HighlightKind kind );

// Append highlighted geometry (hull, mesh, or height field). handle resolves
// to a registered geometry (vbo/ibo). transform + scale match AppendMesh,
// rotation matters because the geometry isn't rotationally symmetric. Logs +
// drops on overflow.
void AppendHighlightGeometry( MeshHandle handle, b3Transform transform, b3Vec3 scale, HighlightKind kind );

// True when at least one shape was submitted this frame. Lets the caller
// skip mask + outline passes entirely on the common empty case.
bool HasHighlightMaskContent( void );

// Open a single-sample render pass against (maskAttachView,
// depthAttachView), clear the mask to 0, and rasterize all submitted
// highlights. Depth-tests GREATER_EQUAL against the supplied prepass depth
// (no write). No-op when no highlights were submitted.
void SubmitHighlightMask( int width, int height, const Mat4* viewProj, b3Vec3 cameraPosWorld, sg_view maskAttachView,
						  sg_view depthAttachView );

#ifdef __cplusplus
} // extern "C"
#endif
