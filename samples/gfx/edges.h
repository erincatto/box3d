// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Renders per-shape edges for hulls, meshes, and heightfields registered in
// the geometry registry. Same shader / batching for both arenas. Split
// into two submission paths to match the silhouette quality of the
// rest of the scene:
//
// Opaque path (RenderEdgesInMSAA): opaque-arena batches (meshes,
//   heightfields, opaque hulls if policy 1b is ever relaxed). Draws
//   inside the caller's currently-open MSAA scene pass, picking up the
//   same 4x MSAA as the opaque shapes. Pipeline tracks scene_target's
//   color/depth formats and sample_count.
//
// Transparent path (RenderEdgesPostTransparent): transparent-arena batches
//   (hulls per policy 1b). Opens its own single-sample pass against the
//   resolved scene color + GTAO prepass depth, so edges sit on top of the
//   transparent draws that have already blended into the resolve. MSAA
//   isn't available here (the resolve already happened), the FS's
//   fwidth-based AA does what it can.

#pragma once

#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct EdgeOverlayParams
{
	bool enabled;
	bool showHulls;
	bool showMeshes;
	bool showHeightfields;
	bool showEdgeConvexity;
	float thicknessPx;
	float zBias;	  // reverse-Z nudge toward camera, ~5e-5 is a good start
	Vec4 convexColor; // straight (non-premultiplied) linear RGBA, premultiplied inside
	Vec4 concaveColor;
	Vec4 flatColor;
} EdgeOverlayParams;

EdgeOverlayParams GetDefaultEdgeParams( void );

void InitEdges( void );
void ShutdownEdges( void );

// Draw edge batches into the caller's currently-open render pass. The pass
// must be the MSAA scene pass (sceneTargetAttachments), the pipeline
// matches its color format (RGBA16F), depth format (sceneTargetDepthFormat),
// and sample_count (4).
void RenderEdgesInMSAA( int width, int height, const Mat4* view, const Mat4* proj, sg_view opaqueInstanceView,
						sg_view transparentInstanceView, const EdgeOverlayParams* params );

// Open a single-sample render pass against (colorAttachView, depthAttachView)
// and draw transparent-arena edge batches. When the
// EDGES_TRANSPARENT_INCLUDES_OPAQUE experiment is enabled, opaque-arena
// batches are also drawn here (and the MSAA path becomes a no-op), so the
// caller must pass a valid opaqueInstanceView too.
void RenderEdgesPostTransparent( int width, int height, const Mat4* view, const Mat4* proj, sg_view colorAttachView,
								 sg_view depthAttachView, sg_view opaqueInstanceView, sg_view transparentInstanceView,
								 const EdgeOverlayParams* params );

#ifdef __cplusplus
} // extern "C"
#endif
