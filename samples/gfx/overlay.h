// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Overlay pipeline, lines and points.
//
// Owns two pipelines (overlay_line, overlay_point), two per-frame instance
// arenas, and the upload-side storage buffers. The renderer wires the
// per-frame reset and the draw call.
//
// Pass placement: the renderer calls OverlaySubmit in the post-tone map
// pass, the same swap chain (or offscreen test) pass that AgX tone map and
// the highlight outline write to, after both and before sg_end_pass. So
// overlay fragments are display-referred: a submitted color reaches the
// screen verbatim, NOT run through exposure + AgX. This keeps debug
// overlays consistent with the ImGui text labels (also post-tone map) and
// makes their colors WYSIWYG. The target is single-sample, the line/point
// shaders carry their own SDF coverage AA, so MSAA isn't needed.
//
// Depth-test = ALWAYS (depth-write off), the fragment shader samples the
// GTAO linear-depth pre-pass for the DIM / DASHED / HIDE occlusion-mode
// branches. Pipelines are created lazily per output format pair (the
// post-tone map target's format isn't known until submit).

#pragma once

#include "gfx/draw_overlay.h"
#include "gfx/utility.h"
#include "sokol_gfx.h"

#ifdef __cplusplus
extern "C"
{
#endif

void OverlayInit( void );
void OverlayShutdown( void );

// Reset both per-primitive instance arenas. Called from
// ResetFrameArena once per frame before any append.
void OverlayResetFrame( void );

// Append an overlay line. Dropped if the arena overflows.
void OverlayAppendLine( b3Vec3 a, b3Vec3 b, Vec4 linearColor, float thickness, OverlayThicknessUnit thicknessUnit,
						OverlayOcclusionMode occlusionMode );

// Append an overlay point. Dropped if the arena overflows.
void OverlayAppendPoint( b3Vec3 p, Vec4 linearColor, float size, OverlayThicknessUnit sizeUnit, OverlayOcclusionMode occlusionMode );

// Encode line + point draws into the currently-open render pass. The
// renderer calls this once per frame in the post-tone map pass, after
// tone map + the highlight outline. No-op if both arenas are empty.
//
// width, height: viewport size in pixels (for ub_frame.viewport).
// view, viewInv, proj, projInv: camera matrices paired with their inverses
// (produced by the camera alongside the forward matrices). The overlay
// shaders read view_proj and inv_view_proj from ub_frame, inv_view_proj is
// built here as viewInv * projInv, no matrix inversion at runtime.
// cameraPos: world-space camera position
// time: seconds since InitRenderer, packed into camera_pos.w.
// linearDepthView: texture view over the R32F linear-depth pre-pass target.
// linearDepthSampler: matching sampler (existing GetAOLinearSampler reused).
// colorFormat: pixel format of the pass's color attachment.
// depthFormat: pixel format of the pass's depth attachment, or
// SG_PIXELFORMAT_NONE for a color-only target. Both select/build the cached pipeline pair.
void OverlaySubmit( int width, int height, const Mat4* view, const Mat4* viewInv, const Mat4* proj, const Mat4* projInv,
					b3Vec3 cameraPos, float time, sg_view linearDepthView, sg_sampler linearDepthSampler, sg_pixel_format colorFormat,
					sg_pixel_format depthFormat );

// Per-frame primitive counts
int OverlayLineCount( void );
int OverlayPointCount( void );

#ifdef __cplusplus
} // extern "C"
#endif
