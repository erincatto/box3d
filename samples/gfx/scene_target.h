// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// HDR scene target.
//
// Renderer-owned 4x MSAA color + matching MSAA depth + single-sample
// resolve color, all sized to the current frame's render width/height.
// The opaque-scene pass writes into the MSAA color (with hardware resolve
// into the resolve image at end-of-pass), the tonemap pass samples the
// resolve image and writes the final swapchain/test target.
//
// Color format is fixed at RGBA16F across all backends, renderable +
// filterable + MSAA-resolvable on D3D11, Metal, and GL 4.5 core. Alpha is
// kept (vs. R11G11B10F) because the transparent pass uses it for
// sorted-alpha transparency.
//
// Depth format mirrors env.defaults.depth_format so the shape pipelines
// (depth.pixel_format = env->defaults.depth_format) remain compatible.
//
// Per-frame sizing: RenderFrame calls ResizeSceneTarget(W,H) before
// opening the lit pass. Recreates on size change, no-op when stable.

#pragma once

#include "sokol_gfx.h"

#ifdef __cplusplus
extern "C"
{
#endif

// MSAA sample count for the scene_hdr pass. Every pipeline that targets
// scene_hdr (lit shapes in renderer.c, sky in sky.c, MSAA edge overlay in
// edges.c) must declare a matching pipeline sample_count, so they all
// reference this constant. Valid values: 1, 2, 4, 8. At 1, the resolve
// attachment is omitted (sokol_gfx rejects a resolve paired with a 1x
// color attachment) and the scene color image is sampled directly by the
// tone map pass, the transparent pass binds it as a regular color
// attachment with LOAD. Change here, rebuild, re-run.
#define SCENE_SAMPLE_COUNT 4

void InitSceneTarget( const sg_environment* env );
void ShutdownSceneTarget( void );

void ResizeSceneTarget( int width, int height );

// Attachment set the opaque-scene pass binds: colors[0] = MSAA color view,
// resolves[0] = single-sample resolve view, depth_stencil = MSAA depth view.
sg_attachments GetSceneTargetAttachments( void );

// Texture-view over the single-sample resolved color image. The tone map
// pass samples this.
sg_view GetSceneTargetResolveSampleView( void );

// Color-attachment view over the single-sample resolved image, for the
// transparent pass that runs after the opaque MSAA pass has resolved.
// The transparent pass binds this with LOADACTION_LOAD so it blends over
// the resolved opaque scene rather than overwriting it.
sg_view GetSceneTargetResolveColorAttachView( void );

// MSAA depth-attachment format used by the scene target. Mirrors the
// env.defaults.depth_format passed to sceneTargetInit, surfaced for pass
// modules (e.g. the edge overlay) that want to add a pipeline state
// matching the MSAA scene attachments without re-querying the env.
sg_pixel_format GetSceneTargetDepthFormat( void );

#ifdef __cplusplus
} // extern "C"
#endif
