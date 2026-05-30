// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// AgX tonemap pass.
//
// Final post-process: reads the resolved HDR scene color (RGBA16F) and
// writes display-encoded sRGB to a linear-format LDR target (swapchain
// in the demo, caller-owned attachment via RenderFrameOffscreen). See
// the shader at shaders/post/tonemap.glsl for the AgX implementation
// and the rationale for skipping the trailing pow(2.2) EOTF.
//
// The pass owns its pipeline cache. The output color format isn't known at
// init time, RenderFrame targets the swapchain format, while
// RenderFrameOffscreen targets whatever the caller supplies. Currently
// both resolve to env.defaults.color_format, but the cache covers a future
// where they diverge (capacity = 2).
//
// Single-threaded, render-thread only.

#pragma once

#include "sokol_gfx.h"

#ifdef __cplusplus
extern "C"
{
#endif

void InitToneMap( void );
void ShutdownToneMap( void );

// Encodes one fullscreen AgX pass into the currently-open render pass.
// The caller is responsible for sg_begin_pass / sg_end_pass on the output
// target, this function only binds + draws.
//
// sceneColorView: texture-view over the resolved (single-sample) HDR
//                       scene color image.
// outputColorFormat: pixel format of the color attachment in the open
//                       pass. Pipeline cache key.
// outputDepthFormat: pixel format of the depth attachment in the open
//                       pass, or SG_PIXELFORMAT_NONE if the target has no
//                       depth. The swapchain path supplies the env default,
// offscreen tests pass NONE. Pipeline cache key,
//                       sokol enforces a pipeline-vs-attachment match even
//                       though the tonemap pipeline doesn't touch depth.
// ev: exposure stops applied before AgX. -2.0 matches
//                       the spec for Preetham at physical strength.
// saturation: AgX-look saturation applied after the AgX sigmoid.
//                       1.0 is the stock Standard look. >1.0 counteracts
//                       AgX's path-to-white desaturation.
void SubmitToneMap( sg_view sceneColorView, sg_pixel_format outputColorFormat, sg_pixel_format outputDepthFormat, float ev,
					float saturation );

#ifdef __cplusplus
} // extern "C"
#endif
