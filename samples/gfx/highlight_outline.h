// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Fullscreen pass that reads the R8 highlight mask (filled by the
// highlight_mask pass) and composites silhouette outline colors onto the
// already-tone-mapped scene image. Runs against the same swap chain /
// offscreen attachment the tone-map pass wrote to, with LOAD on color and
// no depth.
//
// Two outline styles, both authored in linear premultiplied alpha space:
// hover, typically thinner, more subtle (default 1 px, cool color)
// select, wider and more saturated (default 2 px, warm color).

#pragma once

#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct HighlightParams
{
	bool enabled;
	float hoverThicknessPx;
	float selectThicknessPx;
	// straight (non-premultiplied) linear RGBA, premultiplied inside
	Vec4 hoverColor;
	Vec4 selectColor;
} HighlightParams;

HighlightParams GetDefaultHighlightParams( void );

void InitHighlightOutline( void );
void ShutdownHighlightOutline( void );

// Submit the outline post-pass against the currently-open render pass.
// The caller is responsible for having opened a pass against the desired
// destination (the swap-chain after tone map, or the offscreen tone map target)
// with LOAD action on color. The pass's color attachment format must match
// colorFormat and its depth format must match depthFormat (depth is not
// read or written by this pass, but the pipeline declaration needs the
// formats to match for sokol-gfx validation).
//
// maskSampleView is the texture-sample view over the R8 mask image. The
// pipeline is cached per (colorFormat, depthFormat) pair so repeat calls
// from RenderFrame / RenderFrameOffscreen reuse one pipeline each.
void SubmitHighlightOutline( sg_view maskSampleView, sg_pixel_format colorFormat, sg_pixel_format depthFormat,
							 const HighlightParams* params );

#ifdef __cplusplus
} // extern "C"
#endif
