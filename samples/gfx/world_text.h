// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// World-space text: rasterized glyphs over a distance field halo.
//
// Owns the glyph atlas, one pipeline, and a per-frame glyph instance arena.
// Labels themselves are not submitted here: text.c already collects them via
// DrawString, and this module drains the world-space ones at submit time and
// turns each character into a quad. Screen-space labels stay with ImGui,
// which is the right tool for something genuinely 2D.
//
// A label anchors at a world point but holds a fixed pixel height and ignores
// scene depth, matching the ImGui labels it replaced. What this buys over those
// is the halo and one draw call for the whole frame.
//
// The atlas bakes at init from the embedded TTF, two channels from one pass of
// stb_truetype: coverage for the glyph body, a distance field for the halo. It
// bakes at the size labels draw at, and the quads land on whole pixels, so the
// body samples one texel to one pixel and reads as crisply as a UI font. A
// distance field alone cannot manage that at 14 pixels, since resolving
// coverage from it means a smoothstep across a good part of a stem and there is
// no grid fitting to fall back on. Scale independence is no loss here: labels
// are one fixed size, and a size change just rebakes.
//
// Pass placement matches overlay.c exactly: the renderer calls WorldTextSubmit
// in the post-tone map pass, after tone map and the highlight outline.

#pragma once

#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Bakes the glyph atlas from the embedded font, so labels do not depend on the
// working directory. Logs and stays inert if the bake fails rather than taking
// the whole app with it.
void WorldTextInit( void );
void WorldTextShutdown( void );

// Encode world label draws into the currently-open render pass. Called once
// per frame from the post-tone map pass, after tone map + highlight outline.
// No-op when no world labels were submitted this frame.
//
// Arguments match OverlaySubmit minus the depth pre-pass views, which labels
// have no use for, see overlay.h for what each one carries.
void WorldTextSubmit( int width, int height, const Mat4* view, const Mat4* viewInv, const Mat4* proj, const Mat4* projInv,
					  b3Vec3 cameraPos, float time, sg_pixel_format colorFormat, sg_pixel_format depthFormat );

#ifdef __cplusplus
} // extern "C"
#endif
