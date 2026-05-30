// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Single-sample R8 UNORM image at scene resolution, written by the
// highlight_mask pass and sampled by the highlight_outline post-pass. Sized
// alongside the scene target, both resize when the framebuffer does.
//
// The mask encodes one of three states as a UNORM byte: 0.0 = no highlight,
// 0.5 = hover, 1.0 = select. R8 UNORM (rather than R8_UINT) so the outline
// shader can sample via the standard texture/sampler path, sokol-shdc's
// integer-texture binding has cross-backend quirks we'd rather not eat for
// a 3-value mask. Thresholds in the FS (>0.25, >0.75) recover the discrete
// kinds.
//
// Single-sample (no MSAA), keeps the mask pipeline matched to the GTAO
// prepass depth (also 1 sample) it depth-tests against. Silhouette softness
// comes from the outline shader's tap pattern + alpha blend, not from MSAA
// on the mask itself.

#pragma once

#include "sokol_gfx.h"

#ifdef __cplusplus
extern "C"
{
#endif

void InitHighlightTarget( void );
void ShutdownHighlightTarget( void );

// Resize to (width, height) if changed. No-op when stable. Recreates the
// image + views on size change.
void ResizeHighlightTarget( int width, int height );

// Color-attachment view over the mask image. Bound by the highlight_mask pass.
sg_view GetHighlightTargetMaskAttachView( void );

// Texture-sample view over the mask image. Bound by the highlight_outline pass.
sg_view GetHighlightTargetMaskSampleView( void );

#ifdef __cplusplus
} // extern "C"
#endif
