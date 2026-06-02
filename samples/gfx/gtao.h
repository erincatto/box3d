// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// GTAO (ground truth ambient occlusion):
//   * A full-res depth-only pre-pass color attachment carrying positive
//     linear view-space depth (R32F). Shapes rasterize their analytic
//     surface depth into it, the GTAO compute pipeline samples this as
//     the source for its prefiltered depth chain and for the MainPass
//     normal reconstruction taps.
//   * A D32F depth-stencil attachment that runs alongside the R32F color
//     attachment for early-Z and correct closest-fragment selection. Never
//     sampled, without it, overlapping primitives produce "whoever drew
//     last wins" in the R32F color, which is wrong. Also re-attached as
//     the depth_stencil of the transparent pass (LOAD + GREATER, no
//     depth write).
//   * A 5-mip R32F prefiltered linear-depth chain populated by a compute
//     dispatch over the depth pre-pass. Mip 0 is a verbatim copy of the
//     source, mips 1-4 use the XeGTAO depth-MIP filter (weighted average
//     that biases toward the closer of each 2x2 footprint). Consumed by
//     MainPass for the trace samples.
//   * An R32F `edges` texture written by MainPass alongside the noisy AO
//     term. Encodes 4 packed edge gradients per pixel consumed by the
//     Denoise step. R32F (not Esoterica's R8 UNORM) because sokol-shdc /
//     SPIRV-Cross only allows r32f / r32i / r32ui plus rgba/rg{32,16}
//     variants as storage_image access formats.
//   * Two R32F ping-pong AO targets `noisy0` / `noisy1`. MainPass writes
//     the raw noisy term into `noisy0`. Denoise pass 0 reads `noisy0`,
//     writes `noisy1`. Denoise pass 1 reads `noisy1`, writes `noisy0`.
//     Denoise final pass reads `noisy0` and writes `result` with the
//     XE_GTAO_OCCLUSION_TERM_SCALE rescale.
//   * A full-res R32F `result` AO texture sampled by lit shape shaders
//     via `tex_ao`. Cleared to 1.0 when GTAO is disabled at runtime,
//     written by Denoise final apply.
//
// Sizing: targets follow the main pass's width/height. ResizeGTAO is
// called once per frame with the swapchain or offscreen attachment size,
// reallocates only on actual changes.

// Must of the code and shaders for XeGTAO are ported from Esoterica Engine
// https://github.com/BobbyAnguelov/Esoterica

#pragma once

#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

void InitAmbientOcclusion( void );
void ShutdownAmbientOcclusion( void );

// Selects the slice count and steps-per-slice
typedef enum AmbientOcclusionQuality
{
	// 2 slices and 2 steps
	AO_QUALITY_MEDIUM = 0,

	// 3 slices and 3 steps
	AO_QUALITY_HIGH = 1,

	// 9 slices and 3 steps
	AO_QUALITY_ULTRA = 2,
} AmbientOcclusionQuality;

// 3 to 4 is the sweet spot
#define GTAO_MAX_DENOISE_PASSES 5

typedef struct GtaoTraceParams
{
	float radius;			  // EffectRadius, world-space sample radius
	float falloff;			  // EffectFalloffRange, fraction of radius over which the weight tapers
	float radiusMul;		  // RadiusMultiplier, per-MIP scale
	float finalValuePow;	  // FinalValuePower, non-linear contrast on the final term
	float denoiseBlurBeta;	  // DenoiseBlurBeta, bilateral blur strength
	float sampleDistPow;	  // SampleDistributionPower, exponent on the slice-sampling distribution
	float thinOcclComp;		  // ThinOccluderCompensation, extra weight for thin features
	float depthMipSampOffset; // DepthMIPSamplingOffset, bias on the prefilter MIP selection
	AmbientOcclusionQuality quality; // Trace quality tier, picks the slice and step counts internally
	int denoisePassCount;	  // Total denoise dispatches, >=1 (final apply), clamped to GTAO_MAX_DENOISE_PASSES
} GtaoTraceParams;

GtaoTraceParams GetGtaoTraceParams( void );
void SetGtaoTraceParams( GtaoTraceParams p );
GtaoTraceParams GetDefaultGtaoTraceParams( void );

AmbientOcclusionQuality GetAOQualityForPixelScale( float dpiScale );

// Ensure the owned targets are sized to match (width, height). No-op if
// the size hasn't changed, destroys + recreates on resize. Must be called
// before any gtao*View accessor each frame.
void ResizeGTAO( int width, int height );

// Depth pre-pass attachment views. Bind as pass.attachments.colors[0]
// (linear-depth R32F) and pass.attachments.depth_stencil (transient D32F,
// rasterizer-only). The renderer's DepthPrepass writes both.
sg_view GetLinearDepthAttachmentView( void );
sg_view GetPrepassDepthAttachmentView( void );

// Texture view of the R32F linear-depth target. Consumed by the GTAO
// PrefilterDepth compute.
sg_view GetLinearDepthSampleView( void );

// Dispatch the PrefilterDepth compute. Builds GTAOConstants
// from `proj` + viewport dims, then runs one ceil(w/16) x ceil(h/16)
// workgroup over the linear-depth target to populate all 5 mips of the
// prefiltered chain. Must be called after the depth pre-pass has
// written `linearDepthImage` and before MainPass tries to read
// the prefiltered chain.
void PrefilterDepth( const Mat4* proj, int viewportWidth, int viewportHeight );

// Texture view of the full prefiltered linear-depth mip chain (R32F,
// 5 mips). Consumed by MainPass. The chain view samples every mip, the
// bound shader picks a mip via textureLod at trace time.
sg_view GetPrefilterDepthSampleView( void );

// Dispatch the MainPass compute. Per-pixel: reconstructs the
// view-space normal in-place from depth derivatives, generates Hilbert
// + R2 spatial noise (NoiseIndex = 0, no TAA), runs the XeGTAO Ultra-
// tier slice trace (9 slices x 3 steps) against the prefiltered depth
// chain. Writes the noisy AO term (divided by 1.5 per
// XeGTAO_OutputWorkingTerm) to `noisy0Image` and the packed edges to
// `edgesImage`. Must be called after PrefilterDepth has populated
// the prefiltered chain.
//
// `noisy0` is fed into Denoise next, lit shaders sample `result`
// which only receives a write on the denoise final-apply dispatch.
void ComputeNoisyResult( const Mat4* proj, int viewportWidth, int viewportHeight );

// Dispatch the cross-bilateral denoise. Runs N = params.denoisePassCount
// dispatches total: (N - 1) intermediates that ping-pong noisy0 <-> noisy1
// with finalApply = 0, then one final dispatch with finalApply = 1 that
// rescales by XE_GTAO_OCCLUSION_TERM_SCALE (1.5) and writes `result`. At
// N = 1 only the final dispatch runs and reads noisy0 directly.
//
// Each dispatch is a 8x8 num threads compute over ceil(W/16) x ceil(H/8)
// workgroups (each thread handles 2 horizontal pixels). The edge texture
// written by MainPass is bound on every dispatch.
//
// Must be called after ComputeNoisyResult(). This is the last GTAO
// dispatch and lit shaders can sample `result` via GetAOResultSampleView
// right after.
void Denoise( const Mat4* proj, int viewportWidth, int viewportHeight );

// Clear the full-res `result` AO target to 1.0 (no occlusion). Used when
// GTAO is disabled at runtime (FrameInput.disable_gtao = true), lit
// shaders (which always sample tex_ao) see a fully-lit ambient.
void ClearAmbientOcclusion( void );

// Texture view of the full-res `result` AO output (R32F). Lit shape
// shaders bind this and sample at gl_FragCoord.xy * viewport.zw to look
// up their per-pixel AO term. After the Denoise final apply this is the
// rescaled [0, 1.5] visibility term (XE_GTAO_OCCLUSION_TERM_SCALE).
sg_view GetAOResultSampleView( void );

sg_sampler GetAOLinearSampler( void );

#ifdef __cplusplus
} // extern "C"
#endif
