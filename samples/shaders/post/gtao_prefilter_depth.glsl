// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// GTAO PrefilterDepth compute shader.
//
// Port of Esoterica's PrefilterDepth.esf, which is XeGTAO's
// XeGTAO_PrefilterDepths16x16, a single 16x16-pixel tile per group,
// 8x8 threads, each thread handling 2x2 destination pixels. Builds the
// 5-mip prefiltered linear-depth chain consumed by the GTAO main pass.
// Mip0 is the raw input, mips 1..4 use the XeGTAO weighted-
// average depth-MIP filter which biases toward the closer of the four
// taps in each 2x2 footprint (suppresses thin-feature pulled mips).
//
// Departures from XeGTAO_PrefilterDepths16x16:
//
//   * Input depth is **already positive linear view-Z** (Box3D's
//     R32F prepass target), not NDC reverse-Z. Esoterica's
//     XeGTAO_ScreenSpaceToViewSpaceDepth call collapses to a pass-
// through, we just clamp. GTAOConstants.DepthUnpackConsts is
//     therefore unused (kept in the UBO for layout parity).
//
// * Mip-0 fetches use texelFetch on the source texture instead
//     GatherRed + offset(1,1). The mathematical result is identical
//     (point-sampled raw texels at a 2x2 footprint), but texelFetch
//     sidesteps the documented textureGather channel-ordering /
//     Y-flip pitfalls. Cost is 4 texel reads vs 1 gather op,
//     a single dispatch per frame, doesn't matter.
//
//   * lpfloat is `float`. Matches Esoterica's
//     XE_GTAO_USE_HALF_FLOAT_PRECISION=0 / XE_GTAO_FP32_DEPTHS=1
//     configuration, min16float / float16_t paths don't survive
//     sokol-shdc cleanly.
//
// Dispatch: ceil(width / 16) x ceil(height / 16) x 1. Each group writes
// a 16x16 mip-0 region, 8x8 mip-1, 4x4 mip-2, 2x2 mip-3, 1x1 mip-4.
//
// Esoterica early-returns OOB threads at the top of the shader. FXC
// (D3D11) rejects that as "thread sync operation in varying flow"
// (HLSL X3663) when the subsequent barrier participations diverge.
// We instead let every thread run the full reduction so all barriers
// see uniform participation, and guard each imageStore individually
// against the viewport bound. Source-texel reads are clamped to the
// in-bounds edge so OOB threads contribute consistent (replicated-
// edge) depth values to the mip-1..4 reductions. These values are
// only used to keep the groupshared scratch populated, the actual
// mip writes for those threads are skipped by the bounds guard.

#pragma sokol @module gtao_prefilter_depth

#pragma sokol @ctype vec2 b3Vec2
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype mat4 Mat4

#pragma sokol @cs cs

// Per-shader subset of Esoterica's XeGTAO::GTAOConstants, only the
// fields PrefilterDepth's `XeGTAO_DepthMIPFilter` and `main ` actually
// read. The block has an explicit instance name (`gtao.X`) so SPIRV-
// Cross doesn't synthesize one (`_NNN.X`) when emitting GLSL for the GL
// backend. Without that, each unread field that the driver's GLSL
// linker dead-strips triggers a sokol_gfx `glGetUniformLocation` warning
// with an unreadable synthetic name. Keeping the block trimmed to only
// the read fields means there's nothing for the linker to strip.
layout(binding = 0) uniform ub_gtao
{
	ivec2 ViewportSize;
	float EffectRadius;
	float EffectFalloffRange;
	float RadiusMultiplier;
} gtao;

// Outputs: one storage_image per mip level. R32F write-only. Each mip
// has its own bind slot (0..4), sokol-gfx wires each to a sg_view that
// targets a specific mip of the 5-mip prefilterDepth image.
layout(binding = 0, r32f) uniform writeonly image2D out_depth0;
layout(binding = 1, r32f) uniform writeonly image2D out_depth1;
layout(binding = 2, r32f) uniform writeonly image2D out_depth2;
layout(binding = 3, r32f) uniform writeonly image2D out_depth3;
layout(binding = 4, r32f) uniform writeonly image2D out_depth4;

// Source: full-res positive linear view-Z. Bound as a sampled texture so
// texelFetch is available. The sampler is point-clamp (texelFetch ignores
// it but sokol-gfx requires a sampler binding for any texture binding).
// Binding 5 because sokol-shdc shares the storage_image/texture bind-
// space across the compute stage even though the underlying backends
// use disjoint SRV/UAV register banks, storage images take 0..4 so the
// texture starts at 5.
layout(binding = 5) uniform texture2D tex_src_depth;
layout(binding = 0) uniform sampler smp_src_depth;

// XeGTAO_ClampDepth, clamps to fp32 finite. Matches XeGTAO.esh:648-655
// for the XE_GTAO_USE_HALF_FLOAT_PRECISION=0 branch (FLT_MAX upper
// bound). Our sky sentinel of 1e9 sits comfortably below this.
float XeGTAO_ClampDepth(float depth) {
	return clamp(depth, 0.0, 3.402823466e+38);
}

// XeGTAO_DepthMIPFilter, weighted average that biases toward the
// closest tap (smallest depth) and suppresses contributions from taps
// outside an effect-radius-scaled falloff window. Verbatim port
// XeGTAO.esh:619-644, with XE_GTAO_USE_DEFAULT_CONSTANTS=0 branches
// since we drive everything from the UBO.
float XeGTAO_DepthMIPFilter(float depth0, float depth1, float depth2, float depth3) {
	float maxDepth = max(max(depth0, depth1), max(depth2, depth3));
	
const float depthRangeScaleFactor = 0.75; // "found empirically :)" XeGTAO.esh:624
float effectRadius = depthRangeScaleFactor * gtao.EffectRadius * gtao.RadiusMultiplier;
float falloffRange = gtao.EffectFalloffRange * effectRadius;
float falloffFrom = effectRadius * (1.0 - gtao.EffectFalloffRange);
float falloffMul = -1.0 / falloffRange;
float falloffAdd = falloffFrom / falloffRange + 1.0;

float weight0 = clamp((maxDepth - depth0) * falloffMul + falloffAdd, 0.0, 1.0);
float weight1 = clamp((maxDepth - depth1) * falloffMul + falloffAdd, 0.0, 1.0);
float weight2 = clamp((maxDepth - depth2) * falloffMul + falloffAdd, 0.0, 1.0);
float weight3 = clamp((maxDepth - depth3) * falloffMul + falloffAdd, 0.0, 1.0);

float weightSum = weight0 + weight1 + weight2 + weight3;
return (weight0 * depth0 + weight1 * depth1 +
weight2 * depth2 + weight3 * depth3) / weightSum;
}

// Mip-2..4 inter-thread reduction scratch. groupshared, written by the
// mip-N producer threads and read by the mip-(N+1) producers after a
// barrier. Mirrors XeGTAO.esh:657.
shared float g_scratchDepths[8][8];

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1)in;

void main() {
ivec2 dispatchThreadID = ivec2(gl_GlobalInvocationID.xy);
ivec2 groupThreadID = ivec2(gl_LocalInvocationID.xy);
ivec2 viewportMax = ivec2(gtao.ViewportSize) - 1;

// MIP 0
// Esoterica reads via GatherRed at the texel-corner UV with offset
// (1,1), we read the same four texels via texelFetch to avoid the
// textureGather channel-ordering / backend-Y-flip ambiguity (see
// header). Indexing matches Esoterica's (depth0, depth1, depth2,
// depth3) = (top-left, top-right, bottom-left, bottom-right) within
// the 2x2 destination block. Source coords are clamped to the
// in-bounds edge so OOB threads still feed valid (replicated-edge)
// depths into the mip-1..4 reductions, the actual mip writes for
// those threads are gated below.
ivec2 pixCoord = dispatchThreadID * 2;
ivec2 p0 = clamp(pixCoord + ivec2(0, 0), ivec2(0), viewportMax);
ivec2 p1 = clamp(pixCoord + ivec2(1, 0), ivec2(0), viewportMax);
ivec2 p2 = clamp(pixCoord + ivec2(0, 1), ivec2(0), viewportMax);
ivec2 p3 = clamp(pixCoord + ivec2(1, 1), ivec2(0), viewportMax);
float depth0 = XeGTAO_ClampDepth(texelFetch(sampler2D(tex_src_depth, smp_src_depth), p0, 0).r);
float depth1 = XeGTAO_ClampDepth(texelFetch(sampler2D(tex_src_depth, smp_src_depth), p1, 0).r);
float depth2 = XeGTAO_ClampDepth(texelFetch(sampler2D(tex_src_depth, smp_src_depth), p2, 0).r);
float depth3 = XeGTAO_ClampDepth(texelFetch(sampler2D(tex_src_depth, smp_src_depth), p3, 0).r);

if (all(lessThan(pixCoord + ivec2(0, 0), ivec2(gtao.ViewportSize)))) {
	imageStore(out_depth0, pixCoord + ivec2(0, 0), vec4(depth0, 0.0, 0.0, 0.0));
}
if (all(lessThan(pixCoord + ivec2(1, 0), ivec2(gtao.ViewportSize)))) {
	imageStore(out_depth0, pixCoord + ivec2(1, 0), vec4(depth1, 0.0, 0.0, 0.0));
}
if (all(lessThan(pixCoord + ivec2(0, 1), ivec2(gtao.ViewportSize)))) {
	imageStore(out_depth0, pixCoord + ivec2(0, 1), vec4(depth2, 0.0, 0.0, 0.0));
}
if (all(lessThan(pixCoord + ivec2(1, 1), ivec2(gtao.ViewportSize)))) {
	imageStore(out_depth0, pixCoord + ivec2(1, 1), vec4(depth3, 0.0, 0.0, 0.0));
}

// MIP 1
// Every thread produces one mip-1 candidate (always stored in
// scratch so the mip-2 reduction has a uniform view), conditionally
// written to the mip-1 image only if dispatchThreadID is in bounds.
float dm1 = XeGTAO_DepthMIPFilter(depth0, depth1, depth2, depth3);
if (all(lessThan(dispatchThreadID, ivec2(gtao.ViewportSize)))) {
	imageStore(out_depth1, dispatchThreadID, vec4(dm1, 0.0, 0.0, 0.0));
}
g_scratchDepths[groupThreadID.x][groupThreadID.y] = dm1;

barrier();

// MIP 2
// 2x2 reduction over the group's 8x8 mip-1 footprint, one thread per
// mip-2 texel, picked at the (x%2,y%2)==(0,0) corner. Reads stride 1.
// The reduction itself happens uniformly (predicated only on local
// group coords), the bounds check gates the imageStore, never the
// barrier path.
float dm2 = 0.0;
bool isMip2Producer = all(equal(groupThreadID & ivec2(1), ivec2(0)));
if (isMip2Producer) {
	float inTL = g_scratchDepths[groupThreadID.x + 0][groupThreadID.y + 0];
	float inTR = g_scratchDepths[groupThreadID.x + 1][groupThreadID.y + 0];
	float inBL = g_scratchDepths[groupThreadID.x + 0][groupThreadID.y + 1];
	float inBR = g_scratchDepths[groupThreadID.x + 1][groupThreadID.y + 1];
	
	dm2 = XeGTAO_DepthMIPFilter(inTL, inTR, inBL, inBR);
	// gl_GlobalInvocationID is unsigned by spec -> dispatchThreadID is
	// always >= 0, so an unsigned right-shift produces the same value as
	// `dispatchThreadID / 2` without the HLSL signed-divide warning
	// (X3556). Compute once + reuse for the bounds-check + imageStore.
	ivec2 dispatchID2 = ivec2(uvec2(dispatchThreadID) >> 1u);
	if (all(lessThan(dispatchID2, ivec2(gtao.ViewportSize)))) {
		imageStore(out_depth2, dispatchID2, vec4(dm2, 0.0, 0.0, 0.0));
	}
	g_scratchDepths[groupThreadID.x][groupThreadID.y] = dm2;
}

barrier();

// MIP 3
// Reads stride 2 from the same scratch slab, one mip-3 texel per
// 4x4 thread block, picked at the (x%4,y%4)==(0,0) corner.
float dm3 = 0.0;
bool isMip3Producer = all(equal(groupThreadID & ivec2(3), ivec2(0)));
if (isMip3Producer) {
	float inTL = g_scratchDepths[groupThreadID.x + 0][groupThreadID.y + 0];
	float inTR = g_scratchDepths[groupThreadID.x + 2][groupThreadID.y + 0];
	float inBL = g_scratchDepths[groupThreadID.x + 0][groupThreadID.y + 2];
	float inBR = g_scratchDepths[groupThreadID.x + 2][groupThreadID.y + 2];
	
	dm3 = XeGTAO_DepthMIPFilter(inTL, inTR, inBL, inBR);
	ivec2 dispatchID4 = ivec2(uvec2(dispatchThreadID) >> 2u);
	if (all(lessThan(dispatchID4, ivec2(gtao.ViewportSize)))) {
		imageStore(out_depth3, dispatchID4, vec4(dm3, 0.0, 0.0, 0.0));
	}
	g_scratchDepths[groupThreadID.x][groupThreadID.y] = dm3;
}

barrier();

// MIP 4
// Final reduction, single thread per group writes a single texel.
// Esoterica's commented-out scratch write at the end is dead code
// (no consumer), we drop the comment.
if (all(equal(groupThreadID, ivec2(0)))) {
	float inTL = g_scratchDepths[0][0];
	float inTR = g_scratchDepths[4][0];
	float inBL = g_scratchDepths[0][4];
	float inBR = g_scratchDepths[4][4];
	
	float dm4 = XeGTAO_DepthMIPFilter(inTL, inTR, inBL, inBR);
	ivec2 dispatchID8 = ivec2(uvec2(dispatchThreadID) >> 3u);
	if (all(lessThan(dispatchID8, ivec2(gtao.ViewportSize)))) {
		imageStore(out_depth4, dispatchID8, vec4(dm4, 0.0, 0.0, 0.0));
	}
}
}
#pragma sokol @end

#pragma sokol @program prog cs
