// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// GTAO Denoise compute shader.
//
// Port of Esoterica's Denoise.esf (which is XeGTAO's XeGTAO_Denoise), a
// cross-bilateral 3x3 blur of the noisy AO term, weighted by the packed
// per-pixel edges that MainPass wrote alongside the noise. The renderer
// runs N dispatches per frame: (N - 1) intermediate dispatches with
// finalApply = 0 ping-pong noisy0 <-> noisy1, then one final dispatch with
// finalApply = 1 that rescales by XE_GTAO_OCCLUSION_TERM_SCALE (1.5) on
// write and lands in `result` for the lit shaders to sample. N is chosen
// renderer-side and may be 1 (final dispatch only, no intermediates).
//
// 8x8 numthreads, each thread processes 2 horizontal pixels at coords
// (2*tid.x, tid.y) and (2*tid.x + 1, tid.y). Mirrors MainPass.esf's
// "2 pixels per thread" optimization, Esoterica calls this out explicitly
// at line 52.
//
// Departures from Esoterica's Denoise.esf / XeGTAO_Denoise:
//
//   * The 3 edges + 4 AO GatherRed calls in XeGTAO.esh:815-822 are
//     replaced by direct texelFetch reads of the same 12-texel
// neighbourhood (4-wide x 3-tall around pixCoordBase). Same
//     rationale as gtao_prefilter_depth.glsl and gtao_main_pass.glsl:
//     sidesteps the textureGather channel-ordering / backend-Y-flip
//     ambiguity. Cost is 24 single-channel R32F fetches vs 7 gather ops
// per thread, predictably slower but a non-issue for a debug
//     renderer.
//
//   * Dispatch sized as ceil(W / 16) x ceil(H / 8) workgroups (each
//     workgroup writes 16 horizontal x 8 vertical pixels). Esoterica
// dispatches ceil(W / 8) x ceil(H / 8), twice as many threads in X,
//     which over-covers by 2x with the surplus writes landing OOB
//     (silent no-op in D3D11/Vulkan/Metal compute). The visible output
//     is identical, we just skip the wasted threads. Recorded as a
//     deviation.
//
// * `lpfloat` collapses to `float` (matches Esoterica's config.
//     XE_GTAO_USE_HALF_FLOAT_PRECISION=0).
//
// * `AOTermType` is a plain `float`, XE_GTAO_COMPUTE_BENT_NORMALS is
//     undefined on our side (matches Esoterica's default). The
//     XeGTAO_DecodeGatherPartial path collapses to identity, we inline it.
//
//   * `XE_GTAO_SHOW_EDGES` / `XE_GTAO_SHOW_BENT_NORMALS` debug-viz
// branches dropped, Box3D uses its own debug_view_mode plumbing.
//
// * `finalApply` is delivered as a per-dispatch UBO field instead
//     HLSL root constants. Lives in a small dedicated ub_denoise UBO at
//     binding=1, the shared ub_gtao at binding=0 carries the same byte
//     layout as the prefilter + main_pass shaders so the renderer can
//     reuse one C-side build of the constants.
//
//   * Bounds clamping for the 12 neighbours uses
//     `clamp(coord, 0, ViewportSize - 1)` per fetch. The edge-clamped
//     duplicates feed back through the same XeGTAO weighting math, same
//     replicated-edge approximation Esoterica gets implicitly through
//     the gather + clamp_to_edge sampler.

#pragma sokol @module gtao_denoise

#pragma sokol @ctype vec2 b3Vec2
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype mat4 Mat4

#pragma sokol @cs cs

// Per-shader subset of Esoterica's XeGTAO::GTAOConstants. Denoise only
// reads `ViewportSize` (bounds checks) and `DenoiseBlurBeta` (blur
// strength). Explicit `gtao.X` instance name prevents SPIRV-Cross
// synthesizing one (`_NNN.X`), trimming to just-the-read fields
// prevents the GL linker's per-field dead-strip from producing
// `glGetUniformLocation` warnings. See the matching comment in
// gtao_prefilter_depth.glsl.
layout( binding = 0 ) uniform ub_gtao
{
	ivec2 ViewportSize;
	float DenoiseBlurBeta;
}
gtao;
// NoiseIndex intentionally omitted, see gtao_prefilter_depth.glsl.

// Per-dispatch knob. `FinalApply` non-zero on the final (3rd) denoise
// dispatch, rescales the AO term by XE_GTAO_OCCLUSION_TERM_SCALE before
// writing so lit shaders see the over-bright [0, 1.5] visibility range.
layout( binding = 1 ) uniform ub_denoise
{
	int FinalApply;
	int _Pad0;
	int _Pad1;
	int _Pad2;
};

// Storage-image output (R32F, same access-format rationale as MainPass
// / PrefilterDepth: sokol-shdc / SPIRV-Cross restrict storage_image
// access formats to r32f / r32i / r32ui plus rgba/rg{32,16} variants).
layout( binding = 0, r32f ) uniform writeonly image2D out_ao;

// Sampled inputs. Bind base 1: storage images take 0..0, textures start
// at 1. Same point-clamp sampler used by every GTAO compute shader.
layout( binding = 1 ) uniform texture2D tex_edges;
layout( binding = 2 ) uniform texture2D tex_noisy_ao;
layout( binding = 0 ) uniform sampler smp_point_clamp;

#pragma sokol @include ../common/xe_gtao_common.glsl

// XeGTAO.esh:745-751, additive sample with edge weight. Inlined here
// because GLSL's lack of `inout` arrays makes the HLSL helper awkward,
// the math is unchanged.
void XeGTAO_AddSample( float ssaoValue, float edgeValue, inout float sum, inout float sumWeight )
{
	float weight = edgeValue;
	sum += weight * ssaoValue;
	sumWeight += weight;
}

// Edge-clamped texelFetch helpers. We fetch a 4x3 neighbourhood around
// each pixel pair (pixCoordBase, pixCoordBase + (1, 0)). The diagonals
// reach one step out on each side and one step up/down, so the inclusive
// fetch range is (pixCoordBase + (-1, -1)).. (pixCoordBase + (2, 1)).
float fetchEdge( ivec2 coord )
{
	ivec2 vmax = gtao.ViewportSize - 1;
	ivec2 c = clamp( coord, ivec2( 0 ), vmax );
	return texelFetch( sampler2D( tex_edges, smp_point_clamp ), c, 0 ).r;
}

float fetchAO( ivec2 coord )
{
	ivec2 vmax = gtao.ViewportSize - 1;
	ivec2 c = clamp( coord, ivec2( 0 ), vmax );
	return texelFetch( sampler2D( tex_noisy_ao, smp_point_clamp ), c, 0 ).r;
}

// Output helper, XeGTAO.esh:757-774 collapsed to the "kirill: float AO
// term" patch (lines 771-772) since we always carry the float term, not
// the packed-UINT variant.
void XeGTAO_Output( ivec2 pixCoord, float outputValue, bool finalApply )
{
	float scale = finalApply ? XE_GTAO_OCCLUSION_TERM_SCALE : 1.0;
	imageStore( out_ao, pixCoord, vec4( outputValue * scale, 0.0, 0.0, 0.0 ) );
}

layout( local_size_x = 8, local_size_y = 8, local_size_z = 1 ) in;

void main()
{
	ivec2 dispatchThreadID = ivec2( gl_GlobalInvocationID.xy );

	// pixCoordBase = (2 * tid.x, tid.y). With the proper "2 horizontal
	// pixels per thread" dispatch sizing (groups = ceil(W/16) x
	// ceil(H/8)), tid.x ranges 0..ceil(W/16)*8 - 1, so pixCoordBase.x
	// ranges 0..2*ceil(W/16)*8 - 2, at most one pixel past W when W is
	// odd. The early-out + per-side bounds guard handles that tail.
	ivec2 pixCoordBase = ivec2( dispatchThreadID.x * 2, dispatchThreadID.y );
	if ( any( greaterThanEqual( pixCoordBase, gtao.ViewportSize ) ) )
	{
		return;
	}

	bool finalApply = ( FinalApply != 0 );
	float blurAmount = finalApply ? gtao.DenoiseBlurBeta : ( gtao.DenoiseBlurBeta / 5.0 );
	const float diagWeight = 0.85 * 0.5;

	// 4 x 3 neighbourhood of edges + AO, indexed [x = -1..2][y = -1..1].
	// Reading them up-front lets the per-side code below be a direct
	// index lookup, mirroring Esoterica's per-side `visQk[index]` extracts
	// from the gather results.
	float edgesN[4][3];
	float ao[4][3];
	for ( int i = 0; i < 4; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			ivec2 c = pixCoordBase + ivec2( i - 1, j - 1 );
			edgesN[i][j] = fetchEdge( c );
			ao[i][j] = fetchAO( c );
		}
	}

	// Process side 0 (pixel at pixCoordBase) then side 1 (pixCoordBase +
	// (1, 0)). The unrolled per-side block matches XeGTAO.esh:824-892.
	for ( int side = 0; side < 2; ++side )
	{
		ivec2 pixCoord = pixCoordBase + ivec2( side, 0 );
		if ( pixCoord.x >= gtao.ViewportSize.x )
		{
			break;
		}

		// Index helpers: column `c` in the 4-wide patch corresponds to
		// x = pixCoordBase.x + (c - 1). For side==0 the C column is 1
		// (the L/R neighbours sit at columns 0 / 2), for side==1 the C
		// column is 2 (neighbours at 1 / 3).
		int cC = 1 + side;
		int cL = cC - 1;
		int cR = cC + 1;

		vec4 edgesC_LRTB = XeGTAO_UnpackEdges( edgesN[cC][1] );
		vec4 edgesL_LRTB = XeGTAO_UnpackEdges( edgesN[cL][1] );
		vec4 edgesR_LRTB = XeGTAO_UnpackEdges( edgesN[cR][1] );
		vec4 edgesT_LRTB = XeGTAO_UnpackEdges( edgesN[cC][0] );
		vec4 edgesB_LRTB = XeGTAO_UnpackEdges( edgesN[cC][2] );

		// Edge symmetricity nudge, XeGTAO.esh:836-837. Clamps the center
		// pixel's per-direction edge weight against the matching edge on
		// the neighbour pixel, sharpens the blur, especially under TAA
		// (which we don't have, kept for A/B parity).
		edgesC_LRTB *= vec4( edgesL_LRTB.y, edgesR_LRTB.x, edgesT_LRTB.w, edgesB_LRTB.z );

		// Leak, XeGTAO.esh:839-843 (`#if 1` branch is the default and
		// only path Esoterica ships). Allows a small amount of bleed
		// when a pixel is surrounded by 3-4 edges, reducing aliasing.
		const float leak_threshold = 2.5;
		const float leak_strength = 0.5;
		float edginess =
			( clamp( 4.0 - leak_threshold - dot( edgesC_LRTB, vec4( 1.0 ) ), 0.0, 1.0 ) / ( 4.0 - leak_threshold ) ) *
			leak_strength;
		edgesC_LRTB = clamp( edgesC_LRTB + edginess, 0.0, 1.0 );

		// Diagonal weights, XeGTAO.esh:852-855.
		float weightTL = diagWeight * ( edgesC_LRTB.x * edgesL_LRTB.z + edgesC_LRTB.z * edgesT_LRTB.x );
		float weightTR = diagWeight * ( edgesC_LRTB.z * edgesT_LRTB.y + edgesC_LRTB.y * edgesR_LRTB.z );
		float weightBL = diagWeight * ( edgesC_LRTB.w * edgesB_LRTB.x + edgesC_LRTB.x * edgesL_LRTB.w );
		float weightBR = diagWeight * ( edgesC_LRTB.y * edgesR_LRTB.w + edgesC_LRTB.w * edgesB_LRTB.y );

		// AO neighbourhood for this side. Center at (cC, 1), L/R at
		// (cL, 1) / (cR, 1), T/B at (cC, 0) / (cC, 2), diagonals at the
		// 4 corners of the 3x3 around (cC, 1).
		float ssaoValue = ao[cC][1];
		float ssaoValueL = ao[cL][1];
		float ssaoValueR = ao[cR][1];
		float ssaoValueT = ao[cC][0];
		float ssaoValueB = ao[cC][2];
		float ssaoValueTL = ao[cL][0];
		float ssaoValueTR = ao[cR][0];
		float ssaoValueBL = ao[cL][2];
		float ssaoValueBR = ao[cR][2];

		float sumWeight = blurAmount;
		float sum = ssaoValue * sumWeight;

		XeGTAO_AddSample( ssaoValueL, edgesC_LRTB.x, sum, sumWeight );
		XeGTAO_AddSample( ssaoValueR, edgesC_LRTB.y, sum, sumWeight );
		XeGTAO_AddSample( ssaoValueT, edgesC_LRTB.z, sum, sumWeight );
		XeGTAO_AddSample( ssaoValueB, edgesC_LRTB.w, sum, sumWeight );

		XeGTAO_AddSample( ssaoValueTL, weightTL, sum, sumWeight );
		XeGTAO_AddSample( ssaoValueTR, weightTR, sum, sumWeight );
		XeGTAO_AddSample( ssaoValueBL, weightBL, sum, sumWeight );
		XeGTAO_AddSample( ssaoValueBR, weightBR, sum, sumWeight );

		float aoTerm = sum / sumWeight;
		XeGTAO_Output( pixCoord, aoTerm, finalApply );
	}
}
#pragma sokol @end

#pragma sokol @program prog cs
