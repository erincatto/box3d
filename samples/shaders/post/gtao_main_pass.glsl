// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// GTAO MainPass compute shader.
//
// Port of Esoterica's MainPass.esf (which is XeGTAO's MainPass with
// XE_GTAO_GENERATE_NORMALS_INPLACE enabled). For each viewport pixel,
// reconstructs the view-space normal from depth derivatives, generates a
// Hilbert+R2 spatial noise pair, and runs the XeGTAO slice trace against
// the prefiltered depth chain. Slice count and steps-per-slice come from
// the ub_gtao UBO (the AmbientOcclusionQuality preset on the C side) rather than the
// hardcoded Ultra tier.
//
// Outputs:
// * out_noisy_ao (R32F, storage_image): the visibility term divided
// by XE_GTAO_OCCLUSION_TERM_SCALE (1.5) and saturated to [0,1].
// XeGTAO_OutputWorkingTerm's pre-denoise format. Denoise
// will multiply by 1.5 on finalApply to restore the over-bright
// occlusion term. (Deviation: Esoterica uses R16F. sokol-shdc /
// SPIRV-Cross only permit r32f/r32i/r32ui plus rgba/rg{32,16}
// variants for storage_image access formats, mirrors D3D11's
// typed-UAV restriction. R32F is the cheapest single-channel
// fp format that survives cross-compilation.)
// * out_edges (R32F, storage_image): packed edge gradients
// consumed by Denoise. Same R8->R32F deviation rationale as
// out_noisy_ao, the value range still fits in [0,1] so
// XeGTAO_UnpackEdges on the read side is unchanged.
//
// Departures from Esoterica's MainPass.esf:
//
// * Cardinal depth taps (center, L, R, T, B) use texelFetch on the raw
// depth target instead of two GatherRed calls. Two reasons: avoids
// the GatherRed channel-ordering / backend Y-flip ambiguity, and
// matches the same pattern already adopted by
// gtao_prefilter_depth.glsl.
//
// * The trace samples DO use textureLod since they sample at fractional
// mip levels, texelFetch can't pick a mip. The prefilter chain was
// written by the PrefilterDepth compute via imageStore, so the
// UV->texel mapping is internally consistent within the chain even
// across backends (no fragment-shader Y-flip in the producer).
//
// * `lpfloat` collapses to `float` (XE_GTAO_USE_HALF_FLOAT_PRECISION=0,
// XE_GTAO_FP32_DEPTHS=1, matches Esoterica's config).
//
// * SpatioTemporalNoise is called with temporalIndex=0 hardcoded
// (no TAA), producing the same noise pattern every frame. The
// original NoiseIndex UBO field is gone, see the comment under
// ub_gtao.
//
// * The "early-out for OOB threads" at the top of MainPass.esf is kept.
// Unlike PrefilterDepth there is no inter-thread groupshared
// reduction, so divergent control flow at the top is safe (no
// barriers).

#pragma sokol @module gtao_main_pass

#pragma sokol @ctype vec2 b3Vec2
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype mat4 Mat4

#pragma sokol @cs cs

// Per-shader subset of Esoterica's XeGTAO::GTAOConstants, only the
// fields MainPass actually reads (14 of 16). Explicit `gtao.X` instance
// name prevents SPIRV-Cross synthesizing one (`_NNN.X`), trimming to
// just-the-read fields prevents the GL linker's per-field dead-strip
// from producing `glGetUniformLocation` warnings. See the matching
// comment in gtao_prefilter_depth.glsl.
layout( binding = 0 ) uniform ub_gtao
{
	ivec2 ViewportSize;
	vec2 ViewportPixelSize;

	vec2 NDCToViewMul;
	vec2 NDCToViewAdd;

	vec2 NDCToViewMul_x_PixelSize;
	float EffectRadius;
	float EffectFalloffRange;

	float RadiusMultiplier;
	float FinalValuePower;
	float SampleDistributionPower;
	float ThinOccluderCompensation;

	float DepthMIPSamplingOffset;

	// Runtime trace quality: slice count and steps-per-slice. Driven by
	// the AmbientOcclusionQuality preset on the C side (Low/Medium/High/Ultra), the
	// trace loops read these instead of hardcoded Ultra-tier constants.
	float SliceCount;
	float StepsPerSlice;
}
gtao;
// NoiseIndex intentionally omitted, see gtao_prefilter_depth.glsl.

// Storage-image outputs. Bindings 0..1. r32f instead of r16f/r8 (see
// header, sokol-shdc storage_image access-format restriction).
layout( binding = 0, r32f ) uniform writeonly image2D out_noisy_ao;
layout( binding = 1, r32f ) uniform writeonly image2D out_edges;

// Sampled textures. Binding base 2 (sokol-shdc shares the
// storage_image/texture bind-space across the compute stage, storage
// images take 0..1, so textures start at 2). The sampler is point-clamp,
// linear sampling on a prefiltered depth chain would inter-mip-level
// blend depth values which corrupts the trace (see XeGTAO.esh:476).
layout( binding = 2 ) uniform texture2D tex_raw_depth;
layout( binding = 3 ) uniform texture2D tex_prefilter_depth;
layout( binding = 0 ) uniform sampler smp_point_clamp;

#pragma sokol @include ../common/xe_gtao_common.glsl

// SpatioTemporalNoise, MainPass.esf:43-60 with the
// XE_GTAO_HILBERT_LUT_AVAILABLE branch removed (we generate the index
// in-shader, mirrors Esoterica's default). With NoiseIndex=0 the
// `288 * (temporalIndex % 64)` term is zero, the (288*0) is kept as a
// no-op so this function reads verbatim against the reference.
vec2 SpatioTemporalNoise( uvec2 pixCoord, int temporalIndex )
{
	uint index = HilbertIndex( pixCoord.x, pixCoord.y );
	index += 288u * ( uint( temporalIndex ) % 64u );
	// R2 sequence, Roberts 2018.
	return fract( 0.5 + float( index ) * vec2( 0.75487766624669276005, 0.5698402909980532659114 ) );
}

// XeGTAO.esh:897-921 with cardinal taps coming from texelFetch instead
// of GatherRed (see header). Identical math.
vec3 XeGTAO_ComputeViewspaceNormal( ivec2 pixCoord, GTAOConsts consts )
{
	vec2 normalizedScreenPos = ( vec2( pixCoord ) + 0.5 ) * consts.ViewportPixelSize;

	// Clamp the 4 cardinal taps into the viewport. At the viewport edge
	// we replicate the center pixel, same approximation as gather-with-
	// sampler-clamp.
	ivec2 vmax = gtao.ViewportSize - 1;
	ivec2 cC = clamp( pixCoord, ivec2( 0 ), vmax );
	ivec2 cL = clamp( pixCoord + ivec2( -1, 0 ), ivec2( 0 ), vmax );
	ivec2 cR = clamp( pixCoord + ivec2( 1, 0 ), ivec2( 0 ), vmax );
	ivec2 cT = clamp( pixCoord + ivec2( 0, -1 ), ivec2( 0 ), vmax );
	ivec2 cB = clamp( pixCoord + ivec2( 0, 1 ), ivec2( 0 ), vmax );

	float viewspaceZ =
		XeGTAO_ScreenSpaceToViewSpaceDepth( texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), cC, 0 ).r, consts );
	float pixLZ =
		XeGTAO_ScreenSpaceToViewSpaceDepth( texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), cL, 0 ).r, consts );
	float pixRZ =
		XeGTAO_ScreenSpaceToViewSpaceDepth( texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), cR, 0 ).r, consts );
	float pixTZ =
		XeGTAO_ScreenSpaceToViewSpaceDepth( texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), cT, 0 ).r, consts );
	float pixBZ =
		XeGTAO_ScreenSpaceToViewSpaceDepth( texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), cB, 0 ).r, consts );

	vec4 edgesLRTB = XeGTAO_CalculateEdges( viewspaceZ, pixLZ, pixRZ, pixTZ, pixBZ );

	vec3 CENTER = XeGTAO_ComputeViewspacePosition( normalizedScreenPos, viewspaceZ, consts );
	vec3 LEFT =
		XeGTAO_ComputeViewspacePosition( normalizedScreenPos + vec2( -1.0, 0.0 ) * consts.ViewportPixelSize, pixLZ, consts );
	vec3 RIGHT =
		XeGTAO_ComputeViewspacePosition( normalizedScreenPos + vec2( 1.0, 0.0 ) * consts.ViewportPixelSize, pixRZ, consts );
	vec3 TOP =
		XeGTAO_ComputeViewspacePosition( normalizedScreenPos + vec2( 0.0, -1.0 ) * consts.ViewportPixelSize, pixTZ, consts );
	vec3 BOTTOM =
		XeGTAO_ComputeViewspacePosition( normalizedScreenPos + vec2( 0.0, 1.0 ) * consts.ViewportPixelSize, pixBZ, consts );
	return XeGTAO_CalculateNormal( edgesLRTB, CENTER, LEFT, RIGHT, TOP, BOTTOM );
}

// XeGTAO.esh:213-231. Stores the visibility/scale-divided term and
// returns it. `bentNormal` arg dropped, XE_GTAO_COMPUTE_BENT_NORMALS is
// undefined on our side (matches Esoterica).
float XeGTAO_OutputWorkingTerm( ivec2 pixCoord, float visibility )
{
	visibility = clamp( visibility / XE_GTAO_OCCLUSION_TERM_SCALE, 0.0, 1.0 );
	imageStore( out_noisy_ao, pixCoord, vec4( visibility, 0.0, 0.0, 0.0 ) );
	return visibility;
}

// XeGTAO.esh:270-617 with:
// * Cardinal taps via texelFetch (see header).
// * Bent-normals branch (`XE_GTAO_COMPUTE_BENT_NORMALS`) removed,
// undefined in our config.
// * In-place normals branch (`XE_GTAO_GENERATE_NORMALS_INPLACE`) always
// taken. The cardinal-tap reads above already happened, so this
// is just the same XeGTAO_CalculateNormal call as in
// XeGTAO_ComputeViewspaceNormal. We share the call by computing the
// normal in main before invoking MainPass, and pass it in.
// * Debug-viz (`XE_GTAO_SHOW_DEBUG_VIZ`, `XE_GTAO_SHOW_NORMALS`, etc.)
// branches dropped, we use the renderer's debug_view_mode plumbing
// instead.
// * `sliceCount`/`stepsPerSlice` come from the ub_gtao UBO (the
// AmbientOcclusionQuality preset on the C side), not the hardcoded 9/3 Ultra tier of
// MainPass.esf:135. The two `for` loops iterate `float` counters with a
// `<` comparison so the runtime bound cross-compiles cleanly, the
// trade-off is the inner loop can no longer be statically unrolled.
// * Edges are packed and written before the trace, exactly as in
// MainPass.esf. The Denoise pass reads them as bilateral weights.
//
// pixCoord = dispatchThreadID.xy. Returns the saturated visibility scalar
// for visualisation in the renderer's debug_view_mode == 4.
float XeGTAO_MainPass( ivec2 pixCoord, vec2 localNoise, vec3 viewspaceNormal,
					   float centerViewspaceZ, // value already fetched for normal recon
					   vec4 edgesLRTB,		   // ditto, written below
					   GTAOConsts consts )
{
	vec2 normalizedScreenPos = ( vec2( pixCoord ) + 0.5 ) * consts.ViewportPixelSize;
	float viewspaceZ = centerViewspaceZ;

	imageStore( out_edges, pixCoord, vec4( XeGTAO_PackEdges( edgesLRTB ), 0.0, 0.0, 0.0 ) );

	// FP32 depth: push the center pixel slightly towards camera to avoid
	// self-occlusion from depth imprecision. XeGTAO.esh:318-322. With
	// positive linear view-Z + reverse-Z, "towards camera" is multiply by
	// (1 - epsilon).
	viewspaceZ *= 0.99999;

	vec3 pixCenterPos = XeGTAO_ComputeViewspacePosition( normalizedScreenPos, viewspaceZ, consts );
	vec3 viewVec = normalize( -pixCenterPos );

	float sliceCount = consts.SliceCount;
	float stepsPerSlice = consts.StepsPerSlice;
	const float effectRadius = consts.EffectRadius * consts.RadiusMultiplier;
	const float falloffRange = consts.EffectFalloffRange * effectRadius;
	const float falloffFrom = effectRadius * ( 1.0 - consts.EffectFalloffRange );
	const float falloffMul = -1.0 / falloffRange;
	const float falloffAdd = falloffFrom / falloffRange + 1.0;
	const float sampleDistributionPower = consts.SampleDistributionPower;
	const float thinOccluderCompensation = consts.ThinOccluderCompensation;

	float visibility = 0.0;

	{
		const float noiseSlice = localNoise.x;
		const float noiseSample = localNoise.y;
		const float pixelTooCloseThreshold = 1.3;

		// Approx viewspace pixel size at pixCoord. XeGTAO.esh:377.
		vec2 pixelDirRBViewspaceSizeAtCenterZ = vec2( viewspaceZ, viewspaceZ ) * consts.NDCToViewMul_x_PixelSize;
		float screenspaceRadius = effectRadius / pixelDirRBViewspaceSizeAtCenterZ.x;

		// Fade out for small screen radii, XeGTAO.esh:382.
		visibility += clamp( ( 10.0 - screenspaceRadius ) / 100.0, 0.0, 1.0 ) * 0.5;

		float minS = pixelTooCloseThreshold / screenspaceRadius;

		for ( float slice = 0.0; slice < sliceCount; slice += 1.0 )
		{
			float sliceK = ( slice + noiseSlice ) / sliceCount;
			float phi = sliceK * XE_GTAO_PI;
			float cosPhi = cos( phi );
			float sinPhi = sin( phi );
			vec2 omega = vec2( cosPhi, -sinPhi );
			omega *= screenspaceRadius;

			vec3 directionVec = vec3( cosPhi, sinPhi, 0.0 );
			vec3 orthoDirectionVec = directionVec - ( dot( directionVec, viewVec ) * viewVec );
			vec3 axisVec = normalize( cross( orthoDirectionVec, viewVec ) );

			vec3 projectedNormalVec = viewspaceNormal - axisVec * dot( viewspaceNormal, axisVec );

			float signNorm = sign( dot( orthoDirectionVec, projectedNormalVec ) );
			float projectedNormalVecLength = length( projectedNormalVec );
			float cosNorm = clamp( dot( projectedNormalVec, viewVec ) / projectedNormalVecLength, 0.0, 1.0 );

			float n = signNorm * XeGTAO_FastACos( cosNorm );

			const float lowHorizonCos0 = cos( n + XE_GTAO_PI_HALF );
			const float lowHorizonCos1 = cos( n - XE_GTAO_PI_HALF );

			float horizonCos0 = lowHorizonCos0;
			float horizonCos1 = lowHorizonCos1;

			for ( float step_ = 0.0; step_ < stepsPerSlice; step_ += 1.0 )
			{
				// R1 sequence, XeGTAO.esh:459.
				float stepBaseNoise = ( slice + step_ * stepsPerSlice ) * 0.6180339887498948482;
				float stepNoise = fract( noiseSample + stepBaseNoise );

				float s = ( step_ + stepNoise ) / stepsPerSlice;
				// step_, stepNoise (=fract(.)), stepsPerSlice are all >= 0
				// by construction, max(0, .) is redundant at runtime but
				// silences HLSL's pow(negative, n) warning (X3571).
				s = pow( max( 0.0, s ), sampleDistributionPower );
				s += minS;

				vec2 sampleOffset = s * omega;
				float sampleOffsetLength = length( sampleOffset );

				float mipLevel =
					clamp( log2( sampleOffsetLength ) - consts.DepthMIPSamplingOffset, 0.0, XE_GTAO_DEPTH_MIP_LEVELS );

				// Snap to pixel centers, XeGTAO.esh:481.
				sampleOffset = round( sampleOffset ) * consts.ViewportPixelSize;

				vec2 sampleScreenPos0 = normalizedScreenPos + sampleOffset;
				float SZ0 = textureLod( sampler2D( tex_prefilter_depth, smp_point_clamp ), sampleScreenPos0, mipLevel ).x;
				vec3 samplePos0 = XeGTAO_ComputeViewspacePosition( sampleScreenPos0, SZ0, consts );

				vec2 sampleScreenPos1 = normalizedScreenPos - sampleOffset;
				float SZ1 = textureLod( sampler2D( tex_prefilter_depth, smp_point_clamp ), sampleScreenPos1, mipLevel ).x;
				vec3 samplePos1 = XeGTAO_ComputeViewspacePosition( sampleScreenPos1, SZ1, consts );

				vec3 sampleDelta0 = samplePos0 - pixCenterPos;
				vec3 sampleDelta1 = samplePos1 - pixCenterPos;
				float sampleDist0 = length( sampleDelta0 );
				float sampleDist1 = length( sampleDelta1 );

				vec3 sampleHorizonVec0 = sampleDelta0 / sampleDist0;
				vec3 sampleHorizonVec1 = sampleDelta1 / sampleDist1;

				// Esoterica branch: thin-object heuristic (XeGTAO.esh:518-524).
				// XE_GTAO_USE_DEFAULT_CONSTANTS=0 path so falloffBase uses
				// thinOccluderCompensation. With thinOccluderCompensation = 0
				// this is equivalent to the simpler XeGTAO.esh:516-517 form
				// since the z-axis factor becomes 1.
				float falloffBase0 =
					length( vec3( sampleDelta0.x, sampleDelta0.y, sampleDelta0.z * ( 1.0 + thinOccluderCompensation ) ) );
				float falloffBase1 =
					length( vec3( sampleDelta1.x, sampleDelta1.y, sampleDelta1.z * ( 1.0 + thinOccluderCompensation ) ) );
				float weight0 = clamp( falloffBase0 * falloffMul + falloffAdd, 0.0, 1.0 );
				float weight1 = clamp( falloffBase1 * falloffMul + falloffAdd, 0.0, 1.0 );

				float shc0 = dot( sampleHorizonVec0, viewVec );
				float shc1 = dot( sampleHorizonVec1, viewVec );

				shc0 = mix( lowHorizonCos0, shc0, weight0 );
				shc1 = mix( lowHorizonCos1, shc1, weight1 );

				horizonCos0 = max( horizonCos0, shc0 );
				horizonCos1 = max( horizonCos1, shc1 );
			}

			// Slight over-darkening fudge, XeGTAO.esh:571.
			projectedNormalVecLength = mix( projectedNormalVecLength, 1.0, 0.05 );

			float h0 = -XeGTAO_FastACos( horizonCos1 );
			float h1 = XeGTAO_FastACos( horizonCos0 );
			float iarc0 = ( cosNorm + 2.0 * h0 * sin( n ) - cos( 2.0 * h0 - n ) ) * 0.25;
			float iarc1 = ( cosNorm + 2.0 * h1 * sin( n ) - cos( 2.0 * h1 - n ) ) * 0.25;
			float localVisibility = projectedNormalVecLength * ( iarc0 + iarc1 );
			visibility += localVisibility;
		}

		visibility /= sliceCount;
		// visibility is summed from non-negative localVisibility terms and
		// divided by positive sliceCount, so always >= 0. max(0, .) is a
		// no-op at runtime, silences HLSL's pow(negative, n) warning.
		visibility = pow( max( 0.0, visibility ), consts.FinalValuePower );
		// Disallow total occlusion, XeGTAO.esh:597.
		visibility = max( 0.03, visibility );
	}

	return XeGTAO_OutputWorkingTerm( pixCoord, visibility );
}

layout( local_size_x = 8, local_size_y = 8, local_size_z = 1 ) in;

void main()
{
	ivec2 pixCoord = ivec2( gl_GlobalInvocationID.xy );
	if ( any( greaterThanEqual( pixCoord, gtao.ViewportSize ) ) )
	{
		return;
	}

	// Populate GTAOConsts from the UBO, gives the helpers in
	// xe_gtao_common.glsl + the inlined XeGTAO_MainPass a single by-
	// value argument matching the HLSL reference.
	GTAOConsts consts;
	consts.ViewportPixelSize = gtao.ViewportPixelSize;
	consts.NDCToViewMul = gtao.NDCToViewMul;
	consts.NDCToViewAdd = gtao.NDCToViewAdd;
	consts.NDCToViewMul_x_PixelSize = gtao.NDCToViewMul_x_PixelSize;
	consts.EffectRadius = gtao.EffectRadius;
	consts.EffectFalloffRange = gtao.EffectFalloffRange;
	consts.RadiusMultiplier = gtao.RadiusMultiplier;
	consts.FinalValuePower = gtao.FinalValuePower;
	consts.SampleDistributionPower = gtao.SampleDistributionPower;
	consts.ThinOccluderCompensation = gtao.ThinOccluderCompensation;
	consts.DepthMIPSamplingOffset = gtao.DepthMIPSamplingOffset;
	consts.SliceCount = gtao.SliceCount;
	consts.StepsPerSlice = gtao.StepsPerSlice;

	// Cardinal-tap reads happen inside ComputeViewspaceNormal, redo the
	// center read so MainPass has its viewspaceZ + edges without
	// re-fetching the 4 neighbours. Cheaper to repeat the 1-tap fetch
	// than to plumb the 5-tap state through a return struct.
	vec3 viewspaceNormal = XeGTAO_ComputeViewspaceNormal( pixCoord, consts );

	ivec2 vmax = gtao.ViewportSize - 1;
	float centerZ = XeGTAO_ScreenSpaceToViewSpaceDepth(
		texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), clamp( pixCoord, ivec2( 0 ), vmax ), 0 ).r, consts );
	float leftZ = XeGTAO_ScreenSpaceToViewSpaceDepth(
		texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), clamp( pixCoord + ivec2( -1, 0 ), ivec2( 0 ), vmax ), 0 ).r,
		consts );
	float rightZ = XeGTAO_ScreenSpaceToViewSpaceDepth(
		texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), clamp( pixCoord + ivec2( 1, 0 ), ivec2( 0 ), vmax ), 0 ).r,
		consts );
	float topZ = XeGTAO_ScreenSpaceToViewSpaceDepth(
		texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), clamp( pixCoord + ivec2( 0, -1 ), ivec2( 0 ), vmax ), 0 ).r,
		consts );
	float botZ = XeGTAO_ScreenSpaceToViewSpaceDepth(
		texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), clamp( pixCoord + ivec2( 0, 1 ), ivec2( 0 ), vmax ), 0 ).r,
		consts );
	vec4 edgesLRTB = XeGTAO_CalculateEdges( centerZ, leftZ, rightZ, topZ, botZ );

	vec2 noise = SpatioTemporalNoise( uvec2( pixCoord ), 0 );

	XeGTAO_MainPass( pixCoord, noise, viewspaceNormal, centerZ, edgesLRTB, consts );
}
#pragma sokol @end

#pragma sokol @program prog cs
