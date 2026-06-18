// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// XeGTAO helper functions shared between the GTAO compute shaders
// (MainPass, Denoise). Ported from Esoterica HLSL with the following
// GLSL adaptations:
//
// * lpfloat collapses to float (XE_GTAO_USE_HALF_FLOAT_PRECISION=0,
//	XE_GTAO_FP32_DEPTHS=1, matches Esoterica's config).
// * HLSL `out` parameters become inout where needed.
// * `mul(matrix, vec)` is rewritten as `matrix * vec` and column-major
//	conventions are preserved.
// * Functions that took a `Texture2D<float>` / `SamplerState` /
//	`RWTexture2D<float>` parameter in HLSL cannot be cleanly expressed
//	in cross-compiled GLSL. Those functions (XeGTAO_MainPass,
//	XeGTAO_ComputeViewspaceNormal, XeGTAO_OutputWorkingTerm,
//	XeGTAO_Denoise) are inlined into the consuming shader and reference
//	its texture/image bindings directly. This file holds only the pure
//	helpers.
// * GTAOConsts is a plain struct populated once per dispatch from the
//	ub_gtao UBO. Each consumer constructs a GTAOConsts in main and
//	passes it down to the helpers below. The struct is a strict subset
//	of Esoterica's `GTAOConstants`, only the fields the helpers
//	actually read are carried, so each consuming shader's ub_gtao only
//	needs to declare those (plus whatever else its own main reads
//	directly).
//
// Include via `@include xe_gtao_common.glsl` inside the consumer's @cs
// block. The consumer is responsible for declaring the ub_gtao UBO with
// the fields its main reads.

const float XE_GTAO_PI = 3.1415926535897932384626433832795;
const float XE_GTAO_PI_HALF = 1.5707963267948966192313216916398;

// Esoterica's XE_GTAO_OCCLUSION_TERM_SCALE, applied by
// XeGTAO_OutputWorkingTerm (divides) and the denoise finalApply
// (multiplies). Keeps the noisy occlusion-term in [0,1] for storage,
// rescaled to ~[0,1.5] for the final lit lookup.
const float XE_GTAO_OCCLUSION_TERM_SCALE = 1.5;

// XE_GTAO_DEPTH_MIP_LEVELS = 5, the prefiltered depth chain has 5 mips
// (mip 0..4). MainPass clamps the computed sample mip to [0, 5).
const float XE_GTAO_DEPTH_MIP_LEVELS = 5.0;

// HilbertIndex parameters, XE_HILBERT_LEVEL = 6U, width 64.
const uint XE_HILBERT_WIDTH = 64u;

struct GTAOConsts
{
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
	float SliceCount;
	float StepsPerSlice;
};
	
// http://h14s.p5r.org/2012/09/0x5f3759df.html via XeGTAO.esh:184-187.
// Bit-cast through floatBitsToInt / intBitsToFloat for the GLSL idiom.
float XeGTAO_FastSqrt(float x)
{
	return intBitsToFloat(0x1fbd1df5 + (floatBitsToInt(x) >> 1));
}

// Lagarde 2014 approximation, input [-1, 1], output [0, PI]. XeGTAO.esh:189.
float XeGTAO_FastACos(float inX)
{
	float x = abs(inX);
	float res = -0.156583 * x + XE_GTAO_PI_HALF;
	res *= XeGTAO_FastSqrt(1.0 - x);
	return (inX >= 0.0) ? res : (XE_GTAO_PI - res);
}

// Clamp depth into fp32-finite range. XeGTAO.esh:648-655.
float XeGTAO_ClampDepth(float depth)
{
	return clamp(depth, 0.0, 3.402823466e+38);
}

// Pass-through: our input depth is already positive linear view-Z, so the
// "screen-space -> view-space" linearize step is the identity. Kept for
// signature parity with XeGTAO.esh:117-123 so ports of MainPass and
// ComputeViewspaceNormal stay verbatim.
float XeGTAO_ScreenSpaceToViewSpaceDepth(float screenDepth, GTAOConsts consts)
{
	return screenDepth;
}

// XeGTAO.esh:104-115 with the Esoterica reverse-Z patch (negate Z so it
// points OUT of the screen in view space). Box3D's view space is
// also right-handed Y-up with camera looking down -Z, matching Esoterica.
vec3 XeGTAO_ComputeViewspacePosition(vec2 screenPos, float viewspaceDepth, GTAOConsts consts) {
	vec3 ret;
	ret.xy = (consts.NDCToViewMul * screenPos + consts.NDCToViewAdd) * viewspaceDepth;
	ret.z = -viewspaceDepth;
	return ret;
}

// XeGTAO.esh:125-134. Output is in [0,1] per channel, 0 = strong edge,
// 1 = no edge. Used by both the edges-output (PackEdges) and the
// normal-from-depth pass (CalculateNormal).
vec4 XeGTAO_CalculateEdges(float centerZ, float leftZ, float rightZ, float topZ, float bottomZ)
{
	vec4 edgesLRTB = vec4(leftZ, rightZ, topZ, bottomZ) - centerZ;
	
	float slopeLR = (edgesLRTB.y - edgesLRTB.x) * 0.5;
	float slopeTB = (edgesLRTB.w - edgesLRTB.z) * 0.5;
	vec4 edgesLRTBSlopeAdjusted = edgesLRTB + vec4(slopeLR, - slopeLR, slopeTB, - slopeTB);
	edgesLRTB = min(abs(edgesLRTB), abs(edgesLRTBSlopeAdjusted));
	return vec4(clamp((1.25 - edgesLRTB / (centerZ * 0.011)), 0.0, 1.0));
}

// XeGTAO.esh:137-146. Packs 4 edge gradients (2 bits each) into a single
// UNORM8 channel. The dot-product form matches the reference, rounding
// happens before the dot so the four bins are { 0, 64, 128, 192 } / 255.
float XeGTAO_PackEdges(vec4 edgesLRTB)
{
	edgesLRTB = round(clamp(edgesLRTB, 0.0, 1.0) * 2.9);
	return dot(edgesLRTB, vec4(64.0 / 255.0, 16.0 / 255.0, 4.0 / 255.0, 1.0 / 255.0));
}

// XeGTAO.esh:727-737. Reverse of PackEdges. Used by Denoise.
vec4 XeGTAO_UnpackEdges(float packed_val)
{
	uint p = uint(packed_val * 255.5);
	vec4 edgesLRTB;
	edgesLRTB.x = float((p >> 6)& 0x03u) / 3.0;
	edgesLRTB.y = float((p >> 4)& 0x03u) / 3.0;
	edgesLRTB.z = float((p >> 2)& 0x03u) / 3.0;
	edgesLRTB.w = float((p >> 0)& 0x03u) / 3.0;
	return clamp(edgesLRTB, 0.0, 1.0);
}

// XeGTAO.esh:148-173 with the Esoterica reverse-Z cross-product reorder
// (cross args swapped to keep the winding consistent with view space
// pointing OUT of the screen).
vec3 XeGTAO_CalculateNormal(vec4 edgesLRTB, vec3 pixCenterPos,
	vec3 pixLPos, vec3 pixRPos, vec3 pixTPos, vec3 pixBPos)
{
	vec4 acceptedNormals = clamp(vec4(
			edgesLRTB.x * edgesLRTB.z,
			edgesLRTB.z * edgesLRTB.y,
			edgesLRTB.y * edgesLRTB.w,
		edgesLRTB.w * edgesLRTB.x) + 0.01, 0.0, 1.0);
		
	pixLPos = normalize(pixLPos - pixCenterPos);
	pixRPos = normalize(pixRPos - pixCenterPos);
	pixTPos = normalize(pixTPos - pixCenterPos);
	pixBPos = normalize(pixBPos - pixCenterPos);
	
	vec3 pixelNormal = acceptedNormals.x * cross(pixTPos, pixLPos)
	+ acceptedNormals.y * cross(pixRPos, pixTPos)
	+ acceptedNormals.z * cross(pixBPos, pixRPos)
	+ acceptedNormals.w * cross(pixLPos, pixBPos);

	return normalize(pixelNormal);
}
	
// XeGTAO_Types.esh:120-142. Hilbert curve index for a 64x64 tile
// pixCoord (pixCoord & 63). Combined with the R2 sequence in the
// consumer's SpatioTemporalNoise to produce blue-noise-like spatial
// distribution. No LUT, generated in-shader, matches Esoterica's
// `#ifndef XE_GTAO_HILBERT_LUT_AVAILABLE` branch.
uint HilbertIndex(uint posX, uint posY)
{
	uint index = 0u;
	for (uint curLevel = XE_HILBERT_WIDTH / 2u; curLevel > 0u; curLevel /= 2u)
	{
		uint regionX = (posX & curLevel) > 0u ? 1u : 0u;
		uint regionY = (posY & curLevel) > 0u ? 1u : 0u;
		index += curLevel * curLevel * ((3u * regionX)^ regionY);
		if (regionY == 0u)
		{
			if (regionX == 1u)
			{
				posX = (XE_HILBERT_WIDTH - 1u) - posX;
				posY = (XE_HILBERT_WIDTH - 1u) - posY;
			}
			uint temp = posX;
			posX = posY;
			posY = temp;
		}
	}
	return index;
}
