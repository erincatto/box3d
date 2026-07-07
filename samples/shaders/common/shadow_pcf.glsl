// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Cascaded shadow map PCF, shared by the lit shape shaders via
// @include. The texture and comparison sampler are passed as parameters
// so this file has no binding declarations of its own.
//
// 7x7 uniform kernel evaluated with 16 bilinear comparison taps. Tap
// positions and weights steer the hardware 2x2 compare so the fetches
// reproduce the full 49-texel kernel with triangle-filtered sub-texel
// placement, a moving shadow edge slides smoothly instead of stepping
// texel by texel. Follows the optimized PCF from The Witness (Ignacio
// Castano), via MJP's shadows sample.
//
// Manually unrolled: the HLSL compiler emits warning X3570 ("gradient
// instruction used in a loop with varying iteration") on looped texture
// calls. Unrolling keeps cross-compilation clean on the D3D11 backend.

float sampleShadowTap( texture2DArray tex, samplerShadow smp, vec2 uv, float layer, float ref_z )
{
	return texture( sampler2DArrayShadow( tex, smp ), vec4( uv, layer, ref_z ) );
}

// scale multiplies the tap offsets. Cascades cover different world sizes
// per texel, the caller passes GetCascadePcfScale's texel-size ratio so
// the penumbra keeps a constant world width instead of jumping from
// crisp to blurry at every cascade boundary. 1.0 = the full 7x7 kernel.
float sampleShadowPCF( texture2DArray tex, samplerShadow smp, int cascade, vec3 light_uv, float bias, float scale )
{
	// Must match SHADOW_RESOLUTION in shadow.h
	const float size = 4096.0;
	const float inv_size = 1.0 / size;

	float ly = float( cascade );
	float rz = light_uv.z - bias;

	// Texel-space position split into the nearest texel corner and the
	// sub-texel fraction that drives the tap weights.
	vec2 tuv = light_uv.xy * size;
	vec2 base = floor( tuv + 0.5 );
	float s = tuv.x + 0.5 - base.x;
	float t = tuv.y + 0.5 - base.y;
	base = ( base - 0.5 ) * inv_size;

	// Shrinking the offsets bunches the 16 bilinear taps inside a smaller
	// footprint. The box property degrades gracefully, at the 0.25 floor
	// the taps still span ~2 texels and overlap through the hardware 2x2
	// compare, so the response stays smooth.
	float step = inv_size * scale;

	float uw0 = 5.0 * s - 6.0;
	float uw1 = 11.0 * s - 28.0;
	float uw2 = -( 11.0 * s + 17.0 );
	float uw3 = -( 5.0 * s + 1.0 );

	float u0 = ( 4.0 * s - 5.0 ) / uw0 - 3.0;
	float u1 = ( 4.0 * s - 16.0 ) / uw1 - 1.0;
	float u2 = -( 7.0 * s + 5.0 ) / uw2 + 1.0;
	float u3 = -s / uw3 + 3.0;

	float vw0 = 5.0 * t - 6.0;
	float vw1 = 11.0 * t - 28.0;
	float vw2 = -( 11.0 * t + 17.0 );
	float vw3 = -( 5.0 * t + 1.0 );

	float v0 = ( 4.0 * t - 5.0 ) / vw0 - 3.0;
	float v1 = ( 4.0 * t - 16.0 ) / vw1 - 1.0;
	float v2 = -( 7.0 * t + 5.0 ) / vw2 + 1.0;
	float v3 = -t / vw3 + 3.0;

	float sum = 0.0;
	sum += uw0 * vw0 * sampleShadowTap( tex, smp, base + vec2( u0, v0 ) * step, ly, rz );
	sum += uw1 * vw0 * sampleShadowTap( tex, smp, base + vec2( u1, v0 ) * step, ly, rz );
	sum += uw2 * vw0 * sampleShadowTap( tex, smp, base + vec2( u2, v0 ) * step, ly, rz );
	sum += uw3 * vw0 * sampleShadowTap( tex, smp, base + vec2( u3, v0 ) * step, ly, rz );

	sum += uw0 * vw1 * sampleShadowTap( tex, smp, base + vec2( u0, v1 ) * step, ly, rz );
	sum += uw1 * vw1 * sampleShadowTap( tex, smp, base + vec2( u1, v1 ) * step, ly, rz );
	sum += uw2 * vw1 * sampleShadowTap( tex, smp, base + vec2( u2, v1 ) * step, ly, rz );
	sum += uw3 * vw1 * sampleShadowTap( tex, smp, base + vec2( u3, v1 ) * step, ly, rz );

	sum += uw0 * vw2 * sampleShadowTap( tex, smp, base + vec2( u0, v2 ) * step, ly, rz );
	sum += uw1 * vw2 * sampleShadowTap( tex, smp, base + vec2( u1, v2 ) * step, ly, rz );
	sum += uw2 * vw2 * sampleShadowTap( tex, smp, base + vec2( u2, v2 ) * step, ly, rz );
	sum += uw3 * vw2 * sampleShadowTap( tex, smp, base + vec2( u3, v2 ) * step, ly, rz );

	sum += uw0 * vw3 * sampleShadowTap( tex, smp, base + vec2( u0, v3 ) * step, ly, rz );
	sum += uw1 * vw3 * sampleShadowTap( tex, smp, base + vec2( u1, v3 ) * step, ly, rz );
	sum += uw2 * vw3 * sampleShadowTap( tex, smp, base + vec2( u2, v3 ) * step, ly, rz );
	sum += uw3 * vw3 * sampleShadowTap( tex, smp, base + vec2( u3, v3 ) * step, ly, rz );

	return sum / 2704.0;
}
