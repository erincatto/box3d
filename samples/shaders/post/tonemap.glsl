// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// AgX tonemap fullscreen pass.
//
// Reads the resolved HDR scene color (RGBA16F), applies an exposure stop,
// runs AgX (Standard look, identity ASC-CDL), and writes display-encoded
// sRGB to a linear-format (non-sRGB) framebuffer.
//
// Implementation is the "Minimal AgX" port of Troy Sobotka's reference
// (https://iolite-engine.com/blog_posts/minimal_agx_implementation),
// the same shader form used by Three.js and Filament. The 6th-order
// polynomial in agxDefaultContrastApprox is a fit of the AgX sigmoid,
// mean error^2 vs. the reference curve is ~3.7e-6.
//
// Color path:
//   1. linear sRGB HDR (scene)
// 2. x exposure (pow(2, ev))
// 3. AgX inset matrix -> "AgX color space"
// 4. clamp(log2(.), MIN_EV, MAX_EV) -> [0,1] log-space
// 5. polynomial sigmoid -> [0,1] log-space, S-shaped
// 6. AgX look -> luma-preserving saturation (params.z, 1.0 = identity)
// 7. AgX outset matrix -> display-encoded sRGB (no further EOTF)
//
// Output target format is linear unorm (BGRA8/RGBA8, NOT SRGB), so the
// outset-matrix result writes verbatim, no pow(., 2.2) at the end. If
// the target ever becomes sRGB-formatted (driver does linear->sRGB encode
// on store), add `v = pow(max(v, 0.0), vec3(2.2))` after the outset
// matrix to give the driver linear values.
//
// UV handling: the scene RT was written by the rasterizer at SV_Position
// (top-origin on D3D11/Metal/WGPU, bottom-origin on GL). Sampling it here
// with `v_uv = ndc * 0.5 + 0.5` reads the wrong row on D3D11/Metal/WGPU
// because v_uv.y=1 always means "max-V texel", which is top on GL but
// bottom on the others. The C side passes a backend-conditional uvYSign
// (+1 GL, -1 D3D/Metal/WGPU), we flip v_uv.y at sample time on the
// negative-sign backends. Same pattern as brdf_lut.glsl, just applied at
// the consumer (sample-time) instead of the producer (write-time).

#pragma sokol @module tonemap

#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs

out vec2 v_uv;

void main()
{
	vec2 ndc = vec2( float( ( gl_VertexIndex << 1 ) & 2 ), float( gl_VertexIndex & 2 ) ) * 2.0 - 1.0;
	v_uv = ndc * 0.5 + 0.5;
	gl_Position = vec4( ndc, 0.0, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs

layout( binding = 0 ) uniform ub_tonemap
{
	// .x = ev (exposure stops, final multiplier = pow(2, ev))
	// .y = uvYSign (+1 GL, -1 D3D11/Metal/WGPU), flips sample-time v_uv.y
	// so the scene RT reads back upright on every backend.
	// .z = AgX look saturation (1.0 = identity / stock AgX Standard look)
	// .w reserved
	vec4 params;
};

layout( binding = 0 ) uniform texture2D tex_scene;
layout( binding = 0 ) uniform sampler smp_scene;

in vec2 v_uv;
out vec4 out_color;

// 6th-order polynomial fit of the AgX sigmoid (Sobotka). Operates on the
// [0,1] log-space encoded value, output stays in [0,1] log-space but
// S-shaped to match the AgX contrast curve.
vec3 agxDefaultContrastApprox( vec3 x )
{
	vec3 x2 = x * x;
	vec3 x4 = x2 * x2;
	return +15.5 * x4 * x2 - 40.14 * x4 * x + 31.96 * x4 - 6.868 * x2 * x + 0.4298 * x2 + 0.1191 * x - 0.00232;
}

vec3 agx( vec3 v )
{
	// Inset matrix: linear sRGB primaries -> AgX inset primaries.
	const mat3 inset = mat3( 0.842479062253094, 0.0423282422610123, 0.0423756549057051, 0.0784335999999992, 0.878468636469772,
							 0.0784336, 0.0792237451477643, 0.0791661274605434, 0.879142973793104 );
	const float MIN_EV = -12.47393;
	const float MAX_EV = 4.026069;

	v = inset * v;
	// Floor to a tiny positive so log2 stays finite at black.
	v = max( v, vec3( 1.0e-10 ) );
	v = clamp( log2( v ), MIN_EV, MAX_EV );
	v = ( v - MIN_EV ) / ( MAX_EV - MIN_EV );
	v = agxDefaultContrastApprox( v );
	return v;
}

vec3 agxOutset( vec3 v )
{
	// Inverse of the inset matrix, brings the sigmoid output back to display
	// sRGB primaries. Result is display-encoded sRGB (ready for an unorm RT).
	const mat3 outset = mat3( 1.19687900512017, -0.0528968517574562, -0.0529716355144438, -0.0980208811401368, 1.15190312990417,
							  -0.0980434501171241, -0.0990297440797205, -0.0989611768448433, 1.15107367264116 );
	return outset * v;
}

// AgX "look": luma-preserving saturation, applied to the sigmoid output
// while it's still in AgX-encoded [0,1] space (before the outset matrix).
// saturation 1.0 is identity: AgX's stock Standard look. AgX walks bright
// colors toward white ("path to white"), which reads as desaturation.
// saturation > 1.0 pushes them back out, < 1.0 mutes them. Rec.709 luma
// weights hold perceived brightness fixed. The reference Minimal AgX look
// also exposes per-channel slope/power ("Punchy"/"Golden" presets), we
// wire only saturation, the one knob this debug renderer needs.
vec3 agxLook( vec3 v, float saturation )
{
	const vec3 LUMA = vec3( 0.2126, 0.7152, 0.0722 );
	float luma = dot( v, LUMA );
	return luma + saturation * ( v - luma );
}

void main()
{
	// Backend-conditional v_uv.y flip: when params.y < 0 (D3D11/Metal/WGPU),
	// replace v_uv.y with (1 - v_uv.y) so the sample reads the correct row.
	// Equivalent to `mix(1.0-y, y, step(0.0, params.y))` but cheaper.
	float sampleY = ( params.y > 0.0 ) ? v_uv.y : ( 1.0 - v_uv.y );
	vec3 hdr = textureLod( sampler2D( tex_scene, smp_scene ), vec2( v_uv.x, sampleY ), 0.0 ).rgb;
	hdr *= pow( 2.0, params.x );
	vec3 ldr = agxOutset( agxLook( agx( hdr ), params.z ) );
	out_color = vec4( clamp( ldr, 0.0, 1.0 ), 1.0 );
}
#pragma sokol @end

#pragma sokol @program prog vs fs
