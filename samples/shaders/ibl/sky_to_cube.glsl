// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Sky cubemap renderer.
//
// Fills one face of a 6xN^2 RGBA16F cubemap per draw with the Preetham
// analytic daylight model. The C side runs six passes (one per face)
// when the sun direction or turbidity changes, the resulting cubemap
// becomes both the diffuse-irradiance source (via SH projection on the
// CPU) and the input to the GGX prefilter pass.
//
// Direction reconstruction. Each draw receives a per-face mat3 basis
// (right, up, forward). The FS computes a world-space direction from
// the fragment's v_uv in [0,1]^2 as
//	dir = normalize(forward + (2*v_uv.x - 1) * right + (2*v_uv.y - 1) * up)
// The basis vectors are filled C-side to match the GL cubemap sampling
// convention (which sokol abstracts uniformly across backends), so a
// direction d written here is recovered by sampling the cubemap with
// the same d.

// @module sky_to_cube prefixes the generated typedefs (struct, slot
// enum, static byte arrays) so this header coexists in ibl.c with
// brdf_lut.glsl.h and prefilter.glsl.h without collisions.

#pragma sokol @module sky_to_cube

#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs

out vec2 v_uv;

void main()
{
	vec2 ndc = vec2(float((gl_VertexIndex << 1) & 2),
	float(gl_VertexIndex & 2)) * 2.0 - 1.0;
	v_uv = ndc * 0.5 + 0.5;
	gl_Position = vec4(ndc, 0.0, 1.0);
}
#pragma sokol @end

#pragma sokol @fs fs

#pragma sokol @include ../common/preetham.glsl

layout(binding = 0)uniform ub_face
{
	// Three rows of a mat3 basis. Stored as vec4 for std140 alignment,
	// .xyz are the axis vectors. The .w lanes carry scalars to save UBO
	// slots: face_up.w = z-up flag, face_forward.w = below-horizon fade.
	vec4 face_right; // .xyz = world-space face right axis
	vec4 face_up; // .xyz = world-space face up axis, .w = z-up flag (0 or 1)
	vec4 face_forward; // .xyz = world-space face forward axis, .w = fade weight
	vec4 sun_dir_world; // .xyz = world-space direction TO sun, .w = turbidity
};

in vec2 v_uv;
out vec4 out_color;

void main()
{
	vec2 uv = v_uv * 2.0 - 1.0; // [-1, +1] across the face
	vec3 dir = normalize(face_forward.xyz + uv.x * face_right.xyz + uv.y * face_up.xyz);
	
	vec3 rgb = preethamSkyScaled(dir, sun_dir_world.xyz, sun_dir_world.w, face_forward.w, face_up.w);
	out_color = vec4(rgb, 1.0);
}
#pragma sokol @end

#pragma sokol @program build vs fs
