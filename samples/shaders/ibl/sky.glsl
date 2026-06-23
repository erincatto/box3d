// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Procedural sky: Preetham/Shirley/Smits 1999 analytic daylight.
//
// Drawn as a fullscreen triangle inside the main lit pass, AFTER all opaque
// shapes, with depth test GREATER_EQUAL against the reverse-Z far plane
// (clear value = 0). NDC z = 0 puts the triangle at the far plane, opaque
// fragments wrote z > 0 (closer in reverse-Z), so GREATER_EQUAL discards
// the sky there and keeps it on untouched pixels. Depth write is disabled,
// the sky is "infinitely far" and shouldn't write to z.
//
// View-ray reconstruction. VS unprojects each fullscreen-triangle vertex
// to homogeneous world coordinates via inv_view_proj and forwards them
// undivided. All three vertices have gl_Position.w = 1, so perspective-
// correct interpolation degenerates to linear interp, and matrix linearity
// means the interpolated v_world_homog at a fragment equals inv_view_proj
// times that fragment's NDC point exactly. FS divides by w and subtracts
// the camera position to get the world-space view direction.
//
// Below-horizon handling. Preetham is invalid for sun elevations near and
// below the horizon. The renderer C side computes a smoothstep fade weight
// between sin(2 deg) and sin(5 deg) and passes it as sun_dir_world.w, the FS
// just multiplies the output by it. preethamSky in common/preetham.glsl
// clamps its inputs to the upper hemisphere internally to keep the math
// finite while the fade weight drives the output to zero.
//
// Output is linear sRGB radiance, scaled by an empirical luminance factor
// chosen to land near the target sun:sky ratio (~8-15:1) under the
// renderer's sun_color premultiplied-by-pi convention.

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 b3Vec3

#pragma sokol @vs vs

// Per-shader UBO name prefix (sky_*) to keep sokol-shdc's generated typedefs
// disjoint from cube.glsl.h's unprefixed ub_frame_t / ub_pass_t, both
// headers are included from src/gfx/renderer.c.
layout(binding = 0) uniform sky_ub_frame
{
	mat4 inv_view_proj;
};

out vec4 v_world_homog; // inv_view_proj * (ndc, 0, 1), undivided

void main()
{
	// Standard fullscreen-triangle trick: 3 vertices cover the NDC square
	// and 25% extra outside, which the rasterizer clips. Avoids the
	// two-triangle seam-discontinuity that a quad has at its diagonal.
	// vert 0 -> (-1, -1)
	// vert 1 -> ( 3, -1)
	// vert 2 -> (-1, 3).
	vec2 ndc = vec2(float((gl_VertexIndex << 1) & 2), float(gl_VertexIndex & 2)) * 2.0 - 1.0;
	
	// NDC z = 0 -> reverse-Z far plane. Depth test GREATER_EQUAL passes
	// against the cleared depth of 0 on untouched pixels and fails on any
	// opaque pixel (whose depth is > 0 in reverse-Z).
	vec4 clip = vec4(ndc, 0.0, 1.0);
	gl_Position = clip;
	
	v_world_homog = inv_view_proj * clip;
}
#pragma sokol @end

#pragma sokol @fs fs

#pragma sokol @include ../common/preetham.glsl

layout(binding = 1)uniform sky_ub_pass
{
	vec4 sun_dir_world; // .xyz = world dir TO sun (normalized), .w = below-horizon fade weight [0,1]
	vec4 sky_params; // .x = turbidity (Preetham T), .y = z-up flag (0 or 1), .zw = reserved
	vec4 camera_pos; // .xyz = world camera position, .w = unused
};

in vec4 v_world_homog;
out vec4 out_color;

void main()
{
	// World-space view direction at this fragment.
	vec3 world_pos = v_world_homog.xyz / v_world_homog.w;
	vec3 view_dir = normalize(world_pos - camera_pos.xyz);
	
	// Renormalize the sun defensively. Caller is the renderer with
	// sun.dirToSun already normalized, but an undetected zero vector
	// here would emit NaNs.
	vec3 sun_dir = normalize(sun_dir_world.xyz);
	
	// Empirical luminance scale + below-horizon fade live in
	// preethamSkyScaled so the IBL sky cubemap (sky_to_cube.glsl) sees
	// the same backdrop. Fade weight comes from the C side via
	// sun_dir_world.w (1 above horizon, 0 below).
	vec3 rgb = preethamSkyScaled(view_dir, sun_dir, sky_params.x, sun_dir_world.w, sky_params.y);
	
	out_color = vec4(rgb, 1.0);
}
#pragma sokol @end

#pragma sokol @program sky vs fs
