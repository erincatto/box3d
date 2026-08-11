// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Shadow caster for spheres. Rasterizes the same bounding cube the lit
// impostor uses and ray-casts the exact sphere, so caster and receiver
// agree on where the surface is and bias only has to cover projection
// and PCF slope. An inscribed proxy mesh falls short of the true surface
// by a couple percent of the radius, and that gap has to be paid for
// with extra depth bias.
//
// The light is directional, so every ray is parallel: the origin is the
// proxy fragment's world position and the direction is the direction
// light travels. Under an orthographic projection depth is linear in
// distance along that direction, so marching forward from the proxy
// face can only push depth away from the light. Hence
// `layout(depth_greater)`, which lets the hardware keep early-Z
// rejection despite the gl_FragDepth write. The promise rests on the
// pipeline culling back faces, so the rasterized surface is always the
// light-facing one.
//
// Depth is rebuilt from the projection's depth row rather than advanced
// from gl_FragCoord.z, which would be one madd instead of a dot4. D3D11
// rejects a pixel shader that both reads SV_Position and writes
// conservative depth unless the position input is declared
// noperspective centroid or sample, and neither GLSL nor SPIRV-Cross
// gives us a way to qualify it. The two agree to within rounding: the
// only place they can disagree is the single point where the proxy face
// is tangent to the surface.
//
// Per-instance reads from the same sphere instance buffer the lit
// pipeline uses. The rotation rows are ignored (a sphere is
// rotation-invariant) and only the center (xform_row*.w) and radius
// (params.x) participate.
//
// `@module shadow_sphere` namespaces the generated symbols.

#pragma sokol @module shadow_sphere

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 b3Vec3
#pragma sokol @ctype vec2 b3Vec2

#pragma sokol @vs vs

layout( binding = 0 ) uniform ub_frame
{
	mat4 light_view_proj;
};

// per-draw offset, see shadow_caster_cube.glsl.
layout( binding = 2 ) uniform ub_draw
{
	ivec4 inst_base;
};

struct instance
{
	vec4 xform_row0; // .xyz = rotation row 0 (unused), .w = center.x
	vec4 xform_row1; // .xyz = rotation row 1 (unused), .w = center.y
	vec4 xform_row2; // .xyz = rotation row 2 (unused), .w = center.z
	vec4 base_color; // unused in caster, present so layout matches lit pipeline
	vec4 params;	 // .x = radius
	vec4 material;	 // unused in caster, present so layout matches lit pipeline
};

layout( binding = 0 ) readonly buffer instances
{
	instance items[];
};

in vec3 in_pos; // unit-cube corner, half-extent 0.5

out vec3 v_world_pos_proxy;

flat out vec3 v_center;
flat out float v_radius;

void main()
{
	instance inst = items[inst_base.x + gl_InstanceIndex];
	vec3 center = vec3( inst.xform_row0.w, inst.xform_row1.w, inst.xform_row2.w );
	float radius = inst.params.x;

	// Half-extent 0.5 scaled by 2*radius gives a cube exactly enclosing
	// the sphere. Rotation is skipped, a sphere is rotation-invariant.
	vec3 world_pos = center + ( 2.0 * radius ) * in_pos;

	v_world_pos_proxy = world_pos;
	v_center = center;
	v_radius = radius;

	gl_Position = light_view_proj * vec4( world_pos, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs

// UBOs are per-stage in sokol-shdc, so this is a second block rather
// than a reuse of the VS ub_frame.
layout( binding = 1 ) uniform ub_pass
{
	vec4 light_dir; // .xyz = world-space direction light travels (normalized)
	vec4 depth_row; // depth row of the light view-proj, world position to clip depth
};

in vec3 v_world_pos_proxy;

flat in vec3 v_center;
flat in float v_radius;

layout( depth_greater ) out float gl_FragDepth;

void main()
{
	vec3 ro = v_world_pos_proxy;
	vec3 rd = light_dir.xyz;

	// Ray-sphere, near root. Reference: https://iquilezles.org/articles/intersectors/
	vec3 oc = ro - v_center;
	float b = dot( oc, rd );
	float c = dot( oc, oc ) - v_radius * v_radius;
	float h = b * b - c;
	if ( h < 0.0 )
	{
		// Cube corner, the ray clears the sphere.
		discard;
	}

	// Non-negative because the proxy face is tangent to the sphere at
	// worst, never inside it.
	float t = max( -b - sqrt( h ), 0.0 );

	// Orthographic light, so the clip w is 1 and the divide drops out.
	gl_FragDepth = dot( depth_row, vec4( ro + t * rd, 1.0 ) );
}
#pragma sokol @end

#pragma sokol @program caster vs fs
