// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Shadow caster for capsules. As with the sphere caster, we substitute
// a proxy mesh for the analytic impostor. The proxy is a lat/long cylinder
// with two hemispherical caps (see BuildCapsuleProxy in renderer.c for
// counts and the silhouette-straddling scale). Vertices are authored in
// a "unit-capsule" coordinate frame where each component carries enough
// information to scale independently by halfLength and radius.
//
// Each vertex is encoded as two attributes:
// in_axis, direction along the long axis (vec3). Scaled by halfLength.
// For body verts: (+/-1, 0, 0). For cap verts: same as the
//                attached cap (i.e., (+1,0,0) for +X cap, (-1,0,0) for -X).
// in_radial, radial offset (vec3). Scaled by radius. For body verts:
// (0, cos theta, sin theta). For cap verts: an extension of the
//                hemisphere parameterization that includes the cap's
// axial bulge, e.g. for +X cap at latitude alpha: in_radial =
// (sin alpha, cos alpha cos beta, cos alpha sin beta).
//
// The world position is then
//   world = center + R * (in_axis * halfLength + in_radial * radius)
// where R is the capsule's rotation. Since rotation is orthonormal and
// halfLength/radius are scalars, the encoding scales linearly and
// produces a true capsule for any (halfLength, radius) pair.
//
// `@module shadow_capsule` namespaces the generated symbols.

#pragma sokol @module shadow_capsule

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
	vec4 xform_row0; // .xyz = rotation row 0, .w = center.x
	vec4 xform_row1; // .xyz = rotation row 1, .w = center.y
	vec4 xform_row2; // .xyz = rotation row 2, .w = center.z
	vec4 base_color; // unused in caster, present so layout matches lit pipeline
	vec4 params;	 // .x = halfLength, .y = radius
	vec4 material;	 // unused in caster, present so layout matches lit pipeline
};

layout( binding = 0 ) readonly buffer instances
{
	instance items[];
};

in vec3 in_axis;   // axis-direction component, scaled by halfLength
in vec3 in_radial; // radial component, scaled by radius

void main()
{
	instance inst = items[inst_base.x + gl_InstanceIndex];
	vec3 center = vec3( inst.xform_row0.w, inst.xform_row1.w, inst.xform_row2.w );
	float halfLength = inst.params.x;
	float radius = inst.params.y;

	vec3 local_pos = in_axis * halfLength + in_radial * radius;
	vec3 world_pos = center + vec3( dot( inst.xform_row0.xyz, local_pos ), dot( inst.xform_row1.xyz, local_pos ),
									dot( inst.xform_row2.xyz, local_pos ) );
	gl_Position = light_view_proj * vec4( world_pos, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs
void main()
{
}
#pragma sokol @end

#pragma sokol @program caster vs fs
