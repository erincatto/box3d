// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Shadow caster for spheres. The lit sphere pipeline ray-casts an
// analytic sphere from a bounding-cube proxy, but we can't afford to
// write gl_FragDepth in a depth-only shadow pass (early-Z killed, high
// shadow-map resolution makes the cost prohibitive). Instead the shadow
// pipeline rasterizes a low-poly icosphere proxy (subdiv-1, 42 verts, 80
// tris) generated at InitRenderer and stored in dedicated icosphere vbuf/ibuf.
//
// Per-vertex: in_pos is a unit-sphere position (length == 1). Per-instance
// reads from the same sphere instance buffer the lit pipeline uses, the
// rotation rows are ignored (a sphere is rotation-invariant) and only
// the center (xform_row*.w) and radius (params.x) participate.
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

in vec3 in_pos; // unit-icosphere vertex (length == 1)

void main()
{
	instance inst = items[inst_base.x + gl_InstanceIndex];
	vec3 center = vec3( inst.xform_row0.w, inst.xform_row1.w, inst.xform_row2.w );
	float radius = inst.params.x;
	vec3 world_pos = center + radius * in_pos;
	gl_Position = light_view_proj * vec4( world_pos, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs
void main()
{
}
#pragma sokol @end

#pragma sokol @program caster vs fs
