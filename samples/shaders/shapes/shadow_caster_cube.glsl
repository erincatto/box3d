// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Shadow caster for cubes. Depth-only, no fragment output, the
// pipeline runs with color_count=0 and writes only into the cascade's
// depth attachment.
//
// Shares the cube instance buffer layout with cube.glsl: each instance is
// a 3x4 row-major affine plus PBR material data (base_color + material are
// unused here but kept in the layout so one storage buffer feeds both
// pipelines without any repacking, struct stride MUST match the lit
// pipeline's, otherwise indexing breaks).
//
// Shadow space is standard Z (0 near = closer to light, 1 far). The light
// matrix uploaded as ub_frame.light_view_proj is constructed with
// an orthographic projection and mat4LookAt on the C side, which already
// produces the correct convention. No reverse-Z swap.
//
// `@module shadow_cube` keeps the generated symbols (shadow_cube_*) out
// collision with the lit cube pipeline's symbols.

#pragma sokol @module shadow_cube

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 b3Vec3
#pragma sokol @ctype vec2 b3Vec2

#pragma sokol @vs vs

layout( binding = 0 ) uniform ub_frame
{
	mat4 light_view_proj;
};

// per-draw offset. Opaque draws set inst_base.x = 0, transparent
// draws (per the SHADOW_FULL flag) pick one instance at a time from the
// transparent arena.
layout( binding = 2 ) uniform ub_draw
{
	ivec4 inst_base;
};

struct cube_instance
{
	vec4 xform_row0;
	vec4 xform_row1;
	vec4 xform_row2;
	vec4 base_color; // unused in caster, present so layout matches lit pipeline
	vec4 material;	 // unused in caster, present so layout matches lit pipeline
};

layout( binding = 0 ) readonly buffer cube_instances
{
	cube_instance instances[];
};

in vec3 in_pos;

void main()
{
	cube_instance inst = instances[inst_base.x + gl_InstanceIndex];
	vec4 local_pos = vec4( in_pos, 1.0 );
	vec3 world_pos =
		vec3( dot( inst.xform_row0, local_pos ), dot( inst.xform_row1, local_pos ), dot( inst.xform_row2, local_pos ) );
	gl_Position = light_view_proj * vec4( world_pos, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs
// Empty FS: pipeline is created with color_count = 0. The rasterizer
// generates depth values from gl_Position, nothing else gets written.
void main()
{
}
#pragma sokol @end

#pragma sokol @program caster vs fs
