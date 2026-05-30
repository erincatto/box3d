// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Shadow caster for triangle geometries (hulls, meshes, heightfields).
// Depth-only: color_count=0, empty fragment shader, writes only the
// cascade's depth attachment.
//
// Mirrors geom.glsl exactly in instance/vertex layout so the same
// per-frame instance buffer + per-entry vbo/ibo bindings drive both
// pipelines. The lit pipeline samples the surface normal for shading,
// here it's accepted as a vertex attribute and ignored. Keeping the
// vertex layout identical means the renderer doesn't have to maintain
// two parallel buffer schemes.
//
// `@module shadow_geom` keeps the generated symbols out of collision
// with the lit geom pipeline's symbols.

#pragma sokol @module shadow_geom

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 b3Vec3
#pragma sokol @ctype vec2 b3Vec2

#pragma sokol @vs vs

layout( binding = 0 ) uniform ub_frame
{
	mat4 light_view_proj;
};

layout( binding = 2 ) uniform ub_draw
{
	ivec4 inst_base; // .x = first-instance offset into the global geom instance buffer
};

struct instance
{
	vec4 xform_row0; // .xyz = rotation row 0, .w = center.x
	vec4 xform_row1;
	vec4 xform_row2;
	vec4 base_color; // unused here, layout matches lit pipeline
	vec4 scale;		 // .xyz = per-instance scale (mesh only), hulls/heightfields = 1
	vec4 material;	 // unused here, layout matches lit pipeline
};

layout( binding = 0 ) readonly buffer instances
{
	instance items[];
};

// Position-only input. The shared vertex buffer carries position+normal
// (stride 24 bytes), but this caster doesn't read the normal. We just
// don't declare an `in_normal` attribute so glslang doesn't keep it
// around (and the GL driver doesn't warn about an unused attribute slot
// after stripping). The C-side pipeline desc skips the in_normal binding
// to match.
in vec3 in_pos;

void main()
{
	instance inst = items[inst_base.x + gl_InstanceIndex];

	vec3 scaled_pos = in_pos * inst.scale.xyz;
	vec4 local_pos = vec4( scaled_pos, 1.0 );
	vec3 world_pos =
		vec3( dot( inst.xform_row0, local_pos ), dot( inst.xform_row1, local_pos ), dot( inst.xform_row2, local_pos ) );

	gl_Position = light_view_proj * vec4( world_pos, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs
void main()
{
}
#pragma sokol @end

#pragma sokol @program caster vs fs
