// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Depth-only pre-pass for triangle geometries (hulls, meshes,
// height fields). Same vertex math as the lit
// geom pipeline, FS writes positive linear view-space depth only.

#pragma sokol @module depth_only_geom

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 b3Vec3
#pragma sokol @ctype vec2 b3Vec2

#pragma sokol @vs vs

layout(binding = 0) uniform ub_frame
{
	mat4 view_proj;
	mat4 view;
};

layout(binding = 2) uniform ub_draw
{
	ivec4 inst_base; // .x = base index into the global instance array
};

struct instance
{
	vec4 xform_row0;
	vec4 xform_row1;
	vec4 xform_row2;
	vec4 base_color; // unused, layout match
	vec4 scale; // .xyz = per-instance scale
	vec4 material; // unused, layout match
};

layout(binding = 0) readonly buffer instances
{
	instance items[];
};

// Position-only input. The shared vertex buffer carries position+normal
// (stride 24 bytes), but this pre-pass doesn't read the normal. We just
// don't declare an `in_normal` attribute so glslang doesn't keep it
// around (and the GL driver doesn't warn about an unused attribute slot
// after stripping). The C-side pipeline desc skips the in_normal binding
// to match.
in vec3 in_pos;

out float v_linear_view_z;

void main()
{
	instance inst = items[inst_base.x + gl_InstanceIndex];
	
	vec3 scaled_pos = in_pos * inst.scale.xyz;
	vec4 local_pos = vec4(scaled_pos, 1.0);
	vec3 world_pos = vec3(
		dot(inst.xform_row0, local_pos),
		dot(inst.xform_row1, local_pos),
		dot(inst.xform_row2, local_pos)
	);
	
	vec4 view_h = view * vec4(world_pos, 1.0);
	v_linear_view_z = -view_h.z;
	gl_Position = view_proj * vec4(world_pos, 1.0);
}

#pragma sokol @end

#pragma sokol @fs fs

in float v_linear_view_z;
out vec4 out_linear_depth;

void main()
{
	out_linear_depth = vec4(v_linear_view_z, 0.0, 0.0, 0.0);
}

#pragma sokol @end

#pragma sokol @program prog vs fs
