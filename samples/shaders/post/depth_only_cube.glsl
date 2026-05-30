// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Depth-only pre-pass for cubes. Writes positive
// linear view-space depth to an R32F color attachment, the D32F depth
// attachment is transient (rasterizer / early-Z only, never sampled). The
// full XeGTAO port reconstructs normals from depth derivatives inside
// MainPass, so the prepass produces no normal channel.

#pragma sokol @module depth_only_cube

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

struct cube_instance
{
	vec4 xform_row0;
	vec4 xform_row1;
	vec4 xform_row2;
	vec4 base_color; // unused, layout match with lit pipeline
	vec4 material; // unused, layout match with lit pipeline
};

layout(binding = 0) readonly buffer cube_instances
{
	cube_instance instances[];
};

in vec3 in_pos;
out float v_linear_view_z;

void main()
{
	cube_instance inst = instances[gl_InstanceIndex];
	vec4 local_pos = vec4(in_pos, 1.0);
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
