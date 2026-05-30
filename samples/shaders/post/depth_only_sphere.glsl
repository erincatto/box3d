// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Depth-only pre-pass for spheres. Same shape as
// the lit sphere impostor: bounding-cube proxy in the VS, analytic
// ray-sphere intersection in the FS. Writes the analytic surface's linear
// view-space depth to an R32F color attachment and the reverse-Z clip
// depth to gl_FragDepth (transient D32F attachment, rasterizer only).

#pragma sokol @module depth_only_sphere

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 b3Vec3
#pragma sokol @ctype vec2 b3Vec2

#pragma sokol @vs vs

layout(binding = 0) uniform ub_frame
{
	mat4 view_proj;
};

struct instance
{
	vec4 xform_row0; // .w = center.x
	vec4 xform_row1; // .w = center.y
	vec4 xform_row2; // .w = center.z
	vec4 base_color; // unused, layout match
	vec4 params; // .x = radius
	vec4 material; // unused, layout match
};

layout(binding = 0) readonly buffer instances
{
	instance items[];
};

in vec3 in_pos; // unit-cube corner, half-extent 0.5

sample out vec3 v_world_pos_proxy;
flat out vec3 v_center;
flat out float v_radius;

void main()
{
	instance inst = items[gl_InstanceIndex];
	vec3 center = vec3(inst.xform_row0.w, inst.xform_row1.w, inst.xform_row2.w);
	float radius = inst.params.x;
	vec3 world_pos = center + (2.0 * radius) * in_pos;
	
	v_world_pos_proxy = world_pos;
	v_center = center;
	v_radius = radius;
	
	gl_Position = view_proj * vec4(world_pos, 1.0);
}

#pragma sokol @end

#pragma sokol @fs fs

layout(binding = 1) uniform ub_pass
{
	mat4 view_proj; // for gl_FragDepth (reverse-Z)
	mat4 view; // for linear view-space depth
	vec4 camera_pos; // .xyz = world camera position
};

sample in vec3 v_world_pos_proxy;
flat in vec3 v_center;
flat in float v_radius;

out vec4 out_linear_depth;

void main()
{
	vec3 ro = camera_pos.xyz;
	vec3 rd = normalize(v_world_pos_proxy - ro);
	
	vec3 oc = ro - v_center;
	float b = dot(oc, rd);
	float c2 = dot(oc, oc) - v_radius * v_radius;
	float h = b * b - c2;
	if (h < 0.0) discard;
	float t = -b - sqrt(h);
	if (t < 0.0) discard;
	
	vec3 hit_world = ro + t * rd;
	float view_z = -(view * vec4(hit_world, 1.0)).z;
	
	vec4 hit_clip = view_proj * vec4(hit_world, 1.0);
	gl_FragDepth = hit_clip.z / hit_clip.w;
	
	out_linear_depth = vec4(view_z, 0.0, 0.0, 0.0);
}

#pragma sokol @end

#pragma sokol @program prog vs fs
