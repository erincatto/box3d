// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Depth-only pre-pass for capsules. Mirrors
// depth_only_sphere: analytic ray-capsule hit, writes linear view-space
// depth into R32F + reverse-Z clip depth into gl_FragDepth.

#pragma sokol @module depth_only_capsule

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
	vec4 xform_row0; // .xyz = R row 0, .w = center.x
	vec4 xform_row1; // .xyz = R row 1, .w = center.y
	vec4 xform_row2; // .xyz = R row 2, .w = center.z
	vec4 base_color; // unused, layout match
	vec4 params; // .x = halfLength, .y = radius
	vec4 material; // unused, layout match
};

layout(binding = 0) readonly buffer instances
{
	instance items[];
};

in vec3 in_pos; // unit-cube corner, half-extent 0.5

sample out vec3 v_world_pos_proxy;
flat out vec3 v_center;
flat out float v_half_length;
flat out float v_radius;
flat out vec3 v_axis_x_world;

void main()
{
	instance inst = items[gl_InstanceIndex];
	vec3 center = vec3(inst.xform_row0.w, inst.xform_row1.w, inst.xform_row2.w);
	float halfLength = inst.params.x;
	float radius = inst.params.y;
	
	vec3 obj_pos = vec3(
		in_pos.x * 2.0 * (halfLength + radius),
		in_pos.y * 2.0 * radius,
		in_pos.z * 2.0 * radius
	);

	vec3 world_pos = center + vec3(
		dot(inst.xform_row0.xyz, obj_pos),
		dot(inst.xform_row1.xyz, obj_pos),
		dot(inst.xform_row2.xyz, obj_pos)
	);
	
	v_world_pos_proxy = world_pos;
	v_center = center;
	v_half_length = halfLength;
	v_radius = radius;
	v_axis_x_world = vec3(inst.xform_row0.x, inst.xform_row1.x, inst.xform_row2.x);
	
	gl_Position = view_proj * vec4(world_pos, 1.0);
}

#pragma sokol @end

#pragma sokol @fs fs

layout(binding = 1) uniform ub_pass
{
	mat4 view_proj;
	mat4 view;
	vec4 camera_pos; // .xyz = world camera position
};

sample in vec3 v_world_pos_proxy;
flat in vec3 v_center;
flat in float v_half_length;
flat in float v_radius;
flat in vec3 v_axis_x_world;

out vec4 out_linear_depth;

void main()
{
	vec3 ro = camera_pos.xyz;
	vec3 rd = normalize(v_world_pos_proxy - ro);
	
	vec3 axis = v_axis_x_world;
	vec3 pa = v_center - v_half_length * axis;
	vec3 pb = v_center + v_half_length * axis;
	
	vec3 ba = pb - pa;
	vec3 oa = ro - pa;
	float baba = dot(ba, ba);
	float bard = dot(ba, rd);
	float baoa = dot(ba, oa);
	float rdoa = dot(rd, oa);
	float oaoa = dot(oa, oa);
	
	float a_c = baba - bard * bard;
	float b_c = baba * rdoa - baoa * bard;
	float c_c = baba * oaoa - baoa * baoa - v_radius * v_radius * baba;
	float h_c = b_c * b_c - a_c * c_c;
	
	float t = -1.0;
	
	if (h_c >= 0.0 && a_c > 1e-8)
	{
		float t_body = (-b_c - sqrt(h_c)) / a_c;
		float y = baoa + t_body * bard;
		if (t_body > 0.0 && y > 0.0 && y < baba)
		{
			t = t_body;
		}
	}
	
	// Cap A (-X).
	{
		vec3 oc = ro - pa;
		float b = dot(oc, rd);
		float c = dot(oc, oc) - v_radius * v_radius;
		float h = b * b - c;
		if (h >= 0.0)
		{
			float tc = -b - sqrt(h);
			if (tc > 0.0 &&(t < 0.0 || tc < t))
			{
				vec3 hit = ro + tc * rd;
				if (dot(hit - pa, axis) <= 0.0)
				{
					t = tc;
				}
			}
		}
	}
	// Cap B (+X).
	{
		vec3 oc = ro - pb;
		float b = dot(oc, rd);
		float c = dot(oc, oc) - v_radius * v_radius;
		float h = b * b - c;
		if (h >= 0.0)
		{
			float tc = -b - sqrt(h);
			if (tc > 0.0 &&(t < 0.0 || tc < t))
			{
				vec3 hit = ro + tc * rd;
				if (dot(hit - pb, axis) >= 0.0)
				{
					t = tc;
				}
			}
		}
	}
	
	if (t < 0.0)
	{
		discard;
	}
	
	vec3 hit_world = ro + t * rd;
	float view_z = -(view * vec4(hit_world, 1.0)).z;
	
	vec4 hit_clip = view_proj * vec4(hit_world, 1.0);
	gl_FragDepth = hit_clip.z / hit_clip.w;
	
	out_linear_depth = vec4(view_z, 0.0, 0.0, 0.0);
}
#pragma sokol @end

#pragma sokol @program prog vs fs
