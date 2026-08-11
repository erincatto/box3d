// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Shadow caster for capsules. See shadow_caster_sphere.glsl for why the
// analytic surface is worth the fill cost and what `depth_greater` rests
// on.
//
// Rasterizes the same OBB the lit capsule impostor uses, half-extents
// (halfLength + radius, radius, radius) about the local +X axis.
//
// Only the entry point matters here, not the normal, so this is a
// cheaper intersection than the lit shader's. The capsule is the union
// of a finite cylinder and two cap spheres, and the entry into a union
// is the nearest entry across its parts, so the hemisphere-rejection
// the lit shader needs for correct normals is unnecessary.
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

in vec3 in_pos; // unit-cube corner, half-extent 0.5

out vec3 v_world_pos_proxy;

flat out vec3 v_center;
flat out float v_half_length;
flat out float v_radius;
flat out vec3 v_axis_x_world;

void main()
{
	instance inst = items[inst_base.x + gl_InstanceIndex];
	vec3 center = vec3( inst.xform_row0.w, inst.xform_row1.w, inst.xform_row2.w );
	float halfLength = inst.params.x;
	float radius = inst.params.y;

	vec3 obj_pos = vec3( in_pos.x * 2.0 * ( halfLength + radius ), in_pos.y * 2.0 * radius, in_pos.z * 2.0 * radius );
	vec3 world_pos = center + vec3( dot( inst.xform_row0.xyz, obj_pos ), dot( inst.xform_row1.xyz, obj_pos ),
									dot( inst.xform_row2.xyz, obj_pos ) );

	v_world_pos_proxy = world_pos;
	v_center = center;
	v_half_length = halfLength;
	v_radius = radius;
	// Local +X in world is the first column of R.
	v_axis_x_world = vec3( inst.xform_row0.x, inst.xform_row1.x, inst.xform_row2.x );

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
flat in float v_half_length;
flat in float v_radius;
flat in vec3 v_axis_x_world;

layout( depth_greater ) out float gl_FragDepth;

void main()
{
	vec3 ro = v_world_pos_proxy;
	vec3 rd = light_dir.xyz;

	vec3 pa = v_center - v_half_length * v_axis_x_world;
	vec3 pb = v_center + v_half_length * v_axis_x_world;

	// Ray-cylinder, Quilez closed form.
	// Reference: https://iquilezles.org/articles/intersectors/
	vec3 ba = pb - pa;
	vec3 oa = ro - pa;
	float baba = dot( ba, ba );
	float bard = dot( ba, rd );
	float baoa = dot( ba, oa );
	float rdoa = dot( rd, oa );
	float oaoa = dot( oa, oa );

	float a_c = baba - bard * bard;
	float b_c = baba * rdoa - baoa * bard;
	float c_c = baba * oaoa - baoa * baoa - v_radius * v_radius * baba;
	float h_c = b_c * b_c - a_c * c_c;

	float t = -1.0;

	// Entry into each part, clamped forward rather than rejected. The
	// proxy faces are tangent to the surface along a line, and rounding
	// there puts the origin a hair inside, which drives the near root
	// negative. Treating that as a miss punches a dashed hole straight
	// down the middle of the shadow. A part whose far root is behind us
	// too is a genuine miss, which is what the end faces need: they sit
	// beyond one cap and the other cap is legitimately behind.
	//
	// a_c collapses when the ray runs along the axis, the caps cover
	// that case. baba collapses when halfLength is zero, and the axial
	// range test rejects the body for free.
	if ( h_c >= 0.0 && a_c > 1e-8 )
	{
		float q = sqrt( h_c );
		if ( -b_c + q >= 0.0 )
		{
			float t_body = max( ( -b_c - q ) / a_c, 0.0 );
			float y = baoa + t_body * bard;
			if ( y > 0.0 && y < baba )
			{
				t = t_body;
			}
		}
	}

	// Cap spheres at both ends.
	for ( int i = 0; i < 2; ++i )
	{
		vec3 p = ( i == 0 ) ? pa : pb;
		vec3 oc = ro - p;
		float b = dot( oc, rd );
		float c = dot( oc, oc ) - v_radius * v_radius;
		float h = b * b - c;
		if ( h >= 0.0 )
		{
			float q = sqrt( h );
			if ( -b + q >= 0.0 )
			{
				float tc = max( -b - q, 0.0 );
				if ( t < 0.0 || tc < t )
				{
					t = tc;
				}
			}
		}
	}

	if ( t < 0.0 )
	{
		// OBB corner, the ray clears the capsule.
		discard;
	}

	// Orthographic light, so the clip w is 1 and the divide drops out.
	gl_FragDepth = dot( depth_row, vec4( ro + t * rd, 1.0 ) );
}
#pragma sokol @end

#pragma sokol @program caster vs fs
