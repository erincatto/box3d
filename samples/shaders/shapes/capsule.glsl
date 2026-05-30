// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Capsule impostor.
//
// Per-instance: a 3x4 rigid affine (orthonormal rotation rows in .xyz of each
// xform_row*, capsule center in .w of each row), an RGBA color, and a `params`
// row whose .x is halfLength (segment half-length along local +X) and .y is
// the cap radius. The capsule's long axis is the local +X axis: end caps sit
// at object-space (+/- halfLength, 0, 0) with radius `radius`.
//
// Pipeline overview:
//   VS: emit a unit-cube vertex (half-extent 0.5) scaled to an OBB with
//       half-extents (halfLength + radius, radius, radius), then rotated by
//       the instance rotation and translated by center. Pass world position
//       of the proxy vertex (interpolated) plus flat per-instance data.
//   FS: reconstruct view ray as (camera, normalize(v_world_pos_proxy - camera)).
//       Quilez ray-capsule intersection in world space (cylinder body + two
//       end-cap spheres). discard on miss / behind-camera. Compute world-space
//       normal at the hit, transform to object space via transpose(rotation),
//       shade two great-circle rings at the object-space y=0 and z=0
// equators: on caps these appear as half-circles through the apex,
//       on the cylinder body they continue as longitudinal lines. The X
//       equator is omitted: n_obj.x is identically zero on the cylinder
//       (would paint the entire body), and at the cap-cylinder seam the
//       curvature discontinuity makes its line thinner than the others.
//       Constant screen-space pixel thickness via fwidth, on a north/south
//       hemisphere base contrast. Apply Lambert + ambient against
//       world-space sun, write reverse-Z gl_FragDepth.
//
// `@module capsule` prefixes the generated C symbols (capsule_ub_frame_t,
// capsule_instance_t, capsule_shader_desc, ...) so this header coexists with
// cube.glsl.h and sphere.glsl.h in renderer.c without typedef collisions on
// ub_frame_t.
//
// UBO layout matches sphere.glsl exactly (frame data lives in ub_pass on
// the FS side because the renderer always re-applies ub_pass per draw, and
// one FS UBO is simpler than two).
//
// gl_FragDepth disables hi-Z. That's accepted for impostors.

#pragma sokol @module capsule

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 b3Vec3
#pragma sokol @ctype vec2 b3Vec2

#pragma sokol @vs vs

layout( binding = 0 ) uniform ub_frame
{
	mat4 view_proj;
};

// per-draw offset into the instance buffer. See cube.glsl for the
// rationale.
layout( binding = 2 ) uniform ub_draw
{
	ivec4 inst_base;
};

struct instance
{
	vec4 xform_row0; // .xyz = rotation row 0, .w = center.x
	vec4 xform_row1; // .xyz = rotation row 1, .w = center.y
	vec4 xform_row2; // .xyz = rotation row 2, .w = center.z
	vec4 base_color; // .rgb = linear baseColor, .a = alpha
	vec4 params;	 // .x = halfLength, .y = radius, .zw = unused
	vec4 material;	 // .x = metallic, .y = roughness,
					 // .z = shadow-cast bit (, CPU-only, ignored here),
					 // .w = reserved
};

layout( binding = 0 ) readonly buffer instances
{
	instance items[];
};

in vec3 in_pos; // unit-cube corner, half-extent 0.5

// Interpolated, per-pixel. The FS computes an analytical sub-pixel coverage
// from the signed silhouette distance and feeds it through alpha-to-coverage
// (opaque pipeline) or the alpha blend (transparent pipeline). Mirrors
// sphere.glsl.
out vec3 v_world_pos_proxy;

// Flat: same value for every fragment in this instance.
flat out vec3 v_center;
flat out float v_half_length;
flat out float v_radius;
flat out vec3 v_axis_x_world; // world-space direction of local +X (capsule long axis)
flat out vec3 v_rot_row0;	  // R rows for n_world -> n_obj transform in FS
flat out vec3 v_rot_row1;
flat out vec3 v_rot_row2;
flat out vec4 v_base_color;
flat out vec4 v_material;

void main()
{
	instance inst = items[inst_base.x + gl_InstanceIndex];
	vec3 center = vec3( inst.xform_row0.w, inst.xform_row1.w, inst.xform_row2.w );
	float halfLength = inst.params.x;
	float radius = inst.params.y;

	// OBB half-extents: (halfLength + radius) along local X, radius along Y/Z.
	// Multiply unit-cube vertex (half-extent 0.5) by 2*these to get exact OBB.
	vec3 obj_pos = vec3( in_pos.x * 2.0 * ( halfLength + radius ), in_pos.y * 2.0 * radius, in_pos.z * 2.0 * radius );

	// world = R * obj + center, each component is the dot of an R row with obj_pos.
	vec3 world_pos = center + vec3( dot( inst.xform_row0.xyz, obj_pos ), dot( inst.xform_row1.xyz, obj_pos ),
									dot( inst.xform_row2.xyz, obj_pos ) );

	v_world_pos_proxy = world_pos;
	v_center = center;
	v_half_length = halfLength;
	v_radius = radius;
	// Local +X in world = first column of R = (R[0][0], R[1][0], R[2][0])
	// = (xform_row0.x, xform_row1.x, xform_row2.x).
	v_axis_x_world = vec3( inst.xform_row0.x, inst.xform_row1.x, inst.xform_row2.x );
	v_rot_row0 = inst.xform_row0.xyz;
	v_rot_row1 = inst.xform_row1.xyz;
	v_rot_row2 = inst.xform_row2.xyz;
	v_base_color = inst.base_color;
	v_material = inst.material;

	gl_Position = view_proj * vec4( world_pos, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs

#pragma sokol @include ../common/pbr.glsl

layout( binding = 1 ) uniform ub_pass
{
	vec4 sun_dir_world;		  // .xyz = world-space dir TO sun (normalized)
	vec4 sun_color;			  // .rgb = color, .a = ambient strength (flat-ambient fallback)
	ivec4 flags;			  // .x = debug_view_mode (0 lit, 1 view-depth, 2 cascade-index)
	vec4 camera_pos;		  // .xyz = camera world pos, .w unused
	mat4 view_proj;			  // for reverse-Z gl_FragDepth
	mat4 view;				  // for view-space depth debug mode + cascade selection
	vec4 cascade_far_view_z;  // .xyz = far view-space Z per cascade
	mat4 cascade_matrices[3]; // light-space view-proj per cascade
	vec4 sh[9];				  // IBL diffuse SH coefficients (band 0..2, RGB in .xyz)
	vec4 ibl_params;		  // .x = prefilter cube max_lod, .y = IBL enable (1=IBL, 0=flat ambient), .zw reserved
};

#pragma sokol @image_sample_type tex_shadow depth
layout( binding = 1 ) uniform texture2DArray tex_shadow;
#pragma sokol @sampler_type smp_shadow comparison
layout( binding = 0 ) uniform samplerShadow smp_shadow;

// IBL bindings, same convention as cube.glsl: texture bindings 2/3,
// sampler bindings 1/2, disjoint from the shadow path's 1/0.
#pragma sokol @image_sample_type tex_ibl_spec float
layout( binding = 2 ) uniform textureCube tex_ibl_spec;
#pragma sokol @sampler_type smp_ibl_spec filtering
layout( binding = 1 ) uniform sampler smp_ibl_spec;
#pragma sokol @image_sample_type tex_brdf_lut float
layout( binding = 3 ) uniform texture2D tex_brdf_lut;
#pragma sokol @sampler_type smp_brdf_lut filtering
layout( binding = 2 ) uniform sampler smp_brdf_lut;

// GTAO, same convention as cube.glsl. Multiplies only the IBL ambient.
#pragma sokol @image_sample_type tex_ao float
layout( binding = 4 ) uniform texture2D tex_ao;
#pragma sokol @sampler_type smp_ao filtering
layout( binding = 3 ) uniform sampler smp_ao;

// 3x3 PCF over a 1-texel radius. Manually unrolled, see cube.glsl for
// the X3570 rationale.
float sampleShadowPCF( int cascade, vec3 light_uv, float bias )
{
	float t = 1.0 / 2048.0;
	float ly = float( cascade );
	float rz = light_uv.z - bias;
	vec2 c = light_uv.xy;
	float s = 0.0;
	s += texture( sampler2DArrayShadow( tex_shadow, smp_shadow ), vec4( c.x - t, c.y - t, ly, rz ) );
	s += texture( sampler2DArrayShadow( tex_shadow, smp_shadow ), vec4( c.x, c.y - t, ly, rz ) );
	s += texture( sampler2DArrayShadow( tex_shadow, smp_shadow ), vec4( c.x + t, c.y - t, ly, rz ) );
	s += texture( sampler2DArrayShadow( tex_shadow, smp_shadow ), vec4( c.x - t, c.y, ly, rz ) );
	s += texture( sampler2DArrayShadow( tex_shadow, smp_shadow ), vec4( c.x, c.y, ly, rz ) );
	s += texture( sampler2DArrayShadow( tex_shadow, smp_shadow ), vec4( c.x + t, c.y, ly, rz ) );
	s += texture( sampler2DArrayShadow( tex_shadow, smp_shadow ), vec4( c.x - t, c.y + t, ly, rz ) );
	s += texture( sampler2DArrayShadow( tex_shadow, smp_shadow ), vec4( c.x, c.y + t, ly, rz ) );
	s += texture( sampler2DArrayShadow( tex_shadow, smp_shadow ), vec4( c.x + t, c.y + t, ly, rz ) );
	return s / 9.0;
}

float sampleCascade( int cascade, vec3 world_pos, vec3 world_normal )
{
	vec4 light_clip = cascade_matrices[cascade] * vec4( world_pos, 1.0 );
	vec3 light_ndc = light_clip.xyz / light_clip.w;
	// Backend-dependent Y orientation, see cube.glsl for the longer note.
	float ny = light_ndc.y * cascade_far_view_z.w;
	vec3 light_uv = vec3( light_ndc.x * 0.5 + 0.5, ny * 0.5 + 0.5, light_ndc.z );

	if ( any( lessThan( light_uv, vec3( 0.0 ) ) ) || any( greaterThan( light_uv, vec3( 1.0 ) ) ) )
	{
		return 1.0;
	}

	float n_dot_l = max( dot( world_normal, normalize( sun_dir_world.xyz ) ), 0.0 );
	float bias = mix( 0.0040, 0.0008, n_dot_l );

	float pcf = sampleShadowPCF( cascade, light_uv, bias );

	// See cube.glsl for the longer note. Cascade 2 has no fallback at its
	// UV boundary, so fade PCF toward 1.0 over the outer 10% to hide the
	// polygonal seam where cascade 2's spatial extent ends on the ground.
	if ( cascade == 2 )
	{
		vec2 edge = min( light_uv.xy, 1.0 - light_uv.xy );
		float fade = clamp( min( edge.x, edge.y ) * 10.0, 0.0, 1.0 );
		pcf = mix( 1.0, pcf, fade );
	}

	return pcf;
}

float sampleShadowCascaded( vec3 world_pos, vec3 world_normal, float view_z )
{
	int cascade = 2;
	if ( view_z < cascade_far_view_z.x )
		cascade = 0;
	else if ( view_z < cascade_far_view_z.y )
		cascade = 1;

	float shadow = sampleCascade( cascade, world_pos, world_normal );

	// Blend the last 10% of each cascade's range with the next cascade
	// to hide the polygonal step where they meet. See cube.glsl for the
	// longer note.
	if ( cascade < 2 )
	{
		float far_z = ( cascade == 0 ) ? cascade_far_view_z.x : cascade_far_view_z.y;
		float near_z = ( cascade == 0 ) ? 0.0 : cascade_far_view_z.x;
		float blend_start = mix( far_z, near_z, 0.1 );
		if ( view_z > blend_start )
		{
			float alpha = ( view_z - blend_start ) / ( far_z - blend_start );
			shadow = mix( shadow, sampleCascade( cascade + 1, world_pos, world_normal ), alpha );
		}
	}

	return shadow;
}

in vec3 v_world_pos_proxy;
flat in vec3 v_center;
flat in float v_half_length;
flat in float v_radius;
flat in vec3 v_axis_x_world;
flat in vec3 v_rot_row0;
flat in vec3 v_rot_row1;
flat in vec3 v_rot_row2;
flat in vec4 v_base_color;
flat in vec4 v_material;

out vec4 out_color;

void main()
{
	// Ray construction (world space)
	vec3 ro = camera_pos.xyz;
	vec3 rd = normalize( v_world_pos_proxy - ro );

	// Capsule endpoints in world space.
	vec3 axis = v_axis_x_world;				   // already unit (rigid R)
	vec3 pa = v_center - v_half_length * axis; // -X cap center
	vec3 pb = v_center + v_half_length * axis; // +X cap center

	// Ray-capsule (cylinder body), Quilez closed form
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

	// Analytical silhouette AA. A capsule is the Minkowski sum of a segment
	// and a sphere, so distance(ray-line, capsule) = distance(ray-line,
	// segment) - radius. The closest point on the segment to the ray-line is
	// at s_inf (unconstrained) clamped to [0,1]: when s_inf lies inside the
	// segment p_cap is the cylinder's closest-axis point (perp distance =
	// cylinder silhouette), when clamped p_cap is a cap center (perp
	// distance = cap-sphere silhouette). One formula covers body + caps with
	// a single clamp. Computed in uniform control flow before any discard.
	float s_inf = ( baoa - ( b_c / max( a_c, 1e-8 ) ) * bard ) / baba;
	float s_clamp = clamp( s_inf, 0.0, 1.0 );
	vec3 p_cap = pa + s_clamp * ba;
	vec3 oc_cap = ro - p_cap;
	float bb_cap = dot( oc_cap, rd );
	float perp2 = max( 0.0, dot( oc_cap, oc_cap ) - bb_cap * bb_cap );
	float d_sil = sqrt( perp2 ) - v_radius;
	float aa = max( fwidth( d_sil ), 1e-6 );
	float coverage = 1.0 - smoothstep( -aa, aa, d_sil );

	float t = -1.0; // hit parameter, -1 means "no hit yet"
	vec3 n_world = vec3( 0.0 );

	if ( h_c >= 0.0 && a_c > 1e-8 )
	{
		float t_body = ( -b_c - sqrt( h_c ) ) / a_c;
		// Axial coord of the cylinder hit, normalized so 0..baba spans the body.
		float y = baoa + t_body * bard;
		if ( t_body > 0.0 && y > 0.0 && y < baba )
		{
			vec3 hit = ro + t_body * rd;
			// Project hit onto the cylinder centerline, normal is (hit - axis_pt)/r.
			vec3 axis_pt = pa + ba * ( y / baba );
			n_world = ( hit - axis_pt ) / v_radius;
			t = t_body;
		}
	}

	// End-cap spheres
	// Test both caps, keep the closer hit (smallest positive t).
	//
	// Cap A (-X end) at pa.
	{
		vec3 oc = ro - pa;
		float b = dot( oc, rd );
		float c = dot( oc, oc ) - v_radius * v_radius;
		float h = b * b - c;
		if ( h >= 0.0 )
		{
			float tc = -b - sqrt( h );
			if ( tc > 0.0 && ( t < 0.0 || tc < t ) )
			{
				vec3 hit = ro + tc * rd;
				// Reject the half of the cap sphere that's inside the
				// cylinder. The visible part of cap A has (hit - pa). axis <= 0.
				if ( dot( hit - pa, axis ) <= 0.0 )
				{
					n_world = ( hit - pa ) / v_radius;
					t = tc;
				}
			}
		}
	}
	// Cap B (+X end) at pb.
	{
		vec3 oc = ro - pb;
		float b = dot( oc, rd );
		float c = dot( oc, oc ) - v_radius * v_radius;
		float h = b * b - c;
		if ( h >= 0.0 )
		{
			float tc = -b - sqrt( h );
			if ( tc > 0.0 && ( t < 0.0 || tc < t ) )
			{
				vec3 hit = ro + tc * rd;
				if ( dot( hit - pb, axis ) >= 0.0 )
				{
					n_world = ( hit - pb ) / v_radius;
					t = tc;
				}
			}
		}
	}

	// Near-miss fallback: ray didn't hit the capsule, but coverage > 0 means
	// we're inside the silhouette AA band. Shade off the closest-approach
	// point on the ray to the capsule's segment, the resulting normal is
	// the silhouette normal at p_cap, which is what these sub-pixel
	// fragments should display.
	if ( t < 0.0 )
	{
		t = -bb_cap;
		vec3 closest = ro + t * rd;
		n_world = normalize( closest - p_cap );
	}

	if ( coverage <= 0.0 || t < 0.0 )
	{
		// True miss outside the AA band, or capsule behind camera.
		discard;
	}

	vec3 hit_world = ro + t * rd;

	// Object-space normal via R^T
	vec3 n_obj = vec3( v_rot_row0.x * n_world.x + v_rot_row1.x * n_world.y + v_rot_row2.x * n_world.z,
					   v_rot_row0.y * n_world.x + v_rot_row1.y * n_world.y + v_rot_row2.y * n_world.z,
					   v_rot_row0.z * n_world.x + v_rot_row1.z * n_world.y + v_rot_row2.z * n_world.z );

	// Surface pattern
	// Two great-circle rings at object-space y=0 and z=0 equators, drawn
	// uniformly on caps and body. fwidth is taken on the signed normal
	// component (abs has a derivative kink at zero that produces
	// speckled lines). Constant ~LINE_PIXELS screen-space thickness.
	float hemi = ( n_obj.y > 0.0 ) ? 1.0 : 0.65;
	vec3 base = v_base_color.rgb * hemi;

	const float LINE_PIXELS = 1.5;
	float ring_y = 1.0 - smoothstep( 0.0, fwidth( n_obj.y ) * LINE_PIXELS, abs( n_obj.y ) );
	float ring_z = 1.0 - smoothstep( 0.0, fwidth( n_obj.z ) * LINE_PIXELS, abs( n_obj.z ) );
	float line = max( ring_y, ring_z );

	vec3 line_color = base * 0.35;
	vec3 surface = mix( base, line_color, line );

	// Reverse-Z depth
	vec4 hit_clip = view_proj * vec4( hit_world, 1.0 );
	gl_FragDepth = hit_clip.z / hit_clip.w;

	// Output
	if ( flags.x == 1 )
	{
		vec3 hit_view = ( view * vec4( hit_world, 1.0 ) ).xyz;
		float view_depth = -hit_view.z;
		float v = clamp( view_depth / 50.0, 0.0, 1.0 );
		out_color = vec4( v, v, v, 1.0 );
		return;
	}
	if ( flags.x == 2 )
	{
		float view_z_dbg = -( view * vec4( hit_world, 1.0 ) ).z;
		int cascade = 2;
		if ( view_z_dbg < cascade_far_view_z.x )
			cascade = 0;
		else if ( view_z_dbg < cascade_far_view_z.y )
			cascade = 1;
		vec4 light_clip = cascade_matrices[cascade] * vec4( hit_world, 1.0 );
		vec3 light_ndc = light_clip.xyz / light_clip.w;
		float ny = light_ndc.y * cascade_far_view_z.w;
		vec3 light_uv = vec3( light_ndc.x * 0.5 + 0.5, ny * 0.5 + 0.5, light_ndc.z );
		bool oob = any( lessThan( light_uv, vec3( 0.0 ) ) ) || any( greaterThan( light_uv, vec3( 1.0 ) ) );
		vec3 tint = vec3( 1.0 );
		if ( !oob )
		{
			if ( cascade == 0 )
				tint = vec3( 1.0, 0.4, 0.4 );
			else if ( cascade == 1 )
				tint = vec3( 0.4, 1.0, 0.4 );
			else
				tint = vec3( 0.4, 0.4, 1.0 );
		}
		out_color = vec4( tint, 1.0 );
		return;
	}
	if ( flags.x == 3 )
	{
		// View-space normal as RGB, [-1,1] mapped to [0,1].
		vec3 n_view = ( view * vec4( n_world, 0.0 ) ).xyz;
		out_color = vec4( normalize( n_view ) * 0.5 + 0.5, 1.0 );
		return;
	}

	// GTAO sample.
	vec2 ao_uv = gl_FragCoord.xy / vec2( textureSize( sampler2D( tex_ao, smp_ao ), 0 ) );
	float ao = textureLod( sampler2D( tex_ao, smp_ao ), ao_uv, 0.0 ).x;

	if ( flags.x == 4 )
	{
		out_color = vec4( vec3( ao ), 1.0 );
		return;
	}

	vec3 L = normalize( sun_dir_world.xyz );
	float n_dot_l = max( dot( n_world, L ), 0.0 );
	float view_z = -( view * vec4( hit_world, 1.0 ) ).z;
	float shadow = sampleShadowCascaded( hit_world, n_world, view_z );
	vec3 V = normalize( camera_pos.xyz - hit_world );
	vec3 direct = brdf_evaluate( n_world, V, L, sun_color.rgb * shadow, surface, v_material.x, v_material.y );
	vec3 ambient = pbrEvaluateAmbient( ibl_params.y > 0.5, n_world, V, surface, v_material.x, v_material.y, sun_color.a, sh,
									   tex_ibl_spec, smp_ibl_spec, tex_brdf_lut, smp_brdf_lut, ibl_params.x );
	// premultiplied output, see cube.glsl. Alpha is scaled by analytical
	// silhouette coverage so alpha-to-coverage (opaque pipeline) or the
	// premultiplied alpha blend (transparent pipeline) produces a smooth
	// sub-pixel silhouette instead of a binary hit/miss edge.
	float a = v_base_color.a * coverage;
	out_color = vec4( ( direct + ambient * ao ) * a, a );
}
#pragma sokol @end

#pragma sokol @program impostor vs fs
