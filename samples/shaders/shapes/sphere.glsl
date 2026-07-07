// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Sphere impostor.
//
// Per-instance: a 3x4 rigid affine (orthonormal rotation rows in .xyz of each
// xform_row*, sphere center in .w of each row), an RGBA color, and a `params`
// row whose .x is the world-space radius. Rotation does not change the
// silhouette (a sphere is rotation-invariant) so the bounding-cube proxy is
// emitted axis-aligned. Rotation feeds the surface pattern in the FS so spin
// is visible.
//
// Pipeline overview:
//   VS: emit a unit-cube vertex scaled by 2*radius and translated by center.
//       Pass world position of the proxy vertex (interpolated) plus flat
//       per-instance data (center, radius, rotation rows, color) to FS.
//   FS: reconstruct view ray as (camera, normalize(v_world_pos_proxy - camera)).
//       Analytic ray-sphere intersection. discard on miss / behind-camera.
//       Compute world-space normal at the hit, transform to object space via
//       transpose(rotation), shade three great-circle rings at the
//       object-space x=0, y=0, z=0 equators (constant screen-space pixel
//       thickness via fwidth) over a north/south hemisphere base contrast,
//       apply Lambert + ambient against world-space sun, write reverse-Z
//       gl_FragDepth.
//
// `@module sphere` prefixes the generated C symbols (sphere_ub_frame_t,
// sphere_instance_t, sphere_shader_desc, ...) so this header coexists with
// cube.glsl.h in renderer.c without typedef collisions on ub_frame_t.
//
// UBO layout. UBOs are per-stage in sokol-shdc:
//   VS binding=0 ub_frame: just view_proj. That's all the VS needs to emit
//                          the proxy in clip space.
//   FS binding=1 ub_pass:  sun + debug flags + camera_pos + view_proj + view.
//                          The frame-level matrices live here (rather than in
//                          a second FS UBO at a different binding) because
//                          the renderer always re-applies ub_pass per draw,
//                          and one FS UBO is simpler than two. Naming is a
//                          slight stretch (some fields are per-frame, not
//                          per-pass) but correctness-wise irrelevant in a
//                          single-pass renderer.
//
// gl_FragDepth disables hi-Z. That's accepted for impostors.

#pragma sokol @module sphere

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
// rationale. Opaque draws set inst_base.x = 0 + bulk-instance, transparent
// draws pick one instance at a time.
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
	vec4 params;	 // .x = radius, .yzw = unused
	vec4 material;	 // .x = metallic, .y = roughness,
					 // .z = shadow-cast bit (, CPU-only, ignored here),
					 // .w = reserved
};

layout( binding = 0 ) readonly buffer instances
{
	instance items[];
};

in vec3 in_pos; // unit-cube corner, half-extent 0.5

// Interpolated: any point along the camera-to-pixel ray works as a far
// reference, we re-normalize in FS. Per-pixel (not per-sample): the FS
// computes analytical sub-pixel coverage from the signed silhouette
// distance and feeds it through alpha-to-coverage, which replaces the
// per-sample ray test the `sample` qualifier used to do.
out vec3 v_world_pos_proxy;

// Flat: same value for every fragment in this instance.
flat out vec3 v_center;
flat out float v_radius;
flat out vec3 v_rot_row0;
flat out vec3 v_rot_row1;
flat out vec3 v_rot_row2;
flat out vec4 v_base_color;
flat out vec4 v_material;

void main()
{
	instance inst = items[inst_base.x + gl_InstanceIndex];

	// sphere center in world space
	vec3 center = vec3( inst.xform_row0.w, inst.xform_row1.w, inst.xform_row2.w );
	float radius = inst.params.x;

	// Local in_pos has half-extent 0.5, multiply by 2*radius to get a cube
	// half-extent radius exactly enclosing the sphere.
	// This creates an AABB in world space
	vec3 world_pos = center + ( 2.0 * radius ) * in_pos;

	v_world_pos_proxy = world_pos;
	v_center = center;
	v_radius = radius;
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
	vec4 cascade_pcf_scale;	  // .xyz = PCF tap-offset scale per cascade (constant world penumbra)
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

#pragma sokol @include ../common/shadow_pcf.glsl

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

	float pcf = sampleShadowPCF( tex_shadow, smp_shadow, cascade, light_uv, bias, cascade_pcf_scale[cascade] );

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

// sphere center in world space
flat in vec3 v_center;
flat in float v_radius;
flat in vec3 v_rot_row0;
flat in vec3 v_rot_row1;
flat in vec3 v_rot_row2;
flat in vec4 v_base_color;
flat in vec4 v_material;

out vec4 out_color;

void main()
{
	// Ray construction (world space)
	// ray origin
	vec3 ro = camera_pos.xyz;

	// ray direction
	vec3 rd = normalize( v_world_pos_proxy - ro );

	// Ray-sphere intersection
	// |ro + t*rd - c|^2 = r^2 => t^2 + 2*b*t + c2 = 0
	vec3 oc = ro - v_center;
	float b = dot( oc, rd );
	float c2 = dot( oc, oc ) - v_radius * v_radius;
	float h = b * b - c2;

	// Analytical silhouette AA. `d` is the signed perpendicular distance from
	// the ray-line to the sphere surface (in world units): negative when the
	// ray hits, positive when it misses by `d`. fwidth(d) gives world-units
	// per pixel near the silhouette, which becomes the AA half-width. See
	// https://iquilezles.org/articles/spherefunctions . Computed in uniform
	// control flow before any discard so derivatives are defined.
	float d = sqrt( max( 0.0, v_radius * v_radius - h ) ) - v_radius;
	float aa = max( fwidth( d ), 1e-6 );
	float coverage = 1.0 - smoothstep( -aa, aa, d );

	// For hits use the near intersection. For near-misses (h<0) the ray
	// doesn't actually touch the sphere, use the closest-approach distance
	// -b instead, and shade from the closest-approach point. The resulting
	// normal (direction from center to closest-approach) is the silhouette
	// normal, which is what these sub-pixel fragments should display anyway.
	float t = ( h >= 0.0 ) ? ( -b - sqrt( h ) ) : -b;
	if ( coverage <= 0.0 || t < 0.0 )
	{
		// True miss outside the AA band, or sphere behind camera, or camera
		// inside the sphere.
		discard;
	}

	vec3 hit_world = ro + t * rd;
	vec3 n_world = normalize( hit_world - v_center );

	// Object-space normal via R^T
	// n_obj = R^T * n_world. R^T's rows are the columns of R. The .x
	// components of v_rot_row0/1/2 form row 0 of R^T, .y components form row
	// 1, .z components form row 2.
	vec3 n_obj = vec3( v_rot_row0.x * n_world.x + v_rot_row1.x * n_world.y + v_rot_row2.x * n_world.z,
					   v_rot_row0.y * n_world.x + v_rot_row1.y * n_world.y + v_rot_row2.y * n_world.z,
					   v_rot_row0.z * n_world.x + v_rot_row1.z * n_world.y + v_rot_row2.z * n_world.z );

	// Surface pattern
	// Hemisphere split: north 1.0, south 0.65, base shading contrast.
	float hemi = ( n_obj.y > 0.0 ) ? 1.0 : 0.65;

	// Three great-circle rings at the equators of object-space X, Y, Z.
	// Each equator is the locus where the corresponding object-space normal
	// component is zero, so the distance-to-equator is just the absolute
	// value of that component (a chord-distance metric in [0, 1] that's
	// monotonic in the geodesic distance from the great circle).
	//
	// Constant screen-space thickness via per-component fwidth: each ring's
	// band scales with the per-pixel rate of change of *its own* normal
	// component, so it stays ~LINE_PIXELS wide independent of camera zoom
	// and viewing angle. fwidth is taken on the *signed* component (not on
	// abs(.)): abs has a derivative kink at zero (the equator itself),
	// so fwidth(abs(x)) gives garbage on 2x2 quads that straddle the
	// equator. Don't use max of all three fwidths as the shared band:
	// near the silhouette of one ring, the *other* two components change
	// huge amounts per pixel (the surface is edge-on), and a shared band
	// would over-fatten this ring.
	const float LINE_PIXELS = 1.5;
	float ring_x = 1.0 - smoothstep( 0.0, fwidth( n_obj.x ) * LINE_PIXELS, abs( n_obj.x ) );
	float ring_y = 1.0 - smoothstep( 0.0, fwidth( n_obj.y ) * LINE_PIXELS, abs( n_obj.y ) );
	float ring_z = 1.0 - smoothstep( 0.0, fwidth( n_obj.z ) * LINE_PIXELS, abs( n_obj.z ) );
	float line = max( ring_x, max( ring_y, ring_z ) );

	vec3 base = v_base_color.rgb * hemi;
	vec3 line_color = base * 0.35;
	vec3 surface = mix( base, line_color, line );

	// Reverse-Z depth
	vec4 hit_clip = view_proj * vec4( hit_world, 1.0 );
	gl_FragDepth = hit_clip.z / hit_clip.w;

	// Output
	if ( flags.x == 1 )
	{
		// view-space depth grayscale, matches cube debug mode.
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
