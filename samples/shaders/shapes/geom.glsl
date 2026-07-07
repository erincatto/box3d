// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Triangle-geometry shader. Drives every shape with intrinsic topology:
// convex hulls, triangle meshes, and heightfields. One pipeline. The CPU
// builders in src/box3d/box3d_geometry.c emit duplicated-vertex meshes so
// per-face normals are baked into per-vertex data (flat shading).
//
// Per-vertex data: position (vec3), normal (vec3). Per-instance data lives
// in a readonly storage buffer indexed by `inst_base.x + gl_InstanceIndex`.
// `inst_base` is a ub_draw uniform set per draw so a single shared
// instance buffer can hold contiguous slices owned by many geometries.
// (sokol_gfx's sg_draw has no first_instance argument, hence the manual
// base offset.)
//
// Per-instance affine is a 3x4: rotation rows in .xyz of each xform_row*,
// center in .w of each row. Per-instance scale is non-uniform vec3. Mesh
// instances carry b3Mesh's per-instance scale here, hulls and heightfields
// pass (1,1,1) (their hash already includes scale, so the registry entry's
// geometry is pre-scaled). Normals are transformed by R * (n / scale) so
// non-uniform scale doesn't shear the shading.
//
// @module geom prefixes the generated C symbols (geom_instance_t,
// geom_ub_frame_t, geom_triangle_shader_desc, ...) so this header coexists
// with the existing shape-shader headers in renderer.c without typedef
// collisions on the common ub_frame_t / instance_t names.

#pragma sokol @module geom

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 b3Vec3
#pragma sokol @ctype vec2 b3Vec2

#pragma sokol @vs vs

layout( binding = 0 ) uniform ub_frame
{
	mat4 view_proj;
};

layout( binding = 2 ) uniform ub_draw
{
	ivec4 inst_base; // .x = base index into the global instance array
};

struct instance
{
	vec4 xform_row0; // .xyz = rotation row 0, .w = center.x
	vec4 xform_row1; // .xyz = rotation row 1, .w = center.y
	vec4 xform_row2; // .xyz = rotation row 2, .w = center.z
	vec4 base_color; // .rgb = linear baseColor, .a = alpha
	vec4 scale;		 // .xyz = per-instance scale, .w = unused
	vec4 material;	 // .x = metallic, .y = roughness,
					 // .z = materialMode (0=solid, 1=ground grid),
					 // .w = gridCellSize (world units)
};

layout( binding = 0 ) readonly buffer instances
{
	instance items[];
};

in vec3 in_pos;
in vec3 in_normal;

out vec3 v_view_pos;	 // view-space position, for debug_view_mode=1
						 // (forwarded as world_pos, see comment in main)
out vec3 v_world_normal; // world-space normal, normalized in FS
flat out vec4 v_base_color;
flat out vec4 v_material;

void main()
{
	instance inst = items[inst_base.x + gl_InstanceIndex];

	vec3 scaled_pos = in_pos * inst.scale.xyz;
	vec4 local_pos = vec4( scaled_pos, 1.0 );
	vec3 world_pos =
		vec3( dot( inst.xform_row0, local_pos ), dot( inst.xform_row1, local_pos ), dot( inst.xform_row2, local_pos ) );

	// World normal: R * (normal / scale). Inverse-transpose for a diagonal
	// scale + rotation matrix simplifies to R applied to (normal / scale).
	// Renormalized in the FS, non-uniform scale denormalizes here.
	vec3 inv_scaled_n = in_normal / inst.scale.xyz;
	vec3 world_normal = vec3( dot( inst.xform_row0.xyz, inv_scaled_n ), dot( inst.xform_row1.xyz, inv_scaled_n ),
							  dot( inst.xform_row2.xyz, inv_scaled_n ) );

	v_world_normal = world_normal;
	v_base_color = inst.base_color;
	v_material = inst.material;

	gl_Position = view_proj * vec4( world_pos, 1.0 );
	// View matrix lives only in ub_pass on the FS side. To keep ub_frame
	// minimal (matches sphere/capsule), forward world_pos and recompute
	// view_pos in the FS from the pass-side view.
	v_view_pos = world_pos; // sentinel: FS treats as world if needed
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
	vec4 grid_offset;		  // .xy = origin wrapped to grid period (lines), .zw = full origin (axes)
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

in vec3 v_view_pos; // actually world_pos forwarded from VS
in vec3 v_world_normal;
flat in vec4 v_base_color;
flat in vec4 v_material;

out vec4 out_color;

void main()
{
	vec3 n_world = normalize( v_world_normal );

	// Geom meshes render double sided, a per-instance mirror scale can flip
	// winding, so face the shading normal toward the camera.
	if ( dot( n_world, camera_pos.xyz - v_view_pos ) < 0.0 )
	{
		n_world = -n_world;
	}

	if ( flags.x == 1 )
	{
		vec3 hit_view = ( view * vec4( v_view_pos, 1.0 ) ).xyz;
		float view_depth = -hit_view.z;
		float v = clamp( view_depth / 50.0, 0.0, 1.0 );
		out_color = vec4( v, v, v, 1.0 );
		return;
	}
	if ( flags.x == 2 )
	{
		// v_view_pos is actually world_pos forwarded from the VS.
		float view_z_dbg = -( view * vec4( v_view_pos, 1.0 ) ).z;
		int cascade = 2;
		if ( view_z_dbg < cascade_far_view_z.x )
			cascade = 0;
		else if ( view_z_dbg < cascade_far_view_z.y )
			cascade = 1;
		vec4 light_clip = cascade_matrices[cascade] * vec4( v_view_pos, 1.0 );
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

	// v_view_pos here is actually world_pos (see VS comment).
	vec3 world_pos = v_view_pos;

	// Procedural ground grid: when materialMode selects it, replace the
	// solid baseColor with the antialiased grid over world XZ. The branch
	// is on a flat per-instance varying. Within any quad of a single draw
	// every invocation takes the same path, so the fwidth inside is in
	// dynamically uniform control flow (legal for derivatives).
	//
	// The grid is sampled in world XZ, which only makes sense on horizontal
	// surfaces. On side faces of a ground box (or any tilted surface) the
	// pattern degenerates to stripes/aliasing, so we fade the grid
	// contribution toward zero as the surface tilts away from horizontal.
	// pow(n.y, 8) gives a soft falloff while leaving the top face fully
	// patterned and the strictly vertical sides flat.
	vec3 baseColor = v_base_color.rgb;
	if ( v_material.z > 0.5 )
	{
		// world_pos is in the camera relative frame. The grid lines use the
		// origin wrapped to one grid period: the pattern repeats over it, so
		// this draws the same lines but stays small enough for a float to
		// resolve far from the world origin. The axes need the true origin, so
		// they fade out far away instead of wrapping into ghost axes.
		vec2 line_xz = world_pos.xz + grid_offset.xy;
		vec2 axis_xz = world_pos.xz + grid_offset.zw;
		vec3 grid_color = proceduralGrid( line_xz, axis_xz, v_base_color.rgb, v_material.w );
		float grid_blend = pow( clamp( n_world.y, 0.0, 1.0 ), 8.0 );
		baseColor = mix( v_base_color.rgb, grid_color, grid_blend );
	}

	vec3 L = normalize( sun_dir_world.xyz );
	float n_dot_l = max( dot( n_world, L ), 0.0 );
	float view_z = -( view * vec4( world_pos, 1.0 ) ).z;
	float shadow = sampleShadowCascaded( world_pos, n_world, view_z );
	vec3 V = normalize( camera_pos.xyz - world_pos );
	vec3 direct = brdf_evaluate( n_world, V, L, sun_color.rgb * shadow, baseColor, v_material.x, v_material.y );
	vec3 ambient = pbrEvaluateAmbient( ibl_params.y > 0.5, n_world, V, baseColor, v_material.x, v_material.y, sun_color.a, sh,
									   tex_ibl_spec, smp_ibl_spec, tex_brdf_lut, smp_brdf_lut, ibl_params.x );
	// premultiplied output, see cube.glsl.
	out_color = vec4( ( direct + ambient * ao ) * v_base_color.a, v_base_color.a );
}
#pragma sokol @end

#pragma sokol @program triangle vs fs
