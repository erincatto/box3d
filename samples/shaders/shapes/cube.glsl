// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Cube shader, cascaded-shadow-map sampling and
// with PBR shading (GGX+Smith+Schlick+Lambert via
// common/pbr.glsl).
//
// Per-vertex input is a unit-cube corner (8 unique positions, 36 indices).
// Per-instance data lives in a readonly storage buffer indexed by
// gl_InstanceIndex: a 3x4 row-major affine transform plus a PBR material
// (baseColor + metallic + roughness). Flat shading is recovered from
// dFdx/dFdy of view-space position, no per-vertex normals are stored or
// interpolated.
//
// Lighting math runs in VIEW space (n_view, V = normalize(-v_view_pos),
// L = sun_dir_view). Converting the cube's normal computation to world
// space produced inverted normals on D3D11 (likely a screen-Y
// handedness interaction with the cross-product derivation). dot(n, L) is
// rotation-invariant, so the shadow bias formula reuses the view-space
// dot(n, L) unchanged. Shadow projection itself needs world-space coordinates,
// so we forward v_world_pos as a separate varying for the cascade matrix
// lookup.
//
// debug_view_mode (ub_pass.flags.x) selects the output:
//   0 = Lambert + ambient * shadow factor.
//   1 = view-space distance from camera as grayscale.
//   2 = cascade-index tint (one color per cascade, white = UV
//       out of bounds on the chosen cascade, i.e. unsampled).
//
// UBO bindings are stage-specific in sokol-shdc.
// VS uses ub_frame at binding=0, FS uses ub_pass at binding=1. The shadow
// depth array texture binds at texture-binding=0, the comparison sampler
// binds at sampler-binding=0.

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 b3Vec3
#pragma sokol @ctype vec2 b3Vec2

#pragma sokol @vs vs

layout( binding = 0 ) uniform ub_frame
{
	mat4 view;
	mat4 proj;
	mat4 view_proj;
	mat4 inv_view_proj;
	vec4 camera_pos; // .xyz = world pos, .w = time
	vec4 viewport;	 // .xy = size px, .zw = 1/size
};

// per-draw offset into the cube_instances buffer. Opaque draws bind
// the contiguous opaque arena and set inst_base.x = 0 (bulk-instanced).
// Transparent draws bind the transparent arena and set inst_base.x = K to
// pick the K-th transparent instance, then sg_draw with instanceCount = 1.
// The back-to-front sort across shape types is realized through these
// per-instance picks.
layout( binding = 2 ) uniform ub_draw
{
	ivec4 inst_base;
};

struct cube_instance
{
	vec4 xform_row0; // 3x4 affine, rows
	vec4 xform_row1;
	vec4 xform_row2;
	vec4 base_color; // .rgb = linear baseColor, .a = alpha
	vec4 material;	 // .x = metallic, .y = roughness,
					 // .z = shadow-cast bit (, CPU-only, ignored here),
					 // .w = reserved
};

layout( binding = 0 ) readonly buffer cube_instances
{
	cube_instance instances[];
};

in vec3 in_pos; // per-vertex (cube corner, local space)

out vec3 v_view_pos;  // for view-space normal via dFdx/dFdy
out vec3 v_world_pos; // for shadow projection (cascade matrix lookup)
flat out vec4 v_base_color;
flat out vec4 v_material;

void main()
{
	cube_instance inst = instances[inst_base.x + gl_InstanceIndex];
	vec4 local_pos = vec4( in_pos, 1.0 );
	vec3 world_pos =
		vec3( dot( inst.xform_row0, local_pos ), dot( inst.xform_row1, local_pos ), dot( inst.xform_row2, local_pos ) );
	vec4 view_pos = view * vec4( world_pos, 1.0 );
	v_view_pos = view_pos.xyz;
	v_world_pos = world_pos;
	v_base_color = inst.base_color;
	v_material = inst.material;
	gl_Position = proj * view_pos;
}
#pragma sokol @end

#pragma sokol @fs fs

#pragma sokol @include ../common/pbr.glsl
#pragma sokol @include ../common/shadow_cascade.glsl

layout( binding = 1 ) uniform ub_pass
{
	vec4 sun_dir_view;							 // .xyz = view-space dir TO sun (normalized), for direct lighting
	vec4 sun_color;								 // .rgb = color, .a = ambient strength (flat-ambient fallback)
	ivec4 flags;								 // .x = debug_view_mode
	vec4 cascade_far_view_z;					 // far view-space Z per cascade
	vec4 cascade_texel_world;					 // one shadow texel in world units per cascade
	vec4 cascade_depth_bias;					 // residual comparison bias in shadow clip-Z per cascade
	vec4 shadow_params;							 // .x = UV.y sign, .y = one texel in UV
	mat4 cascade_matrices[SHADOW_CASCADE_COUNT]; // light-space view-proj per cascade
	mat4 view;									 // IBL: world->view rotation for view->world n_view transform
	vec4 camera_pos_world;						 // IBL: world camera pos for V_world derivation
	vec4 sh[9];									 // IBL diffuse SH coefficients (band 0..2, RGB in .xyz)
	vec4 ibl_params; // .x = prefilter cube max_lod, .y = IBL enable (1=IBL, 0=flat ambient), .zw reserved
};

// binding=1 (not 0): sokol-shdc enforces disjoint binding numbers between
// storage buffers and textures across all stages of a program. The cube
// instance buffer holds binding=0 in the VS, so the shadow texture takes 1.
#pragma sokol @image_sample_type tex_shadow depth
layout( binding = 1 ) uniform texture2DArray tex_shadow;
#pragma sokol @sampler_type smp_shadow comparison
layout( binding = 0 ) uniform samplerShadow smp_shadow;

// IBL: prefiltered sky cubemap (specular) and Karis BRDF integration
// LUT. Texture bindings 2/3 and sampler bindings 1/2, disjoint from the
// shadow path's 1/0.
#pragma sokol @image_sample_type tex_ibl_spec float
layout( binding = 2 ) uniform textureCube tex_ibl_spec;
#pragma sokol @sampler_type smp_ibl_spec filtering
layout( binding = 1 ) uniform sampler smp_ibl_spec;
#pragma sokol @image_sample_type tex_brdf_lut float
layout( binding = 3 ) uniform texture2D tex_brdf_lut;
#pragma sokol @sampler_type smp_brdf_lut filtering
layout( binding = 2 ) uniform sampler smp_brdf_lut;

// GTAO: full-res R32F ambient occlusion target written by
// post/gtao_denoise. Sampled per pixel via gl_FragCoord and used to
// modulate only the IBL ambient term (direct sun light is untouched,
// per XeGTAO).
#pragma sokol @image_sample_type tex_ao float
layout( binding = 4 ) uniform texture2D tex_ao;
#pragma sokol @sampler_type smp_ao filtering
layout( binding = 3 ) uniform sampler smp_ao;

in vec3 v_view_pos;
in vec3 v_world_pos;
flat in vec4 v_base_color;
flat in vec4 v_material;
out vec4 out_color;

// The comparison sampler returns 0..1 per tap (0 = occluded, 1 = lit).
// Kernel details and the X3570 unrolling note live in the include.
#pragma sokol @include ../common/shadow_pcf.glsl

// Normal-offset shadows. A depth bias pushes the comparison back along
// the light, which detaches contact shadows and can never win at a
// silhouette where the stored depth gradient is effectively infinite.
// Moving the lookup sideways along the surface normal instead reads a
// well-behaved part of the map and leaves the compared depth alone, so
// contacts stay welded to the ground.
float sampleCascade( int cascade, vec3 world_pos, vec3 world_normal, float n_dot_l )
{
	float texel_world = cascade_texel_world[cascade];
	float depth_bias = cascade_depth_bias[cascade];

	// Shrink the kernel in the coarser cascades so the penumbra keeps a
	// constant world width instead of going crisp to blurry at every
	// boundary. The floor leaves the taps about two texels apart, close
	// enough that the hardware 2x2 compare still overlaps them.
	float scale = clamp( cascade_texel_world.x / texel_world, 0.25, 1.0 );

	// Sine of the angle between surface and light. Zero facing the light
	// where no offset is wanted, growing toward grazing. The offset has to
	// clear the whole filter, not one texel, or the outer ring of the
	// kernel self-shadows into a texel-quantized comb along the
	// terminator. 3 texels is the scaled 7x7 half-width, plus the sampling
	// snap, and the root two reaches the kernel's corner tap.
	float sin_theta = sqrt( max( 1.0 - n_dot_l * n_dot_l, 0.0 ) );
	float offset = ( 3.0 * scale + 0.5 ) * 1.41421 * sin_theta * texel_world;

	vec4 light_clip = cascade_matrices[cascade] * vec4( world_pos + world_normal * offset, 1.0 );
	vec3 light_ndc = light_clip.xyz / light_clip.w;

	// UV.y orientation is BACKEND-DEPENDENT. D3D11/Metal/WebGPU sample
	// render targets with V = 0 at the top (NDC.y = +1 -> texture row 0),
	// they need UV.y = 0.5 - NDC.y * 0.5. OpenGL samples with V = 0 at
	// the bottom (NDC.y = +1 -> top row N-1), it needs the textbook
	// UV.y = NDC.y * 0.5 + 0.5. Renderer side sets the sign to -1 on
	// D3D11/Metal/WGPU and +1 on glcore so a single multiply
	// here picks the right convention.
	float ny = light_ndc.y * shadow_params.x;
	vec3 light_uv = vec3( light_ndc.x * 0.5 + 0.5, ny * 0.5 + 0.5, light_ndc.z );

	if ( light_uv.z < 0.0 || light_uv.z > 1.0 )
	{
		return 1.0;
	}

	// No hard XY bounds test. The sampler's white border makes taps past
	// the edge compare lit, so the boundary dissolves across the kernel
	// rather than flipping whole pixels between sampled and lit as the
	// texel snap steps the map. Rejecting only what lies beyond the
	// kernel's reach is exact and spares nine taps for everything well
	// outside the cascade.
	//
	// The 3 * scale here and in the normal offset above both encode the
	// 7x7 kernel's three-texel radius. Widen the kernel and both have to
	// grow, or the outer ring self-shadows along the terminator.
	float reach = ( 3.0 * scale + 1.0 ) * shadow_params.y;
	if ( any( lessThan( light_uv.xy, vec2( -reach ) ) ) || any( greaterThan( light_uv.xy, vec2( 1.0 + reach ) ) ) )
	{
		return 1.0;
	}

	return sampleShadowPCF( tex_shadow, smp_shadow, cascade, light_uv, depth_bias, scale, shadow_params.y );
}

// n_dot_l is the (clamped) dot of the surface normal with the sun
// direction, passed in by the caller in whichever space is convenient
// (rotation-invariant, same value in view and world space). The normal
// itself must be world space, it steers the normal offset.
float sampleShadowCascaded( vec3 world_pos, vec3 world_normal, float n_dot_l, float view_z )
{
	int cascade = selectShadowCascade( cascade_far_view_z, view_z );

	float shadow = sampleCascade( cascade, world_pos, world_normal, n_dot_l );

	// Blend the last 10% of each cascade's range with the next cascade.
	// Texel density jumps by roughly 2.5x at every boundary, so even with
	// matched bias the penumbra width changes there. Without the blend
	// that reads as a polygonal step on the ground.
	if ( cascade < SHADOW_CASCADE_COUNT - 1 )
	{
		float far_z = cascade_far_view_z[cascade];
		float near_z = ( cascade == 0 ) ? 0.0 : cascade_far_view_z[cascade - 1];
		float blend_start = mix( far_z, near_z, 0.1 );
		if ( view_z > blend_start )
		{
			float alpha = ( view_z - blend_start ) / ( far_z - blend_start );
			shadow = mix( shadow, sampleCascade( cascade + 1, world_pos, world_normal, n_dot_l ), alpha );
		}
	}

	return shadow;
}

void main()
{
	// Derivatives at top level: undefined inside non-uniform branches.
	// In view space the cross-product convention works out
	// consistently across GL/D3D11/Metal because dFdx/dFdy are derived
	// from the same screen-space basis the camera uses to define view
	// space. Computing in world space instead inverts the normal on
	// D3D11 (raw screen-Y handedness leaks through the conversion).
	vec3 dx = dFdx( v_view_pos );

	// GL's dFdy points in +view_y (screen origin is lower-left), while HLSL
	// ddy and Metal dfdy point in -view_y (screen origin is upper-left).
	// sokol-shdc cross-compiles dFdy->ddy/dfdy without a sign flip, so the raw
	// value's screen direction depends on the backend. shadow_params.x
	// is +1 on glcore/gles3 and -1 on D3D11/Metal/WGPU (same flag used by
	// the shadow UV.y remap below), negating it gives a multiplier that
	// canonicalizes dFdy to the -view_y direction on every backend. With
	// canonical dy, cross(dy, dx) = (-view_y) x (+view_x) = +view_z, which
	// points toward the camera (out of the screen), the correct outward
	// normal for a back-culled front face.
	vec3 dy = dFdy( v_view_pos ) * ( -shadow_params.x );
	vec3 n_view = normalize( cross( dy, dx ) );

	float view_z = -v_view_pos.z;

	// Debug view : depth
	if ( flags.x == 1 )
	{
		float v = clamp( view_z / 50.0, 0.0, 1.0 );
		out_color = vec4( v, v, v, 1.0 );
		return;
	}

	// Debug view : shadow cascades
	if ( flags.x == 2 )
	{
		int cascade = selectShadowCascade( cascade_far_view_z, view_z );

		vec4 light_clip = cascade_matrices[cascade] * vec4( v_world_pos, 1.0 );
		vec3 light_ndc = light_clip.xyz / light_clip.w;
		float ny = light_ndc.y * shadow_params.x;
		vec3 light_uv = vec3( light_ndc.x * 0.5 + 0.5, ny * 0.5 + 0.5, light_ndc.z );
		bool oob = any( lessThan( light_uv, vec3( 0.0 ) ) ) || any( greaterThan( light_uv, vec3( 1.0 ) ) );
		vec3 tint = vec3( 1.0 );
		if ( !oob )
		{
			tint = shadowCascadeTint( cascade );
		}
		out_color = vec4( tint, 1.0 );
		return;
	}

	// Debug view : normals
	if ( flags.x == 3 )
	{
		// View-space normal as RGB, [-1,1] mapped to [0,1].
		out_color = vec4( n_view * 0.5 + 0.5, 1.0 );
		return;
	}

	// GTAO sample. textureSize avoids needing a viewport uniform in
	// ub_pass, the AO texture matches the swapchain/offscreen target
	// exactly so dividing the pixel-space gl_FragCoord by the AO size
	// yields the right UV regardless of backend Y-orientation.
	vec2 ao_uv = gl_FragCoord.xy / vec2( textureSize( sampler2D( tex_ao, smp_ao ), 0 ) );
	float ao = textureLod( sampler2D( tex_ao, smp_ao ), ao_uv, 0.0 ).x;

	// Debug view : ambient occlusion
	if ( flags.x == 4 )
	{
		out_color = vec4( vec3( ao ), 1.0 );
		return;
	}

	// The shadow normal offset and IBL both need a world-space N. view is
	// the world->view matrix, its 3x3 is orthonormal (camera rotation), so
	// transpose(mat3(view)) takes view-space -> world-space without
	// inverse cost. Stays clear of the D3D11 cross-product handedness trap
	// that forced n_view to be computed in view space. Only rotate here,
	// no derivatives.
	mat3 view_to_world = transpose( mat3( view ) );
	vec3 N_world = normalize( view_to_world * n_view );

	// dot(n, L) is rotation-invariant. Same value with view-space or world-space
	// normals + sun directions. Use the view-space pair for direct light.
	vec3 L = normalize( sun_dir_view.xyz );
	float n_dot_l = max( dot( n_view, L ), 0.0 );
	float shadow = sampleShadowCascaded( v_world_pos, N_world, n_dot_l, view_z );

	// Camera sits at origin in view space, so V_view = normalize(-v_view_pos).
	vec3 V_view = normalize( -v_view_pos );
	vec3 direct = brdf_evaluate( n_view, V_view, L, sun_color.rgb * shadow, v_base_color.rgb, v_material.x, v_material.y );

	vec3 V_world = normalize( camera_pos_world.xyz - v_world_pos );
	vec3 ambient = pbrEvaluateAmbient( ibl_params.y > 0.5, N_world, V_world, v_base_color.rgb, v_material.x, v_material.y,
									   sun_color.a, sh, tex_ibl_spec, smp_ibl_spec, tex_brdf_lut, smp_brdf_lut, ibl_params.x );

	// GTAO modulates only the ambient (IBL) term, per XeGTAO. Direct sun
	// is left alone so contact darkening doesn't fight the primary light's
	// shadowing.
	// Premultiplied output. For opaque draws (a == 1) this is bit-
	// identical to the `vec4(rgb, a)` form. For transparent draws
	// the renderer's premultiplied-alpha blend (ONE, ONE_MINUS_SRC_ALPHA)
	// consumes (rgb * a, a) directly.
	out_color = vec4( ( direct + ambient * ao ) * v_base_color.a, v_base_color.a );
}
#pragma sokol @end

#pragma sokol @program cube vs fs
