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
//   2 = cascade-index tint (red/green/blue = cascade 0/1/2, white = UV
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

layout( binding = 1 ) uniform ub_pass
{
	vec4 sun_dir_view;		  // .xyz = view-space dir TO sun (normalized), for direct lighting
	vec4 sun_color;			  // .rgb = color, .a = ambient strength (flat-ambient fallback)
	ivec4 flags;			  // .x = debug_view_mode
	vec4 cascade_far_view_z;  // .xyz = far view-space Z per cascade
	mat4 cascade_matrices[3]; // light-space view-proj per cascade
	mat4 view;				  // IBL: world->view rotation for view->world n_view transform
	vec4 camera_pos_world;	  // IBL: world camera pos for V_world derivation
	vec4 sh[9];				  // IBL diffuse SH coefficients (band 0..2, RGB in .xyz)
	vec4 ibl_params;		  // .x = prefilter cube max_lod, .y = IBL enable (1=IBL, 0=flat ambient), .zw reserved
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

// 3x3 PCF over a 1-texel radius. The comparison sampler returns 0..1
// per tap (0 = occluded, 1 = lit). Uniform 1/9 weighting.
//
// Manually unrolled: the HLSL compiler emits warning X3570 ("gradient
// instruction used in a loop with varying iteration") on the looped
// version because texture uses implicit derivatives. Unrolling here
// keeps cross-compilation clean on the D3D11 backend.
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

float sampleCascade( int cascade, vec3 world_pos, float n_dot_l )
{
	vec4 light_clip = cascade_matrices[cascade] * vec4( world_pos, 1.0 );
	vec3 light_ndc = light_clip.xyz / light_clip.w;

	// UV.y orientation is BACKEND-DEPENDENT. D3D11/Metal/WebGPU sample
	// render targets with V = 0 at the top (NDC.y = +1 -> texture row 0),
	// they need UV.y = 0.5 - NDC.y * 0.5. OpenGL samples with V = 0 at
	// the bottom (NDC.y = +1 -> top row N-1), it needs the textbook
	// UV.y = NDC.y * 0.5 + 0.5. Renderer side sets cascade_far_view_z.w
	// to -1 on D3D11/Metal/WGPU and +1 on glcore so a single multiply
	// here picks the right convention.
	float ny = light_ndc.y * cascade_far_view_z.w;
	vec3 light_uv = vec3( light_ndc.x * 0.5 + 0.5, ny * 0.5 + 0.5, light_ndc.z );

	if ( any( lessThan( light_uv, vec3( 0.0 ) ) ) || any( greaterThan( light_uv, vec3( 1.0 ) ) ) )
	{
		return 1.0;
	}

	// Slope-scaled bias: tan(angle to light) = sqrt(1 - n.l^2) / n.l.
	// Constant texel-step factor keeps the bias roughly the same world
	// size across cascades (each cascade's depth range scales with its
	// bounding sphere, bias scales with it via the texel step).
	float ndl = clamp( n_dot_l, 0.001, 1.0 );
	float tanAngle = sqrt( 1.0 - ndl * ndl ) / ndl;
	float texelDepthStep = 0.001;
	float bias = clamp( texelDepthStep * tanAngle, 0.0005, 0.05 );

	float pcf = sampleShadowPCF( cascade, light_uv, bias );

	// Cascade 2 is the last cascade, sampleShadowCascaded's view-z blend
	// has nothing to blend into beyond it. On flat ground at glancing sun
	// angles PCF inside the cascade reads slightly below 1.0 (bias slop
	// across a 3x3 kernel that covers ~5 cm of world per texel here), while
	// fragments outside the cascade's spatial sphere fall through to the
	// OOB return of 1.0. Without smoothing, cascade 2's UV boundary reads
	// as a hexagonal polygon on the ground. Fade PCF toward 1.0 over the
	// outer 10% of UV space to hide the seam.
	if ( cascade == 2 )
	{
		vec2 edge = min( light_uv.xy, 1.0 - light_uv.xy );
		float fade = clamp( min( edge.x, edge.y ) * 10.0, 0.0, 1.0 );
		pcf = mix( 1.0, pcf, fade );
	}

	return pcf;
}

// n_dot_l is the (clamped) dot of the surface normal with the sun
// direction, passed in by the caller in whichever space is convenient
// (rotation-invariant, same value in view and world space).
float sampleShadowCascaded( vec3 world_pos, float n_dot_l, float view_z )
{
	int cascade = 2;
	if ( view_z < cascade_far_view_z.x )
	{
		cascade = 0;
	}
	else if ( view_z < cascade_far_view_z.y )
	{
		cascade = 1;
	}

	float shadow = sampleCascade( cascade, world_pos, n_dot_l );

	// Blend the last 10% of each cascade's range with the next cascade.
	// Each cascade uses the same slope-scaled bias in shadow-UV units,
	// but cascades cover very different world extents (cascade 0 ~= 1 m,
	// cascade 2 ~= 25 m+), so PCF response on flat receivers differs
	// slightly per cascade, without this blend the boundary reads as a
	// polygonal step on the ground at glancing sun angles.
	if ( cascade < 2 )
	{
		float far_z = ( cascade == 0 ) ? cascade_far_view_z.x : cascade_far_view_z.y;
		float near_z = ( cascade == 0 ) ? 0.0 : cascade_far_view_z.x;
		float blend_start = mix( far_z, near_z, 0.1 );
		if ( view_z > blend_start )
		{
			float alpha = ( view_z - blend_start ) / ( far_z - blend_start );
			shadow = mix( shadow, sampleCascade( cascade + 1, world_pos, n_dot_l ), alpha );
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
	// value's screen direction depends on the backend. cascade_far_view_z.w
	// is +1 on glcore/gles3 and -1 on D3D11/Metal/WGPU (same flag used by
	// the shadow UV.y remap below), negating it gives a multiplier that
	// canonicalizes dFdy to the -view_y direction on every backend. With
	// canonical dy, cross(dy, dx) = (-view_y) x (+view_x) = +view_z, which
	// points toward the camera (out of the screen), the correct outward
	// normal for a back-culled front face.
	vec3 dy = dFdy( v_view_pos ) * ( -cascade_far_view_z.w );
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
		int cascade = 2;
		if ( view_z < cascade_far_view_z.x )
		{
			cascade = 0;
		}
		else if ( view_z < cascade_far_view_z.y )
		{
			cascade = 1;
		}

		vec4 light_clip = cascade_matrices[cascade] * vec4( v_world_pos, 1.0 );
		vec3 light_ndc = light_clip.xyz / light_clip.w;
		float ny = light_ndc.y * cascade_far_view_z.w;
		vec3 light_uv = vec3( light_ndc.x * 0.5 + 0.5, ny * 0.5 + 0.5, light_ndc.z );
		bool oob = any( lessThan( light_uv, vec3( 0.0 ) ) ) || any( greaterThan( light_uv, vec3( 1.0 ) ) );
		vec3 tint = vec3( 1.0 );
		if ( !oob )
		{
			if ( cascade == 0 )
			{
				tint = vec3( 1.0, 0.4, 0.4 );
			}
			else if ( cascade == 1 )
			{
				tint = vec3( 0.4, 1.0, 0.4 );
			}
			else
			{
				tint = vec3( 0.4, 0.4, 1.0 );
			}
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

	// dot(n, L) is rotation-invariant. Same value with view-space or world-space
	// normals + sun directions. Use the view-space pair for direct light.
	vec3 L = normalize( sun_dir_view.xyz );
	float n_dot_l = max( dot( n_view, L ), 0.0 );
	float shadow = sampleShadowCascaded( v_world_pos, n_dot_l, view_z );

	// Camera sits at origin in view space, so V_view = normalize(-v_view_pos).
	vec3 V_view = normalize( -v_view_pos );
	vec3 direct = brdf_evaluate( n_view, V_view, L, sun_color.rgb * shadow, v_base_color.rgb, v_material.x, v_material.y );

	// IBL needs world-space N and V. view is the world->view matrix, its
	// 3x3 is orthonormal (camera rotation), so transpose(mat3(view))
	// takes view-space -> world-space without inverse cost. Stays clear
	// of the D3D11 cross-product handedness trap that forced n_view to
	// be computed in view space. Only rotate here, no derivatives.
	mat3 view_to_world = transpose( mat3( view ) );
	vec3 N_world = normalize( view_to_world * n_view );
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
