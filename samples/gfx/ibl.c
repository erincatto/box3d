// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/ibl.h"

#include "brdf_lut.glsl.h"
#include "gfx/sky.h"
#include "prefilter.glsl.h"
#include "sky_to_cube.glsl.h"

#include <math.h>

// Per-face basis vectors that map a fragment's v_uv in [0,1]^2 to a
// world-space direction. Match the GL cubemap sampling convention (which
// sokol abstracts uniformly across backends), so a direction d written
// at (face F, uv) is recovered by sampling the cubemap with the same d.
// Order: +X, -X, +Y, -Y, +Z, -Z.
typedef struct
{
	b3Vec3 right;
	b3Vec3 up;
	b3Vec3 forward;
} CubeFaceBasis;

// SH projection grid resolution per face. 32 -> 6 * 32^2 = 6144 sample
// evaluations per rebuild. Cheap on CPU and well above the SH order-2
// Nyquist for the smooth Preetham function.
#define IBL_SH_SAMPLES_PER_FACE 32

// Funk-Hecke / Ramamoorthi diffuse convolution weights, pre-multiplied
// into the stored SH coefficients so the shader reads them as
// E(N) = sum(coef[i] * basis_i(N)), L_diffuse = (albedo / pi) * E(N).
// A_0 = pi, A_1 = 2pi/3, A_2 = pi/4 (Ramamoorthi & Hanrahan 2001, eq.5).
#define SH_A0 ( 3.14159265358979323846f )
#define SH_A1 ( 3.14159265358979323846f * 2.0f / 3.0f )
#define SH_A2 ( 3.14159265358979323846f / 4.0f )

// SH-basis normalization constants (real spherical harmonics, l=0..2).
// Order matches Sloan / Ramamoorthi: index 0 = (l=0,m=0), 1..3 = l=1
// with m = -1,0,+1, 4..8 = l=2 with m = -2,-1,0,+1,+2.
#define SH_Y00 0.282094791773878f  // 0.5 / sqrt(pi)
#define SH_Y1n1 0.488602511902919f // sqrt(3/(4pi))
#define SH_Y10 0.488602511902919f
#define SH_Y1p1 0.488602511902919f
#define SH_Y2n2 1.092548430592079f // sqrt(15/(4pi))
#define SH_Y2n1 1.092548430592079f
#define SH_Y20 0.315391565252520f // sqrt(5/(16pi))
#define SH_Y2p1 1.092548430592079f
#define SH_Y2p2 0.546274215296039f // sqrt(15/(16pi)) = SH_Y2n2/2

static const CubeFaceBasis s_cubeFaceBasis[6] = {
	// +X
	{ { 0.0f, 0.0f, -1.0f }, { 0.0f, -1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } },
	// -X
	{ { 0.0f, 0.0f, 1.0f }, { 0.0f, -1.0f, 0.0f }, { -1.0f, 0.0f, 0.0f } },
	// +Y
	{ { 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f } },
	// -Y
	{ { 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, -1.0f }, { 0.0f, -1.0f, 0.0f } },
	// +Z
	{ { 1.0f, 0.0f, 0.0f }, { 0.0f, -1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } },
	// -Z
	{ { -1.0f, 0.0f, 0.0f }, { 0.0f, -1.0f, 0.0f }, { 0.0f, 0.0f, -1.0f } },
};

static struct
{
	// BRDF integration LUT (Karis split-sum). RG16F at GFX_IBL_LUT_SIZE^2.
	// Generated once at InitImageBasedLighting. Environment-independent so it never
	// needs to be rebuilt.
	sg_image brdfLut;
	sg_view brdfLutColorAttachmentView; // used during generation
	sg_view brdfLutSampleView;			// bound by lit pipelines
	sg_sampler brdfLutSampler;
	sg_shader brdfLutShader;
	sg_pipeline brdfLutPipeline;

	// Raw Preetham sky cube. 1 mip, 6 faces. Filled by sky_to_cube. Read
	// by the prefilter pass as the input cube. Not bound to lit pipelines.
	sg_image skyCubeRaw;
	sg_view skyCubeRawFaceAttachmentViews[6];
	sg_view skyCubeRawSampleView;
	sg_shader skyToCubeShader;
	sg_pipeline skyToCubePipeline;

	// GGX-prefiltered sky cube. GFX_IBL_CUBE_MIPS mips x 6 faces. Mip 0
	// is the mirror (effectively a copy of skyCubeRaw). Deeper mips are
	// progressively rougher. This is what the lit pipelines sample.
	sg_image skyCube;
	sg_view skyCubeFaceMipAttachmentViews[6 * GFX_IBL_CUBE_MIPS];
	sg_view skyCubeSampleView;
	sg_sampler skyCubeSampler; // trilinear, clamp_to_edge
	sg_shader prefilterShader;
	sg_pipeline prefilterPipeline;

	// SH cache. Filled by RebuildImageBasedLightingIfDirty.
	b3Vec3 shCoeffs[9];

	// Dirty flag. Set true by MarkIblDirty + initial state. Cleared
	// by RebuildImageBasedLightingIfDirty after a successful rebuild.
	bool dirty;
	bool ready;

	// Up axis the cube and SH were last baked for. A change reorients the
	// sky, so it forces a rebuild even when sun and turbidity hold.
	bool zUp;
} s_ibl;

// CPU Preetham (mirror of common/preetham.glsl)
//
// Keeps SH projection in lockstep with the GPU sky. The math is verbatim
// from the shader. Comments are deliberately spare to avoid two sources
// of truth, read preetham.glsl for the rationale.

// Rotate a sky direction into the model's Y-up frame, mirror of
// preethamToYUp in common/preetham.glsl.
static b3Vec3 SkyDirToYUpC( b3Vec3 v, bool zUp )
{
	return zUp ? ( b3Vec3 ){ v.x, v.z, -v.y } : v;
}

static float PreethamPerezC( float cos_theta, float cos_gamma, float gamma, float A, float B, float C, float D, float E )
{
	const float ct = ( cos_theta < 0.01f ) ? 0.01f : cos_theta;
	const float term1 = 1.0f + A * expf( B / ct );
	const float term2 = 1.0f + C * expf( D * gamma ) + E * cos_gamma * cos_gamma;
	return term1 * term2;
}

static b3Vec3 PreethamSkyScaledC( b3Vec3 view_dir, b3Vec3 sun_dir, float turbidity, float fade, bool zUp )
{
	const float PI = 3.14159265358979323846f;
	const float LUMINANCE_SCALE = 0.06f; // mirrors PREETHAM_LUMINANCE_SCALE

	view_dir = SkyDirToYUpC( view_dir, zUp );
	sun_dir = SkyDirToYUpC( sun_dir, zUp );

	const float sun_y_clamped = ( sun_dir.y < 0.0f ) ? 0.0f : ( sun_dir.y > 1.0f ) ? 1.0f : sun_dir.y;

	b3Vec3 view_clamped;
	view_clamped.x = view_dir.x;
	view_clamped.y = ( view_dir.y > 0.0f ? view_dir.y : 0.0f ) + 0.01f;
	view_clamped.z = view_dir.z;
	const float vlen =
		sqrtf( view_clamped.x * view_clamped.x + view_clamped.y * view_clamped.y + view_clamped.z * view_clamped.z );
	if ( vlen > 0.0f )
	{
		view_clamped.x /= vlen;
		view_clamped.y /= vlen;
		view_clamped.z /= vlen;
	}

	const float cos_theta = ( view_clamped.y > 0.0f ) ? view_clamped.y : 0.0f;
	float cos_gamma = sun_dir.x * view_clamped.x + sun_dir.y * view_clamped.y + sun_dir.z * view_clamped.z;
	if ( cos_gamma < -1.0f )
		cos_gamma = -1.0f;
	if ( cos_gamma > 1.0f )
		cos_gamma = 1.0f;
	const float gamma = acosf( cos_gamma );
	const float sun_theta = acosf( sun_y_clamped );

	const float T = ( turbidity < 1.0f ) ? 1.0f : turbidity;
	const float T2 = T * T;

	const float A_Y = 0.1787f * T - 1.4630f;
	const float B_Y = -0.3554f * T + 0.4275f;
	const float C_Y = -0.0227f * T + 5.3251f;
	const float D_Y = 0.1206f * T - 2.5771f;
	const float E_Y = -0.0670f * T + 0.3703f;
	const float A_x = -0.0193f * T - 0.2592f;
	const float B_x = -0.0665f * T + 0.0008f;
	const float C_x = -0.0004f * T + 0.2125f;
	const float D_x = -0.0641f * T - 0.8989f;
	const float E_x = -0.0033f * T + 0.0452f;
	const float A_y = -0.0167f * T - 0.2608f;
	const float B_y = -0.0950f * T + 0.0092f;
	const float C_y = -0.0079f * T + 0.2102f;
	const float D_y = -0.0441f * T - 1.6537f;
	const float E_y = -0.0109f * T + 0.0529f;

	const float ts = sun_theta;
	const float ts2 = ts * ts;
	const float ts3 = ts2 * ts;

	const float x_z = ( 0.00166f * ts3 - 0.00375f * ts2 + 0.00209f * ts ) * T2 +
					  ( -0.02903f * ts3 + 0.06377f * ts2 - 0.03202f * ts + 0.00394f ) * T +
					  ( 0.11693f * ts3 - 0.21196f * ts2 + 0.06052f * ts + 0.25886f );
	const float y_z = ( 0.00275f * ts3 - 0.00610f * ts2 + 0.00317f * ts ) * T2 +
					  ( -0.04214f * ts3 + 0.08970f * ts2 - 0.04153f * ts + 0.00516f ) * T +
					  ( 0.15346f * ts3 - 0.26756f * ts2 + 0.06670f * ts + 0.26688f );

	const float chi = ( 4.0f / 9.0f - T / 120.0f ) * ( PI - 2.0f * sun_theta );
	const float Y_z = ( 4.0453f * T - 4.9710f ) * tanf( chi ) - 0.2155f * T + 2.4192f;

	const float cos_zen_gamma = sun_y_clamped;
	const float pY_v = PreethamPerezC( cos_theta, cos_gamma, gamma, A_Y, B_Y, C_Y, D_Y, E_Y );
	const float px_v = PreethamPerezC( cos_theta, cos_gamma, gamma, A_x, B_x, C_x, D_x, E_x );
	const float py_v = PreethamPerezC( cos_theta, cos_gamma, gamma, A_y, B_y, C_y, D_y, E_y );
	const float pY_z = PreethamPerezC( 1.0f, cos_zen_gamma, sun_theta, A_Y, B_Y, C_Y, D_Y, E_Y );
	const float px_z = PreethamPerezC( 1.0f, cos_zen_gamma, sun_theta, A_x, B_x, C_x, D_x, E_x );
	const float py_z = PreethamPerezC( 1.0f, cos_zen_gamma, sun_theta, A_y, B_y, C_y, D_y, E_y );

	const float pY_z_safe = ( pY_z < 1.0e-5f ) ? 1.0e-5f : pY_z;
	const float px_z_safe = ( px_z < 1.0e-5f ) ? 1.0e-5f : px_z;
	const float py_z_safe = ( py_z < 1.0e-5f ) ? 1.0e-5f : py_z;

	float Y = Y_z * pY_v / pY_z_safe;
	if ( Y < 0.0f )
		Y = 0.0f;
	const float x = x_z * px_v / px_z_safe;
	const float y = y_z * py_v / py_z_safe;

	const float yy = ( y < 1.0e-5f ) ? 1.0e-5f : y;
	const float X_xyz = Y * x / yy;
	const float Y_xyz = Y;
	const float Z_xyz = Y * ( 1.0f - x - y ) / yy;

	b3Vec3 rgb;
	rgb.x = ( X_xyz * 3.2404542f ) + ( Y_xyz * -1.5371385f ) + ( Z_xyz * -0.4985314f );
	rgb.y = ( X_xyz * -0.9692660f ) + ( Y_xyz * 1.8760108f ) + ( Z_xyz * 0.0415560f );
	rgb.z = ( X_xyz * 0.0556434f ) + ( Y_xyz * -0.2040259f ) + ( Z_xyz * 1.0572252f );
	if ( rgb.x < 0.0f )
		rgb.x = 0.0f;
	if ( rgb.y < 0.0f )
		rgb.y = 0.0f;
	if ( rgb.z < 0.0f )
		rgb.z = 0.0f;
	rgb.x *= LUMINANCE_SCALE * fade;
	rgb.y *= LUMINANCE_SCALE * fade;
	rgb.z *= LUMINANCE_SCALE * fade;
	return rgb;
}

// Cube-texel solid-angle approximation. Closed form:
// domega = 4 / ((u^2 + v^2 + 1)^1.5 * N^2)
// where (u, v) in [-1, +1] is the texel-center face coordinate and N is
// the face resolution. Sufficient at the 32^2 SH sampling grid.
static float cubeTexelSolidAngle( float u, float v, int n )
{
	const float d2 = u * u + v * v + 1.0f;
	const float denom = sqrtf( d2 * d2 * d2 ) * (float)n * (float)n;
	return 4.0f / denom;
}

// Project the procedural sky into 9 RGB SH coefficients (band 0..2) by
// evaluating closed-form Preetham at 32^2 stratified samples per face,
// accumulating sum L(omega) * Y_i(omega) * domega, then multiplying by the Funk-Hecke
// diffuse convolution weights so the shader's evaluation is a straight
// dot product against the SH basis.
static void ProjectSkyToSh( b3Vec3 dirToSun, float turbidity, float fade, bool zUp )
{
	b3Vec3 coef[9];
	for ( int i = 0; i < 9; ++i )
	{
		coef[i] = b3Vec3_zero;
	}

	const int N = IBL_SH_SAMPLES_PER_FACE;
	const float pixSize = 2.0f / (float)N;

	for ( int face = 0; face < 6; ++face )
	{
		const CubeFaceBasis b = s_cubeFaceBasis[face];
		for ( int row = 0; row < N; ++row )
		{
			for ( int col = 0; col < N; ++col )
			{
				const float u = ( (float)col + 0.5f ) * pixSize - 1.0f;
				const float v = ( (float)row + 0.5f ) * pixSize - 1.0f;

				b3Vec3 dir;
				dir.x = b.forward.x + u * b.right.x + v * b.up.x;
				dir.y = b.forward.y + u * b.right.y + v * b.up.y;
				dir.z = b.forward.z + u * b.right.z + v * b.up.z;
				const float dlen = sqrtf( dir.x * dir.x + dir.y * dir.y + dir.z * dir.z );
				if ( dlen > 0.0f )
				{
					dir.x /= dlen;
					dir.y /= dlen;
					dir.z /= dlen;
				}

				const float dW = cubeTexelSolidAngle( u, v, N );
				// dir stays in sim space, it indexes the SH basis below which
				// the lit shader evaluates at a sim-space normal. Only the
				// Preetham color lookup rotates into the model's Y-up frame.
				const b3Vec3 L = PreethamSkyScaledC( dir, dirToSun, turbidity, fade, zUp );

				const float x = dir.x;
				const float y = dir.y;
				const float z = dir.z;

				const float basis[9] = {
					SH_Y00,
					-SH_Y1n1 * y,
					SH_Y10 * z,
					-SH_Y1p1 * x,
					SH_Y2n2 * x * y,
					-SH_Y2n1 * y * z,
					SH_Y20 * ( 3.0f * z * z - 1.0f ),
					-SH_Y2p1 * x * z,
					SH_Y2p2 * ( x * x - y * y ),
				};

				for ( int i = 0; i < 9; ++i )
				{
					const float w = basis[i] * dW;
					coef[i].x += L.x * w;
					coef[i].y += L.y * w;
					coef[i].z += L.z * w;
				}
			}
		}
	}

	// Apply Funk-Hecke / Ramamoorthi convolution weights so the shader
	// can evaluate irradiance as a straight dot product.
	const float scale[9] = {
		SH_A0, SH_A1, SH_A1, SH_A1, SH_A2, SH_A2, SH_A2, SH_A2, SH_A2,
	};
	for ( int i = 0; i < 9; ++i )
	{
		s_ibl.shCoeffs[i].x = coef[i].x * scale[i];
		s_ibl.shCoeffs[i].y = coef[i].y * scale[i];
		s_ibl.shCoeffs[i].z = coef[i].z * scale[i];
	}
}

static void GenerateBrdfLut( void )
{
	// One-shot pass into the LUT's color attachment. Discard load (we
	// overwrite every pixel), no clear necessary.
	sg_pass pass = { 0 };
	pass.action.colors[0].load_action = SG_LOADACTION_DONTCARE;
	pass.action.colors[0].store_action = SG_STOREACTION_STORE;
	pass.attachments.colors[0] = s_ibl.brdfLutColorAttachmentView;

	sg_push_debug_group( "ibl_brdf_lut" );
	sg_begin_pass( &pass );
	sg_apply_pipeline( s_ibl.brdfLutPipeline );
	// params.x: backend-conditional UV.y sign for the LUT's render-target
	// V orientation, see brdf_lut.glsl. +1 on GL (NDC.y=+1 stores at the
	// texel addressed by sampling V=1), -1 on D3D11/Metal/WGPU (NDC.y=+1
	// stores at the texel addressed by sampling V=0). Without this, the
	// lit shaders' sample at (n_dot_v, roughness) reads the LUT for
	// (n_dot_v, 1-roughness) on non-GL backends, metals look dark and
	// rough materials look mirror-smooth.
	const sg_backend backend = sg_query_backend();
	const float uvYSign = ( backend == SG_BACKEND_GLCORE || backend == SG_BACKEND_GLES3 ) ? 1.0f : -1.0f;
	brdf_lut_ub_brdf_t ub = { 0 };
	ub.params = MakeVec4( uvYSign, 0.0f, 0.0f, 0.0f );
	sg_apply_uniforms( UB_brdf_lut_ub_brdf, &SG_RANGE( ub ) );
	// No texture bindings, sokol's VALIDATE_ABND_EMPTY_BINDINGS rejects
	// an empty sg_apply_bindings call, so skip it entirely.
	sg_draw( 0, 3, 1 );
	sg_end_pass();
	sg_pop_debug_group();
}

static void GenerateSkyCube( b3Vec3 dirToSun, float turbidity, float fade, bool zUp )
{
	// Six passes, one per face. Each pass fills the raw cube's
	// corresponding face via a fullscreen-triangle Preetham eval.
	sg_push_debug_group( "ibl_sky_to_cube" );
	for ( int face = 0; face < 6; ++face )
	{
		sg_pass pass = { 0 };
		pass.action.colors[0].load_action = SG_LOADACTION_DONTCARE;
		pass.action.colors[0].store_action = SG_STOREACTION_STORE;
		pass.attachments.colors[0] = s_ibl.skyCubeRawFaceAttachmentViews[face];

		sg_begin_pass( &pass );
		sg_apply_pipeline( s_ibl.skyToCubePipeline );

		const CubeFaceBasis b = s_cubeFaceBasis[face];
		sky_to_cube_ub_face_t up = { 0 };
		up.face_right = MakeVec4( b.right.x, b.right.y, b.right.z, 0.0f );
		up.face_up = MakeVec4( b.up.x, b.up.y, b.up.z, zUp ? 1.0f : 0.0f );
		up.face_forward = MakeVec4( b.forward.x, b.forward.y, b.forward.z, fade );
		up.sun_dir_world = MakeVec4( dirToSun.x, dirToSun.y, dirToSun.z, turbidity );
		sg_apply_uniforms( UB_sky_to_cube_ub_face, &SG_RANGE( up ) );

		sg_draw( 0, 3, 1 );
		sg_end_pass();
	}
	sg_pop_debug_group();
}

static void GeneratePrefilter( void )
{
	// For each mip x face, importance-sample the raw cube into the
	// prefiltered cube's corresponding face slice. Roughness maps
	// linearly across mips: mip 0 = 0 (mirror), mip N-1 = 1 (max rough).
	sg_push_debug_group( "ibl_prefilter" );
	const float mipDenom = (float)( GFX_IBL_CUBE_MIPS - 1 );

	for ( int mip = 0; mip < GFX_IBL_CUBE_MIPS; ++mip )
	{
		const float roughness = ( mipDenom > 0.0f ) ? ( (float)mip / mipDenom ) : 0.0f;
		for ( int face = 0; face < 6; ++face )
		{
			sg_pass pass = { 0 };
			pass.action.colors[0].load_action = SG_LOADACTION_DONTCARE;
			pass.action.colors[0].store_action = SG_STOREACTION_STORE;
			pass.attachments.colors[0] = s_ibl.skyCubeFaceMipAttachmentViews[face * GFX_IBL_CUBE_MIPS + mip];

			sg_begin_pass( &pass );
			sg_apply_pipeline( s_ibl.prefilterPipeline );

			sg_bindings bindings = { 0 };
			bindings.views[VIEW_prefilter_tex_sky] = s_ibl.skyCubeRawSampleView;
			bindings.samplers[SMP_prefilter_smp_sky] = s_ibl.skyCubeSampler;
			sg_apply_bindings( &bindings );

			const CubeFaceBasis b = s_cubeFaceBasis[face];
			prefilter_ub_pass_t up = { 0 };
			up.face_right = MakeVec4( b.right.x, b.right.y, b.right.z, 0.0f );
			up.face_up = MakeVec4( b.up.x, b.up.y, b.up.z, 0.0f );
			up.face_forward = MakeVec4( b.forward.x, b.forward.y, b.forward.z, 0.0f );
			up.params = MakeVec4( roughness, 0.0f, 0.0f, 0.0f );
			sg_apply_uniforms( UB_prefilter_ub_pass, &SG_RANGE( up ) );

			sg_draw( 0, 3, 1 );
			sg_end_pass();
		}
	}
	sg_pop_debug_group();
}

void InitImageBasedLighting( const sg_environment* env )
{
	(void)env;

	// BRDF LUT image + views + sampler
	sg_image_desc lutDesc = { 0 };
	lutDesc.usage.color_attachment = true;
	lutDesc.width = GFX_IBL_LUT_SIZE;
	lutDesc.height = GFX_IBL_LUT_SIZE;
	lutDesc.pixel_format = SG_PIXELFORMAT_RG16F;
	lutDesc.sample_count = 1;
	lutDesc.label = "ibl_brdf_lut";
	s_ibl.brdfLut = sg_make_image( &lutDesc );

	sg_view_desc lutAttachDesc = { 0 };
	lutAttachDesc.color_attachment.image = s_ibl.brdfLut;
	lutAttachDesc.label = "ibl_brdf_lut_attachment";
	s_ibl.brdfLutColorAttachmentView = sg_make_view( &lutAttachDesc );

	sg_view_desc lutSampleDesc = { 0 };
	lutSampleDesc.texture.image = s_ibl.brdfLut;
	lutSampleDesc.label = "ibl_brdf_lut_sample";
	s_ibl.brdfLutSampleView = sg_make_view( &lutSampleDesc );

	sg_sampler_desc lutSampDesc = { 0 };
	lutSampDesc.min_filter = SG_FILTER_LINEAR;
	lutSampDesc.mag_filter = SG_FILTER_LINEAR;
	lutSampDesc.mipmap_filter = SG_FILTER_NEAREST;
	lutSampDesc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
	lutSampDesc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
	lutSampDesc.wrap_w = SG_WRAP_CLAMP_TO_EDGE;
	lutSampDesc.label = "ibl_brdf_lut_sampler";
	s_ibl.brdfLutSampler = sg_make_sampler( &lutSampDesc );

	s_ibl.brdfLutShader = sg_make_shader( brdf_lut_build_shader_desc( sg_query_backend() ) );

	sg_pipeline_desc lutPipeDesc = { 0 };
	lutPipeDesc.shader = s_ibl.brdfLutShader;
	lutPipeDesc.colors[0].pixel_format = SG_PIXELFORMAT_RG16F;
	lutPipeDesc.color_count = 1;
	lutPipeDesc.depth.pixel_format = SG_PIXELFORMAT_NONE;
	lutPipeDesc.depth.write_enabled = false;
	lutPipeDesc.depth.compare = SG_COMPAREFUNC_ALWAYS;
	lutPipeDesc.cull_mode = SG_CULLMODE_NONE;
	lutPipeDesc.face_winding = SG_FACEWINDING_CCW;
	lutPipeDesc.sample_count = 1;
	lutPipeDesc.label = "ibl_brdf_lut_pipeline";
	s_ibl.brdfLutPipeline = sg_make_pipeline( &lutPipeDesc );

	GenerateBrdfLut();

	// Raw sky cube (sky_to_cube target / prefilter input)
	sg_image_desc rawDesc = { 0 };
	rawDesc.type = SG_IMAGETYPE_CUBE;
	rawDesc.usage.color_attachment = true;
	rawDesc.width = GFX_IBL_CUBE_SIZE;
	rawDesc.height = GFX_IBL_CUBE_SIZE;
	rawDesc.num_mipmaps = 1;
	rawDesc.pixel_format = SG_PIXELFORMAT_RGBA16F;
	rawDesc.sample_count = 1;
	rawDesc.label = "ibl_sky_cube_raw";
	s_ibl.skyCubeRaw = sg_make_image( &rawDesc );

	for ( int face = 0; face < 6; ++face )
	{
		sg_view_desc fdesc = { 0 };
		fdesc.color_attachment.image = s_ibl.skyCubeRaw;
		fdesc.color_attachment.slice = face;
		fdesc.color_attachment.mip_level = 0;
		fdesc.label = "ibl_sky_cube_raw_face";
		s_ibl.skyCubeRawFaceAttachmentViews[face] = sg_make_view( &fdesc );
	}

	sg_view_desc rawSampleDesc = { 0 };
	rawSampleDesc.texture.image = s_ibl.skyCubeRaw;
	rawSampleDesc.label = "ibl_sky_cube_raw_sample";
	s_ibl.skyCubeRawSampleView = sg_make_view( &rawSampleDesc );

	// Prefiltered sky cube (lit-pipeline input)
	sg_image_desc cubeDesc = { 0 };
	cubeDesc.type = SG_IMAGETYPE_CUBE;
	cubeDesc.usage.color_attachment = true;
	cubeDesc.width = GFX_IBL_CUBE_SIZE;
	cubeDesc.height = GFX_IBL_CUBE_SIZE;
	cubeDesc.num_mipmaps = GFX_IBL_CUBE_MIPS;
	cubeDesc.pixel_format = SG_PIXELFORMAT_RGBA16F;
	cubeDesc.sample_count = 1;
	cubeDesc.label = "ibl_sky_cube";
	s_ibl.skyCube = sg_make_image( &cubeDesc );

	for ( int face = 0; face < 6; ++face )
	{
		for ( int mip = 0; mip < GFX_IBL_CUBE_MIPS; ++mip )
		{
			sg_view_desc fdesc = { 0 };
			fdesc.color_attachment.image = s_ibl.skyCube;
			fdesc.color_attachment.slice = face;
			fdesc.color_attachment.mip_level = mip;
			fdesc.label = "ibl_sky_cube_face_mip";
			s_ibl.skyCubeFaceMipAttachmentViews[face * GFX_IBL_CUBE_MIPS + mip] = sg_make_view( &fdesc );
		}
	}

	sg_view_desc cubeSampleDesc = { 0 };
	cubeSampleDesc.texture.image = s_ibl.skyCube;
	cubeSampleDesc.label = "ibl_sky_cube_sample";
	s_ibl.skyCubeSampleView = sg_make_view( &cubeSampleDesc );

	sg_sampler_desc cubeSampDesc = { 0 };
	cubeSampDesc.min_filter = SG_FILTER_LINEAR;
	cubeSampDesc.mag_filter = SG_FILTER_LINEAR;
	cubeSampDesc.mipmap_filter = SG_FILTER_LINEAR;
	cubeSampDesc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
	cubeSampDesc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
	cubeSampDesc.wrap_w = SG_WRAP_CLAMP_TO_EDGE;
	cubeSampDesc.label = "ibl_sky_cube_sampler";
	s_ibl.skyCubeSampler = sg_make_sampler( &cubeSampDesc );

	// sky_to_cube pipeline
	s_ibl.skyToCubeShader = sg_make_shader( sky_to_cube_build_shader_desc( sg_query_backend() ) );

	sg_pipeline_desc skyPipeDesc = { 0 };
	skyPipeDesc.shader = s_ibl.skyToCubeShader;
	skyPipeDesc.colors[0].pixel_format = SG_PIXELFORMAT_RGBA16F;
	skyPipeDesc.color_count = 1;
	skyPipeDesc.depth.pixel_format = SG_PIXELFORMAT_NONE;
	skyPipeDesc.depth.write_enabled = false;
	skyPipeDesc.depth.compare = SG_COMPAREFUNC_ALWAYS;
	skyPipeDesc.cull_mode = SG_CULLMODE_NONE;
	skyPipeDesc.face_winding = SG_FACEWINDING_CCW;
	skyPipeDesc.sample_count = 1;
	skyPipeDesc.label = "ibl_sky_to_cube_pipeline";
	s_ibl.skyToCubePipeline = sg_make_pipeline( &skyPipeDesc );

	// prefilter pipeline
	s_ibl.prefilterShader = sg_make_shader( prefilter_build_shader_desc( sg_query_backend() ) );

	sg_pipeline_desc prePipeDesc = { 0 };
	prePipeDesc.shader = s_ibl.prefilterShader;
	prePipeDesc.colors[0].pixel_format = SG_PIXELFORMAT_RGBA16F;
	prePipeDesc.color_count = 1;
	prePipeDesc.depth.pixel_format = SG_PIXELFORMAT_NONE;
	prePipeDesc.depth.write_enabled = false;
	prePipeDesc.depth.compare = SG_COMPAREFUNC_ALWAYS;
	prePipeDesc.cull_mode = SG_CULLMODE_NONE;
	prePipeDesc.face_winding = SG_FACEWINDING_CCW;
	prePipeDesc.sample_count = 1;
	prePipeDesc.label = "ibl_prefilter_pipeline";
	s_ibl.prefilterPipeline = sg_make_pipeline( &prePipeDesc );

	// SH cache starts zero, RebuildImageBasedLightingIfDirty fills it.
	for ( int i = 0; i < 9; ++i )
	{
		s_ibl.shCoeffs[i] = b3Vec3_zero;
	}
	s_ibl.dirty = true;
	s_ibl.ready = true;
}

void ShutdownImageBasedLighting( void )
{
	if ( !s_ibl.ready )
	{
		return;
	}

	sg_destroy_pipeline( s_ibl.prefilterPipeline );
	sg_destroy_shader( s_ibl.prefilterShader );
	sg_destroy_pipeline( s_ibl.skyToCubePipeline );
	sg_destroy_shader( s_ibl.skyToCubeShader );

	sg_destroy_sampler( s_ibl.skyCubeSampler );
	sg_destroy_view( s_ibl.skyCubeSampleView );
	for ( int i = 0; i < 6 * GFX_IBL_CUBE_MIPS; ++i )
	{
		sg_destroy_view( s_ibl.skyCubeFaceMipAttachmentViews[i] );
	}
	sg_destroy_image( s_ibl.skyCube );

	sg_destroy_view( s_ibl.skyCubeRawSampleView );
	for ( int face = 0; face < 6; ++face )
	{
		sg_destroy_view( s_ibl.skyCubeRawFaceAttachmentViews[face] );
	}
	sg_destroy_image( s_ibl.skyCubeRaw );

	sg_destroy_pipeline( s_ibl.brdfLutPipeline );
	sg_destroy_shader( s_ibl.brdfLutShader );
	sg_destroy_sampler( s_ibl.brdfLutSampler );
	sg_destroy_view( s_ibl.brdfLutSampleView );
	sg_destroy_view( s_ibl.brdfLutColorAttachmentView );
	sg_destroy_image( s_ibl.brdfLut );
	s_ibl.ready = false;
}

void MarkIblDirty( void )
{
	s_ibl.dirty = true;
}

void RebuildImageBasedLightingIfDirty( b3Vec3 dirToSun, float turbidity, bool zUp )
{
	// A changed up axis reorients the baked sky, so treat it like a sun or
	// turbidity change and force a rebuild.
	if ( zUp != s_ibl.zUp )
	{
		s_ibl.zUp = zUp;
		s_ibl.dirty = true;
	}

	if ( s_ibl.dirty == false || s_ibl.ready == false )
	{
		return;
	}

	// Renormalize just in case to avoid NaNs
	float lenSq = dirToSun.x * dirToSun.x + dirToSun.y * dirToSun.y + dirToSun.z * dirToSun.z;
	b3Vec3 sun = ( lenSq > 0.0f ) ? b3Normalize( dirToSun ) : b3Vec3_axisY;
	float fade = SkySunFadeWeight( sun, zUp );

	sg_push_debug_group( "ibl_rebuild" );

	GenerateSkyCube( sun, turbidity, fade, zUp );
	GeneratePrefilter();
	ProjectSkyToSh( sun, turbidity, fade, zUp );

	sg_pop_debug_group();

	s_ibl.dirty = false;
}

sg_view GetIblSpecularView( void )
{
	return s_ibl.skyCubeSampleView;
}

sg_sampler GetIblSpecularSampler( void )
{
	return s_ibl.skyCubeSampler;
}

float GetIblSpecularMaxLod( void )
{
	return (float)( GFX_IBL_CUBE_MIPS - 1 );
}

sg_view GetIblBrdfLutView( void )
{
	return s_ibl.brdfLutSampleView;
}

sg_sampler GetIblBrdfLutSampler( void )
{
	return s_ibl.brdfLutSampler;
}

void GetIblSphericalHarmonicCoefficients( b3Vec3 out[9] )
{
	for ( int i = 0; i < 9; ++i )
	{
		out[i] = s_ibl.shCoeffs[i];
	}
}
