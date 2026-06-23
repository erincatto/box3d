// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/sky.h"

#include "gfx/scene_target.h"
#include "sky.glsl.h"

// Below-horizon fade: the renderer feeds the FS a [0,1] weight derived from
// the sun's elevation, smoothly stepped between BELOW_HORIZON_FADE_START
// (~2 deg) and BELOW_HORIZON_FADE_END (~5 deg). Preetham diverges at and below
// the horizon, so we fade to zero there rather than render garbage. Values
// in radians via sin so we can compare against dirToSun.y directly.
#define BELOW_HORIZON_FADE_START_SIN 0.0349f // sin(2 deg)
#define BELOW_HORIZON_FADE_END_SIN 0.0872f	 // sin(5 deg)

static struct
{
	sg_shader shader;
	sg_pipeline pipeline;
	bool ready;
} s_sky;

void InitSky( const sg_environment* env )
{
	s_sky.shader = sg_make_shader( sky_shader_desc( sg_query_backend() ) );

	sg_pipeline_desc pdesc = { 0 };
	pdesc.shader = s_sky.shader;
	// No vertex attributes, the VS pulls 3 fullscreen-triangle corners from
	// gl_VertexIndex. Leaving layout.attrs zeroed tells sokol the pipeline
	// has no input bindings (D3D11 builds an empty input layout).
	// Color attachment is the HDR scene target (RGBA16F MSAA).
	pdesc.colors[0].pixel_format = SG_PIXELFORMAT_RGBA16F;
	pdesc.depth.pixel_format = env->defaults.depth_format;
	// Reverse-Z: depth was cleared to 0 (far). Sky vertices sit at NDC z=0
	// (also far). GREATER_EQUAL passes against the cleared value on
	// untouched pixels and fails on opaque pixels (whose z > 0 means
	// they're closer than the far plane in reverse-Z).
	pdesc.depth.compare = SG_COMPAREFUNC_GREATER_EQUAL;
	pdesc.depth.write_enabled = false;
	// No back-face winding to enforce, but matching the rest of the
	// pipeline's CCW front face means the rasterizer's culling logic
	// never sees a back face on the fullscreen triangle.
	pdesc.cull_mode = SG_CULLMODE_NONE;
	pdesc.face_winding = SG_FACEWINDING_CCW;
	// Must match the scene_hdr MSAA color attachment's sample count.
	pdesc.sample_count = SCENE_SAMPLE_COUNT;
	pdesc.label = "sky_pipeline";
	s_sky.pipeline = sg_make_pipeline( &pdesc );

	s_sky.ready = true;
}

void ShutdownSky( void )
{
	if ( !s_sky.ready )
	{
		return;
	}
	sg_destroy_pipeline( s_sky.pipeline );
	sg_destroy_shader( s_sky.shader );
	s_sky.ready = false;
}

float SkySunFadeWeight( b3Vec3 dirToSun, bool zUp )
{
	// dirToSun is assumed unit-length. The rendering side renormalizes
	// before calling. sin(elevation) is the component along the sim up axis.
	float elevSin = zUp ? dirToSun.z : dirToSun.y;
	float t = ( elevSin - BELOW_HORIZON_FADE_START_SIN ) / ( BELOW_HORIZON_FADE_END_SIN - BELOW_HORIZON_FADE_START_SIN );
	t = ( t < 0.0f ) ? 0.0f : ( t > 1.0f ? 1.0f : t );
	return t * t * ( 3.0f - 2.0f * t );
}

void DrawSky( b3Vec3 dirToSun, float turbidity, b3Vec3 cameraPos, Mat4 invViewProj, bool zUp )
{
	if ( !s_sky.ready )
	{
		return;
	}

	// Renormalize defensively. Caller is the renderer with sun.dirToSun
	// already normalized, but the FS reads sun_dir_world.xyz expecting unit
	// length and an undetected zero vector here would emit NaNs.
	const float lenSq = dirToSun.x * dirToSun.x + dirToSun.y * dirToSun.y + dirToSun.z * dirToSun.z;
	b3Vec3 sun = ( lenSq > 0.0f ) ? b3Normalize( dirToSun ) : b3Vec3_axisY;

	const float fade = SkySunFadeWeight( sun, zUp );

	sky_ub_frame_t ub = { 0 };
	ub.inv_view_proj = invViewProj;

	// Sun stays in sim space, the FS rotates it into the model's Y-up frame
	// alongside the view ray when zUp is set.
	sky_ub_pass_t up = { 0 };
	up.sun_dir_world = MakeVec4( sun.x, sun.y, sun.z, fade );
	up.sky_params = MakeVec4( turbidity, zUp ? 1.0f : 0.0f, 0.0f, 0.0f );
	up.camera_pos = MakeVec4( cameraPos.x, cameraPos.y, cameraPos.z, 0.0f );

	sg_apply_pipeline( s_sky.pipeline );

	// Deliberately no sg_apply_bindings: the pipeline declares no vertex
	// buffers, no textures, and no storage views, so its required_bindings
	// mask is empty. Sokol's draw-time check is "applied == required". An
	// entirely-empty sg_bindings actually FAILS validation
	// (VALIDATE_ABND_EMPTY_BINDINGS, sokol_gfx.h:23591), so the only way
	// to draw a bindings-free pipeline is to skip the call.
	sg_apply_uniforms( UB_sky_ub_frame, &SG_RANGE( ub ) );
	sg_apply_uniforms( UB_sky_ub_pass, &SG_RANGE( up ) );
	sg_draw( 0, 3, 1 );
}
