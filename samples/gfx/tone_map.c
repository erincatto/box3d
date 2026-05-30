// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/tone_map.h"

#include "gfx/utility.h"
#include "tonemap.glsl.h"

#include <stdio.h>
#include <string.h>

// Two slots cover the only realistic divergence, the swapchain target
// (has depth attachment from sokol_app's default) and the offscreen test
// target (color only, no depth). Hitting this cap means a third
// color/depth pair showed up. Raise it (and probably ask why).
#define TONEMAP_MAX_PIPELINES 2

static struct
{
	sg_shader shader;
	sg_sampler sampler;
	struct
	{
		sg_pixel_format colorFormat;
		sg_pixel_format depthFormat;
		sg_pipeline pipeline;
	} pipelines[TONEMAP_MAX_PIPELINES];
	int pipelineCount;
	bool ready;
} s_tm;

static sg_pipeline ensurePipeline( sg_pixel_format colorFormat, sg_pixel_format depthFormat )
{
	for ( int i = 0; i < s_tm.pipelineCount; ++i )
	{
		if ( s_tm.pipelines[i].colorFormat == colorFormat && s_tm.pipelines[i].depthFormat == depthFormat )
		{
			return s_tm.pipelines[i].pipeline;
		}
	}
	if ( s_tm.pipelineCount >= TONEMAP_MAX_PIPELINES )
	{
		fprintf( stderr, "tonemap: pipeline cache exhausted (max %d format pairs)\n", TONEMAP_MAX_PIPELINES );
		sg_pipeline empty = { 0 };
		return empty;
	}

	sg_pipeline_desc pdesc = { 0 };
	pdesc.shader = s_tm.shader;
	pdesc.colors[0].pixel_format = colorFormat;
	// color_count isn't inferred from a single non-NONE entry, has to be
	// explicit (cf. MEMORY note on MRT color_count gotcha).
	pdesc.color_count = 1;
	// Depth format mirrors the target's depth attachment (or NONE) so
	// sokol's pipeline-vs-attachment validation is happy. Write/compare
	// are off either way, the tonemap is depth-blind.
	pdesc.depth.pixel_format = depthFormat;
	pdesc.depth.write_enabled = false;
	pdesc.depth.compare = SG_COMPAREFUNC_ALWAYS;
	pdesc.cull_mode = SG_CULLMODE_NONE;
	pdesc.face_winding = SG_FACEWINDING_CCW;
	pdesc.sample_count = 1;
	pdesc.label = "tonemap_pipeline";

	const sg_pipeline pip = sg_make_pipeline( &pdesc );
	s_tm.pipelines[s_tm.pipelineCount].colorFormat = colorFormat;
	s_tm.pipelines[s_tm.pipelineCount].depthFormat = depthFormat;
	s_tm.pipelines[s_tm.pipelineCount].pipeline = pip;
	++s_tm.pipelineCount;
	return pip;
}

void InitToneMap( void )
{
	s_tm.shader = sg_make_shader( tonemap_prog_shader_desc( sg_query_backend() ) );

	sg_sampler_desc sdesc = { 0 };
	sdesc.min_filter = SG_FILTER_LINEAR;
	sdesc.mag_filter = SG_FILTER_LINEAR;
	sdesc.mipmap_filter = SG_FILTER_NEAREST;
	sdesc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
	sdesc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
	sdesc.wrap_w = SG_WRAP_CLAMP_TO_EDGE;
	sdesc.label = "tonemap_sampler";
	s_tm.sampler = sg_make_sampler( &sdesc );

	s_tm.ready = true;
}

void ShutdownToneMap( void )
{
	if ( !s_tm.ready )
	{
		return;
	}
	for ( int i = 0; i < s_tm.pipelineCount; ++i )
	{
		sg_destroy_pipeline( s_tm.pipelines[i].pipeline );
		s_tm.pipelines[i].pipeline.id = 0;
		s_tm.pipelines[i].colorFormat = SG_PIXELFORMAT_NONE;
		s_tm.pipelines[i].depthFormat = SG_PIXELFORMAT_NONE;
	}
	s_tm.pipelineCount = 0;
	sg_destroy_sampler( s_tm.sampler );
	sg_destroy_shader( s_tm.shader );
	s_tm.ready = false;
}

void SubmitToneMap( sg_view sceneColorView, sg_pixel_format outputColorFormat, sg_pixel_format outputDepthFormat, float ev,
					float saturation )
{
	if ( !s_tm.ready )
	{
		return;
	}
	const sg_pipeline pip = ensurePipeline( outputColorFormat, outputDepthFormat );
	if ( pip.id == 0 )
	{
		return;
	}

	sg_apply_pipeline( pip );

	sg_bindings bindings = { 0 };
	bindings.views[VIEW_tonemap_tex_scene] = sceneColorView;
	bindings.samplers[SMP_tonemap_smp_scene] = s_tm.sampler;
	sg_apply_bindings( &bindings );

	const sg_backend backend = sg_query_backend();
	const float uvYSign = ( backend == SG_BACKEND_GLCORE || backend == SG_BACKEND_GLES3 ) ? 1.0f : -1.0f;

	tonemap_ub_tonemap_t ub = { 0 };
	ub.params = MakeVec4( ev, uvYSign, saturation, 0.0f );
	sg_apply_uniforms( UB_tonemap_ub_tonemap, &SG_RANGE( ub ) );

	sg_draw( 0, 3, 1 );
}
