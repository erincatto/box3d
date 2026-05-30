// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/highlight_outline.h"

#include "highlight_outline.glsl.h"

#include <stdio.h>
#include <string.h>

#define HIGHLIGHT_OUTLINE_MAX_PIPELINES 2

static struct
{
	sg_shader shader;
	sg_sampler sampler;
	struct
	{
		sg_pixel_format colorFormat;
		sg_pixel_format depthFormat;
		sg_pipeline pipeline;
	} pipelines[HIGHLIGHT_OUTLINE_MAX_PIPELINES];
	int pipelineCount;
	bool ready;
} s_ho;

HighlightParams GetDefaultHighlightParams( void )
{
	HighlightParams p;
	p.enabled = true;
	p.hoverThicknessPx = 1.0f;
	p.selectThicknessPx = 2.0f;
	// Cool cyan for hover at moderate alpha. Warm orange-yellow for select
	// at full alpha. Authored as straight RGBA. Premultiplied in the
	// submit step.
	p.hoverColor = MakeVec4( 0.35f, 0.85f, 1.0f, 0.75f );
	p.selectColor = MakeVec4( 1.0f, 0.78f, 0.20f, 1.0f );
	return p;
}

static sg_pipeline ensurePipeline( sg_pixel_format colorFormat, sg_pixel_format depthFormat )
{
	for ( int i = 0; i < s_ho.pipelineCount; ++i )
	{
		if ( s_ho.pipelines[i].colorFormat == colorFormat && s_ho.pipelines[i].depthFormat == depthFormat )
		{
			return s_ho.pipelines[i].pipeline;
		}
	}
	if ( s_ho.pipelineCount >= HIGHLIGHT_OUTLINE_MAX_PIPELINES )
	{
		fprintf( stderr, "highlight_outline: pipeline cache exhausted (max %d format pairs)\n", HIGHLIGHT_OUTLINE_MAX_PIPELINES );
		sg_pipeline empty = { 0 };
		return empty;
	}

	sg_pipeline_desc pdesc = { 0 };
	pdesc.shader = s_ho.shader;
	pdesc.colors[0].pixel_format = colorFormat;
	pdesc.color_count = 1;
	// Premultiplied-alpha blend over the resolved scene color. ub_pass
	// colors are premultiplied by the C side.
	pdesc.colors[0].blend.enabled = true;
	pdesc.colors[0].blend.src_factor_rgb = SG_BLENDFACTOR_ONE;
	pdesc.colors[0].blend.dst_factor_rgb = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
	pdesc.colors[0].blend.op_rgb = SG_BLENDOP_ADD;
	pdesc.colors[0].blend.src_factor_alpha = SG_BLENDFACTOR_ONE;
	pdesc.colors[0].blend.dst_factor_alpha = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
	pdesc.colors[0].blend.op_alpha = SG_BLENDOP_ADD;
	pdesc.depth.pixel_format = depthFormat;
	pdesc.depth.write_enabled = false;
	pdesc.depth.compare = SG_COMPAREFUNC_ALWAYS;
	pdesc.cull_mode = SG_CULLMODE_NONE;
	pdesc.face_winding = SG_FACEWINDING_CCW;
	pdesc.sample_count = 1;
	pdesc.label = "highlight_outline_pipeline";

	const sg_pipeline pip = sg_make_pipeline( &pdesc );
	s_ho.pipelines[s_ho.pipelineCount].colorFormat = colorFormat;
	s_ho.pipelines[s_ho.pipelineCount].depthFormat = depthFormat;
	s_ho.pipelines[s_ho.pipelineCount].pipeline = pip;
	++s_ho.pipelineCount;
	return pip;
}

void InitHighlightOutline( void )
{
	if ( s_ho.ready )
		return;
	s_ho.shader = sg_make_shader( highlight_outline_prog_shader_desc( sg_query_backend() ) );

	// texelFetch reads ignore sampler state, but sokol-gfx still requires a
	// sampler binding for any texture binding. Nearest + clamp. Nothing
	// exotic.
	sg_sampler_desc sdesc = { 0 };
	sdesc.min_filter = SG_FILTER_NEAREST;
	sdesc.mag_filter = SG_FILTER_NEAREST;
	sdesc.mipmap_filter = SG_FILTER_NEAREST;
	sdesc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
	sdesc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
	sdesc.label = "highlight_outline_sampler";
	s_ho.sampler = sg_make_sampler( &sdesc );

	s_ho.ready = true;
}

void ShutdownHighlightOutline( void )
{
	if ( !s_ho.ready )
		return;
	for ( int i = 0; i < s_ho.pipelineCount; ++i )
	{
		sg_destroy_pipeline( s_ho.pipelines[i].pipeline );
		s_ho.pipelines[i].pipeline.id = 0;
		s_ho.pipelines[i].colorFormat = SG_PIXELFORMAT_NONE;
		s_ho.pipelines[i].depthFormat = SG_PIXELFORMAT_NONE;
	}
	s_ho.pipelineCount = 0;
	sg_destroy_sampler( s_ho.sampler );
	s_ho.sampler.id = 0;
	sg_destroy_shader( s_ho.shader );
	s_ho.shader.id = 0;
	s_ho.ready = false;
}

static inline Vec4 Premultiply( Vec4 c )
{
	return MakeVec4( c.x * c.w, c.y * c.w, c.z * c.w, c.w );
}

void SubmitHighlightOutline( sg_view maskSampleView, sg_pixel_format colorFormat, sg_pixel_format depthFormat,
							 const HighlightParams* params )
{
	if ( !s_ho.ready || !params || !params->enabled )
		return;

	const sg_pipeline pip = ensurePipeline( colorFormat, depthFormat );
	if ( pip.id == 0 )
		return;

	sg_apply_pipeline( pip );

	sg_bindings b = { 0 };
	b.views[VIEW_highlight_outline_tex_mask] = maskSampleView;
	b.samplers[SMP_highlight_outline_smp_mask] = s_ho.sampler;
	sg_apply_bindings( &b );

	highlight_outline_ub_pass_t up = { 0 };
	up.hover_color = Premultiply( params->hoverColor );
	up.select_color = Premultiply( params->selectColor );
	up.thickness_px = MakeVec4( params->hoverThicknessPx, params->selectThicknessPx, 0.0f, 0.0f );
	sg_apply_uniforms( UB_highlight_outline_ub_pass, &SG_RANGE( up ) );

	sg_draw( 0, 3, 1 );
}
