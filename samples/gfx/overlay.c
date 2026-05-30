// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/overlay.h"

#include "gfx/arena.h"
#include "overlay_line.glsl.h"
#include "overlay_point.glsl.h"

#include <stdio.h>
#include <string.h>

#define MAX_LINES 65536
#define MAX_POINTS 65536

// The overlay draws into the post-tone map target (swap chain or the offscreen
// test target). Two slots cover both. The swap chain (has a depth
// attachment) and the color-only offscreen target. Mirrors tonemap.c's
// cache. Hitting the cap means a third color/depth pair appeared.
#define OVERLAY_MAX_PIPELINES 2

static struct
{
	Arena lineArena;
	Arena pointArena;

	overlay_line_line_instance_t* lineBase;
	overlay_point_point_instance_t* pointBase;
	int lineCount;
	int pointCount;

	sg_shader lineShader;
	sg_shader pointShader;

	// Pipelines are created lazily per (color, depth) output-format pair.
	// The post-tone map target's format isn't known at init time.
	struct
	{
		sg_pixel_format colorFormat;
		sg_pixel_format depthFormat;
		sg_pipeline linePipeline;
		sg_pipeline pointPipeline;
	} pipelines[OVERLAY_MAX_PIPELINES];
	int pipelineCount;

	sg_buffer lineInstBuf;
	sg_buffer pointInstBuf;
	sg_view lineInstView;
	sg_view pointInstView;

	bool ready;
} s_overlay;

static uint32_t PackThicknessFlag( OverlayThicknessUnit unit )
{
	return ( unit == OVERLAY_THICKNESS_WORLD ) ? 1u : 0u;
}

static uint32_t PackOcclusionFlag( OverlayOcclusionMode mode )
{
	switch ( mode )
	{
		case OVERLAY_OCCLUSION_DIM:
			return 1u;
		case OVERLAY_OCCLUSION_DASHED:
			return 2u;
		case OVERLAY_OCCLUSION_HIDE: /* fallthrough */
		default:
			return 0u;
	}
}

static Vec4 Premultiply( Vec4 c )
{
	return MakeVec4( c.x * c.w, c.y * c.w, c.z * c.w, c.w );
}

// Lazily create (and cache) the line + point pipelines for one output
// format pair. Mirrors tonemap.c::ensurePipeline. The overlay draws
// post-tonemap into the swapchain or an offscreen test target, both
// single-sample, so sample_count is fixed at 1. The line/point shaders
// carry their own SDF coverage AA. Returns false (and logs) if the cache
// is full.
static bool EnsurePipelines( sg_pixel_format colorFormat, sg_pixel_format depthFormat, sg_pipeline* outLine,
							 sg_pipeline* outPoint )
{
	for ( int i = 0; i < s_overlay.pipelineCount; ++i )
	{
		if ( s_overlay.pipelines[i].colorFormat == colorFormat && s_overlay.pipelines[i].depthFormat == depthFormat )
		{
			*outLine = s_overlay.pipelines[i].linePipeline;
			*outPoint = s_overlay.pipelines[i].pointPipeline;
			return true;
		}
	}
	if ( s_overlay.pipelineCount >= OVERLAY_MAX_PIPELINES )
	{
		fprintf( stderr, "[overlay] pipeline cache exhausted (max %d format pairs)\n", OVERLAY_MAX_PIPELINES );
		return false;
	}

	// Line pipeline
	sg_pipeline_desc linePdesc = { 0 };
	linePdesc.shader = s_overlay.lineShader;
	// No VBO / no IBO, VS expands six vertices per instance from
	// gl_VertexIndex. Default layout (no attrs declared) is correct.
	linePdesc.index_type = SG_INDEXTYPE_NONE;
	linePdesc.cull_mode = SG_CULLMODE_NONE;
	linePdesc.colors[0].pixel_format = colorFormat;
	linePdesc.color_count = 1;
	// Premultiplied-alpha blend. CPU side premultiplies color.rgb by .a.
	linePdesc.colors[0].blend.enabled = true;
	linePdesc.colors[0].blend.src_factor_rgb = SG_BLENDFACTOR_ONE;
	linePdesc.colors[0].blend.dst_factor_rgb = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
	linePdesc.colors[0].blend.op_rgb = SG_BLENDOP_ADD;
	linePdesc.colors[0].blend.src_factor_alpha = SG_BLENDFACTOR_ONE;
	linePdesc.colors[0].blend.dst_factor_alpha = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
	linePdesc.colors[0].blend.op_alpha = SG_BLENDOP_ADD;
	// Depth-blind: compare ALWAYS, write off. pixel_format still has to
	// match the target's depth attachment (or NONE) for sokol's validation.
	linePdesc.depth.pixel_format = depthFormat;
	linePdesc.depth.compare = SG_COMPAREFUNC_ALWAYS;
	linePdesc.depth.write_enabled = false;
	linePdesc.sample_count = 1;
	linePdesc.label = "overlay_line_pipeline";
	const sg_pipeline linePip = sg_make_pipeline( &linePdesc );

	// Point pipeline
	sg_pipeline_desc pointPdesc = { 0 };
	pointPdesc.shader = s_overlay.pointShader;
	pointPdesc.index_type = SG_INDEXTYPE_NONE;
	pointPdesc.cull_mode = SG_CULLMODE_NONE;
	pointPdesc.colors[0].pixel_format = colorFormat;
	pointPdesc.color_count = 1;
	pointPdesc.colors[0].blend.enabled = true;
	pointPdesc.colors[0].blend.src_factor_rgb = SG_BLENDFACTOR_ONE;
	pointPdesc.colors[0].blend.dst_factor_rgb = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
	pointPdesc.colors[0].blend.op_rgb = SG_BLENDOP_ADD;
	pointPdesc.colors[0].blend.src_factor_alpha = SG_BLENDFACTOR_ONE;
	pointPdesc.colors[0].blend.dst_factor_alpha = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
	pointPdesc.colors[0].blend.op_alpha = SG_BLENDOP_ADD;
	pointPdesc.depth.pixel_format = depthFormat;
	pointPdesc.depth.compare = SG_COMPAREFUNC_ALWAYS;
	pointPdesc.depth.write_enabled = false;
	pointPdesc.sample_count = 1;
	pointPdesc.label = "overlay_point_pipeline";
	const sg_pipeline pointPip = sg_make_pipeline( &pointPdesc );

	s_overlay.pipelines[s_overlay.pipelineCount].colorFormat = colorFormat;
	s_overlay.pipelines[s_overlay.pipelineCount].depthFormat = depthFormat;
	s_overlay.pipelines[s_overlay.pipelineCount].linePipeline = linePip;
	s_overlay.pipelines[s_overlay.pipelineCount].pointPipeline = pointPip;
	++s_overlay.pipelineCount;

	*outLine = linePip;
	*outPoint = pointPip;
	return true;
}

void OverlayInit( void )
{
	if ( s_overlay.ready )
	{
		return;
	}

	if ( ArenaInit( &s_overlay.lineArena, (size_t)MAX_LINES * sizeof( overlay_line_line_instance_t ) ) != 0 )
	{
		fprintf( stderr, "[overlay] failed to init line arena\n" );
		return;
	}
	if ( ArenaInit( &s_overlay.pointArena, (size_t)MAX_POINTS * sizeof( overlay_point_point_instance_t ) ) != 0 )
	{
		fprintf( stderr, "[overlay] failed to init point arena\n" );
		ArenaShutdown( &s_overlay.lineArena );
		return;
	}

	// Line instance buffer + shader
	sg_buffer_desc lineBufDesc = { 0 };
	lineBufDesc.usage.storage_buffer = true;
	lineBufDesc.usage.stream_update = true;
	lineBufDesc.size = (size_t)MAX_LINES * sizeof( overlay_line_line_instance_t );
	lineBufDesc.label = "overlay_line_instances";
	s_overlay.lineInstBuf = sg_make_buffer( &lineBufDesc );

	sg_view_desc lineViewDesc = { 0 };
	lineViewDesc.storage_buffer.buffer = s_overlay.lineInstBuf;
	lineViewDesc.label = "overlay_line_instances_view";
	s_overlay.lineInstView = sg_make_view( &lineViewDesc );

	s_overlay.lineShader = sg_make_shader( overlay_line_prog_shader_desc( sg_query_backend() ) );

	// Point instance buffer + shader
	sg_buffer_desc pointBufDesc = { 0 };
	pointBufDesc.usage.storage_buffer = true;
	pointBufDesc.usage.stream_update = true;
	pointBufDesc.size = (size_t)MAX_POINTS * sizeof( overlay_point_point_instance_t );
	pointBufDesc.label = "overlay_point_instances";
	s_overlay.pointInstBuf = sg_make_buffer( &pointBufDesc );

	sg_view_desc pointViewDesc = { 0 };
	pointViewDesc.storage_buffer.buffer = s_overlay.pointInstBuf;
	pointViewDesc.label = "overlay_point_instances_view";
	s_overlay.pointInstView = sg_make_view( &pointViewDesc );

	s_overlay.pointShader = sg_make_shader( overlay_point_prog_shader_desc( sg_query_backend() ) );

	s_overlay.ready = true;
}

void OverlayShutdown( void )
{
	if ( !s_overlay.ready )
	{
		return;
	}
	for ( int i = 0; i < s_overlay.pipelineCount; ++i )
	{
		sg_destroy_pipeline( s_overlay.pipelines[i].linePipeline );
		sg_destroy_pipeline( s_overlay.pipelines[i].pointPipeline );
		s_overlay.pipelines[i].linePipeline.id = 0;
		s_overlay.pipelines[i].pointPipeline.id = 0;
		s_overlay.pipelines[i].colorFormat = SG_PIXELFORMAT_NONE;
		s_overlay.pipelines[i].depthFormat = SG_PIXELFORMAT_NONE;
	}
	s_overlay.pipelineCount = 0;
	sg_destroy_shader( s_overlay.lineShader );
	s_overlay.lineShader.id = 0;
	sg_destroy_shader( s_overlay.pointShader );
	s_overlay.pointShader.id = 0;
	sg_destroy_view( s_overlay.lineInstView );
	s_overlay.lineInstView.id = 0;
	sg_destroy_view( s_overlay.pointInstView );
	s_overlay.pointInstView.id = 0;
	sg_destroy_buffer( s_overlay.lineInstBuf );
	s_overlay.lineInstBuf.id = 0;
	sg_destroy_buffer( s_overlay.pointInstBuf );
	s_overlay.pointInstBuf.id = 0;
	ArenaShutdown( &s_overlay.lineArena );
	ArenaShutdown( &s_overlay.pointArena );
	s_overlay.lineBase = NULL;
	s_overlay.pointBase = NULL;
	s_overlay.lineCount = 0;
	s_overlay.pointCount = 0;
	s_overlay.ready = false;
}

void OverlayResetFrame( void )
{
	ArenaReset( &s_overlay.lineArena );
	ArenaReset( &s_overlay.pointArena );
	s_overlay.lineBase = NULL;
	s_overlay.pointBase = NULL;
	s_overlay.lineCount = 0;
	s_overlay.pointCount = 0;
}

int OverlayLineCount( void )
{
	return s_overlay.lineCount;
}
int OverlayPointCount( void )
{
	return s_overlay.pointCount;
}

void OverlayAppendLine( b3Vec3 a, b3Vec3 b, Vec4 linearColor, float thickness, OverlayThicknessUnit thicknessUnit,
						OverlayOcclusionMode occlusionMode )
{
	if ( s_overlay.lineCount >= MAX_LINES )
	{
		return;
	}
	overlay_line_line_instance_t* inst = (overlay_line_line_instance_t*)ArenaAlloc(
		&s_overlay.lineArena, sizeof( overlay_line_line_instance_t ), _Alignof( overlay_line_line_instance_t ) );
	if ( !inst )
	{
		return;
	}
	if ( s_overlay.lineCount == 0 )
	{
		s_overlay.lineBase = inst;
	}
	s_overlay.lineCount++;

	const Vec4 cpremul = Premultiply( linearColor );
	inst->a = MakeVec4( a.x, a.y, a.z, thickness );
	inst->b = MakeVec4( b.x, b.y, b.z, 0.0f );
	inst->color = cpremul;
	inst->flags[0] = PackThicknessFlag( thicknessUnit );
	inst->flags[1] = PackOcclusionFlag( occlusionMode );
	inst->flags[2] = 0;
	inst->flags[3] = 0;
}

void OverlayAppendPoint( b3Vec3 p, Vec4 linearColor, float size, OverlayThicknessUnit sizeUnit, OverlayOcclusionMode occlusionMode )
{
	if ( s_overlay.pointCount >= MAX_POINTS )
	{
		return;
	}
	overlay_point_point_instance_t* inst = (overlay_point_point_instance_t*)ArenaAlloc(
		&s_overlay.pointArena, sizeof( overlay_point_point_instance_t ), _Alignof( overlay_point_point_instance_t ) );
	if ( !inst )
	{
		return;
	}
	if ( s_overlay.pointCount == 0 )
	{
		s_overlay.pointBase = inst;
	}
	s_overlay.pointCount++;

	const Vec4 cpremul = Premultiply( linearColor );
	inst->p = MakeVec4( p.x, p.y, p.z, size );
	inst->color = cpremul;
	inst->flags[0] = PackThicknessFlag( sizeUnit );
	inst->flags[1] = PackOcclusionFlag( occlusionMode );
	inst->flags[2] = 0;
	inst->flags[3] = 0;
}

void OverlaySubmit( int width, int height, const Mat4* view, const Mat4* viewInv, const Mat4* proj, const Mat4* projInv,
					b3Vec3 cameraPos, float time,
					sg_view linearDepthView, sg_sampler linearDepthSampler, sg_pixel_format colorFormat,
					sg_pixel_format depthFormat )
{
	if ( !s_overlay.ready )
	{
		return;
	}
	if ( s_overlay.lineCount == 0 && s_overlay.pointCount == 0 )
	{
		return;
	}

	sg_pipeline linePipeline = { 0 };
	sg_pipeline pointPipeline = { 0 };
	if ( !EnsurePipelines( colorFormat, depthFormat, &linePipeline, &pointPipeline ) )
	{
		return;
	}

	// Build the shared ub_frame data. Both shaders' reflected ub_frame
	// structs have the same layout (declared identically in both .glsl),
	// so one struct populates both via sg_apply_uniforms.
	const Mat4 view_proj = MulMM4( *proj, *view );
	// inv(P*V) = inv(V)*inv(P). The camera builds both inverses from the
	// same parameters as the forward matrices, so this is a single multiply
	// instead of a general 4x4 inversion.
	const Mat4 inv_view_proj = MulMM4( *viewInv, *projInv );
	const float w = (float)width;
	const float h = (float)height;
	overlay_line_ub_frame_t ub = { 0 };
	ub.view = *view;
	ub.proj = *proj;
	ub.view_proj = view_proj;
	ub.inv_view_proj = inv_view_proj;
	ub.camera_pos = MakeVec4( cameraPos.x, cameraPos.y, cameraPos.z, time );
	ub.viewport = MakeVec4( w, h, w > 0.0f ? 1.0f / w : 0.0f, h > 0.0f ? 1.0f / h : 0.0f );

	// Lines
	if ( s_overlay.lineCount > 0 )
	{
		sg_range r;
		r.ptr = s_overlay.lineBase;
		r.size = (size_t)s_overlay.lineCount * sizeof( overlay_line_line_instance_t );
		sg_update_buffer( s_overlay.lineInstBuf, &r );

		sg_apply_pipeline( linePipeline );
		sg_bindings b = { 0 };
		b.views[VIEW_overlay_line_line_instances] = s_overlay.lineInstView;
		b.views[VIEW_overlay_line_tex_depth] = linearDepthView;
		b.samplers[SMP_overlay_line_smp_depth] = linearDepthSampler;
		sg_apply_bindings( &b );
		sg_apply_uniforms( UB_overlay_line_ub_frame, &SG_RANGE( ub ) );
		sg_draw( 0, 6, s_overlay.lineCount );
	}

	// Points
	if ( s_overlay.pointCount > 0 )
	{
		sg_range r;
		r.ptr = s_overlay.pointBase;
		r.size = (size_t)s_overlay.pointCount * sizeof( overlay_point_point_instance_t );
		sg_update_buffer( s_overlay.pointInstBuf, &r );

		// Same ub_frame data. The generated _t names differ but the layout
		// matches byte-for-byte (both glsl files declare identical ub_frame).
		overlay_point_ub_frame_t pub;
		memcpy( &pub, &ub, sizeof( pub ) );

		sg_apply_pipeline( pointPipeline );
		sg_bindings b = { 0 };
		b.views[VIEW_overlay_point_point_instances] = s_overlay.pointInstView;
		b.views[VIEW_overlay_point_tex_depth] = linearDepthView;
		b.samplers[SMP_overlay_point_smp_depth] = linearDepthSampler;
		sg_apply_bindings( &b );
		sg_apply_uniforms( UB_overlay_point_ub_frame, &SG_RANGE( pub ) );
		sg_draw( 0, 6, s_overlay.pointCount );
	}
}
