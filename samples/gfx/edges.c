// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/edges.h"

#include "edge.glsl.h"
#include "box3d/types.h"
#include "gfx/geometry_registry.h"
#include "gfx/scene_target.h"

// Two pipelines off the same shader:

// pipelineMsaa: sample_count=SCENE_SAMPLE_COUNT, scene_target color+depth,
// for the opaque path (drawn inside the open MSAA scene pass).

// pipelineTransparent: sample_count=1, RGBA16F + SG_PIXELFORMAT_DEPTH, for
// the transparent path (own pass against resolve + GTAO prepass depth).
// Note: the edges are not transparent, they are being drawn after transparent
// shapes, which looks good on transparent hulls.

// Both share blend (premultiplied alpha) and depth state (GREATER_EQUAL,
// no write, reverse-Z, the VS adds zBias toward camera).
static struct
{
	sg_shader shader;
	sg_pipeline pipelineMsaa;
	sg_pipeline pipelineTransparent;
	bool ready;
} s_edges;

EdgeOverlayParams GetDefaultEdgeParams( void )
{
	EdgeOverlayParams p;
	p.enabled = true;
	p.showHulls = true;
	p.showMeshes = true;
	p.showHeightfields = true;
	p.showEdgeConvexity = false;
	p.thicknessPx = 1.5f;
	p.zBias = 1.0e-6f;
	p.convexColor = MakeVec4( 0.1f, 0.8f, 0.1f, 0.75f );
	p.concaveColor = MakeVec4( 0.8f, 0.10f, 0.10f, 0.75f );
	p.flatColor = MakeVec4( 0.6f, 0.6f, 0.6f, 0.5f );
	return p;
}

static void ApplyCommonPipelineState( sg_pipeline_desc* p )
{
	p->shader = s_edges.shader;
	p->index_type = SG_INDEXTYPE_NONE;
	p->cull_mode = SG_CULLMODE_NONE;
	p->colors[0].pixel_format = SG_PIXELFORMAT_RGBA16F;
	p->color_count = 1;
	p->colors[0].blend.enabled = true;
	p->colors[0].blend.src_factor_rgb = SG_BLENDFACTOR_ONE;
	p->colors[0].blend.dst_factor_rgb = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
	p->colors[0].blend.op_rgb = SG_BLENDOP_ADD;
	p->colors[0].blend.src_factor_alpha = SG_BLENDFACTOR_ONE;
	p->colors[0].blend.dst_factor_alpha = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
	p->colors[0].blend.op_alpha = SG_BLENDOP_ADD;
	p->depth.compare = SG_COMPAREFUNC_GREATER_EQUAL;
	p->depth.write_enabled = false;
}

void InitEdges( void )
{
	if ( s_edges.ready )
	{
		return;
	}
	s_edges.shader = sg_make_shader( edge_edge_overlay_shader_desc( sg_query_backend() ) );

	// MSAA pipeline, depth format mirrors the scene target's MSAA depth
	// attachment (env.defaults.depth_format).
	sg_pipeline_desc msaaPdesc = { 0 };
	ApplyCommonPipelineState( &msaaPdesc );
	msaaPdesc.depth.pixel_format = GetSceneTargetDepthFormat();
	msaaPdesc.sample_count = SCENE_SAMPLE_COUNT;
	msaaPdesc.label = "edge_overlay_pipeline_msaa";
	s_edges.pipelineMsaa = sg_make_pipeline( &msaaPdesc );

	// Transparent pipeline, single-sample, depth matches the GTAO prepass attachment
	// (SG_PIXELFORMAT_DEPTH) that the post-resolve pass binds.
	sg_pipeline_desc xpPdesc = { 0 };
	ApplyCommonPipelineState( &xpPdesc );
	xpPdesc.depth.pixel_format = SG_PIXELFORMAT_DEPTH;
	xpPdesc.sample_count = 1;
	xpPdesc.label = "edge_overlay_pipeline_transparent";
	s_edges.pipelineTransparent = sg_make_pipeline( &xpPdesc );

	s_edges.ready = true;
}

void ShutdownEdges( void )
{
	if ( !s_edges.ready )
	{
		return;
	}
	sg_destroy_pipeline( s_edges.pipelineTransparent );
	s_edges.pipelineTransparent.id = 0;
	sg_destroy_pipeline( s_edges.pipelineMsaa );
	s_edges.pipelineMsaa.id = 0;
	sg_destroy_shader( s_edges.shader );
	s_edges.shader.id = 0;
	s_edges.ready = false;
}

static inline Vec4 Premultiply( Vec4 c )
{
	return MakeVec4( c.x * c.w, c.y * c.w, c.z * c.w, c.w );
}

// Per-batch policy: does the toggle want this batch drawn at all?
static bool IsBatchEnabled( const MeshEdgeBatch* b, const EdgeOverlayParams* p )
{
	switch ( b->kind )
	{
		case MESH_KIND_HULL:
			// Hull edges on every hull, opaque and transparent. Opaque hulls
			// pick up MSAA in the scene pass, transparent ones draw post-resolve.
			return p->showHulls;
		case MESH_KIND_MESH:
			return p->showMeshes;
		case MESH_KIND_HEIGHTFIELD:
			return p->showHeightfields;
		case MESH_KIND_UNKNOWN:
		default:
			return false;
	}
}

// Pack ub_frame + ub_pass + select pipeline + iterate batches. When
// `ArenaFilter` is ARENA_FILTER_BOTH, iterates every surviving batch
// and picks the matching instance view per batch (used by the MSAA path
// during the "transparent edges in MSAA pass" experiment). Otherwise restricts to
// the matching arena.
typedef enum ArenaFilter
{
	ARENA_FILTER_OPAQUE_ONLY = 0,
	ARENA_FILTER_TRANSPARENT_ONLY,
	ARENA_FILTER_BOTH,
} ArenaFilter;

static void SubmitBatches( int width, int height, const Mat4* view, const Mat4* proj, sg_pipeline pip, sg_view opaqueInstanceView,
						   sg_view transparentInstanceView, ArenaFilter arenaFilter, const EdgeOverlayParams* params )
{
	sg_apply_pipeline( pip );

	edge_ub_frame_t uf = { 0 };
	uf.view_proj = MulMM4( *proj, *view );
	uf.viewport =
		MakeVec4( (float)width, (float)height, width > 0 ? 1.0f / (float)width : 0.0f, height > 0 ? 1.0f / (float)height : 0.0f );
	sg_apply_uniforms( UB_edge_ub_frame, &SG_RANGE( uf ) );

	// Mesh and heightfield edges color by convexity (or collapse to flat when
	// convexity coloring is off). Hull edges ignore convexity and always draw
	// in the hard-coded hull color, so every class maps to it. Pass colors are
	// shared across a draw, so select per batch kind inside the loop.
	edge_ub_pass_t passMesh = { 0 };
	if ( params->showEdgeConvexity )
	{
		passMesh.convex_color = Premultiply( params->convexColor );
		passMesh.concave_color = Premultiply( params->concaveColor );
		passMesh.flat_color = Premultiply( params->flatColor );
	}
	else
	{
		Vec4 flat = Premultiply( params->flatColor );
		passMesh.convex_color = flat;
		passMesh.concave_color = flat;
		passMesh.flat_color = flat;
	}

	edge_ub_pass_t passHull = { 0 };

	Vec4 hullEdgeColor = { 0.5f, 0.5f, 0.5f, 0.5f };
	Vec4 hull = Premultiply( hullEdgeColor );
	passHull.convex_color = hull;
	passHull.concave_color = hull;
	passHull.flat_color = hull;

	sg_view currentEdgeView = { SG_INVALID_ID };
	sg_view currentInstView = { SG_INVALID_ID };
	int currentPassKind = -1; // -1 none, 0 mesh colors, 1 hull color

	int batchCount = GetMeshEdgeBatchCount();
	for ( int i = 0; i < batchCount; ++i )
	{
		MeshEdgeBatch b = GetMeshEdgeBatch( i );
		if ( arenaFilter == ARENA_FILTER_OPAQUE_ONLY && b.isTransparent )
		{
			continue;
		}
		if ( arenaFilter == ARENA_FILTER_TRANSPARENT_ONLY && b.isTransparent == false )
		{
			continue;
		}
		if ( b.instanceCount <= 0 )
		{
			continue;
		}
		if ( b.edgeCount <= 0 )
		{
			continue;
		}
		if ( IsBatchEnabled( &b, params ) == false )
		{
			continue;
		}

		int passKind = ( b.kind == MESH_KIND_HULL ) ? 1 : 0;
		if ( passKind != currentPassKind )
		{
			if ( passKind == 1 )
			{
				sg_apply_uniforms( UB_edge_ub_pass, &SG_RANGE( passHull ) );
			}
			else
			{
				sg_apply_uniforms( UB_edge_ub_pass, &SG_RANGE( passMesh ) );
			}
			currentPassKind = passKind;
		}

		sg_view instView = b.isTransparent ? transparentInstanceView : opaqueInstanceView;
		if ( instView.id != currentInstView.id || b.edgeStorageView.id != currentEdgeView.id )
		{
			sg_bindings bnd = { 0 };
			bnd.views[VIEW_edge_geom_instances] = instView;
			bnd.views[VIEW_edge_edge_endpoints] = b.edgeStorageView;
			sg_apply_bindings( &bnd );
			currentInstView = instView;
			currentEdgeView = b.edgeStorageView;
		}

		edge_ub_draw_t ud = { 0 };
		ud.inst_base[0] = b.firstInstance;
		ud.edge_params = MakeVec4( params->thicknessPx, params->zBias, 0.0f, 0.0f );
		sg_apply_uniforms( UB_edge_ub_draw, &SG_RANGE( ud ) );

		sg_draw( 0, 6 * b.edgeCount, b.instanceCount );
	}
}

// Cheap pre-walk to avoid issuing pipeline / bindings / uniforms when no
// batch survives the filter.
static bool DoesAnyBatchSurvive( ArenaFilter arenaFilter, const EdgeOverlayParams* params )
{
	const int batchCount = GetMeshEdgeBatchCount();
	for ( int i = 0; i < batchCount; ++i )
	{
		const MeshEdgeBatch b = GetMeshEdgeBatch( i );
		if ( arenaFilter == ARENA_FILTER_OPAQUE_ONLY && b.isTransparent )
			continue;
		if ( arenaFilter == ARENA_FILTER_TRANSPARENT_ONLY && !b.isTransparent )
			continue;
		if ( b.instanceCount <= 0 )
			continue;
		if ( b.edgeCount <= 0 )
			continue;
		if ( IsBatchEnabled( &b, params ) )
			return true;
	}
	return false;
}

// When true, the MSAA path draws BOTH opaque AND transparent edge batches, so
// transparent-hull edges also pick up 4x MSAA at the cost of the translucent body
// blending over them post-resolve.
// Result: rendering transparent hulls after their edges yields jaggies
#define EDGES_MSAA_INCLUDES_TRANSPARENT 0

// Mirror experiment: route ALL edge batches (opaque-arena meshes /
// heightfields as well as the transparent-arena hulls) through the
// post-resolve transparent path and skip the MSAA submission entirely.
// Edges lose 4x MSAA (they fall back to fwidth-based fragment AA) but
// they're guaranteed crisp on top of everything, including transparent
// hulls.
// Result: rendering edges for opaque meshes after MSAA resolve yields jaggies
// probably due to inconsistent AA on the edges and the MSAA opaque geometry.
#define EDGES_TRANSPARENT_INCLUDES_OPAQUE 0

// Conclusion: the split edge rendering is worth it. Must keep.

#if EDGES_MSAA_INCLUDES_TRANSPARENT && EDGES_TRANSPARENT_INCLUDES_OPAQUE
#error "EDGES_MSAA_INCLUDES_TRANSPARENT and EDGES_TRANSPARENT_INCLUDES_OPAQUE are mutually exclusive"
#endif

void RenderEdgesInMSAA( int width, int height, const Mat4* view, const Mat4* proj, sg_view opaqueInstanceView,
						sg_view transparentInstanceView, const EdgeOverlayParams* params )
{
	if ( !s_edges.ready || !params || !params->enabled )
	{
		return;
	}

	// Experiment drawing all edges after transparent shapes with no MSAA
	if ( EDGES_TRANSPARENT_INCLUDES_OPAQUE )
	{
		return;
	}

	const ArenaFilter filter = EDGES_MSAA_INCLUDES_TRANSPARENT ? ARENA_FILTER_BOTH : ARENA_FILTER_OPAQUE_ONLY;
	if ( !DoesAnyBatchSurvive( filter, params ) )
	{
		return;
	}
	SubmitBatches( width, height, view, proj, s_edges.pipelineMsaa, opaqueInstanceView, transparentInstanceView, filter, params );
}

void RenderEdgesPostTransparent( int width, int height, const Mat4* view, const Mat4* proj, sg_view colorAttachView,
								 sg_view depthAttachView, sg_view opaqueInstanceView, sg_view transparentInstanceView,
								 const EdgeOverlayParams* params )
{
	if ( !s_edges.ready || !params || !params->enabled )
	{
		return;
	}

	// Experiment drawing all edges in the MSAA scene, even for edges on transparent hulls.
	if ( EDGES_MSAA_INCLUDES_TRANSPARENT )
	{
		return;
	}
	const ArenaFilter filter = EDGES_TRANSPARENT_INCLUDES_OPAQUE ? ARENA_FILTER_BOTH : ARENA_FILTER_TRANSPARENT_ONLY;
	if ( !DoesAnyBatchSurvive( filter, params ) )
	{
		return;
	}

	sg_pass pass = { 0 };
	pass.action.colors[0].load_action = SG_LOADACTION_LOAD;
	pass.action.colors[0].store_action = SG_STOREACTION_STORE;
	pass.action.depth.load_action = SG_LOADACTION_LOAD;
	pass.action.depth.store_action = SG_STOREACTION_DONTCARE;
	pass.attachments.colors[0] = colorAttachView;
	pass.attachments.depth_stencil = depthAttachView;
	pass.label = "edge_overlay_pass_transparent";
	sg_push_debug_group( "edges_transparent" );
	sg_begin_pass( &pass );
	sg_apply_viewport( 0, 0, width, height, true );

	SubmitBatches( width, height, view, proj, s_edges.pipelineTransparent, opaqueInstanceView, transparentInstanceView, filter,
				   params );

	sg_end_pass();
	sg_pop_debug_group();
}
