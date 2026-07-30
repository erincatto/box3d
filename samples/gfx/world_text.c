// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "gfx/world_text.h"

#include "gfx/fonts.h"
#include "gfx/text.h"
#include "world_text.glsl.h"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#include <math.h>
#include <sokol_app.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Printable ASCII. Labels are numbers, ids and short type names, so this
// range covers everything the samples emit. Anything outside draws as '?'.
#define FIRST_CHAR 32
#define CHAR_COUNT 95

// Distance range carried around each outline, in pixels. The halo fades inside
// it, so this bounds how far the fade can reach.
#define SDF_PADDING 4
#define SDF_ONEDGE 128

// One transparent column and row between packed glyphs so a bilinear tap at
// a glyph edge cannot reach its neighbour.
#define GLYPH_GAP 1

#define ATLAS_DIM 512

// 1024 labels of 32 characters, matching text.c's label cap. Overflow drops
// glyphs rather than growing.
#define MAX_GLYPHS 32768

// The swap chain (color + depth) and the color-only offscreen test target.
// Mirrors overlay.c's cache.
#define TEXT_MAX_PIPELINES 2

// Pen-relative glyph box and atlas window. The box is in pixels and whole, so a
// quad placed at a whole pixel covers its atlas cell one texel to one pixel. The
// advance stays fractional: rounding every advance instead of every origin would
// drift the whole word.
typedef struct Glyph
{
	float x0, y0, x1, y1; // origin at the top left of the line box, Y down
	float u0, v0, u1, v1;
	float advance;
} Glyph;

static struct
{
	Glyph glyphs[CHAR_COUNT];

	// Built from scratch inside each submit rather than accumulated across the
	// frame, so a flat array beats the arena the overlay needs.
	world_text_glyph_instance_t* instances;
	int glyphCount;

	sg_shader shader;
	sg_image atlas;
	sg_view atlasView;
	sg_sampler atlasSampler;

	sg_buffer instBuf;
	sg_view instView;

	struct
	{
		sg_pixel_format colorFormat;
		sg_pixel_format depthFormat;
		sg_pipeline pipeline;
	} pipelines[TEXT_MAX_PIPELINES];
	int pipelineCount;

	float pixelHeight;
	float haloPixels;

	bool ready;
} s_text;

// Rasterize printable ASCII into one RG8 atlas and fill the glyph table. R is a
// coverage bitmap, G a distance field, both from the same outlines at the same
// scale so they agree on where the edge is.
//
// The bake tracks the size labels draw at, which is what buys the glyph body its
// crispness: a coverage bitmap sampled one texel to one pixel is exactly what a
// UI font renderer puts on screen, and no distance field reproduces that at 14
// pixels because it cannot grid fit. The halo has no such constraint, so it
// keeps the field, which is smooth at any size and gives a fade for free.
//
// Packing is a shelf: glyphs from a single bake size have near-uniform heights,
// so rows waste little and this stands in for a real rect packer.
static bool BakeAtlas( void )
{
	int fontSize = 0;
	const unsigned char* fontData = GetFontBytes( &fontSize );

	stbtt_fontinfo font;
	if ( stbtt_InitFont( &font, fontData, stbtt_GetFontOffsetForIndex( fontData, 0 ) ) == 0 )
	{
		fprintf( stderr, "[world_text] stbtt_InitFont failed on %d embedded bytes of %s\n", fontSize, GetFontName() );
		return false;
	}

	// ScaleForPixelHeight makes ascent-to-descent exactly the label height.
	const float scale = stbtt_ScaleForPixelHeight( &font, s_text.pixelHeight );
	int ascent, descent, lineGap;
	stbtt_GetFontVMetrics( &font, &ascent, &descent, &lineGap );

	// Whole pixels, so a glyph box stays whole and lands on the pixel grid. A
	// fractional baseline would put every horizontal stroke between two rows.
	const float ascentPixels = floorf( (float)ascent * scale + 0.5f );

	unsigned char* atlas = (unsigned char*)calloc( (size_t)ATLAS_DIM * ATLAS_DIM * 2, 1 );
	if ( atlas == NULL )
	{
		return false;
	}

	const float invDim = 1.0f / (float)ATLAS_DIM;
	const float distanceScale = (float)SDF_ONEDGE / (float)SDF_PADDING;

	int penX = 0;
	int penY = 0;
	int shelfHeight = 0;
	bool ok = true;

	for ( int i = 0; i < CHAR_COUNT; ++i )
	{
		const int codepoint = FIRST_CHAR + i;
		Glyph* glyph = &s_text.glyphs[i];
		memset( glyph, 0, sizeof( *glyph ) );

		int advance, leftBearing;
		stbtt_GetCodepointHMetrics( &font, codepoint, &advance, &leftBearing );
		glyph->advance = (float)advance * scale;

		int w = 0, h = 0, xoff = 0, yoff = 0;
		unsigned char* sdf =
			stbtt_GetCodepointSDF( &font, scale, codepoint, SDF_PADDING, SDF_ONEDGE, distanceScale, &w, &h, &xoff, &yoff );
		if ( sdf == NULL )
		{
			// Space and friends carry an advance but no outline.
			continue;
		}

		if ( penX + w > ATLAS_DIM )
		{
			penX = 0;
			penY += shelfHeight;
			shelfHeight = 0;
		}
		if ( penY + h > ATLAS_DIM )
		{
			fprintf( stderr, "[world_text] atlas full at codepoint %d, raise ATLAS_DIM or lower the label size\n", codepoint );
			stbtt_FreeSDF( sdf, NULL );
			ok = false;
			break;
		}

		for ( int row = 0; row < h; ++row )
		{
			unsigned char* dst = atlas + ( (size_t)( penY + row ) * ATLAS_DIM + penX ) * 2;
			const unsigned char* src = sdf + (size_t)row * w;
			for ( int col = 0; col < w; ++col )
			{
				dst[col * 2 + 1] = src[col];
			}
		}

		// The coverage bitmap shares the cell. stb derives both boxes from the
		// same glyph box and only the field pads it, so the offsets differ by
		// exactly the padding, but take it from the reported corners anyway.
		int bw = 0, bh = 0, bxoff = 0, byoff = 0;
		unsigned char* bitmap = stbtt_GetCodepointBitmap( &font, scale, scale, codepoint, &bw, &bh, &bxoff, &byoff );
		if ( bitmap != NULL )
		{
			const int insetX = bxoff - xoff;
			const int insetY = byoff - yoff;
			for ( int row = 0; row < bh; ++row )
			{
				const int y = insetY + row;
				if ( y < 0 || y >= h )
				{
					continue;
				}
				unsigned char* dst = atlas + ( (size_t)( penY + y ) * ATLAS_DIM + penX ) * 2;
				const unsigned char* src = bitmap + (size_t)row * bw;
				for ( int col = 0; col < bw; ++col )
				{
					const int x = insetX + col;
					if ( x >= 0 && x < w )
					{
						dst[x * 2] = src[col];
					}
				}
			}
			stbtt_FreeBitmap( bitmap, NULL );
		}

		// stb reports the box against the baseline with Y down. Shift it to
		// the top of the line box so an anchor lands where ImGui put it, and
		// existing call sites that pad with leading spaces still read right.
		glyph->x0 = (float)xoff;
		glyph->y0 = (float)yoff + ascentPixels;
		glyph->x1 = (float)( xoff + w );
		glyph->y1 = (float)yoff + ascentPixels + (float)h;

		glyph->u0 = (float)penX * invDim;
		glyph->v0 = (float)penY * invDim;
		glyph->u1 = (float)( penX + w ) * invDim;
		glyph->v1 = (float)( penY + h ) * invDim;

		penX += w + GLYPH_GAP;
		if ( h + GLYPH_GAP > shelfHeight )
		{
			shelfHeight = h + GLYPH_GAP;
		}

		stbtt_FreeSDF( sdf, NULL );
	}

	if ( ok )
	{
		// A resize rebakes, so drop whatever the last size produced.
		if ( s_text.atlasView.id != 0 )
		{
			sg_destroy_view( s_text.atlasView );
			s_text.atlasView.id = 0;
		}
		if ( s_text.atlas.id != 0 )
		{
			sg_destroy_image( s_text.atlas );
			s_text.atlas.id = 0;
		}

		sg_image_desc imgDesc = { 0 };
		imgDesc.width = ATLAS_DIM;
		imgDesc.height = ATLAS_DIM;
		imgDesc.num_mipmaps = 1;
		imgDesc.pixel_format = SG_PIXELFORMAT_RG8;
		imgDesc.data.mip_levels[0].ptr = atlas;
		imgDesc.data.mip_levels[0].size = (size_t)ATLAS_DIM * ATLAS_DIM * 2;
		imgDesc.label = "world_text_atlas";
		s_text.atlas = sg_make_image( &imgDesc );

		sg_view_desc viewDesc = { 0 };
		viewDesc.texture.image = s_text.atlas;
		viewDesc.label = "world_text_atlas_view";
		s_text.atlasView = sg_make_view( &viewDesc );

		if ( s_text.atlasSampler.id == 0 )
		{
			// Nearest would do for the body, which is sampled one texel to one
			// pixel, but the field wants filtering at the cell edges.
			sg_sampler_desc sampDesc = { 0 };
			sampDesc.min_filter = SG_FILTER_LINEAR;
			sampDesc.mag_filter = SG_FILTER_LINEAR;
			sampDesc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
			sampDesc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
			sampDesc.label = "world_text_atlas_sampler";
			s_text.atlasSampler = sg_make_sampler( &sampDesc );
		}
	}

	free( atlas );
	return ok;
}

// Lazily create (and cache) the pipeline for one output format pair, same
// scheme and same blend state as overlay.c. Depth compare is ALWAYS with
// writes off: a label is meant to be read, so it draws over whatever the
// scene put in front of it.
static bool EnsurePipeline( sg_pixel_format colorFormat, sg_pixel_format depthFormat, sg_pipeline* outPipeline )
{
	for ( int i = 0; i < s_text.pipelineCount; ++i )
	{
		if ( s_text.pipelines[i].colorFormat == colorFormat && s_text.pipelines[i].depthFormat == depthFormat )
		{
			*outPipeline = s_text.pipelines[i].pipeline;
			return true;
		}
	}
	if ( s_text.pipelineCount >= TEXT_MAX_PIPELINES )
	{
		fprintf( stderr, "[world_text] pipeline cache exhausted (max %d format pairs)\n", TEXT_MAX_PIPELINES );
		return false;
	}

	sg_pipeline_desc pdesc = { 0 };
	pdesc.shader = s_text.shader;
	// No VBO / no IBO, the VS expands six vertices per instance.
	pdesc.index_type = SG_INDEXTYPE_NONE;
	pdesc.cull_mode = SG_CULLMODE_NONE;
	pdesc.colors[0].pixel_format = colorFormat;
	pdesc.color_count = 1;
	// Premultiplied-alpha blend. The FS emits premultiplied color, which is
	// also what makes the halo work: alpha without color resolves to black.
	pdesc.colors[0].blend.enabled = true;
	pdesc.colors[0].blend.src_factor_rgb = SG_BLENDFACTOR_ONE;
	pdesc.colors[0].blend.dst_factor_rgb = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
	pdesc.colors[0].blend.op_rgb = SG_BLENDOP_ADD;
	pdesc.colors[0].blend.src_factor_alpha = SG_BLENDFACTOR_ONE;
	pdesc.colors[0].blend.dst_factor_alpha = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
	pdesc.colors[0].blend.op_alpha = SG_BLENDOP_ADD;
	pdesc.depth.pixel_format = depthFormat;
	pdesc.depth.compare = SG_COMPAREFUNC_ALWAYS;
	pdesc.depth.write_enabled = false;
	pdesc.sample_count = 1;
	pdesc.label = "world_text_pipeline";
	const sg_pipeline pipeline = sg_make_pipeline( &pdesc );

	s_text.pipelines[s_text.pipelineCount].colorFormat = colorFormat;
	s_text.pipelines[s_text.pipelineCount].depthFormat = depthFormat;
	s_text.pipelines[s_text.pipelineCount].pipeline = pipeline;
	++s_text.pipelineCount;

	*outPipeline = pipeline;
	return true;
}

void WorldTextInit( void )
{
	if ( s_text.ready )
	{
		return;
	}

	s_text.pixelHeight = floorf( 13.0f * sapp_dpi_scale() );
	s_text.haloPixels = 2.0f;

	if ( BakeAtlas() == false )
	{
		return;
	}

	s_text.instances = (world_text_glyph_instance_t*)malloc( (size_t)MAX_GLYPHS * sizeof( world_text_glyph_instance_t ) );
	if ( s_text.instances == NULL )
	{
		fprintf( stderr, "[world_text] failed to allocate glyph instances\n" );

		// Clean up
		sg_destroy_sampler( s_text.atlasSampler );
		s_text.atlasSampler.id = 0;
		sg_destroy_view( s_text.atlasView );
		s_text.atlasView.id = 0;
		sg_destroy_image( s_text.atlas );
		s_text.atlas.id = 0;

		return;
	}

	sg_buffer_desc bufDesc = { 0 };
	bufDesc.usage.storage_buffer = true;
	bufDesc.usage.stream_update = true;
	bufDesc.size = (size_t)MAX_GLYPHS * sizeof( world_text_glyph_instance_t );
	bufDesc.label = "world_text_glyph_instances";
	s_text.instBuf = sg_make_buffer( &bufDesc );

	sg_view_desc instViewDesc = { 0 };
	instViewDesc.storage_buffer.buffer = s_text.instBuf;
	instViewDesc.label = "world_text_glyph_instances_view";
	s_text.instView = sg_make_view( &instViewDesc );

	s_text.shader = sg_make_shader( world_text_prog_shader_desc( sg_query_backend() ) );

	s_text.ready = true;
}

void WorldTextShutdown( void )
{
	if ( !s_text.ready )
	{
		return;
	}
	for ( int i = 0; i < s_text.pipelineCount; ++i )
	{
		sg_destroy_pipeline( s_text.pipelines[i].pipeline );
		s_text.pipelines[i].pipeline.id = 0;
		s_text.pipelines[i].colorFormat = SG_PIXELFORMAT_NONE;
		s_text.pipelines[i].depthFormat = SG_PIXELFORMAT_NONE;
	}
	s_text.pipelineCount = 0;
	sg_destroy_shader( s_text.shader );
	s_text.shader.id = 0;
	sg_destroy_view( s_text.instView );
	s_text.instView.id = 0;
	sg_destroy_buffer( s_text.instBuf );
	s_text.instBuf.id = 0;
	sg_destroy_sampler( s_text.atlasSampler );
	s_text.atlasSampler.id = 0;
	sg_destroy_view( s_text.atlasView );
	s_text.atlasView.id = 0;
	sg_destroy_image( s_text.atlas );
	s_text.atlas.id = 0;
	free( s_text.instances );
	s_text.instances = NULL;
	s_text.glyphCount = 0;
	s_text.ready = false;
}

// Walk this frame's labels and lay each one out into glyph quads. Advances
// accumulate along the pen, so a label's characters share an anchor and differ
// only in their pen offset, which keeps the whole string bill-boarding as one.
// The pen accumulates fractionally and each origin rounds off it, so spacing
// stays true across a word while every glyph still starts on a whole pixel.
static void BuildGlyphs( void )
{
	s_text.glyphCount = 0;

	const int count = GetTextCount();

	for ( int i = 0; i < count; ++i )
	{
		const TextEntry* entry = GetTextAt( i );
		if ( entry->space != TEXT_SPACE_WORLD )
		{
			continue;
		}

		const Vec4 color = entry->color;
		float pen = 0.0f;

		for ( const char* p = entry->text; *p != '\0'; ++p )
		{
			int index = (unsigned char)*p - FIRST_CHAR;
			if ( index < 0 || index >= CHAR_COUNT )
			{
				index = '?' - FIRST_CHAR;
			}
			const Glyph* glyph = &s_text.glyphs[index];

			// Space has an advance but no box, and nothing else should reach
			// the rasterizer with zero area either.
			if ( glyph->x1 > glyph->x0 && glyph->y1 > glyph->y0 )
			{
				if ( s_text.glyphCount >= MAX_GLYPHS )
				{
					return;
				}
				const float originX = floorf( pen + 0.5f );

				world_text_glyph_instance_t* inst = &s_text.instances[s_text.glyphCount];
				inst->anchor = MakeVec4( entry->worldPos.x, entry->worldPos.y, entry->worldPos.z, 0.0f );
				inst->rect = MakeVec4( originX + glyph->x0, glyph->y0, glyph->x1 - glyph->x0, glyph->y1 - glyph->y0 );
				inst->uv = MakeVec4( glyph->u0, glyph->v0, glyph->u1, glyph->v1 );
				inst->color = color;
				s_text.glyphCount++;
			}

			pen += glyph->advance;
		}
	}
}

void WorldTextSubmit( int width, int height, const Mat4* view, const Mat4* viewInv, const Mat4* proj, const Mat4* projInv,
					  b3Vec3 cameraPos, float time, sg_pixel_format colorFormat, sg_pixel_format depthFormat )
{
	if ( !s_text.ready )
	{
		return;
	}

	BuildGlyphs();
	if ( s_text.glyphCount == 0 )
	{
		return;
	}

	sg_pipeline pipeline = { 0 };
	if ( !EnsurePipeline( colorFormat, depthFormat, &pipeline ) )
	{
		return;
	}

	const Mat4 view_proj = MulMM4( *proj, *view );
	// inv(P*V) = inv(V)*inv(P), see overlay.c for why both inverses arrive
	// prebuilt rather than being inverted here.
	const Mat4 inv_view_proj = MulMM4( *viewInv, *projInv );
	const float w = (float)width;
	const float h = (float)height;

	world_text_ub_frame_t ub = { 0 };
	ub.view = *view;
	ub.proj = *proj;
	ub.view_proj = view_proj;
	ub.inv_view_proj = inv_view_proj;
	ub.camera_pos = MakeVec4( cameraPos.x, cameraPos.y, cameraPos.z, time );
	ub.viewport = MakeVec4( w, h, w > 0.0f ? 1.0f / w : 0.0f, h > 0.0f ? 1.0f / h : 0.0f );

	// Halo width in pixels plus the distance one pixel spans, which the bake fixes
	// since the field is rasterized at the size it draws at. The field saturates
	// SDF_PADDING pixels out, so that is the ceiling on the fade.
	world_text_ub_text_style_t styleUb = { 0 };
	const float distancePerPixel = ( (float)SDF_ONEDGE / (float)SDF_PADDING ) / 255.0f;
	styleUb.style_params = MakeVec4( s_text.haloPixels, (float)SDF_ONEDGE / 255.0f, distancePerPixel, 0.0f );

	// .w is strength at the glyph edge
	styleUb.halo_params = MakeVec4( 0.2f, 0.2f, 0.2f, 0.2f );

	sg_range instanceRange;
	instanceRange.ptr = s_text.instances;
	instanceRange.size = (size_t)s_text.glyphCount * sizeof( world_text_glyph_instance_t );
	sg_update_buffer( s_text.instBuf, &instanceRange );

	sg_apply_pipeline( pipeline );
	sg_bindings bindings = { 0 };
	bindings.views[VIEW_world_text_glyph_instances] = s_text.instView;
	bindings.views[VIEW_world_text_tex_sdf] = s_text.atlasView;
	bindings.samplers[SMP_world_text_smp_sdf] = s_text.atlasSampler;
	sg_apply_bindings( &bindings );
	sg_apply_uniforms( UB_world_text_ub_frame, &SG_RANGE( ub ) );
	sg_apply_uniforms( UB_world_text_ub_text_style, &SG_RANGE( styleUb ) );
	sg_draw( 0, 6, s_text.glyphCount );
}
