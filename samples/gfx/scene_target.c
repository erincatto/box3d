// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/scene_target.h"

#include <stdio.h>

#define SCENE_COLOR_FORMAT SG_PIXELFORMAT_RGBA16F

// At SCENE_SAMPLE_COUNT > 1: colorMsaaImage holds the MSAA color, sokol_gfx
// auto-resolves into colorResolveImage at end-of-pass, and the tone map +
// transparent passes consume the resolve image.
// At SCENE_SAMPLE_COUNT == 1: colorResolveImage is unused (id = 0). The
// "MSAA" image is itself 1x and serves as both the scene_hdr color
// attachment AND the sampled scene color for downstream passes. The
// public sceneTargetResolve* accessors transparently point at it.
static struct
{
	sg_pixel_format depthFormat;
	int width;
	int height;

	sg_image colorMsaaImage;	// sample_count = SCENE_SAMPLE_COUNT
	sg_image depthMsaaImage;	// sample_count = SCENE_SAMPLE_COUNT
	sg_image colorResolveImage; // sample_count = 1, unused when SCENE_SAMPLE_COUNT == 1

	sg_view colorMsaaView;				 // color_attachment over colorMsaaImage
	sg_view depthMsaaView;				 // depth_stencil_attachment over depthMsaaImage
	sg_view colorResolveAttachView;		 // resolve_attachment, unused when SCENE_SAMPLE_COUNT == 1
	sg_view colorResolveColorAttachView; // color_attachment over resolved scene color (transparent pass)
	sg_view colorResolveSampleView;		 // texture view for sampling in tonemap

	bool ready;
} s_st;

static void destroyTargets( void )
{
	if ( s_st.colorResolveSampleView.id )
	{
		sg_destroy_view( s_st.colorResolveSampleView );
		s_st.colorResolveSampleView.id = 0;
	}
	if ( s_st.colorResolveColorAttachView.id )
	{
		sg_destroy_view( s_st.colorResolveColorAttachView );
		s_st.colorResolveColorAttachView.id = 0;
	}
	if ( s_st.colorResolveAttachView.id )
	{
		sg_destroy_view( s_st.colorResolveAttachView );
		s_st.colorResolveAttachView.id = 0;
	}
	if ( s_st.depthMsaaView.id )
	{
		sg_destroy_view( s_st.depthMsaaView );
		s_st.depthMsaaView.id = 0;
	}
	if ( s_st.colorMsaaView.id )
	{
		sg_destroy_view( s_st.colorMsaaView );
		s_st.colorMsaaView.id = 0;
	}
	if ( s_st.colorResolveImage.id )
	{
		sg_destroy_image( s_st.colorResolveImage );
		s_st.colorResolveImage.id = 0;
	}
	if ( s_st.depthMsaaImage.id )
	{
		sg_destroy_image( s_st.depthMsaaImage );
		s_st.depthMsaaImage.id = 0;
	}
	if ( s_st.colorMsaaImage.id )
	{
		sg_destroy_image( s_st.colorMsaaImage );
		s_st.colorMsaaImage.id = 0;
	}
}

void InitSceneTarget( const sg_environment* env )
{
	s_st.depthFormat = env->defaults.depth_format;
	s_st.width = 0;
	s_st.height = 0;
	s_st.ready = true;
}

void ShutdownSceneTarget( void )
{
	if ( !s_st.ready )
	{
		return;
	}
	destroyTargets();
	s_st.width = 0;
	s_st.height = 0;
	s_st.ready = false;
}

void ResizeSceneTarget( int width, int height )
{
	if ( !s_st.ready || width <= 0 || height <= 0 )
	{
		return;
	}
	if ( width == s_st.width && height == s_st.height && s_st.colorMsaaImage.id )
	{
		return;
	}

	destroyTargets();
	s_st.width = width;
	s_st.height = height;

	sg_image_desc cdesc = { 0 };
	cdesc.usage.color_attachment = true;
	cdesc.width = width;
	cdesc.height = height;
	cdesc.pixel_format = SCENE_COLOR_FORMAT;
	cdesc.sample_count = SCENE_SAMPLE_COUNT;
	cdesc.label = "scene_color_msaa";
	s_st.colorMsaaImage = sg_make_image( &cdesc );

	sg_image_desc ddesc = { 0 };
	ddesc.usage.depth_stencil_attachment = true;
	ddesc.width = width;
	ddesc.height = height;
	ddesc.pixel_format = s_st.depthFormat;
	ddesc.sample_count = SCENE_SAMPLE_COUNT;
	ddesc.label = "scene_depth_msaa";
	s_st.depthMsaaImage = sg_make_image( &ddesc );

	if ( SCENE_SAMPLE_COUNT > 1 )
	{
		sg_image_desc rdesc = { 0 };
		rdesc.usage.resolve_attachment = true;
		// Also bind as a regular color_attachment for the transparent pass
		// that runs after the opaque MSAA pass has resolved.
		rdesc.usage.color_attachment = true;
		rdesc.width = width;
		rdesc.height = height;
		rdesc.pixel_format = SCENE_COLOR_FORMAT;
		rdesc.sample_count = 1;
		rdesc.label = "scene_color_resolve";
		s_st.colorResolveImage = sg_make_image( &rdesc );
	}

	sg_view_desc cmvdesc = { 0 };
	cmvdesc.color_attachment.image = s_st.colorMsaaImage;
	cmvdesc.label = "scene_color_msaa_attach";
	s_st.colorMsaaView = sg_make_view( &cmvdesc );

	sg_view_desc dmvdesc = { 0 };
	dmvdesc.depth_stencil_attachment.image = s_st.depthMsaaImage;
	dmvdesc.label = "scene_depth_msaa_attach";
	s_st.depthMsaaView = sg_make_view( &dmvdesc );

	if ( SCENE_SAMPLE_COUNT > 1 )
	{
		sg_view_desc cravdesc = { 0 };
		cravdesc.resolve_attachment.image = s_st.colorResolveImage;
		cravdesc.label = "scene_color_resolve_attach";
		s_st.colorResolveAttachView = sg_make_view( &cravdesc );

		sg_view_desc crcadesc = { 0 };
		crcadesc.color_attachment.image = s_st.colorResolveImage;
		crcadesc.label = "scene_color_resolve_color_attach";
		s_st.colorResolveColorAttachView = sg_make_view( &crcadesc );

		sg_view_desc crsvdesc = { 0 };
		crsvdesc.texture.image = s_st.colorResolveImage;
		crsvdesc.label = "scene_color_resolve_sample";
		s_st.colorResolveSampleView = sg_make_view( &crsvdesc );
	}
	else
	{
		// 1x: there is no resolve. The scene color image IS the 1x image
		// the transparent pass and tonemap want. Reuse it via the same
		// public accessors so callers stay backend-agnostic.
		sg_view_desc crcadesc = { 0 };
		crcadesc.color_attachment.image = s_st.colorMsaaImage;
		crcadesc.label = "scene_color_attach_1x";
		s_st.colorResolveColorAttachView = sg_make_view( &crcadesc );

		sg_view_desc crsvdesc = { 0 };
		crsvdesc.texture.image = s_st.colorMsaaImage;
		crsvdesc.label = "scene_color_sample_1x";
		s_st.colorResolveSampleView = sg_make_view( &crsvdesc );
	}
}

sg_attachments GetSceneTargetAttachments( void )
{
	sg_attachments a = { 0 };
	a.colors[0] = s_st.colorMsaaView;
	// resolves[0] paired with a 1x color attachment fails sokol_gfx
	// validation. At SCENE_SAMPLE_COUNT == 1 the colorMsaaView is itself
	// the single-sample target, no resolve step needed.
	if ( SCENE_SAMPLE_COUNT > 1 )
	{
		a.resolves[0] = s_st.colorResolveAttachView;
	}
	a.depth_stencil = s_st.depthMsaaView;
	return a;
}

sg_view GetSceneTargetResolveSampleView( void )
{
	return s_st.colorResolveSampleView;
}

sg_view GetSceneTargetResolveColorAttachView( void )
{
	return s_st.colorResolveColorAttachView;
}

sg_pixel_format GetSceneTargetDepthFormat( void )
{
	return s_st.depthFormat;
}
