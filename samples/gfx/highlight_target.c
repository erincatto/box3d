// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/highlight_target.h"

static struct
{
	int width;
	int height;
	sg_image image;
	sg_view attachView;
	sg_view sampleView;
	bool ready;
} s_ht;

static void destroyTargets( void )
{
	if ( s_ht.sampleView.id )
	{
		sg_destroy_view( s_ht.sampleView );
		s_ht.sampleView.id = 0;
	}
	if ( s_ht.attachView.id )
	{
		sg_destroy_view( s_ht.attachView );
		s_ht.attachView.id = 0;
	}
	if ( s_ht.image.id )
	{
		sg_destroy_image( s_ht.image );
		s_ht.image.id = 0;
	}
}

void InitHighlightTarget( void )
{
	s_ht.width = 0;
	s_ht.height = 0;
	s_ht.ready = true;
}

void ShutdownHighlightTarget( void )
{
	if ( !s_ht.ready )
		return;
	destroyTargets();
	s_ht.width = 0;
	s_ht.height = 0;
	s_ht.ready = false;
}

void ResizeHighlightTarget( int width, int height )
{
	if ( !s_ht.ready || width <= 0 || height <= 0 )
		return;
	if ( width == s_ht.width && height == s_ht.height && s_ht.image.id )
		return;

	destroyTargets();
	s_ht.width = width;
	s_ht.height = height;

	sg_image_desc idesc = { 0 };
	idesc.usage.color_attachment = true;
	idesc.width = width;
	idesc.height = height;
	idesc.pixel_format = SG_PIXELFORMAT_R8;
	idesc.sample_count = 1;
	idesc.label = "highlight_mask";
	s_ht.image = sg_make_image( &idesc );

	sg_view_desc avdesc = { 0 };
	avdesc.color_attachment.image = s_ht.image;
	avdesc.label = "highlight_mask_attach";
	s_ht.attachView = sg_make_view( &avdesc );

	sg_view_desc svdesc = { 0 };
	svdesc.texture.image = s_ht.image;
	svdesc.label = "highlight_mask_sample";
	s_ht.sampleView = sg_make_view( &svdesc );
}

sg_view GetHighlightTargetMaskAttachView( void )
{
	return s_ht.attachView;
}
sg_view GetHighlightTargetMaskSampleView( void )
{
	return s_ht.sampleView;
}
