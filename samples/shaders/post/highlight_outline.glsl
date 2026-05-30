// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Highlight outline post-pass.
//
// Fullscreen triangle (no vbuf). Reads the R8 UNORM highlight mask written by
// the mask_sphere/mask_capsule/mask_hull pipelines. Each non-zero pixel is
// inside a highlighted shape (encoded 0.5 = hover, 1.0 = select). The shader
// detects silhouette pixels (center is empty but a neighbor is filled)
// and emits the appropriate outline color premultiplied for blend-over the
// resolved scene color.
//
// Two ring radii tunable from C: hoverThicknessPx, selectThicknessPx.
// Select takes priority over hover when both rings catch a neighbor.

#pragma sokol @module highlight_outline

#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs
// Fullscreen triangle by gl_VertexIndex, 3 verts covering NDC
// [-1, 3] x [-1, 3], clipped to the [-1,1] viewport. No vertex buffer.
void main()
{
	vec2 p = vec2( ( gl_VertexIndex == 1 ) ? 3.0 : -1.0, ( gl_VertexIndex == 2 ) ? 3.0 : -1.0 );
	gl_Position = vec4( p, 0.0, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs

layout( binding = 0 ) uniform ub_pass
{
	vec4 hover_color;  // premultiplied linear RGBA
	vec4 select_color; // premultiplied linear RGBA
	vec4 thickness_px; // .x = hover thickness, .y = select thickness, .zw unused
};

layout( binding = 0 ) uniform texture2D tex_mask;
layout( binding = 0 ) uniform sampler smp_mask;

layout( location = 0 ) out vec4 out_color;

// Discrete-kind threshold. 0.5 encodes hover, 1.0 encodes select. Threshold
// at 0.75 splits them. Values < 0.25 are "empty".
const float KIND_EMPTY_THRESHOLD = 0.25;
const float KIND_SELECT_THRESHOLD = 0.75;

void main()
{
	ivec2 px = ivec2( gl_FragCoord.xy );
	ivec2 sz = textureSize( sampler2D( tex_mask, smp_mask ), 0 );

	// Inside a highlighted shape, keep the scene color (output 0 alpha).
	float center = texelFetch( sampler2D( tex_mask, smp_mask ), px, 0 ).x;
	if ( center > KIND_EMPTY_THRESHOLD )
	{
		out_color = vec4( 0.0 );
		return;
	}

	int hoverR = clamp( int( thickness_px.x + 0.5 ), 1, 4 );
	int selectR = clamp( int( thickness_px.y + 0.5 ), 1, 4 );
	int maxR = max( hoverR, selectR );

	// Track the strongest neighbor kind found, and the smallest radius at
	// which we found a select-kind / hover-kind neighbor. The outline rules:
	// * a select-kind pixel anywhere within selectR -> draw select
	// * else a hover-kind pixel within hoverR -> draw hover
	// * else a hover-kind pixel within selectR -> still draw hover
	// (so the wider ring of a hover-only shape stays visible if the
	// user widened thickness).
	bool sawSelectInSelectR = false;
	bool sawHoverInHoverR = false;
	bool sawHoverInSelectR = false;

	for ( int r = 1; r <= maxR; ++r )
	{
		for ( int i = 0; i < 4; ++i )
		{
			ivec2 off;
			if ( i == 0 )
				off = ivec2( r, 0 );
			else if ( i == 1 )
				off = ivec2( -r, 0 );
			else if ( i == 2 )
				off = ivec2( 0, r );
			else
				off = ivec2( 0, -r );

			ivec2 q = px + off;
			if ( q.x < 0 || q.y < 0 || q.x >= sz.x || q.y >= sz.y )
				continue;
			float k = texelFetch( sampler2D( tex_mask, smp_mask ), q, 0 ).x;
			if ( k <= KIND_EMPTY_THRESHOLD )
				continue;

			if ( k >= KIND_SELECT_THRESHOLD )
			{
				if ( r <= selectR )
					sawSelectInSelectR = true;
			}
			else
			{
				if ( r <= hoverR )
					sawHoverInHoverR = true;
				if ( r <= selectR )
					sawHoverInSelectR = true;
			}
		}
	}

	if ( sawSelectInSelectR )
	{
		out_color = select_color;
		return;
	}
	if ( sawHoverInHoverR )
	{
		out_color = hover_color;
		return;
	}
	if ( sawHoverInSelectR )
	{
		out_color = hover_color;
		return;
	}
	out_color = vec4( 0.0 );
}
#pragma sokol @end

#pragma sokol @program prog vs fs
