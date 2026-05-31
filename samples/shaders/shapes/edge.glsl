// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Edge overlay shader.
//
// Renders per-shape edges (face boundaries for hulls, dedup'd triangle edges
// for meshes, grid + diagonals for heightfields) as shader-expanded screen-
// space quads. Edge endpoint positions live in a per-geometry storage buffer
// (GetEdgeStorageView), per-instance transforms come from the existing
// geom instance buffer (GetMeshInstanceView / GetTransparentMeshInstanceView). Six
// vertices per edge, batched as `2 * 6 * edgeCount` vertices x instanceCount
// in one sg_draw, no VBO/IBO bound, gl_VertexIndex selects edge + corner.
//
// Vertex index -> corner layout (matches overlays/line.glsl):
//
// 0 -> endpoint=A, side=-1 3 -> endpoint=A, side=-1
// 1 -> endpoint=B, side=-1 4 -> endpoint=B, side=+1
// 2 -> endpoint=B, side=+1 5 -> endpoint=A, side=+1
//
// Z-fight prevention: reverse-Z clip-space nudge in the VS,
// `clip.z += zBias * clip.w`. Positive bias = toward camera (z=1 is near in
// reverse-Z). 5e-5 is enough at typical scene scales without visible offset
// at glancing angles.
//
// Per-fragment color picks between convex/concave/flat uniforms from the
// per-endpoint flag (0 concave, 1 convex, 2 flat, stored as float to keep
// the std430 endpoint struct a flat vec4, no vec3+uint padding ambiguity).
//
// Pipeline state:
//   * depth_compare = GREATER_EQUAL (reverse-Z). With the zBias nudge this
//     keeps the edge on top of the shape's own surface, transparent shapes
//     drawn after still over-blend the edge correctly.
//   * depth_write = off. The edge isn't real surface depth.
//   * Premultiplied-alpha blend (SRC_ONE, ONE_MINUS_SRC_ALPHA). FS outputs
//     `color * coverage` so coverage drops below 1 at the AA band only.

#pragma sokol @module edge

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs

layout( binding = 0 ) uniform ub_frame
{
	mat4 view_proj;
	vec4 viewport; // .xy = size px, .zw = 1/size
};

layout( binding = 2 ) uniform ub_draw
{
	ivec4 inst_base;  // .x = base index into the geom instance buffer
	vec4 edge_params; // .x = thicknessPx, .y = zBias, .zw reserved
};

struct geom_instance
{
	vec4 xform_row0;
	vec4 xform_row1;
	vec4 xform_row2;
	vec4 base_color;
	vec4 scale;
	vec4 material;
};

layout( binding = 0 ) readonly buffer geom_instances
{
	geom_instance gi_items[];
};

// Endpoint = vec4 (.xyz = position, .w = flag as float). Two consecutive
// entries per edge. sokol-shdc requires storage buffers to hold a struct
// array, not a primitive array, so the vec4 is wrapped.
struct edge_endpoint
{
	vec4 pos_and_flag;
};

layout( binding = 1 ) readonly buffer edge_endpoints
{
	edge_endpoint ee_items[];
};

// Long edges get thinned in the middle with perspective math
noperspective out float v_dist_from_axis_px;

// Flat thickness
flat out float v_half_width_px;

flat out float v_flag;

void main()
{
	// gl_VertexIndex is non-negative, an unsigned divide produces the same
	// result and sidesteps HLSL's signed-divide warning (X3556).
	int edge_idx = int( uint( gl_VertexIndex ) / 6u );
	int corner = gl_VertexIndex - edge_idx * 6;

	bool is_b = ( corner == 1 ) || ( corner == 2 ) || ( corner == 4 );
	float side = ( ( corner == 2 ) || ( corner == 4 ) || ( corner == 5 ) ) ? 1.0 : -1.0;

	geom_instance inst = gi_items[inst_base.x + gl_InstanceIndex];
	vec4 ea = ee_items[2 * edge_idx + 0].pos_and_flag;
	vec4 eb = ee_items[2 * edge_idx + 1].pos_and_flag;

	// Per-instance affine: rotation row dotted with (local_pos, 1). Apply
	// per-instance scale first, same convention as geom.glsl.
	vec3 a_local = ea.xyz * inst.scale.xyz;
	vec3 b_local = eb.xyz * inst.scale.xyz;
	vec3 a_world = vec3( dot( inst.xform_row0, vec4( a_local, 1.0 ) ), dot( inst.xform_row1, vec4( a_local, 1.0 ) ),
						 dot( inst.xform_row2, vec4( a_local, 1.0 ) ) );
	vec3 b_world = vec3( dot( inst.xform_row0, vec4( b_local, 1.0 ) ), dot( inst.xform_row1, vec4( b_local, 1.0 ) ),
						 dot( inst.xform_row2, vec4( b_local, 1.0 ) ) );

	vec4 clip_a = view_proj * vec4( a_world, 1.0 );
	vec4 clip_b = view_proj * vec4( b_world, 1.0 );
	vec4 clip_end = is_b ? clip_b : clip_a;

	// Screen-space pixel coords of both endpoints (post-perspective divide,
	// NDC [-1,1] -> pixel [0, viewport]).
	vec2 ndc_a = clip_a.xy / clip_a.w;
	vec2 ndc_b = clip_b.xy / clip_b.w;
	vec2 sa = ( ndc_a * 0.5 + 0.5 ) * viewport.xy;
	vec2 sb = ( ndc_b * 0.5 + 0.5 ) * viewport.xy;

	vec2 dir_px = sb - sa;
	float len_px = length( dir_px );
	vec2 dir_n = ( len_px > 1e-6 ) ? ( dir_px / len_px ) : vec2( 1.0, 0.0 );
	vec2 perp_n = vec2( -dir_n.y, dir_n.x );

	// Half-width in pixels, floor at 0.5 so very thin requests don't vanish
	// into sub-pixel land (the SDF's AA falloff handles attenuation below).
	float half_width_px = max( edge_params.x * 0.5, 0.5 );

	// Expand the rasterized quad past the true half-width by an AA pad so
	// the FS coverage falloff has geometry to ramp down inside (see
	// overlays/line.glsl for the full rationale). The FS smoothstep
	// completes at d = half_width + fwidth, and fwidth (an L1 norm) peaks
	// at sqrt(2) on a 45 deg line, so the pad must be >= ~1.42 px, 1.5 covers it.
	const float AA_PAD_PX = 1.5;
	float quad_half_px = half_width_px + AA_PAD_PX;

	vec2 offset_px = perp_n * side * quad_half_px;
	vec2 offset_ndc = offset_px * ( 2.0 * viewport.zw );
	vec4 clip_pos = clip_end;
	clip_pos.xy += offset_ndc * clip_end.w;
	// Reverse-Z z-bias nudge (toward camera, z=1 = near).
	clip_pos.z += edge_params.y * clip_pos.w;

	gl_Position = clip_pos;
	v_dist_from_axis_px = side * quad_half_px;
	v_half_width_px = half_width_px;
	v_flag = ea.w; // both endpoints carry the same flag
}
#pragma sokol @end

#pragma sokol @fs fs

layout( binding = 1 ) uniform ub_pass
{
	vec4 convex_color;	// premultiplied linear RGBA
	vec4 concave_color; // premultiplied linear RGBA
	vec4 flat_color;	// premultiplied linear RGBA
};

noperspective in float v_dist_from_axis_px;
flat in float v_half_width_px;
flat in float v_flag;

out vec4 out_color;

void main()
{
	// Coverage from the capsule SDF. d is the fragment's folded
	// perpendicular distance from the axis (px), coverage ramps 1->0 over a
	// 2*aa band straddling the true edge at d = half_width. Smoothstep
	// (cubic, wider than an energy-exact box filter), with fwidth of the
	// raw signed v_dist_from_axis_px to avoid the abs-kink, see
	// overlays/line.glsl for the full rationale.
	float d = abs( v_dist_from_axis_px );
	float aa = fwidth( v_dist_from_axis_px );
	float coverage = 1.0 - smoothstep( v_half_width_px - aa, v_half_width_px + aa, d );

	// Flag is 0 concave, 1 convex, 2 flat (coplanar).
	vec4 col;
	if ( v_flag > 1.5 )
	{
		col = flat_color;
	}
	else if ( v_flag > 0.5 )
	{
		col = convex_color;
	}
	else
	{
		col = concave_color;
	}
	out_color = col * coverage;
}
#pragma sokol @end

#pragma sokol @program edge_overlay vs fs
