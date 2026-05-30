// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Highlight mask, convex hull (triangle-rasterized).
//
// Same vertex math as depth_only_geom (transform + per-instance scale), but the
// FS just emits the highlight kind into the R8_UINT mask. Rasterized depth
// is correct as-is, no gl_FragDepth write. Pipeline depth-tests GREATER_EQUAL
// against the prepass depth.

#pragma sokol @module mask_hull

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 b3Vec3

#pragma sokol @vs vs

layout( binding = 0 ) uniform ub_frame
{
	mat4 view_proj;
};

layout( binding = 2 ) uniform ub_draw
{
	ivec4 inst_base; // .x = base index into the hull-mask instance array
};

struct instance
{
	vec4 xform_row0;
	vec4 xform_row1;
	vec4 xform_row2;
	vec4 scale_kind; // .xyz = per-instance scale, .w = highlight kind
};

layout( binding = 0 ) readonly buffer instances
{
	instance items[];
};

// Position-only input. The shared geom vbo carries position+normal
// (stride 24 bytes), but the mask FS only needs world position. We
// don't declare an `in_normal` attribute so the GL linker doesn't strip
// it and warn. The C-side pipeline desc skips the in_normal binding to
// match.
in vec3 in_pos;

flat out float v_kind;

void main()
{
	instance inst = items[inst_base.x + gl_InstanceIndex];

	vec3 scaled_pos = in_pos * inst.scale_kind.xyz;
	vec4 local_pos = vec4( scaled_pos, 1.0 );
	vec3 world_pos =
		vec3( dot( inst.xform_row0, local_pos ), dot( inst.xform_row1, local_pos ), dot( inst.xform_row2, local_pos ) );

	v_kind = inst.scale_kind.w;
	gl_Position = view_proj * vec4( world_pos, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs

flat in float v_kind;

layout( location = 0 ) out vec4 out_kind;

void main()
{
	// R8 UNORM mask: 1.0=hover, 2.0=select -> 0.5, 1.0.
	out_kind = vec4( v_kind * 0.5, 0.0, 0.0, 1.0 );
}
#pragma sokol @end

#pragma sokol @program prog vs fs
