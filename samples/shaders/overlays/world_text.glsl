// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// World-space text shader. Glyph coverage plus a distance field halo, the two
// channels of one atlas baked at the size labels draw at.
//
// One instance per glyph. Per-instance data (label anchor, pen rect, atlas
// UVs, color) lives in a storage buffer indexed by
// gl_InstanceIndex, the VS expands six vertices per instance from
// gl_VertexIndex with no VBO/IBO bound, same scheme as line.glsl and
// point.glsl. Quad corner order matches point.glsl:
//
// 0 -> (-1, -1) 3 -> (-1, -1)
// 1 -> (+1, -1) 4 -> (+1, +1)
// 2 -> (+1, +1) 5 -> (-1, +1)
//
// Glyphs are screen-aligned, not world-oriented: the anchor projects to
// clip space and the quad expands in pixels around it. Labels stay readable
// from any camera angle, which world-oriented text does not.
//
// Labels hold one pixel height whatever their distance, so a readout stays
// legible across a scene the way the ImGui labels this replaced did. They
// also ignore scene depth: a label names a thing, and a name you cannot read
// because a wall moved in front of it is worse than one that floats.
//
// The halo is why the field is still here. A rasterized bitmap gives the crisp
// body a distance field cannot at these sizes, but knows nothing about what
// lies outside a glyph, and the fading pad needs exactly that. One fetch reads
// both. The alternative in a UI draw list is re-emitting the whole string four
// times, and that only ever buys a hard outline.

#pragma sokol @module world_text

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs

layout(binding = 0)uniform ub_frame
{
	mat4 view;
	mat4 proj;
	mat4 view_proj;
	mat4 inv_view_proj;
	vec4 camera_pos; // .xyz = pos, .w = time
	vec4 viewport; // .xy = size px, .zw = 1/size
};

struct glyph_instance
{
	vec4 anchor; // .xyz = label world anchor, .w unused
	vec4 rect; // .xy = pen offset in pixels, .zw = glyph size in pixels
	vec4 uv; // .xy = atlas uv min, .zw = atlas uv max
	vec4 color; // linear RGBA, not premultiplied
};

layout(binding = 0) readonly buffer glyph_instances
{
	glyph_instance instances[];
};

out vec2 v_uv;
flat out vec4 v_color;

void main()
{
	glyph_instance inst = instances[gl_InstanceIndex];

	int index = gl_VertexIndex;
	float corner_x = ((index == 1)||(index == 2)||(index == 4)) ? 1.0 : -1.0;
	float corner_y = ((index == 2)||(index == 4)||(index == 5)) ? 1.0 : -1.0;
	vec2 corner = vec2(corner_x, corner_y) * 0.5 + 0.5; // 0..1 across the glyph cell

	vec4 clip_c = view_proj * vec4(inst.anchor.xyz, 1.0);

	// Snap the anchor to a whole pixel. Every glyph in a label then shares one
	// pixel phase, so the horizontal strokes they have in common land on the
	// grid together instead of each smearing by its own fraction of a pixel.
	vec2 anchor_px = ((clip_c.xy / clip_c.w) * 0.5 + 0.5) * viewport.xy;
	anchor_px = floor(anchor_px + 0.5);

	// Pen space runs Y down like the atlas and like the glyph metrics that
	// produced the rect, clip space runs Y up, so flip on the way out. The rect
	// is already whole pixels, so with a snapped anchor the quad covers its
	// atlas cell one texel to one pixel.
	vec2 pen_px = inst.rect.xy + corner * inst.rect.zw;
	vec2 pos_px = anchor_px + vec2(pen_px.x, -pen_px.y);
	vec2 pos_ndc = pos_px * viewport.zw * 2.0 - 1.0;

	// Back through the divide the rasterizer is about to do, so the snapped
	// pixel position survives and depth / clipping stay as projected.
	gl_Position = vec4(pos_ndc * clip_c.w, clip_c.z, clip_c.w);
	v_uv = mix(inst.uv.xy, inst.uv.zw, corner);
	v_color = inst.color;
}
#pragma sokol @end

#pragma sokol @fs fs

// Bindings are disjoint across storage buffers and textures for the whole
// program, so the glyph buffer's binding=0 in the VS pushes the atlas up.
#pragma sokol @image_sample_type tex_sdf float
layout(binding = 1)uniform texture2D tex_sdf;

#pragma sokol @sampler_type smp_sdf filtering
layout(binding = 0)uniform sampler smp_sdf;

layout(binding = 1)uniform ub_text_style
{
	vec4 style_params; // .x = halo width in pixels, .y = on-edge value, .z = distance per pixel, .w = reserved
	vec4 halo_params; // .rgb = halo color, .a = halo strength at the glyph edge
};

in vec2 v_uv;
flat in vec4 v_color;
out vec4 out_color;

void main()
{
	// R is glyph coverage, G a signed distance positive inside the glyph and
	// biased by the on-edge value the atlas was baked with.
	vec2 tap = texture(sampler2D(tex_sdf, smp_sdf), v_uv).rg;
	float d = tap.g - style_params.y;

	// The body is the rasterizer's own coverage, sampled one texel to one pixel,
	// so it needs no reconstruction here. That is the whole reason for the
	// second channel: a distance field has to infer coverage from a smoothstep,
	// and at label sizes that ramp is a large part of a stem.
	float glyph = tap.r * v_color.a;

	// The halo fades outward from the glyph edge instead of ending at a fixed
	// offset. A hard ring reads as a sticker outline and, worse, fills the
	// counter of an o at label sizes, where the ring closes in from both
	// sides. A falloff leaves the middle of the counter mostly clear.
	float halo = (1.0 - smoothstep(0.0, style_params.x * style_params.z, -d)) * halo_params.a;

	// Glyph over halo, emitted premultiplied to match the blend state.
	float alpha = glyph + halo * (1.0 - glyph);
	vec3 rgb = v_color.rgb * glyph + halo_params.rgb * halo * (1.0 - glyph);
	out_color = vec4(rgb, alpha);
}
#pragma sokol @end

#pragma sokol @program prog vs fs
