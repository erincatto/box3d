// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Overlay line shader.
//
// Renders fat lines as shader-expanded screen-space quads. Per-instance
// data (two endpoints, color, thickness, mode flags) lives in a storage
// buffer indexed by gl_InstanceIndex, the VS expands six vertices per
// instance from gl_VertexIndex with no VBO/IBO bound.
//
// Vertex index -> corner layout (two triangles forming a quad):
//
// 0 -> endpoint=A, side=-1 3 -> endpoint=A, side=-1
// 1 -> endpoint=B, side=-1 4 -> endpoint=B, side=+1
// 2 -> endpoint=B, side=+1 5 -> endpoint=A, side=+1
//
// Thickness modes (instance flag bit 0):
// PIXELS, screen-space pixels, constant on-screen weight.
// WORLD, world meters, converted to pixels per endpoint using
//            `viewport.y * proj[1][1] / (2 * clip.w)`. Per-endpoint, so
//            a long line tapers correctly with depth.
//
// Occlusion modes (instance flag bit 1, branched in the FS):
// HIDE, output alpha=0 when occluded (single-pipeline approximation
//            of a depth-tested draw, see note below on pipeline state).
// DIM, modulate color (premultiplied -> multiply both rgb and a) when
//            occluded so the line stays readable through walls.
// DASHED, screen-space dash pattern (8 px on / 8 px off) when occluded.
// Pattern is zero-alpha in the gap, NOT discard, avoids
//            disabling early-Z and keeps the FS flow control flat.
//
// Pipeline state notes:
// * depth_compare = ALWAYS, depth_write = off. The shader samples the
//   linear-depth pre-pass (R32F view-space Z, written by GTAO's depth
//   pre-pass) and compares per-fragment for the DIM/DASHED branches.
//   HIDE relies on the same sample + alpha=0. MSAA silhouettes for
//   HIDE may read slightly harder than a depth-tested draw would
//   produce. If that bites visually, split into a second pipeline
//   with depth_compare = GREATER for HIDE-mode instances.
// * Premultiplied-alpha blend (SRC_ONE, ONE_MINUS_SRC_ALPHA). CPU side
//   packs `color.rgb *= color.a` into the instance before upload.
//
// UV handling: the linear-depth target is sampled at
// `gl_FragCoord.xy / textureSize(...)`. This is the same trick cube.glsl
// uses for the GTAO sample. Pixel-space gl_FragCoord matches the
// rasterizer's storage orientation on every backend, so no uvYSign flip
// is needed. The flip is required only when sampling with NDC-derived
// UVs, which we don't do here.

#pragma sokol @module overlay_line

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs

layout(binding = 0) uniform ub_frame
{
	mat4 view;
	mat4 proj;
	mat4 view_proj;
	mat4 inv_view_proj;
	vec4 camera_pos; // .xyz = pos, .w = time
	vec4 viewport; // .xy = size px, .zw = 1/size
};

struct line_instance
{
	vec4 a; // .xyz = world endpoint A, .w = thickness (px or world)
	vec4 b; // .xyz = world endpoint B, .w = reserved
	vec4 color; // premultiplied linear RGBA
	uvec4 flags; // .x = thickness unit (0 px / 1 world)
	// .y = occlusion mode (0 hide / 1 dim / 2 dashed)
	// .zw = reserved
};

layout(binding = 0) readonly buffer line_instances
{
	line_instance instances[];
};

out vec2 v_screen_a; // flat per-instance screen-space pixel coords of A
out vec2 v_screen_b; // flat per-instance screen-space pixel coords of B

noperspective out float v_dist_from_axis_px; // signed perpendicular pixel distance (smooth)
out float v_half_width_px; // half-width in pixels at this fragment (smooth, varies in WORLD mode)

out float v_view_z; // positive view-space depth (smooth, perspective-correct)
flat out vec4 v_color;
flat out uint v_occ_mode;

void main()
{
	line_instance inst = instances[gl_InstanceIndex];
	
	// Decode vertex index -> endpoint + side.
	int index = gl_VertexIndex;
	bool is_b = (index == 1)||(index == 2)||(index == 4);
	float side = ((index == 2)||(index == 4)||(index == 5)) ? 1.0 : -1.0;
	
	vec3 world_a = inst.a.xyz;
	vec3 world_b = inst.b.xyz;
	vec4 clip_a = view_proj * vec4(world_a, 1.0);
	vec4 clip_b = view_proj * vec4(world_b, 1.0);
	vec4 view_a = view * vec4(world_a, 1.0);
	vec4 view_b = view * vec4(world_b, 1.0);
	
	vec4 clip_end = is_b ? clip_b : clip_a;
	float view_z_end = -(is_b ? view_b.z : view_a.z);
	
	// Screen-space pixel coords of both endpoints (after perspective divide,
	// remapped from NDC [-1,1] to pixel [0, viewport]). Used by the FS for
	// dash phase computation, both endpoints needed at every vertex because
	// we don't qualify "flat from provoking vertex" which is impl-defined.
	vec2 ndc_a = clip_a.xy / clip_a.w;
	vec2 ndc_b = clip_b.xy / clip_b.w;
	v_screen_a = (ndc_a * 0.5 + 0.5) * viewport.xy;
	v_screen_b = (ndc_b * 0.5 + 0.5) * viewport.xy;
	
	// Screen-space line direction in pixels (aspect-corrected). Fallback
	// direction if both endpoints project to the same pixel, line is
	// effectively a point, the perpendicular doesn't matter.
	vec2 dir_px = v_screen_b - v_screen_a;
	float len_px = length(dir_px);
	vec2 dir_n = (len_px > 1e-6) ? (dir_px / len_px) : vec2(1.0, 0.0);
	vec2 perp_n = vec2(-dir_n.y, dir_n.x);
	
	// Half-width in pixels, mode-aware. WORLD converts at the endpoint's
	// view-space depth so the line tapers correctly. proj[1][1] is
	// 1/tan(fovY/2) for a standard perspective projection, viewport.y *
	// proj[1][1] / (2 * z) yields pixels per world unit at view-space
	// distance z.
	float thickness = inst.a.w;
	float half_width_px;
	if (inst.flags.x == 0u)
	{
		half_width_px = thickness * 0.5;
	}
	else
	{
		float pixels_per_world = viewport.y * proj[1][1] / (2.0 * view_z_end);
		half_width_px = thickness * 0.5 * pixels_per_world;
	}

	// Floor at 0.5 px so very thin lines don't vanish into sub-pixel land,
	// attenuation for requested widths below this happens FS-side via the
	// box-filter coverage falloff.
	half_width_px = max(half_width_px, 0.5);
	
	// Expand the rasterized quad past the true half-width by an AA pad so
	// the FS coverage falloff has geometry to ramp down inside. Without it
	// the quad edge cuts the falloff off mid-ramp, a hard step along the
	// line that reads as a jaggy no matter how good the FS filter is. The
	// FS uses the TRUE half-width (v_half_width_px), so the pad adds only
	// transparent margin, it does not fatten the visible line. The FS
	// smoothstep falloff completes at d = half_width + fwidth, and fwidth
	// (an L1 norm) peaks at sqrt(2) on a 45 deg line, so the pad must be
	// >= ~1.42 px, 1.5 covers it with margin.
	const float AA_PAD_PX = 1.5;
	float quad_half_px = half_width_px + AA_PAD_PX;
	
	// Offset from the axis in pixels -> in NDC at this endpoint's depth.
	// Multiply by clip_end.w so that after perspective divide it lands at
	// the right NDC position.
	vec2 offset_px = perp_n * side * quad_half_px;
	vec2 offset_ndc = offset_px * (2.0 * viewport.zw);
	vec4 clip_pos = clip_end;
	clip_pos.xy += offset_ndc * clip_end.w;
	
	gl_Position = clip_pos;
	v_dist_from_axis_px = side * quad_half_px;
	v_half_width_px = half_width_px;
	v_view_z = view_z_end;
	v_color = inst.color;
	v_occ_mode = inst.flags.y;
}
#pragma sokol @end

#pragma sokol @fs fs

// binding=1 (not 0): sokol-shdc enforces disjoint binding numbers between
// storage buffers and textures across all stages of a program. The line
// instance buffer holds binding=0 in the VS, so the depth texture takes 1.
#pragma sokol @image_sample_type tex_depth float
layout(binding = 1)uniform texture2D tex_depth;

#pragma sokol @sampler_type smp_depth filtering
layout(binding = 0)uniform sampler smp_depth;

in vec2 v_screen_a;
in vec2 v_screen_b;
noperspective in float v_dist_from_axis_px;
in float v_half_width_px;
in float v_view_z;
flat in vec4 v_color;
flat in uint v_occ_mode;
out vec4 out_color;

const uint OCC_HIDE = 0u;
const uint OCC_DIM = 1u;
const uint OCC_DASHED = 2u;

void main() 
{
	// Coverage from the capsule SDF. d is the fragment's folded
	// perpendicular distance from the axis (px), coverage ramps 1->0 over a
	// 2*aa band straddling the true edge at d = half_width.
	//
	// This is a smoothstep, not an energy-exact box filter. A box filter's
	// ~1px falloff faithfully renders the intrinsic stair-stepping of a
	// thin near-axis-aligned line, this wider cubic falloff spreads each
	// row-to-row step into a gradient that reads smoother, the better
	// tradeoff for thin debug lines. fwidth is taken of the raw signed
	// v_dist_from_axis_px (not abs/sd): fwidth(abs(x)) kinks where x
	// crosses 0 at the line center, the signed value is stable everywhere.
	// The VS quad pad (AA_PAD_PX) must cover this 2*aa band.
	float d = abs(v_dist_from_axis_px);
	float aa = fwidth(v_dist_from_axis_px);
	float coverage = 1.0 - smoothstep(v_half_width_px - aa, v_half_width_px + aa, d);
	
	// Sample scene linear depth. Pixel-space gl_FragCoord divided by the
	// target's textureSize is backend-canonical, no UV.y flip needed.
	vec2 depth_uv = gl_FragCoord.xy / vec2(textureSize(sampler2D(tex_depth, smp_depth), 0));
	float scene_z = textureLod(sampler2D(tex_depth, smp_depth), depth_uv, 0.0).r;

	// Slope-scaled occlusion bias. A fixed bias can't protect a line lying
	// coplanar with a surface: at grazing angles the scene depth changes
	// many mm per pixel, so the sampled scene_z and the line's own v_view_z
	// disagree by more than any small constant, the test flips per pixel
	// and HIDE-mode chunks drop out, breaking the line. fwidth(v_view_z) is
	// the line's per-pixel view-depth change: large at grazing angles, ~0
	// head-on, so the bias grows exactly where it's needed without letting
	// head-on lines bleed through occluders. Same idea as slope-scaled
	// shadow depth bias.
	const float occlusion_bias_m = 0.001; // ~1 mm floor, head-on contact
	const float occlusion_slope_k = 2.0; // grazing-angle multiplier
	float occlusion_bias = occlusion_bias_m + occlusion_slope_k * fwidth(v_view_z);
	bool occluded = v_view_z > scene_z + occlusion_bias;
	
	// Occlusion mode modulation. v_color is premultiplied: multiplying the
	// whole vec4 stays valid (rgb*= and a*= by the same factor preserves
	// the premultiplied invariant).
	vec4 col = v_color;
	
	if (v_occ_mode == OCC_HIDE && occluded)
	{
		col = vec4(0.0);
	}
	else if (v_occ_mode == OCC_DIM && occluded)
	{
		col *= 0.25;
	}
	else if (v_occ_mode == OCC_DASHED && occluded)
	{
		// Screen-space dash phase, in pixels along the line from A.
		vec2 line_vec = v_screen_b - v_screen_a;
		float line_len = max(length(line_vec), 1e-6);
		vec2 line_dir = line_vec / line_len;
		vec2 rel_px = gl_FragCoord.xy - v_screen_a;
		float along_px = dot(rel_px, line_dir);
		
		const float DASH_PERIOD_PX = 16.0;
		const float DASH_HALF = 4.0; // half the solid run inside the period
		float phase = mod(along_px, DASH_PERIOD_PX);
		float dash_sd = abs(phase - DASH_HALF) - DASH_HALF;
		float dash_aa = fwidth(along_px);
		float dash_msk = 1.0 - smoothstep(-dash_aa, dash_aa, dash_sd);
		col *= dash_msk;
	}
	
	out_color = col * coverage;
}
#pragma sokol @end

#pragma sokol @program prog vs fs
