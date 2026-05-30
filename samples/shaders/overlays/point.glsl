// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Overlay point shader.
//
// Renders fat points as shader-expanded screen-space quads centered on the
// projected world position. Per-instance data (world point, color, size,
// mode flags) lives in a storage buffer indexed by gl_InstanceIndex, the
// VS expands six vertices per instance from gl_VertexIndex with no
// VBO/IBO bound. Two triangles form the quad:
//
// 0 -> (-1, -1) 3 -> (-1, -1)
// 1 -> (+1, -1) 4 -> (+1, +1)
// 2 -> (+1, +1) 5 -> (-1, +1)
//
// Size modes (instance flag bit 0) and occlusion modes (bit 1) match
// line.glsl's conventions exactly, see that file for the rationale.
//
// The FS is a circle SDF: signed distance from the projected point center
// in pixels, AA'd via fwidth. Square or diamond markers would be cheaper
// but circles read unambiguously as "point markers" against the line
// primitive which is a capsule SDF.
//
// Dashed-mode for a point is a bit nonsensical (the dashing axis is
// "along the line", a point has no axis), so DASHED behaves identically
// to DIM here. HIDE and DIM behave as documented.

#pragma sokol @module overlay_point

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

struct point_instance
{
	vec4 p; // .xyz = world position, .w = size (px or world)
	vec4 color; // premultiplied linear RGBA
	uvec4 flags; // .x = size unit (0 px / 1 world)
	// .y = occlusion mode (0 hide / 1 dim / 2 dashed->dim)
	// .zw = reserved
};

layout(binding = 0) readonly buffer point_instances
{
	point_instance instances[];
};

out vec2 v_offset_from_center_px; // signed pixel offset from quad center (smooth)
out float v_half_size_px; // true point radius in pixels (flat-equivalent, all 6 verts share)
out float v_view_z; // positive view-space depth (smooth, perspective-correct)
flat out vec4 v_color;
flat out uint v_occ_mode;

void main()
{
	point_instance inst = instances[gl_InstanceIndex];
	
	int index = gl_VertexIndex;
	float corner_x = ((index == 1)||(index == 2)||(index == 4)) ? 1.0 : -1.0;
	float corner_y = ((index == 2)||(index == 4)||(index == 5)) ? 1.0 : -1.0;
	
	vec3 world_p = inst.p.xyz;
	vec4 clip_c = view_proj * vec4(world_p, 1.0);
	vec4 view_c = view * vec4(world_p, 1.0);
	float view_z = -view_c.z;
	
	// Half-size in pixels, mode-aware. WORLD conversion follows the same
	// formula as line.glsl: pixels per world unit = viewport.y *
	// proj[1][1] / (2 * view_z).
	float requested = inst.p.w;
	float half_size_px;
	if (inst.flags.x == 0u)
	{
		half_size_px = requested * 0.5;
	}
	else
	{
		float pixels_per_world = viewport.y * proj[1][1] / (2.0 * view_z);
		half_size_px = requested * 0.5 * pixels_per_world;
	}
	half_size_px = max(half_size_px, 0.5);
	
	// Expand the rasterized quad past the true radius by an AA pad so the
	// FS circle-SDF falloff has geometry to ramp down inside (see
	// overlays/line.glsl for the full rationale). The FS smoothstep
	// completes at d = radius + fwidth, and fwidth (an L1 norm) peaks at
	// sqrt(2), so the pad must be >= ~1.42 px, 1.5 covers it.
	const float AA_PAD_PX = 1.5;
	float quad_half_px = half_size_px + AA_PAD_PX;
	
	vec2 offset_px = vec2(corner_x, corner_y) * quad_half_px;
	vec2 offset_ndc = offset_px * (2.0 * viewport.zw);
	
	vec4 clip_pos = clip_c;
	clip_pos.xy += offset_ndc * clip_c.w;
	
	gl_Position = clip_pos;
	v_offset_from_center_px = offset_px;
	v_half_size_px = half_size_px;
	v_view_z = view_z;
	v_color = inst.color;
	v_occ_mode = inst.flags.y;
}
#pragma sokol @end

#pragma sokol @fs fs

// binding=1 (not 0): sokol-shdc enforces disjoint binding numbers between
// storage buffers and textures across all stages of a program. The point
// instance buffer holds binding=0 in the VS, so the depth texture takes 1.
#pragma sokol @image_sample_type tex_depth float
layout(binding = 1)uniform texture2D tex_depth;

#pragma sokol @sampler_type smp_depth filtering
layout(binding = 0)uniform sampler smp_depth;

in vec2 v_offset_from_center_px;
in float v_half_size_px;
in float v_view_z;
flat in vec4 v_color;
flat in uint v_occ_mode;
out vec4 out_color;

const uint OCC_HIDE = 0u;

void main()
{
	// Coverage from the circle SDF. d is the fragment's radial distance
	// from the point center (px), coverage ramps 1->0 over a 2*aa band
	// straddling the true edge at d = radius. Smoothstep (cubic, wider
	// than an energy-exact box filter), see overlays/line.glsl for why
	// the softer falloff is preferred for thin overlay primitives.
	float d = length(v_offset_from_center_px);
	float aa = fwidth(d);
	float coverage = 1.0 - smoothstep(v_half_size_px - aa, v_half_size_px + aa, d);
	
	vec2 depth_uv = gl_FragCoord.xy / vec2(textureSize(sampler2D(tex_depth, smp_depth), 0));
	float scene_z = textureLod(sampler2D(tex_depth, smp_depth), depth_uv, 0.0).r;

	// Slope-scaled occlusion bias, see overlays/line.glsl for the full
	// rationale. Keeps a point resting on a surface from flickering out in
	// HIDE mode without letting head-on points bleed through occluders.
	const float occlusion_bias_m = 0.001; // ~1 mm floor
	const float occlusion_slope_k = 2.0;
	float occlusion_bias = occlusion_bias_m + occlusion_slope_k * fwidth(v_view_z);
	bool occluded = v_view_z > scene_z + occlusion_bias;
	
	vec4 col = v_color;
	
	if (occluded)
	{
		if (v_occ_mode == OCC_HIDE)
		{
			col = vec4(0.0);
		} else {
			// DIM (1) and DASHED (2, degenerates to DIM for points)
			col *= 0.25;
		}
	}
	
	out_color = col * coverage;
}
#pragma sokol @end

#pragma sokol @program prog vs fs
