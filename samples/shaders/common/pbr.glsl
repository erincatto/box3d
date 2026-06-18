// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Shared PBR helpers, included via `@include../common/pbr.glsl` by each
// lit shape shader (cube, sphere, capsule, geom).
//
// brdf_evaluate: a subset of Disney/Principled. GGX (Trowbridge-Reitz)
// NDF, Smith correlated geometry, Schlick Fresnel, Lambert diffuse. No
// subsurface, transmission, clearcoat, sheen, anisotropy. References:
// Walter et al. 2007, Karis 2013 (UE4), Heitz 2014.
//
// proceduralGrid: fwidth-antialiased two-tier grid pattern over world-space
// XZ. Used by the geom pipeline when v_material.z selects the ground
// material. Designed to feed straight into brdf_evaluate as baseColor.
//
// Callers pick the coordinate space. N, V, L must all be in the same
// space. n.l, n.v, n.h are rotation-invariant. The cube shader uses view
// space (its dFdx/dFdy normal can't safely cross to world on D3D11, see
// cube.glsl), every other lit shader uses world space.

const float PBR_PI = 3.14159265358979323846;

float pbrDistributionGGX(float n_dot_h, float alpha)
{
	float a2 = alpha * alpha;
	float d = (n_dot_h * n_dot_h) * (a2 - 1.0) + 1.0;
	return a2 / (PBR_PI * d * d);
}

// Smith correlated G, Heitz 2014.
float pbrVisibilitySmithCorrelated(float n_dot_v, float n_dot_l, float alpha)
{
	float a2 = alpha * alpha;
	float vv = n_dot_l * sqrt(n_dot_v * n_dot_v * (1.0 - a2) + a2);
	float vl = n_dot_v * sqrt(n_dot_l * n_dot_l * (1.0 - a2) + a2);
	return 0.5 / (vv + vl + 1e-5);
}

vec3 pbrFresnelSchlick(float v_dot_h, vec3 f0)
{
	float t = 1.0 - v_dot_h;
	float t2 = t * t;
	return f0 + (vec3(1.0) - f0) * (t2 * t2 * t);
}

// Direct sun lighting only. sun_radiance is sun_color.rgb premultiplied
// by the shadow factor so the caller's existing shadow path is unchanged.
// Ambient is split out into pbrEvaluateIbl below, callers compose
// direct + ibl_ambient.
vec3 brdf_evaluate(vec3 N, vec3 V, vec3 L, vec3 sun_radiance,
	vec3 base_color, float metallic, float roughness)
{
	float n_dot_l = max(dot(N, L), 0.0);
	float n_dot_v = max(dot(N, V), 1e-4);
	
	vec3 H = normalize(V + L);
	float n_dot_h = max(dot(N, H), 0.0);
	float v_dot_h = max(dot(V, H), 0.0);
	
	// Perceptual roughness -> roughness alpha (Karis). The 0.045 floor
	// keeps the specular highlight from collapsing to a singular point
	// at roughness 0, which causes hot-pixel sparkle under MSAA.
	float perc = max(roughness, 0.045);
	float alpha = perc * perc;
	
	vec3 f0 = mix(vec3(0.04), base_color, metallic);
	
	float D = pbrDistributionGGX(n_dot_h, alpha);
	float Vis = pbrVisibilitySmithCorrelated(n_dot_v, n_dot_l, alpha);
	vec3 F = pbrFresnelSchlick(v_dot_h, f0);
	
	vec3 specular = D * Vis * F;
	vec3 diffuse = (vec3(1.0) - F) * (1.0 - metallic) * base_color / PBR_PI;
	
	return (diffuse + specular) * sun_radiance * n_dot_l;
}

// Evaluate band-0..2 SH at world-space direction N. The stored
// coefficients (sh[0..8].xyz) are already pre-multiplied with the
// Funk-Hecke / Ramamoorthi diffuse convolution weights, so this returns
// irradiance E(N) directly. Pre-multiply convention is fixed C-side in
// projectSkyToSh, the shader just dots against the SH basis.
//
// Index order matches Sloan / Ramamoorthi:
//   0 = (l=0,m= 0)
//   1 = (l=1,m=-1)  2 = (l=1,m= 0)  3 = (l=1,m=+1)
//   4 = (l=2,m=-2)  5 = (l=2,m=-1)  6 = (l=2,m= 0)  7 = (l=2,m=+1)  8 = (l=2,m=+2).
vec3 pbrEvaluateShIrradiance(vec3 N, vec4 sh[9])
{
	const float Y00 = 0.282094791773878;
	const float Y1n1 = 0.488602511902919;
	const float Y10 = 0.488602511902919;
	const float Y1p1 = 0.488602511902919;
	const float Y2n2 = 1.092548430592079;
	const float Y2n1 = 1.092548430592079;
	const float Y20 = 0.315391565252520;
	const float Y2p1 = 1.092548430592079;
	const float Y2p2 = 0.546274215296039;
	
	float b0 = Y00;
	float b1 = -Y1n1 * N.y;
	float b2 = Y10 * N.z;
	float b3 = -Y1p1 * N.x;
	float b4 = Y2n2 * N.x * N.y;
	float b5 = -Y2n1 * N.y * N.z;
	float b6 = Y20 * (3.0 * N.z * N.z - 1.0);
	float b7 = -Y2p1 * N.x * N.z;
	float b8 = Y2p2 * (N.x * N.x - N.y * N.y);
	
	return sh[0].xyz * b0
	+ sh[1].xyz * b1 + sh[2].xyz * b2 + sh[3].xyz * b3
	+ sh[4].xyz * b4 + sh[5].xyz * b5 + sh[6].xyz * b6
	+ sh[7].xyz * b7 + sh[8].xyz * b8;
}

// IBL ambient: SH diffuse + GGX prefilter specular via Karis split-sum.
// Returns the ambient RGB to add to the direct lighting term. N_world
// and V_world must be in world space (the cube shader transforms its
// view-space normal here before calling).
//
// Fresnel-roughness compensates the diffuse fraction at grazing angles
// for rough surfaces, Lagarde 2014. F0_max = max(1-roughness, F0) lets
// dielectrics stay bright even when roughness is high.
vec3 pbrEvaluateIbl(vec3 N_world, vec3 V_world, vec3 base_color, float metallic,
	float roughness, vec4 sh[9],
	textureCube tex_spec_cube, sampler smp_spec_cube,
	texture2D tex_brdf_lut, sampler smp_brdf_lut,
	float max_lod)
{
	// Clamp upper bound too: impostor n_world (sphere/capsule) comes from
	// (hit - center) / radius and isn't explicitly normalized, so float
	// drift can push |n_world| slightly above 1 and n_dot_v above 1. Then
	// 1.0 - n_dot_v < 0 and pow(negative, 5.0) is undefined per GLSL spec
	// (NVIDIA returns NaN), which propagates through F and renders as
	// isolated black pixels on the surface.
	float n_dot_v = clamp(dot(N_world, V_world), 1e-4, 1.0);
	vec3 R = reflect(-V_world, N_world);
	
	vec3 F0 = mix(vec3(0.04), base_color, metallic);
	vec3 F0m = max(vec3(1.0 - roughness), F0);

	// max(0, ...) is a no-op given the n_dot_v clamp above, but the HLSL
	// cross-compile loses the range info and warns about pow(negative, n)
	// (X3571) without it.
	vec3 F = F0 + (F0m - F0) * pow(max(0.0, 1.0 - n_dot_v), 5.0);
	
	vec3 kS = F;
	vec3 kD = (vec3(1.0) - kS) * (1.0 - metallic);
	
	vec3 irradiance = max(pbrEvaluateShIrradiance(N_world, sh), vec3(0.0));
	vec3 diffuse = kD * base_color * irradiance / PBR_PI;
	
	float lod = roughness * max_lod;
	vec3 prefiltered = textureLod(samplerCube(tex_spec_cube, smp_spec_cube), R, lod).rgb;
	vec2 brdf = texture(sampler2D(tex_brdf_lut, smp_brdf_lut),
	vec2(n_dot_v, roughness)).rg;
	vec3 specular = prefiltered * (F0 * brdf.x + brdf.y);
	
	return diffuse + specular;
}

// Ambient selector. iblEnabled chooses between the image-based ambient
// (pbrEvaluateIbl above) and the legacy flat ambient, base_color *
// flat_strength, that the renderer used. The renderer drives
// iblEnabled from a Render Settings panel toggle so the IBL contribution
// can be A/B'd against the flat term.
//
// The branch is on a uniform: every fragment of every lit draw sees the
// same iblEnabled, so this is uniform control flow and the implicit-LOD
// BRDF-LUT fetch inside pbrEvaluateIbl stays well-defined. Callers pass the
// same N_world/V_world/base_color they would have passed to pbrEvaluateIbl.
vec3 pbrEvaluateAmbient(bool iblEnabled,
	vec3 N_world, vec3 V_world,
	vec3 base_color, float metallic, float roughness,
	float flat_strength,
	vec4 sh[9],
	textureCube tex_spec_cube, sampler smp_spec_cube,
	texture2D tex_brdf_lut, sampler smp_brdf_lut,
float max_lod)
{
	// Single result, pre-seeded with the flat-ambient fallback. The early-
	// return form `if (iblEnabled) return...; return flat;` cross-
	// compiles through SPIRV-Cross into a returned-flag pattern whose return
	// temp fxc can't prove definitely-assigned (warning X4000). Seeding the
	// result and overwriting it in the one branch sidesteps that. The IBL
	// path is still evaluated only when enabled, and the value is bit-
	// identical to a direct pbrEvaluateIbl call.
	vec3 result = base_color * flat_strength;
	if (iblEnabled) {
		result = pbrEvaluateIbl(N_world, V_world,
			base_color, metallic, roughness,
			sh,
			tex_spec_cube, smp_spec_cube,
			tex_brdf_lut, smp_brdf_lut,
		max_lod);
	}
	return result;
}

// Two-tier antialiased grid:
//   * minor lines at `cell_size` cadence, slight darkening
//   * major lines every 10 cells, stronger darkening
// `fwidth` of the cell coordinate keeps the line width near 1 screen pixel
// at any distance / viewing angle. Caller passes a flat per-instance
// `cell_size`, the result is suitable as `base_color` for brdf_evaluate.
//
// `line_xz` is the world XZ with the draw origin wrapped to the grid period,
// so the repeating lines stay precise far from the world origin. `axis_xz` is
// the true world XZ for the colored origin axes, which there is only one of, so
// they cannot be wrapped. Far from the origin axis_xz is large and the axes
// vanish, which is what we want (the origin is not in view).
//
// Derivatives are valid here only because the geom shader calls this
// from a branch on a flat per-instance varying (every fragment in a
// quad of a single draw shares the same materialMode).
vec3 proceduralGrid(vec2 line_xz, vec2 axis_xz, vec3 base_color, float cell_size)
{
	vec2 coord_minor = line_xz / cell_size;
	vec2 d_minor = max(fwidth(coord_minor), vec2(1.0e-6));
	vec2 g_minor = abs(fract(coord_minor - 0.5) - 0.5) / d_minor;
	float line_minor = 1.0 - clamp(min(g_minor.x, g_minor.y), 0.0, 1.0);

	vec2 coord_major = line_xz / (cell_size * 10.0);
	vec2 d_major = max(fwidth(coord_major), vec2(1.0e-6));
	vec2 g_major = abs(fract(coord_major - 0.5) - 0.5) / d_major;
	float line_major = 1.0 - clamp(min(g_major.x, g_major.y), 0.0, 1.0);

	vec3 minor_color = base_color * 0.70;
	vec3 major_color = base_color * 0.35;

	vec3 result = mix(base_color, minor_color, line_minor);
	result = mix(result, major_color, line_major);

	// Colored axes: red along +X, blue along +Z. Pixel-wide like the grid
	// lines, gated to the positive half so the origin reads as a corner.
	// Saturated primaries are identical in linear and sRGB, so they need no
	// encoding fixup before the BRDF.
	vec2 d_axis = max(fwidth(axis_xz), vec2(1.0e-6));
	float axis_x = (1.0 - clamp(abs(axis_xz.y) / d_axis.y, 0.0, 1.0)) * step(0.0, axis_xz.x);
	float axis_z = (1.0 - clamp(abs(axis_xz.x) / d_axis.x, 0.0, 1.0)) * step(0.0, axis_xz.y);
	result = mix(result, vec3(1.0, 0.0, 0.0), axis_x);
	result = mix(result, vec3(0.0, 0.0, 1.0), axis_z);
	return result;
}
