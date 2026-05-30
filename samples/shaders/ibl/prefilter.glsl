// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// GGX prefilter fills the radiance mip chain of the IBL specular
// cubemap from the raw Preetham sky cube produced by sky_to_cube.glsl.
//
// One draw per face per mip. The C side passes the destination face's
// (right, up, forward) basis and the target mip's roughness. The FS
// importance-samples the GGX lobe centered on N (the per-pixel face
// direction), reads the input cube, and accumulates n.l-weighted
// radiance. Karis split-sum convention: assume V = N (acceptable for
// IBL, the lost direction-dependence becomes part of the BRDF LUT).
//
// At roughness 0 (mirror, mip 0) the GGX lobe collapses to a delta and
// we read the input cube directly. This both speeds up mip 0 and avoids
// the artefact of GGX importance sampling returning a numerically
// imprecise direction near the singularity.
//
// @module prefilter prefixes the generated typedefs so this header
// coexists in ibl.c alongside brdf_lut.glsl.h and sky_to_cube.glsl.h.

#pragma sokol @module prefilter

#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs

out vec2 v_uv;

void main()
{
	vec2 ndc = vec2(float((gl_VertexIndex << 1) & 2),
	float(gl_VertexIndex & 2)) * 2.0 - 1.0;
	v_uv = ndc * 0.5 + 0.5;
	gl_Position = vec4(ndc, 0.0, 1.0);
}
#pragma sokol @end

#pragma sokol @fs fs

layout(binding = 0) uniform ub_pass
{
	vec4 face_right; // .xyz = world-space face right axis
	vec4 face_up; // .xyz = world-space face up axis
	vec4 face_forward; // .xyz = world-space face forward axis
	vec4 params; // .x = roughness in [0,1], .yzw = unused
};

// Input is the unprefiltered sky cube. Filtering sampler with linear
// mips. Although we sample LOD 0 here, the trilinear filter still
// kicks in on the texel-level lookup.
#pragma sokol @image_sample_type tex_sky float
layout(binding = 0)uniform textureCube tex_sky;
#pragma sokol @sampler_type smp_sky filtering
layout(binding = 0)uniform sampler smp_sky;

in vec2 v_uv;
out vec4 out_color;

const float PREFILTER_PI = 3.14159265358979323846;
const uint NUM_SAMPLES = 1024u;

float radicalInverseVdC(uint bits)
{
	bits = (bits << 16u)|(bits >> 16u);
	bits = ((bits & 0x55555555u) << 1u)|((bits & 0xAAAAAAAAu) >> 1u);
	bits = ((bits & 0x33333333u) << 2u)|((bits & 0xCCCCCCCCu) >> 2u);
	bits = ((bits & 0x0F0F0F0Fu) << 4u)|((bits & 0xF0F0F0F0u) >> 4u);
	bits = ((bits & 0x00FF00FFu) << 8u)|((bits & 0xFF00FF00u) >> 8u);
	return float(bits) * 2.3283064365386963e-10;
}

vec2 hammersley(uint i, uint n)
{
	return vec2(float(i) / float(n), radicalInverseVdC(i));
}

vec3 importanceSampleGGX(vec2 Xi, vec3 N, float roughness)
{
	float a = roughness * roughness;
	float phi = 2.0 * PREFILTER_PI * Xi.x;
	float cos_theta = sqrt((1.0 - Xi.y) / (1.0 + (a * a - 1.0) * Xi.y));
	float sin_theta = sqrt(max(1.0 - cos_theta * cos_theta, 0.0));
	
	vec3 H_tan = vec3(sin_theta * cos(phi), sin_theta * sin(phi), cos_theta);

	// Build an orthonormal basis around N.
	vec3 up = abs(N.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
	vec3 tangent = normalize(cross(up, N));
	vec3 bitan = cross(N, tangent);

	return normalize(tangent * H_tan.x + bitan * H_tan.y + N * H_tan.z);
}

void main()
{
	vec2 uv = v_uv * 2.0 - 1.0;
	vec3 N = normalize(face_forward.xyz + uv.x * face_right.xyz + uv.y * face_up.xyz);
	vec3 V = N; // Karis split-sum, assume V = N

	float roughness = params.x;
	if (roughness < 0.001)
	{
		// Mirror: GGX lobe is a delta, just sample the input direction.
		out_color = vec4(texture(samplerCube(tex_sky, smp_sky), N).rgb, 1.0);
		return;
	}

	vec3 prefiltered = vec3(0.0);
	float total_w = 0.0;

	for(uint i = 0u; i < NUM_SAMPLES; ++i)
	{
		vec2 Xi = hammersley(i, NUM_SAMPLES);
		vec3 H = importanceSampleGGX(Xi, N, roughness);
		vec3 L = normalize(2.0 * dot(V, H) * H - V);
		float n_dot_l = max(dot(N, L), 0.0);
		if (n_dot_l > 0.0)
		{
			prefiltered += texture(samplerCube(tex_sky, smp_sky), L).rgb * n_dot_l;
			total_w += n_dot_l;
		}
	}
	out_color = vec4(prefiltered / max(total_w, 1.0e-4), 1.0);
}
#pragma sokol @end

#pragma sokol @program build vs fs
