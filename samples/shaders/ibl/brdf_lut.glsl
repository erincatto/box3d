// BRDF integration LUT (Karis 2013 split-sum, environment term).
//
// Output is RG16F at 256^2. R = scale, G = bias for the split-sum approx:
//	integrated_specular = prefiltered_env(R(N,V), roughness) * (F0 * lut.r + lut.g)
// where lut is sampled at (n_dot_v, roughness). Generated once at boot,
// independent of sun, environment, and view.
//
// The integral is importance-sampled GGX over 1024 Hammersley samples per
// pixel. Heavy for a single fragment, but it runs once and into
// a 256^2 target so the total cost is bounded (~67M iterations). The result
// is environment-independent, it stays valid for the lifetime of the
// renderer.
//
// VS sets v_uv to the SAMPLE-time UV that addresses this pixel's content,
// not the raw NDC-derived UV. sokol-shdc does NOT normalize render-target
// V orientation across backends: on GL a fragment at NDC.y=+1 stores at
// the texture row that sampling at V=1 returns, on D3D11/Metal/WGPU the
// same fragment stores at the row that sampling at V=0 returns. The FS
// uses v_uv.y as the integration roughness, so for sampling at
// (n_dot_v, roughness) to round-trip we flip via a backend-conditional
// uvYSign (+1 GL, -1 D3D11/Metal/WGPU). Same pattern as gtao.glsl /
// cube.glsl's cascade_far_view_z.w.

#pragma sokol @module brdf_lut

#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs

layout(binding = 0)uniform ub_brdf
{
	// .x = uvYSign (+1 GL, -1 D3D11/Metal/WGPU)
	vec4 params; 
};

out vec2 v_uv;

void main()
{
	vec2 ndc = vec2(float((gl_VertexIndex << 1)& 2),
	float(gl_VertexIndex & 2)) * 2.0 - 1.0;
	v_uv = vec2(ndc.x * 0.5 + 0.5, 0.5 + ndc.y * 0.5 * params.x);
	gl_Position = vec4(ndc, 0.0, 1.0);
}
#pragma sokol @end

#pragma sokol @fs fs

in vec2 v_uv;
out vec2 out_color;

const float BRDF_LUT_PI = 3.14159265358979323846;
const uint NUM_SAMPLES = 1024u;

// Van der Corput radical inverse, bit-reverse trick for low-discrepancy
// sequences. Together with the index, gives a Hammersley sample on [0,1)^2.
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

// GGX importance sampling, returns a microfacet half-vector H in tangent
// space (z-up). For BRDF integration N = (0,0,1) so tangent = world space.
vec3 importanceSampleGGX(vec2 Xi, float alpha)
{
	float a2 = alpha * alpha;
	float phi = 2.0 * BRDF_LUT_PI * Xi.x;
	float cos_theta = sqrt((1.0 - Xi.y) / (1.0 + (a2 - 1.0) * Xi.y));
	float sin_theta = sqrt(max(1.0 - cos_theta * cos_theta, 0.0));
	return vec3(sin_theta * cos(phi), sin_theta * sin(phi), cos_theta);
}

// Smith G with Karis' IBL remap: k = a^2/2 (rather than (a+1)^2/8 used for
// direct lighting). This is the form that pairs correctly with importance-
// sampled GGX integration.
float smithGggxIbl(float n_dot_v, float n_dot_l, float alpha)
{
	float k = (alpha * alpha) * 0.5;
	float gv = n_dot_v / (n_dot_v * (1.0 - k) + k);
	float gl = n_dot_l / (n_dot_l * (1.0 - k) + k);
	return gv * gl;
}

vec2 integrateBRDF(float n_dot_v, float roughness)
{
	// V is in the (x,z) plane with N = +Z. n_dot_v fully determines it.
	vec3 V = vec3(sqrt(max(1.0 - n_dot_v * n_dot_v, 0.0)), 0.0, n_dot_v);

	float A = 0.0;
	float B = 0.0;
	float alpha = roughness * roughness;

	for (uint i = 0u; i < NUM_SAMPLES; ++i)
	{
		vec2 Xi = hammersley(i, NUM_SAMPLES);
		vec3 H = importanceSampleGGX(Xi, alpha);
		vec3 L = 2.0 * dot(V, H) * H - V;

		float n_dot_l = max(L.z, 0.0);
		float n_dot_h = max(H.z, 0.0);
		float v_dot_h = max(dot(V, H), 0.0);

		if (n_dot_l > 0.0)
		{
			float G = smithGggxIbl(n_dot_v, n_dot_l, alpha);
			
			// pdf-cancelled visibility: G * v_dot_h / (n_dot_h * n_dot_v).
			float G_Vis = (G * v_dot_h) / max(n_dot_h * n_dot_v, 1.0e-5);

			// max(0, ...): v_dot_h is max -clamped to [0, inf) above but
			// unit-vector drift could in principle push it just over 1.
			// Wrap to dodge HLSL's pow(negative, n) warning (X3571).
			float Fc = pow(max(0.0, 1.0 - v_dot_h), 5.0);
			
			A += (1.0 - Fc) * G_Vis;
			B += Fc * G_Vis;
		}
	}

	return vec2(A, B) / float(NUM_SAMPLES);
}

void main()
{
	// v_uv ranges over [0.5/N, (N-0.5)/N] across the LUT, never touches 0
	// or 1 exactly, which keeps the integration well-defined at the
	// (n_dot_v=0, roughness=0) corner.
	float n_dot_v = max(v_uv.x, 1.0e-3);
	float roughness = max(v_uv.y, 1.0e-3);
	out_color = integrateBRDF(n_dot_v, roughness);
}
#pragma sokol @end

#pragma sokol @program build vs fs
