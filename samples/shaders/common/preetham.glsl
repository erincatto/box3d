// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Preetham/Shirley/Smits 1999 analytic daylight sky.
//
// preethamSky(view_dir, sun_dir, turbidity) returns linear-sRGB radiance
// for an unoccluded sky direction. Both directions must be unit-length,
// Y is up. Output is unmultiplied, callers apply the luminance scale
// and below-horizon fade themselves, or call preethamSkyScaled which
// does both.
//
// Preetham diverges at and below the horizon. The helper clamps the
// view direction to slightly above the horizon and clamps sun.y to the
// upper hemisphere so the Perez/zenith math stays finite when the sun
// is below the horizon, the caller is responsible for fading output to
// zero in that case.

const float PREETHAM_PI = 3.14159265358979323846;

// Empirical luminance scale, chosen to land near the target ~10:1
// sun:sky ratio under the renderer's sun_color premultiplied-by-pi
// convention. Shared between the sky backdrop (sky.glsl) and the IBL
// sky cubemap (sky_to_cube.glsl) so both consumers see the same sky.
const float PREETHAM_LUMINANCE_SCALE = 0.06;

// Perez luminance distribution:
//   F(theta, gamma) = (1 + A * exp(B / cos(theta)))
// * (1 + C * exp(D * gamma) + E * cos^2(gamma))
// theta = view angle from zenith, gamma = view angle from sun direction.
// cos_theta is clamped >= 0.01 to avoid the exp(B / 0) singularity at
// the horizon (Preetham is invalid there anyway).
float preethamPerez(float cos_theta, float cos_gamma, float gamma,
	float A, float B, float C, float D, float E)
{
	float term1 = 1.0 + A * exp(B / max(cos_theta, 0.01));
	float term2 = 1.0 + C * exp(D * gamma) + E * cos_gamma * cos_gamma;
	return term1 * term2;
}

// CIE xyY -> XYZ.
vec3 preethamXyY_to_XYZ(float x, float y, float Y)
{
	float yy = max(y, 1.0e-5);
	return vec3(Y * x / yy, Y, Y * (1.0 - x - y) / yy);
}

// CIE XYZ (D65) -> linear sRGB.
vec3 preethamXYZ_to_linear_srgb(vec3 c)
{
	return vec3(
		dot(c, vec3(3.2404542, - 1.5371385, - 0.4985314)),
		dot(c, vec3(-0.9692660, 1.8760108, 0.0415560)),
		dot(c, vec3(0.0556434, - 0.2040259, 1.0572252))
	);
}

vec3 preethamSky(vec3 view_dir, vec3 sun_dir, float turbidity)
{
	// Clamp sun's elevation to the upper hemisphere so the Perez/zenith
	// math stays finite below the horizon. The horizontal components
	// sun_dir are intentionally kept unmodified, cos_gamma reads the
	// original direction so the sun's azimuth is preserved.
	float sun_y = clamp(sun_dir.y, 0.0, 1.0);
	
	// Pin the look-up direction to slightly above the horizon when the
	// view points downward. Preetham diverges below the horizon, and
	// this gives the sky a sensible colour where the ground would
	// otherwise occlude it (e.g. scenes with no ground in the lower
	// portion of frame).
	vec3 view_clamped = normalize(vec3(view_dir.x, max(view_dir.y, 0.0) + 0.01, view_dir.z));
		
	float cos_theta = max(view_clamped.y, 0.0); // zenith angle of view
	float cos_gamma = clamp(dot(sun_dir, view_clamped), - 1.0, 1.0); // sun-to-view angle
	float gamma = acos(cos_gamma);
	float sun_theta = acos(sun_y);
		
	// Perez coefficients as functions of turbidity T (Preetham table 2)
	float T = max(turbidity, 1.0);
	float T2 = T * T;
		
	// Y (luminance):
	float A_Y = 0.1787 * T - 1.4630;
	float B_Y = -0.3554 * T + 0.4275;
	float C_Y = -0.0227 * T + 5.3251;
	float D_Y = 0.1206 * T - 2.5771;
	float E_Y = -0.0670 * T + 0.3703;
	// x (chromaticity):
	float A_x = -0.0193 * T - 0.2592;
	float B_x = -0.0665 * T + 0.0008;
	float C_x = -0.0004 * T + 0.2125;
	float D_x = -0.0641 * T - 0.8989;
	float E_x = -0.0033 * T + 0.0452;
	// y (chromaticity):
	float A_y = -0.0167 * T - 0.2608;
	float B_y = -0.0950 * T + 0.0092;
	float C_y = -0.0079 * T + 0.2102;
	float D_y = -0.0441 * T - 1.6537;
	float E_y = -0.0109 * T + 0.0529;
		
	// Zenith chromaticities (Preetham Eq. A.3, polynomial in T and sun_theta).
	float ts = sun_theta;
	float ts2 = ts * ts;
	float ts3 = ts2 * ts;
		
	float x_z =
	(0.00166 * ts3 - 0.00375 * ts2 + 0.00209 * ts) * T2 +
	(-0.02903 * ts3 + 0.06377 * ts2 - 0.03202 * ts + 0.00394) * T +
	(0.11693 * ts3 - 0.21196 * ts2 + 0.06052 * ts + 0.25886);
		
	float y_z =
	(0.00275 * ts3 - 0.00610 * ts2 + 0.00317 * ts) * T2 +
	(-0.04214 * ts3 + 0.08970 * ts2 - 0.04153 * ts + 0.00516) * T +
	(0.15346 * ts3 - 0.26756 * ts2 + 0.06670 * ts + 0.26688);
		
	// Zenith luminance (Preetham Eq. A.2). Result in kcd/m^2.
	float chi = (4.0 / 9.0 - T / 120.0) * (PREETHAM_PI - 2.0 * sun_theta);
	float Y_z = (4.0453 * T - 4.9710) * tan(chi) - 0.2155 * T + 2.4192;
		
	// Normalize: F(theta=0, gamma=sun_theta) at zenith equals zenith
	// values, so divide by perez(zenith) and multiply by perez(view).
	float cos_zen_gamma = sun_y; // dot(zenith, sun_dir) = sun_dir.y
	float pY_v = preethamPerez(cos_theta, cos_gamma, gamma, A_Y, B_Y, C_Y, D_Y, E_Y);
	float px_v = preethamPerez(cos_theta, cos_gamma, gamma, A_x, B_x, C_x, D_x, E_x);
	float py_v = preethamPerez(cos_theta, cos_gamma, gamma, A_y, B_y, C_y, D_y, E_y);
	float pY_z = preethamPerez(1.0, cos_zen_gamma, sun_theta, A_Y, B_Y, C_Y, D_Y, E_Y);
	float px_z = preethamPerez(1.0, cos_zen_gamma, sun_theta, A_x, B_x, C_x, D_x, E_x);
	float py_z = preethamPerez(1.0, cos_zen_gamma, sun_theta, A_y, B_y, C_y, D_y, E_y);
		
	float Y = max(Y_z * pY_v / max(pY_z, 1.0e-5), 0.0);
	float x = x_z * px_v / max(px_z, 1.0e-5);
	float y = y_z * py_v / max(py_z, 1.0e-5);
		
	vec3 XYZ = preethamXyY_to_XYZ(x, y, Y);
	vec3 rgb = preethamXYZ_to_linear_srgb(XYZ);
	return max(rgb, vec3(0.0));
}
	
// Rotate a sky direction from simulation space into the model's Y-up
// frame. Y-up sims pass through. Z-up sims need Rx(-90), (x,y,z) ->
// (x, z, -y), the same map the renderer folds into its view transform so
// sim +Z becomes view up. zUp is 0.0 or 1.0. View ray and sun rotate
// together so their relative angle is preserved, only horizon and zenith
// reorient.
vec3 preethamToYUp(vec3 v, float zUp)
{
	return mix(v, vec3(v.x, v.z, - v.y), zUp);
}

// Convenience wrapper: raw Preetham scaled by the empirical luminance
// factor and the caller's below-horizon fade weight (1 above horizon,
// 0 below). The sky backdrop shader and the IBL sky cubemap both call
// this so they're guaranteed to agree pixel-for-pixel. zUp reorients the
// model for a z-up simulation, see preethamToYUp.
vec3 preethamSkyScaled(vec3 view_dir, vec3 sun_dir, float turbidity, float fade, float zUp)
{
	vec3 v = preethamToYUp(view_dir, zUp);
	vec3 s = preethamToYUp(sun_dir, zUp);
	return preethamSky(v, s, turbidity) * PREETHAM_LUMINANCE_SCALE * fade;
}
	