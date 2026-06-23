// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Procedural sky
//
// Renders a Preetham analytic daylight skybox into the main lit pass after
// all opaque shapes. Uses a fullscreen triangle drawn at the reverse-Z far
// plane (NDC z = 0) with depth test GREATER_EQUAL and depth-write off, so
// pixels covered by opaque geometry pass through unchanged and untouched
// pixels (still at the cleared depth of 0) receive the sky.
//
// The sun direction is the renderer's single source of truth, the shadow
// caster, direct lighting, and sky all consume the same world-space vector
// (set via SetSun). Below-horizon fade is computed C-side and passed in
// alongside the direction so the FS doesn't need to know the threshold.

#pragma once

#include "gfx/utility.h"
#include "sokol_gfx.h"

#ifdef __cplusplus
extern "C"
{
#endif

void InitSky( const sg_environment* env );
void ShutdownSky( void );

// Below-horizon fade weight in [0, 1], 1 well above the horizon, 0
// below it, smoothstep between sin(2 deg) and sin(5 deg). The IBL cubemap
// renderer reads this so its sky matches the backdrop exactly when the
// sun is near the horizon. Elevation is sin = dirToSun.y for a y-up sim,
// dirToSun.z for a z-up sim, selected by zUp.
float SkySunFadeWeight( b3Vec3 dirToSun, bool zUp );

// Issue the sky draw. Call inside the main lit pass after all opaque
// shape draws. Parameters:
//   dirToSun     world-space direction TO the sun, renormalized internally,
//                may be the same vector handed to SetSun.
//   turbidity    Preetham T (project spec = 2.2).
//   cameraPos    world-space camera position, the FS reconstructs view
//                direction as (world_far_point - cameraPos).
//   invViewProj  inverse of proj * view for the current frame, the VS
//                unprojects NDC corners through this to get world rays.
//   zUp          simulation up axis, reorients the sky model for z-up.
void DrawSky( b3Vec3 dirToSun, float turbidity, b3Vec3 cameraPos, Mat4 invViewProj, bool zUp );

#ifdef __cplusplus
} // extern "C"
#endif
