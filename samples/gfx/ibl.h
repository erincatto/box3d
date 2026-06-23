// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Image-based lighting from the procedural sky.
//
// Owns three persistent GPU resources plus a CPU-side cache:
// * BRDF integration LUT (Karis split-sum, RG16F 256^2), generated once
//     at boot. Environment-independent.
// * Sky cubemap with full GGX-prefiltered mip chain (RGBA16F 6x256^2),
//     regenerated when the sun direction or turbidity changes.
// * Diffuse irradiance, 9 spherical-harmonic coefficients per channel,
//     evaluated on the CPU from the closed-form Preetham model. Cached
//     and uploaded into the lit pipelines' ub_pass each frame.
//
// The lit pipelines consume IBL by:
// 1. Calling GetIblSphericalHarmonicCoefficients to get the diffuse SH (copied into ub_pass).
// 2. Binding GetIblSpecularView / GetIblSpecularSampler for the
// prefiltered cubemap and GetIblBrdfLutView / GetIblBrdfLutSampler
//      for the LUT.
// 3. Passing GetIblSpecularMaxLod so the shader can scale roughness
//      to a mip level.
//
// MarkIblDirty + RebuildImageBasedLightingIfDirty are the rebuild hook. The
// renderer marks dirty when SetSun is called, then calls
// RebuildImageBasedLightingIfDirty at the top of DrawScene, the rebuild is a no-op
// when not dirty.

#pragma once

#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Cube faces are 256^2, small enough for cheap regeneration on sun
// change, large enough that mid-roughness prefilter mips read clean.
// 7 mip levels covers all roughness from 0 (mip 0) to 1 (mip 6 = 4^2).
#define GFX_IBL_CUBE_SIZE 256
#define GFX_IBL_CUBE_MIPS 7
#define GFX_IBL_LUT_SIZE 256

void InitImageBasedLighting( const sg_environment* env );
void ShutdownImageBasedLighting( void );

// Mark the cube map + SH as stale. Cheap, the actual GPU regen + CPU SH
// projection happens on the next RebuildImageBasedLightingIfDirty call.
void MarkIblDirty( void );

// If dirty, regenerate the prefiltered sky cube and recompute SH from
// the current sun direction + turbidity. Idempotent when clean. zUp picks
// the sky's up axis, a change forces a rebuild on its own.
void RebuildImageBasedLightingIfDirty( b3Vec3 dirToSun, float turbidity, bool zUp );

// Prefiltered sky cubemap (mip 0 = mirror, mip GFX_IBL_CUBE_MIPS-1 =
// fully rough). Bind once per lit-pipeline draw.
sg_view GetIblSpecularView( void );
sg_sampler GetIblSpecularSampler( void );
float GetIblSpecularMaxLod( void );

// BRDF integration LUT. Sample with (n_dot_v, roughness) UVs.
sg_view GetIblBrdfLutView( void );
sg_sampler GetIblBrdfLutSampler( void );

// Copy the 9 RGB SH coefficients (band 0 through band 2) into `out`.
// Layout matches the lit shaders' ub_pass.sh[9] slot, each coefficient
// is a vec3, written as 9 consecutive b3Vec3.
void GetIblSphericalHarmonicCoefficients( b3Vec3 out[9] );

#ifdef __cplusplus
} // extern "C"
#endif
