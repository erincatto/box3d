// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Cascaded-shadow-map infrastructure.
//
// One depth array texture (D32F) holds all cascades, each cascade gets its
// own depth-only view used as the depth attachment for that cascade's
// shadow pass. A single comparison sampler (LESS_EQUAL with linear PCF)
// is shared by every lit shader that reads from the array.
//
// Light-space matrices and split distances are computed by FitShadows
// each frame from the camera frustum and the sun direction.
//
// Coordinate convention inside shadow space:
// - Standard Z (not reverse-Z): 0 at near, 1 at far. The shadow pass
// clears depth to 1.0, fragments closer to the light overwrite with
// smaller values, the sampler compares LESS_EQUAL to determine "this
// fragment is the closest one, so lit".
// - Right-handed orthographic projection, 0..1 clip-Z.
//
// Public surface:
// InitShadows / ShutdownShadows, lifecycle.
// SetShadowSunDir, store the direction the renderer will use when
// fitting cascades.
// FitShadows, recompute cascade matrices and split distances for a
// given camera frustum.
// BeginShadowPass / EndShadowPass, open/close the depth pass per cascade.
// GetShadowSampleView / GetShadowSampler, for lit pipelines to
// bind once and sample across all cascades.
// GetCascadeMatrix / GetCascadeFarViewZ, for lit
// shaders to transform world positions and select the right cascade.

#pragma once

#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define SHADOW_CASCADE_COUNT 3
#define SHADOW_RESOLUTION 2048

// Default cascade far split, view-space distance from the camera. Scenes
// smaller than this keep the default range, larger ones widen it.
#define SHADOW_SPLIT_FAR 50.0f

void InitShadows( void );
void ShutdownShadows( void );

// Set the world-space direction TO the sun
void SetShadowSunDir( b3Vec3 dirToSun );

// Override the view-space distance range PSSM splits over. Defaults to
// (0.1, SHADOW_SPLIT_FAR), sufficient for the reference scenes but too short
// for larger ones. The split blend is derived from the range, so a wider far
// keeps the near cascade tight. Pass (0, 0) to restore defaults.
void SetShadowSplits( float nearViewZ, float farViewZ );

// Fit the cascade far split to a scene of this view-space depth, keeping the
// current near and recomputing the split blend.
void SetShadowSplitFar( float farViewZ );

// Recompute per-cascade light-space matrices and split distances for the
// camera frustum described by viewInv + proj. viewInv (the camera's world
// matrix) supplies eye + basis directly, so the slice-corner walk does no
// matrix inversion. proj supplies fovY + aspect via proj[1][1] / proj[0][0].
// Cascade slices are fit to the camera frustum and snapped to texel
// boundaries to avoid swimming edges as the camera moves.
void FitShadows( const Mat4* viewInv, const Mat4* proj );

// Begin / end the depth-only render pass for a cascade. Bind this as
// the surrounding pass before issuing caster draws into the cascade.
void BeginShadowPass( int cascade );
void EndShadowPass( void );

// Light-space view-proj matrix for a cascade. Lit shaders multiply a
// world-space position by this to get shadow-space coordinates.
Mat4 GetCascadeMatrix( int cascade );

// Far view-space Z (positive value, distance from camera) for the end
// cascade. Lit shaders compare a fragment's view-space depth against
// these to pick which cascade to sample.
float GetCascadeFarViewZ( int cascade );

// Bound by lit pipelines once for sampling across all cascades. The view
// is the depth array's full-array sampling view. Per-cascade selection
// happens inside the shader by passing `cascade` as the layer index.
sg_view GetShadowSampleView( void );
sg_sampler GetShadowSampler( void );

// One-texel size in shadow space (1 / SHADOW_RESOLUTION). Lit shaders
// use this to size PCF taps and slope-scale bias.
float GetShadowTexelSize( void );

#ifdef __cplusplus
} // extern "C"
#endif
