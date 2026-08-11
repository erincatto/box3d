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

// Must match SHADOW_CASCADE_COUNT in shaders/common/shadow_cascade.glsl,
// which the renderer static asserts against the generated uniform layout.
// Changing it needs a shader regen (BOX3D_BUILD_SHADERS=ON) to reach the
// committed headers. Four is the ceiling: per-cascade values ride in vec4
// lanes and a fifth would need a second vec4 for each of them.
//
// More cascades cannot sharpen the last one. It always has to reach the far
// split, so its texel is pinned near 2 * k * far / resolution with k set by
// the field of view. What they buy is a finer near cascade and a smaller
// step at each boundary, both at the cost of another shadow pass.
#define SHADOW_CASCADE_COUNT 4
#define SHADOW_RESOLUTION 4096

// Default cascade split range, view-space distance from the camera. Scenes
// smaller than the far end keep the default range, larger ones widen it.
//
// The near end sits nowhere near the camera near plane on purpose. It says
// where the split distribution starts, not where shadows start: the first
// cascade's bounding sphere reaches back past the eye whatever this is, so
// close content is covered either way. Starting out at working distance is
// what puts the whole volume being looked at inside the first cascade,
// rather than spending it on the metre in front of the lens and leaving two
// boundaries sitting in the middle of the scene. The cost is a first cascade
// stretched to reach that far, so it trades sharpness for uniformity.
#define SHADOW_SPLIT_NEAR 10.0f
#define SHADOW_SPLIT_FAR 50.0f

// Ceiling on that widening. The last cascade always has to reach the far
// split, so its texel size is set by this number alone and nothing else
// in the fit affects it. Letting a scene push it out coarsens the far
// cascade until its shadows read as blocks, and widens the step from the
// cascade before it. A scene that genuinely needs more reach should ask
// for it rather than have it inferred from how big its ground is.
#define SHADOW_SPLIT_FAR_MAX 80.0f

void InitShadows( void );
void ShutdownShadows( void );

// Set the world-space direction TO the sun
void SetShadowSunDir( b3Vec3 dirToSun );

// Override the view-space distance range PSSM splits over. Defaults to
// (SHADOW_SPLIT_NEAR, SHADOW_SPLIT_FAR), sufficient for the reference scenes
// but wrong for a scene watched from far off, where the near end wastes whole
// cascades on the empty space in front of the camera. Pass (0, 0) to restore
// defaults.
void SetShadowSplits( float nearViewZ, float farViewZ );

// Fit the cascade far split to a scene of this view-space depth, keeping the
// current near.
void SetShadowSplitFar( float farViewZ );

float GetShadowSplitNear( void );
float GetShadowSplitFar( void );

// Blend between the two PSSM split schemes. 0 puts every boundary at an
// equal slice of distance, 1 at an equal multiplicative step. Raising it
// tightens the near cascades at the cost of dragging every boundary toward
// the camera, so the coarse far cascade takes over sooner.
void SetShadowSplitLambda( float lambda );
float GetShadowSplitLambda( void );

// Recompute per-cascade light-space matrices and split distances for the
// camera frustum described by viewInv + proj. viewInv (the camera's world
// matrix) supplies eye + basis directly, so the slice-corner walk does no
// matrix inversion. proj supplies fovY + aspect via proj[1][1] / proj[0][0].
// Cascade slices are fit to the camera frustum and snapped to texel
// boundaries to avoid swimming edges as the camera moves.
void FitShadows( const Mat4* viewInv, const Mat4* proj );

// World-space AABB enclosing every shape that can cast into the cascades:
// each slice's bounding sphere swept toward the sun by the caster margin.
// It reaches well outside the camera frustum, which is the point. A host
// culling its draw submission to the visible volume drops casters that
// belong in the shadow map, and their shadows vanish.
//
// Derived from the camera alone, so it can be queried before FitShadows
// runs for the frame. Takes projInv because that is what the draw path
// already carries. Bounds come back in the same frame viewInv maps into,
// so a caller working against a shifted draw origin must offset them.
void GetShadowCasterBounds( const Mat4* viewInv, const Mat4* projInv, b3Vec3* loOut, b3Vec3* hiOut );

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

// One shadow texel measured in world length units for a cascade. Lit
// shaders scale the normal offset by this so every cascade biases by the
// same visual amount regardless of how much world it covers. The ratio
// against cascade 0 also shrinks the PCF kernel in the coarser cascades,
// which keeps the penumbra a constant world width instead of jumping at
// every cascade boundary.
float GetCascadeTexelWorld( int cascade );

// Residual comparison bias for a cascade, in shadow clip-Z units. The
// normal offset carries the slope term, this only has to cover depth
// quantization. Sized on the CPU because shadow clip-Z spans the whole
// orthographic range, which differs per cascade.
float GetCascadeDepthBias( int cascade );

// Bound by lit pipelines once for sampling across all cascades. The view
// is the depth array's full-array sampling view. Per-cascade selection
// happens inside the shader by passing `cascade` as the layer index.
sg_view GetShadowSampleView( void );
sg_sampler GetShadowSampler( void );

// One-texel size in shadow UV space (1 / SHADOW_RESOLUTION). Lit shaders
// use this to size PCF taps and the cascade-edge reject.
float GetShadowTexelSize( void );

#ifdef __cplusplus
} // extern "C"
#endif
