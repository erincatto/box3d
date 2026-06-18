// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Pick-ray helper.
//
// Build a world-space ray from a screen-pixel coordinate using the latched
// camera state set each frame by RenderFrame / RenderFrameOffscreen.
// Pure-C / header-only so the demo app and the Box3D adapter share one
// implementation without dragging the rest of the renderer in.
//
// Conventions match the rest of the renderer:
//   * Column-major matrices, multiplied as M * v
//   * Reverse-Z clip space (near maps to clip-Z = +w, far to 0)
//   * Mouse (x, y) in framebuffer pixels with origin at the top-left
// (Y grows downward, matches ImGui and sokol_app)
//
// Output matches Box3D's `b3World_CastRayClosest(origin, translation, ...)`
// convention: `*outOrigin` is the world point on the near plane along the
// pick ray, and `*outTranslation` is the unnormalized vector pointing from
// the near to the far plane along that ray. Pass them straight through.
//
// Returns false (and leaves outputs untouched) when the matrices are
// degenerate or the camera state hasn't been initialized (viewportW/H 0).

#pragma once

#include "gfx/renderer.h"
#include "gfx/utility.h"

#include <stdbool.h>

static inline bool BuildPickRay( float mouseX, float mouseY, b3Vec3* outOrigin, b3Vec3* outTranslation )
{
	CameraState s = GetCameraState();
	if ( s.viewportW <= 0 || s.viewportH <= 0 )
	{
		return false;
	}

	// Pixel -> NDC. Origin top-left, Y down. NDC has Y up, X right.
	const float ndcX = ( 2.0f * mouseX / (float)s.viewportW ) - 1.0f;
	const float ndcY = 1.0f - ( 2.0f * mouseY / (float)s.viewportH );

	// Reverse-Z: near plane z = 1, far plane z = 0 in NDC. We pick those
	// two clip-space points (w = 1) and run them through the inverse VP to
	// recover world-space. inv(P*V) = inv(V)*inv(P) = viewInv * projInv,
	// both produced by the camera alongside their forward matrices, no
	// runtime matrix inversion.
	const Mat4 invVP = MulMM4( s.viewInv, s.projInv );

	const Vec4 clipN = MakeVec4( ndcX, ndcY, 1.0f, 1.0f );
	const Vec4 clipF = MakeVec4( ndcX, ndcY, 0.0f, 1.0f );

	const Vec4 wN = MulMV4( invVP, clipN );
	const Vec4 wF = MulMV4( invVP, clipF );

	if ( wN.w == 0.0f || wF.w == 0.0f )
		return false;

	const float invWN = 1.0f / wN.w;
	const float invWF = 1.0f / wF.w;

	b3Vec3 nearWorld = { wN.x * invWN, wN.y * invWN, wN.z * invWN };
	b3Vec3 farWorld = { wF.x * invWF, wF.y * invWF, wF.z * invWF };

	*outOrigin = nearWorld;
	*outTranslation = b3Sub( farWorld, nearWorld );
	return true;
}
