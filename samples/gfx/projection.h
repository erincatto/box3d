// World-to-screen projection helper used by the in-world text overlay
// and its unit test. Header-only / pure C so the unit test can pull it in
// without dragging the rest of the renderer along.
//
// Convention matches the renderer: column-major matrices multiplied as
// `M * v`, reverse-Z clip space (near maps to clip-Z = +w, far to 0), Y-up
// view space. Screen pixels follow ImGui's origin convention: (0,0) at the
// top-left of the viewport, Y growing downward.

#pragma once

#include "gfx/utility.h"

#include <stdbool.h>

// Project a world-space point through `view`/`proj` for a viewport of size
// `viewportW` x `viewportH` pixels. Writes the screen-pixel position into
// `*outScreenX`/`*outScreenY` and returns true when the point is visible
// (in front of the near plane and inside the X/Y frustum). Returns false
// without writing the outputs when the point is behind the camera (clip.w
// non-positive) or outside the frustum in either axis.
static inline bool ProjectWorldToScreen( Mat4 view, Mat4 proj, int viewportW, int viewportH, b3Vec3 worldPos, float* outScreenX,
										 float* outScreenY )
{
	Vec4 world = MakeVec4( worldPos.x, worldPos.y, worldPos.z, 1.0f );
	Vec4 viewP = MulMV4( view, world );
	Vec4 clip = MulMV4( proj, viewP );

	// Reverse-Z still uses a positive w for "in front of camera", w is the
	// view-space depth before the proj divide. Reject behind-camera and
	// numerically degenerate cases together.
	if ( clip.w <= 1e-6f )
	{
		return false;
	}

	const float invW = 1.0f / clip.w;
	const float ndcX = clip.x * invW;
	const float ndcY = clip.y * invW;

	if ( ndcX < -1.0f || ndcX > 1.0f )
	{
		return false;
	}

	if ( ndcY < -1.0f || ndcY > 1.0f )
	{
		return false;
	}

	// NDC (-1..1, Y up) -> screen pixels (0..W, 0..H, Y down).
	*outScreenX = ( ndcX * 0.5f + 0.5f ) * (float)viewportW;
	*outScreenY = ( 1.0f - ( ndcY * 0.5f + 0.5f ) ) * (float)viewportH;

	return true;
}
