// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

// How a line or point reacts when scene geometry sits in front of it.
typedef enum OverlayOcclusionMode
{
	// depth tested
	OVERLAY_OCCLUSION_HIDE = 0,

	// reduced alpha
	OVERLAY_OCCLUSION_DIM = 1,

	// screen-space dash pattern
	OVERLAY_OCCLUSION_DASHED = 2,
} OverlayOcclusionMode;

// How to interpret the `thickness` (lines) or `size` (points) argument
typedef enum OverlayThicknessUnit
{
	// screen-space pixels
	OVERLAY_THICKNESS_PIXELS = 0,

	// world-space meters
	OVERLAY_THICKNESS_WORLD = 1,
} OverlayThicknessUnit;

// Whether a shape draw call participates in the shadow caster pass.
typedef enum TransparentShadowCast
{
	// regular shadow
	TRANSPARENT_SHADOW_FULL = 0,

	// disable shadow
	TRANSPARENT_SHADOW_NONE = 1,
} TransparentShadowCast;
