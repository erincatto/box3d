// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw_bridge.h"

#include "gfx/draw.h"
#include "gfx/draw_overlay.h"
#include "gfx/text.h"

void BridgeSphere( b3Transform transform, float radius, Vec4 color )
{
	DrawSphere( transform, radius, color );
}

void BridgeCapsule( b3Transform transform, float halfLength, float radius, Vec4 color )
{
	DrawCapsule( transform, halfLength, radius, color );
}

void BridgeCube( b3Transform transform, b3Vec3 scale, Vec4 color )
{
	DrawCube( transform, scale, color );
}

void BridgePoint( b3Vec3 point, float sizePx, Vec4 color )
{
	DrawPointEx( point, color, sizePx, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );
}

void BridgeLine( b3Vec3 p1, b3Vec3 p2, Vec4 color )
{
	DrawLine( p1, p2, color );
}

void BridgeArrow( b3Vec3 p1, b3Vec3 p2, Vec4 color )
{
	DrawArrow( p1, p2, color );
}

void BridgeCross( b3Vec3 center, float size, Vec4 color )
{
	DrawCross( center, size, color );
}

void BridgeAabb( b3Vec3 lower, b3Vec3 upper, Vec4 color )
{
	DrawAabb( lower, upper, color );
}

void BridgeAxes( b3Transform transform, float size )
{
	DrawAxes( transform, size );
}

void BridgeGrid( b3Vec3 center, b3Vec3 normal, float halfExtent, int divisions, Vec4 color )
{
	DrawGrid( center, normal, halfExtent, divisions, color );
}

void BridgeTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, Vec4 color )
{
	DrawTriangle( a, b, c, color );
}

void BridgeString( b3Vec3 worldPos, Vec4 color, const char* text )
{
	DrawString( worldPos, color, text );
}
