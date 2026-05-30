// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/draw.h"

#include "gfx/debug_shapes.h"
#include "gfx/overlay.h"
#include "gfx/renderer.h"

void DrawCubeEx( b3Transform transform, b3Vec3 scale, Vec4 baseColor, float metallic, float roughness, TransparentShadowCast shadowCast )
{
	AppendCube( transform, scale, baseColor, metallic, roughness, shadowCast );
}

void DrawCube( b3Transform transform, b3Vec3 scale, Vec4 baseColor )
{
	DrawCubeEx( transform, scale, baseColor, DEFAULT_METALLIC, DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_FULL );
}

void DrawSphereEx( b3Transform transform, float radius, Vec4 baseColor, float metallic, float roughness, TransparentShadowCast shadowCast )
{
	AppendSphere( transform, radius, baseColor, metallic, roughness, shadowCast );
}

void DrawSphere( b3Transform transform, float radius, Vec4 baseColor )
{
	DrawSphereEx( transform, radius, baseColor, DEFAULT_METALLIC, DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_FULL );
}

void DrawCapsuleEx( b3Transform transform, float halfLength, float radius, Vec4 baseColor, float metallic, float roughness,
				   TransparentShadowCast shadowCast )
{
	AppendCapsule( transform, halfLength, radius, baseColor, metallic, roughness, shadowCast );
}

void DrawCapsule( b3Transform transform, float halfLength, float radius, Vec4 baseColor )
{
	DrawCapsuleEx( transform, halfLength, radius, baseColor, DEFAULT_METALLIC, DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_FULL );
}

void DrawLineEx( b3Vec3 a, b3Vec3 b, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit, OverlayOcclusionMode occlusionMode )
{
	OverlayAppendLine( a, b, color, thickness, thicknessUnit, occlusionMode );
}

void DrawLine( b3Vec3 a, b3Vec3 b, Vec4 color )
{
	DrawLineEx( a, b, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

void DrawPointEx( b3Vec3 p, Vec4 color, float size, OverlayThicknessUnit sizeUnit, OverlayOcclusionMode occlusionMode )
{
	OverlayAppendPoint( p, color, size, sizeUnit, occlusionMode );
}

void DrawPoint( b3Vec3 p, Vec4 color )
{
	DrawPointEx( p, color, DEFAULT_POINT_SIZE_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

void DrawArrowEx( b3Vec3 a, b3Vec3 b, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit, OverlayOcclusionMode occlusionMode,
				 float headLengthFrac )
{
	DrawLineEx( a, b, color, thickness, thicknessUnit, occlusionMode );

	b3Vec3 shaft = b3Sub( b, a );
	float shaftLen = b3Length( shaft );
	if ( shaftLen < 1e-6f )
	{
		return;
	}
	b3Vec3 dir = { shaft.x / shaftLen, shaft.y / shaftLen, shaft.z / shaftLen };
	b3Vec3 perp = b3Perp( dir );
	float headLen = shaftLen * headLengthFrac;

	b3Vec3 backFromTip = b3MulSV( -headLen, dir );
	b3Vec3 sideStep = b3MulSV( headLen * 0.5f, perp );
	b3Vec3 tip1 = b3Add( b, b3Add( backFromTip, sideStep ) );
	b3Vec3 tip2 = b3Add( b, b3Sub( backFromTip, sideStep ) );
	DrawLineEx( b, tip1, color, thickness, thicknessUnit, occlusionMode );
	DrawLineEx( b, tip2, color, thickness, thicknessUnit, occlusionMode );
}

void DrawArrow( b3Vec3 a, b3Vec3 b, Vec4 color )
{
	DrawArrowEx( a, b, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE,
				DEFAULT_ARROW_HEAD_FRAC );
}

void DrawCrossEx( b3Vec3 center, float size, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode )
{
	float h = size * 0.5f;
	DrawLineEx( (b3Vec3){ center.x - h, center.y, center.z }, (b3Vec3){ center.x + h, center.y, center.z }, color, thickness,
			   thicknessUnit, occlusionMode );
	DrawLineEx( (b3Vec3){ center.x, center.y - h, center.z }, (b3Vec3){ center.x, center.y + h, center.z }, color, thickness,
			   thicknessUnit, occlusionMode );
	DrawLineEx( (b3Vec3){ center.x, center.y, center.z - h }, (b3Vec3){ center.x, center.y, center.z + h }, color, thickness,
			   thicknessUnit, occlusionMode );
}

void DrawCross( b3Vec3 center, float size, Vec4 color )
{
	DrawCrossEx( center, size, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

void DrawAabbEx( b3Vec3 mn, b3Vec3 mx, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				OverlayOcclusionMode occlusionMode )
{
	// 8 corners of the box.
	b3Vec3 c000 = { mn.x, mn.y, mn.z };
	b3Vec3 c100 = { mx.x, mn.y, mn.z };
	b3Vec3 c010 = { mn.x, mx.y, mn.z };
	b3Vec3 c110 = { mx.x, mx.y, mn.z };
	b3Vec3 c001 = { mn.x, mn.y, mx.z };
	b3Vec3 c101 = { mx.x, mn.y, mx.z };
	b3Vec3 c011 = { mn.x, mx.y, mx.z };
	b3Vec3 c111 = { mx.x, mx.y, mx.z };

	// 4 bottom edges (y = mn.y).
	DrawLineEx( c000, c100, color, thickness, thicknessUnit, occlusionMode );
	DrawLineEx( c100, c101, color, thickness, thicknessUnit, occlusionMode );
	DrawLineEx( c101, c001, color, thickness, thicknessUnit, occlusionMode );
	DrawLineEx( c001, c000, color, thickness, thicknessUnit, occlusionMode );
	// 4 top edges (y = mx.y).
	DrawLineEx( c010, c110, color, thickness, thicknessUnit, occlusionMode );
	DrawLineEx( c110, c111, color, thickness, thicknessUnit, occlusionMode );
	DrawLineEx( c111, c011, color, thickness, thicknessUnit, occlusionMode );
	DrawLineEx( c011, c010, color, thickness, thicknessUnit, occlusionMode );
	// 4 vertical edges.
	DrawLineEx( c000, c010, color, thickness, thicknessUnit, occlusionMode );
	DrawLineEx( c100, c110, color, thickness, thicknessUnit, occlusionMode );
	DrawLineEx( c101, c111, color, thickness, thicknessUnit, occlusionMode );
	DrawLineEx( c001, c011, color, thickness, thicknessUnit, occlusionMode );
}

void DrawAabb( b3Vec3 mn, b3Vec3 mx, Vec4 color )
{
	DrawAabbEx( mn, mx, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

void DrawAxesEx( b3Transform transform, float size, float thickness, OverlayThicknessUnit thicknessUnit, OverlayOcclusionMode occlusionMode )
{
	b3Vec3 origin = transform.p;
	b3Matrix3 basis = b3MakeMatrixFromQuat( transform.q );
	DrawLineEx( origin, b3MulAdd( origin, size, basis.cx ), MakeVec4( 1.0f, 0.0f, 0.0f, 1.0f ), thickness, thicknessUnit,
			   occlusionMode );
	DrawLineEx( origin, b3MulAdd( origin, size, basis.cy ), MakeVec4( 0.0f, 1.0f, 0.0f, 1.0f ), thickness, thicknessUnit,
			   occlusionMode );
	DrawLineEx( origin, b3MulAdd( origin, size, basis.cz ), MakeVec4( 0.0f, 0.0f, 1.0f, 1.0f ), thickness, thicknessUnit,
			   occlusionMode );
}

void DrawAxes( b3Transform transform, float size )
{
	DrawAxesEx( transform, size, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

void DrawGrid( b3Vec3 center, b3Vec3 normal, float halfExtent, int divisions, Vec4 color )
{
	if ( divisions < 1 || halfExtent <= 0.0f )
	{
		return;
	}
	// Orthonormal in-plane axes from the normal.
	b3Vec3 n = b3Normalize( normal );
	b3Vec3 u = b3Normalize( b3Perp( n ) );
	b3Vec3 v = b3Cross( n, u );

	const float step = ( 2.0f * halfExtent ) / (float)divisions;
	for ( int i = 0; i <= divisions; ++i )
	{
		const float o = -halfExtent + (float)i * step;
		// Line spanning u at this offset along v.
		b3Vec3 ua = b3Add( center, b3Add( b3MulSV( -halfExtent, u ), b3MulSV( o, v ) ) );
		b3Vec3 ub = b3Add( center, b3Add( b3MulSV( halfExtent, u ), b3MulSV( o, v ) ) );
		DrawLineEx( ua, ub, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
		// Line spanning v at this offset along u.
		b3Vec3 va = b3Add( center, b3Add( b3MulSV( o, u ), b3MulSV( -halfExtent, v ) ) );
		b3Vec3 vb = b3Add( center, b3Add( b3MulSV( o, u ), b3MulSV( halfExtent, v ) ) );
		DrawLineEx( va, vb, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	}
}

void DrawTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, Vec4 color )
{
	DrawLineEx( a, b, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	DrawLineEx( b, c, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	DrawLineEx( c, a, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}
