// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/draw.h"

#include "gfx/debug_shapes.h"
#include "gfx/overlay.h"
#include "gfx/renderer.h"
#include "gfx/text.h"

#include <stdarg.h>
#include <stdio.h>

void DrawCubeEx( b3Transform transform, b3Vec3 scale, Vec4 baseColor, float metallic, float roughness,
				 TransparentShadowCast shadowCast )
{
	AppendCube( transform, scale, baseColor, metallic, roughness, shadowCast );
}

void DrawCube( b3Transform transform, b3Vec3 scale, Vec4 baseColor )
{
	DrawCubeEx( transform, scale, baseColor, DEFAULT_METALLIC, DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_FULL );
}

void DrawSphereEx( b3Transform transform, float radius, Vec4 baseColor, float metallic, float roughness,
				   TransparentShadowCast shadowCast )
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

void DrawSolidSphere( b3Transform transform, b3Sphere sphere, Vec4 color )
{
	b3Transform world = { b3TransformPoint( transform, sphere.center ), transform.q };
	DrawSphere( world, sphere.radius, color );
}

void DrawSolidCapsule( b3Transform transform, b3Capsule capsule, Vec4 color )
{
	b3Vec3 c1 = b3TransformPoint( transform, capsule.center1 );
	b3Vec3 c2 = b3TransformPoint( transform, capsule.center2 );
	b3Vec3 axis = b3Sub( c2, c1 );
	float length = b3Length( axis );

	// Impostor cylinder runs along local +X, so align +X with the capsule axis.
	b3Quat q = transform.q;
	if ( length > 1e-6f )
	{
		q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisX, b3MulSV( 1.0f / length, axis ) );
	}

	b3Transform world = { b3MulSV( 0.5f, b3Add( c1, c2 ) ), q };
	DrawCapsule( world, 0.5f * length, capsule.radius, color );
}

void DrawHull( b3Transform transform, const b3Hull* hull, Vec4 color )
{
	const b3Vec3* points = b3GetHullPoints( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );

	// Half-edges come in twin pairs, so draw each undirected edge once.
	for ( int i = 0; i < hull->edgeCount; ++i )
	{
		if ( i >= edges[i].twin )
		{
			continue;
		}
		b3Vec3 p1 = b3TransformPoint( transform, points[edges[i].origin] );
		b3Vec3 p2 = b3TransformPoint( transform, points[edges[edges[i].twin].origin] );
		DrawLine( p1, p2, color );
	}
}

void DrawPlane( b3Vec3 normal, b3Vec3 point, Vec4 color )
{
	b3Vec3 perp1 = b3Perp( normal );
	b3Vec3 perp2 = b3Cross( perp1, normal );
	b3Vec3 p1 = b3Add( point, b3Add( perp1, perp2 ) );
	b3Vec3 p2 = b3Add( point, b3Sub( perp2, perp1 ) );
	b3Vec3 p3 = b3Sub( point, b3Add( perp1, perp2 ) );
	b3Vec3 p4 = b3Add( point, b3Sub( perp1, perp2 ) );
	DrawLine( p1, p2, color );
	DrawLine( p2, p3, color );
	DrawLine( p3, p4, color );
	DrawLine( p4, p1, color );
	DrawLine( point, b3Add( point, b3MulSV( 0.5f, normal ) ), color );
	DrawPoint( point, 10.0f, color );
}

void DrawLineEx( b3Vec3 a, b3Vec3 b, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode )
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

void DrawPoint( b3Vec3 p, float size, Vec4 color )
{
	DrawPointEx( p, color, size, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );
}

void DrawArrowEx( b3Vec3 a, b3Vec3 b, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				  OverlayOcclusionMode occlusionMode, float headLengthFrac )
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

void DrawBounds( b3AABB bounds, float extension, Vec4 color )
{
	b3Vec3 e = { extension, extension, extension };
	DrawAabb( b3Sub( bounds.lowerBound, e ), b3Add( bounds.upperBound, e ), color );
}

void DrawAxesEx( b3Transform transform, float size, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode )
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

void DrawGroundGrid( int size )
{
	Vec4 color = MakeVec4( 0.3f, 0.3f, 0.3f, 1.0f );
	DrawGrid( b3Vec3_zero, b3Vec3_axisY, (float)size, size, color );
}

void DrawTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, Vec4 color )
{
	DrawLineEx( a, b, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	DrawLineEx( b, c, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	DrawLineEx( c, a, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

static void DrawDisc( b3Vec3 center, b3Vec3 normal, float radius, int segments, Vec4 color )
{
	b3Vec3 tangent1 = b3Perp( normal );
	b3Vec3 tangent2 = b3Cross( normal, tangent1 );

	float delta = 2.0f * B3_PI / segments;
	float cosine = cosf( delta );
	float sine = sinf( delta );

	float x1 = radius, y1 = 0.0f;
	b3Vec3 vertex1 = b3Add( center, b3Blend2( x1, tangent1, y1, tangent2 ) );

	for ( int i = 0; i < segments; ++i )
	{
		float x2 = cosine * x1 - sine * y1;
		float y2 = sine * x1 + cosine * y1;
		b3Vec3 vertex2 = b3Add( center, b3Blend2( x2, tangent1, y2, tangent2 ) );

		DrawLineEx( vertex1, vertex2, color, 2.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );

		x1 = x2;
		y1 = y2;
		vertex1 = vertex2;
	}
}

static void DrawArc( b3Vec3 center, b3Vec3 normal, float radius, b3Vec3 start, float maxDegrees, int segments, Vec4 color )
{
	b3Vec3 tangent1 = b3Normalize( start );
	b3Vec3 tangent2 = b3Cross( normal, tangent1 );

	float deltaDegrees = maxDegrees / segments;
	b3CosSin cs = b3ComputeCosSin( B3_DEG_TO_RAD * deltaDegrees );
	float x1 = radius, y1 = 0.0f;

	b3Vec3 vertex1 = b3Add( center, b3Blend2( x1, tangent1, y1, tangent2 ) );

	for ( float angle = 0.0f; angle < maxDegrees - 0.001f; angle += deltaDegrees )
	{
		float x2 = cs.cosine * x1 - cs.sine * y1;
		float y2 = cs.sine * x1 + cs.cosine * y1;
		b3Vec3 vertex2 = b3Add( center, b3Blend2( x2, tangent1, y2, tangent2 ) );

		DrawLineEx( vertex1, vertex2, color, 2.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );

		x1 = x2;
		y1 = y2;
		vertex1 = vertex2;
	}
}

void DrawWireSphere( b3Transform transform, const b3Sphere* sphere, int segments, Vec4 color )
{
	b3Vec3 center = b3TransformPoint( transform, sphere->center );
	float radius = sphere->radius;

	b3Vec3 axisX = b3RotateVector( transform.q, (b3Vec3){ 1.0f, 0.0f, 0.0f } );
	b3Vec3 axisY = b3RotateVector( transform.q, (b3Vec3){ 0.0f, 1.0f, 0.0f } );
	b3Vec3 axisZ = b3RotateVector( transform.q, (b3Vec3){ 0.0f, 0.0f, 1.0f } );

	DrawDisc( center, axisX, radius, segments, color );
	DrawDisc( center, axisY, radius, segments, color );
	DrawDisc( center, axisZ, radius, segments, color );
}

void DrawWireCapsule( b3Transform transform, const b3Capsule* capsule, int segments, Vec4 color )
{
	b3Vec3 center1 = b3TransformPoint( transform, capsule->center1 );
	b3Vec3 center2 = b3TransformPoint( transform, capsule->center2 );
	float radius = capsule->radius;

	b3Vec3 normal = b3Normalize( b3Sub( center2, center1 ) );
	b3Vec3 tangent1 = b3Perp( normal );
	b3Vec3 tangent2 = b3Cross( normal, tangent1 );

	DrawLineEx( b3MulAdd( center1, radius, tangent1 ), b3MulAdd( center2, radius, tangent1 ), color, 2.0f,
				OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );
	DrawLineEx( b3MulAdd( center1, radius, tangent2 ), b3MulAdd( center2, radius, tangent2 ), color, 2.0f,
				OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );
	DrawLineEx( b3MulSub( center1, radius, tangent1 ), b3MulSub( center2, radius, tangent1 ), color, 2.0f,
				OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );
	DrawLineEx( b3MulSub( center1, radius, tangent2 ), b3MulSub( center2, radius, tangent2 ), color, 2.0f,
				OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );

	DrawArc( center1, b3Neg( tangent1 ), radius, tangent2, 180.0f, segments / 2, color );
	DrawArc( center1, tangent2, radius, tangent1, 180.0f, segments / 2, color );
	DrawArc( center2, tangent1, radius, tangent2, 180.0f, segments / 2, color );
	DrawArc( center2, b3Neg( tangent2 ), radius, tangent1, 180.0f, segments / 2, color );

	DrawDisc( center1, normal, radius, segments, color );
	DrawDisc( center2, normal, radius, segments, color );
}

void DrawWorldString( b3Vec3 point, Vec4 color, const char* format, ... )
{
	va_list args;
	va_start( args, format );
	char buffer[256];
	vsnprintf( buffer, sizeof( buffer ), format, args );
	va_end( args );
	DrawString( point, color, buffer );
}
