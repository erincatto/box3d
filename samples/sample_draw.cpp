// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "sample_draw.h"

#include "draw_bridge.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

void DrawSphere( Scene*, const b3Transform& transform, const b3Sphere& sphere, b3HexColor color )
{
	b3Transform world = { b3TransformPoint( transform, sphere.center ), transform.q };
	BridgeSphere( world, sphere.radius, MakeColor( color ) );
}

void DrawCapsule( Scene*, const b3Transform& transform, const b3Capsule& capsule, b3HexColor color )
{
	b3Vec3 c1 = b3TransformPoint( transform, capsule.center1 );
	b3Vec3 c2 = b3TransformPoint( transform, capsule.center2 );
	b3Vec3 axis = b3Sub( c2, c1 );
	float length = b3Length( axis );

	// gfx capsules orient their long axis along local +X.
	b3Quat q = transform.q;
	if ( length > 1e-6f )
	{
		q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisX, b3MulSV( 1.0f / length, axis ) );
	}

	b3Transform world = { b3MulSV( 0.5f, b3Add( c1, c2 ) ), q };
	BridgeCapsule( world, 0.5f * length, capsule.radius, MakeColor( color ) );
}

void DrawHull( Scene*, const b3Transform& transform, const b3Hull* hull, b3HexColor color, bool drawNormals )
{
	(void)drawNormals;
	Vec4 c = MakeColor( color );
	const b3Vec3* points = b3GetHullPoints( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );

	// Half-edges come in twin pairs; draw each undirected edge once.
	for ( int i = 0; i < hull->edgeCount; ++i )
	{
		if ( i >= edges[i].twin )
		{
			continue;
		}
		b3Vec3 p1 = b3TransformPoint( transform, points[edges[i].origin] );
		b3Vec3 p2 = b3TransformPoint( transform, points[edges[edges[i].twin].origin] );
		BridgeLine( p1, p2, c );
	}
}

void DrawBox( Scene*, b3Vec3 extents, const b3Transform& transform, b3HexColor color )
{
	BridgeCube( transform, b3MulSV( 2.0f, extents ), MakeColor( color ) );
}

void DrawPoint( Scene*, b3Vec3 point, float size, b3HexColor color )
{
	BridgePoint( point, size, MakeColor( color ) );
}

void DrawLine( Scene*, b3Vec3 point1, b3Vec3 point2, b3HexColor color )
{
	BridgeLine( point1, point2, MakeColor( color ) );
}

void DrawArrow( Scene*, b3Vec3 point1, b3Vec3 point2, float size, b3HexColor color )
{
	(void)size;
	BridgeArrow( point1, point2, MakeColor( color ) );
}

void DrawCross( Scene*, b3Vec3 center, float radius, b3HexColor color )
{
	BridgeCross( center, 2.0f * radius, MakeColor( color ) );
}

void DrawBounds( Scene*, b3AABB bounds, float extension, b3HexColor color )
{
	b3Vec3 e = { extension, extension, extension };
	BridgeAabb( b3Sub( bounds.lowerBound, e ), b3Add( bounds.upperBound, e ), MakeColor( color ) );
}

void DrawTransform( Scene*, const b3Transform& transform, float length )
{
	BridgeAxes( transform, length );
}

void DrawFace( Scene*, b3Vec3 vertex1, b3Vec3 vertex2, b3Vec3 vertex3, b3HexColor color )
{
	BridgeTriangle( vertex1, vertex2, vertex3, MakeColor( color ) );
}

void DrawPlane( Scene*, b3Vec3 normal, b3Vec3 point, b3HexColor color )
{
	Vec4 c = MakeColor( color );
	b3Vec3 perp1 = b3Perp( normal );
	b3Vec3 perp2 = b3Cross( perp1, normal );
	b3Vec3 p1 = b3Add( point, b3Add( perp1, perp2 ) );
	b3Vec3 p2 = b3Add( point, b3Sub( perp2, perp1 ) );
	b3Vec3 p3 = b3Sub( point, b3Add( perp1, perp2 ) );
	b3Vec3 p4 = b3Add( point, b3Sub( perp1, perp2 ) );
	BridgeLine( p1, p2, c );
	BridgeLine( p2, p3, c );
	BridgeLine( p3, p4, c );
	BridgeLine( p4, p1, c );
	BridgeLine( point, b3Add( point, b3MulSV( 0.5f, normal ) ), c );
	BridgePoint( point, 10.0f, c );
}

void DrawGrid( Scene*, int size )
{
	Vec4 color = MakeVec4( 0.3f, 0.3f, 0.3f, 1.0f );
	BridgeGrid( b3Vec3_zero, b3Vec3_axisY, (float)size, size, color );
}

void DrawWorldString( Camera*, b3Vec3 point, b3HexColor color, const char* format, ... )
{
	va_list args;
	va_start( args, format );
	char buffer[256];
	vsnprintf( buffer, sizeof( buffer ), format, args );
	va_end( args );
	BridgeString( point, MakeColor( color ), buffer );
}
