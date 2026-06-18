// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/draw.h"

#include "gfx/overlay.h"
#include "gfx/renderer.h"
#include "gfx/text.h"

#include <stdarg.h>
#include <stdio.h>

// World position every draw demotes against. The host points it at the camera eye each
// frame, so the shift happens in double far from the origin and the overlay and impostor
// primitives only ever see small coordinates. Zero by default, which makes the demotion an
// identity and matches absolute coordinates.
static b3Pos s_drawOrigin;

// Shift a world position into the draw-origin-relative frame the primitives render in.
static inline b3Vec3 ToRelative( b3Pos worldPoint )
{
	return b3SubPos( worldPoint, s_drawOrigin );
}

// Shift a world transform into the relative frame. Rotation rides through unchanged.
static inline b3Transform ToRelativeFrame( b3WorldTransform transform )
{
	return b3ToRelativeTransform( transform, s_drawOrigin );
}

void SetDrawOrigin( b3Pos origin )
{
	s_drawOrigin = origin;
}

b3Pos GetDrawOrigin( void )
{
	return s_drawOrigin;
}

void DrawCubeEx( b3WorldTransform transform, b3Vec3 scale, Vec4 baseColor, float metallic, float roughness,
				 TransparentShadowCast shadowCast )
{
	AppendCube( ToRelativeFrame( transform ), scale, baseColor, metallic, roughness, shadowCast );
}

void DrawCube( b3WorldTransform transform, b3Vec3 scale, Vec4 baseColor )
{
	DrawCubeEx( transform, scale, baseColor, DEFAULT_METALLIC, DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_FULL );
}

void DrawSphereEx( b3WorldTransform transform, float radius, Vec4 baseColor, float metallic, float roughness,
				   TransparentShadowCast shadowCast )
{
	AppendSphere( ToRelativeFrame( transform ), radius, baseColor, metallic, roughness, shadowCast );
}

void DrawSphere( b3WorldTransform transform, float radius, Vec4 baseColor )
{
	DrawSphereEx( transform, radius, baseColor, DEFAULT_METALLIC, DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_FULL );
}

void DrawCapsuleEx( b3WorldTransform transform, float halfLength, float radius, Vec4 baseColor, float metallic, float roughness,
					TransparentShadowCast shadowCast )
{
	AppendCapsule( ToRelativeFrame( transform ), halfLength, radius, baseColor, metallic, roughness, shadowCast );
}

void DrawCapsule( b3WorldTransform transform, float halfLength, float radius, Vec4 baseColor )
{
	DrawCapsuleEx( transform, halfLength, radius, baseColor, DEFAULT_METALLIC, DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_FULL );
}

void DrawSolidSphere( b3WorldTransform transform, b3Sphere sphere, Vec4 color )
{
	b3Transform rel = ToRelativeFrame( transform );
	b3Transform world = { b3TransformPoint( rel, sphere.center ), rel.q };
	AppendSphere( world, sphere.radius, color, DEFAULT_METALLIC, DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_FULL );
}

void DrawSolidCapsule( b3WorldTransform transform, b3Capsule capsule, Vec4 color )
{
	b3Transform rel = ToRelativeFrame( transform );
	b3Vec3 c1 = b3TransformPoint( rel, capsule.center1 );
	b3Vec3 c2 = b3TransformPoint( rel, capsule.center2 );
	b3Vec3 axis = b3Sub( c2, c1 );
	float length = b3Length( axis );

	// Impostor cylinder runs along local +X, so align +X with the capsule axis.
	b3Quat q = rel.q;
	if ( length > 1e-6f )
	{
		q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisX, b3MulSV( 1.0f / length, axis ) );
	}

	b3Transform world = { b3MulSV( 0.5f, b3Add( c1, c2 ) ), q };
	AppendCapsule( world, 0.5f * length, capsule.radius, color, DEFAULT_METALLIC, DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_FULL );
}

void DrawHull( b3WorldTransform transform, const b3HullData* hull, Vec4 color )
{
	b3Transform rel = ToRelativeFrame( transform );
	const b3Vec3* points = b3GetHullPoints( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );

	// Half-edges come in twin pairs, so draw each undirected edge once.
	for ( int i = 0; i < hull->edgeCount; ++i )
	{
		if ( i >= edges[i].twin )
		{
			continue;
		}
		b3Vec3 p1 = b3TransformPoint( rel, points[edges[i].origin] );
		b3Vec3 p2 = b3TransformPoint( rel, points[edges[edges[i].twin].origin] );
		OverlayAppendLine( p1, p2, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	}
}

void DrawPlane( b3Vec3 normal, b3Pos point, Vec4 color )
{
	b3Vec3 c = ToRelative( point );
	b3Vec3 perp1 = b3Perp( normal );
	b3Vec3 perp2 = b3Cross( perp1, normal );
	b3Vec3 p1 = b3Add( c, b3Add( perp1, perp2 ) );
	b3Vec3 p2 = b3Add( c, b3Sub( perp2, perp1 ) );
	b3Vec3 p3 = b3Sub( c, b3Add( perp1, perp2 ) );
	b3Vec3 p4 = b3Add( c, b3Sub( perp1, perp2 ) );
	float th = DEFAULT_LINE_THICKNESS_PX;
	OverlayAppendLine( p1, p2, color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	OverlayAppendLine( p2, p3, color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	OverlayAppendLine( p3, p4, color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	OverlayAppendLine( p4, p1, color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	OverlayAppendLine( c, b3Add( c, b3MulSV( 0.5f, normal ) ), color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	OverlayAppendPoint( c, color, 10.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );
}

void DrawLineEx( b3Pos a, b3Pos b, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode )
{
	OverlayAppendLine( ToRelative( a ), ToRelative( b ), color, thickness, thicknessUnit, occlusionMode );
}

void DrawLine( b3Pos a, b3Pos b, Vec4 color )
{
	DrawLineEx( a, b, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

void DrawPointEx( b3Pos p, Vec4 color, float size, OverlayThicknessUnit sizeUnit, OverlayOcclusionMode occlusionMode )
{
	OverlayAppendPoint( ToRelative( p ), color, size, sizeUnit, occlusionMode );
}

void DrawPoint( b3Pos p, float size, Vec4 color )
{
	DrawPointEx( p, color, size, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );
}

void DrawArrowEx( b3Pos a, b3Pos b, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				  OverlayOcclusionMode occlusionMode, float headLengthFrac )
{
	b3Vec3 tail = ToRelative( a );
	b3Vec3 tip = ToRelative( b );
	OverlayAppendLine( tail, tip, color, thickness, thicknessUnit, occlusionMode );

	b3Vec3 shaft = b3Sub( tip, tail );
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
	b3Vec3 tip1 = b3Add( tip, b3Add( backFromTip, sideStep ) );
	b3Vec3 tip2 = b3Add( tip, b3Sub( backFromTip, sideStep ) );
	OverlayAppendLine( tip, tip1, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( tip, tip2, color, thickness, thicknessUnit, occlusionMode );
}

void DrawArrow( b3Pos a, b3Pos b, Vec4 color )
{
	DrawArrowEx( a, b, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE,
				 DEFAULT_ARROW_HEAD_FRAC );
}

void DrawCrossEx( b3Pos center, float size, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				  OverlayOcclusionMode occlusionMode )
{
	b3Vec3 c = ToRelative( center );
	float h = size * 0.5f;
	OverlayAppendLine( (b3Vec3){ c.x - h, c.y, c.z }, (b3Vec3){ c.x + h, c.y, c.z }, color, thickness, thicknessUnit,
					   occlusionMode );
	OverlayAppendLine( (b3Vec3){ c.x, c.y - h, c.z }, (b3Vec3){ c.x, c.y + h, c.z }, color, thickness, thicknessUnit,
					   occlusionMode );
	OverlayAppendLine( (b3Vec3){ c.x, c.y, c.z - h }, (b3Vec3){ c.x, c.y, c.z + h }, color, thickness, thicknessUnit,
					   occlusionMode );
}

void DrawCross( b3Pos center, float size, Vec4 color )
{
	DrawCrossEx( center, size, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

void DrawAabbEx( b3Vec3 mn, b3Vec3 mx, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode )
{
	// The box is float world space, so promote each corner and difference against the origin in
	// double before it demotes. Far from the origin a plain narrow could clip a corner.
	b3Vec3 c000 = ToRelative( b3ToPos( (b3Vec3){ mn.x, mn.y, mn.z } ) );
	b3Vec3 c100 = ToRelative( b3ToPos( (b3Vec3){ mx.x, mn.y, mn.z } ) );
	b3Vec3 c010 = ToRelative( b3ToPos( (b3Vec3){ mn.x, mx.y, mn.z } ) );
	b3Vec3 c110 = ToRelative( b3ToPos( (b3Vec3){ mx.x, mx.y, mn.z } ) );
	b3Vec3 c001 = ToRelative( b3ToPos( (b3Vec3){ mn.x, mn.y, mx.z } ) );
	b3Vec3 c101 = ToRelative( b3ToPos( (b3Vec3){ mx.x, mn.y, mx.z } ) );
	b3Vec3 c011 = ToRelative( b3ToPos( (b3Vec3){ mn.x, mx.y, mx.z } ) );
	b3Vec3 c111 = ToRelative( b3ToPos( (b3Vec3){ mx.x, mx.y, mx.z } ) );

	// 4 bottom edges (y = mn.y).
	OverlayAppendLine( c000, c100, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c100, c101, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c101, c001, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c001, c000, color, thickness, thicknessUnit, occlusionMode );
	// 4 top edges (y = mx.y).
	OverlayAppendLine( c010, c110, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c110, c111, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c111, c011, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c011, c010, color, thickness, thicknessUnit, occlusionMode );
	// 4 vertical edges.
	OverlayAppendLine( c000, c010, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c100, c110, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c101, c111, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c001, c011, color, thickness, thicknessUnit, occlusionMode );
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

void DrawAxesEx( b3WorldTransform transform, float size, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode )
{
	b3Transform rel = ToRelativeFrame( transform );
	b3Vec3 origin = rel.p;
	b3Matrix3 basis = b3MakeMatrixFromQuat( rel.q );
	OverlayAppendLine( origin, b3MulAdd( origin, size, basis.cx ), MakeVec4( 1.0f, 0.0f, 0.0f, 1.0f ), thickness, thicknessUnit,
					   occlusionMode );
	OverlayAppendLine( origin, b3MulAdd( origin, size, basis.cy ), MakeVec4( 0.0f, 1.0f, 0.0f, 1.0f ), thickness, thicknessUnit,
					   occlusionMode );
	OverlayAppendLine( origin, b3MulAdd( origin, size, basis.cz ), MakeVec4( 0.0f, 0.0f, 1.0f, 1.0f ), thickness, thicknessUnit,
					   occlusionMode );
}

void DrawAxes( b3WorldTransform transform, float size )
{
	DrawAxesEx( transform, size, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

void DrawGrid( b3Pos center, b3Vec3 normal, float halfExtent, int divisions, Vec4 color )
{
	if ( divisions < 1 || halfExtent <= 0.0f )
	{
		return;
	}
	b3Vec3 c = ToRelative( center );

	// Orthonormal in-plane axes from the normal.
	b3Vec3 n = b3Normalize( normal );
	b3Vec3 u = b3Normalize( b3Perp( n ) );
	b3Vec3 v = b3Cross( n, u );

	const float step = ( 2.0f * halfExtent ) / (float)divisions;
	for ( int i = 0; i <= divisions; ++i )
	{
		const float o = -halfExtent + (float)i * step;
		// Line spanning u at this offset along v.
		b3Vec3 ua = b3Add( c, b3Add( b3MulSV( -halfExtent, u ), b3MulSV( o, v ) ) );
		b3Vec3 ub = b3Add( c, b3Add( b3MulSV( halfExtent, u ), b3MulSV( o, v ) ) );
		OverlayAppendLine( ua, ub, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
		// Line spanning v at this offset along u.
		b3Vec3 va = b3Add( c, b3Add( b3MulSV( o, u ), b3MulSV( -halfExtent, v ) ) );
		b3Vec3 vb = b3Add( c, b3Add( b3MulSV( o, u ), b3MulSV( halfExtent, v ) ) );
		OverlayAppendLine( va, vb, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	}
}

void DrawGroundGrid( int size )
{
	Vec4 color = MakeVec4( 0.3f, 0.3f, 0.3f, 1.0f );
	DrawGrid( b3Pos_zero, b3Vec3_axisY, (float)size, size, color );
}

void DrawTriangle( b3WorldTransform transform, b3Vec3 a, b3Vec3 b, b3Vec3 c, Vec4 color )
{
	b3Transform rel = ToRelativeFrame( transform );

	b3Vec3 ra = b3TransformPoint( rel, a );
	b3Vec3 rb = b3TransformPoint( rel, b );
	b3Vec3 rc = b3TransformPoint( rel, c );
	float th = DEFAULT_LINE_THICKNESS_PX;
	OverlayAppendLine( ra, rb, color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	OverlayAppendLine( rb, rc, color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	OverlayAppendLine( rc, ra, color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

// Disc of `segments` lines in the plane through `center` (relative frame) with the given normal.
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

		OverlayAppendLine( vertex1, vertex2, color, 2.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );

		x1 = x2;
		y1 = y2;
		vertex1 = vertex2;
	}
}

// Arc swept `maxDegrees` from `start` about `normal`, in the relative frame.
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

		OverlayAppendLine( vertex1, vertex2, color, 2.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );

		x1 = x2;
		y1 = y2;
		vertex1 = vertex2;
	}
}

void DrawWireSphere( b3WorldTransform transform, const b3Sphere* sphere, int segments, Vec4 color )
{
	b3Transform rel = ToRelativeFrame( transform );
	b3Vec3 center = b3TransformPoint( rel, sphere->center );
	float radius = sphere->radius;

	b3Vec3 axisX = b3RotateVector( rel.q, (b3Vec3){ 1.0f, 0.0f, 0.0f } );
	b3Vec3 axisY = b3RotateVector( rel.q, (b3Vec3){ 0.0f, 1.0f, 0.0f } );
	b3Vec3 axisZ = b3RotateVector( rel.q, (b3Vec3){ 0.0f, 0.0f, 1.0f } );

	DrawDisc( center, axisX, radius, segments, color );
	DrawDisc( center, axisY, radius, segments, color );
	DrawDisc( center, axisZ, radius, segments, color );
}

void DrawWireCapsule( b3WorldTransform transform, const b3Capsule* capsule, int segments, Vec4 color )
{
	b3Transform rel = ToRelativeFrame( transform );
	b3Vec3 center1 = b3TransformPoint( rel, capsule->center1 );
	b3Vec3 center2 = b3TransformPoint( rel, capsule->center2 );
	float radius = capsule->radius;

	b3Vec3 normal = b3Normalize( b3Sub( center2, center1 ) );
	b3Vec3 tangent1 = b3Perp( normal );
	b3Vec3 tangent2 = b3Cross( normal, tangent1 );

	OverlayAppendLine( b3MulAdd( center1, radius, tangent1 ), b3MulAdd( center2, radius, tangent1 ), color, 2.0f,
					   OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );
	OverlayAppendLine( b3MulAdd( center1, radius, tangent2 ), b3MulAdd( center2, radius, tangent2 ), color, 2.0f,
					   OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );
	OverlayAppendLine( b3MulSub( center1, radius, tangent1 ), b3MulSub( center2, radius, tangent1 ), color, 2.0f,
					   OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );
	OverlayAppendLine( b3MulSub( center1, radius, tangent2 ), b3MulSub( center2, radius, tangent2 ), color, 2.0f,
					   OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );

	DrawArc( center1, b3Neg( tangent1 ), radius, tangent2, 180.0f, segments / 2, color );
	DrawArc( center1, tangent2, radius, tangent1, 180.0f, segments / 2, color );
	DrawArc( center2, tangent1, radius, tangent2, 180.0f, segments / 2, color );
	DrawArc( center2, b3Neg( tangent2 ), radius, tangent1, 180.0f, segments / 2, color );

	DrawDisc( center1, normal, radius, segments, color );
	DrawDisc( center2, normal, radius, segments, color );
}

void DrawString3D( b3Pos point, Vec4 color, const char* format, ... )
{
	va_list args;
	va_start( args, format );
	char buffer[256];
	vsnprintf( buffer, sizeof( buffer ), format, args );
	va_end( args );
	DrawString( ToRelative( point ), color, buffer );
}
