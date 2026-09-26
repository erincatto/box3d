// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/draw.h"
#include "gfx/keycodes.h"
#include "sample.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"

#include <imgui.h>

class Manifold : public Sample
{
public:
	explicit Manifold( SampleContext* sampleContext )
		: Sample( sampleContext )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 35.0f, 30.0f, 50.0f, { 0.0f, 5.0f, 0.0f } );
		}

		memset( m_points, 0, sizeof( m_points ) );
		m_manifold = {};
		m_manifold.points = m_points;

		{
			m_transformA.p = { 3.5f, 0.5f, 0.0f };
			m_transformA.q = b3MakeQuatFromAxisAngle( { 0.0f, 1.0f, 0.0f }, 0.5f * B3_PI );
		}

		{
			m_transformB.p = { 0.0f, 1.5f, 3.5f };
			m_transformB.q = b3Quat_identity;
		}

		m_simplexCache = {};
		m_satCache = {};
		m_manualFeature = 0;
		m_baseTranslation = b3Pos_zero;
		m_baseQuaternion = b3Quat_identity;
		m_baseX = 0;
		m_baseY = 0;
		m_origin = b3Pos_zero;
		m_useCache = false;
		m_tracking = false;
		m_rotating = false;
	}

	void Render() override
	{
		DrawTextLine( "origin: %g %g %g", m_origin.x, m_origin.y, m_origin.z );
		DrawTextLine( "count = %d", m_manifold.pointCount );

		DrawAxes( b3WorldTransform_identity, 1.0f );

		if ( m_manifold.pointCount == 0 )
		{
			return;
		}

		float length = 0.5f * b3GetLengthUnitsPerMeter();
		b3Vec3 normal = b3RotateVector( m_transformA.q, m_manifold.normal );

		for ( int pointIndex = 0; pointIndex < m_manifold.pointCount; ++pointIndex )
		{
			const b3LocalManifoldPoint& manifoldPoint = m_manifold.points[pointIndex];

			b3Pos point = b3TransformWorldPoint( m_transformA, manifoldPoint.point );

			DrawLine( point, point + length * normal, MakeColor( b3_colorWhite ) );

			if ( manifoldPoint.separation > 0.0f )
			{
				DrawPoint( point, 10.0f, MakeColor( b3_colorWhite ) );
			}
			else
			{
				DrawPoint( point, 10.0f, MakeColor( b3_colorYellow ) );
			}

			DrawString3D( point, MakeColor( b3_colorWhite ), "   %.3f", manifoldPoint.separation );

			b3Vec3 perp = b3Perp( normal );
			b3FeaturePair pair = manifoldPoint.pair;
			DrawString3D( point + 0.025f * normal + 0.05f * perp, MakeColor( b3_colorPapayaWhip ), "  %X:%X %X:%X", pair.owner1,
						  pair.index1, pair.owner2, pair.index2 );
		}

		Sample::Render();
	}

	bool HasSolverControls() const override
	{
		return false;
	}

	bool DrawControls() override
	{
		ImGui::Checkbox( "Use cache", &m_useCache );
		if ( m_useCache )
		{
			ImGui::RadioButton( "auto", &m_manualFeature, 0 );
			ImGui::RadioButton( "faceA", &m_manualFeature, 1 );
			ImGui::RadioButton( "faceB", &m_manualFeature, 2 );
			ImGui::RadioButton( "edgePair", &m_manualFeature, 3 );
		}

		return true;
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 && ( modifiers & MOD_ALT ) == 0 )
		{
			if ( modifiers & MOD_SHIFT )
			{
				m_baseX = p.x;
				m_baseY = p.y;
				m_baseQuaternion = m_transformB.q;
				m_rotating = true;
			}
			else
			{
				PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
				m_origin = pickRay.origin + 10.0f * b3Normalize( pickRay.translation );
				m_baseTranslation = m_transformB.p;
				m_tracking = true;
			}
		}
	}

	void MouseUp( b3Vec2 p, int button ) override
	{
		m_tracking = false;
		m_rotating = false;
	}

	void MouseMove( b3Vec2 p ) override
	{
		if ( m_tracking )
		{
			PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
			b3Pos origin = pickRay.origin + 10.0f * b3Normalize( pickRay.translation );
			m_transformB.p = m_baseTranslation + b3SubPos( origin, m_origin );
		}

		if ( m_rotating )
		{
			int x = p.x;
			int y = p.y;

			b3Quat qx = b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.01f * ( x - m_baseX ) );
			b3Quat qz = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.01f * ( y - m_baseY ) );
			m_transformB.q = b3NormalizeQuat( b3MulQuat( m_baseQuaternion, b3MulQuat( qx, qz ) ) );
		}
	}

	static constexpr int m_pointCapacity = 64;
	b3LocalManifold m_manifold;
	b3LocalManifoldPoint m_points[m_pointCapacity];
	b3WorldTransform m_transformA;
	b3WorldTransform m_transformB;
	b3Pos m_baseTranslation;
	b3Quat m_baseQuaternion;
	b3Pos m_origin;
	b3SimplexCache m_simplexCache;
	b3SATCache m_satCache;
	int m_manualFeature;
	int m_baseX;
	int m_baseY;
	bool m_useCache;
	bool m_tracking;
	bool m_rotating;
};

class TriangleManifold : public Sample
{
public:
	explicit TriangleManifold( SampleContext* sampleContext )
		: Sample( sampleContext )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 35.0f, 30.0f, 50.0f, { 0.0f, 5.0f, 0.0f } );
		}

		m_manifold = {};
		memset( m_points, 0, sizeof( m_points ) );
		m_manifold.points = m_points;

		{
			m_transformA.p = { 3.5f, 0.5f, 0.0f };
			m_transformA.q = b3MakeQuatFromAxisAngle( { 0.0f, 1.0f, 0.0f }, 0.5f * B3_PI );
		}

		{
			m_transformB.p = { 0.0f, 1.5f, 3.5f };
			m_transformB.q = b3Quat_identity;
		}

		m_simplexCache = {};
		m_satCache = {};
		m_baseTranslation = b3Pos_zero;
		m_baseQuaternion = b3Quat_identity;
		m_baseX = 0;
		m_baseY = 0;
		m_origin = b3Pos_zero;
		m_useCache = false;
		m_tracking = false;
		m_rotating = false;

		m_manualFeature = 0;
	}

	void Render() override
	{
		DrawTextLine( "origin: %g %g %g", m_origin.x, m_origin.y, m_origin.z );
		DrawTextLine( "count = %d", m_manifold.pointCount );
		DrawTextLine( "feature = %d", m_manifold.feature );
		DrawTextLine( "cache hit = %d", m_satCache.hit );

		DrawAxes( b3WorldTransform_identity, 1.0f );

		if ( m_manifold.pointCount > 0 )
		{
			float length = 0.5f * b3GetLengthUnitsPerMeter();
			b3Vec3 normal = b3RotateVector( m_transformB.q, m_manifold.normal );

			for ( int pointIndex = 0; pointIndex < m_manifold.pointCount; ++pointIndex )
			{
				const b3LocalManifoldPoint& manifoldPoint = m_manifold.points[pointIndex];

				b3Pos point = b3TransformWorldPoint( m_transformB, manifoldPoint.point );
				DrawLine( point, point + length * normal, MakeColor( b3_colorWhite ) );

				if ( manifoldPoint.separation > 0.0f )
				{
					DrawPoint( point, 10.0f, MakeColor( b3_colorWhite ) );
				}
				else
				{
					DrawPoint( point, 10.0f, MakeColor( b3_colorYellow ) );
				}

				DrawString3D( point, MakeColor( b3_colorWhite ), "  %.2f", 100.0f * manifoldPoint.separation );

				b3FeaturePair pair = manifoldPoint.pair;
				DrawString3D( point + 0.025f * normal, MakeColor( b3_colorPapayaWhip ), "  %X:%X %X:%X", pair.owner1, pair.index1,
							  pair.owner2, pair.index2 );
			}
		}

		DrawTriangle( m_transformA, m_triangle[0], m_triangle[1], m_triangle[2], MakeColor( b3_colorCyan ) );

		b3Pos p1 = b3TransformWorldPoint( m_transformA, m_triangle[0] );
		b3Pos p2 = b3TransformWorldPoint( m_transformA, m_triangle[1] );
		b3Pos p3 = b3TransformWorldPoint( m_transformA, m_triangle[2] );
		DrawString3D( p1, MakeColor( b3_colorWhite ), "0" );
		DrawString3D( p2, MakeColor( b3_colorWhite ), "1" );
		DrawString3D( p3, MakeColor( b3_colorWhite ), "2" );

		b3Vec3 normal = b3Normalize( b3Cross( p2 - p1, p3 - p1 ) );
		b3Pos center = p1 + ( 1.0f / 3.0f ) * ( ( p2 - p1 ) + ( p3 - p1 ) );
		DrawArrow( center, center + 0.5f * normal, MakeColor( b3_colorMediumPurple ) );

		Sample::Render();
	}

	bool HasSolverControls() const override
	{
		return false;
	}

	bool DrawControls() override
	{
		ImGui::Checkbox( "Use cache", &m_useCache );
		if ( m_useCache )
		{
			ImGui::RadioButton( "auto", &m_manualFeature, 0 );
			ImGui::RadioButton( "faceA", &m_manualFeature, 1 );
			ImGui::RadioButton( "faceB", &m_manualFeature, 2 );
			ImGui::RadioButton( "edgePair", &m_manualFeature, 3 );
		}

		return true;
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 && ( modifiers & MOD_ALT ) == 0 )
		{
			if ( modifiers & MOD_SHIFT )
			{
				m_baseX = p.x;
				m_baseY = p.y;
				m_baseQuaternion = m_transformB.q;
				m_rotating = true;
			}
			else
			{
				PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
				m_origin = pickRay.origin + 10.0f * b3Normalize( pickRay.translation );
				m_baseTranslation = m_transformB.p;
				m_tracking = true;
			}
		}
	}

	void MouseUp( b3Vec2 p, int button ) override
	{
		m_tracking = false;
		m_rotating = false;
	}

	void MouseMove( b3Vec2 p ) override
	{
		if ( m_tracking )
		{
			PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
			b3Pos origin = pickRay.origin + 10.0f * b3Normalize( pickRay.translation );
			m_transformB.p = m_baseTranslation + b3SubPos( origin, m_origin );
		}

		if ( m_rotating )
		{
			int x = p.x;
			int y = p.y;

			b3Quat qx = b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.01f * ( x - m_baseX ) );
			b3Quat qz = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.01f * ( y - m_baseY ) );
			m_transformB.q = b3NormalizeQuat( b3MulQuat( m_baseQuaternion, b3MulQuat( qx, qz ) ) );
		}
	}

	static constexpr int m_pointCapacity = 8;
	b3LocalManifold m_manifold;
	b3LocalManifoldPoint m_points[m_pointCapacity];

	// Triangle transform
	b3WorldTransform m_transformA;

	// Convex shape transform
	b3WorldTransform m_transformB;

	b3Vec3 m_triangle[3] = {};
	b3Pos m_baseTranslation;
	b3Quat m_baseQuaternion;
	b3Pos m_origin;
	b3SimplexCache m_simplexCache;
	b3SATCache m_satCache;
	int m_manualFeature;
	int m_baseX;
	int m_baseY;
	bool m_useCache;
	bool m_tracking;
	bool m_rotating;
};

class SphereAndSphere : public Manifold
{
public:
	explicit SphereAndSphere( SampleContext* sampleContext )
		: Manifold( sampleContext )
	{
		m_sphere = { { 0.5f, 0.0f, -0.25f }, 2.0f };
	}

	void Render() override
	{
		DrawSolidSphere( m_transformA, m_sphere, MakeColor( b3_colorGreen ) );
		DrawSolidSphere( m_transformB, m_sphere, MakeColor( b3_colorCyan ) );

		Manifold::Render();
	}

	void Step() override
	{
		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		b3CollideSpheres( &m_manifold, m_pointCapacity, &m_sphere, &m_sphere, transformBtoA );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new SphereAndSphere( sampleContext );
	}

	b3Sphere m_sphere;
};

static int sampleCollideSpheres = RegisterSample( "Manifold", "Sphere vs Sphere", SphereAndSphere::Create );

class CapsuleAndSphere : public Manifold
{
public:
	static Sample* Create( SampleContext* sampleContext )
	{
		return new CapsuleAndSphere( sampleContext );
	}

	explicit CapsuleAndSphere( SampleContext* sampleContext )
		: Manifold( sampleContext )
	{
		m_capsule = { { -2.0f, 0.0f, 0.0f }, { 2.0f, 0.0f, 0.0f }, 1.0f };
		m_sphere = { { 0.0f, 0.0f, 0.0f }, 2.0f };

		m_transformA = { { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
		m_transformB = { { -4.0f, 0.0f, 0.0f }, b3Quat_identity };
	}

	void Render() override
	{
		DrawSolidCapsule( m_transformA, m_capsule, MakeColor( b3_colorCyan ) );
		DrawSolidSphere( m_transformB, m_sphere, MakeColor( b3_colorGreen ) );

		Manifold::Render();
	}

	void Step() override
	{
		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		b3CollideCapsuleAndSphere( &m_manifold, m_pointCapacity, &m_capsule, &m_sphere, transformBtoA );
	}

	b3Sphere m_sphere;
	b3Capsule m_capsule;
};

static int sampleSphereAndCapsule = RegisterSample( "Manifold", "Capsule vs Sphere", CapsuleAndSphere::Create );

class HullAndSphere : public Manifold
{
public:
	explicit HullAndSphere( SampleContext* sampleContext )
		: Manifold( sampleContext )
	{
		m_sphere = { { 0.0f, 0.0f, 0.0f }, 1.0f };
		m_hull = b3MakeBoxHull( 2.0f, 0.5f, 0.5f );

		m_transformA = { { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
		m_transformB = { { 1.5f, 0.0f, 0.0f }, b3Quat_identity };
	}

	void Render() override
	{
		DrawHull( m_transformA, &m_hull.base, MakeColor( b3_colorCyan ) );
		DrawSolidSphere( m_transformB, m_sphere, MakeColor( b3_colorGreen ) );

		Manifold::Render();
	}

	void Step() override
	{
		if ( m_useCache == false )
		{
			m_simplexCache = {};
		}

		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		b3CollideHullAndSphere( &m_manifold, m_pointCapacity, &m_hull.base, &m_sphere, transformBtoA, &m_simplexCache );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new HullAndSphere( sampleContext );
	}

	b3Sphere m_sphere;
	b3BoxHull m_hull;
};

static int sampleSphereAndHull = RegisterSample( "Manifold", "Hull vs Sphere", HullAndSphere::Create );

class TriangleAndSphere : public TriangleManifold
{
public:
	static Sample* Create( SampleContext* sampleContext )
	{
		return new TriangleAndSphere( sampleContext );
	}

	explicit TriangleAndSphere( SampleContext* sampleContext )
		: TriangleManifold( sampleContext )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 10.0f, b3Pos_zero );
		}

		m_sphere = { { 0.0f, 0.0f, 0.0f }, 0.25f };
		m_triangle[0] = { 0.0f, 0.0f, 0.0f };
		m_triangle[1] = { 4.0f, 0.0f, 4.0f };
		m_triangle[2] = { 4.0f, 0.0f, 0.0f };

		// b3Quat qA = b3MakeQuatFromAxisAngle( { 0.0f, 1.0f, 0.0f }, 2.0f );
		// m_transformA = { { 1.0f, 1.0f, 0.0f }, qA };

		m_transformA = b3WorldTransform_identity;
		m_transformB = { { 2.0f, 0.5f, 1.0f }, b3Quat_identity };
	}

	void Render() override
	{
		DrawSolidSphere( m_transformB, m_sphere, MakeColorAlpha( b3_colorGreen, 0.5f ) );

		TriangleManifold::Render();
	}

	void Step() override
	{
		// Convert triangle to frame B
		b3Transform xf = b3InvMulWorldTransforms( m_transformB, m_transformA );
		b3Vec3 localTriangle[3] = {
			b3TransformPoint( xf, m_triangle[0] ),
			b3TransformPoint( xf, m_triangle[1] ),
			b3TransformPoint( xf, m_triangle[2] ),
		};

		b3CollideTriangleAndSphere( &m_manifold, m_pointCapacity, localTriangle, &m_sphere );
	}

	b3Sphere m_sphere;
};

static int sampleSphereAndTriangle = RegisterSample( "Manifold", "Triangle vs Sphere", TriangleAndSphere::Create );

class CapsuleAndCapsule : public Manifold
{
public:
	explicit CapsuleAndCapsule( SampleContext* sampleContext )
		: Manifold( sampleContext )
	{
		m_capsule = { { -2.0f, 0.0f, 0.0f }, { 2.0f, 0.0f, 0.0f }, 1.0f };

		m_transformA = { { 1.0f, 1.0f, 0.0f }, b3Quat_identity };
		m_transformB = { { -4.0f, 1.0f, 0.0f }, b3Quat_identity };
	}

	void Render() override
	{
		DrawSolidCapsule( m_transformA, m_capsule, MakeColor( b3_colorGreen ) );
		DrawSolidCapsule( m_transformB, m_capsule, MakeColor( b3_colorCyan ) );

		Manifold::Render();
	}

	void Step() override
	{
		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		b3CollideCapsules( &m_manifold, m_pointCapacity, &m_capsule, &m_capsule, transformBtoA );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new CapsuleAndCapsule( sampleContext );
	}

	b3Capsule m_capsule;
};

static int sampleCapsuleAndCapsule = RegisterSample( "Manifold", "Capsule vs Capsule", CapsuleAndCapsule::Create );

class CapsuleAndHull : public Manifold
{
public:
	explicit CapsuleAndHull( SampleContext* sampleContext )
		: Manifold( sampleContext )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 5.0f, b3Pos_zero );
		}

		m_capsule = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 0.15f };
		m_hull = b3MakeBoxHull( 1.0f, 0.5f, 0.5f );

		m_transformA = { { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
		m_transformB = { { 0.0f, 1.0f, 0.0f }, b3Quat_identity };

		/*
			p	[ 1.58523774, 0.729615569, 0.451690674 ]	b3Vec3
			q	[ [ -0.00256555085, -0.0201825816, 0.126076236 ], 0.991811991 ]	b3Quat
		 */
		// m_capsule = {
		//	{ 0.0799999982, -0.0151330000, -0.0918010026 }, { -0.0799999982, -0.0151330000, -0.0918010026 }, 0.100000001 };
		// m_hull = b3CreateBox( { 0.5f, 1.0f, 1.5f } );

		m_transformB = { { 1.58523774, 0.729615569, 0.451690674 },
						 { { -0.00256555085, -0.0201825816, 0.126076236 }, 0.991811991 } };
	}

	void Render() override
	{
		DrawHull( m_transformA, &m_hull.base, MakeColor( b3_colorCyan ) );
		DrawSolidCapsule( m_transformB, m_capsule, MakeColor( b3_colorGreen ) );

		Manifold::Render();
	}

	void Step() override
	{
		if ( m_useCache == false )
		{
			m_simplexCache = {};
		}

		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		b3CollideHullAndCapsule( &m_manifold, m_pointCapacity, &m_hull.base, &m_capsule, transformBtoA, &m_simplexCache );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new CapsuleAndHull( sampleContext );
	}

	b3BoxHull m_hull;
	b3Capsule m_capsule;
};

static int sampleCapsuleAndHull = RegisterSample( "Manifold", "Capsule vs Hull", CapsuleAndHull::Create );

class TriangleAndCapsule : public TriangleManifold
{
public:
	static Sample* Create( SampleContext* sampleContext )
	{
		return new TriangleAndCapsule( sampleContext );
	}

	explicit TriangleAndCapsule( SampleContext* sampleContext )
		: TriangleManifold( sampleContext )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 10.0f, b3Pos_zero );
		}

		m_capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.05f };
		m_triangle[0] = { -4.0f, 0.0f, -4.0f };
		m_triangle[1] = { -4.0f, 0.0f, 0.0f };
		m_triangle[2] = { 0.0f, 0.0f, 0.0f };

		m_transformA = b3WorldTransform_identity;
		m_transformB = { { -1.0f, 0.0f, -1.0f }, b3Quat_identity };
	}

	void Render() override
	{
		DrawSolidCapsule( m_transformB, m_capsule, MakeColorAlpha( b3_colorGreen, 0.5f ) );
		DrawAxes( m_transformB, 0.1f );

		TriangleManifold::Render();
	}

	void Step() override
	{
		if ( m_useCache == false )
		{
			m_simplexCache = {};
		}

		// Put triangle into capsule coordinates
		b3Transform xf = b3InvMulWorldTransforms( m_transformB, m_transformA );
		b3Vec3 localTriangle[3] = {
			b3TransformPoint( xf, m_triangle[0] ),
			b3TransformPoint( xf, m_triangle[1] ),
			b3TransformPoint( xf, m_triangle[2] ),
		};

		b3CollideTriangleAndCapsule( &m_manifold, m_pointCapacity, localTriangle, &m_capsule, &m_simplexCache );
	}

	b3Capsule m_capsule;
};

static int sampleCapsuleAndTriangle = RegisterSample( "Manifold", "Triangle vs Capsule", TriangleAndCapsule::Create );

class HullAndHull : public Manifold
{
public:
	explicit HullAndHull( SampleContext* context )
		: Manifold( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 4.0f, b3Pos_zero );
		}

		// m_boxA = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, { { 0.0f, -0.5f, 0.0f }, b3Quat_identity } );

		b3Transform transform = { { 1.0f, 0.5f, 0.0f }, b3Quat_identity };
		m_boxA = b3MakeTransformedBoxHull( 0.5f, 1.0f, 1.0f, transform );

		// m_hull = CreateConvex( 0.6f, 0.0f, 0.95f, 1.0f, context->arena );
		m_hullA = &m_boxA.base;

		m_boxB = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		// m_hull = CreateConvex( 0.6f, 0.0f, 0.95f, 1.0f, context->arena );
		m_hullB = &m_boxB.base;

		/*
-		xfA	{p=[ -10.0000000, 0.00000000, -10.0000000 ] q=[ [ 0.00000000, 0.00000000, 0.00000000 ], 1.00000000 ] }	b3Transform
+		p	[ -10.0000000, 0.00000000, -10.0000000 ]	b3Vec3
+		q	[ [ 0.00000000, 0.00000000, 0.00000000 ], 1.00000000 ]	b3Quat
-		xfB	{p=[ -10.0000000, 1.00000000, -10.0000000 ] q=[ [ 0.00000000, 0.00000000, 0.00000000 ], 1.00000000 ] }	b3Transform
+		p	[ -10.0000000, 1.00000000, -10.0000000 ]	b3Vec3
+		q	[ [ 0.00000000, 0.00000000, 0.00000000 ], 1.00000000 ]	b3Quat

		*/
		m_transformA = { { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
		m_transformB = { { 0.0f, 0.0f, 0.0f }, b3Quat_identity };

		/*
			p	[ 0.691772044, 0.951781273, 0.0741987228 ]	b3Vec3
			q	[ [ -0.00162522844, -0.0456664972, 0.0355294086 ], 0.998323441 ]	b3Quat
		 */
		// m_transformB = { { 0.691772044, 0.951781273, 0.0741987228 },
		//				 { { -0.00162522844, -0.0456664972, 0.0355294086 }, 0.998323441 } };

		m_satCache = {};
	}

	~HullAndHull() override
	{
		if ( m_hullA != &m_boxA.base )
		{
			b3DestroyHull( m_hullA );
		}

		if ( m_hullB != &m_boxB.base )
		{
			b3DestroyHull( m_hullB );
		}
	}

	b3HullData* CreateConvex( float radius1, float height1, float radius2, float height2 ) const
	{
		constexpr int sideCount = 32;
		const float deltaAlpha = 2.0f * B3_PI / sideCount;

		int vertexCount = 2 * sideCount;
		b3Vec3 vertexBase[2 * sideCount];

		float alpha = 0.0f;
		for ( int sideIndex = 0; sideIndex < sideCount; ++sideIndex )
		{
			b3CosSin cs = b3ComputeCosSin( alpha );

			float x1 = radius1 * cs.cosine;
			float z1 = radius1 * cs.sine;
			float x2 = radius2 * cs.cosine;
			float z2 = radius2 * cs.sine;

			vertexBase[2 * sideIndex + 0] = { x1, height1, z1 };
			vertexBase[2 * sideIndex + 1] = { x2, height2, z2 };
			alpha += deltaAlpha;
		}

		return b3CreateHull( vertexBase, vertexCount, vertexCount );
	}

	void Render() override
	{
		DrawHull( m_transformA, m_hullA, MakeColor( b3_colorGreen ) );
		DrawHull( m_transformB, m_hullB, MakeColor( b3_colorCyan ) );

		Manifold::Render();
	}

	void Step() override
	{
		if ( m_useCache == true )
		{
			if ( m_manualFeature == 1 )
			{
				m_satCache.type = b3_manualFaceAxisA;
			}
			else if ( m_manualFeature == 2 )
			{
				m_satCache.type = b3_manualFaceAxisB;
			}
			else if ( m_manualFeature == 3 )
			{
				m_satCache.type = b3_manualEdgePairAxis;
			}
		}
		else
		{
			m_satCache = {};
		}

		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		b3CollideHulls( &m_manifold, m_pointCapacity, m_hullA, m_hullB, transformBtoA, &m_satCache );

		DrawTextLine( "SAT type: %d", m_satCache.type );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new HullAndHull( sampleContext );
	}

	b3BoxHull m_boxA;
	b3BoxHull m_boxB;
	b3HullData* m_hullA;
	b3HullData* m_hullB;
};

static int sampleCollideHulls = RegisterSample( "Manifold", "Hull vs Hull", HullAndHull::Create );

// Shows the candidate axes culled by the inscribed sphere bound in the separating axis test. No axis n
// can separate the hulls by more than dot(n, centerB - centerA) - innerRadiusA - innerRadiusB, so any
// face or edge whose bound is below the best separation found so far is skipped. This recomputes the
// culling decisions of b3ComputeSeparatingAxis from public hull data and must be kept in sync with it.
class ComplexHullCulling : public Manifold
{
public:
	enum FeatureState
	{
		e_unreached = 0,
		e_culled,
		e_tested,
	};

	explicit ComplexHullCulling( SampleContext* context )
		: Manifold( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 20.0f, 20.0f, 14.0f, { 0.0f, 1.8f, 0.0f } );
		}

		m_hullA = b3CreateComplexHull( 2.0f );
		m_hullB = b3CreateComplexHull( 2.0f );

		m_transformA = { { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
		m_transformB = { { 0.4f, 3.6f, 0.2f }, b3MakeQuatFromAxisAngle( b3Normalize( { 1.0f, 0.0f, 1.0f } ), 0.3f ) };

		m_showSpheres = true;
		m_showCulled = true;
		m_showNormals = true;
		ResetCounts();
	}

	~ComplexHullCulling() override
	{
		b3DestroyHull( m_hullA );
		b3DestroyHull( m_hullB );
	}

	bool DrawControls() override
	{
		ImGui::Checkbox( "Inscribed spheres", &m_showSpheres );
		ImGui::Checkbox( "Culled features", &m_showCulled );
		ImGui::Checkbox( "Face normals", &m_showNormals );
		return true;
	}

	void ResetCounts()
	{
		memset( m_faceStateA, 0, sizeof( m_faceStateA ) );
		memset( m_faceStateB, 0, sizeof( m_faceStateB ) );
		memset( m_edgeStateA, 0, sizeof( m_edgeStateA ) );
		memset( m_edgeStateB, 0, sizeof( m_edgeStateB ) );
		m_seedA = 0;
		m_seedB = 0;
		m_keptEdgeCountA = 0;
		m_keptEdgeCountB = 0;
		m_separationA = -FLT_MAX;
		m_separationB = -FLT_MAX;
		m_gap = 0.0f;
		m_separatedFeature = b3_invalidAxis;
	}

	// Face of A against the vertices of B, in frame A
	static float FaceSeparationA( const b3HullData* hullA, const b3HullData* hullB, b3Transform transformBtoA, int faceIndex )
	{
		b3Plane plane = b3GetHullPlanes( hullA )[faceIndex];
		const b3Vec3* points = b3GetHullPoints( hullB );
		float separation = FLT_MAX;
		for ( int i = 0; i < hullB->vertexCount; ++i )
		{
			b3Vec3 point = b3TransformPoint( transformBtoA, points[i] );
			separation = b3MinFloat( separation, b3Dot( plane.normal, point ) - plane.offset );
		}
		return separation;
	}

	// Face of B against the vertices of A, in frame B
	static float FaceSeparationB( const b3HullData* hullA, const b3HullData* hullB, b3Transform transformBtoA, int faceIndex )
	{
		b3Plane plane = b3GetHullPlanes( hullB )[faceIndex];
		const b3Vec3* points = b3GetHullPoints( hullA );
		float separation = FLT_MAX;
		for ( int i = 0; i < hullA->vertexCount; ++i )
		{
			b3Vec3 point = b3InvTransformPoint( transformBtoA, points[i] );
			separation = b3MinFloat( separation, b3Dot( plane.normal, point ) - plane.offset );
		}
		return separation;
	}

	// Mirrors b3ArcCanReach
	static bool ArcCanReach( float a, float b, float c, float length, float bound )
	{
		const float parallelTolerance = 1.0e-4f;
		float s = 1.0f - c * c;
		float t = a * a + b * b - 2.0f * a * b * c;
		bool endpoint = b3MaxFloat( a, b ) >= bound;
		bool interior =
			a >= c * b && b >= c * a && length >= bound && ( bound <= 0.0f || s < parallelTolerance || t >= bound * bound * s );
		return endpoint || interior;
	}

	// Mirrors the culling in b3ComputeSeparatingAxis with early return enabled
	void ComputeCulling( b3Transform transformBtoA )
	{
		ResetCounts();

		const b3HullData* hullA = m_hullA;
		const b3HullData* hullB = m_hullB;
		const b3Plane* planesA = b3GetHullPlanes( hullA );
		const b3Plane* planesB = b3GetHullPlanes( hullB );
		float speculativeDistance = B3_SPECULATIVE_DISTANCE;

		b3Vec3 deltaCenter = b3Sub( b3TransformPoint( transformBtoA, hullB->center ), hullA->center );
		float centerDistance = b3Length( deltaCenter );
		float radius = hullA->innerRadius + hullB->innerRadius;
		float radiusBound = radius - ( B3_LINEAR_SLOP + 0.001f * ( centerDistance + radius ) );
		m_gap = centerDistance - radius;

		float dotA[B3_MAX_HULL_FACES];
		for ( int i = 0; i < hullA->faceCount; ++i )
		{
			dotA[i] = b3Dot( planesA[i].normal, deltaCenter );
			m_seedA = dotA[i] > dotA[m_seedA] ? i : m_seedA;
		}

		float floorA = b3MinFloat( FaceSeparationA( hullA, hullB, transformBtoA, m_seedA ), speculativeDistance );
		for ( int i = 0; i < hullA->faceCount; ++i )
		{
			if ( dotA[i] - radiusBound < b3MaxFloat( floorA, m_separationA ) )
			{
				m_faceStateA[i] = e_culled;
				continue;
			}

			m_faceStateA[i] = e_tested;
			float separation = FaceSeparationA( hullA, hullB, transformBtoA, i );
			if ( separation > m_separationA )
			{
				m_separationA = separation;
				if ( separation > speculativeDistance )
				{
					m_separatedFeature = b3_faceAxisA;
					return;
				}
			}
		}

		b3Vec3 deltaCenterB = b3Neg( b3InvRotateVector( transformBtoA.q, deltaCenter ) );
		float dotB[B3_MAX_HULL_FACES];
		for ( int i = 0; i < hullB->faceCount; ++i )
		{
			dotB[i] = b3Dot( planesB[i].normal, deltaCenterB );
			m_seedB = dotB[i] > dotB[m_seedB] ? i : m_seedB;
		}

		float floorB = b3MaxFloat( FaceSeparationB( hullA, hullB, transformBtoA, m_seedB ), m_separationA );
		floorB = b3MinFloat( floorB, speculativeDistance );
		for ( int i = 0; i < hullB->faceCount; ++i )
		{
			if ( dotB[i] - radiusBound < b3MaxFloat( floorB, m_separationB ) )
			{
				m_faceStateB[i] = e_culled;
				continue;
			}

			m_faceStateB[i] = e_tested;
			float separation = FaceSeparationB( hullA, hullB, transformBtoA, i );
			if ( separation > m_separationB )
			{
				m_separationB = separation;
				if ( separation > speculativeDistance )
				{
					m_separatedFeature = b3_faceAxisB;
					return;
				}
			}
		}

		float edgeBound = b3MaxFloat( m_separationA, m_separationB ) + radiusBound;

		const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
		for ( int i = 0; i < hullA->edgeCount; i += 2 )
		{
			int face1 = edgesA[i].face;
			int face2 = edgesA[i + 1].face;
			float c = b3Dot( planesA[face1].normal, planesA[face2].normal );
			bool kept = ArcCanReach( dotA[face1], dotA[face2], c, centerDistance, edgeBound );
			m_edgeStateA[i / 2] = kept ? e_tested : e_culled;
			m_keptEdgeCountA += kept ? 1 : 0;
		}

		const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );
		for ( int i = 0; i < hullB->edgeCount; i += 2 )
		{
			int face1 = edgesB[i].face;
			int face2 = edgesB[i + 1].face;
			float c = b3Dot( planesB[face1].normal, planesB[face2].normal );
			bool kept = ArcCanReach( dotB[face1], dotB[face2], c, centerDistance, edgeBound );
			m_edgeStateB[i / 2] = kept ? e_tested : e_culled;
			m_keptEdgeCountB += kept ? 1 : 0;
		}
	}

	void Step() override
	{
		// Clear the cache so every step runs the full separating axis test
		m_satCache = {};
		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		b3CollideHulls( &m_manifold, m_pointCapacity, m_hullA, m_hullB, transformBtoA, &m_satCache );
		ComputeCulling( transformBtoA );
	}

	static int CountState( const uint8_t* states, int count, FeatureState state )
	{
		int n = 0;
		for ( int i = 0; i < count; ++i )
		{
			n += states[i] == state ? 1 : 0;
		}
		return n;
	}

	void DrawEdges( b3WorldTransform transform, const b3HullData* hull, const uint8_t* states, Vec4 keptColor )
	{
		const b3HullHalfEdge* edges = b3GetHullEdges( hull );
		const b3Vec3* points = b3GetHullPoints( hull );
		for ( int i = 0; i < hull->edgeCount; i += 2 )
		{
			b3Pos p1 = b3TransformWorldPoint( transform, points[edges[i].origin] );
			b3Pos p2 = b3TransformWorldPoint( transform, points[edges[i + 1].origin] );
			uint8_t state = states[i / 2];
			if ( state == e_tested )
			{
				DrawLineEx( p1, p2, keptColor, 4.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );
			}
			else if ( state == e_culled && m_showCulled )
			{
				DrawLine( p1, p2, MakeColor( b3_colorSlateGray ) );
			}
			else if ( state == e_unreached )
			{
				DrawLineEx( p1, p2, MakeColor( b3_colorDimGray ), 1.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );
			}
		}
	}

	void DrawFaceNormals( b3WorldTransform transform, const b3HullData* hull, const uint8_t* states, int seed, Vec4 testedColor )
	{
		const b3HullFace* faces = b3GetHullFaces( hull );
		const b3HullHalfEdge* edges = b3GetHullEdges( hull );
		const b3Plane* planes = b3GetHullPlanes( hull );
		const b3Vec3* points = b3GetHullPoints( hull );
		float length = 0.4f * b3GetLengthUnitsPerMeter();

		for ( int i = 0; i < hull->faceCount; ++i )
		{
			uint8_t state = states[i];
			if ( state == e_unreached || ( state == e_culled && m_showCulled == false ) )
			{
				continue;
			}

			b3Vec3 centroid = b3Vec3_zero;
			int count = 0;
			int edgeIndex = faces[i].edge;
			do
			{
				centroid = b3Add( centroid, points[edges[edgeIndex].origin] );
				count += 1;
				edgeIndex = edges[edgeIndex].next;
			}
			while ( edgeIndex != faces[i].edge );
			centroid = b3MulSV( 1.0f / (float)count, centroid );

			b3Pos p1 = b3TransformWorldPoint( transform, centroid );
			b3Pos p2 = b3TransformWorldPoint( transform, b3MulAdd( centroid, length, planes[i].normal ) );

			if ( i == seed )
			{
				DrawArrowEx( p1, p2, MakeColor( b3_colorGold ), 3.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM, 0.25f );
			}
			else if ( state == e_tested )
			{
				DrawArrowEx( p1, p2, testedColor, 2.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM, 0.25f );
			}
			else
			{
				DrawLine( p1, p2, MakeColor( b3_colorSlateGray ) );
			}
		}
	}

	void Render() override
	{
		const b3HullData* hullA = m_hullA;
		const b3HullData* hullB = m_hullB;

		DrawEdges( m_transformA, hullA, m_edgeStateA, MakeColor( b3_colorOrange ) );
		DrawEdges( m_transformB, hullB, m_edgeStateB, MakeColor( b3_colorDeepSkyBlue ) );

		if ( m_showNormals )
		{
			DrawFaceNormals( m_transformA, hullA, m_faceStateA, m_seedA, MakeColor( b3_colorOrange ) );
			DrawFaceNormals( m_transformB, hullB, m_faceStateB, m_seedB, MakeColor( b3_colorDeepSkyBlue ) );
		}

		b3Pos centerA = b3TransformWorldPoint( m_transformA, hullA->center );
		b3Pos centerB = b3TransformWorldPoint( m_transformB, hullB->center );
		DrawLine( centerA, centerB, MakeColor( b3_colorWhite ) );

		if ( m_showSpheres )
		{
			b3Sphere sphereA = { hullA->center, hullA->innerRadius };
			b3Sphere sphereB = { hullB->center, hullB->innerRadius };
			DrawWireSphere( m_transformA, &sphereA, 32, MakeColor( b3_colorOrange ) );
			DrawWireSphere( m_transformB, &sphereB, 32, MakeColor( b3_colorDeepSkyBlue ) );
		}

		int faceCountA = hullA->faceCount;
		int faceCountB = hullB->faceCount;
		int edgeCountA = hullA->edgeCount / 2;
		int edgeCountB = hullB->edgeCount / 2;
		int testedA = CountState( m_faceStateA, faceCountA, e_tested );
		int testedB = CountState( m_faceStateB, faceCountB, e_tested );
		int culledA = CountState( m_faceStateA, faceCountA, e_culled );
		int culledB = CountState( m_faceStateB, faceCountB, e_culled );
		bool edgesReached = m_separatedFeature == b3_invalidAxis;
		int seedCount = m_separatedFeature == b3_faceAxisA ? 1 : 2;

		DrawTextLine( "drag to move B, shift + drag to rotate B" );
		DrawTextLine( "vertices %d, faces %d, edges %d", hullA->vertexCount, faceCountA, edgeCountA );
		DrawTextLine( "|d| - rA - rB = %.3f", m_gap );
		DrawTextLine( "faces A: tested %d, culled %d of %d, best separation %.4f", testedA, culledA, faceCountA, m_separationA );

		if ( m_separatedFeature == b3_faceAxisA )
		{
			DrawTextLine( "separated by face A, B faces and edges not reached" );
		}
		else
		{
			DrawTextLine( "faces B: tested %d, culled %d of %d, best separation %.4f", testedB, culledB, faceCountB,
						  m_separationB );
		}

		if ( m_separatedFeature == b3_faceAxisB )
		{
			DrawTextLine( "separated by face B, edges not reached" );
		}

		DrawTextLine( "support queries: %d of %d (including %d seed)", testedA + testedB + seedCount, faceCountA + faceCountB,
					  seedCount );

		if ( edgesReached )
		{
			int pairCount = m_keptEdgeCountA * m_keptEdgeCountB;
			int totalPairCount = edgeCountA * edgeCountB;
			DrawTextLine( "edges kept: A %d of %d, B %d of %d", m_keptEdgeCountA, edgeCountA, m_keptEdgeCountB, edgeCountB );
			DrawTextLine( "edge pairs: %d of %d (%.1f%%)", pairCount, totalPairCount, 100.0f * pairCount / totalPairCount );
		}

		DrawTextLine( "SAT type: %d", m_satCache.type );

		Manifold::Render();
	}

	static Sample* Create( SampleContext* context )
	{
		return new ComplexHullCulling( context );
	}

	b3HullData* m_hullA;
	b3HullData* m_hullB;
	uint8_t m_faceStateA[B3_MAX_HULL_FACES];
	uint8_t m_faceStateB[B3_MAX_HULL_FACES];
	uint8_t m_edgeStateA[B3_MAX_HULL_EDGES];
	uint8_t m_edgeStateB[B3_MAX_HULL_EDGES];
	int m_seedA;
	int m_seedB;
	int m_keptEdgeCountA;
	int m_keptEdgeCountB;
	float m_separationA;
	float m_separationB;
	float m_gap;
	b3SeparatingFeature m_separatedFeature;
	bool m_showSpheres;
	bool m_showCulled;
	bool m_showNormals;
};

static int sampleComplexHullCulling = RegisterSample( "Manifold", "Complex Hull Culling", ComplexHullCulling::Create );

class TriangleAndHull : public TriangleManifold
{
public:
	static Sample* Create( SampleContext* sampleContext )
	{
		return new TriangleAndHull( sampleContext );
	}

	explicit TriangleAndHull( SampleContext* sampleContext )
		: TriangleManifold( sampleContext )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 3.0f, b3Pos_zero );
		}

		//m_triangle[0] = { 1.00000000, 0, 1.00000000 };
		//m_triangle[1] = { 1.00000000, 0, 0.00000000 };
		//m_triangle[2] = { 0.00000000, 0, 0.00000000 };

		m_triangle[0] = { 0.299769998f, -1.01549578f, -0.744717002f };
		m_triangle[1] = { 0.299769998f, -1.01549578f, 1.28728306f   };
		m_triangle[2] = { 0.299769998f, -0.913895786f, 0.271283031f };

		float bodyHalfWidth = 0.304800004f;
		float bodyHalfHeight = 0.914399981f;

		m_boxHull = b3MakeBoxHull( bodyHalfWidth, bodyHalfHeight, bodyHalfWidth );

		m_transformA = b3WorldTransform_identity;
		m_transformB = b3WorldTransform_identity;
		//m_transformB.p = { -2.16650009f, 0.912535489f, 0.00000000f };

		// b3MeshEdgeFlags
		m_flags = 0;

		/*
		 *		separation	-0.0415397286	float
		type	2 '\x2'	unsigned char
		indexA	0 '\0'	unsigned char
		indexB	1 '\x1'	unsigned char
		hit	1 '\x1'	unsigned char

		 */

		m_satCache = {
			.separation = -0.0107582733f,
			.type = 1,
			.indexA = 0,
			.indexB = 4,
		};

		m_satCache = {};
		m_useCache = false;
		m_cylinder = b3CreateCylinder( 0.4f, 0.05f, 0.0f, 6 );
		m_hull = &m_boxHull.base;
	}

	~TriangleAndHull() override
	{
		b3DestroyHull( m_cylinder );
	}

	void Render() override
	{
		DrawHull( m_transformB, m_hull, MakeColor( b3_colorGreen ) );

		b3WorldTransform xf;
		xf.p = b3TransformWorldPoint( m_transformB, m_hull->center );
		xf.q = m_transformB.q;
		DrawAxes( xf, 0.1f );

		TriangleManifold::Render();
	}

	void Step() override
	{
		if ( m_useCache == true )
		{
			if ( m_manualFeature == 1 )
			{
				m_satCache.type = b3_manualFaceAxisA;
			}
			else if ( m_manualFeature == 2 )
			{
				m_satCache.type = b3_manualFaceAxisB;
			}
			else if ( m_manualFeature == 3 )
			{
				m_satCache.type = b3_manualEdgePairAxis;
			}
		}
		else
		{
			m_satCache = {};
		}

		b3Transform xf = b3InvMulWorldTransforms( m_transformB, m_transformA );
		b3Vec3 localTriangle[3] = {
			b3TransformPoint( xf, m_triangle[0] ),
			b3TransformPoint( xf, m_triangle[1] ),
			b3TransformPoint( xf, m_triangle[2] ),
		};

		b3CollideTriangleAndHull( &m_manifold, m_pointCapacity, localTriangle[0], localTriangle[1], localTriangle[2],
								  m_flags, m_hull, &m_satCache, true );
	}

	int m_flags;
	b3BoxHull m_boxHull;
	b3HullData* m_cylinder;
	b3HullData* m_hull;
};

static int sampleHullAndTriangle = RegisterSample( "Manifold", "Triangle vs Hull", TriangleAndHull::Create );
