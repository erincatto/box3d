// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/draw.h"
#include "gfx/keycodes.h"
#include "sample.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"

#include <imgui.h>

// Trackball about the camera axes so the near side of the shape follows the cursor regardless of the
// shape or camera orientation. A drag across the window height is a full turn.
static b3Quat DragRotation( const Camera* camera, b3Quat q, float dx, float dy )
{
	float scale = 2.0f * B3_PI / b3MaxFloat( (float)camera->m_height, 1.0f );
	b3Quat yaw = b3MakeQuatFromAxisAngle( camera->GetUp(), scale * dx );
	b3Quat pitch = b3MakeQuatFromAxisAngle( camera->GetRight(), scale * dy );
	return b3NormalizeQuat( b3MulQuat( b3MulQuat( pitch, yaw ), q ) );
}

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
		m_lastX = 0.0f;
		m_lastY = 0.0f;
		m_origin = b3Pos_zero;
		m_drawPoints = true;
		m_useCache = false;
		m_tracking = false;
		m_rotating = false;
	}

	void Render() override
	{
		DrawTextLine( "origin: %g %g %g", m_origin.x, m_origin.y, m_origin.z );
		DrawTextLine( "count = %d", m_manifold.pointCount );

		DrawAxes( b3WorldTransform_identity, 0.5f );

		if ( m_manifold.pointCount == 0 || m_drawPoints == false )
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

		ImGui::Checkbox( "Draw Points", &m_drawPoints );

		return true;
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 && ( modifiers & MOD_ALT ) == 0 )
		{
			if ( modifiers & MOD_SHIFT )
			{
				m_lastX = p.x;
				m_lastY = p.y;
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
			m_transformB.q = DragRotation( m_camera, m_transformB.q, p.x - m_lastX, p.y - m_lastY );
			m_lastX = p.x;
			m_lastY = p.y;
		}
	}

	static constexpr int m_pointCapacity = 64;
	b3LocalManifold m_manifold;
	b3LocalManifoldPoint m_points[m_pointCapacity];
	b3WorldTransform m_transformA;
	b3WorldTransform m_transformB;
	b3Pos m_baseTranslation;
	b3Pos m_origin;
	b3SimplexCache m_simplexCache;
	b3SATCache m_satCache;
	int m_manualFeature;
	float m_lastX;
	float m_lastY;
	bool m_useCache;
	bool m_tracking;
	bool m_rotating;
	bool m_drawPoints;
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
		m_lastX = 0.0f;
		m_lastY = 0.0f;
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
				m_lastX = p.x;
				m_lastY = p.y;
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
			m_transformB.q = DragRotation( m_camera, m_transformB.q, p.x - m_lastX, p.y - m_lastY );
			m_lastX = p.x;
			m_lastY = p.y;
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
	b3Pos m_origin;
	b3SimplexCache m_simplexCache;
	b3SATCache m_satCache;
	int m_manualFeature;
	float m_lastX;
	float m_lastY;
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
class HullCulling : public Manifold
{
public:
	enum FeatureState
	{
		e_unreached = 0,
		e_culled,
		e_tested,
	};

	explicit HullCulling( SampleContext* context )
		: Manifold( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 20.0f, 20.0f, 14.0f, { 0.0f, 1.8f, 0.0f } );
		}

		m_useCylinders = false;
		m_cylinderHeight = 2.0f;
		m_cylinderRadius = 2.0f;
		m_cylinderSides = 32;
		m_hullA = nullptr;
		m_hullB = nullptr;
		CreateHulls();

		m_transformA = { { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
		m_transformB = { { 0.4f, 3.6f, 0.2f }, b3MakeQuatFromAxisAngle( b3Normalize( { 1.0f, 0.0f, 1.0f } ), 0.3f ) };

		m_showSpheres = true;
		m_showCulled = true;
		m_showNormals = true;
		ResetCounts();
	}

	~HullCulling() override
	{
		b3DestroyHull( m_hullA );
		b3DestroyHull( m_hullB );
	}

	void CreateHulls()
	{
		if ( m_hullA != nullptr )
		{
			b3DestroyHull( m_hullA );
			b3DestroyHull( m_hullB );
		}

		if ( m_useCylinders )
		{
			float yOffset = -0.5f * m_cylinderHeight;
			m_hullA = b3CreateCylinder( m_cylinderHeight, m_cylinderRadius, yOffset, m_cylinderSides );
			m_hullB = b3CreateCylinder( m_cylinderHeight, m_cylinderRadius, yOffset, m_cylinderSides );
		}
		else
		{
			m_hullA = b3CreateComplexHull( 2.0f );
			m_hullB = b3CreateComplexHull( 2.0f );
		}
	}

	bool DrawControls() override
	{
		Manifold::DrawControls();

		ImGui::Checkbox( "Inscribed spheres", &m_showSpheres );
		ImGui::Checkbox( "Culled features", &m_showCulled );
		ImGui::Checkbox( "Face normals", &m_showNormals );

		bool rebuild = ImGui::Checkbox( "Cylinders", &m_useCylinders );
		if ( m_useCylinders )
		{
			rebuild |= ImGui::SliderFloat( "Height", &m_cylinderHeight, 0.1f, 8.0f, "%.2f" );
			rebuild |= ImGui::SliderFloat( "Radius", &m_cylinderRadius, 0.1f, 4.0f, "%.2f" );
			rebuild |= ImGui::SliderInt( "Sides", &m_cylinderSides, 3, 32 );
		}

		if ( rebuild )
		{
			m_cylinderHeight = b3ClampFloat( m_cylinderHeight, 0.1f, 8.0f );
			m_cylinderRadius = b3ClampFloat( m_cylinderRadius, 0.1f, 4.0f );
			m_cylinderSides = b3ClampInt( m_cylinderSides, 3, 32 );
			CreateHulls();
		}

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
		return new HullCulling( context );
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
	float m_cylinderHeight;
	float m_cylinderRadius;
	int m_cylinderSides;
	bool m_useCylinders;
	bool m_showSpheres;
	bool m_showCulled;
	bool m_showNormals;
};

static int sampleHullCulling = RegisterSample( "Manifold", "Hull Culling", HullCulling::Create );

// Shows that an edge pair can win the separating axis test even when the direction between the hull
// centers peaks outside the Gauss map arc (wedge) of one of its edges. The inscribed sphere bound of such
// an edge peaks at a face normal at the end of its arc, yet the edge axis beats the real separation of
// those faces. So outside wedge edges can only be culled by their sphere bound, not by the face results.
// Everything here is brute force over all faces and all edge pairs, independent of the pruning in the SAT.
class EdgeAxisWedge : public Manifold
{
public:
	struct EdgeAxis
	{
		b3Vec3 normal;
		float separation;
		int edgeA;
		int edgeB;
	};

	// Gauss map arc of one edge in frame A, with the peak of dot(n, d) on its great circle
	struct Arc
	{
		b3Vec3 u;
		b3Vec3 v;
		b3Vec3 peak;
		float a;
		float b;
		float c;
		float separationU;
		float separationV;

		// Angle from the peak to the nearest end of the arc, negative when the peak is on the arc
		float outsideDegrees;
		bool inside;
	};

	explicit EdgeAxisWedge( SampleContext* context )
		: Manifold( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 22.0f, { -4.5f, 1.0f, 0.0f } );
		}

		m_transformA = { { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
		m_transformB = { { 0.0f, 3.6f, 0.0f }, b3Quat_identity };

		m_gaussCenter = { -10.0f, 1.0f, 0.0f };
		m_gaussRadius = 2.0f;
		m_searchCount = 0;
		m_searchFound = false;
		m_hullType = e_slab;
		m_createdHull = nullptr;

		// Start with an example
		SetHull( m_hullType );
	}

	~EdgeAxisWedge() override
	{
		if ( m_createdHull != nullptr )
		{
			b3DestroyHull( m_createdHull );
		}
	}

	enum HullType
	{
		e_slab = 0,
		e_tetrahedron,
		e_complex,
	};

	// Both hulls are the same shape
	void SetHull( int type )
	{
		if ( m_createdHull != nullptr )
		{
			b3DestroyHull( m_createdHull );
			m_createdHull = nullptr;
		}

		if ( type == e_slab )
		{
			m_box = b3MakeBoxHull( 2.0f, 0.25f, 1.0f );
			m_hullA = &m_box.base;
		}
		else if ( type == e_tetrahedron )
		{
			b3Vec3 points[4] = { { 1.0f, 1.0f, 1.0f }, { 1.0f, -1.0f, -1.0f }, { -1.0f, 1.0f, -1.0f }, { -1.0f, -1.0f, 1.0f } };
			m_createdHull = b3CreateHull( points, 4, 4 );
			m_hullA = m_createdHull;
		}
		else
		{
			m_createdHull = b3CreateComplexHull( 2.0f );
			m_hullA = m_createdHull;
		}

		m_hullB = m_hullA;
		m_hullType = type;
		m_seed = 12345;
		Search();
	}

	float RandomFloat()
	{
		m_seed = m_seed * 1664525u + 1013904223u;
		return (float)( m_seed >> 8 ) / 16777216.0f;
	}

	b3Vec3 RandomDirection()
	{
		b3Vec3 v;
		float lengthSquared;
		do
		{
			v = { 2.0f * RandomFloat() - 1.0f, 2.0f * RandomFloat() - 1.0f, 2.0f * RandomFloat() - 1.0f };
			lengthSquared = b3Dot( v, v );
		}
		while ( lengthSquared > 1.0f || lengthSquared < 0.01f );

		return b3Normalize( v );
	}

	// Mirrors the Gauss map test and edge axis of the scalar path in b3ComputeSeparatingAxis, without pruning
	static EdgeAxis FindBestEdgeAxis( const b3HullData* hullA, const b3HullData* hullB, b3Transform transformBtoA )
	{
		const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
		const b3Plane* planesA = b3GetHullPlanes( hullA );
		const b3Vec3* pointsA = b3GetHullPoints( hullA );
		const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );
		const b3Plane* planesB = b3GetHullPlanes( hullB );
		const b3Vec3* pointsB = b3GetHullPoints( hullB );

		const float eps = -0.0001f;
		float squaredTolerance = B3_PARALLEL_EDGE_TOL * B3_PARALLEL_EDGE_TOL;

		EdgeAxis best = { b3Vec3_zero, -FLT_MAX, B3_NULL_INDEX, B3_NULL_INDEX };

		for ( int j = 0; j < hullB->edgeCount; j += 2 )
		{
			// B face normals and vertices in frame A, negated
			b3Vec3 C = b3Neg( b3RotateVector( transformBtoA.q, planesB[edgesB[j].face].normal ) );
			b3Vec3 D = b3Neg( b3RotateVector( transformBtoA.q, planesB[edgesB[j + 1].face].normal ) );
			b3Vec3 bv0 = b3Neg( b3TransformPoint( transformBtoA, pointsB[edgesB[j].origin] ) );
			b3Vec3 bv1 = b3Neg( b3TransformPoint( transformBtoA, pointsB[edgesB[j + 1].origin] ) );
			b3Vec3 DC = b3Sub( bv1, bv0 );

			for ( int i = 0; i < hullA->edgeCount; i += 2 )
			{
				b3Vec3 n0 = planesA[edgesA[i].face].normal;
				b3Vec3 n1 = planesA[edgesA[i + 1].face].normal;
				b3Vec3 av0 = pointsA[edgesA[i].origin];
				b3Vec3 edge = b3Sub( pointsA[edgesA[i + 1].origin], av0 );

				float CBA = b3Dot( C, edge );
				float DBA = b3Dot( D, edge );
				float ADC = b3Dot( n0, DC );
				float BDC = b3Dot( n1, DC );
				if ( CBA * DBA >= eps || ADC * BDC >= eps || CBA * BDC >= eps )
				{
					continue;
				}

				if ( b3MaxFloat( CBA * CBA, DBA * DBA ) <= squaredTolerance * b3Dot( edge, edge ) )
				{
					continue;
				}

				float t = -CBA / ( DBA - CBA );
				b3Vec3 normal = b3Normalize( b3MulAdd( C, t, b3Sub( D, C ) ) );
				float separation = -b3Dot( normal, b3Add( av0, bv0 ) );
				if ( separation > best.separation )
				{
					best = { normal, separation, i, j };
				}
			}
		}

		return best;
	}

	// The peak of dot(n, d) over the great circle through u and v, and whether it lies on the arc
	static Arc MakeArc( b3Vec3 u, b3Vec3 v, b3Vec3 d, float separationU, float separationV )
	{
		Arc arc;
		arc.u = u;
		arc.v = v;
		arc.a = b3Dot( u, d );
		arc.b = b3Dot( v, d );
		arc.c = b3Dot( u, v );
		arc.inside = arc.a >= arc.c * arc.b && arc.b >= arc.c * arc.a;
		arc.separationU = separationU;
		arc.separationV = separationV;

		b3Vec3 axis = b3Normalize( b3Cross( u, v ) );
		b3Vec3 inPlane = b3MulSub( d, b3Dot( d, axis ), axis );
		arc.peak = b3LengthSquared( inPlane ) > 1.0e-12f ? b3Normalize( inPlane ) : u;

		float arcAngle = acosf( b3ClampFloat( arc.c, -1.0f, 1.0f ) );
		float angleU = acosf( b3ClampFloat( b3Dot( u, arc.peak ), -1.0f, 1.0f ) );
		float angleV = acosf( b3ClampFloat( b3Dot( v, arc.peak ), -1.0f, 1.0f ) );
		float nearest = b3MinFloat( angleU, angleV ) * 180.0f / B3_PI;
		arc.outsideDegrees = ( angleU <= arcAngle && angleV <= arcAngle ) ? -nearest : nearest;
		return arc;
	}

	float MaxSeparation( b3Transform transformBtoA, float* faceSeparationA, float* faceSeparationB, EdgeAxis* edgeAxis ) const
	{
		float separationA = -FLT_MAX;
		for ( int i = 0; i < m_hullA->faceCount; ++i )
		{
			separationA = b3MaxFloat( separationA, HullCulling::FaceSeparationA( m_hullA, m_hullB, transformBtoA, i ) );
		}

		float separationB = -FLT_MAX;
		for ( int i = 0; i < m_hullB->faceCount; ++i )
		{
			separationB = b3MaxFloat( separationB, HullCulling::FaceSeparationB( m_hullA, m_hullB, transformBtoA, i ) );
		}

		EdgeAxis edge = FindBestEdgeAxis( m_hullA, m_hullB, transformBtoA );

		if ( faceSeparationA != nullptr )
		{
			*faceSeparationA = separationA;
			*faceSeparationB = separationB;
			*edgeAxis = edge;
		}

		return b3MaxFloat( edge.separation, b3MaxFloat( separationA, separationB ) );
	}

	// Random poses placed just into contact until an edge axis wins with an edge outside its wedge
	void Search()
	{
		m_searchFound = false;
		m_searchCount = 0;

		for ( int trial = 0; trial < 500 && m_searchFound == false; ++trial )
		{
			m_searchCount += 1;

			b3Vec3 direction = RandomDirection();
			b3Vec3 axis = RandomDirection();
			float angle = 2.0f * B3_PI * RandomFloat();
			b3Quat rotation = b3MakeQuatFromAxisAngle( axis, angle );

			// Bisect on the center distance to reach a slight overlap
			float target = -0.02f * b3GetLengthUnitsPerMeter();
			float lower = 0.0f;
			float upper = 1.1f * ( b3Length( b3AABB_Extents( m_hullA->aabb ) ) + b3Length( b3AABB_Extents( m_hullB->aabb ) ) );
			for ( int iteration = 0; iteration < 24; ++iteration )
			{
				float mid = 0.5f * ( lower + upper );
				b3Transform transform = { b3MulSV( mid, direction ), rotation };
				if ( MaxSeparation( transform, nullptr, nullptr, nullptr ) < target )
				{
					lower = mid;
				}
				else
				{
					upper = mid;
				}
			}

			m_transformB = { m_transformA.p + b3MulSV( upper, direction ), rotation };
			Compute();

			// Only accept clear examples, well outside the wedge with a clear edge win
			float margin = m_edge.separation - b3MaxFloat( m_faceSeparationA, m_faceSeparationB );
			float outside = b3MaxFloat( m_arcA.outsideDegrees, m_arcB.outsideDegrees );
			m_searchFound = m_edgeWins && outside >= 10.0f && margin >= 0.02f * b3GetLengthUnitsPerMeter();
		}
	}

	void Compute()
	{
		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		MaxSeparation( transformBtoA, &m_faceSeparationA, &m_faceSeparationB, &m_edge );
		m_edgeWins = m_edge.edgeA != B3_NULL_INDEX && m_edge.separation > b3MaxFloat( m_faceSeparationA, m_faceSeparationB );

		m_centerOffset = b3Sub( b3TransformPoint( transformBtoA, m_hullB->center ), m_hullA->center );
		m_radius = m_hullA->innerRadius + m_hullB->innerRadius;

		if ( m_edge.edgeA == B3_NULL_INDEX )
		{
			return;
		}

		const b3HullHalfEdge* edgesA = b3GetHullEdges( m_hullA );
		const b3Plane* planesA = b3GetHullPlanes( m_hullA );
		int faceA1 = edgesA[m_edge.edgeA].face;
		int faceA2 = edgesA[m_edge.edgeA + 1].face;
		m_arcA = MakeArc( planesA[faceA1].normal, planesA[faceA2].normal, m_centerOffset,
						  HullCulling::FaceSeparationA( m_hullA, m_hullB, transformBtoA, faceA1 ),
						  HullCulling::FaceSeparationA( m_hullA, m_hullB, transformBtoA, faceA2 ) );

		const b3HullHalfEdge* edgesB = b3GetHullEdges( m_hullB );
		const b3Plane* planesB = b3GetHullPlanes( m_hullB );
		int faceB1 = edgesB[m_edge.edgeB].face;
		int faceB2 = edgesB[m_edge.edgeB + 1].face;
		b3Vec3 C = b3Neg( b3RotateVector( transformBtoA.q, planesB[faceB1].normal ) );
		b3Vec3 D = b3Neg( b3RotateVector( transformBtoA.q, planesB[faceB2].normal ) );
		m_arcB = MakeArc( C, D, m_centerOffset, HullCulling::FaceSeparationB( m_hullA, m_hullB, transformBtoA, faceB1 ),
						  HullCulling::FaceSeparationB( m_hullA, m_hullB, transformBtoA, faceB2 ) );
	}

	bool DrawControls() override
	{
		const char* hullNames[] = { "Slab", "Tetrahedron", "Complex Hull" };
		int hullType = m_hullType;
		if ( ImGui::Combo( "Hull", &hullType, hullNames, IM_ARRAYSIZE( hullNames ) ) )
		{
			SetHull( hullType );
		}

		if ( ImGui::Button( "Find Example" ) )
		{
			Search();
		}

		return true;
	}

	void Step() override
	{
		Compute();
	}

	b3Pos GaussPoint( b3Vec3 directionInA ) const
	{
		return m_gaussCenter + b3MulSV( m_gaussRadius, b3RotateVector( m_transformA.q, directionInA ) );
	}

	void DrawGreatArc( b3Vec3 u, b3Vec3 v, Vec4 color, float thickness, OverlayOcclusionMode mode ) const
	{
		constexpr int segmentCount = 32;
		b3Pos previous = GaussPoint( u );
		for ( int i = 1; i <= segmentCount; ++i )
		{
			float s = (float)i / segmentCount;
			b3Pos point = GaussPoint( b3Normalize( b3Add( b3MulSV( 1.0f - s, u ), b3MulSV( s, v ) ) ) );
			DrawLineEx( previous, point, color, thickness, OVERLAY_THICKNESS_PIXELS, mode );
			previous = point;
		}
	}

	// The full great circle through the arc, dashed
	void DrawGreatCircle( b3Vec3 u, b3Vec3 v, Vec4 color ) const
	{
		b3Vec3 axis = b3Normalize( b3Cross( u, v ) );
		b3Vec3 w = b3Cross( axis, u );
		constexpr int segmentCount = 96;
		b3Pos previous = GaussPoint( u );
		for ( int i = 1; i <= segmentCount; ++i )
		{
			float angle = 2.0f * B3_PI * i / segmentCount;
			b3CosSin cs = b3ComputeCosSin( angle );
			b3Pos point = GaussPoint( b3Add( b3MulSV( cs.cosine, u ), b3MulSV( cs.sine, w ) ) );
			DrawLineEx( previous, point, color, 1.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );
			previous = point;
		}
	}

	void DrawEdge( b3WorldTransform transform, const b3HullData* hull, int edgeIndex, Vec4 color ) const
	{
		const b3HullHalfEdge* edges = b3GetHullEdges( hull );
		const b3Vec3* points = b3GetHullPoints( hull );
		b3Pos p1 = b3TransformWorldPoint( transform, points[edges[edgeIndex].origin] );
		b3Pos p2 = b3TransformWorldPoint( transform, points[edges[edgeIndex + 1].origin] );
		DrawLineEx( p1, p2, color, 5.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );
	}

	void DrawArcText( const char* name, const Arc& arc, float edgeBound )
	{
		if ( arc.inside )
		{
			DrawTextLine( "%s edge: d peak inside wedge", name );
		}
		else
		{
			DrawTextLine( "%s edge: d peak OUTSIDE wedge by %.1f degrees", name, arc.outsideDegrees );
		}
		DrawTextLine( "   sphere bound  u %.3f  v %.3f  n %.3f", arc.a - m_radius, arc.b - m_radius, edgeBound );
		DrawTextLine( "   separation    u %.4f  v %.4f  n %.4f", arc.separationU, arc.separationV, m_edge.separation );
	}

	void Render() override
	{
		DrawHull( m_transformA, m_hullA, MakeColor( b3_colorSlateGray ) );
		DrawHull( m_transformB, m_hullB, MakeColor( b3_colorSlateGray ) );

		b3Pos centerA = b3TransformWorldPoint( m_transformA, m_hullA->center );
		b3Pos centerB = b3TransformWorldPoint( m_transformB, m_hullB->center );
		DrawLine( centerA, centerB, MakeColor( b3_colorWhite ) );

		DrawTextLine( "drag to move B, shift + drag to rotate B" );
		DrawTextLine( "search: %s after %d poses", m_searchFound ? "found" : "not found", m_searchCount );
		DrawTextLine( "best separation: face A %.4f, face B %.4f, edge %.4f -> %s", m_faceSeparationA, m_faceSeparationB,
					  m_edge.separation, m_edgeWins ? "EDGE WINS" : "face wins" );

		if ( m_edge.edgeA == B3_NULL_INDEX )
		{
			Manifold::Render();
			return;
		}

		Vec4 colorA = MakeColor( b3_colorOrange );
		Vec4 colorB = MakeColor( b3_colorDeepSkyBlue );
		Vec4 colorN = MakeColor( b3_colorGold );
		Vec4 colorPeak = MakeColor( b3_colorMagenta );

		DrawEdge( m_transformA, m_hullA, m_edge.edgeA, colorA );
		DrawEdge( m_transformB, m_hullB, m_edge.edgeB, colorB );

		// Winning axis from the midpoint of edge A
		{
			const b3HullHalfEdge* edges = b3GetHullEdges( m_hullA );
			const b3Vec3* points = b3GetHullPoints( m_hullA );
			b3Vec3 mid = b3Lerp( points[edges[m_edge.edgeA].origin], points[edges[m_edge.edgeA + 1].origin], 0.5f );
			b3Pos p = b3TransformWorldPoint( m_transformA, mid );
			b3Vec3 n = b3RotateVector( m_transformA.q, m_edge.normal );
			DrawArrowEx( p, p + b3MulSV( 1.0f, n ), colorN, 3.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM, 0.2f );
		}

		// Gauss map: both arcs cross at the edge axis n. The magenta dots are the peaks of dot(n, d) on each
		// great circle. The peak of an arc is only reachable when it lies on the arc.
		b3Sphere unitSphere = { b3Vec3_zero, m_gaussRadius };
		DrawWireSphere( { m_gaussCenter, b3Quat_identity }, &unitSphere, 24, MakeColor( b3_colorDimGray ) );

		DrawGreatCircle( m_arcA.u, m_arcA.v, colorA );
		DrawGreatCircle( m_arcB.u, m_arcB.v, colorB );
		DrawGreatArc( m_arcA.u, m_arcA.v, colorA, 5.0f, OVERLAY_OCCLUSION_DIM );
		DrawGreatArc( m_arcB.u, m_arcB.v, colorB, 5.0f, OVERLAY_OCCLUSION_DIM );

		DrawPoint( GaussPoint( m_arcA.u ), 8.0f, colorA );
		DrawPoint( GaussPoint( m_arcA.v ), 8.0f, colorA );
		DrawPoint( GaussPoint( m_arcB.u ), 8.0f, colorB );
		DrawPoint( GaussPoint( m_arcB.v ), 8.0f, colorB );
		DrawString3D( GaussPoint( m_arcA.u ), colorA, " uA" );
		DrawString3D( GaussPoint( m_arcA.v ), colorA, " vA" );
		DrawString3D( GaussPoint( m_arcB.u ), colorB, " uB" );
		DrawString3D( GaussPoint( m_arcB.v ), colorB, " vB" );

		DrawPoint( GaussPoint( m_arcA.peak ), 10.0f, colorPeak );
		DrawString3D( GaussPoint( m_arcA.peak ), colorPeak, " peak A" );
		DrawPoint( GaussPoint( m_arcB.peak ), 10.0f, colorPeak );
		DrawString3D( GaussPoint( m_arcB.peak ), colorPeak, " peak B" );

		DrawPoint( GaussPoint( m_edge.normal ), 12.0f, colorN );
		DrawString3D( GaussPoint( m_edge.normal ), colorN, " n" );

		b3Vec3 dHat = b3Normalize( m_centerOffset );
		DrawPoint( GaussPoint( dHat ), 10.0f, MakeColor( b3_colorWhite ) );
		DrawString3D( GaussPoint( dHat ), MakeColor( b3_colorWhite ), " d" );

		float edgeBound = b3Dot( m_edge.normal, m_centerOffset ) - m_radius;
		DrawTextLine( "|d| - rA - rB = %.3f", b3Length( m_centerOffset ) - m_radius );
		DrawArcText( "A", m_arcA, edgeBound );
		DrawArcText( "B", m_arcB, edgeBound );

		Manifold::Render();
	}

	static Sample* Create( SampleContext* context )
	{
		return new EdgeAxisWedge( context );
	}

	b3HullData* m_hullA;
	b3HullData* m_hullB;
	EdgeAxis m_edge;
	Arc m_arcA;
	Arc m_arcB;
	b3Vec3 m_centerOffset;
	b3Pos m_gaussCenter;
	float m_gaussRadius;
	float m_radius;
	float m_faceSeparationA;
	float m_faceSeparationB;
	uint32_t m_seed;
	int m_searchCount;
	bool m_searchFound;
	bool m_edgeWins;
	b3BoxHull m_box;
	b3HullData* m_createdHull;
	int m_hullType;
};

static int sampleEdgeAxisWedge = RegisterSample( "Manifold", "Edge Axis Wedge", EdgeAxisWedge::Create );

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

		// m_triangle[0] = { 1.00000000, 0, 1.00000000 };
		// m_triangle[1] = { 1.00000000, 0, 0.00000000 };
		// m_triangle[2] = { 0.00000000, 0, 0.00000000 };

		m_triangle[0] = { 0.299769998f, -1.01549578f, -0.744717002f };
		m_triangle[1] = { 0.299769998f, -1.01549578f, 1.28728306f };
		m_triangle[2] = { 0.299769998f, -0.913895786f, 0.271283031f };

		float bodyHalfWidth = 0.304800004f;
		float bodyHalfHeight = 0.914399981f;

		m_boxHull = b3MakeBoxHull( bodyHalfWidth, bodyHalfHeight, bodyHalfWidth );

		m_transformA = b3WorldTransform_identity;
		m_transformB = b3WorldTransform_identity;
		// m_transformB.p = { -2.16650009f, 0.912535489f, 0.00000000f };

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

		b3CollideTriangleAndHull( &m_manifold, m_pointCapacity, localTriangle[0], localTriangle[1], localTriangle[2], m_flags,
								  m_hull, &m_satCache, true );
	}

	int m_flags;
	b3BoxHull m_boxHull;
	b3HullData* m_cylinder;
	b3HullData* m_hull;
};

static int sampleHullAndTriangle = RegisterSample( "Manifold", "Triangle vs Hull", TriangleAndHull::Create );
