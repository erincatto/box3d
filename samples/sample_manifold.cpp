// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/draw.h"
#include "gfx/keycodes.h"
#include "sample.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"

#include <imgui.h>
#include <vector>

#include <ImGuizmo.h>

static void DrawGizmoControls( bool* show, bool* local )
{
	ImGui::Checkbox( "Gizmo (G)", show );
	ImGui::Checkbox( "Local axes", local );
}

// Must run inside the ImGui frame. The view is eye relative, so the shape is placed against the
// draw origin.
static void DrawTransformGizmo( const Camera* camera, b3WorldTransform* transform, bool local )
{
	ImGuizmo::BeginFrame();

	// Behind the panels like the in-world labels
	ImGuizmo::SetDrawlist( ImGui::GetBackgroundDrawList() );

	ImGuiViewport* viewport = ImGui::GetMainViewport();
	ImGuizmo::SetRect( viewport->Pos.x, viewport->Pos.y, viewport->Size.x, viewport->Size.y );
	ImGuizmo::SetOrthographic( false );

	// Alt drag orbits the camera
	ImGuizmo::Enable( ImGui::GetIO().KeyAlt == false );

	ImGuizmo::OPERATION operation = ImGuizmo::TRANSLATE | ImGuizmo::ROTATE;

	b3Pos origin = GetDrawOrigin();
	b3Vec3 offset = b3SubPos( transform->p, origin );
	b3Matrix3 r = b3MakeMatrixFromQuat( transform->q );
	Mat4 matrix = {
		{ r.cx.x, r.cx.y, r.cx.z, 0.0f },
		{ r.cy.x, r.cy.y, r.cy.z, 0.0f },
		{ r.cz.x, r.cz.y, r.cz.z, 0.0f },
		{ offset.x, offset.y, offset.z, 1.0f },
	};

	Mat4 view = camera->View();
	Mat4 proj = camera->Proj();
	ImGuizmo::MODE space = local ? ImGuizmo::LOCAL : ImGuizmo::WORLD;
	if ( ImGuizmo::Manipulate( &view.cx.x, &proj.cx.x, operation, space, &matrix.cx.x ) )
	{
		// Rotation drags accumulate in the matrix, the quaternion keeps it orthonormal
		b3Matrix3 m = {
			{ matrix.cx.x, matrix.cx.y, matrix.cx.z },
			{ matrix.cy.x, matrix.cy.y, matrix.cy.z },
			{ matrix.cz.x, matrix.cz.y, matrix.cz.z },
		};
		transform->q = b3NormalizeQuat( b3MakeQuatFromMatrix( &m ) );
		transform->p = b3OffsetPos( origin, { matrix.cw.x, matrix.cw.y, matrix.cw.z } );
	}
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
		m_drawPoints = true;
		m_useCache = false;
		m_showGizmo = true;
		m_gizmoLocal = false;
	}

	void Render() override
	{
		DrawTextLine( "count = %d", m_manifold.pointCount );

		DrawAxes( b3WorldTransform_identity, 0.5f );
		DrawGroundGrid( 80 );

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
			DrawString3D( point + 0.025f * normal + 0.05f * perp, MakeColor( b3_colorPapayaWhip ),
						  "  %X:%X %X:%X", pair.owner1, pair.index1, pair.owner2, pair.index2 );
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
		DrawGizmoControls( &m_showGizmo, &m_gizmoLocal );

		return true;
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		if ( key == KEY_G && action == ACTION_PRESS && modifiers == 0 )
		{
			m_showGizmo = !m_showGizmo;
		}
	}

	void DrawSampleWindows() override
	{
		if ( m_showGizmo )
		{
			DrawTransformGizmo( m_camera, &m_transformB, m_gizmoLocal );
		}
	}

	// The gizmo moves the shape, so keep the picking in Sample off
	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
	}

	static constexpr int m_pointCapacity = 64;
	b3LocalManifold m_manifold;
	b3LocalManifoldPoint m_points[m_pointCapacity];
	b3WorldTransform m_transformA;
	b3WorldTransform m_transformB;
	b3SimplexCache m_simplexCache;
	b3SATCache m_satCache;
	int m_manualFeature;
	bool m_useCache;
	bool m_drawPoints;
	bool m_showGizmo;
	bool m_gizmoLocal;
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
		m_useCache = false;

		m_manualFeature = 0;
		m_showGizmo = true;
		m_gizmoLocal = false;
	}

	void Render() override
	{
		DrawTextLine( "count = %d", m_manifold.pointCount );
		DrawTextLine( "feature = %d", m_manifold.feature );
		DrawTextLine( "cache hit = %d", m_satCache.hit );

		DrawAxes( b3WorldTransform_identity, 1.0f );
		DrawGroundGrid( 10 );

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

				DrawString3D( point, MakeColor( b3_colorWhite ), "  %.2f",
							  100.0f * manifoldPoint.separation );

				b3FeaturePair pair = manifoldPoint.pair;
				DrawString3D( point + 0.025f * normal, MakeColor( b3_colorPapayaWhip ),
							  "  %X:%X %X:%X", pair.owner1, pair.index1, pair.owner2, pair.index2 );
			}
		}

		DrawTriangle( m_transformA, m_triangle[0], m_triangle[1], m_triangle[2],
					  MakeColor( b3_colorCyan ) );

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

		DrawGizmoControls( &m_showGizmo, &m_gizmoLocal );

		return true;
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		if ( key == KEY_G && action == ACTION_PRESS && modifiers == 0 )
		{
			m_showGizmo = !m_showGizmo;
		}
	}

	void DrawSampleWindows() override
	{
		if ( m_showGizmo )
		{
			DrawTransformGizmo( m_camera, &m_transformB, m_gizmoLocal );
		}
	}

	// The gizmo moves the shape, so keep the picking in Sample off
	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
	}

	static constexpr int m_pointCapacity = 8;
	b3LocalManifold m_manifold;
	b3LocalManifoldPoint m_points[m_pointCapacity];

	// Triangle transform
	b3WorldTransform m_transformA;

	// Convex shape transform
	b3WorldTransform m_transformB;

	b3Vec3 m_triangle[3] = {};
	b3SimplexCache m_simplexCache;
	b3SATCache m_satCache;
	int m_manualFeature;
	bool m_useCache;
	bool m_showGizmo;
	bool m_gizmoLocal;
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

static int sampleCollideSpheres =
	RegisterSample( "Manifold", "Sphere vs Sphere", SphereAndSphere::Create );

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
		b3CollideCapsuleAndSphere( &m_manifold, m_pointCapacity, &m_capsule, &m_sphere,
								   transformBtoA );
	}

	b3Sphere m_sphere;
	b3Capsule m_capsule;
};

static int sampleSphereAndCapsule =
	RegisterSample( "Manifold", "Capsule vs Sphere", CapsuleAndSphere::Create );

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
		b3CollideHullAndSphere( &m_manifold, m_pointCapacity, &m_hull.base, &m_sphere,
								transformBtoA, &m_simplexCache );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new HullAndSphere( sampleContext );
	}

	b3Sphere m_sphere;
	b3BoxHull m_hull;
};

static int sampleSphereAndHull =
	RegisterSample( "Manifold", "Hull vs Sphere", HullAndSphere::Create );

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

static int sampleSphereAndTriangle =
	RegisterSample( "Manifold", "Triangle vs Sphere", TriangleAndSphere::Create );

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

static int sampleCapsuleAndCapsule =
	RegisterSample( "Manifold", "Capsule vs Capsule", CapsuleAndCapsule::Create );

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
		//	{ 0.0799999982, -0.0151330000, -0.0918010026 }, { -0.0799999982, -0.0151330000,
		//-0.0918010026 }, 0.100000001 };
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
		b3CollideHullAndCapsule( &m_manifold, m_pointCapacity, &m_hull.base, &m_capsule,
								 transformBtoA, &m_simplexCache );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new CapsuleAndHull( sampleContext );
	}

	b3BoxHull m_hull;
	b3Capsule m_capsule;
};

static int sampleCapsuleAndHull =
	RegisterSample( "Manifold", "Capsule vs Hull", CapsuleAndHull::Create );

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

		b3CollideTriangleAndCapsule( &m_manifold, m_pointCapacity, localTriangle, &m_capsule,
									 &m_simplexCache );
	}

	b3Capsule m_capsule;
};

static int sampleCapsuleAndTriangle =
	RegisterSample( "Manifold", "Triangle vs Capsule", TriangleAndCapsule::Create );

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

		// m_boxA = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, { { 0.0f, -0.5f, 0.0f },
		// b3Quat_identity } );

		b3Transform transform = { { 1.0f, 0.5f, 0.0f }, b3Quat_identity };
		m_boxA = b3MakeTransformedBoxHull( 0.5f, 1.0f, 1.0f, transform );

		// m_hull = CreateConvex( 0.6f, 0.0f, 0.95f, 1.0f, context->arena );
		m_hullA = &m_boxA.base;

		m_boxB = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		// m_hull = CreateConvex( 0.6f, 0.0f, 0.95f, 1.0f, context->arena );
		m_hullB = &m_boxB.base;

		/*
-		xfA	{p=[ -10.0000000, 0.00000000, -10.0000000 ] q=[ [ 0.00000000, 0.00000000, 0.00000000
], 1.00000000 ] }	b3Transform +		p	[ -10.0000000, 0.00000000, -10.0000000 ]	b3Vec3 +
q	[ [ 0.00000000, 0.00000000, 0.00000000 ], 1.00000000 ]	b3Quat -		xfB	{p=[
-10.0000000, 1.00000000, -10.0000000 ] q=[ [ 0.00000000, 0.00000000, 0.00000000 ], 1.00000000 ] }
b3Transform +		p	[ -10.0000000, 1.00000000, -10.0000000 ]	b3Vec3 +		q	[ [
0.00000000, 0.00000000, 0.00000000 ], 1.00000000 ]	b3Quat

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
		b3CollideHulls( &m_manifold, m_pointCapacity, m_hullA, m_hullB, transformBtoA,
						&m_satCache );

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

// Shows the candidate axes culled in the separating axis test. No axis n can separate the hulls by
// more than dot(n, centerB - centerA) - innerRadiusA - innerRadiusB, so any face whose bound is
// below the best separation found so far is skipped. An edge survives only if the plane separation
// of the other hull's inscribed sphere can beat the best face separation somewhere over the Gauss
// map arc of the edge. The support points of the best faces lie on the other hull, so they give a
// second, tighter arc test. This recomputes the culling decisions of b3ComputeSeparatingAxis from
// public hull data and must be kept in sync with it.
class HullCulling : public Manifold
{
public:
	enum FeatureState
	{
		e_unreached = 0,
		e_culled,
		e_tested,
	};

	enum EdgeBound
	{
		e_noBound = 0,
		e_sphereBound,
		e_supportBound,
		e_bothBounds,
	};

	// Mirrors B3_EDGE_PROBE_MIN_TESTS
	static constexpr int probeMinTests = 10;

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
		m_transformB = { { 0.4f, 3.6f, 0.2f },
						 b3MakeQuatFromAxisAngle( b3Normalize( { 1.0f, 0.0f, 1.0f } ), 0.3f ) };

		m_showSpheres = true;
		m_showCulled = true;
		m_showNormals = true;
		m_edgeBound = e_bothBounds;
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
			m_hullA =
				b3CreateCylinder( m_cylinderHeight, m_cylinderRadius, yOffset, m_cylinderSides );
			m_hullB =
				b3CreateCylinder( m_cylinderHeight, m_cylinderRadius, yOffset, m_cylinderSides );
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
		const char* edgeBounds[] = { "None", "Sphere", "Support", "Sphere + Support" };
		ImGuiStyle& style = ImGui::GetStyle();
		ImGui::SetNextItemWidth( ImGui::CalcTextSize( edgeBounds[3] ).x + ImGui::GetFrameHeight() +
								 2.0f * style.FramePadding.x );
		ImGui::Combo( "Edge bound", &m_edgeBound, edgeBounds, IM_ARRAYSIZE( edgeBounds ) );

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
		m_probeVertexA = 0;
		m_probeVertexB = 0;
		m_keptEdgeCountA = 0;
		m_keptEdgeCountB = 0;
		m_sphereKeptEdgeCountA = 0;
		m_sphereKeptEdgeCountB = 0;
		m_separationA = -FLT_MAX;
		m_separationB = -FLT_MAX;
		m_gap = 0.0f;
		m_probeApplied = false;
		m_separatedFeature = b3_invalidAxis;
	}

	// Face of A against the vertices of B, in frame A. The deepest vertex of B is the support
	// point.
	static float FaceSeparationA( const b3HullData* hullA, const b3HullData* hullB,
								  b3Transform transformBtoA, int faceIndex,
								  int* vertexIndex = nullptr )
	{
		b3Plane plane = b3GetHullPlanes( hullA )[faceIndex];
		const b3Vec3* points = b3GetHullPoints( hullB );
		float separation = FLT_MAX;
		int bestIndex = 0;
		for ( int i = 0; i < hullB->vertexCount; ++i )
		{
			b3Vec3 point = b3TransformPoint( transformBtoA, points[i] );
			float s = b3Dot( plane.normal, point ) - plane.offset;
			if ( s < separation )
			{
				separation = s;
				bestIndex = i;
			}
		}

		if ( vertexIndex != nullptr )
		{
			*vertexIndex = bestIndex;
		}

		return separation;
	}

	// Face of B against the vertices of A, in frame B. The deepest vertex of A is the support
	// point.
	static float FaceSeparationB( const b3HullData* hullA, const b3HullData* hullB,
								  b3Transform transformBtoA, int faceIndex,
								  int* vertexIndex = nullptr )
	{
		b3Plane plane = b3GetHullPlanes( hullB )[faceIndex];
		const b3Vec3* points = b3GetHullPoints( hullA );
		float separation = FLT_MAX;
		int bestIndex = 0;
		for ( int i = 0; i < hullA->vertexCount; ++i )
		{
			b3Vec3 point = b3InvTransformPoint( transformBtoA, points[i] );
			float s = b3Dot( plane.normal, point ) - plane.offset;
			if ( s < separation )
			{
				separation = s;
				bestIndex = i;
			}
		}

		if ( vertexIndex != nullptr )
		{
			*vertexIndex = bestIndex;
		}

		return separation;
	}

	// Mirrors b3TestEdgeCandidateSorted
	static bool ArcCanReachSorted( float a, float b, float c, float bound )
	{
		const float parallelTolerance = 1.0e-4f;
		float hi = b3MaxFloat( a, b );
		float lo = b3MinFloat( a, b );
		float u = lo - c * hi;
		float s = 1.0f - c * c;
		bool exterior = hi >= bound;
		bool interior =
			u >= 0.0f && ( u * u >= ( bound - hi ) * ( bound + hi ) * s || s < parallelTolerance );
		return exterior || interior;
	}

	// Arc test of every surviving edge against the plane separations of a point on the other hull.
	// Mirrors the gather loops and b3FilterEdgeCandidates, edges that fail are culled.
	static int CullEdges( const b3HullData* hull, b3Vec3 point, float bound, uint8_t* states )
	{
		const b3HullHalfEdge* edges = b3GetHullEdges( hull );
		const b3Plane* planes = b3GetHullPlanes( hull );
		const float* cosines = b3GetHullEdgeCosines( hull );
		int keptCount = 0;
		for ( int i = 0; i < hull->edgeCount; i += 2 )
		{
			if ( states[i / 2] == e_culled )
			{
				continue;
			}

			b3Plane plane1 = planes[edges[i].face];
			b3Plane plane2 = planes[edges[i + 1].face];
			float d1 = b3Dot( plane1.normal, point ) - plane1.offset;
			float d2 = b3Dot( plane2.normal, point ) - plane2.offset;
			bool kept = ArcCanReachSorted( d1, d2, cosines[i >> 1], bound );
			states[i / 2] = kept ? e_tested : e_culled;
			keptCount += kept ? 1 : 0;
		}
		return keptCount;
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

		b3Vec3 deltaCenter =
			b3Sub( b3TransformPoint( transformBtoA, hullB->center ), hullA->center );
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

		// The seed support vertex stands in for the probe if no face improves on it
		float seedSeparationA =
			FaceSeparationA( hullA, hullB, transformBtoA, m_seedA, &m_probeVertexB );
		float floorA = b3MinFloat( seedSeparationA, speculativeDistance );
		for ( int i = 0; i < hullA->faceCount; ++i )
		{
			if ( dotA[i] - radiusBound < b3MaxFloat( floorA, m_separationA ) )
			{
				m_faceStateA[i] = e_culled;
				continue;
			}

			m_faceStateA[i] = e_tested;
			int vertexIndex;
			float separation = FaceSeparationA( hullA, hullB, transformBtoA, i, &vertexIndex );
			if ( separation > m_separationA )
			{
				m_separationA = separation;
				m_probeVertexB = vertexIndex;
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

		float seedSeparationB =
			FaceSeparationB( hullA, hullB, transformBtoA, m_seedB, &m_probeVertexA );
		float floorB = b3MaxFloat( seedSeparationB, m_separationA );
		floorB = b3MinFloat( floorB, speculativeDistance );
		for ( int i = 0; i < hullB->faceCount; ++i )
		{
			if ( dotB[i] - radiusBound < b3MaxFloat( floorB, m_separationB ) )
			{
				m_faceStateB[i] = e_culled;
				continue;
			}

			m_faceStateB[i] = e_tested;
			int vertexIndex;
			float separation = FaceSeparationB( hullA, hullB, transformBtoA, i, &vertexIndex );
			if ( separation > m_separationB )
			{
				m_separationB = separation;
				m_probeVertexA = vertexIndex;
				if ( separation > speculativeDistance )
				{
					m_separatedFeature = b3_faceAxisB;
					return;
				}
			}
		}

		bool useSphere = m_edgeBound == e_sphereBound || m_edgeBound == e_bothBounds;
		bool useSupport = m_edgeBound == e_supportBound || m_edgeBound == e_bothBounds;
		float maxFaceSeparation = b3MaxFloat( m_separationA, m_separationB );

		if ( useSphere )
		{
			// Inscribed sphere bound, the arc inputs are plane separations of the other hull's
			// center
			float boundSlack = radius - radiusBound;
			float thresholdA = maxFaceSeparation + hullB->innerRadius - boundSlack;
			float thresholdB = maxFaceSeparation + hullA->innerRadius - boundSlack;
			b3Vec3 centerBinA = b3TransformPoint( transformBtoA, hullB->center );
			b3Vec3 centerAinB = b3InvTransformPoint( transformBtoA, hullA->center );
			m_keptEdgeCountA = CullEdges( hullA, centerBinA, thresholdA, m_edgeStateA );
			m_keptEdgeCountB = CullEdges( hullB, centerAinB, thresholdB, m_edgeStateB );
		}
		else
		{
			m_keptEdgeCountA = hullA->edgeCount / 2;
			m_keptEdgeCountB = hullB->edgeCount / 2;
			memset( m_edgeStateA, e_tested, m_keptEdgeCountA );
			memset( m_edgeStateB, e_tested, m_keptEdgeCountB );
		}

		m_sphereKeptEdgeCountA = m_keptEdgeCountA;
		m_sphereKeptEdgeCountB = m_keptEdgeCountB;

		// The probe is gated on the edge pair work in SIMD groups of A edges
		int testCount = m_keptEdgeCountB * ( ( m_keptEdgeCountA + 3 ) >> 2 );
		m_probeApplied = useSupport && testCount >= probeMinTests;
		if ( m_probeApplied )
		{
			float probeBound =
				maxFaceSeparation -
				( 0.1f * B3_LINEAR_SLOP + 0.001f * b3AbsFloat( centerDistance + radius ) );
			b3Vec3 probeB =
				b3TransformPoint( transformBtoA, b3GetHullPoints( hullB )[m_probeVertexB] );
			b3Vec3 probeA =
				b3InvTransformPoint( transformBtoA, b3GetHullPoints( hullA )[m_probeVertexA] );
			m_keptEdgeCountA = CullEdges( hullA, probeB, probeBound, m_edgeStateA );
			m_keptEdgeCountB = CullEdges( hullB, probeA, probeBound, m_edgeStateB );
		}
	}

	void Step() override
	{
		// Clear the cache so every step runs the full separating axis test
		m_satCache = {};
		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		b3CollideHulls( &m_manifold, m_pointCapacity, m_hullA, m_hullB, transformBtoA,
						&m_satCache );
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

	void DrawEdges( b3WorldTransform transform, const b3HullData* hull, const uint8_t* states,
					Vec4 keptColor )
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
				DrawLineEx( p1, p2, keptColor, 4.0f, OVERLAY_THICKNESS_PIXELS,
							OVERLAY_OCCLUSION_DIM );
			}
			else if ( state == e_culled && m_showCulled )
			{
				DrawLine( p1, p2, MakeColor( b3_colorSlateGray ) );
			}
			else if ( state == e_unreached )
			{
				DrawLineEx( p1, p2, MakeColor( b3_colorDimGray ), 1.0f, OVERLAY_THICKNESS_PIXELS,
							OVERLAY_OCCLUSION_DASHED );
			}
		}
	}

	void DrawFaceNormals( b3WorldTransform transform, const b3HullData* hull, const uint8_t* states,
						  int seed, Vec4 testedColor )
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
			b3Pos p2 =
				b3TransformWorldPoint( transform, b3MulAdd( centroid, length, planes[i].normal ) );

			if ( i == seed )
			{
				DrawArrowEx( p1, p2, MakeColor( b3_colorGold ), 3.0f, OVERLAY_THICKNESS_PIXELS,
							 OVERLAY_OCCLUSION_DIM, 0.25f );
			}
			else if ( state == e_tested )
			{
				DrawArrowEx( p1, p2, testedColor, 2.0f, OVERLAY_THICKNESS_PIXELS,
							 OVERLAY_OCCLUSION_DIM, 0.25f );
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
			DrawFaceNormals( m_transformA, hullA, m_faceStateA, m_seedA,
							 MakeColor( b3_colorOrange ) );
			DrawFaceNormals( m_transformB, hullB, m_faceStateB, m_seedB,
							 MakeColor( b3_colorDeepSkyBlue ) );
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

		if ( m_probeApplied )
		{
			b3Pos probeA =
				b3TransformWorldPoint( m_transformA, b3GetHullPoints( hullA )[m_probeVertexA] );
			b3Pos probeB =
				b3TransformWorldPoint( m_transformB, b3GetHullPoints( hullB )[m_probeVertexB] );
			DrawPoint( probeA, 12.0f, MakeColor( b3_colorMagenta ) );
			DrawPoint( probeB, 12.0f, MakeColor( b3_colorMagenta ) );
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

		DrawTextLine( "vertices %d, faces %d, edges %d", hullA->vertexCount, faceCountA,
					  edgeCountA );
		DrawTextLine( "|d| - rA - rB = %.3f", m_gap );
		DrawTextLine( "faces A: tested %d, culled %d of %d, best separation %.4f", testedA, culledA,
					  faceCountA, m_separationA );

		if ( m_separatedFeature == b3_faceAxisA )
		{
			DrawTextLine( "separated by face A, B faces and edges not reached" );
		}
		else
		{
			DrawTextLine( "faces B: tested %d, culled %d of %d, best separation %.4f", testedB,
						  culledB, faceCountB, m_separationB );
		}

		if ( m_separatedFeature == b3_faceAxisB )
		{
			DrawTextLine( "separated by face B, edges not reached" );
		}

		DrawTextLine( "support queries: %d of %d (including %d seed)",
					  testedA + testedB + seedCount, faceCountA + faceCountB, seedCount );

		if ( edgesReached )
		{
			int pairCount = m_keptEdgeCountA * m_keptEdgeCountB;
			int totalPairCount = edgeCountA * edgeCountB;
			DrawTextLine( "edges kept: A %d of %d, B %d of %d", m_keptEdgeCountA, edgeCountA,
						  m_keptEdgeCountB, edgeCountB );
			DrawTextLine( "edge pairs: %d of %d (%.1f%%)", pairCount, totalPairCount,
						  100.0f * pairCount / totalPairCount );

			if ( m_probeApplied && m_edgeBound == e_bothBounds )
			{
				int spherePairCount = m_sphereKeptEdgeCountA * m_sphereKeptEdgeCountB;
				DrawTextLine( "sphere bound: A %d, B %d, edge pairs %d (%.1f%%)",
							  m_sphereKeptEdgeCountA, m_sphereKeptEdgeCountB, spherePairCount,
							  100.0f * spherePairCount / totalPairCount );
			}

			if ( m_probeApplied )
			{
				DrawTextLine( "magenta points are the support probes" );
			}
			else if ( m_edgeBound == e_supportBound || m_edgeBound == e_bothBounds )
			{
				DrawTextLine( "probe skipped, fewer than %d edge tests", probeMinTests );
			}
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

	// Support vertex on each hull for the best face of the other, it probes the edges of the other
	// hull
	int m_probeVertexA;
	int m_probeVertexB;

	int m_keptEdgeCountA;
	int m_keptEdgeCountB;
	int m_sphereKeptEdgeCountA;
	int m_sphereKeptEdgeCountB;
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
	int m_edgeBound;
	bool m_probeApplied;
};

static int sampleHullCulling = RegisterSample( "Manifold", "Hull Culling", HullCulling::Create );

// Shows that an edge pair can win the separating axis test even when the direction between the hull
// centers peaks outside the Gauss map arc (wedge) of one of its edges. The inscribed sphere bound
// of such an edge peaks at a face normal at the end of its arc, yet the edge axis beats the real
// separation of those faces. So outside wedge edges can only be culled by their sphere bound, not
// by the face results. Everything here is brute force over all faces and all edge pairs,
// independent of the pruning in the SAT.
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
			b3Vec3 points[4] = { { 1.0f, 1.0f, 1.0f },
								 { 1.0f, -1.0f, -1.0f },
								 { -1.0f, 1.0f, -1.0f },
								 { -1.0f, -1.0f, 1.0f } };
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
			v = { 2.0f * RandomFloat() - 1.0f, 2.0f * RandomFloat() - 1.0f,
				  2.0f * RandomFloat() - 1.0f };
			lengthSquared = b3Dot( v, v );
		}
		while ( lengthSquared > 1.0f || lengthSquared < 0.01f );

		return b3Normalize( v );
	}

	// Mirrors the Gauss map test and edge axis of the scalar path in b3ComputeSeparatingAxis,
	// without pruning
	static EdgeAxis FindBestEdgeAxis( const b3HullData* hullA, const b3HullData* hullB,
									  b3Transform transformBtoA )
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
			b3Vec3 D =
				b3Neg( b3RotateVector( transformBtoA.q, planesB[edgesB[j + 1].face].normal ) );
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

	float MaxSeparation( b3Transform transformBtoA, float* faceSeparationA, float* faceSeparationB,
						 EdgeAxis* edgeAxis ) const
	{
		float separationA = -FLT_MAX;
		for ( int i = 0; i < m_hullA->faceCount; ++i )
		{
			separationA = b3MaxFloat(
				separationA, HullCulling::FaceSeparationA( m_hullA, m_hullB, transformBtoA, i ) );
		}

		float separationB = -FLT_MAX;
		for ( int i = 0; i < m_hullB->faceCount; ++i )
		{
			separationB = b3MaxFloat(
				separationB, HullCulling::FaceSeparationB( m_hullA, m_hullB, transformBtoA, i ) );
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
			float upper = 1.1f * ( b3Length( b3AABB_Extents( m_hullA->aabb ) ) +
								   b3Length( b3AABB_Extents( m_hullB->aabb ) ) );
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
			m_searchFound =
				m_edgeWins && outside >= 10.0f && margin >= 0.02f * b3GetLengthUnitsPerMeter();
		}
	}

	void Compute()
	{
		b3Transform transformBtoA = b3InvMulWorldTransforms( m_transformA, m_transformB );
		MaxSeparation( transformBtoA, &m_faceSeparationA, &m_faceSeparationB, &m_edge );
		m_edgeWins = m_edge.edgeA != B3_NULL_INDEX &&
					 m_edge.separation > b3MaxFloat( m_faceSeparationA, m_faceSeparationB );

		m_centerOffset =
			b3Sub( b3TransformPoint( transformBtoA, m_hullB->center ), m_hullA->center );
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
		m_arcB = MakeArc( C, D, m_centerOffset,
						  HullCulling::FaceSeparationB( m_hullA, m_hullB, transformBtoA, faceB1 ),
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

		DrawGizmoControls( &m_showGizmo, &m_gizmoLocal );

		return true;
	}

	void Step() override
	{
		Compute();
	}

	b3Pos GaussPoint( b3Vec3 directionInA ) const
	{
		return m_gaussCenter +
			   b3MulSV( m_gaussRadius, b3RotateVector( m_transformA.q, directionInA ) );
	}

	void DrawGreatArc( b3Vec3 u, b3Vec3 v, Vec4 color, float thickness,
					   OverlayOcclusionMode mode ) const
	{
		constexpr int segmentCount = 32;
		b3Pos previous = GaussPoint( u );
		for ( int i = 1; i <= segmentCount; ++i )
		{
			float s = (float)i / segmentCount;
			b3Pos point =
				GaussPoint( b3Normalize( b3Add( b3MulSV( 1.0f - s, u ), b3MulSV( s, v ) ) ) );
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
		b3CosSin cs = b3ComputeCosSin( 2.0f * B3_PI / segmentCount );
		float x1 = 1.0f, y1 = 0.0f;
		b3Pos previous = GaussPoint( u );
		for ( int i = 0; i < segmentCount; ++i )
		{
			float x2 = cs.cosine * x1 - cs.sine * y1;
			float y2 = cs.sine * x1 + cs.cosine * y1;
			b3Pos point = GaussPoint( b3Add( b3MulSV( x2, u ), b3MulSV( y2, w ) ) );
			DrawLineEx( previous, point, color, 1.0f, OVERLAY_THICKNESS_PIXELS,
						OVERLAY_OCCLUSION_DASHED );
			x1 = x2;
			y1 = y2;
			previous = point;
		}
	}

	void DrawEdge( b3WorldTransform transform, const b3HullData* hull, int edgeIndex,
				   Vec4 color ) const
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
			DrawTextLine( "%s edge: d peak OUTSIDE wedge by %.1f degrees", name,
						  arc.outsideDegrees );
		}
		DrawTextLine( "   sphere bound  u %.3f  v %.3f  n %.3f", arc.a - m_radius, arc.b - m_radius,
					  edgeBound );
		DrawTextLine( "   separation    u %.4f  v %.4f  n %.4f", arc.separationU, arc.separationV,
					  m_edge.separation );
	}

	void Render() override
	{
		DrawHull( m_transformA, m_hullA, MakeColor( b3_colorSlateGray ) );
		DrawHull( m_transformB, m_hullB, MakeColor( b3_colorSlateGray ) );

		b3Pos centerA = b3TransformWorldPoint( m_transformA, m_hullA->center );
		b3Pos centerB = b3TransformWorldPoint( m_transformB, m_hullB->center );
		DrawLine( centerA, centerB, MakeColor( b3_colorWhite ) );

		DrawTextLine( "search: %s after %d poses", m_searchFound ? "found" : "not found",
					  m_searchCount );
		DrawTextLine( "best separation: face A %.4f, face B %.4f, edge %.4f -> %s",
					  m_faceSeparationA, m_faceSeparationB, m_edge.separation,
					  m_edgeWins ? "EDGE WINS" : "face wins" );

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
			b3Vec3 mid = b3Lerp( points[edges[m_edge.edgeA].origin],
								 points[edges[m_edge.edgeA + 1].origin], 0.5f );
			b3Pos p = b3TransformWorldPoint( m_transformA, mid );
			b3Vec3 n = b3RotateVector( m_transformA.q, m_edge.normal );
			DrawArrowEx( p, p + b3MulSV( 1.0f, n ), colorN, 3.0f, OVERLAY_THICKNESS_PIXELS,
						 OVERLAY_OCCLUSION_DIM, 0.2f );
		}

		// Gauss map: both arcs cross at the edge axis n. The magenta dots are the peaks of dot(n,
		// d) on each great circle. The peak of an arc is only reachable when it lies on the arc.
		b3Sphere unitSphere = { b3Vec3_zero, m_gaussRadius };
		DrawWireSphere( { m_gaussCenter, b3Quat_identity }, &unitSphere, 24,
						MakeColor( b3_colorDimGray ) );

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

static int sampleEdgeAxisWedge =
	RegisterSample( "Manifold", "Edge Axis Wedge", EdgeAxisWedge::Create );

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

		b3CollideTriangleAndHull( &m_manifold, m_pointCapacity, localTriangle[0], localTriangle[1],
								  localTriangle[2], m_flags, m_hull, &m_satCache, true );
	}

	int m_flags;
	b3BoxHull m_boxHull;
	b3HullData* m_cylinder;
	b3HullData* m_hull;
};

static int sampleHullAndTriangle =
	RegisterSample( "Manifold", "Triangle vs Hull", TriangleAndHull::Create );

// Minimal meshes that each show one kind of ghost collision and how ghost handling treats it.
// Contacts are computed fresh every step with the same triangle pipeline the world uses, so a
// breakpoint in the mesh collide function stops on this scene. Derive per shape type.
struct GhostMesh
{
	std::vector<b3Vec3> vertices;
	std::vector<int32_t> indices;
};

typedef void GhostMeshBuildFcn( std::vector<GhostMesh>& meshes );

struct GhostScene
{
	const char* name;
	const char* ghost;
	const char* handling;
	GhostMeshBuildFcn* build;

	// Interpreted by the derived sample, for example box half extents
	b3Vec3 extents;

	b3Vec3 position;

	// Degrees, applied as yaw * pitch * roll
	b3Vec3 angles;

	bool speculative;
};

// Most surfaces in these scenes face up, so wind each triangle upward unless told otherwise
static void AddGhostTriangle( GhostMesh& mesh, int a, int b, int c, b3Vec3 facing = b3Vec3_axisY )
{
	b3Vec3 va = mesh.vertices[a];
	b3Vec3 normal = b3Cross( b3Sub( mesh.vertices[b], va ), b3Sub( mesh.vertices[c], va ) );
	if ( b3Dot( normal, facing ) < 0.0f )
	{
		int temp = b;
		b = c;
		c = temp;
	}

	mesh.indices.push_back( a );
	mesh.indices.push_back( b );
	mesh.indices.push_back( c );
}

class GhostManifold : public Sample
{
public:
	static constexpr int m_maxScenes = 16;
	static constexpr int m_manifoldCapacity = 32;
	static constexpr int m_triangleCapacity = 64;
	static constexpr int m_trianglePointCapacity = 64;

	enum EdgeType
	{
		e_boundaryEdge,
		e_flatEdge,
		e_convexEdge,
		e_concaveEdge,
	};

	struct MeshView
	{
		GhostMesh source;
		std::vector<EdgeType> edgeTypes;
		b3MeshData* data;
	};

	struct TriangleResult
	{
		b3LocalManifold manifold;
		b3LocalManifoldPoint points[m_trianglePointCapacity];
	};

	explicit GhostManifold( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 30.0f, 25.0f, 4.0f, { 0.0f, 0.3f, 0.0f } );
		}

		m_scenes = nullptr;
		m_sceneCount = 0;
		m_sceneIndex = 0;
		m_handleGhosts = true;
		m_speculative = true;
		m_drawTriangleManifolds = false;
		m_cullDistance = 0.5f;
		m_showGizmo = false;
		m_gizmoLocal = false;
		m_transform = b3WorldTransform_identity;
		m_manifoldCount = 0;
		m_triangleManifoldCount = 0;
	}

	~GhostManifold() override
	{
		DestroyMeshes();
	}

	// Derived samples call this from their constructor
	void SetScenes( const GhostScene* scenes, int count )
	{
		m_scenes = scenes;
		m_sceneCount = count;
		LoadScene( 0 );
	}

	virtual void CreateShape( const GhostScene& scene ) = 0;

	virtual int CollideMesh( b3Manifold* manifolds, int capacity, const b3Mesh* mesh,
							 b3Transform transformBtoA, bool speculative, bool handleGhosts ) = 0;

	// Triangle in the shape frame, before any ghost handling
	virtual void CollideTriangle( b3LocalManifold* manifold, int capacity, const b3Vec3* triangle,
								  bool speculative ) = 0;

	virtual void DrawShape( b3WorldTransform transform ) = 0;

	virtual b3AABB ComputeShapeBounds( b3Transform transform ) = 0;

	void DestroyMeshes()
	{
		for ( MeshView& mesh : m_meshes )
		{
			b3DestroyMesh( mesh.data );
		}
		m_meshes.clear();
	}

	void LoadScene( int index )
	{
		DestroyMeshes();

		m_sceneIndex = index;
		const GhostScene& scene = m_scenes[index];

		std::vector<GhostMesh> sources;
		scene.build( sources );

		for ( GhostMesh& source : sources )
		{
			b3MeshDef meshDef = {};
			meshDef.vertices = source.vertices.data();
			meshDef.indices = source.indices.data();
			meshDef.vertexCount = (int)source.vertices.size();
			meshDef.triangleCount = (int)source.indices.size() / 3;
			meshDef.weldVertices = true;
			meshDef.weldTolerance = 0.001f;
			meshDef.identifyEdges = true;

			MeshView mesh;
			mesh.source = source;
			mesh.data = b3CreateMesh( &meshDef, nullptr, 0 );
			ClassifyEdges( mesh );
			m_meshes.push_back( mesh );
		}

		ResetPose();
		m_speculative = scene.speculative;
		CreateShape( scene );
	}

	void ResetPose()
	{
		const GhostScene& scene = m_scenes[m_sceneIndex];
		float toRadians = B3_PI / 180.0f;
		b3Quat qx = b3MakeQuatFromAxisAngle( b3Vec3_axisX, scene.angles.x * toRadians );
		b3Quat qy = b3MakeQuatFromAxisAngle( b3Vec3_axisY, scene.angles.y * toRadians );
		b3Quat qz = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, scene.angles.z * toRadians );
		m_transform.q = b3MulQuat( qy, b3MulQuat( qx, qz ) );
		m_transform.p = b3OffsetPos( b3Pos_zero, scene.position );
	}

	// Matches edges by vertex index, so a t-junction or a seam between meshes is a boundary edge,
	// as in the mesh builder
	static void ClassifyEdges( MeshView& mesh )
	{
		const std::vector<b3Vec3>& v = mesh.source.vertices;
		const std::vector<int32_t>& idx = mesh.source.indices;
		int triangleCount = (int)idx.size() / 3;
		mesh.edgeTypes.assign( 3 * triangleCount, e_boundaryEdge );

		for ( int i = 0; i < triangleCount; ++i )
		{
			const int32_t* t = idx.data() + 3 * i;
			b3Vec3 normal =
				b3Normalize( b3Cross( b3Sub( v[t[1]], v[t[0]] ), b3Sub( v[t[2]], v[t[0]] ) ) );

			for ( int k = 0; k < 3; ++k )
			{
				int a = t[k];
				int b = t[k < 2 ? k + 1 : 0];

				for ( int j = 0; j < triangleCount; ++j )
				{
					const int32_t* u = idx.data() + 3 * j;
					for ( int m = 0; m < 3 && j != i; ++m )
					{
						if ( u[m] != b || u[m < 2 ? m + 1 : 0] != a )
						{
							continue;
						}

						int opposite = u[m > 0 ? m - 1 : 2];
						float distance = b3Dot( normal, b3Sub( v[opposite], v[a] ) );
						float tolerance = 0.001f;
						EdgeType type = distance < -tolerance
											? e_convexEdge
											: ( distance > tolerance ? e_concaveEdge : e_flatEdge );
						mesh.edgeTypes[3 * i + k] = type;
					}
				}
			}
		}
	}

	static bool CollectTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
	{
		GhostManifold* self = (GhostManifold*)context;
		if ( self->m_triangleManifoldCount == m_triangleCapacity )
		{
			return false;
		}

		self->m_queryTriangles[self->m_triangleManifoldCount] = { a, b, c };
		self->m_queryIndices[self->m_triangleManifoldCount] = triangleIndex;
		self->m_triangleManifoldCount += 1;
		return true;
	}

	void Step() override
	{
		// Mesh frame is the world frame
		b3Transform transformBtoA =
			b3InvMulWorldTransforms( b3WorldTransform_identity, m_transform );

		m_manifoldCount = 0;
		for ( int i = 0; i < (int)m_meshes.size(); ++i )
		{
			b3Mesh mesh = { m_meshes[i].data, b3Vec3_one };
			int count =
				CollideMesh( m_manifolds + m_manifoldCount, m_manifoldCapacity - m_manifoldCount,
							 &mesh, transformBtoA, m_speculative, m_handleGhosts );
			for ( int j = 0; j < count; ++j )
			{
				m_manifoldMesh[m_manifoldCount + j] = i;
			}
			m_manifoldCount += count;
		}

		m_triangleManifoldCount = 0;
		if ( m_drawTriangleManifolds == false )
		{
			return;
		}

		b3AABB bounds = ComputeShapeBounds( transformBtoA );
		float margin = B3_SPECULATIVE_DISTANCE;
		bounds.lowerBound = b3Sub( bounds.lowerBound, { margin, margin, margin } );
		bounds.upperBound = b3Add( bounds.upperBound, { margin, margin, margin } );

		b3Transform transformAtoB =
			b3InvMulWorldTransforms( m_transform, b3WorldTransform_identity );
		for ( MeshView& drawMesh : m_meshes )
		{
			b3Mesh mesh = { drawMesh.data, b3Vec3_one };
			int base = m_triangleManifoldCount;
			b3QueryMesh( &mesh, bounds, CollectTriangle, this );

			for ( int i = base; i < m_triangleManifoldCount; ++i )
			{
				TriangleQuery& query = m_queryTriangles[i];
				b3Vec3 triangle[3] = {
					b3TransformPoint( transformAtoB, query.a ),
					b3TransformPoint( transformAtoB, query.b ),
					b3TransformPoint( transformAtoB, query.c ),
				};

				TriangleResult& tm = m_triangleManifolds[i];
				tm.manifold = {};
				tm.manifold.points = tm.points;
				CollideTriangle( &tm.manifold, m_trianglePointCapacity, triangle, m_speculative );
				tm.manifold.triangleIndex = m_queryIndices[i];
			}
		}
	}

	void Render() override
	{
		const GhostScene& scene = m_scenes[m_sceneIndex];
		DrawTextLine( "%s", scene.name );
		DrawTextLine( "ghost: %s", scene.ghost );
		DrawTextLine( "handling: %s", scene.handling );
		DrawTextLine( "manifolds: %d", m_manifoldCount );

		DrawGroundGrid( 80 );

		Vec4 edgeColors[] = {
			MakeColor( b3_colorRed ),
			MakeColor( b3_colorGray ),
			MakeColor( b3_colorOrange ),
			MakeColor( b3_colorDodgerBlue ),
		};

		Vec4 faceColors[] = { MakeColor( b3_colorDarkSlateGray ),
							  MakeColor( b3_colorDarkOliveGreen ) };

		for ( int i = 0; i < (int)m_meshes.size(); ++i )
		{
			const MeshView& mesh = m_meshes[i];
			const std::vector<b3Vec3>& v = mesh.source.vertices;
			const std::vector<int32_t>& idx = mesh.source.indices;
			int triangleCount = (int)idx.size() / 3;

			for ( int t = 0; t < triangleCount; ++t )
			{
				b3Vec3 a = v[idx[3 * t]], b = v[idx[3 * t + 1]], c = v[idx[3 * t + 2]];
				DrawTriangle( b3WorldTransform_identity, a, b, c, faceColors[i & 1] );

				b3Vec3 corners[3] = { a, b, c };
				for ( int k = 0; k < 3; ++k )
				{
					b3Pos p1 = b3OffsetPos( b3Pos_zero, corners[k] );
					b3Pos p2 = b3OffsetPos( b3Pos_zero, corners[k < 2 ? k + 1 : 0] );
					DrawLine( p1, p2, edgeColors[mesh.edgeTypes[3 * t + k]] );
				}
			}
		}

		DrawShape( m_transform );

		float length = 0.3f * b3GetLengthUnitsPerMeter();
		for ( int i = 0; i < m_manifoldCount; ++i )
		{
			b3Vec3 offset = {0.0f, 0.01f * i, 0.0f};
			const b3Manifold& manifold = m_manifolds[i];
			for ( int j = 0; j < manifold.pointCount; ++j )
			{
				const b3ManifoldPoint& mp = manifold.points[j];
				if ( mp.separation > 0.01f * m_cullDistance )
				{
					continue;
				}
				b3Pos point = b3OffsetPos( b3Pos_zero, mp.anchorA );
				point = b3OffsetPos( point, offset );
				Vec4 color =
					mp.separation > 0.0f ? MakeColor( b3_colorWhite ) : MakeColor( b3_colorYellow );
				DrawArrow( point, point + length * manifold.normal, color );
				DrawPoint( point, 8.0f, color );
				DrawString3D( point, MakeColor( b3_colorWhite ), "  %.1f mm",
							  1000.0f * mp.separation );
			}

			DrawTextLine( "  mesh %d normal (%.3f %.3f %.3f) points %d", m_manifoldMesh[i],
						  manifold.normal.x, manifold.normal.y, manifold.normal.z,
						  manifold.pointCount );
		}

		if ( m_drawTriangleManifolds )
		{
			const char* featureNames[] = { "none",	"tri face", "hull face", "edge1",  "edge2",
										   "edge3", "vertex1",	"vertex2",	 "vertex3" };
			b3Matrix3 rotation = b3MakeMatrixFromQuat( m_transform.q );

			for ( int i = 0; i < m_triangleManifoldCount; ++i )
			{
				const b3LocalManifold& manifold = m_triangleManifolds[i].manifold;
				if ( manifold.pointCount == 0 )
				{
					continue;
				}

				b3Vec3 normal = b3MulMV( rotation, manifold.normal );
				for ( int j = 0; j < manifold.pointCount; ++j )
				{
					b3Pos point = b3TransformWorldPoint( m_transform, manifold.points[j].point );
					DrawArrow( point, point + 0.5f * length * normal,
							   MakeColor( b3_colorOrange ) );
				}

				b3Pos point = b3TransformWorldPoint( m_transform, manifold.points[0].point );
				DrawString3D( point + 0.5f * length * normal, MakeColor( b3_colorOrange ),
							  " t%d %s", manifold.triangleIndex, featureNames[manifold.feature] );
			}
		}

		DrawAxes( b3WorldTransform_identity, 0.25f );
	}

	bool HasSolverControls() const override
	{
		return false;
	}

	bool HasProfile() const override
	{
		return false;
	}

	bool DrawControls() override
	{
		const char* names[m_maxScenes];
		int count = b3MinInt( m_sceneCount, m_maxScenes );
		for ( int i = 0; i < count; ++i )
		{
			names[i] = m_scenes[i].name;
		}

		ImGui::PushItemWidth( 220.0f );
		int index = m_sceneIndex;
		if ( ImGui::Combo( "Scene", &index, names, count ) )
		{
			LoadScene( index );
		}
		ImGui::PopItemWidth();

		ImGui::Checkbox( "Ghost Handling", &m_handleGhosts );
		ImGui::Checkbox( "Speculative", &m_speculative );
		ImGui::Checkbox( "Triangle Manifolds", &m_drawTriangleManifolds );
		ImGui::SliderFloat( "Cull Distance", &m_cullDistance, 0.0f, 200.0f, "%0.1f cm" );
		if ( ImGui::Button( "Reset Pose" ) )
		{
			ResetPose();
		}

		DrawGizmoControls( &m_showGizmo, &m_gizmoLocal );

		ImGui::TextColored( ImVec4( 1.0f, 0.0f, 0.0f, 1.0f ), "boundary" );
		ImGui::SameLine();
		ImGui::TextColored( ImVec4( 0.5f, 0.5f, 0.5f, 1.0f ), "flat" );
		ImGui::SameLine();
		ImGui::TextColored( ImVec4( 1.0f, 0.65f, 0.0f, 1.0f ), "convex" );
		ImGui::SameLine();
		ImGui::TextColored( ImVec4( 0.12f, 0.56f, 1.0f, 1.0f ), "concave" );
		return true;
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		if ( key == KEY_G && action == ACTION_PRESS && modifiers == 0 )
		{
			m_showGizmo = !m_showGizmo;
		}
	}

	void DrawSampleWindows() override
	{
		if ( m_showGizmo )
		{
			DrawTransformGizmo( m_camera, &m_transform, m_gizmoLocal );
		}
	}

	// The gizmo moves the shape, so keep the picking in Sample off
	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
	}

	struct TriangleQuery
	{
		b3Vec3 a, b, c;
	};

	const GhostScene* m_scenes;
	int m_sceneCount;
	int m_sceneIndex;
	std::vector<MeshView> m_meshes;

	b3WorldTransform m_transform;
	bool m_handleGhosts;
	float m_cullDistance;
	bool m_speculative;
	bool m_drawTriangleManifolds;
	bool m_showGizmo;
	bool m_gizmoLocal;

	b3Manifold m_manifolds[m_manifoldCapacity] = {};
	int m_manifoldMesh[m_manifoldCapacity] = {};
	int m_manifoldCount;

	TriangleQuery m_queryTriangles[m_triangleCapacity] = {};
	int m_queryIndices[m_triangleCapacity] = {};
	TriangleResult m_triangleManifolds[m_triangleCapacity] = {};
	int m_triangleManifoldCount;
};

// Two triangles sharing the edge x = 0, optionally folded along it
static void BuildGhostFold( std::vector<GhostMesh>& meshes, float crestHeight, float sideHeight )
{
	float w = 2.0f, l = 2.0f;
	GhostMesh mesh;
	mesh.vertices = { { 0.0f, crestHeight, -w },
					  { 0.0f, crestHeight, w },
					  { -l, sideHeight, 0.0f },
					  { l, sideHeight, 0.0f } };
	AddGhostTriangle( mesh, 0, 1, 2 );
	AddGhostTriangle( mesh, 1, 0, 3 );
	meshes.push_back( mesh );
}

static void BuildGhostFlat( std::vector<GhostMesh>& meshes )
{
	BuildGhostFold( meshes, 0.0f, 0.0f );
}

// 15 degree slopes
static void BuildGhostCrest( std::vector<GhostMesh>& meshes )
{
	BuildGhostFold( meshes, 2.0f * tanf( 15.0f * B3_PI / 180.0f ), 0.0f );
}

static void BuildGhostValley( std::vector<GhostMesh>& meshes )
{
	BuildGhostFold( meshes, 0.0f, 2.0f * tanf( 15.0f * B3_PI / 180.0f ) );
}

// Rotated so the box corner arrives in the middle of a wedge
static void BuildGhostFan( std::vector<GhostMesh>& meshes )
{
	GhostMesh mesh;
	mesh.vertices.push_back( b3Vec3_zero );
	for ( int i = 0; i < 6; ++i )
	{
		float angle = ( i + 0.5f ) * B3_PI / 3.0f;
		mesh.vertices.push_back( { 2.0f * cosf( angle ), 0.0f, 2.0f * sinf( angle ) } );
	}

	for ( int i = 0; i < 6; ++i )
	{
		AddGhostTriangle( mesh, 0, 1 + i, 1 + ( i + 1 ) % 6 );
	}
	meshes.push_back( mesh );
}

// The right side is split at a vertex in the middle of the left triangle edge
static void BuildGhostTJunction( std::vector<GhostMesh>& meshes )
{
	GhostMesh mesh;
	mesh.vertices = { { 0.0f, 0.0f, -2.0f },
					  { 0.0f, 0.0f, 2.0f },
					  { -2.0f, 0.0f, 0.0f },
					  { 2.0f, 0.0f, 0.0f },
					  b3Vec3_zero };
	AddGhostTriangle( mesh, 0, 1, 2 );
	AddGhostTriangle( mesh, 4, 1, 3 );
	AddGhostTriangle( mesh, 0, 4, 3 );
	meshes.push_back( mesh );
}

// s&box units are inches. Beam top at y = 0 for x > 0, chamfer dropping 1 inch over 1.5 inch for x
// < 0, 8 inch cells along z with the chamfer quads split like the s&box sample. A side depth in inches
// adds a vertical face below the chamfer, so the chamfer bottom edge is convex instead of a boundary.
static void BuildChamferBeam( std::vector<GhostMesh>& meshes, float sideDepth )
{
	float inch = 0.0254f;
	bool hasSide = sideDepth > 0.0f;
	int rowSize = hasSide ? 4 : 3;
	GhostMesh mesh;
	for ( int k = 0; k <= 4; ++k )
	{
		float z = 4.0f * ( -16.0f + 8.0f * k ) * inch;
		mesh.vertices.push_back( { -1.5f * inch, -1.0f * inch, z } );
		mesh.vertices.push_back( { 0.0f, 0.0f, z } );
		mesh.vertices.push_back( { 48.0f * inch, 0.0f, z } );
		if ( hasSide )
		{
			mesh.vertices.push_back( { -1.5f * inch, ( -1.0f - sideDepth ) * inch, z } );
		}
	}

	for ( int k = 0; k < 4; ++k )
	{
		int low0 = rowSize * k, top0 = low0 + 1, far0 = low0 + 2;
		int low1 = low0 + rowSize, top1 = top0 + rowSize, far1 = far0 + rowSize;
		AddGhostTriangle( mesh, low0, low1, top1 );
		AddGhostTriangle( mesh, low0, top1, top0 );
		AddGhostTriangle( mesh, top0, top1, far1 );
		AddGhostTriangle( mesh, top0, far1, far0 );

		if ( hasSide )
		{
			int side0 = low0 + 3, side1 = low1 + 3;
			b3Vec3 facing = { -1.0f, 0.0f, 0.0f };
			AddGhostTriangle( mesh, side0, side1, low1, facing );
			AddGhostTriangle( mesh, side0, low1, low0, facing );
		}
	}
	meshes.push_back( mesh );
}

static void BuildGhostChamfer( std::vector<GhostMesh>& meshes )
{
	BuildChamferBeam( meshes, 0.0f );
}

static void BuildGhostChamferExtended( std::vector<GhostMesh>& meshes )
{
	BuildChamferBeam( meshes, 6.0f );
}

// A conveyor side rail running along x, from the Conveyor Circuit sample. The upper half of the inner
// wall facing -z and the 8 cm rail top meet at a convex edge, two cells along x so the vertex at x = 0
// is interior. Quad diagonals alternate as in the conveyor mesh.
static void BuildGhostRail( std::vector<GhostMesh>& meshes )
{
	GhostMesh mesh;
	b3Vec2 section[3] = { { 0.0f, 0.175f }, { 0.0f, 0.35f }, { 0.08f, 0.35f } };
	for ( int row = 0; row < 3; ++row )
	{
		for ( int column = 0; column < 3; ++column )
		{
			mesh.vertices.push_back( { -0.25f + 0.25f * column, section[row].y, section[row].x } );
		}
	}

	b3Vec3 facings[2] = { { 0.0f, 0.0f, -1.0f }, b3Vec3_axisY };
	for ( int row = 0; row < 2; ++row )
	{
		for ( int column = 0; column < 2; ++column )
		{
			int a = 3 * row + column;
			int b = a + 1;
			int c = a + 4;
			int d = a + 3;
			if ( ( row + column ) & 1 )
			{
				AddGhostTriangle( mesh, a, d, b, facings[row] );
				AddGhostTriangle( mesh, d, c, b, facings[row] );
			}
			else
			{
				AddGhostTriangle( mesh, a, d, c, facings[row] );
				AddGhostTriangle( mesh, c, b, a, facings[row] );
			}
		}
	}
	meshes.push_back( mesh );
}

static const float s_inch = 0.0254f;

static const GhostScene s_hullGhostScenes[] = {
	{
		"Flat edge, side face",
		"box 1 cm deep, leading face 5 mm past the shared edge, the right triangle picks the box "
		"side face",
		"flat edge, the hull face snaps to the triangle normal",
		BuildGhostFlat,
		{ 0.5f, 0.5f, 0.5f },
		{ -0.495f, 0.49f, 0.0f },
		{ 0.0f, 0.0f, 0.0f },
		true,
	},
	{
		"Flat edge, corner",
		"box yawed 45 degrees, a corner 1 mm deep stops 1 mm short of the shared edge, edge pair "
		"normal",
		"flat edge, snap to the triangle face, which has no points here",
		BuildGhostFlat,
		{ 0.5f, 0.5f, 0.5f },
		{ -0.708f, 0.499f, 0.0f },
		{ 0.0f, 45.0f, 0.0f },
		true,
	},
	{
		"Flat vertex",
		"the corner stops 3 mm short of the fan center, triangles ahead see the center vertex and "
		"the box side faces",
		"flat vertex and edges, snap",
		BuildGhostFan,
		{ 0.5f, 0.5f, 0.5f },
		{ -0.71f, 0.499f, 0.0f },
		{ 0.0f, 45.0f, 0.0f },
		true,
	},
	{
		"Convex edge",
		"box on the right slope tilted 8 degrees, the left triangle crest edge gives a normal past "
		"the right slope",
		"convex edge, the normal is outside the edge cone so the left triangle skips and the right "
		"one owns the contact",
		BuildGhostCrest,
		{ 0.5f, 0.5f, 0.5f },
		{ 0.57f, 0.959f, 0.0f },
		{ 0.0f, 0.0f, -8.0f },
		true,
	},
	{
		"Concave edge",
		"box on the left slope tilted 20 degrees, its corner in the fold, the right triangle picks "
		"the box side face",
		"concave edge, snap to the triangle face",
		BuildGhostValley,
		{ 0.5f, 0.5f, 0.5f },
		{ -0.3f, 0.64f, 0.0f },
		{ 0.0f, 0.0f, -20.0f },
		true,
	},
	{
		"T-junction",
		"a vertex in the middle of the left triangle edge, no edge matches so all are boundary "
		"edges",
		"boundary edge, the block pass tests the normal against the other triangles near the box "
		"and drops it",
		BuildGhostTJunction,
		{ 0.5f, 0.5f, 0.5f },
		{ -0.708f, 0.499f, 0.3f },
		{ 0.0f, 45.0f, 0.0f },
		true,
	},
	{
		"Chamfer",
		"none without handling, SAT picks the box bottom for the chamfer triangles, 0.07 inch deep",
		"causes the ghost, the deepest point is on a flat chamfer vertex or diagonal next to the "
		"convex top edge so the "
		"hull face snaps to the 34 degree chamfer normal (ghost plan step 2c)",
		BuildGhostChamfer,
		{ 16.0f * s_inch, 36.0f * s_inch, 16.0f * s_inch },
		{ 4.0f * s_inch, 36.0f * s_inch - 0.073f * s_inch, 3.0f * s_inch },
		{ 0.0f, 0.0f, 0.0f },
		false,
	},
	{
		"Chamfer extended",
		"the chamfer scene with 6 inches of vertical beam side below the chamfer, 0.07 inch deep",
		"the chamfer bottom edge is convex instead of a boundary",
		BuildGhostChamferExtended,
		{ 16.0f * s_inch, 36.0f * s_inch, 16.0f * s_inch },
		{ 4.0f * s_inch, 36.0f * s_inch - 0.073f * s_inch, 3.0f * s_inch },
		{ 0.0f, 0.0f, 0.0f },
		false,
	},
	{
		"Rail edge",
		"box pitched 60 degrees across the rail, its face cuts the convex rail edge 7 cm past the x = 0 "
		"vertex, the real contact is edge on edge and perpendicular to the rail",
		"the edge pair is shallower than the hull face and the hull face normal crosses the rail edge, so "
		"SAT takes the edge pair. Without that the hull face is admitted at a rail edge vertex and leans 60 "
		"degrees along the rail.",
		BuildGhostRail,
		{ 0.4f, 0.2f, 0.4f },
		{ 0.1f, 0.77f, -0.15f },
		{ -15.0f, 0.0f, 60.0f },
		true,
	},
};

class HullGhostManifold : public GhostManifold
{
public:
	explicit HullGhostManifold( SampleContext* context )
		: GhostManifold( context )
	{
		m_box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		SetScenes( s_hullGhostScenes, IM_ARRAYSIZE( s_hullGhostScenes ) );
	}

	void CreateShape( const GhostScene& scene ) override
	{
		m_box = b3MakeBoxHull( scene.extents.x, scene.extents.y, scene.extents.z );
	}

	int CollideMesh( b3Manifold* manifolds, int capacity, const b3Mesh* mesh,
					 b3Transform transformBtoA, bool speculative, bool handleGhosts ) override
	{
		return b3CollideMeshAndHull( manifolds, capacity, mesh, &m_box.base, transformBtoA,
									 speculative, handleGhosts );
	}

	void CollideTriangle( b3LocalManifold* manifold, int capacity, const b3Vec3* triangle,
						  bool speculative ) override
	{
		b3SATCache cache = {};
		b3CollideTriangleAndHull( manifold, capacity, triangle[0], triangle[1], triangle[2], 0,
								  &m_box.base, &cache, speculative );
	}

	void DrawShape( b3WorldTransform transform ) override
	{
		DrawHull( transform, &m_box.base, MakeColor( b3_colorGreen ) );
	}

	b3AABB ComputeShapeBounds( b3Transform transform ) override
	{
		return b3ComputeHullAABB( &m_box.base, transform );
	}

	static Sample* Create( SampleContext* context )
	{
		return new HullGhostManifold( context );
	}

	b3BoxHull m_box;
};

static int sampleHullGhostManifold =
	RegisterSample( "Manifold", "Hull Ghost Manifold", HullGhostManifold::Create );
