// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "GLFW/glfw3.h"
#include "camera.h"
#include "renderer.h"
#include "sample.h"
#include "scene.h"

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
		m_baseTranslation = b3Vec3_zero;
		m_baseQuaternion = b3Quat_identity;
		m_baseX = 0;
		m_baseY = 0;
		m_origin = b3Vec3_zero;
		m_useCache = false;
		m_tracking = false;
		m_rotating = false;
	}

	void Render() override
	{
		DrawTextLine( "origin: %g %g %g", m_origin.x, m_origin.y, m_origin.z );
		DrawTextLine( "count = %d", m_manifold.pointCount );

		DrawTransform( m_scene, b3Transform_identity, 1.0f );

		if ( m_manifold.pointCount == 0 )
		{
			return;
		}

		float length = 0.5f * b3GetLengthUnitsPerMeter();
		b3Vec3 normal = b3RotateVector( m_transformA.q, m_manifold.normal );

		for ( int pointIndex = 0; pointIndex < m_manifold.pointCount; ++pointIndex )
		{
			const b3LocalManifoldPoint& manifoldPoint = m_manifold.points[pointIndex];

			b3Vec3 point = b3TransformPoint( m_transformA, manifoldPoint.point );

			DrawLine( m_scene, point, point + length * normal, b3_colorWhite );

			if ( manifoldPoint.separation > 0.0f )
			{
				DrawPoint( m_scene, point, 10.0f, b3_colorWhite );
			}
			else
			{
				DrawPoint( m_scene, point, 10.0f, b3_colorYellow );
			}

			DrawWorldString( m_camera, point, b3_colorWhite, "   %.3f", manifoldPoint.separation );

			b3Vec3 perp = b3Perp( normal );
			b3FeaturePair pair = manifoldPoint.pair;
			DrawWorldString( m_camera, point + 0.025f * normal + 0.05f * perp, b3_colorPapayaWhip, "  %X:%X %X:%X", pair.owner1,
							 pair.index1, pair.owner2, pair.index2 );
		}

		Sample::Render();
	}

	void UpdateUI() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 12.0f * fontSize;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->m_height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 10.0f * fontSize, height ) );
		ImGui::Begin( "Manifold", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );
		ImGui::Checkbox( "Use cache", &m_useCache );
		if ( m_useCache )
		{
			ImGui::RadioButton( "auto", &m_manualFeature, 0 );
			ImGui::RadioButton( "faceA", &m_manualFeature, 1 );
			ImGui::RadioButton( "faceB", &m_manualFeature, 2 );
			ImGui::RadioButton( "edgePair", &m_manualFeature, 3 );
		}

		ImGui::End();
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 && ( modifiers & GLFW_MOD_ALT ) == 0 )
		{
			if ( modifiers & GLFW_MOD_SHIFT )
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
			b3Vec3 origin = pickRay.origin + 10.0f * b3Normalize( pickRay.translation );
			m_transformB.p = m_baseTranslation + origin - m_origin;
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
	b3Transform m_transformA;
	b3Transform m_transformB;
	b3Vec3 m_baseTranslation;
	b3Quat m_baseQuaternion;
	b3Vec3 m_origin;
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
		m_baseTranslation = b3Vec3_zero;
		m_baseQuaternion = b3Quat_identity;
		m_baseX = 0;
		m_baseY = 0;
		m_origin = b3Vec3_zero;
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

		DrawTransform( m_scene, b3Transform_identity, 1.0f );

		if ( m_manifold.pointCount > 0 )
		{
			float length = 0.5f * b3GetLengthUnitsPerMeter();

			for ( int pointIndex = 0; pointIndex < m_manifold.pointCount; ++pointIndex )
			{
				const b3LocalManifoldPoint& manifoldPoint = m_manifold.points[pointIndex];

				b3Vec3 point = manifoldPoint.point;

				DrawLine( m_scene, point, point + length * m_manifold.normal, b3_colorWhite );

				if ( manifoldPoint.separation > 0.0f )
				{
					DrawPoint( m_scene, point, 10.0f, b3_colorWhite );
				}
				else
				{
					DrawPoint( m_scene, point, 10.0f, b3_colorYellow );
				}

				DrawWorldString( m_camera, point, b3_colorWhite, "   %.2f", 100.0f * manifoldPoint.separation );

				b3FeaturePair pair = manifoldPoint.pair;
				DrawWorldString( m_camera, point + 0.025f * m_manifold.normal, b3_colorPapayaWhip, "  %X:%X %X:%X", pair.owner1,
								 pair.index1, pair.owner2, pair.index2 );
			}
		}

		b3Vec3 p1 = b3TransformPoint( m_transformA, m_triangle[0] );
		b3Vec3 p2 = b3TransformPoint( m_transformA, m_triangle[1] );
		b3Vec3 p3 = b3TransformPoint( m_transformA, m_triangle[2] );
		DrawFace( m_scene, p1, p2, p3, b3_colorCyan );

		DrawWorldString( m_camera, p1, b3_colorWhite, "0" );
		DrawWorldString( m_camera, p2, b3_colorWhite, "1" );
		DrawWorldString( m_camera, p3, b3_colorWhite, "2" );

		b3Vec3 center = 1.0f / 3.0f * ( p1 + p2 + p3 );
		b3Vec3 normal = b3Normalize( b3Cross( p2 - p1, p3 - p1 ) );
		DrawArrow( m_scene, center, center + 0.5f * normal, 0.05f, b3_colorMediumPurple );

		Sample::Render();
	}

	void UpdateUI() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 12.0f * fontSize;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->m_height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 10.0f * fontSize, height ) );

		ImGui::Begin( "Triangle Manifold", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );
		ImGui::Checkbox( "Use cache", &m_useCache );
		if ( m_useCache )
		{
			ImGui::RadioButton( "auto", &m_manualFeature, 0 );
			ImGui::RadioButton( "faceA", &m_manualFeature, 1 );
			ImGui::RadioButton( "faceB", &m_manualFeature, 2 );
			ImGui::RadioButton( "edgePair", &m_manualFeature, 3 );
		}

		ImGui::End();
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 && ( modifiers & GLFW_MOD_ALT ) == 0 )
		{
			if ( modifiers & GLFW_MOD_SHIFT )
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
			b3Vec3 origin = pickRay.origin + 10.0f * b3Normalize( pickRay.translation );
			m_transformB.p = m_baseTranslation + origin - m_origin;
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
	b3Transform m_transformA;
	b3Transform m_transformB;
	b3Vec3 m_triangle[3] = {};
	b3Vec3 m_baseTranslation;
	b3Quat m_baseQuaternion;
	b3Vec3 m_origin;
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
		DrawSphere( m_scene, m_transformA, m_sphere, b3_colorGreen );
		DrawSphere( m_scene, m_transformB, m_sphere, b3_colorCyan );

		Manifold::Render();
	}

	void Step() override
	{
		b3Transform transformBtoA = b3InvMulTransforms( m_transformA, m_transformB );
		b3CollideSpheres( &m_manifold, m_pointCapacity, &m_sphere, &m_sphere, transformBtoA );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new SphereAndSphere( sampleContext );
	}

	b3Sphere m_sphere;
};

static int sampleCollideSpheres = SampleManager::Register( "Manifold", "Sphere vs Sphere", SphereAndSphere::Create );

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
		DrawCapsule( m_scene, m_transformA, m_capsule, b3_colorCyan );
		DrawSphere( m_scene, m_transformB, m_sphere, b3_colorGreen );

		Manifold::Render();
	}

	void Step() override
	{
		b3Transform transformBtoA = b3InvMulTransforms( m_transformA, m_transformB );
		b3CollideCapsuleAndSphere( &m_manifold, m_pointCapacity, &m_capsule, &m_sphere, transformBtoA );
	}

	b3Sphere m_sphere;
	b3Capsule m_capsule;
};

static int sampleSphereAndCapsule = SampleManager::Register( "Manifold", "Capsule vs Sphere", CapsuleAndSphere::Create );

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
		DrawHull( m_scene, m_transformA, &m_hull.base, b3_colorCyan, false );
		DrawSphere( m_scene, m_transformB, m_sphere, b3_colorGreen );

		Manifold::Render();
	}

	void Step() override
	{
		if ( m_useCache == false )
		{
			m_simplexCache = {};
		}

		b3Transform transformBtoA = b3InvMulTransforms( m_transformA, m_transformB );
		b3CollideHullAndSphere( &m_manifold, m_pointCapacity, &m_hull.base, &m_sphere, transformBtoA, &m_simplexCache );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new HullAndSphere( sampleContext );
	}

	b3Sphere m_sphere;
	b3BoxHull m_hull;
};

static int sampleSphereAndHull = SampleManager::Register( "Manifold", "Hull vs Sphere", HullAndSphere::Create );

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
			m_camera->SetView( 0.0f, 30.0f, 10.0f, b3Vec3_zero );
		}

		m_sphere = { { 0.0f, 0.0f, 0.0f }, 1.0f };
		m_triangle[0] = { 0.0f, 0.0f, 0.0f };
		m_triangle[1] = { 4.0f, 0.0f, 4.0f };
		m_triangle[2] = { 4.0f, 0.0f, 0.0f };

		// b3Quat qA = b3MakeQuatFromAxisAngle( { 0.0f, 1.0f, 0.0f }, 2.0f );
		// m_transformA = { { 1.0f, 1.0f, 0.0f }, qA };

		m_transformA = b3Transform_identity;
		m_transformB = { { 2.0f, 0.5f, 1.0f }, b3Quat_identity };
	}

	void Render() override
	{
		DrawSphere( m_scene, m_transformB, m_sphere, b3_colorGreen );

		TriangleManifold::Render();
	}

	void Step() override
	{
		// Convert triangle to frame B
		b3Transform xf = b3InvMulTransforms( m_transformB, m_transformA );
		b3Vec3 localTriangle[3] = {
			b3TransformPoint( xf, m_triangle[0] ),
			b3TransformPoint( xf, m_triangle[1] ),
			b3TransformPoint( xf, m_triangle[2] ),
		};

		b3CollideSphereAndTriangle( &m_manifold, m_pointCapacity, &m_sphere, localTriangle );

		// Convert manifold from frame B to world space
		m_manifold.normal = b3RotateVector( m_transformB.q, m_manifold.normal );
		for ( int i = 0; i < m_manifold.pointCount; ++i )
		{
			m_manifold.points[i].point = b3TransformPoint( m_transformB, m_manifold.points[i].point );
		}
	}

	b3Sphere m_sphere;
};

static int sampleSphereAndTriangle = SampleManager::Register( "Manifold", "Triangle vs Sphere", TriangleAndSphere::Create );

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
		DrawCapsule( m_scene, m_transformA, m_capsule, b3_colorGreen );
		DrawCapsule( m_scene, m_transformB, m_capsule, b3_colorCyan );

		Manifold::Render();
	}

	void Step() override
	{
		b3Transform transformBtoA = b3InvMulTransforms( m_transformA, m_transformB );
		b3CollideCapsules( &m_manifold, m_pointCapacity, &m_capsule, &m_capsule, transformBtoA );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new CapsuleAndCapsule( sampleContext );
	}

	b3Capsule m_capsule;
};

static int sampleCapsuleAndCapsule = SampleManager::Register( "Manifold", "Capsule vs Capsule", CapsuleAndCapsule::Create );

class CapsuleAndHull : public Manifold
{
public:
	explicit CapsuleAndHull( SampleContext* sampleContext )
		: Manifold( sampleContext )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 5.0f, b3Vec3_zero );
		}

		m_capsule = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 0.5f };
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
		DrawHull( m_scene, m_transformA, &m_hull.base, b3_colorCyan, false );
		DrawCapsule( m_scene, m_transformB, m_capsule, b3_colorGreen );

		Manifold::Render();
	}

	void Step() override
	{
		if ( m_useCache == false )
		{
			m_simplexCache = {};
		}

		b3Transform transformBtoA = b3InvMulTransforms( m_transformA, m_transformB );
		b3CollideHullAndCapsule( &m_manifold, m_pointCapacity, &m_hull.base, &m_capsule, transformBtoA, &m_simplexCache );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new CapsuleAndHull( sampleContext );
	}

	b3BoxHull m_hull;
	b3Capsule m_capsule;
};

static int sampleCapsuleAndHull = SampleManager::Register( "Manifold", "Capsule vs Hull", CapsuleAndHull::Create );

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
			m_camera->SetView( 0.0f, 30.0f, 10.0f, b3Vec3_zero );
		}

		m_capsule = { { 0.0f, -0.2f, 0.0f }, { 0.0f, 0.2f, 0.0f }, 0.05f };
		m_triangle[0] = { -4.0f, 0.0f, -4.0f };
		m_triangle[1] = { -4.0f, 0.0f, 0.0f };
		m_triangle[2] = { 0.0f, 0.0f, 0.0f };

		m_transformA = b3Transform_identity;
		m_transformB = { { 0.0f, 0.5f, 0.0f }, b3Quat_identity };
		m_transformB.q = b3MakeQuatFromAxisAngle( { 0.0f, 0.0f, 1.0f }, 0.5f * B3_PI );

		m_transformB = { { -0.500000000, 0.123778239, -0.500000000 },
						 { { -0.157559350, 0.294042289, 0.821513653 }, -0.462417006 } };
	}

	void Render() override
	{
		DrawCapsule( m_scene, m_transformB, m_capsule, b3_colorGreen );
		DrawTransform( m_scene, m_transformB, 0.1f );

		TriangleManifold::Render();
	}

	void Step() override
	{
		if ( m_useCache == false )
		{
			m_simplexCache = {};
		}

		// Put triangle into capsule coordinates
		b3Transform xf = b3InvMulTransforms( m_transformB, m_transformA );
		b3Vec3 localTriangle[3] = {
			b3TransformPoint( xf, m_triangle[0] ),
			b3TransformPoint( xf, m_triangle[1] ),
			b3TransformPoint( xf, m_triangle[2] ),
		};

		b3CollideCapsuleAndTriangle( &m_manifold, m_pointCapacity, &m_capsule, localTriangle, &m_simplexCache );

		// Convert manifold from frame B to world space
		m_manifold.normal = b3RotateVector( m_transformB.q, m_manifold.normal );
		for ( int i = 0; i < m_manifold.pointCount; ++i )
		{
			m_manifold.points[i].point = b3TransformPoint( m_transformB, m_manifold.points[i].point );
		}
	}

	b3Capsule m_capsule;
};

static int sampleCapsuleAndTriangle = SampleManager::Register( "Manifold", "Triangle vs Capsule", TriangleAndCapsule::Create );

class HullAndHull : public Manifold
{
public:
	explicit HullAndHull( SampleContext* context )
		: Manifold( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 4.0f, b3Vec3_zero );
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

	b3Hull* CreateConvex( float radius1, float height1, float radius2, float height2, Arena arena ) const
	{
		const int sideCount = 32;
		const float deltaAlpha = 2.0f * B3_PI / sideCount;

		int vertexCount = 2 * sideCount;
		b3Vec3* vertexBase = static_cast<b3Vec3*>( arena.Allocate( vertexCount * sizeof( b3Vec3 ) ) );

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
		DrawHull( m_scene, m_transformA, m_hullA, b3_colorGreen, false );
		DrawHull( m_scene, m_transformB, m_hullB, b3_colorCyan, false );

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

		b3Transform transformBtoA = b3InvMulTransforms( m_transformA, m_transformB );
		b3CollideHulls( &m_manifold, m_pointCapacity, m_hullA, m_hullB, transformBtoA, &m_satCache );

		DrawTextLine( "SAT type: %d", m_satCache.type );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new HullAndHull( sampleContext );
	}

	b3BoxHull m_boxA;
	b3BoxHull m_boxB;
	b3Hull* m_hullA;
	b3Hull* m_hullB;
};

static int sampleCollideHulls = SampleManager::Register( "Manifold", "Hull vs Hull", HullAndHull::Create );

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
			m_camera->SetView( 0.0f, 30.0f, 3.0f, b3Vec3_zero );
		}

		m_triangle[0] = { 1.00000000, 0, 1.00000000 };
		m_triangle[1] = { 1.00000000, 0, 0.00000000 };
		m_triangle[2] = { 0.00000000, 0, 0.00000000 };

		m_boxHull = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

		m_transformA = b3Transform_identity;
		m_transformB = b3Transform_identity;
		m_transformB.p = { 0.0f, 0.45f, 0.1f };

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
		DrawHull( m_scene, m_transformB, m_hull, b3_colorGreen, false );

		b3Transform xf;
		xf.p = b3TransformPoint( m_transformB, m_hull->center );
		xf.q = m_transformB.q;
		DrawTransform( m_scene, xf, 0.1f );

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

		b3Transform xf = b3InvMulTransforms( m_transformB, m_transformA );
		b3Vec3 localTriangle[3] = {
			b3TransformPoint( xf, m_triangle[0] ),
			b3TransformPoint( xf, m_triangle[1] ),
			b3TransformPoint( xf, m_triangle[2] ),
		};

		b3CollideHullAndTriangle( &m_manifold, m_pointCapacity, m_hull, localTriangle[0], localTriangle[1], localTriangle[2],
								  m_flags, &m_satCache );

		// Convert manifold from frame B to world space
		m_manifold.normal = b3RotateVector( m_transformB.q, m_manifold.normal );
		for ( int i = 0; i < m_manifold.pointCount; ++i )
		{
			m_manifold.points[i].point = b3TransformPoint( m_transformB, m_manifold.points[i].point );
		}
	}

	int m_flags;
	b3BoxHull m_boxHull;
	b3Hull* m_cylinder;
	b3Hull* m_hull;
};

static int sampleHullAndTriangle = SampleManager::Register( "Manifold", "Triangle vs Hull", TriangleAndHull::Create );
