// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "camera.h"
#include "mesh_loader.h"
#include "renderer.h"
#include "sample.h"
#include "scene.h"

#include "box3d/box3d.h"

#include <glad/glad.h>
// prevent clang format sorting glad.h with glfw3.h
#include <GLFW/glfw3.h>
#include <imgui.h>

class CapsulePlane : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new CapsulePlane( context );
	}

	explicit CapsulePlane( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 120.0f, 30.0f, 20.0f, { 0.0f, 1.5f, 0.0f } );
		}

		m_transform = { { 0.0f, 1.0f, 0.4f }, b3Quat_identity };
		m_capsule = { { 0.0f, -0.5f, 0.0f }, { 0.0f, 0.5f, 0.0f }, 0.25f };

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { 0.0f, 1.0f, 1.0f };
		b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3CreateHullShape( body, &shapeDef, &box.base );

		m_baseTranslation = b3Vec3_zero;
		m_baseX = 0;
		m_baseY = 0;
		m_origin = b3Vec3_zero;
		m_tracking = false;
		m_planeCount = 0;
	}

	void Solve()
	{
		b3PlaneSolverResult result = b3SolvePlanes( b3Vec3_zero, m_planes, m_planeCount );
		m_transform.p += result.delta;
	}

	void Render() override
	{
		Sample::Render();

		DrawGrid( m_scene, 10 );
		DrawLine( m_scene, b3Vec3_zero, 2.0f * b3Vec3_axisX, b3_colorRed );
		DrawLine( m_scene, b3Vec3_zero, 2.0f * b3Vec3_axisY, b3_colorGreen );
		DrawLine( m_scene, b3Vec3_zero, 2.0f * b3Vec3_axisZ, b3_colorBlue );
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos( ImVec2( 10.0f, 600.0f ) );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, 80.0f ) );
		ImGui::Begin( "Capsule Plane", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		if ( ImGui::Button( "Solve" ) )
		{
			Solve();
		}

		ImGui::End();
	}

	static bool PlaneResultFcn( b3ShapeId shape, const b3PlaneResult* results, int planeCount, void* context )
	{
		CapsulePlane* self = static_cast<CapsulePlane*>( context );
		for ( int i = 0; i < planeCount && self->m_planeCount < 3; ++i )
		{
			self->m_planes[self->m_planeCount] = { results[i].plane, FLT_MAX, 0.0f, true };
			self->m_planeCount += 1;
		}
		return true;
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 && ( modifiers & GLFW_MOD_ALT ) == 0 )
		{
			PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
			m_origin = pickRay.origin + 10.0f * b3Normalize( pickRay.translation );
			m_baseTranslation = m_transform.p;
			m_tracking = true;
		}
	}

	void MouseUp( b3Vec2 p, int button ) override
	{
		m_tracking = false;
	}

	void MouseMove( b3Vec2 p ) override
	{
		if ( m_tracking )
		{
			PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
			b3Vec3 origin = pickRay.origin + 10.0f * b3Normalize( pickRay.translation );
			m_transform.p = m_baseTranslation + origin - m_origin;
		}
	}

	void Step() override
	{
		m_planeCount = 0;

		DrawCapsule( m_scene, m_transform, m_capsule, b3_colorGreen );
		b3QueryFilter filter = b3DefaultQueryFilter();

		b3Capsule capsule = { m_capsule.center1 + m_transform.p, m_capsule.center2 + m_transform.p, m_capsule.radius };
		b3World_CollideMover( m_worldId, &capsule, filter, PlaneResultFcn, this );

		for ( int i = 0; i < m_planeCount; ++i )
		{
			b3Plane plane = m_planes[i].plane;
			b3Vec3 p1 = m_transform.p + ( plane.offset - m_capsule.radius ) * plane.normal;
			b3Vec3 p2 = p1 + 0.1f * plane.normal;
			DrawPoint( m_scene, p1, 5.0f, b3_colorYellow );
			DrawLine( m_scene, p1, p2, b3_colorYellow );
		}
	}

	static constexpr int m_planeCapacity = 3;

	b3Transform m_transform;
	b3Capsule m_capsule;
	b3CollisionPlane m_planes[m_planeCapacity] = {};
	int m_planeCount;

	b3Vec3 m_baseTranslation;
	b3Vec3 m_origin;
	int m_baseX;
	int m_baseY;
	bool m_tracking;
};

static int sampleCapsulePlane = SampleManager::Register( "Character", "CapsulePlane", CapsulePlane::Create );

class BasicMover : public Sample
{
public:
	explicit BasicMover( SampleContext* context )
		: Sample( context )
	{
		b3Vec3 moverPosition = { 7.5f, 0.75f, 9.0f };

		if ( m_context->restart == false )
		{
			m_camera->SetView( 120.0f, 30.0f, 5.0f, moverPosition );
		}

		m_mover.Initialize( this, moverPosition );

		{
			m_levelMesh = CreateMeshData( "data/meshes/test_map01.obj", 1.0f, false, false, true, true );
			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3SurfaceMaterial materials[3];
			materials[0] = { 0.6f, 0.0f, 0 };
			materials[1] = { 0.6f, 1.0f, 1 };
			materials[2] = { 0.1f, 0.0f, 2 };
			shapeDef.materials = materials;
			shapeDef.materialCount = 3;

			b3CreateMeshShape( body, &shapeDef, m_levelMesh, b3Vec3_one );

			// b3Transform transform = { { 0.0f, 1.0f, 14.0f }, b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.75f * B3_PI ) };
			// b3BoxHull box = b3MakeTransformedBoxHull( 5.0f, 1.0f, 0.5f, transform );
			// b3CreateHullShape( body, &shapeDef, &box.base );

			{
				b3Transform transform = { { 4.0f, 1.0f, 14.0f }, b3Quat_identity };
				b3BoxHull box = b3MakeTransformedBoxHull( 1.0f, 1.0f, 1.0f, transform );
				b3CreateHullShape( body, &shapeDef, &box.base );
			}
			{
				b3Transform transform = { { 4.0f, 1.0f, 13.95f }, b3Quat_identity };
				b3BoxHull box = b3MakeTransformedBoxHull( 1.0f, 1.0f, 1.0f, transform );
				b3CreateHullShape( body, &shapeDef, &box.base );
			}
			{
				b3Transform transform = { { 5.8f, 1.0f, 13.7f }, b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.1f * B3_PI ) };
				b3BoxHull box = b3MakeTransformedBoxHull( 1.0f, 1.0f, 1.0f, transform );
				b3CreateHullShape( body, &shapeDef, &box.base );
			}
		}

		{
			m_stairs = CreateMeshData( "data/meshes/stairs.obj", 1.0f, false, false, true, true );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { -10.0f, 0.0f, 0.0f };

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3CreateMeshShape( body, &shapeDef, m_stairs, { 0.75f, 0.75f, -1.5f } );
		}

		{
			m_torus = b3CreateTorusMesh( 10, 12, 2.0f, 1.0f );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { -10.0f, 1.0f, -8.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.5f * B3_PI );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3CreateMeshShape( body, &shapeDef, m_torus, { -0.75f, 1.5f, 0.5f } );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 20.0f, 0.0f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			m_heightField = b3CreateWave( 50.0f, 50.0f, b3Vec3_one, 0.02f, 0.04f, true );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3SurfaceMaterial materials[3];
			materials[0] = { 0.6f, 0.0f, 0 };
			materials[1] = { 0.6f, 1.0f, 1 };
			materials[2] = { 0.1f, 0.0f, 2 };
			shapeDef.materials = materials;
			shapeDef.materialCount = 3;

			b3CreateHeightFieldShape( body, &shapeDef, m_heightField );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, 1.4f, 6.0f };

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			m_enemyShape.maxPush = 1.0f;
			m_enemyShape.clipVelocity = true;

			b3Capsule capsule = {
				{ 0.0f, -0.5f, 0.0f },
				{ 0.0f, 0.5f, 0.0f },
				0.3f,
			};

			//shapeDef.filter = { 2u, ~0u, 0 };
			shapeDef.userData = &m_enemyShape;
			shapeDef.baseMaterial.customColor = b3_colorMediumVioletRed;
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3CreateCapsuleShape( body, &shapeDef, &capsule );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, 1.4f, 5.0f };

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			m_friendlyShape.maxPush = 0.01f;
			m_friendlyShape.clipVelocity = false;

			b3Capsule capsule = {
				{ 0.0f, -0.5f, 0.0f },
				{ 0.0f, 0.5f, 0.0f },
				0.3f,
			};

			shapeDef.filter = { 2u, ~0u, 0 };
			shapeDef.userData = &m_friendlyShape;
			shapeDef.baseMaterial.customColor = b3_colorLimeGreen;
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3CreateCapsuleShape( body, &shapeDef, &capsule );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 7.0f, 5.0f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3Sphere sphere = { b3Vec3_zero, 0.5f };
			b3CreateSphereShape( body, &shapeDef, &sphere );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 7.0f, 2.0f, -3.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.baseMaterial.customColor = b3_colorFloralWhite;
			b3BoxHull box = b3MakeBoxHull( 0.5f, 0.25f, 0.5f );
			m_ignoreShapeId = b3CreateHullShape( body, &shapeDef, &box.base );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -2.0f, 1.6f, 0.0f };
			bodyDef.gravityScale = 2.0f;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 1000.0f;

			b3BoxHull box = b3MakeBoxHull( 0.75f, 1.5f, 0.1f );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );

			b3Quat axisQuat = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, b3Vec3_axisY );
			b3Vec3 offset = { -0.75f, 0.0f, 0.0f };

			b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b3Add( bodyDef.position, offset );
			jointDef.base.localFrameA.q = axisQuat;
			jointDef.base.localFrameB.p = offset;
			jointDef.base.localFrameB.q = axisQuat;

			jointDef.enableLimit = true;
			jointDef.lowerAngle = B3_DEG_TO_RAD * -90.0f;
			jointDef.upperAngle = B3_DEG_TO_RAD * 90.0f;
			jointDef.enableSpring = true;
			jointDef.hertz = 1.0f;
			jointDef.dampingRatio = 0.5f;
			jointDef.enableMotor = false;
			jointDef.maxMotorTorque = 100.0f;
			jointDef.base.drawScale = 2.0f;

			b3CreateRevoluteJoint( m_worldId, &jointDef );
		}

		//{
		//	b3BodyDef BodyDef = b3DefaultBodyDef();
		//	BodyDef.type = b3_dynamicBody;
		//	BodyDef.position = { 25.0f, 5.0f, 10.0f };
		//	b3BodyId Body = b3CreateBody(m_worldId, &bodyDef );

		//	b3ShapeDef ShapeDef = b3DefaultShapeDef();
		//	b3Sphere Sphere = { b3Vec3_zero, 0.5f };
		//	Body->AddSphere( &ShapeDef, Sphere );
		//}

		m_camera->m_thirdPerson = true;
		m_clipVelocity = true;
		glfwSetInputMode( m_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED );

		//m_haveMouseLast = false;
		//m_mouseLast = { 0.0f, 0.0f };
		//m_mouseDelta = { 0.0f, 0.0f };
	}

	~BasicMover() override
	{
		m_camera->m_thirdPerson = false;
		glfwSetInputMode( m_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL );
		b3DestroyMesh( m_levelMesh );
		b3DestroyMesh( m_stairs );
		b3DestroyMesh( m_torus );
		b3DestroyHeightField( m_heightField );
	}

	void Render() override
	{
		Sample::Render();
		DrawTransform( m_scene, { { 0.0f, 0.0f, 0.02f }, b3Quat_identity }, 2.0f );
	}

	void UpdateUI() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 9.0f * fontSize;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->m_height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 19.0f * fontSize, height ) );

		ImGui::Begin( "Mover", nullptr, ImGuiWindowFlags_NoResize );

		bool thirdPerson = m_camera->m_thirdPerson;
		if ( ImGui::Checkbox( "Third Person (Key: T)", &thirdPerson ) )
		{
			ToggleThirdPerson();
		}

		ImGui::Checkbox( "Clip Velocity", &m_clipVelocity );

		ImGui::End();
	}

	void Keyboard( int key, int action, int mods ) override
	{
		if ( key == GLFW_KEY_T && action == GLFW_PRESS )
		{
			ToggleThirdPerson();
		}
	}

	void Step() override
	{
		m_mover.Step(&m_ignoreShapeId, 1, m_clipVelocity);
		DrawTextLine( "third person (T) = %d", m_camera->m_thirdPerson );
		DrawTextLine( "deltaX = %g, deltaY = %g", m_mouseDelta.x, m_mouseDelta.y );

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BasicMover( context );
	}

	CharacterMover m_mover;
	MoverShapeUserData m_enemyShape;
	MoverShapeUserData m_friendlyShape;
	b3MeshData* m_levelMesh;
	b3MeshData* m_stairs;
	b3MeshData* m_torus;
	b3HeightField* m_heightField;
	b3ShapeId m_ignoreShapeId;
	bool m_clipVelocity;
};

static int sampleMover = SampleManager::Register( "Character", "Mover", BasicMover::Create );

