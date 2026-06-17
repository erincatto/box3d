// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "human.h"
#include "imgui.h"
#include "sample.h"
#include "gfx/draw.h"

#include "box3d/box3d.h"

class RagdollOnBox : public Sample
{
public:
	explicit RagdollOnBox( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 6.0f, b3Pos_zero );
		}

		AddGroundBox( 20.0f );

		m_jointFrictionTorque = 5.0f;
		m_jointHertz = 1.0f;
		m_jointDampingRatio = 0.7f;

		m_human = {};

		Spawn();
	}

	void Spawn()
	{
		CreateHuman( &m_human, m_worldId, { 0.0f, 2.0f, 0.0f }, m_jointFrictionTorque, m_jointHertz, m_jointDampingRatio, 1,
					 nullptr, false );
		// Human_ApplyRandomAngularImpulse( &m_human, 10.0f );
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		if ( ImGui::SliderFloat( "Joint Friction", &m_jointFrictionTorque, 0.0f, 20.0f, "%3.0f" ) )
		{
			Human_SetJointFrictionTorque( &m_human, m_jointFrictionTorque );
		}

		if ( ImGui::SliderFloat( "Hertz", &m_jointHertz, 0.0f, 20.0f, "%3.1f" ) )
		{
			Human_SetJointSpringHertz( &m_human, m_jointHertz );
		}

		if ( ImGui::SliderFloat( "Damping", &m_jointDampingRatio, 0.0f, 4.0f, "%3.1f" ) )
		{
			Human_SetJointDampingRatio( &m_human, m_jointDampingRatio );
		}

		if ( ImGui::Button( "Respawn" ) )
		{
			DestroyHuman( &m_human );
			Spawn();
		}
		ImGui::PopItemWidth();
		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new RagdollOnBox( context );
	}

	Human m_human;
	float m_jointFrictionTorque;
	float m_jointHertz;
	float m_jointDampingRatio;
};

static int sampleRagdollOnBox = RegisterSample( "Ragdoll", "Box", RagdollOnBox::Create );

class RagdollOnMesh : public Sample
{
public:
	explicit RagdollOnMesh( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 6.0f, b3Pos_zero );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			m_groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			m_groundMesh = b3CreateGridMesh( 20, 20, 2.0f, 2, true );
			b3CreateMeshShape( m_groundId, &shapeDef, m_groundMesh, b3Vec3_one );
		}

		{
			b3Transform transform;
			transform.p = { 0.0f, 5.0f, -20.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 20.0f, 5.0f, 0.1f, transform );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( m_groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { 0.0f, 5.0f, 20.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 20.0f, 5.0f, 0.1f, transform );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( m_groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { -20.0f, 5.0f, 0.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, 5.0f, 20.0f, transform );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( m_groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { 20.0f, 5.0f, 0.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, 5.0f, 20.0f, transform );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( m_groundId, &shapeDef, &wallBox.base );
		}

		m_jointFrictionTorque = 5.0f;
		m_jointHertz = 2.0f;
		m_jointDampingRatio = 0.7f;

		m_human = {};

		Spawn();
	}

	~RagdollOnMesh() override
	{
		b3DestroyMesh( m_groundMesh );
	}

	void Spawn()
	{
		CreateHuman( &m_human, m_worldId, { 0.0f, 1.0f, 0.0f }, m_jointFrictionTorque, m_jointHertz, m_jointDampingRatio, 1,
					 nullptr, false );
		// Human_AlignSpring( &m_human, m_worldId, m_groundId, 25.0f, 1.0f );
		//  Human_ApplyRandomAngularImpulse( &m_human, 10.0f );
		// b3Body_SetType( m_human.bones[bone_thigh_l].bodyId, b3_kinematicBody );
		// b3Body_SetType( m_human.bones[bone_thigh_r].bodyId, b3_kinematicBody );
		// b3Body_SetType( m_human.bones[bone_pelvis].bodyId, b3_kinematicBody );
		// Human_CreateMotorAnchors( &m_human, m_worldId );
		Human_CreateParallelAnchors( &m_human, m_worldId );
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		if ( ImGui::SliderFloat( "Joint Friction", &m_jointFrictionTorque, 0.0f, 20.0f, "%3.0f" ) )
		{
			Human_SetJointFrictionTorque( &m_human, m_jointFrictionTorque );
		}

		if ( ImGui::SliderFloat( "Hertz", &m_jointHertz, 0.0f, 20.0f, "%3.1f" ) )
		{
			Human_SetJointSpringHertz( &m_human, m_jointHertz );
		}

		if ( ImGui::SliderFloat( "Damping", &m_jointDampingRatio, 0.0f, 4.0f, "%3.1f" ) )
		{
			Human_SetJointDampingRatio( &m_human, m_jointDampingRatio );
		}

		if ( ImGui::Button( "Respawn" ) )
		{
			DestroyHuman( &m_human );
			Spawn();
		}
		ImGui::PopItemWidth();
		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new RagdollOnMesh( context );
	}

	b3MeshData* m_groundMesh;
	b3BodyId m_groundId;
	Human m_human;
	float m_jointFrictionTorque;
	float m_jointHertz;
	float m_jointDampingRatio;
};

static int sampleRagdollMesh = RegisterSample( "Ragdoll", "Mesh", RagdollOnMesh::Create );

class RagdollPile : public Sample
{
public:
	enum
	{
#ifdef NDEBUG
		e_count = 20
#else
		e_count = 8
#endif
	};

	explicit RagdollPile( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 180.0f, 30.0f, 20.0f, b3Pos_zero );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		m_groundMesh = b3CreateGridMesh( 20, 20, 1.0f, 1, true );
		b3CreateMeshShape( groundId, &shapeDef, m_groundMesh, b3Vec3_one );

		for ( int i = 0; i < e_count; ++i )
		{
			b3Pos position = { 0.1f * i, 2.0f + 0.5f * i, -0.1f * i };
			float torque = 10.0f;
			float hertz = 0.5f;
			float damping = 0.7f;
			int groupIndex = i;
			void* userData = nullptr;
			bool colorize = false;
			CreateHuman( m_humans + i, m_worldId, position, torque, hertz, damping, groupIndex, userData, colorize );
		}
	}

	~RagdollPile() override
	{
		b3DestroyMesh( m_groundMesh );
	}

	static Sample* Create( SampleContext* context )
	{
		return new RagdollPile( context );
	}

	b3MeshData* m_groundMesh;
	Human m_humans[e_count] = {};
};

static int sampleRagdollPile = RegisterSample( "Ragdoll", "Pile", RagdollPile::Create );

class RagdollIncline : public Sample
{
public:
	explicit RagdollIncline( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( -20.0f, 30.0f, 25.0f, b3Pos_zero );
		}

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		m_groundMesh = b3CreateGridMesh( 4, 4, 2.0f, 1, true );

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { -10.0f, 2.0f, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, -0.2f * B3_PI );
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateMeshShape( groundId, &shapeDef, m_groundMesh, b3Vec3_one );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, 0.0f, 0.0f };
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );
			b3Vec3 scale = { 4.0f, 4.0f, 4.0f };
			b3CreateMeshShape( groundId, &shapeDef, m_groundMesh, scale );
		}

		m_human = {};
		b3Pos position = { -12.0f, 6.0f, 0.0f };
		float torque = 10.0f;
		float hertz = 2.0f;
		float damping = 0.7f;
		int groupIndex = 1;
		void* userData = nullptr;
		bool colorize = false;
		CreateHuman( &m_human, m_worldId, position, torque, hertz, damping, groupIndex, userData, colorize );
		m_time = 0.0f;
		m_motorized = true;
	}

	~RagdollIncline() override
	{
		b3DestroyMesh( m_groundMesh );
	}

	void Step() override
	{
		if ( m_time > 2.0f && m_motorized == true )
		{
			Human_SetJointFrictionTorque( &m_human, 0.5f );
			Human_SetJointSpringHertz( &m_human, 0.5f );
			m_motorized = false;
		}

		m_time += m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new RagdollIncline( context );
	}

	b3MeshData* m_groundMesh;
	Human m_human;
	float m_time;
	bool m_motorized;
};

static int sampleRagdollIncline = RegisterSample( "Ragdoll", "Incline", RagdollIncline::Create );

#if 0
class RagdollPose : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new RagdollPose( context );
	}

	explicit RagdollPose( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 6.0f, b3Vec3_zero );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { 0.0f, 0.0f, 0.0f };
		// bodyDef.rotation = b3Rotation(B3_VEC3_AXIS_Y, 0.25f * B3_PI);
		b3BodyId groundId = m_worldId->CreateBody( &bodyDef );
		m_ground = b3CreateGrid( 4, 2.0f, 0.0f );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		groundId->AddMesh( &shapeDef, m_ground );

		b3HullData* Hull = b3CreateOffsetBox( { { -3.0f, 0.5f, 0.0f }, b3Quat_identity }, { 0.25f, 0.5f, 3.0f } );
		groundId->AddHull( &shapeDef, Hull );
		b3DestroyHull( Hull );

		m_motorized = false;
		m_motorHertz = 1.0f;
		m_human.Spawn( m_worldId, { 0.0f, 0.1f, 0.0f }, m_motorHertz, m_motorized );

		m_poseControl = true;
		m_poseHertz = 2.0f;
		m_human.EnablePoseControl( m_worldId, m_poseHertz, m_poseControl );

		m_time = 0.0f;

		{
			b3HullData* Cylinder = b3CreateCylinder( 0.5f, 0.5f, 0.0f, 16 );
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 3.0f, 0.5f, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( B3_VEC3_AXIS_Z, 0.5f * B3_PI );
			b3BodyId Body = m_worldId->CreateBody( &bodyDef );
			Body->AddHull( &shapeDef, Cylinder );
			b3DestroyHull( Cylinder );
		}

		m_angle = 0.0f;
		m_angularVelocity = 0.0f;
	}

	~RagdollPose() override
	{
		b3DestroyMesh( m_ground );
	}

	void OnRenderUI( GLFWwindow* ) override
	{
		ImGui::SetNextWindowPos( ImVec2( 10.0f, 600.0f ) );
		ImGui::SetNextWindowSize( ImVec2( 260.0f, 160.0f ) );
		ImGui::Begin( "Pose", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		if ( ImGui::Checkbox( "motors", &m_motorized ) )
		{
			m_human.EnableMotors( m_motorized );
			m_human.AdjustMotors( m_motorHertz );
		}

		if ( ImGui::SliderFloat( "motor hertz", &m_motorHertz, 0.01f, 20.0f, "%.2f" ) )
		{
			m_human.AdjustMotors( m_motorHertz );
		}

		if ( ImGui::Checkbox( "pose", &m_poseControl ) )
		{
			m_human.EnablePoseControl( m_worldId, m_poseHertz, m_poseControl );
		}

		if ( ImGui::SliderFloat( "pose hertz", &m_poseHertz, 0.01f, 8.0f, "%.2f" ) )
		{
			m_human.AdjustPoseControl( m_poseHertz );
		}

		ImGui::SliderFloat( "omega", &m_angularVelocity, 0.0f, 8.0f, "%.1f" );

		ImGui::End();
	}

	void OnUpdate() override
	{
		float timeStep = 1.0f / m_context->Settings.Frequency;
		m_time += timeStep;

		if ( m_poseControl )
		{
			b3Transform transform = m_human.m_rootBody->GetTransform();
			transform.p.x = 4.0f * sinf( 0.5f * m_time );
			transform.p.y = 0.5f * ( cosf( 1.0f * m_time + B3_PI ) + 1.0f );
			m_angle += m_angularVelocity * timeStep;
			transform.q = b3MakeQuatFromAxisAngle( B3_VEC3_AXIS_Y, m_angle );
			m_human.DriveBase( transform, timeStep );
		}

		// transform = mHuman.m_bones[0].poseJoint->GetRelativeTransform();
		// transform.Translation.Y = 0.5f * ( cosf( 1.0f * mTime + B3_PI ) + 1.0f );
		// mHuman.m_bones[0].poseJoint->SetRelativeTransform( transform );

		Sample::OnUpdate();
	}

	b3MeshData* m_ground;
	Human m_human;
	float m_time;
	float m_motorHertz;
	float m_poseHertz;
	float m_angle;
	float m_angularVelocity;
	bool m_motorized;
	bool m_poseControl;
};

static int sampleRagodllPose = RegisterSample( "Ragdoll", "Pose", RagdollPose::Create );
#endif
