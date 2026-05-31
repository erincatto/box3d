// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "imgui.h"
#include "sample.h"
#include "sample_draw.h"

#include "gfx/keycodes.h"

#include "box3d/box3d.h"

// Test the distance joint and all options
class DistanceJoint : public Sample
{
public:
	enum
	{
		e_maxCount = 10
	};

	explicit DistanceJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 0.0f, 40.0f, { 0.0f, 10.0f, 0.0f } );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			m_groundId = b3CreateBody( m_worldId, &bodyDef );
		}

		m_count = 0;
		m_hertz = 5.0f;
		m_dampingRatio = 0.5f;
		m_length = 1.0f;
		m_minLength = m_length;
		m_maxLength = m_length;
		m_tensionForce = 2000.0f;
		m_compressionForce = 100.0f;
		m_enableSpring = false;
		m_enableLimit = false;

		for ( int i = 0; i < e_maxCount; ++i )
		{
			m_bodyIds[i] = b3_nullBodyId;
			m_jointIds[i] = b3_nullJointId;
		}

		CreateScene( 1 );
	}

	void CreateScene( int newCount )
	{
		for ( int i = 0; i < m_count; ++i )
		{
			b3DestroyJoint( m_jointIds[i], false );
			m_jointIds[i] = b3_nullJointId;
		}

		for ( int i = 0; i < m_count; ++i )
		{
			b3DestroyBody( m_bodyIds[i] );
			m_bodyIds[i] = b3_nullBodyId;
		}

		m_count = newCount;

		float radius = 0.25f;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, radius };

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 20.0f;

		float yOffset = 20.0f;

		b3DistanceJointDef jointDef = b3DefaultDistanceJointDef();
		jointDef.hertz = m_hertz;
		jointDef.dampingRatio = m_dampingRatio;
		jointDef.length = m_length;
		jointDef.lowerSpringForce = -m_tensionForce;
		jointDef.upperSpringForce = m_compressionForce;
		jointDef.minLength = m_minLength;
		jointDef.maxLength = m_maxLength;
		jointDef.enableSpring = m_enableSpring;
		jointDef.enableLimit = m_enableLimit;

		b3BodyId prevBodyId = m_groundId;
		for ( int i = 0; i < m_count; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.angularDamping = 1.0f;
			bodyDef.position = { m_length * ( i + 1.0f ), yOffset };
			m_bodyIds[i] = b3CreateBody( m_worldId, &bodyDef );
			b3CreateSphereShape( m_bodyIds[i], &shapeDef, &sphere );

			b3Vec3 pivotA = { m_length * i, yOffset, 0.0f };
			b3Vec3 pivotB = { m_length * ( i + 1.0f ), yOffset, 0.0f };
			jointDef.base.bodyIdA = prevBodyId;
			jointDef.base.bodyIdB = m_bodyIds[i];
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( prevBodyId, pivotA );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( m_bodyIds[i], pivotB );
			m_jointIds[i] = b3CreateDistanceJoint( m_worldId, &jointDef );

			prevBodyId = m_bodyIds[i];
		}
	}

	void UpdateUI() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 20.0f * fontSize;
		ImGui::SetNextWindowPos( { 0.5f * fontSize, m_camera->m_height - height - 2.0f * fontSize }, ImGuiCond_Once );
		ImGui::SetNextWindowSize( { 18.0f * fontSize, height } );

		ImGui::Begin( "Distance Joint", nullptr, ImGuiWindowFlags_NoResize );
		ImGui::PushItemWidth( 10.0f * fontSize );

		if ( ImGui::SliderFloat( "Length", &m_length, 0.1f, 4.0f, "%3.1f" ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				b3DistanceJoint_SetLength( m_jointIds[i], m_length );
				b3Joint_WakeBodies( m_jointIds[i] );
			}
		}

		if ( ImGui::Checkbox( "Spring", &m_enableSpring ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				b3DistanceJoint_EnableSpring( m_jointIds[i], m_enableSpring );
				b3Joint_WakeBodies( m_jointIds[i] );
			}
		}

		if ( m_enableSpring )
		{
			if ( ImGui::SliderFloat( "Tension", &m_tensionForce, 0.0f, 4000.0f ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b3DistanceJoint_SetSpringForceRange( m_jointIds[i], -m_tensionForce, m_compressionForce );
					b3Joint_WakeBodies( m_jointIds[i] );
				}
			}

			if ( ImGui::SliderFloat( "Compression", &m_compressionForce, 0.0f, 200.0f ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b3DistanceJoint_SetSpringForceRange( m_jointIds[i], -m_tensionForce, m_compressionForce );
					b3Joint_WakeBodies( m_jointIds[i] );
				}
			}

			if ( ImGui::SliderFloat( "Hertz", &m_hertz, 0.0f, 15.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b3DistanceJoint_SetSpringHertz( m_jointIds[i], m_hertz );
					b3Joint_WakeBodies( m_jointIds[i] );
				}
			}

			if ( ImGui::SliderFloat( "Damping", &m_dampingRatio, 0.0f, 4.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b3DistanceJoint_SetSpringDampingRatio( m_jointIds[i], m_dampingRatio );
					b3Joint_WakeBodies( m_jointIds[i] );
				}
			}
		}

		if ( ImGui::Checkbox( "Limit", &m_enableLimit ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				b3DistanceJoint_EnableLimit( m_jointIds[i], m_enableLimit );
				b3Joint_WakeBodies( m_jointIds[i] );
			}
		}

		if ( m_enableLimit )
		{
			if ( ImGui::SliderFloat( "Min Length", &m_minLength, 0.1f, 4.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b3DistanceJoint_SetLengthRange( m_jointIds[i], m_minLength, m_maxLength );
					b3Joint_WakeBodies( m_jointIds[i] );
				}
			}

			if ( ImGui::SliderFloat( "Max Length", &m_maxLength, 0.1f, 4.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b3DistanceJoint_SetLengthRange( m_jointIds[i], m_minLength, m_maxLength );
					b3Joint_WakeBodies( m_jointIds[i] );
				}
			}
		}

		int count = m_count;
		if ( ImGui::SliderInt( "Count", &count, 1, e_maxCount ) )
		{
			CreateScene( count );
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}

	void Render() override
	{
		Sample::Render();

		DrawGrid( m_scene, 20 );
	}

	static Sample* Create( SampleContext* context )
	{
		return new DistanceJoint( context );
	}

	b3BodyId m_groundId;
	b3BodyId m_bodyIds[e_maxCount];
	b3JointId m_jointIds[e_maxCount];
	int m_count;
	float m_hertz;
	float m_dampingRatio;
	float m_length;
	float m_tensionForce;
	float m_compressionForce;
	float m_minLength;
	float m_maxLength;
	bool m_enableSpring;
	bool m_enableLimit;
};

static int sampleDistanceJoint = SampleManager::Register( "Joints", "Distance Joint", DistanceJoint::Create );

class FilterJoint : public Sample
{
public:
	explicit FilterJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
			EnableGrid( m_scene, true );
		}

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
			b3CreateHullShape( groundId, &shapeDef, &groundBox.base );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 2.0f, 4.0f, 0.0f };
		b3BodyId bodyId1 = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3CreateHullShape( bodyId1, &shapeDef, &box.base );

		bodyDef.position = { -2.0f, 4.0f, 0.0f };
		b3BodyId bodyId2 = b3CreateBody( m_worldId, &bodyDef );
		b3CreateHullShape( bodyId2, &shapeDef, &box.base );

		b3FilterJointDef jointDef = b3DefaultFilterJointDef();
		jointDef.base.bodyIdA = bodyId1;
		jointDef.base.bodyIdB = bodyId2;
		b3CreateFilterJoint( m_worldId, &jointDef );
	}

	void Render() override
	{
		Sample::Render();

		b3Transform transform = b3Transform_identity;
		transform.p.y += 0.05f;
		DrawTransform( m_scene, transform, 2.0f );
	}

	static Sample* Create( SampleContext* context )
	{
		return new FilterJoint( context );
	}
};

static int sampleFilterJoint = SampleManager::Register( "Joints", "Filter", FilterJoint::Create );

/// This test shows how to use a motor joint. A motor joint
/// can be used to animate a dynamic body. With finite motor forces
/// the body can be blocked by collision with other bodies.
class MotorJoint : public Sample
{
public:
	explicit MotorJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 0.0f, 40.0f, { 0.0f, 10.0f, 0.0f } );
			EnableGrid( m_scene, true );
		}

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position.y = -1.0f;
			groundId = b3CreateBody( m_worldId, &bodyDef );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull box = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
			b3CreateHullShape( groundId, &shapeDef, &box.base );
		}

		m_transform = { .p = { 0.0f, 10.0f, 0.0f }, .q = b3Quat_identity };

		// Define a target body
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_kinematicBody;
			bodyDef.position = m_transform.p;
			m_targetId = b3CreateBody( m_worldId, &bodyDef );
		}

		// Define motorized body
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = m_transform.p;
			m_bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 1.0f, 0.25f, 0.25f );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( m_bodyId, &shapeDef, &box.base );

			m_maxForce = 400000.0f;
			m_maxTorque = 500000.0f;

			b3MotorJointDef jointDef = b3DefaultMotorJointDef();
			jointDef.base.bodyIdA = m_targetId;
			jointDef.base.bodyIdB = m_bodyId;
			jointDef.linearHertz = 4.0f;
			jointDef.linearDampingRatio = 0.7f;
			jointDef.angularHertz = 4.0f;
			jointDef.angularDampingRatio = 0.7f;
			jointDef.maxSpringForce = m_maxForce;
			jointDef.maxSpringTorque = m_maxTorque;

			m_jointId = b3CreateMotorJoint( m_worldId, &jointDef );
		}

		// Define spring body
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -2.0f, 2.0f, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( bodyId, &shapeDef, &box.base );

			b3MotorJointDef jointDef = b3DefaultMotorJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b3Add( bodyDef.position, { 0.25f, 1.25f, 0.0f } );
			jointDef.base.localFrameB.p = { 0.25f, 0.25f };
			jointDef.base.collideConnected = true;
			jointDef.linearHertz = 7.5f;
			jointDef.linearDampingRatio = 0.7f;
			jointDef.angularHertz = 7.5f;
			jointDef.angularDampingRatio = 0.7f;
			jointDef.maxSpringForce = 200000.0f;
			jointDef.maxSpringTorque = 10000.0f;

			b3CreateMotorJoint( m_worldId, &jointDef );
		}

		m_speed = 0.0f;
		m_time = 0.0f;
	}

	void UpdateUI() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 180.0f;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->m_height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Motor Joint", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::SliderFloat( "Speed", &m_speed, -5.0f, 5.0f, "%.0f" ) )
		{
		}

		if ( ImGui::SliderFloat( "Max Force", &m_maxForce, 0.0f, 1000000.0f, "%.0f" ) )
		{
			b3MotorJoint_SetMaxSpringForce( m_jointId, m_maxForce );
		}

		if ( ImGui::SliderFloat( "Max Torque", &m_maxTorque, 0.0f, 1000000.0f, "%.0f" ) )
		{
			b3MotorJoint_SetMaxSpringTorque( m_jointId, m_maxTorque );
		}

		if ( ImGui::Button( "Apply Impulse" ) )
		{
			b3Body_ApplyLinearImpulseToCenter( m_bodyId, { 100000.0f, 0.0f }, true );
		}

		ImGui::End();
	}

	void Step() override
	{
		float timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;

		if ( m_context->pause )
		{
			if ( m_context->singleStep == 0 )
			{
				timeStep = 0.0f;
			}
		}

		if ( timeStep > 0.0f )
		{
			m_time += m_speed * timeStep;

			b3Vec3 linearOffset;
			linearOffset.x = 6.0f * sinf( 2.0f * m_time );
			linearOffset.y = 10.0f + 4.0f * sinf( 1.0f * m_time );
			linearOffset.z = 0.0f;

			float angularOffset = 2.0f * m_time;
			m_transform = { linearOffset, b3MakeQuatFromAxisAngle( b3Vec3_axisZ, angularOffset ) };

			b3Body_SetTargetTransform( m_targetId, m_transform, timeStep, true );
		}

		DrawTransform( m_scene, m_transform, 1.0f );

		Sample::Step();

		b3Vec3 force = b3Joint_GetConstraintForce( m_jointId );
		b3Vec3 torque = b3Joint_GetConstraintTorque( m_jointId );

		DrawTextLine( "force = %3.f, torque = %3.f", b3Length( force ), b3Length( torque ) );
	}

	static Sample* Create( SampleContext* context )
	{
		return new MotorJoint( context );
	}

	b3BodyId m_targetId;
	b3BodyId m_bodyId;
	b3JointId m_jointId;
	b3Transform m_transform;
	float m_time;
	float m_speed;
	float m_maxForce;
	float m_maxTorque;
};

static int sampleMotorJoint = SampleManager::Register( "Joints", "Motor Joint", MotorJoint::Create );

class TopDownFriction : public Sample
{
public:
	explicit TopDownFriction( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 0.0f, 25.0f, { 0.0f, 10.0f, 0.0f } );
		}

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			groundId = b3CreateBody( m_worldId, &bodyDef );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull box = b3MakeTransformedBoxHull( 10.0f, 0.5f, 50.0f, { b3Vec3_zero, b3Quat_identity } );
			b3CreateHullShape( groundId, &shapeDef, &box.base );

			box = b3MakeTransformedBoxHull( 0.5f, 10.0f, 50.0f, { { -10.0f, 10.0f, 0.0f }, b3Quat_identity } );
			b3CreateHullShape( groundId, &shapeDef, &box.base );

			box = b3MakeTransformedBoxHull( 0.5f, 10.0f, 50.0f, { { 10.0f, 10.0f, 0.0f }, b3Quat_identity } );
			b3CreateHullShape( groundId, &shapeDef, &box.base );

			box = b3MakeTransformedBoxHull( 10.0f, 0.5f, 50.0f, { { 00.0f, 20.0f, 0.0f }, b3Quat_identity } );
			b3CreateHullShape( groundId, &shapeDef, &box.base );
		}

		b3MotorJointDef jointDef = b3DefaultMotorJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.collideConnected = true;
		jointDef.maxVelocityForce = 1000.0f;
		jointDef.maxVelocityTorque = 1000.0f;

		b3Capsule capsule = { { -0.25f, 0.0f }, { 0.25f, 0.0f }, 0.25f };
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.35f };
		b3BoxHull cube = b3MakeBoxHull( 0.35f, 0.35f, 0.35f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.gravityScale = 0.0f;
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.restitution = 0.8f;

		int n = 10;
		float x = -5.0f, y = 15.0f;
		for ( int i = 0; i < n; ++i )
		{
			for ( int j = 0; j < n; ++j )
			{
				bodyDef.position = { x, y };
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

				int remainder = ( n * i + j ) % 4;
				if ( remainder == 0 )
				{
					b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
				}
				else if ( remainder == 1 )
				{
					b3CreateSphereShape( bodyId, &shapeDef, &sphere );
				}
				else
				{
					b3CreateHullShape( bodyId, &shapeDef, &cube.base );
				}

				jointDef.base.bodyIdB = bodyId;
				b3CreateMotorJoint( m_worldId, &jointDef );

				x += 1.0f;
			}

			x = -5.0f;
			y -= 1.0f;
		}
	}

	void UpdateUI() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 180.0f;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->m_height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Top Down Friction", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Button( "Explode" ) )
		{
			b3Sphere sphere = { { 0.0f, 10.0f, 0.0 }, 10.0f };
			b3ExplosionDef def = b3DefaultExplosionDef();
			def.position = sphere.center;
			def.radius = sphere.radius;
			def.falloff = 5.0f;
			def.impulsePerArea = 10000.0f;
			b3World_Explode( m_worldId, &def );

			DrawSphere( m_scene, b3Transform_identity, sphere, b3_colorWhite );
		}

		ImGui::End();
	}

	static Sample* Create( SampleContext* context )
	{
		return new TopDownFriction( context );
	}
};

static int sampleTopDownFriction = SampleManager::Register( "Joints", "Top Down Friction", TopDownFriction::Create );

class PrismaticJoint : public Sample
{
public:
	explicit PrismaticJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
			EnableGrid( m_scene, true );
		}

		m_targetTranslation = 0.0f;
		m_motorSpeed = 0.0f;
		m_motorForce = 20.0f;
		m_hertz = 2.0f;
		m_dampingRatio = 0.7f;
		m_lowerTranslation = -1.0f;
		m_upperTranslation = 1.0f;
		m_enableSpring = true;
		m_enableMotor = false;
		m_enableLimit = false;

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
			b3CreateHullShape( groundId, &shapeDef, &groundBox.base );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 4.0f, 0.0f };
		bodyDef.gravityScale = 0.0f;
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		b3BoxHull box = b3MakeBoxHull( 0.5f, 1.5f, 0.25f );
		b3CreateHullShape( m_bodyId, &shapeDef, &box.base );

		b3PrismaticJointDef jointDef = b3DefaultPrismaticJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = m_bodyId;
		jointDef.base.localFrameA.p = { 0.0f, 6.5f, 0.0f };
		jointDef.base.localFrameB.p = { 0.0f, 1.5f, 0.0f };
		jointDef.base.constraintHertz = 120.0f;
		jointDef.enableLimit = m_enableLimit;
		jointDef.lowerTranslation = m_lowerTranslation;
		jointDef.upperTranslation = m_upperTranslation;
		jointDef.enableSpring = m_enableSpring;
		jointDef.hertz = m_hertz;
		jointDef.dampingRatio = m_dampingRatio;
		jointDef.targetTranslation = m_targetTranslation;
		jointDef.enableMotor = m_enableMotor;
		jointDef.maxMotorForce = m_motorForce;
		jointDef.motorSpeed = m_motorSpeed;

		m_jointId = b3CreatePrismaticJoint( m_worldId, &jointDef );
	}

	void UpdateUI() override
	{
		float height = 320.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_camera->m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 300.0f, height ) );

		ImGui::Begin( "Prismatic Joint", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Checkbox( "Limit", &m_enableLimit ) )
		{
			b3PrismaticJoint_EnableLimit( m_jointId, m_enableLimit );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableLimit )
		{
			if ( ImGui::SliderFloat( "Lower Translation", &m_lowerTranslation, -10.0f, 10.0f, "%.1f" ) )
			{
				m_lowerTranslation = b3MinFloat( m_lowerTranslation, m_upperTranslation );
				b3PrismaticJoint_SetLimits( m_jointId, m_lowerTranslation, m_upperTranslation );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Upper Translation", &m_upperTranslation, -10.0f, 10.0f, "%.1f" ) )
			{
				m_upperTranslation = b3MaxFloat( m_upperTranslation, m_lowerTranslation );
				b3PrismaticJoint_SetLimits( m_jointId, m_lowerTranslation, m_upperTranslation );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		if ( ImGui::Checkbox( "Motor", &m_enableMotor ) )
		{
			b3PrismaticJoint_EnableMotor( m_jointId, m_enableMotor );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableMotor )
		{
			if ( ImGui::SliderFloat( "Max Force", &m_motorForce, 0.0f, 100000.0f, "%.0f" ) )
			{
				b3PrismaticJoint_SetMaxMotorForce( m_jointId, m_motorForce );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Speed", &m_motorSpeed, -10.0f, 10.0f, "%.0f" ) )
			{
				b3PrismaticJoint_SetMotorSpeed( m_jointId, m_motorSpeed );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		if ( ImGui::Checkbox( "Spring", &m_enableSpring ) )
		{
			b3PrismaticJoint_EnableSpring( m_jointId, m_enableSpring );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableSpring )
		{
			if ( ImGui::SliderFloat( "Hertz", &m_hertz, 0.0f, 10.0f, "%.1f" ) )
			{
				b3PrismaticJoint_SetSpringHertz( m_jointId, m_hertz );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Damping", &m_dampingRatio, 0.0f, 2.0f, "%.1f" ) )
			{
				b3PrismaticJoint_SetSpringDampingRatio( m_jointId, m_dampingRatio );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Translation", &m_targetTranslation, -20.0f, 20.0f, "%.1f" ) )
			{
				b3PrismaticJoint_SetTargetTranslation( m_jointId, m_targetTranslation );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		ImGui::End();
	}

	void Render() override
	{
		Sample::Render();
	}

	static Sample* Create( SampleContext* context )
	{
		return new PrismaticJoint( context );
	}

	b3BodyId m_bodyId;
	b3JointId m_jointId;
	float m_targetTranslation;
	float m_motorSpeed;
	float m_motorForce;
	float m_hertz;
	float m_dampingRatio;
	float m_lowerTranslation;
	float m_upperTranslation;
	bool m_enableSpring;
	bool m_enableMotor;
	bool m_enableLimit;
};

static int samplePrismaticJoint = SampleManager::Register( "Joints", "Prismatic", PrismaticJoint::Create );

class SphericalJoint : public Sample
{
public:
	explicit SphericalJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
			EnableGrid( m_scene, true );
		}

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
			b3CreateHullShape( groundId, &shapeDef, &groundBox.base );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 4.0f, 1.5f };
		bodyDef.gravityScale = 0.0f;
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.25f, 1.5f );
		b3CreateHullShape( m_bodyId, &shapeDef, &box.base );

		b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = m_bodyId;
		jointDef.base.drawScale = 2.0f;
		jointDef.base.localFrameA.p = { 0.0f, 5.5f, 0.0f };
		jointDef.base.localFrameB.p = { 0.0f, 0.0f, -1.5f };
		jointDef.enableConeLimit = m_enableConeLimit;
		jointDef.coneAngle = B3_DEG_TO_RAD * m_coneAngleDegrees;
		jointDef.enableTwistLimit = m_enableTwistLimit;
		jointDef.lowerTwistAngle = B3_DEG_TO_RAD * m_lowerTwistDegrees;
		jointDef.upperTwistAngle = B3_DEG_TO_RAD * m_upperTwistDegrees;
		jointDef.enableSpring = m_enableSpring;
		jointDef.hertz = m_hertz;
		jointDef.dampingRatio = m_dampingRatio;
		jointDef.enableMotor = m_enableMotor;
		jointDef.maxMotorTorque = m_motorTorque;
		jointDef.motorVelocity = m_motorVelocity;

		m_jointId = b3CreateSphericalJoint( m_worldId, &jointDef );
	}

	void UpdateUI() override
	{
		float height = 360.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_camera->m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 300.0f, height ) );

		ImGui::Begin( "Spherical Joint", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Checkbox( "Cone Limit", &m_enableConeLimit ) )
		{
			b3SphericalJoint_EnableConeLimit( m_jointId, m_enableConeLimit );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableConeLimit )
		{
			if ( ImGui::SliderFloat( "Cone Angle", &m_coneAngleDegrees, 0.0f, 90.0f, "%.0f" ) )
			{
				b3SphericalJoint_SetConeLimit( m_jointId, B3_DEG_TO_RAD * m_coneAngleDegrees );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		if ( ImGui::Checkbox( "Twist Limit", &m_enableTwistLimit ) )
		{
			b3SphericalJoint_EnableTwistLimit( m_jointId, m_enableTwistLimit );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableTwistLimit )
		{
			if ( ImGui::SliderFloat( "Lower Twist", &m_lowerTwistDegrees, -180.0f, 180.0f, "%.0f" ) )
			{
				m_lowerTwistDegrees = b3MinFloat( m_lowerTwistDegrees, m_upperTwistDegrees );
				b3SphericalJoint_SetTwistLimits( m_jointId, B3_DEG_TO_RAD * m_lowerTwistDegrees,
												 B3_DEG_TO_RAD * m_upperTwistDegrees );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Upper Twist", &m_upperTwistDegrees, -180.0f, 180.0f, "%.0f" ) )
			{
				m_upperTwistDegrees = b3MaxFloat( m_upperTwistDegrees, m_lowerTwistDegrees );
				b3SphericalJoint_SetTwistLimits( m_jointId, B3_DEG_TO_RAD * m_lowerTwistDegrees,
												 B3_DEG_TO_RAD * m_upperTwistDegrees );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		if ( ImGui::Checkbox( "Motor", &m_enableMotor ) )
		{
			b3SphericalJoint_EnableMotor( m_jointId, m_enableMotor );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableMotor )
		{
			if ( ImGui::SliderFloat( "Max Torque", &m_motorTorque, 0.0f, 50.0f, "%.0f" ) )
			{
				b3SphericalJoint_SetMaxMotorTorque( m_jointId, m_motorTorque );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat3( "Velocity", &m_motorVelocity.x, -10.0f, 10.0f, "%.0f" ) )
			{
				b3SphericalJoint_SetMotorVelocity( m_jointId, m_motorVelocity );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		if ( ImGui::Checkbox( "Spring", &m_enableSpring ) )
		{
			b3SphericalJoint_EnableSpring( m_jointId, m_enableSpring );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableSpring )
		{
			if ( ImGui::SliderFloat( "Hertz", &m_hertz, 0.0f, 10.0f, "%.1f" ) )
			{
				b3SphericalJoint_SetSpringHertz( m_jointId, m_hertz );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Damping", &m_dampingRatio, 0.0f, 2.0f, "%.1f" ) )
			{
				b3SphericalJoint_SetSpringDampingRatio( m_jointId, m_dampingRatio );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat3( "Rotation", &m_targetRotation.x, -180.0f, 180.0f, "%.0f" ) )
			{
				b3Quat qx = b3MakeQuatFromAxisAngle( b3Vec3_axisX, B3_DEG_TO_RAD * m_targetRotation.x );
				b3Quat qy = b3MakeQuatFromAxisAngle( b3Vec3_axisY, B3_DEG_TO_RAD * m_targetRotation.y );
				b3Quat qz = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, B3_DEG_TO_RAD * m_targetRotation.z );
				b3Quat q = b3MulQuat( qz, b3MulQuat( qy, qx ) );
				b3SphericalJoint_SetTargetRotation( m_jointId, q );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		ImGui::End();
	}

	void Render() override
	{
		Sample::Render();

		b3Transform transform = b3Transform_identity;
		transform.p.y += 0.05f;
		DrawTransform( m_scene, transform, 2.0f );

		transform = b3Body_GetTransform( m_bodyId );
		DrawTransform( m_scene, transform, 2.0f );
	}

	static Sample* Create( SampleContext* context )
	{
		return new SphericalJoint( context );
	}

	b3BodyId m_bodyId;
	b3JointId m_jointId;
	b3Vec3 m_targetRotation = b3Vec3_zero;
	b3Vec3 m_motorVelocity = b3Vec3_zero;
	float m_motorTorque = 20.0f;
	float m_hertz = 2.0f;
	float m_dampingRatio = 0.7f;
	float m_coneAngleDegrees = 30.0f;
	float m_lowerTwistDegrees = -35.0f;
	float m_upperTwistDegrees = 35.0f;
	bool m_enableSpring = true;
	bool m_enableMotor = false;
	bool m_enableTwistLimit = false;
	bool m_enableConeLimit = false;
};

static int sampleSphericalJoint = SampleManager::Register( "Joints", "Spherical", SphericalJoint::Create );

class ParallelJoint : public Sample
{
public:
	explicit ParallelJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
			EnableGrid( m_scene, true );
		}

		m_hertz = 10.0f;
		m_dampingRatio = 0.7f;
		m_maxTorque = 5000.0f;

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
			b3CreateHullShape( groundId, &shapeDef, &groundBox.base );
		}

		{
			b3Transform transform;
			transform.p = { 0.0f, 5.0f, -20.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 20.0f, 5.0f, 0.1f, transform );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { 0.0f, 5.0f, 20.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 20.0f, 5.0f, 0.1f, transform );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { -20.0f, 5.0f, 0.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, 5.0f, 20.0f, transform );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { 20.0f, 5.0f, 0.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, 5.0f, 20.0f, transform );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 4.0f, 0.0f };
		bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.25f * B3_PI );
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		b3BoxHull box = b3MakeBoxHull( 0.5f, 1.5f, 0.25f );
		b3CreateHullShape( m_bodyId, &shapeDef, &box.base );

		b3ParallelJointDef jointDef = b3DefaultParallelJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = m_bodyId;
		jointDef.base.localFrameA.q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, b3Vec3_axisY );
		jointDef.base.localFrameB.q = b3InvMulQuat( bodyDef.rotation, jointDef.base.localFrameA.q );

		jointDef.base.drawScale = 2.0f;
		jointDef.base.collideConnected = true;
		jointDef.hertz = m_hertz;
		jointDef.dampingRatio = m_dampingRatio;

		m_jointId = b3CreateParallelJoint( m_worldId, &jointDef );
	}

	void UpdateUI() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 20.0f * fontSize;
		ImGui::SetNextWindowPos( { 1.0f * fontSize, m_camera->m_height - height - 3.0f * fontSize }, ImGuiCond_Once );
		ImGui::SetNextWindowSize( { 20.0f * fontSize, height } );

		ImGui::Begin( "Parallel Joint", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::SliderFloat( "Hertz", &m_hertz, 0.0f, 5.0f, "%.1f" ) )
		{
			b3ParallelJoint_SetSpringHertz( m_jointId, m_hertz );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( ImGui::SliderFloat( "Damping", &m_dampingRatio, 0.0f, 2.0f, "%.1f" ) )
		{
			b3ParallelJoint_SetSpringDampingRatio( m_jointId, m_dampingRatio );
			b3Joint_WakeBodies( m_jointId );
		}

		ImGui::End();
	}

	static Sample* Create( SampleContext* context )
	{
		return new ParallelJoint( context );
	}

	b3BodyId m_bodyId;
	b3JointId m_jointId;
	float m_maxTorque;
	float m_hertz;
	float m_dampingRatio;
};

static int sampleParallelJoint = SampleManager::Register( "Joints", "Parallel Spring", ParallelJoint::Create );

class RevoluteJoint : public Sample
{
public:
	explicit RevoluteJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
			EnableGrid( m_scene, true );
		}

		m_targetAngle = 0.0f;
		m_motorSpeed = 0.0f;
		m_motorTorque = 5000.0f;
		m_hertz = 2.0f;
		m_dampingRatio = 0.7f;
		m_lowerDegrees = -35.0f;
		m_upperDegrees = 35.0f;
		m_enableSpring = false;
		m_enableMotor = false;
		m_enableLimit = false;

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
			b3CreateHullShape( groundId, &shapeDef, &groundBox.base );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 4.0f, 0.0f };
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		b3BoxHull box = b3MakeBoxHull( 0.5f, 1.5f, 0.25f );
		b3CreateHullShape( m_bodyId, &shapeDef, &box.base );

		b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = m_bodyId;
		jointDef.base.localFrameA.p = { 0.0f, 6.5f, 0.0f };
		jointDef.base.localFrameB.p = { 0.0f, 1.5f, 0.0f };
		jointDef.base.drawScale = 2.0f;
		jointDef.enableLimit = m_enableLimit;
		jointDef.lowerAngle = B3_DEG_TO_RAD * m_lowerDegrees;
		jointDef.upperAngle = B3_DEG_TO_RAD * m_upperDegrees;
		jointDef.enableSpring = m_enableSpring;
		jointDef.hertz = m_hertz;
		jointDef.dampingRatio = m_dampingRatio;
		jointDef.enableMotor = m_enableMotor;
		jointDef.maxMotorTorque = m_motorTorque;
		jointDef.motorSpeed = m_motorSpeed;

		m_jointId = b3CreateRevoluteJoint( m_worldId, &jointDef );
	}

	void UpdateUI() override
	{
		float height = 320.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_camera->m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 300.0f, height ) );

		ImGui::Begin( "Revolute Joint", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Checkbox( "Limit", &m_enableLimit ) )
		{
			b3RevoluteJoint_EnableLimit( m_jointId, m_enableLimit );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableLimit )
		{
			if ( ImGui::SliderFloat( "Lower Angle", &m_lowerDegrees, -180.0f, 180.0f, "%.0f" ) )
			{
				m_lowerDegrees = b3MinFloat( m_lowerDegrees, m_upperDegrees );
				b3RevoluteJoint_SetLimits( m_jointId, B3_DEG_TO_RAD * m_lowerDegrees, B3_DEG_TO_RAD * m_upperDegrees );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Upper Angle", &m_upperDegrees, -180.0f, 180.0f, "%.0f" ) )
			{
				m_upperDegrees = b3MaxFloat( m_upperDegrees, m_lowerDegrees );
				b3RevoluteJoint_SetLimits( m_jointId, B3_DEG_TO_RAD * m_lowerDegrees, B3_DEG_TO_RAD * m_upperDegrees );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		if ( ImGui::Checkbox( "Motor", &m_enableMotor ) )
		{
			b3RevoluteJoint_EnableMotor( m_jointId, m_enableMotor );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableMotor )
		{
			if ( ImGui::SliderFloat( "Max Torque", &m_motorTorque, 0.0f, 50000.0f, "%.0f" ) )
			{
				b3RevoluteJoint_SetMaxMotorTorque( m_jointId, m_motorTorque );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Speed", &m_motorSpeed, -10.0f, 10.0f, "%.0f" ) )
			{
				b3RevoluteJoint_SetMotorSpeed( m_jointId, m_motorSpeed );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		if ( ImGui::Checkbox( "Spring", &m_enableSpring ) )
		{
			b3RevoluteJoint_EnableSpring( m_jointId, m_enableSpring );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableSpring )
		{
			if ( ImGui::SliderFloat( "Hertz", &m_hertz, 0.0f, 10.0f, "%.1f" ) )
			{
				b3RevoluteJoint_SetSpringHertz( m_jointId, m_hertz );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Damping", &m_dampingRatio, 0.0f, 2.0f, "%.1f" ) )
			{
				b3RevoluteJoint_SetSpringDampingRatio( m_jointId, m_dampingRatio );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Rotation", &m_targetAngle, -180.0f, 180.0f, "%.0f" ) )
			{
				b3RevoluteJoint_SetTargetAngle( m_jointId, B3_DEG_TO_RAD * m_targetAngle );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		ImGui::End();
	}

	void Render() override
	{
		Sample::Render();

		b3MassData massData = b3Body_GetMassData( m_bodyId );
		b3Vec3 angularVelocity = b3Body_GetAngularVelocity( m_bodyId );
		b3Vec3 linearVelocity = b3Body_GetLinearVelocity( m_bodyId );
		float kineticEnergy = 0.5f * b3Dot( angularVelocity, b3MulMV( massData.inertia, angularVelocity ) );
		kineticEnergy += 0.5f * massData.mass * b3Dot( linearVelocity, linearVelocity );
		b3Vec3 center = b3Body_GetWorldCenterOfMass( m_bodyId );
		b3Vec3 gravity = b3World_GetGravity( m_worldId );
		float potentialEnergy = -massData.mass * center.y * gravity.y;
		DrawTextLine( "kinetic energy = %g", kineticEnergy );
		DrawTextLine( "potential energy = %g", potentialEnergy );
		DrawTextLine( "total energy = %g", kineticEnergy + potentialEnergy );
	}

	static Sample* Create( SampleContext* context )
	{
		return new RevoluteJoint( context );
	}

	b3BodyId m_bodyId;
	b3JointId m_jointId;
	float m_targetAngle;
	float m_motorSpeed;
	float m_motorTorque;
	float m_hertz;
	float m_dampingRatio;
	float m_lowerDegrees;
	float m_upperDegrees;
	bool m_enableSpring;
	bool m_enableMotor;
	bool m_enableLimit;
};

static int sampleRevoluteJoint = SampleManager::Register( "Joints", "Revolute", RevoluteJoint::Create );

class WeldJoint : public Sample
{
public:
	explicit WeldJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
			EnableGrid( m_scene, true );
		}

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
			b3CreateHullShape( groundId, &shapeDef, &groundBox.base );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 4.0f, 0.0f };
		bodyDef.gravityScale = 0.0f;
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		b3BoxHull box = b3MakeBoxHull( 0.5f, 1.5f, 0.25f );
		b3CreateHullShape( m_bodyId, &shapeDef, &box.base );

		b3WeldJointDef jointDef = b3DefaultWeldJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = m_bodyId;
		jointDef.base.localFrameA.p = { 0.0f, 6.5f, 0.0f };
		jointDef.base.localFrameB.p = { 0.0f, 1.5f, 0.0f };
		jointDef.base.constraintHertz = 240.0f;
		jointDef.linearHertz = m_linearHertz;
		jointDef.linearDampingRatio = m_linearDampingRatio;
		jointDef.angularHertz = m_angularHertz;
		jointDef.angularDampingRatio = m_angularDampingRatio;
		jointDef.base.drawScale = 2.0f;

		m_jointId = b3CreateWeldJoint( m_worldId, &jointDef );
	}

	void UpdateUI() override
	{
		float height = 150.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_camera->m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 350.0f, height ) );

		ImGui::Begin( "Weld Joint", nullptr, ImGuiWindowFlags_NoResize );

		ImGui::PushItemWidth( 200.0f );

		if ( ImGui::SliderFloat( "Linear Hertz", &m_linearHertz, 0.0f, 10.0f, "%.1f" ) )
		{
			b3WeldJoint_SetLinearHertz( m_jointId, m_linearHertz );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( ImGui::SliderFloat( "Linear Damping", &m_linearDampingRatio, 0.0f, 2.0f, "%.1f" ) )
		{
			b3WeldJoint_SetLinearDampingRatio( m_jointId, m_linearDampingRatio );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( ImGui::SliderFloat( "Angular Hertz", &m_angularHertz, 0.0f, 10.0f, "%.1f" ) )
		{
			b3WeldJoint_SetAngularHertz( m_jointId, m_angularHertz );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( ImGui::SliderFloat( "Angular Damping", &m_angularDampingRatio, 0.0f, 2.0f, "%.1f" ) )
		{
			b3WeldJoint_SetAngularDampingRatio( m_jointId, m_angularDampingRatio );
			b3Joint_WakeBodies( m_jointId );
		}

		ImGui::PopItemWidth();

		ImGui::End();
	}

	void Render() override
	{
		Sample::Render();
	}

	static Sample* Create( SampleContext* context )
	{
		return new WeldJoint( context );
	}

	b3BodyId m_bodyId;
	b3JointId m_jointId;
	float m_linearHertz = 0.0f;
	float m_linearDampingRatio = 0.0f;
	float m_angularHertz = 2.0f;
	float m_angularDampingRatio = 0.7f;
};

static int sampleWeldJoint = SampleManager::Register( "Joints", "Weld", WeldJoint::Create );

class WheelJoint : public Sample
{
public:
	explicit WheelJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 25.0f, 20.0f, 7.0f, { 0.0f, 2.0f, 0.0f } );
			EnableGrid( m_scene, true );
		}

		m_spinSpeed = 0.0f;
		m_maxSpinTorque = 20.0f;
		m_suspensionHertz = 2.0f;
		m_suspensionDampingRatio = 0.7f;
		m_lowerTranslation = -1.0f;
		m_upperTranslation = 1.0f;
		m_enableSuspension = false;
		m_enableSpinMotor = false;
		m_enableSuspensionLimit = false;

		m_enableSteering = false;
		m_steeringHertz = 1.0f;
		m_steeringDampingRatio = 0.7f;
		m_enableSteeringLimit = false;
		m_lowerSteeringDegrees = -45.0f;
		m_upperSteeringDegrees = 45.0f;
		m_maxSteeringTorque = 20.0f;
		m_targetSteeringDegrees = 0.0f;

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
			b3CreateHullShape( groundId, &shapeDef, &groundBox.base );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 2.0f, 0.0f };
		bodyDef.rotation = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisY, b3Vec3_axisZ );
		// bodyDef.gravityScale = 0.0f;
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		b3Hull* hull = b3CreateCylinder( 0.25f, 0.4f, 0.0f, 12 );
		// b3BoxHull box = b3MakeBoxHull( 0.5f, 1.5f, 0.25f );
		b3CreateHullShape( m_bodyId, &shapeDef, hull );
		b3DestroyHull( hull );

		b3WheelJointDef jointDef = b3DefaultWheelJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = m_bodyId;
		jointDef.base.localFrameA.p = { 0.0f, 3.0f, 0.0f };
		jointDef.base.localFrameA.q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisX, b3Vec3_axisY );
		jointDef.base.localFrameB.p = { 0.0f, 0.0f, 0.0f };
		jointDef.base.localFrameB.q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, b3Vec3_axisY );
		jointDef.base.collideConnected = true;
		jointDef.enableSuspensionLimit = m_enableSuspensionLimit;
		jointDef.lowerSuspensionLimit = m_lowerTranslation;
		jointDef.upperSuspensionLimit = m_upperTranslation;
		jointDef.enableSuspensionSpring = m_enableSuspension;
		jointDef.suspensionHertz = m_suspensionHertz;
		jointDef.suspensionDampingRatio = m_suspensionDampingRatio;
		jointDef.enableSpinMotor = m_enableSpinMotor;
		jointDef.maxSpinTorque = m_maxSpinTorque;
		jointDef.spinSpeed = m_spinSpeed;
		jointDef.enableSteering = m_enableSteering;
		jointDef.steeringHertz = m_steeringHertz;
		jointDef.steeringDampingRatio = m_steeringDampingRatio;
		jointDef.targetSteeringAngle = B3_PI / 180.0f * m_targetSteeringDegrees;
		jointDef.maxSteeringTorque = m_maxSteeringTorque;
		jointDef.enableSteeringLimit = m_enableSteeringLimit;
		jointDef.lowerSteeringLimit = B3_PI / 180.0f * m_lowerSteeringDegrees;
		jointDef.upperSteeringLimit = B3_PI / 180.0f * m_upperSteeringDegrees;

		m_jointId = b3CreateWheelJoint( m_worldId, &jointDef );
	}

	void UpdateUI() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 30.0f * fontSize;
		ImGui::SetNextWindowPos( { 1.0f * fontSize, m_camera->m_height - height - 3.0f * fontSize }, ImGuiCond_Once );
		ImGui::SetNextWindowSize( { 25.0f * fontSize, height } );

		ImGui::Begin( "Wheel Joint", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Checkbox( "Suspension Limit", &m_enableSuspensionLimit ) )
		{
			b3WheelJoint_EnableSuspensionLimit( m_jointId, m_enableSuspensionLimit );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableSuspensionLimit )
		{
			if ( ImGui::SliderFloat( "Lower Translation", &m_lowerTranslation, -10.0f, 10.0f, "%.1f" ) )
			{
				m_lowerTranslation = b3MinFloat( m_lowerTranslation, m_upperTranslation );
				b3WheelJoint_SetSuspensionLimits( m_jointId, m_lowerTranslation, m_upperTranslation );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Upper Translation", &m_upperTranslation, -10.0f, 10.0f, "%.1f" ) )
			{
				m_upperTranslation = b3MaxFloat( m_upperTranslation, m_lowerTranslation );
				b3WheelJoint_SetSuspensionLimits( m_jointId, m_lowerTranslation, m_upperTranslation );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		if ( ImGui::Checkbox( "Motor", &m_enableSpinMotor ) )
		{
			b3WheelJoint_EnableSpinMotor( m_jointId, m_enableSpinMotor );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableSpinMotor )
		{
			if ( ImGui::SliderFloat( "Max Torque", &m_maxSpinTorque, 0.0f, 100.0f, "%.0f" ) )
			{
				b3WheelJoint_SetMaxSpinTorque( m_jointId, m_maxSpinTorque );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Speed", &m_spinSpeed, -10.0f, 10.0f, "%.0f" ) )
			{
				b3WheelJoint_SetSpinMotorSpeed( m_jointId, m_spinSpeed );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		if ( ImGui::Checkbox( "Suspension", &m_enableSuspension ) )
		{
			b3WheelJoint_EnableSuspension( m_jointId, m_enableSuspension );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableSuspension )
		{
			if ( ImGui::SliderFloat( "Suspension Hertz", &m_suspensionHertz, 0.0f, 10.0f, "%.1f" ) )
			{
				b3WheelJoint_SetSuspensionHertz( m_jointId, m_suspensionHertz );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Suspension Damping", &m_suspensionDampingRatio, 0.0f, 2.0f, "%.1f" ) )
			{
				b3WheelJoint_SetSuspensionDampingRatio( m_jointId, m_suspensionDampingRatio );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		if ( ImGui::Checkbox( "Steering", &m_enableSteering ) )
		{
			b3WheelJoint_EnableSteering( m_jointId, m_enableSteering );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableSteering )
		{
			if ( ImGui::SliderFloat( "Steering Hertz", &m_steeringHertz, 0.0f, 10.0f, "%.1f" ) )
			{
				b3WheelJoint_SetSteeringHertz( m_jointId, m_steeringHertz );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Steering Damping", &m_steeringDampingRatio, 0.0f, 2.0f, "%.1f" ) )
			{
				b3WheelJoint_SetSuspensionDampingRatio( m_jointId, m_suspensionDampingRatio );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Steering Degrees", &m_targetSteeringDegrees, -90.0f, 90.0f, "%.0f" ) )
			{
				b3WheelJoint_SetTargetSteeringAngle( m_jointId, m_targetSteeringDegrees * B3_PI / 180.0f );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::Checkbox( "Steering Limit", &m_enableSteeringLimit ) )
			{
				b3WheelJoint_EnableSteeringLimit( m_jointId, m_enableSteeringLimit );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( m_enableSteeringLimit )
			{
				if ( ImGui::SliderFloat( "Lower Degrees", &m_lowerSteeringDegrees, -90.0f, 0.0f, "%.0f" ) )
				{
					b3WheelJoint_SetSteeringLimits( m_jointId, B3_PI / 180.0f * m_lowerSteeringDegrees,
													B3_PI / 180.0f * m_upperSteeringDegrees );
					b3Joint_WakeBodies( m_jointId );
				}

				if ( ImGui::SliderFloat( "Upper Degrees", &m_upperSteeringDegrees, 0.0f, 90.0f, "%.0f" ) )
				{
					b3WheelJoint_SetSteeringLimits( m_jointId, B3_PI / 180.0f * m_lowerSteeringDegrees,
													B3_PI / 180.0f * m_upperSteeringDegrees );
					b3Joint_WakeBodies( m_jointId );
				}
			}
		}

		ImGui::End();
	}

	void Render() override
	{
		Sample::Render();

		float angle = b3WheelJoint_GetSteeringAngle( m_jointId );
		DrawTextLine( "steering degrees = %.1f", 180.0f / B3_PI * angle );

		b3Transform transform = b3Transform_identity;
		transform.p.y += 0.05f;
		DrawTransform( m_scene, transform, 2.0f );
	}

	static Sample* Create( SampleContext* context )
	{
		return new WheelJoint( context );
	}

	b3BodyId m_bodyId;
	b3JointId m_jointId;
	float m_spinSpeed;
	float m_maxSpinTorque;
	float m_suspensionHertz;
	float m_suspensionDampingRatio;
	float m_lowerTranslation;
	float m_upperTranslation;
	bool m_enableSuspension;
	bool m_enableSpinMotor;
	bool m_enableSuspensionLimit;
	bool m_enableSteering;
	float m_steeringHertz;
	float m_steeringDampingRatio;
	bool m_enableSteeringLimit;
	float m_lowerSteeringDegrees;
	float m_upperSteeringDegrees;
	float m_maxSteeringTorque;
	float m_targetSteeringDegrees;
};

static int sampleWheelJoint = SampleManager::Register( "Joints", "Wheel", WheelJoint::Create );

class BallAndChain : public Sample
{
public:
	explicit BallAndChain( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 180.0f, 15.0f, 50.0f, { 0.0f, -20.0f, 0.0f } );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundBody = b3CreateBody( m_worldId, &bodyDef );

		float linkRadius = 0.125f;
		float linkExtent = 0.5f;
		b3Capsule capsule = { { -linkExtent, 0.0f, 0.0f }, { linkExtent, 0.0f, 0.0f }, linkRadius };
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		int linkCount = 32;

		bodyDef.type = b3_dynamicBody;
		b3BodyId parent = groundBody;
		b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
		jointDef.base.localFrameA = b3Transform_identity;
		jointDef.base.localFrameB = { { -linkExtent, 0.0f, 0.0f }, b3Quat_identity };
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 10.0f;

		for ( int i = 0; i < linkCount; ++i )
		{
			bodyDef.position = { ( 1.0f + 2.0f * i ) * linkExtent, 0.0f, 0.0f };
			b3BodyId childId = b3CreateBody( m_worldId, &bodyDef );

			b3CreateCapsuleShape( childId, &shapeDef, &capsule );

			jointDef.base.bodyIdA = parent;
			jointDef.base.bodyIdB = childId;
			b3CreateSphericalJoint( m_worldId, &jointDef );

			jointDef.base.localFrameA = { { linkExtent, 0.0f, 0.0f }, b3Quat_identity };
			parent = childId;
		}

		float sphereRadius = 2.0f;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, sphereRadius };

		bodyDef.position = { ( 1.0f + 2.0f * linkCount ) * linkExtent + sphereRadius - linkExtent, 0.0f, 0.0f };

		b3BodyId childId = b3CreateBody( m_worldId, &bodyDef );

		b3CreateSphereShape( childId, &shapeDef, &sphere );
		jointDef.base.bodyIdA = parent;
		jointDef.base.bodyIdB = childId;
		jointDef.base.localFrameB = { { -sphereRadius, 0.0f, 0.0f }, b3Quat_identity };
		b3CreateSphericalJoint( m_worldId, &jointDef );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BallAndChain( context );
	}
};

static int sampleBallAndChain = SampleManager::Register( "Joints", "Ball and Chain", BallAndChain::Create );

class Door : public Sample
{
public:
	explicit Door( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
			EnableGrid( m_scene, true );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			m_groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
			b3CreateHullShape( m_groundId, &shapeDef, &groundBox.base );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 0.0f, 1.5f, 0.0f };
			bodyDef.gravityScale = 2.0f;
			m_doorId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 1000.0f;

			b3BoxHull box = b3MakeBoxHull( 0.75f, 1.5f, 0.1f );
			b3CreateHullShape( m_doorId, &shapeDef, &box.base );
		}

		//{
		//	b3BoxHull cube = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		//	b3BodyDef bodyDef = b3DefaultBodyDef();
		//	bodyDef.name = "cube";
		//	bodyDef.type = b3_dynamicBody;
		//	bodyDef.position = { 0.0f, 2.0f, 0.0f };
		//	bodyDef.gravityScale = 2.0f;
		//	b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		//	b3ShapeDef shapeDef = b3DefaultShapeDef();
		//	b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		//}

		m_magnitude = 50000.0f;
		m_twoJoints = true;
		m_constraintHertz = 120.0f;
		m_constraintDampingRatio = 0.0f;

		CreateJoints();
	}

	void CreateJoints()
	{
		if ( b3Joint_IsValid( m_jointId1 ) )
		{
			b3DestroyJoint( m_jointId1, false );
			m_jointId1 = b3_nullJointId;
		}

		if ( b3Joint_IsValid( m_jointId2 ) )
		{
			b3DestroyJoint( m_jointId2, false );
			m_jointId2 = b3_nullJointId;
		}

		b3Quat axisQuat = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, b3Vec3_axisY );

		{
			b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = m_groundId;
			jointDef.base.bodyIdB = m_doorId;
			jointDef.base.localFrameA.p = { -0.75f, 1.0f, 0.0f };
			jointDef.base.localFrameA.q = axisQuat;
			jointDef.base.localFrameB.p = { -0.75f, -1.5f, 0.0f };
			jointDef.base.localFrameB.q = axisQuat;
			jointDef.base.constraintHertz = m_constraintHertz;
			jointDef.base.constraintDampingRatio = m_constraintDampingRatio;
			jointDef.enableLimit = true;
			jointDef.lowerAngle = B3_DEG_TO_RAD * -90.0f;
			jointDef.upperAngle = B3_DEG_TO_RAD * 90.0f;
			jointDef.enableSpring = true;
			jointDef.hertz = 1.0f;
			jointDef.dampingRatio = 0.5f;
			jointDef.enableMotor = false;
			jointDef.maxMotorTorque = 100.0f;
			jointDef.motorSpeed = 0.0f;
			jointDef.base.drawScale = 2.0f;

			m_jointId1 = b3CreateRevoluteJoint( m_worldId, &jointDef );
		}

		if ( m_twoJoints )
		{
			b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = m_groundId;
			jointDef.base.bodyIdB = m_doorId;
			jointDef.base.localFrameA.p = { -0.75f, 4.0f, 0.0f };
			jointDef.base.localFrameA.q = axisQuat;
			jointDef.base.localFrameB.p = { -0.75f, 1.5f, 0.0f };
			jointDef.base.localFrameB.q = axisQuat;
			jointDef.base.constraintHertz = m_constraintHertz;
			jointDef.base.constraintDampingRatio = m_constraintDampingRatio;
			jointDef.enableLimit = true;
			jointDef.lowerAngle = B3_DEG_TO_RAD * -90.0f;
			jointDef.upperAngle = B3_DEG_TO_RAD * 90.0f;
			jointDef.enableSpring = true;
			jointDef.hertz = 1.0f;
			jointDef.dampingRatio = 0.5f;
			jointDef.enableMotor = false;
			jointDef.maxMotorTorque = 100.0f;
			jointDef.motorSpeed = 0.0f;
			jointDef.base.drawScale = 2.0f;

			m_jointId2 = b3CreateRevoluteJoint( m_worldId, &jointDef );
		}
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 && modifiers == 2 )
		{
			PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );

			b3RayResult result = b3World_CastRayClosest( m_worldId, pickRay.origin, pickRay.translation, b3DefaultQueryFilter() );

			if ( result.hit )
			{
				b3BodyId bodyId = b3Shape_GetBody( result.shapeId );

				b3Vec3 impulse = m_magnitude * b3Normalize( pickRay.translation );
				b3Body_ApplyLinearImpulse( bodyId, impulse, result.point, true );
			}
		}
		else
		{
			Sample::MouseDown( p, button, modifiers );
		}
	}

	void UpdateUI() override
	{
		float height = 220.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_camera->m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 350.0f, height ) );

		ImGui::Begin( "Door", nullptr, ImGuiWindowFlags_NoResize );

		ImGui::PushItemWidth( 200.0f );

		if ( ImGui::Button( "impulse" ) )
		{
			b3Vec3 p = b3Body_GetWorldPoint( m_doorId, { 0.75f, 0.0f, 0.0f } );
			b3Body_ApplyLinearImpulse( m_doorId, { 0.0f, 0.0f, -m_magnitude }, p, true );
			m_translationError1 = 0.0f;
			m_translationError2 = 0.0f;
		}

		ImGui::SliderFloat( "magnitude", &m_magnitude, 1000.0f, 100000.0f, "%.0f" );

		if ( ImGui::Checkbox( "limit", &m_enableLimit ) )
		{
			b3RevoluteJoint_EnableLimit( m_jointId1, m_enableLimit );

			if ( b3Joint_IsValid( m_jointId2 ) )
			{
				b3RevoluteJoint_EnableLimit( m_jointId2, m_enableLimit );
			}
		}

		if ( ImGui::Checkbox( "two joints", &m_twoJoints ) )
		{
			CreateJoints();
		}

		if ( ImGui::SliderFloat( "hertz", &m_constraintHertz, 15.0f, 240.0f, "%.0f" ) )
		{
			b3Joint_SetConstraintTuning( m_jointId1, m_constraintHertz, m_constraintDampingRatio );

			if ( B3_IS_NON_NULL( m_jointId2 ) )
			{
				b3Joint_SetConstraintTuning( m_jointId2, m_constraintHertz, m_constraintDampingRatio );
			}
		}

		if ( ImGui::SliderFloat( "damping", &m_constraintDampingRatio, 0.0f, 10.0f, "%.1f" ) )
		{
			b3Joint_SetConstraintTuning( m_jointId1, m_constraintHertz, m_constraintDampingRatio );

			if ( B3_IS_NON_NULL( m_jointId2 ) )
			{
				b3Joint_SetConstraintTuning( m_jointId2, m_constraintHertz, m_constraintDampingRatio );
			}
		}

		ImGui::PopItemWidth();

		ImGui::End();
	}

	void Step() override
	{
		Sample::Step();

		b3Vec3 p = b3Body_GetWorldPoint( m_doorId, { 0.75f, 0.0f, 0.0f } );
		DrawPoint( m_scene, p, 10.0f, b3_colorDarkKhaki );

		DrawTransform( m_scene, b3Transform_identity, 1.0f );

		float translationError1 = b3Joint_GetLinearSeparation( m_jointId1 );
		m_translationError1 = b3MaxFloat( m_translationError1, translationError1 );
		DrawTextLine( "translation error 1 = %g", m_translationError1 );

		if ( B3_IS_NON_NULL( m_jointId2 ) )
		{
			float translationError2 = b3Joint_GetLinearSeparation( m_jointId2 );
			m_translationError2 = b3MaxFloat( m_translationError2, translationError2 );
			DrawTextLine( "translation error 2 = %g", m_translationError2 );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new Door( context );
	}

	b3BodyId m_groundId;
	b3BodyId m_doorId;
	b3JointId m_jointId1;
	b3JointId m_jointId2;
	float m_magnitude;
	float m_translationError1;
	float m_translationError2;
	float m_constraintHertz;
	float m_constraintDampingRatio;
	bool m_enableLimit;
	bool m_twoJoints;
};

static int sampleDoor = SampleManager::Register( "Joints", "Door", Door::Create );

// A suspension bridge
class Bridge : public Sample
{
public:
	explicit Bridge( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 20.0f, 35.0, { 0.0f, 10.0f, 0.0f } );
		}

		b3BodyId groundId = b3_nullBodyId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			groundId = b3CreateBody( m_worldId, &bodyDef );
		}

		{
			float a = 0.125f;

			b3BoxHull box = b3MakeBoxHull( a, 0.125f, 0.5f );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 20.0f;

			b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
			// b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.constraintHertz = 1000.0f;
			jointDef.enableSpring = true;
			jointDef.hertz = 2.0f;
			jointDef.dampingRatio = 1.0f;

			m_gravityScale = 1.0f;

			float xbase = -160.0f * a;

			b3BodyId prevBodyId = groundId;
			for ( int i = 0; i < m_count; ++i )
			{
				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.type = b3_dynamicBody;
				bodyDef.position = { xbase + a * ( 1.0f + 2.0f * i ), 20.0f };
				bodyDef.linearDamping = 0.1f;
				bodyDef.angularDamping = 0.1f;
				m_bodyIds[i] = b3CreateBody( m_worldId, &bodyDef );
				b3CreateHullShape( m_bodyIds[i], &shapeDef, &box.base );

				{
					b3Vec3 pivot = { xbase + 2.0f * a * i, 20.0f, -0.5f };
					jointDef.base.bodyIdA = prevBodyId;
					jointDef.base.bodyIdB = m_bodyIds[i];
					jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
					jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
					b3CreateSphericalJoint( m_worldId, &jointDef );
					// b3CreateRevoluteJoint( m_worldId, &jointDef );
				}

				{
					b3Vec3 pivot = { xbase + 2.0f * a * i, 20.0f, 0.5f };
					jointDef.base.bodyIdA = prevBodyId;
					jointDef.base.bodyIdB = m_bodyIds[i];
					jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
					jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
					b3CreateSphericalJoint( m_worldId, &jointDef );
					// b3CreateRevoluteJoint( m_worldId, &jointDef );
				}

				prevBodyId = m_bodyIds[i];
			}

			{
				b3Vec3 pivot = { xbase + 2.0f * a * m_count, 20.0f, -0.5f };
				jointDef.base.bodyIdA = prevBodyId;
				jointDef.base.bodyIdB = groundId;
				jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
				jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
				b3CreateSphericalJoint( m_worldId, &jointDef );
				// b3CreateRevoluteJoint( m_worldId, &jointDef );
			}

			{
				b3Vec3 pivot = { xbase + 2.0f * a * m_count, 20.0f, 0.5f };
				jointDef.base.bodyIdA = prevBodyId;
				jointDef.base.bodyIdB = groundId;
				jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
				jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
				b3CreateSphericalJoint( m_worldId, &jointDef );
				// b3CreateRevoluteJoint( m_worldId, &jointDef );
			}
		}
	}

	void UpdateUI() override
	{
		float height = 80.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Bridge", nullptr, ImGuiWindowFlags_NoResize );

		// Slider takes half the window
		ImGui::PushItemWidth( ImGui::GetWindowWidth() * 0.5f );

		if ( ImGui::SliderFloat( "Gravity scale", &m_gravityScale, -1.0f, 1.0f, "%.1f" ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				b3Body_SetGravityScale( m_bodyIds[i], m_gravityScale );
			}
		}

		ImGui::End();
	}

	void Render() override
	{
		Sample::Render();

		DrawGrid( m_scene, 20 );
	}

	static Sample* Create( SampleContext* context )
	{
		return new Bridge( context );
	}

	static constexpr int m_count = 150;
	b3BodyId m_bodyIds[m_count];
	float m_gravityScale;
};

static int sampleBridgeIndex = SampleManager::Register( "Joints", "Bridge", Bridge::Create );

// This test ensures joints work correctly with bodies that have motion locks
class MotionLocks : public Sample
{
public:
	explicit MotionLocks( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 40.0f, { 0.0f, 5.0f, 0.0f } );
			EnableGrid( m_scene, true );
		}

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			groundId = b3CreateBody( m_worldId, &bodyDef );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull box = b3MakeTransformedBoxHull( 20.0f, 1.0f, 20.0f, { { 0.0f, -1.0f, 0.0f }, b3Quat_identity } );
			b3CreateHullShape( groundId, &shapeDef, &box.base );
		}

		m_motionLocks = {};

		for ( int i = 0; i < m_capacity; ++i )
		{
			m_bodyIds[i] = b3_nullBodyId;
		}

		b3Vec3 position = { -12.5f, 10.0, 0.0f };
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.enableSleep = false;

		b3BoxHull box = b3MakeBoxHull( 1.0f, 1.0f, 0.5f );

		m_count = 0;

		float forceThreshold = 20000.0f;
		float torqueThreshold = 10000.0f;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 1.0f;

		// distance joint
		{
			assert( m_count < m_capacity );

			bodyDef.position = position;
			m_bodyIds[m_count] = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( m_bodyIds[m_count], &shapeDef, &box.base );

			float length = 2.0f;
			b3Vec3 pivot1 = { position.x, position.y + 1.0f + length, 0.0f };
			b3Vec3 pivot2 = { position.x, position.y + 1.0f, 0.0f };
			b3DistanceJointDef jointDef = b3DefaultDistanceJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = m_bodyIds[m_count];
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot1 );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot2 );
			jointDef.length = length;
			jointDef.base.forceThreshold = forceThreshold;
			jointDef.base.torqueThreshold = torqueThreshold;
			jointDef.base.collideConnected = true;
			jointDef.base.userData = (void*)(intptr_t)m_count;
			b3CreateDistanceJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++m_count;

		// motor joint
#if 0
		{
			assert( m_bodyCount < e_count );

			bodyDef.position = position;
			m_bodyIds[m_bodyCount] = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( m_bodyIds[m_bodyCount], &shapeDef, &box.base );

			b3MotorJointDef jointDef = b3DefaultMotorJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = m_bodyIds[m_bodyCount];
			jointDef.base.localFrameA.p = position;
			jointDef.maxForce = 1000.0f;
			jointDef.maxTorque = 20.0f;
			jointDef.base.forceThreshold = forceThreshold;
			jointDef.base.torqueThreshold = torqueThreshold;
			jointDef.base.collideConnected = true;
			jointDef.base.userData = (void*)(intptr_t)m_bodyCount;
			b3CreateMotorJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++m_count;
#endif

		// prismatic joint
		{
			assert( m_count < m_capacity );

			bodyDef.position = position;
			m_bodyIds[m_count] = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( m_bodyIds[m_count], &shapeDef, &box.base );

			b3Vec3 pivot = { position.x - 1.0f, position.y, 0.0f };
			b3PrismaticJointDef jointDef = b3DefaultPrismaticJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = m_bodyIds[m_count];
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
			jointDef.base.forceThreshold = forceThreshold;
			jointDef.base.torqueThreshold = torqueThreshold;
			jointDef.base.collideConnected = true;
			jointDef.base.userData = (void*)(intptr_t)m_count;
			b3CreatePrismaticJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++m_count;

		// revolute joint
		{
			assert( m_count < m_capacity );

			bodyDef.position = position;
			m_bodyIds[m_count] = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( m_bodyIds[m_count], &shapeDef, &box.base );

			b3Vec3 pivot = { position.x - 1.0f, position.y, 0.0f };
			b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = m_bodyIds[m_count];
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
			jointDef.base.forceThreshold = forceThreshold;
			jointDef.base.torqueThreshold = torqueThreshold;
			jointDef.base.collideConnected = true;
			jointDef.base.userData = (void*)(intptr_t)m_count;
			b3CreateRevoluteJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++m_count;

		// weld joint
		{
			assert( m_count < m_capacity );

			bodyDef.position = position;
			m_bodyIds[m_count] = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( m_bodyIds[m_count], &shapeDef, &box.base );

			b3Vec3 pivot = { position.x - 1.0f, position.y, 0.0f };
			b3WeldJointDef jointDef = b3DefaultWeldJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = m_bodyIds[m_count];
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
			jointDef.angularHertz = 2.0f;
			jointDef.angularDampingRatio = 0.5f;
			jointDef.base.forceThreshold = forceThreshold;
			jointDef.base.torqueThreshold = torqueThreshold;
			jointDef.base.collideConnected = true;
			jointDef.base.userData = (void*)(intptr_t)m_count;
			b3CreateWeldJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++m_count;

		// wheel joint
#if 0
		{
			assert( index < e_count );

			bodyDef.position = position;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );

			b3Vec3 pivot = { position.x - 1.0f, position.y, 0.0f };
			b3WheelJointDef jointDef = b3DefaultWheelJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
			jointDef.hertz = 1.0f;
			jointDef.dampingRatio = 0.7f;
			jointDef.lowerTranslation = -1.0f;
			jointDef.upperTranslation = 1.0f;
			jointDef.enableLimit = true;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 10.0f;
			jointDef.motorSpeed = 1.0f;
			jointDef.base.forceThreshold = forceThreshold;
			jointDef.base.torqueThreshold = torqueThreshold;
			jointDef.base.collideConnected = true;
			jointDef.base.userData = (void*)(intptr_t)index;
			b3CreateWheelJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++m_count;
#endif
	}

	void UpdateUI() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 13.0f * fontSize;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->m_height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 10.0f * fontSize, height ) );

		ImGui::Begin( "Motion Locks", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Checkbox( "Lock Linear X", &m_motionLocks.linearX ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				b3Body_SetMotionLocks( m_bodyIds[i], m_motionLocks );
				b3Body_SetAwake( m_bodyIds[i], true );
			}
		}

		if ( ImGui::Checkbox( "Lock Linear Y", &m_motionLocks.linearY ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				b3Body_SetMotionLocks( m_bodyIds[i], m_motionLocks );
				b3Body_SetAwake( m_bodyIds[i], true );
			}
		}

		if ( ImGui::Checkbox( "Lock Linear Z", &m_motionLocks.linearZ ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				b3Body_SetMotionLocks( m_bodyIds[i], m_motionLocks );
				b3Body_SetAwake( m_bodyIds[i], true );
			}
		}

		if ( ImGui::Checkbox( "Lock Angular X", &m_motionLocks.angularX ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				b3Body_SetMotionLocks( m_bodyIds[i], m_motionLocks );
				b3Body_SetAwake( m_bodyIds[i], true );
			}
		}

		if ( ImGui::Checkbox( "Lock Angular Y", &m_motionLocks.angularY ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				b3Body_SetMotionLocks( m_bodyIds[i], m_motionLocks );
				b3Body_SetAwake( m_bodyIds[i], true );
			}
		}

		if ( ImGui::Checkbox( "Lock Angular Z", &m_motionLocks.angularZ ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				b3Body_SetMotionLocks( m_bodyIds[i], m_motionLocks );
				b3Body_SetAwake( m_bodyIds[i], true );
			}
		}

		ImGui::End();

		if ( IsKeyDown( KEY_L ) )
		{
			b3Body_ApplyLinearImpulseToCenter( m_bodyIds[0], { 100.0f, 0.0f }, true );
		}
	}

	void Render() override
	{
		Sample::Render();

		b3Transform transform = { { 0.0f, 0.1f, 0.0f }, b3Quat_identity };
		DrawTransform( m_scene, transform, 1.0f );
	}

	static Sample* Create( SampleContext* context )
	{
		return new MotionLocks( context );
	}

	static constexpr int m_capacity = 6;
	b3BodyId m_bodyIds[m_capacity];
	int m_count;
	b3MotionLocks m_motionLocks;
};

static int sampleMotionLocks = SampleManager::Register( "Joints", "Motion Locks", MotionLocks::Create );

class Driving : public Sample
{
public:
	explicit Driving( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 25.0f, 20.0f, 7.0f, { 0.0f, 2.0f, 0.0f } );
		}

		m_spinSpeed = 30.0f;
		m_maxSpinTorque = 5.0f;
		m_suspensionHertz = 4.0f;
		m_suspensionDampingRatio = 0.7f;
		m_lowerTranslation = -0.2f;
		m_upperTranslation = 0.2f;
		m_steeringHertz = 10.0f;
		m_steeringDampingRatio = 0.7f;
		m_lowerSteeringDegrees = -45.0f;
		m_upperSteeringDegrees = 45.0f;
		m_maxSteeringTorque = 5.0f;
		m_targetSteeringDegrees = 0.0f;

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			// bodyDef.position = { 0.0f, -1.0f, 0.0f };
			bodyDef.position = { -20.0f, 0.0f, -20.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			// b3BoxHull groundBox = b3MakeBoxHull( 200.0f, 1.0f, 200.0f );
			// b3CreateHullShape( groundId, &shapeDef, &groundBox.base );

			m_heightField = b3CreateWave( 50.0f, 50.0f, { 4.0f, 2.0f, 4.0f }, 0.02f, 0.04f, false );
			// b3ShapeDef shapeDef = b3DefaultShapeDef();
			// b3SurfaceMaterial materials[3] = b3DefaultSurfaceMaterial();
			// materials[0] = { .friction = 0.6f, .restitution = 0.0f, 0 };
			// materials[1] = { .friction = 0.6f, .restitution = 1.0f, 1 };
			// materials[2] = { .friction = 0.6f, .restitution = 0.0f, 2 };
			// shapeDef.materials = materials;
			// shapeDef.materialCount = 3;

			b3CreateHeightFieldShape( groundId, &shapeDef, m_heightField );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		{
			bodyDef.position = { 0.0f, 2.5f, 0.0f };
			bodyDef.type = b3_dynamicBody;
			m_chassisId = b3CreateBody( m_worldId, &bodyDef );

			shapeDef.density = 0.5f;
			b3BoxHull box = b3MakeBoxHull( 2.0f, 0.5f, 1.0f );
			b3CreateHullShape( m_chassisId, &shapeDef, &box.base );
		}

		// Keep vehicle upright
		{
			b3ParallelJointDef parallelJointDef = b3DefaultParallelJointDef();
			parallelJointDef.base.bodyIdA = groundId;
			parallelJointDef.base.bodyIdB = m_chassisId;
			parallelJointDef.base.localFrameA.q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, b3Vec3_axisY );
			parallelJointDef.base.localFrameB.q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, b3Vec3_axisY );
			parallelJointDef.base.drawScale = 2.0f;
			parallelJointDef.base.collideConnected = true;
			parallelJointDef.hertz = 0.5f;
			parallelJointDef.dampingRatio = 1.0f;
			b3CreateParallelJoint( m_worldId, &parallelJointDef );
		}

		shapeDef.density = 2.0f;
		shapeDef.baseMaterial.friction = 3.0f;

		bodyDef.type = b3_dynamicBody;
		bodyDef.allowFastRotation = true;
		bodyDef.rotation = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisY, b3Vec3_axisZ );

		// b3Hull* hull = b3CreateCylinder( 0.25f, 0.4f, 0.0f, 16 );

		b3WheelJointDef jointDef = b3DefaultWheelJointDef();
		jointDef.base.bodyIdA = m_chassisId;
		jointDef.base.localFrameA.q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisX, b3Vec3_axisY );
		jointDef.base.localFrameB.q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, b3Vec3_axisY );
		jointDef.enableSuspensionLimit = true;
		jointDef.lowerSuspensionLimit = m_lowerTranslation;
		jointDef.upperSuspensionLimit = m_upperTranslation;
		jointDef.enableSuspensionSpring = true;
		jointDef.suspensionHertz = m_suspensionHertz;
		jointDef.suspensionDampingRatio = m_suspensionDampingRatio;
		jointDef.enableSpinMotor = true;
		jointDef.maxSpinTorque = m_maxSpinTorque;
		jointDef.enableSteering = true;
		jointDef.steeringHertz = m_steeringHertz;
		jointDef.steeringDampingRatio = m_steeringDampingRatio;
		jointDef.targetSteeringAngle = 0.0f;
		jointDef.maxSteeringTorque = m_maxSteeringTorque;
		jointDef.enableSteeringLimit = true;
		jointDef.lowerSteeringLimit = B3_PI / 180.0f * m_lowerSteeringDegrees;
		jointDef.upperSteeringLimit = B3_PI / 180.0f * m_upperSteeringDegrees;

		b3Sphere sphere = { b3Vec3_zero, 0.4f };

		{
			bodyDef.position = { 1.5f, 2.0f, 0.8f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			// b3CreateHullShape( bodyId, &shapeDef, hull );

			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = { 1.5f, -0.5f, 0.8f };
			jointDef.enableSteering = true;
			jointDef.enableSpinMotor = false;
			m_frontLeftId = b3CreateWheelJoint( m_worldId, &jointDef );
		}

		{
			bodyDef.position = { 1.5f, 2.0f, -0.8f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			// b3CreateHullShape( bodyId, &shapeDef, hull );

			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = { 1.5f, -0.5f, -0.8f };
			jointDef.enableSteering = true;
			jointDef.enableSpinMotor = false;
			m_frontRightId = b3CreateWheelJoint( m_worldId, &jointDef );
		}

		{
			bodyDef.position = { -1.5f, 2.0f, 0.8f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			// b3CreateHullShape( bodyId, &shapeDef, hull );

			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = { -1.5f, -0.5f, 0.8f };
			jointDef.enableSteering = false;
			jointDef.enableSpinMotor = true;
			m_rearLeftId = b3CreateWheelJoint( m_worldId, &jointDef );
		}

		{
			bodyDef.position = { -1.5f, 2.0f, -0.8f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			// b3CreateHullShape( bodyId, &shapeDef, hull );

			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = { -1.5f, -0.5f, -0.8f };
			jointDef.enableSteering = false;
			jointDef.enableSpinMotor = true;
			m_rearRightId = b3CreateWheelJoint( m_worldId, &jointDef );
		}

		// b3DestroyHull( hull );

		m_camera->m_thirdPerson = true;
		sapp_lock_mouse( true );

		m_haveMouseLast = false;
		m_mouseLast = { 0.0f, 0.0f };
		m_mouseDelta = { 0.0f, 0.0f };
	}

	~Driving() override
	{
		m_camera->m_thirdPerson = false;
		sapp_lock_mouse( false );
		b3DestroyHeightField( m_heightField );
	}

#if 0
	void MouseMove( b3Vec2 p ) override
	{
		if ( m_camera->m_thirdPerson )
		{
			if ( m_haveMouseLast == false )
			{
				m_mouseLast = p;
				m_haveMouseLast = true;
			}
			else
			{
				m_mouseDelta = { p.x - m_mouseLast.x, p.y - m_mouseLast.y };
				m_mouseLast = p;
			}

			const float sensitivity = 0.05f;
			m_camera->m_yaw -= 2.0f * sensitivity * m_mouseDelta.x;
			m_camera->m_pitch += sensitivity * m_mouseDelta.y;
			m_camera->m_pitch = b3ClampFloat( m_camera->m_pitch, -85.0f, 85.0f );
		}

		Sample::MouseMove( p );
	}
#endif

	void Keyboard( int key, int action, int mods ) override
	{
		if ( key == KEY_T && action == ACTION_PRESS )
		{
			ToggleThirdPerson();
		}
	}

	void UpdateUI() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 22.0f * fontSize;
		ImGui::SetNextWindowPos( { 1.0f * fontSize, m_camera->m_height - height - 3.0f * fontSize }, ImGuiCond_Once );
		ImGui::SetNextWindowSize( { 25.0f * fontSize, height } );

		ImGui::Begin( "Driving", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::SliderFloat( "Lower Translation", &m_lowerTranslation, -10.0f, 10.0f, "%.1f" ) )
		{
			m_lowerTranslation = b3MinFloat( m_lowerTranslation, m_upperTranslation );
			b3WheelJoint_SetSuspensionLimits( m_frontLeftId, m_lowerTranslation, m_upperTranslation );
			b3WheelJoint_SetSuspensionLimits( m_frontRightId, m_lowerTranslation, m_upperTranslation );
			b3WheelJoint_SetSuspensionLimits( m_rearLeftId, m_lowerTranslation, m_upperTranslation );
			b3WheelJoint_SetSuspensionLimits( m_rearRightId, m_lowerTranslation, m_upperTranslation );
		}

		if ( ImGui::SliderFloat( "Upper Translation", &m_upperTranslation, -10.0f, 10.0f, "%.1f" ) )
		{
			m_upperTranslation = b3MaxFloat( m_upperTranslation, m_lowerTranslation );
			b3WheelJoint_SetSuspensionLimits( m_frontLeftId, m_lowerTranslation, m_upperTranslation );
			b3WheelJoint_SetSuspensionLimits( m_frontRightId, m_lowerTranslation, m_upperTranslation );
			b3WheelJoint_SetSuspensionLimits( m_rearLeftId, m_lowerTranslation, m_upperTranslation );
			b3WheelJoint_SetSuspensionLimits( m_rearRightId, m_lowerTranslation, m_upperTranslation );
		}

		ImGui::SliderFloat( "Max Torque", &m_maxSpinTorque, 0.0f, 100.0f, "%.0f" );
		ImGui::SliderFloat( "Speed", &m_spinSpeed, 0.0f, 100.0f, "%.0f" );

		if ( ImGui::SliderFloat( "Suspension Hertz", &m_suspensionHertz, 0.0f, 10.0f, "%.1f" ) )
		{
			b3WheelJoint_SetSuspensionHertz( m_frontLeftId, m_suspensionHertz );
			b3WheelJoint_SetSuspensionHertz( m_frontRightId, m_suspensionHertz );
			b3WheelJoint_SetSuspensionHertz( m_rearLeftId, m_suspensionHertz );
			b3WheelJoint_SetSuspensionHertz( m_rearRightId, m_suspensionHertz );
		}

		if ( ImGui::SliderFloat( "Suspension Damping", &m_suspensionDampingRatio, 0.0f, 2.0f, "%.1f" ) )
		{
			b3WheelJoint_SetSuspensionDampingRatio( m_frontLeftId, m_suspensionDampingRatio );
			b3WheelJoint_SetSuspensionDampingRatio( m_frontRightId, m_suspensionDampingRatio );
			b3WheelJoint_SetSuspensionDampingRatio( m_rearLeftId, m_suspensionDampingRatio );
			b3WheelJoint_SetSuspensionDampingRatio( m_rearRightId, m_suspensionDampingRatio );
		}

		if ( ImGui::SliderFloat( "Steering Hertz", &m_steeringHertz, 0.0f, 10.0f, "%.1f" ) )
		{
			b3WheelJoint_SetSteeringHertz( m_frontLeftId, m_steeringHertz );
			b3WheelJoint_SetSteeringHertz( m_frontRightId, m_steeringHertz );
		}

		if ( ImGui::SliderFloat( "Steering Damping", &m_steeringDampingRatio, 0.0f, 2.0f, "%.1f" ) )
		{
			b3WheelJoint_SetSteeringDampingRatio( m_frontLeftId, m_steeringDampingRatio );
			b3WheelJoint_SetSteeringDampingRatio( m_frontRightId, m_steeringDampingRatio );
		}

		if ( ImGui::SliderFloat( "Steering Torque", &m_maxSteeringTorque, 0.0f, 20.0f, "%.1f" ) )
		{
			b3WheelJoint_SetMaxSteeringTorque( m_frontLeftId, m_maxSteeringTorque );
			b3WheelJoint_SetMaxSteeringTorque( m_frontRightId, m_maxSteeringTorque );
		}

		if ( ImGui::SliderFloat( "Lower Degrees", &m_lowerSteeringDegrees, -90.0f, 0.0f, "%.0f" ) )
		{
			b3WheelJoint_SetSteeringLimits( m_frontLeftId, B3_PI / 180.0f * m_lowerSteeringDegrees,
											B3_PI / 180.0f * m_upperSteeringDegrees );
			b3WheelJoint_SetSteeringLimits( m_frontRightId, B3_PI / 180.0f * m_lowerSteeringDegrees,
											B3_PI / 180.0f * m_upperSteeringDegrees );
		}

		if ( ImGui::SliderFloat( "Upper Degrees", &m_upperSteeringDegrees, 0.0f, 90.0f, "%.0f" ) )
		{
			b3WheelJoint_SetSteeringLimits( m_frontLeftId, B3_PI / 180.0f * m_lowerSteeringDegrees,
											B3_PI / 180.0f * m_upperSteeringDegrees );
			b3WheelJoint_SetSteeringLimits( m_frontRightId, B3_PI / 180.0f * m_lowerSteeringDegrees,
											B3_PI / 180.0f * m_upperSteeringDegrees );
		}

		bool thirdPerson = m_camera->m_thirdPerson;
		if ( ImGui::Checkbox( "Third Person (Key: T)", &thirdPerson ) )
		{
			ToggleThirdPerson();
		}

		ImGui::End();
	}

	void Render() override
	{
		Sample::Render();

		b3Vec3 velocity = b3Body_GetLinearVelocity( m_chassisId );
		b3Quat quat = b3Body_GetRotation( m_chassisId );
		b3Vec3 forward = b3RotateVector( quat, { -1.0f, 0.0f, 0.0f } );
		float speed = b3Dot( velocity, forward );
		DrawTextLine( "speed = %.1f", speed );

		float leftSpeed = b3WheelJoint_GetSpinSpeed( m_rearLeftId );
		float rightSpeed = b3WheelJoint_GetSpinSpeed( m_rearRightId );
		DrawTextLine( "spin speed = %.1f/%.1f", leftSpeed, rightSpeed );

		float leftSpinTorque = b3WheelJoint_GetSpinTorque( m_rearLeftId );
		float rightSpinTorque = b3WheelJoint_GetSpinTorque( m_rearRightId );
		DrawTextLine( "spin torque = %.1f/%.1f", leftSpinTorque, rightSpinTorque );

		float angleLeft = b3WheelJoint_GetSteeringAngle( m_frontLeftId );
		float angleRight = b3WheelJoint_GetSteeringAngle( m_frontRightId );
		DrawTextLine( "steering degrees = %.1f/%.1f", 180.0f / B3_PI * angleLeft, 180.0f / B3_PI * angleRight );

		float leftSteerTorque = b3WheelJoint_GetSteeringTorque( m_frontLeftId );
		float rightSteerTorque = b3WheelJoint_GetSteeringTorque( m_frontRightId );
		DrawTextLine( "steering torque = %.1f/%.1f", leftSteerTorque, rightSteerTorque );

		b3Transform transform = b3Transform_identity;
		transform.p.y += 0.05f;
		DrawTransform( m_scene, transform, 2.0f );
	}

	void Step() override
	{
		b3Vec2 throttle = { 0.0f, 0.0f };

		if ( m_camera->m_thirdPerson )
		{
			if ( IsKeyDown( KEY_W ) )
			{
				throttle.x += 1.0f;
				b3Body_SetAwake( m_chassisId, true );
			}

			if ( IsKeyDown( KEY_S ) )
			{
				throttle.x -= 1.0f;
				b3Body_SetAwake( m_chassisId, true );
			}

			if ( IsKeyDown( KEY_A ) )
			{
				throttle.y += 1.0f;
				b3Body_SetAwake( m_chassisId, true );
			}

			if ( IsKeyDown( KEY_D ) )
			{
				throttle.y -= 1.0f;
				b3Body_SetAwake( m_chassisId, true );
			}
		}

		float maxSteeringAngle = 0.25f * B3_PI;
		b3WheelJoint_SetTargetSteeringAngle( m_frontLeftId, maxSteeringAngle * throttle.y );
		b3WheelJoint_SetTargetSteeringAngle( m_frontRightId, maxSteeringAngle * throttle.y );

		b3WheelJoint_SetSpinMotorSpeed( m_rearLeftId, -m_spinSpeed * throttle.x );
		b3WheelJoint_SetSpinMotorSpeed( m_rearRightId, -m_spinSpeed * throttle.x );

		if ( m_camera->m_thirdPerson )
		{
			b3Transform transform = b3Body_GetTransform( m_chassisId );
			m_camera->m_pivot = transform.p;
			m_camera->UpdateTransform();
		}

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new Driving( context );
	}

	b3HeightField* m_heightField;
	b3BodyId m_chassisId;
	b3JointId m_frontLeftId;
	b3JointId m_frontRightId;
	b3JointId m_rearLeftId;
	b3JointId m_rearRightId;

	float m_spinSpeed;
	float m_maxSpinTorque;
	float m_suspensionHertz;
	float m_suspensionDampingRatio;
	float m_lowerTranslation;
	float m_upperTranslation;
	float m_steeringHertz;
	float m_steeringDampingRatio;
	float m_lowerSteeringDegrees;
	float m_upperSteeringDegrees;
	float m_maxSteeringTorque;
	float m_targetSteeringDegrees;

	b3Vec2 m_mouseLast;
	b3Vec2 m_mouseDelta;
	bool m_haveMouseLast;
};

static int sampleDriving = SampleManager::Register( "Joints", "Driving", Driving::Create );
