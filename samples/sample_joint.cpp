// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "earcut.h"
#include "gfx/draw.h"
#include "gfx/keycodes.h"
#include "imgui.h"
#include "sample.h"
#include "utils.h"

#include "box3d/box3d.h"

#include <array>
#include <vector>

// Test the distance joint and all options
class DistanceJoint : public Sample
{
public:
	explicit DistanceJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 0.0f, 40.0f, { 0.0f, 10.0f, 0.0f } );
		}

		AddGroundBox( 20.0f );

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

		for ( int i = 0; i < m_maxCount; ++i )
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

			b3Pos pivotA = { m_length * i, yOffset, 0.0f };
			b3Pos pivotB = { m_length * ( i + 1.0f ), yOffset, 0.0f };
			jointDef.base.bodyIdA = prevBodyId;
			jointDef.base.bodyIdB = m_bodyIds[i];
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( prevBodyId, pivotA );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( m_bodyIds[i], pivotB );
			m_jointIds[i] = b3CreateDistanceJoint( m_worldId, &jointDef );

			prevBodyId = m_bodyIds[i];
		}
	}

	bool DrawControls() override
	{
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
			if ( ImGui::SliderFloat( "Tension##Spring", &m_tensionForce, 0.0f, 4000.0f ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b3DistanceJoint_SetSpringForceRange( m_jointIds[i], -m_tensionForce, m_compressionForce );
					b3Joint_WakeBodies( m_jointIds[i] );
				}
			}

			if ( ImGui::SliderFloat( "Compression##Spring", &m_compressionForce, 0.0f, 200.0f ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b3DistanceJoint_SetSpringForceRange( m_jointIds[i], -m_tensionForce, m_compressionForce );
					b3Joint_WakeBodies( m_jointIds[i] );
				}
			}

			if ( ImGui::SliderFloat( "Hertz##Spring", &m_hertz, 0.0f, 15.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b3DistanceJoint_SetSpringHertz( m_jointIds[i], m_hertz );
					b3Joint_WakeBodies( m_jointIds[i] );
				}
			}

			if ( ImGui::SliderFloat( "Damping##Spring", &m_dampingRatio, 0.0f, 4.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b3DistanceJoint_SetSpringDampingRatio( m_jointIds[i], m_dampingRatio );
					b3Joint_WakeBodies( m_jointIds[i] );
				}
			}

			ImGui::Separator();
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
			if ( ImGui::SliderFloat( "Min##Limit", &m_minLength, 0.1f, 4.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b3DistanceJoint_SetLengthRange( m_jointIds[i], m_minLength, m_maxLength );
					b3Joint_WakeBodies( m_jointIds[i] );
				}
			}

			if ( ImGui::SliderFloat( "Max##Limit", &m_maxLength, 0.1f, 4.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b3DistanceJoint_SetLengthRange( m_jointIds[i], m_minLength, m_maxLength );
					b3Joint_WakeBodies( m_jointIds[i] );
				}
			}

			ImGui::Separator();
		}

		int count = m_count;
		if ( ImGui::SliderInt( "Count", &count, 1, m_maxCount ) )
		{
			CreateScene( count );
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new DistanceJoint( context );
	}

	static constexpr int m_maxCount = 20;

	b3BodyId m_groundId;
	b3BodyId m_bodyIds[m_maxCount];
	b3JointId m_jointIds[m_maxCount];
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

static int sampleDistanceJoint = RegisterSample( "Joints", "Distance Joint", DistanceJoint::Create );

class FilterJoint : public Sample
{
public:
	explicit FilterJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
		}

		AddGroundBox( 20.0f );

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

	static Sample* Create( SampleContext* context )
	{
		return new FilterJoint( context );
	}
};

static int sampleFilterJoint = RegisterSample( "Joints", "Filter", FilterJoint::Create );

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
			m_camera->SetView( 0.0f, 0.0f, 25.0f, { 0.0f, 8.0f, 0.0f } );
		}

		AddGroundBox( 20.0f );

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position.y = -1.0f;
			groundId = b3CreateBody( m_worldId, &bodyDef );
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
			bodyDef.position = { -2.0, 2.0, 0.0 };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( bodyId, &shapeDef, &box.base );

			b3MotorJointDef jointDef = b3DefaultMotorJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = { -1.75f, 3.25f, 0.0f };
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

	bool DrawControls() override
	{
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

		return true;
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

			b3Pos linearOffset;
			linearOffset.x = 6.0f * sinf( 2.0f * m_time );
			linearOffset.y = 10.0f + 4.0f * sinf( 1.0f * m_time );
			linearOffset.z = 0.0f;

			float angularOffset = 2.0f * m_time;
			m_transform = { linearOffset, b3MakeQuatFromAxisAngle( b3Vec3_axisZ, angularOffset ) };

			b3Body_SetTargetTransform( m_targetId, m_transform, timeStep, true );
		}

		DrawAxes( m_transform, 1.0f );

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
	b3WorldTransform m_transform;
	float m_time;
	float m_speed;
	float m_maxForce;
	float m_maxTorque;
};

static int sampleMotorJoint = RegisterSample( "Joints", "Motor Joint", MotorJoint::Create );

class TopDownFriction : public Sample
{
public:
	explicit TopDownFriction( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 0.0f, 26.0f, { 0.0f, 10.0f, 0.0f } );
		}

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			groundId = b3CreateBody( m_worldId, &bodyDef );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull box = b3MakeTransformedBoxHull( 10.0f, 0.5f, 4.0f, { b3Vec3_zero, b3Quat_identity } );
			b3CreateHullShape( groundId, &shapeDef, &box.base );

			box = b3MakeTransformedBoxHull( 0.5f, 10.0f, 4.0f, { { -10.0f, 10.0f, 0.0f }, b3Quat_identity } );
			b3CreateHullShape( groundId, &shapeDef, &box.base );

			box = b3MakeTransformedBoxHull( 0.5f, 10.0f, 4.0f, { { 10.0f, 10.0f, 0.0f }, b3Quat_identity } );
			b3CreateHullShape( groundId, &shapeDef, &box.base );

			box = b3MakeTransformedBoxHull( 10.0f, 0.5f, 4.0f, { { 00.0f, 20.0f, 0.0f }, b3Quat_identity } );
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

	bool DrawControls() override
	{
		if ( ImGui::Button( "Explode" ) )
		{
			b3Sphere sphere = { { 0.0f, 10.0f, 0.0 }, 10.0f };
			b3ExplosionDef def = b3DefaultExplosionDef();
			def.position = { 0.0, 10.0, 0.0 };
			def.radius = sphere.radius;
			def.falloff = 5.0f;
			def.impulsePerArea = 10000.0f;
			b3World_Explode( m_worldId, &def );

			DrawSolidSphere( b3WorldTransform_identity, sphere, MakeColor( b3_colorWhite ) );
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new TopDownFriction( context );
	}
};

static int sampleTopDownFriction = RegisterSample( "Joints", "Top Down Friction", TopDownFriction::Create );

class PrismaticJoint : public Sample
{
public:
	explicit PrismaticJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
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

		AddGroundBox( 20.0f );

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );
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

	bool DrawControls() override
	{
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

		ImGui::Separator();

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

		ImGui::Separator();

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

		return true;
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

static int samplePrismaticJoint = RegisterSample( "Joints", "Prismatic", PrismaticJoint::Create );

class SphericalJoint : public Sample
{
public:
	explicit SphericalJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
		}

		AddGroundBox( 20.0f );

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 4.0f, 0.0f };
		bodyDef.gravityScale = 0.0f;
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 100.0f;

		b3BoxHull box = b3MakeBoxHull( 0.5f, 1.5f, 0.25f );
		b3CreateHullShape( m_bodyId, &shapeDef, &box.base );

		b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = m_bodyId;
		jointDef.base.drawScale = 2.0f;
		jointDef.base.localFrameA.p = { 0.0f, 6.5f, 0.0f };
		jointDef.base.localFrameB.p = { 0.0f, 1.5f, 0.0f };
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

	bool DrawControls() override
	{
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

		ImGui::Separator();

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

		ImGui::Separator();

		if ( ImGui::Checkbox( "Motor", &m_enableMotor ) )
		{
			b3SphericalJoint_EnableMotor( m_jointId, m_enableMotor );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableMotor )
		{
			if ( ImGui::SliderFloat( "Max Torque", &m_motorTorque, 0.0f, 10000.0f, "%.0f" ) )
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

		ImGui::Separator();

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

		return true;
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

static int sampleSphericalJoint = RegisterSample( "Joints", "Spherical", SphericalJoint::Create );

class ParallelJoint : public Sample
{
public:
	explicit ParallelJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
		}

		m_hertz = 10.0f;
		m_dampingRatio = 0.7f;
		m_maxTorque = 5000.0f;

		AddGroundBox( 20.0f );

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );
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

	bool DrawControls() override
	{
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

		return true;
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

static int sampleParallelJoint = RegisterSample( "Joints", "Parallel Spring", ParallelJoint::Create );

class RevoluteJoint : public Sample
{
public:
	explicit RevoluteJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
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

		AddGroundBox( 20.0f );

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );
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

	bool DrawControls() override
	{
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

		ImGui::Separator();

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

		ImGui::Separator();

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

		return true;
	}

	void Render() override
	{
		Sample::Render();

		b3MassData massData = b3Body_GetMassData( m_bodyId );
		b3Vec3 angularVelocity = b3Body_GetAngularVelocity( m_bodyId );
		b3Vec3 linearVelocity = b3Body_GetLinearVelocity( m_bodyId );
		float kineticEnergy = 0.5f * b3Dot( angularVelocity, b3MulMV( massData.inertia, angularVelocity ) );
		kineticEnergy += 0.5f * massData.mass * b3Dot( linearVelocity, linearVelocity );
		b3Pos center = b3Body_GetWorldCenter( m_bodyId );
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

static int sampleRevoluteJoint = RegisterSample( "Joints", "Revolute", RevoluteJoint::Create );

class WeldJoint : public Sample
{
public:
	explicit WeldJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
		}

		AddGroundBox( 20.0f );

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );
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

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

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

		return true;
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

static int sampleWeldJoint = RegisterSample( "Joints", "Weld", WeldJoint::Create );

class WheelJoint : public Sample
{
public:
	explicit WheelJoint( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 25.0f, 20.0f, 7.0f, { 0.0f, 2.0f, 0.0f } );
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

		AddGroundBox( 20.0f );

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 2.0f, 0.0f };
		bodyDef.rotation = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisY, b3Vec3_axisZ );
		// bodyDef.gravityScale = 0.0f;
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		b3HullData* hull = b3CreateCylinder( 0.25f, 0.4f, 0.0f, 12 );
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

	bool DrawControls() override
	{
		if ( ImGui::Checkbox( "Suspension Limit", &m_enableSuspensionLimit ) )
		{
			b3WheelJoint_EnableSuspensionLimit( m_jointId, m_enableSuspensionLimit );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableSuspensionLimit )
		{
			if ( ImGui::SliderFloat( "Min##Limit", &m_lowerTranslation, -10.0f, 10.0f, "%.1f" ) )
			{
				m_lowerTranslation = b3MinFloat( m_lowerTranslation, m_upperTranslation );
				b3WheelJoint_SetSuspensionLimits( m_jointId, m_lowerTranslation, m_upperTranslation );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Max##Limit", &m_upperTranslation, -10.0f, 10.0f, "%.1f" ) )
			{
				m_upperTranslation = b3MaxFloat( m_upperTranslation, m_lowerTranslation );
				b3WheelJoint_SetSuspensionLimits( m_jointId, m_lowerTranslation, m_upperTranslation );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		ImGui::Separator();

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

		ImGui::Separator();

		if ( ImGui::Checkbox( "Suspension Spring", &m_enableSuspension ) )
		{
			b3WheelJoint_EnableSuspension( m_jointId, m_enableSuspension );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableSuspension )
		{
			if ( ImGui::SliderFloat( "Hertz##Suspension", &m_suspensionHertz, 0.0f, 10.0f, "%.1f" ) )
			{
				b3WheelJoint_SetSuspensionHertz( m_jointId, m_suspensionHertz );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Damping##Suspension", &m_suspensionDampingRatio, 0.0f, 2.0f, "%.1f" ) )
			{
				b3WheelJoint_SetSuspensionDampingRatio( m_jointId, m_suspensionDampingRatio );
				b3Joint_WakeBodies( m_jointId );
			}
		}

		ImGui::Separator();

		if ( ImGui::Checkbox( "Steering", &m_enableSteering ) )
		{
			b3WheelJoint_EnableSteering( m_jointId, m_enableSteering );
			b3Joint_WakeBodies( m_jointId );
		}

		if ( m_enableSteering )
		{
			if ( ImGui::SliderFloat( "Hertz##Steering", &m_steeringHertz, 0.0f, 10.0f, "%.1f" ) )
			{
				b3WheelJoint_SetSteeringHertz( m_jointId, m_steeringHertz );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Damping##Steering", &m_steeringDampingRatio, 0.0f, 2.0f, "%.1f" ) )
			{
				b3WheelJoint_SetSuspensionDampingRatio( m_jointId, m_suspensionDampingRatio );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Degrees##Steering", &m_targetSteeringDegrees, -90.0f, 90.0f, "%.0f" ) )
			{
				b3WheelJoint_SetTargetSteeringAngle( m_jointId, m_targetSteeringDegrees * B3_PI / 180.0f );
				b3Joint_WakeBodies( m_jointId );
			}

			ImGui::Separator();

			if ( ImGui::Checkbox( "Steering Limit", &m_enableSteeringLimit ) )
			{
				b3WheelJoint_EnableSteeringLimit( m_jointId, m_enableSteeringLimit );
				b3Joint_WakeBodies( m_jointId );
			}

			if ( m_enableSteeringLimit )
			{
				if ( ImGui::SliderFloat( "Min Degrees", &m_lowerSteeringDegrees, -90.0f, 0.0f, "%.0f" ) )
				{
					b3WheelJoint_SetSteeringLimits( m_jointId, B3_PI / 180.0f * m_lowerSteeringDegrees,
													B3_PI / 180.0f * m_upperSteeringDegrees );
					b3Joint_WakeBodies( m_jointId );
				}

				if ( ImGui::SliderFloat( "Max Degrees", &m_upperSteeringDegrees, 0.0f, 90.0f, "%.0f" ) )
				{
					b3WheelJoint_SetSteeringLimits( m_jointId, B3_PI / 180.0f * m_lowerSteeringDegrees,
													B3_PI / 180.0f * m_upperSteeringDegrees );
					b3Joint_WakeBodies( m_jointId );
				}
			}
		}

		return true;
	}

	void Render() override
	{
		Sample::Render();

		float angle = b3WheelJoint_GetSteeringAngle( m_jointId );
		DrawTextLine( "steering degrees = %.1f", 180.0f / B3_PI * angle );
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

static int sampleWheelJoint = RegisterSample( "Joints", "Wheel", WheelJoint::Create );

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

static int sampleBallAndChain = RegisterSample( "Joints", "Ball and Chain", BallAndChain::Create );

class Door : public Sample
{
public:
	explicit Door( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
		}

		m_groundId = AddGroundBox( 20.0f );

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

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		if ( ImGui::Button( "Impulse##Door" ) )
		{
			b3Pos p = b3Body_GetWorldPoint( m_doorId, { 0.75f, 0.0f, 0.0f } );
			b3Body_ApplyLinearImpulse( m_doorId, { 0.0f, 0.0f, -m_magnitude }, p, true );
			m_translationError1 = 0.0f;
			m_translationError2 = 0.0f;
		}

		ImGui::SliderFloat( "Magnitude##Door", &m_magnitude, 1000.0f, 100000.0f, "%.0f" );

		if ( ImGui::Checkbox( "Limit##Door", &m_enableLimit ) )
		{
			b3RevoluteJoint_EnableLimit( m_jointId1, m_enableLimit );

			if ( b3Joint_IsValid( m_jointId2 ) )
			{
				b3RevoluteJoint_EnableLimit( m_jointId2, m_enableLimit );
			}
		}

		if ( ImGui::Checkbox( "Two joints##Door", &m_twoJoints ) )
		{
			CreateJoints();
		}

		if ( ImGui::SliderFloat( "Hertz##Door", &m_constraintHertz, 15.0f, 240.0f, "%.0f" ) )
		{
			b3Joint_SetConstraintTuning( m_jointId1, m_constraintHertz, m_constraintDampingRatio );

			if ( B3_IS_NON_NULL( m_jointId2 ) )
			{
				b3Joint_SetConstraintTuning( m_jointId2, m_constraintHertz, m_constraintDampingRatio );
			}
		}

		if ( ImGui::SliderFloat( "Damping##Door", &m_constraintDampingRatio, 0.0f, 10.0f, "%.1f" ) )
		{
			b3Joint_SetConstraintTuning( m_jointId1, m_constraintHertz, m_constraintDampingRatio );

			if ( B3_IS_NON_NULL( m_jointId2 ) )
			{
				b3Joint_SetConstraintTuning( m_jointId2, m_constraintHertz, m_constraintDampingRatio );
			}
		}

		ImGui::PopItemWidth();

		return true;
	}

	void Step() override
	{
		Sample::Step();

		b3Pos p = b3Body_GetWorldPoint( m_doorId, { 0.75f, 0.0f, 0.0f } );
		DrawPoint( p, 10.0f, MakeColor( b3_colorDarkKhaki ) );

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

static int sampleDoor = RegisterSample( "Joints", "Door", Door::Create );

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

		AddGroundBox( 60.0f );

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
					b3Pos pivot = { xbase + 2.0f * a * i, 20.0, -0.5 };
					jointDef.base.bodyIdA = prevBodyId;
					jointDef.base.bodyIdB = m_bodyIds[i];
					jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
					jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
					b3CreateSphericalJoint( m_worldId, &jointDef );
					// b3CreateRevoluteJoint( m_worldId, &jointDef );
				}

				{
					b3Pos pivot = { xbase + 2.0f * a * i, 20.0, 0.5 };
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
				b3Pos pivot = { xbase + 2.0f * a * m_count, 20.0, -0.5 };
				jointDef.base.bodyIdA = prevBodyId;
				jointDef.base.bodyIdB = groundId;
				jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
				jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
				b3CreateSphericalJoint( m_worldId, &jointDef );
				// b3CreateRevoluteJoint( m_worldId, &jointDef );
			}

			{
				b3Pos pivot = { xbase + 2.0f * a * m_count, 20.0, 0.5 };
				jointDef.base.bodyIdA = prevBodyId;
				jointDef.base.bodyIdB = groundId;
				jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
				jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
				b3CreateSphericalJoint( m_worldId, &jointDef );
				// b3CreateRevoluteJoint( m_worldId, &jointDef );
			}
		}
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		if ( ImGui::SliderFloat( "Gravity scale", &m_gravityScale, -1.0f, 1.0f, "%.1f" ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				b3Body_SetGravityScale( m_bodyIds[i], m_gravityScale );
			}
		}

		ImGui::PopItemWidth();
		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new Bridge( context );
	}

	static constexpr int m_count = 150;
	b3BodyId m_bodyIds[m_count];
	float m_gravityScale;
};

static int sampleBridgeIndex = RegisterSample( "Joints", "Bridge", Bridge::Create );

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
		}

		AddGroundBox( 20.0f );

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			groundId = b3CreateBody( m_worldId, &bodyDef );
		}

		m_motionLocks = {};

		for ( int i = 0; i < m_capacity; ++i )
		{
			m_bodyIds[i] = b3_nullBodyId;
		}

		b3Pos position = { -12.5f, 10.0, 0.0f };
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
			b3Pos pivot1 = { position.x, position.y + 1.0f + length, 0.0f };
			b3Pos pivot2 = { position.x, position.y + 1.0f, 0.0f };
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

			b3Pos pivot = { position.x - 1.0f, position.y, 0.0f };
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

			b3Pos pivot = { position.x - 1.0f, position.y, 0.0f };
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

			b3Pos pivot = { position.x - 1.0f, position.y, 0.0f };
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

	bool DrawControls() override
	{
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

		if ( IsKeyDown( KEY_L ) )
		{
			b3Body_ApplyLinearImpulseToCenter( m_bodyIds[0], { 100.0f, 0.0f }, true );
		}

		return true;
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

static int sampleMotionLocks = RegisterSample( "Joints", "Motion Locks", MotionLocks::Create );

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

		// b3HullData* hull = b3CreateCylinder( 0.25f, 0.4f, 0.0f, 16 );

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

		m_camera->m_thirdPerson = false;

		m_haveMouseLast = false;
		m_mouseLast = { 0.0f, 0.0f };
		m_mouseDelta = { 0.0f, 0.0f };
	}

	~Driving() override
	{
		if (m_camera->m_thirdPerson)
		{
			m_camera->m_thirdPerson = false;
			sapp_lock_mouse( false );
		}
		b3DestroyHeightField( m_heightField );
	}

	void Keyboard( int key, int action, int mods ) override
	{
		if ( key == KEY_T && action == ACTION_PRESS )
		{
			ToggleThirdPerson();
		}
	}

	bool DrawControls() override
	{
		ImGui::Text( "Suspension" );
		if ( ImGui::SliderFloat( "Min##Suspension", &m_lowerTranslation, -10.0f, 10.0f, "%.1f" ) )
		{
			m_lowerTranslation = b3MinFloat( m_lowerTranslation, m_upperTranslation );
			b3WheelJoint_SetSuspensionLimits( m_frontLeftId, m_lowerTranslation, m_upperTranslation );
			b3WheelJoint_SetSuspensionLimits( m_frontRightId, m_lowerTranslation, m_upperTranslation );
			b3WheelJoint_SetSuspensionLimits( m_rearLeftId, m_lowerTranslation, m_upperTranslation );
			b3WheelJoint_SetSuspensionLimits( m_rearRightId, m_lowerTranslation, m_upperTranslation );
		}

		if ( ImGui::SliderFloat( "Max##Suspension", &m_upperTranslation, -10.0f, 10.0f, "%.1f" ) )
		{
			m_upperTranslation = b3MaxFloat( m_upperTranslation, m_lowerTranslation );
			b3WheelJoint_SetSuspensionLimits( m_frontLeftId, m_lowerTranslation, m_upperTranslation );
			b3WheelJoint_SetSuspensionLimits( m_frontRightId, m_lowerTranslation, m_upperTranslation );
			b3WheelJoint_SetSuspensionLimits( m_rearLeftId, m_lowerTranslation, m_upperTranslation );
			b3WheelJoint_SetSuspensionLimits( m_rearRightId, m_lowerTranslation, m_upperTranslation );
		}

		if ( ImGui::SliderFloat( "Hertz##Suspension", &m_suspensionHertz, 0.0f, 10.0f, "%.1f" ) )
		{
			b3WheelJoint_SetSuspensionHertz( m_frontLeftId, m_suspensionHertz );
			b3WheelJoint_SetSuspensionHertz( m_frontRightId, m_suspensionHertz );
			b3WheelJoint_SetSuspensionHertz( m_rearLeftId, m_suspensionHertz );
			b3WheelJoint_SetSuspensionHertz( m_rearRightId, m_suspensionHertz );
		}

		if ( ImGui::SliderFloat( "Damping##Suspension", &m_suspensionDampingRatio, 0.0f, 2.0f, "%.1f" ) )
		{
			b3WheelJoint_SetSuspensionDampingRatio( m_frontLeftId, m_suspensionDampingRatio );
			b3WheelJoint_SetSuspensionDampingRatio( m_frontRightId, m_suspensionDampingRatio );
			b3WheelJoint_SetSuspensionDampingRatio( m_rearLeftId, m_suspensionDampingRatio );
			b3WheelJoint_SetSuspensionDampingRatio( m_rearRightId, m_suspensionDampingRatio );
		}

		ImGui::Separator();

		ImGui::Text( "Motor" );

		ImGui::SliderFloat( "Max Torque", &m_maxSpinTorque, 0.0f, 100.0f, "%.0f" );
		ImGui::SliderFloat( "Speed", &m_spinSpeed, 0.0f, 100.0f, "%.0f" );

		ImGui::Separator();

		ImGui::Text( "Steering" );

		if ( ImGui::SliderFloat( "Hertz##Steering", &m_steeringHertz, 0.0f, 10.0f, "%.1f" ) )
		{
			b3WheelJoint_SetSteeringHertz( m_frontLeftId, m_steeringHertz );
			b3WheelJoint_SetSteeringHertz( m_frontRightId, m_steeringHertz );
		}

		if ( ImGui::SliderFloat( "Damping##Steering", &m_steeringDampingRatio, 0.0f, 2.0f, "%.1f" ) )
		{
			b3WheelJoint_SetSteeringDampingRatio( m_frontLeftId, m_steeringDampingRatio );
			b3WheelJoint_SetSteeringDampingRatio( m_frontRightId, m_steeringDampingRatio );
		}

		if ( ImGui::SliderFloat( "Torque##Steering", &m_maxSteeringTorque, 0.0f, 20.0f, "%.1f" ) )
		{
			b3WheelJoint_SetMaxSteeringTorque( m_frontLeftId, m_maxSteeringTorque );
			b3WheelJoint_SetMaxSteeringTorque( m_frontRightId, m_maxSteeringTorque );
		}

		if ( ImGui::SliderFloat( "Min Deg##Steering", &m_lowerSteeringDegrees, -90.0f, 0.0f, "%.0f" ) )
		{
			b3WheelJoint_SetSteeringLimits( m_frontLeftId, B3_PI / 180.0f * m_lowerSteeringDegrees,
											B3_PI / 180.0f * m_upperSteeringDegrees );
			b3WheelJoint_SetSteeringLimits( m_frontRightId, B3_PI / 180.0f * m_lowerSteeringDegrees,
											B3_PI / 180.0f * m_upperSteeringDegrees );
		}

		if ( ImGui::SliderFloat( "Max Deg##Steering", &m_upperSteeringDegrees, 0.0f, 90.0f, "%.0f" ) )
		{
			b3WheelJoint_SetSteeringLimits( m_frontLeftId, B3_PI / 180.0f * m_lowerSteeringDegrees,
											B3_PI / 180.0f * m_upperSteeringDegrees );
			b3WheelJoint_SetSteeringLimits( m_frontRightId, B3_PI / 180.0f * m_lowerSteeringDegrees,
											B3_PI / 180.0f * m_upperSteeringDegrees );
		}

		ImGui::Separator();

		bool thirdPerson = m_camera->m_thirdPerson;
		if ( ImGui::Checkbox( "Third Person (T)", &thirdPerson ) )
		{
			ToggleThirdPerson();
		}

		return true;
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

		b3WorldTransform transform = b3WorldTransform_identity;
		transform.p.y += 0.05f;
		DrawAxes( transform, 2.0f );
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
			b3WorldTransform transform = b3Body_GetTransform( m_chassisId );
			m_camera->m_pivot = transform.p;
			m_camera->UpdateTransform();
		}

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new Driving( context );
	}

	b3HeightFieldData* m_heightField;
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

static int sampleDriving = RegisterSample( "Joints", "Driving", Driving::Create );

// https://www.linkedin.com/feed/update/urn:li:activity:7308603994082353153/
class GearLift : public Sample
{
public:
	explicit GearLift( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 18.0f, 12.0f, 17.0f, { -1.5f, 4.5f, 0.0f } );
		}

		AddGroundBox( 20.0f );

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			groundId = b3CreateBody( m_worldId, &bodyDef );
		}

		CreateMesh( groundId );

		// Gear cores shared by both shafts: a disk at each depth with an axle bridging
		// them, all cylinders along z so the gears spin about z.
		b3HullData* diskNear = MakeZCylinder( m_gearRadius, -m_gearZ - m_gearHalfDepth, -m_gearZ + m_gearHalfDepth, m_gearSides );
		b3HullData* diskFar = MakeZCylinder( m_gearRadius, m_gearZ - m_gearHalfDepth, m_gearZ + m_gearHalfDepth, m_gearSides );
		b3HullData* axle = MakeZCylinder( m_axleRadius, -m_gearZ, m_gearZ, m_axleSides );

		m_motorTorque = 30000.0f;
		m_motorSpeed = -0.3f;
		m_enableMotor = true;

		b3Pos gearPosition1 = { -4.25f, 9.75f, 0.0f };
		b3Pos gearPosition2 = { -2.25f, 10.75f, 0.0f };

		b3BodyId driverId = BuildGearBody( gearPosition1, m_gearRadius + m_toothHalfHeight, diskNear, diskFar, axle );
		b3BodyId followerId = BuildGearBody( gearPosition2, m_gearRadius + m_toothHalfWidth, diskNear, diskFar, axle );

		b3DestroyHull( diskNear );
		b3DestroyHull( diskFar );
		b3DestroyHull( axle );

		// Driver shaft, motorized
		{
			b3RevoluteJointDef revoluteDef = b3DefaultRevoluteJointDef();
			revoluteDef.base.bodyIdA = groundId;
			revoluteDef.base.bodyIdB = driverId;
			revoluteDef.base.localFrameA.p = b3Body_GetLocalPoint( groundId, gearPosition1 );
			revoluteDef.base.localFrameB.p = { 0.0f, 0.0f, 0.0f };
			revoluteDef.enableMotor = m_enableMotor;
			revoluteDef.maxMotorTorque = m_motorTorque;
			revoluteDef.motorSpeed = m_motorSpeed;
			m_driverId = b3CreateRevoluteJoint( m_worldId, &revoluteDef );
		}

		// Follower shaft, swept between angle limits
		{
			b3RevoluteJointDef revoluteDef = b3DefaultRevoluteJointDef();
			revoluteDef.base.bodyIdA = groundId;
			revoluteDef.base.bodyIdB = followerId;
			revoluteDef.base.localFrameA.p = b3Body_GetLocalPoint( groundId, gearPosition2 );
			revoluteDef.base.localFrameA.q = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.25f * B3_PI );
			revoluteDef.base.localFrameB.p = { 0.0f, 0.0f, 0.0f };
			revoluteDef.enableMotor = true;
			revoluteDef.maxMotorTorque = 0.5f;
			revoluteDef.lowerAngle = -0.3f * B3_PI;
			revoluteDef.upperAngle = 0.8f * B3_PI;
			revoluteDef.enableLimit = true;
			b3CreateRevoluteJoint( m_worldId, &revoluteDef );
		}

		// One chain hangs from the follower rim at each depth and both lift the gate.
		b3Pos linkAttach = { gearPosition2.x + m_gearRadius + 2.0f * m_toothHalfWidth + m_toothRadius, gearPosition2.y, 0.0f };
		b3Pos doorPosition = { linkAttach.x, linkAttach.y - ( 2.0f * m_linkCount * m_linkHalfLength + m_doorHalfHeight ), 0.0f };

		b3BodyId nearLink = CreateChain( followerId, { linkAttach.x, linkAttach.y, -m_gearZ } );
		b3BodyId farLink = CreateChain( followerId, { linkAttach.x, linkAttach.y, m_gearZ } );

		CreateDoor( groundId, doorPosition, nearLink, farLink );

		CreateDebris();
	}

	~GearLift() override
	{
		b3DestroyMesh( m_mesh );
	}

	void CreateMesh( b3BodyId groundId )
	{
		// Silhouette of the Box2D stairwell traced as a closed loop. It is extruded
		// four meters along z into a single triangle mesh, the 3D analogue of a
		// b2Chain loop. The solid is inside the loop, so the windings face outward
		// to put the collision normals on the open basin side where the debris sits.
		static const b3Vec2 points[32] = {
			{ -11.3000f, -0.2167f }, { 9.3375f, -0.2167f },	 { 9.3375f, 7.1917f },	{ 8.8083f, 7.1917f },  { 8.8083f, 0.3125f },
			{ 0.3417f, 0.3125f },	 { 0.3417f, 0.8417f },	 { -0.1875f, 0.8417f }, { -0.1875f, 1.3708f }, { -0.7167f, 1.3708f },
			{ -0.7167f, 1.9000f },	 { -1.2458f, 1.9000f },	 { -1.2458f, 2.4292f }, { -1.7750f, 2.4292f }, { -1.7750f, 2.9583f },
			{ -2.3042f, 2.9583f },	 { -2.3042f, 3.4875f },	 { -2.8333f, 3.4875f }, { -2.8333f, 4.0167f }, { -3.3625f, 4.0167f },
			{ -3.3625f, 4.5458f },	 { -3.8917f, 4.5458f },	 { -3.8917f, 5.0750f }, { -4.4208f, 5.0750f }, { -4.4208f, 5.6042f },
			{ -4.9500f, 5.6042f },	 { -4.9500f, 6.1333f },	 { -5.4792f, 6.1333f }, { -5.4792f, 6.6625f }, { -6.0083f, 6.6625f },
			{ -6.0083f, 7.1917f },	 { -11.3000f, 7.1917f },
		};

		float zMin = -2.0f; // four meters across z
		float zMax = 2.0f;

		// Two vertices per silhouette point, one at each depth.
		b3Vec3 vertices[64];
		for ( int i = 0; i < 32; ++i )
		{
			vertices[2 * i + 0] = { points[i].x, points[i].y, zMin };
			vertices[2 * i + 1] = { points[i].x, points[i].y, zMax };
		}

		std::vector<int> indices;

		// Side walls: two triangles per silhouette edge, wound so the normal faces
		// the open basin. The lo ring sits at zMin, the hi ring at zMax.
		for ( int i = 0; i < 32; ++i )
		{
			int j = ( i + 1 ) % 32;
			int aLo = 2 * i, aHi = 2 * i + 1;
			int bLo = 2 * j, bHi = 2 * j + 1;

			indices.push_back( aLo );
			indices.push_back( bLo );
			indices.push_back( bHi );

			indices.push_back( aLo );
			indices.push_back( bHi );
			indices.push_back( aHi );
		}

		// End caps close the prism into a watertight manifold with no boundary edges.
		// The non convex silhouette is triangulated with earcut, then each triangle is
		// wound so the cap normal points out of the solid: the zMin cap faces -z and
		// the zMax cap faces +z. The two ring edges each gain a second owner this way.
		std::vector<std::array<double, 2>> ring;
		ring.reserve( 32 );
		for ( int i = 0; i < 32; ++i )
		{
			ring.push_back( { points[i].x, points[i].y } );
		}
		std::vector<std::vector<std::array<double, 2>>> polygon = { ring };
		std::vector<uint32_t> cap = mapbox::earcut<uint32_t>( polygon );

		for ( size_t k = 0; k + 3 <= cap.size(); k += 3 )
		{
			int r0 = (int)cap[k], r1 = (int)cap[k + 1], r2 = (int)cap[k + 2];
			PushCap( indices, points, r0, r1, r2, 1, true );  // zMax cap, +z out
			PushCap( indices, points, r0, r1, r2, 0, false ); // zMin cap, -z out
		}

		b3MeshDef def = {};
		def.vertices = vertices;
		def.vertexCount = 64;
		def.indices = indices.data();
		def.triangleCount = (int)( indices.size() / 3 );
		def.identifyEdges = true;

		m_mesh = b3CreateMesh( &def, nullptr, 0 );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.customColor = b3_colorDarkSeaGreen;
		b3SurfaceMaterial material = shapeDef.baseMaterial;
		shapeDef.materials = &material;
		shapeDef.materialCount = 1;
		b3CreateMeshShape( groundId, &shapeDef, m_mesh, b3Vec3_one );

		// Back wall: a 0.1 m thick box closing the far side over the full mesh extent.
		b3Vec2 lower = points[0];
		b3Vec2 upper = points[0];
		for ( int i = 1; i < 32; ++i )
		{
			lower.x = b3MinFloat( lower.x, points[i].x );
			lower.y = b3MinFloat( lower.y, points[i].y );
			upper.x = b3MaxFloat( upper.x, points[i].x );
			upper.y = b3MaxFloat( upper.y, points[i].y );
		}

		float wallHalfThick = 0.05f;
		b3Vec3 wallCenter = { 0.5f * ( lower.x + upper.x ), 0.5f * ( lower.y + upper.y ), -zMax - wallHalfThick };
		b3BoxHull wall =
			b3MakeOffsetBoxHull( 0.5f * ( upper.x - lower.x ), 0.5f * ( upper.y - lower.y ), wallHalfThick, wallCenter );
		b3CreateHullShape( groundId, &shapeDef, &wall.base );
	}

	// Push one cap triangle, flipping the winding so its z-normal has the wanted sign.
	void PushCap( std::vector<int>& indices, const b3Vec2* poly, int r0, int r1, int r2, int vOffset, bool wantPositiveZ )
	{
		float cross =
			( poly[r1].x - poly[r0].x ) * ( poly[r2].y - poly[r0].y ) - ( poly[r1].y - poly[r0].y ) * ( poly[r2].x - poly[r0].x );
		bool positive = cross > 0.0f;

		int v0 = 2 * r0 + vOffset;
		int v1 = 2 * r1 + vOffset;
		int v2 = 2 * r2 + vOffset;

		indices.push_back( v0 );
		if ( positive == wantPositiveZ )
		{
			indices.push_back( v1 );
			indices.push_back( v2 );
		}
		else
		{
			indices.push_back( v2 );
			indices.push_back( v1 );
		}
	}

	// A cylinder built directly along z (depth) so it needs no reorientation.
	b3HullData* MakeZCylinder( float radius, float zMin, float zMax, int sides )
	{
		b3Vec3 points[64];
		for ( int i = 0; i < sides; ++i )
		{
			float angle = ( 2.0f * B3_PI * i ) / sides;
			float c = cosf( angle );
			float s = sinf( angle );
			points[2 * i + 0] = { radius * c, radius * s, zMin };
			points[2 * i + 1] = { radius * c, radius * s, zMax };
		}

		return b3CreateHull( points, 2 * sides, 2 * sides );
	}

	// One gear shaft: a disk and tooth ring at each depth joined by an axle, all on
	// one rigid body so the two depth gears share an angle and read as a clean gear.
	b3BodyId BuildGearBody( b3Pos position, float toothCenterRadius, b3HullData* diskNear, b3HullData* diskFar, b3HullData* axle )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = position;
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.1f;
		shapeDef.baseMaterial.customColor = b3_colorSaddleBrown;
		b3CreateHullShape( bodyId, &shapeDef, diskNear );
		b3CreateHullShape( bodyId, &shapeDef, diskFar );

		shapeDef.baseMaterial.customColor = b3_colorSlateGray;
		b3CreateHullShape( bodyId, &shapeDef, axle );

		AddTeeth( bodyId, toothCenterRadius, -m_gearZ );
		AddTeeth( bodyId, toothCenterRadius, m_gearZ );

		return bodyId;
	}

	// Chain of capsule links hanging from a body down to the gate. Returns the last link.
	b3BodyId CreateChain( b3BodyId topBodyId, b3Pos attach )
	{
		b3Capsule capsule = { { 0.0f, -m_linkHalfLength, 0.0f }, { 0.0f, m_linkHalfLength, 0.0f }, m_linkRadius };

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.customColor = b3_colorLightSteelBlue;

		b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
		jointDef.maxMotorTorque = 0.05f;
		jointDef.enableMotor = true;
		jointDef.base.drawScale = 0.2f;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		b3Pos position = { attach.x, attach.y - m_linkHalfLength, attach.z };

		b3BodyId prevBodyId = topBodyId;
		for ( int i = 0; i < m_linkCount; ++i )
		{
			bodyDef.position = position;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );

			b3Pos pivot = { position.x, position.y + m_linkHalfLength, attach.z };
			jointDef.base.bodyIdA = prevBodyId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( prevBodyId, pivot );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( bodyId, pivot );
			b3CreateRevoluteJoint( m_worldId, &jointDef );

			position.y -= 2.0f * m_linkHalfLength;
			prevBodyId = bodyId;
		}

		return prevBodyId;
	}

	// A single gate box raised by both chains and held upright sliding along y.
	void CreateDoor( b3BodyId groundId, b3Pos doorPosition, b3BodyId nearLink, b3BodyId farLink )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = doorPosition;
		b3BodyId doorId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density *= 0.5f;
		shapeDef.baseMaterial.friction = 0.1f;
		shapeDef.baseMaterial.customColor = b3MakeDebugColor( b3_colorDarkCyan, b3_debugMaterialMetallic );
		b3BoxHull box = b3MakeBoxHull( 0.05f, m_doorHalfHeight, m_doorHalfDepth );
		b3CreateHullShape( doorId, &shapeDef, &box.base );

		// Hinge the gate to each chain at its depth.
		b3BodyId links[2] = { nearLink, farLink };
		float depths[2] = { -m_gearZ, m_gearZ };
		for ( int i = 0; i < 2; ++i )
		{
			b3Pos pivot = { doorPosition.x, doorPosition.y + m_doorHalfHeight, depths[i] };
			b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = links[i];
			jointDef.base.bodyIdB = doorId;
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( links[i], pivot );
			jointDef.base.localFrameB.p = { 0.0f, m_doorHalfHeight, depths[i] };
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 50.0f;
			b3CreateRevoluteJoint( m_worldId, &jointDef );
		}

		// Prismatic axis is frame A local x, so rotate it onto world y.
		b3Quat slideAxis = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisX, b3Vec3_axisY );
		b3PrismaticJointDef jointDef = b3DefaultPrismaticJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = doorId;
		jointDef.base.localFrameA.p = b3Body_GetLocalPoint( groundId, doorPosition );
		jointDef.base.localFrameA.q = slideAxis;
		jointDef.base.localFrameB.p = { 0.0f, 0.0f, 0.0f };
		jointDef.base.localFrameB.q = slideAxis;
		jointDef.maxMotorForce = 200.0f;
		jointDef.enableMotor = true;
		jointDef.base.collideConnected = true;
		b3CreatePrismaticJoint( m_worldId, &jointDef );
	}

	// Ring of 16 teeth around the gear rim at depth zCenter, in the body frame. Box3D
	// has no rounded hulls, so each tooth tapers tangentially toward the tip to clear
	// the meshing teeth the way Box2D's rounded teeth do.
	void AddTeeth( b3BodyId bodyId, float centerRadius, float zCenter )
	{
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.1f;
		shapeDef.baseMaterial.customColor = b3_colorGray;

		int count = 16;
		float deltaAngle = 2.0f * B3_PI / count;

		float hx = m_toothHalfWidth;					   // radial half extent
		float hz = m_gearHalfDepth;						   // depth half extent
		float baseHalf = m_toothHalfHeight;				   // tangential half width at the base
		float tipHalf = m_toothHalfHeight - m_toothRadius; // narrower at the tip

		for ( int i = 0; i < count; ++i )
		{
			b3Quat q = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, i * deltaAngle );
			b3Vec3 center = b3RotateVector( q, { centerRadius, 0.0f, 0.0f } );
			center.z = zCenter;

			// Base at the inner radius, narrower tip at the outer radius.
			b3Vec3 local[8] = {
				{ -hx, -baseHalf, -hz }, { -hx, baseHalf, -hz }, { -hx, baseHalf, hz }, { -hx, -baseHalf, hz },
				{ hx, -tipHalf, -hz },	 { hx, tipHalf, -hz },	 { hx, tipHalf, hz },	{ hx, -tipHalf, hz },
			};

			b3Vec3 points[8];
			for ( int k = 0; k < 8; ++k )
			{
				points[k] = b3Add( center, b3RotateVector( q, local[k] ) );
			}

			b3HullData* tooth = b3CreateHull( points, 8, 8 );
			b3CreateHullShape( bodyId, &shapeDef, tooth );
			b3DestroyHull( tooth );
		}
	}

	void CreateDebris()
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.rollingResistance = 0.3f;

		b3HexColor colors[5] = {
			b3_colorGray, b3_colorGainsboro, b3_colorLightGray, b3_colorLightSlateGray, b3_colorDarkGray,
		};

		b3HullData* rockHull = b3CreateRock( m_rockRadius );

		float x = -5.0f;
		int xCount = 12, yCount = 10;
		for ( int i = 0; i < xCount; ++i )
		{
			float y = 6.5f - 0.25f * i;
			for ( int j = 0; j < yCount; ++j )
			{
				// Spread the debris across the depth of the stairwell.
				bodyDef.position = { x, y, RandomFloatRange( -1.65f, 0.35f ) };
				bodyDef.rotation = RandomQuat();
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

				shapeDef.baseMaterial.customColor = colors[(int)RandomIntRange( 0, 4 )];
				b3CreateHullShape( bodyId, &shapeDef, rockHull );

				y += 0.2f;
			}
			x += 0.3f;
		}

		b3DestroyHull( rockHull );
	}

	void SetMotorSpeed( float speed )
	{
		b3RevoluteJoint_SetMotorSpeed( m_driverId, speed );
		b3Joint_WakeBodies( m_driverId );
	}

	bool DrawControls() override
	{
		if ( ImGui::Checkbox( "Motor", &m_enableMotor ) )
		{
			b3RevoluteJoint_EnableMotor( m_driverId, m_enableMotor );
			b3Joint_WakeBodies( m_driverId );
		}

		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		if ( ImGui::SliderFloat( "Max Torque", &m_motorTorque, 0.0f, 100000.0f, "%.0f" ) )
		{
			b3RevoluteJoint_SetMaxMotorTorque( m_driverId, m_motorTorque );
			b3Joint_WakeBodies( m_driverId );
		}

		if ( ImGui::SliderFloat( "Speed", &m_motorSpeed, -0.3f, 0.3f, "%.2f" ) )
		{
			SetMotorSpeed( m_motorSpeed );
		}

		ImGui::PopItemWidth();

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new GearLift( context );
	}

	static constexpr float m_gearRadius = 1.0f;
	static constexpr float m_gearHalfDepth = 0.125f; // 25 cm gear depth
	static constexpr float m_gearZ = 1.5f;			 // near and far gear planes
	static constexpr float m_axleRadius = 0.2f;
	static constexpr float m_toothHalfWidth = 0.11f;
	static constexpr float m_toothHalfHeight = 0.09f;
	static constexpr float m_toothRadius = 0.03f;
	static constexpr float m_linkHalfLength = 0.07f;
	static constexpr float m_linkRadius = 0.05f;
	static constexpr int m_linkCount = 40;
	static constexpr float m_doorHalfHeight = 1.5f;
	static constexpr float m_doorHalfDepth = 1.95f;
	static constexpr int m_gearSides = 24;
	static constexpr int m_axleSides = 12;
	static constexpr float m_rockRadius = 0.3f;

	b3MeshData* m_mesh;
	b3JointId m_driverId;
	float m_motorTorque;
	float m_motorSpeed;
	bool m_enableMotor;
};

static int sampleGearLift = RegisterSample( "Joints", "Gear Lift", GearLift::Create );
