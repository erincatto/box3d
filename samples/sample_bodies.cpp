// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "GLFW/glfw3.h"
#include "camera.h"
#include "imgui.h"
#include "renderer.h"
#include "sample.h"
#include "scene.h"

#include "box3d/box3d.h"

class BodyType : public Sample
{
public:
	explicit BodyType( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 30.0f, { 0.0f, 1.5f, 0.0f } );
		}

		m_type = b3_dynamicBody;
		m_isEnabled = true;

		b3BodyId groundId = b3_nullBodyId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position.y = -1.0f;
			bodyDef.name = "ground";
			groundId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( groundId, &shapeDef, &box.base );
		}

		// Define attachment
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -2.0f, 3.0f, 0.0f };
			bodyDef.name = "attach1";
			m_attachmentId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 0.5f, 2.0f, 0.5f );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 1.0f;
			b3CreateHullShape( m_attachmentId, &shapeDef, &box.base );
		}

		// Define second attachment
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = { 3.0f, 3.0f };
			bodyDef.name = "attach2";
			m_secondAttachmentId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 0.5f, 2.0f, 0.5f );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 1.0f;
			b3CreateHullShape( m_secondAttachmentId, &shapeDef, &box.base );
		}

		// Define platform
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = { -4.0f, 5.0f };
			bodyDef.name = "platform";
			m_platformId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeTransformedBoxHull(
				0.5f, 4.0f, 0.5f, { { 4.0f, 0.0f, 0.0f }, b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI ) } );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 2.0f;
			b3CreateHullShape( m_platformId, &shapeDef, &box.base );

			b3RevoluteJointDef revoluteDef = b3DefaultRevoluteJointDef();
			b3Vec3 pivot = { -2.0f, 5.0f, 0.0f };
			revoluteDef.base.bodyIdA = m_attachmentId;
			revoluteDef.base.bodyIdB = m_platformId;
			revoluteDef.base.localFrameA.p = b3Body_GetLocalPoint( m_attachmentId, pivot );
			revoluteDef.base.localFrameB.p = b3Body_GetLocalPoint( m_platformId, pivot );
			revoluteDef.maxMotorTorque = 50.0f;
			revoluteDef.enableMotor = true;
			b3CreateRevoluteJoint( m_worldId, &revoluteDef );

			pivot = { 3.0f, 5.0f };
			revoluteDef.base.bodyIdA = m_secondAttachmentId;
			revoluteDef.base.bodyIdB = m_platformId;
			revoluteDef.base.localFrameA.p = b3Body_GetLocalPoint( m_secondAttachmentId, pivot );
			revoluteDef.base.localFrameB.p = b3Body_GetLocalPoint( m_platformId, pivot );
			revoluteDef.maxMotorTorque = 50.0f;
			revoluteDef.enableMotor = true;
			b3CreateRevoluteJoint( m_worldId, &revoluteDef );

			b3PrismaticJointDef prismaticDef = b3DefaultPrismaticJointDef();
			b3Vec3 anchor = { 0.0f, 5.0f, 0.0f };
			prismaticDef.base.bodyIdA = groundId;
			prismaticDef.base.bodyIdB = m_platformId;
			prismaticDef.base.localFrameA.p = b3Body_GetLocalPoint( groundId, anchor );
			prismaticDef.base.localFrameB.p = b3Body_GetLocalPoint( m_platformId, anchor );
			prismaticDef.maxMotorForce = 1000.0f;
			prismaticDef.motorSpeed = 0.0f;
			prismaticDef.enableMotor = true;
			prismaticDef.lowerTranslation = -10.0f;
			prismaticDef.upperTranslation = 10.0f;
			prismaticDef.enableLimit = true;

			b3CreatePrismaticJoint( m_worldId, &prismaticDef );

			m_speed = 3.0f;
		}

		// Create a payload
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -3.0f, 8.0f };
			bodyDef.name = "crate1";
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 0.75f, 0.75f, 0.75f );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 2.0f;

			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}

		// Create a second payload
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = { 2.0f, 8.0f };
			bodyDef.name = "crate2";
			m_secondPayloadId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 0.75f, 0.75f, 0.75f );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 2.0f;

			b3CreateHullShape( m_secondPayloadId, &shapeDef, &box.base );
		}

		// Create a separate body on the ground
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = { 8.0f, 0.2f };
			bodyDef.name = "debris";
			m_touchingBodyId = b3CreateBody( m_worldId, &bodyDef );

			b3Capsule capsule = { { 0.0f, 0.0f }, { 1.0f, 0.0f }, 0.25f };

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 2.0f;

			b3CreateCapsuleShape( m_touchingBodyId, &shapeDef, &capsule );
		}

		// Create a separate floating body
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = { -8.0f, 12.0f };
			bodyDef.gravityScale = 0.0f;
			bodyDef.name = "floater";
			m_floatingBodyId = b3CreateBody( m_worldId, &bodyDef );

			b3Sphere sphere = { { 0.0f, 0.5f, 0.0f }, 0.25f };

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 2.0f;

			b3CreateSphereShape( m_floatingBodyId, &shapeDef, &sphere );
		}
	}

	void UpdateUI() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 11.0f * fontSize;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->m_height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 9.0f * fontSize, height ) );
		ImGui::Begin( "Body Type", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		if ( ImGui::RadioButton( "Static", m_type == b3_staticBody ) )
		{
			m_type = b3_staticBody;
			b3Body_SetType( m_platformId, b3_staticBody );
			b3Body_SetType( m_secondAttachmentId, b3_staticBody );
			b3Body_SetType( m_secondPayloadId, b3_staticBody );
			b3Body_SetType( m_touchingBodyId, b3_staticBody );
			b3Body_SetType( m_floatingBodyId, b3_staticBody );
		}

		if ( ImGui::RadioButton( "Kinematic", m_type == b3_kinematicBody ) )
		{
			m_type = b3_kinematicBody;
			b3Body_SetType( m_platformId, b3_kinematicBody );
			b3Body_SetLinearVelocity( m_platformId, { -m_speed, 0.0f } );
			b3Body_SetAngularVelocity( m_platformId, b3Vec3_zero );

			b3Body_SetType( m_secondAttachmentId, b3_kinematicBody );
			b3Body_SetLinearVelocity( m_secondAttachmentId, b3Vec3_zero );
			b3Body_SetAngularVelocity( m_secondAttachmentId, b3Vec3_zero );

			b3Body_SetType( m_secondPayloadId, b3_kinematicBody );
			b3Body_SetType( m_touchingBodyId, b3_kinematicBody );
			b3Body_SetType( m_floatingBodyId, b3_kinematicBody );
		}

		if ( ImGui::RadioButton( "Dynamic", m_type == b3_dynamicBody ) )
		{
			m_type = b3_dynamicBody;
			b3Body_SetType( m_platformId, b3_dynamicBody );
			b3Body_SetType( m_secondAttachmentId, b3_dynamicBody );
			b3Body_SetType( m_secondPayloadId, b3_dynamicBody );
			b3Body_SetType( m_touchingBodyId, b3_dynamicBody );
			b3Body_SetType( m_floatingBodyId, b3_dynamicBody );
		}

		if ( ImGui::Checkbox( "Enable", &m_isEnabled ) )
		{
			if ( m_isEnabled )
			{
				b3Body_Enable( m_attachmentId );
				b3Body_Enable( m_secondPayloadId );
				b3Body_Enable( m_floatingBodyId );
			}
			else
			{
				b3Body_Disable( m_attachmentId );
				b3Body_Disable( m_secondPayloadId );
				b3Body_Disable( m_floatingBodyId );
			}
		}

		ImGui::End();
	}

	void Step() override
	{
		// Drive the kinematic body.
		if ( m_type == b3_kinematicBody )
		{
			b3Vec3 p = b3Body_GetPosition( m_platformId );
			b3Vec3 v = b3Body_GetLinearVelocity( m_platformId );

			if ( ( p.x < -14.0f && v.x < 0.0f ) || ( p.x > 6.0f && v.x > 0.0f ) )
			{
				v.x = -v.x;
				b3Body_SetLinearVelocity( m_platformId, v );
			}
		}

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BodyType( context );
	}

	b3BodyId m_attachmentId;
	b3BodyId m_secondAttachmentId;
	b3BodyId m_platformId;
	b3BodyId m_secondPayloadId;
	b3BodyId m_touchingBodyId;
	b3BodyId m_floatingBodyId;
	b3BodyType m_type;
	float m_speed;
	bool m_isEnabled;
};

static int sampleBodyType = SampleManager::Register( "Bodies", "Body Type", BodyType::Create );

class SpinningBooks : public Sample
{
public:
	explicit SpinningBooks( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 10.0f, { 0.0f, 1.0f, 0.0f } );
		}

		b3BoxHull box = b3MakeBoxHull( 0.35f, 0.08f, 0.5f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.gravityScale = 0.0f;

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		bodyDef.position = { -2.0f, 2.0f, 0.0f };
		bodyDef.angularVelocity = { 5.0f, 0.01f, 0.01f };

		b3BodyId body1 = b3CreateBody( m_worldId, &bodyDef );
		b3CreateHullShape( body1, &shapeDef, &box.base );

		bodyDef.position = { 0.0f, 2.0f, 0.0f };
		bodyDef.angularVelocity = { 0.01f, 5.0f, 0.01f };

		b3BodyId body2 = b3CreateBody( m_worldId, &bodyDef );
		b3CreateHullShape( body2, &shapeDef, &box.base );

		bodyDef.position = { 2.0f, 2.0f, 0.0f };
		bodyDef.angularVelocity = { 0.01f, 0.01f, -5.0f };

		b3BodyId body3 = b3CreateBody( m_worldId, &bodyDef );
		b3CreateHullShape( body3, &shapeDef, &box.base );
	}

	void Render() override
	{
		DrawGrid( m_scene, 10 );
		Sample::Render();
	}

	static Sample* Create( SampleContext* context )
	{
		return new SpinningBooks( context );
	}
};

static int sampleBook = SampleManager::Register( "Bodies", "Spinning Book", SpinningBooks::Create );

// Dzhanibekov effect
class GyroscopicTorque : public Sample
{
public:
	explicit GyroscopicTorque( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 20.0f, 4.0f, { 0.0f, 2.0f, 0.0f } );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 2.0f, 0.0f };
		bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, -0.5f * B3_PI );
		bodyDef.angularVelocity = { 0.01f, 0.01f, 10.0f };
		bodyDef.gravityScale = 0.0f;
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3Hull* cylinder = b3CreateCylinder( 0.6f, 0.15f, 0.0f, 32 );
		b3BoxHull box = b3MakeBoxHull( 1.0f, 0.05f, 0.1f );
		b3CreateHullShape( bodyId, &shapeDef, cylinder );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );

		b3DestroyHull( cylinder );
	}

	void Render() override
	{
		Sample::Render();
		DrawGrid( m_scene, 10 );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new GyroscopicTorque( sampleContext );
	}
};

static int sampleGyroscopicTorque = SampleManager::Register( "Bodies", "Gyroscopic Torque", GyroscopicTorque::Create );

class Weeble : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new Weeble( context );
	}

	explicit Weeble( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 25.0f, 25.0f, b3Vec3_zero );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { 0.0f, -1.0f, 0.0f };
		b3BodyId groundBody = b3CreateBody( m_worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( 32.0f, 1.0f, 32.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHullShape( groundBody, &shapeDef, &box.base );

		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 3.0f, 0.0f };
		m_weebleId = b3CreateBody( m_worldId, &bodyDef );

		b3Capsule capsule = { { 0.0f, -1.0f }, { 0.0f, 1.0f }, 1.0f };
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3CreateCapsuleShape( m_weebleId, &shapeDef, &capsule );

		float mass = b3Body_GetMass( m_weebleId );
		b3Matrix3 inertiaTensor = b3Body_GetLocalRotationalInertia( m_weebleId );

		b3Vec3 offset = { 0.0f, -1.5f, 0.0f };

		// See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
		inertiaTensor += b3Steiner( mass, offset );

		b3MassData massData = {
			.mass = mass,
			.center = offset,
			.inertia = inertiaTensor,
		};

		b3Body_SetMassData( m_weebleId, massData );

		m_explosionPosition = { 0.0f, -0.1f, 0.0f };
		m_explosionRadius = 8.0f;
		m_explosionMagnitude = 10000.0f;
	}

	void UpdateUI() override
	{
		float height = 120.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_camera->m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 200.0f, height ) );

		ImGui::Begin( "Weeble", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );
		if ( ImGui::Button( "Teleport" ) )
		{
			b3Body_SetTransform( m_weebleId, { 0.0f, 5.0f, 0.0f }, b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.95f * B3_PI ) );
			b3Body_SetAwake( m_weebleId, true );
		}

		if ( ImGui::Button( "Explode" ) )
		{
			b3ExplosionDef def = b3DefaultExplosionDef();
			def.position = m_explosionPosition;
			def.radius = m_explosionRadius;
			def.falloff = 0.1f;
			def.impulsePerArea = m_explosionMagnitude;
			b3World_Explode( m_worldId, &def );
		}
		ImGui::PushItemWidth( 100.0f );

		ImGui::SliderFloat( "Magnitude", &m_explosionMagnitude, -100000.0f, 100000.0f, "%.0f" );

		ImGui::PopItemWidth();
		ImGui::End();
	}

	void Render() override
	{
		Sample::Render();
		b3Transform transform = { { 0.0f, 0.1f, 0.0f }, b3Quat_identity };
		DrawTransform( m_scene, transform, 4.0f );
	}

	void Step() override
	{
		Sample::Step();

		b3Sphere sphere = { b3Vec3_zero, m_explosionRadius };
		DrawSphere( m_scene, { m_explosionPosition, b3Quat_identity }, sphere, b3_colorAzure );

		// This shows how to get the velocity of a point on a body
		b3Vec3 localPoint = { 0.0f, 2.0f, 0.0f };
		b3Vec3 worldPoint = b3Body_GetWorldPoint( m_weebleId, localPoint );

		b3Vec3 v1 = b3Body_GetLocalPointVelocity( m_weebleId, localPoint );
		b3Vec3 v2 = b3Body_GetWorldPointVelocity( m_weebleId, worldPoint );

		b3Vec3 offset = { 0.05f, 0.0f };
		DrawLine( m_scene, worldPoint, worldPoint + v1, b3_colorRed );
		DrawLine( m_scene, worldPoint + offset, worldPoint + v2 + offset, b3_colorGreen );
	}

	b3BodyId m_weebleId;
	b3Vec3 m_explosionPosition;
	float m_explosionRadius;
	float m_explosionMagnitude;
};

static int sampleWeeble = SampleManager::Register( "Bodies", "Weeble", Weeble::Create );

class DisableBody : public Sample
{
public:
	enum
	{
		e_count = 4
	};

	explicit DisableBody( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 25.0f, 10.0f, b3Vec3_zero );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { 0.0f, -1.0f, 0.0f };

		b3BodyId groundBody = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 25.0f, 1.0f, 25.0f );
		b3CreateHullShape( groundBody, &shapeDef, &box.base );

		float linkRadius = 0.1f;
		float linkLength = 5.0f * linkRadius;
		b3Capsule capsule = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, linkLength }, linkRadius };

		b3BodyId parentId = {};
		for ( int link = 0; link < e_count; ++link )
		{
			bodyDef.position = { 0.0f, 0.8f * float( e_count ) * linkLength, link * linkLength };
			bodyDef.type = B3_IS_NULL( parentId ) ? b3_kinematicBody : b3_dynamicBody;
			b3BodyId childId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateCapsuleShape( childId, &shapeDef, &capsule );
			m_bodyIds[link] = childId;

			if ( B3_IS_NON_NULL( parentId ) )
			{
				b3WeldJointDef jointDef = b3DefaultWeldJointDef();
				jointDef.base.bodyIdA = parentId;
				jointDef.base.bodyIdB = childId;
				jointDef.base.localFrameA.p = { 0.0f, 0.0f, linkLength };
				jointDef.angularHertz = 10.0f;
				jointDef.angularDampingRatio = 1.0f;
				b3CreateWeldJoint( m_worldId, &jointDef );
			}

			parentId = childId;
		}

		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 3.0f, 3.0f, 0.0f };
		m_ballId = b3CreateBody( m_worldId, &bodyDef );

		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		b3CreateSphereShape( m_ballId, &shapeDef, &sphere );
	}

	void UpdateUI() override
	{
		float height = 100.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_camera->m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 200.0f, height ) );

		ImGui::Begin( "Distance", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		{
			bool enabled = b3Body_IsEnabled( m_bodyIds[2] );
			if ( ImGui::Checkbox( "Enable Link", &enabled ) )
			{
				if ( enabled )
				{
					b3Body_Enable( m_bodyIds[2] );
				}
				else
				{
					b3Body_Disable( m_bodyIds[2] );
				}
			}
		}

		{
			bool enabled = b3Body_IsEnabled( m_ballId );
			if ( ImGui::Checkbox( "Enable Ball", &enabled ) )
			{
				if ( enabled )
				{
					b3Body_Enable( m_ballId );
				}
				else
				{
					b3Body_Disable( m_ballId );
				}
			}
		}

		ImGui::End();
	}

	void Step() override
	{
		b3Body_ApplyLinearImpulseToCenter( m_bodyIds[2], { 0.0f, 0.1f, 0.0f }, true );

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new DisableBody( context );
	}

	b3BodyId m_bodyIds[e_count];
	b3BodyId m_ballId;
};

static int sampleDisable = SampleManager::Register( "Bodies", "Disable", DisableBody::Create );

class BodyCast : public Sample
{
public:
	explicit BodyCast( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 120.0f, 30.0f, 20.0f, { 0.0f, 1.5f, 0.0f } );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_kinematicBody;
		bodyDef.position = { 5.0f, 5.0f, 0.0f };
		bodyDef.angularVelocity = { 0.1f, -0.1f, 0.1f };
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		m_cylinder = b3CreateCylinder( 2.0f, 0.5f, 0.0f, 16 );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHullShape( m_bodyId, &shapeDef, m_cylinder );

		m_transform.p = { -10.0f, 2.0f, 0.0f };
		m_transform.q = b3MakeQuatFromAxisAngle( b3Normalize( { 1.0f, -2.0f, 3.0f } ), 0.75f * B3_PI );

		m_baseTranslation = b3Vec3_zero;
		m_baseX = 0;
		m_baseY = 0;
		m_origin = b3Vec3_zero;
		m_tracking = false;
	}

	~BodyCast() override
	{
		b3DestroyHull( m_cylinder );
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 && modifiers == GLFW_MOD_ALT )
		{
			PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
			m_origin = pickRay.origin + 10.0f * b3Normalize( pickRay.translation );
			m_baseTranslation = m_transform.p;
			m_tracking = true;
		}
		else
		{
			m_tracking = false;
		}
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

	void Render() override
	{
		Sample::Render();

		DrawGrid( m_scene, 10 );
		b3Transform transform = { { 0.0f, 0.1f, 0.0f }, b3Quat_identity };
		DrawTransform( m_scene, transform, 4.0f );

		DrawHull( m_scene, m_transform, m_cylinder, b3_colorBlue, false );
	}

	void Step() override
	{
		// Cast ray
		{
			b3BodyRayCastInput input = {};
			input.origin = { -9.75f, 3.0f, -4.0f };
			input.translation = { 0.0f, 0.0f, 8.0f };
			input.filter = b3DefaultQueryFilter();
			input.maxFraction = 1.0f;
			b3BodyCastResult result = b3Body_CastRay( m_bodyId, &input, m_transform );

			DrawLine( m_scene, input.origin, input.origin + input.maxFraction * input.translation, b3_colorCyan );

			if ( result.hit )
			{
				DrawLine( m_scene, result.point, result.point + 0.2f * result.normal, b3_colorYellow );
				DrawPoint( m_scene, result.point, 10.0f, b3_colorYellow );
			}

			DrawPoint( m_scene, input.origin, 10.0f, b3_colorGreen );
			DrawPoint( m_scene, input.origin + input.translation, 10.0f, b3_colorRed );
		}

		// Cast sphere
		{
			b3Sphere sphere = { { -14.5f, 2.5f, 0.5f }, 0.2f };

			b3BodyShapeCastInput input = {};
			input.proxy = { &sphere.center, 1, sphere.radius };
			input.translation = { 8.0f, 0.0f, 0.0f };
			input.filter = b3DefaultQueryFilter();
			input.maxFraction = 1.0f;
			input.canEncroach = true;
			b3BodyCastResult result = b3Body_CastShape( m_bodyId, &input, m_transform );

			if ( result.hit )
			{
				b3Transform transform = { result.fraction * input.translation, b3Quat_identity };
				DrawSphere( m_scene, transform, sphere, b3_colorGreen );
				DrawLine( m_scene, result.point, result.point + 0.2f * result.normal, b3_colorYellow );
			}
			else
			{
				b3Transform transform = { input.maxFraction * input.translation, b3Quat_identity };
				DrawSphere( m_scene, transform, sphere, b3_colorWhite );
			}

			DrawLine( m_scene, sphere.center, sphere.center + input.maxFraction * input.translation, b3_colorWhite );
			DrawPoint( m_scene, sphere.center, 10.0f, b3_colorGreen );
			DrawPoint( m_scene, sphere.center + input.maxFraction * input.translation, 10.0f, b3_colorRed );
		}

		// Overlap capsule
		{
			b3Capsule capsule = { { -10.5f, 2.0f, 0.5f }, { -9.5f, 1.0f, 0.5f }, 0.5f };
			b3ShapeProxy proxy = { &capsule.center1, 2, capsule.radius };
			bool overlaps = b3Body_OverlapShape( m_bodyId, &proxy, b3DefaultQueryFilter(), m_transform );

			if ( overlaps )
			{
				DrawCapsule( m_scene, b3Transform_identity, capsule, b3_colorGreen );
			}
			else
			{
				DrawCapsule( m_scene, b3Transform_identity, capsule, b3_colorGray );
			}
		}

		// Collide capsule
		{
			b3Capsule capsule = { { -10.25f, 2.0f, -0.75f }, { -10.25f, 3.0f, -0.75f }, 0.3f };
			b3BodyPlaneResult bodyPlanes[4];
			int count = b3Body_CollideMover( m_bodyId, bodyPlanes, 4, &capsule, b3DefaultQueryFilter(), m_transform );
			DrawCapsule( m_scene, b3Transform_identity, capsule, b3_colorPurple );

			for ( int i = 0; i < count; ++i )
			{
				b3PlaneResult result = bodyPlanes[i].result;
				DrawPlane( m_scene, result.plane.normal, result.point, b3_colorOrange );
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BodyCast( context );
	}

	b3Hull* m_cylinder;
	b3BodyId m_bodyId;
	b3Transform m_transform;

	b3Vec3 m_baseTranslation;
	b3Vec3 m_origin;
	int m_baseX;
	int m_baseY;
	bool m_tracking;
};

static int sampleBodyCast = SampleManager::Register( "Bodies", "Cast", BodyCast::Create );

// This shows how to drive a kinematic body to reach a target
class Kinematic : public Sample
{
public:
	explicit Kinematic( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 10.0f, { 0.0f, 1.5f, 0.0f } );
		}

		m_amplitude = 2.0f;

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_kinematicBody;
			bodyDef.position.x = 2.0f * m_amplitude;
			bodyDef.position.y = m_amplitude;

			m_bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 0.1f, 1.0f, 0.2f );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( m_bodyId, &shapeDef, &box.base );
		}

		m_time = 0.0f;
	}

	void Render() override
	{
		Sample::Render();
		DrawGrid( m_scene, 10 );
		DrawTransform( m_scene, b3Transform_identity, 4.0f );
	}

	void Step() override
	{
		float timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
		if ( m_context->pause && m_context->singleStep == 0 )
		{
			timeStep = 0.0f;
		}

		float delay = 2.0f;

		if ( timeStep > 0.0f && m_time > delay )
		{
			float t = m_time - delay;

			b3Vec3 point;
			point.x = 2.0f * m_amplitude * cosf( t );
			point.y = m_amplitude * ( sinf( 2.0f * t ) + 1.0f );
			point.z = 0.0f;
			b3Quat rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 2.0f * t );

			b3Vec3 axis = b3RotateVector( rotation, { 0.0f, 1.0f, 0.0f } );
			DrawLine( m_scene, point - 0.5f * axis, point + 0.5f * axis, b3_colorPlum );
			DrawPoint( m_scene, point, 10.0f, b3_colorPlum );

			b3Body_SetTargetTransform( m_bodyId, { point, rotation }, timeStep, true );
		}

		Sample::Step();

		m_time += timeStep;
	}

	static Sample* Create( SampleContext* context )
	{
		return new Kinematic( context );
	}

	b3BodyId m_bodyId;
	float m_amplitude;
	float m_time;
};

static int sampleKinematic = SampleManager::Register( "Bodies", "Kinematic", Kinematic::Create );

class LockMixing : public Sample
{
public:
	explicit LockMixing( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 40.0f, b3Vec3_zero );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };

			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull hull = b3MakeBoxHull( 15.0f, 1.0f, 15.0f );
			b3CreateHullShape( groundId, &shapeDef, &hull.base );
		}

		b3BoxHull cube = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.name = "free";
			bodyDef.position = { 0.0f, 2.0f, 0.0f };

			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.name = "angular xz";
			bodyDef.position = { 2.0f, 2.0f, 0.0f };
			bodyDef.motionLocks.angularX = true;
			// bodyDef.motionLocks.angularY = true;
			bodyDef.motionLocks.angularZ = true;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.name = "linear xyz";
			bodyDef.position = { -2.0f, 2.0f, 0.0f };
			bodyDef.motionLocks.linearX = true;
			bodyDef.motionLocks.linearY = true;
			bodyDef.motionLocks.linearZ = true;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.name = "full";
			bodyDef.position = { 0.0f, 1.0f, 2.0f };
			bodyDef.motionLocks.linearX = true;
			bodyDef.motionLocks.linearY = true;
			bodyDef.motionLocks.linearZ = true;
			bodyDef.motionLocks.angularX = true;
			bodyDef.motionLocks.angularY = true;
			bodyDef.motionLocks.angularZ = true;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "static";
			bodyDef.position = { 0.0f, 1.0f, -3.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new LockMixing( context );
	}
};

static int sampleLockMixing = SampleManager::Register( "Bodies", "Lock Mixing", LockMixing::Create );

// A fully rotation locked body uses a zero inverse inertia tensor
class FixedRotation : public Sample
{
public:
	explicit FixedRotation( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 10.0f, b3Vec3_zero );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };

			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull hull = b3MakeBoxHull( 15.0f, 1.0f, 15.0f );
			b3CreateHullShape( groundId, &shapeDef, &hull.base );
		}

		b3Capsule capsule = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 0.3f };
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, 0.5f, 0.0f };

			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.3f, 0.5f, 0.0f };
			bodyDef.type = b3_dynamicBody;
			bodyDef.gravityScale = 0.0f;
			bodyDef.enableSleep = false;
			bodyDef.motionLocks.angularX = true;
			bodyDef.motionLocks.angularY = true;
			bodyDef.motionLocks.angularZ = true;

			capsule.radius = 0.2f;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new FixedRotation( context );
	}
};

static int sampleFixedRotation = SampleManager::Register( "Bodies", "Fixed Rotation", FixedRotation::Create );
