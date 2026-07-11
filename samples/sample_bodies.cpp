// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/draw.h"
#include "gfx/keycodes.h"
#include "imgui.h"
#include "sample.h"

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

		b3BodyId groundId = AddGroundBox( 20.0f );

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
			b3Pos pivot = { -2.0f, 5.0f, 0.0f };
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
			b3Pos anchor = { 0.0f, 5.0f, 0.0f };
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

	bool DrawControls() override
	{
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

		return true;
	}

	void Step() override
	{
		// Drive the kinematic body.
		if ( m_type == b3_kinematicBody )
		{
			b3Pos p = b3Body_GetPosition( m_platformId );
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

static int sampleBodyType = RegisterSample( "Bodies", "Body Type", BodyType::Create );

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

		AddGroundBox( 10.0f );

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

	static Sample* Create( SampleContext* context )
	{
		return new SpinningBooks( context );
	}
};

static int sampleBook = RegisterSample( "Bodies", "Spinning Book", SpinningBooks::Create );

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

		AddGroundBox( 20.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 2.0f, 0.0f };
		bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, -0.5f * B3_PI );
		bodyDef.gravityScale = 0.0f;
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.updateBodyMass = false;
		b3HullData* cylinder = b3CreateCylinder( 0.6f, 0.15f, 0.0f, 32 );
		b3BoxHull box = b3MakeBoxHull( 1.0f, 0.05f, 0.1f );
		b3CreateHullShape( m_bodyId, &shapeDef, cylinder );
		b3CreateHullShape( m_bodyId, &shapeDef, &box.base );
		b3Body_ApplyMassFromShapes( m_bodyId );

		// Set the angular velocity after creating the shapes and the local center of mass is fixed.
		b3Body_SetAngularVelocity( m_bodyId, { 0.01f, 0.01f, 10.0f } );

		b3DestroyHull( cylinder );
	}

	void Step() override
	{
		Sample::Step();

		b3Pos c = b3Body_GetWorldCenter( m_bodyId );
		DrawTextLine( "center %.3g %.3g %.3g", c.x, c.y, c.z );
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new GyroscopicTorque( sampleContext );
	}

	b3BodyId m_bodyId;
};

static int sampleGyroscopicTorque = RegisterSample( "Bodies", "Gyroscopic Torque", GyroscopicTorque::Create );

// Spinning tops. Ported from PEEL.
// Each top is tilted and spun about the world up axis. Offsetting the spin axis from the
// symmetry axis is what makes them precess under gravity instead of spinning true.
// todo disabling this until issue #65 is fixed
#if 0
class GyroscopicPrecession : public Sample
{
public:
	explicit GyroscopicPrecession( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 40.0f, 30.0f, 75.0f, { 0.0f, 2.0f, 0.0f } );
		}

		AddGroundBox( 40.0f );

		// Top shape: a wide n-gon rim up top and a point at the origin, so it balances on its tip.
		constexpr int numSegs = 7;
		constexpr float r = 2.0f;
		constexpr float h = 2.0f;
		b3Vec3 hullPoints[numSegs + 1];
		const float dphi = 2.0f * B3_PI / numSegs;
		for ( int i = 0; i < numSegs; ++i )
		{
			hullPoints[i] = { r * cosf( i * dphi ), h, r * sinf( i * dphi ) };
		}
		hullPoints[numSegs] = b3Vec3_zero;
		b3HullData* hull = b3CreateHull( hullPoints, numSegs + 1, numSegs + 1 );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		// Tilt the symmetry axis, then spin about the world up axis so the offset drives precession.
		b3Quat rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 15.0f * B3_PI / 180.0f );
		b3Vec3 angularVelocity = b3RotateVector( rotation, { 0.0f, 75.0f, 0.0f } );

		constexpr int count = 8;
		constexpr float separation = 6.0f;
		for ( int x = 0; x < count; ++x )
		{
			for ( int z = 0; z < count; ++z )
			{
				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.type = b3_dynamicBody;
				bodyDef.position = { ( x - count / 2 ) * separation, h, ( z - count / 2 ) * separation };
				bodyDef.rotation = rotation;

				// The spin rate exceeds the default cap, so bypass it as the test intends.
				bodyDef.allowFastRotation = true;

				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
				b3CreateHullShape( bodyId, &shapeDef, hull );

				b3Body_SetAngularVelocity( bodyId, angularVelocity );
			}
		}

		b3DestroyHull( hull );
	}

	static Sample* Create( SampleContext* context )
	{
		return new GyroscopicPrecession( context );
	}
};

static int sampleGyroscopicPrecession = RegisterSample( "Bodies", "Gyroscopic Precession", GyroscopicPrecession::Create );
#endif

class Weeble : public Sample
{
public:
	explicit Weeble( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 25.0f, 25.0f, b3Pos_zero );
		}

		AddGroundBox( 30.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 3.0f, 0.0f };
		m_weebleId = b3CreateBody( m_worldId, &bodyDef );

		b3Capsule capsule = { { 0.0f, -1.0f }, { 0.0f, 1.0f }, 1.0f };
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3CreateCapsuleShape( m_weebleId, &shapeDef, &capsule );

		float mass = b3Body_GetMass( m_weebleId );
		b3Matrix3 inertiaTensor = b3Body_GetLocalRotationalInertia( m_weebleId );

		b3Vec3 offset = { 0.0f, -1.5f, 0.0f };

		// See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
		inertiaTensor = b3AddMM( inertiaTensor, b3Steiner( mass, offset ) );

		b3MassData massData = {
			.mass = mass,
			.center = offset,
			.inertia = inertiaTensor,
		};

		b3Body_SetMassData( m_weebleId, massData );

		m_explosionPosition = { 0.0f, -0.1f, 0.0f };
		m_explosionRadius = 8.0f;
		m_explosionMagnitude = 20000.0f;
	}

	bool DrawControls() override
	{
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
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		ImGui::SliderFloat( "Magnitude", &m_explosionMagnitude, -100000.0f, 100000.0f, "%.0f" );

		ImGui::PopItemWidth();
		return true;
	}

	void Step() override
	{
		Sample::Step();

		b3Sphere sphere = { b3Vec3_zero, m_explosionRadius };
		DrawWireSphere( { m_explosionPosition, b3Quat_identity }, &sphere, 64, MakeColor( b3_colorAzure ) );

		// This shows how to get the velocity of a point on a body
		b3Vec3 localPoint = { 0.0f, 2.0f, 0.0f };
		b3Pos worldPoint = b3Body_GetWorldPoint( m_weebleId, localPoint );

		b3Vec3 v1 = b3Body_GetLocalPointVelocity( m_weebleId, localPoint );
		b3Vec3 v2 = b3Body_GetWorldPointVelocity( m_weebleId, worldPoint );

		b3Vec3 offset = { 0.05f, 0.0f };
		DrawLine( worldPoint, worldPoint + v1, MakeColor( b3_colorRed ) );
		DrawLine( worldPoint + offset, worldPoint + v2 + offset, MakeColor( b3_colorGreen ) );
	}

	static Sample* Create( SampleContext* context )
	{
		return new Weeble( context );
	}

	b3BodyId m_weebleId;
	b3Pos m_explosionPosition;
	float m_explosionRadius;
	float m_explosionMagnitude;
};

static int sampleWeeble = RegisterSample( "Bodies", "Weeble", Weeble::Create );

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
			m_camera->SetView( 45.0f, 25.0f, 10.0f, b3Pos_zero );
		}

		AddGroundBox( 20.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		float linkRadius = 0.1f;
		float linkLength = 5.0f * linkRadius;
		b3Capsule capsule = { { 0.0f, 0.0f, 0.0f }, { 0.0f, -linkLength, 0.0f }, linkRadius };

		b3BodyId parentId = {};
		for ( int link = 0; link < e_count; ++link )
		{
			bodyDef.position = { 0.0f, ( float( e_count ) - link ) * linkLength + 1.0f, 0.0f };
			bodyDef.type = B3_IS_NULL( parentId ) ? b3_kinematicBody : b3_dynamicBody;
			b3BodyId childId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateCapsuleShape( childId, &shapeDef, &capsule );
			m_bodyIds[link] = childId;

			if ( B3_IS_NON_NULL( parentId ) )
			{
				b3WeldJointDef jointDef = b3DefaultWeldJointDef();
				jointDef.base.bodyIdA = parentId;
				jointDef.base.bodyIdB = childId;
				jointDef.base.localFrameA.p = { 0.0f, -linkLength, 0.0f };
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

	bool DrawControls() override
	{
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

		return true;
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

static int sampleDisable = RegisterSample( "Bodies", "Disable", DisableBody::Create );

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

		m_baseTranslation = b3Pos_zero;
		m_baseX = 0;
		m_baseY = 0;
		m_origin = b3Pos_zero;
		m_tracking = false;
	}

	~BodyCast() override
	{
		b3DestroyHull( m_cylinder );
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 && modifiers == MOD_SHIFT )
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

	void MouseUp( b3Vec2 p, int button ) override
	{
		if ( button == 0 )
		{
			m_tracking = false;
		}
	}

	void MouseMove( b3Vec2 p ) override
	{
		if ( m_tracking )
		{
			PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
			b3Pos origin = pickRay.origin + 10.0f * b3Normalize( pickRay.translation );
			m_transform.p = m_baseTranslation + b3SubPos( origin, m_origin );
		}
	}

	void Render() override
	{
		Sample::Render();

		DrawGroundGrid( 10 );
		b3WorldTransform t = { { 0.0f, 0.1f, 0.0f }, b3Quat_identity };
		DrawAxes( t, 4.0f );

		DrawHull( m_transform, m_cylinder, MakeColor( b3_colorBlue ) );
	}

	void Step() override
	{
		// Cast ray
		{
			b3Pos origin = { -9.75f, 3.0f, -4.0f };
			b3Vec3 translation = { 0.0f, 0.0f, 8.0f };
			b3QueryFilter filter = b3DefaultQueryFilter();
			float maxFraction = 1.0f;
			b3BodyCastResult result = b3Body_CastRay( m_bodyId, origin, translation, filter, maxFraction, m_transform );

			DrawLine( origin, origin + maxFraction * translation, MakeColor( b3_colorCyan ) );

			if ( result.hit )
			{
				b3Pos hitPoint = result.point;
				DrawLine( hitPoint, hitPoint + 0.2f * result.normal, MakeColor( b3_colorYellow ) );
				DrawPoint( hitPoint, 10.0f, MakeColor( b3_colorYellow ) );
			}

			DrawPoint( origin, 10.0f, MakeColor( b3_colorGreen ) );
			DrawPoint( origin + translation, 10.0f, MakeColor( b3_colorRed ) );
		}

		// Cast sphere
		{
			b3Pos origin = { -14.5f, 2.5f, 0.5f };
			b3Sphere sphere = { b3Vec3_zero, 0.2f };

			b3ShapeProxy proxy = { &sphere.center, 1, sphere.radius };
			b3Vec3 translation = { 8.0f, 0.0f, 0.0f };
			b3QueryFilter filter = b3DefaultQueryFilter();
			float maxFraction = 1.0f;
			bool canEncroach = true;
			b3BodyCastResult result =
				b3Body_CastShape( m_bodyId, origin, &proxy, translation, filter, maxFraction, canEncroach, m_transform );

			b3Pos sphereCenter = b3OffsetPos( origin, sphere.center );
			if ( result.hit )
			{
				b3WorldTransform t = { origin + result.fraction * translation, b3Quat_identity };
				DrawSolidSphere( t, sphere, MakeColor( b3_colorGreen ) );
				b3Pos hitPoint = result.point;
				DrawLine( hitPoint, hitPoint + 0.2f * result.normal, MakeColor( b3_colorYellow ) );
			}
			else
			{
				b3WorldTransform t = { origin + maxFraction * translation, b3Quat_identity };
				DrawSolidSphere( t, sphere, MakeColor( b3_colorWhite ) );
			}

			DrawLine( sphereCenter, sphereCenter + maxFraction * translation, MakeColor( b3_colorWhite ) );
			DrawPoint( sphereCenter, 10.0f, MakeColor( b3_colorGreen ) );
			DrawPoint( sphereCenter + maxFraction * translation, 10.0f, MakeColor( b3_colorRed ) );
		}

		// Overlap capsule
		{
			b3Pos origin = { -10.0f, 1.0f, 0.5f };
			b3Capsule capsule = { { -.5f, 1.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.5f };
			b3ShapeProxy proxy = { &capsule.center1, 2, capsule.radius };
			bool overlaps = b3Body_OverlapShape( m_bodyId, origin, &proxy, b3DefaultQueryFilter(), m_transform );

			if ( overlaps )
			{
				DrawSolidCapsule( { origin, b3Quat_identity }, capsule, MakeColor( b3_colorGreen ) );
			}
			else
			{
				DrawSolidCapsule( { origin, b3Quat_identity }, capsule, MakeColor( b3_colorGray ) );
			}
		}

		// Collide capsule
		{
			b3Pos origin = { -10.0f, 2.0f, -0.75f };
			b3Capsule capsule = { { -0.25f, 0.0f, 0.0f }, { 0.25f, 1.0f, 0.0f }, 0.3f };
			b3BodyPlaneResult bodyPlanes[4];
			int count = b3Body_CollideMover( m_bodyId, bodyPlanes, 4, origin, &capsule, b3DefaultQueryFilter(), m_transform );
			DrawSolidCapsule( { origin, b3Quat_identity }, capsule, MakeColor( b3_colorPurple ) );

			for ( int i = 0; i < count; ++i )
			{
				b3PlaneResult result = bodyPlanes[i].result;
				DrawPlane( result.plane.normal, b3ToPos( result.point ), MakeColor( b3_colorOrange ) );
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BodyCast( context );
	}

	b3HullData* m_cylinder;
	b3BodyId m_bodyId;
	b3WorldTransform m_transform;

	b3Pos m_baseTranslation;
	b3Pos m_origin;
	int m_baseX;
	int m_baseY;
	bool m_tracking;
};

static int sampleBodyCast = RegisterSample( "Bodies", "Cast", BodyCast::Create );

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

		AddGroundBox( 20.0f );

		m_amplitude = 2.0f;

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_kinematicBody;
			bodyDef.position.x = 2.0f * m_amplitude;
			bodyDef.position.y = m_amplitude + 1.0f;

			m_bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 0.1f, 1.0f, 0.2f );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( m_bodyId, &shapeDef, &box.base );
		}

		m_time = 0.0f;
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

			b3Pos point;
			point.x = 2.0f * m_amplitude * cosf( t );
			point.y = m_amplitude * ( sinf( 2.0f * t ) + 1.0f ) + 1.0f;
			point.z = 0.0f;
			b3Quat rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 2.0f * t );

			b3Vec3 axis = b3RotateVector( rotation, { 0.0f, 1.0f, 0.0f } );
			DrawLine( point - 0.5f * axis, point + 0.5f * axis, MakeColor( b3_colorPlum ) );
			DrawPoint( point, 10.0f, MakeColor( b3_colorPlum ) );

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

static int sampleKinematic = RegisterSample( "Bodies", "Kinematic", Kinematic::Create );

class LockMixing : public Sample
{
public:
	explicit LockMixing( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 40.0f, b3Pos_zero );
		}

		AddGroundBox( 20.0f );

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

static int sampleLockMixing = RegisterSample( "Bodies", "Lock Mixing", LockMixing::Create );

// A fully rotation locked body uses a zero inverse inertia tensor
class FixedRotation : public Sample
{
public:
	explicit FixedRotation( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 10.0f, b3Pos_zero );
		}

		AddGroundBox( 20.0f );

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

static int sampleFixedRotation = RegisterSample( "Bodies", "Fixed Rotation", FixedRotation::Create );
