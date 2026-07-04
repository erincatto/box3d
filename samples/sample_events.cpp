// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "imgui.h"
#include "sample.h"
#include "gfx/draw.h"
#include "utils.h"

#include "gfx/keycodes.h"

#include "box3d/box3d.h"

#include <assert.h>

class SensorVisit : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new SensorVisit( context );
	}

	explicit SensorVisit( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 20.0f, { 0.0f, 5.0f, 0.0f } );
		}

		// Visitor
		b3BoxHull dynamicBox = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 12.5f, 0.0f };
		b3BodyId dynamicBody = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableSensorEvents = true;
		b3CreateHullShape( dynamicBody, &shapeDef, &dynamicBox.base );

		// Sensor
		b3BoxHull sensorBox = b3MakeBoxHull( 2.0f, 2.0f, 2.0f );

		bodyDef.type = b3_kinematicBody;
		bodyDef.position = { 0.0f, 2.0f, 0.0f };
		b3BodyId sensorBody = b3CreateBody( m_worldId, &bodyDef );
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		m_sensorShapeId = b3CreateHullShape( sensorBody, &shapeDef, &sensorBox.base );
	}

	void Render() override
	{
		DrawGroundGrid( 10 );
		Sample::Render();
	}

	void Step() override
	{
		Sample::Step();

		b3SensorEvents events = b3World_GetSensorEvents( m_worldId );

		for ( int i = 0; i < events.beginCount; ++i )
		{
			b3SensorBeginTouchEvent* event = events.beginEvents + i;
			if ( B3_ID_EQUALS( event->sensorShapeId, m_sensorShapeId ) )
			{
				b3BodyId bodyId = b3Shape_GetBody( event->visitorShapeId );
				b3DestroyBody( bodyId );
				break;
			}
		}
	}

	b3ShapeId m_sensorShapeId;
};

static int sampleSensorVisit = RegisterSample( "Events", "Sensor Visit", SensorVisit::Create );

class HitEvent : public Sample
{
public:
	enum
	{
		e_maxEvents = 32,
	};

	static Sample* Create( SampleContext* context )
	{
		return new HitEvent( context );
	}

	explicit HitEvent( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 100.0f, { 0.0f, 5.0f, 0.0f } );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			constexpr int materialCount = 6;
			m_gridMesh = b3CreateGridMesh( 20, 20, 8.0f, materialCount, true );
			b3ShapeDef shapeDef = b3DefaultShapeDef();

			b3SurfaceMaterial materials[materialCount];
			for ( int i = 0; i < materialCount; ++i )
			{
				materials[i] = b3DefaultSurfaceMaterial();
				materials[i].userMaterialId = i + 1;
			}

			shapeDef.materials = materials;
			shapeDef.materialCount = materialCount;

			b3CreateMeshShape( groundId, &shapeDef, m_gridMesh, b3Vec3_one );
			// b3BoxHull groundBox = b3MakeTransformedBoxHull( 80.0f, 1.0f, 80.0f, { { 0.0f, -1.0f, 0.0f }, b3Quat_identity } );
			// b3CreateHullShape( groundId, &shapeDef, &groundBox.base );
		}

		b3WeldJointDef jointDef = b3DefaultWeldJointDef();
		jointDef.angularHertz = 10.0f;
		jointDef.angularDampingRatio = 2.0f;

		float r = 0.75f;
		float y = r;
		float l = 1.5f;
		float offset = 0.05f;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableHitEvents = true;
		shapeDef.baseMaterial.rollingResistance = 0.2f;
		shapeDef.baseMaterial.userMaterialId = 42;
		shapeDef.updateBodyMass = false;

		b3Vec3 origin = { 0.0f, 0.0f, 0.0f };

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3OffsetPos( b3Pos_zero, origin );

		b3BodyId prevBodyId = {};
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
		int shapeCount = 22;
		float velocityScale = 0.5f;
		int shapesPerBody = 3;

		for ( int i = 0; i < shapeCount; ++i )
		{
			b3Capsule capsule = { { offset, y, 0.0f }, { 0.0f, y + l, -offset }, r };
			b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );

			if ( ( i + 1 ) % shapesPerBody == 0 || i == shapeCount - 1 )
			{
				b3Body_ApplyMassFromShapes( bodyId );

				b3Pos center = b3Body_GetWorldCenter( bodyId );
				b3Vec3 omega = { 0.0f, 0.0f, -1.0f * velocityScale };
				b3Vec3 v = b3Cross( omega, b3SubPos( center, b3OffsetPos( b3Pos_zero, origin ) ) );
				b3Body_SetAngularVelocity( bodyId, omega );
				b3Body_SetLinearVelocity( bodyId, v );

				if ( i < shapeCount - 1 )
				{
					prevBodyId = bodyId;

					if ( i < shapeCount - 1 )
					{
						bodyId = b3CreateBody( m_worldId, &bodyDef );

						if ( B3_IS_NON_NULL( prevBodyId ) )
						{
							jointDef.base.bodyIdA = prevBodyId;
							jointDef.base.bodyIdB = bodyId;
							jointDef.base.localFrameA.p = { 0.0f, y + l + r, 0.0f };
							jointDef.base.localFrameB.p = { 0.0f, y + l + r, 0.0f };

							b3CreateWeldJoint( m_worldId, &jointDef );
						}

						velocityScale *= 0.75f;
					}
				}
			}

			y += l + 2.0f * r;
			r = 0.95f * r;
			offset = -offset;
		}

		m_eventCount = 0;
	}

	~HitEvent() override
	{
		b3DestroyMesh( m_gridMesh );
	}

	void Render() override
	{
		Sample::Render();
		b3Transform transform = { { 0.0f, 0.1f, 0.0f }, b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( transform ), 4.0f );

		for ( int i = 0; i < m_eventCount; ++i )
		{
			// void DrawLine(Scene * Scene, b3Vector3 Vertex1, b3Vector3 Vertex2, b3Color Color);
			// void DrawPoint(Scene * Scene, b3Vector3 Point, b3Color Color, float Size);

			b3Pos p1 = m_events[i].point;
			b3Pos p2 = b3OffsetPos( p1, -m_events[i].approachSpeed * m_events[i].normal );

			DrawPoint( p1, 10.0f, MakeColor( b3_colorYellow ) );
			DrawLine( p1, p2, MakeColor( b3_colorYellow ) );
			DrawString3D( p1, MakeColor( b3_colorWhite ), "%.1f, %d", m_events[i].approachSpeed, m_events[i].userMaterialIdA );
		}

		DrawTextLine( "event count = %d", m_eventCount );
	}

	void Step() override
	{
		Sample::Step();

		b3ContactEvents events = b3World_GetContactEvents( m_worldId );
		for ( int i = 0; i < events.hitCount && m_eventCount < e_maxEvents; ++i )
		{
			m_events[m_eventCount] = events.hitEvents[i];
			m_eventCount += 1;
		}
	}

	static constexpr int m_maxEvents = 32;
	b3MeshData* m_gridMesh;
	b3ContactHitEvent m_events[m_maxEvents] = {};
	int m_eventCount;
};

static int sampleHitEvent = RegisterSample( "Events", "Hit", HitEvent::Create );

class MoveEvent : public Sample
{
public:
	explicit MoveEvent( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 40.0f, { 0.0f, 5.0f, 0.0f } );
			
		}

		AddGroundBox( 40.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3Pos pivot = { 0.0f, 1.0f, 0.0f };
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = pivot;
		bodyDef.name = "big box";
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		m_localPivot = b3Body_GetLocalPoint( m_bodyId, pivot );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableHitEvents = true;
		b3BoxHull dynamicBox = b3MakeTransformedBoxHull( 0.5f, 10.0f, 0.5f, { { 0.0f, 10.0f, 0.0f }, b3Quat_identity } );
		b3CreateHullShape( m_bodyId, &shapeDef, &dynamicBox.base );

		b3Pos center = b3Body_GetWorldCenter( m_bodyId );

		b3Vec3 r = b3SubPos( pivot, center );
		float rr = b3LengthSquared( r );
		if ( rr > 0.0f )
		{
			b3Vec3 v = { -10.0f, 0.0f, 0.0f };
			b3Vec3 omega = ( 1.0f / rr ) * b3Cross( v, r );
			b3Body_SetAngularVelocity( m_bodyId, omega );
			b3Body_SetLinearVelocity( m_bodyId, v );
		}
	}

	void Render() override
	{
		Sample::Render();

		b3Vec3 vp = b3Body_GetLocalPointVelocity( m_bodyId, m_localPivot );
		b3Vec3 v = b3Body_GetLinearVelocity( m_bodyId );
		b3Vec3 omega = b3Body_GetAngularVelocity( m_bodyId );

		DrawTextLine( "vp = [%.2f, %.2f, %.2f], v = [%.2f, %.2f, %.2f], w = [%.2f, %.2f, %.2f]", vp.x, vp.y, vp.z, v.x, v.y, v.z,
					  omega.x, omega.y, omega.z );
	}

	void Step() override
	{
		Sample::Step();

		b3BodyEvents moveEvents = b3World_GetBodyEvents( m_worldId );
		for ( int i = 0; i < moveEvents.moveCount; ++i )
		{
			b3BodyId body = moveEvents.moveEvents[i].bodyId;
			if ( moveEvents.moveEvents[i].fellAsleep )
			{
				DrawTextLine( "%s fell asleep", b3Body_GetName( body ) );
			}
			else
			{
				DrawTextLine( "%s moved", b3Body_GetName( body ) );
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new MoveEvent( context );
	}

	b3BodyId m_bodyId;
	b3Vec3 m_localPivot;
};

static int sampleMoveEvent = RegisterSample( "Events", "Move", MoveEvent::Create );

// This sample shows how to break joints when the internal reaction force becomes large. Instead of polling, this uses events.
class JointEvent : public Sample
{
public:
	enum
	{
		e_count = 6
	};

	explicit JointEvent( SampleContext* context )
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

		for ( int i = 0; i < e_count; ++i )
		{
			m_jointIds[i] = b3_nullJointId;
		}

		b3Vec3 position = { -12.5f, 10.0, 0.0f };
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.enableSleep = false;

		b3BoxHull box = b3MakeBoxHull( 1.0f, 1.0f, 0.5f );

		int index = 0;

		float forceThreshold = 3000.0f;
		float torqueThreshold = 10000.0f;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 1.0f;

		// distance joint
		{
			assert( index < e_count );

			bodyDef.position = b3OffsetPos( b3Pos_zero, position );
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );

			float length = 2.0f;
			b3Pos pivot1 = { position.x, position.y + 1.0f + length, 0.0f };
			b3Pos pivot2 = { position.x, position.y + 1.0f, 0.0f };
			b3DistanceJointDef jointDef = b3DefaultDistanceJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot1 );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot2 );
			jointDef.length = length;
			jointDef.base.forceThreshold = forceThreshold;
			jointDef.base.torqueThreshold = torqueThreshold;
			jointDef.base.collideConnected = true;
			jointDef.base.userData = (void*)(intptr_t)index;
			m_jointIds[index] = b3CreateDistanceJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

		// motor joint
#if 0
		{
			assert( index < e_count );

			bodyDef.position = position;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );

			b3MotorJointDef jointDef = b3DefaultMotorJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = position;
			jointDef.maxForce = 1000.0f;
			jointDef.maxTorque = 20.0f;
			jointDef.base.forceThreshold = forceThreshold;
			jointDef.base.torqueThreshold = torqueThreshold;
			jointDef.base.collideConnected = true;
			jointDef.base.userData = (void*)(intptr_t)index;
			m_jointIds[index] = b3CreateMotorJoint( m_worldId, &jointDef );
		}
#else
		m_jointIds[index] = {};
#endif

		position.x += 5.0f;
		++index;

		// prismatic joint
		{
			assert( index < e_count );

			bodyDef.position = b3OffsetPos( b3Pos_zero, position );
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );

			b3Pos pivot = { position.x - 1.0f, position.y, 0.0f };
			b3PrismaticJointDef jointDef = b3DefaultPrismaticJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
			jointDef.base.forceThreshold = forceThreshold;
			jointDef.base.torqueThreshold = torqueThreshold;
			jointDef.base.collideConnected = true;
			jointDef.base.userData = (void*)(intptr_t)index;
			m_jointIds[index] = b3CreatePrismaticJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

		// revolute joint
		{
			assert( index < e_count );

			bodyDef.position = b3OffsetPos( b3Pos_zero, position );
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );

			b3Pos pivot = { position.x - 1.0f, position.y, 0.0f };
			b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
			jointDef.base.forceThreshold = forceThreshold;
			jointDef.base.torqueThreshold = torqueThreshold;
			jointDef.base.collideConnected = true;
			jointDef.base.userData = (void*)(intptr_t)index;
			m_jointIds[index] = b3CreateRevoluteJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

		// weld joint
		{
			assert( index < e_count );

			bodyDef.position = b3OffsetPos( b3Pos_zero, position );
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );

			b3Pos pivot = { position.x - 1.0f, position.y, 0.0f };
			b3WeldJointDef jointDef = b3DefaultWeldJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
			jointDef.angularHertz = 2.0f;
			jointDef.angularDampingRatio = 0.5f;
			jointDef.base.forceThreshold = forceThreshold;
			jointDef.base.torqueThreshold = torqueThreshold;
			jointDef.base.collideConnected = true;
			jointDef.base.userData = (void*)(intptr_t)index;
			m_jointIds[index] = b3CreateWeldJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

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
			m_jointIds[index] = b3CreateWheelJoint( m_worldId, &jointDef );
		}
#else
		m_jointIds[index] = {};
#endif

		position.x += 5.0f;
		++index;
	}

	void Step() override
	{
		Sample::Step();

		// Process joint events
		b3JointEvents events = b3World_GetJointEvents( m_worldId );
		for ( int i = 0; i < events.count; ++i )
		{
			// Destroy the joint if it is still valid
			const b3JointEvent* event = events.jointEvents + i;

			if ( b3Joint_IsValid( event->jointId ) )
			{
				int index = (int)(intptr_t)event->userData;
				assert( 0 <= index && index < e_count );
				b3DestroyJoint( event->jointId, true );
				m_jointIds[index] = b3_nullJointId;
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new JointEvent( context );
	}

	b3JointId m_jointIds[e_count];
};

static int sampleJointEvent = RegisterSample( "Events", "Joint", JointEvent::Create );

class PersistentContact : public Sample
{
public:
	explicit PersistentContact( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 40.0f, { 0.0f, 5.0f, 0.0f } );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );
			m_meshData = b3CreateGridMesh( 20, 20, 2.0f, 2, true );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateMeshShape( groundId, &shapeDef, m_meshData, b3Vec3_one );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -18.0f, 1.0f, 0.5f };
			bodyDef.linearVelocity = { 4.0f, 0.0f, 0.0f };

			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 20.0f;
			shapeDef.enableContactEvents = true;
			shapeDef.baseMaterial.rollingResistance = 0.01f;
			b3Sphere sphere = { b3Vec3_zero, 0.5f };
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		}

		m_contactId = b3_nullContactId;
	}

	~PersistentContact() override
	{
		b3DestroyMesh( m_meshData );
	}

	void Step() override
	{
		Sample::Step();

		b3ContactEvents events = b3World_GetContactEvents( m_worldId );
		for ( int i = 0; i < events.beginCount && i < 1; ++i )
		{
			m_contactId = events.beginEvents[i].contactId;
		}

		for ( int i = 0; i < events.endCount; ++i )
		{
			if ( B3_ID_EQUALS( m_contactId, events.endEvents[i].contactId ) )
			{
				m_contactId = b3_nullContactId;
				break;
			}
		}

		if ( B3_IS_NON_NULL( m_contactId ) && b3Contact_IsValid( m_contactId ) )
		{
			b3ContactData data = b3Contact_GetData( m_contactId );
			b3Pos centerOfMass = b3Body_GetWorldCenter( b3Shape_GetBody( data.shapeIdA ) );

			for ( int i = 0; i < data.manifoldCount; ++i )
			{
				const b3Manifold* manifold = data.manifolds + i;
				b3Vec3 normal = manifold->normal;
				for ( int j = 0; j < manifold->pointCount; ++j )
				{
					const b3ManifoldPoint* manifoldPoint = manifold->points + j;
					b3Pos p1 = b3OffsetPos( centerOfMass, manifoldPoint->anchorA );
					b3Pos p2 = b3OffsetPos( p1, manifoldPoint->totalNormalImpulse * normal );
					DrawLine( p1, p2, MakeColor( b3_colorCrimson ) );
					DrawPoint( p1, 6.0f, MakeColor( b3_colorCrimson ) );
					DrawString3D( p1, MakeColor( b3_colorGray ), "%.2f", manifoldPoint->totalNormalImpulse );
				}
			}
		}
		else
		{
			m_contactId = b3_nullContactId;
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new PersistentContact( context );
	}

	b3ContactId m_contactId;
	b3MeshData* m_meshData;
};

static int samplePersistentContact = RegisterSample( "Events", "Persistent Contact", PersistentContact::Create );

class SensorHits : public Sample
{
public:
	explicit SensorHits( SampleContext* context )
		: Sample( context )
		, m_transforms{}
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 40.0f, { 0.0f, 5.0f, 0.0f } );
			
		}

		AddGroundBox( 10.0f );

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "ground";

			groundId = b3CreateBody( m_worldId, &bodyDef );
			b3ShapeDef shapeDef = b3DefaultShapeDef();

			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, 5.0f, 5.0f, { { 10.0f, 5.0f, 0.0f }, b3Quat_identity } );
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		m_gridMesh = b3CreateGridMesh( 2, 2, 5.0f, 0, true );

		// Static sensor
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "static sensor";
			bodyDef.position = { -4.0f, 6.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI );

			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;

			m_staticSensorId = b3CreateMeshShape( bodyId, &shapeDef, m_gridMesh, b3Vec3_one );
		}

		// Kinematic sensor
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "kinematic sensor";
			bodyDef.type = b3_kinematicBody;
			bodyDef.position = { 0.0f, 6.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI );
			bodyDef.linearVelocity = { 0.5f, 0.0f };

			m_kinematicBodyId = b3CreateBody( m_worldId, &bodyDef );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;

			m_kinematicSensorId = b3CreateMeshShape( m_kinematicBodyId, &shapeDef, m_gridMesh, b3Vec3_one );
		}

		// Dynamic sensor
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "dynamic sensor";
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 4.0f, 1.0f };

			m_dynamicBodyId = b3CreateBody( m_worldId, &bodyDef );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;

			b3Capsule capsule = { { 0.0f, 1.0f }, { 0.0f, 9.0f }, 0.1f };
			m_dynamicSensorId = b3CreateCapsuleShape( m_dynamicBodyId, &shapeDef, &capsule );

			b3Pos pivot = b3OffsetPos( bodyDef.position, b3Vec3{ 0.0f, 6.0f, 0.0f } );
			b3PrismaticJointDef jointDef = b3DefaultPrismaticJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = m_dynamicBodyId;
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( groundId, pivot );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( m_dynamicBodyId, pivot );
			jointDef.enableMotor = true;
			jointDef.maxMotorForce = 1000.0f;
			jointDef.motorSpeed = 0.5f;

			m_jointId = b3CreatePrismaticJoint( m_worldId, &jointDef );
		}

		m_beginCount = 0;
		m_endCount = 0;
		m_bodyId = {};
		m_shapeId = {};
		m_transformCount = 0;
		m_isBullet = true;

		Launch();
	}

	~SensorHits() override
	{
		b3DestroyMesh( m_gridMesh );
	}

	void Launch()
	{
		if ( B3_IS_NON_NULL( m_bodyId ) )
		{
			b3DestroyBody( m_bodyId );
		}

		m_transformCount = 0;
		m_beginCount = 0;
		m_endCount = 0;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { -26.7f, 6.0f };
		float speed = RandomFloatRange( 200.0f, 300.0f );
		bodyDef.linearVelocity = { speed, 0.0f };
		bodyDef.isBullet = m_isBullet;
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableSensorEvents = true;
		shapeDef.baseMaterial.friction = 0.8f;
		shapeDef.baseMaterial.rollingResistance = 0.01f;
		b3Sphere sphere = { { 0.0f, 0.0f }, 0.25f };
		m_shapeId = b3CreateSphereShape( m_bodyId, &shapeDef, &sphere );
	}

	bool DrawControls() override
	{
		ImGui::Checkbox( "Bullet", &m_isBullet );

		if ( ImGui::Button( "Launch" ) || IsKeyDown( KEY_B ) )
		{
			Launch();
		}

		return true;
	}

	void CollectTransforms( b3ShapeId sensorShapeId )
	{
		constexpr int capacity = 5;
		b3ShapeId visitorIds[capacity];
		int count = b3Shape_GetSensorData( sensorShapeId, visitorIds, capacity );

		for ( int i = 0; i < count && m_transformCount < m_transformCapacity; ++i )
		{
			b3BodyId sensorBodyId = b3Shape_GetBody( sensorShapeId );
			b3WorldTransform t = b3Body_GetTransform( sensorBodyId );
			t.p = b3Body_GetWorldCenter( sensorBodyId );
			m_transforms[m_transformCount] = t;
			m_transformCount += 1;
		}
	}

	void Step() override
	{
		b3Pos p = b3Body_GetPosition( m_kinematicBodyId );
		if ( p.x > 1.0f )
		{
			b3Body_SetLinearVelocity( m_kinematicBodyId, { -0.5f, 0.0f } );
		}
		else if ( p.x < -1.0f )
		{
			b3Body_SetLinearVelocity( m_kinematicBodyId, { 0.5f, 0.0f } );
		}

		float x = b3PrismaticJoint_GetTranslation( m_jointId );
		if ( x > 1.0f )
		{
			b3PrismaticJoint_SetMotorSpeed( m_jointId, -0.5f );
		}
		else if ( x < -1.0f )
		{
			b3PrismaticJoint_SetMotorSpeed( m_jointId, 0.5f );
		}

		Sample::Step();

		for ( int i = 0; i < m_transformCount; ++i )
		{
			DrawAxes( m_transforms[i], 0.1f );
		}

		b3SensorEvents sensorEvents = b3World_GetSensorEvents( m_worldId );
		m_beginCount += sensorEvents.beginCount;
		m_endCount += sensorEvents.endCount;

		for ( int i = 0; i < sensorEvents.beginCount; ++i )
		{
			const b3SensorBeginTouchEvent* event = sensorEvents.beginEvents + i;
			CollectTransforms( event->sensorShapeId );
		}

		DrawTextLine( "begin touch count = %d", m_beginCount );
		DrawTextLine( "end touch count = %d", m_endCount );
	}

	static Sample* Create( SampleContext* context )
	{
		return new SensorHits( context );
	}

	b3MeshData* m_gridMesh;

	b3ShapeId m_staticSensorId;
	b3ShapeId m_kinematicSensorId;
	b3ShapeId m_dynamicSensorId;

	b3BodyId m_kinematicBodyId;
	b3BodyId m_dynamicBodyId;
	b3JointId m_jointId;

	b3BodyId m_bodyId;
	b3ShapeId m_shapeId;

	static constexpr int m_transformCapacity = 20;
	int m_transformCount;
	b3WorldTransform m_transforms[m_transformCapacity];

	bool m_isBullet;
	int m_beginCount;
	int m_endCount;
};

static int sampleSensorHits = RegisterSample( "Events", "Sensor Hits", SensorHits::Create );
