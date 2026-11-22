// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "dynamic_mover.h"

#include "gfx/draw.h"

#include "box3d/box3d.h"
#include "box3d/math_functions.h"

#include <float.h>

struct CastContext
{
	DynamicMoverCastResult result;
	b3BodyId moverId;
};

static float CastCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction,
						   uint64_t userMaterialId, int triangleIndex, int childIndex,
						   void* context )
{
	(void)triangleIndex;
	(void)childIndex;

	CastContext* castContext = (CastContext*)context;
	b3BodyId bodyId = b3Shape_GetBody( shapeId );

	// The probes start inside or on the capsule. Sensors can't hold up the mover.
	if ( B3_ID_EQUALS( bodyId, castContext->moverId ) || b3Shape_IsSensor( shapeId ) )
	{
		return -1.0f;
	}

	DynamicMoverCastResult* result = &castContext->result;
	result->point = point;
	result->normal = normal;
	result->bodyId = bodyId;
	result->userMaterialId = userMaterialId;
	result->fraction = fraction;
	result->hit = true;
	return fraction;
}

DynamicMover::DynamicMover()
{
	DynamicMoverDef def;
	m_jumpSpeed = def.jumpSpeed;
	m_maxSpeed = def.maxSpeed;
	m_minSpeed = def.minSpeed;
	m_stopSpeed = def.stopSpeed;
	m_accelerate = def.accelerate;
	m_airSteer = def.airSteer;
	m_friction = def.friction;
	m_gravityScale = def.gravityScale;
	m_maxGroundForce = def.maxGroundForce;
	m_maxAirForce = def.maxAirForce;
	m_pogoRestLength = def.pogoRestLength;
	m_pogoHertz = def.pogoHertz;
	m_pogoDampingRatio = def.pogoDampingRatio;
	m_pogoCompressionScale = def.pogoCompressionScale;
	m_pogoTensionScale = def.pogoTensionScale;
	m_enableSkirtPogo = def.enableSkirtPogo;
	m_platformMaterialId = def.platformMaterialId;
	m_minGroundNormalY = 0.7f;

	m_capsule = def.capsule;
	m_filter = def.filter;

	m_worldId = b3_nullWorldId;
	m_groundId = b3_nullBodyId;
	m_moverId = b3_nullBodyId;
	m_surfaceBodyId = b3_nullBodyId;
	m_moverJointId = b3_nullJointId;
	m_pogoJointId = b3_nullJointId;

	m_castResult = {};

	m_velocity = b3Vec3_zero;
	m_timeSliding = 0.0f;
	m_slideDuration = 0.2f;
	m_onGround = false;
	m_walkable = false;
	m_jumping = false;
	m_jumpTicks = 0;

	m_pogoImpulse = 0.0f;
	m_pogoVelocity = 0.0f;
	m_pogoLength = 0.0f;

	for ( DynamicMoverProbe& probe : m_probes )
	{
		probe = { b3Pos_zero, b3Vec3_zero, 1.0f, false };
	}

	m_probeCount = 0;
	m_selectedProbe = 0;
}

void DynamicMover::Create( b3WorldId worldId, const DynamicMoverDef* def )
{
	m_capsule = def->capsule;
	m_filter = def->filter;
	m_jumpSpeed = def->jumpSpeed;
	m_maxSpeed = def->maxSpeed;
	m_minSpeed = def->minSpeed;
	m_stopSpeed = def->stopSpeed;
	m_accelerate = def->accelerate;
	m_airSteer = def->airSteer;
	m_friction = def->friction;
	m_gravityScale = def->gravityScale;
	m_maxGroundForce = def->maxGroundForce;
	m_maxAirForce = def->maxAirForce;
	m_pogoRestLength = def->pogoRestLength;
	m_pogoHertz = def->pogoHertz;
	m_pogoDampingRatio = def->pogoDampingRatio;
	m_pogoCompressionScale = def->pogoCompressionScale;
	m_pogoTensionScale = def->pogoTensionScale;
	m_enableSkirtPogo = def->enableSkirtPogo;
	m_platformMaterialId = def->platformMaterialId;

	m_worldId = worldId;
	m_velocity = b3Vec3_zero;
	m_onGround = false;
	m_walkable = false;
	m_jumping = false;
	m_jumpTicks = 0;
	m_pogoImpulse = 0.0f;
	m_pogoVelocity = 0.0f;
	m_pogoLength = 0.0f;
	m_pogoJointId = b3_nullJointId;

	b3BodyDef bodyDef = b3DefaultBodyDef();
	m_groundId = b3CreateBody( worldId, &bodyDef );

	// Position is the center of the capsule.
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = def->position;
	bodyDef.gravityScale = m_gravityScale;
	bodyDef.fixedRotation = true;
	bodyDef.enableSleep = false;
	bodyDef.name = "mover";

	m_moverId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = def->density;

	// The mover joint and the pogo joint handle all the movement, surface friction would fight
	// them.
	shapeDef.baseMaterial.friction = 0.0f;
	shapeDef.baseMaterial.restitution = 0.0f;
	shapeDef.baseMaterial.customColor = b3_colorDodgerBlue;
	shapeDef.filter = def->filter;
	shapeDef.enablePreSolveEvents = def->enablePreSolveEvents;
	shapeDef.enableSensorEvents = def->enableSensorEvents;

	b3CreateCapsuleShape( m_moverId, &shapeDef, &m_capsule );

	CreateMoverJoint( m_groundId );
}

void DynamicMover::CreateMoverJoint( b3BodyId baseId )
{
	b3MoverJointDef moverDef = b3DefaultMoverJointDef();
	moverDef.linearVelocity = m_velocity;
	moverDef.maxForce = m_maxGroundForce;
	moverDef.base.bodyIdA = baseId;
	moverDef.base.bodyIdB = m_moverId;
	moverDef.base.collideConnected = true;

	m_moverJointId = b3CreateMoverJoint( m_worldId, &moverDef );
}

void DynamicMover::Destroy()
{
	if ( B3_IS_NON_NULL( m_moverId ) )
	{
		b3DestroyBody( m_moverId );
	}

	if ( B3_IS_NON_NULL( m_groundId ) )
	{
		b3DestroyBody( m_groundId );
	}

	m_groundId = b3_nullBodyId;
	m_moverId = b3_nullBodyId;

	// Joints are implicitly destroyed.
	m_moverJointId = b3_nullJointId;
	m_pogoJointId = b3_nullJointId;
}

void DynamicMover::SetGravityScale( float gravityScale )
{
	m_gravityScale = gravityScale;

	if ( B3_IS_NON_NULL( m_moverId ) )
	{
		b3Body_SetGravityScale( m_moverId, gravityScale );
	}
}

bool DynamicMover::Jump()
{
	if ( m_onGround == false || m_walkable == false )
	{
		return false;
	}

	float surfaceVelocity = 0.0f;
	if ( b3Body_IsValid( m_castResult.bodyId ) )
	{
		b3Vec3 v = b3Body_GetWorldPointVelocity( m_castResult.bodyId, m_castResult.point );
		surfaceVelocity = v.y;
	}

	float mass = b3Body_GetMass( m_moverId );
	float vy = b3Body_GetLinearVelocity( m_moverId ).y;

	// Remove the pogo constraint velocity but add the surface velocity.
	float dv = b3MaxFloat( 0.0f, m_jumpSpeed - vy ) + surfaceVelocity;
	b3Body_ApplyLinearImpulseToCenter( m_moverId, { 0.0f, mass * dv, 0.0f }, true );

	Launch();
	return true;
}

void DynamicMover::Launch()
{
	// This removes the pogo step down on the next update.
	m_onGround = false;
	m_walkable = false;
	m_jumping = true;
	m_jumpTicks = 0;
}

// Movement follows the Quake ground and air model:
// https://github.com/id-Software/Quake/blob/master/QW/client/pmove.c#L390

// onGround - controls friction, mover max force, and pogo step-down extension
// walkable - controls air steer
// jumping - blocks the pogo from pulling down
void DynamicMover::Update( float timeStep, b3Vec3 forward, b3Vec3 right, b3Vec2 throttle,
						   float speedScale )
{
	// Reach further while grounded so the spring can find the ground over a step
	float stepDownLength = m_pogoRestLength;
	float rayLength = m_onGround ? m_pogoRestLength + stepDownLength : m_pogoRestLength;
	b3Vec3 translation = { 0.0f, -rayLength, 0.0f };

	b3Pos position = b3Body_GetPosition( m_moverId );
	b3Pos origin = position + m_capsule.center1;

	b3QueryFilter queryFilter = {
		.categoryBits = m_filter.categoryBits,
		.maskBits = m_filter.maskBits,
		.id = 0,
		.name = "dynamic_mover_pogo",
	};

	CastContext castContext = { {}, m_moverId };
	b3World_CastRay( m_worldId, origin, translation, queryFilter, CastCallback, &castContext );
	m_castResult = castContext.result;

	m_probes[0] = { origin, translation, m_castResult.hit ? m_castResult.fraction : 1.0f,
					m_castResult.hit };
	m_probeCount = 1;
	m_selectedProbe = 0;

	b3Vec3 pogoOffset = b3Vec3_zero;

	bool centerWalkable = m_castResult.hit && m_castResult.normal.y >= m_minGroundNormalY;
	if ( centerWalkable == false && m_enableSkirtPogo )
	{
		// The skirt turns with the direction of travel so ties go to the leading probe
		b3Vec3 moverVelocity = b3Body_GetLinearVelocity( m_moverId );
		b3Vec3 heading = { moverVelocity.x, 0.0f, moverVelocity.z };
		if ( b3LengthSquared( heading ) < FLT_EPSILON )
		{
			heading = b3Vec3_axisX;
		}

		heading = b3Normalize( heading );
		b3Vec3 side = { -heading.z, 0.0f, heading.x };

		float bestDisplacement = FLT_MAX;
		for ( int i = 0; i < m_skirtCount; ++i )
		{
			b3CosSin cs = b3ComputeCosSin( 2.0f * B3_PI * i / m_skirtCount );
			b3Vec3 offset = m_capsule.radius * ( cs.cosine * heading + cs.sine * side );

			b3Pos skirtOrigin = origin + offset;
			CastContext skirtContext = { {}, m_moverId };
			b3World_CastRay( m_worldId, skirtOrigin, translation, queryFilter, CastCallback,
							 &skirtContext );
			const DynamicMoverCastResult& result = skirtContext.result;

			m_probes[m_probeCount] = { skirtOrigin, translation,
									   result.hit ? result.fraction : 1.0f, result.hit };

			// Use the walkable result that causes the least movement. With none the mover keeps
			// the central hit and slides off the steep surface.
			if ( result.hit && result.normal.y >= m_minGroundNormalY )
			{
				float displacement = b3AbsFloat( result.fraction * rayLength - m_pogoRestLength );
				if ( displacement < bestDisplacement )
				{
					bestDisplacement = displacement;
					m_castResult = result;
					m_selectedProbe = m_probeCount;
					pogoOffset = offset;
				}
			}

			m_probeCount += 1;
		}
	}

	// Should jumping end?
	if ( m_jumping )
	{
		m_jumpTicks += 1;

		// Jump ticks allow time for the jump to leave the ground. Otherwise jumping ends
		// when the character approaches the current ground.
		if ( m_jumpTicks > 2 && m_castResult.hit &&
			 b3Dot( m_velocity, m_castResult.normal ) <= 0.0f )
		{
			m_jumping = false;
		}
	}

	if ( m_castResult.hit )
	{
		// Is the ground too steep to walk?
		m_walkable = m_castResult.normal.y >= m_minGroundNormalY;

		// The cast can still hit while jumping.
		m_onGround = m_jumping == false;

		if ( m_onGround )
		{
			if ( m_walkable )
			{
				m_timeSliding = b3MaxFloat( m_timeSliding - timeStep, 0.0f );
			}
			else
			{
				m_timeSliding = b3MinFloat( m_timeSliding + timeStep, m_slideDuration );
			}
		}
	}
	else
	{
		m_onGround = false;
		m_walkable = false;
		m_timeSliding = b3MaxFloat( m_timeSliding - timeStep, 0.0f );
	}

	// Connect the mover joint to platform. Acts like friction.
	b3BodyId baseId = m_groundId;
	if ( m_castResult.hit && m_platformMaterialId != 0 &&
		 m_castResult.userMaterialId == m_platformMaterialId &&
		 b3Body_GetType( m_castResult.bodyId ) == b3_kinematicBody )
	{
		baseId = m_castResult.bodyId;
	}

	// The mover joint can be destroyed if attached to a platform that gets destroyed.
	bool moverJointValid = b3Joint_IsValid( m_moverJointId );

	if ( moverJointValid == false ||
		 B3_ID_EQUALS( b3Joint_GetBodyA( m_moverJointId ), baseId ) == false )
	{
		if ( moverJointValid )
		{
			b3DestroyJoint( m_moverJointId, false );
		}

		CreateMoverJoint( baseId );
	}

	m_velocity = b3Body_GetLinearVelocity( m_moverId ) - b3Body_GetLinearVelocity( baseId );

	// Friction acts in the horizontal plane. The pogo owns the vertical motion.
	if ( m_onGround )
	{
		float speed = sqrtf( m_velocity.x * m_velocity.x + m_velocity.z * m_velocity.z );
		if ( speed < m_minSpeed )
		{
			m_velocity.x = 0.0f;
			m_velocity.z = 0.0f;
		}
		else
		{
			// Linear damping above stopSpeed and fixed reduction below stopSpeed
			float control = speed < m_stopSpeed ? m_stopSpeed : speed;

			// friction has units of 1/time
			float drop = control * m_friction * timeStep;
			float newSpeed = b3MaxFloat( 0.0f, speed - drop );
			float ratio = newSpeed / speed;
			m_velocity.x *= ratio;
			m_velocity.z *= ratio;
		}
	}

	// Mover force
	float maxForce;
	if ( m_onGround )
	{
		if ( m_slideDuration > 0.0f )
		{
			float scale = b3ClampFloat( 1.0f - m_timeSliding / m_slideDuration, 0.0f, 1.0f );
			maxForce = scale * m_maxGroundForce;
		}
		else
		{
			maxForce = m_walkable ? m_maxGroundForce : 0.0f;
		}
	}
	else
	{
		maxForce = m_maxAirForce;
	}

	b3MoverJoint_SetMaxVelocityForce( m_moverJointId, maxForce );

	float maxSpeed = speedScale * m_maxSpeed;
	b3Vec3 desiredVelocity = maxSpeed * throttle.x * forward + maxSpeed * throttle.y * right;
	desiredVelocity.y = 0.0f;

	float desiredSpeed;
	b3Vec3 desiredDirection = b3GetLengthAndNormalize( &desiredSpeed, desiredVelocity );

	if ( desiredSpeed > maxSpeed )
	{
		desiredSpeed = maxSpeed;
	}

	// Accelerate
	float currentSpeed = b3Dot( m_velocity, desiredDirection );
	float addSpeed = desiredSpeed - currentSpeed;
	if ( addSpeed > 0.0f )
	{
		float steer = m_walkable ? 1.0f : m_airSteer;
		float accelSpeed = steer * m_accelerate * maxSpeed * timeStep;
		if ( accelSpeed > addSpeed )
		{
			accelSpeed = addSpeed;
		}

		m_velocity.x += accelSpeed * desiredDirection.x;
		m_velocity.z += accelSpeed * desiredDirection.z;
	}

	if ( m_castResult.hit == true && b3IsNormalized( m_castResult.normal ) )
	{
		m_pogoLength = m_castResult.fraction * rayLength;

		float moverMass = b3Body_GetMass( m_moverId );
		float gravity = b3Length( b3World_GetGravity( m_worldId ) );
		float moverWeight = m_gravityScale * gravity * moverMass;

		bool pogoJointValid = b3Joint_IsValid( m_pogoJointId );

		// Create or recreate the pogo joint as needed.
		if ( B3_ID_EQUALS( m_castResult.bodyId, m_surfaceBodyId ) == false ||
			 pogoJointValid == false )
		{
			if ( pogoJointValid )
			{
				m_pogoImpulse = b3PogoJoint_GetImpulse( m_pogoJointId );
				m_pogoVelocity = b3PogoJoint_GetCurrentSpeed( m_pogoJointId );

				b3DestroyJoint( m_pogoJointId, false );
				m_pogoJointId = b3_nullJointId;
			}

			m_surfaceBodyId = m_castResult.bodyId;

			b3PogoJointDef pogoDef = b3DefaultPogoJointDef();
			pogoDef.base.localFrameA.p =
				b3Body_GetLocalPoint( m_surfaceBodyId, m_castResult.point );
			pogoDef.base.localFrameB.p = m_capsule.center1 + pogoOffset;
			pogoDef.normal = m_castResult.normal;
			pogoDef.base.bodyIdA = m_surfaceBodyId;
			pogoDef.base.bodyIdB = m_moverId;
			pogoDef.base.collideConnected = true;
			pogoDef.restLength = m_pogoRestLength;
			pogoDef.hertz = m_pogoHertz;
			pogoDef.dampingRatio = m_pogoDampingRatio;
			pogoDef.maxCompressionForce = m_pogoCompressionScale * moverWeight;

			// Warm start from the joint that was just destroyed
			pogoDef.impulse = m_pogoImpulse;
			pogoDef.velocity = m_pogoVelocity;

			if ( m_jumping )
			{
				// Don't allow the pogo to pull down
				pogoDef.maxTensionForce = 0.0f;
			}
			else
			{
				// The pogo can pull down at a multiple of the gravity force.
				pogoDef.maxTensionForce = m_pogoTensionScale * moverWeight;
			}

			m_pogoJointId = b3CreatePogoJoint( m_worldId, &pogoDef );
		}
		else
		{
			// Preserve the joint, improving performance.

			b3Transform localFrameA;
			localFrameA.p = b3Body_GetLocalPoint( m_surfaceBodyId, m_castResult.point );
			localFrameA.q = b3Quat_identity;
			b3Joint_SetLocalFrameA( m_pogoJointId, localFrameA );

			b3Transform localFrameB;
			localFrameB.p = m_capsule.center1 + pogoOffset;
			localFrameB.q = b3Quat_identity;
			b3Joint_SetLocalFrameB( m_pogoJointId, localFrameB );

			b3PogoJoint_SetNormal( m_pogoJointId, m_castResult.normal );

			if ( m_jumping )
			{
				// Don't allow the pogo to pull down
				b3PogoJoint_SetMaxTensionForce( m_pogoJointId, 0.0f );
			}
			else
			{
				// The pogo can pull down at a multiple of the gravity force.
				b3PogoJoint_SetMaxTensionForce( m_pogoJointId, m_pogoTensionScale * moverWeight );
			}

			b3PogoJoint_SetMaxCompressionForce( m_pogoJointId,
												m_pogoCompressionScale * moverWeight );

			b3PogoJoint_SetSpringHertz( m_pogoJointId, m_pogoHertz );
			b3PogoJoint_SetSpringDampingRatio( m_pogoJointId, m_pogoDampingRatio );
		}
	}
	else
	{
		if ( b3Joint_IsValid( m_pogoJointId ) )
		{
			b3DestroyJoint( m_pogoJointId, false );
			m_pogoJointId = b3_nullJointId;
		}

		m_surfaceBodyId = b3_nullBodyId;
		m_pogoImpulse = 0.0f;
		m_pogoVelocity = 0.0f;
		m_pogoLength = 0.0f;
	}

	b3MoverJoint_SetLinearVelocity( m_moverJointId, m_velocity );
}

void DynamicMover::Draw() const
{
	for ( int i = 0; i < m_probeCount; ++i )
	{
		const DynamicMoverProbe& probe = m_probes[i];
		b3Pos end = b3OffsetPos( probe.origin, probe.fraction * probe.translation );

		if ( probe.hit == false )
		{
			DrawLine( probe.origin, end, MakeColor( b3_colorGray ) );
			continue;
		}

		b3HexColor color = i == m_selectedProbe ? b3_colorGreen : b3_colorYellow;
		DrawLine( probe.origin, end, MakeColor( color ) );
		DrawPoint( end, 6.0f, MakeColor( color ) );
	}
}
