// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "physics_world.h"
#include "recording.h"
#include "solver.h"
#include "solver_set.h"

// needed for dll export
#include "box3d/box3d.h"

#include <stdio.h>

void b3MoverJoint_SetLinearVelocity( b3JointId jointId, b3Vec3 velocity )
{
	B3_ASSERT( b3IsValidVec3( velocity ) );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, MoverJointSetLinearVelocity, jointId, velocity );
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_moverJoint );
	joint->moverJoint.linearVelocity = velocity;
}

b3Vec3 b3MoverJoint_GetLinearVelocity( b3JointId jointId )
{
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_moverJoint );
	return joint->moverJoint.linearVelocity;
}

void b3MoverJoint_SetMaxVelocityForce( b3JointId jointId, float maxForce )
{
	B3_ASSERT( b3IsValidFloat( maxForce ) && maxForce >= 0.0f );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, MoverJointSetMaxVelocityForce, jointId, maxForce );
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_moverJoint );
	joint->moverJoint.maxForce = maxForce;
}

float b3MoverJoint_GetMaxVelocityForce( b3JointId jointId )
{
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_moverJoint );
	return joint->moverJoint.maxForce;
}

b3Vec3 b3GetMoverJointForce( b3World* world, b3JointSim* base )
{
	b3MoverJoint* joint = &base->moverJoint;
	b3Vec3 P = b3Blend2( joint->impulse.x, joint->axisX, joint->impulse.y, joint->axisZ );
	return b3MulSV( world->inv_h, P );
}

void b3PrepareMoverJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_moverJoint );

	int idA = base->bodyIdA;
	int idB = base->bodyIdB;

	b3World* world = context->world;

	b3Body* bodyA = b3Array_Get( world->bodies, idA );
	b3Body* bodyB = b3Array_Get( world->bodies, idB );

	B3_ASSERT( bodyA->setIndex == b3_awakeSet || bodyB->setIndex == b3_awakeSet );

	b3SolverSet* setA = b3GetSolverSet( world, bodyA->setIndex );
	b3SolverSet* setB = b3GetSolverSet( world, bodyB->setIndex );

	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;

	b3BodySim* bodySimA = b3Array_Get( setA->bodySims, localIndexA );
	b3BodySim* bodySimB = b3Array_Get( setB->bodySims, localIndexB );

	float mA = bodySimA->invMass;
	float mB = bodySimB->invMass;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = b3Mat3_zero;
	base->invIB = b3Mat3_zero;

	b3MoverJoint* joint = &base->moverJoint;
	joint->indexA = bodyA->setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	b3Quat quatB = b3MulQuat( bodySimB->transform.q, base->localFrameB.q );
	b3Matrix3 matrixB = b3MakeMatrixFromQuat( quatB );
	joint->axisX = matrixB.cx;
	joint->axisZ = matrixB.cz;

	float k = mA + mB;
	joint->linearMass = k > 0.0f ? 1.0f / k : 0.0f;

	if ( context->enableWarmStarting == false )
	{
		joint->impulse = (b3Vec2){ 0 };
	}
}

void b3WarmStartMoverJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_moverJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;

	b3MoverJoint* joint = &base->moverJoint;

	// dummy state for static bodies
	b3BodyState dummyState = b3_identityBodyState;

	b3BodyState* stateA =
		joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB =
		joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 P = b3Blend2( joint->impulse.x, joint->axisX, joint->impulse.y, joint->axisZ );
	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = b3MulSub( stateA->linearVelocity, mA, P );
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = b3MulAdd( stateB->linearVelocity, mB, P );
	}
}

void b3SolveMoverJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_moverJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;

	// dummy state for static bodies
	b3BodyState dummyState = b3_identityBodyState;

	b3MoverJoint* joint = &base->moverJoint;
	b3BodyState* stateA =
		joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB =
		joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 vB = stateB->linearVelocity;

	float maxForce = joint->maxForce;
	b3Vec3 vr = b3Sub( b3Sub( vB, vA ), joint->linearVelocity );
	b3Vec2 cdot = { b3Dot( vr, joint->axisX ), b3Dot( vr, joint->axisZ ) };
	b3Vec2 deltaImpulse = b3MulSV2( -joint->linearMass, cdot );
	b3Vec2 newImpulse = {
		joint->impulse.x + deltaImpulse.x,
		joint->impulse.y + deltaImpulse.y,
	};

	float maxImpulse = context->h * maxForce;

	float lengthSquared = b3Dot2( newImpulse, newImpulse );
	if ( lengthSquared > maxImpulse * maxImpulse )
	{
		float scale = maxImpulse / sqrtf( lengthSquared );
		newImpulse.x *= scale;
		newImpulse.y *= scale;
	}

	deltaImpulse = b3Sub2( newImpulse, joint->impulse );
	joint->impulse = newImpulse;

	b3Vec3 P = b3Blend2( deltaImpulse.x, joint->axisX, deltaImpulse.y, joint->axisZ );
	vA = b3MulSub( vA, mA, P );
	vB = b3MulAdd( vB, mB, P );

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = vA;
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = vB;
	}
}

static b3HexColor b3LerpColor( b3HexColor a, b3HexColor b, float t )
{
	uint32_t result = 0;
	for ( int shift = 0; shift < 24; shift += 8 )
	{
		float ca = (float)( ( (uint32_t)a >> shift ) & 0xFF );
		float cb = (float)( ( (uint32_t)b >> shift ) & 0xFF );
		uint32_t c = (uint32_t)( b3LerpFloat( ca, cb, t ) + 0.5f );
		result |= c << shift;
	}
	return (b3HexColor)result;
}

static void b3DrawArrow( b3DebugDraw* draw, b3Pos a, b3Pos b, b3Vec3 planeNormal, float size,
						 b3HexColor color )
{
	draw->DrawSegmentFcn( a, b, color, draw->context );

	b3Vec3 shaft = b3SubPos( b, a );
	float shaftLen = b3Length( shaft );
	if ( shaftLen < 1e-6f )
	{
		return;
	}

	b3Vec3 dir = { shaft.x / shaftLen, shaft.y / shaftLen, shaft.z / shaftLen };
	float headLen = size;

	b3Vec3 perp = b3Normalize( b3Cross( planeNormal, dir ) );
	b3Vec3 backFromTip = b3MulSV( -headLen, dir );
	b3Vec3 sideStep = b3MulSV( headLen * 0.5f, perp );
	b3Pos tip1 = b3OffsetPos( b, b3Add( backFromTip, sideStep ) );
	b3Pos tip2 = b3OffsetPos( b, b3Sub( backFromTip, sideStep ) );
	draw->DrawSegmentFcn( b, tip1, color, draw->context );
	draw->DrawSegmentFcn( b, tip2, color, draw->context );
}

void b3DrawMoverJoint( b3DebugDraw* draw, b3World* world, b3JointSim* base,
					   b3WorldTransform transformA, b3WorldTransform transformB )
{
	B3_ASSERT( base->type == b3_moverJoint );
	B3_UNUSED( transformA );

	b3MoverJoint* joint = &base->moverJoint;

	b3WorldTransform frameB = b3MulWorldTransforms( transformB, base->localFrameB );
	b3Matrix3 matrixB = b3MakeMatrixFromQuat( frameB.q );
	b3Vec3 axisX = matrixB.cx;
	b3Vec3 axisZ = matrixB.cz;

	b3Body* bodyA = b3Array_Get( world->bodies, base->bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, base->bodyIdB );
	b3BodyState* stateA = b3GetBodyState( world, bodyA );
	b3BodyState* stateB = b3GetBodyState( world, bodyB );
	b3Vec3 vA = stateA != NULL ? stateA->linearVelocity : b3Vec3_zero;
	b3Vec3 vB = stateB != NULL ? stateB->linearVelocity : b3Vec3_zero;

	b3Vec3 velocityError = b3Sub( b3Sub( vB, vA ), joint->linearVelocity );
	b3Vec2 planarVelocityError = { b3Dot( velocityError, axisX ), b3Dot( velocityError, axisZ ) };
	b3Vec2 targetVelocity = { b3Dot( joint->linearVelocity, axisX ),
							  b3Dot( joint->linearVelocity, axisZ ) };

	// Colorize ring by ratio of velocity error magnitude to target speed.
	b3HexColor color = b3_colorWhite;
	float targetSpeed = b3Length2( targetVelocity );
	if ( targetSpeed > 0.0f )
	{
		float t = b3ClampFloat( b3Length2( planarVelocityError ) / targetSpeed, 0.0f, 1.0f );
		color = b3LerpColor( b3_colorWhite, b3_colorBlue, t );
	}

	float radius = draw->jointScale;
	b3Vec3 rx = b3MulSV( radius, axisX );
	b3Vec3 rz = b3MulSV( radius, axisZ );
	const int segmentCount = 32;
	b3CosSin cs = b3ComputeCosSin( 2.0f * B3_PI / segmentCount );
	float x1 = 1.0f, y1 = 0.0f;
	b3Pos p1 = b3OffsetPos( frameB.p, rx );
	for ( int i = 0; i < segmentCount; ++i )
	{
		float x2 = cs.cosine * x1 - cs.sine * y1;
		float y2 = cs.sine * x1 + cs.cosine * y1;
		b3Pos p2 = b3OffsetPos( frameB.p, b3Blend2( x2, rx, y2, rz ) );
		draw->DrawSegmentFcn( p1, p2, color, draw->context );
		x1 = x2;
		y1 = y2;
		p1 = p2;
	}

	if ( joint->maxForce > 0.0f )
	{
		b3Vec3 force = b3Blend2( joint->impulse.x, rx, joint->impulse.y, rz );
		b3Vec3 dir = b3Normalize( force );
		b3Pos origin = b3OffsetPos( frameB.p, b3MulSV( -radius, dir ) );
		float scale = world->inv_h / joint->maxForce;
		b3Pos tip = b3OffsetPos( origin, b3MulSV( scale, force ) );
		b3DrawArrow( draw, origin, tip, matrixB.cy, 0.1f * draw->jointScale, b3_colorWhite );
	}
}
