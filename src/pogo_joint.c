// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "math_internal.h"
#include "physics_world.h"
#include "recording.h"
#include "solver.h"
#include "solver_set.h"

// needed for dll export
#include "box3d/box3d.h"

#include <stdio.h>

void b3PogoJoint_SetRestLength( b3JointId jointId, float length )
{
	B3_ASSERT( b3IsValidFloat( length ) && length >= 0.0f );
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_pogoJoint );
	if ( joint->pogoJoint.restLength == length )
	{
		return;
	}

	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, PogoJointSetRestLength, jointId, length );
	joint->pogoJoint.restLength = length;
}

float b3PogoJoint_GetRestLength( b3JointId jointId )
{
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_pogoJoint );
	return joint->pogoJoint.restLength;
}

void b3PogoJoint_SetSpringHertz( b3JointId jointId, float hertz )
{
	B3_ASSERT( b3IsValidFloat( hertz ) && hertz >= 0.0f );
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_pogoJoint );
	if ( joint->pogoJoint.hertz == hertz )
	{
		return;
	}

	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, PogoJointSetSpringHertz, jointId, hertz );
	joint->pogoJoint.hertz = hertz;
}

float b3PogoJoint_GetSpringHertz( b3JointId jointId )
{
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_pogoJoint );
	return joint->pogoJoint.hertz;
}

void b3PogoJoint_SetSpringDampingRatio( b3JointId jointId, float dampingRatio )
{
	B3_ASSERT( b3IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f );
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_pogoJoint );
	if ( joint->pogoJoint.dampingRatio == dampingRatio )
	{
		return;
	}

	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, PogoJointSetSpringDampingRatio, jointId, dampingRatio );
	joint->pogoJoint.dampingRatio = dampingRatio;
}

float b3PogoJoint_GetSpringDampingRatio( b3JointId jointId )
{
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_pogoJoint );
	return joint->pogoJoint.dampingRatio;
}

void b3PogoJoint_SetMaxTensionForce( b3JointId jointId, float force )
{
	B3_ASSERT( b3IsValidFloat( force ) && force >= 0.0f );
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_pogoJoint );
	if ( joint->pogoJoint.maxTensionForce == force )
	{
		return;
	}

	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, PogoJointSetMaxTensionForce, jointId, force );
	joint->pogoJoint.maxTensionForce = force;
}

float b3PogoJoint_GetMaxTensionForce(b3JointId jointId)
{
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_pogoJoint );
	return joint->pogoJoint.maxTensionForce;
}

void b3PogoJoint_SetMaxCompressionForce(b3JointId jointId, float force)
{
	B3_ASSERT( b3IsValidFloat( force ) && force >= 0.0f );
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_pogoJoint );
	if ( joint->pogoJoint.maxCompressionForce == force )
	{
		return;
	}

	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, PogoJointSetMaxCompressionForce, jointId, force );
	joint->pogoJoint.maxCompressionForce = force;
}

float b3PogoJoint_GetMaxCompressionForce(b3JointId jointId)
{
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_pogoJoint );
	return joint->pogoJoint.maxCompressionForce;
}

void b3PogoJoint_SetNormal( b3JointId jointId, b3Vec3 normal )
{
	B3_ASSERT( b3IsValidVec3( normal ) && b3IsNormalized( normal ) );
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_pogoJoint );
	b3Vec3 current = joint->pogoJoint.normal;
	if ( current.x == normal.x && current.y == normal.y && current.z == normal.z )
	{
		return;
	}

	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, PogoJointSetNormal, jointId, normal );
	joint->pogoJoint.normal = normal;
}

b3Vec3 b3PogoJoint_GetNormal( b3JointId jointId )
{
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_pogoJoint );
	return joint->pogoJoint.normal;
}

float b3PogoJoint_GetCurrentLength( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_pogoJoint );

	b3WorldTransform worldTransformA = b3GetBodyTransform( world, base->bodyIdA );
	b3Transform transformA = b3ToRelativeTransform( worldTransformA, worldTransformA.p );
	b3Transform transformB =
		b3ToRelativeTransform( b3GetBodyTransform( world, base->bodyIdB ), worldTransformA.p );

	b3Vec3 axis = b3RotateVector( b3MulQuat( transformB.q, base->localFrameB.q ), b3Vec3_axisY );
	b3Vec3 pA = b3TransformPoint( transformA, base->localFrameA.p );
	b3Vec3 pB = b3TransformPoint( transformB, base->localFrameB.p );
	return b3Dot( b3Sub( pB, pA ), axis );
}

float b3PogoJoint_GetImpulse( b3JointId jointId )
{
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_pogoJoint );
	return joint->pogoJoint.impulse;
}

float b3PogoJoint_GetCurrentSpeed( b3JointId jointId )
{
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_pogoJoint );
	return joint->pogoJoint.velocity;
}

b3Vec3 b3GetPogoJointForce( b3World* world, b3JointSim* base )
{
	return b3MulSV( world->inv_h * base->pogoJoint.impulse, base->pogoJoint.normal );
}

void b3PreparePogoJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_pogoJoint );

	b3World* world = context->world;

	// chase body id to the solver set where the body lives
	int idA = base->bodyIdA;
	int idB = base->bodyIdB;

	b3Body* bodyA = b3Array_Get( world->bodies, idA );
	b3Body* bodyB = b3Array_Get( world->bodies, idB );

	B3_ASSERT( bodyA->setIndex == b3_awakeSet || bodyB->setIndex == b3_awakeSet );

	b3SolverSet* setA = b3GetFixedSolverSet( world, bodyA->setIndex );
	b3SolverSet* setB = b3GetFixedSolverSet( world, bodyB->setIndex );

	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;

	b3BodySim* bodySimA = b3Array_Get( setA->bodySims, localIndexA );
	b3BodySim* bodySimB = b3Array_Get( setB->bodySims, localIndexB );

	float mA = bodySimA->invMass;
	float mB = bodySimB->invMass;
	b3Matrix3 iA = bodySimA->invInertiaWorld;
	b3Matrix3 iB = bodySimB->invInertiaWorld;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = iA;
	base->invIB = iB;

	b3PogoJoint* joint = &base->pogoJoint;
	joint->indexA = bodyA->setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint->frameA.q = b3MulQuat( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b3RotateVector( bodySimA->transform.q,
									  b3Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b3MulQuat( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b3RotateVector( bodySimB->transform.q,
									  b3Sub( base->localFrameB.p, bodySimB->localCenter ) );

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint->deltaCenter = b3SubPos( bodySimB->center, bodySimA->center );

	b3Vec3 rnA = b3Cross( joint->frameA.p, joint->normal );
	b3Vec3 rnB = b3Cross( joint->frameB.p, joint->normal );
	float k = mA + mB + b3Dot( rnA, b3MulMV( iA, rnA ) ) + b3Dot( rnB, b3MulMV( iB, rnB ) );
	joint->linearMass = k > 0.0f ? 1.0f / k : 0.0f;

	if ( context->enableWarmStarting == false )
	{
		joint->velocity = 0.0f;
		joint->impulse = 0.0f;
	}
}

void b3WarmStartPogoJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_pogoJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	b3PogoJoint* joint = &base->pogoJoint;

	if ( joint->hertz == 0.0f )
	{
		joint->impulse = 0.0f;
		return;
	}

	// dummy state for static bodies
	b3BodyState dummyState = b3_identityBodyState;

	b3BodyState* stateA =
		joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB =
		joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

	b3Vec3 linearImpulse = b3MulSV( joint->impulse, joint->normal );

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = b3MulSub( stateA->linearVelocity, mA, linearImpulse );
		stateA->angularVelocity =
			b3Sub( stateA->angularVelocity, b3MulMV( iA, b3Cross( rA, linearImpulse ) ) );
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = b3MulAdd( stateB->linearVelocity, mB, linearImpulse );
		stateB->angularVelocity =
			b3Add( stateB->angularVelocity, b3MulMV( iB, b3Cross( rB, linearImpulse ) ) );
	}
}

void b3SolvePogoJoint( b3JointSim* base, b3StepContext* context, bool useBias )
{
	B3_ASSERT( base->type == b3_pogoJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	// dummy state for static bodies
	b3BodyState dummyState = b3_identityBodyState;

	b3PogoJoint* joint = &base->pogoJoint;
	if ( joint->hertz == 0.0f )
	{
		joint->impulse = 0.0f;
		joint->velocity = 0.0f;
		return;
	}

	b3BodyState* stateA =
		joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB =
		joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

	float bias = 0.0f;
	if ( useBias )
	{
		// This is a custom pogo position correction designed to avoid feeding velocity into
		// the mover body. Otherwise the mover can make huge jumps going up stairs. This still
		// works with continuous collision.

		b3Vec3 dcA = stateA->deltaPosition;
		b3Vec3 dcB = stateB->deltaPosition;
		b3Vec3 d = b3Add( b3Add( b3Sub( dcB, dcA ), b3Sub( rB, rA ) ), joint->deltaCenter );

		// The pogo axis is usually fixed as the world y-axis. Different than the ground normal.
		b3Vec3 pogoAxis = b3RotateVector( joint->frameB.q, b3Vec3_axisY );
		float c = b3Dot( pogoAxis, d ) - joint->restLength;

		// This could be divided by dot(joint->normal, pogoAxis) to account for the constraint
		// direction, but I'd rather diminish the pogo recovery rate than risk making it huge.
		joint->velocity =
			b3SpringDamper( joint->hertz, joint->dampingRatio, c, joint->velocity, context->h );
		bias = -joint->velocity;
	}

	b3Vec3 vr = b3Sub( b3Add( vB, b3Cross( wB, rB ) ), b3Add( vA, b3Cross( wA, rA ) ) );
	float cdot = b3Dot( joint->normal, vr );

	float maxTensionImpulse = context->h * joint->maxTensionForce;
	float maxCompressionImpulse = context->h * joint->maxCompressionForce;
	float oldImpulse = joint->impulse;
	float impulse = -joint->linearMass * ( cdot + bias );
	joint->impulse =
		b3ClampFloat( joint->impulse + impulse, -maxTensionImpulse, maxCompressionImpulse );
	impulse = joint->impulse - oldImpulse;

	b3Vec3 P = b3MulSV( impulse, joint->normal );
	vA = b3MulSub( vA, mA, P );
	wA = b3Sub( wA, b3MulMV( iA, b3Cross( rA, P ) ) );
	vB = b3MulAdd( vB, mB, P );
	wB = b3Add( wB, b3MulMV( iB, b3Cross( rB, P ) ) );

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = vA;
		stateA->angularVelocity = wA;
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = vB;
		stateB->angularVelocity = wB;
	}
}

void b3DrawPogoJoint( b3DebugDraw* draw, b3World* world, b3JointSim* base,
					  b3WorldTransform transformA, b3WorldTransform transformB, float scale )
{
	B3_ASSERT( base->type == b3_pogoJoint );

	b3PogoJoint* joint = &base->pogoJoint;

	b3Pos pA = b3TransformWorldPoint( transformA, base->localFrameA.p );
	b3Pos pB = b3TransformWorldPoint( transformB, base->localFrameB.p );
	b3Vec3 axis = b3RotateVector( b3MulQuat( transformB.q, base->localFrameB.q ), b3Vec3_axisY );
	b3Vec3 perp1 = b3Perp( axis );
	b3Vec3 perp2 = b3Cross( axis, perp1 );

	float length = b3Dot( b3SubPos( pB, pA ), axis );
	b3Pos foot = b3OffsetPos( pB, b3MulSV( -length, axis ) );

	float force = world->inv_h * joint->impulse;

	// Tolerance for reaching max force.
	float saturation = 0.99f;

	b3HexColor color;
	if ( joint->hertz == 0.0f )
	{
		color = b3_colorGray;
	}
	else if ( force > 0.0f && force >= saturation * joint->maxCompressionForce )
	{
		color = b3_colorRed;
	}
	else if ( force < 0.0f && -force >= saturation * joint->maxTensionForce )
	{
		color = b3_colorRed;
	}
	else if ( force > 0.0f )
	{
		color = b3_colorLimeGreen;
	}
	else if ( force < 0.0f )
	{
		color = b3_colorDeepSkyBlue;
	}
	else
	{
		color = b3_colorLightGray;
	}

	// Spring
	int segmentCount = 12;
	float width = 0.1f * scale;
	b3Pos p1 = pB;
	for ( int i = 1; i <= segmentCount; ++i )
	{
		float side = i == segmentCount ? 0.0f : ( i & 1 ) ? width : -width;
		float t = (float)i / (float)segmentCount;
		b3Pos p2 = b3OffsetPos( pB, b3MulAdd( b3MulSV( -t * length, axis ), side, perp2 ) );
		draw->DrawSegmentFcn( p1, p2, color, draw->context );
		p1 = p2;
	}

	// Ground plane
	b3Pos rest = b3OffsetPos( pB, b3MulSV( -joint->restLength, axis ) );
	float tick = 1.5f * width;
	draw->DrawSegmentFcn( b3OffsetPos( rest, b3MulSV( -tick, perp1 ) ),
						  b3OffsetPos( rest, b3MulSV( tick, perp1 ) ), b3_colorWhite,
						  draw->context );
	draw->DrawSegmentFcn( b3OffsetPos( rest, b3MulSV( -tick, perp2 ) ),
						  b3OffsetPos( rest, b3MulSV( tick, perp2 ) ), b3_colorWhite,
						  draw->context );

	if ( b3Length( b3SubPos( pA, foot ) ) > B3_LINEAR_SLOP )
	{
		draw->DrawSegmentFcn( foot, pA, b3_colorDimGray, draw->context );
	}

	draw->DrawPointFcn( pA, 6.0f, color, draw->context );
	draw->DrawSegmentFcn( pA, b3OffsetPos( pA, b3MulSV( 0.5f * scale, joint->normal ) ),
						  b3_colorOrange, draw->context );

	if ( draw->drawJointExtras )
	{
		char buffer[96];
		snprintf( buffer, 96, "length = %.3f, rest = %.3f, f = %.1f, v = %.3f", length,
				  joint->restLength, force, joint->velocity );
		draw->DrawStringFcn( pB, buffer, color, draw->context );
	}
}
