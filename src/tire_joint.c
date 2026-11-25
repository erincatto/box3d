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

#include "box3d/box3d.h"

#include <stdio.h>

// Used to clamp the dot product between the suspension axis and the contact normal. Limits the effect of the normal
// on the suspension compression.
#define B3_TIRE_MIN_COSINE 0.25f

void b3TireJoint_SetContact( b3JointId jointId, b3Vec3 localPointA, b3Vec3 normal )
{
	B3_CHECK_INPUT( b3IsValidVec3( localPointA ) );
	B3_CHECK_INPUT( b3IsValidVec3( normal ) && b3IsNormalized( normal ) );

	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_tireJoint );
	b3TireJoint* joint = &base->tireJoint;

	b3Quat rotationA = b3GetBodyTransform( world, base->bodyIdA ).q;
	b3Vec3 localNormal = b3InvRotateVector( rotationA, normal );

	b3Vec3 p = base->localFrameA.p;
	b3Vec3 n = joint->localNormal;
	if ( p.x == localPointA.x && p.y == localPointA.y && p.z == localPointA.z && n.x == localNormal.x &&
		 n.y == localNormal.y && n.z == localNormal.z )
	{
		return;
	}

	B3_REC( world, TireJointSetContact, jointId, localPointA, normal );
	base->localFrameA.p = localPointA;
	joint->localNormal = localNormal;
}

b3Vec3 b3TireJoint_GetNormal( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_tireJoint );
	b3Quat rotationA = b3GetBodyTransform( world, base->bodyIdA ).q;
	return b3RotateVector( rotationA, base->tireJoint.localNormal );
}

void b3TireJoint_SetBrakeForce( b3JointId jointId, float force )
{
	B3_CHECK_INPUT( b3IsValidFloat( force ) && force >= 0.0f );

	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_tireJoint );
	if ( base->tireJoint.brakeForce == force )
	{
		return;
	}

	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, TireJointSetBrakeForce, jointId, force );
	base->tireJoint.brakeForce = force;
}

float b3TireJoint_GetBrakeForce( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_tireJoint );
	return base->tireJoint.brakeForce;
}

// Distance of contact point to the axle accounting for lean. Equal to the wheel radius
// when lean is zero.
static float b3GetTireContactRadius( const b3TireJoint* joint, b3Vec3 normal, b3Vec3 axle )
{
	float lean = b3Dot( normal, axle );
	float upright = sqrtf( b3MaxFloat( 0.0f, 1.0f - lean * lean ) );
	return ( joint->radius - joint->halfWidth ) * upright + joint->halfWidth;
}

static float b3ComputeSuspensionLength( const b3JointSim* base, b3WorldTransform transformA,
										b3WorldTransform transformB )
{
	const b3TireJoint* joint = &base->tireJoint;
	b3Pos pA = b3TransformWorldPoint( transformA, base->localFrameA.p );
	b3Pos mount = b3TransformWorldPoint( transformB, base->localFrameB.p );
	b3Vec3 normal = b3RotateVector( transformA.q, joint->localNormal );
	b3Quat frameQB = b3MulQuat( transformB.q, base->localFrameB.q );
	b3Vec3 axis = b3RotateVector( frameQB, b3Vec3_axisX );
	b3Vec3 axle = b3RotateVector( frameQB, b3Vec3_axisZ );
	float cosine = b3MaxFloat( b3Dot( axis, normal ), B3_TIRE_MIN_COSINE );
	float clearance = b3Dot( b3SubPos( mount, pA ), normal ) - b3GetTireContactRadius( joint, normal, axle );
	return clearance / cosine;
}

float b3TireJoint_GetSuspensionLength( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_tireJoint );
	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3WorldTransform transformB = b3GetBodyTransform( world, base->bodyIdB );
	return b3ComputeSuspensionLength( base, transformA, transformB );
}

float b3TireJoint_GetNormalForce( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_tireJoint );
	return world->inv_h * ( base->tireJoint.suspensionImpulse + base->tireJoint.stopImpulse );
}

float b3TireJoint_GetLateralForce( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_tireJoint );
	return world->inv_h * base->tireJoint.lateralImpulse;
}

float b3TireJoint_GetRollingForce( b3JointId jointId )
{
	b3World* world = b3GetWorld( jointId.world0 );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_tireJoint );
	return world->inv_h * base->tireJoint.rollingImpulse;
}

bool b3TireJoint_IsSaturated( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_tireJoint );
	return base->tireJoint.saturated;
}

float b3TireJoint_GetSuspensionImpulse( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_tireJoint );
	return base->tireJoint.suspensionImpulse;
}

float b3TireJoint_GetStopImpulse( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_tireJoint );
	return base->tireJoint.stopImpulse;
}

float b3TireJoint_GetLateralImpulse( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_tireJoint );
	return base->tireJoint.lateralImpulse;
}

float b3TireJoint_GetRollingImpulse( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_tireJoint );
	return base->tireJoint.rollingImpulse;
}

b3Vec3 b3GetTireJointForce( b3World* world, b3JointSim* base )
{
	b3TireJoint* joint = &base->tireJoint;
	b3Vec3 impulse = b3MulSV( joint->suspensionImpulse + joint->stopImpulse, joint->normal );
	impulse = b3MulAdd( impulse, joint->lateralImpulse, joint->lateralAxis );
	impulse = b3MulAdd( impulse, joint->rollingImpulse, joint->rollingAxis );
	return b3MulSV( world->inv_h, impulse );
}

static float b3ComputeAxialMass( float mA, float mB, b3Matrix3 iA, b3Matrix3 iB, b3Vec3 rA, b3Vec3 rB, b3Vec3 axis )
{
	b3Vec3 rnA = b3Cross( rA, axis );
	b3Vec3 rnB = b3Cross( rB, axis );
	float k = mA + mB + b3Dot( rnA, b3MulMV( iA, rnA ) ) + b3Dot( rnB, b3MulMV( iB, rnB ) );
	return k > 0.0f ? 1.0f / k : 0.0f;
}

void b3PrepareTireJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_tireJoint );

	b3World* world = context->world;

	b3Body* bodyA = b3Array_Get( world->bodies, base->bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, base->bodyIdB );

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

	b3TireJoint* joint = &base->tireJoint;
	joint->indexA = bodyA->setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	b3Quat qA = bodySimA->transform.q;
	b3Quat qB = bodySimB->transform.q;
	b3Quat frameQB = b3MulQuat( qB, base->localFrameB.q );

	joint->originA = b3RotateVector( qA, b3Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->mountB = b3RotateVector( qB, b3Sub( base->localFrameB.p, bodySimB->localCenter ) );
	joint->axis = b3RotateVector( frameQB, b3Vec3_axisX );
	joint->deltaCenter = b3SubPos( bodySimB->center, bodySimA->center );

	b3Vec3 normal = b3RotateVector( qA, joint->localNormal );
	joint->normal = normal;
	joint->cosine = b3MaxFloat( b3Dot( joint->axis, normal ), B3_TIRE_MIN_COSINE );

	b3Vec3 axle = b3RotateVector( frameQB, b3Vec3_axisZ );
	joint->contactRadius = b3GetTireContactRadius( joint, normal, axle );

	b3Vec3 separation = b3Sub( b3Add( joint->deltaCenter, joint->mountB ), joint->originA );
	float clearance = b3Dot( separation, normal ) - joint->contactRadius;
	float length = b3ClampFloat( clearance / joint->cosine, 0.0f, joint->maxLength );

	b3Vec3 wheelContact =
		b3Sub( b3MulSub( joint->mountB, length, joint->axis ), b3MulSV( joint->contactRadius, normal ) );
	joint->anchorB = wheelContact;

	b3Vec3 contactFromA = b3Add( joint->deltaCenter, wheelContact );
	float height = b3Dot( b3Sub( contactFromA, joint->originA ), normal );
	joint->anchorA = b3MulSub( contactFromA, height, normal );
	joint->frictionAnchorB = b3MulAdd( wheelContact, joint->forceHeight, normal );

	b3Vec3 lateral = b3MulSub( axle, b3Dot( axle, normal ), normal );
	float lateralLength;
	lateral = b3GetLengthAndNormalize( &lateralLength, lateral );
	if ( lateralLength < 0.1f )
	{
		lateral = b3Perp( normal );
	}
	joint->lateralAxis = lateral;
	joint->rollingAxis = b3Cross( normal, lateral );

	joint->normalMass = b3ComputeAxialMass( mA, mB, iA, iB, joint->anchorA, joint->anchorB, normal );
	joint->lateralMass = b3ComputeAxialMass( mA, mB, iA, iB, joint->anchorA, joint->frictionAnchorB, lateral );
	joint->rollingMass =
		b3ComputeAxialMass( mA, mB, iA, iB, joint->anchorA, joint->frictionAnchorB, joint->rollingAxis );

	// Softness for extension, compression, and bottom out.
	joint->springSoftness = b3MakeSoft( joint->hertz, joint->dampingRatio, context->h );
	joint->bumpSoftness = b3MakeSoft( joint->hertz, joint->bumpDampingRatio, context->h );
	joint->stopSoftness = b3MakeSoft( joint->stopHertz, joint->stopDampingRatio, context->h );

	if ( context->enableWarmStarting == false )
	{
		joint->suspensionImpulse = 0.0f;
		joint->stopImpulse = 0.0f;
		joint->lateralImpulse = 0.0f;
		joint->rollingImpulse = 0.0f;
	}
}

void b3WarmStartTireJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_tireJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	b3TireJoint* joint = &base->tireJoint;

	b3BodyState dummyState = b3_identityBodyState;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 normalImpulse = b3MulSV( joint->suspensionImpulse + joint->stopImpulse, joint->normal );
	b3Vec3 frictionImpulse = b3MulSV( joint->lateralImpulse, joint->lateralAxis );
	frictionImpulse = b3MulAdd( frictionImpulse, joint->rollingImpulse, joint->rollingAxis );
	b3Vec3 linearImpulse = b3Add( normalImpulse, frictionImpulse );

	b3Vec3 angularImpulseA = b3Cross( joint->anchorA, linearImpulse );
	b3Vec3 angularImpulseB =
		b3Add( b3Cross( joint->anchorB, normalImpulse ), b3Cross( joint->frictionAnchorB, frictionImpulse ) );

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = b3MulSub( stateA->linearVelocity, mA, linearImpulse );
		stateA->angularVelocity = b3Sub( stateA->angularVelocity, b3MulMV( iA, angularImpulseA ) );
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = b3MulAdd( stateB->linearVelocity, mB, linearImpulse );
		stateB->angularVelocity = b3Add( stateB->angularVelocity, b3MulMV( iB, angularImpulseB ) );
	}
}

void b3SolveTireJoint( b3JointSim* base, b3StepContext* context, bool useBias )
{
	B3_ASSERT( base->type == b3_tireJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	b3TireJoint* joint = &base->tireJoint;

	b3BodyState dummyState = b3_identityBodyState;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	b3Vec3 normal = joint->normal;
	b3Vec3 anchorA = joint->anchorA;
	b3Vec3 anchorB = joint->anchorB;

	b3Vec3 originA = b3RotateVector( stateA->deltaRotation, joint->originA );
	b3Vec3 mountB = b3RotateVector( stateB->deltaRotation, joint->mountB );
	b3Vec3 dp = b3Sub( stateB->deltaPosition, stateA->deltaPosition );
	b3Vec3 separation = b3Add( b3Add( dp, joint->deltaCenter ), b3Sub( mountB, originA ) );
	float clearance = b3Dot( separation, normal ) - joint->contactRadius;
	float length = clearance / joint->cosine;

	// Suspension
	{
		float oldImpulse = joint->suspensionImpulse;
		float newImpulse = 0.0f;

		if ( joint->hertz > 0.0f && length <= joint->maxLength )
		{
			float c = clearance - joint->restLength * joint->cosine;
			float vn =
				b3Dot( b3Sub( b3Add( vB, b3Cross( wB, anchorB ) ), b3Add( vA, b3Cross( wA, anchorA ) ) ), normal );

			// Used different softness for compression and extension. Lower damping on compression to smooth the ride.
			b3Softness softness = vn < 0.0f ? joint->bumpSoftness : joint->springSoftness;
			float impulse = -softness.massScale * joint->normalMass * ( vn + softness.biasRate * c ) -
							softness.impulseScale * oldImpulse;
			float maxImpulse = context->h * joint->maxSuspensionForce;
			newImpulse = b3ClampFloat( oldImpulse + impulse, 0.0f, maxImpulse );
		}

		joint->suspensionImpulse = newImpulse;
		float impulse = newImpulse - oldImpulse;

		b3Vec3 P = b3MulSV( impulse, normal );
		vA = b3MulSub( vA, mA, P );
		wA = b3Sub( wA, b3MulMV( iA, b3Cross( anchorA, P ) ) );
		vB = b3MulAdd( vB, mB, P );
		wB = b3Add( wB, b3MulMV( iB, b3Cross( anchorB, P ) ) );
	}

	// Bottom out
	{
		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;

		bool active = true;
		if ( joint->stopHertz > 0.0f )
		{
			// Bottom out?
			active = clearance <= 0.0f;
			bias = joint->stopSoftness.massScale * joint->stopSoftness.biasRate * clearance;
			massScale = joint->stopSoftness.massScale;
			impulseScale = joint->stopSoftness.impulseScale;
		}
		else if ( clearance > 0.0f )
		{
			// Speculative
			bias = clearance * context->inv_h;
		}
		else if ( useBias )
		{
			b3Softness softness = joint->indexA == B3_NULL_INDEX ? context->staticSoftness : context->contactSoftness;
			bias = b3MaxFloat( softness.massScale * softness.biasRate * clearance, -context->world->contactSpeed );
			massScale = softness.massScale;
			impulseScale = softness.impulseScale;
		}

		float vn = b3Dot( b3Sub( b3Add( vB, b3Cross( wB, anchorB ) ), b3Add( vA, b3Cross( wA, anchorA ) ) ), normal );
		float impulse = -joint->normalMass * ( massScale * vn + bias ) - impulseScale * joint->stopImpulse;
		float newImpulse = active ? b3MaxFloat( joint->stopImpulse + impulse, 0.0f ) : 0.0f;
		impulse = newImpulse - joint->stopImpulse;
		joint->stopImpulse = newImpulse;

		b3Vec3 P = b3MulSV( impulse, normal );
		vA = b3MulSub( vA, mA, P );
		wA = b3Sub( wA, b3MulMV( iA, b3Cross( anchorA, P ) ) );
		vB = b3MulAdd( vB, mB, P );
		wB = b3Add( wB, b3MulMV( iB, b3Cross( anchorB, P ) ) );
	}

	// Friction. Technically shouldn't eval during bias, but testing shows it works better with it and performance
	// doesn't matter much for this joint.
	{
		b3Vec3 frictionAnchorB = joint->frictionAnchorB;
		b3Vec3 lateral = joint->lateralAxis;
		b3Vec3 rolling = joint->rollingAxis;

		float normalImpulse = joint->suspensionImpulse + joint->stopImpulse;

		b3Vec3 dv = b3Sub( b3Add( vB, b3Cross( wB, frictionAnchorB ) ), b3Add( vA, b3Cross( wA, anchorA ) ) );
		float lateralImpulse = joint->lateralImpulse - joint->lateralMass * b3Dot( dv, lateral );
		float rollingImpulse = joint->rollingImpulse - joint->rollingMass * b3Dot( dv, rolling );

		// Braking.
		float maxBrakeImpulse = context->h * joint->brakeForce + joint->rollingResistance * normalImpulse;
		rollingImpulse = b3ClampFloat( rollingImpulse, -maxBrakeImpulse, maxBrakeImpulse );

		// Lateral and longitudinal friction. Restricted to an ellipse.
		float maxLateral = joint->lateralFriction * normalImpulse;
		float maxRolling = joint->rollingFriction * normalImpulse;
		bool saturated = false;
		if ( maxLateral > 0.0f && maxRolling > 0.0f )
		{
			float x = lateralImpulse / maxLateral;
			float y = rollingImpulse / maxRolling;
			float s2 = x * x + y * y;
			if ( s2 > 1.0f )
			{
				float s = 1.0f / sqrtf( s2 );
				lateralImpulse *= s;
				rollingImpulse *= s;
				saturated = true;
			}
		}
		else
		{
			lateralImpulse = b3ClampFloat( lateralImpulse, -maxLateral, maxLateral );
			rollingImpulse = b3ClampFloat( rollingImpulse, -maxRolling, maxRolling );
		}

		joint->saturated = saturated;

		float deltaLateral = lateralImpulse - joint->lateralImpulse;
		float deltaRolling = rollingImpulse - joint->rollingImpulse;
		joint->lateralImpulse = lateralImpulse;
		joint->rollingImpulse = rollingImpulse;

		b3Vec3 P = b3MulAdd( b3MulSV( deltaLateral, lateral ), deltaRolling, rolling );
		vA = b3MulSub( vA, mA, P );
		wA = b3Sub( wA, b3MulMV( iA, b3Cross( anchorA, P ) ) );
		vB = b3MulAdd( vB, mB, P );
		wB = b3Add( wB, b3MulMV( iB, b3Cross( frictionAnchorB, P ) ) );
	}

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

void b3DrawTireJoint( b3DebugDraw* draw, b3World* world, b3JointSim* base, b3WorldTransform transformA,
					  b3WorldTransform transformB, float scale )
{
	B3_ASSERT( base->type == b3_tireJoint );
	B3_UNUSED( scale );

	b3TireJoint* joint = &base->tireJoint;

	b3Quat frameQB = b3MulQuat( transformB.q, base->localFrameB.q );
	b3Vec3 axis = b3RotateVector( frameQB, b3Vec3_axisX );
	b3Vec3 axle = b3RotateVector( frameQB, b3Vec3_axisZ );
	b3Vec3 normal = b3RotateVector( transformA.q, joint->localNormal );
	b3Pos pA = b3TransformWorldPoint( transformA, base->localFrameA.p );
	b3Pos mount = b3TransformWorldPoint( transformB, base->localFrameB.p );

	float length = b3ComputeSuspensionLength( base, transformA, transformB );
	float drawLength = b3ClampFloat( length, 0.0f, joint->maxLength );
	b3Pos hub = b3OffsetPos( mount, b3MulSV( -drawLength, axis ) );

	float normalForce = world->inv_h * ( joint->suspensionImpulse + joint->stopImpulse );

	b3HexColor color;
	if ( normalForce == 0.0f )
	{
		color = b3_colorGray;
	}
	else if ( joint->saturated )
	{
		color = b3_colorRed;
	}
	else if ( joint->stopImpulse > 0.0f )
	{
		color = b3_colorOrange;
	}
	else
	{
		color = b3_colorLimeGreen;
	}

	draw->DrawSegmentFcn( mount, hub, b3_colorWhite, draw->context );
	draw->DrawPointFcn( mount, 5.0f, b3_colorWhite, draw->context );

	b3Vec3 u = b3Cross( axle, axis );
	float ul;
	u = b3GetLengthAndNormalize( &ul, u );
	b3Vec3 v = b3Cross( axle, u );

	int segmentCount = 16;
	float radius = joint->radius;
	b3Pos p1 = b3OffsetPos( hub, b3MulSV( radius, u ) );
	for ( int i = 1; i <= segmentCount; ++i )
	{
		float angle = 2.0f * B3_PI * (float)i / (float)segmentCount;
		b3CosSin cs = b3ComputeCosSin( angle );
		b3Vec3 offset = b3MulAdd( b3MulSV( radius * cs.cosine, u ), radius * cs.sine, v );
		b3Pos p2 = b3OffsetPos( hub, offset );
		draw->DrawSegmentFcn( p1, p2, color, draw->context );
		p1 = p2;
	}

	draw->DrawPointFcn( pA, 6.0f, color, draw->context );
	draw->DrawSegmentFcn( pA, b3OffsetPos( pA, b3MulSV( 0.25f * radius, normal ) ), b3_colorOrange, draw->context );

	if ( draw->drawJointExtras )
	{
		char buffer[128];
		snprintf( buffer, 128, "L = %.3f, N = %.0f, lat = %.0f, roll = %.0f", length, normalForce,
				  world->inv_h * joint->lateralImpulse, world->inv_h * joint->rollingImpulse );
		draw->DrawStringFcn( hub, buffer, color, draw->context );
	}
}
