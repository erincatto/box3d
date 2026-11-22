// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "joint.h"
#include "math_internal.h"
#include "physics_world.h"
#include "recording.h"
#include "solver.h"
#include "solver_set.h"

// needed for dll export
#include "box3d/box3d.h"

void b3MotionLockJoint_EnableLinearLock( b3JointId jointId, int axis, bool flag )
{
	B3_CHECK_INPUT( 0 <= axis && axis < 3 );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, MotionLockJointEnableLinearLock, jointId, axis, flag );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_motionLockJoint );
	b3MotionLockJoint* joint = &base->motionLockJoint;
	if ( flag != joint->lockLinear[axis] )
	{
		joint->lockLinear[axis] = flag;
		( &joint->linearImpulse.x )[axis] = 0.0f;
	}
}

bool b3MotionLockJoint_IsLinearLockEnabled( b3JointId jointId, int axis )
{
	B3_CHECK_INPUT_RETURN( 0 <= axis && axis < 3, false );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_motionLockJoint );
	return base->motionLockJoint.lockLinear[axis];
}

void b3MotionLockJoint_EnableAngularLock( b3JointId jointId, bool flag )
{
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, MotionLockJointEnableAngularLock, jointId, flag );
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_motionLockJoint );
	b3MotionLockJoint* joint = &base->motionLockJoint;
	if ( flag != joint->lockRotationXY )
	{
		joint->lockRotationXY = flag;
		joint->angularImpulse = (b3Vec2){ 0.0f, 0.0f };
	}
}

bool b3MotionLockJoint_IsAngularLockEnabled( b3JointId jointId )
{
	b3JointSim* base = b3GetJointSimCheckType( jointId, b3_motionLockJoint );
	return base->motionLockJoint.lockRotationXY;
}

b3Vec3 b3GetMotionLockJointForce( b3World* world, b3JointSim* base )
{
	b3WorldTransform transformA = b3GetBodyTransform( world, base->bodyIdA );
	b3Vec3 force = b3MulSV( world->inv_h, base->motionLockJoint.linearImpulse );
	force = b3RotateVector( base->localFrameA.q, force );
	return b3RotateVector( transformA.q, force );
}

b3Vec3 b3GetMotionLockJointTorque( b3World* world, b3JointSim* base )
{
	b3MotionLockJoint* joint = &base->motionLockJoint;
	b3Vec3 impulse = b3Blend2( joint->angularImpulse.x, joint->perpAxisX, joint->angularImpulse.y, joint->perpAxisY );
	return b3MulSV( world->inv_h, impulse );
}

static inline bool b3IsZeroInverseInertia( b3Matrix3 m )
{
	return m.cx.x == 0.0f && m.cy.y == 0.0f && m.cz.z == 0.0f;
}

static inline void b3ComputePerpAxes( b3Quat quatA, b3Quat relQ, b3Vec3* perpAxisX, b3Vec3* perpAxisY )
{
	b3Vec3 rX = b3Add( b3MulSV( relQ.s, b3Vec3_axisX ), b3Cross( relQ.v, b3Vec3_axisX ) );
	b3Vec3 rY = b3Add( b3MulSV( relQ.s, b3Vec3_axisY ), b3Cross( relQ.v, b3Vec3_axisY ) );
	*perpAxisX = b3MulSV( 0.5f, b3RotateVector( quatA, rX ) );
	*perpAxisY = b3MulSV( 0.5f, b3RotateVector( quatA, rY ) );
}

static inline b3Vec3 b3GetMotionLockSeparation( const b3MotionLockJoint* joint, const b3BodyState* stateA,
										 const b3BodyState* stateB, b3Vec3 rA, b3Vec3 rB )
{
	b3Vec3 dpA = b3Add( stateA->deltaPosition, b3Sub( rA, joint->frameA.p ) );
	b3Vec3 dpB = b3Add( stateB->deltaPosition, b3Sub( rB, joint->frameB.p ) );
	return b3Add( joint->separation, b3Sub( dpB, dpA ) );
}

void b3PrepareMotionLockJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_motionLockJoint );

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

	base->invMassA = bodySimA->invMass;
	base->invMassB = bodySimB->invMass;
	base->invIA = bodySimA->invInertiaWorld;
	base->invIB = bodySimB->invInertiaWorld;

	b3Matrix3 invInertiaSum = b3AddMM( base->invIA, base->invIB );
	base->fixedRotation = b3Det( invInertiaSum ) < 1000.0f * FLT_MIN;

	b3MotionLockJoint* joint = &base->motionLockJoint;
	joint->indexA = bodyA->setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	joint->frameA.q = b3MulQuat( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b3RotateVector( bodySimA->transform.q, b3Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b3MulQuat( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b3RotateVector( bodySimB->transform.q, b3Sub( base->localFrameB.p, bodySimB->localCenter ) );

	// In double precision the frameA position can be very large when body A is at the origin. This helps mitigate it.
	// But it is probably better to use a motion lock body that is closer.
	b3Pos posA = b3OffsetPos( bodySimA->center, joint->frameA.p );
	b3Pos posB = b3OffsetPos( bodySimB->center, joint->frameB.p );
	joint->separation = b3SubPos( posB, posA );

	bool zeroIA = b3IsZeroInverseInertia( base->invIA );
	bool zeroIB = b3IsZeroInverseInertia( base->invIB );
	bool anchorAtCenterB = joint->frameB.p.x == 0.0f && joint->frameB.p.y == 0.0f && joint->frameB.p.z == 0.0f;
	joint->diagonal = zeroIA && ( zeroIB || anchorAtCenterB );

	b3Quat quatB = joint->frameB.q;
	if ( b3DotQuat( joint->frameA.q, quatB ) < 0.0f )
	{
		quatB = b3NegateQuat( quatB );
	}

	b3Quat relQ = b3InvMulQuat( joint->frameA.q, quatB );
	b3ComputePerpAxes( joint->frameA.q, relQ, &joint->perpAxisX, &joint->perpAxisY );

	if ( context->enableWarmStarting == false )
	{
		joint->linearImpulse = b3Vec3_zero;
		joint->angularImpulse = (b3Vec2){ 0.0f, 0.0f };
	}
	else if ( base->fixedRotation )
	{
		joint->angularImpulse = (b3Vec2){ 0.0f, 0.0f };
	}
}

void b3WarmStartMotionLockJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_motionLockJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3MotionLockJoint* joint = &base->motionLockJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
	b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );
	b3Vec3 d = b3GetMotionLockSeparation( joint, stateA, stateB, rA, rB );

	b3Matrix3 axes = b3MakeMatrixFromQuat( b3MulQuat( stateA->deltaRotation, joint->frameA.q ) );
	b3Vec3 lambda = joint->linearImpulse;
	b3Vec3 P = b3Blend3( lambda.x, axes.cx, lambda.y, axes.cy, lambda.z, axes.cz );

	b3Vec3 angularImpulse =
		b3Blend2( joint->angularImpulse.x, joint->perpAxisX, joint->angularImpulse.y, joint->perpAxisY );
	b3Vec3 LA = b3Add( b3Cross( b3Add( rA, d ), P ), angularImpulse );
	b3Vec3 LB = b3Add( b3Cross( rB, P ), angularImpulse );

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = b3MulSub( stateA->linearVelocity, mA, P );
		stateA->angularVelocity = b3Sub( stateA->angularVelocity, b3MulMV( iA, LA ) );
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = b3MulAdd( stateB->linearVelocity, mB, P );
		stateB->angularVelocity = b3Add( stateB->angularVelocity, b3MulMV( iB, LB ) );
	}
}

void b3SolveMotionLockJoint( b3JointSim* base, b3StepContext* context, bool useBias )
{
	B3_ASSERT( base->type == b3_motionLockJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	b3Matrix3 iA = base->invIA;
	b3Matrix3 iB = base->invIB;

	b3BodyState dummyState = b3_identityBodyState;

	b3MotionLockJoint* joint = &base->motionLockJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 wA = stateA->angularVelocity;
	b3Vec3 vB = stateB->linearVelocity;
	b3Vec3 wB = stateB->angularVelocity;

	b3Quat quatA = b3MulQuat( stateA->deltaRotation, joint->frameA.q );

	float massScale = 1.0f;
	float impulseScale = 0.0f;
	if ( useBias )
	{
		massScale = base->constraintSoftness.massScale;
		impulseScale = base->constraintSoftness.impulseScale;
	}

	if ( joint->lockRotationXY && base->fixedRotation == false )
	{
		b3Quat quatB = b3MulQuat( stateB->deltaRotation, joint->frameB.q );
		if ( b3DotQuat( quatA, quatB ) < 0.0f )
		{
			quatB = b3NegateQuat( quatB );
		}

		b3Quat relQ = b3InvMulQuat( quatA, quatB );

		b3Vec3 perpAxisX, perpAxisY;
		b3ComputePerpAxes( quatA, relQ, &perpAxisX, &perpAxisY );
		joint->perpAxisX = perpAxisX;
		joint->perpAxisY = perpAxisY;

		b3Vec2 bias = { 0.0f, 0.0f };
		if ( useBias )
		{
			float rate = base->constraintSoftness.biasRate;
			bias = (b3Vec2){ .x = rate * relQ.v.x, .y = rate * relQ.v.y };
		}

		// Block solver to constrain rotation.
		b3Matrix3 invInertiaSum = b3AddMM( iA, iB );
		b3Vec3 kx = b3MulMV( invInertiaSum, perpAxisX );
		b3Vec3 ky = b3MulMV( invInertiaSum, perpAxisY );
		float kxx = b3Dot( perpAxisX, kx );
		float kxy = b3Dot( perpAxisY, kx );
		float kyy = b3Dot( perpAxisY, ky );
		b3Matrix2 k = { { kxx, kxy }, { kxy, kyy } };

		b3Vec3 wRel = b3Sub( wB, wA );
		b3Vec2 cdot = { b3Dot( wRel, perpAxisX ) + bias.x, b3Dot( wRel, perpAxisY ) + bias.y };
		b3Vec2 solution = b3Solve2( k, cdot );

		b3Vec2 oldImpulse = joint->angularImpulse;
		b3Vec2 impulse = {
			-massScale * solution.x - impulseScale * oldImpulse.x,
			-massScale * solution.y - impulseScale * oldImpulse.y,
		};
		joint->angularImpulse = b3Add2( oldImpulse, impulse );

		b3Vec3 angularImpulse = b3Blend2( impulse.x, perpAxisX, impulse.y, perpAxisY );
		wA = b3Sub( wA, b3MulMV( iA, angularImpulse ) );
		wB = b3Add( wB, b3MulMV( iB, angularImpulse ) );
	}

	int axisIndices[3];
	int count = 0;
	for ( int i = 0; i < 3; ++i )
	{
		if ( joint->lockLinear[i] )
		{
			axisIndices[count] = i;
			count += 1;
		}
	}

	if ( count > 0 )
	{
		b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
		b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );
		b3Vec3 d = b3GetMotionLockSeparation( joint, stateA, stateB, rA, rB );
		b3Vec3 rAd = b3Add( rA, d );

		b3Matrix3 frameAxes = b3MakeMatrixFromQuat( quatA );
		b3Vec3 allAxes[3] = { frameAxes.cx, frameAxes.cy, frameAxes.cz };

		b3Vec3 vRel = b3Sub( b3Add( vB, b3Cross( wB, rB ) ), b3Add( vA, b3Cross( wA, rAd ) ) );
		float biasRate = useBias ? base->constraintSoftness.biasRate : 0.0f;

		b3Vec3 axes[3];
		b3Vec3 sA[3];
		b3Vec3 sB[3];
		float rhs[3];
		float* oldImpulses = &joint->linearImpulse.x;
		float impulses[3] = { 0.0f, 0.0f, 0.0f };

		for ( int i = 0; i < count; ++i )
		{
			b3Vec3 axis = allAxes[axisIndices[i]];
			axes[i] = axis;
			sA[i] = b3Cross( rAd, axis );
			sB[i] = b3Cross( rB, axis );
			rhs[i] = b3Dot( vRel, axis ) + biasRate * b3Dot( d, axis );
		}

		float m = mA + mB;

		if ( joint->diagonal )
		{
			// The block matrix is diagonal. No need to do a matrix inverse.
			float invM = m > 0.0f ? 1.0f / m : 0.0f;
			for ( int i = 0; i < count; ++i )
			{
				impulses[i] = -massScale * invM * rhs[i] - impulseScale * oldImpulses[axisIndices[i]];
			}
		}
		else
		{
			float kd[3][3];
			for ( int i = 0; i < count; ++i )
			{
				b3Vec3 iAsA = b3MulMV( iA, sA[i] );
				b3Vec3 iBsB = b3MulMV( iB, sB[i] );
				for ( int j = i; j < count; ++j )
				{
					float k = b3Dot( sA[j], iAsA ) + b3Dot( sB[j], iBsB );
					if ( i == j )
					{
						k += m;
					}
					kd[i][j] = k;
					kd[j][i] = k;
				}
			}

			float solution[3] = { 0.0f, 0.0f, 0.0f };
			if ( count == 1 )
			{
				solution[0] = kd[0][0] > 0.0f ? rhs[0] / kd[0][0] : 0.0f;
			}
			else if ( count == 2 )
			{
				b3Matrix2 k = { { kd[0][0], kd[1][0] }, { kd[0][1], kd[1][1] } };
				b3Vec2 s = b3Solve2( k, (b3Vec2){ rhs[0], rhs[1] } );
				solution[0] = s.x;
				solution[1] = s.y;
			}
			else
			{
				b3Matrix3 k = {
					{ kd[0][0], kd[1][0], kd[2][0] },
					{ kd[0][1], kd[1][1], kd[2][1] },
					{ kd[0][2], kd[1][2], kd[2][2] },
				};
				b3Vec3 s = b3Solve3( k, (b3Vec3){ rhs[0], rhs[1], rhs[2] } );
				solution[0] = s.x;
				solution[1] = s.y;
				solution[2] = s.z;
			}

			for ( int i = 0; i < count; ++i )
			{
				impulses[i] = -massScale * solution[i] - impulseScale * oldImpulses[axisIndices[i]];
			}
		}

		b3Vec3 P = b3Vec3_zero;
		b3Vec3 LA = b3Vec3_zero;
		b3Vec3 LB = b3Vec3_zero;
		for ( int i = 0; i < count; ++i )
		{
			oldImpulses[axisIndices[i]] += impulses[i];
			P = b3MulAdd( P, impulses[i], axes[i] );
			LA = b3MulAdd( LA, impulses[i], sA[i] );
			LB = b3MulAdd( LB, impulses[i], sB[i] );
		}

		vA = b3MulSub( vA, mA, P );
		vB = b3MulAdd( vB, mB, P );

		if ( joint->diagonal == false )
		{
			wA = b3Sub( wA, b3MulMV( iA, LA ) );
			wB = b3Add( wB, b3MulMV( iB, LB ) );
		}
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

void b3DrawMotionLockJoint( b3DebugDraw* draw, b3JointSim* base, b3WorldTransform transformA,
							b3WorldTransform transformB, float scale )
{
	b3MotionLockJoint* joint = &base->motionLockJoint;

	b3WorldTransform frameA = b3MulWorldTransforms( transformA, base->localFrameA );
	b3WorldTransform frameB = b3MulWorldTransforms( transformB, base->localFrameB );

	float length = 0.1f * scale;
	b3Matrix3 axesA = b3MakeMatrixFromQuat( frameA.q );
	b3Vec3 axes[3] = { axesA.cx, axesA.cy, axesA.cz };
	b3HexColor colors[3] = { b3_colorRed, b3_colorGreen, b3_colorBlue };

	for ( int i = 0; i < 3; ++i )
	{
		if ( joint->lockLinear[i] == false )
		{
			continue;
		}

		b3Vec3 offset = b3MulSV( length, axes[i] );
		draw->DrawSegmentFcn( b3OffsetPos( frameA.p, b3Neg( offset ) ), b3OffsetPos( frameA.p, offset ), colors[i],
							  draw->context );
	}

	if ( joint->lockRotationXY )
	{
		b3Vec3 axisB = b3RotateVector( frameB.q, b3Vec3_axisZ );
		draw->DrawSegmentFcn( frameA.p, b3OffsetPos( frameA.p, b3MulSV( 2.0f * length, axes[2] ) ), b3_colorYellow,
							  draw->context );
		draw->DrawSegmentFcn( frameB.p, b3OffsetPos( frameB.p, b3MulSV( 2.0f * length, axisB ) ), b3_colorOrange,
							  draw->context );
	}

	draw->DrawSegmentFcn( frameA.p, frameB.p, b3_colorDarkSeaGreen, draw->context );
}
