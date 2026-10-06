// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact_solver.h"

#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "math_internal.h"
#include "physics_world.h"
#include "platform.h"
#include "simd.h"
#include "solver_set.h"

#if B3_ENABLE_VALIDATION
#include "shape.h"
#endif

// todo clean this file up because it is only used for overflow now

// contact separation for sub-stepping
// s = s0 + dot(cB + rB - cA - rA, normal)
// normal is held constant
// body positions c can translation and anchors r can rotate
// s(t) = s0 + dot(cB(t) + rB(t) - cA(t) - rA(t), normal)
// s(t) = s0 + dot(cB0 + dpB + rot(dqB, rB0) - cA0 - dpA - rot(dqA, rA0), normal)
// s(t) = s0 + dot(cB0 - cA0, normal) + dot(dpB - dpA + rot(dqB, rB0) - rot(dqA, rA0), normal)
// s_base = s0 + dot(cB0 - cA0, normal)

// Prepare overflow constraints
void b3PrepareContacts_Mesh( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( prepare_contact, "Prepare Contact", b3_colorYellow, true );

	b3World* world = context->world;
	b3BodySim* bodySims = context->sims;
	b3BodyState* states = context->states;

	float warmStartScale = world->enableWarmStarting ? 1.0f : 0.0f;
	bool anyRestitution = false;

	// Used for friction center weighting.
	float invTau = 1.0f / B3_SPECULATIVE_DISTANCE;

	B3_ASSERT( block.blockType == b3_overflowBlock );
	b3GraphColor* overflow = world->constraintGraph.colors + B3_OVERFLOW_INDEX;
	b3ContactPrepareSpan* spans = context->overflowSpans;
	b3ManifoldConstraint* manifoldBase = overflow->manifoldConstraints;
	b3ContactConstraint* base = overflow->contactConstraints;

	int index = block.startIndex;
	int endIndex = block.startIndex + block.count;

	// Find color for start index. Linear search but fast.
	int colorIndex = 0;
	while ( spans[colorIndex + 1].start <= index )
	{
		colorIndex += 1;
	}

	// Loop over block
	while ( index < endIndex )
	{
		int colorStart = spans[colorIndex].start;
		int colorEndIndex = b3MinInt( spans[colorIndex + 1].start, endIndex );
		b3ContactSpec* specs = spans[colorIndex].contacts;

		// Loop over color
		for ( ; index < colorEndIndex; ++index )
		{
			b3ContactConstraint* contactConstraint = base + index;

			int localIndex = index - colorStart;
			B3_ASSERT( 0 <= localIndex && localIndex < spans[colorIndex].count );
			int contactId = specs[localIndex].contactId;
			b3Contact* contact = b3Array_Get( world->contacts, contactId );
			B3_ASSERT( contact->contactId == contactId );

			int indexA = b3DecodeAwakeIndex( contact->encodedBodySimA );
			int indexB = b3DecodeAwakeIndex( contact->encodedBodySimB );

#if B3_ENABLE_VALIDATION
			{
				b3Body* bodyA = b3Array_Get( world->bodies, contact->edges[0].bodyId );
				b3Body* bodyB = b3Array_Get( world->bodies, contact->edges[1].bodyId );
				B3_ASSERT( contact->encodedBodySimA == b3EncodeBodySimIndex( bodyA ) );
				B3_ASSERT( contact->encodedBodySimB == b3EncodeBodySimIndex( bodyB ) );
			}
#endif

			// Body A data
			float mA;
			b3Matrix3 iA;

			if ( indexA == B3_NULL_INDEX )
			{
				mA = 0.0f;
				iA = b3Mat3_zero;
			}
			else
			{
				b3BodySim* simA = bodySims + indexA;
				mA = simA->invMass;
				iA = simA->invInertiaWorld;
			}

			// Body B data
			float mB;
			b3Matrix3 iB;

			if ( indexB == B3_NULL_INDEX )
			{
				mB = 0.0f;
				iB = b3Mat3_zero;
			}
			else
			{
				b3BodySim* simB = bodySims + indexB;
				mB = simB->invMass;
				iB = simB->invInertiaWorld;
			}

			int manifoldCount = contact->manifoldCount;
			contactConstraint->contact = contact;
			contactConstraint->manifoldCount = manifoldCount;
			contactConstraint->indexA = indexA;
			contactConstraint->indexB = indexB;
			contactConstraint->invIA = iA;
			contactConstraint->invMassA = mA;
			contactConstraint->invIB = iB;
			contactConstraint->invMassB = mB;
			contactConstraint->rollingMass = b3InvertMatrix( b3AddMM( iA, iB ) );
			contactConstraint->softness =
				( contact->flags & b3_contactStaticFlag ) != 0 ? context->staticSoftness : context->contactSoftness;
			contactConstraint->friction = contact->friction;
			contactConstraint->restitution = contact->restitution;
			contactConstraint->rollingResistance = contact->rollingResistance;

			// Only sample contact point normal velocity if needed.
			bool haveRestitution = contact->restitution > 0.0f;
			bool hitEvents = ( contact->flags & b3_simEnableHitEvent ) != 0;
			bool sampleVelocity = haveRestitution || hitEvents;
			anyRestitution = anyRestitution || haveRestitution;

			b3Vec3 vA = b3Vec3_zero;
			b3Vec3 wA = b3Vec3_zero;
			b3Vec3 vB = b3Vec3_zero;
			b3Vec3 wB = b3Vec3_zero;

			if ( sampleVelocity )
			{
				if ( indexA != B3_NULL_INDEX )
				{
					vA = states[indexA].linearVelocity;
					wA = states[indexA].angularVelocity;
				}

				if ( indexB != B3_NULL_INDEX )
				{
					vB = states[indexB].linearVelocity;
					wB = states[indexB].angularVelocity;
				}
			}

			b3ManifoldConstraint* manifoldConstraints = manifoldBase + specs[localIndex].manifoldStart;
			contactConstraint->constraints = manifoldConstraints;

			for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
			{
				b3Manifold* manifold = contact->manifolds + manifoldIndex;
				b3ManifoldConstraint* constraint = manifoldConstraints + manifoldIndex;
				int pointCount = manifold->pointCount;
				b3Vec3 normal = manifold->normal;
				b3Vec3 tangent1 = b3Perp( normal );
				b3Vec3 tangent2 = b3Cross( tangent1, normal );

				constraint->pointCount = pointCount;
				constraint->normal = normal;
				constraint->tangent1 = tangent1;
				constraint->tangent2 = tangent2;

				// Stiffer for static contacts to avoid bodies getting pushed through the ground
				constraint->tangentVelocity1 = b3Dot( contact->tangentVelocity, constraint->tangent1 );
				constraint->tangentVelocity2 = b3Dot( contact->tangentVelocity, constraint->tangent2 );

				b3Vec3 centerA = b3Vec3_zero;
				b3Vec3 centerB = b3Vec3_zero;
				float totalFrictionWeight = 0.0f;

				for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
				{
					b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;

					// Copy data from manifold point
					b3ManifoldPoint* mp = manifold->points + pointIndex;
					cp->rA = mp->anchorA;
					cp->rB = mp->anchorB;

					float s = mp->separation;
					cp->baseSeparation = s - b3Dot( b3Sub( cp->rB, cp->rA ), normal );
					cp->normalImpulse = warmStartScale * mp->normalImpulse;
					cp->totalNormalImpulse = 0.0f;
					cp->restitutionImpulse = 0.0f;

					b3Vec3 rA = cp->rA;
					b3Vec3 rB = cp->rB;

					b3Vec3 rnA = b3Cross( rA, normal );
					b3Vec3 rnB = b3Cross( rB, normal );
					float kNormal = mA + mB + b3Dot( rnA, b3MulMV( iA, rnA ) ) + b3Dot( rnB, b3MulMV( iB, rnB ) );
					cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

					// Only compute the normal velocity terms if needed.
					if ( sampleVelocity )
					{
						b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
						b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
						float vn = b3Dot( normal, b3Sub( vrB, vrA ) );

						cp->relativeVelocity = vn;
						mp->normalVelocity = hitEvents ? vn : 0.0f;
					}
					else
					{
						cp->relativeVelocity = 0.0f;
						mp->normalVelocity = 0.0f;
					}

					// C0 friction center decay. Needed to prevent spinning top drift (GyroscopicPrecession sample).
					// Contacts with separation greater than twice the speculative distance only matter for CCD and
					// should not contribute to the friction center. They are not important for jitter reduction. Closer
					// points may begin to touch on and off, so the friction center needs to move smoothly.
					// Epsilon to avoid a branch below (or divide by zero). Small enough to get washed out normally.
					// 
					// Dirk suggested to weight the points by their normal impulse. This would use the total impulse from
					// the previous time step. I suspect this could jump around if the active point set jitters, leading
					// to a limit cycle.
					float weight = b3ClampFloat( 2.0f - s * invTau, B3_MIN_FRICTION_WEIGHT, 1.0f );
					centerA = b3MulAdd( centerA, weight, rA );
					centerB = b3MulAdd( centerB, weight, rB );
					totalFrictionWeight += weight;
				}

				float invWeight = 1.0f / totalFrictionWeight;
				centerA = b3MulSV( invWeight, centerA );
				centerB = b3MulSV( invWeight, centerB );
				constraint->centerA = centerA;
				constraint->centerB = centerB;

				for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
				{
					b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;
					cp->leverArm = b3Distance( cp->rA, centerA );
				}

				b3Vec3 rtA1 = b3Cross( centerA, tangent1 );
				b3Vec3 rtA2 = b3Cross( centerA, tangent2 );
				b3Vec3 rtB1 = b3Cross( centerB, tangent1 );
				b3Vec3 rtB2 = b3Cross( centerB, tangent2 );

				{
					b3Matrix2 k;
					k.cx.x = mA + mB + b3Dot( rtA1, b3MulMV( iA, rtA1 ) ) + b3Dot( rtB1, b3MulMV( iB, rtB1 ) );
					k.cy.y = mA + mB + b3Dot( rtA2, b3MulMV( iA, rtA2 ) ) + b3Dot( rtB2, b3MulMV( iB, rtB2 ) );
					k.cx.y = k.cy.x = b3Dot( rtA1, b3MulMV( iA, rtA2 ) ) + b3Dot( rtB1, b3MulMV( iB, rtB2 ) );

					constraint->tangentMass = b3Invert2( k );
					constraint->frictionImpulse.x = warmStartScale * b3Dot( manifold->frictionImpulse, tangent1 );
					constraint->frictionImpulse.y = warmStartScale * b3Dot( manifold->frictionImpulse, tangent2 );
				}

				{
					float k = b3Dot( normal, b3MulMV( b3AddMM( iA, iB ), normal ) );
					constraint->twistMass = k > 0.0f ? 1.0f / k : 0.0f;
					constraint->twistImpulse = warmStartScale * manifold->twistImpulse;
				}

				{
					constraint->rollingImpulse = b3MulSV( warmStartScale, manifold->rollingImpulse );
				}
			}
		}

		// Advance to next color
		colorIndex += 1;
	}

	if ( anyRestitution )
	{
		b3AtomicStoreInt( &context->anyRestitution, 1 );
	}

	b3TracyCZoneEnd( prepare_contact );
}

void b3WarmStartContacts_Mesh( b3SolverBlock block, b3StepContext* context )
{
	b3World* world = context->world;
	b3GraphColor* color = world->constraintGraph.colors + block.colorIndex;
	b3SolverSet* awakeSet = b3Array_Get( world->solverSets, b3_awakeSet );
	b3BodyState* states = awakeSet->bodyStates.data;
	b3ContactConstraint* constraints = color->contactConstraints;

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	b3BodyState dummyState = b3_identityBodyState;

	int startIndex = block.startIndex;
	int endIndex = startIndex + block.count;

	for ( int constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex )
	{
		const b3ContactConstraint* contactConstraint = constraints + constraintIndex;

		b3Vec3 linearImpulse = b3Vec3_zero;
		b3Vec3 angularImpulseA = b3Vec3_zero;
		b3Vec3 angularImpulseB = b3Vec3_zero;

		int manifoldCount = contactConstraint->manifoldCount;
		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3ManifoldConstraint* constraint = contactConstraint->constraints + manifoldIndex;

			// Normal impulses
			b3Vec3 normal = constraint->normal;
			float totalNormalImpulse = 0.0f;
			b3Vec3 momentA = b3Vec3_zero;
			b3Vec3 momentB = b3Vec3_zero;

			int pointCount = constraint->pointCount;
			for ( int j = 0; j < pointCount; ++j )
			{
				const b3ManifoldConstraintPoint* cp = constraint->points + j;

				// fixed anchors
				totalNormalImpulse += cp->normalImpulse;
				momentA = b3MulAdd( momentA, cp->normalImpulse, cp->rA );
				momentB = b3MulAdd( momentB, cp->normalImpulse, cp->rB );
			}

			// Central friction
			b3Vec3 frictionImpulse = b3MulSV( constraint->frictionImpulse.x, constraint->tangent1 );
			frictionImpulse = b3MulAdd( frictionImpulse, constraint->frictionImpulse.y, constraint->tangent2 );

			linearImpulse = b3Add( linearImpulse, b3MulAdd( frictionImpulse, totalNormalImpulse, normal ) );

			// Central twist friction
			b3Vec3 twistImpulse = b3MulSV( constraint->twistImpulse, normal );

			b3Vec3 angularA =
				b3Add( b3Add( b3Cross( momentA, normal ), b3Cross( constraint->centerA, frictionImpulse ) ), twistImpulse );
			b3Vec3 angularB =
				b3Add( b3Add( b3Cross( momentB, normal ), b3Cross( constraint->centerB, frictionImpulse ) ), twistImpulse );

			// Rolling resistance
			angularImpulseA = b3Add( angularImpulseA, b3Add( angularA, constraint->rollingImpulse ) );
			angularImpulseB = b3Add( angularImpulseB, b3Add( angularB, constraint->rollingImpulse ) );
		}

		int indexA = contactConstraint->indexA;
		int indexB = contactConstraint->indexB;

		b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : states + indexA;
		b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : states + indexB;

		if ( stateA->flags & b3_dynamicFlag )
		{
			stateA->linearVelocity = b3MulSub( stateA->linearVelocity, contactConstraint->invMassA, linearImpulse );
			stateA->angularVelocity = b3Sub( stateA->angularVelocity, b3MulMV( contactConstraint->invIA, angularImpulseA ) );
		}

		if ( stateB->flags & b3_dynamicFlag )
		{
			stateB->linearVelocity = b3MulAdd( stateB->linearVelocity, contactConstraint->invMassB, linearImpulse );
			stateB->angularVelocity = b3Add( stateB->angularVelocity, b3MulMV( contactConstraint->invIB, angularImpulseB ) );
		}
	}
}

// Merged normal and friction loops. This is much more stable for the Jenga stack.
// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
void b3PushContacts_Mesh( b3SolverBlock block, b3StepContext* context )
{
	b3World* world = context->world;
	b3GraphColor* color = world->constraintGraph.colors + block.colorIndex;
	b3ContactConstraint* contactConstraints = color->contactConstraints;
	b3BodyState* states = context->states;

	// This is a dummy state to represent a static body because static bodies have no solver body.
	b3BodyState dummyState = b3_identityBodyState;

	// The last block might not be full
	int startIndex = block.startIndex;
	int endIndex = startIndex + block.count;

	float inv_h = context->inv_h;
	const float contactSpeed = context->world->contactSpeed;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3ContactConstraint* contactConstraint = contactConstraints + i;
		int manifoldCount = contactConstraint->manifoldCount;

		int indexA = contactConstraint->indexA;
		int indexB = contactConstraint->indexB;

		float mA = contactConstraint->invMassA;
		b3Matrix3 iA = contactConstraint->invIA;
		float mB = contactConstraint->invMassB;
		b3Matrix3 iB = contactConstraint->invIB;

		b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : states + indexA;
		b3Vec3 vA = stateA->linearVelocity;
		b3Vec3 wA = stateA->angularVelocity;
		b3Quat dqA = stateA->deltaRotation;

		b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : states + indexB;
		b3Vec3 vB = stateB->linearVelocity;
		b3Vec3 wB = stateB->angularVelocity;
		b3Quat dqB = stateB->deltaRotation;

		b3Vec3 dp = b3Sub( stateB->deltaPosition, stateA->deltaPosition );
		b3Softness softness = contactConstraint->softness;

		for ( int j = 0; j < manifoldCount; ++j )
		{
			b3ManifoldConstraint* constraint = contactConstraint->constraints + j;

			int pointCount = constraint->pointCount;
			b3Vec3 normal = constraint->normal;

			for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
			{
				b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;

				// Fixed anchor points for applying impulses
				b3Vec3 rA = cp->rA;
				b3Vec3 rB = cp->rB;

				// compute current separation
				// this is subject to round-off error if the anchor is far from the body center of mass
				b3Vec3 ds = b3Add( dp, b3Sub( b3RotateVector( dqB, rB ), b3RotateVector( dqA, rA ) ) );
				float s = b3Dot( ds, normal ) + cp->baseSeparation;

				float velocityBias;
				float massScale;
				float impulseScale;
				if ( s > 0.0f )
				{
					// speculative bias is positive
					velocityBias = s * inv_h;
					massScale = 1.0f;
					impulseScale = 0.0f;
				}
				else
				{
					// overlap bias is negative
					velocityBias = b3MaxFloat( softness.massScale * softness.biasRate * s, -contactSpeed );
					massScale = softness.massScale;
					impulseScale = softness.impulseScale;
				}

				// relative normal velocity at contact
				b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
				b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
				float vn = b3Dot( b3Sub( vrB, vrA ), normal );

				// incremental normal impulse
				float deltaImpulse = -cp->normalMass * ( massScale * vn + velocityBias ) - impulseScale * cp->normalImpulse;

				// clamp the accumulated impulse
				float newImpulse = b3MaxFloat( cp->normalImpulse + deltaImpulse, 0.0f );
				deltaImpulse = newImpulse - cp->normalImpulse;
				cp->normalImpulse = newImpulse;

				// apply normal impulse
				b3Vec3 P = b3MulSV( deltaImpulse, normal );
				vA = b3MulSub( vA, mA, P );
				wA = b3Sub( wA, b3MulMV( iA, b3Cross( rA, P ) ) );

				vB = b3MulAdd( vB, mB, P );
				wB = b3Add( wB, b3MulMV( iB, b3Cross( rB, P ) ) );
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
}

// Solve contacts: normal, friction and rolling resistance.
void b3SolveContacts_Mesh( b3SolverBlock block, b3StepContext* context )
{
	b3World* world = context->world;
	b3GraphColor* color = world->constraintGraph.colors + block.colorIndex;
	b3ContactConstraint* contactConstraints = color->contactConstraints;
	b3BodyState* states = context->states;

	// This is a dummy state to represent a static body because static bodies have no solver body.
	b3BodyState dummyState = b3_identityBodyState;

	// The last block might not be full
	int startIndex = block.startIndex;
	int endIndex = startIndex + block.count;

	float inv_h = context->inv_h;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3ContactConstraint* contactConstraint = contactConstraints + i;
		int manifoldCount = contactConstraint->manifoldCount;

		int indexA = contactConstraint->indexA;
		int indexB = contactConstraint->indexB;

		float mA = contactConstraint->invMassA;
		b3Matrix3 iA = contactConstraint->invIA;
		float mB = contactConstraint->invMassB;
		b3Matrix3 iB = contactConstraint->invIB;

		b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : states + indexA;
		b3Vec3 vA = stateA->linearVelocity;
		b3Vec3 wA = stateA->angularVelocity;
		b3Quat dqA = stateA->deltaRotation;

		b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : states + indexB;
		b3Vec3 vB = stateB->linearVelocity;
		b3Vec3 wB = stateB->angularVelocity;
		b3Quat dqB = stateB->deltaRotation;

		b3Vec3 dp = b3Sub( stateB->deltaPosition, stateA->deltaPosition );
		float friction = contactConstraint->friction;
		float rollingResistance = contactConstraint->rollingResistance;

		for ( int j = 0; j < manifoldCount; ++j )
		{
			b3ManifoldConstraint* constraint = contactConstraint->constraints + j;

			int pointCount = constraint->pointCount;
			b3Vec3 normal = constraint->normal;

			float totalNormalImpulse = 0.0f;
			float totalTwistLimit = 0.0f;

			for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
			{
				b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;

				// Fixed anchor points for applying impulses
				b3Vec3 rA = cp->rA;
				b3Vec3 rB = cp->rB;

				// compute current separation
				// this is subject to round-off error if the anchor is far from the body center of mass
				b3Vec3 ds = b3Add( dp, b3Sub( b3RotateVector( dqB, rB ), b3RotateVector( dqA, rA ) ) );
				float s = b3Dot( ds, normal ) + cp->baseSeparation;

				// speculative bias, zero when overlapped
				float velocityBias = s > 0.0f ? s * inv_h : 0.0f;

				// relative normal velocity at contact
				b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
				b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
				float vn = b3Dot( b3Sub( vrB, vrA ), normal );

				// incremental normal impulse
				float deltaImpulse = -cp->normalMass * ( vn + velocityBias );

				// clamp the accumulated impulse
				float newImpulse = b3MaxFloat( cp->normalImpulse + deltaImpulse, 0.0f );
				deltaImpulse = newImpulse - cp->normalImpulse;
				cp->normalImpulse = newImpulse;
				cp->totalNormalImpulse += newImpulse;
				totalNormalImpulse += newImpulse;
				totalTwistLimit += cp->leverArm * newImpulse;

				// apply normal impulse
				b3Vec3 P = b3MulSV( deltaImpulse, normal );
				vA = b3MulSub( vA, mA, P );
				wA = b3Sub( wA, b3MulMV( iA, b3Cross( rA, P ) ) );

				vB = b3MulAdd( vB, mB, P );
				wB = b3Add( wB, b3MulMV( iB, b3Cross( rB, P ) ) );
			}

			// Central twist friction
			{
				float twistSpeed = b3Dot( constraint->normal, b3Sub( wB, wA ) );
				float maxImpulse = friction * totalTwistLimit;
				float deltaImpulse = -constraint->twistMass * twistSpeed;
				float oldImpulse = constraint->twistImpulse;
				constraint->twistImpulse = b3ClampFloat( oldImpulse + deltaImpulse, -maxImpulse, maxImpulse );
				deltaImpulse = constraint->twistImpulse - oldImpulse;

				wA = b3Sub( wA, b3MulMV( iA, b3MulSV( deltaImpulse, constraint->normal ) ) );
				wB = b3Add( wB, b3MulMV( iB, b3MulSV( deltaImpulse, constraint->normal ) ) );
			}

			// Rolling resistance
			if ( rollingResistance > 0.0f )
			{
				b3Vec3 deltaImpulse = b3Neg( b3MulMV( contactConstraint->rollingMass, b3Sub( wB, wA ) ) );
				b3Vec3 oldImpulse = constraint->rollingImpulse;
				constraint->rollingImpulse = b3Add( oldImpulse, deltaImpulse );

				float maxImpulse = rollingResistance * totalNormalImpulse;
				float magSqr = b3Dot( constraint->rollingImpulse, constraint->rollingImpulse );
				if ( magSqr > maxImpulse * maxImpulse + FLT_EPSILON )
				{
					constraint->rollingImpulse = b3MulSV( maxImpulse / sqrtf( magSqr ), constraint->rollingImpulse );
				}

				deltaImpulse = b3Sub( constraint->rollingImpulse, oldImpulse );

				wA = b3Sub( wA, b3MulMV( iA, deltaImpulse ) );
				wB = b3Add( wB, b3MulMV( iB, deltaImpulse ) );
			}

			// Central friction
			{
				b3Vec3 tangent1 = constraint->tangent1;
				b3Vec3 tangent2 = constraint->tangent2;

				// Fixed anchor points for applying impulses
				b3Vec3 rA = constraint->centerA;
				b3Vec3 rB = constraint->centerB;

				// Relative tangent velocity at contact
				b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
				b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
				b3Vec3 vr = b3Sub( vrB, vrA );
				b3Vec2 vt = {
					b3Dot( vr, tangent1 ) - constraint->tangentVelocity1,
					b3Dot( vr, tangent2 ) - constraint->tangentVelocity2,
				};

				// Incremental tangent impulse
				b3Vec2 tm = b3MulMV2( constraint->tangentMass, vt );
				b3Vec2 deltaImpulse = { -tm.x, -tm.y };
				b3Vec2 newImpulse = {
					constraint->frictionImpulse.x + deltaImpulse.x,
					constraint->frictionImpulse.y + deltaImpulse.y,
				};

				float maxImpulse = friction * totalNormalImpulse;

				// Clamp the accumulated impulse
				float lengthSquared = b3Dot2( newImpulse, newImpulse );
				if ( lengthSquared > maxImpulse * maxImpulse )
				{
					float scale = maxImpulse / sqrtf( lengthSquared );
					newImpulse.x *= scale;
					newImpulse.y *= scale;
				}
				deltaImpulse = b3Sub2( newImpulse, constraint->frictionImpulse );
				constraint->frictionImpulse = newImpulse;

				// Apply delta impulse
				b3Vec3 P = b3Blend2( deltaImpulse.x, tangent1, deltaImpulse.y, tangent2 );
				vA = b3MulSub( vA, mA, P );
				wA = b3Sub( wA, b3MulMV( iA, b3Cross( rA, P ) ) );
				vB = b3MulAdd( vB, mB, P );
				wB = b3Add( wB, b3MulMV( iB, b3Cross( rB, P ) ) );
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
}

void b3ApplyRestitution_Mesh( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( restitution_mesh, "Restitution Mesh", b3_colorViolet, true );

	b3World* world = context->world;
	b3GraphColor* color = world->constraintGraph.colors + block.colorIndex;
	b3ContactConstraint* contactConstraints = color->contactConstraints;
	b3BodyState* states = context->states;

	float threshold = world->restitutionThreshold;
	float inv_h = context->inv_h;
	bool propagate = world->enableRestitutionPropagation;

	b3BodyState dummyState = b3_identityBodyState;

	int startIndex = block.startIndex;
	int endIndex = startIndex + block.count;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3ContactConstraint* contactConstraint = contactConstraints + i;
		float restitution = contactConstraint->restitution;
		if ( propagate == false && restitution == 0.0f )
		{
			continue;
		}

		int manifoldCount = contactConstraint->manifoldCount;

		int indexA = contactConstraint->indexA;
		int indexB = contactConstraint->indexB;

		float mA = contactConstraint->invMassA;
		b3Matrix3 iA = contactConstraint->invIA;
		float mB = contactConstraint->invMassB;
		b3Matrix3 iB = contactConstraint->invIB;

		b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : states + indexA;
		b3Vec3 vA = stateA->linearVelocity;
		b3Vec3 wA = stateA->angularVelocity;
		b3Quat dqA = stateA->deltaRotation;

		b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : states + indexB;
		b3Vec3 vB = stateB->linearVelocity;
		b3Vec3 wB = stateB->angularVelocity;
		b3Quat dqB = stateB->deltaRotation;

		b3Vec3 dp = b3Sub( stateB->deltaPosition, stateA->deltaPosition );

		for ( int j = 0; j < manifoldCount; ++j )
		{
			b3ManifoldConstraint* constraint = contactConstraint->constraints + j;

			int pointCount = constraint->pointCount;
			b3Vec3 normal = constraint->normal;

			for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
			{
				b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;

				b3Vec3 rA = cp->rA;
				b3Vec3 rB = cp->rB;

				// The total normal impulse is 0 for speculative points.
				float compressionImpulse = cp->totalNormalImpulse - cp->restitutionImpulse;
				bool armed = restitution > 0.0f && cp->relativeVelocity < -threshold && compressionImpulse > 0.0f;

				float velocityBias;
				if ( armed )
				{
					velocityBias = restitution * cp->relativeVelocity;
				}
				else
				{
					b3Vec3 ds = b3Add( dp, b3Sub( b3RotateVector( dqB, rB ), b3RotateVector( dqA, rA ) ) );
					float s = b3Dot( ds, normal ) + cp->baseSeparation;

					velocityBias = s > 0.0f ? s * inv_h : 0.0f;
				}

				b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
				b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
				float vn = b3Dot( b3Sub( vrB, vrA ), normal );

				float impulse = -cp->normalMass * ( vn + velocityBias );

				float newImpulse = b3MaxFloat( cp->normalImpulse + impulse, 0.0f );
				impulse = newImpulse - cp->normalImpulse;

				float approachImpulse = b3MinFloat( b3MaxFloat( -cp->normalMass * vn, 0.0f ), b3MaxFloat( impulse, 0.0f ) );

				if ( armed )
				{
					// Poisson kinetic restitution guarantees no energy gain.
					float allowance = restitution * ( compressionImpulse + approachImpulse ) - cp->restitutionImpulse;
					impulse = b3MinFloat( impulse, approachImpulse + b3MaxFloat( allowance, 0.0f ) );
				}

				cp->normalImpulse += impulse;
				cp->restitutionImpulse += impulse - approachImpulse;
				cp->totalNormalImpulse += impulse;

				b3Vec3 P = b3MulSV( impulse, normal );
				vA = b3MulSub( vA, mA, P );
				wA = b3Sub( wA, b3MulMV( iA, b3Cross( rA, P ) ) );

				vB = b3MulAdd( vB, mB, P );
				wB = b3Add( wB, b3MulMV( iB, b3Cross( rB, P ) ) );
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

	b3TracyCZoneEnd( restitution_mesh );
}

// Don't need to use spans for colors for this because the constraint to contact association
// is already linked by pointer.
void b3StoreImpulses_Mesh( b3SolverBlock block, b3StepContext* context, int workerIndex )
{
	b3World* world = context->world;

	B3_ASSERT( block.blockType == b3_overflowBlock );
	b3GraphColor* overflow = world->constraintGraph.colors + B3_OVERFLOW_INDEX;
	b3ContactPrepareSpan* spans = context->overflowSpans;
	b3ContactConstraint* base = overflow->contactConstraints;

	b3TaskContext* taskContext = world->taskContexts.data + workerIndex;
	b3BitSet* hitEventBitSet = &taskContext->hitEventBitSet;
	bool hasHitEvents = taskContext->hasHitEvents;
	float negHitThreshold = -world->hitEventThreshold;

	int index = block.startIndex;
	int endIndex = block.startIndex + block.count;

	// Find color for start index. Linear search but fast.
	int colorIndex = 0;
	while ( spans[colorIndex + 1].start <= index )
	{
		colorIndex += 1;
	}

	// Loop over block
	while ( index < endIndex )
	{
		int colorStart = spans[colorIndex].start;
		int colorEndIndex = b3MinInt( spans[colorIndex + 1].start, endIndex );

		// Loop over color
		for ( ; index < colorEndIndex; ++index )
		{
			b3ContactConstraint* contactConstraint = base + index;

			int localIndex = index - colorStart;
			B3_UNUSED( localIndex );
			B3_ASSERT( 0 <= localIndex && localIndex < spans[colorIndex].count );

			// Having this contact pointer simplifies impulse storage
			b3Contact* contact = contactConstraint->contact;
			B3_ASSERT( contact != NULL );

			// Catches the wrong-(base, spans) pairing: the contact pointer stashed by
			// b3PrepareContacts_Mesh at this flat slot must reference the same contact
			// the span at this slot describes.
			B3_VALIDATE( contact->contactId == spans[colorIndex].contacts[localIndex].contactId );

			int manifoldCount = contactConstraint->manifoldCount;
			B3_ASSERT( manifoldCount == contact->manifoldCount );

			bool checkHitEvents = ( contact->flags & b3_simEnableHitEvent ) != 0;
			bool flagged = false;

			for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
			{
				b3Manifold* manifold = contact->manifolds + manifoldIndex;
				b3ManifoldConstraint* constraint = contactConstraint->constraints + manifoldIndex;
				manifold->twistImpulse = constraint->twistImpulse;
				manifold->frictionImpulse = b3Blend2( constraint->frictionImpulse.x, constraint->tangent1,
													  constraint->frictionImpulse.y, constraint->tangent2 );
				manifold->rollingImpulse = constraint->rollingImpulse;

				int count = constraint->pointCount;
				B3_ASSERT( count == manifold->pointCount );
				for ( int pointIndex = 0; pointIndex < count; ++pointIndex )
				{
					b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;
					b3ManifoldPoint* mp = manifold->points + pointIndex;
					mp->normalImpulse = cp->normalImpulse;
					mp->totalNormalImpulse = cp->totalNormalImpulse;

					if ( checkHitEvents && flagged == false && mp->normalVelocity < negHitThreshold &&
						 mp->totalNormalImpulse > 0.0f )
					{
						b3SetBit( hitEventBitSet, contact->contactId );
						hasHitEvents = true;
						flagged = true;
					}
				}
			}
		}

		// Advance to next color
		colorIndex += 1;
	}

	taskContext->hasHitEvents = hasHitEvents;
}

void b3PrepareContacts_Overflow( b3StepContext* context )
{
	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;

	B3_ASSERT( color->contacts.count <= UINT16_MAX );
	uint16_t count = (uint16_t)color->contacts.count;
	if ( count == 0 )
	{
		return;
	}

	b3SolverBlock block = {
		.startIndex = 0,
		.count = count,
		.blockType = b3_overflowBlock,
		.colorIndex = B3_OVERFLOW_INDEX,
	};

	b3PrepareContacts_Mesh( block, context );
}

void b3WarmStartContacts_Overflow( b3StepContext* context )
{
	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;

	uint16_t count = (uint16_t)color->contacts.count;
	if ( count == 0 )
	{
		return;
	}

	b3SolverBlock block = {
		.startIndex = 0,
		.count = count,
		.blockType = b3_overflowBlock,
		.colorIndex = B3_OVERFLOW_INDEX,
	};

	b3WarmStartContacts_Mesh( block, context );
}

void b3PushContacts_Overflow( b3StepContext* context )
{
	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;

	uint16_t count = (uint16_t)color->contacts.count;
	if ( count == 0 )
	{
		return;
	}

	b3SolverBlock block = {
		.startIndex = 0,
		.count = count,
		.blockType = b3_overflowBlock,
		.colorIndex = B3_OVERFLOW_INDEX,
	};

	b3PushContacts_Mesh( block, context );
}

void b3SolveContacts_Overflow( b3StepContext* context )
{
	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;

	uint16_t count = (uint16_t)color->contacts.count;
	if ( count == 0 )
	{
		return;
	}

	b3SolverBlock block = {
		.startIndex = 0,
		.count = count,
		.blockType = b3_overflowBlock,
		.colorIndex = B3_OVERFLOW_INDEX,
	};

	b3SolveContacts_Mesh( block, context );
}

void b3ApplyRestitution_Overflow( b3StepContext* context )
{
	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;

	uint16_t count = (uint16_t)color->contacts.count;
	if ( count == 0 )
	{
		return;
	}

	b3SolverBlock block = {
		.startIndex = 0,
		.count = count,
		.blockType = b3_overflowBlock,
		.colorIndex = B3_OVERFLOW_INDEX,
	};

	b3ApplyRestitution_Mesh( block, context );
}

void b3StoreImpulses_Overflow( b3StepContext* context )
{
	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;

	uint16_t count = (uint16_t)color->contacts.count;
	if ( count == 0 )
	{
		return;
	}

	b3SolverBlock block = {
		.startIndex = 0,
		.count = count,
		.blockType = b3_overflowBlock,
		.colorIndex = B3_OVERFLOW_INDEX,
	};

	b3StoreImpulses_Mesh( block, context, 0 );
}

void b3PrepareContacts_Convex( b3SolverBlock block, b3StepContext* context )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b3PrepareContacts_ConvexW8( block, context );
		return;
	}
#endif

	b3PrepareContacts_ConvexW4( block, context );
}

void b3WarmStartContacts_Convex( b3SolverBlock block, b3StepContext* context )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b3WarmStartContacts_ConvexW8( block, context );
		return;
	}
#endif

	b3WarmStartContacts_ConvexW4( block, context );
}

void b3PushContacts_Convex( b3SolverBlock block, b3StepContext* context )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b3PushContacts_ConvexW8( block, context );
		return;
	}
#endif

	b3PushContacts_ConvexW4( block, context );
}

void b3SolveContacts_Convex( b3SolverBlock block, b3StepContext* context )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b3SolveContacts_ConvexW8( block, context );
		return;
	}
#endif

	b3SolveContacts_ConvexW4( block, context );
}

void b3ApplyRestitution_Convex( b3SolverBlock block, b3StepContext* context )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b3ApplyRestitution_ConvexW8( block, context );
		return;
	}
#endif

	b3ApplyRestitution_ConvexW4( block, context );
}

void b3StoreImpulses_Convex( b3SolverBlock block, b3StepContext* context, int workerIndex )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b3StoreImpulses_ConvexW8( block, context, workerIndex );
		return;
	}
#endif

	b3StoreImpulses_ConvexW4( block, context, workerIndex );
}

int b3GetWideContactConstraintByteCount( int simdWidth )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( simdWidth == 8 )
	{
		return b3GetWideContactConstraintByteCountW8();
	}
#else
	B3_UNUSED( simdWidth );
#endif

	return b3GetWideContactConstraintByteCountW4();
}

void b3PrepareContacts_MeshWide( b3SolverBlock block, b3StepContext* context )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b3PrepareContacts_MeshWideW8( block, context );
		return;
	}
#endif

	b3PrepareContacts_MeshWideW4( block, context );
}

void b3WarmStartContacts_MeshWide( b3SolverBlock block, b3StepContext* context )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b3WarmStartContacts_MeshWideW8( block, context );
		return;
	}
#endif

	b3WarmStartContacts_MeshWideW4( block, context );
}

void b3PushContacts_MeshWide( b3SolverBlock block, b3StepContext* context )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b3PushContacts_MeshWideW8( block, context );
		return;
	}
#endif

	b3PushContacts_MeshWideW4( block, context );
}

void b3SolveContacts_MeshWide( b3SolverBlock block, b3StepContext* context )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b3SolveContacts_MeshWideW8( block, context );
		return;
	}
#endif

	b3SolveContacts_MeshWideW4( block, context );
}

void b3ApplyRestitution_MeshWide( b3SolverBlock block, b3StepContext* context )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b3ApplyRestitution_MeshWideW8( block, context );
		return;
	}
#endif

	b3ApplyRestitution_MeshWideW4( block, context );
}

void b3StoreImpulses_MeshWide( b3SolverBlock block, b3StepContext* context, int workerIndex )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b3StoreImpulses_MeshWideW8( block, context, workerIndex );
		return;
	}
#endif

	b3StoreImpulses_MeshWideW4( block, context, workerIndex );
}

int b3GetWideMeshConstraintByteCount( int simdWidth )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( simdWidth == 8 )
	{
		return b3GetWideMeshConstraintByteCountW8();
	}
#else
	B3_UNUSED( simdWidth );
#endif

	return b3GetWideMeshConstraintByteCountW4();
}

int b3GetWideMeshManifoldByteCount( int simdWidth )
{
#if defined( B3_SIMD_HAS_WIDTH_8 )
	if ( simdWidth == 8 )
	{
		return b3GetWideMeshManifoldByteCountW8();
	}
#else
	B3_UNUSED( simdWidth );
#endif

	return b3GetWideMeshManifoldByteCountW4();
}
