// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/math_functions.h"

// One sub-test per joint type. Each creates the joint, exercises the shared
// b3Joint_* API plus every type-specific accessor, then steps to make sure the
// joint solves without tripping a validation assert.

typedef struct JointFixture
{
	b3WorldId worldId;
	b3BodyId groundId;
	b3BodyId bodyId;
} JointFixture;

// Static ground plus a dynamic box, anchored so a point-coincident joint starts
// satisfied. Gravity is off so the body stays put across the handful of steps
// each sub-test takes.
static JointFixture CreateJointFixture( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;

	JointFixture f;
	f.worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	f.groundId = b3CreateBody( f.worldId, &groundDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 4.0f, 0.0f };
	f.bodyId = b3CreateBody( f.worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3BoxHull box = b3MakeCubeHull( 0.5f );
	b3CreateHullShape( f.bodyId, &shapeDef, &box.base );

	return f;
}

// Place the joint anchor at the dynamic body so both local frames map to the
// same world point.
static void SetCommonFrames( b3JointDef* base, const JointFixture* f )
{
	base->bodyIdA = f->groundId;
	base->bodyIdB = f->bodyId;
	base->localFrameA.p = (b3Vec3){ 0.0f, 4.0f, 0.0f };
	base->localFrameB.p = (b3Vec3){ 0.0f, 0.0f, 0.0f };
}

// Step a few times, destroy the joint, then the world. Destroying the joint
// explicitly also covers b3DestroyJoint and stale-handle detection.
static int FinishJoint( b3JointId jointId, b3WorldId worldId )
{
	for ( int i = 0; i < 8; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3DestroyJoint( jointId, true );
	ENSURE( b3Joint_IsValid( jointId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

// Exercise the API shared by every joint type. Frames are saved and restored so
// the caller's setup survives.
static int ExerciseJointBase( b3JointId jointId, b3WorldId worldId, b3BodyId bodyIdA, b3BodyId bodyIdB, b3JointType expectedType )
{
	ENSURE( b3Joint_IsValid( jointId ) );
	ENSURE( b3Joint_GetType( jointId ) == expectedType );
	ENSURE( B3_ID_EQUALS( b3Joint_GetBodyA( jointId ), bodyIdA ) );
	ENSURE( B3_ID_EQUALS( b3Joint_GetBodyB( jointId ), bodyIdB ) );

	// B3_ID_EQUALS does not work for world ids
	b3WorldId gotWorld = b3Joint_GetWorld( jointId );
	ENSURE( gotWorld.index1 == worldId.index1 && gotWorld.generation == worldId.generation );

	b3Transform originalA = b3Joint_GetLocalFrameA( jointId );
	b3Transform originalB = b3Joint_GetLocalFrameB( jointId );

	b3Transform frameA = { { 0.1f, 0.2f, 0.3f }, b3Quat_identity };
	b3Joint_SetLocalFrameA( jointId, frameA );
	b3Transform gotA = b3Joint_GetLocalFrameA( jointId );
	ENSURE( gotA.p.x == frameA.p.x && gotA.p.y == frameA.p.y && gotA.p.z == frameA.p.z );

	b3Transform frameB = { { -0.4f, 0.5f, -0.6f }, b3Quat_identity };
	b3Joint_SetLocalFrameB( jointId, frameB );
	b3Transform gotB = b3Joint_GetLocalFrameB( jointId );
	ENSURE( gotB.p.x == frameB.p.x && gotB.p.y == frameB.p.y && gotB.p.z == frameB.p.z );

	b3Joint_SetCollideConnected( jointId, true );
	ENSURE( b3Joint_GetCollideConnected( jointId ) == true );
	b3Joint_SetCollideConnected( jointId, false );
	ENSURE( b3Joint_GetCollideConnected( jointId ) == false );

	int userData = 0;
	b3Joint_SetUserData( jointId, &userData );
	ENSURE( b3Joint_GetUserData( jointId ) == &userData );

	b3Joint_SetConstraintTuning( jointId, 90.0f, 3.0f );
	float hertz = 0.0f;
	float dampingRatio = 0.0f;
	b3Joint_GetConstraintTuning( jointId, &hertz, &dampingRatio );
	ENSURE( hertz == 90.0f );
	ENSURE( dampingRatio == 3.0f );

	b3Joint_SetForceThreshold( jointId, 100.0f );
	ENSURE( b3Joint_GetForceThreshold( jointId ) == 100.0f );

	b3Joint_SetTorqueThreshold( jointId, 200.0f );
	ENSURE( b3Joint_GetTorqueThreshold( jointId ) == 200.0f );

	b3Joint_WakeBodies( jointId );

	// No stable value to assert before the first step, call for coverage
	b3Vec3 force = b3Joint_GetConstraintForce( jointId );
	b3Vec3 torque = b3Joint_GetConstraintTorque( jointId );
	float linearSeparation = b3Joint_GetLinearSeparation( jointId );
	MAYBE_UNUSED( force );
	MAYBE_UNUSED( torque );
	MAYBE_UNUSED( linearSeparation );

	// Wheel joint angular separation is an unimplemented todo in joint.c that
	// asserts. Every other type computes it.
	if ( expectedType != b3_wheelJoint )
	{
		float angularSeparation = b3Joint_GetAngularSeparation( jointId );
		MAYBE_UNUSED( angularSeparation );
	}

	b3Joint_SetLocalFrameA( jointId, originalA );
	b3Joint_SetLocalFrameB( jointId, originalB );

	return 0;
}

static int TestParallelJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3ParallelJointDef def = b3DefaultParallelJointDef();
	SetCommonFrames( &def.base, &f );
	def.hertz = 2.0f;
	def.dampingRatio = 0.5f;
	def.maxTorque = 100.0f;
	b3JointId jointId = b3CreateParallelJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_parallelJoint ) != 0 )
	{
		return 1;
	}

	b3ParallelJoint_SetSpringHertz( jointId, 5.0f );
	ENSURE( b3ParallelJoint_GetSpringHertz( jointId ) == 5.0f );

	b3ParallelJoint_SetSpringDampingRatio( jointId, 0.7f );
	ENSURE( b3ParallelJoint_GetSpringDampingRatio( jointId ) == 0.7f );

	b3ParallelJoint_SetMaxTorque( jointId, 250.0f );
	ENSURE( b3ParallelJoint_GetMaxTorque( jointId ) == 250.0f );

	return FinishJoint( jointId, f.worldId );
}

static int TestDistanceJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3DistanceJointDef def = b3DefaultDistanceJointDef();
	SetCommonFrames( &def.base, &f );
	def.length = 2.0f;
	b3JointId jointId = b3CreateDistanceJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_distanceJoint ) != 0 )
	{
		return 1;
	}

	b3DistanceJoint_SetLength( jointId, 3.0f );
	ENSURE( b3DistanceJoint_GetLength( jointId ) == 3.0f );

	b3DistanceJoint_EnableSpring( jointId, true );
	ENSURE( b3DistanceJoint_IsSpringEnabled( jointId ) == true );

	b3DistanceJoint_SetSpringForceRange( jointId, -50.0f, 75.0f );
	float lowerForce = 0.0f;
	float upperForce = 0.0f;
	b3DistanceJoint_GetSpringForceRange( jointId, &lowerForce, &upperForce );
	ENSURE( lowerForce == -50.0f && upperForce == 75.0f );

	b3DistanceJoint_SetSpringHertz( jointId, 4.0f );
	ENSURE( b3DistanceJoint_GetSpringHertz( jointId ) == 4.0f );

	b3DistanceJoint_SetSpringDampingRatio( jointId, 0.6f );
	ENSURE( b3DistanceJoint_GetSpringDampingRatio( jointId ) == 0.6f );

	b3DistanceJoint_EnableLimit( jointId, true );
	ENSURE( b3DistanceJoint_IsLimitEnabled( jointId ) == true );

	b3DistanceJoint_SetLengthRange( jointId, 1.0f, 5.0f );
	ENSURE( b3DistanceJoint_GetMinLength( jointId ) == 1.0f );
	ENSURE( b3DistanceJoint_GetMaxLength( jointId ) == 5.0f );

	float currentLength = b3DistanceJoint_GetCurrentLength( jointId );
	MAYBE_UNUSED( currentLength );

	b3DistanceJoint_EnableMotor( jointId, true );
	ENSURE( b3DistanceJoint_IsMotorEnabled( jointId ) == true );

	b3DistanceJoint_SetMotorSpeed( jointId, 1.5f );
	ENSURE( b3DistanceJoint_GetMotorSpeed( jointId ) == 1.5f );

	b3DistanceJoint_SetMaxMotorForce( jointId, 25.0f );
	ENSURE( b3DistanceJoint_GetMaxMotorForce( jointId ) == 25.0f );

	float motorForce = b3DistanceJoint_GetMotorForce( jointId );
	MAYBE_UNUSED( motorForce );

	return FinishJoint( jointId, f.worldId );
}

static int TestFilterJoint( void )
{
	JointFixture f = CreateJointFixture();

	// The filter joint has no type-specific API. It only disables collision and
	// keeps both bodies in the same island.
	b3FilterJointDef def = b3DefaultFilterJointDef();
	def.base.bodyIdA = f.groundId;
	def.base.bodyIdB = f.bodyId;
	b3JointId jointId = b3CreateFilterJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_filterJoint ) != 0 )
	{
		return 1;
	}

	return FinishJoint( jointId, f.worldId );
}

static int TestMotorJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3MotorJointDef def = b3DefaultMotorJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateMotorJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_motorJoint ) != 0 )
	{
		return 1;
	}

	b3Vec3 linearVelocity = { 1.0f, 2.0f, 3.0f };
	b3MotorJoint_SetLinearVelocity( jointId, linearVelocity );
	b3Vec3 gotLinear = b3MotorJoint_GetLinearVelocity( jointId );
	ENSURE( gotLinear.x == 1.0f && gotLinear.y == 2.0f && gotLinear.z == 3.0f );

	b3Vec3 angularVelocity = { 0.1f, 0.2f, 0.3f };
	b3MotorJoint_SetAngularVelocity( jointId, angularVelocity );
	b3Vec3 gotAngular = b3MotorJoint_GetAngularVelocity( jointId );
	ENSURE( gotAngular.x == 0.1f && gotAngular.y == 0.2f && gotAngular.z == 0.3f );

	b3MotorJoint_SetMaxVelocityForce( jointId, 500.0f );
	ENSURE( b3MotorJoint_GetMaxVelocityForce( jointId ) == 500.0f );

	b3MotorJoint_SetMaxVelocityTorque( jointId, 600.0f );
	ENSURE( b3MotorJoint_GetMaxVelocityTorque( jointId ) == 600.0f );

	b3MotorJoint_SetLinearHertz( jointId, 3.0f );
	ENSURE( b3MotorJoint_GetLinearHertz( jointId ) == 3.0f );

	b3MotorJoint_SetLinearDampingRatio( jointId, 0.8f );
	ENSURE( b3MotorJoint_GetLinearDampingRatio( jointId ) == 0.8f );

	b3MotorJoint_SetAngularHertz( jointId, 4.0f );
	ENSURE( b3MotorJoint_GetAngularHertz( jointId ) == 4.0f );

	b3MotorJoint_SetAngularDampingRatio( jointId, 0.9f );
	ENSURE( b3MotorJoint_GetAngularDampingRatio( jointId ) == 0.9f );

	b3MotorJoint_SetMaxSpringForce( jointId, 700.0f );
	ENSURE( b3MotorJoint_GetMaxSpringForce( jointId ) == 700.0f );

	b3MotorJoint_SetMaxSpringTorque( jointId, 800.0f );
	ENSURE( b3MotorJoint_GetMaxSpringTorque( jointId ) == 800.0f );

	return FinishJoint( jointId, f.worldId );
}

static int TestPrismaticJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreatePrismaticJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_prismaticJoint ) != 0 )
	{
		return 1;
	}

	b3PrismaticJoint_EnableSpring( jointId, true );
	ENSURE( b3PrismaticJoint_IsSpringEnabled( jointId ) == true );

	b3PrismaticJoint_SetSpringHertz( jointId, 5.0f );
	ENSURE( b3PrismaticJoint_GetSpringHertz( jointId ) == 5.0f );

	b3PrismaticJoint_SetSpringDampingRatio( jointId, 0.5f );
	ENSURE( b3PrismaticJoint_GetSpringDampingRatio( jointId ) == 0.5f );

	b3PrismaticJoint_SetTargetTranslation( jointId, 1.0f );
	ENSURE( b3PrismaticJoint_GetTargetTranslation( jointId ) == 1.0f );

	b3PrismaticJoint_EnableLimit( jointId, true );
	ENSURE( b3PrismaticJoint_IsLimitEnabled( jointId ) == true );

	b3PrismaticJoint_SetLimits( jointId, -2.0f, 2.0f );
	ENSURE( b3PrismaticJoint_GetLowerLimit( jointId ) == -2.0f );
	ENSURE( b3PrismaticJoint_GetUpperLimit( jointId ) == 2.0f );

	b3PrismaticJoint_EnableMotor( jointId, true );
	ENSURE( b3PrismaticJoint_IsMotorEnabled( jointId ) == true );

	b3PrismaticJoint_SetMotorSpeed( jointId, 1.5f );
	ENSURE( b3PrismaticJoint_GetMotorSpeed( jointId ) == 1.5f );

	b3PrismaticJoint_SetMaxMotorForce( jointId, 30.0f );
	ENSURE( b3PrismaticJoint_GetMaxMotorForce( jointId ) == 30.0f );

	float motorForce = b3PrismaticJoint_GetMotorForce( jointId );
	float translation = b3PrismaticJoint_GetTranslation( jointId );
	float speed = b3PrismaticJoint_GetSpeed( jointId );
	MAYBE_UNUSED( motorForce );
	MAYBE_UNUSED( translation );
	MAYBE_UNUSED( speed );

	return FinishJoint( jointId, f.worldId );
}

static int TestRevoluteJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateRevoluteJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_revoluteJoint ) != 0 )
	{
		return 1;
	}

	b3RevoluteJoint_EnableSpring( jointId, true );
	ENSURE( b3RevoluteJoint_IsSpringEnabled( jointId ) == true );

	b3RevoluteJoint_SetSpringHertz( jointId, 5.0f );
	ENSURE( b3RevoluteJoint_GetSpringHertz( jointId ) == 5.0f );

	b3RevoluteJoint_SetSpringDampingRatio( jointId, 0.5f );
	ENSURE( b3RevoluteJoint_GetSpringDampingRatio( jointId ) == 0.5f );

	b3RevoluteJoint_SetTargetAngle( jointId, 0.5f );
	ENSURE( b3RevoluteJoint_GetTargetAngle( jointId ) == 0.5f );

	float angle = b3RevoluteJoint_GetAngle( jointId );
	MAYBE_UNUSED( angle );

	b3RevoluteJoint_EnableLimit( jointId, true );
	ENSURE( b3RevoluteJoint_IsLimitEnabled( jointId ) == true );

	b3RevoluteJoint_SetLimits( jointId, -1.0f, 1.0f );
	ENSURE( b3RevoluteJoint_GetLowerLimit( jointId ) == -1.0f );
	ENSURE( b3RevoluteJoint_GetUpperLimit( jointId ) == 1.0f );

	b3RevoluteJoint_EnableMotor( jointId, true );
	ENSURE( b3RevoluteJoint_IsMotorEnabled( jointId ) == true );

	b3RevoluteJoint_SetMotorSpeed( jointId, 2.0f );
	ENSURE( b3RevoluteJoint_GetMotorSpeed( jointId ) == 2.0f );

	b3RevoluteJoint_SetMaxMotorTorque( jointId, 40.0f );
	ENSURE( b3RevoluteJoint_GetMaxMotorTorque( jointId ) == 40.0f );

	float motorTorque = b3RevoluteJoint_GetMotorTorque( jointId );
	MAYBE_UNUSED( motorTorque );

	return FinishJoint( jointId, f.worldId );
}

static int TestSphericalJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3SphericalJointDef def = b3DefaultSphericalJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateSphericalJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_sphericalJoint ) != 0 )
	{
		return 1;
	}

	b3SphericalJoint_EnableConeLimit( jointId, true );
	ENSURE( b3SphericalJoint_IsConeLimitEnabled( jointId ) == true );

	b3SphericalJoint_SetConeLimit( jointId, 0.5f );
	ENSURE( b3SphericalJoint_GetConeLimit( jointId ) == 0.5f );

	float coneAngle = b3SphericalJoint_GetConeAngle( jointId );
	MAYBE_UNUSED( coneAngle );

	b3SphericalJoint_EnableTwistLimit( jointId, true );
	ENSURE( b3SphericalJoint_IsTwistLimitEnabled( jointId ) == true );

	b3SphericalJoint_SetTwistLimits( jointId, -0.5f, 0.5f );
	ENSURE( b3SphericalJoint_GetLowerTwistLimit( jointId ) == -0.5f );
	ENSURE( b3SphericalJoint_GetUpperTwistLimit( jointId ) == 0.5f );

	float twistAngle = b3SphericalJoint_GetTwistAngle( jointId );
	MAYBE_UNUSED( twistAngle );

	b3SphericalJoint_EnableSpring( jointId, true );
	ENSURE( b3SphericalJoint_IsSpringEnabled( jointId ) == true );

	b3SphericalJoint_SetSpringHertz( jointId, 5.0f );
	ENSURE( b3SphericalJoint_GetSpringHertz( jointId ) == 5.0f );

	b3SphericalJoint_SetSpringDampingRatio( jointId, 0.5f );
	ENSURE( b3SphericalJoint_GetSpringDampingRatio( jointId ) == 0.5f );

	// 90 degrees about z, a unit quaternion that round-trips through storage
	b3Quat targetRotation = { { 0.0f, 0.0f, 0.7071068f }, 0.7071068f };
	b3SphericalJoint_SetTargetRotation( jointId, targetRotation );
	b3Quat gotRotation = b3SphericalJoint_GetTargetRotation( jointId );
	ENSURE_SMALL( gotRotation.v.x - targetRotation.v.x, 1.0e-5f );
	ENSURE_SMALL( gotRotation.v.y - targetRotation.v.y, 1.0e-5f );
	ENSURE_SMALL( gotRotation.v.z - targetRotation.v.z, 1.0e-5f );
	ENSURE_SMALL( gotRotation.s - targetRotation.s, 1.0e-5f );

	b3SphericalJoint_EnableMotor( jointId, true );
	ENSURE( b3SphericalJoint_IsMotorEnabled( jointId ) == true );

	b3Vec3 motorVelocity = { 0.1f, 0.2f, 0.3f };
	b3SphericalJoint_SetMotorVelocity( jointId, motorVelocity );
	b3Vec3 gotMotorVelocity = b3SphericalJoint_GetMotorVelocity( jointId );
	ENSURE( gotMotorVelocity.x == 0.1f && gotMotorVelocity.y == 0.2f && gotMotorVelocity.z == 0.3f );

	b3SphericalJoint_SetMaxMotorTorque( jointId, 50.0f );
	ENSURE( b3SphericalJoint_GetMaxMotorTorque( jointId ) == 50.0f );

	b3Vec3 motorTorque = b3SphericalJoint_GetMotorTorque( jointId );
	MAYBE_UNUSED( motorTorque );

	return FinishJoint( jointId, f.worldId );
}

static int TestWeldJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3WeldJointDef def = b3DefaultWeldJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateWeldJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_weldJoint ) != 0 )
	{
		return 1;
	}

	b3WeldJoint_SetLinearHertz( jointId, 3.0f );
	ENSURE( b3WeldJoint_GetLinearHertz( jointId ) == 3.0f );

	b3WeldJoint_SetLinearDampingRatio( jointId, 0.5f );
	ENSURE( b3WeldJoint_GetLinearDampingRatio( jointId ) == 0.5f );

	b3WeldJoint_SetAngularHertz( jointId, 4.0f );
	ENSURE( b3WeldJoint_GetAngularHertz( jointId ) == 4.0f );

	b3WeldJoint_SetAngularDampingRatio( jointId, 0.7f );
	ENSURE( b3WeldJoint_GetAngularDampingRatio( jointId ) == 0.7f );

	return FinishJoint( jointId, f.worldId );
}

static int TestWheelJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3WheelJointDef def = b3DefaultWheelJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateWheelJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_wheelJoint ) != 0 )
	{
		return 1;
	}

	b3WheelJoint_EnableSuspension( jointId, true );
	ENSURE( b3WheelJoint_IsSuspensionEnabled( jointId ) == true );

	b3WheelJoint_SetSuspensionHertz( jointId, 5.0f );
	ENSURE( b3WheelJoint_GetSuspensionHertz( jointId ) == 5.0f );

	b3WheelJoint_SetSuspensionDampingRatio( jointId, 0.5f );
	ENSURE( b3WheelJoint_GetSuspensionDampingRatio( jointId ) == 0.5f );

	b3WheelJoint_EnableSuspensionLimit( jointId, true );
	ENSURE( b3WheelJoint_IsSuspensionLimitEnabled( jointId ) == true );

	b3WheelJoint_SetSuspensionLimits( jointId, -1.0f, 1.0f );
	ENSURE( b3WheelJoint_GetLowerSuspensionLimit( jointId ) == -1.0f );
	ENSURE( b3WheelJoint_GetUpperSuspensionLimit( jointId ) == 1.0f );

	b3WheelJoint_EnableSpinMotor( jointId, true );
	ENSURE( b3WheelJoint_IsSpinMotorEnabled( jointId ) == true );

	b3WheelJoint_SetSpinMotorSpeed( jointId, 6.0f );
	ENSURE( b3WheelJoint_GetSpinMotorSpeed( jointId ) == 6.0f );

	b3WheelJoint_SetMaxSpinTorque( jointId, 35.0f );
	ENSURE( b3WheelJoint_GetMaxSpinTorque( jointId ) == 35.0f );

	float spinSpeed = b3WheelJoint_GetSpinSpeed( jointId );
	float spinTorque = b3WheelJoint_GetSpinTorque( jointId );
	MAYBE_UNUSED( spinSpeed );
	MAYBE_UNUSED( spinTorque );

	b3WheelJoint_EnableSteering( jointId, true );
	ENSURE( b3WheelJoint_IsSteeringEnabled( jointId ) == true );

	b3WheelJoint_SetSteeringHertz( jointId, 7.0f );
	ENSURE( b3WheelJoint_GetSteeringHertz( jointId ) == 7.0f );

	b3WheelJoint_SetSteeringDampingRatio( jointId, 0.8f );
	ENSURE( b3WheelJoint_GetSteeringDampingRatio( jointId ) == 0.8f );

	b3WheelJoint_SetMaxSteeringTorque( jointId, 45.0f );
	ENSURE( b3WheelJoint_GetMaxSteeringTorque( jointId ) == 45.0f );

	b3WheelJoint_EnableSteeringLimit( jointId, true );
	ENSURE( b3WheelJoint_IsSteeringLimitEnabled( jointId ) == true );

	b3WheelJoint_SetSteeringLimits( jointId, -0.6f, 0.6f );
	ENSURE( b3WheelJoint_GetLowerSteeringLimit( jointId ) == -0.6f );
	ENSURE( b3WheelJoint_GetUpperSteeringLimit( jointId ) == 0.6f );

	b3WheelJoint_SetTargetSteeringAngle( jointId, 0.25f );
	ENSURE( b3WheelJoint_GetTargetSteeringAngle( jointId ) == 0.25f );

	float steeringAngle = b3WheelJoint_GetSteeringAngle( jointId );
	float steeringTorque = b3WheelJoint_GetSteeringTorque( jointId );
	MAYBE_UNUSED( steeringAngle );
	MAYBE_UNUSED( steeringTorque );

	return FinishJoint( jointId, f.worldId );
}

int JointTest( void )
{
	RUN_SUBTEST( TestParallelJoint );
	RUN_SUBTEST( TestDistanceJoint );
	RUN_SUBTEST( TestFilterJoint );
	RUN_SUBTEST( TestMotorJoint );
	RUN_SUBTEST( TestPrismaticJoint );
	RUN_SUBTEST( TestRevoluteJoint );
	RUN_SUBTEST( TestSphericalJoint );
	RUN_SUBTEST( TestWeldJoint );
	RUN_SUBTEST( TestWheelJoint );

	return 0;
}
