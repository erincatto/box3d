// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/math_functions.h"

#include <math.h>

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

static int TestMotionLockJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3MotionLockJointDef def = b3DefaultMotionLockJointDef();
	SetCommonFrames( &def.base, &f );
	def.lockLinearY = true;
	def.lockAngularXY = true;
	b3JointId jointId = b3CreateMotionLockJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_motionLockJoint ) != 0 )
	{
		return 1;
	}

	ENSURE( b3MotionLockJoint_IsLinearLockEnabled( jointId, 0 ) == false );
	ENSURE( b3MotionLockJoint_IsLinearLockEnabled( jointId, 1 ) );
	ENSURE( b3MotionLockJoint_IsLinearLockEnabled( jointId, 2 ) == false );
	ENSURE( b3MotionLockJoint_IsAngularLockEnabled( jointId ) );

	b3MotionLockJoint_EnableLinearLock( jointId, 0, true );
	b3MotionLockJoint_EnableLinearLock( jointId, 2, true );
	ENSURE( b3MotionLockJoint_IsLinearLockEnabled( jointId, 0 ) );
	ENSURE( b3MotionLockJoint_IsLinearLockEnabled( jointId, 2 ) );

	b3MotionLockJoint_EnableAngularLock( jointId, false );
	ENSURE( b3MotionLockJoint_IsAngularLockEnabled( jointId ) == false );

	return FinishJoint( jointId, f.worldId );
}

typedef struct MotionLockScene
{
	b3WorldId worldId;
	b3BodyId groundId;
	b3BodyId bodyId;
	b3JointId jointId;
} MotionLockScene;

static const b3Vec3 kLockOrigin = { 1.0f, 4.0f, -2.0f };

// A box under gravity locked to the ground. An anchor at the center of mass takes the decoupled path.
static MotionLockScene CreateMotionLockScene( b3MotionLockJointDef def, b3Vec3 localAnchor )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };

	MotionLockScene s;
	s.worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	s.groundId = b3CreateBody( s.worldId, &groundDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ kLockOrigin.x, kLockOrigin.y, kLockOrigin.z };
	bodyDef.enableSleep = false;
	s.bodyId = b3CreateBody( s.worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.25f, 1.0f );
	b3CreateHullShape( s.bodyId, &shapeDef, &box.base );

	def.base.bodyIdA = s.groundId;
	def.base.bodyIdB = s.bodyId;
	def.base.localFrameA.p = b3Add( kLockOrigin, localAnchor );
	def.base.localFrameB.p = localAnchor;
	s.jointId = b3CreateMotionLockJoint( s.worldId, &def );
	return s;
}

static void StepScene( b3WorldId worldId, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
}

// Every combination of linear locks holds its axes against gravity and a push while the free axes move.
static int TestMotionLockJointLinear( void )
{
	b3Vec3 push = { 2.0f, 3.0f, -1.0f };

	for ( int mask = 1; mask < 8; ++mask )
	{
		b3MotionLockJointDef def = b3DefaultMotionLockJointDef();
		def.lockLinearX = ( mask & 1 ) != 0;
		def.lockLinearY = ( mask & 2 ) != 0;
		def.lockLinearZ = ( mask & 4 ) != 0;

		MotionLockScene s = CreateMotionLockScene( def, b3Vec3_zero );
		b3Body_SetLinearVelocity( s.bodyId, push );
		StepScene( s.worldId, 60 );

		b3Pos p = b3Body_GetPosition( s.bodyId );
		b3Vec3 delta = { (float)p.x - kLockOrigin.x, (float)p.y - kLockOrigin.y, (float)p.z - kLockOrigin.z };

		ENSURE( def.lockLinearX ? b3AbsFloat( delta.x ) < 0.01f : b3AbsFloat( delta.x ) > 1.0f );
		ENSURE( def.lockLinearY ? b3AbsFloat( delta.y ) < 0.01f : b3AbsFloat( delta.y ) > 1.0f );
		ENSURE( def.lockLinearZ ? b3AbsFloat( delta.z ) < 0.01f : b3AbsFloat( delta.z ) > 0.5f );
		ENSURE( b3Joint_GetLinearSeparation( s.jointId ) < 0.01f );

		b3DestroyWorld( s.worldId );
	}

	return 0;
}

// An anchor off the center of mass couples the rows. The point lock becomes a pendulum pivot.
static int TestMotionLockJointCoupled( void )
{
	b3MotionLockJointDef def = b3DefaultMotionLockJointDef();
	def.lockLinearX = true;
	def.lockLinearY = true;
	def.lockLinearZ = true;

	MotionLockScene s = CreateMotionLockScene( def, (b3Vec3){ 0.5f, 0.0f, 0.5f } );
	StepScene( s.worldId, 120 );

	ENSURE( b3Joint_GetLinearSeparation( s.jointId ) < 0.01f );
	ENSURE( b3IsValidVec3( b3Body_GetLinearVelocity( s.bodyId ) ) );

	b3MotionLockJoint_EnableLinearLock( s.jointId, 1, false );
	StepScene( s.worldId, 60 );
	ENSURE( b3Joint_GetLinearSeparation( s.jointId ) < 0.01f );

	b3DestroyWorld( s.worldId );
	return 0;
}

// The rotation lock keeps the body z axis on the world z axis while spin about it stays free.
static int TestMotionLockJointRotation( void )
{
	b3MotionLockJointDef def = b3DefaultMotionLockJointDef();
	def.lockLinearX = true;
	def.lockLinearY = true;
	def.lockLinearZ = true;
	def.lockAngularXY = true;

	MotionLockScene s = CreateMotionLockScene( def, b3Vec3_zero );
	b3Body_SetAngularVelocity( s.bodyId, (b3Vec3){ 3.0f, -2.0f, 4.0f } );
	StepScene( s.worldId, 60 );

	b3Vec3 w = b3Body_GetAngularVelocity( s.bodyId );
	ENSURE( b3AbsFloat( w.x ) < 0.01f );
	ENSURE( b3AbsFloat( w.y ) < 0.01f );
	ENSURE( b3AbsFloat( w.z ) > 1.0f );

	b3Vec3 axisZ = b3RotateVector( b3Body_GetRotation( s.bodyId ), b3Vec3_axisZ );
	ENSURE( axisZ.z > 0.999f );
	ENSURE( b3Joint_GetAngularSeparation( s.jointId ) < 0.01f );

	b3DestroyWorld( s.worldId );
	return 0;
}

// Issue 211: a heavy sphere resting on a light platform held by a point lock and a rotation lock.
// The lock rows iterate with the contact, so the platform supports the sphere like a static body.
static int TestMotionLockJointPlatform( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -20.0f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	b3BodyDef platformDef = b3DefaultBodyDef();
	platformDef.type = b3_dynamicBody;
	platformDef.enableSleep = false;
	b3BodyId platformId = b3CreateBody( worldId, &platformDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( 2.0f, 0.05f, 0.5f );
	b3CreateHullShape( platformId, &shapeDef, &box.base );

	b3MassData massData = b3Body_GetMassData( platformId );
	massData.inertia = b3MulSM( 1.0f / massData.mass, massData.inertia );
	massData.mass = 1.0f;
	b3Body_SetMassData( platformId, massData );

	b3MotionLockJointDef lockDef = b3DefaultMotionLockJointDef();
	lockDef.base.bodyIdA = groundId;
	lockDef.base.bodyIdB = platformId;
	lockDef.lockLinearX = true;
	lockDef.lockLinearY = true;
	lockDef.lockLinearZ = true;
	lockDef.lockAngularXY = true;
	b3CreateMotionLockJoint( worldId, &lockDef );

	b3BodyDef sphereDef = b3DefaultBodyDef();
	sphereDef.type = b3_dynamicBody;
	sphereDef.position = (b3Pos){ 0.0f, 0.65f, 0.0f };
	sphereDef.enableSleep = false;
	b3BodyId sphereId = b3CreateBody( worldId, &sphereDef );

	b3Sphere sphere = { b3Vec3_zero, 0.5f };
	b3CreateSphereShape( sphereId, &shapeDef, &sphere );

	massData = b3Body_GetMassData( sphereId );
	massData.inertia = b3MulSM( 10.0f / massData.mass, massData.inertia );
	massData.mass = 10.0f;
	b3Body_SetMassData( sphereId, massData );

	for ( int i = 0; i < 200; ++i )
	{
		b3World_Step( worldId, 1.0f / 50.0f, 4 );
	}

	float gap = (float)b3Body_GetPosition( sphereId ).y - 0.55f;
	ENSURE( b3AbsFloat( gap ) < 0.02f );

	b3DestroyWorld( worldId );
	return 0;
}

// A revolute pivot off the center of mass plus a full point lock at the center is redundant. The two
// constraints share the load through iteration instead of forming one singular block.
static int TestMotionLockJointRedundant( void )
{
	b3MotionLockJointDef def = b3DefaultMotionLockJointDef();
	def.lockLinearX = true;
	def.lockLinearY = true;
	def.lockLinearZ = true;

	MotionLockScene s = CreateMotionLockScene( def, b3Vec3_zero );

	b3RevoluteJointDef revoluteDef = b3DefaultRevoluteJointDef();
	revoluteDef.base.bodyIdA = s.groundId;
	revoluteDef.base.bodyIdB = s.bodyId;
	revoluteDef.base.localFrameA.p = (b3Vec3){ kLockOrigin.x - 0.5f, kLockOrigin.y, kLockOrigin.z };
	revoluteDef.base.localFrameB.p = (b3Vec3){ -0.5f, 0.0f, 0.0f };
	b3CreateRevoluteJoint( s.worldId, &revoluteDef );

	StepScene( s.worldId, 120 );

	ENSURE( b3Length( b3Body_GetLinearVelocity( s.bodyId ) ) < 0.1f );
	ENSURE( b3Length( b3Body_GetAngularVelocity( s.bodyId ) ) < 0.1f );
	ENSURE( b3Joint_GetLinearSeparation( s.jointId ) < 0.01f );

	b3DestroyWorld( s.worldId );
	return 0;
}

#if defined( BOX3D_DOUBLE_PRECISION )
// Ground center at the origin with the locks 1e7 m away. The separation must stay in double precision, a
// float difference of the two centers rounds to 1 m and lets the body sag under gravity.
static float RunFarMotionLock( b3Vec3 localAnchorB, bool lockZ )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	double far = 1.0e7;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ far - localAnchorB.x, far - localAnchorB.y, far - localAnchorB.z };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3MotionLockJointDef def = b3DefaultMotionLockJointDef();
	def.base.bodyIdA = groundId;
	def.base.bodyIdB = bodyId;
	def.base.localFrameA.p = (b3Vec3){ (float)far, (float)far, (float)far };
	def.base.localFrameB.p = localAnchorB;
	def.lockLinearX = true;
	def.lockLinearY = true;
	def.lockLinearZ = lockZ;
	b3JointId jointId = b3CreateMotionLockJoint( worldId, &def );

	float maxSeparation = 0.0f;
	for ( int i = 0; i < 240; ++i )
	{
		float push = 20.0f * sinf( 0.05f * i );
		b3Body_ApplyForceToCenter( bodyId, (b3Vec3){ push, 0.0f, push }, true );
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		maxSeparation = b3MaxFloat( maxSeparation, b3Joint_GetLinearSeparation( jointId ) );
	}

	b3DestroyWorld( worldId );
	return maxSeparation;
}

static int TestMotionLockJointFar( void )
{
	ENSURE( RunFarMotionLock( b3Vec3_zero, false ) < 1.0e-3f );
	ENSURE( RunFarMotionLock( (b3Vec3){ 0.5f, 0.5f, 0.5f }, true ) < 1.0e-3f );
	return 0;
}
#endif

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

static int TestMoverJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3MoverJointDef def = b3DefaultMoverJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateMoverJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_moverJoint ) != 0 )
	{
		return 1;
	}

	b3Vec3 velocity = { 1.0f, 2.0f, -3.0f };
	b3MoverJoint_SetLinearVelocity( jointId, velocity );
	b3Vec3 gotVelocity = b3MoverJoint_GetLinearVelocity( jointId );
	ENSURE( gotVelocity.x == 1.0f && gotVelocity.y == 2.0f && gotVelocity.z == -3.0f );

	// The y axis has no force budget, so it must stay at rest while x and z reach the target
	float maxForce = 1000.0f;
	b3MoverJoint_SetMaxVelocityForce( jointId, maxForce );
	float gotForce = b3MoverJoint_GetMaxVelocityForce( jointId );
	ENSURE( gotForce == 1000.0f );

	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( f.worldId, 1.0f / 60.0f, 4 );
	}

	b3Vec3 v = b3Body_GetLinearVelocity( f.bodyId );
	ENSURE_SMALL( v.x - 1.0f, 1.0e-4f );
	ENSURE_SMALL( v.y, 1.0e-4f );
	ENSURE_SMALL( v.z + 3.0f, 1.0e-4f );

	b3Vec3 w = b3Body_GetAngularVelocity( f.bodyId );
	ENSURE( w.x == 0.0f && w.y == 0.0f && w.z == 0.0f );

	return FinishJoint( jointId, f.worldId );
}

// A box held up by a pogo spring under gravity should settle at the rest length
static int TestPogoJoint( void )
{
	JointFixture f = CreateJointFixture();
	b3World_SetGravity( f.worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	float restLength = 2.0f;
	float mass = b3Body_GetMass( f.bodyId );

	b3PogoJointDef def = b3DefaultPogoJointDef();
	def.base.bodyIdA = f.groundId;
	def.base.bodyIdB = f.bodyId;
	def.base.localFrameA.p = (b3Vec3){ 0.0f, 1.0f, 0.0f };
	def.base.localFrameB.p = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	def.normal = b3Vec3_axisY;
	def.restLength = restLength;
	def.hertz = 2.0f;
	def.dampingRatio = 1.0f;
	def.maxCompressionForce = 100.0f * 10.0f * mass;
	def.maxTensionForce = 100.0f * 10.0f * mass;
	b3JointId jointId = b3CreatePogoJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_pogoJoint ) != 0 )
	{
		return 1;
	}

	b3PogoJoint_SetRestLength( jointId, restLength );
	ENSURE( b3PogoJoint_GetRestLength( jointId ) == restLength );

	b3PogoJoint_SetSpringHertz( jointId, 3.0f );
	ENSURE( b3PogoJoint_GetSpringHertz( jointId ) == 3.0f );

	b3PogoJoint_SetSpringDampingRatio( jointId, 0.9f );
	ENSURE( b3PogoJoint_GetSpringDampingRatio( jointId ) == 0.9f );

	float maxTension = 50.0f * 10.0f * mass;
	b3PogoJoint_SetMaxTensionForce( jointId, maxTension );
	ENSURE( b3PogoJoint_GetMaxTensionForce( jointId ) == maxTension );

	float maxCompression = 60.0f * 10.0f * mass;
	b3PogoJoint_SetMaxCompressionForce( jointId, maxCompression );
	ENSURE( b3PogoJoint_GetMaxCompressionForce( jointId ) == maxCompression );

	b3Vec3 normal = b3Normalize( (b3Vec3){ 0.2f, 1.0f, 0.0f } );
	b3PogoJoint_SetNormal( jointId, normal );
	b3Vec3 gotNormal = b3PogoJoint_GetNormal( jointId );
	ENSURE( gotNormal.x == normal.x && gotNormal.y == normal.y && gotNormal.z == normal.z );
	b3PogoJoint_SetNormal( jointId, b3Vec3_axisY );

	// Starts stretched by one meter
	ENSURE_SMALL( b3PogoJoint_GetCurrentLength( jointId ) - 3.0f, 1.0e-5f );

	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( f.worldId, 1.0f / 60.0f, 4 );
	}

	ENSURE_SMALL( b3PogoJoint_GetCurrentLength( jointId ) - restLength, 0.01f );
	ENSURE_SMALL( b3Body_GetLinearVelocity( f.bodyId ).y, 0.01f );

	// Holding the weight takes an impulse of m * g * h each sub-step
	ENSURE_SMALL( b3PogoJoint_GetImpulse( jointId ) - mass * 10.0f / 240.0f, 0.001f * mass );

	b3Vec3 force = b3Joint_GetConstraintForce( jointId );
	ENSURE_SMALL( force.y - mass * 10.0f, 0.1f * mass );

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

static int TestSphericalMotorFrame( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_kinematicBody;
	bodyDef.angularVelocity = (b3Vec3){ 0.0f, 2.0f, 0.0f };
	b3BodyId bodyIdA = b3CreateBody( worldId, &bodyDef );

	bodyDef.type = b3_dynamicBody;
	bodyDef.angularVelocity = b3Vec3_zero;
	b3BodyId bodyIdB = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeCubeHull( 0.5f );
	b3CreateHullShape( bodyIdB, &shapeDef, &box.base );

	b3Vec3 motorVelocity = { 1.0f, 0.0f, 0.0f };
	b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
	jointDef.base.bodyIdA = bodyIdA;
	jointDef.base.bodyIdB = bodyIdB;
	jointDef.enableMotor = true;
	jointDef.maxMotorTorque = 1000.0f;
	jointDef.motorVelocity = motorVelocity;
	b3JointId jointId = b3CreateSphericalJoint( worldId, &jointDef );

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Vec3 relativeVelocity = b3Sub( b3Body_GetAngularVelocity( bodyIdB ), b3Body_GetAngularVelocity( bodyIdA ) );
	b3Vec3 localVelocity = b3InvRotateVector( b3Body_GetRotation( bodyIdA ), relativeVelocity );
	ENSURE_SMALL( localVelocity.x - motorVelocity.x, 0.02f );
	ENSURE_SMALL( localVelocity.y - motorVelocity.y, 0.02f );
	ENSURE_SMALL( localVelocity.z - motorVelocity.z, 0.02f );

	ENSURE( b3Joint_IsValid( jointId ) );
	b3DestroyWorld( worldId );
	return 0;
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

// The body starts at -3 radians with a spring target of +3. The short way crosses pi and never
// comes near zero. The long way sweeps through zero.
static int CheckSpringShortWay( b3JointType type )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

	b3Vec3 axis = type == b3_revoluteJoint ? b3Vec3_axisZ : b3Vec3_axisX;
	bodyDef.type = b3_dynamicBody;
	bodyDef.rotation = b3MakeQuatFromAxisAngle( axis, -3.0f );
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.1f, 0.1f );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3JointId jointId;
	if ( type == b3_revoluteJoint )
	{
		b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
		def.base.bodyIdA = groundId;
		def.base.bodyIdB = bodyId;
		def.enableSpring = true;
		def.hertz = 2.0f;
		def.dampingRatio = 1.0f;
		def.targetAngle = 3.0f;
		jointId = b3CreateRevoluteJoint( worldId, &def );
	}
	else
	{
		b3WheelJointDef def = b3DefaultWheelJointDef();
		def.base.bodyIdA = groundId;
		def.base.bodyIdB = bodyId;
		def.enableSteering = true;
		def.steeringHertz = 2.0f;
		def.steeringDampingRatio = 1.0f;
		def.maxSteeringTorque = 1000.0f;
		def.targetSteeringAngle = 3.0f;
		jointId = b3CreateWheelJoint( worldId, &def );
	}

	float startAngle =
		type == b3_revoluteJoint ? b3RevoluteJoint_GetAngle( jointId ) : b3WheelJoint_GetSteeringAngle( jointId );
	ENSURE_SMALL( startAngle + 3.0f, 0.01f );

	float closestToZero = B3_PI;
	float angle = startAngle;
	for ( int i = 0; i < 180; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		angle = type == b3_revoluteJoint ? b3RevoluteJoint_GetAngle( jointId ) : b3WheelJoint_GetSteeringAngle( jointId );
		closestToZero = b3MinFloat( closestToZero, fabsf( angle ) );
	}

	ENSURE( closestToZero > 2.5f );
	ENSURE_SMALL( angle - 3.0f, 0.05f );

	b3DestroyWorld( worldId );
	return 0;
}

static int TestRevoluteSpringShortWay( void )
{
	return CheckSpringShortWay( b3_revoluteJoint );
}

static int TestWheelSteeringShortWay( void )
{
	return CheckSpringShortWay( b3_wheelJoint );
}

static void SetHeldFrames( b3JointDef* base, b3BodyId groundId, b3BodyId bodyId, b3Vec3 origin, b3Vec3 anchor, b3Quat q )
{
	base->bodyIdA = groundId;
	base->bodyIdB = bodyId;
	base->localFrameA = (b3Transform){ b3Add( origin, anchor ), q };
	base->localFrameB = (b3Transform){ anchor, q };
}

// A box held still under gravity by one joint. The force at the anchor carries the weight and the
// torque cancels the moment of that force about the center of mass.
static int CheckHeldWeight( b3JointType type, float anchorOffset )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	b3Vec3 origin = { 0.0f, 4.0f, 0.0f };
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ origin.x, origin.y, origin.z };
	bodyDef.enableSleep = false;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.25f, 1.0f );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	// Yaw keeps the frame x and z axes level, so neither a slider nor a free spin axis takes gravity
	b3Quat q = b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.5f );
	b3Vec3 anchor = b3RotateVector( q, (b3Vec3){ 0.0f, 0.0f, anchorOffset } );

	b3JointId jointId = b3_nullJointId;
	switch ( type )
	{
		case b3_motionLockJoint:
		{
			b3MotionLockJointDef def = b3DefaultMotionLockJointDef();
			SetHeldFrames( &def.base, groundId, bodyId, origin, anchor, q );
			def.lockLinearX = true;
			def.lockLinearY = true;
			def.lockLinearZ = true;
			def.lockAngularXY = true;
			jointId = b3CreateMotionLockJoint( worldId, &def );
		}
		break;

		case b3_prismaticJoint:
		{
			b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
			SetHeldFrames( &def.base, groundId, bodyId, origin, anchor, q );
			jointId = b3CreatePrismaticJoint( worldId, &def );
		}
		break;

		case b3_revoluteJoint:
		{
			b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
			SetHeldFrames( &def.base, groundId, bodyId, origin, anchor, q );
			jointId = b3CreateRevoluteJoint( worldId, &def );
		}
		break;

		case b3_sphericalJoint:
		{
			b3SphericalJointDef def = b3DefaultSphericalJointDef();
			SetHeldFrames( &def.base, groundId, bodyId, origin, anchor, q );
			jointId = b3CreateSphericalJoint( worldId, &def );
		}
		break;

		case b3_weldJoint:
		{
			b3WeldJointDef def = b3DefaultWeldJointDef();
			SetHeldFrames( &def.base, groundId, bodyId, origin, anchor, q );
			jointId = b3CreateWeldJoint( worldId, &def );
		}
		break;

		case b3_wheelJoint:
		{
			b3WheelJointDef def = b3DefaultWheelJointDef();
			SetHeldFrames( &def.base, groundId, bodyId, origin, anchor, q );
			jointId = b3CreateWheelJoint( worldId, &def );
		}
		break;

		default:
			ENSURE( false );
	}

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	float weight = 10.0f * b3Body_GetMass( bodyId );
	b3Vec3 expectedForce = { 0.0f, weight, 0.0f };
	b3Vec3 expectedTorque = b3Neg( b3Cross( anchor, expectedForce ) );

	b3Vec3 force = b3Joint_GetConstraintForce( jointId );
	b3Vec3 torque = b3Joint_GetConstraintTorque( jointId );
	ENSURE_SMALL( b3Length( b3Sub( force, expectedForce ) ), 0.02f * weight );
	ENSURE_SMALL( b3Length( b3Sub( torque, expectedTorque ) ), 0.02f * weight );

	float torqueLength = b3Length( expectedTorque );
	b3Joint_SetForceThreshold( jointId, FLT_MAX );
	b3Joint_SetTorqueThreshold( jointId, 1.1f * torqueLength + 0.05f * weight );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( b3World_GetJointEvents( worldId ).count == 0 );

	if ( torqueLength > 0.1f * weight )
	{
		b3Joint_SetTorqueThreshold( jointId, 0.9f * torqueLength );
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		b3JointEvents events = b3World_GetJointEvents( worldId );
		ENSURE( events.count == 1 && B3_ID_EQUALS( events.jointEvents[0].jointId, jointId ) );
	}

	b3Joint_SetTorqueThreshold( jointId, FLT_MAX );

	// A body that cannot rotate needs no angular impulse, so none may linger for warm starting
	b3Body_SetFixedRotation( bodyId, true );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	force = b3Joint_GetConstraintForce( jointId );
	torque = b3Joint_GetConstraintTorque( jointId );
	ENSURE_SMALL( b3Length( b3Sub( force, expectedForce ) ), 0.02f * weight );
	ENSURE( b3Length( torque ) == 0.0f );

	b3DestroyWorld( worldId );
	return 0;
}

static int TestConstraintForceAndTorque( void )
{
	b3JointType offsetTypes[] = {
		b3_motionLockJoint, b3_prismaticJoint, b3_revoluteJoint, b3_weldJoint, b3_wheelJoint,
	};

	for ( int i = 0; i < (int)( sizeof( offsetTypes ) / sizeof( offsetTypes[0] ) ); ++i )
	{
		if ( CheckHeldWeight( offsetTypes[i], 0.0f ) != 0 || CheckHeldWeight( offsetTypes[i], 0.5f ) != 0 )
		{
			printf( "joint type %d\n", (int)offsetTypes[i] );
			return 1;
		}
	}

	// A ball joint cannot hold a moment, so its anchor sits at the center of mass
	return CheckHeldWeight( b3_sphericalJoint, 0.0f );
}

// Issue #117: a joint created with collideConnected false must clear contacts
// that already exist between the two bodies.
static int TestCreateJointClearsContacts( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	bodyDef.position = (b3Pos){ 0.0f, 0.0f, 0.0f };
	b3BodyId bodyIdA = b3CreateBody( worldId, &bodyDef );

	// Overlapping so a touching contact forms
	bodyDef.position = (b3Pos){ 0.5f, 0.0f, 0.0f };
	b3BodyId bodyIdB = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3CreateHullShape( bodyIdA, &shapeDef, &box.base );
	b3CreateHullShape( bodyIdB, &shapeDef, &box.base );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( b3Body_GetContactCapacity( bodyIdA ) > 0 );

	b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
	jointDef.base.bodyIdA = bodyIdA;
	jointDef.base.bodyIdB = bodyIdB;
	jointDef.base.collideConnected = false;
	b3CreateRevoluteJoint( worldId, &jointDef );

	ENSURE( b3Body_GetContactCapacity( bodyIdA ) == 0 );
	ENSURE( b3Body_GetContactCapacity( bodyIdB ) == 0 );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( b3Body_GetContactCapacity( bodyIdA ) == 0 );

	b3DestroyWorld( worldId );
	return 0;
}

// Issue #164: a joint with no dynamic body lives in the static set even when one body is an awake
// kinematic. Disabling and enabling that body has to respect this, whether the joint was created
// that way or got there through body type changes.
static int TestNonDynamicJointDisable( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyIdA = b3CreateBody( worldId, &bodyDef );
	bodyDef.position = (b3Pos){ 2.0f, 0.0f, 0.0f };
	b3BodyId bodyIdB = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeCubeHull( 0.5f );
	b3CreateHullShape( bodyIdA, &shapeDef, &box.base );
	b3CreateHullShape( bodyIdB, &shapeDef, &box.base );

	b3WeldJointDef jointDef = b3DefaultWeldJointDef();
	jointDef.base.bodyIdA = bodyIdA;
	jointDef.base.bodyIdB = bodyIdB;
	b3JointId weldId = b3CreateWeldJoint( worldId, &jointDef );

	b3Body_SetType( bodyIdA, b3_staticBody );
	b3Body_SetType( bodyIdB, b3_kinematicBody );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	b3Body_Disable( bodyIdB );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	b3Body_Enable( bodyIdB );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( b3Joint_IsValid( weldId ) );

	b3Body_SetType( bodyIdB, b3_dynamicBody );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( b3Joint_IsAwake( weldId ) );

	// Created directly between a static and a kinematic body
	bodyDef.type = b3_kinematicBody;
	bodyDef.position = (b3Pos){ 4.0f, 0.0f, 0.0f };
	b3BodyId bodyIdC = b3CreateBody( worldId, &bodyDef );
	b3CreateHullShape( bodyIdC, &shapeDef, &box.base );

	b3RevoluteJointDef revoluteDef = b3DefaultRevoluteJointDef();
	revoluteDef.base.bodyIdA = bodyIdA;
	revoluteDef.base.bodyIdB = bodyIdC;
	b3JointId revoluteId = b3CreateRevoluteJoint( worldId, &revoluteDef );

	b3Body_Disable( bodyIdC );
	b3Body_Enable( bodyIdC );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( b3Joint_IsAwake( revoluteId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

enum ReleaseCase
{
	e_motorSpringForce,
	e_motorLinearHertz,
	e_motorVelocityForce,
	e_motorSpringTorque,
	e_motorAngularHertz,
	e_motorVelocityTorque,
	e_distanceHertz,
	e_distanceLimit,
	e_parallelTorque,
	e_releaseCaseCount,
};

static b3JointId CreateReleaseJoint( JointFixture* f, int releaseCase )
{
	switch ( releaseCase )
	{
		case e_motorSpringForce:
		case e_motorLinearHertz:
		{
			b3MotorJointDef def = b3DefaultMotorJointDef();
			SetCommonFrames( &def.base, f );
			def.linearHertz = 5.0f;
			def.linearDampingRatio = 1.0f;
			def.maxSpringForce = 1000.0f;
			return b3CreateMotorJoint( f->worldId, &def );
		}

		case e_motorVelocityForce:
		{
			b3MotorJointDef def = b3DefaultMotorJointDef();
			SetCommonFrames( &def.base, f );
			def.maxVelocityForce = 1000.0f;
			return b3CreateMotorJoint( f->worldId, &def );
		}

		case e_motorSpringTorque:
		case e_motorAngularHertz:
		{
			b3MotorJointDef def = b3DefaultMotorJointDef();
			SetCommonFrames( &def.base, f );
			def.angularHertz = 5.0f;
			def.angularDampingRatio = 1.0f;
			def.maxSpringTorque = 1000.0f;
			return b3CreateMotorJoint( f->worldId, &def );
		}

		case e_motorVelocityTorque:
		{
			b3MotorJointDef def = b3DefaultMotorJointDef();
			SetCommonFrames( &def.base, f );
			def.maxVelocityTorque = 1000.0f;
			return b3CreateMotorJoint( f->worldId, &def );
		}

		case e_distanceHertz:
		case e_distanceLimit:
		{
			b3DistanceJointDef def = b3DefaultDistanceJointDef();
			def.base.bodyIdA = f->groundId;
			def.base.bodyIdB = f->bodyId;
			def.base.localFrameA.p = (b3Vec3){ 0.0f, 6.0f, 0.0f };
			def.length = 2.0f;
			def.enableSpring = true;
			if ( releaseCase == e_distanceHertz )
			{
				def.hertz = 5.0f;
				def.dampingRatio = 1.0f;
			}
			else
			{
				def.enableLimit = true;
				def.maxLength = 2.0f;
			}
			return b3CreateDistanceJoint( f->worldId, &def );
		}

		default:
		{
			b3ParallelJointDef def = b3DefaultParallelJointDef();
			SetCommonFrames( &def.base, f );
			def.hertz = 5.0f;
			def.dampingRatio = 1.0f;
			def.maxTorque = 1000.0f;
			return b3CreateParallelJoint( f->worldId, &def );
		}
	}
}

static void ReleaseJoint( b3JointId jointId, int releaseCase )
{
	switch ( releaseCase )
	{
		case e_motorSpringForce:
			b3MotorJoint_SetMaxSpringForce( jointId, 0.0f );
			break;
		case e_motorLinearHertz:
			b3MotorJoint_SetLinearHertz( jointId, 0.0f );
			break;
		case e_motorVelocityForce:
			b3MotorJoint_SetMaxVelocityForce( jointId, 0.0f );
			break;
		case e_motorSpringTorque:
			b3MotorJoint_SetMaxSpringTorque( jointId, 0.0f );
			break;
		case e_motorAngularHertz:
			b3MotorJoint_SetAngularHertz( jointId, 0.0f );
			break;
		case e_motorVelocityTorque:
			b3MotorJoint_SetMaxVelocityTorque( jointId, 0.0f );
			break;
		case e_distanceHertz:
			b3DistanceJoint_SetSpringHertz( jointId, 0.0f );
			break;
		case e_distanceLimit:
			b3DistanceJoint_EnableLimit( jointId, false );
			break;
		default:
			b3ParallelJoint_SetMaxTorque( jointId, 0.0f );
			break;
	}
}

// A joint term that is switched off must let go of a loaded body, not keep warm starting the impulse that held it
static int TestJointReleasesLoad( void )
{
	for ( int releaseCase = 0; releaseCase < e_releaseCaseCount; ++releaseCase )
	{
		JointFixture f = CreateJointFixture();
		b3JointId jointId = CreateReleaseJoint( &f, releaseCase );

		bool angular = releaseCase == e_motorSpringTorque || releaseCase == e_motorAngularHertz ||
					   releaseCase == e_motorVelocityTorque || releaseCase == e_parallelTorque;

		// Unit cube, so 1 kg and 1/6 kg m^2
		b3Vec3 force = angular ? b3Vec3_zero : (b3Vec3){ 0.0f, -10.0f, 0.0f };
		b3Vec3 torque = angular ? (b3Vec3){ 1.0f, 0.0f, 0.0f } : b3Vec3_zero;

		for ( int i = 0; i < 120; ++i )
		{
			if ( i == 60 )
			{
				b3Vec3 v = b3Body_GetLinearVelocity( f.bodyId );
				b3Vec3 w = b3Body_GetAngularVelocity( f.bodyId );
				ENSURE( b3Length( v ) < 0.1f && b3Length( w ) < 0.1f );
				ReleaseJoint( jointId, releaseCase );
			}

			b3Body_ApplyForceToCenter( f.bodyId, force, true );
			b3Body_ApplyTorque( f.bodyId, torque, true );
			b3World_Step( f.worldId, 1.0f / 60.0f, 4 );
		}

		// Free motion after one second is 10 m/s and 6 rad/s
		if ( angular )
		{
			b3Vec3 w = b3Body_GetAngularVelocity( f.bodyId );
			ENSURE( w.x > 5.0f );
		}
		else
		{
			b3Vec3 v = b3Body_GetLinearVelocity( f.bodyId );
			ENSURE( v.y < -9.0f );
		}

		b3DestroyWorld( f.worldId );
	}

	return 0;
}

// Issue #208. Joints to one kinematic body share graph colors, so a multithreaded warm start or solve must never
// store to the kinematic body. Only ThreadSanitizer sees the race itself; this checks the kinematic body is untouched.
static int TestKinematicJointFan( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 8;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3Vec3 anchorVelocity = { 1.0f, 0.0f, 0.0f };
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_kinematicBody;
	anchorDef.linearVelocity = anchorVelocity;
	b3BodyId anchorId = b3CreateBody( worldId, &anchorDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { .center = { 0.0f, 0.0f, 0.0f }, .radius = 0.25f };

	enum
	{
		e_jointTypeCount = 4,
		e_bodyCount = 128,
	};

	for ( int i = 0; i < e_bodyCount; ++i )
	{
		// Spaced so the spheres never touch, only joints
		b3Vec3 p = { (float)( i % 16 ), 2.0f + (float)( i / 16 ), 0.0f };

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ p.x, p.y, p.z };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );

		switch ( i % e_jointTypeCount )
		{
			case 0:
			{
				b3MotorJointDef def = b3DefaultMotorJointDef();
				def.base.bodyIdA = anchorId;
				def.base.bodyIdB = bodyId;
				def.base.localFrameA.p = p;
				def.linearHertz = 4.0f;
				def.linearDampingRatio = 0.7f;
				def.maxSpringForce = 1000.0f;
				def.maxVelocityForce = 100.0f;
				b3CreateMotorJoint( worldId, &def );
			}
			break;

			case 1:
			{
				b3SphericalJointDef def = b3DefaultSphericalJointDef();
				def.base.bodyIdA = anchorId;
				def.base.bodyIdB = bodyId;
				def.base.localFrameA.p = p;
				b3CreateSphericalJoint( worldId, &def );
			}
			break;

			case 2:
			{
				b3WeldJointDef def = b3DefaultWeldJointDef();
				def.base.bodyIdA = anchorId;
				def.base.bodyIdB = bodyId;
				def.base.localFrameA.p = p;
				b3CreateWeldJoint( worldId, &def );
			}
			break;

			default:
			{
				b3DistanceJointDef def = b3DefaultDistanceJointDef();
				def.base.bodyIdA = anchorId;
				def.base.bodyIdB = bodyId;
				def.base.localFrameA.p = p;
				def.length = 0.5f;
				def.enableSpring = true;
				def.hertz = 2.0f;
				def.dampingRatio = 0.5f;
				b3CreateDistanceJoint( worldId, &def );
			}
			break;
		}
	}

	for ( int i = 0; i < 20; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Vec3 v = b3Body_GetLinearVelocity( anchorId );
	b3Vec3 w = b3Body_GetAngularVelocity( anchorId );
	ENSURE( v.x == anchorVelocity.x && v.y == anchorVelocity.y && v.z == anchorVelocity.z );
	ENSURE( w.x == 0.0f && w.y == 0.0f && w.z == 0.0f );

	b3DestroyWorld( worldId );
	return 0;
}

int JointTest( void )
{
	RUN_SUBTEST( TestMotionLockJoint );
	RUN_SUBTEST( TestMotionLockJointLinear );
	RUN_SUBTEST( TestMotionLockJointCoupled );
	RUN_SUBTEST( TestMotionLockJointRotation );
	RUN_SUBTEST( TestMotionLockJointPlatform );
	RUN_SUBTEST( TestMotionLockJointRedundant );
#if defined( BOX3D_DOUBLE_PRECISION )
	RUN_SUBTEST( TestMotionLockJointFar );
#endif
	RUN_SUBTEST( TestParallelJoint );
	RUN_SUBTEST( TestDistanceJoint );
	RUN_SUBTEST( TestFilterJoint );
	RUN_SUBTEST( TestMotorJoint );
	RUN_SUBTEST( TestMoverJoint );
	RUN_SUBTEST( TestPogoJoint );
	RUN_SUBTEST( TestPrismaticJoint );
	RUN_SUBTEST( TestRevoluteJoint );
	RUN_SUBTEST( TestSphericalJoint );
	RUN_SUBTEST( TestSphericalMotorFrame );
	RUN_SUBTEST( TestWeldJoint );
	RUN_SUBTEST( TestWheelJoint );
	RUN_SUBTEST( TestRevoluteSpringShortWay );
	RUN_SUBTEST( TestWheelSteeringShortWay );
	RUN_SUBTEST( TestConstraintForceAndTorque );
	RUN_SUBTEST( TestCreateJointClearsContacts );
	RUN_SUBTEST( TestNonDynamicJointDisable );
	RUN_SUBTEST( TestJointReleasesLoad );
	RUN_SUBTEST( TestKinematicJointFan );

	return 0;
}
