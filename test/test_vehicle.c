// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "wagon.h"

#include "box3d/box3d.h"

#include <math.h>
#include <stdio.h>

// Smoke tests for the tire joint and the shared wagon. The wagon tuning is still being driven and
// changed by hand, so nothing here depends on its values.

static b3WorldId CreateVehicleWorld( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	return b3CreateWorld( &worldDef );
}

// A static slab whose top passes through the origin. Positive slope descends along +x.
static b3BodyId CreateGround( b3WorldId worldId, float slope )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, -atanf( slope ) );
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeOffsetBoxHull( 200.0f, 1.0f, 200.0f, (b3Vec3){ 0.0f, -1.0f, 0.0f } );
	b3CreateHullShape( groundId, &shapeDef, &box.base );
	return groundId;
}

static Wagon CreateTestWagon( b3WorldId worldId, float slope )
{
	WagonDef def = GetDefaultWagonDef();
	def.position = (b3Pos){ 0.0f, 0.1f, 0.0f };
	def.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, -atanf( slope ) );
	return CreateWagon( worldId, &def );
}

static void StepWagon( b3WorldId worldId, Wagon* wagon, WagonInput input )
{
	float timeStep = 1.0f / 60.0f;
	UpdateWagon( wagon, input, timeStep );
	b3World_Step( worldId, timeStep, 4 );
	UpdateWagonResults( wagon, timeStep );
}

static bool AllGrounded( const Wagon* wagon )
{
	for ( int i = 0; i < e_wagonWheelCount; ++i )
	{
		if ( wagon->wheels[i].grounded == false || wagon->wheels[i].normalForce <= 0.0f )
		{
			return false;
		}
	}
	return true;
}

static int TireJointApi( void )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	b3BodyId groundId = CreateGround( worldId, 0.0f );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 1.0f, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3TireJointDef def = b3DefaultTireJointDef();
	def.base.bodyIdA = groundId;
	def.base.bodyIdB = bodyId;
	def.base.localFrameB.q = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI );
	def.base.collideConnected = true;
	b3JointId jointId = b3CreateTireJoint( worldId, &def );

	ENSURE( b3Joint_IsValid( jointId ) );
	ENSURE( b3Joint_GetType( jointId ) == b3_tireJoint );

	b3Vec3 tilted = b3Normalize( (b3Vec3){ 0.1f, 1.0f, 0.0f } );
	b3TireJoint_SetContact( jointId, (b3Vec3){ 0.2f, 0.0f, 0.0f }, tilted );
	ENSURE_SMALL( b3Length( b3Sub( b3TireJoint_GetNormal( jointId ), tilted ) ), 1.0e-6f );
	ENSURE( b3Joint_GetLocalFrameA( jointId ).p.x == 0.2f );

	b3TireJoint_SetBrakeForce( jointId, 50.0f );
	ENSURE( b3TireJoint_GetBrakeForce( jointId ) == 50.0f );

	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	ENSURE( b3IsValidVec3( b3Joint_GetConstraintForce( jointId ) ) );

	b3DestroyJoint( jointId, false );
	ENSURE( b3Joint_IsValid( jointId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

// Dropped on flat ground the wagon lands on all four wheels and goes to sleep
static int SettleFlat( void )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateGround( worldId, 0.0f );
	Wagon wagon = CreateTestWagon( worldId, 0.0f );

	WagonInput input = { e_wagonHalt, 0.0f, true };
	bool grounded = false;
	int sleepStep = -1;
	for ( int i = 0; i < 10 * 60 && sleepStep < 0; ++i )
	{
		StepWagon( worldId, &wagon, input );
		if ( b3Body_IsAwake( wagon.chassisId ) )
		{
			grounded = AllGrounded( &wagon );
		}
		else
		{
			sleepStep = i;
		}
	}

	printf( "  settle flat: grounded %d, sleep step %d\n", grounded ? 1 : 0, sleepStep );
	ENSURE( grounded );
	ENSURE( sleepStep >= 0 );

	DestroyWagon( &wagon );
	b3DestroyWorld( worldId );
	return 0;
}

// Parked with the brake on a 10 percent slope the wagon goes to sleep
static int SlopeSleep( void )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateGround( worldId, 0.1f );
	Wagon wagon = CreateTestWagon( worldId, 0.1f );

	WagonInput input = { e_wagonHalt, 0.0f, true };
	int sleepStep = -1;
	for ( int i = 0; i < 10 * 60 && sleepStep < 0; ++i )
	{
		StepWagon( worldId, &wagon, input );
		if ( b3Body_IsAwake( wagon.chassisId ) == false )
		{
			sleepStep = i;
		}
	}

	printf( "  slope sleep: sleep step %d\n", sleepStep );
	ENSURE( sleepStep >= 0 );

	DestroyWagon( &wagon );
	b3DestroyWorld( worldId );
	return 0;
}

static uint32_t HashBytes( uint32_t hash, const void* data, int size )
{
	const uint8_t* bytes = data;
	for ( int i = 0; i < size; ++i )
	{
		hash = ( hash ^ bytes[i] ) * 16777619u;
	}
	return hash;
}

// The wagon drives over a curb under its left wheels and through light crates while steering
static uint32_t RunDeterminismScene( int workerCount )
{
	b3WorldId worldId = CreateVehicleWorld( workerCount );
	CreateGround( worldId, 0.0f );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 60.0f, 0.05f, -5.0f };
		b3BodyId curbId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 50.0f, 0.05f, 5.0f );
		b3CreateHullShape( curbId, &shapeDef, &box.base );
	}

	for ( int i = 0; i < 3; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 4.0f + 1.5f * (float)i, 0.08f, 0.725f };
		b3BodyId boxId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 150.0f;
		b3BoxHull box = b3MakeBoxHull( 0.3f, 0.08f, 0.3f );
		b3CreateHullShape( boxId, &shapeDef, &box.base );
	}

	Wagon wagon = CreateTestWagon( worldId, 0.0f );

	WagonInput input = { e_wagonTrot, 0.0f, false };
	uint32_t hash = 2166136261u;
	for ( int i = 0; i < 8 * 60; ++i )
	{
		input.steer = i < 240 ? 0.05f : -0.05f;
		StepWagon( worldId, &wagon, input );

		b3WorldTransform transform = b3Body_GetTransform( wagon.chassisId );
		b3Vec3 v = b3Body_GetLinearVelocity( wagon.chassisId );
		b3Vec3 w = b3Body_GetAngularVelocity( wagon.chassisId );
		hash = HashBytes( hash, &transform, sizeof( transform ) );
		hash = HashBytes( hash, &v, sizeof( v ) );
		hash = HashBytes( hash, &w, sizeof( w ) );
	}

	DestroyWagon( &wagon );
	b3DestroyWorld( worldId );
	return hash;
}

static int MultithreadedDeterminism( void )
{
	uint32_t reference = RunDeterminismScene( 1 );
	printf( "  determinism: workers 1 hash 0x%08X\n", reference );
	for ( int workerCount = 2; workerCount <= 4; ++workerCount )
	{
		uint32_t hash = RunDeterminismScene( workerCount );
		printf( "  determinism: workers %d hash 0x%08X\n", workerCount, hash );
		ENSURE( hash == reference );
	}

	ENSURE( RunDeterminismScene( 1 ) == reference );
	return 0;
}

int VehicleTest( void )
{
	RUN_SUBTEST( TireJointApi );
	RUN_SUBTEST( SettleFlat );
	RUN_SUBTEST( SlopeSleep );
	RUN_SUBTEST( MultithreadedDeterminism );
	return 0;
}
