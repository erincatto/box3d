// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "benchmarks.h"
#include "overflow_color.h"
#include "stability.h"
#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <stdio.h>

// This is a simple example of building and running a simulation
// using Box3D. Here we create a large ground box and a small dynamic
// box.
// There are no graphics for this example. Box3D is meant to be used
// with your rendering engine in your game engine.
int HelloWorld( void )
{
	// Construct a world object, which will hold and simulate the rigid bodies.
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };

	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	// Define the ground body.
	b3BodyDef groundBodyDef = b3DefaultBodyDef();
	groundBodyDef.position = (b3Vec3){ 0.0f, -10.0f, 0.0f };

	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	b3BodyId groundId = b3CreateBody( worldId, &groundBodyDef );
	ENSURE( b3Body_IsValid( groundId ) );

	// Define the ground box shape. The extents are the half-widths of the box.
	b3BoxHull groundBox = b3MakeBoxHull( 50.0f, 10.0f, 50.0f );

	// Add the box shape to the ground body.
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	// Define the dynamic body. We set its position and call the body factory.
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Vec3){ 0.0f, 4.0f, 0.0f };

	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	// Define another box shape for our dynamic body.
	b3BoxHull dynamicBox = b3MakeCubeHull( 1.0f );

	// Define the dynamic body shape
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	// Set the box density to be non-zero, so it will be dynamic.
	shapeDef.density = 1.0f;

	// Override the default friction.
	shapeDef.baseMaterial.friction = 0.3f;

	// Add the shape to the body.
	b3CreateHullShape( bodyId, &shapeDef, &dynamicBox.base );

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 4 sub-steps. This provides a high quality simulation
	// in most game scenarios.
	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;

	b3Vec3 position = b3Body_GetPosition( bodyId );
	b3Quat rotation = b3Body_GetRotation( bodyId );

	// This is our little game loop.
	for ( int i = 0; i < 90; ++i )
	{
		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		b3World_Step( worldId, timeStep, subStepCount );

		// Now print the position and angle of the body.
		position = b3Body_GetPosition( bodyId );
		rotation = b3Body_GetRotation( bodyId );

		// printf("%4.2f %4.2f %4.2f\n", position.x, position.y, b3Rot_GetAngle(rotation));
	}

	// When the world destructor is called, all bodies and joints are freed. This can
	// create orphaned ids, so be careful about your world management.
	b3DestroyWorld( worldId );

	ENSURE_SMALL( position.y - 1.00f, 0.01f );
	ENSURE_SMALL( rotation.v.x, 0.01f );
	ENSURE_SMALL( rotation.v.z, 0.01f );

	return 0;
}

int EmptyWorld( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) == true );

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 1;

	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3DestroyWorld( worldId );

	ENSURE( b3World_IsValid( worldId ) == false );

	return 0;
}

#define BODY_COUNT 10
int DestroyAllBodiesWorld( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) == true );

	int count = 0;
	bool creating = true;

	b3BodyId bodyIds[BODY_COUNT];
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BoxHull cube = b3MakeCubeHull( 0.5f );

	for ( int i = 0; i < 2 * BODY_COUNT + 10; ++i )
	{
		if ( creating )
		{
			if ( count < BODY_COUNT )
			{
				bodyIds[count] = b3CreateBody( worldId, &bodyDef );

				b3ShapeDef shapeDef = b3DefaultShapeDef();
				b3CreateHullShape( bodyIds[count], &shapeDef, &cube.base );
				count += 1;
			}
			else
			{
				creating = false;
			}
		}
		else if ( count > 0 )
		{
			b3DestroyBody( bodyIds[count - 1] );
			bodyIds[count - 1] = b3_nullBodyId;
			count -= 1;
		}

		b3World_Step( worldId, 1.0f / 60.0f, 3 );
	}

	b3Counters counters = b3World_GetCounters( worldId );
	ENSURE( counters.bodyCount == 0 );

	b3DestroyWorld( worldId );

	ENSURE( b3World_IsValid( worldId ) == false );

	return 0;
}

static int TestIsValid( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef bodyDef = b3DefaultBodyDef();

	b3BodyId bodyId1 = b3CreateBody( worldId, &bodyDef );
	ENSURE( b3Body_IsValid( bodyId1 ) == true );

	b3BodyId bodyId2 = b3CreateBody( worldId, &bodyDef );
	ENSURE( b3Body_IsValid( bodyId2 ) == true );

	b3DestroyBody( bodyId1 );
	ENSURE( b3Body_IsValid( bodyId1 ) == false );

	b3DestroyBody( bodyId2 );
	ENSURE( b3Body_IsValid( bodyId2 ) == false );

	b3DestroyWorld( worldId );

	ENSURE( b3World_IsValid( worldId ) == false );
	ENSURE( b3Body_IsValid( bodyId2 ) == false );
	ENSURE( b3Body_IsValid( bodyId1 ) == false );

	return 0;
}

#define WORLD_COUNT ( B3_MAX_WORLDS / 2 )

int TestWorldRecycle( void )
{
	_Static_assert( WORLD_COUNT > 0, "world count" );

	int count = 100;

	b3WorldId worldIds[WORLD_COUNT];

	for ( int i = 0; i < count; ++i )
	{
		b3WorldDef worldDef = b3DefaultWorldDef();
		for ( int j = 0; j < WORLD_COUNT; ++j )
		{
			worldIds[j] = b3CreateWorld( &worldDef );
			ENSURE( b3World_IsValid( worldIds[j] ) == true );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3CreateBody( worldIds[j], &bodyDef );
		}

		for ( int j = 0; j < WORLD_COUNT; ++j )
		{
			float timeStep = 1.0f / 60.0f;
			int subStepCount = 1;

			for ( int k = 0; k < 10; ++k )
			{
				b3World_Step( worldIds[j], timeStep, subStepCount );
			}
		}

		for ( int j = WORLD_COUNT - 1; j >= 0; --j )
		{
			b3DestroyWorld( worldIds[j] );
			ENSURE( b3World_IsValid( worldIds[j] ) == false );
			worldIds[j] = b3_nullWorldId;
		}
	}

	return 0;
}

static bool CustomFilter( b3ShapeId shapeIdA, b3ShapeId shapeIdB, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	ENSURE( context == NULL );
	return true;
}

static bool PreSolveStatic( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3Vec3 point, b3Vec3 normal, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	(void)point;
	(void)normal;
	ENSURE( context == NULL );
	return false;
}

// This test is here to ensure all API functions link correctly.
int TestWorldCoverage( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();

	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	b3World_EnableSleeping( worldId, true );
	b3World_EnableSleeping( worldId, false );
	bool flag = b3World_IsSleepingEnabled( worldId );
	ENSURE( flag == false );

	b3World_EnableContinuous( worldId, false );
	b3World_EnableContinuous( worldId, true );
	flag = b3World_IsContinuousEnabled( worldId );
	ENSURE( flag == true );

	b3World_SetRestitutionThreshold( worldId, 0.0f );
	b3World_SetRestitutionThreshold( worldId, 2.0f );
	float value = b3World_GetRestitutionThreshold( worldId );
	ENSURE( value == 2.0f );

	b3World_SetHitEventThreshold( worldId, 0.0f );
	b3World_SetHitEventThreshold( worldId, 100.0f );
	value = b3World_GetHitEventThreshold( worldId );
	ENSURE( value == 100.0f );

	b3World_SetCustomFilterCallback( worldId, CustomFilter, NULL );
	b3World_SetPreSolveCallback( worldId, PreSolveStatic, NULL );

	b3Vec3 g = { 1.0f, 2.0f };
	b3World_SetGravity( worldId, g );
	b3Vec3 v = b3World_GetGravity( worldId );
	ENSURE( v.x == g.x );
	ENSURE( v.y == g.y );

	b3ExplosionDef explosionDef = b3DefaultExplosionDef();
	b3World_Explode( worldId, &explosionDef );

	b3World_SetContactTuning( worldId, 10.0f, 2.0f, 4.0f );

	b3World_SetMaximumLinearSpeed( worldId, 10.0f );
	value = b3World_GetMaximumLinearSpeed( worldId );
	ENSURE( value == 10.0f );

	b3World_EnableWarmStarting( worldId, true );
	flag = b3World_IsWarmStartingEnabled( worldId );
	ENSURE( flag == true );

	int count = b3World_GetAwakeBodyCount( worldId );
	ENSURE( count == 0 );

	b3World_SetUserData( worldId, &value );
	void* userData = b3World_GetUserData( worldId );
	ENSURE( userData == &value );

	b3World_Step( worldId, 1.0f, 1 );

	b3DestroyWorld( worldId );

	return 0;
}

static int TestSensor( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Wall from x = 1 to x = 2
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Vec3){ 1.5f, 11.0f, 0.0f };
	b3BodyId wallId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 0.5f, 10.0f, 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableSensorEvents = true;
	b3CreateHullShape( wallId, &shapeDef, &box.base );

	// Bullet fired towards the wall
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.isBullet = true;
	bodyDef.gravityScale = 0.0f;
	bodyDef.position = (b3Vec3){ 7.39814f, 4.0f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ -20.0f, 0.0f, 0.0f };
	b3BodyId bulletId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.isSensor = true;
	shapeDef.enableSensorEvents = true;
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.1f };
	b3CreateSphereShape( bulletId, &shapeDef, &sphere );

	int beginCount = 0;
	int endCount = 0;

	while ( true )
	{
		float timeStep = 1.0f / 60.0f;
		int subStepCount = 4;
		b3World_Step( worldId, timeStep, subStepCount );

		b3Vec3 bulletPos = b3Body_GetPosition( bulletId );
		// printf( "Bullet pos: %g %g\n", bulletPos.x, bulletPos.y );

		b3SensorEvents events = b3World_GetSensorEvents( worldId );

		if ( events.beginCount > 0 )
		{
			beginCount += 1;
		}

		if ( events.endCount > 0 )
		{
			endCount += 1;
		}

		if ( bulletPos.x < -1.0f )
		{
			break;
		}
	}

	b3DestroyWorld( worldId );

	ENSURE( beginCount == 1 );
	ENSURE( endCount == 1 );

	return 0;
}

static int TestContactEvents( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Static ground
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Vec3){ 0.0f, -0.5f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull groundBox = b3MakeBoxHull( 10.0f, 0.5f, 10.0f );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3ShapeId groundShapeId = b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	// Dynamic sphere dropped onto the ground; restitution causes it to bounce so we get end events
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Vec3){ 0.0f, 5.0f, 0.0f };
	b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enableContactEvents = true;
	shapeDef.baseMaterial.restitution = 0.6f;
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3ShapeId sphereShapeId = b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );

	int beginCount = 0;
	int endCount = 0;
	bool idsChecked = false;

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		b3ContactEvents events = b3World_GetContactEvents( worldId );

		if ( events.beginCount > 0 && idsChecked == false )
		{
			b3ContactBeginTouchEvent be = events.beginEvents[0];
			bool aIsSphere = B3_ID_EQUALS( be.shapeIdA, sphereShapeId );
			bool bIsSphere = B3_ID_EQUALS( be.shapeIdB, sphereShapeId );
			bool aIsGround = B3_ID_EQUALS( be.shapeIdA, groundShapeId );
			bool bIsGround = B3_ID_EQUALS( be.shapeIdB, groundShapeId );
			ENSURE( ( aIsSphere && bIsGround ) || ( aIsGround && bIsSphere ) );
			ENSURE( b3Contact_IsValid( be.contactId ) );
			idsChecked = true;
		}

		beginCount += events.beginCount;
		endCount += events.endCount;
	}

	b3DestroyWorld( worldId );

	ENSURE( idsChecked );
	ENSURE( beginCount >= 1 );
	ENSURE( endCount >= 1 );

	return 0;
}

static int TestHitEvents( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.hitEventThreshold = 1.0f;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Static ground
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Vec3){ 0.0f, -0.5f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull groundBox = b3MakeBoxHull( 10.0f, 0.5f, 10.0f );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	// Sphere driven into the ground fast enough to clear the hit threshold
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.gravityScale = 0.0f;
	bodyDef.position = (b3Vec3){ 0.0f, 2.0f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ 0.0f, -30.0f, 0.0f };
	b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enableHitEvents = true;
	shapeDef.baseMaterial.userMaterialId = 7;
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );

	int hitCount = 0;
	float capturedSpeed = 0.0f;
	uint64_t capturedMaterialA = 0;
	uint64_t capturedMaterialB = 0;
	b3Vec3 capturedNormal = { 0.0f, 0.0f, 0.0f };

	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		b3ContactEvents events = b3World_GetContactEvents( worldId );
		if ( events.hitCount > 0 && hitCount == 0 )
		{
			b3ContactHitEvent hit = events.hitEvents[0];
			capturedSpeed = hit.approachSpeed;
			capturedNormal = hit.normal;
			capturedMaterialA = hit.userMaterialIdA;
			capturedMaterialB = hit.userMaterialIdB;
		}

		hitCount += events.hitCount;
	}

	b3DestroyWorld( worldId );

	ENSURE( hitCount >= 1 );
	ENSURE( capturedSpeed > 1.0f );
	// Head-on vertical impact: normal lies along Y
	ENSURE_SMALL( capturedNormal.x, 0.01f );
	ENSURE_SMALL( capturedNormal.z, 0.01f );
	// One side of the contact carries the sphere's user material
	ENSURE( capturedMaterialA == 7 || capturedMaterialB == 7 );

	return 0;
}

// Hit-event material lookup must respect the compound child that participated in the
// contact. Two children with distinct userMaterialIds at separated positions, dropped
// sphere strikes one specifically. Without the fix, both children would report
// materials[0] and the strike on hull 1 would be misattributed.
static int TestCompoundHitEvents( void )
{
	const uint64_t kHullMaterialA = 11;
	const uint64_t kHullMaterialB = 22;
	const uint64_t kSphereMaterial = 99;
	const float kHullCenterX = 3.0f;

	for ( int side = 0; side < 2; ++side )
	{
		uint64_t expectedHullMaterial = ( side == 0 ) ? kHullMaterialA : kHullMaterialB;
		float spawnX = ( side == 0 ) ? -kHullCenterX : kHullCenterX;

		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.hitEventThreshold = 1.0f;
		b3WorldId worldId = b3CreateWorld( &worldDef );

		// Build a compound with two hulls at opposite x positions, distinct userMaterialIds
		b3BoxHull boxA = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
		b3BoxHull boxB = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );

		b3SurfaceMaterial matA = b3DefaultSurfaceMaterial();
		matA.userMaterialId = kHullMaterialA;

		b3SurfaceMaterial matB = b3DefaultSurfaceMaterial();
		matB.userMaterialId = kHullMaterialB;

		b3CompoundHullDef hulls[2];
		hulls[0].hull = &boxA.base;
		hulls[0].transform = (b3Transform){ { -kHullCenterX, 0.0f, 0.0f }, b3Quat_identity };
		hulls[0].material = matA;
		hulls[1].hull = &boxB.base;
		hulls[1].transform = (b3Transform){ { kHullCenterX, 0.0f, 0.0f }, b3Quat_identity };
		hulls[1].material = matB;

		b3CompoundDef compoundDef = { 0 };
		compoundDef.hulls = hulls;
		compoundDef.hullCount = 2;
		b3Compound* compound = b3CreateCompound( &compoundDef );
		ENSURE( compound != NULL );

		// Static body holds the compound
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		b3BodyId compoundBodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef compoundShapeDef = b3DefaultShapeDef();
		b3CreateCompoundShape( compoundBodyId, &compoundShapeDef, compound );

		// Sphere driven straight down onto the chosen child
		bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.gravityScale = 0.0f;
		bodyDef.position = (b3Vec3){ spawnX, 3.0f, 0.0f };
		bodyDef.linearVelocity = (b3Vec3){ 0.0f, -30.0f, 0.0f };
		b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef sphereShapeDef = b3DefaultShapeDef();
		sphereShapeDef.density = 1.0f;
		sphereShapeDef.enableHitEvents = true;
		sphereShapeDef.baseMaterial.userMaterialId = kSphereMaterial;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		b3CreateSphereShape( sphereBodyId, &sphereShapeDef, &sphere );

		int hitCount = 0;
		uint64_t capturedMaterialA = 0;
		uint64_t capturedMaterialB = 0;

		for ( int i = 0; i < 30; ++i )
		{
			b3World_Step( worldId, 1.0f / 60.0f, 4 );

			b3ContactEvents events = b3World_GetContactEvents( worldId );
			if ( events.hitCount > 0 && hitCount == 0 )
			{
				b3ContactHitEvent hit = events.hitEvents[0];
				capturedMaterialA = hit.userMaterialIdA;
				capturedMaterialB = hit.userMaterialIdB;
			}
			hitCount += events.hitCount;
		}

		b3DestroyWorld( worldId );
		b3DestroyCompound( compound );

		ENSURE( hitCount >= 1 );
		// Sphere material on one side
		ENSURE( capturedMaterialA == kSphereMaterial || capturedMaterialB == kSphereMaterial );
		// Struck compound child's material on the other side. The pre-fix code returned
		// materials[0] (kHullMaterialA) for both sides, so a strike on the +x child would fail.
		ENSURE( capturedMaterialA == expectedHullMaterial || capturedMaterialB == expectedHullMaterial );
	}

	return 0;
}

static int TestSetWorkerCount( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );
	ENSURE( b3World_GetWorkerCount( worldId ) == 1 );

	CreateJunkyard( worldId );
	StepJunkyard( worldId, 1 );

	b3World_SetWorkerCount( worldId, 4 );
	ENSURE( b3World_GetWorkerCount( worldId ) == 4 );

	StepJunkyard( worldId, 2 );

	b3World_SetWorkerCount( worldId, 4 );
	ENSURE( b3World_GetWorkerCount( worldId ) == 4 );

	StepJunkyard( worldId, 3 );

	b3World_SetWorkerCount( worldId, 0 );
	ENSURE( b3World_GetWorkerCount( worldId ) == 1 );

	StepJunkyard( worldId, 4 );

	b3World_SetWorkerCount( worldId, -5 );
	ENSURE( b3World_GetWorkerCount( worldId ) == 1 );

	StepJunkyard( worldId, 5 );

	b3World_SetWorkerCount( worldId, B3_MAX_WORKERS + 10 );
	ENSURE( b3World_GetWorkerCount( worldId ) == B3_MAX_WORKERS );

	StepJunkyard( worldId, 2 );

	b3DestroyWorld( worldId );

	return 0;
}

// This tests continuous collision and mesh contact stability
static int TestMeshDrop( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	// todo GetCoreCount
	worldDef.workerCount = 4;

	b3WorldId worldId = b3CreateWorld( &worldDef );

	MeshDropData data = CreateMeshDrop( worldId );

	float timeStep = 1.0f / 60.0f;

	int stepIndex = 0;
	int stepLimit = 400;

	while ( stepIndex < stepLimit )
	{
		int subStepCount = 4;
		b3World_Step( worldId, timeStep, subStepCount );

		b3BodyEvents events = b3World_GetBodyEvents( worldId );
		if (events.moveCount == 0)
		{
			// All bodies sleeping
			break;
		}

		stepIndex += 1;
	}

	printf( "  TestMeshDrop stepIndex = %d\n", stepIndex );

	DestroyMeshDrop( &data );
	b3DestroyWorld( worldId );

	ENSURE( stepIndex < stepLimit );

	return 0;
}

// Verifies the b3*_Overflow solver path. The scene puts >B3_DYNAMIC_COLOR_COUNT
// dyn-dyn contacts on a single hub body so several land in the overflow color.
// The new Prepare/Store contactId-pairing asserts in contact_solver.c fire here
// if Store reads constraints from the wrong (base, spans) pair, so the test
// catches the bug even though world state ends up plausible (memory layout is
// stable within a build, so determinism alone is not a witness).
static int TestOverflowColorPile( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	OverflowColorPileData data = CreateOverflowColorPile( worldId );
	(void)data;

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;

	// One step would be enough to trip the asserts, but several steps also
	// exercise the warm-start path (Store -> manifold impulse -> Prepare).
	int stepCount = 10;
	for ( int i = 0; i < stepCount; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	// Confirm the scene actually populated the overflow color. Without this,
	// a future change to graph coloring could silently turn the test into a
	// no-op.
	b3Counters counters = b3World_GetCounters( worldId );
	int overflowContacts = counters.colorCounts[B3_GRAPH_COLOR_COUNT - 1];

	b3DestroyWorld( worldId );

	ENSURE( overflowContacts > 0 );
	return 0;
}

// Exposes that b3Body_EnableSleep mutates body->flags but never syncs
// bodySim->flags / bodyState->flags. When a body created with
// enableSleep=false is later flipped on, body->flags gains b3_enableSleep
// but bodySim/bodyState do not. The flag-sync assertion in
// b3ValidateSolverSets then fires on the next world step.
//
// Under Debug + BOX3D_VALIDATE this crashes; after b3Body_EnableSleep
// calls b3SyncBodyFlags the test runs cleanly.
static int EnableSleepFlagSyncTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	ENSURE( b3Body_IsSleepEnabled( bodyId ) == false );

	b3Body_EnableSleep( bodyId, true );
	ENSURE( b3Body_IsSleepEnabled( bodyId ) == true );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	b3DestroyWorld( worldId );
	return 0;
}

// Exposes that b3Body_SetBullet writes only bodySim->flags without
// touching body->flags, while b3SyncBodyFlags overwrites bodySim from
// body->flags. Any subsequent b3Body_SetMotionLocks or b3Body_SetType
// silently wipes (or re-asserts) the bullet bit.
//
// Reproducer A: create with isBullet=false, SetBullet(true), then
// SetMotionLocks. b3Body_IsBullet returns false today (wiped); should be
// true after fix.
//
// Reproducer B: create with isBullet=true, SetBullet(false), then
// SetMotionLocks. b3Body_IsBullet returns true today (re-asserted); should
// be false after fix.
static int SetBulletDriftTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = false;
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

		ENSURE( b3Body_IsBullet( bodyId ) == false );

		b3Body_SetBullet( bodyId, true );
		ENSURE( b3Body_IsBullet( bodyId ) == true );

		b3MotionLocks locks = { 0 };
		locks.linearX = true;
		b3Body_SetMotionLocks( bodyId, locks );

		ENSURE( b3Body_IsBullet( bodyId ) == true );
	}

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = true;
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

		ENSURE( b3Body_IsBullet( bodyId ) == true );

		b3Body_SetBullet( bodyId, false );
		ENSURE( b3Body_IsBullet( bodyId ) == false );

		b3MotionLocks locks = { 0 };
		locks.linearX = true;
		b3Body_SetMotionLocks( bodyId, locks );

		ENSURE( b3Body_IsBullet( bodyId ) == false );
	}

	b3DestroyWorld( worldId );
	return 0;
}

// Regression: b3Body_EnableSleep used to set world->locked = true before checking
// for a no-op change, then early-return without unlocking. The next mutator call
// would then trip the assert in b3GetUnlockedWorld.
static int EnableSleepNoopUnlockTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = true;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	// No-op: enableSleep is already true. Must not leak the world lock.
	b3Body_EnableSleep( bodyId, true );

	// Would assert in b3GetUnlockedWorld if the lock had leaked.
	b3Body_EnableSleep( bodyId, false );
	ENSURE( b3Body_IsSleepEnabled( bodyId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

static int EnableContactRecyclingTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	// Default is enabled
	b3BodyId bodyA = b3CreateBody( worldId, &bodyDef );
	ENSURE( b3Body_IsContactRecyclingEnabled( bodyA ) == true );

	b3Body_EnableContactRecycling( bodyA, false );
	ENSURE( b3Body_IsContactRecyclingEnabled( bodyA ) == false );

	b3Body_EnableContactRecycling( bodyA, true );
	ENSURE( b3Body_IsContactRecyclingEnabled( bodyA ) == true );

	// Per-def opt-out at creation
	bodyDef.enableContactRecycling = false;
	b3BodyId bodyB = b3CreateBody( worldId, &bodyDef );
	ENSURE( b3Body_IsContactRecyclingEnabled( bodyB ) == false );

	// Stepping after toggling must not trip the flag-sync validator
	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	b3DestroyWorld( worldId );
	return 0;
}

int WorldTest( void )
{
	RUN_SUBTEST( HelloWorld );
	RUN_SUBTEST( EmptyWorld );
	RUN_SUBTEST( DestroyAllBodiesWorld );
	RUN_SUBTEST( TestIsValid );
	RUN_SUBTEST( TestWorldRecycle );
	RUN_SUBTEST( TestWorldCoverage );
	RUN_SUBTEST( TestSensor );
	RUN_SUBTEST( TestContactEvents );
	RUN_SUBTEST( TestHitEvents );
	RUN_SUBTEST( TestCompoundHitEvents );
	RUN_SUBTEST( TestMeshDrop );
	RUN_SUBTEST( TestOverflowColorPile );
	RUN_SUBTEST( SetBulletDriftTest );
	RUN_SUBTEST( EnableSleepFlagSyncTest );
	RUN_SUBTEST( EnableSleepNoopUnlockTest );
	RUN_SUBTEST( EnableContactRecyclingTest );
	RUN_SUBTEST( TestSetWorkerCount );

	return 0;
}
