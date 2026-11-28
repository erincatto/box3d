// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "wagon.h"

#include "box3d/box3d.h"

#include <float.h>
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

	// The front wheels rest at full extension, so one reads no load now and then. Grounded means every
	// wheel carried load in the last few steps before sleep.
	WagonInput input = { e_wagonHalt, 0.0f, true };
	bool grounded = false;
	int lastLoaded[e_wagonWheelCount] = { -100, -100, -100, -100 };
	int sleepStep = -1;
	for ( int i = 0; i < 10 * 60 && sleepStep < 0; ++i )
	{
		StepWagon( worldId, &wagon, input );
		if ( b3Body_IsAwake( wagon.chassisId ) )
		{
			grounded = true;
			for ( int j = 0; j < e_wagonWheelCount; ++j )
			{
				const WagonWheel* wheel = wagon.wheels + j;
				lastLoaded[j] = wheel->grounded && wheel->normalForce > 0.0f ? i : lastLoaded[j];
				grounded = grounded && wheel->grounded && i - lastLoaded[j] < 10;
			}
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

static void CreateTree( b3WorldId worldId, float x, float z, float radius )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = (b3Pos){ x, 0.0f, z };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3HullData* hull = b3CreateCylinder( 8.0f, radius, 0.0f, 16 );
	b3CreateHullShape( bodyId, &shapeDef, hull );
	b3DestroyHull( hull );
}

// Gap between a capsule and a tall tree in the horizontal plane, negative when they overlap. The
// cylinder hull is a polygon inside the radius, so this is slightly pessimistic.
static float CapsuleTreeGap( b3ShapeId shapeId, float x, float z, float radius )
{
	b3WorldTransform transform = b3Body_GetTransform( b3Shape_GetBody( shapeId ) );
	b3Capsule capsule = b3Shape_GetCapsule( shapeId );
	b3Pos p1 = b3TransformWorldPoint( transform, capsule.center1 );
	b3Pos p2 = b3TransformWorldPoint( transform, capsule.center2 );

	float ax = (float)p1.x - x, az = (float)p1.z - z;
	float dx = (float)( p2.x - p1.x ), dz = (float)( p2.z - p1.z );
	float lengthSquared = dx * dx + dz * dz;
	float t = lengthSquared > 0.0f ? b3ClampFloat( -( ax * dx + az * dz ) / lengthSquared, 0.0f, 1.0f ) : 0.0f;
	float cx = ax + t * dx, cz = az + t * dz;
	return sqrtf( cx * cx + cz * cz ) - capsule.radius - radius;
}

// Gap between a hull and a tall tree down to a depth of the tolerance. GJK reports zero for any
// overlap, so it measures against the tree shrunk by the tolerance.
static float HullTreeGap( b3ShapeId shapeId, float x, float z, float radius )
{
	float tolerance = 0.1f;
	const b3HullData* hull = b3Shape_GetHull( shapeId );
	b3WorldTransform transform = b3Body_GetTransform( b3Shape_GetBody( shapeId ) );
	b3Vec3 axis[2] = { { 0.0f, -10.0f, 0.0f }, { 0.0f, 10.0f, 0.0f } };

	b3DistanceInput input = { 0 };
	input.proxyA = (b3ShapeProxy){ b3GetHullPoints( hull ), hull->vertexCount, 0.0f };
	input.proxyB = (b3ShapeProxy){ axis, 2, radius - tolerance };
	input.transform = b3InvMulWorldTransforms( transform, (b3WorldTransform){ { x, 0.0f, z }, b3Quat_identity } );
	input.useRadii = true;
	b3SimplexCache cache = { 0 };
	b3DistanceOutput output = b3ShapeDistance( &input, &cache, NULL, 0 );
	return output.distance - tolerance;
}

// A fast approach can close past the speculative margin in one step and sink in for that step
// before the contact pushes out
#define MAX_TREE_DEPTH 0.05f

typedef struct TreeRun
{
	// Step the whole wagon cleared the tree, or -1
	int passStep;
	float minHorseGap;
	float minHullGap;
	float maxHorseOffset;
	float minUp;
	int horseContactSteps;
	int hullContactSteps;
	bool asleep;
} TreeRun;

// The wagon starts at the origin heading along +x toward a tree. After the drive it halts with the
// brake on for the rest of the run.
static TreeRun DriveAtTree( float treeZ, float treeRadius, WagonGait gait, int driveSteps, int totalSteps )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateGround( worldId, 0.0f );
	float treeX = 12.0f;
	CreateTree( worldId, treeX, treeZ, treeRadius );
	Wagon wagon = CreateTestWagon( worldId, 0.0f );

	TreeRun run = { -1, FLT_MAX, FLT_MAX, 0.0f, 1.0f, 0, 0, false };
	float touch = 0.5f * B3_SPECULATIVE_DISTANCE;
	for ( int i = 0; i < totalSteps; ++i )
	{
		WagonInput input = { gait, 0.0f, false };
		if ( i >= driveSteps )
		{
			input = (WagonInput){ e_wagonHalt, 0.0f, true };
		}

		StepWagon( worldId, &wagon, input );

		float horseGap = CapsuleTreeGap( wagon.horseShapeId, treeX, treeZ, treeRadius );
		float hullGap = HullTreeGap( wagon.chassisShapeId, treeX, treeZ, treeRadius );
		run.minHorseGap = b3MinFloat( run.minHorseGap, horseGap );
		run.minHullGap = b3MinFloat( run.minHullGap, hullGap );
		run.horseContactSteps += horseGap < touch ? 1 : 0;
		run.hullContactSteps += hullGap < touch ? 1 : 0;
		// The wagon starts in the air and the weightless horse lags the drop
		if ( i >= 60 )
		{
			run.maxHorseOffset = b3MaxFloat( run.maxHorseOffset, wagon.horseOffset );
		}

		b3Vec3 up = b3RotateVector( b3Body_GetRotation( wagon.chassisId ), b3Vec3_axisY );
		run.minUp = b3MinFloat( run.minUp, up.y );

		// The back of the wagon
		b3Pos back = b3Body_GetWorldPoint( wagon.chassisId, (b3Vec3){ -1.6f, 0.0f, 0.0f } );
		if ( run.passStep < 0 && (float)back.x > treeX + treeRadius )
		{
			run.passStep = i;
		}
	}

	run.asleep = b3Body_IsAwake( wagon.chassisId ) == false && b3Body_IsAwake( wagon.horseId ) == false;

	DestroyWagon( &wagon );
	b3DestroyWorld( worldId );
	return run;
}

static void PrintTreeRun( const char* name, TreeRun run )
{
	printf( "  %s: pass step %d, gap horse %.4f hull %.4f, contact steps horse %d hull %d, horse offset %.4f, "
			"min up %.3f, asleep %d\n",
			name, run.passStep, run.minHorseGap, run.minHullGap, run.horseContactSteps, run.hullContactSteps,
			run.maxHorseOffset, run.minUp, run.asleep ? 1 : 0 );
}

// The horse meets a trunk off center. The shafts turn the front gear away and the wagon slides past.
static int TreeGlance( void )
{
	TreeRun run = DriveAtTree( 0.5f, 0.2f, e_wagonTrot, 15 * 60, 25 * 60 );
	PrintTreeRun( "tree glance", run );
	ENSURE( run.horseContactSteps > 0 );
	ENSURE( run.passStep >= 0 );
	ENSURE( run.minHorseGap > -MAX_TREE_DEPTH );
	ENSURE( run.minHullGap > -MAX_TREE_DEPTH );
	ENSURE( run.maxHorseOffset < 0.05f );
	ENSURE( run.minUp > 0.9f );
	ENSURE( run.asleep );
	return 0;
}

// A trunk outside the horse slides along the side of the hull
static int TreeSideSwipe( void )
{
	TreeRun run = DriveAtTree( 1.0f, 0.3f, e_wagonTrot, 15 * 60, 15 * 60 );
	PrintTreeRun( "tree side swipe", run );
	ENSURE( run.horseContactSteps == 0 );
	ENSURE( run.hullContactSteps > 0 );
	ENSURE( run.passStep >= 0 );
	ENSURE( run.minHullGap > -MAX_TREE_DEPTH );
	ENSURE( run.minUp > 0.9f );
	return 0;
}

// Dead center the horse stalls against the trunk, straining until spent. It stays out of the trunk
// and the shafts hold. Halted and braked against the trunk the wagon goes to sleep.
static int TreeHeadOn( void )
{
	TreeRun run = DriveAtTree( 0.0f, 0.3f, e_wagonWalk, 15 * 60, 25 * 60 );
	PrintTreeRun( "tree head on", run );
	ENSURE( run.horseContactSteps > 0 );
	ENSURE( run.minHorseGap > -MAX_TREE_DEPTH );
	ENSURE( run.maxHorseOffset < 0.05f );
	ENSURE( run.minUp > 0.9f );
	ENSURE( run.asleep );
	return 0;
}

// A gallop into a trunk. The light horse is squeezed between the wagon and the trunk and sinks in
// for a few steps, so this only checks the wagon stays upright and settles.
static int TreeCrash( void )
{
	TreeRun run = DriveAtTree( 0.0f, 0.3f, e_wagonGallop, 10 * 60, 20 * 60 );
	PrintTreeRun( "tree crash", run );
	ENSURE( run.horseContactSteps > 0 );
	ENSURE( run.minUp > 0.9f );
	ENSURE( run.asleep );
	return 0;
}

// The leg force along the axes the shafts lock, as a multiple of the wagon weight. Anything here is the
// world holding the wagon through the horse.
static float LegDrag( const Wagon* wagon )
{
	b3Vec3 force = b3Joint_GetConstraintForce( wagon->legJointId );
	b3Quat q = b3Body_GetRotation( wagon->chassisId );
	float forward = b3Dot( force, b3RotateVector( q, b3Vec3_axisX ) );
	float lateral = b3Dot( force, b3RotateVector( q, b3Vec3_axisZ ) );
	return sqrtf( forward * forward + lateral * lateral ) / wagon->weight;
}

// A deck with its top at the given height that ends in a drop to the ground at x = 10
static void CreateLedge( b3WorldId worldId, float height )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = (b3Pos){ -10.0f, 0.5f * height, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( 20.0f, 0.5f * height, 5.0f );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );
}

// The wagon canters off the end of a deck. The horse floats into the drop instead of pitching down to
// the ground below or tucking under the bed, and its legs do not hold the wagon back while the wagon is
// still on the deck. Once the wagon tips over the edge the chassis turns within a step and the legs
// pass it a little load.
static int LedgeDrop( void )
{
	float height = 1.5f;
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateGround( worldId, 0.0f );
	CreateLedge( worldId, height );

	WagonDef def = GetDefaultWagonDef();
	def.position = (b3Pos){ -5.0f, height + 0.1f, 0.0f };
	Wagon wagon = CreateWagon( worldId, &def );

	float minPitch = 0.0f, maxPitch = 0.0f, maxDrop = 0.0f, deckDrag = 0.0f, fallDrag = 0.0f;
	for ( int i = 0; i < 6 * 60; ++i )
	{
		StepWagon( worldId, &wagon, (WagonInput){ e_wagonCanter, 0.0f, false } );

		// The wagon starts in the air
		if ( i < 60 )
		{
			continue;
		}

		b3WorldTransform horse = b3Body_GetTransform( wagon.horseId );
		b3Vec3 forward = b3RotateVector( horse.q, b3Vec3_axisX );
		float pitch = asinf( b3ClampFloat( forward.y, -1.0f, 1.0f ) );
		minPitch = b3MinFloat( minPitch, pitch );
		maxPitch = b3MaxFloat( maxPitch, pitch );

		b3Pos kingbolt = GetWagonKingbolt( &wagon );
		maxDrop = b3MaxFloat( maxDrop, (float)( kingbolt.y - horse.p.y ) );

		if ( kingbolt.x < 9.5f )
		{
			deckDrag = b3MaxFloat( deckDrag, LegDrag( &wagon ) );
		}
		else
		{
			fallDrag = b3MaxFloat( fallDrag, LegDrag( &wagon ) );
		}
	}

	float dropLimit = wagon.tugHeight - wagon.tugDistance * sinf( wagon.tuning.minShaftPitch );
	printf( "  ledge drop: horse pitch %.1f to %.1f deg, drop below kingbolt %.2f m (limit %.2f), "
			"leg drag %.4f W on the deck, %.4f W falling\n",
			minPitch * 180.0f / B3_PI, maxPitch * 180.0f / B3_PI, maxDrop, dropLimit, deckDrag, fallDrag );

	ENSURE( minPitch > wagon.tuning.minHorsePitch - 0.05f );
	ENSURE( maxPitch < wagon.tuning.maxHorsePitch + 0.05f );
	ENSURE( maxDrop < dropLimit + 0.05f );
	ENSURE( deckDrag < 1.0e-3f );

	DestroyWagon( &wagon );
	b3DestroyWorld( worldId );
	return 0;
}

// The wagon canters up a hill. The horse pitches with it and its legs never hold the wagon back.
static int HillClimb( void )
{
	float slope = -0.12f;
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateGround( worldId, slope );
	Wagon wagon = CreateTestWagon( worldId, slope );

	float maxDrag = 0.0f, maxShaftForce = 0.0f, maxPitch = 0.0f, speed = 0.0f, maxAnchorGap = 0.0f;
	b3Pos startAnchor = b3Body_GetPosition( wagon.horseAnchorId );
	for ( int i = 0; i < 10 * 60; ++i )
	{
		StepWagon( worldId, &wagon, (WagonInput){ e_wagonCanter, 0.0f, false } );
		b3Vec3 anchorGap = b3SubPos( b3Body_GetPosition( wagon.horseAnchorId ), wagon.horseTransform.p );
		maxAnchorGap = b3MaxFloat( maxAnchorGap, b3Length( anchorGap ) );
		if ( i < 120 )
		{
			continue;
		}

		maxDrag = b3MaxFloat( maxDrag, LegDrag( &wagon ) );
		maxShaftForce =
			b3MaxFloat( maxShaftForce, b3Length( b3Joint_GetConstraintForce( wagon.shaftJointId ) ) / wagon.weight );
		b3Vec3 forward = b3RotateVector( b3Body_GetRotation( wagon.horseId ), b3Vec3_axisX );
		maxPitch = b3MaxFloat( maxPitch, asinf( b3ClampFloat( forward.y, -1.0f, 1.0f ) ) );
		speed = wagon.speed;
	}

	float hill = atanf( -slope );
	printf( "  hill climb: speed %.2f m/s, horse pitch %.1f deg (hill %.1f), leg drag %.4f W, shaft force %.4f W\n",
			speed, maxPitch * 180.0f / B3_PI, hill * 180.0f / B3_PI, maxDrag, maxShaftForce );

	float anchorTravel = b3Length( b3SubPos( b3Body_GetPosition( wagon.horseAnchorId ), startAnchor ) );
	printf( "  hill climb: anchor gap %.2f m, anchor travel %.2f m\n", maxAnchorGap, anchorTravel );

	ENSURE( speed > 0.5f * wagon.tuning.gaitSpeed[e_wagonCanter] );
	ENSURE( b3AbsFloat( maxPitch - hill ) < 0.05f );
	ENSURE( maxDrag < 1.0e-3f );

	// The anchor follows the horse so the leg frame offset stays small
	ENSURE( anchorTravel > 10.0f );
	ENSURE( maxAnchorGap < 11.0f );

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

// The wagon drives over a curb under its left wheels and through light crates while steering, and the
// horse brushes a trunk
static uint32_t RunDeterminismScene( int workerCount, int* horseContactSteps )
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

	CreateTree( worldId, 13.0f, -1.5f, 0.25f );

	Wagon wagon = CreateTestWagon( worldId, 0.0f );

	WagonInput input = { e_wagonTrot, 0.0f, false };
	uint32_t hash = 2166136261u;
	*horseContactSteps = 0;
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

		b3WorldTransform horse = b3Body_GetTransform( wagon.horseId );
		hash = HashBytes( hash, &horse, sizeof( horse ) );
		*horseContactSteps += wagon.horseContactCount > 0 ? 1 : 0;
	}

	DestroyWagon( &wagon );
	b3DestroyWorld( worldId );
	return hash;
}

static int MultithreadedDeterminism( void )
{
	int contactSteps;
	uint32_t reference = RunDeterminismScene( 1, &contactSteps );
	printf( "  determinism: workers 1 hash 0x%08X, horse contact steps %d\n", reference, contactSteps );
	ENSURE( contactSteps > 0 );
	for ( int workerCount = 2; workerCount <= 4; ++workerCount )
	{
		uint32_t hash = RunDeterminismScene( workerCount, &contactSteps );
		printf( "  determinism: workers %d hash 0x%08X\n", workerCount, hash );
		ENSURE( hash == reference );
	}

	ENSURE( RunDeterminismScene( 1, &contactSteps ) == reference );
	return 0;
}

int VehicleTest( void )
{
	RUN_SUBTEST( TireJointApi );
	RUN_SUBTEST( SettleFlat );
	RUN_SUBTEST( SlopeSleep );
	RUN_SUBTEST( TreeGlance );
	RUN_SUBTEST( TreeSideSwipe );
	RUN_SUBTEST( TreeHeadOn );
	RUN_SUBTEST( TreeCrash );
	RUN_SUBTEST( HillClimb );
	RUN_SUBTEST( LedgeDrop );
	RUN_SUBTEST( MultithreadedDeterminism );
	return 0;
}
