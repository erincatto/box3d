// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "box3d/box3d.h"
#include "test_macros.h"

#include <math.h>

#define STACK_COUNT 6
#define MAX_STEPS 400

typedef struct StackResult
{
	// Final body positions relative to the base, so origin and far runs are directly comparable
	b3Vec3 relativePositions[STACK_COUNT];

	// First step on which the top body fell asleep, or -1 if it never settled
	int sleepStep;
} StackResult;

// Drop a short stack of boxes onto a ground box centered at baseX. Records each body's final
// position relative to the base and the step on which the stack settles.
static StackResult RunStack( float baseX )
{
	b3Pos base = (b3Pos){ baseX, 0.0f, 0.0f };

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.position = base;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull groundBox = b3MakeBoxHull( 10.0f, 1.0f, 10.0f );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	b3BodyId bodies[STACK_COUNT];
	for ( int i = 0; i < STACK_COUNT; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3OffsetPos( base, (b3Vec3){ 0.0f, 2.0f + 1.05f * i, 0.0f } );
		bodies[i] = b3CreateBody( worldId, &bodyDef );

		b3BoxHull box = b3MakeCubeHull( 0.5f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 1.0f;
		b3CreateHullShape( bodies[i], &shapeDef, &box.base );
	}

	StackResult result = { 0 };
	result.sleepStep = -1;

	for ( int step = 0; step < MAX_STEPS; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		if ( result.sleepStep < 0 && b3Body_IsAwake( bodies[STACK_COUNT - 1] ) == false )
		{
			result.sleepStep = step;
		}
	}

	for ( int i = 0; i < STACK_COUNT; ++i )
	{
		b3Pos p = b3Body_GetPosition( bodies[i] );
		result.relativePositions[i] = b3SubPos( p, base );
	}

	b3DestroyWorld( worldId );
	return result;
}

// A stack at the origin and a stack far from the origin should behave identically in double
// precision: same settle frame, same relative geometry.
static int LargeWorldStackTest( void )
{
	StackResult origin = RunStack( 0.0f );
	ENSURE( origin.sleepStep >= 0 );

#if defined( BOX3D_DOUBLE_PRECISION )
	StackResult far = RunStack( 1.0e7f );
	ENSURE( far.sleepStep >= 0 );

	// Sleeps on the same frame and lands in the same relative configuration
	ENSURE( far.sleepStep == origin.sleepStep );
	for ( int i = 0; i < STACK_COUNT; ++i )
	{
		ENSURE_SMALL( far.relativePositions[i].x - origin.relativePositions[i].x, 1.0e-3f );
		ENSURE_SMALL( far.relativePositions[i].y - origin.relativePositions[i].y, 1.0e-3f );
		ENSURE_SMALL( far.relativePositions[i].z - origin.relativePositions[i].z, 1.0e-3f );
	}
#endif

	return 0;
}

// Fire a fast bullet at a thin wall. Returns the bullet's final x relative to the base. If
// continuous collision works the bullet stops at the wall instead of tunneling past it.
static float RunBullet( float baseX )
{
	b3Pos base = (b3Pos){ baseX, 0.0f, 0.0f };

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Thin static wall at x = base + 5, spanning y and z
	b3BodyDef wallDef = b3DefaultBodyDef();
	wallDef.type = b3_staticBody;
	wallDef.position = b3OffsetPos( base, (b3Vec3){ 5.0f, 0.0f, 0.0f } );
	b3BodyId wallId = b3CreateBody( worldId, &wallDef );
	b3BoxHull wallBox = b3MakeBoxHull( 0.05f, 5.0f, 5.0f );
	b3ShapeDef wallShapeDef = b3DefaultShapeDef();
	b3CreateHullShape( wallId, &wallShapeDef, &wallBox.base );

	// Small fast bullet aimed at the wall, no gravity
	b3BodyDef bulletDef = b3DefaultBodyDef();
	bulletDef.type = b3_dynamicBody;
	bulletDef.isBullet = true;
	bulletDef.gravityScale = 0.0f;
	bulletDef.position = base;
	bulletDef.linearVelocity = (b3Vec3){ 200.0f, 0.0f, 0.0f };
	b3BodyId bulletId = b3CreateBody( worldId, &bulletDef );
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.1f };
	b3ShapeDef bulletShapeDef = b3DefaultShapeDef();
	bulletShapeDef.density = 1.0f;
	b3CreateSphereShape( bulletId, &bulletShapeDef, &sphere );

	for ( int step = 0; step < 30; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Vec3 relative = b3SubPos( b3Body_GetPosition( bulletId ), base );
	b3DestroyWorld( worldId );
	return relative.x;
}

// The bullet must be caught by the wall, not tunnel through it. The blocking check is that this
// still holds far from the origin where the swept query box rounds back to float with large ULP.
static int LargeWorldBulletTest( void )
{
	// Wall front face is at x = 5 - 0.05; the bullet radius is 0.1, so a caught bullet stays well
	// short of the wall center at x = 5.
	float originX = RunBullet( 0.0f );
	ENSURE( originX < 5.0f );

#if defined( BOX3D_DOUBLE_PRECISION )
	float farX = RunBullet( 1.0e7f );
	ENSURE( farX < 5.0f );
#endif

	return 0;
}

int LargeWorldTest( void )
{
	RUN_SUBTEST( LargeWorldStackTest );
	RUN_SUBTEST( LargeWorldBulletTest );
	return 0;
}
