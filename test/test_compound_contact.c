// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "test_macros.h"

// A multi-shape body, so a body pair carries many sub-contacts and clustering engages.
static b3BodyId CreateSlab( b3WorldId worldId, float x, float y, float z, b3BodyType type, float restitution, float density )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = type;
	bodyDef.position = ( b3Pos ){ x, y, z };
	b3BodyId body = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = density;
	shapeDef.baseMaterial.friction = 0.5f;
	shapeDef.baseMaterial.restitution = restitution;

	for ( int i = -1; i <= 1; ++i )
	{
		for ( int k = -1; k <= 1; ++k )
		{
			b3Vec3 offset = { (float)i, 0.0f, (float)k };
			b3BoxHull box = b3MakeOffsetBoxHull( 0.5f, 0.5f, 0.5f, offset );
			b3CreateHullShape( body, &shapeDef, &box.base );
		}
	}

	return body;
}

static void AddGround( b3WorldId worldId )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = ( b3Pos ){ 0.0f, -1.0f, 0.0f };
	b3BodyId ground = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( ground, &shapeDef, &box.base );
}

static void StepN( b3WorldId worldId, int steps )
{
	for ( int i = 0; i < steps; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
}

static float BodySpeed( b3BodyId body )
{
	return b3Length( b3Body_GetLinearVelocity( body ) ) + b3Length( b3Body_GetAngularVelocity( body ) );
}

static void RunStack( float* topY, float* maxSpeed )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	AddGround( worldId );

	b3BodyId slabs[3];
	for ( int i = 0; i < 3; ++i )
	{
		slabs[i] = CreateSlab( worldId, 0.0f, 0.55f + i * 1.05f, 0.0f, b3_dynamicBody, 0.0f, 1.0f );
	}

	StepN( worldId, 600 );

	float ms = 0.0f;
	for ( int i = 0; i < 3; ++i )
	{
		float s = BodySpeed( slabs[i] );
		ms = s > ms ? s : ms;
	}

	*maxSpeed = ms;
	*topY = (float)b3Body_GetPosition( slabs[2] ).y;

	b3DestroyWorld( worldId );
}

// A stack of multi-shape slabs must settle without gaining energy or sinking through itself.
static int CompoundStackTest( void )
{
	float topY, maxSpeed;
	RunStack( &topY, &maxSpeed );

	ENSURE( maxSpeed < 0.1f );
	ENSURE( topY > 2.4f && topY < 2.6f );

	return 0;
}

static float RunBounce( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	AddGround( worldId );
	b3BodyId slab = CreateSlab( worldId, 0.0f, 3.0f, 0.0f, b3_dynamicBody, 0.7f, 1.0f );

	float peak = 0.0f;
	bool contacted = false;
	for ( int i = 0; i < 250; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		float y = (float)b3Body_GetPosition( slab ).y;
		contacted = contacted || y < 0.7f;
		if ( contacted && y > peak )
		{
			peak = y;
		}
	}

	b3DestroyWorld( worldId );
	return peak;
}

// Restitution stays physical: the slab bounces up but never above its drop height.
static int CompoundRestitutionTest( void )
{
	float peak = RunBounce();

	ENSURE( peak > 1.0f );
	ENSURE( peak < 3.0f );

	return 0;
}

// Compound on mesh contacts: the slab rests on the mesh without sinking through.
static int CompoundMeshTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef meshBodyDef = b3DefaultBodyDef();
	b3BodyId meshBody = b3CreateBody( worldId, &meshBodyDef );
	b3MeshData* mesh = b3CreateGridMesh( 10, 10, 1.0f, 0, false );
	ENSURE( mesh != NULL );
	b3ShapeDef meshShapeDef = b3DefaultShapeDef();
	b3CreateMeshShape( meshBody, &meshShapeDef, mesh, ( b3Vec3 ){ 1.0f, 1.0f, 1.0f } );

	b3BodyId slab = CreateSlab( worldId, 0.0f, 2.0f, 0.0f, b3_dynamicBody, 0.0f, 1.0f );

	StepN( worldId, 400 );

	float y = (float)b3Body_GetPosition( slab ).y;
	ENSURE( y > 0.3f && y < 0.7f );
	ENSURE( BodySpeed( slab ) < 0.1f );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );

	return 0;
}

static float RunMassRatio( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	AddGround( worldId );
	CreateSlab( worldId, 0.0f, 0.55f, 0.0f, b3_dynamicBody, 0.0f, 1.0f );
	b3BodyId heavy = CreateSlab( worldId, 0.0f, 1.6f, 0.0f, b3_dynamicBody, 0.0f, 200.0f );

	StepN( worldId, 600 );

	float y = (float)b3Body_GetPosition( heavy ).y;
	b3DestroyWorld( worldId );
	return y;
}

// A heavy body on a light one must not crush through the reduced contact set.
static int CompoundMassRatioTest( void )
{
	float y = RunMassRatio();

	ENSURE( y > 1.35f && y < 1.65f );

	return 0;
}

// A spinning body must not gain energy from contact point churn.
static int CompoundRollingTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	AddGround( worldId );
	b3BodyId slab = CreateSlab( worldId, 0.0f, 0.5f, 0.0f, b3_dynamicBody, 0.0f, 1.0f );
	b3Body_SetAngularVelocity( slab, ( b3Vec3 ){ 0.0f, 8.0f, 0.0f } );

	float maxSpeed = 0.0f;
	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		float s = BodySpeed( slab );
		maxSpeed = s > maxSpeed ? s : maxSpeed;
	}

	float finalSpeed = BodySpeed( slab );
	float finalY = (float)b3Body_GetPosition( slab ).y;
	b3DestroyWorld( worldId );

	ENSURE( maxSpeed < 8.5f );
	ENSURE( finalSpeed < 8.0f );
	ENSURE( finalY > 0.3f && finalY < 0.7f );

	return 0;
}

// Touch and hit events still fire with sane data for merged contacts.
static int CompoundEventsTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = ( b3Pos ){ 0.0f, -1.0f, 0.0f };
		b3BodyId ground = b3CreateBody( worldId, &bodyDef );
		b3BoxHull box = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		shapeDef.enableHitEvents = true;
		b3CreateHullShape( ground, &shapeDef, &box.base );
	}

	b3BodyId slab = CreateSlab( worldId, 0.0f, 4.0f, 0.0f, b3_dynamicBody, 0.0f, 1.0f );
	b3Body_SetLinearVelocity( slab, ( b3Vec3 ){ 0.0f, -10.0f, 0.0f } );

	int beginTotal = 0;
	int hitTotal = 0;
	bool hitDataValid = true;
	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		b3ContactEvents events = b3World_GetContactEvents( worldId );
		beginTotal += events.beginCount;
		hitTotal += events.hitCount;
		for ( int e = 0; e < events.hitCount; ++e )
		{
			const b3ContactHitEvent* h = events.hitEvents + e;
			b3Vec3 pt = { (float)h->point.x, (float)h->point.y, (float)h->point.z };
			if ( b3IsValidVec3( pt ) == false || b3IsValidVec3( h->normal ) == false || h->approachSpeed <= 0.0f )
			{
				hitDataValid = false;
			}
		}
	}

	b3DestroyWorld( worldId );

	ENSURE( beginTotal > 0 );
	ENSURE( hitTotal > 0 );
	ENSURE( hitDataValid );

	return 0;
}

// Spread normals on a sphere must stay finite and bounded (no NaN or tunnelling).
static int CompoundCurvedTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	AddGround( worldId );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = ( b3Pos ){ 0.0f, 2.0f, 0.0f };
		b3BodyId sphereBody = b3CreateBody( worldId, &bodyDef );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 2.0f };
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateSphereShape( sphereBody, &shapeDef, &sphere );
	}

	b3BodyId slab = CreateSlab( worldId, 0.0f, 6.0f, 0.0f, b3_dynamicBody, 0.0f, 1.0f );

	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		b3Pos p = b3Body_GetPosition( slab );
		b3Vec3 v = { (float)p.x, (float)p.y, (float)p.z };
		ENSURE( b3IsValidVec3( v ) );
		ENSURE( v.y > -1.0f && v.y < 12.0f );
	}

	b3DestroyWorld( worldId );

	return 0;
}

// Friction on the merged contacts must brake a sliding body.
static int CompoundSlidingTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	AddGround( worldId );
	b3BodyId slab = CreateSlab( worldId, 0.0f, 0.5f, 0.0f, b3_dynamicBody, 0.0f, 1.0f );
	b3Body_SetLinearVelocity( slab, ( b3Vec3 ){ 5.0f, 0.0f, 0.0f } );

	StepN( worldId, 300 );

	float speed = BodySpeed( slab );
	float y = (float)b3Body_GetPosition( slab ).y;
	b3DestroyWorld( worldId );

	ENSURE( speed < 0.2f );
	ENSURE( y > 0.3f && y < 0.7f );

	return 0;
}

int CompoundContactTest( void )
{
	RUN_SUBTEST( CompoundStackTest );
	RUN_SUBTEST( CompoundRestitutionTest );
	RUN_SUBTEST( CompoundMeshTest );
	RUN_SUBTEST( CompoundMassRatioTest );
	RUN_SUBTEST( CompoundRollingTest );
	RUN_SUBTEST( CompoundEventsTest );
	RUN_SUBTEST( CompoundCurvedTest );
	RUN_SUBTEST( CompoundSlidingTest );

	return 0;
}
