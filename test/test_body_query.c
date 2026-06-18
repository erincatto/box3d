// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"

// The per-body query functions take an explicit world origin and a world body transform.
// Everything is re-centered on the origin so the float collision math stays accurate far from
// the world origin. These tests pin that framing: results come back in world space, the supplied
// transform drives the geometry (not the body's stored pose), and a large origin offset must not
// change a hit fraction or normal.
//
// CastRay and CastShape never touch the body's stored transform, so a static body at the origin
// holding local-frame shapes is enough. No step is needed for any query.

static b3WorldId CreateQueryWorld( b3BodyId* bodyId )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	*bodyId = b3CreateBody( worldId, &bodyDef );
	return worldId;
}

static b3WorldTransform IdentityAt( float x, float y, float z )
{
	return (b3WorldTransform){ .p = (b3Pos){ x, y, z }, .q = b3Quat_identity };
}

// CastRay ----------------------------------------------------------------------------------

static int CastRayHitsSphere( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 1.0f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	// Body sphere at world (5,0,0), ray straight at it along +X.
	b3WorldTransform bodyTransform = IdentityAt( 5.0f, 0.0f, 0.0f );
	b3BodyCastResult result =
		b3Body_CastRay( bodyId, (b3Pos){ 0.0f, 0.0f, 0.0f }, (b3Vec3){ 10.0f, 0.0f, 0.0f }, b3DefaultQueryFilter(), 1.0f, bodyTransform );

	ENSURE( result.hit );
	ENSURE( b3Shape_IsValid( result.shapeId ) );
	ENSURE_SMALL( result.fraction - 0.4f, 1e-5f );
	ENSURE_SMALL( result.normal.x + 1.0f, 1e-5f );
	ENSURE_SMALL( result.normal.y, 1e-5f );
	ENSURE_SMALL( result.normal.z, 1e-5f );

	b3Vec3 point = b3ToVec3( result.point );
	ENSURE_SMALL( point.x - 4.0f, 1e-4f );
	ENSURE_SMALL( point.y, 1e-4f );
	ENSURE_SMALL( point.z, 1e-4f );

	b3DestroyWorld( worldId );
	return 0;
}

static int CastRayMiss( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 1.0f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	// Ray runs parallel to the body, never reaching it.
	b3WorldTransform bodyTransform = IdentityAt( 5.0f, 0.0f, 0.0f );
	b3BodyCastResult result =
		b3Body_CastRay( bodyId, (b3Pos){ 0.0f, 0.0f, 0.0f }, (b3Vec3){ 0.0f, 10.0f, 0.0f }, b3DefaultQueryFilter(), 1.0f, bodyTransform );

	ENSURE( result.hit == false );

	b3DestroyWorld( worldId );
	return 0;
}

static int CastRayClosestShape( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere nearSphere = { { 0.0f, 0.0f, 0.0f }, 1.0f };
	b3Sphere farSphere = { { 4.0f, 0.0f, 0.0f }, 1.0f };
	b3ShapeId nearId = b3CreateSphereShape( bodyId, &shapeDef, &nearSphere );
	b3CreateSphereShape( bodyId, &shapeDef, &farSphere );

	// Ray crosses both spheres; the loop must shrink maxFraction to the nearer hit.
	b3WorldTransform bodyTransform = IdentityAt( 0.0f, 0.0f, 0.0f );
	b3BodyCastResult result =
		b3Body_CastRay( bodyId, (b3Pos){ -5.0f, 0.0f, 0.0f }, (b3Vec3){ 10.0f, 0.0f, 0.0f }, b3DefaultQueryFilter(), 1.0f, bodyTransform );

	ENSURE( result.hit );
	ENSURE( result.shapeId.index1 == nearId.index1 );
	ENSURE( result.shapeId.generation == nearId.generation );
	ENSURE_SMALL( result.fraction - 0.4f, 1e-5f );

	b3DestroyWorld( worldId );
	return 0;
}

static int CastRayRotatedBody( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	// Local center (0,2,0) rotated +90 deg about Z lands at world (-2,0,0).
	b3Sphere sphere = { { 0.0f, 2.0f, 0.0f }, 0.5f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	b3WorldTransform bodyTransform = {
		.p = (b3Pos){ 0.0f, 0.0f, 0.0f },
		.q = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 0.0f, 1.0f }, 0.5f * B3_PI ),
	};
	b3BodyCastResult result =
		b3Body_CastRay( bodyId, (b3Pos){ 0.0f, 0.0f, 0.0f }, (b3Vec3){ -4.0f, 0.0f, 0.0f }, b3DefaultQueryFilter(), 1.0f, bodyTransform );

	ENSURE( result.hit );
	ENSURE_SMALL( result.fraction - 0.375f, 1e-5f );
	ENSURE_SMALL( result.normal.x - 1.0f, 1e-5f );

	b3Vec3 point = b3ToVec3( result.point );
	ENSURE_SMALL( point.x + 1.5f, 1e-4f );

	b3DestroyWorld( worldId );
	return 0;
}

static int CastRayFarFromOrigin( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 1.0f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	// Same geometry as CastRayHitsSphere shifted far from the world origin. The relative framing
	// keeps the subtraction exact, so fraction and normal must be unchanged.
	b3Pos origin = { 1.0e6f, -2.0e6f, 5.0e5f };
	b3WorldTransform bodyTransform = { .p = b3OffsetPos( origin, (b3Vec3){ 5.0f, 0.0f, 0.0f } ), .q = b3Quat_identity };
	b3BodyCastResult result =
		b3Body_CastRay( bodyId, origin, (b3Vec3){ 10.0f, 0.0f, 0.0f }, b3DefaultQueryFilter(), 1.0f, bodyTransform );

	ENSURE( result.hit );
	ENSURE_SMALL( result.fraction - 0.4f, 1e-5f );
	ENSURE_SMALL( result.normal.x + 1.0f, 1e-5f );
	ENSURE_SMALL( result.normal.y, 1e-5f );
	ENSURE_SMALL( result.normal.z, 1e-5f );

	b3DestroyWorld( worldId );
	return 0;
}

// CastShape --------------------------------------------------------------------------------

static int CastShapeHitsBox( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	// Sphere proxy of radius 0.5 cast along +X into a box whose front face is at world x = 4.
	b3Vec3 point = { 0.0f, 0.0f, 0.0f };
	b3ShapeProxy proxy = { &point, 1, 0.5f };
	b3WorldTransform bodyTransform = IdentityAt( 5.0f, 0.0f, 0.0f );
	b3BodyCastResult result = b3Body_CastShape( bodyId, (b3Pos){ 0.0f, 0.0f, 0.0f }, &proxy, (b3Vec3){ 10.0f, 0.0f, 0.0f },
												b3DefaultQueryFilter(), 1.0f, false, bodyTransform );

	// Front face at world x = 4. The fraction carries a small shape-cast skin, the contact point
	// and normal do not.
	ENSURE( result.hit );
	ENSURE( b3Shape_IsValid( result.shapeId ) );
	ENSURE_SMALL( result.fraction - 0.35f, 1e-2f );
	ENSURE_SMALL( result.normal.x + 1.0f, 1e-4f );

	b3Vec3 hit = b3ToVec3( result.point );
	ENSURE_SMALL( hit.x - 4.0f, 1e-3f );

	b3DestroyWorld( worldId );
	return 0;
}

static int CastShapeMiss( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3Vec3 point = { 0.0f, 0.0f, 0.0f };
	b3ShapeProxy proxy = { &point, 1, 0.5f };
	b3WorldTransform bodyTransform = IdentityAt( 5.0f, 0.0f, 0.0f );
	b3BodyCastResult result = b3Body_CastShape( bodyId, (b3Pos){ 0.0f, 0.0f, 0.0f }, &proxy, (b3Vec3){ 0.0f, 10.0f, 0.0f },
												b3DefaultQueryFilter(), 1.0f, false, bodyTransform );

	ENSURE( result.hit == false );

	b3DestroyWorld( worldId );
	return 0;
}

static int CastShapeRotatedBody( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	// Body sphere local center (0,2,0) rotated +90 deg about Z lands at world (-2,0,0).
	b3Sphere sphere = { { 0.0f, 2.0f, 0.0f }, 1.0f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	b3Vec3 point = { 0.0f, 0.0f, 0.0f };
	b3ShapeProxy proxy = { &point, 1, 0.5f };
	b3WorldTransform bodyTransform = {
		.p = (b3Pos){ 0.0f, 0.0f, 0.0f },
		.q = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 0.0f, 1.0f }, 0.5f * B3_PI ),
	};
	b3BodyCastResult result = b3Body_CastShape( bodyId, (b3Pos){ 0.0f, 0.0f, 0.0f }, &proxy, (b3Vec3){ -4.0f, 0.0f, 0.0f },
												b3DefaultQueryFilter(), 1.0f, false, bodyTransform );

	ENSURE( result.hit );
	ENSURE_SMALL( result.fraction - 0.125f, 1e-2f );
	ENSURE_SMALL( result.normal.x - 1.0f, 1e-4f );

	b3Vec3 hit = b3ToVec3( result.point );
	ENSURE_SMALL( hit.x + 1.0f, 1e-3f );

	b3DestroyWorld( worldId );
	return 0;
}

static int CastShapeFarFromOrigin( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3Vec3 point = { 0.0f, 0.0f, 0.0f };
	b3ShapeProxy proxy = { &point, 1, 0.5f };
	b3Pos origin = { 1.0e6f, -2.0e6f, 5.0e5f };
	b3WorldTransform bodyTransform = { .p = b3OffsetPos( origin, (b3Vec3){ 5.0f, 0.0f, 0.0f } ), .q = b3Quat_identity };
	b3BodyCastResult result = b3Body_CastShape( bodyId, origin, &proxy, (b3Vec3){ 10.0f, 0.0f, 0.0f }, b3DefaultQueryFilter(),
												1.0f, false, bodyTransform );

	ENSURE( result.hit );
	ENSURE_SMALL( result.fraction - 0.35f, 1e-2f );
	ENSURE_SMALL( result.normal.x + 1.0f, 1e-4f );

	b3DestroyWorld( worldId );
	return 0;
}

// OverlapShape -----------------------------------------------------------------------------

static int OverlapTrue( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	// Proxy sits at the box center.
	b3Vec3 point = { 0.0f, 0.0f, 0.0f };
	b3ShapeProxy proxy = { &point, 1, 0.5f };
	b3WorldTransform bodyTransform = IdentityAt( 5.0f, 0.0f, 0.0f );
	bool overlaps = b3Body_OverlapShape( bodyId, (b3Pos){ 5.0f, 0.0f, 0.0f }, &proxy, b3DefaultQueryFilter(), bodyTransform );

	ENSURE( overlaps );

	b3DestroyWorld( worldId );
	return 0;
}

static int OverlapFalse( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3Vec3 point = { 0.0f, 0.0f, 0.0f };
	b3ShapeProxy proxy = { &point, 1, 0.5f };
	b3WorldTransform bodyTransform = IdentityAt( 5.0f, 0.0f, 0.0f );
	bool overlaps = b3Body_OverlapShape( bodyId, (b3Pos){ 20.0f, 0.0f, 0.0f }, &proxy, b3DefaultQueryFilter(), bodyTransform );

	ENSURE( overlaps == false );

	b3DestroyWorld( worldId );
	return 0;
}

static int OverlapRespectsBodyTransform( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	// Fixed proxy and origin: only the supplied transform decides the overlap.
	b3Vec3 point = { 0.0f, 0.0f, 0.0f };
	b3ShapeProxy proxy = { &point, 1, 0.5f };
	b3Pos origin = { 0.0f, 0.0f, 0.0f };

	ENSURE( b3Body_OverlapShape( bodyId, origin, &proxy, b3DefaultQueryFilter(), IdentityAt( 0.0f, 0.0f, 0.0f ) ) );
	ENSURE( b3Body_OverlapShape( bodyId, origin, &proxy, b3DefaultQueryFilter(), IdentityAt( 20.0f, 0.0f, 0.0f ) ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

static int OverlapFilter( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3Vec3 point = { 0.0f, 0.0f, 0.0f };
	b3ShapeProxy proxy = { &point, 1, 0.5f };
	b3WorldTransform bodyTransform = IdentityAt( 0.0f, 0.0f, 0.0f );

	// Geometry overlaps, but a zero mask rejects every category.
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = 0;
	bool overlaps = b3Body_OverlapShape( bodyId, (b3Pos){ 0.0f, 0.0f, 0.0f }, &proxy, filter, bodyTransform );

	ENSURE( overlaps == false );

	b3DestroyWorld( worldId );
	return 0;
}

// CollideMover -----------------------------------------------------------------------------

static int MoverTouchesBox( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	// Mover core runs above the +Y face; its 0.2 radius reaches 0.1 into it.
	b3Capsule mover = { { -0.3f, 0.6f, 0.0f }, { 0.3f, 0.6f, 0.0f }, 0.2f };
	b3BodyPlaneResult planes[4];
	b3WorldTransform bodyTransform = IdentityAt( 0.0f, 0.0f, 0.0f );
	int count = b3Body_CollideMover( bodyId, planes, 4, (b3Pos){ 0.0f, 0.0f, 0.0f }, &mover, b3DefaultQueryFilter(), bodyTransform );

	ENSURE( count == 1 );
	ENSURE( b3Shape_IsValid( planes[0].shapeId ) );
	ENSURE( b3IsNormalized( planes[0].result.plane.normal ) );
	ENSURE( planes[0].result.plane.normal.y > 0.99f );
	ENSURE_SMALL( planes[0].result.plane.offset - 0.1f, 1e-4f );

	b3DestroyWorld( worldId );
	return 0;
}

static int MoverSeparated( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3Capsule mover = { { -0.3f, 5.0f, 0.0f }, { 0.3f, 5.0f, 0.0f }, 0.2f };
	b3BodyPlaneResult planes[4];
	b3WorldTransform bodyTransform = IdentityAt( 0.0f, 0.0f, 0.0f );
	int count = b3Body_CollideMover( bodyId, planes, 4, (b3Pos){ 0.0f, 0.0f, 0.0f }, &mover, b3DefaultQueryFilter(), bodyTransform );

	ENSURE( count == 0 );

	b3DestroyWorld( worldId );
	return 0;
}

static int MoverRotatedBody( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	// Rotating +90 deg about X turns the local +Y face toward world +Z. The mover sits above the
	// world +Z face, so the returned normal must come back rotated into world space.
	b3Capsule mover = { { -0.3f, 0.0f, 0.6f }, { 0.3f, 0.0f, 0.6f }, 0.2f };
	b3BodyPlaneResult planes[4];
	b3WorldTransform bodyTransform = {
		.p = (b3Pos){ 0.0f, 0.0f, 0.0f },
		.q = b3MakeQuatFromAxisAngle( (b3Vec3){ 1.0f, 0.0f, 0.0f }, 0.5f * B3_PI ),
	};
	int count = b3Body_CollideMover( bodyId, planes, 4, (b3Pos){ 0.0f, 0.0f, 0.0f }, &mover, b3DefaultQueryFilter(), bodyTransform );

	ENSURE( count == 1 );
	ENSURE( b3IsNormalized( planes[0].result.plane.normal ) );
	ENSURE( planes[0].result.plane.normal.z > 0.99f );
	ENSURE_SMALL( planes[0].result.plane.offset - 0.1f, 1e-4f );

	b3DestroyWorld( worldId );
	return 0;
}

static int MoverCapacity( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	// Two spheres each touch a mover that runs between them along X at y = 0.
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere left = { { -0.4f, 0.6f, 0.0f }, 0.5f };
	b3Sphere right = { { 0.4f, 0.6f, 0.0f }, 0.5f };
	b3CreateSphereShape( bodyId, &shapeDef, &left );
	b3CreateSphereShape( bodyId, &shapeDef, &right );

	b3Capsule mover = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 0.2f };
	b3BodyPlaneResult planes[4];
	b3WorldTransform bodyTransform = IdentityAt( 0.0f, 0.0f, 0.0f );

	// Capacity caps the result and prevents writing past the buffer.
	int capped = b3Body_CollideMover( bodyId, planes, 1, (b3Pos){ 0.0f, 0.0f, 0.0f }, &mover, b3DefaultQueryFilter(), bodyTransform );
	ENSURE( capped == 1 );

	int full = b3Body_CollideMover( bodyId, planes, 4, (b3Pos){ 0.0f, 0.0f, 0.0f }, &mover, b3DefaultQueryFilter(), bodyTransform );
	ENSURE( full == 2 );

	b3DestroyWorld( worldId );
	return 0;
}

int BodyQueryTest( void )
{
	RUN_SUBTEST( CastRayHitsSphere );
	RUN_SUBTEST( CastRayMiss );
	RUN_SUBTEST( CastRayClosestShape );
	RUN_SUBTEST( CastRayRotatedBody );
	RUN_SUBTEST( CastRayFarFromOrigin );

	RUN_SUBTEST( CastShapeHitsBox );
	RUN_SUBTEST( CastShapeMiss );
	RUN_SUBTEST( CastShapeRotatedBody );
	RUN_SUBTEST( CastShapeFarFromOrigin );

	RUN_SUBTEST( OverlapTrue );
	RUN_SUBTEST( OverlapFalse );
	RUN_SUBTEST( OverlapRespectsBodyTransform );
	RUN_SUBTEST( OverlapFilter );

	RUN_SUBTEST( MoverTouchesBox );
	RUN_SUBTEST( MoverSeparated );
	RUN_SUBTEST( MoverRotatedBody );
	RUN_SUBTEST( MoverCapacity );

	return 0;
}
