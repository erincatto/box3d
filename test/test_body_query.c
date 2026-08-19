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

static bool CountOverlapCallback( b3ShapeId shapeId, void* context )
{
	(void)shapeId;
	*(int*)context += 1;
	return true;
}

// A box hull proxy built around a world target with a zero origin must hit the same shapes as the
// same box built at the local origin and queried with the target as origin. This is the origin
// relative equivalence the world query promises, and the pattern users reach for when they bake a
// query box with b3MakeTransformedBoxHull.
static int OverlapHullProxyEquivalence( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ 10.0f, 0.0f, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3World_Step( worldId, 1.0f / 60.0f, 1 );

	b3QueryFilter filter = b3DefaultQueryFilter();

	// Overlapping target: a 10 wide query box centered on the body.
	{
		b3Vec3 offset = { 10.0f, 0.0f, 0.0f };

		b3BoxHull baked = b3MakeTransformedBoxHull( 5.0f, 5.0f, 5.0f, (b3Transform){ offset, b3Quat_identity } );
		b3ShapeProxy bakedProxy = { baked.boxPoints, baked.base.vertexCount, 0.0f };
		int bakedHits = 0;
		b3World_OverlapShape( worldId, b3Pos_zero, &bakedProxy, filter, CountOverlapCallback, &bakedHits );

		b3Pos origin = b3OffsetPos( b3Pos_zero, offset );
		b3BoxHull local = b3MakeBoxHull( 5.0f, 5.0f, 5.0f );
		b3ShapeProxy localProxy = { local.boxPoints, local.base.vertexCount, 0.0f };
		int localHits = 0;
		b3World_OverlapShape( worldId, origin, &localProxy, filter, CountOverlapCallback, &localHits );

		ENSURE( bakedHits == 1 );
		ENSURE( localHits == bakedHits );
	}

	// Clearing target: same box far from the body, both formulations agree on the miss.
	{
		b3Vec3 offset = { 100.0f, 0.0f, 0.0f };

		b3BoxHull baked = b3MakeTransformedBoxHull( 5.0f, 5.0f, 5.0f, (b3Transform){ offset, b3Quat_identity } );
		b3ShapeProxy bakedProxy = { baked.boxPoints, baked.base.vertexCount, 0.0f };
		int bakedHits = 0;
		b3World_OverlapShape( worldId, b3Pos_zero, &bakedProxy, filter, CountOverlapCallback, &bakedHits );

		b3Pos origin = b3OffsetPos( b3Pos_zero, offset );
		b3BoxHull local = b3MakeBoxHull( 5.0f, 5.0f, 5.0f );
		b3ShapeProxy localProxy = { local.boxPoints, local.base.vertexCount, 0.0f };
		int localHits = 0;
		b3World_OverlapShape( worldId, origin, &localProxy, filter, CountOverlapCallback, &localHits );

		ENSURE( bakedHits == 0 );
		ENSURE( localHits == bakedHits );
	}

	b3DestroyWorld( worldId );
	return 0;
}

// A quarter turn baked into the query hull must reach the overlap test. A long thin bar hits a body
// off the origin when aligned along X and clears it once rotated to lie along Z.
static int OverlapHullProxyRotation( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ 3.0f, 0.0f, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3World_Step( worldId, 1.0f / 60.0f, 1 );

	b3QueryFilter filter = b3DefaultQueryFilter();

	// Bar long in local X, centered at the origin, reaches the body at x = 3.
	b3BoxHull aligned = b3MakeTransformedBoxHull( 4.0f, 0.3f, 0.3f, b3Transform_identity );
	b3ShapeProxy alignedProxy = { aligned.boxPoints, aligned.base.vertexCount, 0.0f };
	int alignedHits = 0;
	b3World_OverlapShape( worldId, b3Pos_zero, &alignedProxy, filter, CountOverlapCallback, &alignedHits );
	ENSURE( alignedHits == 1 );

	// Rotated a quarter turn about Y the long axis points along Z, so the bar no longer reaches x = 3.
	b3Quat q = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 1.0f, 0.0f }, 0.5f * B3_PI );
	b3BoxHull turned = b3MakeTransformedBoxHull( 4.0f, 0.3f, 0.3f, (b3Transform){ b3Vec3_zero, q } );
	b3ShapeProxy turnedProxy = { turned.boxPoints, turned.base.vertexCount, 0.0f };
	int turnedHits = 0;
	b3World_OverlapShape( worldId, b3Pos_zero, &turnedProxy, filter, CountOverlapCallback, &turnedHits );
	ENSURE( turnedHits == 0 );

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

// TimeOfImpactMover ------------------------------------------------------------------------

// The mover capsule is expressed in the query frame, so a core segment starting at the origin
// stands the character on the query point. Targets sit on the sweep line at y = 0.
static b3Capsule MakeStandingMover( void )
{
	return (b3Capsule){ { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 0.25f };
}

static int MoverTOIHitsBox( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId shapeId = b3CreateHullShape( bodyId, &shapeDef, &box.base );

	// Face at x = 4.5, mover radius 0.25, so 4.25 of the 10 unit sweep is free.
	b3Capsule mover = MakeStandingMover();
	b3WorldTransform bodyTransform = IdentityAt( 5.0f, 0.0f, 0.0f );
	b3BodyTOIResult result = b3Body_TimeOfImpactMover( bodyId, b3Pos_zero, &mover, (b3Vec3){ 10.0f, 0.0f, 0.0f },
													  b3DefaultQueryFilter(), bodyTransform, bodyTransform );

	ENSURE( result.state == b3_toiStateHit );
	ENSURE_SMALL( result.fraction - 0.425f, 1e-2f );
	ENSURE( b3IsNormalized( result.normal ) );
	ENSURE_SMALL( result.normal.x - 1.0f, 1e-3f );

	// The result carries a shape id, so the hit shape must come back identified.
	ENSURE( b3Shape_IsValid( result.shapeId ) );
	ENSURE( result.shapeId.index1 == shapeId.index1 );
	ENSURE( result.shapeId.generation == shapeId.generation );

	b3DestroyWorld( worldId );
	return 0;
}

static int MoverTOISeparated( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	// Sweeping along +Y holds the X gap at 4.25 for the whole interval.
	b3Capsule mover = MakeStandingMover();
	b3WorldTransform bodyTransform = IdentityAt( 5.0f, 0.0f, 0.0f );
	b3BodyTOIResult result = b3Body_TimeOfImpactMover( bodyId, b3Pos_zero, &mover, (b3Vec3){ 0.0f, 10.0f, 0.0f },
													  b3DefaultQueryFilter(), bodyTransform, bodyTransform );

	ENSURE( result.state == b3_toiStateSeparated );
	ENSURE_SMALL( result.fraction - 1.0f, 1e-6f );
	ENSURE( b3Shape_IsValid( result.shapeId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

static int MoverTOIOverlapped( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId shapeId = b3CreateHullShape( bodyId, &shapeDef, &box.base );

	// Mover starts buried in the box, so there is no free interval to search.
	b3Capsule mover = MakeStandingMover();
	b3WorldTransform bodyTransform = IdentityAt( 0.0f, 0.0f, 0.0f );
	b3BodyTOIResult result = b3Body_TimeOfImpactMover( bodyId, b3Pos_zero, &mover, (b3Vec3){ 10.0f, 0.0f, 0.0f },
													  b3DefaultQueryFilter(), bodyTransform, bodyTransform );

	ENSURE( result.state == b3_toiStateOverlapped );
	ENSURE_SMALL( result.fraction, 1e-6f );
	ENSURE( result.shapeId.index1 == shapeId.index1 );

	b3DestroyWorld( worldId );
	return 0;
}

// Two shapes on the sweep line must resolve to the nearer one whatever order the shape list
// hands them to the loop. Shapes are pushed on the head of the list, so the two bodies below
// walk their shapes in opposite orders.
static int MoverTOIClosestShape( void )
{
	b3BodyId nearFirstId;
	b3WorldId worldId = CreateQueryWorld( &nearFirstId );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId nearLastId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere nearSphere = { { 5.0f, 0.0f, 0.0f }, 0.5f };
	b3Sphere farSphere = { { 9.0f, 0.0f, 0.0f }, 0.5f };

	b3ShapeId nearFirstHit = b3CreateSphereShape( nearFirstId, &shapeDef, &nearSphere );
	b3CreateSphereShape( nearFirstId, &shapeDef, &farSphere );

	b3CreateSphereShape( nearLastId, &shapeDef, &farSphere );
	b3ShapeId nearLastHit = b3CreateSphereShape( nearLastId, &shapeDef, &nearSphere );

	b3Capsule mover = MakeStandingMover();
	b3WorldTransform bodyTransform = IdentityAt( 0.0f, 0.0f, 0.0f );
	b3Vec3 translation = { 10.0f, 0.0f, 0.0f };

	b3BodyTOIResult nearFirst = b3Body_TimeOfImpactMover( nearFirstId, b3Pos_zero, &mover, translation, b3DefaultQueryFilter(),
														 bodyTransform, bodyTransform );
	b3BodyTOIResult nearLast = b3Body_TimeOfImpactMover( nearLastId, b3Pos_zero, &mover, translation, b3DefaultQueryFilter(),
														bodyTransform, bodyTransform );

	ENSURE( nearFirst.state == b3_toiStateHit );
	ENSURE_SMALL( nearFirst.fraction - 0.425f, 1e-2f );
	ENSURE( nearFirst.shapeId.index1 == nearFirstHit.index1 );

	ENSURE( nearLast.state == b3_toiStateHit );
	ENSURE_SMALL( nearLast.fraction - nearFirst.fraction, 1e-4f );
	ENSURE( nearLast.shapeId.index1 == nearLastHit.index1 );

	b3DestroyWorld( worldId );
	return 0;
}

// A shape the sweep clears must not erase a hit found earlier in the list. The off path sphere
// is created first so the list hands it over last.
static int MoverTOIKeepsHitAfterMiss( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere offPath = { { 5.0f, 0.0f, 20.0f }, 0.5f };
	b3Sphere onPath = { { 5.0f, 0.0f, 0.0f }, 0.5f };
	b3CreateSphereShape( bodyId, &shapeDef, &offPath );
	b3ShapeId onPathId = b3CreateSphereShape( bodyId, &shapeDef, &onPath );

	b3Capsule mover = MakeStandingMover();
	b3WorldTransform bodyTransform = IdentityAt( 0.0f, 0.0f, 0.0f );
	b3BodyTOIResult result = b3Body_TimeOfImpactMover( bodyId, b3Pos_zero, &mover, (b3Vec3){ 10.0f, 0.0f, 0.0f },
													  b3DefaultQueryFilter(), bodyTransform, bodyTransform );

	ENSURE( result.state == b3_toiStateHit );
	ENSURE_SMALL( result.fraction - 0.425f, 1e-2f );
	ENSURE( result.shapeId.index1 == onPathId.index1 );

	b3DestroyWorld( worldId );
	return 0;
}

// The mover capsule points live in the query frame, so sliding both the mover and the body by
// the same offset must not move the impact.
static int MoverTOIMoverOffset( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3Vec3 translation = { 10.0f, 0.0f, 0.0f };
	b3QueryFilter filter = b3DefaultQueryFilter();

	b3Capsule atOrigin = MakeStandingMover();
	b3WorldTransform transformA = IdentityAt( 5.0f, 0.0f, 0.0f );
	b3BodyTOIResult resultA =
		b3Body_TimeOfImpactMover( bodyId, b3Pos_zero, &atOrigin, translation, filter, transformA, transformA );

	b3Capsule offset = { { 2.0f, 0.0f, 0.0f }, { 2.0f, 1.0f, 0.0f }, 0.25f };
	b3WorldTransform transformB = IdentityAt( 7.0f, 0.0f, 0.0f );
	b3BodyTOIResult resultB = b3Body_TimeOfImpactMover( bodyId, b3Pos_zero, &offset, translation, filter, transformB, transformB );

	ENSURE( resultA.state == b3_toiStateHit );
	ENSURE( resultB.state == b3_toiStateHit );
	ENSURE_SMALL( resultB.fraction - resultA.fraction, 1e-4f );
	ENSURE_SMALL( resultB.point.x - resultA.point.x - 2.0f, 1e-3f );

	b3DestroyWorld( worldId );
	return 0;
}

// The body sweeps between the two supplied transforms. A bar spinning a quarter turn about Y
// reaches a mover that its start pose clears.
static int MoverTOIRotatingBody( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull bar = b3MakeBoxHull( 2.0f, 0.25f, 0.25f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &bar.base );

	// Query origin puts the mover at world (0,0,2.1), just inside the swept end of the bar.
	b3Pos origin = { 0.0f, 0.0f, 2.1f };
	b3Capsule mover = MakeStandingMover();
	b3WorldTransform transform1 = IdentityAt( 0.0f, 0.0f, 0.0f );
	b3WorldTransform transform2 = {
		.p = (b3Pos){ 0.0f, 0.0f, 0.0f },
		.q = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 1.0f, 0.0f }, 0.5f * B3_PI ),
	};

	b3BodyTOIResult spinning =
		b3Body_TimeOfImpactMover( bodyId, origin, &mover, b3Vec3_zero, b3DefaultQueryFilter(), transform1, transform2 );

	ENSURE( spinning.state == b3_toiStateHit );
	ENSURE( 0.0f < spinning.fraction && spinning.fraction < 1.0f );

	// Holding the start pose leaves the bar along X and well clear.
	b3BodyTOIResult still =
		b3Body_TimeOfImpactMover( bodyId, origin, &mover, b3Vec3_zero, b3DefaultQueryFilter(), transform1, transform1 );

	ENSURE( still.state == b3_toiStateSeparated );

	b3DestroyWorld( worldId );
	return 0;
}

// Everything is re-centered on the origin, so a huge origin must not shift the fraction and the
// hit point comes back in the origin frame.
static int MoverTOIFarFromOrigin( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3Pos origin = { 1.0e6f, -2.0e6f, 5.0e5f };
	b3Capsule mover = MakeStandingMover();
	b3WorldTransform bodyTransform = { .p = b3OffsetPos( origin, (b3Vec3){ 5.0f, 0.0f, 0.0f } ), .q = b3Quat_identity };
	b3BodyTOIResult result = b3Body_TimeOfImpactMover( bodyId, origin, &mover, (b3Vec3){ 10.0f, 0.0f, 0.0f },
													  b3DefaultQueryFilter(), bodyTransform, bodyTransform );

	ENSURE( result.state == b3_toiStateHit );
	ENSURE_SMALL( result.fraction - 0.425f, 1e-2f );
	ENSURE_SMALL( result.normal.x - 1.0f, 1e-3f );
	ENSURE_SMALL( (result.point.x - origin.x) - 4.4f, 0.5f );

	b3DestroyWorld( worldId );
	return 0;
}

static int MoverTOIFilter( void )
{
	b3BodyId bodyId;
	b3WorldId worldId = CreateQueryWorld( &bodyId );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	// Geometry is on the sweep line, but a zero mask rejects every category.
	b3Capsule mover = MakeStandingMover();
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = 0;
	b3WorldTransform bodyTransform = IdentityAt( 5.0f, 0.0f, 0.0f );
	b3BodyTOIResult result = b3Body_TimeOfImpactMover( bodyId, b3Pos_zero, &mover, (b3Vec3){ 10.0f, 0.0f, 0.0f }, filter,
													  bodyTransform, bodyTransform );

	ENSURE( result.state == b3_toiStateSeparated );
	ENSURE_SMALL( result.fraction - 1.0f, 1e-6f );

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
	RUN_SUBTEST( OverlapHullProxyEquivalence );
	RUN_SUBTEST( OverlapHullProxyRotation );

	RUN_SUBTEST( MoverTouchesBox );
	RUN_SUBTEST( MoverSeparated );
	RUN_SUBTEST( MoverRotatedBody );
	RUN_SUBTEST( MoverCapacity );
	RUN_SUBTEST( MoverTOIHitsBox );
	RUN_SUBTEST( MoverTOISeparated );
	RUN_SUBTEST( MoverTOIOverlapped );
	RUN_SUBTEST( MoverTOIClosestShape );
	RUN_SUBTEST( MoverTOIKeepsHitAfterMiss );
	RUN_SUBTEST( MoverTOIMoverOffset );
	RUN_SUBTEST( MoverTOIRotatingBody );
	RUN_SUBTEST( MoverTOIFarFromOrigin );
	RUN_SUBTEST( MoverTOIFilter );

	return 0;
}
