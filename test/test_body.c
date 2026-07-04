// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

// Reach into internals to observe body extents and the dirty mass flag.
#include "body.h"
#include "physics_world.h"

#include <float.h>

// b3UpdateBodyMassData shifts each shape's inertia to the body center of mass with the parallel
// axis theorem. When shapes sit far from the body origin the shift term dwarfs the central inertia,
// so any error in the per shape framing blows up the tensor. Spheres make a clean oracle: the
// central inertia is isotropic and independent of placement, so the shift is the only thing tested.

static b3MassData SphereBodyMass( const b3Vec3* centers, int count, float radius, float density )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = density;

	for ( int i = 0; i < count; ++i )
	{
		b3Sphere sphere = { centers[i], radius };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}

	b3Body_ApplyMassFromShapes( bodyId );
	b3MassData massData = b3Body_GetMassData( bodyId );

	b3DestroyWorld( worldId );
	return massData;
}

// One sphere far from the body origin. The center of mass lands on the sphere and the inertia about
// it must be the bare central inertia, with no trace of the offset.
static int FarSingleSphereMass( void )
{
	float radius = 0.5f;
	float density = 1.0f;
	b3Vec3 center = { 100.0f, -50.0f, 75.0f };
	b3MassData md = SphereBodyMass( &center, 1, radius, density );

	float mass = density * ( 4.0f / 3.0f ) * B3_PI * radius * radius * radius;
	float central = 0.4f * mass * radius * radius;

	ENSURE_SMALL( md.mass - mass, 1e-4f );

	ENSURE_SMALL( md.center.x - center.x, 1e-3f );
	ENSURE_SMALL( md.center.y - center.y, 1e-3f );
	ENSURE_SMALL( md.center.z - center.z, 1e-3f );

	ENSURE_SMALL( md.inertia.cx.x - central, 1e-3f );
	ENSURE_SMALL( md.inertia.cy.y - central, 1e-3f );
	ENSURE_SMALL( md.inertia.cz.z - central, 1e-3f );

	ENSURE_SMALL( md.inertia.cy.x, 1e-3f );
	ENSURE_SMALL( md.inertia.cz.x, 1e-3f );
	ENSURE_SMALL( md.inertia.cz.y, 1e-3f );

	return 0;
}

// Eight equal spheres on the corners of a cube, the whole cube parked far from the body origin.
// The center of mass is the cube center and the products of inertia cancel by symmetry, so the
// tensor stays diagonal no matter how far out the cube sits.
static int FarCubeSphereMass( void )
{
	float radius = 0.5f;
	float density = 1.0f;
	float h = 1.0f;
	b3Vec3 p = { 100.0f, 100.0f, 100.0f };

	b3Vec3 centers[8];
	int k = 0;
	for ( int sx = -1; sx <= 1; sx += 2 )
	{
		for ( int sy = -1; sy <= 1; sy += 2 )
		{
			for ( int sz = -1; sz <= 1; sz += 2 )
			{
				centers[k++] = (b3Vec3){ p.x + sx * h, p.y + sy * h, p.z + sz * h };
			}
		}
	}

	b3MassData md = SphereBodyMass( centers, 8, radius, density );

	float mass = density * ( 4.0f / 3.0f ) * B3_PI * radius * radius * radius;
	float totalMass = 8.0f * mass;

	// Per sphere central inertia summed, plus the parallel axis term for each corner offset
	// (dy^2 + dz^2) = (h^2 + h^2) about every axis.
	float diag = 8.0f * 0.4f * mass * radius * radius + 16.0f * mass * h * h;

	ENSURE_SMALL( md.mass - totalMass, 1e-3f );

	ENSURE_SMALL( md.center.x - p.x, 1e-2f );
	ENSURE_SMALL( md.center.y - p.y, 1e-2f );
	ENSURE_SMALL( md.center.z - p.z, 1e-2f );

	ENSURE_SMALL( md.inertia.cx.x - diag, 1e-2f );
	ENSURE_SMALL( md.inertia.cy.y - diag, 1e-2f );
	ENSURE_SMALL( md.inertia.cz.z - diag, 1e-2f );

	ENSURE_SMALL( md.inertia.cy.x, 1e-2f );
	ENSURE_SMALL( md.inertia.cz.x, 1e-2f );
	ENSURE_SMALL( md.inertia.cz.y, 1e-2f );

	return 0;
}

// Shapes added with updateBodyMass = false defer the mass update, which is also the only place
// body extents are computed. A body that reaches the solver with minExtent == B3_HUGE never passes
// the continuous collision gate. The dirty mass flag must track the deferral, and both
// ApplyMassFromShapes and SetMassData must leave finite extents behind.
static int DeferredMassExtents( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.updateBodyMass = false;

	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };

	// Deferred create leaves mass and extents untouched but marks the body dirty.
	b3BodyId applyId = b3CreateBody( worldId, &bodyDef );
	b3CreateSphereShape( applyId, &shapeDef, &sphere );

	b3World* world = b3GetWorld( applyId.world0 );
	b3Body* applyBody = b3GetBodyFullId( world, applyId );
	b3BodySim* applySim = b3GetBodySim( world, applyBody );

	ENSURE( ( applyBody->flags & b3_dirtyMass ) != 0 );
	ENSURE( applySim->minExtent == B3_HUGE );

	// ApplyMassFromShapes computes extents and clears the flag.
	b3Body_ApplyMassFromShapes( applyId );
	ENSURE( ( applyBody->flags & b3_dirtyMass ) == 0 );
	ENSURE( applySim->minExtent < B3_HUGE );

	// SetMassData alone must also produce finite extents and clear the flag (the issue #35 repro).
	b3BodyId massId = b3CreateBody( worldId, &bodyDef );
	b3CreateSphereShape( massId, &shapeDef, &sphere );

	b3Body* massBody = b3GetBodyFullId( world, massId );
	b3BodySim* massSim = b3GetBodySim( world, massBody );
	ENSURE( ( massBody->flags & b3_dirtyMass ) != 0 );

	b3Matrix3 inertia = { { 0.2f, 0.0f, 0.0f }, { 0.0f, 0.2f, 0.0f }, { 0.0f, 0.0f, 0.2f } };
	b3MassData massData = { 2.0f, { 0.0f, 0.0f, 0.0f }, inertia };
	b3Body_SetMassData( massId, massData );

	ENSURE( ( massBody->flags & b3_dirtyMass ) == 0 );
	ENSURE( massSim->minExtent < B3_HUGE );

	b3DestroyWorld( worldId );
	return 0;
}

// b3Body_SetMassData overrides the mass properties directly, bypassing the shapes. It must derive
// everything the solver reads from the supplied tensor: the inverse mass, the local inverse inertia,
// and the world inverse inertia rotated by the body orientation. Fixed rotation zeros the angular part.
// These tests drive it through the public getters, no shapes required.

// Diagonal inertia with inverses that are exact in float, so tolerances stay tight.
static const b3Matrix3 kDiagInertia = { { 2.0f, 0.0f, 0.0f }, { 0.0f, 4.0f, 0.0f }, { 0.0f, 0.0f, 8.0f } };

static int SetMassDataRoundTrip( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 5.0f, -3.0f, 2.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3Vec3 center = { 0.1f, 0.2f, 0.3f };
	b3MassData massData = { 3.0f, center, kDiagInertia };
	b3Body_SetMassData( bodyId, massData );

	ENSURE_SMALL( b3Body_GetMass( bodyId ) - 3.0f, 1e-6f );
	ENSURE_SMALL( b3Body_GetInverseMass( bodyId ) - 1.0f / 3.0f, 1e-6f );

	b3MassData md = b3Body_GetMassData( bodyId );
	ENSURE_SMALL( md.mass - 3.0f, 1e-6f );
	ENSURE_SMALL( md.center.x - center.x, 1e-6f );
	ENSURE_SMALL( md.center.y - center.y, 1e-6f );
	ENSURE_SMALL( md.center.z - center.z, 1e-6f );
	ENSURE_SMALL( md.inertia.cx.x - 2.0f, 1e-6f );
	ENSURE_SMALL( md.inertia.cy.y - 4.0f, 1e-6f );
	ENSURE_SMALL( md.inertia.cz.z - 8.0f, 1e-6f );

	b3Vec3 localCenter = b3Body_GetLocalCenter( bodyId );
	ENSURE_SMALL( localCenter.x - center.x, 1e-6f );
	ENSURE_SMALL( localCenter.y - center.y, 1e-6f );
	ENSURE_SMALL( localCenter.z - center.z, 1e-6f );

	b3Matrix3 localInertia = b3Body_GetLocalRotationalInertia( bodyId );
	ENSURE_SMALL( localInertia.cx.x - 2.0f, 1e-6f );
	ENSURE_SMALL( localInertia.cy.y - 4.0f, 1e-6f );
	ENSURE_SMALL( localInertia.cz.z - 8.0f, 1e-6f );

	// Identity rotation, so the world inverse inertia is just the local inverse: diag(1/2, 1/4, 1/8).
	b3Matrix3 invWorld = b3Body_GetWorldInverseRotationalInertia( bodyId );
	ENSURE_SMALL( invWorld.cx.x - 0.5f, 1e-5f );
	ENSURE_SMALL( invWorld.cy.y - 0.25f, 1e-5f );
	ENSURE_SMALL( invWorld.cz.z - 0.125f, 1e-5f );
	ENSURE_SMALL( invWorld.cy.x, 1e-5f );
	ENSURE_SMALL( invWorld.cz.x, 1e-5f );
	ENSURE_SMALL( invWorld.cx.y, 1e-5f );
	ENSURE_SMALL( invWorld.cz.y, 1e-5f );
	ENSURE_SMALL( invWorld.cx.z, 1e-5f );
	ENSURE_SMALL( invWorld.cy.z, 1e-5f );

	// World center of mass is the body origin plus the local center under identity rotation.
	b3Pos worldCenter = b3Body_GetWorldCenter( bodyId );
	ENSURE_SMALL( worldCenter.x - ( 5.0f + center.x ), 1e-5f );
	ENSURE_SMALL( worldCenter.y - ( -3.0f + center.y ), 1e-5f );
	ENSURE_SMALL( worldCenter.z - ( 2.0f + center.z ), 1e-5f );

	b3DestroyWorld( worldId );
	return 0;
}

// The world inverse inertia must be the local inverse rotated into world space. A 90 degree turn about
// z swaps the x and y principal moments, so diag(1/2, 1/4, 1/8) becomes diag(1/4, 1/2, 1/8). This is the
// regression guard: before SetMassData rotated the tensor it left the world inverse inertia stale.
static int SetMassDataWorldInertiaRotated( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI );
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3MassData massData = { 1.0f, { 0.0f, 0.0f, 0.0f }, kDiagInertia };
	b3Body_SetMassData( bodyId, massData );

	// The local inertia is stored untouched by the world transform.
	b3Matrix3 localInertia = b3Body_GetLocalRotationalInertia( bodyId );
	ENSURE_SMALL( localInertia.cx.x - 2.0f, 1e-6f );
	ENSURE_SMALL( localInertia.cy.y - 4.0f, 1e-6f );
	ENSURE_SMALL( localInertia.cz.z - 8.0f, 1e-6f );

	b3Matrix3 invWorld = b3Body_GetWorldInverseRotationalInertia( bodyId );
	ENSURE_SMALL( invWorld.cx.x - 0.25f, 1e-4f );
	ENSURE_SMALL( invWorld.cy.y - 0.5f, 1e-4f );
	ENSURE_SMALL( invWorld.cz.z - 0.125f, 1e-4f );
	ENSURE_SMALL( invWorld.cy.x, 1e-4f );
	ENSURE_SMALL( invWorld.cz.x, 1e-4f );
	ENSURE_SMALL( invWorld.cx.y, 1e-4f );
	ENSURE_SMALL( invWorld.cz.y, 1e-4f );
	ENSURE_SMALL( invWorld.cx.z, 1e-4f );
	ENSURE_SMALL( invWorld.cy.z, 1e-4f );

	b3DestroyWorld( worldId );
	return 0;
}

// Fixed rotation must leave the mass intact but drive the whole angular inertia to zero, even when the
// caller hands in a real tensor.
static int SetMassDataFixedRotation( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.motionLocks.angularX = true;
	bodyDef.motionLocks.angularY = true;
	bodyDef.motionLocks.angularZ = true;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3MassData massData = { 5.0f, { 0.0f, 0.0f, 0.0f }, kDiagInertia };
	b3Body_SetMassData( bodyId, massData );

	ENSURE_SMALL( b3Body_GetMass( bodyId ) - 5.0f, 1e-6f );
	ENSURE_SMALL( b3Body_GetInverseMass( bodyId ) - 0.2f, 1e-6f );

	b3Matrix3 localInertia = b3Body_GetLocalRotationalInertia( bodyId );
	ENSURE_SMALL( localInertia.cx.x, 1e-6f );
	ENSURE_SMALL( localInertia.cy.y, 1e-6f );
	ENSURE_SMALL( localInertia.cz.z, 1e-6f );

	b3Matrix3 invWorld = b3Body_GetWorldInverseRotationalInertia( bodyId );
	ENSURE_SMALL( invWorld.cx.x, 1e-6f );
	ENSURE_SMALL( invWorld.cy.y, 1e-6f );
	ENSURE_SMALL( invWorld.cz.z, 1e-6f );

	b3MassData md = b3Body_GetMassData( bodyId );
	ENSURE_SMALL( md.inertia.cx.x, 1e-6f );
	ENSURE_SMALL( md.inertia.cy.y, 1e-6f );
	ENSURE_SMALL( md.inertia.cz.z, 1e-6f );

	b3DestroyWorld( worldId );
	return 0;
}

// Zero mass and a zero tensor have zero determinant, so the inverses must collapse to zero rather than
// divide by it.
static int SetMassDataZeroMass( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3MassData massData = { 0.0f, { 0.0f, 0.0f, 0.0f }, b3Mat3_zero };
	b3Body_SetMassData( bodyId, massData );

	ENSURE_SMALL( b3Body_GetInverseMass( bodyId ), 1e-6f );

	b3Matrix3 localInertia = b3Body_GetLocalRotationalInertia( bodyId );
	ENSURE_SMALL( localInertia.cx.x, 1e-6f );
	ENSURE_SMALL( localInertia.cy.y, 1e-6f );
	ENSURE_SMALL( localInertia.cz.z, 1e-6f );

	b3Matrix3 invWorld = b3Body_GetWorldInverseRotationalInertia( bodyId );
	ENSURE_SMALL( invWorld.cx.x, 1e-6f );
	ENSURE_SMALL( invWorld.cy.y, 1e-6f );
	ENSURE_SMALL( invWorld.cz.z, 1e-6f );

	b3DestroyWorld( worldId );
	return 0;
}

// The stored linear velocity tracks the center of mass. Moving the center picks a different material
// point, so a spinning body must have its velocity re-referenced by omega x (newCenter - oldCenter),
// otherwise the mass edit silently injects or drains kinetic energy. Under identity rotation the world
// shift equals the supplied local center, keeping the expected values exact in float.
static int SetMassDataConsistentVelocity( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 7.0f, 1.0f, -4.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	// Spin about the origin center, then shift the center of mass off the origin.
	b3Vec3 omega = { 1.0f, 2.0f, 4.0f };
	b3Body_SetLinearVelocity( bodyId, (b3Vec3){ 1.0f, -2.0f, 3.0f } );
	b3Body_SetAngularVelocity( bodyId, omega );

	b3Vec3 center = { 0.5f, 0.25f, 0.125f };
	b3MassData massData = { 3.0f, center, kDiagInertia };
	b3Body_SetMassData( bodyId, massData );

	// omega x center = ( 2*0.125 - 4*0.25, 4*0.5 - 1*0.125, 1*0.25 - 2*0.5 ) = ( -0.75, 1.875, -0.75 )
	b3Vec3 v = b3Body_GetLinearVelocity( bodyId );
	ENSURE_SMALL( v.x - ( 1.0f - 0.75f ), 1e-6f );
	ENSURE_SMALL( v.y - ( -2.0f + 1.875f ), 1e-6f );
	ENSURE_SMALL( v.z - ( 3.0f - 0.75f ), 1e-6f );

	// Only the reference point moved, the angular velocity is untouched.
	b3Vec3 w = b3Body_GetAngularVelocity( bodyId );
	ENSURE_SMALL( w.x - omega.x, 1e-6f );
	ENSURE_SMALL( w.y - omega.y, 1e-6f );
	ENSURE_SMALL( w.z - omega.z, 1e-6f );

	b3DestroyWorld( worldId );
	return 0;
}

int BodyTest( void )
{
	RUN_SUBTEST( FarSingleSphereMass );
	RUN_SUBTEST( FarCubeSphereMass );
	RUN_SUBTEST( DeferredMassExtents );
	RUN_SUBTEST( SetMassDataRoundTrip );
	RUN_SUBTEST( SetMassDataWorldInertiaRotated );
	RUN_SUBTEST( SetMassDataFixedRotation );
	RUN_SUBTEST( SetMassDataZeroMass );
	RUN_SUBTEST( SetMassDataConsistentVelocity );
	return 0;
}
