// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

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

int BodyTest( void )
{
	RUN_SUBTEST( FarSingleSphereMass );
	RUN_SUBTEST( FarCubeSphereMass );
	return 0;
}
