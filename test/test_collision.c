// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "aabb.h"
#include "test_macros.h"

#include "box3d/math_functions.h"

static int AABBTest( void )
{
	b3AABB a;
	a.lowerBound = (b3Vec3){ -1.0f, -1.0f, -1.0f };
	a.upperBound = (b3Vec3){ -2.0f, -2.0f, -2.0f };

	ENSURE( b3IsValidAABB( a ) == false );

	a.upperBound = (b3Vec3){ 1.0f, 1.0f };
	ENSURE( b3IsValidAABB( a ) == true );

	b3AABB b = { { 2.0f, 2.0f }, { 4.0f, 4.0f } };
	ENSURE( b3AABB_Overlaps( a, b ) == false );
	ENSURE( b3AABB_Contains( a, b ) == false );

	return 0;
}

static int TestRayAABBIntersection( void )
{
	// Test 1: Ray passing through center of AABB
	{
		b3AABB a = { { -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f } };
		b3Vec3 p1 = { -2.0f, 0.0f, 0.0f };
		b3Vec3 p2 = { 2.0f, 0.0f, 0.0f };
		float minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( b3AbsFloat( minFraction - 0.25f ) < 0.001f ); // Enters at 25% of ray
		ENSURE( b3AbsFloat( maxFraction - 0.75f ) < 0.001f ); // Exits at 75% of ray
	}

	// Test 2: Ray starting inside AABB
	{
		b3AABB a = { { -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f } };
		b3Vec3 p1 = { 0.0f, 0.0f, 0.0f };
		b3Vec3 p2 = { 2.0f, 0.0f, 0.0f };
		float minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( minFraction == 0.0f );						 // Starts inside
		ENSURE( b3AbsFloat( maxFraction - 0.5f ) < 0.001f ); // Exits at 50% of ray
	}

	// Test 3: Ray ending inside AABB
	{
		b3AABB a = { { -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f } };
		b3Vec3 p1 = { -2.0f, 0.0f, 0.0f };
		b3Vec3 p2 = { 0.0f, 0.0f, 0.0f };
		float minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( b3AbsFloat( minFraction - 0.5f ) < 0.001f ); // Enters at 50% of ray
		ENSURE( maxFraction == 1.0f );						 // Ends inside
	}

	// Test 4: Ray completely inside AABB
	{
		b3AABB a = { { -2.0f, -2.0f, -2.0f }, { 2.0f, 2.0f, 2.0f } };
		b3Vec3 p1 = { -1.0f, 0.0f, 0.0f };
		b3Vec3 p2 = { 1.0f, 0.0f, 0.0f };
		float minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( minFraction == 0.0f );
		ENSURE( maxFraction == 1.0f );
	}

	// Test 5: Ray missing AABB
	{
		b3AABB a = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } };
		b3Vec3 p1 = { -1.0f, 2.0f, 0.5f };
		b3Vec3 p2 = { 2.0f, 2.0f, 0.5f };
		float minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == false );
	}

	// Test 6: Ray parallel to AABB face (no intersection)
	{
		b3AABB a = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } };
		b3Vec3 p1 = { -1.0f, 2.0f, 0.5f };
		b3Vec3 p2 = { 2.0f, 2.0f, 0.5f };
		float minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == false );
	}

	// Test 7: Ray parallel to AABB face (within bounds)
	{
		b3AABB a = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } };
		b3Vec3 p1 = { -1.0f, 0.5f, 0.5f };
		b3Vec3 p2 = { 2.0f, 0.5f, 0.5f };
		float minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( b3AbsFloat( minFraction - 1.0f / 3.0f ) < 0.001f );
		ENSURE( b3AbsFloat( maxFraction - 2.0f / 3.0f ) < 0.001f );
	}

	// Test 8: Degenerate ray (point) inside AABB
	{
		b3AABB a = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } };
		b3Vec3 p1 = { 0.5f, 0.5f, 0.5f };
		b3Vec3 p2 = { 0.5f, 0.5f, 0.5f };
		float minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( minFraction == 0.0f );
		ENSURE( maxFraction == 0.0f );
	}

	// Test 9: Degenerate ray (point) outside AABB
	{
		b3AABB a = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } };
		b3Vec3 p1 = { 2.0f, 2.0f, 2.0f };
		b3Vec3 p2 = { 2.0f, 2.0f, 2.0f };
		float minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == false );
	}

	// Test 10: Ray pointing away from AABB
	{
		b3AABB a = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } };
		b3Vec3 p1 = { -1.0f, 0.5f, 0.5f };
		b3Vec3 p2 = { -2.0f, 0.5f, 0.5f };
		float minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == false );
	}

	// Test 11: Ray hitting corner of AABB
	{
		b3AABB a = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } };
		b3Vec3 p1 = { -1.0f, -1.0f, -1.0f };
		b3Vec3 p2 = { 2.0f, 2.0f, 2.0f };
		float minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( b3AbsFloat( minFraction - 1.0f / 3.0f ) < 0.001f );
		ENSURE( b3AbsFloat( maxFraction - 2.0f / 3.0f ) < 0.001f );
	}

	// Test 12: Ray grazing edge of AABB
	{
		b3AABB a = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } };
		b3Vec3 p1 = { -1.0f, 0.0f, 0.5f };
		b3Vec3 p2 = { 2.0f, 0.0f, 0.5f };
		float minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( b3AbsFloat( minFraction - 1.0f / 3.0f ) < 0.001f );
		ENSURE( b3AbsFloat( maxFraction - 2.0f / 3.0f ) < 0.001f );
	}

	return 0;
}

int CollisionTest( void )
{
	RUN_SUBTEST( AABBTest );
	RUN_SUBTEST( TestRayAABBIntersection );

	return 0;
}
