// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <math.h>
#include <string.h>

static const b3Vec3 s_cubeCorners[8] = {
	{ 1.0f, 1.0f, 1.0f },	{ -1.0f, 1.0f, 1.0f },	 { -1.0f, -1.0f, 1.0f },  { 1.0f, -1.0f, 1.0f },
	{ 1.0f, 1.0f, -1.0f },	{ -1.0f, 1.0f, -1.0f },	 { -1.0f, -1.0f, -1.0f }, { 1.0f, -1.0f, -1.0f },
};

static int CreateHullCubeTest( void )
{
	b3HullData* hull = b3CreateHull( s_cubeCorners, 8, 8 );
	ENSURE( hull != NULL );

	ENSURE( hull->vertexCount == 8 );
	ENSURE( hull->edgeCount == 24 );
	ENSURE( hull->faceCount == 6 );

	// Euler's identity for convex polyhedron
	ENSURE( hull->vertexCount - hull->edgeCount / 2 + hull->faceCount == 2 );

	b3BoxHull ref = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );

	ENSURE_SMALL( hull->volume - ref.base.volume, 1e-4f );
	ENSURE_SMALL( hull->surfaceArea - ref.base.surfaceArea, 1e-4f );
	ENSURE_SMALL( hull->innerRadius - ref.base.innerRadius, FLT_EPSILON );

	ENSURE_SMALL( hull->center.x - ref.base.center.x, 1e-5f );
	ENSURE_SMALL( hull->center.y - ref.base.center.y, 1e-5f );
	ENSURE_SMALL( hull->center.z - ref.base.center.z, 1e-5f );

	ENSURE_SMALL( hull->aabb.lowerBound.x + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.lowerBound.y + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.lowerBound.z + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.upperBound.x - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.upperBound.y - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.upperBound.z - 1.0f, FLT_EPSILON );

	b3Matrix3 d = b3SubMM( hull->centralInertia, ref.base.centralInertia );
	ENSURE_SMALL( d.cx.x, 1e-4f );
	ENSURE_SMALL( d.cy.y, 1e-4f );
	ENSURE_SMALL( d.cz.z, 1e-4f );
	ENSURE_SMALL( d.cx.y, 1e-4f );
	ENSURE_SMALL( d.cx.z, 1e-4f );
	ENSURE_SMALL( d.cy.z, 1e-4f );
	ENSURE_SMALL( d.cy.x, 1e-4f );
	ENSURE_SMALL( d.cz.x, 1e-4f );
	ENSURE_SMALL( d.cz.y, 1e-4f );

	b3DestroyHull( hull );
	return 0;
}

static int CreateHullTetrahedronTest( void )
{
	b3Vec3 points[4] = {
		{ 0.0f, 0.0f, 0.0f },
		{ 1.0f, 0.0f, 0.0f },
		{ 0.0f, 1.0f, 0.0f },
		{ 0.0f, 0.0f, 1.0f },
	};

	b3HullData* hull = b3CreateHull( points, 4, 4 );
	ENSURE( hull != NULL );

	ENSURE( hull->vertexCount == 4 );
	ENSURE( hull->edgeCount == 12 );
	ENSURE( hull->faceCount == 4 );
	ENSURE( hull->vertexCount - hull->edgeCount / 2 + hull->faceCount == 2 );

	// Analytic values for the unit-corner tetrahedron at the origin.
	float expectedVolume = 1.0f / 6.0f;
	float expectedSurfaceArea = 1.5f + 0.5f * sqrtf( 3.0f );
	float expectedInnerRadius = 0.25f / sqrtf( 3.0f );

	ENSURE_SMALL( hull->volume - expectedVolume, 1e-5f );
	ENSURE_SMALL( hull->surfaceArea - expectedSurfaceArea, 1e-5f );
	ENSURE_SMALL( hull->innerRadius - expectedInnerRadius, 1e-5f );

	ENSURE_SMALL( hull->center.x - 0.25f, 1e-5f );
	ENSURE_SMALL( hull->center.y - 0.25f, 1e-5f );
	ENSURE_SMALL( hull->center.z - 0.25f, 1e-5f );

	ENSURE_SMALL( hull->aabb.lowerBound.x, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.lowerBound.y, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.lowerBound.z, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.upperBound.x - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.upperBound.y - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.upperBound.z - 1.0f, FLT_EPSILON );

	b3DestroyHull( hull );
	return 0;
}

static int CreateHullDeterminismTest( void )
{
	b3HullData* h1 = b3CreateHull( s_cubeCorners, 8, 8 );
	b3HullData* h2 = b3CreateHull( s_cubeCorners, 8, 8 );
	ENSURE( h1 != NULL && h2 != NULL );

	ENSURE( h1->byteCount == h2->byteCount );
	ENSURE( h1->hash != 0 );
	ENSURE( h1->hash == h2->hash );
	ENSURE( memcmp( h1, h2, h1->byteCount ) == 0 );

	b3DestroyHull( h1 );
	b3DestroyHull( h2 );
	return 0;
}

#define SPHERE_N 6

static int CreateHullMaxVertexTest( void )
{
	// Sphere-sampled point cloud, dense enough that the builder has room to grow.
	b3Vec3 points[SPHERE_N * SPHERE_N];
	int index = 0;
	for ( int i = 0; i < SPHERE_N; ++i )
	{
		float theta = B3_PI * (float)i / (float)( SPHERE_N - 1 );
		for ( int j = 0; j < SPHERE_N; ++j )
		{
			float phi = 2.0f * B3_PI * (float)j / (float)SPHERE_N;
			points[index].x = sinf( theta ) * cosf( phi );
			points[index].y = sinf( theta ) * sinf( phi );
			points[index].z = cosf( theta );
			index += 1;
		}
	}
	int count = SPHERE_N * SPHERE_N;

	// maxVertexCount honored as a strict cap.
	b3HullData* h1 = b3CreateHull( points, count, 8 );
	ENSURE( h1 != NULL );
	ENSURE( h1->vertexCount <= 8 );
	b3DestroyHull( h1 );

	// Below the floor: clamps up to 4.
	b3HullData* h2 = b3CreateHull( points, count, 1 );
	ENSURE( h2 != NULL );
	ENSURE( h2->vertexCount >= 4 && h2->vertexCount <= B3_MAX_HULL_VERTICES );
	b3DestroyHull( h2 );

	// Above the ceiling: clamps down to B3_MAX_HULL_VERTICES.
	b3HullData* h3 = b3CreateHull( points, count, 1000 );
	ENSURE( h3 != NULL );
	ENSURE( h3->vertexCount >= 4 && h3->vertexCount <= B3_MAX_HULL_VERTICES );
	b3DestroyHull( h3 );

	return 0;
}

static int CreateHullRedundantInputTest( void )
{
	// 8 cube corners + duplicates + interior points. Builder should produce the cube.
	b3Vec3 points[20] = {
		{ 1.0f, 1.0f, 1.0f },	// corners
		{ -1.0f, 1.0f, 1.0f },
		{ -1.0f, -1.0f, 1.0f },
		{ 1.0f, -1.0f, 1.0f },
		{ 1.0f, 1.0f, -1.0f },
		{ -1.0f, 1.0f, -1.0f },
		{ -1.0f, -1.0f, -1.0f },
		{ 1.0f, -1.0f, -1.0f },
		{ 1.0f, 1.0f, 1.0f },	// duplicates
		{ 1.0f, 1.0f, 1.0f },
		{ 0.0f, 0.0f, 0.0f },	// interior points
		{ 0.5f, 0.0f, 0.0f },
		{ 0.0f, 0.5f, 0.0f },
		{ 0.0f, 0.0f, 0.5f },
		{ -0.5f, 0.0f, 0.0f },
		{ 0.0f, -0.5f, 0.0f },
		{ 0.0f, 0.0f, -0.5f },
		{ 0.25f, 0.25f, 0.25f },
		{ -0.25f, -0.25f, -0.25f },
		{ 0.5f, 0.5f, 0.5f },
	};

	b3HullData* hull = b3CreateHull( points, 20, 8 );
	ENSURE( hull != NULL );

	ENSURE( hull->vertexCount == 8 );
	ENSURE( hull->edgeCount == 24 );
	ENSURE( hull->faceCount == 6 );

	b3BoxHull ref = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );

	ENSURE_SMALL( hull->volume - ref.base.volume, 1e-4f );
	ENSURE_SMALL( hull->surfaceArea - ref.base.surfaceArea, 1e-4f );
	ENSURE_SMALL( hull->innerRadius - ref.base.innerRadius, FLT_EPSILON );

	ENSURE_SMALL( hull->center.x - ref.base.center.x, 1e-5f );
	ENSURE_SMALL( hull->center.y - ref.base.center.y, 1e-5f );
	ENSURE_SMALL( hull->center.z - ref.base.center.z, 1e-5f );

	ENSURE_SMALL( hull->aabb.lowerBound.x + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.lowerBound.y + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.lowerBound.z + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.upperBound.x - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.upperBound.y - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.upperBound.z - 1.0f, FLT_EPSILON );

	b3DestroyHull( hull );
	return 0;
}

static int CreateHullCloneTest( void )
{
	b3HullData* original = b3CreateHull( s_cubeCorners, 8, 8 );
	ENSURE( original != NULL );

	b3HullData* clone = b3CloneHull( original );
	ENSURE( clone != NULL );
	ENSURE( clone->byteCount == original->byteCount );
	ENSURE( memcmp( clone, original, original->byteCount ) == 0 );

	b3DestroyHull( clone );
	b3DestroyHull( original );
	return 0;
}

static int CreateHullCylinderTest( void )
{
	const float height = 2.0f;
	const float radius = 1.0f;
	const int sides = 8;
	const float yOffset = 0.0f;

	b3HullData* hull = b3CreateCylinder( height, radius, yOffset, sides );
	ENSURE( hull != NULL );

	ENSURE( hull->vertexCount == 2 * sides );
	ENSURE( hull->edgeCount == 6 * sides );
	ENSURE( hull->faceCount == sides + 2 );

	// Analytic n-gon prism values (exact targets, not the circular cylinder approximations).
	float halfAngle = B3_PI / (float)sides;
	float capArea = (float)sides * 0.5f * radius * radius * sinf( 2.0f * halfAngle );
	float chordLen = 2.0f * radius * sinf( halfAngle );
	float lateralArea = (float)sides * chordLen * height;

	float expectedVolume = capArea * height;
	float expectedSurfaceArea = 2.0f * capArea + lateralArea;
	float expectedInnerRadius = radius * cosf( halfAngle );

	ENSURE_SMALL( ( hull->volume - expectedVolume ) / expectedVolume, 1e-4f );
	ENSURE_SMALL( ( hull->surfaceArea - expectedSurfaceArea ) / expectedSurfaceArea, 1e-4f );
	ENSURE_SMALL( hull->innerRadius - expectedInnerRadius, 1e-5f );

	ENSURE_SMALL( hull->center.x, 1e-5f );
	ENSURE_SMALL( hull->center.y - ( yOffset + 0.5f * height ), 1e-5f );
	ENSURE_SMALL( hull->center.z, 1e-5f );

	ENSURE_SMALL( hull->aabb.lowerBound.x + radius, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.lowerBound.y - yOffset, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.lowerBound.z + radius, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.upperBound.x - radius, FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.upperBound.y - ( yOffset + height ), FLT_EPSILON );
	ENSURE_SMALL( hull->aabb.upperBound.z - radius, FLT_EPSILON );

	b3DestroyHull( hull );
	return 0;
}

// Inlined XorShift32 + Shoemake unit-vector recipe matches shared/utils.h exactly so
// generated points are bit-identical to samples that share the seed.
static void FillSphereSample( b3Vec3* points, int count, uint32_t seed )
{
	const uint32_t RAND_LIMIT_LOCAL = 32767u;
	for ( int i = 0; i < count; ++i )
	{
		float u[3];
		for ( int k = 0; k < 3; ++k )
		{
			seed ^= seed << 13;
			seed ^= seed >> 17;
			seed ^= seed << 5;
			float r = (float)( seed & RAND_LIMIT_LOCAL );
			r /= (float)RAND_LIMIT_LOCAL;
			u[k] = r;
		}
		float u1 = u[0];
		float u2 = 2.0f * B3_PI * u[1];
		float u3 = 2.0f * B3_PI * u[2];
		float sqrt1MinusU1 = sqrtf( 1.0f - u1 );
		float sqrtU1 = sqrtf( u1 );
		points[i].x = sqrt1MinusU1 * sinf( u2 );
		points[i].y = sqrt1MinusU1 * cosf( u2 );
		points[i].z = sqrtU1 * sinf( u3 );
	}
}

// Reproduces the HullReduction sample (Sphere, 64 points, count=20) that used to assert on
// b->faceCount < b->faceCapacity. The free-list reclaim in NewFace/NewEdge keeps the bump
// counts proportional to the live hull instead of cumulative creations.
static int CreateHullSphereReductionTest( void )
{
	b3Vec3 points[64];
	FillSphereSample( points, 64, 12345 ); // RAND_SEED

	b3HullData* hull = b3CreateHull( points, 64, 20 );
	ENSURE( hull != NULL );
	ENSURE( hull->vertexCount >= 4 && hull->vertexCount <= 20 );
	ENSURE( hull->vertexCount - hull->edgeCount / 2 + hull->faceCount == 2 );
	b3DestroyHull( hull );

	return 0;
}

// XorShift32 + uniform-cube point generator. Same engine as FillSphereSample.
static void FillCubeSample( b3Vec3* points, int count, uint32_t seed )
{
	const uint32_t RAND_LIMIT_LOCAL = 32767u;
	for ( int i = 0; i < count; ++i )
	{
		float v[3];
		for ( int k = 0; k < 3; ++k )
		{
			seed ^= seed << 13;
			seed ^= seed >> 17;
			seed ^= seed << 5;
			float r = (float)( seed & RAND_LIMIT_LOCAL );
			r /= (float)RAND_LIMIT_LOCAL;
			v[k] = 2.0f * r - 1.0f;
		}
		points[i].x = v[0];
		points[i].y = v[1];
		points[i].z = v[2];
	}
}

// Pushes the working-stage bump capacities (faceCapacity = 5*M - 10, edgeCapacity =
// 24*M - 48 in src/hull.c::b3ComputeHullWorkSizes) close to their peak by sweeping M
// on a dense random sphere. If the peak face/edge count exceeds the cap and the free
// list fails to reclaim slots in time, the assertion at src/hull.c::NewFace/NewEdge
// fires (exit code 3). A dense sphere hull is fully triangulated, so F = 2M - 4 and the
// final face limit B3_MAX_HULL_FACES binds well before the vertex or edge limits.
static int CreateHullSphereStressTest( void )
{
	enum { N = 512 };
	b3Vec3 points[N];

	// Multiple seeds exercise different horizon-size / merge-cascade sequences.
	const uint32_t seeds[] = { 12345u, 1u, 0xdeadbeefu, 0xcafef00du };

	// M kept <= 32 so the final face count 2M - 4 stays under B3_MAX_HULL_FACES
	const int M_values[] = { 16, 24, 32 };

	for ( int s = 0; s < ARRAY_COUNT( seeds ); ++s )
	{
		FillSphereSample( points, N, seeds[s] );

		for ( int m = 0; m < ARRAY_COUNT( M_values ); ++m )
		{
			int M = M_values[m];
			b3HullData* hull = b3CreateHull( points, N, M );
			ENSURE( hull != NULL );
			ENSURE( hull->vertexCount >= 4 && hull->vertexCount <= M );
			ENSURE( hull->vertexCount - hull->edgeCount / 2 + hull->faceCount == 2 );
			ENSURE( hull->faceCount >= 4 );
			b3DestroyHull( hull );
		}
	}

	return 0;
}

// Random points inside a cube produce a small final hull (8 corners, 6 faces) but
// generate heavy internal churn: every interior point gets fed to the conflict-list
// machinery, and most cone faces created during apex insertion are then merged out
// when their newly-created neighbors are coplanar. Exercises the merge cascade in
// b3HullBuilder_ConnectEdges/ConnectFaces and the corresponding RetireFace/RetireEdge
// reclaim path much harder than the sphere case (which has few merges).
static int CreateHullMergeChurnStressTest( void )
{
	enum { N = 4096 };
	static b3Vec3 points[N];

	const uint32_t seeds[] = { 12345u, 0xdeadbeefu };

	for ( int s = 0; s < ARRAY_COUNT( seeds ); ++s )
	{
		FillCubeSample( points, N, seeds[s] );

		// Stamp the 8 corners last so they're guaranteed extremes.
		for ( int c = 0; c < 8; ++c )
		{
			points[N - 8 + c].x = ( c & 1 ) ? 1.0f : -1.0f;
			points[N - 8 + c].y = ( c & 2 ) ? 1.0f : -1.0f;
			points[N - 8 + c].z = ( c & 4 ) ? 1.0f : -1.0f;
		}

		b3HullData* hull = b3CreateHull( points, N, 64 );
		ENSURE( hull != NULL );
		ENSURE( hull->vertexCount == 8 );
		ENSURE( hull->edgeCount == 24 );
		ENSURE( hull->faceCount == 6 );
		b3DestroyHull( hull );
	}

	return 0;
}

// b3ComputeCosSin is a coarse approximation, so build rotations from libm to keep the
// analytic corner and plane positions exact.
static b3Quat ExactQuat( b3Vec3 axis, float radians )
{
	float half = 0.5f * radians;
	float s = sinf( half );
	return (b3Quat){ { s * axis.x, s * axis.y, s * axis.z }, cosf( half ) };
}

// Corner sign pattern baked by b3MakeTransformedBoxHull, in order.
static const b3Vec3 s_boxCornerSigns[8] = {
	{ 1.0f, 1.0f, 1.0f },  { -1.0f, 1.0f, 1.0f },	{ -1.0f, -1.0f, 1.0f },	 { 1.0f, -1.0f, 1.0f },
	{ 1.0f, 1.0f, -1.0f }, { -1.0f, 1.0f, -1.0f },	{ -1.0f, -1.0f, -1.0f }, { 1.0f, -1.0f, -1.0f },
};

// The baked box hull is only exercised elsewhere through mass properties, which are analytic
// and never read boxPoints or boxPlanes. Pin the geometry itself against the transform so an
// axis swap in the point or plane bake cannot pass silently.
static int CheckTransformedBox( b3Vec3 h, b3Transform xf )
{
	b3BoxHull box = b3MakeTransformedBoxHull( h.x, h.y, h.z, xf );

	const float tol = 1e-5f;

	// Each corner is the transform of the signed half extent.
	for ( int i = 0; i < 8; ++i )
	{
		b3Vec3 local = { s_boxCornerSigns[i].x * h.x, s_boxCornerSigns[i].y * h.y, s_boxCornerSigns[i].z * h.z };
		b3Vec3 expected = b3TransformPoint( xf, local );
		ENSURE_SMALL( box.boxPoints[i].x - expected.x, tol );
		ENSURE_SMALL( box.boxPoints[i].y - expected.y, tol );
		ENSURE_SMALL( box.boxPoints[i].z - expected.z, tol );
	}

	// Face normals rotate with the box, offsets carry the rotated normal through the translation.
	b3Vec3 localNormals[6] = {
		b3Neg( b3Vec3_axisX ), b3Vec3_axisX, b3Neg( b3Vec3_axisY ), b3Vec3_axisY, b3Neg( b3Vec3_axisZ ), b3Vec3_axisZ,
	};
	float localOffsets[6] = { h.x, h.x, h.y, h.y, h.z, h.z };
	for ( int i = 0; i < 6; ++i )
	{
		b3Vec3 n = b3RotateVector( xf.q, localNormals[i] );
		float offset = localOffsets[i] + b3Dot( n, xf.p );
		ENSURE_SMALL( box.boxPlanes[i].normal.x - n.x, tol );
		ENSURE_SMALL( box.boxPlanes[i].normal.y - n.y, tol );
		ENSURE_SMALL( box.boxPlanes[i].normal.z - n.z, tol );
		ENSURE_SMALL( box.boxPlanes[i].offset - offset, tol );
	}

	// SoA vertex mirrors match the AoS points on all eight lanes.
	for ( int i = 0; i < 8; ++i )
	{
		ENSURE_SMALL( box.vx[i] - box.boxPoints[i].x, tol );
		ENSURE_SMALL( box.vy[i] - box.boxPoints[i].y, tol );
		ENSURE_SMALL( box.vz[i] - box.boxPoints[i].z, tol );
	}

	// SoA normal mirrors carry the six real faces and zero the two pad lanes.
	for ( int i = 0; i < 6; ++i )
	{
		ENSURE_SMALL( box.nx[i] - box.boxPlanes[i].normal.x, tol );
		ENSURE_SMALL( box.ny[i] - box.boxPlanes[i].normal.y, tol );
		ENSURE_SMALL( box.nz[i] - box.boxPlanes[i].normal.z, tol );
	}
	ENSURE( box.nx[6] == 0.0f && box.nx[7] == 0.0f );
	ENSURE( box.ny[6] == 0.0f && box.ny[7] == 0.0f );
	ENSURE( box.nz[6] == 0.0f && box.nz[7] == 0.0f );

	// The stored AABB bounds every baked corner.
	b3AABB pointBox = b3MakeAABB( box.boxPoints, 8, 0.0f );
	ENSURE( b3AABB_Contains( box.base.aabb, pointBox ) );

	return 0;
}

static int TransformedBoxHullTest( void )
{
	b3Vec3 h = { 0.25f, 0.5f, 0.3f };

	// Identity.
	if ( CheckTransformedBox( h, b3Transform_identity ) )
		return 1;

	// Translation only.
	b3Transform translated = { { 0.4f, -0.7f, 0.1f }, b3Quat_identity };
	if ( CheckTransformedBox( h, translated ) )
		return 1;

	// Rotation only. A quarter turn about Y swaps the world X and Z extents.
	b3Transform rotated = { b3Vec3_zero, ExactQuat( b3Vec3_axisY, 0.25f * B3_PI ) };
	if ( CheckTransformedBox( h, rotated ) )
		return 1;

	// Translation and rotation together.
	b3Transform transformed = { { 3.0f, -2.0f, 1.5f }, ExactQuat( b3Vec3_axisZ, 0.25f * B3_PI ) };
	if ( CheckTransformedBox( h, transformed ) )
		return 1;

	return 0;
}

static int CreateHullDegenerateTest( void )
{
	// Real (non-null) buffer; pointCount < 4 cases are guarded inside Construct().
	b3Vec3 collinear[8] = {
		{ 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, { 2.0f, 0.0f, 0.0f }, { 3.0f, 0.0f, 0.0f },
		{ 4.0f, 0.0f, 0.0f }, { 5.0f, 0.0f, 0.0f }, { 6.0f, 0.0f, 0.0f }, { 7.0f, 0.0f, 0.0f },
	};

	// Empty input.
	ENSURE( b3CreateHull( collinear, 0, 8 ) == NULL );

	// Fewer than 4 points.
	ENSURE( b3CreateHull( collinear, 3, 8 ) == NULL );

	// 8 coincident points.
	b3Vec3 coincident[8] = {
		{ 1.0f, 2.0f, 3.0f }, { 1.0f, 2.0f, 3.0f }, { 1.0f, 2.0f, 3.0f }, { 1.0f, 2.0f, 3.0f },
		{ 1.0f, 2.0f, 3.0f }, { 1.0f, 2.0f, 3.0f }, { 1.0f, 2.0f, 3.0f }, { 1.0f, 2.0f, 3.0f },
	};
	ENSURE( b3CreateHull( coincident, 8, 8 ) == NULL );

	// Collinear (along x-axis).
	ENSURE( b3CreateHull( collinear, 8, 8 ) == NULL );

	// Coplanar (in the xy-plane).
	b3Vec3 coplanar[6] = {
		{ 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f },
		{ 1.0f, 1.0f, 0.0f }, { 2.0f, 0.5f, 0.0f }, { 0.5f, 2.0f, 0.0f },
	};
	ENSURE( b3CreateHull( coplanar, 6, 8 ) == NULL );

	return 0;
}

int HullTest( void )
{
	RUN_SUBTEST( CreateHullCubeTest );
	RUN_SUBTEST( CreateHullTetrahedronTest );
	RUN_SUBTEST( CreateHullDeterminismTest );
	RUN_SUBTEST( CreateHullMaxVertexTest );
	RUN_SUBTEST( CreateHullRedundantInputTest );
	RUN_SUBTEST( CreateHullCloneTest );
	RUN_SUBTEST( CreateHullCylinderTest );
	RUN_SUBTEST( CreateHullSphereReductionTest );
	RUN_SUBTEST( CreateHullSphereStressTest );
	RUN_SUBTEST( CreateHullMergeChurnStressTest );
	RUN_SUBTEST( CreateHullDegenerateTest );
	RUN_SUBTEST( TransformedBoxHullTest );

	return 0;
}
