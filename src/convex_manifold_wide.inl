// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "algorithm.h"
#include "hull.h"
#include "manifold.h"
#include "shape.h"
#include "simd.h"
#include "simd_wide.h"

_Static_assert( B3_SIMD_WIDTH <= B3_HULL_SOA_PADDING, "hull SoA padding must cover the SIMD width" );

static inline int b3RoundUpToSIMDWidth( int count )
{
	return ( count + B3_SIMD_WIDTH - 1 ) & ~( B3_SIMD_WIDTH - 1 );
}

// Transform a SoA point/normal stream (already split into X/Y/Z) by out = -(R*v (+t)).
// The inputs come straight from the hull's stored SoA arrays, so there's no transpose here.
static inline void b3NegativeTransformFromSoA( b3Matrix3 R, b3Vec3 p, const float* inX, const float* inY, const float* inZ, int n,
											   float* outX, float* outY, float* outZ, bool isPoint )
{
	// row-column
	b3Vec3W r0 = { b3SplatW( R.cx.x ), b3SplatW( R.cy.x ), b3SplatW( R.cz.x ) };
	b3Vec3W r1 = { b3SplatW( R.cx.y ), b3SplatW( R.cy.y ), b3SplatW( R.cz.y ) };
	b3Vec3W r2 = { b3SplatW( R.cx.z ), b3SplatW( R.cy.z ), b3SplatW( R.cz.z ) };

	b3Vec3W t = { b3ZeroW(), b3ZeroW(), b3ZeroW() };
	if ( isPoint )
	{
		t = b3SplatVW( p );
	}

	for ( int i = 0; i < n; i += B3_SIMD_WIDTH )
	{
		b3Vec3W v = b3LoadVW( inX + i, inY + i, inZ + i );

		// Rotate one vector per lane
		b3Vec3W out = { b3DotW( r0, v ), b3DotW( r1, v ), b3DotW( r2, v ) };

		if ( isPoint )
		{
			out = b3AddVW( out, t );
		}

		b3StoreVW( outX + i, outY + i, outZ + i, b3NegVW( out ) );
	}
}

_Static_assert( B3_MAX_HULL_VERTICES == 128, "must be 128" );

#define B3_HULL_BIT_COUNT 7

// SIMD support point calculation using a SoA vertex array padded with repeats of the first vertex
// to a multiple of 8.
//
// This minimizes (bias - dot), where the caller is expected to provide a bias that makes this always positive.
// It can be direction dependent. The bias should be just big enough to ensure the value is positive because
// an excessive bias causes a precision loss in the support calculation.
//
// The vertex index is embedded in the low B3_HULL_BIT_COUNT mantissa bits of the value. By minimizing a value that
// is always positive, the minimum carries the smallest index so that padded SoA values will never win. This is the
// purpose of using the bias instead of maximizing the dot directly.
//
// The support is then recomputed exactly as dot(normal, vertex), without the embedded index.
// todo consider using this for GJK
static inline void b3GetSupportWide( b3Vec3 normal, const float* vx, const float* vy, const float* vz, int n, float bias,
									 float* support, int* vertexIndex )
{
	const b3Vec3W normalW = b3SplatVW( normal );
	const b3FloatW biasV = b3SplatW( bias );

	// Start the minimum at a large value.
	b3FloatW minValue = b3SplatW( INFINITY );

	// Tail lanes hold vertex 0 with index bits >= vertexCount, so they never become the min value.
	for ( int i = 0; i < n; i += B3_SIMD_WIDTH )
	{
		b3Vec3W v = b3LoadVW( vx + i, vy + i, vz + i );
		b3FloatW d = b3DotW( normalW, v );

		// This is always positive.
		b3FloatW value = b3SubW( biasV, d );
		b3FloatW augmentedValue = b3EmbedIndexW( value, i, B3_HULL_BIT_COUNT );
		minValue = b3MinW( minValue, augmentedValue );
	}

	// One horizontal min, the winning lane's value and index bits ride through.
	int vi = b3MinIndexW( minValue, B3_HULL_BIT_COUNT );

	// Exact support for the chosen vertex.
	*vertexIndex = vi;

	// Dot product
	*support = normal.x * vx[vi] + normal.y * vy[vi] + normal.z * vz[vi];
}

static inline float b3GetFaceSeparation( b3Vec3 direction, float planeSeparation, const float* vx, const float* vy,
										 const float* vz, int n, b3Vec3 center, b3Vec3 extents, int* vertexIndex )
{
	float bias = b3Dot( direction, center ) + 1.0625f * b3Dot( b3Abs( direction ), extents );
	float support;
	b3GetSupportWide( direction, vx, vy, vz, n, bias, &support, vertexIndex );
	return planeSeparation - support;
}

// Wide dot(n, d) for all face normals n of the hull, padded to the SIMD width.
static inline void b3GetFaceDots( const b3HullData* hull, b3Vec3 d, float* dots )
{
	int faceStride = b3GetHullSoaStride( hull->faceCount );
	const float* nx = b3GetHullSoaNormals( hull );
	const float* ny = nx + faceStride;
	const float* nz = ny + faceStride;

	b3Vec3W dW = b3SplatVW( d );

	int wideFaceCount = b3RoundUpToSIMDWidth( hull->faceCount );
	for ( int i = 0; i < wideFaceCount; i += B3_SIMD_WIDTH )
	{
		// dot product per lane
		b3FloatW m = b3DotW( b3LoadVW( nx + i, ny + i, nz + i ), dW );
		b3StoreW( dots + i, m );
	}
}

#define B3_PARALLEL_TOL 1e-4f

// Inscribed sphere edge test. https://box2d.org/posts/2026/09/inscribed-spheres/
// The implementation here is a mix of the versions from Cairn Overturf and Dirk Gregorius:
// https://gist.github.com/cairnc/dee7a2866da0709f2d9a77b6493b57b5
// https://gist.github.com/dgregorius/e6751b5c00937cd21af63cba3c53c861
// This benchmarks faster than the version from the blog post.
static inline int b3TestEdgeCandidateSorted( float a1, float a2, float c, float bound )
{
	// We want to maximize
	//
	//   f(t) = dot(d, nlerp(n1, n2, t))
	//
	// over t in [0, 1], where
	//
	//   a1 = dot(n1, d)
	//   a2 = dot(n2, d)
	//   c  = dot(n1, n2).
	//
	// The maximum can occur either at an endpoint or at an interior
	// critical point where f'(t) = 0.

	// Since the endpoint values are a1 and a2, only the larger endpoint
	// needs to be tested against the bound.
	float hi = b3MaxFloat( a1, a2 );
	float lo = b3MinFloat( a1, a2 );
	int exterior = hi >= bound;

	// Differentiating f(t) gives
	//
	//           a2 - c*a1 - (1 - c)*(a1 + a2)*t
	//   f'(t) = --------------------------------------
	//           ((1 - t)^2 + t^2 + 2*c*t*(1 - t))^(3/2)
	//
	// The denominator is positive and the numerator is linear in t.
	// An interior maximum therefore requires
	//
	//   f'(0) >= 0  ->  a2 >= c*a1
	//   f'(1) <= 0  ->  a1 >= c*a2.
	//
	// After sorting these become
	//
	//   hi >= c*lo
	//   lo >= c*hi.
	//
	// The second is always the stricter condition since
	//
	//   (hi - c*lo) - (lo - c*hi)
	//       = (1 + c)*(hi - lo) >= 0.
	//
	// Thus both conditions reduce to u >= 0.
	float u = lo - c * hi;
	int maxIsInterior = u >= 0.0f;

	// Solving f'(t) = 0 and evaluating f(t) at tmax gives
	//
	//   f(tmax)^2 =
	//       (a1^2 + a2^2 - 2*c*a1*a2) / (1 - c^2).
	//
	// After sorting,
	//
	//   hi^2 + lo^2 - 2*c*hi*lo
	//       = hi^2*(1 - c^2) + u^2.
	//
	// Let s = 1 - c^2 and rearrange f(tmax) >= bound:
	//
	//   u^2 >= (bound^2 - hi^2)*s.
	float s = 1.0f - c * c;
	float boundTerm = ( bound - hi ) * ( bound + hi );
	float lhs = u * u;
	float rhs = boundTerm * s;
	int maxBeatsBound = lhs >= rhs;

	// The interior expression contains 1 - c^2 in its denominator.
	// When this approaches zero the arc is degenerate or ill-conditioned,
	// so conservatively keep the edge as a candidate.
	int nearlyParallel = s < B3_PARALLEL_TOL;

	int interior = maxIsInterior & ( maxBeatsBound | nearlyParallel );

	return exterior | interior;
}

// dot(n, otherCenter) - offset for all faces of the hull, padded to the SIMD width.
static inline void b3GetFacePlaneSeparations( const b3HullData* hull, b3Vec3 otherCenter, float* separations )
{
	b3GetFaceDots( hull, otherCenter, separations );

	const b3Plane* planes = b3GetHullPlanes( hull );
	int faceCount = hull->faceCount;
	for ( int i = 0; i < faceCount; ++i )
	{
		separations[i] -= planes[i].offset;
	}
}

// The number of edge pair tests needed to make the extra culling pass worthwhile.
#define B3_EDGE_PROBE_MIN_TESTS 10

// Re-test edge candidates against the plane separations of a probe point on the other hull
static inline int b3FilterEdgeCandidates( const b3HullData* hull, const float* planeSeparations, float bound, int* edgeIndices,
										  int count )
{
	const b3HullHalfEdge* halfEdges = b3GetHullEdges( hull );
	const float* cosines = b3GetHullEdgeCosines( hull );
	int keptCount = 0;

	for ( int k = 0; k < count; ++k )
	{
		int i = edgeIndices[k];
		int i1 = halfEdges[i].face;
		int i2 = halfEdges[i + 1].face;
		float c = cosines[i >> 1];
		edgeIndices[keptCount] = i;
		keptCount += b3TestEdgeCandidateSorted( planeSeparations[i1], planeSeparations[i2], c, bound );
	}

	return keptCount;
}

// Temporary abbreviations for convenience.
#define NE ( B3_MAX_HULL_EDGES + B3_SIMD_WIDTH )
#define NF ( B3_MAX_HULL_FACES + B3_SIMD_WIDTH )
#define NV ( B3_MAX_HULL_VERTICES + B3_SIMD_WIDTH )

// SIMD separating axis test based on an implementation developed by Cairn Overturf.
// See his article: https://cairnc.github.io/posts/improvements-to-the-separating-axis/
b3AxisQuery B3_WIDE( b3ComputeSeparatingAxis )( const b3HullData* hullA, const b3HullData* hullB, b3Transform xfB,
												bool earlyReturn )
{
	b3Matrix3 R = b3MakeMatrixFromQuat( xfB.q );
	b3Matrix3 invR = b3Transpose( R );

	float speculativeDistance = B3_SPECULATIVE_DISTANCE;

	b3AxisQuery res = {
		.faceA =
			{
				.normal = b3Vec3_zero,
				.separation = -INFINITY,
				.indexA = B3_NULL_INDEX,
				.indexB = B3_NULL_INDEX,
				.type = b3_faceAxisA,
			},
		.faceB =
			{
				.normal = b3Vec3_zero,
				.separation = -INFINITY,
				.indexA = B3_NULL_INDEX,
				.indexB = B3_NULL_INDEX,
				.type = b3_faceAxisB,
			},
		.edge =
			{
				.normal = b3Vec3_zero,
				.separation = -INFINITY,
				.indexA = B3_NULL_INDEX,
				.indexB = B3_NULL_INDEX,
				.type = b3_edgePairAxis,
			},
		.separatedFeature = b3_invalidAxis,
	};

	int faceCountA = hullA->faceCount;
	const b3Plane* planesA = b3GetHullPlanes( hullA );

	int vertexStrideB = b3GetHullSoaStride( hullB->vertexCount );
	const float* vxB = b3GetHullSoaVertices( hullB );
	const float* vyB = vxB + vertexStrideB;
	const float* vzB = vyB + vertexStrideB;
	int wideVertexCountB = b3RoundUpToSIMDWidth( hullB->vertexCount );

	b3Vec3 cB = b3AABB_Center( hullB->aabb );
	b3Vec3 hB = b3AABB_Extents( hullB->aabb );

	// The hulls have a precomputed inner radius and centroid.
	// A given axis cannot achieve a separation larger than:
	// dot(axis, centerB - centerA) - innerRadiusA - innerRadiusB
	// So this is the upper bound for the separation of a candidate axis.
	// An axis can be skipped if it has an upper bound that is less than the current
	// best separation. This lets me skip many of the candidates without computing
	// support points.

	b3Vec3 deltaCenter = b3Sub( b3Add( b3MulMV( R, hullB->center ), xfB.p ), hullA->center );
	float centerDistance = b3Length( deltaCenter );
	float radius = hullA->innerRadius + hullB->innerRadius;

	// Adjust the radius to ensure the best axis isn't skipped.
	float radiusBound = radius - ( B3_LINEAR_SLOP + 0.001f * ( centerDistance + radius ) );

	// Compute dot(normalA, centerDelta) for all face normals of hullA.
	_Alignas( B3_WIDE_ALIGNMENT ) float dotA[NF];
	b3GetFaceDots( hullA, deltaCenter, dotA );

	// Find the face of hullA that most aligns with centerDelta.
	int seedIndexA = 0;
	float maxDotA = dotA[0];
	for ( int i = 1; i < faceCountA; ++i )
	{
		if ( dotA[i] > maxDotA )
		{
			maxDotA = dotA[i];
			seedIndexA = i;
		}
	}

	// Use the seed to get a lower bound on the separation for the faces of hullA.
	float floorA = -INFINITY;
	float seedSeparationA = -INFINITY;
	int seedVertexB = 0;
	if ( earlyReturn )
	{
		b3Plane plane = planesA[seedIndexA];
		b3Vec3 direction = b3Neg( b3MulMV( invR, plane.normal ) );
		float planeSeparation = b3Dot( plane.normal, xfB.p ) - plane.offset;
		seedSeparationA = b3GetFaceSeparation( direction, planeSeparation, vxB, vyB, vzB, wideVertexCountB, cB, hB, &seedVertexB );
		floorA = b3MinFloat( seedSeparationA, speculativeDistance );
	}

	// Test A's face planes against B's vertices.
	for ( int i = 0; i < faceCountA; ++i )
	{
		// The bound offset ensures the seed will be evaluated.
		if ( dotA[i] - radiusBound < b3MaxFloat( floorA, res.faceA.separation ) )
		{
			continue;
		}

		b3Plane plane = planesA[i];
		int vertexIndex = seedVertexB;
		float separation = seedSeparationA;

		// Avoid recomputing the seed face separation.
		if ( earlyReturn == false || i != seedIndexA )
		{
			b3Vec3 direction = b3Neg( b3MulMV( invR, plane.normal ) );
			float planeSeparation = b3Dot( plane.normal, xfB.p ) - plane.offset;
			separation = b3GetFaceSeparation( direction, planeSeparation, vxB, vyB, vzB, wideVertexCountB, cB, hB, &vertexIndex );
		}

		if ( separation > res.faceA.separation )
		{
			res.faceA.normal = plane.normal;
			res.faceA.separation = separation;
			res.faceA.indexA = i;
			res.faceA.indexB = vertexIndex;
			if ( separation > speculativeDistance && earlyReturn )
			{
				res.separatedFeature = b3_faceAxisA;
				return res;
			}
		}
	}

	B3_VALIDATE( res.faceA.indexA != B3_NULL_INDEX );

	int faceCountB = hullB->faceCount;
	const b3Plane* planesB = b3GetHullPlanes( hullB );

	int vertexStrideA = b3GetHullSoaStride( hullA->vertexCount );
	const float* vxA = b3GetHullSoaVertices( hullA );
	const float* vyA = vxA + vertexStrideA;
	const float* vzA = vyA + vertexStrideA;
	int wideVertexCountA = b3RoundUpToSIMDWidth( hullA->vertexCount );

	b3Vec3 cA = b3AABB_Center( hullA->aabb );
	b3Vec3 hA = b3AABB_Extents( hullA->aabb );

	// Similarly, find the face of hullB that most aligns with the vector pointing from centerB to centerA.
	_Alignas( B3_WIDE_ALIGNMENT ) float dotB[NF];
	b3GetFaceDots( hullB, b3Neg( b3MulMV( invR, deltaCenter ) ), dotB );

	int seedIndexB = 0;
	float maxDotB = dotB[0];
	for ( int i = 1; i < faceCountB; ++i )
	{
		if ( dotB[i] > maxDotB )
		{
			maxDotB = dotB[i];
			seedIndexB = i;
		}
	}

	// Get a lower bound on the separation for the faces of hullB.
	float floorB = -INFINITY;
	float seedSeparationB = -INFINITY;
	int seedVertexA = 0;
	if ( earlyReturn )
	{
		b3Plane plane = planesB[seedIndexB];
		b3Vec3 direction = b3Neg( b3MulMV( R, plane.normal ) );
		float planeSeparation = b3Dot( direction, xfB.p ) - plane.offset;
		seedSeparationB = b3GetFaceSeparation( direction, planeSeparation, vxA, vyA, vzA, wideVertexCountA, cA, hA, &seedVertexA );

		// Include the floor set by hull A faces.
		floorB = b3MaxFloat( seedSeparationB, res.faceA.separation );
		floorB = b3MinFloat( floorB, speculativeDistance );
	}

	// Test B's face planes against A's vertices.
	for ( int i = 0; i < faceCountB; ++i )
	{
		if ( dotB[i] - radiusBound < b3MaxFloat( floorB, res.faceB.separation ) )
		{
			continue;
		}

		b3Plane plane = planesB[i];
		b3Vec3 direction = b3Neg( b3MulMV( R, plane.normal ) );
		int vertexIndex = seedVertexA;
		float separation = seedSeparationB;

		// Avoid recomputing the seed face separation.
		if ( earlyReturn == false || i != seedIndexB )
		{
			float planeSeparation = b3Dot( direction, xfB.p ) - plane.offset;
			separation = b3GetFaceSeparation( direction, planeSeparation, vxA, vyA, vzA, wideVertexCountA, cA, hA, &vertexIndex );
		}

		if ( separation > res.faceB.separation )
		{
			res.faceB.normal = direction;
			res.faceB.separation = separation;
			res.faceB.indexA = vertexIndex;
			res.faceB.indexB = i;
			if ( separation > speculativeDistance && earlyReturn )
			{
				res.separatedFeature = b3_faceAxisB;
				return res;
			}
		}
	}

	// Transform B into A's space once, into SoA arrays. Extra space so tail can be set to zero.
	_Static_assert( ( B3_MAX_HULL_EDGES & ( B3_SIMD_WIDTH - 1 ) ) == 0, "must be multiple of SIMD width" );
	_Static_assert( ( B3_MAX_HULL_FACES & ( B3_HULL_SOA_PADDING - 1 ) ) == 0, "must be multiple of hull SoA padding" );
	_Static_assert( ( B3_MAX_HULL_VERTICES & ( B3_HULL_SOA_PADDING - 1 ) ) == 0, "must be multiple of hull SoA padding" );

	B3_VALIDATE( earlyReturn == false ||
				 centerDistance >= b3MaxFloat( res.faceA.separation, res.faceB.separation ) + radiusBound );

	// Gather edges of A that can feasibly create a separating axis that beats the maximum face separation.
	int halfEdgeCountA = hullA->edgeCount;
	const b3HullHalfEdge* halfEdgesA = b3GetHullEdges( hullA );
	int edgeIndicesA[NE];
	int na = 0;

	int halfEdgeCountB = hullB->edgeCount;
	const b3HullHalfEdge* halfEdgesB = b3GetHullEdges( hullB );
	int edgeIndicesB[B3_MAX_HULL_EDGES];
	int nb = 0;

	float maxFaceSeparation = b3MaxFloat( res.faceA.separation, res.faceB.separation );
	float boundSlack = radius - radiusBound;
	float thresholdA = earlyReturn ? maxFaceSeparation + hullB->innerRadius - boundSlack : -INFINITY;
	float thresholdB = earlyReturn ? maxFaceSeparation + hullA->innerRadius - boundSlack : -INFINITY;

	b3Vec3 centerBinA = b3Add( b3MulMV( R, hullB->center ), xfB.p );
	b3Vec3 centerAinB = b3MulMV( invR, b3Sub( hullA->center, xfB.p ) );

	_Alignas( B3_WIDE_ALIGNMENT ) float planeDotA[NF];
	_Alignas( B3_WIDE_ALIGNMENT ) float planeDotB[NF];
	b3GetFacePlaneSeparations( hullA, centerBinA, planeDotA );
	b3GetFacePlaneSeparations( hullB, centerAinB, planeDotB );

	const float* cosinesA = b3GetHullEdgeCosines( hullA );
	const float* cosinesB = b3GetHullEdgeCosines( hullB );

	for ( int i = 0; i < halfEdgeCountA; i += 2 )
	{
		int i1 = halfEdgesA[i].face;
		int i2 = halfEdgesA[i + 1].face;
		float c = cosinesA[i >> 1];
		edgeIndicesA[na] = i;
		na += b3TestEdgeCandidateSorted( planeDotA[i1], planeDotA[i2], c, thresholdA );
	}

	for ( int i = 0; i < halfEdgeCountB; i += 2 )
	{
		int i1 = halfEdgesB[i].face;
		int i2 = halfEdgesB[i + 1].face;
		float c = cosinesB[i >> 1];
		edgeIndicesB[nb] = i;
		nb += b3TestEdgeCandidateSorted( planeDotB[i1], planeDotB[i2], c, thresholdB );
	}

	// Apply additional culling use the support points for the best face axes.
	// The support vertices of the best faces are points on the other hull, so an edge pair axis cannot
	// have a larger separation than the plane separation of these points over the arc of the edge.
	// This can slow down boxes so skip this if there are not enough edge candidates.
	if ( earlyReturn && nb * ( ( na + 3 ) >> 2 ) >= B3_EDGE_PROBE_MIN_TESTS )
	{
		float probeBound = maxFaceSeparation - ( 0.1f * B3_LINEAR_SLOP + 0.001f * b3AbsFloat( centerDistance + radius ) );

		int vertexB = res.faceA.indexB != B3_NULL_INDEX ? res.faceA.indexB : seedVertexB;
		b3Vec3 probeB = { vxB[vertexB], vyB[vertexB], vzB[vertexB] };
		b3GetFacePlaneSeparations( hullA, b3Add( b3MulMV( R, probeB ), xfB.p ), planeDotA );
		na = b3FilterEdgeCandidates( hullA, planeDotA, probeBound, edgeIndicesA, na );

		int vertexA = res.faceB.indexA != B3_NULL_INDEX ? res.faceB.indexA : seedVertexA;
		b3Vec3 probeA = { vxA[vertexA], vyA[vertexA], vzA[vertexA] };
		b3GetFacePlaneSeparations( hullB, b3MulMV( invR, b3Sub( probeA, xfB.p ) ), planeDotB );
		nb = b3FilterEdgeCandidates( hullB, planeDotB, probeBound, edgeIndicesB, nb );
	}

	if ( na == 0 || nb == 0 )
	{
		// No edge candidates found.
		return res;
	}

	// The alignments below are not necessary, but they don't hurt.

	// B face normals in A space, negated.
	_Alignas( B3_WIDE_ALIGNMENT ) float bFNx[NF];
	_Alignas( B3_WIDE_ALIGNMENT ) float bFNy[NF];
	_Alignas( B3_WIDE_ALIGNMENT ) float bFNz[NF];

	// B vertices in A space, negated.
	_Alignas( B3_WIDE_ALIGNMENT ) float bWx[NV];
	_Alignas( B3_WIDE_ALIGNMENT ) float bWy[NV];
	_Alignas( B3_WIDE_ALIGNMENT ) float bWz[NV];

	int faceStrideB = b3GetHullSoaStride( faceCountB );
	const float* nxB = b3GetHullSoaNormals( hullB );
	const float* nyB = nxB + faceStrideB;
	const float* nzB = nyB + faceStrideB;

	int wideFaceCountB = b3RoundUpToSIMDWidth( faceCountB );
	b3NegativeTransformFromSoA( R, xfB.p, nxB, nyB, nzB, wideFaceCountB, bFNx, bFNy, bFNz, false );
	b3NegativeTransformFromSoA( R, xfB.p, vxB, vyB, vzB, wideVertexCountB, bWx, bWy, bWz, true );

	// Per A edge data, already in A's space so just gathered. n0 and n1 are the two face
	// normals, d the edge vector av1-av0, v0 the first vertex. Tol is the
	// parallel edge tolerance, scaled by the edge length.
	_Alignas( B3_WIDE_ALIGNMENT ) float aN0x[NE];
	_Alignas( B3_WIDE_ALIGNMENT ) float aN0y[NE];
	_Alignas( B3_WIDE_ALIGNMENT ) float aN0z[NE];
	_Alignas( B3_WIDE_ALIGNMENT ) float aN1x[NE];
	_Alignas( B3_WIDE_ALIGNMENT ) float aN1y[NE];
	_Alignas( B3_WIDE_ALIGNMENT ) float aN1z[NE];
	// dir = av1 - av0
	_Alignas( B3_WIDE_ALIGNMENT ) float aDx[NE];
	_Alignas( B3_WIDE_ALIGNMENT ) float aDy[NE];
	_Alignas( B3_WIDE_ALIGNMENT ) float aDz[NE];
	_Alignas( B3_WIDE_ALIGNMENT ) float aV0x[NE];
	_Alignas( B3_WIDE_ALIGNMENT ) float aV0y[NE];
	_Alignas( B3_WIDE_ALIGNMENT ) float aV0z[NE];
	_Alignas( B3_WIDE_ALIGNMENT ) float aTol[NE];

	float squaredTol = B3_PARALLEL_EDGE_TOL * B3_PARALLEL_EDGE_TOL;
	for ( int k = 0; k < na; ++k )
	{
		const b3HullHalfEdge* edge = halfEdgesA + edgeIndicesA[k];
		const b3HullHalfEdge* twin = edge + 1;

		b3Vec3 A = planesA[edge->face].normal;
		b3Vec3 B = planesA[twin->face].normal;
		aN0x[k] = A.x;
		aN0y[k] = A.y;
		aN0z[k] = A.z;
		aN1x[k] = B.x;
		aN1y[k] = B.y;
		aN1z[k] = B.z;

		int v0 = edge->origin;
		int v1 = twin->origin;

		aDx[k] = vxA[v1] - vxA[v0];
		aDy[k] = vyA[v1] - vyA[v0];
		aDz[k] = vzA[v1] - vzA[v0];
		aV0x[k] = vxA[v0];
		aV0y[k] = vyA[v0];
		aV0z[k] = vzA[v0];

		aTol[k] = squaredTol * ( aDx[k] * aDx[k] + aDy[k] * aDy[k] + aDz[k] * aDz[k] );
	}

	// Zero the tail lanes.
	b3FloatW zero = b3ZeroW();
	b3Vec3W zeroV = { zero, zero, zero };
	b3StoreVW( aN0x + na, aN0y + na, aN0z + na, zeroV );
	b3StoreVW( aN1x + na, aN1y + na, aN1z + na, zeroV );
	b3StoreVW( aDx + na, aDy + na, aDz + na, zeroV );
	b3StoreVW( aV0x + na, aV0y + na, aV0z + na, zeroV );

	b3StoreW( aTol + na, zero );

	float linearSlop = B3_LINEAR_SLOP;

#if defined( B3_SIMD_NONE )

	// The SIMD emulated version of this code is very slow. This is a purely scalar version
	// for platforms that don't have SIMD capability. It is much faster than SIMD emulation.
	// WARNING: this math needs to match the SIMD version for cross platform determinism.

	const float EPS = -linearSlop * linearSlop;

	for ( int j = 0; j < nb; ++j )
	{
		const b3HullHalfEdge* edge = halfEdgesB + edgeIndicesB[j];
		const b3HullHalfEdge* twin = edge + 1;
		int f0 = edge->face;
		int f1 = twin->face;
		int v0 = edge->origin;
		int v1 = twin->origin;

		b3Vec3 C = { bFNx[f0], bFNy[f0], bFNz[f0] };
		b3Vec3 D = { bFNx[f1], bFNy[f1], bFNz[f1] };
		b3Vec3 bv0 = { bWx[v0], bWy[v0], bWz[v0] };
		b3Vec3 bv1 = { bWx[v1], bWy[v1], bWz[v1] };
		b3Vec3 DC = b3Sub( bv1, bv0 );

		for ( int i = 0; i < na; ++i )
		{
			b3Vec3 d = { aDx[i], aDy[i], aDz[i] };

			// CBA = C.dir, DBA = D.dir, where dir = B_x_A
			float CBA = b3Dot( C, d );
			float DBA = b3Dot( D, d );
			if ( CBA * DBA >= EPS )
			{
				continue;
			}

			b3Vec3 n0 = { aN0x[i], aN0y[i], aN0z[i] };
			b3Vec3 n1 = { aN1x[i], aN1y[i], aN1z[i] };

			// ADC = n0.DC, BDC = n1.DC, where DC = D_x_C
			float ADC = b3Dot( n0, DC );
			float BDC = b3Dot( n1, DC );
			if ( ADC * BDC >= EPS || CBA * BDC >= EPS )
			{
				continue;
			}

			// Reject near parallel edges
			float maxCD = b3MaxFloat( CBA * CBA, DBA * DBA );
			if ( maxCD <= aTol[i] )
			{
				continue;
			}

			// t = -CBA / (DBA - CBA)
			float t = -CBA / ( DBA - CBA );

			// normal = lerp(t, C, D) = C + (D-C)*t
			b3Vec3 n = b3MulAdd( C, t, b3Sub( D, C ) );
			float len2 = b3Dot( n, n );

			float inv = 1.0f / sqrtf( len2 );
			n = b3MulSV( inv, n );

			// separation = -dot(normal, av0 + bv0)
			b3Vec3 av0 = { aV0x[i], aV0y[i], aV0z[i] };
			float separation = -b3Dot( b3Add( av0, bv0 ), n );

			if ( separation > res.edge.separation )
			{
				res.edge.normal = n;
				res.edge.separation = separation;

				// Half edge index
				res.edge.indexA = edgeIndicesA[i];
				res.edge.indexB = edgeIndicesB[j];

				if ( separation > speculativeDistance && earlyReturn )
				{
					res.separatedFeature = b3_edgePairAxis;
					return res;
				}
			}
		}
	}

#else

	// This tolerance can skip edges shorter than 1cm.
	const b3FloatW EPS = b3SplatW( -linearSlop * linearSlop );
	const b3FloatW INF = b3SplatW( INFINITY );

	for ( int j = 0; j < nb; ++j )
	{
		const b3HullHalfEdge* edge = halfEdgesB + edgeIndicesB[j];
		const b3HullHalfEdge* twin = edge + 1;
		int f0 = edge->face;
		int f1 = twin->face;
		int v0 = edge->origin;
		int v1 = twin->origin;

		b3Vec3 nC = { bFNx[f0], bFNy[f0], bFNz[f0] };
		b3Vec3 nD = { bFNx[f1], bFNy[f1], bFNz[f1] };
		b3Vec3 pB = { bWx[v0], bWy[v0], bWz[v0] };
		b3Vec3 qB = { bWx[v1], bWy[v1], bWz[v1] };

		const b3Vec3W C = b3SplatVW( nC );
		const b3Vec3W D = b3SplatVW( nD );
		const b3Vec3W DC = b3SplatVW( b3Sub( qB, pB ) );
		const b3Vec3W bv0 = b3SplatVW( pB );

		for ( int i = 0; i < na; i += B3_SIMD_WIDTH )
		{
			b3Vec3W n0 = b3LoadVW( aN0x + i, aN0y + i, aN0z + i );
			b3Vec3W n1 = b3LoadVW( aN1x + i, aN1y + i, aN1z + i );
			b3Vec3W d = b3LoadVW( aDx + i, aDy + i, aDz + i );
			b3Vec3W av0 = b3LoadVW( aV0x + i, aV0y + i, aV0z + i );

			b3FloatW tol = b3LoadW( aTol + i );

			// CBA = C.dir, DBA = D.dir, where dir = B_x_A
			b3FloatW CBA = b3DotW( C, d );
			b3FloatW DBA = b3DotW( D, d );
			// ADC = n0.DC, BDC = n1.DC, where DC = D_x_C
			b3FloatW ADC = b3DotW( n0, DC );
			b3FloatW BDC = b3DotW( n1, DC );

			// Gauss map arc crossing test, CBA*DBA<eps and ADC*BDC<eps and CBA*BDC<eps
			b3FloatW m1 = b3LessThanW( b3MulW( CBA, DBA ), EPS );
			b3FloatW m2 = b3LessThanW( b3MulW( ADC, BDC ), EPS );
			b3FloatW m3 = b3LessThanW( b3MulW( CBA, BDC ), EPS );

			// Reject near parallel edges. The arc lerp is ill conditioned when both of B's normals are nearly
			// perpendicular to edge A, a scale invariant sine threshold relative to the edge length.
			b3FloatW maxCD = b3MaxW( b3MulW( CBA, CBA ), b3MulW( DBA, DBA ) );
			b3FloatW notParallel = b3GreaterThanW( maxCD, tol );
			b3FloatW mask = b3AndW( b3AndW( m1, m2 ), b3AndW( m3, notParallel ) );

			// Most A-edges fail the Gauss test, so skip the divide, sqrt and support work when no
			// lane passed.
			if ( b3AnyTrueW( mask ) == false )
			{
				continue;
			}

			// t = -CBA / (DBA - CBA)
			b3FloatW t = b3DivW( b3NegW( CBA ), b3SubW( DBA, CBA ) );

			// normal = lerp(t, C, D) = C + (D-C)*t
			b3Vec3W n = b3MulAddSVW( C, t, b3SubVW( D, C ) );

			// normalize
			b3FloatW len2 = b3DotW( n, n );
			b3FloatW inv = b3DivW( b3SplatW( 1.0f ), b3SqrtW( len2 ) );
			n = b3MulSVW( inv, n );

			// support = dot(normal, av0 + bv0)
			b3FloatW support = b3DotW( b3AddVW( av0, bv0 ), n );

			// Lanes that fail the Gauss test can never win.
			support = b3BlendW( INF, support, mask );
			b3FloatW separation = b3NegW( support );

			// Test all supports against the running best at once. If none beats it, skip the
			// store and scalar reduction.
			b3FloatW improves = b3GreaterThanW( separation, b3SplatW( res.edge.separation ) );
			if ( b3AnyTrueW( improves ) == false )
			{
				continue;
			}

			_Alignas( B3_WIDE_ALIGNMENT ) float sA[B3_SIMD_WIDTH];
			_Alignas( B3_WIDE_ALIGNMENT ) float nxA[B3_SIMD_WIDTH];
			_Alignas( B3_WIDE_ALIGNMENT ) float nyA[B3_SIMD_WIDTH];
			_Alignas( B3_WIDE_ALIGNMENT ) float nzA[B3_SIMD_WIDTH];
			b3StoreW( sA, separation );
			b3StoreVW( nxA, nyA, nzA, n );

			// Reduce in lane order so ties keep the first edge and the early out takes the first
			// improving support below zero. Padded tail lanes carry +INF support, so they never
			// update or index edges out of range.
			for ( int lane = 0; lane < B3_SIMD_WIDTH; lane++ )
			{
				int ei = i + lane;
				float s = sA[lane];
				if ( s > res.edge.separation )
				{
					res.edge.normal = (b3Vec3){ nxA[lane], nyA[lane], nzA[lane] };
					res.edge.separation = s;

					// Half edge index
					res.edge.indexA = edgeIndicesA[ei];
					res.edge.indexB = edgeIndicesB[j];

					if ( s > speculativeDistance && earlyReturn )
					{
						res.separatedFeature = b3_edgePairAxis;
						return res;
					}
				}
			}
		}
	}
#endif

	return res;
}

#undef NE
#undef NF
#undef NV
