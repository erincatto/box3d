// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

// b3GetHeightFieldTriangle and b3IntersectRayTriangle are internal
#include "shape.h"
#include "simd.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <stdio.h>

static int HeightFieldCreate( void )
{
	b3Vec3 scale = { 1.0f, 1.0f, 1.0f };
	b3HeightFieldData* hf = b3CreateGrid( 4, 4, scale, false );

	ENSURE( hf->rowCount == 4 );
	ENSURE( hf->columnCount == 4 );
	ENSURE( hf->clockwise == false );
	ENSURE( hf->version == B3_HEIGHT_FIELD_VERSION );

	ENSURE_SMALL( hf->aabb.lowerBound.x, FLT_EPSILON );
	ENSURE_SMALL( hf->aabb.lowerBound.y, FLT_EPSILON );
	ENSURE_SMALL( hf->aabb.lowerBound.z, FLT_EPSILON );

	ENSURE_SMALL( hf->aabb.upperBound.x - 3.0f, FLT_EPSILON );
	ENSURE_SMALL( hf->aabb.upperBound.y, FLT_EPSILON );
	ENSURE_SMALL( hf->aabb.upperBound.z - 3.0f, FLT_EPSILON );

	b3DestroyHeightField( hf );
	return 0;
}

static int HeightFieldTriangleIndex( void )
{
	// Asymmetric grid (rows != cols) catches off-by-one errors between
	// vertex stride (columnCount) and cell stride (columnCount - 1).
	int rowCount = 4;
	int columnCount = 5;
	b3Vec3 scale = { 1.0f, 1.0f, 1.0f };
	b3HeightFieldData* hf = b3CreateGrid( rowCount, columnCount, scale, false );

	int triangleCount = 2 * ( rowCount - 1 ) * ( columnCount - 1 );

	for ( int triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex )
	{
		int quadIndex = triangleIndex >> 1;
		int sub = triangleIndex & 1;
		int row = quadIndex / ( columnCount - 1 );
		int column = quadIndex - row * ( columnCount - 1 );

		int index11 = row * columnCount + column;
		int index12 = index11 + 1;
		int index21 = ( row + 1 ) * columnCount + column;
		int index22 = index21 + 1;

		b3Triangle t = b3GetHeightFieldTriangle( hf, triangleIndex );

		if ( sub == 0 )
		{
			// Triangle 0 (CCW): {11, 21, 12}
			ENSURE( t.i1 == index11 );
			ENSURE( t.i2 == index21 );
			ENSURE( t.i3 == index12 );
		}
		else
		{
			// Triangle 1 (CCW): {22, 12, 21}
			ENSURE( t.i1 == index22 );
			ENSURE( t.i2 == index12 );
			ENSURE( t.i3 == index21 );
		}
	}

	b3DestroyHeightField( hf );
	return 0;
}

static int HeightFieldWinding( void )
{
	// Build the same flat 3x3 field with CCW and CW winding. The cross-product
	// normal of triangle 0 must flip sign accordingly.
	float heights[9] = { 0 };
	uint8_t materials[4] = { 0, 0, 0, 0 };

	b3HeightFieldDef def = { 0 };
	def.heights = heights;
	def.materialIndices = materials;
	def.scale = (b3Vec3){ 1.0f, 1.0f, 1.0f };
	def.countX = 3;
	def.countZ = 3;
	def.globalMinimumHeight = -1.0f;
	def.globalMaximumHeight = 1.0f;

	def.clockwiseWinding = false;
	b3HeightFieldData* ccw = b3CreateHeightField( &def );

	def.clockwiseWinding = true;
	b3HeightFieldData* cw = b3CreateHeightField( &def );

	b3Triangle ta = b3GetHeightFieldTriangle( ccw, 0 );
	b3Triangle tb = b3GetHeightFieldTriangle( cw, 0 );

	b3Vec3 na =
		b3Normalize( b3Cross( b3Sub( ta.vertices[1], ta.vertices[0] ), b3Sub( ta.vertices[2], ta.vertices[0] ) ) );
	b3Vec3 nb =
		b3Normalize( b3Cross( b3Sub( tb.vertices[1], tb.vertices[0] ), b3Sub( tb.vertices[2], tb.vertices[0] ) ) );

	ENSURE_SMALL( na.x, FLT_EPSILON );
	ENSURE_SMALL( na.y - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( na.z, FLT_EPSILON );

	ENSURE_SMALL( nb.x, FLT_EPSILON );
	ENSURE_SMALL( nb.y + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( nb.z, FLT_EPSILON );

	b3DestroyHeightField( ccw );
	b3DestroyHeightField( cw );
	return 0;
}

static int RayCastFlatField( void )
{
	// Build a flat 4x4 field with a tight quantization range so the recovered
	// surface stays within ~1e-5 of y=0 (b3CreateGrid uses -256..256 which
	// blows the 1/UINT16_MAX quantum up to ~4e-3 in y).
	float heights[16] = { 0 };
	uint8_t materials[9] = { 0 };

	b3HeightFieldDef def = { 0 };
	def.heights = heights;
	def.materialIndices = materials;
	def.scale = (b3Vec3){ 1.0f, 1.0f, 1.0f };
	def.countX = 4;
	def.countZ = 4;
	def.globalMinimumHeight = -1.0f;
	def.globalMaximumHeight = 1.0f;
	def.clockwiseWinding = false;

	b3HeightFieldData* hf = b3CreateHeightField( &def );

	// Origin sits clearly inside triangle 0 of cell (1, 1) — off the cell
	// diagonal x+z = 3. The translation overshoots the surface so the hit
	// fraction is strictly less than maxFraction.
	b3RayCastInput input = { 0 };
	input.origin = (b3Vec3){ 1.25f, 10.0f, 1.25f };
	input.translation = (b3Vec3){ 0.0f, -20.0f, 0.0f };
	input.maxFraction = 1.0f;

	b3CastOutput out = b3RayCastHeightField( hf, &input );

	ENSURE( out.hit == true );
	ENSURE_SMALL( out.fraction - 0.5f, 1e-5f );
	ENSURE_SMALL( out.normal.x, 1e-5f );
	ENSURE_SMALL( out.normal.y - 1.0f, 1e-5f );
	ENSURE_SMALL( out.normal.z, 1e-5f );

	b3DestroyHeightField( hf );
	return 0;
}

static int OverlapAtSurface( void )
{
	b3Vec3 scale = { 1.0f, 1.0f, 1.0f };
	b3HeightFieldData* hf = b3CreateGrid( 4, 4, scale, false );

	// Sphere center 1.0 above the surface, radius 0.5 — clear gap.
	b3Vec3 above = { 1.5f, 1.0f, 1.5f };
	b3ShapeProxy proxyAbove = { &above, 1, 0.5f };
	bool hitAbove = b3OverlapHeightField( hf, b3Transform_identity, &proxyAbove );
	ENSURE( hitAbove == false );

	// Sphere centered on the surface — radius pokes through.
	b3Vec3 through = { 1.5f, 0.0f, 1.5f };
	b3ShapeProxy proxyThrough = { &through, 1, 0.5f };
	bool hitThrough = b3OverlapHeightField( hf, b3Transform_identity, &proxyThrough );
	ENSURE( hitThrough == true );

	b3DestroyHeightField( hf );
	return 0;
}

static int FileRoundtrip( void )
{
	float heights[9] = { 0.0f, 0.5f, -0.3f, 0.1f, 0.0f, 0.0f, 0.0f, 0.2f, 0.0f };
	uint8_t materials[4] = { 0, B3_HEIGHT_FIELD_HOLE, 1, 2 };

	b3HeightFieldDef def = { 0 };
	def.heights = heights;
	def.materialIndices = materials;
	def.scale = (b3Vec3){ 1.5f, 2.0f, 0.75f };
	def.countX = 3;
	def.countZ = 3;
	def.globalMinimumHeight = -1.0f;
	def.globalMaximumHeight = 1.0f;
	def.clockwiseWinding = true;

	const char* path = "test_height_field_roundtrip.dat";
	b3DumpHeightData( &def, path );

	b3HeightFieldData* loaded = b3LoadHeightField( path );
	remove( path );

	ENSURE( loaded != NULL );
	ENSURE( loaded->rowCount == def.countZ );
	ENSURE( loaded->columnCount == def.countX );
	ENSURE( loaded->clockwise == def.clockwiseWinding );

	ENSURE_SMALL( loaded->scale.x - def.scale.x, FLT_EPSILON );
	ENSURE_SMALL( loaded->scale.y - def.scale.y, FLT_EPSILON );
	ENSURE_SMALL( loaded->scale.z - def.scale.z, FLT_EPSILON );
	ENSURE_SMALL( loaded->minHeight - def.globalMinimumHeight, FLT_EPSILON );
	ENSURE_SMALL( loaded->maxHeight - def.globalMaximumHeight, FLT_EPSILON );

	int cellCount = ( def.countX - 1 ) * ( def.countZ - 1 );
	for ( int i = 0; i < cellCount; ++i )
	{
		ENSURE( b3GetHeightFieldMaterialIndices( loaded )[i] == materials[i] );
	}

	// Recovered heights round-trip within the quantization tolerance.
	float quantum = ( def.globalMaximumHeight - def.globalMinimumHeight ) / (float)UINT16_MAX;
	for ( int i = 0; i < def.countX * def.countZ; ++i )
	{
		float recovered = loaded->minHeight + loaded->heightScale * b3GetHeightFieldCompressedHeights( loaded )[i];
		ENSURE_SMALL( recovered - heights[i], 2.0f * quantum );
	}

	b3DestroyHeightField( loaded );
	return 0;
}

static int ShapeCastVerticalStraddle( void )
{
	// Regression: a vertical shape cast whose swept volume straddles a cell
	// boundary must test every cell it overlaps. The field is flat at y = 0 with
	// only cell (0,0) solid; the surrounding cells are holes. Each sphere is
	// dropped straight down with its center nudged just past a boundary of the
	// solid cell, so that cell sits on the trailing (-x / -z) side of the sweep.
	// A cull AABB pinned to the leading box corner skips the solid cell entirely
	// and reports a miss.
	float heights[9] = { 0 };
	uint8_t materials[4] = { 0, B3_HEIGHT_FIELD_HOLE, B3_HEIGHT_FIELD_HOLE, B3_HEIGHT_FIELD_HOLE };

	b3HeightFieldDef def = { 0 };
	def.heights = heights;
	def.materialIndices = materials;
	def.scale = (b3Vec3){ 1.0f, 1.0f, 1.0f };
	def.countX = 3;
	def.countZ = 3;
	def.globalMinimumHeight = -1.0f;
	def.globalMaximumHeight = 1.0f;
	def.clockwiseWinding = false;

	b3HeightFieldData* hf = b3CreateHeightField( &def );

	// Solid cell (0,0) spans x,z in [0,1]. Radius 0.3 with the center 0.05 past a
	// boundary still reaches back into the solid cell.
	const float radius = 0.3f;

	// Straddle the x = 1 edge: solid cell is on the -x side. Contact lands on the
	// cell edge, sqrt(0.05^2 + cy^2) = radius -> cy = 0.2958040,
	// fraction = (10 - cy) / 20 = 0.4852098.
	{
		b3Vec3 center = { 1.05f, 10.0f, 0.5f };
		b3ShapeCastInput input = { 0 };
		input.proxy = (b3ShapeProxy){ &center, 1, radius };
		input.translation = (b3Vec3){ 0.0f, -20.0f, 0.0f };
		input.maxFraction = 1.0f;

		b3CastOutput out = b3ShapeCastHeightField( hf, &input );
		ENSURE( out.hit == true );
		ENSURE_SMALL( out.fraction - 0.4852098f, 2e-3f );
	}

	// Straddle the z = 1 edge: solid cell is on the -z side (same geometry).
	{
		b3Vec3 center = { 0.5f, 10.0f, 1.05f };
		b3ShapeCastInput input = { 0 };
		input.proxy = (b3ShapeProxy){ &center, 1, radius };
		input.translation = (b3Vec3){ 0.0f, -20.0f, 0.0f };
		input.maxFraction = 1.0f;

		b3CastOutput out = b3ShapeCastHeightField( hf, &input );
		ENSURE( out.hit == true );
		ENSURE_SMALL( out.fraction - 0.4852098f, 2e-3f );
	}

	// Straddle the (1,1) corner: solid cell is diagonally trailing. Contact lands
	// on the corner vertex, sqrt(2*0.05^2 + cy^2) = radius -> cy = 0.2915476,
	// fraction = (10 - cy) / 20 = 0.4854226.
	{
		b3Vec3 center = { 1.05f, 10.0f, 1.05f };
		b3ShapeCastInput input = { 0 };
		input.proxy = (b3ShapeProxy){ &center, 1, radius };
		input.translation = (b3Vec3){ 0.0f, -20.0f, 0.0f };
		input.maxFraction = 1.0f;

		b3CastOutput out = b3ShapeCastHeightField( hf, &input );
		ENSURE( out.hit == true );
		ENSURE_SMALL( out.fraction - 0.4854226f, 2e-3f );
	}

	b3DestroyHeightField( hf );
	return 0;
}

// Brute-force shape cast: cast the proxy against every (non-hole) triangle and
// keep the closest hit. This is the ground truth for b3ShapeCastHeightField.
static b3CastOutput BruteForceShapeCast( const b3HeightFieldData* hf, const b3ShapeCastInput* input )
{
	b3CastOutput best = { 0 };
	float bestFraction = input->maxFraction;

	int triangleCount = b3GetHeightFieldTriangleCount( hf );
	for ( int t = 0; t < triangleCount; ++t )
	{
		int cellIndex = t >> 1;
		if ( b3GetHeightFieldMaterialIndices( hf )[cellIndex] == B3_HEIGHT_FIELD_HOLE )
		{
			continue;
		}

		b3Triangle tri = b3GetHeightFieldTriangle( hf, t );

		b3ShapeCastPairInput pair = { 0 };
		pair.proxyA = (b3ShapeProxy){ tri.vertices, 3, 0.0f };
		pair.proxyB = input->proxy;
		pair.transform = b3Transform_identity;
		pair.translationB = input->translation;
		pair.maxFraction = bestFraction;
		pair.canEncroach = input->canEncroach;

		b3CastOutput out = b3ShapeCast( &pair );
		if ( out.hit && out.fraction < bestFraction )
		{
			bestFraction = out.fraction;
			best = out;
			best.triangleIndex = t;
		}
	}

	return best;
}

static int ShapeCastBruteForce( void )
{
	// b3ShapeCastHeightField walks the grid and culls cells; the brute-force cast
	// against every triangle is the ground truth. The grid walk must never miss a
	// closer hit, regardless of cast direction, origin or radius.
	b3Vec3 scale = { 2.0f, 1.5f, 2.0f };
	b3HeightFieldData* hf = b3CreateWave( 10, 10, scale, 0.1f, 0.03333f, false );

	// Documented repro from sample/sample_mesh.cpp "Height Field": a sphere cast
	// that moves only in z (and y). Body at (-9,0,-9), world origin (5.5,4,2.913)
	// -> local (14.5,4,11.913). The grid walk used to terminate one row early
	// because it compared a clamped-sweep fraction against an input-space one.
	{
		b3Vec3 origin = { 14.5f, 4.0f, 11.913f };
		b3ShapeCastInput input = { 0 };
		input.proxy = (b3ShapeProxy){ &origin, 1, 0.2f };
		input.translation = (b3Vec3){ 0.0f, -8.0f, 6.397f };
		input.maxFraction = 1.0f;

		b3CastOutput grid = b3ShapeCastHeightField( hf, &input );
		b3CastOutput brute = BruteForceShapeCast( hf, &input );
		ENSURE( brute.hit == true );
		ENSURE( grid.hit == brute.hit );
		ENSURE_SMALL( grid.fraction - brute.fraction, 2e-3f );
	}

	// Sweep origins across the field with assorted directions and radii.
	const float radii[] = { 0.15f, 0.4f, 0.9f };
	const b3Vec3 deltas[] = {
		{ 0.0f, -8.0f, 0.0f },	// vertical
		{ 0.0f, -8.0f, 6.4f },	// z only (+ y)
		{ 5.1f, -8.0f, 0.0f },	// x only (+ y)
		{ 0.0f, -8.0f, -6.4f }, // -z
		{ -5.1f, -8.0f, 0.0f }, // -x
		{ 6.0f, -8.0f, 5.0f },	// diagonal
		{ -7.0f, -8.0f, 4.0f }, // diagonal, mixed sign
		{ 9.0f, -3.0f, -9.0f }, // shallow diagonal
	};

	int failures = 0;
	for ( int xi = 0; xi < 5; ++xi )
	{
		for ( int zi = 0; zi < 5; ++zi )
		{
			// 0.05 nudge keeps the swept box straddling cell boundaries.
			b3Vec3 origin = { 1.0f + 4.0f * xi + 0.05f, 4.0f, 1.0f + 4.0f * zi + 0.05f };

			for ( int di = 0; di < ARRAY_COUNT( deltas ); ++di )
			{
				for ( int ri = 0; ri < ARRAY_COUNT( radii ); ++ri )
				{
					b3ShapeCastInput input = { 0 };
					input.proxy = (b3ShapeProxy){ &origin, 1, radii[ri] };
					input.translation = deltas[di];
					input.maxFraction = 1.0f;

					b3CastOutput grid = b3ShapeCastHeightField( hf, &input );
					b3CastOutput brute = BruteForceShapeCast( hf, &input );

					float diff = grid.fraction - brute.fraction;
					diff = diff < 0.0f ? -diff : diff;

					if ( grid.hit != brute.hit || ( brute.hit && diff > 2e-3f ) )
					{
						printf( "  mismatch: origin=(%.2f,%.2f,%.2f) delta=(%.2f,%.2f,%.2f) r=%.2f"
								" grid(hit=%d,f=%.5f) brute(hit=%d,f=%.5f tri=%d)\n",
								origin.x, origin.y, origin.z, deltas[di].x, deltas[di].y, deltas[di].z, radii[ri],
								grid.hit, grid.fraction, brute.hit, brute.fraction, brute.triangleIndex );
						failures += 1;
					}
				}
			}
		}
	}

	ENSURE( failures == 0 );

	b3DestroyHeightField( hf );
	return 0;
}

// Brute-force ray cast: cast the ray against every (non-hole) triangle and keep
// the closest hit. b3GetHeightFieldTriangle returns vertices in the same winding
// order that b3ShapeCastHeightField feeds to b3IntersectRayTriangle, so this is a
// pure traversal/culling check — the per-triangle math is identical.
static b3CastOutput BruteForceRayCast( const b3HeightFieldData* hf, const b3RayCastInput* input )
{
	b3CastOutput best = { 0 };
	float bestFraction = input->maxFraction;

	b3V32 rayStart = b3LoadV( &input->origin.x );
	b3V32 rayDelta = b3LoadV( &input->translation.x );

	int triangleCount = b3GetHeightFieldTriangleCount( hf );
	for ( int t = 0; t < triangleCount; ++t )
	{
		int cellIndex = t >> 1;
		if ( b3GetHeightFieldMaterialIndices( hf )[cellIndex] == B3_HEIGHT_FIELD_HOLE )
		{
			continue;
		}

		b3Triangle tri = b3GetHeightFieldTriangle( hf, t );
		b3V32 v1 = b3LoadV( &tri.vertices[0].x );
		b3V32 v2 = b3LoadV( &tri.vertices[1].x );
		b3V32 v3 = b3LoadV( &tri.vertices[2].x );

		// b3IntersectRayTriangle returns 1.0 on a miss.
		float alpha = b3IntersectRayTriangle( rayStart, rayDelta, v1, v2, v3 );
		if ( alpha < bestFraction )
		{
			bestFraction = alpha;
			best.hit = true;
			best.fraction = alpha;
			best.triangleIndex = t;
		}
	}

	return best;
}

static int RayCastBruteForce( void )
{
	// b3RayCastHeightField routes through the same grid walk as the shape cast
	// (radius-zero point proxy), so it is subject to the same early-termination
	// bug. The brute-force cast against every triangle is the ground truth.
	b3Vec3 scale = { 2.0f, 1.5f, 2.0f };
	b3HeightFieldData* hf = b3CreateWave( 10, 10, scale, 0.1f, 0.03333f, false );

	const b3Vec3 deltas[] = {
		{ 0.0f, -8.0f, 0.0f },	  // straight down
		{ 0.0f, -8.0f, 12.0f },	  // down + z
		{ 12.0f, -8.0f, 0.0f },	  // down + x
		{ 0.0f, -8.0f, -12.0f },  // down - z
		{ -12.0f, -8.0f, 0.0f },  // down - x
		{ 14.0f, -8.0f, 11.0f },  // diagonal
		{ -13.0f, -8.0f, 9.0f },  // diagonal, mixed sign
		{ 16.0f, -4.0f, -15.0f }, // shallow diagonal
	};

	int failures = 0;
	for ( int xi = 0; xi < 5; ++xi )
	{
		for ( int zi = 0; zi < 5; ++zi )
		{
			// 0.05 nudge keeps the ray off cell boundaries.
			b3Vec3 origin = { 1.0f + 4.0f * xi + 0.05f, 4.0f, 1.0f + 4.0f * zi + 0.05f };

			for ( int di = 0; di < ARRAY_COUNT( deltas ); ++di )
			{
				b3RayCastInput input = { 0 };
				input.origin = origin;
				input.translation = deltas[di];
				input.maxFraction = 1.0f;

				b3CastOutput grid = b3RayCastHeightField( hf, &input );
				b3CastOutput brute = BruteForceRayCast( hf, &input );

				float diff = grid.fraction - brute.fraction;
				diff = diff < 0.0f ? -diff : diff;

				if ( grid.hit != brute.hit || ( brute.hit && diff > 1e-4f ) )
				{
					printf( "  mismatch: origin=(%.2f,%.2f,%.2f) delta=(%.2f,%.2f,%.2f)"
							" grid(hit=%d,f=%.6f tri=%d) brute(hit=%d,f=%.6f tri=%d)\n",
							origin.x, origin.y, origin.z, deltas[di].x, deltas[di].y, deltas[di].z, grid.hit,
							grid.fraction, grid.triangleIndex, brute.hit, brute.fraction, brute.triangleIndex );
					failures += 1;
				}
			}
		}
	}

	ENSURE( failures == 0 );

	b3DestroyHeightField( hf );
	return 0;
}

int HeightFieldTest( void )
{
	RUN_SUBTEST( HeightFieldCreate );
	RUN_SUBTEST( HeightFieldTriangleIndex );
	RUN_SUBTEST( HeightFieldWinding );
	RUN_SUBTEST( RayCastFlatField );
	RUN_SUBTEST( OverlapAtSurface );
	RUN_SUBTEST( FileRoundtrip );
	RUN_SUBTEST( ShapeCastVerticalStraddle );
	RUN_SUBTEST( ShapeCastBruteForce );
	RUN_SUBTEST( RayCastBruteForce );

	return 0;
}
