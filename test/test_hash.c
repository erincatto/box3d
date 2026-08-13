// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"
#include "hull.h"
#include "physics_world.h"
#include "table.h"
#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <stdint.h>
#include <string.h>

// Separates the two things a repeated hash can mean. Identical bytes repeating is normal, since
// distinct generator parameters can still bake to the same geometry. Distinct bytes sharing a hash
// is the failure this file exists to catch, so the invariant under test is that different content
// gets different digests, never that different parameters do.
//
// Zero doubles as "no collision", since the hash reserves that value. Results are read back only
// after teardown so a failing test does not also dump a leak and bury which condition broke.
typedef struct HashEntry
{
	uint64_t hash;
	int byteCount;
	uint8_t* bytes;
} HashEntry;

typedef struct HashProbe
{
	b3HashSet set;
	HashEntry* entries;
	int count;
	int capacity;
	uint64_t collision;
	int duplicates;
	bool sawZero;
	bool overflow;
} HashProbe;

typedef struct HashResult
{
	uint64_t collision;
	int count;
	int duplicates;
	bool sawZero;
	bool overflow;
} HashResult;

static HashProbe ProbeBegin( int capacity )
{
	HashProbe probe = { 0 };
	probe.set = b3CreateSet( capacity );
	probe.entries = B3_ALLOC( HashEntry, capacity );
	probe.capacity = capacity;
	return probe;
}

// Pass NULL bytes when the caller builds provably distinct inputs, so any repeat is a collision by
// construction and there is nothing to compare against.
static void ProbeAdd( HashProbe* probe, uint64_t hash, const void* bytes, int byteCount )
{
	if ( hash == 0 )
	{
		probe->sawZero = true;
	}

	if ( probe->count == probe->capacity )
	{
		probe->overflow = true;
		return;
	}

	if ( b3AddKey( &probe->set, hash ) )
	{
		bool sameContent = false;
		if ( bytes != NULL )
		{
			for ( int i = 0; i < probe->count; ++i )
			{
				if ( probe->entries[i].hash != hash )
				{
					continue;
				}

				sameContent = probe->entries[i].byteCount == byteCount &&
							  memcmp( probe->entries[i].bytes, bytes, (size_t)byteCount ) == 0;
				break;
			}
		}

		if ( sameContent )
		{
			probe->duplicates += 1;
		}
		else if ( probe->collision == 0 )
		{
			probe->collision = hash;
		}
	}

	HashEntry* entry = probe->entries + probe->count;
	entry->hash = hash;
	entry->byteCount = 0;
	entry->bytes = NULL;

	if ( bytes != NULL && byteCount > 0 )
	{
		entry->byteCount = byteCount;
		entry->bytes = B3_ALLOC( uint8_t, byteCount );
		memcpy( entry->bytes, bytes, (size_t)byteCount );
	}

	probe->count += 1;
}

static HashResult ProbeEnd( HashProbe* probe )
{
	HashResult result = { probe->collision, probe->count, probe->duplicates, probe->sawZero, probe->overflow };

	for ( int i = 0; i < probe->count; ++i )
	{
		if ( probe->entries[i].bytes != NULL )
		{
			B3_FREE( probe->entries[i].bytes, uint8_t, probe->entries[i].byteCount );
		}
	}

	B3_FREE( probe->entries, HashEntry, probe->capacity );
	b3DestroySet( &probe->set );
	return result;
}

// Pins that the upper bytes of each 8 byte word reach the low bits of the hash. Multiply carries
// bits upward only, so a hash that keeps just the low half of its product leaves the low bits blind
// to those bytes, and blobs differing only up there collapse onto a tiny range no finalizer can
// spread back out. Varying the top byte of two words is the sharp case. A pairwise avalanche check
// cannot catch this, since a finalizer scatters any single difference across all bits no matter how
// weak the mixing behind it.
static int HashWordFamily( void )
{
	enum
	{
		blobSize = 16,
		familySize = 256 * 256
	};

	HashProbe probe = ProbeBegin( familySize );

	uint8_t blob[blobSize];
	for ( int i = 0; i < blobSize; ++i )
	{
		blob[i] = (uint8_t)( 0x5A + i );
	}

	for ( int a = 0; a < 256; ++a )
	{
		for ( int b = 0; b < 256; ++b )
		{
			blob[7] = (uint8_t)a;
			blob[15] = (uint8_t)b;
			ProbeAdd( &probe, b3Hash64NonZero( blob, blobSize ), NULL, 0 );
		}
	}

	HashResult result = ProbeEnd( &probe );
	ENSURE( result.overflow == false );
	ENSURE( result.count == familySize );
	ENSURE( result.sawZero == false );
	ENSURE( result.collision == 0 );
	return 0;
}

// Every input bit must move the digest, at every offset. Flipping one byte in one position is a
// weak check: a mixer can pass it while still being blind to whole regions of a longer blob.
static int HashBitSweep( void )
{
	enum
	{
		blobSize = 32,
		bitCount = 8 * blobSize
	};

	HashProbe probe = ProbeBegin( bitCount + 1 );

	uint8_t blob[blobSize];
	for ( int i = 0; i < blobSize; ++i )
	{
		blob[i] = (uint8_t)( 0xA5 ^ ( i * 17 ) );
	}

	ProbeAdd( &probe, b3Hash64NonZero( blob, blobSize ), NULL, 0 );

	for ( int bit = 0; bit < bitCount; ++bit )
	{
		uint8_t mask = (uint8_t)( 1u << ( bit & 7 ) );
		blob[bit >> 3] ^= mask;
		ProbeAdd( &probe, b3Hash64NonZero( blob, blobSize ), NULL, 0 );
		blob[bit >> 3] ^= mask;
	}

	HashResult result = ProbeEnd( &probe );
	ENSURE( result.overflow == false );
	ENSURE( result.count == bitCount + 1 );
	ENSURE( result.sawZero == false );
	ENSURE( result.collision == 0 );
	return 0;
}

// Baked blobs carry explicit padding and unused array slots, so long runs of zeros are common and
// content alone cannot separate them. Length has to reach the digest.
static int HashZeroLengths( void )
{
	enum
	{
		maxLength = 4096
	};

	uint8_t* zeros = B3_ALLOC( uint8_t, maxLength );
	memset( zeros, 0, maxLength );

	HashProbe probe = ProbeBegin( maxLength );
	for ( int n = 0; n < maxLength; ++n )
	{
		ProbeAdd( &probe, b3Hash64NonZero( zeros, n ), NULL, 0 );
	}

	HashResult result = ProbeEnd( &probe );
	B3_FREE( zeros, uint8_t, maxLength );

	ENSURE( result.overflow == false );
	ENSURE( result.count == maxLength );
	ENSURE( result.sawZero == false );
	ENSURE( result.collision == 0 );
	return 0;
}

// Mirrored geometry differs from its original only in sign bits, which sit at the top of every
// float. A mixer that cannot carry high bits downward maps the whole family onto a few digests.
static int HashFloatSigns( void )
{
	enum
	{
		floatCount = 12,
		comboCount = 1 << floatCount
	};

	HashProbe probe = ProbeBegin( comboCount );

	float values[floatCount];
	for ( int i = 0; i < floatCount; ++i )
	{
		values[i] = 1.0f + 0.25f * (float)i;
	}

	for ( int mask = 0; mask < comboCount; ++mask )
	{
		float flipped[floatCount];
		for ( int i = 0; i < floatCount; ++i )
		{
			flipped[i] = ( mask & ( 1 << i ) ) != 0 ? -values[i] : values[i];
		}

		ProbeAdd( &probe, b3Hash64NonZero( (const uint8_t*)flipped, (int)sizeof( flipped ) ), NULL, 0 );
	}

	HashResult result = ProbeEnd( &probe );
	ENSURE( result.overflow == false );
	ENSURE( result.count == comboCount );
	ENSURE( result.sawZero == false );
	ENSURE( result.collision == 0 );
	return 0;
}

// Procedurally placed vertices land one ulp apart. Those blobs differ in a single low mantissa bit
// buried in a long run of identical bytes.
static int HashFloatUlp( void )
{
	enum
	{
		floatCount = 64
	};

	HashProbe probe = ProbeBegin( floatCount + 1 );

	float values[floatCount];
	for ( int i = 0; i < floatCount; ++i )
	{
		values[i] = 100.0f + (float)i;
	}

	ProbeAdd( &probe, b3Hash64NonZero( (const uint8_t*)values, (int)sizeof( values ) ), NULL, 0 );

	for ( int i = 0; i < floatCount; ++i )
	{
		float saved = values[i];
		uint32_t bits;
		memcpy( &bits, &saved, sizeof( bits ) );
		bits += 1;
		memcpy( values + i, &bits, sizeof( bits ) );

		ProbeAdd( &probe, b3Hash64NonZero( (const uint8_t*)values, (int)sizeof( values ) ), NULL, 0 );
		values[i] = saved;
	}

	HashResult result = ProbeEnd( &probe );
	ENSURE( result.overflow == false );
	ENSURE( result.count == floatCount + 1 );
	ENSURE( result.sawZero == false );
	ENSURE( result.collision == 0 );
	return 0;
}

// Real baked hulls. Every box shares its entire topology section and differs in a handful of floats,
// which is the closest thing the engine produces to a worst case for a content hash.
static int HashBoxHulls( void )
{
	enum
	{
		steps = 16
	};

	HashProbe probe = ProbeBegin( steps * steps * steps );

	for ( int i = 0; i < steps; ++i )
	{
		for ( int j = 0; j < steps; ++j )
		{
			for ( int k = 0; k < steps; ++k )
			{
				b3BoxHull box = b3MakeBoxHull( 0.5f + 0.25f * (float)i, 0.5f + 0.25f * (float)j, 0.5f + 0.25f * (float)k );
				ProbeAdd( &probe, box.base.hash, &box, box.base.byteCount );
			}
		}
	}

	HashResult result = ProbeEnd( &probe );
	ENSURE( result.overflow == false );
	ENSURE( result.count == steps * steps * steps );
	ENSURE( result.sawZero == false );
	ENSURE( result.collision == 0 );
	return 0;
}

// Same box, moved and turned. The extent bytes are identical across the family so the hash has to
// separate these on transform alone.
static int HashTransformedBoxHulls( void )
{
	enum
	{
		steps = 12
	};

	HashProbe probe = ProbeBegin( steps * steps );

	for ( int i = 0; i < steps; ++i )
	{
		for ( int j = 0; j < steps; ++j )
		{
			b3Vec3 axis = b3Normalize( (b3Vec3){ 1.0f, 0.5f + 0.1f * (float)j, 0.25f } );
			b3Transform transform = {
				.p = { 0.125f * (float)i, -0.25f * (float)j, 0.5f * (float)( i + j ) },
				.q = b3MakeQuatFromAxisAngle( axis, 0.05f * (float)( i * steps + j ) ),
			};

			b3BoxHull box = b3MakeTransformedBoxHull( 1.0f, 2.0f, 3.0f, transform );
			ProbeAdd( &probe, box.base.hash, &box, box.base.byteCount );
		}
	}

	HashResult result = ProbeEnd( &probe );
	ENSURE( result.overflow == false );
	ENSURE( result.count == steps * steps );
	ENSURE( result.sawZero == false );
	ENSURE( result.collision == 0 );
	return 0;
}

// Tessellated hulls across a parameter sweep. Neighboring parameters produce blobs that agree
// almost everywhere, including identical vertex counts and topology.
static int HashProceduralHulls( void )
{
	HashProbe probe = ProbeBegin( 1024 );

	for ( int sides = 3; sides <= 18; ++sides )
	{
		for ( int r = 1; r <= 5; ++r )
		{
			for ( int h = 1; h <= 4; ++h )
			{
				b3HullData* cylinder = b3CreateCylinder( 0.5f * (float)h, 0.25f * (float)r, 0.0f, sides );
				if ( cylinder != NULL )
				{
					ProbeAdd( &probe, cylinder->hash, cylinder, cylinder->byteCount );
					b3DestroyHull( cylinder );
				}

				// Cones need at least four slices
				if ( sides >= 4 )
				{
					b3HullData* cone = b3CreateCone( 0.5f * (float)h, 0.25f * (float)r, 0.1f * (float)r, sides );
					if ( cone != NULL )
					{
						ProbeAdd( &probe, cone->hash, cone, cone->byteCount );
						b3DestroyHull( cone );
					}
				}
			}
		}
	}

	HashResult result = ProbeEnd( &probe );
	ENSURE( result.overflow == false );
	ENSURE( result.count > 0 );
	ENSURE( result.sawZero == false );
	ENSURE( result.collision == 0 );
	return 0;
}

// Height fields are mostly a flat array of heights, so grids of nearby dimensions differ in very
// little beyond their counts. Small wave grids flatten to the plain grid, which is why the probe
// has to tell a duplicate blob apart from a collision.
static int HashHeightFields( void )
{
	HashProbe probe = ProbeBegin( 512 );

	for ( int rows = 2; rows <= 14; ++rows )
	{
		for ( int cols = 2; cols <= 14; ++cols )
		{
			b3Vec3 scale = { 1.0f, 1.0f, 1.0f };

			b3HeightFieldData* grid = b3CreateGrid( rows, cols, scale, false );
			ProbeAdd( &probe, grid->hash, grid, grid->byteCount );
			b3DestroyHeightField( grid );

			b3HeightFieldData* wave = b3CreateWave( rows, cols, scale, 0.25f * (float)rows, 0.125f * (float)cols, false );
			ProbeAdd( &probe, wave->hash, wave, wave->byteCount );
			b3DestroyHeightField( wave );
		}
	}

	HashResult result = ProbeEnd( &probe );
	ENSURE( result.overflow == false );
	ENSURE( result.count > 0 );
	ENSURE( result.sawZero == false );
	ENSURE( result.collision == 0 );
	return 0;
}

// Meshes carry a baked BVH, so most of the blob is derived data that moves in lockstep with small
// parameter changes.
static int HashMeshes( void )
{
	HashProbe probe = ProbeBegin( 256 );

	for ( int x = 2; x <= 10; ++x )
	{
		for ( int z = 2; z <= 10; ++z )
		{
			b3MeshData* grid = b3CreateGridMesh( x, z, 1.0f, 1, false );
			ProbeAdd( &probe, grid->hash, grid, grid->byteCount );
			b3DestroyMesh( grid );
		}
	}

	for ( int radial = 3; radial <= 12; ++radial )
	{
		for ( int tubular = 3; tubular <= 12; ++tubular )
		{
			b3MeshData* torus = b3CreateTorusMesh( radial, tubular, 1.0f, 0.25f );
			ProbeAdd( &probe, torus->hash, torus, torus->byteCount );
			b3DestroyMesh( torus );
		}
	}

	HashResult result = ProbeEnd( &probe );
	ENSURE( result.overflow == false );
	ENSURE( result.count > 0 );
	ENSURE( result.sawZero == false );
	ENSURE( result.collision == 0 );
	return 0;
}

// Voxel colliders sit on a regular grid, so their coordinate floats vary only in high bits. The
// hull database takes its home bucket from the low bits of the hash, so a mixer that cannot carry
// high bits downward funnels every hull into one bucket. The digests stay distinct throughout,
// which is exactly why the collision tests above cannot see it. From issue 120.
static int HashVoxelDispersion( void )
{
	enum
	{
		hullCount = 3000,
		lowBits = 13,
		lowCount = 1 << lowBits
	};

	bool* seen = B3_ALLOC( bool, lowCount );
	memset( seen, 0, lowCount * sizeof( bool ) );

	int distinctLow = 0;
	for ( int i = 0; i < hullCount; ++i )
	{
		const float cell = 0.25f;
		b3Vec3 offset = {
			(float)( i % 15 ) * cell,
			(float)( ( i / 15 ) % 20 ) * cell,
			(float)( i / 300 ) * cell,
		};

		b3BoxHull hull = b3MakeOffsetBoxHull( 0.5f * cell, 0.5f * cell, 0.5f * cell, offset );

		int low = (int)( hull.base.hash & ( lowCount - 1 ) );
		if ( seen[low] == false )
		{
			seen[low] = true;
			distinctLow += 1;
		}
	}

	B3_FREE( seen, bool, lowCount );

	// Filling 8192 slots with 3000 draws tops out near 2500. The 32 bit hash this replaced reached 1.
	ENSURE( distinctLow > hullCount / 2 );
	return 0;
}

// The consequence of the above, measured where it hurt. These hulls drove the world hull database
// to two million buckets and a hundred and fifty milliseconds, since every insert walked one chain.
static int HashVoxelHullDatabase( void )
{
	enum
	{
		hullCount = 3000
	};

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	b3World* world = b3GetWorldFromId( worldId );
	b3HullMap* map = (b3HullMap*)world->hullDatabase;

	bool created = true;
	for ( int i = 0; i < hullCount; ++i )
	{
		const float cell = 0.25f;
		b3Vec3 offset = {
			(float)( i % 15 ) * cell,
			(float)( ( i / 15 ) % 20 ) * cell,
			(float)( i / 300 ) * cell,
		};

		b3BoxHull hull = b3MakeOffsetBoxHull( 0.5f * cell, 0.5f * cell, 0.5f * cell, offset );
		b3ShapeId shapeId = b3CreateHullShape( bodyId, &shapeDef, &hull.base );
		if ( shapeId.index1 == 0 )
		{
			created = false;
			break;
		}
	}

	size_t entryCount = b3HullMap_size( map );
	size_t bucketCount = b3HullMap_bucket_count( map );

	b3DestroyWorld( worldId );

	ENSURE( created );
	ENSURE( entryCount == hullCount );

	// Healthy growth settles at 4096 for this many distinct hulls
	ENSURE( bucketCount <= 4 * hullCount );
	return 0;
}

// Identical input must bake to an identical hash, or dedup silently stops working.
static int HashStability( void )
{
	b3BoxHull box1 = b3MakeBoxHull( 1.0f, 2.0f, 3.0f );
	b3BoxHull box2 = b3MakeBoxHull( 1.0f, 2.0f, 3.0f );
	ENSURE( box1.base.hash == box2.base.hash );

	b3HullData* cylinder1 = b3CreateCylinder( 1.0f, 0.5f, 0.0f, 12 );
	b3HullData* cylinder2 = b3CreateCylinder( 1.0f, 0.5f, 0.0f, 12 );
	bool sameHash = cylinder1->hash == cylinder2->hash;
	bool sameSize = cylinder1->byteCount == cylinder2->byteCount;
	bool sameBytes = sameSize && memcmp( cylinder1, cylinder2, (size_t)cylinder1->byteCount ) == 0;
	b3DestroyHull( cylinder1 );
	b3DestroyHull( cylinder2 );

	ENSURE( sameHash );
	ENSURE( sameBytes );

	// An empty blob has no content to mix, so it takes the reserved value
	ENSURE( b3Hash64NonZero( NULL, 0 ) == 1 );
	return 0;
}

int HashTest( void )
{
	RUN_SUBTEST( HashWordFamily );
	RUN_SUBTEST( HashBitSweep );
	RUN_SUBTEST( HashZeroLengths );
	RUN_SUBTEST( HashFloatSigns );
	RUN_SUBTEST( HashFloatUlp );
	RUN_SUBTEST( HashBoxHulls );
	RUN_SUBTEST( HashTransformedBoxHulls );
	RUN_SUBTEST( HashProceduralHulls );
	RUN_SUBTEST( HashHeightFields );
	RUN_SUBTEST( HashMeshes );
	RUN_SUBTEST( HashVoxelDispersion );
	RUN_SUBTEST( HashVoxelHullDatabase );
	RUN_SUBTEST( HashStability );
	return 0;
}
