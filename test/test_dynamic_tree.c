// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "aabb.h"
#include "dynamic_tree.h"
#include "test_macros.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <stdlib.h>
#include <string.h>

#define PROXY_COUNT 300

static uint32_t s_seed = 12345;

static uint32_t NextRandom( void )
{
	s_seed = 1664525u * s_seed + 1013904223u;
	return s_seed;
}

static float RandomFloat( float lo, float hi )
{
	float t = (float)( NextRandom() >> 8 ) / (float)( 1 << 24 );
	return lo + t * ( hi - lo );
}

static b3AABB RandomBox( float extent, float size )
{
	b3Vec3 center = { RandomFloat( -extent, extent ), RandomFloat( -extent, extent ), RandomFloat( -extent, extent ) };
	b3Vec3 half = { RandomFloat( 0.1f, size ), RandomFloat( 0.1f, size ), RandomFloat( 0.1f, size ) };
	return (b3AABB){ b3Sub( center, half ), b3Add( center, half ) };
}

typedef struct HitSet
{
	int ids[PROXY_COUNT];
	int count;
	bool bad;
} HitSet;

// Every test seeds user data with the index of the box it created the proxy from, so the sets
// are keyed on user data. Proxy ids recycle, box indices do not.
// A callback cannot fail the test directly, so bad news rides back on the context.
static bool CollectCallback( int proxyId, uint64_t userData, void* context )
{
	HitSet* set = context;
	if ( set->count >= PROXY_COUNT || (int)userData < 0 || (int)userData >= PROXY_COUNT )
	{
		set->bad = true;
		return false;
	}

	(void)proxyId;
	set->ids[set->count] = (int)userData;
	set->count += 1;
	return true;
}

static int CompareInt( const void* a, const void* b )
{
	int x = *(const int*)a;
	int y = *(const int*)b;
	return x < y ? -1 : ( x > y ? 1 : 0 );
}

static int SameSet( HitSet* a, HitSet* b )
{
	ENSURE( a->bad == false );
	ENSURE( b->bad == false );
	ENSURE( a->count == b->count );
	qsort( a->ids, (size_t)a->count, sizeof( int ), CompareInt );
	qsort( b->ids, (size_t)b->count, sizeof( int ), CompareInt );
	for ( int i = 0; i < a->count; ++i )
	{
		ENSURE( a->ids[i] == b->ids[i] );
	}

	return 0;
}

// The tree must agree with a linear scan over the stored boxes. This is the gate on the
// SIMD box helpers, which have three separate lane layouts.
static int TreeQueryBruteForce( void )
{
	s_seed = 12345;

	b3DynamicTree tree = b3DynamicTree_Create( 16 );

	b3AABB boxes[PROXY_COUNT];
	int proxies[PROXY_COUNT];

	for ( int i = 0; i < PROXY_COUNT; ++i )
	{
		boxes[i] = RandomBox( 6.0f, 1.5f );
		proxies[i] = b3DynamicTree_CreateProxy( &tree, boxes[i], B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
		ENSURE( proxies[i] == i );
	}

	b3DynamicTree_Validate( &tree );
	ENSURE( b3DynamicTree_GetProxyCount( &tree ) == PROXY_COUNT );

	int totalHits = 0;
	for ( int round = 0; round < 200; ++round )
	{
		b3AABB queryBox = RandomBox( 7.0f, 3.0f );

		HitSet treeHits = { 0 };
		b3DynamicTree_Query( &tree, queryBox, B3_DEFAULT_MASK_BITS, false, CollectCallback, &treeHits );

		HitSet bruteHits = { 0 };
		for ( int i = 0; i < PROXY_COUNT; ++i )
		{
			if ( b3AABB_Overlaps( boxes[i], queryBox ) )
			{
				bruteHits.ids[bruteHits.count] = i;
				bruteHits.count += 1;
			}
		}

		ENSURE( SameSet( &treeHits, &bruteHits ) == 0 );
		totalHits += treeHits.count;
	}

	// Guard against the comparison above agreeing on nothing
	ENSURE( totalHits > 1000 );

	b3DynamicTree_Destroy( &tree );
	return 0;
}

// Moving and enlarging every proxy exercises remove, insert, the ordered flag, the sweep refit
// and the partial rebuild, then the query has to still agree with the scan.
static int TreeMoveRefitRebuild( void )
{
	s_seed = 777;

	b3DynamicTree tree = b3DynamicTree_Create( 16 );

	b3AABB boxes[PROXY_COUNT];
	int proxies[PROXY_COUNT];

	for ( int i = 0; i < PROXY_COUNT; ++i )
	{
		boxes[i] = RandomBox( 6.0f, 1.0f );
		proxies[i] = b3DynamicTree_CreateProxy( &tree, boxes[i], B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
	}

	b3DynamicTree_Rebuild( &tree, true );
	b3DynamicTree_Validate( &tree );
	ENSURE( tree.dfsOrdered );

	int totalHits = 0;
	for ( int round = 0; round < 8; ++round )
	{
		// Half the proxies move, half grow in place
		for ( int i = 0; i < PROXY_COUNT; ++i )
		{
			if ( ( i & 1 ) == 0 )
			{
				boxes[i] = RandomBox( 6.0f, 1.0f );
				b3DynamicTree_MoveProxy( &tree, proxies[i], boxes[i] );
			}
			else
			{
				b3AABB grown = b3AABB_Inflate( boxes[i], 0.35f );
				boxes[i] = grown;
				b3DynamicTree_EnlargeProxy( &tree, proxies[i], grown );
			}
		}

		b3DynamicTree_Refit( &tree );
		b3DynamicTree_Validate( &tree );

		b3DynamicTree_Rebuild( &tree, false );
		b3DynamicTree_Validate( &tree );
		b3DynamicTree_ValidateNoMoved( &tree );
		ENSURE( tree.dfsOrdered );

		// Every stored box must still be inside its leaf and inside the root
		b3AABB rootBounds = b3DynamicTree_GetRootBounds( &tree );
		for ( int i = 0; i < PROXY_COUNT; ++i )
		{
			ENSURE( b3AABB_Contains( b3DynamicTree_GetAABB( &tree, proxies[i] ), boxes[i] ) );
			ENSURE( b3AABB_Contains( rootBounds, boxes[i] ) );
			ENSURE( (int)b3DynamicTree_GetUserData( &tree, proxies[i] ) == i );
		}

		b3AABB queryBox = RandomBox( 7.0f, 3.0f );

		HitSet treeHits = { 0 };
		b3DynamicTree_Query( &tree, queryBox, B3_DEFAULT_MASK_BITS, false, CollectCallback, &treeHits );

		HitSet bruteHits = { 0 };
		for ( int i = 0; i < PROXY_COUNT; ++i )
		{
			if ( b3AABB_Overlaps( boxes[i], queryBox ) )
			{
				bruteHits.ids[bruteHits.count] = i;
				bruteHits.count += 1;
			}
		}

		ENSURE( SameSet( &treeHits, &bruteHits ) == 0 );
		totalHits += treeHits.count;
	}

	// Guard against the comparison above agreeing on nothing
	ENSURE( totalHits > 50 );

	b3DynamicTree_Destroy( &tree );
	return 0;
}

// Destroying proxies in a scrambled order recycles sibling pairs through the free list and
// leaves the array unordered, which is the path the fallback refit has to handle.
static int TreeDestroyScrambled( void )
{
	s_seed = 4242;

	b3DynamicTree tree = b3DynamicTree_Create( 16 );

	b3AABB boxes[PROXY_COUNT];
	int proxies[PROXY_COUNT];
	bool alive[PROXY_COUNT];

	for ( int i = 0; i < PROXY_COUNT; ++i )
	{
		boxes[i] = RandomBox( 10.0f, 0.8f );
		proxies[i] = b3DynamicTree_CreateProxy( &tree, boxes[i], B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
		alive[i] = true;
	}

	for ( int i = 0; i < PROXY_COUNT; i += 3 )
	{
		b3DynamicTree_DestroyProxy( &tree, proxies[i] );
		alive[i] = false;
	}

	b3DynamicTree_Validate( &tree );

	// Re-create into the holes
	for ( int i = 0; i < PROXY_COUNT; i += 3 )
	{
		boxes[i] = RandomBox( 10.0f, 0.8f );
		proxies[i] = b3DynamicTree_CreateProxy( &tree, boxes[i], B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
		alive[i] = true;
	}

	b3DynamicTree_Validate( &tree );
	ENSURE( b3DynamicTree_GetProxyCount( &tree ) == PROXY_COUNT );

	// The user data has to have followed each proxy through the churn
	for ( int i = 0; i < PROXY_COUNT; ++i )
	{
		ENSURE( alive[i] );
		ENSURE( (int)b3DynamicTree_GetUserData( &tree, proxies[i] ) == i );
		ENSURE( b3AABB_Contains( b3DynamicTree_GetAABB( &tree, proxies[i] ), boxes[i] ) );
	}

	b3AABB queryBox = { { -10.0f, -10.0f, -10.0f }, { 10.0f, 10.0f, 10.0f } };
	HitSet treeHits = { 0 };
	b3DynamicTree_Query( &tree, queryBox, B3_DEFAULT_MASK_BITS, false, CollectCallback, &treeHits );

	HitSet bruteHits = { 0 };
	for ( int i = 0; i < PROXY_COUNT; ++i )
	{
		if ( b3AABB_Overlaps( boxes[i], queryBox ) )
		{
			bruteHits.ids[bruteHits.count] = i;
			bruteHits.count += 1;
		}
	}

	ENSURE( SameSet( &treeHits, &bruteHits ) == 0 );
	ENSURE( treeHits.count == PROXY_COUNT );

	// Empty the tree
	for ( int i = 0; i < PROXY_COUNT; ++i )
	{
		b3DynamicTree_DestroyProxy( &tree, proxies[i] );
	}

	b3DynamicTree_Validate( &tree );
	ENSURE( b3DynamicTree_GetProxyCount( &tree ) == 0 );
	ENSURE( b3DynamicTree_GetHeight( &tree ) == 0 );

	// Queries on an empty tree must be silent
	HitSet emptyHits = { 0 };
	b3DynamicTree_Query( &tree, queryBox, B3_DEFAULT_MASK_BITS, false, CollectCallback, &emptyHits );
	ENSURE( emptyHits.count == 0 );

	b3DynamicTree_Destroy( &tree );
	return 0;
}

typedef struct LeafRootHit
{
	int count;
	int proxyId;
	uint64_t userData;
} LeafRootHit;

static bool LeafRootCallback( int proxyId, uint64_t userData, void* context )
{
	LeafRootHit* hit = context;
	hit->count += 1;
	hit->proxyId = proxyId;
	hit->userData = userData;
	return true;
}

// A one proxy tree is a leaf at the root with an empty node beside it, which every traversal
// has to survive.
static int TreeLeafRoot( void )
{
	b3DynamicTree tree = b3DynamicTree_Create( 16 );

	b3AABB box = { { -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f } };
	int proxyId = b3DynamicTree_CreateProxy( &tree, box, B3_DEFAULT_CATEGORY_BITS, 7 );

	b3DynamicTree_Validate( &tree );
	ENSURE( b3DynamicTree_GetProxyCount( &tree ) == 1 );
	ENSURE( b3DynamicTree_GetHeight( &tree ) == 0 );
	ENSURE( b3DynamicTree_GetAreaRatio( &tree ) == 0.0f );
	ENSURE( b3DynamicTree_GetUserData( &tree, proxyId ) == 7 );

	LeafRootHit hit = { 0 };
	b3AABB hitBox = { { 0.5f, 0.5f, 0.5f }, { 2.0f, 2.0f, 2.0f } };
	b3DynamicTree_Query( &tree, hitBox, B3_DEFAULT_MASK_BITS, false, LeafRootCallback, &hit );
	ENSURE( hit.count == 1 );
	ENSURE( hit.proxyId == proxyId );
	ENSURE( hit.userData == 7 );

	hit.count = 0;
	b3AABB missBox = { { 4.0f, 4.0f, 4.0f }, { 5.0f, 5.0f, 5.0f } };
	b3DynamicTree_Query( &tree, missBox, B3_DEFAULT_MASK_BITS, false, LeafRootCallback, &hit );
	ENSURE( hit.count == 0 );

	// The rebuild has one build leaf and the refit has nothing above it
	b3DynamicTree_Rebuild( &tree, true );
	b3DynamicTree_Validate( &tree );
	b3DynamicTree_Refit( &tree );
	b3DynamicTree_Validate( &tree );
	ENSURE( b3DynamicTree_GetProxyCount( &tree ) == 1 );

	b3AABB grown = b3AABB_Inflate( box, 1.0f );
	b3DynamicTree_EnlargeProxy( &tree, proxyId, grown );
	ENSURE( b3AABB_Contains( b3DynamicTree_GetAABB( &tree, proxyId ), grown ) );

	b3DynamicTree_Destroy( &tree );
	return 0;
}

// Category bits live on the proxy now, so the filter has to bite at the leaf.
static int TreeCategoryFilter( void )
{
	s_seed = 99;

	b3DynamicTree tree = b3DynamicTree_Create( 16 );

	b3AABB boxes[PROXY_COUNT];
	uint64_t bits[PROXY_COUNT];
	int proxies[PROXY_COUNT];

	for ( int i = 0; i < PROXY_COUNT; ++i )
	{
		boxes[i] = RandomBox( 8.0f, 0.6f );
		bits[i] = 1ull << ( i % 3 );
		proxies[i] = b3DynamicTree_CreateProxy( &tree, boxes[i], bits[i], (uint64_t)i );
		ENSURE( b3DynamicTree_GetCategoryBits( &tree, proxies[i] ) == bits[i] );
	}

	b3AABB queryBox = { { -8.0f, -8.0f, -8.0f }, { 8.0f, 8.0f, 8.0f } };

	// A single bit mask makes requireAll equivalent to the any-bit test, so the last pass is
	// the one that proves requireAll accepts rather than just rejects
	static const uint64_t masks[3] = { 0x1ull, 0x3ull, 0x2ull };
	static const bool requireAlls[3] = { false, false, true };

	for ( int pass = 0; pass < 3; ++pass )
	{
		uint64_t mask = masks[pass];
		bool requireAll = requireAlls[pass];

		HitSet treeHits = { 0 };
		b3DynamicTree_Query( &tree, queryBox, mask, requireAll, CollectCallback, &treeHits );

		HitSet bruteHits = { 0 };
		for ( int i = 0; i < PROXY_COUNT; ++i )
		{
			bool accept = requireAll ? ( bits[i] & mask ) == mask : ( bits[i] & mask ) != 0;
			if ( accept && b3AABB_Overlaps( boxes[i], queryBox ) )
			{
				bruteHits.ids[bruteHits.count] = i;
				bruteHits.count += 1;
			}
		}

		ENSURE( SameSet( &treeHits, &bruteHits ) == 0 );
		ENSURE( treeHits.count > PROXY_COUNT / 4 );
	}

	// requireAll with two bits can never match a single bit proxy
	HitSet noHits = { 0 };
	b3DynamicTree_Query( &tree, queryBox, 0x5ull, true, CollectCallback, &noHits );
	ENSURE( noHits.count == 0 );

	// Changing the bits has to change the filter without touching the structure
	b3DynamicTree_SetCategoryBits( &tree, proxies[0], 0x8ull );
	ENSURE( b3DynamicTree_GetCategoryBits( &tree, proxies[0] ) == 0x8ull );
	b3DynamicTree_Validate( &tree );

	HitSet oneHit = { 0 };
	b3DynamicTree_Query( &tree, queryBox, 0x8ull, false, CollectCallback, &oneHit );
	ENSURE( oneHit.count == 1 );
	ENSURE( oneHit.ids[0] == 0 );

	b3DynamicTree_Destroy( &tree );
	return 0;
}

// Returning -1 skips the proxy without shrinking the fraction, so the traversal keeps going
// and reports every proxy whose node box the segment crosses.
static float RayCastCollectCallback( const b3RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)input;
	(void)proxyId;
	HitSet* set = context;
	if ( set->count >= PROXY_COUNT || (int)userData < 0 || (int)userData >= PROXY_COUNT )
	{
		set->bad = true;
		return 0.0f;
	}

	set->ids[set->count] = (int)userData;
	set->count += 1;
	return -1.0f;
}

static float BoxCastCollectCallback( const b3BoxCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)input;
	(void)proxyId;
	HitSet* set = context;
	if ( set->count >= PROXY_COUNT || (int)userData < 0 || (int)userData >= PROXY_COUNT )
	{
		set->bad = true;
		return 0.0f;
	}

	set->ids[set->count] = (int)userData;
	set->count += 1;
	return -1.0f;
}

// The casts pop a pair, test both nodes and order the survivors. A ray that skips every hit
// must still reach every proxy its segment crosses.
static int TreeCastBruteForce( void )
{
	s_seed = 31337;

	b3DynamicTree tree = b3DynamicTree_Create( 16 );

	b3AABB boxes[PROXY_COUNT];
	int proxies[PROXY_COUNT];

	for ( int i = 0; i < PROXY_COUNT; ++i )
	{
		boxes[i] = RandomBox( 6.0f, 1.0f );
		proxies[i] = b3DynamicTree_CreateProxy( &tree, boxes[i], B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
	}

	b3DynamicTree_Rebuild( &tree, true );
	b3DynamicTree_Validate( &tree );

	int totalCrossings = 0;
	for ( int round = 0; round < 100; ++round )
	{
		b3Vec3 origin = { RandomFloat( -8.0f, 8.0f ), RandomFloat( -8.0f, 8.0f ), RandomFloat( -8.0f, 8.0f ) };
		b3Vec3 translation = { RandomFloat( -16.0f, 16.0f ), RandomFloat( -16.0f, 16.0f ), RandomFloat( -16.0f, 16.0f ) };

		b3RayCastInput rayInput = { .origin = origin, .translation = translation, .maxFraction = 1.0f };

		HitSet treeHits = { 0 };
		b3DynamicTree_RayCast( &tree, &rayInput, B3_DEFAULT_MASK_BITS, false, RayCastCollectCallback, &treeHits );
		ENSURE( treeHits.bad == false );

		// The tree may report a superset because it tests the node box, not the shape. Every
		// proxy the segment actually crosses has to be in there.
		b3Vec3 p2 = b3MulAdd( origin, 1.0f, translation );
		for ( int i = 0; i < PROXY_COUNT; ++i )
		{
			float minFraction = 0.0f;
			float maxFraction = 1.0f;
			if ( b3RayCastAABB( boxes[i], origin, p2, &minFraction, &maxFraction ) == false )
			{
				continue;
			}

			bool found = false;
			for ( int j = 0; j < treeHits.count; ++j )
			{
				if ( treeHits.ids[j] == i )
				{
					found = true;
					break;
				}
			}

			ENSURE( found );
			totalCrossings += 1;
		}

		// Every reported proxy has to at least overlap the segment box
		b3AABB segmentBox = { b3Min( origin, p2 ), b3Max( origin, p2 ) };
		for ( int j = 0; j < treeHits.count; ++j )
		{
			ENSURE( b3AABB_Overlaps( boxes[treeHits.ids[j]], segmentBox ) );
		}

		// A zero extent box cast has to reach the same proxies as the ray
		b3AABB castBox = { origin, origin };
		b3BoxCastInput boxInput = { .box = castBox, .translation = translation, .maxFraction = 1.0f };

		HitSet boxHits = { 0 };
		b3DynamicTree_BoxCast( &tree, &boxInput, B3_DEFAULT_MASK_BITS, false, BoxCastCollectCallback, &boxHits );
		ENSURE( boxHits.bad == false );

		for ( int j = 0; j < treeHits.count; ++j )
		{
			bool found = false;
			for ( int k = 0; k < boxHits.count; ++k )
			{
				if ( boxHits.ids[k] == treeHits.ids[j] )
				{
					found = true;
					break;
				}
			}

			ENSURE( found );
		}
	}

	// Guard against the comparisons above agreeing on nothing
	ENSURE( totalCrossings > 100 );

	b3DynamicTree_Destroy( &tree );
	return 0;
}

typedef struct ClosestResult
{
	const b3AABB* boxes;
	b3Vec3 point;
	bool bad;
} ClosestResult;

static float DistanceToBoxSqr( b3Vec3 point, b3AABB box )
{
	b3Vec3 r = b3Sub( point, b3Clamp( point, box.lowerBound, box.upperBound ) );
	return b3Dot( r, r );
}

static float ClosestCollectCallback( float distanceSqrMin, int proxyId, uint64_t userData, void* context )
{
	ClosestResult* result = context;
	if ( (int)userData != proxyId )
	{
		result->bad = true;
		return distanceSqrMin;
	}

	float d = DistanceToBoxSqr( result->point, result->boxes[proxyId] );
	return d < distanceSqrMin ? d : distanceSqrMin;
}

// The closest query orders by distance and prunes on the running minimum, so seeding it from
// the root pair has to be right or it prunes away a whole half of the tree.
static int TreeQueryClosest( void )
{
	s_seed = 5150;

	b3DynamicTree tree = b3DynamicTree_Create( 16 );

	b3AABB boxes[PROXY_COUNT];
	for ( int i = 0; i < PROXY_COUNT; ++i )
	{
		boxes[i] = RandomBox( 6.0f, 0.9f );
		int proxyId = b3DynamicTree_CreateProxy( &tree, boxes[i], B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
		ENSURE( proxyId == i );
	}

	for ( int round = 0; round < 100; ++round )
	{
		b3Vec3 point = { RandomFloat( -8.0f, 8.0f ), RandomFloat( -8.0f, 8.0f ), RandomFloat( -8.0f, 8.0f ) };

		float bruteBest = FLT_MAX;
		for ( int i = 0; i < PROXY_COUNT; ++i )
		{
			float d = DistanceToBoxSqr( point, boxes[i] );
			if ( d < bruteBest )
			{
				bruteBest = d;
			}
		}

		// The callback reports the exact box distance, so the pruned search has to land on the
		// same minimum as the scan. Same arithmetic both ways, so this is an exact compare.
		ClosestResult result = { .boxes = boxes, .point = point };
		float minDistanceSqr = FLT_MAX;
		b3DynamicTree_QueryClosest( &tree, point, B3_DEFAULT_MASK_BITS, false, ClosestCollectCallback, &result,
									&minDistanceSqr );

		ENSURE( result.bad == false );
		ENSURE( minDistanceSqr == bruteBest );
	}

	b3DynamicTree_Destroy( &tree );
	return 0;
}

// A saved tree carries only its live node range. A baked compound tree has no parent array, so
// saving one has to bail out rather than write from a null pointer.
static int TreeSaveLoadRoundtrip( void )
{
	s_seed = 4242;

	b3DynamicTree tree = b3DynamicTree_Create( 16 );

	b3AABB boxes[PROXY_COUNT];
	int proxies[PROXY_COUNT];

	for ( int i = 0; i < PROXY_COUNT; ++i )
	{
		boxes[i] = RandomBox( 6.0f, 1.0f );
		proxies[i] = b3DynamicTree_CreateProxy( &tree, boxes[i], B3_DEFAULT_CATEGORY_BITS, (uint64_t)i );
	}

	// Destroy a few so the pair free list is populated and the node tail is stale
	for ( int i = 0; i < PROXY_COUNT; i += 37 )
	{
		b3DynamicTree_DestroyProxy( &tree, proxies[i] );
	}

	b3DynamicTree_Rebuild( &tree, true );
	b3DynamicTree_Validate( &tree );

	const char* treePath = "test_dynamic_tree_roundtrip.dat";
	b3DynamicTree_Save( &tree, treePath );

	b3DynamicTree loaded = b3DynamicTree_Load( treePath, 1.0f );
	remove( treePath );

	ENSURE( loaded.nodes != NULL );
	ENSURE( loaded.nodeEnd == tree.nodeEnd );
	ENSURE( loaded.nodeCapacity == tree.nodeEnd );
	ENSURE( b3DynamicTree_GetProxyCount( &loaded ) == b3DynamicTree_GetProxyCount( &tree ) );
	ENSURE( memcmp( loaded.nodes, tree.nodes, (size_t)tree.nodeEnd * sizeof( b3TreeNode ) ) == 0 );
	ENSURE( memcmp( loaded.parents, tree.parents, (size_t)tree.nodeEnd * sizeof( int32_t ) ) == 0 );
	b3DynamicTree_Validate( &loaded );

	b3DynamicTree_Destroy( &loaded );
	b3DynamicTree_Destroy( &tree );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	b3CompoundHullDef hulls[2];
	hulls[0].hull = &box.base;
	hulls[0].transform = (b3Transform){ { -1.0f, 0.0f, 0.0f }, b3Quat_identity };
	hulls[0].material = b3DefaultSurfaceMaterial();
	hulls[1].hull = &box.base;
	hulls[1].transform = (b3Transform){ { 1.0f, 0.0f, 0.0f }, b3Quat_identity };
	hulls[1].material = b3DefaultSurfaceMaterial();

	b3CompoundDef compoundDef = { 0 };
	compoundDef.hulls = hulls;
	compoundDef.hullCount = 2;
	b3CompoundData* compound = b3CreateCompound( &compoundDef );
	ENSURE( compound != NULL );
	ENSURE( compound->tree.parents == NULL );
	ENSURE( compound->tree.proxyCapacity == compound->tree.proxyCount );

	const char* compoundPath = "test_dynamic_tree_compound.dat";
	remove( compoundPath );
	b3DynamicTree_Save( &compound->tree, compoundPath );

	b3DynamicTree noTree = b3DynamicTree_Load( compoundPath, 1.0f );
	remove( compoundPath );
	ENSURE( noTree.nodes == NULL );

	b3DestroyCompound( compound );
	return 0;
}

int DynamicTreeTest( void )
{
	RUN_SUBTEST( TreeLeafRoot );
	RUN_SUBTEST( TreeQueryBruteForce );
	RUN_SUBTEST( TreeCategoryFilter );
	RUN_SUBTEST( TreeMoveRefitRebuild );
	RUN_SUBTEST( TreeDestroyScrambled );
	RUN_SUBTEST( TreeCastBruteForce );
	RUN_SUBTEST( TreeQueryClosest );
	RUN_SUBTEST( TreeSaveLoadRoundtrip );

	return 0;
}
