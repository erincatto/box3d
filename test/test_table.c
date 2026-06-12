// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"
#include "ctz.h"
#include "platform.h"
#include "table.h"
#include "test_macros.h"

#include "box3d/base.h"

#define SET_SPAN 317
#define ITEM_COUNT ( ( SET_SPAN * SET_SPAN - SET_SPAN ) / 2 )

int TableTest( void )
{
	int power = b3BoundingPowerOf2( 3008 );
	ENSURE( power == 12 );

	int nextPowerOf2 = b3RoundUpPowerOf2( 3008 );
	ENSURE( nextPowerOf2 == ( 1 << power ) );

	const int32_t N = SET_SPAN;
	const uint32_t itemCount = ITEM_COUNT;
	bool removed[ITEM_COUNT] = { 0 };

	for ( int32_t iter = 0; iter < 1; ++iter )
	{
		b3HashSet set = b3CreateSet( 16 );

		// Fill set
		for ( int32_t i = 0; i < N; ++i )
		{
			for ( int32_t j = i + 1; j < N; ++j )
			{
				uint64_t key = b3ShapePairKey( i, j, 0 );
				b3AddKey( &set, key );
			}
		}

		ENSURE( set.count == itemCount );

		// Remove a portion of the set
		int32_t k = 0;
		uint32_t removeCount = 0;
		for ( int32_t i = 0; i < N; ++i )
		{
			for ( int32_t j = i + 1; j < N; ++j )
			{
				if ( j == i + 1 )
				{
					uint64_t key = b3ShapePairKey( i, j, 0 );
					b3RemoveKey( &set, key );
					removed[k++] = true;
					removeCount += 1;
				}
				else
				{
					removed[k++] = false;
				}
			}
		}

		ENSURE( set.count == ( itemCount - removeCount ) );

#ifndef NDEBUG
		extern b3AtomicInt b3_probeCount;
		b3AtomicStoreInt(&b3_probeCount, 0);
#endif

		// Test key search
		// ~5ns per search on an AMD 7950x
		uint64_t ticks = b3GetTicks();

		k = 0;
		for ( int32_t i = 0; i < N; ++i )
		{
			for ( int32_t j = i + 1; j < N; ++j )
			{
				uint64_t key = b3ShapePairKey( j, i, 0 );
				ENSURE( b3ContainsKey( &set, key ) || removed[k] );
				k += 1;
			}
		}

		float ms = b3GetMilliseconds( ticks );
		printf( "set: count = %d, b3ContainsKey = %.5f ms, ave = %.5f us\n", itemCount, ms, 1000.0f * ms / itemCount );

#ifndef NDEBUG
		int probeCount = b3AtomicLoadInt( &b3_probeCount );
		float aveProbeCount = (float)probeCount / (float)itemCount;
		printf( "item count = %d, probe count = %d, ave probe count %.2f\n", itemCount, probeCount, aveProbeCount );
#endif

		// Remove all keys from set
		for ( int32_t i = 0; i < N; ++i )
		{
			for ( int32_t j = i + 1; j < N; ++j )
			{
				uint64_t key = b3ShapePairKey( i, j, 0 );
				b3RemoveKey( &set, key );
			}
		}

		ENSURE( set.count == 0 );

		b3DestroySet( &set );
	}

	return 0;
}
