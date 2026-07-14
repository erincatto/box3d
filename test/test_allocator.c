// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "block_allocator.h"
#include "core.h"
#include "test_macros.h"

#include "box3d/types.h"

typedef struct
{
	int value1;
	float value2;
} Foo;

_Static_assert( sizeof( Foo ) >= sizeof( void* ), "too small" );

static int TestBlockAllocate( void )
{
	b3BlockAllocator allocator = b3CreateBlockAllocator( (int)sizeof( Foo ), 2 );

	Foo* item1 = b3AllocateElement( &allocator );
	ENSURE( item1 != NULL );

	Foo* item2 = b3AllocateElement( &allocator );
	ENSURE( item2 != NULL );

	b3DestroyBlockAllocator( &allocator );
	return 0;
}

static int TestBlockClear( void )
{
	b3BlockAllocator allocator = b3CreateBlockAllocator( (int)sizeof( Foo ), 0 );

	Foo* item1 = b3AllocateElement( &allocator );
	Foo* item2 = b3AllocateElement( &allocator );

	b3FreeElement( &allocator, item1 );
	b3FreeElement( &allocator, item2 );

	b3DestroyBlockAllocator( &allocator );
	return 0;
}

// The free list link is a pointer stored in the element, so a stride that is not a multiple
// of the alignment leaves odd elements on a bad boundary. See issue #78.
static int TestBlockAlignment( void )
{
	// The manifold sizes are the reported case
	int elementSizes[] = { 12, (int)sizeof( b3Manifold ), 3 * (int)sizeof( b3Manifold ) };

	for ( int i = 0; i < ARRAY_COUNT( elementSizes ); ++i )
	{
		b3BlockAllocator allocator = b3CreateBlockAllocator( elementSizes[i], 0 );

		// Hold the elements so the indices advance and a block boundary is crossed
		void* elements[B3_BLOCK_SIZE + 8];
		int count = ARRAY_COUNT( elements );

		for ( int j = 0; j < count; ++j )
		{
			elements[j] = b3AllocateElement( &allocator );
			ENSURE( elements[j] != NULL );
			ENSURE( ( (uintptr_t)elements[j] & ( B3_ALIGNMENT - 1 ) ) == 0 );
		}

		for ( int j = 0; j < count; ++j )
		{
			b3FreeElement( &allocator, elements[j] );
		}

		// Now drain the free list
		for ( int j = 0; j < count; ++j )
		{
			elements[j] = b3AllocateElement( &allocator );
			ENSURE( elements[j] != NULL );
			ENSURE( ( (uintptr_t)elements[j] & ( B3_ALIGNMENT - 1 ) ) == 0 );
		}

		for ( int j = 0; j < count; ++j )
		{
			b3FreeElement( &allocator, elements[j] );
		}

		b3DestroyBlockAllocator( &allocator );
	}

	return 0;
}

int AllocatorTest( void )
{
	RUN_SUBTEST( TestBlockAllocate );
	RUN_SUBTEST( TestBlockClear );
	RUN_SUBTEST( TestBlockAlignment );

	return 0;
}
