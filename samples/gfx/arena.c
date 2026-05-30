// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/arena.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

int ArenaInit( Arena* a, size_t capacity )
{
	assert( a );
	assert( capacity > 0 );
	a->base = (uint8_t*)malloc( capacity );
	if ( !a->base )
	{
		a->capacity = 0;
		a->used = 0;
		return -1;
	}
	a->capacity = capacity;
	a->used = 0;
	return 0;
}

void ArenaShutdown( Arena* a )
{
	if ( !a )
	{
		return;
	}
	free( a->base );
	a->base = NULL;
	a->capacity = 0;
	a->used = 0;
}

void* ArenaAlloc( Arena* a, size_t size, size_t align )
{
	assert( a );
	assert( align > 0 && ( align & ( align - 1 ) ) == 0 ); // power of two
	// Align the absolute address, malloc's base is only guaranteed to be
	// alignof(max_align_t)-aligned (16 on x64 MSVC), so aligning the offset
	// alone breaks for align > 16.
	const uintptr_t mask = (uintptr_t)align - 1;
	const uintptr_t cur = (uintptr_t)a->base + a->used;
	const uintptr_t aligned = ( cur + mask ) & ~mask;
	const size_t alignedUsed = (size_t)( aligned - (uintptr_t)a->base );
	if ( alignedUsed > a->capacity || size > a->capacity - alignedUsed )
	{
		fprintf( stderr, "[arena/error] out of space: used=%zu cap=%zu req=%zu align=%zu\n", a->used, a->capacity, size, align );
		return NULL;
	}
	a->used = alignedUsed + size;
	return (void*)aligned;
}

void ArenaReset( Arena* a )
{
	assert( a );
	a->used = 0;
}
