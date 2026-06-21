// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stddef.h>
#include <string.h>

#if 0
// __typeof__ is a GCC/Clang extension. MSVC only gained it in VS 17.9 (toolset 14.39) and
// spells it typeof there only under /std:clatest, so older toolchains have nothing. The sole
// use is casting the allocator's void* back to the element pointer type, which C does
// implicitly, so void* is a sufficient stand in.
#if defined( __clang__ ) || defined( __GNUC__ )
#define B3_TYPE_OF( A ) __typeof__( A )
#else
#define B3_TYPE_OF( A ) void*
#endif
#endif

// Swap two same-size lvalues through a byte buffer. Avoids __typeof__ so it builds on any C compiler.
#define B3_SWAP( x, y )                                                                                                          \
	do                                                                                                                           \
	{                                                                                                                            \
		char B3_SWAP_TEMP[sizeof( x )];                                                                                          \
		memcpy( B3_SWAP_TEMP, &( x ), sizeof( x ) );                                                                             \
		memcpy( &( x ), &( y ), sizeof( x ) );                                                                                   \
		memcpy( &( y ), B3_SWAP_TEMP, sizeof( x ) );                                                                             \
	}                                                                                                                            \
	while ( 0 )
