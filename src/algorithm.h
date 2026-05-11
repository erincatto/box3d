// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stddef.h>

#define B3_TYPE_OF( A ) __typeof__( A )

#define B3_SWAP( x, y )                                                                                                          \
	do                                                                                                                           \
	{                                                                                                                            \
		B3_TYPE_OF( x ) B3_SWAP_TEMP = x;                                                                                        \
		x = y;                                                                                                                   \
		y = B3_SWAP_TEMP;                                                                                                        \
	}                                                                                                                            \
	while ( 0 )
