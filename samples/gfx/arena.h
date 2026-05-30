// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct Arena
{
	uint8_t* base; // owned heap allocation, capacity bytes
	size_t capacity;
	size_t used;
} Arena;

// Returns 0 on success, -1 on allocation failure. capacity must be > 0.
int ArenaInit( Arena* a, size_t capacity );

// Free backing storage. Safe on a zero-initialized arena.
void ArenaShutdown( Arena* a );

// Bump-allocate `size` bytes with `align` alignment (must be power-of-two).
// Returns NULL if the request would overflow the arena, logs to stderr.
void* ArenaAlloc( Arena* a, size_t size, size_t align );

// Reset to empty without freeing backing storage. Invalidates all
// pointers handed out before the reset.
void ArenaReset( Arena* a );

#ifdef __cplusplus
} // extern "C"
#endif
