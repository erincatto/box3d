// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "dynamic_tree.h"
#include "table.h"

#include "box3d/collision.h"
#include "box3d/types.h"

typedef struct b3Shape b3Shape;
typedef struct b3Stack b3Stack;
typedef struct b3World b3World;

// Store the proxy type in the lower 2 bits of the proxy key. This leaves 30 bits for the id.
#define B3_PROXY_TYPE( KEY ) ( (b3BodyType)( ( KEY ) & 3 ) )
#define B3_PROXY_ID( KEY ) ( ( KEY ) >> 2 )
#define B3_PROXY_KEY( ID, TYPE ) ( ( ( ID ) << 2 ) | ( TYPE ) )

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
typedef struct b3BroadPhase
{
	// One tree for each body type.
	b3DynamicTree trees[b3_bodyTypeCount];

	// The moved siblings gathered from the dynamic body tree.
	int* movedSiblings;

	// Tracks shape pairs that have a b3Contact
	// todo pairSet can grow quite large on the first time step and remain large
	b3HashSet pairSet;
} b3BroadPhase;

void b3CreateBroadPhase( b3BroadPhase* bp, const b3Capacity* capacity );
void b3DestroyBroadPhase( b3BroadPhase* bp );

int b3BroadPhase_CreateProxy( b3BroadPhase* bp, b3BodyType proxyType, b3AABB aabb, uint64_t categoryBits, int shapeIndex,
							  bool forcePairCreation );
void b3BroadPhase_DestroyProxy( b3BroadPhase* bp, int proxyKey );

void b3BroadPhase_MoveProxy( b3BroadPhase* bp, int proxyKey, b3AABB aabb );

int b3BroadPhase_GetShapeIndex( b3BroadPhase* bp, int proxyKey );

void b3UpdateBroadPhasePairs( b3World* world );
bool b3BroadPhase_TestOverlap( const b3BroadPhase* bp, int proxyKeyA, int proxyKeyB );

void b3ValidateBroadPhase( const b3BroadPhase* bp );
void b3ValidateNoMoved( const b3BroadPhase* bp );

static inline void b3BroadPhase_MarkProxyMovedSerial( b3BroadPhase* bp, int proxyKey )
{
	b3BodyType proxyType = B3_PROXY_TYPE( proxyKey );
	int proxyId = B3_PROXY_ID( proxyKey );
	b3DynamicTree_MarkProxyMovedSerial( bp->trees + proxyType, proxyId );
}

static inline void b3BroadPhase_MarkProxyMoved( b3BroadPhase* bp, int proxyKey, b3AABB aabb )
{
	b3BodyType proxyType = B3_PROXY_TYPE( proxyKey );
	int proxyId = B3_PROXY_ID( proxyKey );
	b3DynamicTree_MarkProxyMoved( bp->trees + proxyType, proxyId, aabb );
}
