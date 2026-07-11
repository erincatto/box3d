// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "arena_allocator.h"
#include "container.h"

#include "box3d/collision.h"
#include "box3d/types.h"

#define B3_FORCE_GHOST_COLLISIONS 0

typedef struct b3Shape b3Shape;
typedef struct b3World b3World;

typedef union b3ContactCache
{
	b3SATCache satCache;
	b3SimplexCache simplexCache;
} b3ContactCache;

typedef struct b3TriangleCache
{
	int triangleIndex;
	b3ContactCache cache;
} b3TriangleCache;

b3DeclareArray( b3TriangleCache );

enum b3ContactFlags
{
	// Set when the solid shapes are touching.
	b3_contactTouchingFlag = 0x00000001,

	// Contact has a hit event
	b3_contactHitEventFlag = 0x00000002,

	// This contact wants contact events
	b3_contactEnableContactEvents = 0x00000004,

	// This is contact is between a dynamic and static body
	b3_contactStaticFlag = 0x00000008,

	b3_contactRecycleFlag = 0x00000010,

	// Set when the shapes are touching
	b3_simTouchingFlag = 0x00010000,

	// This contact no longer has overlapping AABBs
	b3_simDisjoint = 0x00020000,

	// This contact started touching
	b3_simStartedTouching = 0x00040000,

	// This contact stopped touching
	b3_simStoppedTouching = 0x00080000,

	// This contact has a hit event
	b3_simEnableHitEvent = 0x00100000,

	// This contact wants pre-solve events
	b3_simEnablePreSolveEvents = 0x00200000,

	// This is a mesh contact
	b3_simMeshContact = 0x00400000,

	// Relative transform is cached for contact recycling
	b3_relativeTransformValid = 0x00800000,

	// Enable speculative contact points
	b3_enableSpeculativePoints = 0x01000000,

	// Contact lives on the graph color's scalar list (multi manifold path)
	b3_contactScalarPlacement = 0x02000000,

	// Manifold count no longer fits the wide path, move to the scalar list
	b3_simGraphMove = 0x04000000,
};

// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// edges, one for each attached body.
typedef struct b3ContactEdge
{
	int bodyId;
	int prevKey;
	int nextKey;
} b3ContactEdge;

typedef struct b3MeshContact
{
	b3Array( b3TriangleCache ) triangleCache;
	b3AABB queryBounds;
} b3MeshContact;

typedef struct b3ConvexContact
{
	b3ContactCache cache;
} b3ConvexContact;

// One narrowphase shape pair within a body-pair contact
typedef struct b3SubContact
{
	int shapeIdA;
	int shapeIdB;
	int childIndex;

	bool touching;
	bool reportedTouching;
	bool isMesh;

	// Usage determined by b3_simMeshContact style dispatch on the shapes
	union
	{
		b3ConvexContact convexContact;
		b3MeshContact meshContact;
	};
} b3SubContact;

// Represents the persistent interaction between two shapes
typedef struct b3Contact
{
	// index of simulation set stored in b3World
	// B3_NULL_INDEX when slot is free
	int setIndex;

	// index into the constraint graph color array
	// B3_NULL_INDEX for non-touching or sleeping contacts
	// B3_NULL_INDEX when slot is free
	int colorIndex;

	// contact index within set or graph color
	// B3_NULL_INDEX when slot is free
	int localIndex;

	b3ContactEdge edges[2];

	// A contact only belongs to an island if touching, otherwise B3_NULL_INDEX.
	int islandId;

	// Index into the island's contacts array for O(1) swap-removal.
	// B3_NULL_INDEX when not in an island.
	int islandIndex;

	// Back index into b3World::contacts
	int contactId;

	// These are transient and cached for improved performance. B3_NULL_INDEX for static bodies.
	int bodySimIndexA;
	int bodySimIndexB;

	// b3ContactFlags
	uint32_t flags;

	b3Manifold* manifolds;
	int manifoldCount;

	// Cache for contact recycling.
	b3Quat cachedRotationA;
	b3Quat cachedRotationB;
	b3Transform cachedRelativePose;

	// Mixed friction and restitution
	float friction;

	float restitution;
	float rollingResistance;
	b3Vec3 tangentVelocity;

	// This is monotonically advanced when a contact is allocated in this slot
	// Used to check for invalid b3ContactId
	uint32_t generation;

	uint16_t subCount;
	uint16_t extraCapacity;
	b3SubContact* extraSubs;
	b3SubContact sub0;
} b3Contact;

typedef struct b3ContactSpec
{
	int contactId;

	// Start of the global manifold constraint array
	int manifoldStart;
	uint16_t manifoldCount;
} b3ContactSpec;

b3DeclareArray( b3ContactSpec );

static inline b3SubContact* b3GetSubContact( b3Contact* contact, int index )
{
	return index == 0 ? &contact->sub0 : contact->extraSubs + ( index - 1 );
}

void b3InitializeContactRegisters( void );

void b3CreateContact( b3World* world, b3Shape* shapeA, b3Shape* shapeB, int childIndex );
void b3DestroyContact( b3World* world, b3Contact* contact, bool wakeBodies );
void b3RemoveSubContact( b3World* world, b3Contact* contact, int subIndex, bool wakeBodies );
void b3SyncSubTouchEvents( b3World* world, b3Contact* contact );

bool b3UpdateContact( b3World* world, int workerIndex, b3Contact* contact, b3Shape* shapeA, b3Vec3 localCenterA, b3WorldTransform xfA,
					  b3Shape* shapeB, b3Vec3 localCenterB, b3WorldTransform xfB, bool isFast, b3Arena arena );

bool b3ComputeMeshManifolds( b3World* world, int workerIndex, b3Contact* contact, const b3Shape* shapeA, const int* materialMap,
							 b3WorldTransform xfA, const b3Shape* shapeB, b3WorldTransform xfB, bool isFast, b3Arena arena );

bool b3ComputeMultiSubManifolds( b3World* world, int workerIndex, b3Contact* contact, b3WorldTransform xfA, b3WorldTransform xfB,
								 bool isFast, b3Arena arena );
