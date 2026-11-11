// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "box3d/collision.h"
#include "box3d/id.h"
#include "box3d/math_functions.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Mesh Drop scene, shared by the Continuous / Mesh Drop sample and the headless mesh soak. A generation
// drops a grid of small bodies with random velocities on a wave mesh and steps until they sleep. A body
// that falls out of the world tunneled. The depth tracker measures how far each body sinks into the
// wave, which flags contact trouble far more often than tunnels do.

typedef enum MeshDropShape
{
	e_meshDropBox,
	e_meshDropCapsule,
	e_meshDropCylinder,
	e_meshDropSphere,
	e_meshDropShapeCount
} MeshDropShape;

#define MESH_DROP_GRID_COUNT 32
#define MESH_DROP_BODY_COUNT ( MESH_DROP_GRID_COUNT * MESH_DROP_GRID_COUNT )
#define MESH_DROP_DEPTH_LEVEL_COUNT 4

// Wave mesh cells along x and z
#define MESH_DROP_WAVE_COUNT 40

// Enough for the cylinder hull
#define MESH_DROP_MAX_PROBES 16

extern const char* g_meshDropShapeNames[e_meshDropShapeCount];

// Sink depth thresholds in linear slop
extern const float g_meshDropDepthLevels[MESH_DROP_DEPTH_LEVEL_COUNT];

typedef struct MeshDropSettings
{
	MeshDropShape shape;
	float amplitude;

	// Let the bodies collide with each other. Off isolates mesh collision.
	bool collide;

	// Create only this grid index, or -1 for all. Without body collision a body moves the same alone.
	int soloIndex;
} MeshDropSettings;

MeshDropSettings GetDefaultMeshDropSettings( void );

// Hashes the base before adding the generation, so nearby base seeds share no generations
uint32_t GetMeshDropSeed( uint32_t baseSeed, int generation );

// Steps a generation may take to fall asleep
int GetMeshDropStepLimit( MeshDropShape shape );

// A body below this height tunneled
float GetMeshDropTunnelHeight( void );

b3MeshData* CreateMeshDropMesh( float amplitude );
b3HullData* CreateMeshDropCylinder( void );

// Creates a static body with the wave mesh and four walls
b3BodyId CreateMeshDropGround( b3WorldId worldId, const b3MeshData* mesh );

// Creates the bodies of one generation in grid order. Bodies left out by the solo index are null. The
// body user data encodes the grid index, see GetMeshDropBodyIndex.
void CreateMeshDropBodies( b3WorldId worldId, MeshDropSettings settings, uint32_t seed, const b3HullData* cylinder,
						   b3BodyId* bodies );

// Grid index from body user data, or -1 if the body is not part of the drop
int GetMeshDropBodyIndex( void* userData );

typedef struct MeshDropDepth
{
	// Wave heights at the mesh vertices
	float heights[( MESH_DROP_WAVE_COUNT + 1 ) * ( MESH_DROP_WAVE_COUNT + 1 )];

	// Collision points in the body frame, rounded by the radius
	b3Vec3 probes[MESH_DROP_MAX_PROBES];
	int probeCount;
	float radius;
	float boundingRadius;
	float amplitude;

	// Deepest sink of each body so far, positive when inside the wave
	float bodyDepths[MESH_DROP_BODY_COUNT];

	float maxDepth;
	int maxIndex;
	int maxStep;
	int stepCount;
} MeshDropDepth;

// Starts tracking a new generation
void ResetMeshDropDepth( MeshDropDepth* depth, MeshDropSettings settings, const b3HullData* cylinder );

// Call after each world step
void UpdateMeshDropDepth( b3WorldId worldId, MeshDropDepth* depth );

// Number of bodies that sank deeper than each depth level
void GetMeshDropDepthCounts( const MeshDropDepth* depth, int counts[MESH_DROP_DEPTH_LEVEL_COUNT] );

#ifdef __cplusplus
}
#endif
