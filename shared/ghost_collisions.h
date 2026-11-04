// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "box3d/collision.h"
#include "box3d/id.h"
#include "box3d/math_functions.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Ghost collision scenes, shared by the Mesh / Ghost Collisions sample and the headless ghost scan.
// Each ground is a profile in the xy-plane extruded along z and finely tessellated, so a sliding
// body crosses many internal edges. A manifold that pushes on the body is a ghost when its normal
// points outside the span of face normals under the body. Those are the ones that block or launch
// a body.

typedef enum GhostScene
{
	e_ghostFlat,
	e_ghostConcave,
	e_ghostCliff,
	e_ghostCrest,
	e_ghostBumps,
	e_ghostTJunction,
	e_ghostInternal,
	e_ghostChamfer,
	e_ghostRail,
	e_ghostSceneCount
} GhostScene;

typedef enum GhostBody
{
	e_ghostBox,
	e_ghostHCapsule,
	e_ghostVCapsule,
	e_ghostSphere,
	e_ghostCylinder,
	e_ghostBodyCount
} GhostBody;

// Mesh variants of the PEEL InternalEdges presets
#define GHOST_PEEL_PRESET_COUNT 4

// Chamfer has the longest profile
#define GHOST_MAX_PROFILE_POINTS 128

extern const char* g_ghostSceneNames[e_ghostSceneCount];
extern const char* g_ghostBodyNames[e_ghostBodyCount];
extern const char* g_ghostPeelPresetNames[GHOST_PEEL_PRESET_COUNT];

typedef struct GhostSettings
{
	GhostScene scene;
	GhostBody body;
	int peelPreset;
	bool mirror;

	// Build the ground from height fields instead of meshes. Each height field is its own shape, so the
	// T-junction scene becomes a seam between a fine and a coarse tile.
	bool heightField;

	float cellSize;
	float speed;

	// Degrees
	float yaw;

	float friction;
	bool fixedRotation;
	bool speculative;
} GhostSettings;

GhostSettings GetDefaultGhostSettings( void );

typedef struct GhostReport
{
	b3Pos point;
	b3Vec3 normal;
	float separation;
	float impulse;
} GhostReport;

typedef struct GhostCollisionsData
{
	GhostSettings settings;
	b3Vec2 profile[GHOST_MAX_PROFILE_POINTS];
	int profileCount;

	// The ground spans z in [-halfWidth, halfWidth]
	float halfWidth;
	b3MeshData* meshes[2];
	int meshCount;

	// A height field shape has no local transform, so each one gets its own static body
	b3HeightFieldData* heightFields[2];
	b3BodyId heightBodyIds[2];
	int heightFieldCount;
	b3HullData* cylinder;
	b3HullData* peelCylinder;
	b3BodyId groundId;
	b3BodyId bodyId;
	b3ShapeId shapeId;

	// Body bounds at the start of the step, where the contacts were computed. A fast body can cross a fold
	// within a step, so the bounds after the step can miss the face a contact came from.
	b3AABB contactBounds;

	// Steps in the current run
	int runSteps;

	// Completed runs, each ending when the body reaches the end or falls off
	int runCount;

	int runGhostCount;
	int ghostCount;
} GhostCollisionsData;

// Creates the ground and launches the body
GhostCollisionsData CreateGhostCollisions( b3WorldId worldId, GhostSettings settings );

// Rebuild the ground after a scene, preset, mirror, height field or cell size change. Relaunch after this.
void CreateGhostGround( b3WorldId worldId, GhostCollisionsData* data );

// Launch a new body from the start, after a body, speed, yaw, friction or rotation change
void LaunchGhostBody( b3WorldId worldId, GhostCollisionsData* data );

// Call after each world step. Relaunches a body that finished its run, then counts the ghost
// manifolds on the body. Returns the ghost count for this step and writes up to capacity reports.
int UpdateGhostCollisions( b3WorldId worldId, GhostCollisionsData* data, GhostReport* reports, int capacity );

void DestroyGhostCollisions( GhostCollisionsData* data );

#ifdef __cplusplus
}
#endif
