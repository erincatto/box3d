// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "box3d/id.h"
#include "box3d/math_functions.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MESH_DROP_GRID_COUNT 20

// Thin fast boxes dropped on a wave mesh. Stresses continuous collision and mesh contact
// stability, and doubles as a determinism scenario via the sleep hash.
typedef struct MeshDropData
{
	struct b3MeshData* mesh;
	b3BodyId bodies[MESH_DROP_GRID_COUNT * MESH_DROP_GRID_COUNT];
	int stepCount;
	int sleepStep;
	uint32_t hash;
} MeshDropData;

MeshDropData CreateMeshDrop( b3WorldId worldId, b3Pos origin );
bool UpdateMeshDrop( b3WorldId worldId, MeshDropData* data );
void DestroyMeshDrop( MeshDropData* data );

#ifdef __cplusplus
}
#endif
