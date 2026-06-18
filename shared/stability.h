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

typedef struct MeshDropData
{
	struct b3MeshData* mesh;
} MeshDropData;

MeshDropData CreateMeshDrop( b3WorldId worldId, b3Pos origin );
void DestroyMeshDrop( MeshDropData* data );

#ifdef __cplusplus
}
#endif
