// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/math_functions.h"

#include <vector>

struct b3MeshData;

// Returns null if the mesh cannot be created.
b3MeshData* CreateMeshData( const char* path, float scale, bool zUp, bool useMedianSplit, bool identifyConvexEdges,
							bool weldVertices );

// Null tolerant.
void DestroyMeshData( b3MeshData* meshData );

struct TempMesh
{
	std::vector<b3Vec3> vertices;
	std::vector<int> indices;
	std::vector<uint8_t> materialIndices;
};

bool LoadTempMesh( const char* path, TempMesh* tempMesh, float scale, bool zUp );
