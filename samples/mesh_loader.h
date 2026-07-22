// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/math_functions.h"

#include <vector>

struct b3MeshData;

// Both return failure when the file is missing so a sample can build without its
// geometry instead of taking the app down.
b3MeshData* CreateMeshData( const char* path, float scale, bool zUp, bool useMedianSplit, bool identifyConvexEdges,
							bool weldVertices );

// Null tolerant, so a sample whose geometry failed to load still tears down cleanly.
void DestroyMeshData( b3MeshData* meshData );

struct TempMesh
{
	std::vector<b3Vec3> vertices;
	std::vector<int> indices;
	std::vector<uint8_t> materialIndices;
};

bool LoadTempMesh( const char* path, TempMesh* tempMesh, float scale, bool zUp );
