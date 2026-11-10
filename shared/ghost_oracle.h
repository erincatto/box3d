// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Separated-pose oracle for mesh ghost handling. With the hull clear of the mesh the union of the
// triangles is the reference surface, and none of the Voronoi logic under test is needed to judge a
// contact against it.
//
// A normal is valid at a mesh location when the union has it there: the face normal of a triangle
// containing the location, or a proximal normal, meaning the location is the closest union point to a
// point a little way out along the normal. That second test gives the convex edge wedge and the vertex
// cone exactly, and nothing at a concave crease beyond the two face normals. A manifold shares one normal
// across its points, so it is a ghost when the normal is valid at none of its points within linear slop of
// its deepest.
//
// The contacts the union itself has are the local minima of the distance between the hull and the
// union. A triangle's closest pair (GJK) is one unless a triangle that also contains its mesh point is
// closer to the hull. The distance to a triangle is convex over the triangle, so that test is exact. The
// union closest pair must be represented by a collided point, otherwise the contact was missed.

typedef enum OracleScene
{
	e_oracleFlat,
	e_oracleFan,
	e_oracleCrest,
	e_oracleValley,
	e_oracleRidge,
	e_oracleSaddleRidge,
	e_oracleRail,
	e_oracleWave,
	e_oracleSceneCount
} OracleScene;

extern const char* g_oracleSceneNames[e_oracleSceneCount];

#define ORACLE_MAX_TRIANGLES 128
#define ORACLE_MAX_POINTS 128

typedef enum OracleVerdict
{
	// Face normal of a triangle containing the point
	e_oracleFace,

	// Proximal normal at an edge or vertex
	e_oracleFeature,

	// Not valid here, but valid at a deep point of the same manifold
	e_oracleShared,

	// Point of a manifold whose normal is valid at none of its deep points
	e_oracleGhost,
} OracleVerdict;

typedef struct OraclePair
{
	int triangleIndex;
	b3Vec3 vertices[3];
	b3Vec3 meshPoint;
	b3Vec3 hullPoint;
	b3Vec3 normal;
	float distance;
	bool owned;

	// Triangle index of a closer triangle containing the mesh point, or -1
	int dominatedBy;
} OraclePair;

typedef struct OraclePoint
{
	b3Vec3 point;

	// Closest point on the point's triangle
	b3Vec3 meshPoint;
	b3Vec3 normal;

	// Geometric separation, without the mesh rest offset the collider subtracts
	float separation;
	int triangleIndex;
	OracleVerdict verdict;
} OraclePoint;

typedef struct OracleResult
{
	// Distance between the hull and the mesh
	float distance;
	b3Vec3 closestMeshPoint;
	b3Vec3 closestHullPoint;

	OraclePair pairs[ORACLE_MAX_TRIANGLES];
	int pairCount;

	OraclePoint points[ORACLE_MAX_POINTS];
	int pointCount;

	// Ghost manifolds
	int ghostCount;
	bool missed;
} OracleResult;

// The poses keep the hull separated by at least this much, where the closest direction is well defined
#define ORACLE_MIN_DISTANCE ( 0.1f * B3_LINEAR_SLOP )

// Caller destroys with b3DestroyMesh
b3MeshData* CreateOracleMesh( OracleScene scene );

// Deterministic random box and pose for the scene and index, separated from the mesh by
// [ORACLE_MIN_DISTANCE, B3_SPECULATIVE_DISTANCE]. Returns false if no separated pose was found.
bool MakeOraclePose( const b3MeshData* mesh, OracleScene scene, int index, b3BoxHull* box, b3Transform* transform );

// Returns false if the hull is not separated by ORACLE_MIN_DISTANCE, in which case the result is empty.
bool EvaluateGhostOracle( OracleResult* result, const b3MeshData* mesh, const b3HullData* hull, b3Transform transform,
						  bool handleGhosts );

#ifdef __cplusplus
}
#endif
