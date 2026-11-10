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

#define ORACLE_MAX_TRIANGLES 256
#define ORACLE_MAX_POINTS 128

// Poses per scene in the unit test, and in the sample scan so the two report the same counts
#define ORACLE_SCAN_POSES 500

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
	b3Vec3 faceNormal;
	b3Vec3 meshPoint;
	b3Vec3 hullPoint;
	b3Vec3 normal;
	float distance;
	bool owned;

	// Triangle index of a closer triangle containing the mesh point, or -1
	int dominatedBy;
} OraclePair;

// The union of the triangles near the hull. Depends only on the pose, not on the collider.
typedef struct OracleReference
{
	// Distance between the hull and the mesh
	float distance;
	b3Vec3 closestMeshPoint;
	b3Vec3 closestHullPoint;

	OraclePair pairs[ORACLE_MAX_TRIANGLES];
	int pairCount;
} OracleReference;

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

// The collider output judged against the reference
typedef struct OracleResult
{
	OraclePoint points[ORACLE_MAX_POINTS];
	int pointCount;

	// Ghost manifolds
	int ghostCount;

	// The union closest pair has no collided point
	bool missed;
} OracleResult;

typedef struct OraclePose
{
	b3BoxHull box;
	b3Transform transform;
	OracleReference reference;

	// Collided without and with ghost handling
	OracleResult raw;
	OracleResult handled;
} OraclePose;

typedef struct OracleScan
{
	// Separated poses
	int validCount;

	// Poses with a ghost manifold, without and with ghost handling
	int rawGhostPoses;
	int ghostPoses;

	// Poses where ghost handling lost the closest pair
	int missedPoses;

	// Pose indices, -1 for none
	int firstGhost;
	int firstMissed;
} OracleScan;

// The poses keep the hull separated by at least this much, where the closest direction is well defined
#define ORACLE_MIN_DISTANCE ( 0.1f * B3_LINEAR_SLOP )

// Caller destroys with b3DestroyMesh
b3MeshData* CreateOracleMesh( OracleScene scene );

// Deterministic random box and pose for the scene and index, separated from the mesh by
// [ORACLE_MIN_DISTANCE, B3_SPECULATIVE_DISTANCE], collided and judged with and without ghost handling.
// Returns false if no separated pose was found, in which case the reference and results mean nothing.
bool EvaluateOraclePose( OraclePose* pose, const b3MeshData* mesh, OracleScene scene, int index );

// A miss counts against ghost handling only when the collider represents the closest pair without it
bool IsOracleCullingMiss( const OraclePose* pose );

// Poses [0, poseCount) of the scene
OracleScan ScanGhostOracle( const b3MeshData* mesh, OracleScene scene, int poseCount );

#ifdef __cplusplus
}
#endif
