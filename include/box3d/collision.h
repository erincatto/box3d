// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"
#include "math_functions.h"

#include <stdbool.h>
#include <stddef.h>

/// Material properties supported per triangle on meshes and height fields
/// @ingroup shape
typedef struct b3SurfaceMaterial
{
	/// The Coulomb (dry) friction coefficient, usually in the range [0,1].
	float friction;

	/// The coefficient of restitution (bounce) usually in the range [0,1].
	/// https://en.wikipedia.org/wiki/Coefficient_of_restitution
	float restitution;

	/// The rolling resistance usually in the range [0,1]. This is only used for spheres and capsules.
	float rollingResistance;

	/// The tangent velocity for conveyor belts. This is local to the shape and will be projected
	/// onto the contact surface.
	b3Vec3 tangentVelocity;

	/// User material identifier. This is passed with query results and to friction and restitution
	/// combining functions. It is not used internally.
	uint64_t userMaterialId;

	/// Custom debug draw color. Ignored if 0.
	/// @see b3HexColor
	uint32_t customColor;
} b3SurfaceMaterial;

/// Use this to initialize your surface material
/// @ingroup shape
B3_API b3SurfaceMaterial b3DefaultSurfaceMaterial( void );

/**
 * @defgroup geometry Geometry
 * @brief Geometry types and algorithms
 *
 * Definitions of spheres, capsules, hulls, meshes, and height fields.
 * Functions should take the shape as the first argument to assist editor auto-complete.
 * @{
 */

#define B3_MAX_SHAPE_CAST_POINTS 64

/// Low level ray cast input data
typedef struct b3RayCastInput
{
	/// Start point of the ray cast
	b3Vec3 origin;

	/// Translation of the ray cast
	b3Vec3 translation;

	/// The maximum fraction of the translation to consider, typically 1
	float maxFraction;
} b3RayCastInput;

/// A shape proxy is used by the GJK algorithm. It can represent a convex shape.
typedef struct b3ShapeProxy
{
	/// The point cloud
	const b3Vec3* points;

	/// The number of points. Do not exceed B3_MAX_SHAPE_CAST_POINTS.
	int count;

	/// The external radius of the point cloud
	float radius;
} b3ShapeProxy;

/// Low level shape cast input in generic form. This allows casting an arbitrary point
/// cloud wrap with a radius. For example, a sphere is a single point with a non-zero radius.
/// A capsule is two points with a non-zero radius. A box is four points with a zero radius.
typedef struct b3ShapeCastInput
{
	/// A generic query shape
	b3ShapeProxy proxy;

	/// The translation of the shape cast
	b3Vec3 translation;

	/// The maximum fraction of the translation to consider, typically 1
	float maxFraction;

	/// Allow shape cast to encroach when initially touching. This only works if the radius is greater than zero.
	bool canEncroach;
} b3ShapeCastInput;

/// Low level ray cast or shape-cast output data
typedef struct b3CastOutput
{
	/// The surface normal at the hit point
	b3Vec3 normal;

	/// The surface hit point
	b3Vec3 point;

	/// The fraction of the input translation at collision
	float fraction;

	/// The number of iterations used
	int iterations;

	/// The index of the mesh or height field triangle hit
	int triangleIndex;

	/// The index of the compound child shape
	int childIndex;

	/// The material index. May be -1 for null.
	int materialIndex;

	/// Did the cast hit?
	bool hit;
} b3CastOutput;

/**
 * @defgroup tree Dynamic Tree
 * The dynamic tree is a binary AABB tree to organize and query large numbers of geometric objects
 *
 * Box2D uses the dynamic tree internally to sort collision shapes into a binary bounding volume hierarchy.
 * This data structure may have uses in games for organizing other geometry data and may be used independently
 * of Box2D rigid body simulation.
 *
 * A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
 * A dynamic tree arranges data in a binary tree to accelerate
 * queries such as AABB queries and ray casts. Leaf nodes are proxies
 * with an AABB. These are used to hold a user collision object.
 * Nodes are pooled and relocatable, so I use node indices rather than pointers.
 * The dynamic tree is made available for advanced users that would like to use it to organize
 * spatial game data besides rigid bodies.
 * @{
 */

typedef enum b3TreeNodeFlags
{
	b3_allocatedNode = 0x0001,
	b3_enlargedNode = 0x0002,
	b3_leafNode = 0x0004,
} b3TreeNodeFlags;

typedef struct b3TreeNodeChildren
{
	int child1;
	int child2;
} b3TreeNodeChildren;

/// A node in the dynamic tree. This is private data placed here for performance reasons.
/// todo test padding to 64 bytes to avoid straddling cache lines
typedef struct b3TreeNode
{
	/// The node bounding box
	b3AABB aabb; // 24

	/// Category bits for collision filtering
	uint64_t categoryBits; // 8

	union
	{
		/// Children (internal node)
		b3TreeNodeChildren children;

		/// User data (leaf node)
		uint64_t userData;
	}; // 8

	union
	{
		/// The node parent index (allocated node)
		int parent;

		/// The node freelist next index (free node)
		int next;
	}; // 4

	/// Height of the node. Leaves have a height of 0.
	uint16_t height; // 2
	uint16_t flags;	 // 2
} b3TreeNode;

#define B3_DYNAMIC_TREE_VERSION 0x8E867C390754064Bull

/// The dynamic tree structure. This should be considered private data.
/// It is placed here for performance reasons.
typedef struct b3DynamicTree
{
	uint64_t version;

	/// The tree nodes
	b3TreeNode* nodes;

	/// The root index
	int root;

	/// The number of nodes
	int nodeCount;

	/// The allocated node space
	int nodeCapacity;

	/// Number of proxies created
	int proxyCount;

	/// Node free list
	int freeList;

	/// Leaf indices for rebuild
	int* leafIndices;

	/// Leaf bounding boxes for rebuild
	b3AABB* leafBoxes;

	/// Leaf bounding box centers for rebuild
	b3Vec3* leafCenters;

	/// Bins for sorting during rebuild
	int* binIndices;

	/// Allocated space for rebuilding
	int rebuildCapacity;
} b3DynamicTree;

/// These are performance results returned by dynamic tree queries.
typedef struct b3TreeStats
{
	/// Number of internal nodes visited during the query
	int nodeVisits;

	/// Number of leaf nodes visited during the query
	int leafVisits;
} b3TreeStats;

/// Constructing the tree initializes the node pool.
B3_API b3DynamicTree b3DynamicTree_Create( int proxyCapacity );

/// Destroy the tree, freeing the node pool.
B3_API void b3DynamicTree_Destroy( b3DynamicTree* tree );

/// Create a proxy. Provide an AABB and a userData value.
B3_API int b3DynamicTree_CreateProxy( b3DynamicTree* tree, b3AABB aabb, uint64_t categoryBits, uint64_t userData );

/// Destroy a proxy. This asserts if the id is invalid.
B3_API void b3DynamicTree_DestroyProxy( b3DynamicTree* tree, int proxyId );

/// Move a proxy to a new AABB by removing and reinserting into the tree.
B3_API void b3DynamicTree_MoveProxy( b3DynamicTree* tree, int proxyId, b3AABB aabb );

/// Enlarge a proxy and enlarge ancestors as necessary.
B3_API void b3DynamicTree_EnlargeProxy( b3DynamicTree* tree, int proxyId, b3AABB aabb );

/// Modify the category bits on a proxy. This is an expensive operation.
B3_API void b3DynamicTree_SetCategoryBits( b3DynamicTree* tree, int proxyId, uint64_t categoryBits );

/// Get the category bits on a proxy.
B3_API uint64_t b3DynamicTree_GetCategoryBits( b3DynamicTree* tree, int proxyId );

/// This function receives proxies found in the AABB query.
/// @return true if the query should continue
typedef bool b3TreeQueryCallbackFcn( int proxyId, uint64_t userData, void* context );

/// Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.
///	@return performance data
B3_API b3TreeStats b3DynamicTree_Query( const b3DynamicTree* tree, b3AABB aabb, uint64_t maskBits, bool requireAllBits,
										b3TreeQueryCallbackFcn* callback, void* context );

/// This function receives the minimum distance squared so far and proxy to check in the closest query.
/// @return minimum distance squared to user objects in the proxy
typedef float b3TreeQueryClosestCallbackFcn( float distanceSqrMin, int proxyId, uint64_t userData, void* context );

/// Query an AABB for the closest object. The callback class is called for each proxy that might be closest to the supplied point.
/// @param tree the dynamic tree to query
/// @param point the query point
/// @param maskBits nodes are skipped if the bit-wise AND with the node category bits is zero
/// @param requireAllBits nodes are skipped if the bit-wise AND with the node category bits does not equal the maskBits
/// @param callback a user provided instance of b3TreeQueryClosestCallbackFcn
/// @param context a user context object that is provide to the callback
/// @param minDistanceSqr the initial and final minimum squared distance. Provide a small initial to restrict the search and
/// improve performance. If the value is large this query has performance that scales linearly with the number of proxies and
/// would be slower than a brute force search.
///	@return performance data
B3_API b3TreeStats b3DynamicTree_QueryClosest( const b3DynamicTree* tree, b3Vec3 point, uint64_t maskBits, bool requireAllBits,
											   b3TreeQueryClosestCallbackFcn* callback, void* context, float* minDistanceSqr );

/// This function receives clipped ray cast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
typedef float b3TreeRayCastCallbackFcn( const b3RayCastInput* input, int proxyId, uint64_t userData, void* context );

/// Ray cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
/// Bit-wise filtering using mask bits can greatly improve performance in some scenarios.
///	However, this filtering may be approximate, so the user should still apply filtering to results.
/// @param tree the dynamic tree to ray cast
/// @param input the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1)
/// @param maskBits bit mask test: `bool accept = (maskBits & node->categoryBits) != 0;`
/// @param requireAllBits modifies bit mask test: `bool accept = (maskBits & node->categoryBits) == maskBits;`
/// @param callback a callback class that is called for each proxy that is hit by the ray
/// @param context user context that is passed to the callback
///	@return performance data
B3_API b3TreeStats b3DynamicTree_RayCast( const b3DynamicTree* tree, const b3RayCastInput* input, uint64_t maskBits,
										  bool requireAllBits, b3TreeRayCastCallbackFcn* callback, void* context );

/// This function receives clipped ray cast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
typedef float b3TreeShapeCastCallbackFcn( const b3ShapeCastInput* input, int proxyId, uint64_t userData, void* context );

/// Ray cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
/// @param tree the dynamic tree to ray cast
/// @param input the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
/// @param maskBits bit mask test: `bool accept = (maskBits & node->categoryBits) != 0;`
/// @param requireAllBits modifies bit mask test: `bool accept = (maskBits & node->categoryBits) == maskBits;`
/// @param callback a callback class that is called for each proxy that is hit by the shape
/// @param context user context that is passed to the callback
///	@return performance data
B3_API b3TreeStats b3DynamicTree_ShapeCast( const b3DynamicTree* tree, const b3ShapeCastInput* input, uint64_t maskBits,
											bool requireAllBits, b3TreeShapeCastCallbackFcn* callback, void* context );

/// Validate this tree. For testing.
B3_API void b3DynamicTree_Validate( const b3DynamicTree* tree );

/// Get the height of the binary tree.
B3_API int b3DynamicTree_GetHeight( const b3DynamicTree* tree );

/// Get the ratio of the sum of the node areas to the root area.
B3_API float b3DynamicTree_GetAreaRatio( const b3DynamicTree* tree );

/// Get the bounding box that contains the entire tree
B3_API b3AABB b3DynamicTree_GetRootBounds( const b3DynamicTree* tree );

/// Get the number of proxies created
B3_API int b3DynamicTree_GetProxyCount( const b3DynamicTree* tree );

/// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
B3_API int b3DynamicTree_Rebuild( b3DynamicTree* tree, bool fullBuild );

/// Get the number of bytes used by this tree
B3_API int b3DynamicTree_GetByteCount( const b3DynamicTree* tree );

/// Validate this tree. For testing.
B3_API void b3DynamicTree_Validate( const b3DynamicTree* tree );

/// Validate this tree has no enlarged AABBs. For testing.
B3_API void b3DynamicTree_ValidateNoEnlarged( const b3DynamicTree* tree );

/// Save this tree to a file for debugging
B3_API void b3DynamicTree_Save( const b3DynamicTree* tree, const char* fileName );

/// Load a file for debugging
B3_API b3DynamicTree b3DynamicTree_Load( const char* fileName, float scale );

/// Get proxy user data
B3_INLINE uint64_t b3DynamicTree_GetUserData( const b3DynamicTree* tree, int proxyId )
{
	return tree->nodes[proxyId].userData;
}

/// Get the AABB of a proxy
B3_INLINE b3AABB b3DynamicTree_GetAABB( const b3DynamicTree* tree, int proxyId )
{
	return tree->nodes[proxyId].aabb;
}

/**@}*/

/// This holds the mass data computed for a shape.
typedef struct b3MassData
{
	float mass;
	b3Vec3 center;

	// todo_erin this should be central inertia
	b3Matrix3 inertia;
} b3MassData;

/// A solid sphere
typedef struct b3Sphere
{
	/// The local center
	b3Vec3 center;

	/// The radius
	float radius;
} b3Sphere;

/// A solid capsule can be viewed as two hemispheres connected
/// by a rectangle.
typedef struct b3Capsule
{
	/// Local center of the first hemisphere
	b3Vec3 center1;

	/// Local center of the second hemisphere
	b3Vec3 center2;

	/// The radius of the hemispheres
	float radius;
} b3Capsule;

typedef struct b3HullVertex
{
	// A half-edge that has this vertex as the origin
	// Can be used along with edge twins and winding order
	// to traverse all the edges connected to this vertex.
	uint8_t edge;
} b3HullVertex;

/// Half-edge for hull data structure
typedef struct b3HullHalfEdge
{
	// Next edge index CCW
	uint8_t next;

	// Twin edge index
	uint8_t twin;

	// index of origin vertex and point
	uint8_t origin;

	// Face to the left of this edge
	uint8_t face;
} b3HullHalfEdge;

typedef struct b3HullFace
{
	// An arbitrary half-edge on this face
	uint8_t edge;
} b3HullFace;

#define B3_HULL_VERSION 0x8F5034CF987D4FD9ull

/// A convex hull.
/// @note This data structure has data hanging off the end and cannot be directly copied.
typedef struct b3Hull
{
	// Version must be first and match B3_HULL_VERSION
	uint64_t version;
	int byteCount;

	// Hash of this hull (this field is zero when the hash is computed)
	uint32_t hash;

	// Bulk properties
	b3AABB aabb;
	float surfaceArea;
	float volume;
	float innerRadius;
	b3Vec3 center;
	b3Matrix3 centralInertia;

	// Geometry
	int vertexCount;
	int vertexOffset;
	int pointOffset;

	// This is the half-edge count (double the edge count)
	int edgeCount;
	int edgeOffset;

	int faceCount;
	int faceOffset;
	int planeOffset;

} b3Hull;

B3_INLINE const b3HullVertex* b3GetHullVertices( const b3Hull* hull )
{
	if ( hull->vertexOffset == 0 )
	{
		return NULL;
	}

	return (const b3HullVertex*)( (intptr_t)hull + hull->vertexOffset );
}

B3_INLINE const b3Vec3* b3GetHullPoints( const b3Hull* hull )
{
	if ( hull->pointOffset == 0 )
	{
		return NULL;
	}

	return (const b3Vec3*)( (intptr_t)hull + hull->pointOffset );
}

B3_INLINE const b3HullHalfEdge* b3GetHullEdges( const b3Hull* hull )
{
	if ( hull->edgeOffset == 0 )
	{
		return NULL;
	}

	return (const b3HullHalfEdge*)( (intptr_t)hull + hull->edgeOffset );
}

B3_INLINE const b3HullFace* b3GetHullFaces( const b3Hull* hull )
{
	if ( hull->faceOffset == 0 )
	{
		return NULL;
	}

	return (const b3HullFace*)( (intptr_t)hull + hull->faceOffset );
}

B3_INLINE const b3Plane* b3GetHullPlanes( const b3Hull* hull )
{
	if ( hull->planeOffset == 0 )
	{
		return NULL;
	}

	return (const b3Plane*)( (intptr_t)hull + hull->planeOffset );
}

B3_API b3Hull* b3CreateCylinder( float height, float radius, float yOffset, int sides );
B3_API b3Hull* b3CreateCone( float height, float radius1, float radius2, int slices );
B3_API b3Hull* b3CreateHull( const b3Vec3* points, int pointCount, int maxVertexCount );
B3_API b3Hull* b3CloneHull( const b3Hull* hull );
B3_API b3Hull* b3CloneAndTransformHull( const b3Hull* original, b3Transform transform, b3Vec3 scale );
B3_API void b3DestroyHull( b3Hull* hull );

/// Efficient box hull
typedef struct b3BoxHull
{
	b3Hull base;
	b3HullVertex boxVertices[8];
	b3Vec3 boxPoints[8];
	b3HullHalfEdge boxEdges[24];
	b3HullFace boxFaces[6];
	b3Plane boxPlanes[6];
} b3BoxHull;

/// This makes a box hull cube. Do not call b3DestroyHull on this.
B3_API b3BoxHull b3MakeCubeHull( float halfWidth );

/// This makes a box hull. Do not call b3DestroyHull on this.
B3_API b3BoxHull b3MakeBoxHull( float hx, float hy, float hz );

B3_API b3BoxHull b3MakeOffsetBoxHull( float hx, float hy, float hz, b3Vec3 offset );

/// This makes an transformed box hull. Do not call b3DestroyHull on this.
/// @param hx, hy, hz positive half widths
/// @param transform local transform of box
B3_API b3BoxHull b3MakeTransformedBoxHull( float hx, float hy, float hz, b3Transform transform );

/// This makes an transformed box hull with post scaling. This is useful for boxes that are scaled in
/// a level editor. Such scaling can have reflection and sheer. In the case of sheer the result
/// may be approximate. If you need to support sheer consider using b3CreateHull.
/// Do not call b3DestroyHull on this.
/// @param halfWidths positive half widths
/// @param transform local transform of box
/// @param postScale scale applied after the transform, may be negative
B3_API b3BoxHull b3MakeScaledBoxHull( b3Vec3 halfWidths, b3Transform transform, b3Vec3 postScale );

/// This takes a box with a transform and post scale and converts it into a box with the post scale
/// resolved with new half-widths and transform. This accepts non-uniform and negative scale.
/// This is approximate if there is shear.
/// @param halfWidths [in/out] the box half widths
/// @param transform [in/out] the box transform with rotation and translation
/// @param postScale the post scale being applied to the box after the transform
/// @param minHalfWidth the minimum half width after scale is applied
B3_API void b3ScaleBox( b3Vec3* halfWidths, b3Transform* transform, b3Vec3 postScale, float minHalfWidth );

// 64-bit guid that is changed every time the format of the mesh changes
#define B3_MESH_VERSION 0x4A1E7B2C8D5F3091ull

typedef enum b3MeshEdgeFlags
{
	b3_concaveEdge1 = 0x01,
	b3_concaveEdge2 = 0x02,
	b3_concaveEdge3 = 0x04,

	b3_inverseConcaveEdge1 = 0x10,
	b3_inverseConcaveEdge2 = 0x20,
	b3_inverseConcaveEdge3 = 0x40,

	b3_allConcaveEdges = b3_concaveEdge1 | b3_concaveEdge2 | b3_concaveEdge3,

	b3_flatEdge1 = b3_concaveEdge1 | b3_inverseConcaveEdge1,
	b3_flatEdge2 = b3_concaveEdge2 | b3_inverseConcaveEdge2,
	b3_flatEdge3 = b3_concaveEdge3 | b3_inverseConcaveEdge3,

	b3_allFlatEdges = b3_flatEdge1 | b3_flatEdge2 | b3_flatEdge3,

} b3MeshEdgeFlags;

typedef struct b3MeshTriangle
{
	int32_t index1;
	int32_t index2;
	int32_t index3;
} b3MeshTriangle;

typedef struct b3MeshNode
{
	b3Vec3 lowerBound;
	union
	{
		// Internal node
		struct
		{
			uint32_t axis : 2;
			uint32_t childOffset : 30;
		} asNode;

		// Leaf node
		struct
		{
			uint32_t type : 2;
			uint32_t triangleCount : 30;
		} asLeaf;
	} data;

	b3Vec3 upperBound;
	uint32_t triangleOffset;
} b3MeshNode;

/// This is a sorted triangle collision tree.
/// @note This struct has data hanging off the end and cannot be directly copied.
typedef struct b3MeshData
{
	// Version must be first
	uint64_t version;
	int byteCount;

	// Hash of this mesh (this field is zero when the hash is computed)
	uint32_t hash;

	b3AABB bounds;

	/// Combined surface area of all triangles. Single-sided.
	float surfaceArea;

	int treeHeight;
	int degenerateCount;

	// Offset of the node array in bytes from the struct address
	int nodeOffset;
	int nodeCount;

	// Offset of the vertex array in bytes from the struct address
	int vertexOffset;
	int vertexCount;

	// Offset of the triangle array in bytes from the struct address
	int triangleOffset;
	int triangleCount;

	// Offset of the material array in bytes from the struct address
	int materialOffset;
	int materialCount;

	// Offset of the triangle flag array in bytes from the struct address
	int flagsOffset;
} b3MeshData;

/// This small struct allows a mesh data to be re-used with different scale.
typedef struct b3Mesh
{
	const b3MeshData* data;

	/// This scale may be non-uniform and have negative components. However,
	/// no component may be zero or very small in magnitude.
	b3Vec3 scale;

} b3Mesh;

// todo these don't need to be inline
B3_INLINE const b3MeshNode* b3GetMeshNodes( const b3MeshData* mesh )
{
	if ( mesh->nodeOffset == 0 )
	{
		return NULL;
	}

	return (const b3MeshNode*)( (intptr_t)mesh + mesh->nodeOffset );
}

B3_INLINE const b3Vec3* b3GetMeshVertices( const b3MeshData* mesh )
{
	if ( mesh->vertexOffset == 0 )
	{
		return NULL;
	}

	return (const b3Vec3*)( (intptr_t)mesh + mesh->vertexOffset );
}

B3_INLINE const b3MeshTriangle* b3GetMeshTriangles( const b3MeshData* mesh )
{
	if ( mesh->triangleOffset == 0 )
	{
		return NULL;
	}

	return (const b3MeshTriangle*)( (intptr_t)mesh + mesh->triangleOffset );
}

// @note count is equal to the triangle count
B3_INLINE const uint8_t* b3GetMeshMaterialIndices( const b3MeshData* mesh )
{
	if ( mesh->materialOffset == 0 )
	{
		return NULL;
	}

	return (const uint8_t*)( (intptr_t)mesh + mesh->materialOffset );
}

// @note count is equal to the triangle count
B3_INLINE const uint8_t* b3GetMeshFlags( const b3MeshData* mesh )
{
	if ( mesh->flagsOffset == 0 )
	{
		return NULL;
	}

	return (const uint8_t*)( (intptr_t)mesh + mesh->flagsOffset );
}

/// This is used to create a re-usable collision mesh
typedef struct b3MeshDef
{
	/// Triangle vertices
	b3Vec3* vertices;

	/// Triangle vertex indices. 3 for each triangle.
	int32_t* indices;

	/// Triangle material index. 1 per triangle. Indexes into b3ShapeDef::materials.
	/// This allows different run-time material data to be associated with different
	/// instances of this mesh.
	uint8_t* materialIndices;

	/// Tolerance for vertex welding in length units.
	float weldTolerance;

	int vertexCount;
	int triangleCount;

	/// Optionally weld nearby vertices.
	bool weldVertices;

	/// Use the median split instead of SAH to speed up mesh creation. Good
	/// for meshes that are structured like a grid.
	bool useMedianSplit;

	/// Compute triangle adjacency information using shared edges
	bool identifyEdges;

} b3MeshDef;

/// Creates a grid mesh along the x and z axes.
/// @param xCount the number of rows in the x direction
/// @param zCount the number of rows in the z direction
/// @param cellWidth the width of each cell
/// @param materialCount the number of materials to generate
/// @param identifyEdges compute adjacency information
B3_API b3MeshData* b3CreateGridMesh( int xCount, int zCount, float cellWidth, int materialCount, bool identifyEdges );

B3_API b3MeshData* b3CreateWaveMesh( int xCount, int zCount, float cellWidth, float amplitude, float rowFrequency,
									 float columnFrequency );
B3_API b3MeshData* b3CreateTorusMesh( int radialResolution, int tubularResolution, float radius, float thickness );
B3_API b3MeshData* b3CreateBoxMesh( b3Vec3 center, b3Vec3 extent, bool identifyEdges );
B3_API b3MeshData* b3CreatePlatformMesh( b3Vec3 center, float height, float topWidth, float bottomWidth );
B3_API b3MeshData* b3CreateMesh( const b3MeshDef* def, int* degenerateTriangleIndices, int degenerateCapacity );
B3_API void b3DestroyMesh( b3MeshData* mesh );
B3_API int b3GetHeight( const b3MeshData* mesh );

/// Data used to create a height field
typedef struct b3HeightFieldDef
{
	// Grid point heights
	// count = countX * countZ
	float* heights;

	// Grid cell material
	// A value of 0xFF is reserved for holes
	// count = (countX - 1) * (countZ - 1)
	uint8_t* materialIndices;

	b3Vec3 scale;
	int countX;
	int countZ;

	// Global minimum and maximum heights used for quantization. This is important
	// if you want height fields to be placed next to each other and line up exactly.
	// In that case, both height fields should use the same minimum and maximum heights.
	// All height values are clamped to this range.
	// These values are in unscaled space.
	float globalMinimumHeight;
	float globalMaximumHeight;

	bool clockwiseWinding;
} b3HeightFieldDef;

// This material index is used to designate holes in a height field.
#define B3_HEIGHT_FIELD_HOLE 0xFF

// 64-bit guid that is changed every time the format of the height field changes
#define B3_HEIGHT_FIELD_VERSION 0xB6E83F1D947A28C5ull

// A height field with compressed storage.
typedef struct b3HeightField
{
	// Version must be first
	uint64_t version;

	// todo_erin use offset storage
	uint16_t* compressedHeights;
	uint8_t* materialIndices;
	uint8_t* flags;

	// Hash of this height field (this field is zero when the hash is computed).
	// Pointer values above are skipped; the bulk arrays they reference and the
	// scalar block below are folded in at construction time.
	uint32_t hash;

	b3AABB aabb;
	float minHeight;
	float maxHeight;
	float heightScale;
	b3Vec3 scale;
	int columnCount;
	int rowCount;
	bool clockwise;
} b3HeightField;

// Create a height field
B3_API b3HeightField* b3CreateHeightField( const b3HeightFieldDef* data );
B3_API b3HeightField* b3CreateGrid( int rowCount, int columnCount, b3Vec3 scale, bool makeHoles );
B3_API b3HeightField* b3CreateWave( int rowCount, int columnCount, b3Vec3 scale, float rowFrequency, float columnFrequency,
									bool makeHoles );
B3_API void b3DestroyHeightField( b3HeightField* heightField );

// Save input height data to a file
B3_API void b3DumpHeightData( const b3HeightFieldDef* data, const char* fileName );

// Create a height field by loading a previously saved height data
B3_API b3HeightField* b3LoadHeightField( const char* fileName );

typedef struct b3CompoundCapsuleDef
{
	b3Capsule capsule;
	b3SurfaceMaterial material;
} b3CompoundCapsuleDef;

typedef struct b3CompoundHullDef
{
	const b3Hull* hull;
	b3Transform transform;
	b3SurfaceMaterial material;
} b3CompoundHullDef;

typedef struct b3CompoundMeshDef
{
	const b3MeshData* meshData;
	b3Transform transform;
	b3Vec3 scale;

	// This array must line up with the material indices on the triangles.
	const b3SurfaceMaterial* materials;
	int materialCount;

} b3CompoundMeshDef;

typedef struct b3CompoundSphereDef
{
	b3Sphere sphere;
	b3SurfaceMaterial material;
} b3CompoundSphereDef;

/// Definition to creating a compound shape. No data in this structure
/// is referenced in the compound.
typedef struct b3CompoundDef
{
	b3CompoundCapsuleDef* capsules;
	int capsuleCount;

	/// Hulls instances.
	b3CompoundHullDef* hulls;
	int hullCount;

	/// Mesh instances
	b3CompoundMeshDef* meshes;
	int meshCount;

	/// Spheres are fully cloned
	b3CompoundSphereDef* spheres;
	int sphereCount;
} b3CompoundDef;

#define B3_COMPOUND_VERSION ( 0x902AC5D34D9BD452ull ^ B3_DYNAMIC_TREE_VERSION ^ B3_MESH_VERSION ^ B3_HULL_VERSION )

/// Meshes used in compounds have limited space for materials. If you have
/// a mesh with many materials, you can use it outside of the the compound.
#define B3_MAX_COMPOUND_MESH_MATERIALS 4

typedef struct b3Compound
{
	uint64_t version;
	int byteCount;

	// immutable tree
	// The tree node pointer must be fixed up using the node offset
	// todo use a custom tree like b3MeshData to save space and improve performance
	int nodeOffset;
	b3DynamicTree tree;

	int materialOffset;
	int materialCount;

	int capsuleOffset;
	int capsuleCount;

	int hullOffset;
	int hullCount;
	int sharedHullCount;

	int meshOffset;
	int meshCount;
	int sharedMeshCount;

	int sphereOffset;
	int sphereCount;
} b3Compound;

typedef struct b3CompoundCapsule
{
	b3Capsule capsule;
	int materialIndex;
} b3CompoundCapsule;

typedef struct b3CompoundHull
{
	const b3Hull* hull;
	b3Transform transform;
	int materialIndex;
} b3CompoundHull;

typedef struct b3CompoundMesh
{
	const b3MeshData* meshData;
	b3Transform transform;
	b3Vec3 scale;

	// This is used to access the surface material from b3GetCompoundMaterials
	// materialIndex = materialIndices[triangle->materialIndex]
	int materialIndices[B3_MAX_COMPOUND_MESH_MATERIALS];
} b3CompoundMesh;

typedef struct b3CompoundSphere
{
	b3Sphere sphere;
	int materialIndex;
} b3CompoundSphere;

B3_API b3CompoundCapsule b3GetCompoundCapsule( const b3Compound* compound, int index );
B3_API b3CompoundHull b3GetCompoundHull( const b3Compound* compound, int index );
B3_API b3CompoundMesh b3GetCompoundMesh( const b3Compound* compound, int index );
B3_API b3CompoundSphere b3GetCompoundSphere( const b3Compound* compound, int index );
B3_API const b3SurfaceMaterial* b3GetCompoundMaterials( const b3Compound* compound );

// All input data in the definition is cloned into the resulting compound
B3_API b3Compound* b3CreateCompound( const b3CompoundDef* def );
B3_API void b3DestroyCompound( b3Compound* compound );

/// If bytes is null then this returns the number of required bytes. This clones all the
/// data into the bytes buffer. This is expected to run offline or asynchronously.
/// This mutates the compound to nullify pointers, leaving the compound in an unusable state.
B3_API uint8_t* b3ConvertCompoundToBytes( b3Compound* compound );

/// Convert bytes to compound. This does not clone. The bytes must remain in scope while the
/// compound is used. This is done to improve run-time performance and allow for instancing.
/// The bytes are mutated to fixup pointers.
B3_API b3Compound* b3ConvertBytesToCompound( uint8_t* bytes, int byteCount );

/// Compute mass properties of a sphere
B3_API b3MassData b3ComputeSphereMass( const b3Sphere* shape, float density );

/// Compute mass properties of a capsule
B3_API b3MassData b3ComputeCapsuleMass( const b3Capsule* shape, float density );

/// Compute mass properties of a hull
B3_API b3MassData b3ComputeHullMass( const b3Hull* shape, float density );

/// Compute the bounding box of a transformed sphere
B3_API b3AABB b3ComputeSphereAABB( const b3Sphere* shape, b3Transform transform );

/// Compute the bounding box of a transformed capsule
B3_API b3AABB b3ComputeCapsuleAABB( const b3Capsule* shape, b3Transform transform );

/// Compute the bounding box of a transformed hull
B3_API b3AABB b3ComputeHullAABB( const b3Hull* shape, b3Transform transform );

/// Compute the bounding box of a transformed mesh. Scale may be non-uniform and have negative components.
B3_API b3AABB b3ComputeMeshAABB( const b3MeshData* shape, b3Transform transform, b3Vec3 scale );

/// Compute the bounding box of a transformed height-field
B3_API b3AABB b3ComputeHeightFieldAABB( const b3HeightField* shape, b3Transform transform );

/// Compute the bounding box of a compound
B3_API b3AABB b3ComputeCompoundAABB( const b3Compound* shape, b3Transform transform );

/// Use this to ensure your ray cast input is valid and avoid internal assertions.
B3_API bool b3IsValidRay( const b3RayCastInput* input );

/// Overlap shape versus capsule
B3_API bool b3OverlapCapsule( const b3Capsule* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy );

/// Overlap shape versus compound
B3_API bool b3OverlapCompound( const b3Compound* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy );

/// Overlap shape versus height field
B3_API bool b3OverlapHeightField( const b3HeightField* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy );

/// Overlap shape versus hull
B3_API bool b3OverlapHull( const b3Hull* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy );

/// Overlap shape versus mesh
B3_API bool b3OverlapMesh( const b3Mesh* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy );

/// Overlap shape versus sphere
B3_API bool b3OverlapSphere( const b3Sphere* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy );

/// Ray cast versus sphere in local space. Initial overlap is treated as a miss.
B3_API b3CastOutput b3RayCastSphere( const b3Sphere* shape, const b3RayCastInput* input );

/// Ray cast versus hollow sphere in local space.
B3_API b3CastOutput b3RayCastHollowSphere( const b3Sphere* shape, const b3RayCastInput* input );

/// Ray cast versus capsule in local space. Initial overlap is treated as a miss.
B3_API b3CastOutput b3RayCastCapsule( const b3Capsule* shape, const b3RayCastInput* input );

/// Ray cast versus compound in local space. Initial overlap is treated as a miss.
B3_API b3CastOutput b3RayCastCompound( const b3Compound* shape, const b3RayCastInput* input );

/// Ray cast versus hull shape in local space. Initial overlap is treated as a miss.
B3_API b3CastOutput b3RayCastHull( const b3Hull* shape, const b3RayCastInput* input );

/// Ray cast versus mesh in local space.
B3_API b3CastOutput b3RayCastMesh( const b3Mesh* shape, const b3RayCastInput* input );

/// Ray cast versus height field in local space.
B3_API b3CastOutput b3RayCastHeightField( const b3HeightField* shape, const b3RayCastInput* input );

/// Shape cast versus a sphere. Initial overlap is treated as a miss.
B3_API b3CastOutput b3ShapeCastSphere( const b3Sphere* shape, const b3ShapeCastInput* input );

/// Shape cast versus a capsule. Initial overlap is treated as a miss.
B3_API b3CastOutput b3ShapeCastCapsule( const b3Capsule* shape, const b3ShapeCastInput* input );

/// Shape cast versus compound. Initial overlap is treated as a miss.
B3_API b3CastOutput b3ShapeCastCompound( const b3Compound* shape, const b3ShapeCastInput* input );

/// Shape cast versus a hull. Initial overlap is treated as a miss.
B3_API b3CastOutput b3ShapeCastHull( const b3Hull* shape, const b3ShapeCastInput* input );

/// Shape cast versus a mesh. Initial overlap is treated as a miss.
B3_API b3CastOutput b3ShapeCastMesh( const b3Mesh* shape, const b3ShapeCastInput* input );

/// Shape cast versus a height field. Initial overlap is treated as a miss.
B3_API b3CastOutput b3ShapeCastHeightField( const b3HeightField* shape, const b3ShapeCastInput* input );

/// Query callback.
typedef bool b3MeshQueryFcn( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context );

/// Query a mesh for triangles overlapping a bounding box in local space. May have false positives. Useful for debug draw.
/// @param mesh the mesh to query, includes scale
/// @param bounds the bounding box in local space
/// @param fcn a user function to collect triangles
/// @param context the context sent to the user function.
B3_API void b3QueryMesh( const b3Mesh* mesh, const b3AABB bounds, b3MeshQueryFcn* fcn, void* context );

/// Query a height field for triangles overlapping a bounding box in local space. May have false positives. Useful for debug draw.
/// @param heightField the height field to query
/// @param bounds the bounding box in local space
/// @param fcn a user function to collect triangles
/// @param context the context sent to the user function.
B3_API void b3QueryHeightField( const b3HeightField* heightField, b3AABB bounds, b3MeshQueryFcn* fcn, void* context );

/**
 * @defgroup distance Distance
 * Functions for computing the distance between shapes.
 *
 * These are advanced functions you can use to perform distance calculations. There
 * are functions for computing the closest points between shapes, doing linear shape casts,
 * and doing rotational shape casts. The latter is called time of impact (TOI).
 * @{
 */

/// Result of computing the distance between two line segments
typedef struct b3SegmentDistanceResult
{
	/// The closest point on the first segment
	b3Vec3 closest1;

	/// The closest point on the second segment
	b3Vec3 closest2;

	/// The barycentric coordinate on the first segment
	float fraction1;

	/// The barycentric coordinate on the second segment
	float fraction2;

	/// The squared distance between the closest points
	float distanceSquared;
} b3SegmentDistanceResult;

/// Compute the distance between two line segments, clamping at the end points if needed.
B3_API b3SegmentDistanceResult b3SegmentDistance( b3Vec3 p1, b3Vec3 q1, b3Vec3 p2, b3Vec3 q2 );

/// Used to warm start the GJK simplex. If you call this function multiple times with nearby
/// transforms this might improve performance. Otherwise you can zero initialize this.
/// The distance cache must be initialized to zero on the first call.
/// Users should generally just zero initialize this structure for each call.
typedef struct b3SimplexCache
{
	/// Value use to compare length, area, volume of two simplexes.
	float metric;

	// todo use an index of 0xFF as a sentinel and remove the count
	/// The number of stored simplex points
	uint16_t count;

	/// The cached simplex indices on shape A
	uint8_t indexA[4];

	/// The cached simplex indices on shape B
	uint8_t indexB[4];

} b3SimplexCache;

static const b3SimplexCache b3_emptyDistanceCache = B3_ZERO_INIT;

/// Input for b3ShapeDistance
typedef struct b3DistanceInput
{
	/// The proxy for shape A
	b3ShapeProxy proxyA;

	/// The proxy for shape B
	b3ShapeProxy proxyB;

	/// The world transform for shape A
	b3Transform transformA;

	/// The world transform for shape B
	b3Transform transformB;

	/// Should the proxy radius be considered?
	bool useRadii;
} b3DistanceInput;

/// Output for b3ShapeDistance
typedef struct b3DistanceOutput
{
	b3Vec3 pointA;	  ///< Closest point on shapeA
	b3Vec3 pointB;	  ///< Closest point on shapeB
	b3Vec3 normal;	  ///< Normal vector pointing from A to B
	float distance;	  ///< The final distance, zero if overlapped
	int iterations;	  ///< Number of GJK iterations used
	int simplexCount; ///< The number of simplexes stored in the simplex array
} b3DistanceOutput;

/// Simplex vertex for debugging the GJK algorithm
typedef struct b3SimplexVertex
{
	b3Vec3 wA;	///< support point in proxyA
	b3Vec3 wB;	///< support point in proxyB
	b3Vec3 w;	///< wB - wA
	float a;	///< barycentric coordinates
	int indexA; ///< wA index
	int indexB; ///< wB index
} b3SimplexVertex;

/// Simplex from the GJK algorithm
typedef struct b3Simplex
{
	b3SimplexVertex vertices[4]; ///< vertices
	int count;					 ///< number of valid vertices
} b3Simplex;

/// Compute the closest points between two shapes represented as point clouds.
/// b3SimplexCache cache is input/output. On the first call set b3SimplexCache.count to zero.
/// The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity. You may pass in NULL and 0 for these.
/// todo make cache optional (can be null)
B3_API b3DistanceOutput b3ShapeDistance( const b3DistanceInput* input, b3SimplexCache* cache, b3Simplex* simplexes,
										 int simplexCapacity );

/// Input parameters for b3ShapeCast
typedef struct b3ShapeCastPairInput
{
	b3ShapeProxy proxyA;	///< The proxy for shape A
	b3ShapeProxy proxyB;	///< The proxy for shape B
	b3Transform transformA; ///< The world transform for shape A
	b3Transform transformB; ///< The world transform for shape B
	b3Vec3 translationB;	///< The translation of shape B
	float maxFraction;		///< The fraction of the translation to consider, typically 1
	bool canEncroach;		///< Allows shapes with a radius to move slightly closer if already touching
} b3ShapeCastPairInput;

/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
B3_API b3CastOutput b3ShapeCast( const b3ShapeCastPairInput* input );

/// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
/// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
/// position.
typedef struct b3Sweep
{
	b3Vec3 localCenter; ///< Local center of mass position
	b3Vec3 c1;			///< Starting center of mass world position
	b3Vec3 c2;			///< Ending center of mass world position
	b3Quat q1;			///< Starting world rotation
	b3Quat q2;			///< Ending world rotation
} b3Sweep;

/// Evaluate the transform sweep at a specific time.
B3_API b3Transform b3GetSweepTransform( const b3Sweep* sweep, float time );

/// Time of impact input
typedef struct b3TOIInput
{
	b3ShapeProxy proxyA; ///< The proxy for shape A
	b3ShapeProxy proxyB; ///< The proxy for shape B
	b3Sweep sweepA;		 ///< The movement of shape A
	b3Sweep sweepB;		 ///< The movement of shape B
	float maxFraction;	 ///< Defines the sweep interval [0, tMax]
} b3TOIInput;

/// Describes the TOI output
typedef enum b3TOIState
{
	b3_toiStateUnknown,
	b3_toiStateFailed,
	b3_toiStateOverlapped,
	b3_toiStateHit,
	b3_toiStateSeparated
} b3TOIState;

/// Time of impact output
typedef struct b3TOIOutput
{
	/// The type of result
	b3TOIState state;

	/// The hit point
	b3Vec3 point;

	/// The hit normal
	b3Vec3 normal;

	/// The sweep time of the collision
	float fraction;

	/// The final distance
	float distance;

	/// Number of outer iterations
	int distanceIterations;

	/// Total number of push back iterations
	int pushBackIterations;

	/// Total number of root iterations
	int rootIterations;

	bool usedFallback;
} b3TOIOutput;

/// Compute the upper bound on time before two shapes penetrate. Time is represented as
/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collisions. If you change the time interval, you should call this function
/// again.
B3_API b3TOIOutput b3TimeOfImpact( const b3TOIInput* input );

/**@}*/

/// A manifold point is a contact point belonging to a contact manifold.
/// It holds details related to the geometry and dynamics of the contact points.
/// Box3D uses speculative collision so some contact points may be separated.
/// You may use the maxNormalImpulse to determine if there was an interaction during
/// the time step.
typedef struct b3ManifoldPoint
{
	/// Location of the contact point relative to the bodyA center of mass in world space.
	b3Vec3 anchorA;

	/// Location of the contact point relative to the bodyB center of mass in world space.
	b3Vec3 anchorB;

	/// The separation of the contact point, negative if penetrating
	float separation;

	/// Cached separation used for contact recycling
	float baseSeparation;

	/// The impulse along the manifold normal vector. Since Box3D uses sub-stepping, this is
	/// result from the final sub-step.
	float normalImpulse;

	/// The total normal impulse applied during sub-stepping. This is important
	/// to identify speculative contact points that had an interaction in the time step.
	float totalNormalImpulse;

	/// Relative normal velocity pre-solve. Used for hit events. If the normal impulse is
	/// zero then there was no hit. Negative means shapes are approaching.
	float normalVelocity;

	/// Local point for matching
	/// Uniquely identifies a contact point between two shapes
	uint32_t featureId;

	/// Triangle index if one of the shapes is a mesh or height field
	int triangleIndex;

	/// Did this contact point exist the previous step?
	bool persisted;
} b3ManifoldPoint;

/// The maximum number of contact points between two touching shapes.
#define B3_MAX_MANIFOLD_POINTS 4

/// A contact manifold describes the contact points between colliding shapes.
/// @note Box3D uses speculative collision so some contact points may be separated.
typedef struct b3Manifold
{
	/// The manifold points. There may be 1 to 4 valid points.
	b3ManifoldPoint points[B3_MAX_MANIFOLD_POINTS];

	/// The unit normal vector in world space, points from shape A to bodyB
	b3Vec3 normal;

	// Central friction angular impulse (applied about the normal)
	float twistImpulse;

	// Central friction linear impulse
	b3Vec3 frictionImpulse;

	// Rolling resistance angular impulse
	b3Vec3 rollingImpulse;

	/// The number of contacts points, will be 0, 1, or 2
	int pointCount;

} b3Manifold;

// todo adopt this for SAT
typedef enum
{
	b3_invalidAxis = 0,
	b3_backsideAxis,
	b3_faceAxisA,
	b3_faceAxisB,
	b3_edgePairAxis,
	b3_closestPointsAxis,

	// These are for testing
	b3_manualFaceAxisA,
	b3_manualFaceAxisB,
	b3_manualEdgePairAxis,
} b3SeparatingFeature;

typedef enum
{
	b3_featureNone = 0,
	b3_featureTriangleFace,
	b3_featureHullFace,
	// v1-v2
	b3_featureEdge1,
	// v2-v3
	b3_featureEdge2,
	// v3-v1
	b3_featureEdge3,
	b3_featureVertex1,
	b3_featureVertex2,
	b3_featureVertex3
} b3TriangleFeature;

typedef struct
{
	float separation;
	// b3SeparatingFeature
	uint8_t type;
	uint8_t indexA;
	uint8_t indexB;
	uint8_t hit;
} b3SATCache;

/// Contact points are always the result of two edges intersecting.
/// It can be two edges of the same shape, which is just a shape vertex.
/// Or a contact point can be the result of two edges crossing from different shapes.
/// This is designed to support hull versus hull, but it is adapted to work
/// with all shape types. The feature pair is used to identify contact points
/// for temporal coherence and warm starting.
typedef struct b3FeaturePair
{
	// Incoming type (either edge on shape A or shape B)
	uint8_t owner1;
	// Incoming edge index (into associated shape array)
	uint8_t index1;
	// Outgoing type (either edge on shape A or shape B)
	uint8_t owner2;
	// Outgoing edge index (into associated shape array)
	uint8_t index2;
} b3FeaturePair;

/// A local manifold point and normal in frame A
typedef struct b3LocalManifoldPoint
{
	b3Vec3 point;
	float separation;
	b3FeaturePair pair;
	int triangleIndex;
} b3LocalManifoldPoint;

/// A local manifold with no dynamic information. Used by b3Collide functions.
typedef struct b3LocalManifold
{
	b3Vec3 normal;
	b3Vec3 triangleNormal;
	b3LocalManifoldPoint* points;
	int pointCount;
	int triangleIndex;
	int i1, i2, i3;
	float squaredDistance;
	b3TriangleFeature feature;
	int triangleFlags;
} b3LocalManifold;

B3_API void b3CollideSpheres( b3LocalManifold* manifold, int capacity, const b3Sphere* sphereA, const b3Sphere* sphereB,
							  b3Transform transformBtoA );

B3_API void b3CollideCapsuleAndSphere( b3LocalManifold* manifold, int capacity, const b3Capsule* capsuleA, const b3Sphere* sphereB,
									   b3Transform transformBtoA );

B3_API void b3CollideHullAndSphere( b3LocalManifold* manifold, int capacity, const b3Hull* hullA, const b3Sphere* sphereB,
									b3Transform transformBtoA, b3SimplexCache* cache );

B3_API void b3CollideCapsules( b3LocalManifold* manifold, int capacity, const b3Capsule* capsuleA, const b3Capsule* capsuleB,
							   b3Transform transformBtoA );

B3_API void b3CollideHullAndCapsule( b3LocalManifold* manifold, int capacity, const b3Hull* hullA, const b3Capsule* capsuleB,
									 b3Transform transformBtoA, b3SimplexCache* cache );

B3_API void b3CollideHulls( b3LocalManifold* manifold, int capacity, const b3Hull* hullA, const b3Hull* hullB,
							b3Transform transformBtoA, b3SATCache* cache );

B3_API void b3CollideCapsuleAndTriangle( b3LocalManifold* manifold, int capacity, const b3Capsule* capsuleA,
										 const b3Vec3* triangleB, b3SimplexCache* cache );

// b3MeshEdgeFlags
B3_API void b3CollideHullAndTriangle( b3LocalManifold* manifold, int capacity, const b3Hull* hullA, b3Vec3 v1, b3Vec3 v2, b3Vec3 v3,
									  int triangleFlags, b3SATCache* cache );

B3_API void b3CollideSphereAndTriangle( b3LocalManifold* manifold, int capacity, const b3Sphere* sphereA, const b3Vec3* triangleB );

B3_API bool b3IntersectRayAndAABB( b3Vec3 lowerBound, b3Vec3 upperBound, b3Vec3 p1, b3Vec3 p2, float* minFraction,
								   float* maxFraction );

/**@}*/

/**
 * @defgroup character
 * Experimental character movement solver
 * @{
 */

typedef struct b3PlaneResult
{
	b3Plane plane;
	b3Vec3 point;
} b3PlaneResult;

typedef struct b3CollisionPlane
{
	b3Plane plane;
	float pushLimit;
	float push;
	bool clipVelocity;
} b3CollisionPlane;

typedef struct b3PlaneSolverResult
{
	b3Vec3 delta;
	int iterationCount;
} b3PlaneSolverResult;

B3_API b3PlaneSolverResult b3SolvePlanes( b3Vec3 targetDelta, b3CollisionPlane* planes, int count );
B3_API b3Vec3 b3ClipVector( b3Vec3 vector, const b3CollisionPlane* planes, int count );

/**@}*/

typedef struct b3ClosestApproachResult
{
	b3Vec3 point1;
	float lambda1;
	b3Vec3 point2;
	float lambda2;
} b3ClosestApproachResult;

/// Compute the closest point on the segment a-b to the target q
B3_API b3Vec3 b3ClosestPointOnSegment( b3Vec3 a, b3Vec3 b, b3Vec3 q );

/// Compute the closest points on two infinite lines
B3_API b3ClosestApproachResult b3ClosestApproachLines( b3Vec3 p1, b3Vec3 d1, b3Vec3 p2, b3Vec3 d2 );

/// Compute the closest points on two line segments
B3_API b3ClosestApproachResult b3ClosestApproachSegments( b3Vec3 p1, b3Vec3 q1, b3Vec3 p2, b3Vec3 q2 );

typedef struct b3Point2D
{
	b3Vec2 p;
	float separation;
	int16_t originalIndex;
	bool persisted;
} b3Point2D;

B3_API int b3Hull2D( b3Point2D* pts, int count, b3Point2D* hull );
B3_API int b3SimplifyHull2D( b3Point2D* hull, int count, int target );
B3_API int b3CullPoints( b3Point2D* points, int count, int target );
