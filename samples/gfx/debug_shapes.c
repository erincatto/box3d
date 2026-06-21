// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/debug_shapes.h"

#include "gfx/qsort.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// vertex/index buffer construction helpers

typedef struct BuildBuffer
{
	MeshVertex* vertices;
	int vertexCount;
	int vertexCapacity;

	uint32_t* indices;
	int indexCount;
	int indexCapacity;
} BuildBuffer;

// Edge build record. Stored only on the CPU during shape acquire, sorted
// and deduplicated in place, then expanded into EdgeVertex pairs at upload
// time. `v0 < v1` after canonicalization so duplicate edges from adjacent
// triangles compare equal regardless of which triangle emitted them.
typedef struct EdgeRecord
{
	uint32_t v0;
	uint32_t v1;
	uint32_t flags;
} EdgeRecord;

typedef struct EdgeBuilder
{
	EdgeRecord* edges;
	int count;
	int capacity;
} EdgeBuilder;

// Render class carried in the edge flag. Convex must stay 1 so hull edges
// (which emit 1u directly) keep their color. Concave stays 0 to match the
// old binary flag value.
typedef enum EdgeClass
{
	EDGE_CONCAVE = 0,
	EDGE_CONVEX = 1,
	EDGE_FLAT = 2,
} EdgeClass;

// Collapse Box3D's two-bit edge flags to a render class. Concave bit clear
// means convex, a true convex edge or an open boundary with neither bit.
// Both bits set means nearly coplanar.
static uint32_t ClassifyEdge( uint8_t flags, uint8_t concaveBit, uint8_t inverseBit )
{
	if ( flags & concaveBit )
	{
		return ( flags & inverseBit ) ? (uint32_t)EDGE_FLAT : (uint32_t)EDGE_CONCAVE;
	}
	return (uint32_t)EDGE_CONVEX;
}

// Dedup priority when two adjacent triangles disagree on a shared edge. They
// normally agree, this only matters at non-manifold joints. Convex wins as the
// most prominent feature, then concave, then flat.
static uint32_t EdgeClassRank( uint32_t c )
{
	if ( c == EDGE_CONVEX )
	{
		return 2;
	}
	if ( c == EDGE_CONCAVE )
	{
		return 1;
	}
	return 0;
}

static bool EdgeBuilderReserve( EdgeBuilder* b, int extra )
{
	const int need = b->count + extra;
	if ( need <= b->capacity )
	{
		return true;
	}
	int newCap = b->capacity == 0 ? 64 : b->capacity;
	while ( newCap < need )
	{
		newCap *= 2;
	}
	EdgeRecord* grown = (EdgeRecord*)realloc( b->edges, (size_t)newCap * sizeof( EdgeRecord ) );
	if ( !grown )
	{
		fprintf( stderr, "error: edge buffer realloc failed (cap=%d)\n", newCap );
		return false;
	}
	b->edges = grown;
	b->capacity = newCap;
	return true;
}

static void EdgeBuilderFree( EdgeBuilder* b )
{
	free( b->edges );
	b->edges = NULL;
	b->count = 0;
	b->capacity = 0;
}

// Push one canonicalized edge into the builder. Returns false on OOM.
static bool EmitEdge( EdgeBuilder* b, uint32_t a, uint32_t c, uint32_t flag )
{
	if ( !EdgeBuilderReserve( b, 1 ) )
	{
		return false;
	}
	EdgeRecord* r = &b->edges[b->count++];
	if ( a <= c )
	{
		r->v0 = a;
		r->v1 = c;
	}
	else
	{
		r->v0 = c;
		r->v1 = a;
	}
	r->flags = flag;
	return true;
}

// Expand a sorted, deduplicated EdgeRecord array into the EdgeVertex pair
// layout the geometry registry wants. Two consecutive EdgeVertex entries per
// edge (the two endpoints), each carrying the same `flags`. Returns NULL on
// OOM (caller treats as "no edges").
static EdgeVertex* ExpandEdgesForUpload( const EdgeRecord* edges, int edgeCount, const b3Vec3* vertices )
{
	if ( edgeCount == 0 )
	{
		return NULL;
	}

	EdgeVertex* out = (EdgeVertex*)malloc( (size_t)( 2 * edgeCount ) * sizeof( EdgeVertex ) );
	if ( !out )
	{
		fprintf( stderr, "error: edge upload buffer alloc failed (edges=%d)\n", edgeCount );
		return NULL;
	}

	for ( int i = 0; i < edgeCount; ++i )
	{
		const EdgeRecord r = edges[i];
		const b3Vec3 a = vertices[r.v0];
		const b3Vec3 c = vertices[r.v1];
		const float flagF = (float)r.flags;
		out[2 * i + 0].position[0] = a.x;
		out[2 * i + 0].position[1] = a.y;
		out[2 * i + 0].position[2] = a.z;
		out[2 * i + 0].flag = flagF;
		out[2 * i + 1].position[0] = c.x;
		out[2 * i + 1].position[1] = c.y;
		out[2 * i + 1].position[2] = c.z;
		out[2 * i + 1].flag = flagF;
	}

	return out;
}

static bool BufferReserveVertices( BuildBuffer* b, int extra )
{
	const int need = b->vertexCount + extra;
	if ( need <= b->vertexCapacity )
	{
		return true;
	}
	int newCap = b->vertexCapacity == 0 ? 256 : b->vertexCapacity;
	while ( newCap < need )
	{
		newCap *= 2;
	}
	MeshVertex* grown = (MeshVertex*)realloc( b->vertices, (size_t)newCap * sizeof( MeshVertex ) );
	if ( !grown )
	{
		fprintf( stderr, "error: vertex buffer realloc failed (cap=%d)\n", newCap );
		return false;
	}
	b->vertices = grown;
	b->vertexCapacity = newCap;
	return true;
}

static bool BufferReserveIndices( BuildBuffer* b, int extra )
{
	const int need = b->indexCount + extra;
	if ( need <= b->indexCapacity )
	{
		return true;
	}
	int newCap = b->indexCapacity == 0 ? 384 : b->indexCapacity;
	while ( newCap < need )
	{
		newCap *= 2;
	}
	uint32_t* grown = (uint32_t*)realloc( b->indices, (size_t)newCap * sizeof( uint32_t ) );
	if ( !grown )
	{
		fprintf( stderr, "error: index buffer realloc failed (cap=%d)\n", newCap );
		return false;
	}
	b->indices = grown;
	b->indexCapacity = newCap;
	return true;
}

static void BufferFree( BuildBuffer* b )
{
	free( b->vertices );
	free( b->indices );
	b->vertices = NULL;
	b->indices = NULL;
	b->vertexCount = 0;
	b->vertexCapacity = 0;
	b->indexCount = 0;
	b->indexCapacity = 0;
}

// Append three vertices (one triangle) with a shared face normal and add
// three consecutive indices that reference them. Returns false on OOM.
static bool EmitFlatTriangle( BuildBuffer* b, b3Vec3 p0, b3Vec3 p1, b3Vec3 p2, b3Vec3 normal )
{
	if ( !BufferReserveVertices( b, 3 ) || !BufferReserveIndices( b, 3 ) )
	{
		return false;
	}

	MeshVertex* v = &b->vertices[b->vertexCount];
	v[0].position[0] = p0.x;
	v[0].position[1] = p0.y;
	v[0].position[2] = p0.z;
	v[0].normal[0] = normal.x;
	v[0].normal[1] = normal.y;
	v[0].normal[2] = normal.z;
	v[1].position[0] = p1.x;
	v[1].position[1] = p1.y;
	v[1].position[2] = p1.z;
	v[1].normal[0] = normal.x;
	v[1].normal[1] = normal.y;
	v[1].normal[2] = normal.z;
	v[2].position[0] = p2.x;
	v[2].position[1] = p2.y;
	v[2].position[2] = p2.z;
	v[2].normal[0] = normal.x;
	v[2].normal[1] = normal.y;
	v[2].normal[2] = normal.z;

	const uint32_t base = (uint32_t)b->vertexCount;
	b->indices[b->indexCount + 0] = base + 0u;
	b->indices[b->indexCount + 1] = base + 1u;
	b->indices[b->indexCount + 2] = base + 2u;

	b->vertexCount += 3;
	b->indexCount += 3;
	return true;
}

static b3Vec3 TriangleNormal( b3Vec3 a, b3Vec3 b, b3Vec3 c )
{
	const float ex = b.x - a.x, ey = b.y - a.y, ez = b.z - a.z;
	const float fx = c.x - a.x, fy = c.y - a.y, fz = c.z - a.z;
	b3Vec3 n;
	n.x = ey * fz - ez * fy;
	n.y = ez * fx - ex * fz;
	n.z = ex * fy - ey * fx;
	const float lenSq = n.x * n.x + n.y * n.y + n.z * n.z;
	if ( lenSq > 1.0e-20f )
	{
		const float inv = 1.0f / sqrtf( lenSq );
		n.x *= inv;
		n.y *= inv;
		n.z *= inv;
	}
	else
	{
		n.x = 0.0f;
		n.y = 1.0f;
		n.z = 0.0f;
	}
	return n;
}

static MeshHandle BuildHull( const b3HullData* hull )
{
	const b3Vec3* points = b3GetHullPoints( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	if ( !points || !edges || !faces || !planes )
	{
		fprintf( stderr, "error: hull missing arrays (hash=0x%08x)\n", hull->hash );
		return InvalidMeshHandle();
	}

	BuildBuffer buf = { 0 };
	EdgeBuilder eb = { 0 };

	// Per-face fan triangulation. Each face's half-edges form a CCW loop
	// (viewed from outside, against the face's outward normal). The fan
	// pivot is the loop's first vertex, subsequent triangles are
	// (v0, v_i, v_{i+1}) for i from 1 to count-2. Plane normal carries
	// the outward direction.
	for ( int faceIdx = 0; faceIdx < hull->faceCount; ++faceIdx )
	{
		const b3HullFace face = faces[faceIdx];
		const b3Vec3 normal = planes[faceIdx].normal;
		const uint8_t startEdge = face.edge;

		// First pass: count loop length so we know how to fan it. b3 hull
		// faces are bounded (max half-edge count fits a uint8_t) so this
		// terminates cleanly.
		uint8_t e = startEdge;
		int loopLen = 0;
		do
		{
			loopLen += 1;
			e = edges[e].next;
			if ( loopLen > 256 )
			{
				fprintf( stderr, "error: hull face loop runaway (hash=0x%08x)\n", hull->hash );
				BufferFree( &buf );
				return InvalidMeshHandle();
			}
		}
		while ( e != startEdge );

		if ( loopLen < 3 )
		{
			continue;
		}

		// Second pass: emit fan. Collect the loop's vertex indices on the stack.
		// loopLen <= 256.
		uint8_t loop[256];
		e = startEdge;
		for ( int i = 0; i < loopLen; ++i )
		{
			loop[i] = edges[e].origin;
			e = edges[e].next;
		}

		b3Vec3 p0 = points[loop[0]];
		for ( int i = 1; i < loopLen - 1; ++i )
		{
			b3Vec3 p1 = points[loop[i]];
			b3Vec3 p2 = points[loop[i + 1]];
			if ( !EmitFlatTriangle( &buf, p0, p1, p2, normal ) )
			{
				BufferFree( &buf );
				return InvalidMeshHandle();
			}
		}
	}

	if ( buf.indexCount == 0 )
	{
		BufferFree( &buf );
		return InvalidMeshHandle();
	}

	// Emit hull edges. This automatically prevents duplicates. No sorting needed.
	int pairCount = hull->edgeCount / 2;
	for ( int i = 0; i < pairCount; i += 1 )
	{
		uint8_t a = edges[2 * i + 0].origin;
		uint8_t c = edges[2 * i + 1].origin;

		if ( !EmitEdge( &eb, (uint32_t)a, (uint32_t)c, 1u ) )
		{
			EdgeBuilderFree( &eb );
			BufferFree( &buf );
			return InvalidMeshHandle();
		}
	}

	MeshHandle h = RegisterMesh( hull->hash, buf.vertices, buf.vertexCount, buf.indices, buf.indexCount, "geom_hull" );

	if ( IsMeshHandleValid( h ) )
	{
		SetMeshKind( h, MESH_KIND_HULL );
		if ( eb.count > 0 )
		{
			EdgeVertex* upload = ExpandEdgesForUpload( eb.edges, eb.count, points );
			if ( upload )
			{
				RegisterMeshEdges( h, upload, eb.count, "geom_hull_edges" );
				free( upload );
			}
		}
	}

	EdgeBuilderFree( &eb );
	BufferFree( &buf );
	return h;
}

// Mesh edge sort/dedup. EdgeRecord is canonicalized (v0 <= v1) by emitEdge,
// so a shared interior edge from two adjacent triangles compares equal and
// the unique pass drops the duplicate. Sort key is (v0, v1) lex order.
static inline int MeshEdgeLess( const EdgeRecord* a, const EdgeRecord* b )
{
	if ( a->v0 != b->v0 )
	{
		return a->v0 < b->v0;
	}
	return a->v1 < b->v1;
}

static MeshHandle BuildMeshData( const b3MeshData* meshData )
{
	const b3Vec3* verts = b3GetMeshVertices( meshData );
	const b3MeshTriangle* tris = b3GetMeshTriangles( meshData );
	const uint8_t* flags = b3GetMeshFlags( meshData );
	if ( !verts || !tris || meshData->triangleCount <= 0 )
	{
		fprintf( stderr, "error: mesh missing arrays or empty (hash=0x%08x)\n", meshData->hash );
		return InvalidMeshHandle();
	}

	BuildBuffer buf = { 0 };
	EdgeBuilder eb = { 0 };

	for ( int t = 0; t < meshData->triangleCount; ++t )
	{
		const b3MeshTriangle tri = tris[t];
		const b3Vec3 a = verts[tri.index1];
		const b3Vec3 b = verts[tri.index2];
		const b3Vec3 c = verts[tri.index3];
		const b3Vec3 n = TriangleNormal( a, b, c );
		if ( !EmitFlatTriangle( &buf, a, b, c, n ) )
		{
			EdgeBuilderFree( &eb );
			BufferFree( &buf );
			return InvalidMeshHandle();
		}

		// Emit all three edges, classified by convexity
		const uint8_t f = flags ? flags[t] : 0u;
		bool ok = EmitEdge( &eb, (uint32_t)tri.index1, (uint32_t)tri.index2,
							ClassifyEdge( f, b3_concaveEdge1, b3_inverseConcaveEdge1 ) );
		if ( ok )
		{
			ok = EmitEdge( &eb, (uint32_t)tri.index2, (uint32_t)tri.index3,
						   ClassifyEdge( f, b3_concaveEdge2, b3_inverseConcaveEdge2 ) );
		}

		if ( ok )
		{
			ok = EmitEdge( &eb, (uint32_t)tri.index3, (uint32_t)tri.index1,
						   ClassifyEdge( f, b3_concaveEdge3, b3_inverseConcaveEdge3 ) );
		}

		if ( !ok )
		{
			EdgeBuilderFree( &eb );
			BufferFree( &buf );
			return InvalidMeshHandle();
		}
	}

	if ( buf.indexCount == 0 )
	{
		EdgeBuilderFree( &eb );
		BufferFree( &buf );
		return InvalidMeshHandle();
	}

	// Sort + unique on the canonicalized (v0, v1) key. svpv/qsort macro
	// inlines the comparator and swap so this is fast even for very dense
	// meshes, runs once at acquire, not per frame.
	if ( eb.count > 1 )
	{
#define LESS( i, j ) MeshEdgeLess( &eb.edges[(int)( i )], &eb.edges[(int)( j )] )
#define SWAP( i, j )                                                                                                             \
	do                                                                                                                           \
	{                                                                                                                            \
		EdgeRecord tmp_ = eb.edges[(int)( i )];                                                                                  \
		eb.edges[(int)( i )] = eb.edges[(int)( j )];                                                                             \
		eb.edges[(int)( j )] = tmp_;                                                                                             \
	}                                                                                                                            \
	while ( 0 )

		QSORT( eb.count, LESS, SWAP );
#undef LESS
#undef SWAP

		// In-place unique. Two records compare equal when (v0, v1) match.
		// On a flag tie the higher-rank class wins so the overlay renders the
		// edge in the more prominent color when the two triangles' bits
		// disagree (rare but possible at non-manifold joints).
		int write = 0;
		for ( int read = 1; read < eb.count; ++read )
		{
			EdgeRecord* w = &eb.edges[write];
			EdgeRecord* r = &eb.edges[read];
			if ( w->v0 == r->v0 && w->v1 == r->v1 )
			{
				if ( EdgeClassRank( r->flags ) > EdgeClassRank( w->flags ) )
				{
					w->flags = r->flags;
				}
			}
			else
			{
				++write;
				eb.edges[write] = *r;
			}
		}
		eb.count = write + 1;
	}

	const MeshHandle h = RegisterMesh( meshData->hash, buf.vertices, buf.vertexCount, buf.indices, buf.indexCount, "geom_mesh" );

	if ( IsMeshHandleValid( h ) )
	{
		SetMeshKind( h, MESH_KIND_MESH );
		if ( eb.count > 0 )
		{
			EdgeVertex* upload = ExpandEdgesForUpload( eb.edges, eb.count, verts );
			if ( upload )
			{
				RegisterMeshEdges( h, upload, eb.count, "geom_mesh_edges" );
				free( upload );
			}
		}
	}

	EdgeBuilderFree( &eb );
	BufferFree( &buf );
	return h;
}

static b3Vec3 HeightFieldSample( const b3HeightFieldData* hf, int row, int col )
{
	int index = row * hf->columnCount + col;
	float decoded = hf->minHeight + hf->heightScale * (float)b3GetHeightFieldCompressedHeights( hf )[index];
	b3Vec3 scale = hf->scale;
	b3Vec3 v;
	v.x = scale.x * (float)col;
	v.y = scale.y * decoded;
	v.z = scale.z * (float)row;
	return v;
}

static MeshHandle BuildHeightField( const b3HeightFieldData* hf )
{
	if ( hf->columnCount < 2 || hf->rowCount < 2 )
	{
		fprintf( stderr, "error: heightfield degenerate (hash=0x%08x)\n", hf->hash );
		return InvalidMeshHandle();
	}

	BuildBuffer buf = { 0 };
	EdgeBuilder eb = { 0 };

	const int cols = hf->columnCount;
	const int rows = hf->rowCount;
	const uint8_t* materials = b3GetHeightFieldMaterialIndices( hf ); // may be NULL if all solid
	const uint8_t* edgeFlags = b3GetHeightFieldFlags( hf );			 // per-triangle concave bits
	const bool clockwise = hf->clockwise;

	// Build a (rows x cols) grid vertex array up front. Triangle emission
	// stays per-cell with duplicated corners for flat shading, the edge
	// builder refers to grid indices, so it needs a contiguous grid array
	// to expand against at upload time.
	b3Vec3* gridVertices = (b3Vec3*)malloc( (size_t)( rows * cols ) * sizeof( b3Vec3 ) );
	if ( !gridVertices )
	{
		fprintf( stderr, "error: hf grid alloc failed (rows=%d cols=%d)\n", rows, cols );
		return InvalidMeshHandle();
	}
	for ( int row = 0; row < rows; ++row )
	{
		for ( int col = 0; col < cols; ++col )
		{
			gridVertices[row * cols + col] = HeightFieldSample( hf, row, col );
		}
	}

	// Each cell (row, col) covers world rectangle:
	//   x in [scale.x * col,     scale.x * (col + 1)]
	//   z in [scale.z * row,     scale.z * (row + 1)]
	// with corner heights at the four grid corners. b3HeightFieldData holds
	// per-cell material. B3_HEIGHT_FIELD_HOLE (0xFF) marks a hole, skip
	// both triangles for that cell. Triangulation matches Box3D's collision
	// triangulation (see height_field.c line 228+):
	//
	//   tri0: (col, row),     (col, row+1),     (col+1, row)
	//   tri1: (col+1, row+1), (col+1, row),     (col, row+1)
	//
	// Box3D's `clockwise` flag flips the index order. The "clockwise"
	// terminology is the source-side winding seen from +Y above, the
	// renderer wants CCW-from-outside (which for an upward-facing
	// heightfield means CCW from +Y). When clockwise=true we mirror the
	// indices to flip the winding.
	for ( int row = 0; row < rows - 1; ++row )
	{
		for ( int col = 0; col < cols - 1; ++col )
		{
			const int cellIndex = row * ( cols - 1 ) + col;
			if ( materials && materials[cellIndex] == B3_HEIGHT_FIELD_HOLE )
			{
				continue;
			}

			const uint32_t i00 = (uint32_t)( row * cols + col );
			const uint32_t i10 = (uint32_t)( row * cols + col + 1 );
			const uint32_t i01 = (uint32_t)( ( row + 1 ) * cols + col );
			const uint32_t i11 = (uint32_t)( ( row + 1 ) * cols + col + 1 );

			const b3Vec3 p00 = gridVertices[i00];
			const b3Vec3 p10 = gridVertices[i10];
			const b3Vec3 p01 = gridVertices[i01];
			const b3Vec3 p11 = gridVertices[i11];

			b3Vec3 a0, b0, c0, a1, b1, c1;
			if ( clockwise )
			{
				a0 = p00;
				b0 = p10;
				c0 = p01;
				a1 = p11;
				b1 = p01;
				c1 = p10;
			}
			else
			{
				a0 = p00;
				b0 = p01;
				c0 = p10;
				a1 = p11;
				b1 = p10;
				c1 = p01;
			}

			const b3Vec3 n0 = TriangleNormal( a0, b0, c0 );
			const b3Vec3 n1 = TriangleNormal( a1, b1, c1 );
			if ( !EmitFlatTriangle( &buf, a0, b0, c0, n0 ) || !EmitFlatTriangle( &buf, a1, b1, c1, n1 ) )
			{
				EdgeBuilderFree( &eb );
				BufferFree( &buf );
				free( gridVertices );
				return InvalidMeshHandle();
			}

			// Diagonal edge (shared between tri0 and tri1), classified by
			// convexity. b3's flag interpretation is fixed to the
			// source-order triangulation, independent of our render-side
			// clockwise flip.
			const int tri0 = 2 * cellIndex;
			const uint8_t f0 = edgeFlags ? edgeFlags[tri0] : 0u;
			if ( !EmitEdge( &eb, i10, i01, ClassifyEdge( f0, b3_concaveEdge2, b3_inverseConcaveEdge2 ) ) )
			{
				EdgeBuilderFree( &eb );
				BufferFree( &buf );
				free( gridVertices );
				return InvalidMeshHandle();
			}
		}
	}

	if ( buf.indexCount == 0 )
	{
		EdgeBuilderFree( &eb );
		BufferFree( &buf );
		free( gridVertices );
		return InvalidMeshHandle();
	}

	// Row edges (horizontal, between (row, col) and (row, col+1)). Iterate
	// every grid row including the outer two. Flag from concaveEdge3 of the
	// adjacent tri0 (cell below the edge) or, on the boundary row, of the
	// adjacent tri1 (cell above). Skip edges whose only adjacent cell is a
	// hole.
	for ( int row = 0; row < rows; ++row )
	{
		for ( int col = 0; col < cols - 1; ++col )
		{
			int triangleIndex = -1;
			if ( row < rows - 1 )
			{
				const int cellIndex = row * ( cols - 1 ) + col;
				if ( !materials || materials[cellIndex] != B3_HEIGHT_FIELD_HOLE )
				{
					triangleIndex = 2 * cellIndex;
				}
			}
			if ( triangleIndex < 0 && row > 0 )
			{
				const int cellIndex = ( row - 1 ) * ( cols - 1 ) + col;
				if ( !materials || materials[cellIndex] != B3_HEIGHT_FIELD_HOLE )
				{
					triangleIndex = 2 * cellIndex + 1;
				}
			}
			if ( triangleIndex < 0 )
			{
				continue;
			}

			const uint8_t f = edgeFlags ? edgeFlags[triangleIndex] : 0u;
			const uint32_t i0 = (uint32_t)( row * cols + col );
			const uint32_t i1 = (uint32_t)( row * cols + col + 1 );
			if ( !EmitEdge( &eb, i0, i1, ClassifyEdge( f, b3_concaveEdge3, b3_inverseConcaveEdge3 ) ) )
			{
				EdgeBuilderFree( &eb );
				BufferFree( &buf );
				free( gridVertices );
				return InvalidMeshHandle();
			}
		}
	}

	// Column edges (vertical, between (row, col) and (row+1, col)). Same
	// flag-sourcing logic, looking at the cell to the right (concaveEdge1
	// of its tri0) or, on the boundary column, the cell to the left
	// (concaveEdge1 of its tri1).
	for ( int col = 0; col < cols; ++col )
	{
		for ( int row = 0; row < rows - 1; ++row )
		{
			int triangleIndex = -1;
			if ( col < cols - 1 )
			{
				const int cellIndex = row * ( cols - 1 ) + col;
				if ( !materials || materials[cellIndex] != B3_HEIGHT_FIELD_HOLE )
				{
					triangleIndex = 2 * cellIndex;
				}
			}
			if ( triangleIndex < 0 && col > 0 )
			{
				const int cellIndex = row * ( cols - 1 ) + col - 1;
				if ( !materials || materials[cellIndex] != B3_HEIGHT_FIELD_HOLE )
				{
					triangleIndex = 2 * cellIndex + 1;
				}
			}
			if ( triangleIndex < 0 )
			{
				continue;
			}

			const uint8_t f = edgeFlags ? edgeFlags[triangleIndex] : 0u;
			const uint32_t i0 = (uint32_t)( row * cols + col );
			const uint32_t i1 = (uint32_t)( ( row + 1 ) * cols + col );
			if ( !EmitEdge( &eb, i0, i1, ClassifyEdge( f, b3_concaveEdge1, b3_inverseConcaveEdge1 ) ) )
			{
				EdgeBuilderFree( &eb );
				BufferFree( &buf );
				free( gridVertices );
				return InvalidMeshHandle();
			}
		}
	}

	const MeshHandle h = RegisterMesh( hf->hash, buf.vertices, buf.vertexCount, buf.indices, buf.indexCount, "geom_heightfield" );

	if ( IsMeshHandleValid( h ) )
	{
		SetMeshKind( h, MESH_KIND_HEIGHTFIELD );
		if ( eb.count > 0 )
		{
			EdgeVertex* upload = ExpandEdgesForUpload( eb.edges, eb.count, gridVertices );
			if ( upload )
			{
				RegisterMeshEdges( h, upload, eb.count, "geom_heightfield_edges" );
				free( upload );
			}
		}
	}

	EdgeBuilderFree( &eb );
	BufferFree( &buf );
	free( gridVertices );
	return h;
}

MeshHandle FindOrAddHull( const b3HullData* hull )
{
	if ( !hull || hull->hash == 0u )
	{
		return InvalidMeshHandle();
	}

	MeshHandle existing = FindMesh( hull->hash );
	if ( IsMeshHandleValid( existing ) )
	{
		AddMeshReference( existing );
		return existing;
	}

	return BuildHull( hull );
}

MeshHandle FindOrAddMesh( const b3MeshData* meshData )
{
	if ( !meshData || meshData->hash == 0u )
	{
		return InvalidMeshHandle();
	}

	MeshHandle existing = FindMesh( meshData->hash );
	if ( IsMeshHandleValid( existing ) )
	{
		AddMeshReference( existing );
		return existing;
	}

	return BuildMeshData( meshData );
}

MeshHandle FindOrAddHeightField( const b3HeightFieldData* heightField )
{
	if ( !heightField || heightField->hash == 0u )
	{
		return InvalidMeshHandle();
	}
	MeshHandle existing = FindMesh( heightField->hash );
	if ( IsMeshHandleValid( existing ) )
	{
		AddMeshReference( existing );
		return existing;
	}
	return BuildHeightField( heightField );
}
