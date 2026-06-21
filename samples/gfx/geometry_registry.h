// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Ref-counted GPU buffer store for triangle geometries (convex hulls,
// triangle meshes, heightfields). Owned by the renderer
// Keyed by uint32_t content hash for Box3D shapes we
// pass through b3HullData.hash / b3MeshData.hash / b3HeightFieldData.hash
//
// Lifecycle:
//
//  handle = FindMesh(hash)
//  if (!IsMeshHandleValid(handle)) {
//      // Build CPU verts + indices, then:
//      handle = RegisterMesh(hash, verts, vc, index, ic, "label");
//  } else {
//      AddMeshReference(handle);
//  }
//  // ...later, when the owning shape is destroyed:
//  ReleaseMeshReference(handle);
//
// Per-frame:
//
//   ResetFrameForMeshes(); // reset all entry counts
//   AppendMesh(handle, &transform, scale, color); // many times, across handles
//
// Renderer drains via GetMeshDrawSpanCount + GetMeshDrawSpan.
//
// All instances across all entries land in one shared GPU storage buffer.
// Each draw call rebinds the entry's vbo/ibo and sets `inst_base` (the
// offset into the global instance array) via ub_draw so the shader can
// index correctly. sokol_gfx's sg_draw has no first_instance parameter,
// so the offset is plumbed as a per-draw uniform.

#pragma once

#include "gfx/draw_overlay.h"
#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Upper bound on transparent mesh instances drained in a frame. Shared so the
// renderer can size its sort scratch to match the registry cap.
#define MAX_GEOM_XP_INSTANCES_GLOBAL ( 16 * 1024 )

typedef struct MeshHandle
{
	int index;	   // -1 means invalid
	uint32_t hash; // for cheap validation against index reuse
} MeshHandle;

static inline MeshHandle InvalidMeshHandle( void )
{
	MeshHandle h = { -1, 0u };
	return h;
}

static inline bool IsMeshHandleValid( MeshHandle h )
{
	return h.index >= 0;
}

// Per-vertex data uploaded to the GPU.
typedef struct MeshVertex
{
	float position[3];
	float normal[3];
} MeshVertex;

// Per-edge-endpoint data for the edge overlay. Two consecutive entries
// per edge (the two endpoints). `flag` carries the Box3D edge classification
// as 0.0 = concave, 1.0 = convex/face boundary, 2.0 = flat (coplanar), used
// by the FS to pick between the concave/convex/flat color uniforms. Stored as
// a float (not uint) so the shader struct can be a flat vec4, sidesteps
// std430 vec3+uint padding ambiguity across GLSL/HLSL/MSL cross-compilers.
typedef struct EdgeVertex
{
	float position[3];
	float flag;
} EdgeVertex;

// One draw span returned by the registry to the renderer. Spans are
// produced in registry-entry order. The renderer iterates and issues one
// sg_draw per span.
typedef struct MeshDrawSpan
{
	sg_buffer vbo;
	sg_buffer ibo;
	int indexCount;
	int firstInstance;
	int instanceCount;
} MeshDrawSpan;

// Shape kind tag carried by a registry entry
typedef enum MeshKind
{
	MESH_KIND_UNKNOWN = 0,
	MESH_KIND_HULL,
	MESH_KIND_MESH,
	MESH_KIND_HEIGHTFIELD,
} MeshKind;

// Per-entry edge-draw batch produced by UploadMeshInstances. One batch
// per (entry x arena) with non-empty instance count and registered edges:
// up to two batches per entry (opaque + transparent). `firstInstance`
// indexes the opaque buffer when `isXp == false`, the xp buffer otherwise,
// the edge pass picks the matching storage view via
// GetMeshInstanceView / GetTransparentMeshInstanceView.
typedef struct MeshEdgeBatch
{
	MeshKind kind;
	sg_view edgeStorageView;
	int edgeCount;
	int firstInstance;
	int instanceCount;
	bool isTransparent;
} MeshEdgeBatch;

// One transparent geom instance, ready to feed into the back-to-front
// sort and a per-instance draw. `firstInstance` indexes the shared
// transparent geom buffer (bound via GetTransparentMeshInstanceView). `origin` is
// the world-space position of the instance, recovered from xform_row*.w.
// The renderer uses it as the back-to-front sort key. `shadowCast` is 1.0
// when SHADOW_FULL, 0.0 otherwise (packed from the instance's material.z
// so the renderer doesn't have to repeat the unpack).
typedef struct MeshXpInstance
{
	sg_buffer vbo;
	sg_buffer ibo;
	int indexCount;
	int firstInstance;
	b3Vec3 origin;
	float shadowCast;
} MeshXpInstance;

void CreateMeshRegistry( void );
void DestroyMeshRegistry( void );

MeshHandle FindMesh( uint32_t hash );

MeshHandle RegisterMesh( uint32_t hash, const MeshVertex* vertices, int vertexCount, const uint32_t* indices,
								int indexCount, const char* debugLabel );

// Optional: register an edge list for a geometry already registered via
// RegisterMesh. Must be called at most once per registration (i.e. between
// RegisterMesh and the matching ReleaseMeshReference that drops refCount to 0).
// `edges` points to `2 * edgeCount` EdgeVertex entries (two endpoints per
// edge, consecutive). Storage-buffer-backed and immutable on the GPU. Caller
// is free to free the input array after this returns. Safe no-op if `h` is
// invalid.
void RegisterMeshEdges( MeshHandle h, const EdgeVertex* edges, int edgeCount, const char* debugLabel );

// Edge data queries used by the edge overlay pass.
bool HasEdges( MeshHandle h );
int GetEdgeCount( MeshHandle h );
sg_view GetEdgeStorageView( MeshHandle h );

// Shape-kind tag, see MeshKind enum above. Safe no-op on invalid handle.
void SetMeshKind( MeshHandle h, MeshKind kind );
MeshKind GetMeshKind( MeshHandle h );

// Direct GPU buffer accessor, used by passes that need to draw a single
// registered geometry by handle (e.g. highlight mask issues one draw
// per selected hull, bypassing the per-frame instance arenas). Writes
// (vbo, ibo, indexCount) into *out and returns true. Returns false +
// leaves *out untouched if the handle is invalid.
typedef struct MeshBuffers
{
	sg_buffer vbo;
	sg_buffer ibo;
	int indexCount;
} MeshBuffers;

bool GetMeshBuffers( MeshHandle h, MeshBuffers* out );

void AddMeshReference( MeshHandle h );
void ReleaseMeshReference( MeshHandle h );

void ResetFrameForMeshes( void );

// Material modes packed into the instance buffer's material.z slot. The
// fragment shader branches on this to either feed baseColor straight into
// the BRDF or run a procedural-grid shader first (used for ground planes
// tagged via the Box3D adapter).
typedef enum MeshMaterialMode
{
	MESH_MATERIAL_MODE_SOLID = 0,
	MESH_MATERIAL_MODE_GROUND_GRID = 1,
} MeshMaterialMode;

void AppendMesh( MeshHandle h, b3Transform transform, b3Vec3 scale, Vec4 baseColor, float metallic, float roughness,
					MeshMaterialMode materialMode, float gridCellSize, TransparentShadowCast shadowCast );

// Pack per-entry instances into the global GPU buffer. Called once per
// frame by the renderer before draw. Returns the total instance count
// (opaque + transparent combined, transparent instances live in a
// separate GPU buffer accessed through GetTransparentMeshInstanceView).
int UploadMeshInstances( void );

int GetMeshDrawSpanCount( void );

MeshDrawSpan GetMeshDrawSpan( int i );

// Mirror spans carry negative-determinant (mirror-scaled) instances. Their
// flipped winding needs the opposite cull face, so the renderer draws them
// with a front-cull pipeline after the back-cull pass over the normal spans.
// Usually empty, only a few sample scenes mirror a mesh.
int GetMeshMirrorDrawSpanCount( void );

MeshDrawSpan GetMeshMirrorDrawSpan( int i );

// Bound to the geom pipeline by the renderer. Exposed so renderer.c can
// bind a single view across all draws without recreating it per span.
sg_view GetMeshInstanceView( void );

// Transparent geom queries. Returns the count of transparent geom
// instances submitted this frame (after UploadMeshInstances) and the
// per-instance metadata for each. The renderer mixes these with cube/
// sphere/capsule transparent instances in its back-to-front sort, then
// issues one sg_draw per entry using inst_base = firstInstance.
int GetTransparentMeshInstanceCount( void );
MeshXpInstance GetTransparentMeshInstance( int i );
sg_view GetTransparentMeshInstanceView( void );

// Per-entry edge-draw batches built during UploadMeshInstances.
// Up to two batches per entry (opaque + xp) when both arenas are non-empty
// and the entry has edges registered. The edge pass iterates these in
// arbitrary order, depth-test handles occlusion.
int GetMeshEdgeBatchCount( void );
MeshEdgeBatch GetMeshEdgeBatch( int i );

// Total registry entry count for diagnostics.
int GetMeshEntryCount( void );

#ifdef __cplusplus
} // extern "C"
#endif
