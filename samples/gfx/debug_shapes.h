// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Box3D -> gfx geometry-registry bridge for shapes with intrinsic
// topology (hulls, triangle meshes, heightfields).
//
// Each Acquire* function checks the registry for an existing entry under
// the Box3D struct's `hash` field (computed by Box3D at construction) and,
// on miss, builds a flat-shaded triangle list and registers it. On hit
// the existing entry's refcount is bumped. Symmetric Release* via the
// returned MeshHandle goes through ReleaseMeshReference directly.
//
// All three builders emit duplicated vertices (one per triangle corner)
// so the per-vertex normal is the face normal, flat shading falls out
// of the rasterizer without any extra shader logic. This costs ~3x the
// vertex memory of a deduplicated mesh, acceptable at this scale.
//
// Builders run from createDebugShape callbacks (Box3D path). They are NOT hot-path,
// running once per distinct geometry per renderer session, so transient build
// buffers use malloc/free rather than the frame arena.

#pragma once

#include "box3d/collision.h"
#include "gfx/geometry_registry.h"

#ifdef __cplusplus
extern "C"
{
#endif

// Acquire a registry handle for the given Box3D hull. Returns an invalid
// handle on registry-full or zero hash. The caller owns one refcount. Pair
// each Acquire with a single ReleaseMeshReference when the consumer goes away.
MeshHandle FindOrAddHull( const b3HullData* hull );

// Acquire for b3MeshData. The b3Mesh struct's per-instance `scale` is NOT
// part of the geometry, callers pass it as the per-instance scale at
// draw time. Two b3Mesh sharing the same b3MeshData share one entry.
MeshHandle FindOrAddMesh( const b3MeshData* meshData );

// Acquire for b3HeightFieldData. The `scale` vector is part of the height
// field's hash (folded into construction), so two heightfields with
// different scales hash to different entries, per-instance scale stays
// at (1, 1, 1) for heightfield draws.
MeshHandle FindOrAddHeightField( const b3HeightFieldData* heightField );

#ifdef __cplusplus
} // extern "C"
#endif
