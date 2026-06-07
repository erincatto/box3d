# Compound Shapes

A compound is a single immutable shape that aggregates many child primitives —
spheres, capsules, convex hulls, and triangle meshes — under one internal AABB
tree. To the simulation it appears as one shape, but collision queries descend
the internal tree and test individual children, so the cost scales with what is
actually touched rather than the total child count.

## Design Intent

Compounds are **static-body only** and **immutable after creation**. They are
not a runtime mutation primitive. The intended workflow is:

1. Author geometry offline (level mesh, terrain tile, building shell).
2. Bake the compound once with `b3CreateCompound`.
3. Serialize it to a byte buffer with `b3ConvertCompoundToBytes` and store it on
   disk or in a streaming cache.
4. At runtime, load the bytes and reconstruct the compound with
   `b3ConvertBytesToCompound`, then attach it to a static body.

This makes compounds well-suited to open-world streaming: tiles are baked
offline, kept in memory as flat byte buffers (which the engine uses directly
without copying), and attached to static bodies when a region loads, then
detached when it unloads.

Do not use a compound on dynamic or kinematic bodies — `b3CreateCompoundShape`
enforces static-only attachment.

## Building a Compound

Fill a `b3CompoundDef` with arrays of child-shape descriptors:

```c
// Child type descriptors
b3CompoundCapsuleDef capsuleDefs[N_CAPSULES];
b3CompoundHullDef    hullDefs[N_HULLS];
b3CompoundMeshDef    meshDefs[N_MESHES];
b3CompoundSphereDef  sphereDefs[N_SPHERES];

// Fill capsuleDefs[i].capsule, .material ...
// Fill hullDefs[i].hull, .transform, .material ...
// Fill meshDefs[i].meshData, .transform, .scale, .materials, .materialCount ...
// Fill sphereDefs[i].sphere, .material ...

b3CompoundDef def;
def.capsules     = capsuleDefs;   def.capsuleCount  = N_CAPSULES;
def.hulls        = hullDefs;      def.hullCount     = N_HULLS;
def.meshes       = meshDefs;      def.meshCount     = N_MESHES;
def.spheres      = sphereDefs;    def.sphereCount   = N_SPHERES;

b3Compound* compound = b3CreateCompound(&def);
```

`b3CreateCompound` clones all input data into the compound. The source arrays
can be freed immediately after the call.

Mesh children share the mesh pointer rather than cloning triangle data — the
`b3MeshData` must remain valid for the lifetime of the compound. Triangle
materials are limited to `B3_MAX_COMPOUND_MESH_MATERIALS` (4) slots per mesh
child; if your mesh needs more materials, attach it as a standalone mesh shape
on the static body instead.

## The b3Compound Structure

```c
typedef struct b3Compound {
    uint64_t version;   // versioned against the tree/mesh/hull formats
    int      byteCount; // total size when serialized
    // internal tree, child arrays, material table
    // ... (treat as opaque)
} b3Compound;
```

`version` is a compile-time constant (`B3_COMPOUND_VERSION`) that incorporates
the dynamic tree, mesh, and hull format versions. A version mismatch at load
time means the bytes were baked with a different engine version and cannot be
used.

## Serialization

Convert a live compound to a self-contained byte buffer:

```c
uint8_t* bytes = b3ConvertCompoundToBytes(compound);
// compound is now in an unusable state; its pointers have been nullified.
// Write bytes[0 .. compound->byteCount - 1] to disk.
```

Reconstruct from bytes at runtime (zero-copy; bytes must remain in scope):

```c
b3Compound* compound = b3ConvertBytesToCompound(bytes, byteCount);
```

The bytes are mutated in place to fixup internal pointers. Multiple static
bodies can use the same byte buffer simultaneously (instancing), since the
compound itself holds no per-body state.

## Attaching to a Static Body

```c
b3BodyDef bodyDef = b3DefaultBodyDef();
bodyDef.type = b3_staticBody;
bodyDef.position = tileOrigin;
b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);

b3ShapeDef shapeDef = b3DefaultShapeDef();
b3ShapeId shapeId = b3CreateCompoundShape(bodyId, &shapeDef, compound);
```

`b3CreateCompoundShape` asserts that the body is static.

## Querying a Compound

Compute the world AABB:

```c
b3AABB aabb = b3ComputeCompoundAABB(compound, bodyTransform);
```

Query child shapes by AABB:

```c
typedef bool b3CompoundQueryFcn(const b3Compound* compound, int childIndex, void* context);

b3QueryCompound(compound, queryAABB, MyChildCallback, ctx);
```

Access a specific child by index:

```c
b3ChildShape child = b3GetCompoundChild(compound, childIndex);
// child.type tells you capsule / hull / mesh / sphere
// child.transform is the child's local transform within the compound
```

Individual typed accessors are also available:

```c
b3CompoundCapsule cc = b3GetCompoundCapsule(compound, i);
b3CompoundHull    ch = b3GetCompoundHull(compound, i);
b3CompoundMesh    cm = b3GetCompoundMesh(compound, i);
b3CompoundSphere  cs = b3GetCompoundSphere(compound, i);
const b3SurfaceMaterial* mats = b3GetCompoundMaterials(compound);
```

## Teardown

Destroy a live compound (not needed if you are using the byte-buffer path, where
you manage the buffer lifetime yourself):

```c
b3DestroyCompound(compound);
```

Do not call `b3DestroyCompound` on a compound reconstructed from bytes with
`b3ConvertBytesToCompound` — free the byte buffer instead.
