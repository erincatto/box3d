// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

// b3CollideMoverAndCompound, b3GetCompoundChild, b3QueryCompound are internal
#include "compound.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static b3SurfaceMaterial MakeMaterial( float friction, uint64_t userId )
{
	b3SurfaceMaterial m = b3DefaultSurfaceMaterial();
	m.friction = friction;
	m.userMaterialId = userId;
	return m;
}

static int CompoundCreateMixed( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();

	b3CompoundCapsuleDef capsules[1] = {
		{ .capsule = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 0.25f }, .material = mat },
	};

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3CompoundHullDef hulls[1] = {
		{ .hull = &box.base, .transform = b3Transform_identity, .material = mat },
	};

	b3MeshData* meshData = b3CreateBoxMesh( b3Vec3_zero, (b3Vec3){ 0.5f, 0.5f, 0.5f }, false );
	b3CompoundMeshDef meshes[1] = {
		{
			.meshData = meshData,
			.transform = b3Transform_identity,
			.scale = { 1.0f, 1.0f, 1.0f },
			.materials = &mat,
			.materialCount = 1,
		},
	};

	b3CompoundSphereDef spheres[2] = {
		{ .sphere = { { 5.0f, 0.0f, 0.0f }, 0.5f }, .material = mat },
		{ .sphere = { { -5.0f, 0.0f, 0.0f }, 0.5f }, .material = mat },
	};

	b3CompoundDef def = {
		.capsules = capsules,
		.capsuleCount = 1,
		.hulls = hulls,
		.hullCount = 1,
		.meshes = meshes,
		.meshCount = 1,
		.spheres = spheres,
		.sphereCount = 2,
	};

	b3CompoundData* compound = b3CreateCompound( &def );
	ENSURE( compound != NULL );
	ENSURE( compound->version == B3_COMPOUND_VERSION );
	ENSURE( compound->byteCount > (int)sizeof( b3CompoundData ) );

	ENSURE( compound->capsuleCount == 1 );
	ENSURE( compound->hullCount == 1 );
	ENSURE( compound->meshCount == 1 );
	ENSURE( compound->sphereCount == 2 );

	ENSURE( compound->materialCount == 1 );
	ENSURE( compound->sharedHullCount == 1 );
	ENSURE( compound->sharedMeshCount == 1 );

	ENSURE( compound->tree.nodeCount > 0 );
	ENSURE( compound->tree.nodes != NULL );

	b3DestroyCompound( compound );
	b3DestroyMesh( meshData );
	return 0;
}

static int CompoundCreateSingleType( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();

	// Capsule only
	{
		b3CompoundCapsuleDef cap = { .capsule = { { 0, 0, 0 }, { 1, 0, 0 }, 0.5f }, .material = mat };
		b3CompoundDef def = { .capsules = &cap, .capsuleCount = 1 };
		b3CompoundData* c = b3CreateCompound( &def );
		ENSURE( c != NULL && c->capsuleCount == 1 && c->hullCount == 0 && c->meshCount == 0 && c->sphereCount == 0 );
		b3DestroyCompound( c );
	}

	// Hull only
	{
		b3BoxHull box = b3MakeBoxHull( 1, 1, 1 );
		b3CompoundHullDef h = { .hull = &box.base, .transform = b3Transform_identity, .material = mat };
		b3CompoundDef def = { .hulls = &h, .hullCount = 1 };
		b3CompoundData* c = b3CreateCompound( &def );
		ENSURE( c != NULL && c->hullCount == 1 && c->sharedHullCount == 1 && c->capsuleCount == 0 );
		b3DestroyCompound( c );
	}

	// Mesh only
	{
		b3MeshData* md = b3CreateBoxMesh( b3Vec3_zero, (b3Vec3){ 1, 1, 1 }, false );
		b3CompoundMeshDef m = {
			.meshData = md,
			.transform = b3Transform_identity,
			.scale = { 1, 1, 1 },
			.materials = &mat,
			.materialCount = 1,
		};
		b3CompoundDef def = { .meshes = &m, .meshCount = 1 };
		b3CompoundData* c = b3CreateCompound( &def );
		ENSURE( c != NULL && c->meshCount == 1 && c->sharedMeshCount == 1 );
		b3DestroyCompound( c );
		b3DestroyMesh( md );
	}

	// Sphere only
	{
		b3CompoundSphereDef s = { .sphere = { { 0, 0, 0 }, 1.0f }, .material = mat };
		b3CompoundDef def = { .spheres = &s, .sphereCount = 1 };
		b3CompoundData* c = b3CreateCompound( &def );
		ENSURE( c != NULL && c->sphereCount == 1 );
		b3DestroyCompound( c );
	}

	return 0;
}

static int CompoundMaterialDedup( void )
{
	b3SurfaceMaterial mat = MakeMaterial( 0.4f, 7 );

	b3CompoundCapsuleDef caps[3];
	for ( int i = 0; i < 3; ++i )
	{
		caps[i].capsule = (b3Capsule){ { (float)i, 0, 0 }, { (float)i + 1, 0, 0 }, 0.25f };
		caps[i].material = mat;
	}

	b3CompoundDef def = { .capsules = caps, .capsuleCount = 3 };
	b3CompoundData* c = b3CreateCompound( &def );
	ENSURE( c->materialCount == 1 );
	for ( int i = 0; i < 3; ++i )
	{
		b3CompoundCapsule cc = b3GetCompoundCapsule( c, i );
		ENSURE( cc.materialIndex == 0 );
	}
	b3DestroyCompound( c );
	return 0;
}

static int CompoundMaterialDistinct( void )
{
	b3CompoundCapsuleDef caps[3];
	for ( int i = 0; i < 3; ++i )
	{
		caps[i].capsule = (b3Capsule){ { (float)i, 0, 0 }, { (float)i + 1, 0, 0 }, 0.25f };
		caps[i].material = MakeMaterial( 0.1f * (float)( i + 1 ), (uint64_t)( i + 1 ) );
	}

	b3CompoundDef def = { .capsules = caps, .capsuleCount = 3 };
	b3CompoundData* c = b3CreateCompound( &def );
	ENSURE( c->materialCount == 3 );

	const b3SurfaceMaterial* mats = b3GetCompoundMaterials( c );
	ENSURE( mats != NULL );

	for ( int i = 0; i < 3; ++i )
	{
		b3CompoundCapsule cc = b3GetCompoundCapsule( c, i );
		ENSURE( cc.materialIndex >= 0 && cc.materialIndex < 3 );
		ENSURE( mats[cc.materialIndex].userMaterialId == (uint64_t)( i + 1 ) );
	}

	b3DestroyCompound( c );
	return 0;
}

static int CompoundMaterialCrossShape( void )
{
	// One material shared across capsule, hull, and sphere -> 1 material slot.
	b3SurfaceMaterial mat = MakeMaterial( 0.5f, 99 );

	b3CompoundCapsuleDef cap = { .capsule = { { 0, 0, 0 }, { 1, 0, 0 }, 0.25f }, .material = mat };
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3CompoundHullDef hull = { .hull = &box.base, .transform = b3Transform_identity, .material = mat };
	b3CompoundSphereDef sph = { .sphere = { { 5, 0, 0 }, 0.5f }, .material = mat };

	b3CompoundDef def = {
		.capsules = &cap,
		.capsuleCount = 1,
		.hulls = &hull,
		.hullCount = 1,
		.spheres = &sph,
		.sphereCount = 1,
	};
	b3CompoundData* c = b3CreateCompound( &def );
	ENSURE( c->materialCount == 1 );

	ENSURE( b3GetCompoundCapsule( c, 0 ).materialIndex == 0 );
	ENSURE( b3GetCompoundHull( c, 0 ).materialIndex == 0 );
	ENSURE( b3GetCompoundSphere( c, 0 ).materialIndex == 0 );

	b3DestroyCompound( c );
	return 0;
}

static int CompoundMaterialMeshShared( void )
{
	// Mesh material entries are routed through the same material map as convex
	// materials, so an identical material is deduped across mesh and convex.
	// (The comment in compound.c about meshes "not being shared" refers to the
	// per-instance materialIndices arrays, not the b3SurfaceMaterial table.)
	b3SurfaceMaterial mat = MakeMaterial( 0.3f, 11 );

	b3MeshData* md = b3CreateBoxMesh( b3Vec3_zero, (b3Vec3){ 1, 1, 1 }, false );
	ENSURE( md->materialCount == 1 );

	b3CompoundSphereDef sph = { .sphere = { { 5, 0, 0 }, 0.5f }, .material = mat };
	b3CompoundMeshDef mesh = {
		.meshData = md,
		.transform = b3Transform_identity,
		.scale = { 1, 1, 1 },
		.materials = &mat,
		.materialCount = 1,
	};

	b3CompoundDef def = {
		.meshes = &mesh,
		.meshCount = 1,
		.spheres = &sph,
		.sphereCount = 1,
	};
	b3CompoundData* c = b3CreateCompound( &def );
	ENSURE( c->materialCount == 1 );

	b3DestroyCompound( c );
	b3DestroyMesh( md );
	return 0;
}

// ---------------------------------------------------------------------------
// Hull / mesh blob sharing
// ---------------------------------------------------------------------------

static int CompoundHullSharingPointer( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
	b3BoxHull box = b3MakeBoxHull( 1, 1, 1 );

	b3CompoundHullDef hulls[3];
	for ( int i = 0; i < 3; ++i )
	{
		hulls[i].hull = &box.base;
		hulls[i].transform = b3Transform_identity;
		hulls[i].transform.p.x = (float)( 4 * i );
		hulls[i].material = mat;
	}

	b3CompoundDef def = { .hulls = hulls, .hullCount = 3 };
	b3CompoundData* c = b3CreateCompound( &def );
	ENSURE( c->hullCount == 3 );
	ENSURE( c->sharedHullCount == 1 );
	b3DestroyCompound( c );
	return 0;
}

static int CompoundHullSharingContent( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();

	// Two box hulls built independently with identical args are byte-identical
	// (b3MakeBoxHull is deterministic and the hash is computed over the bytes).
	b3BoxHull boxA = b3MakeBoxHull( 1, 1, 1 );
	b3BoxHull boxB = b3MakeBoxHull( 1, 1, 1 );
	ENSURE( &boxA != &boxB );

	b3CompoundHullDef hulls[2] = {
		{ .hull = &boxA.base, .transform = b3Transform_identity, .material = mat },
		{ .hull = &boxB.base, .transform = b3Transform_identity, .material = mat },
	};
	hulls[1].transform.p.x = 5.0f;

	b3CompoundDef def = { .hulls = hulls, .hullCount = 2 };
	b3CompoundData* c = b3CreateCompound( &def );
	ENSURE( c->sharedHullCount == 1 );
	b3DestroyCompound( c );
	return 0;
}

static int CompoundHullDistinct( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
	b3BoxHull boxA = b3MakeBoxHull( 1, 1, 1 );
	b3BoxHull boxB = b3MakeBoxHull( 2, 1, 1 );

	b3CompoundHullDef hulls[2] = {
		{ .hull = &boxA.base, .transform = b3Transform_identity, .material = mat },
		{ .hull = &boxB.base, .transform = b3Transform_identity, .material = mat },
	};
	hulls[1].transform.p.x = 5.0f;

	b3CompoundDef def = { .hulls = hulls, .hullCount = 2 };
	b3CompoundData* c = b3CreateCompound( &def );
	ENSURE( c->sharedHullCount == 2 );
	b3DestroyCompound( c );
	return 0;
}

static int CompoundMeshSharingPointer( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
	b3MeshData* md = b3CreateBoxMesh( b3Vec3_zero, (b3Vec3){ 1, 1, 1 }, false );

	b3CompoundMeshDef meshes[3];
	for ( int i = 0; i < 3; ++i )
	{
		meshes[i].meshData = md;
		meshes[i].transform = b3Transform_identity;
		meshes[i].transform.p.x = (float)( 4 * i );
		meshes[i].scale = (b3Vec3){ 1, 1, 1 };
		meshes[i].materials = &mat;
		meshes[i].materialCount = 1;
	}

	b3CompoundDef def = { .meshes = meshes, .meshCount = 3 };
	b3CompoundData* c = b3CreateCompound( &def );
	ENSURE( c->meshCount == 3 );
	ENSURE( c->sharedMeshCount == 1 );

	b3DestroyCompound( c );
	b3DestroyMesh( md );
	return 0;
}

static int CompoundMeshSharingContent( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
	b3MeshData* mdA = b3CreateBoxMesh( b3Vec3_zero, (b3Vec3){ 1, 1, 1 }, false );
	b3MeshData* mdB = b3CreateBoxMesh( b3Vec3_zero, (b3Vec3){ 1, 1, 1 }, false );
	ENSURE( mdA != mdB );

	b3CompoundMeshDef meshes[2] = {
		{ .meshData = mdA, .transform = b3Transform_identity, .scale = { 1, 1, 1 }, .materials = &mat, .materialCount = 1 },
		{ .meshData = mdB, .transform = b3Transform_identity, .scale = { 1, 1, 1 }, .materials = &mat, .materialCount = 1 },
	};
	meshes[1].transform.p.x = 5.0f;

	b3CompoundDef def = { .meshes = meshes, .meshCount = 2 };
	b3CompoundData* c = b3CreateCompound( &def );
	ENSURE( c->sharedMeshCount == 1 );

	b3DestroyCompound( c );
	b3DestroyMesh( mdA );
	b3DestroyMesh( mdB );
	return 0;
}

static int CompoundMeshDistinct( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
	b3MeshData* mdA = b3CreateBoxMesh( b3Vec3_zero, (b3Vec3){ 1, 1, 1 }, false );
	b3MeshData* mdB = b3CreateBoxMesh( b3Vec3_zero, (b3Vec3){ 2, 1, 1 }, false );

	b3CompoundMeshDef meshes[2] = {
		{ .meshData = mdA, .transform = b3Transform_identity, .scale = { 1, 1, 1 }, .materials = &mat, .materialCount = 1 },
		{ .meshData = mdB, .transform = b3Transform_identity, .scale = { 1, 1, 1 }, .materials = &mat, .materialCount = 1 },
	};
	meshes[1].transform.p.x = 5.0f;

	b3CompoundDef def = { .meshes = meshes, .meshCount = 2 };
	b3CompoundData* c = b3CreateCompound( &def );
	ENSURE( c->sharedMeshCount == 2 );

	b3DestroyCompound( c );
	b3DestroyMesh( mdA );
	b3DestroyMesh( mdB );
	return 0;
}

static int CompoundChildDispatch( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3MeshData* md = b3CreateBoxMesh( b3Vec3_zero, (b3Vec3){ 0.5f, 0.5f, 0.5f }, false );

	b3CompoundCapsuleDef caps[2] = {
		{ .capsule = { { 0, 0, 0 }, { 1, 0, 0 }, 0.2f }, .material = mat },
		{ .capsule = { { 0, 2, 0 }, { 1, 2, 0 }, 0.2f }, .material = mat },
	};
	b3CompoundHullDef hulls[1] = { { .hull = &box.base, .transform = b3Transform_identity, .material = mat } };
	hulls[0].transform.p = (b3Vec3){ 5, 0, 0 };
	b3CompoundMeshDef meshes[1] = { {
		.meshData = md,
		.transform = b3Transform_identity,
		.scale = { 1, 1, 1 },
		.materials = &mat,
		.materialCount = 1,
	} };
	meshes[0].transform.p = (b3Vec3){ 0, 0, 5 };
	b3CompoundSphereDef spheres[1] = { { .sphere = { { -5, 0, 0 }, 0.5f }, .material = mat } };

	b3CompoundDef def = {
		.capsules = caps,
		.capsuleCount = 2,
		.hulls = hulls,
		.hullCount = 1,
		.meshes = meshes,
		.meshCount = 1,
		.spheres = spheres,
		.sphereCount = 1,
	};
	b3CompoundData* c = b3CreateCompound( &def );

	// Index ordering is capsules -> hulls -> meshes -> spheres.
	ENSURE( b3GetCompoundChild( c, 0 ).type == b3_capsuleShape );
	ENSURE( b3GetCompoundChild( c, 1 ).type == b3_capsuleShape );
	ENSURE( b3GetCompoundChild( c, 2 ).type == b3_hullShape );
	ENSURE( b3GetCompoundChild( c, 3 ).type == b3_meshShape );
	ENSURE( b3GetCompoundChild( c, 4 ).type == b3_sphereShape );

	// Capsule and sphere children always report identity transform. The position
	// is encoded in the shape itself (capsule->center{1,2}, sphere->center).
	b3ChildShape cap0 = b3GetCompoundChild( c, 0 );
	ENSURE_SMALL( cap0.transform.p.x, FLT_EPSILON );
	ENSURE_SMALL( cap0.transform.p.y, FLT_EPSILON );
	ENSURE_SMALL( cap0.transform.p.z, FLT_EPSILON );
	ENSURE_SMALL( cap0.capsule.center2.x - 1.0f, FLT_EPSILON );

	b3ChildShape sph = b3GetCompoundChild( c, 4 );
	ENSURE_SMALL( sph.transform.p.x, FLT_EPSILON );
	ENSURE_SMALL( sph.sphere.center.x + 5.0f, FLT_EPSILON );

	// Hull and mesh children carry their stored transform.
	b3ChildShape hull = b3GetCompoundChild( c, 2 );
	ENSURE_SMALL( hull.transform.p.x - 5.0f, FLT_EPSILON );

	b3ChildShape mesh = b3GetCompoundChild( c, 3 );
	ENSURE_SMALL( mesh.transform.p.z - 5.0f, FLT_EPSILON );
	ENSURE( mesh.mesh.data != NULL );

	b3DestroyCompound( c );
	b3DestroyMesh( md );
	return 0;
}

static int CompoundAABBContainsChildren( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
	b3CompoundSphereDef spheres[2] = {
		{ .sphere = { { -3, 0, 0 }, 1.0f }, .material = mat },
		{ .sphere = { { 4, 0, 0 }, 0.5f }, .material = mat },
	};
	b3CompoundDef def = { .spheres = spheres, .sphereCount = 2 };
	b3CompoundData* c = b3CreateCompound( &def );

	b3AABB local = b3ComputeCompoundAABB( c, b3Transform_identity );
	ENSURE( local.lowerBound.x <= -4.0f + 1e-5f );
	ENSURE( local.upperBound.x >= 4.5f - 1e-5f );
	ENSURE( local.lowerBound.y <= -1.0f + 1e-5f );
	ENSURE( local.upperBound.y >= 1.0f - 1e-5f );

	// Translation commutes through the bounding-box transform.
	b3Transform xf = { .p = { 10, 20, 30 }, .q = b3Quat_identity };
	b3AABB world = b3ComputeCompoundAABB( c, xf );
	ENSURE_SMALL( world.lowerBound.x - ( local.lowerBound.x + 10.0f ), 1e-4f );
	ENSURE_SMALL( world.upperBound.y - ( local.upperBound.y + 20.0f ), 1e-4f );
	ENSURE_SMALL( world.lowerBound.z - ( local.lowerBound.z + 30.0f ), 1e-4f );

	b3DestroyCompound( c );
	return 0;
}

static int CompoundRayCastMiss( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
	b3CompoundSphereDef sph = { .sphere = { { 0, 0, 0 }, 0.5f }, .material = mat };
	b3CompoundDef def = { .spheres = &sph, .sphereCount = 1 };
	b3CompoundData* c = b3CreateCompound( &def );

	// Ray well above the sphere on a parallel path.
	b3RayCastInput input = { .origin = { -5, 5, 0 }, .translation = { 10, 0, 0 }, .maxFraction = 1.0f };
	b3CastOutput out = b3RayCastCompound( c, &input );
	ENSURE( out.hit == false );

	b3DestroyCompound( c );
	return 0;
}

static int CompoundRayCastClosest( void )
{
	b3SurfaceMaterial matA = MakeMaterial( 0.4f, 100 );
	b3SurfaceMaterial matB = MakeMaterial( 0.4f, 200 );

	// Two unit spheres along +X. Ray from origin must hit the nearer one first.
	b3CompoundSphereDef spheres[2] = {
		{ .sphere = { { 5, 0, 0 }, 1.0f }, .material = matA },
		{ .sphere = { { 10, 0, 0 }, 1.0f }, .material = matB },
	};
	b3CompoundDef def = { .spheres = spheres, .sphereCount = 2 };
	b3CompoundData* c = b3CreateCompound( &def );

	b3RayCastInput input = { .origin = { 0, 0, 0 }, .translation = { 20, 0, 0 }, .maxFraction = 1.0f };
	b3CastOutput out = b3RayCastCompound( c, &input );
	ENSURE( out.hit == true );

	// Front face of the nearer sphere is at x=4 -> fraction 4/20 = 0.2.
	ENSURE_SMALL( out.fraction - 0.2f, 1e-4f );
	ENSURE_SMALL( out.normal.x + 1.0f, 1e-4f );
	ENSURE( out.childIndex == 0 );

	const b3SurfaceMaterial* mats = b3GetCompoundMaterials( c );
	ENSURE( mats[out.materialIndex].userMaterialId == 100 );

	b3DestroyCompound( c );
	return 0;
}

static int CompoundRayCastHullNormalRotation( void )
{
	// A unit box rotated 90 degrees about Z, placed at compound +X. The ray hits the
	// face that, in compound space, points back toward -X. Verifies that the
	// normal returned by the cast has been rotated from hull-local space back
	// into compound space.
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
	b3BoxHull box = b3MakeBoxHull( 1, 1, 1 );

	b3CompoundHullDef hull = {
		.hull = &box.base,
		.transform = { .p = { 5, 0, 0 }, .q = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI ) },
		.material = mat,
	};
	b3CompoundDef def = { .hulls = &hull, .hullCount = 1 };
	b3CompoundData* c = b3CreateCompound( &def );

	b3RayCastInput input = { .origin = { 0, 0, 0 }, .translation = { 20, 0, 0 }, .maxFraction = 1.0f };
	b3CastOutput out = b3RayCastCompound( c, &input );
	ENSURE( out.hit == true );
	// Box has |hx|=1; with the rotation a face still intersects the +X ray at x=4.
	ENSURE_SMALL( out.fraction - 0.2f, 1e-4f );
	ENSURE_SMALL( out.normal.x + 1.0f, 1e-3f );
	ENSURE_SMALL( out.normal.y, 1e-3f );
	ENSURE_SMALL( out.normal.z, 1e-3f );

	b3DestroyCompound( c );
	return 0;
}

static int CompoundShapeCastClosest( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
	b3CompoundSphereDef spheres[2] = {
		{ .sphere = { { 5, 0, 0 }, 1.0f }, .material = mat },
		{ .sphere = { { 10, 0, 0 }, 1.0f }, .material = mat },
	};
	b3CompoundDef def = { .spheres = spheres, .sphereCount = 2 };
	b3CompoundData* c = b3CreateCompound( &def );

	b3Vec3 point = { 0, 0, 0 };
	b3ShapeCastInput input = {
		.proxy = { .points = &point, .count = 1, .radius = 0.25f },
		.translation = { 20, 0, 0 },
		.maxFraction = 1.0f,
		.canEncroach = false,
	};
	b3CastOutput out = b3ShapeCastCompound( c, &input );
	ENSURE( out.hit == true );
	// Closest contact: caster radius 0.25 + sphere radius 1.0 -> first contact at x ~= 3.75.
	ENSURE_SMALL( out.fraction - 3.75f / 20.0f, 1e-3f );
	ENSURE( out.childIndex == 0 );

	b3DestroyCompound( c );
	return 0;
}

static int CompoundOverlap( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
	b3CompoundSphereDef spheres[2] = {
		{ .sphere = { { -3, 0, 0 }, 0.5f }, .material = mat },
		{ .sphere = { { 3, 0, 0 }, 0.5f }, .material = mat },
	};
	b3CompoundDef def = { .spheres = spheres, .sphereCount = 2 };
	b3CompoundData* c = b3CreateCompound( &def );

	// Proxy at the origin lies in the gap between the two spheres.
	b3Vec3 origin = { 0, 0, 0 };
	b3ShapeProxy gap = { .points = &origin, .count = 1, .radius = 0.25f };
	ENSURE( b3OverlapCompound( c, b3Transform_identity, &gap ) == false );

	// Proxy at the center of the second sphere overlaps it.
	b3Vec3 onSecond = { 3, 0, 0 };
	b3ShapeProxy hit = { .points = &onSecond, .count = 1, .radius = 0.1f };
	ENSURE( b3OverlapCompound( c, b3Transform_identity, &hit ) == true );

	b3DestroyCompound( c );
	return 0;
}

struct QueryAccumulator
{
	int childIndices[8];
	int count;
	int stopAfter; // -1 = never stop early
};

static bool QueryCallback( const b3CompoundData* compound, int childIndex, void* context )
{
	(void)compound;
	struct QueryAccumulator* acc = context;
	if ( acc->count < (int)( sizeof( acc->childIndices ) / sizeof( acc->childIndices[0] ) ) )
	{
		acc->childIndices[acc->count++] = childIndex;
	}
	if ( acc->stopAfter >= 0 && acc->count >= acc->stopAfter )
	{
		return false;
	}
	return true;
}

static int CompoundQuery( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
	b3CompoundSphereDef spheres[3] = {
		{ .sphere = { { -10, 0, 0 }, 0.5f }, .material = mat },
		{ .sphere = { { 0, 0, 0 }, 0.5f }, .material = mat },
		{ .sphere = { { 10, 0, 0 }, 0.5f }, .material = mat },
	};
	b3CompoundDef def = { .spheres = spheres, .sphereCount = 3 };
	b3CompoundData* c = b3CreateCompound( &def );

	// Tight box around the middle sphere. Only it should be reported.
	b3AABB middle = { { -1, -1, -1 }, { 1, 1, 1 } };
	struct QueryAccumulator acc = { .stopAfter = -1 };
	b3QueryCompound( c, middle, QueryCallback, &acc );
	ENSURE( acc.count == 1 );
	ENSURE( acc.childIndices[0] == 1 );

	// Wide box overlapping all three; early-exit after the first reported child.
	b3AABB wide = { { -20, -1, -1 }, { 20, 1, 1 } };
	struct QueryAccumulator stop = { .stopAfter = 1 };
	b3QueryCompound( c, wide, QueryCallback, &stop );
	ENSURE( stop.count == 1 );

	// Without early-exit, all three are visited.
	struct QueryAccumulator all = { .stopAfter = -1 };
	b3QueryCompound( c, wide, QueryCallback, &all );
	ENSURE( all.count == 3 );

	b3DestroyCompound( c );
	return 0;
}

static int CompoundMover( void )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	// Two boxes side-by-side along X, gap of 1 between them.
	b3CompoundHullDef hulls[2] = {
		{ .hull = &box.base, .transform = { .p = { -1.0f, 0, 0 }, .q = b3Quat_identity }, .material = mat },
		{ .hull = &box.base, .transform = { .p = { 1.0f, 0, 0 }, .q = b3Quat_identity }, .material = mat },
	};
	b3CompoundDef def = { .hulls = hulls, .hullCount = 2 };
	b3CompoundData* c = b3CreateCompound( &def );

	// Small capsule mover sitting on top of the boxes, low enough to penetrate both.
	b3Capsule mover = { { -1.0f, 0.6f, 0 }, { 1.0f, 0.6f, 0 }, 0.2f };

	b3PlaneResult planes[8] = { 0 };
	int planeCount = b3CollideMoverAndCompound( planes, 8, c, &mover );

	// Both boxes contribute at least one plane each; the +Y face of each box
	// should produce a plane whose normal points roughly +Y in compound space.
	ENSURE( planeCount >= 2 );

	int upPlanes = 0;
	for ( int i = 0; i < planeCount; ++i )
	{
		if ( planes[i].plane.normal.y > 0.9f )
		{
			upPlanes += 1;
		}
	}
	ENSURE( upPlanes >= 2 );

	// Capacity cap is honored: ask for 1 and the call must not exceed it.
	b3PlaneResult one[1] = { 0 };
	int capped = b3CollideMoverAndCompound( one, 1, c, &mover );
	ENSURE( capped <= 1 );

	b3DestroyCompound( c );
	return 0;
}

static b3CompoundData* BuildSerializableCompound( b3MeshData* md )
{
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	b3CompoundCapsuleDef cap = { .capsule = { { -2, 0, 0 }, { -1, 0, 0 }, 0.2f }, .material = mat };
	b3CompoundHullDef hull = { .hull = &box.base, .transform = { .p = { 5, 0, 0 }, .q = b3Quat_identity }, .material = mat };
	b3CompoundMeshDef mesh = {
		.meshData = md,
		.transform = { .p = { 0, 0, 5 }, .q = b3Quat_identity },
		.scale = { 1, 1, 1 },
		.materials = &mat,
		.materialCount = 1,
	};
	b3CompoundSphereDef sph = { .sphere = { { -5, 0, 0 }, 0.5f }, .material = mat };

	b3CompoundDef def = {
		.capsules = &cap,
		.capsuleCount = 1,
		.hulls = &hull,
		.hullCount = 1,
		.meshes = &mesh,
		.meshCount = 1,
		.spheres = &sph,
		.sphereCount = 1,
	};
	return b3CreateCompound( &def );
}

static int CompoundSerializeRoundtrip( void )
{
	b3MeshData* md = b3CreateBoxMesh( b3Vec3_zero, (b3Vec3){ 0.5f, 0.5f, 0.5f }, false );
	b3CompoundData* a = BuildSerializableCompound( md );

	// Snapshot a few queries on the original.
	b3AABB aabbA = b3ComputeCompoundAABB( a, b3Transform_identity );
	b3RayCastInput rayInput = { .origin = { 0, 0, 0 }, .translation = { 20, 0, 0 }, .maxFraction = 1.0f };
	b3CastOutput rayA = b3RayCastCompound( a, &rayInput );
	ENSURE( rayA.hit );

	int byteCount = a->byteCount;
	uint8_t* buffer = (uint8_t*)malloc( (size_t)byteCount );
	ENSURE( buffer != NULL );

	uint8_t* serialized = b3ConvertCompoundToBytes( a );
	memcpy( buffer, serialized, (size_t)byteCount );
	b3DestroyCompound( a );

	b3CompoundData* b = b3ConvertBytesToCompound( buffer, byteCount );
	ENSURE( b != NULL );
	ENSURE( b->byteCount == byteCount );
	ENSURE( b->capsuleCount == 1 );
	ENSURE( b->hullCount == 1 );
	ENSURE( b->meshCount == 1 );
	ENSURE( b->sphereCount == 1 );
	ENSURE( b->tree.nodes != NULL );

	// Bounds and ray cast on the deserialized compound match the original.
	b3AABB aabbB = b3ComputeCompoundAABB( b, b3Transform_identity );
	ENSURE_SMALL( aabbA.lowerBound.x - aabbB.lowerBound.x, 1e-5f );
	ENSURE_SMALL( aabbA.upperBound.z - aabbB.upperBound.z, 1e-5f );

	b3CastOutput rayB = b3RayCastCompound( b, &rayInput );
	ENSURE( rayB.hit == rayA.hit );
	ENSURE_SMALL( rayB.fraction - rayA.fraction, 1e-5f );
	ENSURE( rayB.childIndex == rayA.childIndex );

	free( buffer );
	b3DestroyMesh( md );
	return 0;
}

static int CompoundSerializeBadVersion( void )
{
	b3MeshData* md = b3CreateBoxMesh( b3Vec3_zero, (b3Vec3){ 0.5f, 0.5f, 0.5f }, false );
	b3CompoundData* a = BuildSerializableCompound( md );

	int byteCount = a->byteCount;
	uint8_t* buffer = (uint8_t*)malloc( (size_t)byteCount );
	memcpy( buffer, b3ConvertCompoundToBytes( a ), (size_t)byteCount );
	b3DestroyCompound( a );

	// Corrupt the version word (first 8 bytes of b3Compound).
	( (b3CompoundData*)buffer )->version ^= 0x1ull;

	b3CompoundData* b = b3ConvertBytesToCompound( buffer, byteCount );
	ENSURE( b == NULL );

	free( buffer );
	b3DestroyMesh( md );
	return 0;
}

static int CompoundSerializeWrongByteCount( void )
{
	b3MeshData* md = b3CreateBoxMesh( b3Vec3_zero, (b3Vec3){ 0.5f, 0.5f, 0.5f }, false );
	b3CompoundData* a = BuildSerializableCompound( md );

	int byteCount = a->byteCount;
	uint8_t* buffer = (uint8_t*)malloc( (size_t)byteCount );
	memcpy( buffer, b3ConvertCompoundToBytes( a ), (size_t)byteCount );
	b3DestroyCompound( a );

	// Off-by-one in the declared length is rejected.
	b3CompoundData* b = b3ConvertBytesToCompound( buffer, byteCount + 1 );
	ENSURE( b == NULL );

	free( buffer );
	b3DestroyMesh( md );
	return 0;
}

int CompoundTest( void )
{
	RUN_SUBTEST( CompoundCreateMixed );
	RUN_SUBTEST( CompoundCreateSingleType );

	RUN_SUBTEST( CompoundMaterialDedup );
	RUN_SUBTEST( CompoundMaterialDistinct );
	RUN_SUBTEST( CompoundMaterialCrossShape );
	RUN_SUBTEST( CompoundMaterialMeshShared );

	RUN_SUBTEST( CompoundHullSharingPointer );
	RUN_SUBTEST( CompoundHullSharingContent );
	RUN_SUBTEST( CompoundHullDistinct );
	RUN_SUBTEST( CompoundMeshSharingPointer );
	RUN_SUBTEST( CompoundMeshSharingContent );
	RUN_SUBTEST( CompoundMeshDistinct );

	RUN_SUBTEST( CompoundChildDispatch );

	RUN_SUBTEST( CompoundAABBContainsChildren );

	RUN_SUBTEST( CompoundRayCastMiss );
	RUN_SUBTEST( CompoundRayCastClosest );
	RUN_SUBTEST( CompoundRayCastHullNormalRotation );
	RUN_SUBTEST( CompoundShapeCastClosest );

	RUN_SUBTEST( CompoundOverlap );
	RUN_SUBTEST( CompoundQuery );

	RUN_SUBTEST( CompoundMover );

	RUN_SUBTEST( CompoundSerializeRoundtrip );
	RUN_SUBTEST( CompoundSerializeBadVersion );
	RUN_SUBTEST( CompoundSerializeWrongByteCount );

	return 0;
}
