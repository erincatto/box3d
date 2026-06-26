// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/debug_adapter.h"

#include "gfx/debug_shapes.h"
#include "gfx/draw.h"
#include "gfx/geometry_registry.h"
#include "gfx/highlight_mask.h"
#include "gfx/overlay.h"
#include "gfx/renderer.h"
#include "gfx/text.h"

#include "box3d/box3d.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define BOX3D_USER_SHAPE_CAPACITY 65536
#define BOX3D_FREELIST_END ( -1 )
#define BOX3D_MATERIAL_OVERRIDE_CAPACITY 256

typedef enum
{
	Box3DUS_Free = 0,
	Box3DUS_Sphere,
	Box3DUS_Capsule,
	Box3DUS_Hull,
	Box3DUS_Mesh,
	Box3DUS_HeightField,
	Box3DUS_Compound,
} DebugShapeKind;

typedef struct
{
	b3Vec3 center;
	float radius;
} DebugSphere;

typedef struct
{
	b3Transform localFrame;
	float halfLength;
	float radius;
} DebugCapsule;

// Hull/mesh/heightfield. Scale only for mesh.
typedef struct
{
	MeshHandle handle;
	b3Vec3 scale;
} DebugMesh;

typedef struct
{
	DebugShapeKind kind;
	// valid only when kind == Box3DUS_Free
	int nextFree;
	b3BodyType bodyType;
	bool isGround;
	b3ShapeId shapeId;
	b3BodyId bodyId;
	bool hasMaterial;
	Vec4 color;
	float metallic;
	float roughness;

	// Compound children are flattened into their own pool slots. A child links
	// to its sibling through nextChild and is placed by childTransform under the
	// owning body transform. Identity transform and no sibling for top-level shapes.
	int nextChild;
	b3Transform childTransform;
	union
	{
		DebugSphere sphere;
		DebugCapsule capsule;
		DebugMesh geom;
		struct
		{
			int firstChild;
		} compound;
	};
} DebugShape;

typedef struct
{
	b3ShapeId shapeId;
	Vec4 color;
	float metallic;
	float roughness;
} MaterialOverride;

typedef struct
{
	DebugShape pool[BOX3D_USER_SHAPE_CAPACITY];
	int firstFree;
	int allocCount;

	// For the ground grid
	b3ShapeId groundShapeId;

	// Per-shape PBR overrides
	MaterialOverride materialOverrides[BOX3D_MATERIAL_OVERRIDE_CAPACITY];
	int materialOverrideCount;

	b3BodyId selectedBodyId;
	b3ShapeId selectedShapeId;

	// Function pointers, drawingBounds, and context inside this struct are NOT used.
	// Only used for settings
	b3DebugDraw guiDraw;

	bool transparentDynamic;
} AdapterState;

static AdapterState s_adapter;

// Per-bodyType PBR material defaults. Indexed by b3BodyType
// (b3_staticBody = 0, b3_kinematicBody = 1, b3_dynamicBody = 2).
static const float kBodyTypeMetallic[3] = { 0.0f, 0.0f, 0.0f };
static const float kBodyTypeRoughness[3] = { 0.70f, 0.55f, 0.40f };

// PBR presets packed into the debug color high byte. Indexed by b3DebugMaterial.
// Default (0) is unused here, the caller falls back to the per-bodyType table.
//                                                  default matte soft dead glossy metal
static const float kDebugMaterialMetallic[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.85f };
static const float kDebugMaterialRoughness[6] = { 0.50f, 0.85f, 0.65f, 0.95f, 0.30f, 0.35f };

void InitAdapter( void )
{
	for ( int i = 0; i < BOX3D_USER_SHAPE_CAPACITY; ++i )
	{
		s_adapter.pool[i].kind = Box3DUS_Free;
		s_adapter.pool[i].nextFree = ( i + 1 < BOX3D_USER_SHAPE_CAPACITY ) ? ( i + 1 ) : BOX3D_FREELIST_END;
	}
	s_adapter.firstFree = 0;
	s_adapter.allocCount = 0;
	s_adapter.groundShapeId = b3_nullShapeId;
	s_adapter.materialOverrideCount = 0;
	s_adapter.selectedBodyId = b3_nullBodyId;
	s_adapter.selectedShapeId = b3_nullShapeId;
	s_adapter.transparentDynamic = false;

	s_adapter.guiDraw = b3DefaultDebugDraw();
	s_adapter.guiDraw.drawShapes = true;
}

void ResetAdapterPool( void )
{
	// Release GPU mesh refs still held by live geom shapes, then rebuild the
	// free list from scratch. Mirrors InitAdapter's pool setup. The O(capacity)
	// sweep is cheap and guarantees a clean slate even if a shape leaked a ref.
	for ( int i = 0; i < BOX3D_USER_SHAPE_CAPACITY; ++i )
	{
		DebugShape* us = &s_adapter.pool[i];
		if ( us->kind == Box3DUS_Hull || us->kind == Box3DUS_Mesh || us->kind == Box3DUS_HeightField )
		{
			ReleaseMeshReference( us->geom.handle );
		}
		us->kind = Box3DUS_Free;
		us->nextFree = ( i + 1 < BOX3D_USER_SHAPE_CAPACITY ) ? ( i + 1 ) : BOX3D_FREELIST_END;
	}
	s_adapter.firstFree = 0;
	s_adapter.allocCount = 0;

	s_adapter.groundShapeId = b3_nullShapeId;
	s_adapter.materialOverrideCount = 0;
	s_adapter.selectedBodyId = b3_nullBodyId;
	s_adapter.selectedShapeId = b3_nullShapeId;
	s_adapter.transparentDynamic = false;
}

b3DebugDraw* GetGuiDraw( void )
{
	return &s_adapter.guiDraw;
}

void ApplyGuiFlags( b3DebugDraw* out )
{
	assert( out != NULL );
	out->forceScale = s_adapter.guiDraw.forceScale;
	out->jointScale = s_adapter.guiDraw.jointScale;
	out->drawShapes = s_adapter.guiDraw.drawShapes;
	out->drawJoints = s_adapter.guiDraw.drawJoints;
	out->drawJointExtras = s_adapter.guiDraw.drawJointExtras;
	out->drawBounds = s_adapter.guiDraw.drawBounds;
	out->drawMass = s_adapter.guiDraw.drawMass;
	out->drawBodyNames = s_adapter.guiDraw.drawBodyNames;
	out->drawContacts = s_adapter.guiDraw.drawContacts;
	out->drawAnchorA = s_adapter.guiDraw.drawAnchorA;
	out->drawGraphColors = s_adapter.guiDraw.drawGraphColors;
	out->drawContactFeatures = s_adapter.guiDraw.drawContactFeatures;
	out->drawContactNormals = s_adapter.guiDraw.drawContactNormals;
	out->drawContactForces = s_adapter.guiDraw.drawContactForces;
	out->drawFrictionForces = s_adapter.guiDraw.drawFrictionForces;
	out->drawIslands = s_adapter.guiDraw.drawIslands;
}

void SetTransparentDynamic( bool enabled )
{
	s_adapter.transparentDynamic = enabled;
}

bool GetTransparentDynamic( void )
{
	return s_adapter.transparentDynamic;
}

void SetGroundShape( b3ShapeId shapeId )
{
	s_adapter.groundShapeId = shapeId;
}

void SetShapeMaterial( b3ShapeId shapeId, Vec4 color, float metallic, float roughness )
{
	for ( int i = 0; i < s_adapter.materialOverrideCount; ++i )
	{
		if ( B3_ID_EQUALS( s_adapter.materialOverrides[i].shapeId, shapeId ) )
		{
			s_adapter.materialOverrides[i].color = color;
			s_adapter.materialOverrides[i].metallic = metallic;
			s_adapter.materialOverrides[i].roughness = roughness;
			return;
		}
	}

	if ( s_adapter.materialOverrideCount >= BOX3D_MATERIAL_OVERRIDE_CAPACITY )
	{
		return;
	}

	int index = s_adapter.materialOverrideCount++;
	s_adapter.materialOverrides[index].shapeId = shapeId;
	s_adapter.materialOverrides[index].color = color;
	s_adapter.materialOverrides[index].metallic = metallic;
	s_adapter.materialOverrides[index].roughness = roughness;
}

int GetDebugShapeCount( void )
{
	return s_adapter.allocCount;
}

void SetSelectedBody( b3BodyId bodyId )
{
	s_adapter.selectedBodyId = b3Body_IsValid( bodyId ) ? bodyId : b3_nullBodyId;
	s_adapter.selectedShapeId = b3_nullShapeId;
}

void SetSelectedShape( b3ShapeId shapeId )
{
	s_adapter.selectedShapeId = b3Shape_IsValid( shapeId ) ? shapeId : b3_nullShapeId;
	s_adapter.selectedBodyId = b3_nullBodyId;
}

void ClearSelection( void )
{
	s_adapter.selectedBodyId = b3_nullBodyId;
	s_adapter.selectedShapeId = b3_nullShapeId;
}

b3BodyId GetSelectedBody( void )
{
	return s_adapter.selectedBodyId;
}

bool IsBodySelected( b3BodyId bodyId )
{
	if ( B3_IS_NULL( s_adapter.selectedBodyId ) )
	{
		return false;
	}

	return B3_ID_EQUALS( s_adapter.selectedBodyId, bodyId );
}

static bool IsShapeSelected( b3ShapeId shapeId )
{
	if ( B3_IS_NULL( s_adapter.selectedShapeId ) )
	{
		return false;
	}

	return B3_ID_EQUALS( s_adapter.selectedShapeId, shapeId );
}

static HighlightKind ResolveHighlightKind( b3BodyId bodyId, b3ShapeId shapeId )
{
	return ( IsShapeSelected( shapeId ) || IsBodySelected( bodyId ) ) ? HIGHLIGHT_KIND_SELECT : HIGHLIGHT_KIND_NONE;
}

static int AllocDebugShape( void )
{
	if ( s_adapter.firstFree == BOX3D_FREELIST_END )
	{
		return -1;
	}
	const int index = s_adapter.firstFree;
	s_adapter.firstFree = s_adapter.pool[index].nextFree;
	s_adapter.allocCount += 1;
	return index;
}

static void FreeDebugShape( int index )
{
	assert( index >= 0 && index < BOX3D_USER_SHAPE_CAPACITY );

	// A sample switch cleans the pool twice: ResetAdapterPool sweeps it, then
	// b3DestroyWorld fires the destroy callback per shape. Relinking a slot
	// that is already free loops the free list, and a later alloc then hands
	// one slot to two shapes so the second overwrites the first.
	if ( s_adapter.pool[index].kind == Box3DUS_Free )
	{
		return;
	}

	s_adapter.pool[index].kind = Box3DUS_Free;
	s_adapter.pool[index].nextFree = s_adapter.firstFree;
	s_adapter.firstFree = index;
	s_adapter.allocCount -= 1;
}

// Color + transform helpers

// b3HexColor is the SVG palette as 0xRRGGBB hex literals, sRGB-encoded,
// like every CSS/SVG color.

// sRGB values used directly as linear albedo for overlays and in world text
static Vec4 HexColorToVec4( b3HexColor color )
{
	const uint32_t v = (uint32_t)color;
	const float r = (float)( ( v >> 16 ) & 0xFFu ) * ( 1.0f / 255.0f );
	const float g = (float)( ( v >> 8 ) & 0xFFu ) * ( 1.0f / 255.0f );
	const float b = (float)( v & 0xFFu ) * ( 1.0f / 255.0f );
	return MakeVec4( r, g, b, 1.0f );
}

static Vec4 HexColorAToVec4( b3HexColor color, float alpha )
{
	const uint32_t v = (uint32_t)color;
	const float r = (float)( ( v >> 16 ) & 0xFFu ) * ( 1.0f / 255.0f );
	const float g = (float)( ( v >> 8 ) & 0xFFu ) * ( 1.0f / 255.0f );
	const float b = (float)( v & 0xFFu ) * ( 1.0f / 255.0f );
	return MakeVec4( r, g, b, alpha );
}

// sRGB EOTF, the standard piecewise curve (IEC 61966-2-1), one channel.
static float SRGBToLinear( float c )
{
	return ( c <= 0.04045f ) ? ( c * ( 1.0f / 12.92f ) ) : powf( ( c + 0.055f ) * ( 1.0f / 1.055f ), 2.4f );
}

// sRGB 0xRRGGBB -> linear RGBA. Alpha carries no transfer function. Used
// for lit shape base colors, see the hexColorToVec4 note above.
static Vec4 HexColorToLinear( b3HexColor color )
{
	const Vec4 s = HexColorToVec4( color );
	return MakeVec4( SRGBToLinear( s.x ), SRGBToLinear( s.y ), SRGBToLinear( s.z ), s.w );
}

// Get a capsule transform to represent axial spin
static b3Transform GetCapsuleLocalFrame( b3Vec3 c1, b3Vec3 c2, float* outHalfLength )
{
	b3Vec3 d = b3Sub( c2, c1 );
	float length = b3Length( d );
	*outHalfLength = 0.5f * length;

	b3Transform t;
	t.q = b3Quat_identity;
	if ( length > 1.0e-6f )
	{
		b3Vec3 axis = b3MulSV( 1.0f / length, d );

		// For rendering, the capsule axis is along x
		t.q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisX, axis );
	}
	t.p = b3Lerp( c1, c2, 0.5f );
	return t;
}

static void RefreshMaterialFromOverride( DebugShape* us )
{
	us->hasMaterial = false;
	for ( int i = 0; i < s_adapter.materialOverrideCount; ++i )
	{
		if ( B3_ID_EQUALS( s_adapter.materialOverrides[i].shapeId, us->shapeId ) )
		{
			us->hasMaterial = true;
			us->color = s_adapter.materialOverrides[i].color;
			us->metallic = s_adapter.materialOverrides[i].metallic;
			us->roughness = s_adapter.materialOverrides[i].roughness;
			return;
		}
	}
}

static void PopulateCommonFields( DebugShape* us, const b3DebugShape* debugShape )
{
	const b3BodyId bodyId = b3Shape_GetBody( debugShape->shapeId );
	us->bodyType = b3Body_GetType( bodyId );
	us->isGround = B3_ID_EQUALS( debugShape->shapeId, s_adapter.groundShapeId );
	us->shapeId = debugShape->shapeId;
	us->bodyId = bodyId;
	RefreshMaterialFromOverride( us );
}

// Resolve one compound child into its own pool slot, mirroring the top-level
// create path. Hull and mesh children take a mesh reference released on destroy.
// Common fields are inherited from the parent so material and highlight stay
// keyed to the compound's single shape. Returns the pool index, or -1 if the
// pool is full or the geometry could not be registered.
static int CreateCompoundChild( const b3ChildShape* child, const DebugShape* parent )
{
	const bool needsHandle = ( child->type == b3_hullShape || child->type == b3_meshShape );
	MeshHandle handle = InvalidMeshHandle();
	if ( child->type == b3_hullShape )
	{
		handle = FindOrAddHull( child->hull );
	}
	else if ( child->type == b3_meshShape )
	{
		handle = FindOrAddMesh( child->mesh.data );
	}
	if ( needsHandle && !IsMeshHandleValid( handle ) )
	{
		return -1;
	}

	const int index = AllocDebugShape();
	if ( index < 0 )
	{
		if ( needsHandle )
		{
			ReleaseMeshReference( handle );
		}
		return -1;
	}

	DebugShape* us = &s_adapter.pool[index];
	us->bodyType = parent->bodyType;
	us->isGround = parent->isGround;
	us->shapeId = parent->shapeId;
	us->bodyId = parent->bodyId;
	us->hasMaterial = parent->hasMaterial;
	us->color = parent->color;
	us->metallic = parent->metallic;
	us->roughness = parent->roughness;

	switch ( child->type )
	{
		case b3_sphereShape:
			us->kind = Box3DUS_Sphere;
			us->sphere.center = child->sphere.center;
			us->sphere.radius = child->sphere.radius;
			break;
		case b3_capsuleShape:
			us->kind = Box3DUS_Capsule;
			us->capsule.localFrame =
				GetCapsuleLocalFrame( child->capsule.center1, child->capsule.center2, &us->capsule.halfLength );
			us->capsule.radius = child->capsule.radius;
			break;
		case b3_hullShape:
			us->kind = Box3DUS_Hull;
			us->geom.handle = handle;
			us->geom.scale = b3Vec3_one;
			break;
		case b3_meshShape:
			us->kind = Box3DUS_Mesh;
			us->geom.handle = handle;
			us->geom.scale = (b3Vec3){ child->mesh.scale.x, child->mesh.scale.y, child->mesh.scale.z };
			break;
		default:
			assert( false );
			break;
	}

	us->childTransform = child->transform;
	us->nextChild = -1;
	return index;
}

static void* AdapterCreateDebugShape( const b3DebugShape* debugShape, void* context )
{
	(void)context;
	assert( debugShape != NULL );

	if ( debugShape->type == b3_sphereShape )
	{
		int index = AllocDebugShape();
		if ( index < 0 )
		{
			return NULL; // pool exhausted, Box3D treats NULL as "skip draw"
		}
		DebugShape* us = &s_adapter.pool[index];
		us->kind = Box3DUS_Sphere;
		PopulateCommonFields( us, debugShape );
		us->sphere.center = debugShape->sphere->center;
		us->sphere.radius = debugShape->sphere->radius;
		return us;
	}

	if ( debugShape->type == b3_capsuleShape )
	{
		int index = AllocDebugShape();
		if ( index < 0 )
		{
			return NULL;
		}
		const b3Capsule* cap = debugShape->capsule;
		DebugShape* us = &s_adapter.pool[index];
		us->kind = Box3DUS_Capsule;
		PopulateCommonFields( us, debugShape );
		us->capsule.localFrame = GetCapsuleLocalFrame( cap->center1, cap->center2, &us->capsule.halfLength );
		us->capsule.radius = cap->radius;
		return us;
	}

	if ( debugShape->type == b3_hullShape )
	{
		const b3HullData* hull = debugShape->hull;
		MeshHandle handle = FindOrAddHull( hull );
		if ( !IsMeshHandleValid( handle ) )
		{
			return NULL;
		}
		int index = AllocDebugShape();
		if ( index < 0 )
		{
			ReleaseMeshReference( handle );
			return NULL;
		}
		DebugShape* us = &s_adapter.pool[index];
		us->kind = Box3DUS_Hull;
		PopulateCommonFields( us, debugShape );
		us->geom.handle = handle;
		us->geom.scale = b3Vec3_one;
		return us;
	}

	if ( debugShape->type == b3_meshShape )
	{
		const b3Mesh* mesh = debugShape->mesh;
		MeshHandle handle = FindOrAddMesh( mesh->data );
		if ( !IsMeshHandleValid( handle ) )
		{
			return NULL;
		}
		int index = AllocDebugShape();
		if ( index < 0 )
		{
			ReleaseMeshReference( handle );
			return NULL;
		}
		DebugShape* us = &s_adapter.pool[index];
		us->kind = Box3DUS_Mesh;
		PopulateCommonFields( us, debugShape );
		us->geom.handle = handle;
		us->geom.scale = (b3Vec3){ mesh->scale.x, mesh->scale.y, mesh->scale.z };
		return us;
	}

	if ( debugShape->type == b3_heightShape )
	{
		const b3HeightFieldData* hf = debugShape->heightField;
		const MeshHandle handle = FindOrAddHeightField( hf );
		if ( !IsMeshHandleValid( handle ) )
		{
			return NULL;
		}
		const int index = AllocDebugShape();
		if ( index < 0 )
		{
			ReleaseMeshReference( handle );
			return NULL;
		}
		DebugShape* us = &s_adapter.pool[index];
		us->kind = Box3DUS_HeightField;
		PopulateCommonFields( us, debugShape );
		us->geom.handle = handle;
		us->geom.scale = b3Vec3_one;
		return us;
	}

	if ( debugShape->type == b3_compoundShape )
	{
		const int index = AllocDebugShape();
		if ( index < 0 )
		{
			return NULL;
		}
		DebugShape* us = &s_adapter.pool[index];
		us->kind = Box3DUS_Compound;
		PopulateCommonFields( us, debugShape );
		us->compound.firstChild = -1;

		// Flatten the children once. The fixed pool never relocates, so the
		// parent pointer stays valid while children are allocated.
		const b3CompoundData* compound = debugShape->compound;
		const int total = compound->capsuleCount + compound->hullCount + compound->meshCount + compound->sphereCount;
		int prev = -1;
		for ( int i = 0; i < total; ++i )
		{
			b3ChildShape child = b3GetCompoundChild( compound, i );
			const int childIndex = CreateCompoundChild( &child, us );
			if ( childIndex < 0 )
			{
				continue; // skip a child we couldn't register, keep the rest
			}
			if ( prev < 0 )
			{
				us->compound.firstChild = childIndex;
			}
			else
			{
				s_adapter.pool[prev].nextChild = childIndex;
			}
			prev = childIndex;
		}
		return us;
	}

	// Unknown shape type. Return NULL so Box3D skips drawing it.
	return NULL;
}

static void DestroyDebugShape( void* userShape, void* context )
{
	(void)context;
	if ( userShape == NULL )
	{
		return;
	}
	DebugShape* us = (DebugShape*)userShape;

	if ( us->kind == Box3DUS_Compound )
	{
		int ci = us->compound.firstChild;
		while ( ci != -1 )
		{
			DebugShape* child = &s_adapter.pool[ci];
			const int next = child->nextChild;
			if ( child->kind == Box3DUS_Hull || child->kind == Box3DUS_Mesh || child->kind == Box3DUS_HeightField )
			{
				ReleaseMeshReference( child->geom.handle );
			}
			FreeDebugShape( ci );
			ci = next;
		}
	}
	else if ( us->kind == Box3DUS_Hull || us->kind == Box3DUS_Mesh || us->kind == Box3DUS_HeightField )
	{
		ReleaseMeshReference( us->geom.handle );
	}

	const int index = (int)( us - s_adapter.pool );
	FreeDebugShape( index );
}

// BOX3D_GROUND_GRID_CELL_SIZE lives in the header so the host can wrap the draw
// origin to the same grid period.

// Alpha applied to non-static shapes when box3dAdapterSetTransparentDynamic is on.
#define BOX3D_TRANSPARENT_DYNAMIC_ALPHA 0.5f

// Emit one resolved primitive at baseTransform. The per-kind offset (sphere
// center, capsule frame) layers on top of baseTransform, so the same path
// serves a top-level shape (base = body transform) and a compound child
// (base = body transform composed with the child transform).
static void AppendResolvedShape( const DebugShape* us, b3Transform baseTransform, Vec4 c, float metallic, float roughness,
								 TransparentShadowCast shadowCast, HighlightKind hk )
{
	if ( us->kind == Box3DUS_Sphere )
	{
		b3Transform transform = { b3TransformPoint( baseTransform, us->sphere.center ), baseTransform.q };
		AppendSphere( transform, us->sphere.radius, c, metallic, roughness, shadowCast );
		if ( hk != HIGHLIGHT_KIND_NONE )
		{
			AppendHighlightSphere( transform, us->sphere.radius, hk );
		}
	}
	else if ( us->kind == Box3DUS_Capsule )
	{
		b3Transform transform = b3MulTransforms( baseTransform, us->capsule.localFrame );
		AppendCapsule( transform, us->capsule.halfLength, us->capsule.radius, c, metallic, roughness, shadowCast );
		if ( hk != HIGHLIGHT_KIND_NONE )
		{
			AppendHighlightCapsule( transform, us->capsule.halfLength, us->capsule.radius, hk );
		}
	}
	else if ( us->kind == Box3DUS_Hull || us->kind == Box3DUS_Mesh || us->kind == Box3DUS_HeightField )
	{
		MeshMaterialMode mode = us->isGround ? MESH_MATERIAL_MODE_GROUND_GRID : MESH_MATERIAL_MODE_SOLID;
		float cell = us->isGround ? BOX3D_GROUND_GRID_CELL_SIZE : 0.0f;
		AppendMesh( us->geom.handle, baseTransform, us->geom.scale, c, metallic, roughness, mode, cell, shadowCast );
		if ( hk != HIGHLIGHT_KIND_NONE )
		{
			AppendHighlightGeometry( us->geom.handle, baseTransform, us->geom.scale, hk );
		}
	}
}

static bool DrawShape( void* userShape, b3WorldTransform shapeTransform, b3HexColor color, void* context )
{
	(void)context;
	if ( userShape == NULL )
	{
		return true; // unsupported shape type, skip and continue
	}

	// Shift the world transform into the camera relative frame the primitives render in
	b3Transform shapeRelative = b3ToRelativeTransform( shapeTransform, GetDrawOrigin() );

	DebugShape* us = (DebugShape*)userShape;

	RefreshMaterialFromOverride( us );

	Vec4 c;
	float metallic;
	float roughness;
	if ( us->hasMaterial )
	{
		c = us->color;
		metallic = us->metallic;
		roughness = us->roughness;
	}
	else
	{
		c = HexColorToLinear( color );

		// State-driven preset rides in the color high byte. Default falls back to
		// the per-bodyType material.
		b3DebugMaterial preset = (b3DebugMaterial)( ( (uint32_t)color >> 24 ) & 0xFFu );
		if ( preset > b3_debugMaterialDefault && preset <= b3_debugMaterialMetallic )
		{
			metallic = kDebugMaterialMetallic[preset];
			roughness = kDebugMaterialRoughness[preset];
		}
		else
		{
			int type = (int)us->bodyType;
			metallic = ( type >= 0 && type < 3 ) ? kBodyTypeMetallic[type] : 0.0f;
			roughness = ( type >= 0 && type < 3 ) ? kBodyTypeRoughness[type] : 0.5f;
		}
	}

	if ( s_adapter.transparentDynamic && us->bodyType == b3_dynamicBody )
	{
		c.w = BOX3D_TRANSPARENT_DYNAMIC_ALPHA;
	}

	TransparentShadowCast shadowCast = TRANSPARENT_SHADOW_NONE;

	// Resolve selection state once. A selected shape outlines alone, a selected
	// body outlines all its shapes, compound children included.
	HighlightKind hk = ResolveHighlightKind( us->bodyId, us->shapeId );

	if ( us->kind == Box3DUS_Compound )
	{
		for ( int ci = us->compound.firstChild; ci != -1; ci = s_adapter.pool[ci].nextChild )
		{
			b3Transform base = b3MulTransforms( shapeRelative, s_adapter.pool[ci].childTransform );
			AppendResolvedShape( &s_adapter.pool[ci], base, c, metallic, roughness, shadowCast, hk );
		}
	}
	else
	{
		AppendResolvedShape( us, shapeRelative, c, metallic, roughness, shadowCast, hk );
	}

	return true;
}

#define BOX3D_LINE_THICKNESS_PX 2.5f
#define BOX3D_TRANSFORM_LENGTH ( 0.25f * b3GetLengthUnitsPerMeter() )

static void DrawSegmentFcn( b3Pos p1, b3Pos p2, b3HexColor color, void* context )
{
	(void)context;
	b3Pos o = GetDrawOrigin();
	OverlayAppendLine( b3SubPos( p1, o ), b3SubPos( p2, o ), HexColorToVec4( color ), BOX3D_LINE_THICKNESS_PX,
					   OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

static void DrawTransformFcn( b3WorldTransform transform, void* context )
{
	(void)context;
	b3Transform local = b3ToRelativeTransform( transform, GetDrawOrigin() );
	b3Vec3 origin = local.p;
	b3Matrix3 basis = b3MakeMatrixFromQuat( local.q );
	float L = BOX3D_TRANSFORM_LENGTH;
	Vec4 red = MakeVec4( 1.0f, 0.0f, 0.0f, 1.0f );
	Vec4 green = MakeVec4( 0.0f, 1.0f, 0.0f, 1.0f );
	Vec4 blue = MakeVec4( 0.0f, 0.0f, 1.0f, 1.0f );
	OverlayAppendLine( origin, b3MulAdd( origin, L, basis.cx ), red, BOX3D_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS,
					   OVERLAY_OCCLUSION_HIDE );
	OverlayAppendLine( origin, b3MulAdd( origin, L, basis.cy ), green, BOX3D_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS,
					   OVERLAY_OCCLUSION_HIDE );
	OverlayAppendLine( origin, b3MulAdd( origin, L, basis.cz ), blue, BOX3D_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS,
					   OVERLAY_OCCLUSION_HIDE );
}

static void DrawPointFcn( b3Pos p, float size, b3HexColor color, void* context )
{
	(void)context;
	OverlayAppendPoint( b3SubPos( p, GetDrawOrigin() ), HexColorToVec4( color ), size, OVERLAY_THICKNESS_PIXELS,
						OVERLAY_OCCLUSION_HIDE );
}

static void DrawSphereFcn( b3Pos p, float radius, b3HexColor color, float alpha, void* context )
{
	(void)context;
	DrawSphereEx( (b3WorldTransform){ p, b3Quat_identity }, radius, HexColorAToVec4( color, alpha ), DEFAULT_METALLIC,
				  DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_NONE );
}

static void DrawCapsuleFcn( b3Pos p1, b3Pos p2, float radius, b3HexColor color, float alpha, void* context )
{
	(void)context;

	b3Vec3 e = b3SubPos(p2, p1);
	float length = b3Length( e );
	if (length < FLT_EPSILON)
	{
		DrawSphereEx( (b3WorldTransform){ p1, b3Quat_identity }, radius, HexColorAToVec4( color, alpha ), DEFAULT_METALLIC,
					  DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_NONE );
		return;
	}

	b3Vec3 en = b3MulSV( 1.0f / length, e );
	b3WorldTransform transform;
	transform.p = b3OffsetPos( p1, b3MulSV( 0.5f, e ));
	transform.q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisX, en );

	DrawCapsuleEx( transform, 0.5f * length, radius, HexColorAToVec4( color, alpha ), DEFAULT_METALLIC,
				  DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_NONE );
}

static void DrawBoundsFcn( b3AABB aabb, b3HexColor color, void* context )
{
	(void)context;

	// The fat AABB is float world space, difference against the origin in double before the corners demote
	b3Pos origin = GetDrawOrigin();
	b3Vec3 lower = b3SubPos( b3ToPos( aabb.lowerBound ), origin );
	b3Vec3 upper = b3SubPos( b3ToPos( aabb.upperBound ), origin );
	Vec4 c = HexColorToVec4( color );

	b3Vec3 c000 = { lower.x, lower.y, lower.z };
	b3Vec3 c100 = { upper.x, lower.y, lower.z };
	b3Vec3 c010 = { lower.x, upper.y, lower.z };
	b3Vec3 c110 = { upper.x, upper.y, lower.z };
	b3Vec3 c001 = { lower.x, lower.y, upper.z };
	b3Vec3 c101 = { upper.x, lower.y, upper.z };
	b3Vec3 c011 = { lower.x, upper.y, upper.z };
	b3Vec3 c111 = { upper.x, upper.y, upper.z };

	float th = BOX3D_LINE_THICKNESS_PX;
	OverlayThicknessUnit u = OVERLAY_THICKNESS_PIXELS;
	OverlayOcclusionMode o = OVERLAY_OCCLUSION_HIDE;

	// Bottom face
	OverlayAppendLine( c000, c100, c, th, u, o );
	OverlayAppendLine( c100, c101, c, th, u, o );
	OverlayAppendLine( c101, c001, c, th, u, o );
	OverlayAppendLine( c001, c000, c, th, u, o );

	// Top face
	OverlayAppendLine( c010, c110, c, th, u, o );
	OverlayAppendLine( c110, c111, c, th, u, o );
	OverlayAppendLine( c111, c011, c, th, u, o );
	OverlayAppendLine( c011, c010, c, th, u, o );

	// Vertical edges
	OverlayAppendLine( c000, c010, c, th, u, o );
	OverlayAppendLine( c100, c110, c, th, u, o );
	OverlayAppendLine( c101, c111, c, th, u, o );
	OverlayAppendLine( c001, c011, c, th, u, o );
}

// Oriented box: 8 corners are transform * (+/-ex, +/-ey, +/-ez). 12 edges
// connect adjacent corners along each axis.
static void DrawBoxFcn( b3Vec3 extents, b3WorldTransform transform, b3HexColor color, void* context )
{
	(void)context;
	Vec4 c = HexColorToVec4( color );

	// Shift into the camera relative frame, then transform the local corners
	b3Transform local = b3ToRelativeTransform( transform, GetDrawOrigin() );

	// Local-space corners (8 = 2^3 sign combinations along each axis).
	float signs[2] = { -1.0f, 1.0f };
	b3Vec3 corners[8];
	for ( int xi = 0; xi < 2; ++xi )
	{
		for ( int yi = 0; yi < 2; ++yi )
		{
			for ( int zi = 0; zi < 2; ++zi )
			{
				float lx = signs[xi] * extents.x;
				float ly = signs[yi] * extents.y;
				float lz = signs[zi] * extents.z;
				corners[( xi << 2 ) | ( yi << 1 ) | zi] = b3TransformPoint( local, (b3Vec3){ lx, ly, lz } );
			}
		}
	}

	// Edges: bit-pattern walk where each edge connects two corner indices
	// that differ in exactly one axis bit.
	int edges[12][2] = {
		{ 0, 4 }, { 4, 5 }, { 5, 1 }, { 1, 0 }, // bottom face (yi = 0)
		{ 2, 6 }, { 6, 7 }, { 7, 3 }, { 3, 2 }, // top face (yi = 1)
		{ 0, 2 }, { 4, 6 }, { 5, 7 }, { 1, 3 }, // verticals (yi differs)
	};

	float th = BOX3D_LINE_THICKNESS_PX;
	OverlayThicknessUnit u = OVERLAY_THICKNESS_PIXELS;
	OverlayOcclusionMode o = OVERLAY_OCCLUSION_HIDE;

	for ( int i = 0; i < 12; ++i )
	{
		OverlayAppendLine( corners[edges[i][0]], corners[edges[i][1]], c, th, u, o );
	}
}

static void DrawStringFcn( b3Pos p, const char* s, b3HexColor color, void* context )
{
	(void)context;
	if ( s == NULL )
	{
		return;
	}
	DrawString( b3SubPos( p, GetDrawOrigin() ), HexColorToVec4( color ), s );
}

void AttachToWorldDef( b3WorldDef* def )
{
	assert( def != NULL );
	def->createDebugShape = AdapterCreateDebugShape;
	def->destroyDebugShape = DestroyDebugShape;
	def->userDebugShapeContext = NULL;
}

void MakeDebugDraw( b3DebugDraw* out )
{
	assert( out != NULL );

	*out = b3DefaultDebugDraw();

	out->DrawShapeFcn = DrawShape;
	out->DrawSegmentFcn = DrawSegmentFcn;
	out->DrawTransformFcn = DrawTransformFcn;
	out->DrawPointFcn = DrawPointFcn;
	out->DrawSphereFcn = DrawSphereFcn;
	out->DrawCapsuleFcn = DrawCapsuleFcn;
	out->DrawBoundsFcn = DrawBoundsFcn;
	out->DrawBoxFcn = DrawBoxFcn;
	out->DrawStringFcn = DrawStringFcn;

	// No context needed because globals are used
	out->context = NULL;
}
