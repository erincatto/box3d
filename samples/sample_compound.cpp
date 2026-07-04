// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/debug_adapter.h"
#include "gfx/draw.h"
#include "human.h"
#include "mesh_loader.h"
#include "sample.h"
#include "utils.h"

#include "box3d/box3d.h"

#include <imgui.h>

class SimpleCompound : public Sample
{
public:
	explicit SimpleCompound( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 45.0f, b3Pos_zero );
		}

		{
			float a = 4.0f;
			b3Vec3 extents = { a, 0.125f * a, a };
			b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
			b3BoxHull box = b3MakeBoxHull( extents.x, extents.y, extents.z );

			b3Transform hullTransform;
			hullTransform.p = { 1.0f, -0.125f * a, 0.0f };
			hullTransform.q = b3MakeQuatFromAxisAngle( b3Normalize( { 1.0f, 0.0f, 1.0f } ), 0.0f * B3_PI );

			b3CompoundHullDef hullDef = {
				.hull = &box.base,
				.transform = hullTransform,
				.material = material,
			};

			b3CompoundDef def = {};
			def.hulls = &hullDef;
			def.hullCount = 1;

			m_compound = b3CreateCompound( &def );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 2.0f, -1.0f, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( { 0.0f, 1.0f, 0.0f }, 0.25f * B3_PI );
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			(void)b3CreateCompoundShape( groundId, &shapeDef, m_compound );
		}

		b3World_SetContactRecycleDistance( m_worldId, 0.0f );

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 0.0f, 2.0f, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3Sphere sphere = { b3Vec3_zero, 0.25f };
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		}

		//{
		//	b3BodyDef bodyDef = b3DefaultBodyDef();
		//	bodyDef.type = b3_dynamicBody;
		//	bodyDef.position = { 0.0f, 10.0f, 2.0f };
		//	b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		//	b3ShapeDef shapeDef = b3DefaultShapeDef();
		//	b3BoxHull box = b3MakeBoxHull( 0.2f, 0.3f, 0.4f );
		//	b3CreateHullShape( bodyId, &shapeDef, &box.base );
		//}

		m_launchSpeedScale = 1.0f;
	}

	~SimpleCompound() override
	{
		b3DestroyCompound( m_compound );
	}

	void Render() override
	{
		Sample::Render();
		b3Transform transform = { { 0.0f, 0.01f, 0.0f }, b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( transform ), 1.0f );

		int height = b3DynamicTree_GetHeight( &m_compound->tree );
		DrawTextLine( "compound tree height = %d", height );
	}

	static Sample* Create( SampleContext* context )
	{
		return new SimpleCompound( context );
	}

	b3CompoundData* m_compound;
};

static int sampleSimple = RegisterSample( "Compound", "Simple", SimpleCompound::Create );

class CompoundSpheres : public Sample
{
public:
	explicit CompoundSpheres( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 45.0f, b3Pos_zero );
		}

		float h = 10.0f;
		b3Vec3 lower = { -h, -h, -h };
		b3Vec3 upper = { h, h, h };
		b3CompoundSphereDef spheres[m_count];
		for ( int i = 0; i < m_count; ++i )
		{
			spheres[i].sphere.center = RandomVec3( lower, upper );
			spheres[i].sphere.radius = RandomFloatRange( 0.01f * h, 0.05f * h );
			spheres[i].material = b3DefaultSurfaceMaterial();
		}

		b3CompoundDef def = {};
		def.spheres = spheres;
		def.sphereCount = m_count;

		m_compound = b3CreateCompound( &def );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		(void)b3CreateCompoundShape( groundId, &shapeDef, m_compound );
	}

	~CompoundSpheres() override
	{
		b3DestroyCompound( m_compound );
	}

	void Render() override
	{
		Sample::Render();
		b3Transform transform = { { 0.0f, 0.01f, 0.0f }, b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( transform ), 1.0f );

		int height = b3DynamicTree_GetHeight( &m_compound->tree );
		DrawTextLine( "compound tree height = %d", height );
	}

	static Sample* Create( SampleContext* context )
	{
		return new CompoundSpheres( context );
	}

	static constexpr int m_count = 20;
	b3CompoundData* m_compound;
};

static int sampleCompoundSpheres = RegisterSample( "Compound", "Spheres", CompoundSpheres::Create );

class CompoundHulls : public Sample
{
public:
	explicit CompoundHulls( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 45.0f, b3Pos_zero );
		}

		float h = 10.0f;
		b3Vec3 lower = { -h, -h, -h };
		b3Vec3 upper = { h, h, h };
		b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
		b3BoxHull boxHulls[m_count];
		b3CompoundHullDef hulls[m_count];
		for ( int i = 0; i < m_count; ++i )
		{
			b3Vec3 extents = {
				.x = RandomFloatRange( 0.01f * h, 0.05f * h ),
				.y = RandomFloatRange( 0.01f * h, 0.05f * h ),
				.z = RandomFloatRange( 0.01f * h, 0.05f * h ),
			};

			b3Transform transform;
			transform.p = RandomVec3( lower, upper );
			transform.q = RandomQuat();

			boxHulls[i] = b3MakeBoxHull( extents.x, extents.y, extents.z );
			hulls[i].hull = &boxHulls[i].base;
			hulls[i].transform = transform;
			hulls[i].material = material;
		}

		b3CompoundDef def = {};
		def.hulls = hulls;
		def.hullCount = m_count;

		m_compound = b3CreateCompound( &def );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		(void)b3CreateCompoundShape( groundId, &shapeDef, m_compound );
	}

	~CompoundHulls() override
	{
		b3DestroyCompound( m_compound );
	}

	void Render() override
	{
		Sample::Render();
		b3Transform transform = { { 0.0f, 0.01f, 0.0f }, b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( transform ), 1.0f );

		int height = b3DynamicTree_GetHeight( &m_compound->tree );
		DrawTextLine( "compound tree height = %d", height );
	}

	static Sample* Create( SampleContext* context )
	{
		return new CompoundHulls( context );
	}

	static constexpr int m_count = 20;
	b3CompoundData* m_compound;
};

static int sampleCompoundHulls = RegisterSample( "Compound", "Hulls", CompoundHulls::Create );

class TileFloor : public Sample
{
public:
	explicit TileFloor( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 45.0f, b3Pos_zero );
		}

		{
			constexpr int gridCount = 50;
			constexpr int boxCount = gridCount * gridCount;

			float a = 4.0f;
			b3Vec3 extents = { a, 0.5f * a, a };
			b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
			b3BoxHull box = b3MakeBoxHull( extents.x, extents.y, extents.z );

			// Must allocate to avoid stack overflow
			b3CompoundHullDef* hulls = new b3CompoundHullDef[boxCount];
			b3Transform transform = b3Transform_identity;

			int index = 0;
			for ( int i = 0; i < gridCount; ++i )
			{
				transform.p.x = ( 2.0f * i - gridCount ) * a;

				for ( int j = 0; j < gridCount; ++j )
				{
					transform.p.z = ( 2.0f * j - gridCount ) * a;
					transform.p.y = RandomFloatRange( -0.5f, 0.25f ) * a;

					assert( index < boxCount );

					hulls[index].hull = &box.base;
					hulls[index].transform = transform;
					hulls[index].material = material;

					index += 1;
				}
			}

			b3CompoundDef def = {};
			def.hulls = hulls;
			def.hullCount = boxCount;

			m_compound = b3CreateCompound( &def );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { -2.0f, 1.0f, -3.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Normalize( { 1.0f, -1.0f, 0.5f } ), 0.0f );
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			(void)b3CreateCompoundShape( groundId, &shapeDef, m_compound );

			delete[] hulls;
			hulls = nullptr;
		}

		// b3World_SetContactRecycleDistance( m_worldId, 0.0f );

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 3.0f, 12.0f, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3Sphere sphere = { b3Vec3_zero, 0.25f };
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		}

#if 0
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 0.0f, 10.0f, 2.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull box = b3MakeBoxHull( 0.2f, 0.3f, 0.4f );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
#endif
	}

	~TileFloor() override
	{
		b3DestroyCompound( m_compound );
	}

	void Render() override
	{
		Sample::Render();
		b3Transform transform = { { 0.0f, 0.01f, 0.0f }, b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( transform ), 1.0f );

		DrawTextLine( "compound hull count = %d, mesh count = %d", m_compound->hullCount, m_compound->meshCount );
		DrawTextLine( "compound byte count = %d", m_compound->byteCount );

		int treeBytes = b3DynamicTree_GetByteCount( &m_compound->tree );
		int height = b3DynamicTree_GetHeight( &m_compound->tree );
		DrawTextLine( "compound tree byte count = %d, height = %d", treeBytes, height );
	}

	static Sample* Create( SampleContext* context )
	{
		return new TileFloor( context );
	}

	b3CompoundData* m_compound;
};

static int sampleTileFloor = RegisterSample( "Compound", "Tile Floor", TileFloor::Create );

class MeshTile : public Sample
{
public:
	explicit MeshTile( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 45.0f, b3Pos_zero );
		}

		{
			constexpr int gridCount = 2;
			constexpr int boxCount = gridCount * gridCount;

			float a = 4.0f;
			b3Vec3 extents = { a, 0.5f * a, a };
			b3SurfaceMaterial material = b3DefaultSurfaceMaterial();

			b3MeshData* box = b3CreateBoxMesh( { 0.0f, 0.0f, 0.0f }, extents, true );
			b3CompoundMeshDef* meshes = new b3CompoundMeshDef[boxCount];
			b3Transform transform = b3Transform_identity;
			transform.p.y = -0.5f * a;

			int index = 0;
			for ( int i = 0; i < gridCount; ++i )
			{
				transform.p.x = ( 2.0f * i - gridCount ) * a;

				for ( int j = 0; j < gridCount; ++j )
				{
					transform.p.z = ( 2.0f * j - gridCount ) * a;

					transform.p.y = RandomFloatRange( -0.5f, 0.25f ) * a;

					assert( index < boxCount );

					meshes[index].meshData = box;
					meshes[index].transform = transform;
					meshes[index].scale = { 1.0f, 1.0f, 1.0f };
					meshes[index].materials = &material;
					meshes[index].materialCount = 1;

					index += 1;
				}
			}

			b3CompoundDef def = {};
			def.meshes = meshes;
			def.meshCount = boxCount;

			m_compound = b3CreateCompound( &def );
			delete[] meshes;
			b3DestroyMesh( box );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			(void)b3CreateCompoundShape( groundId, &shapeDef, m_compound );
		}

#if 0
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 3.0f, 12.0f, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3Sphere sphere = { b3Vec3_zero, 0.25f };
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		}
#endif
		//{
		//	b3BodyDef bodyDef = b3DefaultBodyDef();
		//	bodyDef.type = b3_dynamicBody;
		//	bodyDef.position = { 0.0f, 10.0f, 2.0f };
		//	b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		//	b3ShapeDef shapeDef = b3DefaultShapeDef();
		//	b3BoxHull box = b3MakeBoxHull( 0.2f, 0.3f, 0.4f );
		//	b3CreateHullShape( bodyId, &shapeDef, &box.base );
		//}
	}

	~MeshTile() override
	{
		b3DestroyCompound( m_compound );
	}

	void Render() override
	{
		Sample::Render();
		b3Transform transform = { { 0.0f, 0.01f, 0.0f }, b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( transform ), 1.0f );

		DrawTextLine( "compound instance count = %d, byte count = %d", m_compound->meshCount, m_compound->byteCount );

		int treeBytes = b3DynamicTree_GetByteCount( &m_compound->tree );
		int height = b3DynamicTree_GetHeight( &m_compound->tree );
		DrawTextLine( "compound tree byte count = %d, height = %d", treeBytes, height );
	}

	static Sample* Create( SampleContext* context )
	{
		return new MeshTile( context );
	}

	b3CompoundData* m_compound;
};

static int sampleCompoundMesh = RegisterSample( "Compound", "Mesh Tile", MeshTile::Create );

static bool OverlapResultFcn( b3ShapeId shapeId, void* context )
{
	(void)shapeId;
	bool* overlap = (bool*)context;
	*overlap = true;
	return false;
}

class Village : public Sample
{
public:
	explicit Village( SampleContext* context )
		: Sample( context )
	{
		constexpr int gridCount = m_isDebug ? 8 : 200;
		constexpr float a = 4.0f;
		m_worldWidth = 2.0f * gridCount * a;

		b3Pos position = { 0.0f, 10.0f, 0.0f };
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 10.0f, 5.0f, position );
		}

		m_launchSpeedScale = 2.0f;
		m_mover.Initialize( this, position );

		{
			constexpr int capsuleCapacity = gridCount * gridCount / 8 + 1;
			constexpr int hullCount = gridCount * gridCount;
			constexpr int sphereCapacity = gridCount * gridCount / 8 + 1;

			b3Vec3 extents = { a, 0.5f * a, a };
			b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
			b3BoxHull box = b3MakeBoxHull( extents.x, extents.y, extents.z );

			// Must allocate to avoid stack overflow
			b3CompoundCapsuleDef* capsules = new b3CompoundCapsuleDef[capsuleCapacity];
			b3CompoundHullDef* hulls = new b3CompoundHullDef[hullCount];
			b3CompoundSphereDef* spheres = new b3CompoundSphereDef[sphereCapacity];

			b3Transform transform = b3Transform_identity;

			int capsuleIndex = 0;
			int hullIndex = 0;
			int sphereIndex = 0;

			for ( int i = 0; i < gridCount; ++i )
			{
				transform.p.x = ( 2.0f * i - gridCount ) * a;

				for ( int j = 0; j < gridCount; ++j )
				{
					transform.p.z = ( 2.0f * j - gridCount ) * a;
					transform.p.y = RandomFloatRange( -0.25f, 0.125f ) * a;

					assert( hullIndex < hullCount );

					if ( ( i & 1 ) && ( j & 1 ) )
					{
						b3Vec3 p1 = transform.p + RandomVec3( { -a, a, -a }, { a, 2.0f * a, a } );
						b3Vec3 p2 = transform.p + RandomVec3( { -a, a, -a }, { a, 2.0f * a, a } );
						float radius = RandomFloatRange( 0.1f, 0.5f );

						if ( capsuleIndex < sphereIndex )
						{
							assert( capsuleIndex < capsuleCapacity );
							capsules[capsuleIndex].capsule = { p1, p2, radius };
							capsules[capsuleIndex].material = material;
							capsuleIndex += 1;
						}
						else
						{
							assert( sphereIndex < capsuleCapacity );
							spheres[sphereIndex].sphere = { p1, radius };
							spheres[sphereIndex].material = material;
							sphereIndex += 1;
						}
					}

					hulls[hullIndex].hull = &box.base;
					hulls[hullIndex].transform = transform;
					hulls[hullIndex].material = material;

					hullIndex += 1;
				}
			}

			assert( capsuleIndex <= capsuleCapacity );
			assert( hullIndex == hullCount );
			assert( sphereIndex <= sphereCapacity );

			constexpr int meshGridCount = gridCount / 4;
			constexpr int meshCount = meshGridCount * meshGridCount;
			constexpr float b = 4.0f * a;
			constexpr int materialCapacity = 5;

			b3SurfaceMaterial meshMaterials[materialCapacity];
			b3MeshData* buildingMesh = CreateMeshData( "data/meshes/building.obj", 1.0f, false, false, true, true );

			int materialCount = buildingMesh->materialCount;
			assert( materialCount <= materialCapacity );

			for ( int i = 0; i < materialCount; ++i )
			{
				meshMaterials[i] = b3DefaultSurfaceMaterial();
				if ( i == 0 )
				{
					meshMaterials[i].friction = 0.0f;
				}
				else if ( i == 1 )
				{
					meshMaterials[i].restitution = 0.5f;
				}
				meshMaterials[i].userMaterialId = i + 42;
			}

			b3CompoundMeshDef* meshes = new b3CompoundMeshDef[meshCount];
			transform = b3Transform_identity;

			int meshIndex = 0;
			for ( int i = 0; i < meshGridCount; ++i )
			{
				transform.p.x = ( 2.0f * i - meshGridCount ) * b + 0.5f * b;

				for ( int j = 0; j < meshGridCount; ++j )
				{
					transform.p.y = 0.5f * a;
					transform.p.z = ( 2.0f * j - meshGridCount ) * b + 0.5f * b;
					transform.q = b3MakeQuatFromAxisAngle( { 0.0f, 1.0f, 0.0 }, RandomFloatRange( -B3_PI, B3_PI ) );

					assert( meshIndex < meshCount );

					meshes[meshIndex].meshData = buildingMesh;
					meshes[meshIndex].transform = transform;
					meshes[meshIndex].scale = RandomVec3( { 0.5f, 0.5f, 0.5f }, { 2.0f, 2.0f, 2.0f } );

					if ( meshIndex & 1 )
					{
						meshes[meshIndex].scale.x = -meshes[meshIndex].scale.x;
					}

					if ( meshIndex & 3 )
					{
						meshes[meshIndex].scale.z = -meshes[meshIndex].scale.z;
					}

					meshes[meshIndex].materials = meshMaterials;
					meshes[meshIndex].materialCount = materialCount;

					meshIndex += 1;
				}
			}

			assert( meshIndex == meshCount );

			b3CompoundDef def = {};
			def.capsules = capsules;
			def.capsuleCount = capsuleIndex;
			def.hullCount = hullCount;
			def.hulls = hulls;
			def.hullCount = hullCount;
			def.meshes = meshes;
			def.meshCount = meshCount;
			def.spheres = spheres;
			def.sphereCount = sphereIndex;

			m_compound = b3CreateCompound( &def );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { -1.0f, -0.5f, 2.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( { 0.0f, 1.0f, 0.0f }, -1.15f * B3_PI );
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			(void)b3CreateCompoundShape( groundId, &shapeDef, m_compound );

			delete[] capsules;
			capsules = nullptr;

			delete[] hulls;
			hulls = nullptr;

			delete[] meshes;
			meshes = nullptr;

			delete[] spheres;
			spheres = nullptr;

			b3DestroyMesh( buildingMesh );
		}

		m_rayOrigin = { -0.45f * m_worldWidth, 20.0f, -0.45f * m_worldWidth };
	}

	~Village() override
	{
		b3DestroyCompound( m_compound );
	}

	void Keyboard( int key, int action, int mods ) override
	{
		if ( key == 'T' && action == 1 )
		{
			ToggleThirdPerson();
		}
	}

	void Render() override
	{
		Sample::Render();
		b3Transform transform = { { 0.0f, 0.01f, 0.0f }, b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( transform ), 4.0f );

		DrawTextLine( "surface type = %d", m_userMaterialId );
		DrawTextLine( "compound capsules/hulls/meshes/sphere = %d / %d / %d / %d", m_compound->capsuleCount,
					  m_compound->hullCount, m_compound->meshCount, m_compound->sphereCount );
		DrawTextLine( "compound byte count = %d", m_compound->byteCount );

		int treeBytes = b3DynamicTree_GetByteCount( &m_compound->tree );
		int height = b3DynamicTree_GetHeight( &m_compound->tree );
		DrawTextLine( "compound tree byte count = %d, height = %d", treeBytes, height );

		int total = 0;
		int drawn = GetLastCompoundDrawStats( &total );
		DrawTextLine( "compound children drawn = %d / %d", drawn, total );
	}

	void Step() override
	{
		m_mover.Step( nullptr, 0, true );
		DrawTextLine( "third person (T) = %d", m_camera->m_thirdPerson );

		b3Vec3 translation = { 10.0f, -40.0f, -5.0f };
		b3QueryFilter filter = b3DefaultQueryFilter();

		{
			CastClosestContext context = {};
			(void)b3World_CastRay( m_worldId, m_rayOrigin, translation, filter, CastClosestCallback, &context );

			DrawLine( m_rayOrigin, b3OffsetPos( m_rayOrigin, translation ), MakeColor( b3_colorAliceBlue ) );
			if ( context.hit )
			{
				b3Pos p1 = context.point;
				b3Pos p2 = p1 + 0.5f * context.normal;
				DrawLine( p1, p2, MakeColor( b3_colorYellow ) );
				DrawPoint( p1, 8.0f, MakeColor( b3_colorLightCoral ) );
				DrawTextLine( "ray hit triangle/child/material = %d / %d / %d", context.triangleIndex, context.childIndex,
							  context.materialId );
			}
			else
			{
				DrawTextLine( "ray miss" );
			}
		}

		{
			CastClosestContext context = {};
			b3Pos origin = m_rayOrigin - b3Vec3{ 1.0f, 0.0f, 1.0f };
			b3ShapeProxy proxy = { &b3Vec3_zero, 1, 0.25f };
			b3World_CastShape( m_worldId, origin, &proxy, translation, filter, CastClosestCallback, &context );

			DrawLine( origin, b3OffsetPos( origin, translation ), MakeColor( b3_colorAliceBlue ) );
			if ( context.hit )
			{
				b3Pos position = b3OffsetPos( origin, context.fraction * translation );
				b3Pos p1 = context.point;
				b3Pos p2 = p1 + 0.5f * context.normal;
				DrawLine( p1, p2, MakeColor( b3_colorYellow ) );
				DrawPoint( p1, 8.0f, MakeColor( b3_colorLightCoral ) );
				b3Sphere sphere = { b3Vec3_zero, 0.25f };
				DrawSolidSphere( { position, b3Quat_identity }, sphere, MakeColor( b3_colorOrchid ) );
				DrawTextLine( "shape hit triangle/child/material = %d / %d / %d", context.triangleIndex, context.childIndex,
							  context.materialId );
			}
			else
			{
				DrawTextLine( "shape miss" );
			}
		}

		{
			bool overlap = false;
			b3Pos origin = { m_rayOrigin.x - 1.0f, 2.0f, m_rayOrigin.z - 1.0f };
			b3ShapeProxy proxy = { &b3Vec3_zero, 1, 0.3f };
			b3World_OverlapShape( m_worldId, origin, &proxy, filter, OverlapResultFcn, &overlap );

			b3HexColor color = overlap ? b3_colorDarkMagenta : b3_colorDarkSeaGreen;
			b3Sphere sphere = { b3Vec3_zero, 0.3f };
			DrawSolidSphere( { origin, b3Quat_identity }, sphere, MakeColor( color ) );
		}

		if ( m_rayOrigin.x > 0.45f * m_worldWidth )
		{
			m_rayOrigin.x = -0.45f * m_worldWidth;
			m_rayOrigin.z += 8.0f;
		}

		if ( m_rayOrigin.z > 0.45f * m_worldWidth )
		{
			m_rayOrigin.z = -0.45f * m_worldWidth;
		}

		float timeStep = 0.0f;
		if ( m_context->pause == false || m_context->singleStep > 0 )
		{
			timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
		}

		m_rayOrigin.x += 2.0f * timeStep;

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new Village( context );
	}

	b3CompoundData* m_compound;
	CharacterMover m_mover;
	float m_worldWidth;
	b3Pos m_rayOrigin;
};

static int sampleVillage = RegisterSample( "Compound", "Village", Village::Create );
