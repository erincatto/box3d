// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/debug_adapter.h"
#include "gfx/draw.h"
#include "imgui.h"
#include "mesh_drop.h"
#include "sample.h"
#include "utils.h"

#include "box3d/box3d.h"

#include <stdio.h>
#include <stdlib.h>

class ThinWall : public Sample
{
public:
	explicit ThinWall( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 30.0f, b3Pos_zero );
		}

		AddGroundBox( 40.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		bodyDef.position = { 0.0f, 10.0f, 0.0f };
		bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 90.0f * B3_DEG_TO_RAD );
		b3BodyId wallId = b3CreateBody( m_worldId, &bodyDef );
		b3BoxHull wallBox = b3MakeBoxHull( 10.0f, 0.1f, 10.0f );
		b3CreateHullShape( wallId, &shapeDef, &wallBox.base );

		bodyDef.type = b3_dynamicBody;
		bodyDef.rotation = b3Quat_identity;
		shapeDef.baseMaterial.rollingResistance = 0.1f;

		bodyDef.position = { -5.0, 10.0f, 20.0f };
		bodyDef.linearVelocity = { 0.0f, 0.0, -180.0f };
		bodyDef.angularVelocity = { 20.0f, 0.0f, 0.0f };
		b3BodyId sphereBodyId = b3CreateBody( m_worldId, &bodyDef );

		b3Sphere sphere = { b3Vec3_zero, 0.1f };
		b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );

		bodyDef.position = { 0.0, 10.0f, 20.0f };
		bodyDef.linearVelocity = { 0.0f, 0.0, -180.0f };
		bodyDef.angularVelocity = { 20.0f, -5.0f, 0.0f };
		b3BodyId capsuleBodyId = b3CreateBody( m_worldId, &bodyDef );

		b3Capsule capsule = { { -0.3f, 0.0f, 0.0f }, { 0.3f, 0.0f, 0.0f }, 0.1f };
		b3CreateCapsuleShape( capsuleBodyId, &shapeDef, &capsule );

		bodyDef.position = { 5.0, 10.0f, 20.0f };
		bodyDef.linearVelocity = { 0.0f, 0.0, -180.0f };
		bodyDef.angularVelocity = { 20.0f, 5.0f, 0.0f };
		b3BodyId boxBodyId = b3CreateBody( m_worldId, &bodyDef );
		b3BoxHull box = b3MakeBoxHull( 0.4f, 0.1f, 0.1f );
		b3CreateHullShape( boxBodyId, &shapeDef, &box.base );
	}

	static Sample* Create( SampleContext* context )
	{
		return new ThinWall( context );
	}
};

static int sampleThinWall = RegisterSample( "Continuous", "Thin Wall", ThinWall::Create );

class BounceHouse : public Sample
{
public:
	explicit BounceHouse( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 45.0f, 50.0f, b3Pos_zero );
		}

		AddGroundBox( 10.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();

		bodyDef.position = { 0.0f, -1.0f, 0.0f };
		b3BodyId groundBodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		{
			b3Transform transform = { { 10.0f, 5.0f, 0.0f }, b3Quat_identity };
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, 5.0f, 10.0f, transform );
			b3CreateHullShape( groundBodyId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform = { { -10.0f, 5.0f, 0.0f }, b3Quat_identity };
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, 5.0f, 10.0f, transform );
			b3CreateHullShape( groundBodyId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform = { { 0.0f, 5.0f, -10.0f }, b3Quat_identity };
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 10.0f, 5.0f, 0.1f, transform );
			b3CreateHullShape( groundBodyId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform = { { 0.0f, 5.0f, 10.0f }, b3Quat_identity };
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 10.0f, 5.0f, 0.1f, transform );
			b3CreateHullShape( groundBodyId, &shapeDef, &wallBox.base );
		}

		bodyDef.type = b3_dynamicBody;
		bodyDef.gravityScale = 0.0f;
		bodyDef.position = { -8.0, 4.0f, 0.0f };
		bodyDef.linearVelocity = { 120.0f, 0.0, 120.0f };
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
		b3Sphere sphere = { b3Vec3_zero, 0.5f };
		shapeDef.baseMaterial.friction = 0.0f;
		shapeDef.baseMaterial.restitution = 1.0f;
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );

		// This mesh tries to collide with the hulls. Not a good setup.
		m_dummyMesh = b3CreateBoxMesh( b3Vec3_zero, { 0.8f, 0.4f, 0.4f }, true );
		// b3CreateMeshShape( bodyId, &shapeDef, m_dummyMesh, b3Vec3_one );
	}

	~BounceHouse() override
	{
		b3DestroyMesh( m_dummyMesh );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BounceHouse( context );
	}

	b3MeshData* m_dummyMesh;
};

static int sampleBounceHouse = RegisterSample( "Continuous", "Bounce House", BounceHouse::Create );

class SpinningStick : public Sample
{
public:
	explicit SpinningStick( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 25.0f, 20.0f, { 0.0f, 2.0f, 0.0f } );
		}

		AddGroundBox( 10.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { 0.0f, 0.5f, 0.0f };
		b3BodyId wallBodyId = b3CreateBody( m_worldId, &bodyDef );

		b3BoxHull wallBox = b3MakeBoxHull( 0.125f, 0.5f, 10.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHullShape( wallBodyId, &shapeDef, &wallBox.base );

		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 20.0f, 0.5f };
		bodyDef.linearVelocity = { 0.0f, -100.0f, 0.0f };
		b3Vec3 range = { 50.0f, 50.0f, 50.0f };
		b3Vec3 omega = RandomVec3( -range, range );
		bodyDef.angularVelocity = omega;
		printf( "%f, %f, %f\n", omega.x, omega.y, omega.z );
		b3BodyId stickBodyId = b3CreateBody( m_worldId, &bodyDef );
		b3BoxHull stickBox = b3MakeBoxHull( 2.0f, 0.1f, 0.1f );
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3CreateHullShape( stickBodyId, &shapeDef, &stickBox.base );
	}

	static Sample* Create( SampleContext* context )
	{
		return new SpinningStick( context );
	}
};

static int sampleSpinningStick = RegisterSample( "Continuous", "Spinning Stick", SpinningStick::Create );

// This tests bullets and chain reactions
class BulletVersusStack : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new BulletVersusStack( context );
	}

	explicit BulletVersusStack( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 15.0f, 20.0f, 30.0f, { 0.0f, 2.0f, 0.0f } );
		}

		AddGroundBox( 50.0f );

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			b3BodyId groundBodyId = b3CreateBody( m_worldId, &bodyDef );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3Transform transform = { { -1.0f, 5.0f, 0.0f }, b3Quat_identity };
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, 5.0f, 10.0f, transform );
			b3CreateHullShape( groundBodyId, &shapeDef, &wallBox.base );
		}

		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		for ( int row = 0; row < 10; ++row )
		{
			bodyDef.position = { 0.0f, 0.5f + 1.1f * row, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}

		m_bulletId = b3_nullBodyId;
	}

	void Launch()
	{
		if ( B3_IS_NON_NULL( m_bulletId ) )
		{
			b3DestroyBody( m_bulletId );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.position = { 20.5f, 5.5f, 0.0f };
		bodyDef.linearVelocity = { -500.0f, 0.0f, 0.0f };
		m_bulletId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density *= 10.0f;
		b3Sphere sphere = { b3Vec3_zero, 0.25f };
		b3CreateSphereShape( m_bulletId, &shapeDef, &sphere );
	}

	void Keyboard( int key, int action, int mods ) override
	{
		if ( key == 'L' )
		{
			Launch();
		}
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		if ( ImGui::Button( "Launch" ) )
		{
			Launch();
		}

		ImGui::PopItemWidth();

		return true;
	}

	b3BodyId m_bulletId;
};

static int sampleContinuous6 = RegisterSample( "Continuous", "Bullet vs Stack", BulletVersusStack::Create );

class NeedleMesh : public Sample
{
public:
	explicit NeedleMesh( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 25.0f, 4.0f, { 0.0f, 1.2f, 0.0f } );
		}

		int slices = 8;
		m_needle1 = CreateNeedle( 0.99f, 0.1f, { 0.2f, 0.0f, 0.2f }, slices );
		m_needle2 = CreateNeedle( 1.01f, 0.1f, { 0.2f, 0.0f, -0.2f }, slices );
		m_needle3 = CreateNeedle( 0.98f, 0.1f, { -0.2f, 0.0f, -0.2f }, slices );
		m_needle4 = CreateNeedle( 1.02f, 0.1f, { -0.2f, 0.0f, 0.2f }, slices );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundBodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateMeshShape( groundBodyId, &shapeDef, m_needle1, b3Vec3_one );
		b3CreateMeshShape( groundBodyId, &shapeDef, m_needle2, b3Vec3_one );
		b3CreateMeshShape( groundBodyId, &shapeDef, m_needle3, b3Vec3_one );
		b3CreateMeshShape( groundBodyId, &shapeDef, m_needle4, b3Vec3_one );

		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 5.0f, 0.0f };
		bodyDef.linearVelocity = { 0.0f, -10.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( 0.3f, 0.01f, 0.3f );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	~NeedleMesh() override
	{
		b3DestroyMesh( m_needle4 );
		b3DestroyMesh( m_needle3 );
		b3DestroyMesh( m_needle2 );
		b3DestroyMesh( m_needle1 );
	}

	void Render() override
	{
		DrawGroundGrid( 10 );
		Sample::Render();
	}

	b3MeshData* CreateNeedle( float height, float radius, b3Vec3 center, int slices ) const
	{
		// Vertices
		int vertexCount = slices + 1;
		b3Vec3* vertices = (b3Vec3*)malloc( vertexCount * sizeof( b3Vec3 ) );
		B3_ASSERT( vertices );

		float alpha = 0.0f;
		float deltaAlpha = 2.0f * B3_PI / slices;

		vertices[0] = b3Vec3{ 0.0f, height, 0.0f } + center;
		for ( int index = 1; index < vertexCount; ++index )
		{
			b3CosSin cs = b3ComputeCosSin( alpha );
			vertices[index] = b3Vec3{ radius * cs.cosine, 0.0f, radius * cs.sine } + center;
			alpha += deltaAlpha;
		}

		// Indices
		int triangleCount = slices;
		int* indexBase = (int*)malloc( 3 * triangleCount * sizeof( int ) );
		B3_ASSERT( indexBase );

		int index1 = vertexCount - 1;
		for ( int index = 0; index < triangleCount; ++index )
		{
			int index2 = index + 1;
			indexBase[3 * index + 0] = 0;
			indexBase[3 * index + 1] = index2;
			indexBase[3 * index + 2] = index1;
			index1 = index2;
		}

		b3MeshDef def = {};
		def.vertexCount = vertexCount;
		def.vertices = vertices;
		def.triangleCount = triangleCount;
		def.indices = indexBase;
		def.useMedianSplit = true;

		b3MeshData* data = b3CreateMesh( &def, nullptr, 0 );

		free( vertices );
		free( indexBase );

		return data;
	}

	static Sample* Create( SampleContext* context )
	{
		return new NeedleMesh( context );
	}

	b3MeshData* m_needle1;
	b3MeshData* m_needle2;
	b3MeshData* m_needle3;
	b3MeshData* m_needle4;
	b3HullData* mConvex;
};

static int sampleNeedleMesh = RegisterSample( "Continuous", "Needle Mesh", NeedleMesh::Create );

class MeshDrop : public Sample
{
public:
	explicit MeshDrop( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 20.0f, b3Pos_zero );
			GetGuiDraw()->forceScale = 0.1f;
		}

		m_settings = GetDefaultMeshDropSettings();
		m_groundId = b3_nullBodyId;
		m_groundMesh = nullptr;
		CreateGround();

		for ( int i = 0; i < MESH_DROP_BODY_COUNT; ++i )
		{
			m_bodies[i] = b3_nullBodyId;
		}

		m_cylinder = CreateMeshDropCylinder();
		m_seed = 0;
		m_runCount = 0;
		m_failure = false;
		m_autoGenerate = false;
		m_stepWhilePaused = true;

		Generate();
	}

	~MeshDrop() override
	{
		b3DestroyHull( m_cylinder );
		b3DestroyMesh( m_groundMesh );
	}

	void CreateGround()
	{
		if ( B3_IS_NON_NULL( m_groundId ) )
		{
			b3DestroyBody( m_groundId );
			m_groundId = b3_nullBodyId;
		}

		if ( m_groundMesh != nullptr )
		{
			b3DestroyMesh( m_groundMesh );
			m_groundMesh = nullptr;
		}

		m_groundMesh = CreateMeshDropMesh( m_settings.amplitude );
		m_groundId = CreateMeshDropGround( m_worldId, m_groundMesh );
	}

	void Generate()
	{
		for ( int i = 0; i < MESH_DROP_BODY_COUNT; ++i )
		{
			if ( B3_IS_NULL( m_bodies[i] ) )
			{
				continue;
			}

			b3DestroyBody( m_bodies[i] );
			m_bodies[i] = b3_nullBodyId;
		}

		// A non-zero seed replays a generation reported by the mesh_soak tool
		m_seed = m_replaySeed != 0 ? m_replaySeed : (uint32_t)b3GetTicks();

		m_runCount += 1;
		m_stepCount = 0;
		printf( "run %d seed %u\n", m_runCount, m_seed );

		MeshDropSettings settings = m_settings;
		if ( m_solo[0] >= 0 && m_solo[1] >= 0 )
		{
			settings.soloIndex = MESH_DROP_GRID_COUNT * m_solo[0] + m_solo[1];
		}

		CreateMeshDropBodies( m_worldId, settings, m_seed, m_cylinder, m_bodies );
		ResetMeshDropDepth( &m_depth, settings, m_cylinder );
	}

	bool DrawControls() override
	{
		int shapeType = (int)m_settings.shape;
		if ( ImGui::Combo( "Type", &shapeType, g_meshDropShapeNames, e_meshDropShapeCount ) )
		{
			m_stepCount = 0;
			m_settings.shape = MeshDropShape( shapeType );
			Generate();
		}

		if ( ImGui::SliderFloat( "Amplitude", &m_settings.amplitude, 0.0f, 1.0f ) )
		{
			CreateGround();
			Generate();
		}

		if ( ImGui::Checkbox( "Collide", &m_settings.collide ) )
		{
			Generate();
		}

		ImGui::InputScalar( "Seed", ImGuiDataType_U32, &m_replaySeed );

		// Drop only the body at these indices, -1 for all
		if ( ImGui::InputInt2( "Solo", m_solo ) )
		{
			m_solo[0] = b3ClampInt( m_solo[0], -1, MESH_DROP_GRID_COUNT - 1 );
			m_solo[1] = b3ClampInt( m_solo[1], -1, MESH_DROP_GRID_COUNT - 1 );
		}

		if ( ImGui::Button( "Generate" ) )
		{
			Generate();
		}

		if ( ImGui::Button( "Auto Generate" ) )
		{
			m_autoGenerate = !m_autoGenerate;
			m_stepCount = 0;
		}

		return true;
	}

	void Step() override
	{
		int stepCount = m_stepCount;
		Sample::Step();

		if ( m_stepCount != stepCount )
		{
			UpdateMeshDropDepth( m_worldId, &m_depth );
		}

		{
			PickRay pickRay = m_camera->BuildPickRay( m_context->mouseX, m_context->mouseY );

			b3RayResult result = b3World_CastRayClosest( m_worldId, pickRay.origin, pickRay.translation, b3DefaultQueryFilter() );

			if ( result.hit )
			{
				b3BodyId bodyId = b3Shape_GetBody( result.shapeId );
				int index = GetMeshDropBodyIndex( b3Body_GetUserData( bodyId ) );
				if ( index >= 0 )
				{
					DrawTextLine( "indices: (%d, %d)", index / MESH_DROP_GRID_COUNT, index % MESH_DROP_GRID_COUNT );
				}
			}
		}

		DrawTextLine( "seed %u", m_seed );

		if ( m_depth.maxIndex >= 0 )
		{
			int index = m_depth.maxIndex;
			DrawTextLine( "deepest: (%d, %d) %.1f slop at step %d", index / MESH_DROP_GRID_COUNT, index % MESH_DROP_GRID_COUNT,
						  m_depth.maxDepth / B3_LINEAR_SLOP, m_depth.maxStep );
		}

		float tunnelHeight = GetMeshDropTunnelHeight();
		for ( int i = 0; i < MESH_DROP_BODY_COUNT && m_failure == false; i += 1 )
		{
			b3BodyId bodyId = m_bodies[i];
			if ( B3_IS_NULL( bodyId ) )
			{
				continue;
			}

			b3Pos massCenter = b3Body_GetWorldCenter( bodyId );
			if ( massCenter.y < tunnelHeight )
			{
				printf( "index1: %d - index2: %d\n", i / MESH_DROP_GRID_COUNT, i % MESH_DROP_GRID_COUNT );
				printf( "(%6.2f, %6.2f, %6.2f)\n", massCenter.x, massCenter.y, massCenter.z );
				m_context->pause = true;
				m_failure = true;
				m_autoGenerate = false;
			}
		}

		if ( m_autoGenerate )
		{
			b3BodyEvents bodyEvents = b3World_GetBodyEvents( m_worldId );
			if ( bodyEvents.moveCount == 0 )
			{
				Generate();
			}

			float timeStep = 0.0f;
			if ( m_context->pause == false || m_context->singleStep > 0 )
			{
				timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
				m_context->singleStep = b3MaxInt( 0, m_context->singleStep - 1 );
			}

			// Many steps
			if ( timeStep > 0.0f )
			{
				int n = 20;
				for ( int i = 0; i < n; ++i )
				{
					b3World_Step( m_worldId, timeStep, m_context->subStepCount );
					UpdateMeshDropDepth( m_worldId, &m_depth );
				}

				m_stepCount += n;
			}

			if ( m_stepCount > GetMeshDropStepLimit( m_settings.shape ) )
			{
				m_context->pause = true;
				m_failure = true;
				m_autoGenerate = false;
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new MeshDrop( context );
	}

	MeshDropSettings m_settings;
	MeshDropDepth m_depth;
	b3MeshData* m_groundMesh;
	b3BodyId m_groundId;
	b3HullData* m_cylinder;
	b3BodyId m_bodies[MESH_DROP_BODY_COUNT];
	uint32_t m_seed;
	int m_runCount;
	bool m_failure;
	bool m_autoGenerate;
	uint32_t m_replaySeed = 0;
	int m_solo[2] = { -1, -1 };
};

static int sampleMeshDrop = RegisterSample( "Continuous", "Mesh Drop", MeshDrop::Create );

// This sample shows that clustering based on the manifold normal can lead to clipping and/or tunneling.
class HumpMesh : public Sample
{
public:
	explicit HumpMesh( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 25.0f, 10.0f, { 0.0f, 1.2f, 0.0f } );
		}

		AddGroundBox( 20.0f );

		m_hump = CreateHump( 8.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundBodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateMeshShape( groundBodyId, &shapeDef, m_hump, b3Vec3_one );

		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 5.0f, 0.0f };
		bodyDef.linearVelocity = { 0.0f, -50.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.05f, 1.0f );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	~HumpMesh() override
	{
		b3DestroyMesh( m_hump );
	}

	b3MeshData* CreateHump( float cellWidth ) const
	{
		// Vertices
		b3Vec3 vertices[6];

		int index = 0;
		float x = -0.5f * cellWidth;
		for ( int ix = 0; ix <= 1; ++ix )
		{
			float z = -cellWidth;
			for ( int iz = 0; iz <= 2; ++iz )
			{
				vertices[index] = { x, 0.0f, z };
				if ( iz == 1 )
				{
					vertices[index].y = 0.05f * cellWidth;
				}
				z += cellWidth;
				index += 1;
			}
			x += cellWidth;
		}
		B3_ASSERT( index == 6 );

		int triangleCount = 4;
		int indices[3 * 4];

		index = 0;
		for ( int ix = 0; ix < 1; ++ix )
		{
			for ( int iz = 0; iz < 2; ++iz )
			{
				int index1 = iz + 3 * ix;
				int index2 = index1 + 1;
				int index3 = index2 + 3;
				int index4 = index3 - 1;

				B3_ASSERT( index1 < 6 );
				B3_ASSERT( index2 < 6 );
				B3_ASSERT( index3 < 6 );
				B3_ASSERT( index4 < 6 );

				indices[index + 0] = index1;
				indices[index + 1] = index2;
				indices[index + 2] = index3;

				indices[index + 3] = index3;
				indices[index + 4] = index4;
				indices[index + 5] = index1;

				index += 6;
			}
		}
		B3_ASSERT( index == 3 * triangleCount );

		b3MeshDef def = {};
		def.vertexCount = 6;
		def.vertices = vertices;
		def.triangleCount = 4;
		def.indices = indices;
		def.useMedianSplit = true;
		def.identifyEdges = true;

		b3MeshData* meshData = b3CreateMesh( &def, nullptr, 0 );
		return meshData;
	}

	static Sample* Create( SampleContext* context )
	{
		return new HumpMesh( context );
	}

	b3MeshData* m_hump;
};

static int sampleHumpMesh = RegisterSample( "Continuous", "Hump Mesh", HumpMesh::Create );

class IsFast : public Sample
{
public:
	explicit IsFast( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 50.0f, { 0.0f, 15.0f, 0.0f } );
		}

		AddGroundBox( 40.0f );

		b3BoxHull box = b3MakeBoxHull( 0.5f, 10.0f, 0.5f );
		b3Capsule capsule = { { 0.0f, -9.5f, 0.0f }, { 0.0f, 9.5f, 0.0f }, 0.5f };

		bool useCapsule = false;

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -12.0f, 20.0f, 0.0f };
			bodyDef.gravityScale = 0.0f;
			bodyDef.angularVelocity = { 0.0f, 0.0f, 4.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			if ( useCapsule )
			{
				b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			}
			else
			{
				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 0.0f, 20.0f, 0.0f };
			bodyDef.gravityScale = 0.0f;
			bodyDef.angularVelocity = { 0.0f, 4.0f, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			if ( useCapsule )
			{
				b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			}
			else
			{
				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 12.0f, 20.0f, 0.0f };
			bodyDef.gravityScale = 0.0f;
			bodyDef.angularVelocity = { 4.0f, 0.0f, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			if ( useCapsule )
			{
				b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			}
			else
			{
				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new IsFast( context );
	}
};

static int sampleIsFast = RegisterSample( "Continuous", "Is Fast", IsFast::Create );

// CCD versus a dense mesh can stall the solver. CCD does an AABB query so it considers
// along the sweep.
class Stall : public Sample
{
public:
	explicit Stall( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 130.0f, 15.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
		}

		AddGroundBox( 500.0f );

		{
			m_mesh = b3CreateTorusMesh( 200, 200, 2.0f, 1.0f );
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "torus";
			bodyDef.position.y = 2.0f;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateMeshShape( bodyId, &shapeDef, m_mesh, b3Vec3_one );
		}

		Launch();
	}

	void Launch()
	{
		if ( B3_IS_NON_NULL( m_bulletId ) )
		{
			b3DestroyBody( m_bulletId );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.name = "rock";
		bodyDef.position = { 0.0f, 1.0f, -10.0f };
		// This exceeds the default maximum speed.
		bodyDef.linearVelocity = { 0.0f, 0.0f, 600.0f };
		bodyDef.angularVelocity = { 0.0f, 0.0f, 20.0f };
		m_bulletId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3HullData* rock = b3CreateRock( 0.25f );
		b3CreateHullShape( m_bulletId, &shapeDef, rock );
		b3DestroyHull( rock );
	}

	bool DrawControls() override
	{
		if ( ImGui::Button( "Launch" ) )
		{
			Launch();
		}

		return true;
	}

	~Stall() override
	{
		b3DestroyMesh( m_mesh );
	}

	static Sample* Create( SampleContext* context )
	{
		return new Stall( context );
	}

	b3MeshData* m_mesh = nullptr;
	b3BodyId m_bulletId = b3_nullBodyId;
};

static int sampleStall = RegisterSample( "Continuous", "Stall", Stall::Create );

// This shows how adjusting the CCD safety factor can engage continuous collision at
// lower speeds to avoid the slight overlap that happens with discrete collision detection.
// The best way to run this sample is:
// 1. press pause (P)
// 2. restart (R) or press the Drop button
// 3. then single step (.)
// Then look at the metrics and overlap. The shape is drawn orange while it is using
// continuous collision detection.
class SafetyFactor : public Sample
{
public:
	explicit SafetyFactor( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 3.0f, 8.0f, { 0.0f, 1.0f, 0.0f } );
		}

		AddGroundBox( 20.0f );

		m_bodyId = b3_nullBodyId;
		m_extent = 0.5f;
		m_height = 0.3f;
		m_safetyFactor = 0.1f;
		m_overlap = 0.0f;

		Launch();
	}

	void Launch()
	{
		if ( B3_IS_NON_NULL( m_bodyId ) )
		{
			b3DestroyBody( m_bodyId );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, m_height + m_extent, 0.0f };
		bodyDef.safetyFactor = m_safetyFactor;
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( m_extent, m_extent, m_extent );
		b3CreateHullShape( m_bodyId, &shapeDef, &box.base );

		m_overlap = 0.0f;
	}

	void Step() override
	{
		Sample::Step();

		if ( m_didStep )
		{
			// Contacts are computed at the beginning of the step, so the manifold holds the initial separation.
			b3ContactData data;
			int count = b3Body_GetContactData( m_bodyId, &data, 1 );
			if ( count == 1 )
			{
				for ( int i = 0; i < data.manifoldCount; ++i )
				{
					const b3Manifold* manifold = data.manifolds + i;
					for ( int j = 0; j < manifold->pointCount; ++j )
					{
						m_overlap = b3MaxFloat( m_overlap, -manifold->points[j].separation );
					}
				}
			}
		}

		// The overlap line sits below the ground surface, so dash it where the ground hides it
		DrawLineEx( { -3.0f, -m_overlap, 0.0f }, { 3.0f, -m_overlap, 0.0f }, MakeColor( b3_colorRed ),
					DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );

		float timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
		float motion = timeStep * b3Length( b3Body_GetLinearVelocity( m_bodyId ) );

		DrawTextLine( "actual movement = %.3f m", motion );
		DrawTextLine( "fast movement = %.3f m", m_safetyFactor * m_extent );
		DrawTextLine( "overlap = %.4f m", m_overlap );
	}

	bool DrawControls() override
	{
		bool changed = false;

		ImGui::PushItemWidth( 10.0f * ImGui::GetFontSize() );
		changed |= ImGui::SliderFloat( "Height", &m_height, 0.0f, 2.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Safety", &m_safetyFactor, 0.0f, 1.0f, "%.2f" );
		ImGui::PopItemWidth();

		if ( ImGui::Button( "Drop" ) || changed )
		{
			Launch();
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new SafetyFactor( context );
	}

	b3BodyId m_bodyId;
	float m_extent;
	float m_height;
	float m_safetyFactor;
	float m_overlap;
};

static int sampleSafetyFactor = RegisterSample( "Continuous", "Safety Factor", SafetyFactor::Create );
