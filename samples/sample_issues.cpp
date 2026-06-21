// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "imgui.h"
#include "mesh_loader.h"
#include "sample.h"
#include "gfx/draw.h"

#include "box3d/box3d.h"

#include <vector>

class DumpLoader : public Sample
{
public:
	explicit DumpLoader( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
			// m_camera->SetView( 45.0f, 30.0f, 300.0f, { 3910.62109f, 9862.50293f, 875.395081f } );
		}

		b3SetLengthUnitsPerMeter( 1.0f );

		const char* dumpPrefix = "data/dumps/single_box/";

#include "dumps/single_box/box3d_dump.inl"
	}

	~DumpLoader() override
	{
		for ( b3MeshData* md : m_meshes )
		{
			b3DestroyMesh( md );
		}

		b3SetLengthUnitsPerMeter( 1.0f );
	}

	static Sample* Create( SampleContext* context )
	{
		return new DumpLoader( context );
	}

	std::vector<b3MeshData*> m_meshes;
};

static int sampleDumpLoader = RegisterSample( "Issues", "Dump Loader", DumpLoader::Create );

class Crash : public Sample
{
public:
	explicit Crash( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
		}

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f, 0.0f };
			groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			m_gridMesh = b3CreateGridMesh( 20, 20, 2, 0, true );
			b3CreateMeshShape( groundId, &shapeDef, m_gridMesh, b3Vec3_one );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 2.0f, 4.0f, 0.0f };
		m_bodyId1 = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3CreateHullShape( m_bodyId1, &shapeDef, &box.base );

		bodyDef.position = { -2.0f, 4.0f, 0.0f };
		m_bodyId2 = b3CreateBody( m_worldId, &bodyDef );
		b3CreateHullShape( m_bodyId2, &shapeDef, &box.base );
	}

	~Crash() override
	{
		b3DestroyMesh( m_gridMesh );
	}

	bool DrawControls() override
	{
		if ( ImGui::Button( "Add Joint" ) )
		{
			b3WeldJointDef jointDef = b3DefaultWeldJointDef();
			jointDef.base.bodyIdA = m_bodyId1;
			jointDef.base.bodyIdB = m_bodyId2;
			b3CreateWeldJoint( m_worldId, &jointDef );
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new Crash( context );
	}

	b3BodyId m_bodyId1;
	b3BodyId m_bodyId2;
	b3MeshData* m_gridMesh;
};

static int sampleCrash = RegisterSample( "Issues", "Crash", Crash::Create );

class MultiplePrismatic : public Sample
{
public:
	explicit MultiplePrismatic( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 0.0f, 25.0f, { 0.0f, 5.0f, 0.0f } );
		}

		b3BodyId groundId;
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			groundId = b3CreateBody( m_worldId, &bodyDef );
		}

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3PrismaticJointDef jointDef = b3DefaultPrismaticJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.localFrameA.p = { 0.0f, 0.0f, 0.0f };
		jointDef.base.localFrameB.p = { 0.0f, -0.6f, 0.0f };
		jointDef.base.drawScale = 2.0f;
		jointDef.base.constraintHertz = 240.0f;
		jointDef.lowerTranslation = -6.0f;
		jointDef.upperTranslation = 6.0f;
		jointDef.enableLimit = true;

		for ( int i = 0; i < 6; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, 0.6f + 1.2f * i, 0.0f };
			bodyDef.type = b3_dynamicBody;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );

			jointDef.base.bodyIdB = bodyId;
			b3CreatePrismaticJoint( m_worldId, &jointDef );

			jointDef.base.bodyIdA = bodyId;
			jointDef.base.localFrameA.p = { 0.0f, 0.6f, 0.0f };
		}

		// huge mouse force
		m_mouseForceScale = 1000000.0f;
	}

	static Sample* Create( SampleContext* context )
	{
		return new MultiplePrismatic( context );
	}
};

static int sampleMultiplePrismatic = RegisterSample( "Issues", "Multiple Prismatic", MultiplePrismatic::Create );

class HullCrash : public Sample
{
public:
	explicit HullCrash( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 5.0f, b3Pos_zero );
		}

		m_hull = nullptr;

#if 0
		// bad hull SM_Waterfall_MED_Wide_01
		b3Vec3 points[] = {
			{ 0.100183107, -498.925385, -1275.39966 }, { 0.100183107, -498.925415, 0.125000000 },
			{ 0.100183107, 486.343750, 0.125000000 },  { 0.100183107, 486.343719, -1275.39966 },
			{ -395.117462, 486.343781, -1462.43750 },  { -395.117462, 486.343750, -96.7426758 },
			{ -395.117462, -498.925415, -96.7424469 }, { -395.117462, -498.925446, -1462.52612 },
			{ -186.979691, 486.294891, -1462.47949 },  { -298.250000, 486.294891, 0.125000000 },
			{ -395.121216, 486.294891, -1462.52612 },  { -186.984360, -498.913361, -1462.48413 },
			{ -298.250000, -498.913361, 0.125000000 },
		};

#elif 1
		b3Vec3 points[] = {
			{ 100.000000, -142.292389, 130.826111 },  { 99.5354385, -71.3011093, 130.826111 },
			{ 99.5930862, -80.1112213, -100.000000 }, { 100.000000, -142.292389, -100.000000 },
			{ 99.5930862, -80.1112213, 130.826111 },
		};
#else
		b3Vec3 points[] = {
			{ -11.3861933, -24.2451687, -12.0037909 }, { -11.3889809, -24.2466526, -11.9013014 },
			{ -11.3804407, -24.3151531, -12.0046492 }, { -11.3832273, -24.3166409, -11.9021587 },
			{ -14.4396200, -24.3636723, -12.1324549 }, { -14.4432650, -24.3655701, -12.0299988 },
			{ -14.4356947, -24.4337788, -12.1336164 }, { -14.4393377, -24.4356804, -12.0311594 },
		};
#endif

		static_assert( sizeof( points ) / sizeof( points[0] ) < m_capacity, "bad" );

		m_count = sizeof( points ) / sizeof( points[0] );
		for ( int i = 0; i < m_count; ++i )
		{
			m_points[i] = 0.01f * points[i];
		}

		// This shift shouldn't be necessary but I'm doing it so the hull
		// appears on the screen.
		// for ( int i = 0; i < m_count; ++i )
		//{
		//	m_points[i] -= m_points[0];
		//	m_points[i] *= 0.01f;
		//}

		m_hull = b3CreateHull( m_points, m_count, m_count );

		(void)m_hull;
	}

	~HullCrash() override
	{
		if ( m_hull != nullptr )
		{
			b3DestroyHull( m_hull );
		}
	}

	void Render() override
	{
		if ( m_hull != nullptr )
		{
			DrawHull( b3WorldTransform_identity, m_hull, MakeColor( b3_colorYellow ) );
		}
		else
		{
			for ( int i = 0; i < m_count; ++i )
			{
				DrawPoint( b3ToPos( m_points[i] ), 5.0f, MakeColor( b3_colorWhite ) );
			}
		}

		DrawAxes( b3WorldTransform_identity, 1.0f );

		Sample::Render();
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new HullCrash( sampleContext );
	}

	static constexpr int m_capacity = 64;
	b3HullData* m_hull;
	b3Vec3 m_points[m_capacity];
	int m_count;
};

static int sampleHullCrash = RegisterSample( "Issues", "Hull Crash", HullCrash::Create );

class ConvexJitter : public Sample
{
public:
	explicit ConvexJitter( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 10.0f, { 0.0f, 2.0f, 0.0f } );
			
		}

		AddGroundBox( 10.0f );

		float s = 0.01f;

		{
			b3Vec3 b = { -459.292877f, 217.398331f, 1.00115335f };
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { s * b.x, s * b.z + 2.0f, s * b.y };
			bodyDef.rotation = { { 0.0f, -0.707106769f, 0.0f }, 0.707106769f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();

			constexpr int count = 16;
			b3Vec3 points[count];
			points[0] = { -44.8770714, -91.6598053, -1.92012548 };
			points[1] = { -92.5001831, 51.0151291, 15.8006573 };
			points[2] = { -91.0282211, -9.44371605, 15.6148796 };
			points[3] = { 90.2375641, 77.3870087, 15.9356089 };
			points[4] = { -85.5353241, 91.3750992, -1.36629653 };
			points[5] = { 88.9092178, -87.2975464, -1.86754704 };
			points[6] = { 83.7932816, -89.8572235, 15.4168339 };
			points[7] = { 87.0243988, 88.9776535, -1.32423306 };
			points[8] = { -91.6564941, -85.4949493, 15.3782759 };
			points[9] = { -90.2922516, -87.2074127, -1.92012548 };
			points[10] = { -87.2944870, 89.9510498, 15.9215889 };
			points[11] = { 79.2338104, 89.9690781, 15.9724140 };
			points[12] = { -91.6744461, 81.0823212, -1.39959598 };
			points[13] = { 90.3452759, -76.4459610, 15.4588966 };
			points[14] = { -87.4021912, -89.2263107, 15.3677588 };
			points[15] = { 76.3258057, 92.0059967, 1.82873762 };

			for ( int i = 0; i < count; ++i )
			{
				b3Vec3 p = points[i];
				points[i] = { s * p.x, s * p.z, s * p.y };
			}

			b3HullData* hull = b3CreateHull( points, count, count );

			b3CreateHullShape( bodyId, &shapeDef, hull );
			b3DestroyHull( hull );
		}

		{
			b3Vec3 b = { -402.321838f, 157.310364f, 16.8169250f };
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { s * b.x, s * b.z + 2.0f, s * b.y };
			bodyDef.rotation = { { 0.0f, -0.00152086187f, 0.0f }, 0.999998868f };
			bodyDef.type = b3_dynamicBody;

			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.baseMaterial.rollingResistance = 0.1f;

			constexpr int count = 18;
			b3Vec3 points[count];
			points[0] = { 29.5000000, 17.1488495, 0.175081104 };
			points[1] = { 29.5000000, -17.2990532, 0.125000000 };
			points[2] = { 29.4840164, -17.3057766, 24.0200863 };
			points[3] = { 29.4840164, 17.1648350, 24.1781254 };
			points[4] = { -29.1345520, 17.5529804, 0.125000000 };
			points[5] = { -29.1345520, 17.5529804, 23.7899799 };
			points[6] = { -29.1441040, 16.9679585, 24.3750000 };
			points[7] = { -29.1345520, -17.2990532, 24.3750000 };
			points[8] = { -29.1345520, -17.2990532, 0.175081253 };
			points[9] = { 29.0720215, 17.5529785, 0.125000000 };
			points[10] = { 29.0859070, 17.5629406, 23.8120594 };
			points[11] = { 29.1401348, -17.2990532, 24.3750000 };
			points[12] = { 29.1123581, 16.9722290, 24.4027710 };
			points[13] = { 29.3944912, 17.2543602, 24.1206398 };
			points[14] = { -29.1345520, -17.2990532, 24.0759430 };
			points[15] = { -29.1345520, -16.9722252, 24.4027710 };
			points[16] = { 29.1123619, -16.9722271, 24.4027729 };
			points[17] = { 29.5000000, 17.3429642, 24.0000000 };

			for ( int i = 0; i < count; ++i )
			{
				b3Vec3 p = points[i];
				points[i] = { s * p.x, s * p.z, s * p.y };
			}

			b3HullData* hull = b3CreateHull( points, count, count );

			b3CreateHullShape( bodyId, &shapeDef, hull );
			b3DestroyHull( hull );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new ConvexJitter( context );
	}
};

static int sampleConvexJitter = RegisterSample( "Issues", "Convex Jitter", ConvexJitter::Create );

class SBoxMover : public Sample
{
public:
	explicit SBoxMover( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 12.0f, b3Pos_zero );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { -10.0f, 0.0f, -10.0f };
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			m_heightField = b3CreateGrid( 40, 40, { 0.5f, 1.0f, 0.5f }, false );
			//m_heightField = b3CreateWave( 40, 40, {1.0f, 2.0f, 1.0f}, 0.02f, 0.04f, false );
			b3CreateHeightFieldShape( groundId, &shapeDef, m_heightField );

			m_gridMesh = b3CreateGridMesh( 40, 40, 0.5f, 1, true );
			//b3CreateMeshShape( groundId, &shapeDef, m_gridMesh, b3Vec3_one );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			// m_boxMesh = b3CreateBoxMesh( { 0.0f, 1.0f, 0.0f }, { 1.0f, 1.0f, 1.0f }, true );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			m_boxMesh = b3CreatePlatformMesh( { 0.0f, 0.5f, 0.0f }, 1.0f, 2.0f, 5.0f );
			b3Vec3 scale = b3Vec3_one;
			b3CreateMeshShape( groundId, &shapeDef, m_boxMesh, scale );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 0.0f, 3.5f, 0.0f };
			bodyDef.motionLocks.angularX = true;
			bodyDef.motionLocks.angularY = true;
			bodyDef.motionLocks.angularZ = true;
			bodyDef.enableContactRecycling = false;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull box = b3MakeBoxHull( 0.25f, 1.0f, 0.25f );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}

	~SBoxMover() override
	{
		b3DestroyMesh( m_boxMesh );
		b3DestroyHeightField( m_heightField );
		b3DestroyMesh( m_gridMesh );
	}

	void Render() override
	{
		Sample::Render();
		b3Transform transform = { { 0.0f, 1.1f, 0.0f }, b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( transform ), 3.0f );
	}

	static Sample* Create( SampleContext* context )
	{
		return new SBoxMover( context );
	}

	b3MeshData* m_boxMesh;
	b3HeightFieldData* m_heightField;
	b3MeshData* m_gridMesh;
};

static int sampleBoxMesh = RegisterSample( "Issues", "s&box mover", SBoxMover::Create );

class CapsuleMeshBug : public Sample
{
public:
	explicit CapsuleMeshBug( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 20.0f, 10.0f, 30.0f, { 0.0f, 2.0f, 0.0f } );
		}

		// --- Ground plane ---
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull ground = b3MakeBoxHull( 50.0f, 0.1f, 50.0f );
			b3CreateHullShape( body, &shapeDef, &ground.base );
		}

		// --- Building mesh on top of ground ---
		m_building = CreateMeshData( "data/meshes/building.obj", 1.0f, false, false, true, true );
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, 0.1f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateMeshShape( body, &shapeDef, m_building, b3Vec3_one );
		}

		// --- Locked capsule (same setup as player controller body) ---
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 0.0f, 4.0f, 10.0f };
			bodyDef.motionLocks.angularX = true;
			bodyDef.motionLocks.angularY = true;
			bodyDef.motionLocks.angularZ = true;
			bodyDef.enableSleep = false;
			bodyDef.enableContactRecycling = false;
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.baseMaterial.friction = 0.3f;
			shapeDef.baseMaterial.customColor = b3_colorMagenta;

			b3Capsule capsule = { { 0.0f, -0.5f, 0.0f }, { 0.0f, 0.5f, 0.0f }, 0.3f };
			b3CreateCapsuleShape( body, &shapeDef, &capsule );
		}
	}

	~CapsuleMeshBug() override
	{
		if ( m_building )
		{
			b3DestroyMesh( m_building );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new CapsuleMeshBug( context );
	}

	b3MeshData* m_building = nullptr;
};

static int sampleIndex = RegisterSample( "Issues", "Capsule Mesh", CapsuleMeshBug::Create );
