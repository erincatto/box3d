// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "benchmarks.h"
#include "gfx/debug_adapter.h"
#include "gfx/draw.h"
#include "imgui.h"
#include "sample.h"
#include "utils.h"

#include "box3d/box3d.h"

#include <set>

inline bool operator<( b3BodyId a, b3BodyId b )
{
	uint64_t ua = b3StoreBodyId( a );
	uint64_t ub = b3StoreBodyId( b );
	return ua < ub;
}

class BenchmarkLargePyramid : public Sample
{
public:
	explicit BenchmarkLargePyramid( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 40.0f, -10.0f, 110.0f, { 0.0f, 40.0f, 0.0f } );
		}

		CreateLargePyramid( m_worldId );

		SetGroundShape( GetGroundShapeId() );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkLargePyramid( context );
	}
};

static int sampleLargePyramid = RegisterSample( "Benchmark", "Large Pyramid", BenchmarkLargePyramid::Create );

class BenchmarkWidePyramid : public Sample
{
public:
	explicit BenchmarkWidePyramid( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 5.0f, 80.0f, { 0.0f, 18.0f, 0.0f } );
		}

		CreateWidePyramid( m_worldId );

		SetGroundShape( GetGroundShapeId() );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkWidePyramid( context );
	}
};

static int sampleWidePyramid = RegisterSample( "Benchmark", "Wide Pyramid", BenchmarkWidePyramid::Create );

class BenchmarkManyPyramids : public Sample
{
public:
	explicit BenchmarkManyPyramids( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -10.0f, 10.0f, 120.0f, { 0.0f, 5.0f, 0.0f } );
		}

		CreateManyPyramids( m_worldId );

		// float frictionTorque = 5.0f;
		// float hertz = 1.0f;
		// float dampingRatio = 0.7f;
		// bool colorize = true;

		// Human human = {};
		// b3Vec3 position = m_isDebug ? b3Vec3{ 0.0f, 20.0f, 0.0f } : b3Vec3{ 5.0f, 20.0f, 53.0f };
		// CreateHuman( &human, m_worldId, position, frictionTorque, hertz, dampingRatio, 0, nullptr, colorize );

		SetGroundShape( GetGroundShapeId() );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkManyPyramids( context );
	}
};

static int sampleManyPyramids = RegisterSample( "Benchmark", "Many Pyramids", BenchmarkManyPyramids::Create );

class BenchmarkRain : public Sample
{
public:
	explicit BenchmarkRain( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 25.0f, 10.0f, 70.0f, b3Pos_zero );
			GetGuiDraw()->drawJoints = false;
		}

		b3Capacity capacity = {};
		GetRainCapacity( &capacity );
		CreateWorld( &capacity );

		CreateRain( m_worldId );
	}

	~BenchmarkRain() override
	{
		DestroyRain();
	}

	void Step() override
	{
		StepRain( m_worldId, m_stepCount );

		// This is for testing adjustable worker count
		// if (m_stepCount == 200)
		//{
		//	m_scheduler->Initialize( 3 );
		//	b3World_SetWorkerCount( m_worldId, 3 );
		//}

		Sample::Step();
	}

	void Render() override
	{
		Sample::Render();
		b3Transform t = { { 0.0f, 0.1f, 0.0f }, b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( t ), 2.0f );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkRain( context );
	}
};

static int sampleRain = RegisterSample( "Benchmark", "Rain", BenchmarkRain::Create );

#if 0
class BenchmarkLargeWorld : public Sample
{
public:
	explicit BenchmarkLargeWorld( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 40.0f, b3Vec3_zero );
		}

		CreateRain( m_worldId );
	}

	~BenchmarkLargeWorld() override
	{
		DestroyRain();
	}

	void Step() override
	{
		StepRain( m_worldId, m_stepCount );

		// This is for testing adjustable worker count
		// if (m_stepCount == 200)
		//{
		//	m_scheduler->Initialize( 3 );
		//	b3World_SetWorkerCount( m_worldId, 3 );
		//}

		Sample::Step();
	}

	void Render() override
	{
		Sample::Render();
		b3Transform transform = { { 0.0f, 0.1f, 0.0f }, b3Quat_identity };
		DrawAxes( transform, 2.0f );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkLargeWorld( context );
	}
};

static int sampleLargeWorld = RegisterSample( "Benchmark", "Large World", BenchmarkLargeWorld::Create );
#endif

class BenchmarkJointGrid : public Sample
{
public:
	explicit BenchmarkJointGrid( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -25.0f, 25.0f, 94.0f, { 30.0f, -30.0f, 30.0f } );
		}

		CreateJointGrid( m_worldId );
	}

	void Render() override
	{
		Sample::Render();
		b3Transform t = { { 0.0f, 0.1f, 0.0f }, b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( t ), 4.0f );

		// transform.p.y = 0.0f;
		// transform.p.z = -5.0f;

		// for (int i = 0; i < 100; ++i)
		//{
		//	transform.p.x = -50.0f;
		//	for (int j = 0; j < 100; ++j)
		//	{
		//		AddSphere( m_scene, transform, 0.5f, b3_colorYellowGreen );
		//		transform.p.x += 1.0f;
		//	}

		//	transform.p.y -= 1.0f;
		//}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkJointGrid( context );
	}
};

static int sampleJointGrid = RegisterSample( "Benchmark", "Joint Grid", BenchmarkJointGrid::Create );

class FallingBoxes : public Sample
{
public:
	explicit FallingBoxes( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 10.0f, 80.0f, { 0.0f, 20.0f, 0.0f } );
		}

		AddGroundBox( 100.0f );

		{
			constexpr int n = m_isDebug ? 4 : 50;
			float a = 0.5f;

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BoxHull box = b3MakeCubeHull( a );

			for ( int i = 0; i < n; ++i )
			{
				for ( int j = 0; j < 8; ++j )
				{
					for ( int k = 0; k < 8; ++k )
					{
						bodyDef.position = { -16.0f * a + 4.0f * a * j, 4.0f * a * i + 5.0f * a, -16.0f * a + 4.0f * a * k };
						b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
						b3CreateHullShape( bodyId, &shapeDef, &box.base );
					}
				}
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new FallingBoxes( context );
	}
};

static int sampleFallingBoxes = RegisterSample( "Benchmark", "Falling Boxes", FallingBoxes::Create );

class CandyCups : public Sample
{
public:
	explicit CandyCups( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			float radius = m_isDebug ? 20.0f : 70.0f;
			m_camera->SetView( 45.0f, 20.0f, radius, b3Pos_zero );
		}

		AddGroundBox( 60.0f );

		{
			constexpr int n = m_isDebug ? 4 : 16;
			constexpr int m = m_isDebug ? 4 : 16;

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			m_convex = CreateConvex( 0.6f, 0.0f, 0.95f, 1.0f );
			for ( int i = 0; i < n; ++i )
			{
				for ( int j = 0; j < m; ++j )
				{
					for ( int k = 0; k < m; ++k )
					{
						bodyDef.position = { -10.0f + 2.5f * j, 1.0f * i, -10.0f + 2.5f * k };
						b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
						b3CreateHullShape( bodyId, &shapeDef, m_convex );
					}
				}
			}
		}
	}

	~CandyCups() override
	{
		b3DestroyHull( m_convex );
	}

	b3HullData* CreateConvex( float radius1, float height1, float radius2, float height2 ) const
	{
		constexpr int sideCount = 8;
		const float deltaAlpha = 2.0f * B3_PI / sideCount;

		int vertexCount = 2 * sideCount;
		b3Vec3 vertexBase[2 * sideCount];

		float alpha = 0.0f;
		for ( int sideIndex = 0; sideIndex < sideCount; ++sideIndex )
		{
			b3CosSin cs = b3ComputeCosSin( alpha );

			float x1 = radius1 * cs.cosine;
			float z1 = radius1 * cs.sine;
			float x2 = radius2 * cs.cosine;
			float z2 = radius2 * cs.sine;

			vertexBase[2 * sideIndex + 0] = { x1, height1, z1 };
			vertexBase[2 * sideIndex + 1] = { x2, height2, z2 };
			alpha += deltaAlpha;
		}

		return b3CreateHull( vertexBase, vertexCount, vertexCount );
	}

	static Sample* Create( SampleContext* context )
	{
		return new CandyCups( context );
	}

	b3HullData* m_convex;
};

static int sampleSmallConvexes = RegisterSample( "Benchmark", "Candy Cups", CandyCups::Create );

class BenchmarkExplosion : public Sample
{
public:
	explicit BenchmarkExplosion( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			float radius = m_isDebug ? 15.0f : 30.0f;
			m_camera->SetView( 45.0f, 20.0f, radius, { 0.0f, 0.0f, 0.0f } );
		}

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		m_gridMesh = b3CreateGridMesh( 40, 40, 1.0f, 0, true );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );
		b3CreateMeshShape( groundId, &shapeDef, m_gridMesh, b3Vec3_one );

		float hy = 1.0f;

		{
			b3Transform transform;
			transform.p = { 0.0f, hy, -20.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 20.0f, hy, 0.1f, transform );
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { 0.0f, hy, 20.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 20.0f, hy, 0.1f, transform );
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { -20.0f, hy, 0.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, hy, 20.0f, transform );
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { 20.0f, hy, 0.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, hy, 20.0f, transform );
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		// Using 15 sides rather than 16 to avoid manifold degeneracies
		m_cylinder = b3CreateCylinder( 0.5f, 0.2f, 0.0f, 15 );

		constexpr int n = m_isDebug ? 3 : 16;

		bodyDef.type = b3_dynamicBody;
		shapeDef.explosionScale = 2.0f;

		for ( int i = -n; i <= n; ++i )
		{
			for ( int k = -n; k <= n; ++k )
			{
				bodyDef.position = { 1.0f * i, 0.0f, 1.0f * k };

				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
				b3CreateHullShape( bodyId, &shapeDef, m_cylinder );
			}
		}

		m_impulse = 1000.0f;

		m_stepWhilePaused = false;
	}

	~BenchmarkExplosion() override
	{
		b3DestroyHull( m_cylinder );
		b3DestroyMesh( m_gridMesh );
	}

	void Explode()
	{
		b3ExplosionDef def = b3DefaultExplosionDef();
		def.radius = 16.0f;
		def.position = { 0.0f, -4.0f, 0.0f };
		def.impulsePerArea = m_impulse;

		b3World_Explode( m_worldId, &def );
	}

	bool DrawControls() override
	{
		ImGui::SliderFloat( "Magnitude", &m_impulse, 0.0f, 2000.0f, "%.0f" );

		if ( ImGui::Button( "Explode" ) )
		{
			Explode();
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkExplosion( context );
	}

	b3MeshData* m_gridMesh;
	b3HullData* m_cylinder;
	float m_impulse;
};

static int sampleExplosion = RegisterSample( "Benchmark", "Explosion", BenchmarkExplosion::Create );

class BenchmarkHeightField : public Sample
{
public:
	struct Context
	{
		b3Pos point;
		b3Vec3 normal;
		float fraction;
		bool hit;
	};

	static float CastCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
							   int triangleIndex, int childIndex, void* context )
	{
		(void)shapeId;
		(void)userMaterialId;
		(void)triangleIndex;
		(void)childIndex;

		Context* rayContext = (Context*)context;
		rayContext->point = point;
		rayContext->normal = normal;
		rayContext->fraction = fraction;
		rayContext->hit = true;
		return fraction;
	}

	explicit BenchmarkHeightField( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 20.0f, 50.0f, b3Pos_zero );
		}

		m_columnCount = 50;
		m_rowCount = 50;
		m_radius = 0.1f;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { -0.5f * m_columnCount, 0.0f, -0.5f * m_rowCount };
		b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

		m_heightField = b3CreateWave( 50, 50, b3Vec3_one, 0.02f, 0.04f, true );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHeightFieldShape( body, &shapeDef, m_heightField );
	}

	~BenchmarkHeightField() override
	{
		b3DestroyHeightField( m_heightField );
	}

	bool DrawControls() override
	{
		ImGui::SliderFloat( "Radius", &m_radius, 0.0f, 1.0f, "%.1f" );
		return true;
	}

	void Render() override
	{
		Sample::Render();

		DrawLine( b3Pos_zero, b3OffsetPos( b3Pos_zero, 0.4f * b3Vec3_axisX ), MakeColor( b3_colorRed ) );
		DrawLine( b3Pos_zero, b3OffsetPos( b3Pos_zero, 0.4f * b3Vec3_axisY ), MakeColor( b3_colorGreen ) );
		DrawLine( b3Pos_zero, b3OffsetPos( b3Pos_zero, 0.4f * b3Vec3_axisZ ), MakeColor( b3_colorBlue ) );

		int hitCount = 0;
		int iterationCount = 0;
		int innerIterationCount = 0;

		float delta = m_isDebug ? 2.0f * 0.95f : 0.4f;

		float spanX = 0.94f * 0.5f * m_columnCount;
		float spanZ = 0.96f * 0.5f * m_rowCount;

		uint64_t startTick = b3GetTicks();

		b3Vec3 rayTranslation = { 80000.0f, -80000.0f, 8.0f };

		int castCount = 0;

		for ( float x = -spanX; x <= spanX; x += delta )
		{
			for ( float z = -spanZ; z <= spanZ; z += delta )
			{
				b3Pos rayOrigin = { x, 2.0f, z };

				b3RayResult result = {};
				if ( m_radius == 0.0f )
				{
					result = b3World_CastRayClosest( m_worldId, rayOrigin, rayTranslation, b3DefaultQueryFilter() );
				}
				else
				{
					Context context = {};
					b3ShapeProxy proxy = { &b3Vec3_zero, 1, m_radius };
					b3World_CastShape( m_worldId, rayOrigin, &proxy, rayTranslation, b3DefaultQueryFilter(), CastCallback,
									   &context );

					if ( context.hit )
					{
						result.point = context.point;
						result.normal = context.normal;
						result.fraction = context.fraction;
						result.hit = true;
					}
				}

				castCount += 1;

				if ( result.hit )
				{
					hitCount += 1;
					if ( m_isDebug )
					{
						b3Pos point = result.point;
						DrawLine( point, point + 0.5f * result.normal, MakeColor( b3_colorGreen ) );
						DrawPoint( point, 10.0f, MakeColor( b3_colorGreen ) );

						if ( m_radius > 0.0f )
						{
							b3WorldTransform t = b3WorldTransform_identity;
							t.p = rayOrigin + result.fraction * rayTranslation;
							DrawSphereEx( t, m_radius, MakeColorAlpha( b3_colorPurple, 0.5f ), 0.0f, 0.5f,
										  TRANSPARENT_SHADOW_NONE );
						}

						b3Pos rayEnd = rayOrigin + result.fraction * rayTranslation;
						DrawLine( rayOrigin, rayEnd, MakeColor( b3_colorYellow ) );
						DrawPoint( rayOrigin, 2.0f, MakeColor( b3_colorRed ) );
						DrawPoint( rayEnd, 2.0f, MakeColor( b3_colorRed ) );
					}
				}
				else if ( m_isDebug )
				{
					b3Pos rayEnd = rayOrigin + rayTranslation;
					DrawLine( rayOrigin, rayEnd, MakeColor( b3_colorYellow ) );
					DrawPoint( rayOrigin, 2.0f, MakeColor( b3_colorRed ) );
					DrawPoint( rayEnd, 2.0f, MakeColor( b3_colorRed ) );
				}
			}
		}

		float milliseconds = b3GetMilliseconds( startTick );
		uint64_t tickCount = b3GetTicks() - startTick;
		float aveIterations = float( iterationCount ) / float( castCount );
		float aveInner = float( innerIterationCount ) / float( castCount );

		float aveCastTime = 1000.0f * milliseconds / float( castCount );

		DrawTextLine( "count = %d, hit count = %d, iterations = %d, inner = %d, ticks = %ld", castCount, hitCount, iterationCount,
					  innerIterationCount, tickCount );

		DrawTextLine( "ave iterations = %.1f, ave inner = %.1f, ave cast us %.3f", aveIterations, aveInner, aveCastTime );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkHeightField( context );
	}

	b3HeightFieldData* m_heightField;
	int m_columnCount;
	int m_rowCount;
	float m_radius;
};

static int sampleHeightFieldBenchmark = RegisterSample( "Benchmark", "Height Field", BenchmarkHeightField::Create );

class BenchmarkFallingTrees : public Sample
{
public:
	explicit BenchmarkFallingTrees( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 20.0f, 0.0f, 140.0f, { 0.0f, 15.0f, 0.0f } );
		}

		m_gridSize = 100;
		CreateTrees100( m_worldId );
	}

	~BenchmarkFallingTrees() override
	{
		DestroyTrees();
	}

	void Generate()
	{
		ResetProfile();

		CreateWorld( nullptr );

		DestroyTrees();

		if ( m_gridSize == 100 )
		{
			CreateTrees100( m_worldId );
		}
		else if ( m_gridSize == 50 )
		{
			CreateTrees50( m_worldId );
		}
		else
		{
			CreateTrees25( m_worldId );
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::RadioButton( "100cm", &m_gridSize, 100 ) )
		{
			Generate();
		}

		if ( ImGui::RadioButton( "50cm", &m_gridSize, 50 ) )
		{
			Generate();
		}

		if ( ImGui::RadioButton( "25cm", &m_gridSize, 25 ) )
		{
			Generate();
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkFallingTrees( context );
	}

	int m_gridSize;
};

static int sampleFallingTrees = RegisterSample( "Benchmark", "Falling Trees", BenchmarkFallingTrees::Create );

struct ShapeUserData
{
	int row;
	bool active;
};

class BenchmarkSensor : public Sample
{
public:
	explicit BenchmarkSensor( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 0.0f, 250.0f, { 0.0f, 110.0f, 0.0f } );
		}

		b3World_SetCustomFilterCallback( m_worldId, FilterFcn, this );

		m_activeSensor.row = 0;
		m_activeSensor.active = true;

		{
			float gridSize = 3.0f;

			// These destroy anything they touch, including themselves.
			b3BoxHull box = b3MakeCubeHull( 0.48f * gridSize );
			b3BodyDef bodyDef = b3DefaultBodyDef();

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;
			shapeDef.userData = &m_activeSensor;
			shapeDef.baseMaterial.customColor = b3MakeDebugColor( (b3HexColor)0x505050u, b3_debugMaterialMetallic );

			float y = 0.0f;
			float x = -40.0f * gridSize;
			for ( int i = 0; i < 81; ++i )
			{
				bodyDef.position = { x, y, 0.0f };
				b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );
				b3CreateHullShape( groundId, &shapeDef, &box.base );
				x += gridSize;
			}
		}

		{
			g_randomSeed = 42;

			float shift = 5.0f;
			float xCenter = 0.5f * shift * m_columnCount;

			b3BodyDef bodyDef = b3DefaultBodyDef();

			b3BoxHull box = b3MakeCubeHull( 0.5f );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;

			float yStart = 10.0f;
			m_filterRow = m_rowCount >> 1;

			for ( int j = 0; j < m_rowCount; ++j )
			{
				m_passiveSensors[j].row = j;
				m_passiveSensors[j].active = false;
				shapeDef.userData = m_passiveSensors + j;

				if ( j == m_filterRow )
				{
					shapeDef.enableCustomFiltering = true;
					shapeDef.baseMaterial.customColor = b3_colorFuchsia;
				}
				else
				{
					shapeDef.enableCustomFiltering = false;
					shapeDef.baseMaterial.customColor = 0;
				}

				float y = j * shift + yStart;
				for ( int i = 0; i < m_columnCount; ++i )
				{
					float x = i * shift - xCenter;
					bodyDef.position = { x, y, 0.0f };
					b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );
					b3CreateHullShape( groundId, &shapeDef, &box.base );
				}
			}
		}

		m_maxBeginCount = 0;
		m_maxEndCount = 0;
		m_lastStepCount = 0;
	}

	void CreateRow( float y )
	{
		float shift = 5.0f;
		float xCenter = 0.5f * shift * m_columnCount;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.gravityScale = 0.0f;
		bodyDef.linearVelocity = { 0.0f, -5.0f };

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableSensorEvents = true;

		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		for ( int i = 0; i < m_columnCount; ++i )
		{
			// stagger bodies to avoid bunching up events into a single update
			float yOffset = RandomFloatRange( -1.0f, 1.0f );
			bodyDef.position = { shift * i - xCenter, y + yOffset, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		}
	}

	void Step() override
	{
		Sample::Step();

		if ( m_stepCount == m_lastStepCount )
		{
			return;
		}

		std::set<b3BodyId> zombies;

		b3SensorEvents events = b3World_GetSensorEvents( m_worldId );
		for ( int i = 0; i < events.beginCount; ++i )
		{
			b3SensorBeginTouchEvent* event = events.beginEvents + i;

			// shapes on begin touch are always valid

			ShapeUserData* userData = static_cast<ShapeUserData*>( b3Shape_GetUserData( event->sensorShapeId ) );
			if ( userData->active )
			{
				zombies.emplace( b3Shape_GetBody( event->visitorShapeId ) );
			}
			else
			{
				// Check custom filter correctness
				assert( userData->row != m_filterRow );

				// Modify color while overlapped with a sensor
				b3SurfaceMaterial surfaceMaterial = b3Shape_GetSurfaceMaterial( event->visitorShapeId );
				surfaceMaterial.customColor = b3_colorLime;
				b3Shape_SetSurfaceMaterial( event->visitorShapeId, surfaceMaterial );
			}
		}

		for ( int i = 0; i < events.endCount; ++i )
		{
			b3SensorEndTouchEvent* event = events.endEvents + i;

			if ( b3Shape_IsValid( event->visitorShapeId ) == false )
			{
				continue;
			}

			// Restore color to default
			b3SurfaceMaterial surfaceMaterial = b3Shape_GetSurfaceMaterial( event->visitorShapeId );
			surfaceMaterial.customColor = 0;
			b3Shape_SetSurfaceMaterial( event->visitorShapeId, surfaceMaterial );
		}

		for ( b3BodyId bodyId : zombies )
		{
			b3DestroyBody( bodyId );
		}

		int delay = 0x1F;

		if ( ( m_stepCount & delay ) == 0 )
		{
			CreateRow( 10.0f + m_rowCount * 5.0f );
		}

		m_lastStepCount = m_stepCount;

		m_maxBeginCount = b3MaxInt( events.beginCount, m_maxBeginCount );
		m_maxEndCount = b3MaxInt( events.endCount, m_maxEndCount );
		DrawTextLine( "max begin touch events = %d", m_maxBeginCount );
		DrawTextLine( "max end touch events = %d", m_maxEndCount );
	}

	bool Filter( b3ShapeId idA, b3ShapeId idB )
	{
		ShapeUserData* userData = nullptr;
		if ( b3Shape_IsSensor( idA ) )
		{
			userData = (ShapeUserData*)b3Shape_GetUserData( idA );
		}
		else if ( b3Shape_IsSensor( idB ) )
		{
			userData = (ShapeUserData*)b3Shape_GetUserData( idB );
		}

		if ( userData != nullptr )
		{
			return userData->active == true || userData->row != m_filterRow;
		}

		return true;
	}

	static bool FilterFcn( b3ShapeId idA, b3ShapeId idB, void* context )
	{
		BenchmarkSensor* self = (BenchmarkSensor*)context;
		return self->Filter( idA, idB );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkSensor( context );
	}

	static constexpr int m_columnCount = 40;
	static constexpr int m_rowCount = 40;
	int m_maxBeginCount;
	int m_maxEndCount;
	ShapeUserData m_passiveSensors[m_rowCount];
	ShapeUserData m_activeSensor;
	int m_lastStepCount;
	int m_filterRow;
};

static int benchmarkSensor = RegisterSample( "Benchmark", "Sensor", BenchmarkSensor::Create );

class BenchmarkWasher : public Sample
{
public:
	explicit BenchmarkWasher( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 15.0f, 20.0f, 60.0, { 0.0f, 15.0f, 0.0f } );
		}

		b3Capacity capacity = {};
		GetWasherCapacity( &capacity );
		CreateWorld( &capacity );

		CreateWasher( m_worldId );
		SetGroundShape( GetGroundShapeId() );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkWasher( context );
	}
};

static int benchmarkWasher = RegisterSample( "Benchmark", "Washer", BenchmarkWasher::Create );

class BenchmarkLargeWorld : public Sample
{
public:
	explicit BenchmarkLargeWorld( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 10.0f, 250.0f, b3Pos_zero );
		}

		b3Capacity capacity = {};
		GetLargeWorldCapacity( &capacity );
		CreateWorld( &capacity );

		CreateLargeWorld( m_worldId );
	}

	void Step() override
	{
		StepLargeWorld( m_worldId, m_stepCount );
		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkLargeWorld( context );
	}
};

static int sampleLargeWorld = RegisterSample( "Benchmark", "Large World", BenchmarkLargeWorld::Create );

class BenchmarkHull : public Sample
{
public:
	explicit BenchmarkHull( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 5.0f, b3Pos_zero );
		}

		g_randomSeed = 42;

		m_count = 64;
		for ( int i = 0; i < m_count; ++i )
		{
			m_points[i] = RandomVec3( { -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f } );
		}

		m_hull = b3CreateHull( m_points, m_count, m_count );
		m_scale = { -1.0f, 1.0f, 1.0f };
		m_transformedHull = b3CloneAndTransformHull( m_hull, b3Transform_identity, m_scale );
	}

	~BenchmarkHull() override
	{
		b3DestroyHull( m_hull );
		b3DestroyHull( m_transformedHull );
	}

	void Render() override
	{
		b3Transform t1 = { { -2.0f, 0.0f, 0.0f }, b3Quat_identity };
		b3Transform t2 = { { 2.0f, 0.0f, 0.0f }, b3Quat_identity };

		DrawHull( b3MakeWorldTransform( t1 ), m_hull, MakeColor( b3_colorGreen ) );
		DrawHull( b3MakeWorldTransform( t2 ), m_transformedHull, MakeColor( b3_colorYellow ) );

		Sample::Render();
	}

	void Step() override
	{
		uint64_t startTick = b3GetTicks();
		float area = 0.0f;
		int trials = m_isDebug ? 200 : 2000;

		for ( int i = 0; i < trials; ++i )
		{
			b3HullData* hull = b3CreateHull( m_points, m_count, m_count );
			area += hull->surfaceArea;
			b3DestroyHull( hull );
		}

		float createTime = b3GetMillisecondsAndReset( &startTick );

		float scaledArea = 0.0f;
		for ( int i = 0; i < trials; ++i )
		{
			b3HullData* hull = b3CloneAndTransformHull( m_hull, b3Transform_identity, m_scale );
			scaledArea += hull->surfaceArea;
			b3DestroyHull( hull );
		}

		float cloneTime = b3GetMilliseconds( startTick );

		DrawTextLine( "trials = %d", trials );
		DrawTextLine( "createTime (us) = %.2f, area = %.2f", 1000.0f * createTime / trials, area / trials );
		DrawTextLine( "cloneTime (us) = %.2f, area = %.2f", 1000.0f * cloneTime / trials, scaledArea / trials );
		DrawTextLine( "createTime / cloneTime = %.2f", createTime / cloneTime );
		Sample::Step();
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new BenchmarkHull( sampleContext );
	}

	static constexpr int m_capacity = 64;
	b3HullData* m_hull;
	b3HullData* m_transformedHull;
	b3Vec3 m_scale;
	b3Vec3 m_points[m_capacity];
	int m_count;
};

static int sampleBenchmarkHull = RegisterSample( "Benchmark", "Hull", BenchmarkHull::Create );

class BenchmarkChains : public Sample
{
public:
	explicit BenchmarkChains( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 50.0f, { 0.0f, 5.0f, 0.0f } );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

		m_meshData = b3CreateWaveMesh( 80, 80, 1.0f, 0.5f, 0.05f, 0.01f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateMeshShape( groundId, &shapeDef, m_meshData, b3Vec3_one );

		float linkRadius = 0.125f;
		float linkExtent = 0.25f;
		b3Capsule capsule = { { 0.0f, -linkExtent, 0.0f }, { 0.0f, linkExtent, 0.0f }, linkRadius };

		bodyDef.enableSleep = false;

		int linkCount = 4;

		b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
		jointDef.base.localFrameA = { { 0.0f, -linkExtent, 0.0f }, b3Quat_identity };
		jointDef.base.localFrameB = { { 0.0f, linkExtent, 0.0f }, b3Quat_identity };
		jointDef.enableSpring = true;
		jointDef.hertz = 1.0f;
		jointDef.dampingRatio = 0.7f;
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 1.0f;

		int shapeIndex = 0;

		float x = -1.0f * m_gridCount;
		for ( int rowIndex = 0; rowIndex < m_gridCount; ++rowIndex )
		{
			float z = -1.0f * m_gridCount;
			for ( int columnIndex = 0; columnIndex < m_gridCount; ++columnIndex )
			{
				for ( int i = 0; i < linkCount; ++i )
				{
					bodyDef.position = { x, ( 1.0f - 2.0f * i ) * linkExtent + 3.0f, z };

					bodyDef.type = i == 0 ? b3_staticBody : b3_dynamicBody;

					b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

					b3ShapeId shapeId = b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
					if ( i == linkCount - 1 )
					{
						m_shapeIds[shapeIndex] = shapeId;
						shapeIndex += 1;
					}

					if ( i > 0 )
					{
						jointDef.base.bodyIdB = bodyId;
						b3CreateSphericalJoint( m_worldId, &jointDef );
					}

					jointDef.base.bodyIdA = bodyId;
				}

				z += 2.0f;
			}

			x += 2.0f;
		}

		m_noise = {};
	}

	~BenchmarkChains() override
	{
		b3DestroyMesh( m_meshData );
	}

	void Step() override
	{
		b3Vec3 baseWind = { 20.0f, 0.0f, 0.0f };
		float speed;
		b3Vec3 direction = b3GetLengthAndNormalize( &speed, baseWind );
		b3Vec3 wind = b3MulSV( speed, b3Add( direction, m_noise ) );

		for ( int i = 0; i < m_gridCount * m_gridCount; ++i )
		{
			b3Shape_ApplyWind( m_shapeIds[i], wind, 1.0f, 1.0f, 20.0f, false );
		}

		b3Vec3 rand = RandomVec3( { -0.3f, -0.3f, -0.3f }, { 0.3f, 0.3f, 0.3f } );
		m_noise = b3Lerp( m_noise, rand, 0.05f );

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkChains( context );
	}

	static constexpr int m_gridCount = m_isDebug ? 10 : 25;
	b3ShapeId m_shapeIds[m_gridCount * m_gridCount];
	b3MeshData* m_meshData;
	b3Vec3 m_noise;
};

static int sampleBenchmarkChains = RegisterSample( "Benchmark", "Chains", BenchmarkChains::Create );

class BenchmarkDestruction : public Sample
{
public:
	explicit BenchmarkDestruction( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			if ( m_small )
			{
				m_camera->SetView( 0.0f, 40.0f, 20.0f, { 0.0f, 0.0f, 0.0f } );
			}
			else
			{
				m_camera->SetView( 0.0f, 40.0f, 30.0f, { 0.0f, 0.0f, 0.0f } );
			}
		}

		int bodyCapacity = m_gridCount * m_gridCount * m_gridCount;

		b3Capacity capacity = {};
		capacity.dynamicShapeCount = bodyCapacity;
		capacity.dynamicBodyCount = bodyCapacity;
		capacity.contactCount = m_small ? 10000 : 50000;
		CreateWorld( &capacity );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		m_gridMesh = b3CreateGridMesh( 40, 40, 1.0f, 0, true );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );
		b3CreateMeshShape( groundId, &shapeDef, m_gridMesh, b3Vec3_one );

#if 0
		{
			b3Transform transform;
			transform.p = { 0.0f, 5.0f, -20.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 20.0f, 5.0f, 0.1f, transform );
			shapeDef.name = "wall1";
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { 0.0f, 5.0f, 20.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 20.0f, 5.0f, 0.1f, transform );
			shapeDef.name = "wall2";
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { -20.0f, 5.0f, 0.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, 5.0f, 20.0f, transform );
			shapeDef.name = "wall3";
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { 20.0f, 5.0f, 0.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, 5.0f, 20.0f, transform );
			shapeDef.name = "wall4";
			b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
		}
#endif

		m_bodyIds = (b3BodyId*)malloc( bodyCapacity * sizeof( b3BodyId ) );
		memset( m_bodyIds, 0, bodyCapacity * sizeof( b3BodyId ) );
		m_bodyCount = 0;

		m_explosionDef = b3DefaultExplosionDef();
		m_explosionDef.radius = m_extent;
		m_explosionDef.falloff = 0.5f * m_extent;
		m_explosionDef.position = { 0.0f, 2.0f * m_extent, 0.0f };
		m_explosionDef.impulsePerArea = m_small ? 200.0f : 1000.0f;

		m_spawnMilliseconds = 0.0f;
		m_destroyMilliseconds = 0.0f;

		Spawn();
	}

	~BenchmarkDestruction() override
	{
		b3DestroyMesh( m_gridMesh );
		free( m_bodyIds );
	}

	void DestroyBodies()
	{
		uint64_t ticks = b3GetTicks();

		for ( int i = 0; i < m_bodyCount; ++i )
		{
			b3DestroyBody( m_bodyIds[i] );
		}

		m_destroyMilliseconds = b3GetMilliseconds( ticks );
	}

	void Spawn()
	{
		uint64_t ticks = b3GetTicks();

		float a = m_extent / m_gridCount;
		b3BoxHull box = b3MakeBoxHull( 0.8f * a, 0.8f * a, 0.8f * a );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		int randomRange = m_small ? 3 : 2;
		m_bodyCount = 0;
		for ( int i = 0; i < m_gridCount; ++i )
		{
			for ( int j = 0; j < m_gridCount; ++j )
			{
				for ( int k = 0; k < m_gridCount; ++k )
				{
					if ( RandomIntRange( 1, randomRange ) == 1 )
					{
						continue;
					}

					bodyDef.position = { ( 2.0f * i - m_gridCount + 1.0f ) * a, ( 2.0f * j + 1.0f ) * a,
										 ( 2.0f * k - m_gridCount + 1.0f ) * a };

					b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
					b3CreateHullShape( bodyId, &shapeDef, &box.base );

					m_bodyIds[m_bodyCount] = bodyId;
					m_bodyCount += 1;
				}
			}
		}

		b3World_Explode( m_worldId, &m_explosionDef );

		m_spawnMilliseconds = b3GetMilliseconds( ticks );
	}

	void Render() override
	{
		Sample::Render();

		DrawTextLine( "spawn = %.2f ms", m_spawnMilliseconds );
		DrawTextLine( "destroy = %.2f ms", m_destroyMilliseconds );

		float r = m_explosionDef.radius;
		b3Sphere sphere1 = { b3Vec3_zero, r };
		DrawWireSphere( { m_explosionDef.position, b3Quat_identity }, &sphere1, 24, MakeColor( b3_colorAqua ) );

		float rf = r + m_explosionDef.falloff;
		b3Sphere sphere2 = { b3Vec3_zero, rf };
		DrawWireSphere( { m_explosionDef.position, b3Quat_identity }, &sphere2, 24, MakeColor( b3_colorCornsilk ) );
	}

	void Step() override
	{
		Sample::Step();

		int spawnStep = m_small ? 80 : 140;
		if ( m_stepCount % spawnStep == 0 )
		{
			DestroyBodies();
			Spawn();
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkDestruction( context );
	}

	static constexpr bool m_small = m_isDebug;
	static constexpr int m_gridCount = m_small ? 6 : 20;
	static constexpr float m_extent = m_small ? 0.75f : 2.5f;

	b3BodyId* m_bodyIds;
	int m_bodyCount;
	b3ExplosionDef m_explosionDef;
	b3MeshData* m_gridMesh;
	float m_spawnMilliseconds;
	float m_destroyMilliseconds;
};

static int sampleDestruction = RegisterSample( "Benchmark", "Destruction", BenchmarkDestruction::Create );

class BenchmarkJunkyard : public Sample
{
public:
	explicit BenchmarkJunkyard( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 125.0f, b3Pos_zero );
			GetGuiDraw()->drawJoints = false;
		}

		CreateJunkyard( m_worldId );

		SetGroundShape( GetGroundShapeId() );
	}

	void Step() override
	{
		if ( m_context->pause == false || m_context->singleStep == 0 )
		{
			StepJunkyard( m_worldId, m_stepCount );
		}

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkJunkyard( context );
	}
};

static int sampleJunkyard = RegisterSample( "Benchmark", "Junkyard", BenchmarkJunkyard::Create );
