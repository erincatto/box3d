// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "imgui.h"
#include "utils.h"
#include "sample.h"
#include "gfx/draw.h"

#include "box3d/box3d.h"

#include <queue>
#include <stdio.h>
#include <unordered_map>
#include <vector>

constexpr int colorCount = 12;
b3HexColor colors[colorCount] = {
	b3_colorRoyalBlue, b3_colorPink,  b3_colorLightGreen, b3_colorOrange,	b3_colorYellow,	  b3_colorViolet,
	b3_colorAqua,	   b3_colorCoral, b3_colorLightGray,  b3_colorMoccasin, b3_colorLavender, b3_colorMediumOrchid,
};

struct FileConfig
{
	float scale;
	bool zUp;
};

struct Proxy
{
	b3AABB aabb;
	int proxyId;
	int queryTimeStamp;
	int rayTimeStamp;
};

struct Ray
{
	b3Vec3 origin, translation;
};

class TreeBenchmark : public Sample
{
public:
	explicit TreeBenchmark( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 45.0f, 250.0f, b3Pos_zero );
		}

		m_fileNames[0] = { "bounds01" };
		m_fileNames[1] = { "bounds02" };
		m_fileNames[2] = { "bounds03" };
		m_config[0] = { 1.0f, false };
		m_config[1] = { 1.0f, false };
		m_config[2] = { 0.01f, false };
		m_fileIndex = 0;

		m_closestPoint = b3Pos_zero;
		m_timeStamp = 1;
		m_doOverlap = false;
		m_doClosest = false;
		m_doRay = false;
		m_testIndex = 0;
		m_rayTime = 0.0f;
		m_overlapTime = 0.0f;
		m_closestTime = 0.0f;
		m_haveClosest = false;
		m_buildTime = 0.0f;
		m_areaRatio = 0.0f;
		m_drawDistance = 1.0f;
		m_loadScale = 1.0f;
		m_height = 0;
		m_tree = {};

		CreateTree();
	}

	~TreeBenchmark() override
	{
		b3DynamicTree_Destroy( &m_tree );
	}

	// Breath-first search to compute node depth values start with 0 at the root.
	int ComputeDepths()
	{
		m_depths.resize( m_tree.nodeCapacity, 0 );

		int* queue = (int*)malloc( m_tree.nodeCount * sizeof( int ) );
		int front = 0;
		int back = 0;

		b3TreeNode* nodes = m_tree.nodes;
		queue[0] = m_tree.root;
		back += 1;
		int depth = 0;

		while ( back > front )
		{
			int index = queue[front];
			front += 1;

			b3TreeNode* node = nodes + index;
			if ( node->parent == B3_NULL_INDEX )
			{
				m_depths[index] = 0;
			}
			else
			{
				m_depths[index] = m_depths[node->parent] + 1;
				depth = b3MaxInt( depth, m_depths[index] );
			}

			if ( ( node->flags & b3_leafNode ) == 0 )
			{
				B3_ASSERT( back < m_tree.nodeCount - 1 );
				queue[back++] = node->children.child1;
				queue[back++] = node->children.child2;
			}
		}

		free( queue );

		return depth;
	}

	void CreateTree()
	{
		if ( m_tree.nodeCapacity > 0 )
		{
			b3DynamicTree_Destroy( &m_tree );
			m_tree = {};
		}

		m_proxies.clear();

		m_tree = b3DynamicTree_Create( 512 );

		char filePath[64] = {};
		snprintf( filePath, 64, "data/trees/%s.txt", m_fileNames[m_fileIndex] );
		float scale = m_config[m_fileIndex].scale;
		bool zUp = m_config[m_fileIndex].zUp;

		FILE* file = fopen( filePath, "r" );
		if ( file == nullptr )
		{
			return;
		}

		constexpr int bufferSize = 512;
		char buffer[bufferSize];

		int maxArea = 0.0f;
		int maxAreaIndex = -1;
		uint64_t key = 0;

		while ( fgets( buffer, sizeof( buffer ), file ) )
		{
			char* trim = buffer;
			while ( *trim == ' ' || *trim == '\t' )
			{
				++trim;
			}

			if ( *trim == '\0' || *trim == '\n' || *trim == '#' )
			{
				continue;
			}

			Proxy rec = {};
			float b1, b2, b3, b4, b5, b6;
			int parsed = sscanf( buffer, "%f %f %f %f %f %f", &b1, &b2, &b3, &b4, &b5, &b6 );

			if ( parsed != 6 )
			{
				fprintf( stderr, "Skipping malformed line: %s", buffer );
				continue;
			}

			if ( zUp )
			{
				rec.aabb.lowerBound.x = scale * b1;
				rec.aabb.lowerBound.y = scale * b3;
				rec.aabb.lowerBound.z = scale * b2;
				rec.aabb.upperBound.x = scale * b4;
				rec.aabb.upperBound.y = scale * b6;
				rec.aabb.upperBound.z = scale * b5;
			}
			else
			{
				rec.aabb.lowerBound.x = scale * b1;
				rec.aabb.lowerBound.y = scale * b2;
				rec.aabb.lowerBound.z = scale * b3;
				rec.aabb.upperBound.x = scale * b4;
				rec.aabb.upperBound.y = scale * b5;
				rec.aabb.upperBound.z = scale * b6;
			}

			float area = b3AABB_Area( rec.aabb );
			if ( area > maxArea )
			{
				maxArea = area;
				maxAreaIndex = (int)m_proxies.size();
			}

			m_proxies[key] = rec;
			key += 1;

			// if ( m_proxies.size() == 320 )
			//{
			//	break;
			// }
		}

		printf( "max index = %d\n", maxAreaIndex );

		// constexpr int n = m_isDebug ? 1000 : INT_MAX;

		uint64_t ticks = b3GetTicks();
		int count = (int)m_proxies.size();
		for ( int i = 0; i < count; ++i )
		{
			// Arbitrary category bits to ensure correctness
			m_proxies[i].proxyId = b3DynamicTree_CreateProxy( &m_tree, m_proxies[i].aabb, 1, i );
			m_proxies[i].queryTimeStamp = 0;
			m_proxies[i].rayTimeStamp = 0;
		}

		m_buildTime = b3GetMilliseconds( ticks );
		b3DynamicTree_Validate( &m_tree );
		m_areaRatio = b3DynamicTree_GetAreaRatio( &m_tree );

		m_height = ComputeDepths();
		m_drawLevel = -1;

		Generate();
	}

	void LoadTree()
	{
		if ( m_tree.nodeCapacity > 0 )
		{
			b3DynamicTree_Destroy( &m_tree );
			m_tree = {};
		}

		m_proxies.clear();

		{
			//bool zUp = true;
			m_tree = b3DynamicTree_Load( m_saveFileName, m_loadScale );
		}

		constexpr int bufferSize = 512;
		char buffer[bufferSize];

		int maxArea = 0.0f;
		int maxAreaIndex = -1;

		for ( int i = 0; i < m_tree.nodeCapacity; ++i )
		{
			b3TreeNode* node = m_tree.nodes + i;

			if ( ( node->flags & b3_leafNode ) == 0 )
			{
				continue;
			}

			assert( node->flags & b3_allocatedNode );

			Proxy proxy = {};
			proxy.aabb = node->aabb;

			float area = b3AABB_Area( proxy.aabb );
			if ( area > maxArea )
			{
				maxArea = area;
				maxAreaIndex = (int)m_proxies.size();
			}

			proxy.proxyId = i;
			m_proxies[node->userData] = proxy;
		}

		printf( "max index = %d\n", maxAreaIndex );

		// constexpr int n = m_isDebug ? 1000 : INT_MAX;

		m_buildTime = 0.0f;
		b3DynamicTree_Validate( &m_tree );
		m_areaRatio = b3DynamicTree_GetAreaRatio( &m_tree );

		m_height = ComputeDepths();
		m_drawLevel = -1;

		Generate();
	}

	static float RayCallback( const b3RayCastInput* input, int proxyId, uint64_t userData, void* context )
	{
		TreeBenchmark* sample = static_cast<TreeBenchmark*>( context );
		Proxy& proxy = sample->m_proxies[userData];
		proxy.rayTimeStamp = sample->m_timeStamp;
		return 1.0f;
	}

	static bool QueryCallback( int proxyId, uint64_t userData, void* context )
	{
		TreeBenchmark* sample = static_cast<TreeBenchmark*>( context );
		Proxy& proxy = sample->m_proxies[userData];
		proxy.queryTimeStamp = sample->m_timeStamp;
		return true;
	}

	static float ClosetPointCallback( float minDistanceSquared, int proxyId, uint64_t userData, void* context )
	{
		(void)proxyId;

		TreeBenchmark* sample = static_cast<TreeBenchmark*>( context );
		int testIndex = sample->m_testIndex;
		Proxy& proxy = sample->m_proxies[userData];
		proxy.queryTimeStamp = sample->m_timeStamp;
		b3Vec3 queryPoint = sample->m_closestPointQueries[testIndex].center;
		b3Vec3 closestPoint = b3ClosestPointToAABB( queryPoint, proxy.aabb );
		float distanceSquared = b3DistanceSquared( queryPoint, closestPoint );
		if ( distanceSquared < minDistanceSquared )
		{
			sample->m_closestPoint = b3ToPos( closestPoint );
			sample->m_haveClosest = true;
		}

		return distanceSquared;
	}

	void Generate()
	{
		srand( 42 );

		b3AABB bounds = m_tree.nodes[m_tree.root].aabb;
		b3Vec3 extents = b3AABB_Extents( bounds );
		float radius = ( extents.x + extents.y + extents.z ) / 3.0f;

		for ( int i = 0; i < m_testCount; ++i )
		{
			m_rays[i].origin = RandomVec3( bounds.lowerBound, bounds.upperBound );

			b3Vec3 end = RandomVec3( bounds.lowerBound, bounds.upperBound );
			m_rays[i].translation = end - m_rays[i].origin;

			float s = RandomFloatRange( 0.01f, 0.2f );
			b3Vec3 c = RandomVec3( bounds.lowerBound, bounds.upperBound );
			b3Vec3 p1 = c - s * extents;
			b3Vec3 p2 = c + s * extents;

			m_overlapQueries[i].lowerBound = b3Min( p1, p2 );
			m_overlapQueries[i].upperBound = b3Max( p1, p2 );

			m_closestPointQueries[i] = {
				.center = c,
				.radius = s * radius,
			};
		}
	}

	void Profile()
	{
		uint64_t ticks = b3GetTicks();
		for ( int i = 0; i < m_testCount; ++i )
		{
			Ray ray = m_rays[i];
			b3RayCastInput input = { ray.origin, ray.translation, 1.0f };

			b3DynamicTree_RayCast( &m_tree, &input, B3_DEFAULT_MASK_BITS, false, RayCallback, this );
		}
		m_rayTime = b3GetMillisecondsAndReset( &ticks );

		for ( int i = 0; i < m_testCount; ++i )
		{
			b3DynamicTree_Query( &m_tree, m_overlapQueries[i], B3_DEFAULT_MASK_BITS, false, QueryCallback, this );
		}
		m_overlapTime = b3GetMilliseconds( ticks );

		for ( int i = 0; i < m_testCount; ++i )
		{
			b3Vec3 point = m_closestPointQueries[i].center;
			float radius = m_closestPointQueries[i].radius;
			float distanceSquared = radius * radius;
			m_closestPoint = b3ToPos( point );
			m_haveClosest = false;
			b3DynamicTree_QueryClosest( &m_tree, point, B3_DEFAULT_MASK_BITS, false, ClosetPointCallback, this,
										&distanceSquared );
		}

		m_closestTime = b3GetMilliseconds( ticks );
	}

	bool HasSolverControls() const override
	{
		return false;
	}

	bool DrawControls() override
	{
		DrawTextLine( "leaves = %d, height = %d, area = %g", m_tree.proxyCount, m_height, m_areaRatio );
		DrawTextLine( "build time = %g ms", m_buildTime );
		DrawTextLine( "total: ray = %g ms, overlap = %g ms, closest = %g ms", m_rayTime, m_overlapTime, m_closestTime );

		float s = 1000.0f / m_testCount;
		DrawTextLine( "ave: ray = %g us, overlap = %g us, closest = %g us", s * m_rayTime, s * m_overlapTime, s * m_closestTime );

		if ( ImGui::Combo( "File", &m_fileIndex, m_fileNames, m_fileCount ) )
		{
			CreateTree();
		}

		if ( ImGui::Button( "Top Down" ) )
		{
			uint64_t ticks = b3GetTicks();
			b3DynamicTree_Rebuild( &m_tree, true );
			m_buildTime = b3GetMilliseconds( ticks );
			m_areaRatio = b3DynamicTree_GetAreaRatio( &m_tree );
			m_height = ComputeDepths();
		}

		ImGui::Separator();

		ImGui::Checkbox( "Ray Cast", &m_doRay );
		ImGui::Checkbox( "Overlap", &m_doOverlap );
		ImGui::Checkbox( "Closet Point", &m_doClosest );

		if ( ImGui::Button( "Profile" ) )
		{
			Profile();
		}

		ImGui::SliderInt( "Test", &m_testIndex, 0, m_testCount - 1 );
		ImGui::SliderInt( "Level", &m_drawLevel, -1, m_height );
		ImGui::SliderFloat( "Kilometers", &m_drawDistance, 0.5f, 20.0f, "%.1f" );

		if ( ImGui::Button( "Save" ) )
		{
			b3DynamicTree_Save( &m_tree, m_saveFileName );
		}

		if ( ImGui::Button( "Load" ) )
		{
			LoadTree();
		}

		ImGui::SliderFloat( "Load Scale", &m_loadScale, 0.01f, 1.0f );

		return true;
	}

	void Render() override
	{
		Sample::Render();

		DrawAxes( b3WorldTransform_identity, 2.0f );

		b3Pos cp = m_camera->DrawOrigin();
		b3TreeNode* nodes = m_tree.nodes;
		float distSquared = m_drawDistance * m_drawDistance * 1000.0f * 1000.0f;

		if ( m_drawLevel >= 0 )
		{
			constexpr int colorCount = 20;
			b3HexColor colors[colorCount] = {
				b3_colorAliceBlue, b3_colorAntiqueWhite,   b3_colorAqua,		   b3_colorAquamarine, b3_colorAzure,
				b3_colorBeige,	   b3_colorBisque,		   b3_colorBlanchedAlmond, b3_colorBlue,	   b3_colorBlueViolet,
				b3_colorBrown,	   b3_colorBurlywood,	   b3_colorCadetBlue,	   b3_colorChartreuse, b3_colorChocolate,
				b3_colorCoral,	   b3_colorCornflowerBlue, b3_colorCornsilk,	   b3_colorCrimson,	   b3_colorCyan,
			};

			int capacity = m_tree.nodeCapacity;
			for ( int i = 0; i < capacity; ++i )
			{
				b3TreeNode* node = nodes + i;
				if ( m_depths[i] != m_drawLevel || ( node->flags & b3_allocatedNode ) == 0 )
				{
					// skip internal nodes
					continue;
				}

				b3Vec3 c = b3AABB_Center( node->aabb );
				if ( m_drawLevel < 10 || b3LengthSquared( b3SubPos( cp, b3ToPos( c ) ) ) < distSquared )
				{
					DrawBounds( node->aabb, 0.0f, MakeColor( colors[m_drawLevel % colorCount] ) );
				}
			}
		}
		else
		{
			uint16_t requiredFlags = b3_allocatedNode | b3_leafNode;
			for ( int i = 0; i < m_tree.nodeCapacity; ++i )
			{
				b3TreeNode* node = nodes + i;

				if ( node->flags != requiredFlags )
				{
					continue;
				}

				b3Vec3 c = b3AABB_Center( node->aabb );
				if ( b3LengthSquared( b3SubPos( cp, b3ToPos( c ) ) ) > distSquared )
				{
					continue;
				}

				Proxy& proxy = m_proxies[node->userData];
				if ( proxy.queryTimeStamp == m_timeStamp || proxy.rayTimeStamp == m_timeStamp )
				{
					DrawBounds( node->aabb, 0.0f, MakeColor( b3_colorLightGray ) );
				}
				else
				{
					// float extension = 0.01f * node->height;
					float extension = 0.0f;
					DrawBounds( node->aabb, extension, MakeColor( b3_colorLightBlue ) );
				}
			}
		}

		if ( m_doRay )
		{
			Ray ray = m_rays[m_testIndex];
			DrawLine( b3ToPos( ray.origin ), b3ToPos( ray.origin + ray.translation ), MakeColor( b3_colorRed ) );
		}

		if ( m_doOverlap )
		{
			DrawBounds( m_overlapQueries[m_testIndex], 0.0f, MakeColor( b3_colorRed ) );
		}

		if ( m_doClosest )
		{
			DrawSolidSphere( b3WorldTransform_identity, m_closestPointQueries[m_testIndex], MakeColor( b3_colorCyan ) );
			if ( m_haveClosest )
			{
				DrawPoint( m_closestPoint, 15.0f, MakeColor( b3_colorOrange ) );
			}
		}
	}

	void Step() override
	{
		Sample::Step();
		++m_timeStamp;
		
		if ( m_doRay )
		{
			Ray ray = m_rays[m_testIndex];
			b3RayCastInput input = { ray.origin, ray.translation, 1.0f };

			b3DynamicTree_RayCast( &m_tree, &input, B3_DEFAULT_MASK_BITS, false, RayCallback, this );
		}

		if ( m_doOverlap )
		{
			b3DynamicTree_Query( &m_tree, m_overlapQueries[m_testIndex], B3_DEFAULT_MASK_BITS, false, QueryCallback, this );
		}

		if ( m_doClosest )
		{
			b3Vec3 point = m_closestPointQueries[m_testIndex].center;
			float radius = m_closestPointQueries[m_testIndex].radius;
			float distanceSquared = radius * radius;
			m_closestPoint = b3ToPos( point );
			m_haveClosest = false;
			b3DynamicTree_QueryClosest( &m_tree, point, B3_DEFAULT_MASK_BITS, false, ClosetPointCallback, this,
										&distanceSquared );
		}

		// constexpr int n = m_isDebug ? 0 : 100;
		// for ( int i = 0; i < n; ++i )
		//{
		//	Profile();
		// }
	}

	static Sample* Create( SampleContext* context )
	{
		return new TreeBenchmark( context );
	}

	static constexpr int m_fileCount = 3;
	static constexpr int m_testCount = 1024;
	static constexpr const char* m_saveFileName = "tree.dt";

	const char* m_fileNames[m_fileCount];
	FileConfig m_config[m_fileCount];
	int m_fileIndex;

	std::unordered_map<uint64_t, Proxy> m_proxies;
	std::vector<uint16_t> m_depths;
	b3DynamicTree m_tree;
	Ray m_rays[m_testCount];
	b3AABB m_overlapQueries[m_testCount];
	b3Sphere m_closestPointQueries[m_testCount];
	b3Pos m_closestPoint;
	float m_areaRatio;
	float m_rayTime;
	float m_overlapTime;
	float m_closestTime;
	float m_buildTime;
	float m_drawDistance;
	float m_loadScale;
	int m_testIndex;
	int m_timeStamp;
	int m_drawLevel;
	int m_height;
	bool m_haveClosest;
	bool m_doOverlap;
	bool m_doClosest;
	bool m_doRay;
};

static int sampleTree = RegisterSample( "Tree", "Benchmark", TreeBenchmark::Create );
