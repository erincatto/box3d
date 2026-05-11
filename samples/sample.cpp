// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "sample.h"

#include "camera.h"
#include "imgui.h"
#include "implot.h"
#include "jsmn.h"
#include "renderer.h"
#include "scene.h"
#include "shader.h"
#include "utils.h"

// clang-format off
#include "glad/glad.h"
#include "GLFW/glfw3.h"
// clang-format on

#include "human.h"

#include "box3d/box3d.h"

#include <stdio.h>
#include <stdlib.h>

// Load a file. You must free the character array.
static char* ReadFile( int& size, const char* filename )
{
	FILE* file = fopen( filename, "rb" );
	if ( file == nullptr )
	{
		return nullptr;
	}

	fseek( file, 0, SEEK_END );
	size = static_cast<int>( ftell( file ) );
	fseek( file, 0, SEEK_SET );

	if ( size == 0 )
	{
		fclose( file );
		return nullptr;
	}

	char* data = (char*)malloc( size + 1 );
	fread( data, size, 1, file );
	fclose( file );
	data[size] = 0;

	return data;
}

static const char* settingsFileName = "settings.ini";

void Arena::Create( int byteCount )
{
	data = (char*)malloc( byteCount );
	capacity = byteCount;
	index = 0;
}

void Arena::Destroy()
{
	free( data );
	data = nullptr;
	capacity = 0;
	index = 0;
}

void* Arena::Allocate( int byteCount )
{
	if ( index + byteCount > capacity )
	{
		assert( false );
		return nullptr;
	}

	void* mem = data + index;
	index += byteCount;
	return mem;
}

void Arena::Clear()
{
	index = 0;
}

static bool DrawShapeCallback( void* userShape, b3Transform transform, b3HexColor color, void* context )
{
	DebugShape* shape = (DebugShape*)userShape;
	SampleContext* sampleContext = (SampleContext*)context;
	float alpha = sampleContext->transparentDynamic && shape->bodyType != b3_staticBody ? 0.6f : 1.0f;
	DrawDebugShape( sampleContext->scene, shape, transform, color, alpha );
	return true;
}

static void DrawPointCallback( b3Vec3 point, float size, b3HexColor color, void* context )
{
	SampleContext* sampleContext = (SampleContext*)context;
	DrawPoint( sampleContext->scene, point, size, color );
}

static void DrawLineCallback( b3Vec3 vertex1, b3Vec3 vertex2, b3HexColor color, void* context )
{
	SampleContext* sampleContext = (SampleContext*)context;
	DrawLine( sampleContext->scene, vertex1, vertex2, color );
}

static void DrawBoundsCallback( b3AABB bounds, b3HexColor color, void* context )
{
	SampleContext* sampleContext = (SampleContext*)context;
	float extension = 0.0f;
	DrawBounds( sampleContext->scene, bounds, extension, color );
}

static void DrawBoxCallback( b3Vec3 extents, b3Transform transform, b3HexColor color, void* context )
{
	SampleContext* sampleContext = (SampleContext*)context;
	DrawBox( sampleContext->scene, extents, transform, color );
}

static void DrawTransformCallback( b3Transform transform, void* context )
{
	SampleContext* sampleContext = (SampleContext*)context;
	DrawTransform( sampleContext->scene, transform, 0.25f );
}

static void DrawSceneTextCallback( b3Vec3 position, const char* text, b3HexColor color, void* context )
{
	SampleContext* sampleContext = (SampleContext*)context;
	DrawWorldString( &sampleContext->camera, position, color, text );
}

void SampleContext::Save()
{
	FILE* file = fopen( settingsFileName, "w" );
	fprintf( file, "{\n" );
	fprintf( file, "  \"sampleIndex\": %d,\n", sampleIndex );
	fprintf( file, "  \"drawShapes\": %s,\n", debugDraw.drawShapes ? "true" : "false" );
	fprintf( file, "  \"drawJoints\": %s,\n", debugDraw.drawJoints ? "true" : "false" );
	fprintf( file, "  \"drawContacts\": %s,\n", debugDraw.drawContacts ? "true" : "false" );
	fprintf( file, "  \"drawImpulses\": %s,\n", debugDraw.drawContactForces ? "true" : "false" );
	fprintf( file, "  \"drawBounds\": %s,\n", debugDraw.drawBounds ? "true" : "false" );
	fprintf( file, "  \"drawCounters\": %s,\n", drawCounters ? "true" : "false" );
	fprintf( file, "  \"drawProfile\": %s,\n", drawProfile ? "true" : "false" );
	fprintf( file, "  \"transparent\": %s,\n", transparent ? "true" : "false" );
	fprintf( file, "  \"enableSleep\": %s\n", enableSleep ? "true" : "false" );
	fprintf( file, "  \"enableContinuous\": %s,\n", enableContinuous ? "true" : "false" );
	fprintf( file, "}\n" );
	fclose( file );
}

#define MAX_TOKENS 32

static int jsoneq( const char* json, jsmntok_t* tok, const char* s )
{
	if ( tok->type == JSMN_STRING && (int)strlen( s ) == tok->end - tok->start &&
		 strncmp( json + tok->start, s, tok->end - tok->start ) == 0 )
	{
		return 0;
	}
	return -1;
}

void SampleContext::Load()
{
	debugDraw = b3DefaultDebugDraw();
	debugDraw.DrawShapeFcn = DrawShapeCallback;
	debugDraw.DrawSegmentFcn = DrawLineCallback;
	debugDraw.DrawPointFcn = DrawPointCallback;
	debugDraw.DrawBoundsFcn = DrawBoundsCallback;
	debugDraw.DrawBoxFcn = DrawBoxCallback;
	debugDraw.DrawTransformFcn = DrawTransformCallback;
	debugDraw.DrawStringFcn = DrawSceneTextCallback;
	debugDraw.context = this;

	recycleDistance = B3_CONTACT_RECYCLE_DISTANCE;

	int size = 0;
	char* data = ReadFile( size, settingsFileName );
	if ( data == nullptr )
	{
		return;
	}

	jsmn_parser parser;
	jsmntok_t tokens[MAX_TOKENS];

	jsmn_init( &parser );

	int tokenCount = jsmn_parse( &parser, data, size, tokens, MAX_TOKENS );
	char buffer[32];

	for ( int i = 0; i < tokenCount; ++i )
	{
		if ( jsoneq( data, &tokens[i], "sampleIndex" ) == 0 )
		{
			int count = tokens[i + 1].end - tokens[i + 1].start;
			assert( count < 32 );
			const char* s = data + tokens[i + 1].start;
			strncpy( buffer, s, count );
			buffer[count] = 0;
			char* dummy;
			sampleIndex = (int)strtol( buffer, &dummy, 10 );

			if ( sampleIndex < 0 )
			{
				sampleIndex = 0;
			}
			else if ( sampleIndex >= SampleManager::sEntryCount )
			{
				sampleIndex = SampleManager::sEntryCount - 1;
			}
		}
		else if ( jsoneq( data, &tokens[i], "drawShapes" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			if ( strncmp( s, "true", 4 ) == 0 )
			{
				debugDraw.drawShapes = true;
			}
			else if ( strncmp( s, "false", 5 ) == 0 )
			{
				debugDraw.drawShapes = false;
			}
		}
	}

	free( data );
}

Sample::Sample( SampleContext* context )
{
	m_context = context;
	m_window = context->window;
	m_scene = m_context->scene;
	m_camera = &m_context->camera;

	m_worldId = b3_nullWorldId;

	m_mouseBodyId = {};
	m_mouseJointId = {};
	m_mouseFraction = 0.0f;
	m_mouseForceScale = 100.0f;

	m_textLine = 0;
	m_textIncrement = 22;
	m_stepCount = 0;
	m_userMaterialId = 0;

	memset( m_profiles, 0, sizeof( m_profiles ) );
	m_currentProfileIndex = 0;
	m_profileReadIndex = 0;
	m_profileWriteIndex = 0;

	m_stepWhilePaused = true;
	m_didStep = false;

	m_haveMouseLast = false;
	m_mouseLast = { 0.0f, 0.0f };
	m_mouseDelta = { 0.0f, 0.0f };
	m_launchSpeedScale = 5.0f;

	g_randomSeed = RAND_SEED;

	b3Capacity capacity = {};
	CreateWorld( &capacity );
}

Sample::~Sample()
{
	b3DestroyWorld( m_worldId );

	DestroySharedMeshes( m_context->scene );
}

void Sample::CreateWorld( b3Capacity* capacity )
{
	if ( B3_IS_NON_NULL( m_worldId ) )
	{
		b3DestroyWorld( m_worldId );
	}

	m_mouseBodyId = {};
	m_mouseJointId = {};
	m_mousePoint = {};

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = m_context->workerCount;
	worldDef.enableSleep = m_context->enableSleep;
	worldDef.createDebugShape = CreateDebugShape;
	worldDef.destroyDebugShape = DestroyDebugShape;
	worldDef.userDebugShapeContext = m_context;
	if (capacity != nullptr)
	{
		worldDef.capacity = *capacity;
	}
	m_worldId = b3CreateWorld( &worldDef );

	b3World_SetContactRecycleDistance( m_worldId, m_context->recycleDistance );
}

void Sample::Step()
{
	m_didStep = false;

	float timeStep = 0.0f;
	if ( m_context->pause == false || m_context->singleStep > 0 )
	{
		timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
		m_context->singleStep = b3MaxInt( 0, m_context->singleStep - 1 );
	}

	if ( B3_IS_NON_NULL( m_mouseJointId ) && b3Joint_IsValid( m_mouseJointId ) == false )
	{
		// The world or attached body was destroyed.
		m_mouseJointId = {};

		if ( B3_IS_NON_NULL( m_mouseBodyId ) )
		{
			b3DestroyBody( m_mouseBodyId );
			m_mouseBodyId = {};
		}
	}

	if ( B3_IS_NON_NULL( m_mouseBodyId ) && timeStep > 0.0f )
	{
		b3Body_SetTargetTransform( m_mouseBodyId, { m_mousePoint, b3Quat_identity }, timeStep, true );
	}

	b3World_EnableSleeping( m_worldId, m_context->enableSleep );
	b3World_EnableWarmStarting( m_worldId, m_context->enableWarmStarting );
	b3World_EnableContinuous( m_worldId, m_context->enableContinuous );

	if ( timeStep > 0.0f || m_stepWhilePaused )
	{
		b3World_Step( m_worldId, timeStep, m_context->subStepCount );
	}

	// if (glfwGetKey( m_window, 'F' ))
	//{
	//	b3World_DumpShapeBounds( m_worldId, b3_staticBody );
	// }

	if ( timeStep > 0.0f )
	{
		m_stepCount += 1;
		m_didStep = true;

		if ( m_profileWriteIndex - m_profileReadIndex == m_profileCapacity )
		{
			m_profileReadIndex += 1;
		}

		m_currentProfileIndex = m_profileWriteIndex & ( m_profileCapacity - 1 );
		m_profiles[m_currentProfileIndex] = b3World_GetProfile( m_worldId );

		m_profileWriteIndex += 1;
	}

	m_triangleIndex = -1;
	m_userMaterialId = 0;

	{
		double screenX, screenY;
		glfwGetCursorPos( m_window, &screenX, &screenY );
		PickRay pickRay = m_camera->BuildPickRay( (float)screenX, (float)screenY );

		b3RayResult result = b3World_CastRayClosest( m_worldId, pickRay.origin, pickRay.translation, b3DefaultQueryFilter() );

		if ( result.hit )
		{
			b3ShapeType type = b3Shape_GetType( result.shapeId );
			if ( type == b3_meshShape )
			{
				m_triangleIndex = result.triangleIndex;
			}

			m_userMaterialId = result.userMaterialId;
		}
	}
}

void Sample::Render()
{
	// Draw world
	b3Vec3 position = m_context->camera.GetPosition();
	b3Vec3 r = { 1000.0f, 1000.0f, 1000.0f };
	b3AABB bounds = { position - r, position + r };
	m_context->debugDraw.drawingBounds = bounds;

	b3World_Draw( m_worldId, &m_context->debugDraw, B3_DEFAULT_MASK_BITS );

	DrawTextLine( "camera radius = %g", m_camera->m_radius );
}

void Sample::ResetProfile()
{
	m_stepCount = 0;
}

void Sample::UpdateUI()
{
	float fontSize = ImGui::GetFontSize();

	if ( m_context->drawProfile )
	{
		ImGui::SetNextWindowPos( { fontSize, 8.0f * fontSize }, ImGuiCond_FirstUseEver );
		ImGui::Begin( "Profile (ms)", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize );

		const int count = static_cast<int>( m_profileWriteIndex - m_profileReadIndex );

		// Unroll ring buffer into per-field histories.
		constexpr int kRowCount = 22;
		float histories[kRowCount][m_profileCapacity];
		float totals[kRowCount] = {};
		for ( int i = 0; i < count; ++i )
		{
			int idx = static_cast<int>( ( m_profileReadIndex + i ) & ( m_profileCapacity - 1 ) );
			const b3Profile& p = m_profiles[idx];
			histories[0][i] = p.step;
			histories[1][i] = p.pairs;
			histories[2][i] = p.collide;
			histories[3][i] = p.solve;
			histories[4][i] = p.solverSetup;
			histories[5][i] = p.constraints;
			histories[6][i] = p.prepareConstraints;
			histories[7][i] = p.integrateVelocities;
			histories[8][i] = p.warmStart;
			histories[9][i] = p.solveImpulses;
			histories[10][i] = p.integratePositions;
			histories[11][i] = p.relaxImpulses;
			histories[12][i] = p.applyRestitution;
			histories[13][i] = p.storeImpulses;
			histories[14][i] = p.splitIslands;
			histories[15][i] = p.transforms;
			histories[16][i] = p.jointEvents;
			histories[17][i] = p.hitEvents;
			histories[18][i] = p.refit;
			histories[19][i] = p.sleepIslands;
			histories[20][i] = p.bullets;
			histories[21][i] = p.sensors;

			totals[0] += p.step;
			totals[1] += p.pairs;
			totals[2] += p.collide;
			totals[3] += p.solve;
			totals[4] += p.solverSetup;
			totals[5] += p.constraints;
			totals[6] += p.prepareConstraints;
			totals[7] += p.integrateVelocities;
			totals[8] += p.warmStart;
			totals[9] += p.solveImpulses;
			totals[10] += p.integratePositions;
			totals[11] += p.relaxImpulses;
			totals[12] += p.applyRestitution;
			totals[13] += p.storeImpulses;
			totals[14] += p.splitIslands;
			totals[15] += p.transforms;
			totals[16] += p.jointEvents;
			totals[17] += p.hitEvents;
			totals[18] += p.refit;
			totals[19] += p.sleepIslands;
			totals[20] += p.bullets;
			totals[21] += p.sensors;
		}

		const b3Profile& cur = m_profiles[m_currentProfileIndex];
		const float now[kRowCount] = {
			cur.step,
			cur.pairs,
			cur.collide,
			cur.solve,
			cur.solverSetup,
			cur.constraints,
			cur.prepareConstraints,
			cur.integrateVelocities,
			cur.warmStart,
			cur.solveImpulses,
			cur.integratePositions,
			cur.relaxImpulses,
			cur.applyRestitution,
			cur.storeImpulses,
			cur.splitIslands,
			cur.transforms,
			cur.jointEvents,
			cur.hitEvents,
			cur.refit,
			cur.sleepIslands,
			cur.bullets,
			cur.sensors,
		};

		// Rolling average
		float avg[kRowCount] = {};
		if ( count > 0 )
		{
			float scale = 1.0f / count;
			for ( int i = 0; i < kRowCount; ++i )
			{
				avg[i] = scale * totals[i];
			}
		}

		// Match Frame Time chart's first three colors so rows read with the line plot.
		const ImU32 colorStep = IM_COL32( 102, 153, 255, 255 );
		const ImU32 colorCollide = IM_COL32( 255, 140, 51, 255 );
		const ImU32 colorSolve = IM_COL32( 102, 204, 102, 255 );
		const ImU32 colorDefault = IM_COL32( 220, 220, 220, 255 );

		struct RowDef
		{
			const char* name;
			int indent;
			ImU32 color;
		};
		const RowDef rows[kRowCount] = {
			{ "step", 0, colorStep },			{ "pairs", 0, colorDefault },		 { "collide", 0, colorCollide },
			{ "solve", 0, colorSolve },			{ "setup", 1, colorDefault },		 { "constraints", 1, colorDefault },
			{ "prepare", 2, colorDefault },		{ "velocities", 2, colorDefault },	 { "warm start", 2, colorDefault },
			{ "bias", 2, colorDefault },		{ "positions", 2, colorDefault },	 { "relax", 2, colorDefault },
			{ "restitution", 2, colorDefault }, { "store", 2, colorDefault },		 { "split islands", 2, colorDefault },
			{ "transforms", 1, colorDefault },	{ "joint events", 1, colorDefault }, { "hit events", 1, colorDefault },
			{ "refit BVH", 1, colorDefault },	{ "sleep", 1, colorDefault },		 { "bullets", 1, colorDefault },
			{ "sensors", 0, colorDefault },
		};

		// Derive parent/child links from the indent levels so we can collapse subtrees.
		int parents[kRowCount];
		bool hasChildren[kRowCount] = {};
		{
			int stack[8];
			int stackSize = 0;
			for ( int i = 0; i < kRowCount; ++i )
			{
				while ( stackSize > 0 && rows[stack[stackSize - 1]].indent >= rows[i].indent )
				{
					--stackSize;
				}
				parents[i] = stackSize > 0 ? stack[stackSize - 1] : -1;
				stack[stackSize++] = i;
				if ( parents[i] >= 0 )
				{
					hasChildren[parents[i]] = true;
				}
			}
		}

		static bool s_rowOpen[kRowCount];
		static bool s_showPlots = false;

		// Bars are drawn relative to the step row so the proportions are visually consistent.
		const float stepNow = b3MaxFloat( cur.step, 0.001f );

		if ( ImGui::Button( "Reset" ) )
		{
			ResetProfile();
		}
		ImGui::SameLine();
		ImGui::Checkbox( "Show plots", &s_showPlots );

		const ImGuiTableFlags tableFlags = ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;

		const int colCount = s_showPlots ? 6 : 5;
		if ( ImGui::BeginTable( "profile", colCount, tableFlags ) )
		{
			ImGui::TableSetupColumn( "section", ImGuiTableColumnFlags_WidthFixed, 8.0f * fontSize );
			ImGui::TableSetupColumn( "now", ImGuiTableColumnFlags_WidthFixed, 3.0f * fontSize );
			ImGui::TableSetupColumn( "avg", ImGuiTableColumnFlags_WidthFixed, 3.0f * fontSize );
			ImGui::TableSetupColumn( "max", ImGuiTableColumnFlags_WidthFixed, 3.0f * fontSize );
			ImGui::TableSetupColumn( "% step", ImGuiTableColumnFlags_WidthFixed, 8.0f * fontSize );
			if ( s_showPlots )
			{
				ImGui::TableSetupColumn( "history", ImGuiTableColumnFlags_WidthFixed, 16.0f * fontSize );
			}
			ImGui::TableHeadersRow();

			const float rowHeight = 1.5f * fontSize;

			for ( int r = 0; r < kRowCount; ++r )
			{
				bool visible = true;
				for ( int p = parents[r]; p >= 0; p = parents[p] )
				{
					if ( !s_rowOpen[p] )
					{
						visible = false;
						break;
					}
				}
				if ( !visible )
				{
					continue;
				}

				const RowDef& d = rows[r];
				const float* hist = histories[r];

				float rollingMax = 0.0f;
				for ( int i = 0; i < count; ++i )
				{
					rollingMax = b3MaxFloat( rollingMax, hist[i] );
				}

				ImGui::TableNextRow();

				ImGui::TableNextColumn();
				if ( d.indent > 0 )
				{
					ImGui::Indent( d.indent * fontSize );
				}
				if ( hasChildren[r] )
				{
					ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick |
											   ImGuiTreeNodeFlags_NoTreePushOnOpen;
					ImGui::PushStyleColor( ImGuiCol_Text, d.color );
					s_rowOpen[r] = ImGui::TreeNodeEx( d.name, flags );
					ImGui::PopStyleColor();
				}
				else
				{
					float leafIndent = ImGui::GetTreeNodeToLabelSpacing();
					ImGui::Indent( leafIndent );
					ImGui::PushStyleColor( ImGuiCol_Text, d.color );
					ImGui::TextUnformatted( d.name );
					ImGui::PopStyleColor();
					ImGui::Unindent( leafIndent );
				}
				if ( d.indent > 0 )
				{
					ImGui::Unindent( d.indent * fontSize );
				}

				ImGui::TableNextColumn();
				ImGui::Text( "%6.2f", now[r] );
				ImGui::TableNextColumn();
				ImGui::Text( "%6.2f", avg[r] );
				ImGui::TableNextColumn();
				ImGui::Text( "%6.2f", rollingMax );

				ImGui::TableNextColumn();
				float frac = b3ClampFloat( now[r] / stepNow, 0.0f, 1.0f );
				ImGui::PushStyleColor( ImGuiCol_PlotHistogram, d.color );
				ImGui::ProgressBar( frac, ImVec2( -FLT_MIN, 0.0f ), "" );
				ImGui::PopStyleColor();

				if ( s_showPlots )
				{
					ImGui::TableNextColumn();
					if ( count > 1 )
					{
						char id[16];
						snprintf( id, sizeof( id ), "##h%d", r );
						ImGui::PushStyleColor( ImGuiCol_PlotLines, d.color );
						ImGui::PlotLines( id, hist, count, 0, nullptr, 0.0f, rollingMax * 1.05f + 0.001f,
										  ImVec2( -FLT_MIN, rowHeight ) );
						ImGui::PopStyleColor();
					}
				}
			}
			ImGui::EndTable();
		}

		ImGui::End();
	}

	if ( m_context->drawCounters )
	{
		b3Counters s = b3World_GetCounters( m_worldId );
		constexpr int colorCount = sizeof( s.colorCounts ) / sizeof( s.colorCounts[0] );
		const int overflowIndex = colorCount - 1;
		constexpr int manifoldBucketCount = sizeof( s.manifoldCounts ) / sizeof( s.manifoldCounts[0] );

		// Bars are scaled to the largest non-overflow color so the distribution shape reads clearly;
		// overflow gets its own bar against the same scale, with a red tint to flag coupling problems.
		int totalCount = 0;
		int maxCount = 1;
		for ( int i = 0; i < colorCount; ++i )
		{
			totalCount += s.colorCounts[i];
			if ( i != overflowIndex && s.colorCounts[i] > maxCount )
			{
				maxCount = s.colorCounts[i];
			}
		}

		int totalManifolds = 0;
		int maxManifolds = 1;
		for ( int i = 0; i < manifoldBucketCount; ++i )
		{
			totalManifolds += s.manifoldCounts[i];
			if ( s.manifoldCounts[i] > maxManifolds )
			{
				maxManifolds = s.manifoldCounts[i];
			}
		}

		ImGui::SetNextWindowPos( { fontSize, 8.0f * fontSize }, ImGuiCond_FirstUseEver );
		ImGui::Begin( "Counters", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize );

		ImGui::Text( "bodies/shapes/contacts/joints = %d/%d/%d/%d", s.bodyCount, s.shapeCount, s.contactCount, s.jointCount );
		{
			float frac = s.awakeContactCount > 0
							 ? b3ClampFloat( (float)s.recycledContactCount / (float)s.awakeContactCount, 0.0f, 1.0f )
							 : 0.0f;

			char overlay[32];
			snprintf( overlay, sizeof( overlay ), "%d / %d", s.recycledContactCount, s.awakeContactCount );

			ImGui::TextUnformatted( "recycled contacts" );
			ImGui::SameLine();
			ImGui::ProgressBar( frac, ImVec2( -FLT_MIN, 0.0f ), overlay );
		}
		ImGui::Text( "islands/tasks = %d/%d", s.islandCount, s.taskCount );
		ImGui::Text( "tree height static/movable = %d/%d", s.staticTreeHeight, s.treeHeight );
		ImGui::Text( "sat call/hit = %d/%d", s.satCallCount, s.satCacheHitCount );
		ImGui::Text( "stack allocator size = %d K", s.stackUsed / 1024 );
		ImGui::Text( "arena capacity = %d K", s.arenaCapacity / 1024 );
		ImGui::Text( "total allocation = %d K", s.byteCount / 1024 );

		ImGui::Separator();
		b3Capacity c = b3World_GetMaxCapacity( m_worldId );
		ImGui::Text( "max capacities" );
		ImGui::BulletText( "static shapes/bodies = %d/%d", c.staticShapeCount, c.staticBodyCount );
		ImGui::BulletText( "dynamic shapes/bodies = %d/%d", c.dynamicShapeCount, c.dynamicBodyCount );
		ImGui::BulletText( "contacts = %d", c.contactCount );

		ImGui::Separator();
		ImGui::Text( "%d constraints across %d colors", totalCount, colorCount );

		const ImGuiTableFlags tableFlags = ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;
		if ( ImGui::BeginTable( "graphColors", 3, tableFlags ) )
		{
			ImGui::TableSetupColumn( "color", ImGuiTableColumnFlags_WidthFixed, 3.5f * fontSize );
			ImGui::TableSetupColumn( "count", ImGuiTableColumnFlags_WidthFixed, 5.0f * fontSize );
			ImGui::TableSetupColumn( "share", ImGuiTableColumnFlags_WidthFixed, 16.0f * fontSize );
			ImGui::TableHeadersRow();

			const float invMax = 1.0f / static_cast<float>( maxCount );

			for ( int i = 0; i < colorCount; ++i )
			{
				int count = s.colorCounts[i];
				bool isOverflow = ( i == overflowIndex );

				// Skip empty slots, but always show overflow — a non-zero overflow row is the signal we care about.
				if ( count == 0 && !isOverflow )
				{
					continue;
				}

				uint32_t hex = static_cast<uint32_t>( b3GetGraphColor( i ) );
				ImU32 swatch = IM_COL32( ( hex >> 16 ) & 0xFF, ( hex >> 8 ) & 0xFF, hex & 0xFF, 255 );
				ImU32 barColor = isOverflow ? IM_COL32( 220, 60, 60, 255 ) : swatch;

				ImGui::TableNextRow();

				ImGui::TableNextColumn();
				if ( isOverflow )
				{
					ImGui::PushStyleColor( ImGuiCol_Text, IM_COL32( 220, 60, 60, 255 ) );
					ImGui::TextUnformatted( "over" );
					ImGui::PopStyleColor();
				}
				else
				{
					ImGui::PushStyleColor( ImGuiCol_Text, swatch );
					ImGui::Text( "%d", i );
					ImGui::PopStyleColor();
				}

				ImGui::TableNextColumn();
				ImGui::Text( "%d", count );

				ImGui::TableNextColumn();
				float frac = b3ClampFloat( count * invMax, 0.0f, 1.0f );
				ImGui::PushStyleColor( ImGuiCol_PlotHistogram, barColor );
				ImGui::ProgressBar( frac, ImVec2( -FLT_MIN, 0.0f ), "" );
				ImGui::PopStyleColor();
			}
			ImGui::EndTable();
		}

		ImGui::Separator();
		ImGui::Text( "%d manifolds across %d buckets", totalManifolds, manifoldBucketCount );
		if ( ImGui::BeginTable( "manifolds", 3, tableFlags ) )
		{
			ImGui::TableSetupColumn( "points", ImGuiTableColumnFlags_WidthFixed, 3.5f * fontSize );
			ImGui::TableSetupColumn( "count", ImGuiTableColumnFlags_WidthFixed, 5.0f * fontSize );
			ImGui::TableSetupColumn( "share", ImGuiTableColumnFlags_WidthFixed, 16.0f * fontSize );
			ImGui::TableHeadersRow();

			const float invMax = 1.0f / static_cast<float>( maxManifolds );

			for ( int i = 0; i < manifoldBucketCount; ++i )
			{
				int count = s.manifoldCounts[i];
				if ( count == 0 )
				{
					continue;
				}

				ImGui::TableNextRow();

				ImGui::TableNextColumn();
				ImGui::Text( "%d", i + 1 );

				ImGui::TableNextColumn();
				ImGui::Text( "%d", count );

				ImGui::TableNextColumn();
				float frac = b3ClampFloat( count * invMax, 0.0f, 1.0f );
				ImGui::ProgressBar( frac, ImVec2( -FLT_MIN, 0.0f ), "" );
			}
			ImGui::EndTable();
		}

		ImGui::End();
	}

	if ( m_context->frameTime )
	{
		float frameTimeHeight = 400.0f;
		float frameTimeWidth = 800.0f;

		ImGui::SetNextWindowPos( { 30.0f, 30.0f }, ImGuiCond_FirstUseEver );
		ImGui::SetNextWindowSize( { frameTimeWidth, frameTimeHeight }, ImGuiCond_FirstUseEver );

		ImGui::Begin( "Frame Time", nullptr, ImGuiWindowFlags_NoCollapse );

		ImGui::PushItemWidth( ImGui::GetWindowWidth() - 20.0f );

		float maxValue = 0.0f;
		float times[m_profileCapacity];
		float stepTimes[m_profileCapacity];
		float collideTimes[m_profileCapacity];
		float solveTimes[m_profileCapacity];
		int count = m_profileWriteIndex - m_profileReadIndex;
		for ( int i = 0; i < count; ++i )
		{
			int index = ( m_profileReadIndex + i ) & ( m_profileCapacity - 1 );
			times[i] = i / 60.0f;
			stepTimes[i] = m_profiles[index].step;
			collideTimes[i] = m_profiles[index].collide;
			solveTimes[i] = m_profiles[index].solve;
			maxValue = b3MaxFloat( stepTimes[i], maxValue );
		}

		// This is the pixel size, not the range.
		ImVec2 plotSize = { -1, 22.0f * ImGui::GetTextLineHeight() };
		if ( ImPlot::BeginPlot( "Profile", plotSize, ImPlotFlags_NoTitle ) )
		{
			ImPlot::SetupAxes( "t", "ms" );
			ImPlot::SetupAxes( "t", "ms", 0, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit );
			ImPlot::SetupAxesLimits( 0, m_profileCapacity / 60.0, 0.0, maxValue );
			ImPlot::PlotLine( "step", times, stepTimes, count );
			ImPlot::PlotLine( "collide", times, collideTimes, count );
			ImPlot::PlotLine( "solve", times, solveTimes, count );
			ImPlot::EndPlot();
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}
}

void Sample::MouseDown( b3Vec2 p, int button, int modifiers )
{
	if ( modifiers == 0 )
	{
		PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );

		b3RayResult result = b3World_CastRayClosest( m_worldId, pickRay.origin, pickRay.translation, b3DefaultQueryFilter() );

		if ( result.hit )
		{
			m_mousePoint = result.point;

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_kinematicBody;
			bodyDef.position = m_mousePoint;
			bodyDef.enableSleep = false;
			m_mouseBodyId = b3CreateBody( m_worldId, &bodyDef );

			b3BodyId bodyId = b3Shape_GetBody( result.shapeId );

			b3MotorJointDef jointDef = b3DefaultMotorJointDef();
			jointDef.base.bodyIdA = m_mouseBodyId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( bodyId, result.point );
			jointDef.linearHertz = 7.5f;
			jointDef.linearDampingRatio = 1.0f;

			b3MassData massData = b3Body_GetMassData( bodyId );
			float g = b3Length( b3World_GetGravity( m_worldId ) );
			float mg = massData.mass * g;
			jointDef.maxSpringForce = m_mouseForceScale * mg;

			if ( massData.mass > 0.0f )
			{
				// This acts like angular friction
				float trace = massData.inertia.cx.x + massData.inertia.cy.y + massData.inertia.cz.z;
				float lever = sqrtf( trace / ( 3.0f * massData.mass ) );
				jointDef.maxVelocityTorque = 0.5f * lever * mg;
			}

			m_mouseJointId = b3CreateMotorJoint( m_worldId, &jointDef );

			b3Body_SetAwake( bodyId, true );

			m_mouseFraction = result.fraction;
		}
	}
	else if ( modifiers & GLFW_MOD_SHIFT )
	{
		PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
		b3Vec3 direction = b3Normalize( pickRay.translation );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		if ( modifiers & GLFW_MOD_CONTROL )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = pickRay.origin + 2.0f * direction;
			bodyDef.linearVelocity = ( 10.0f * m_launchSpeedScale ) * direction;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3Hull* hull = b3CreateCylinder( 2.0f, 0.15f, 0.0f, 6 );
			b3CreateHullShape( bodyId, &shapeDef, hull );
			b3DestroyHull( hull );
		}
		else if ( modifiers & GLFW_MOD_ALT )
		{
			b3Vec3 position = pickRay.origin + 2.0f * direction;
			Human human = {};
			CreateHuman( &human, m_worldId, position, 1.0f, 1.0f, 1.0f, 0, nullptr, true );
			Human_SetVelocity( &human, ( 5.0f * m_launchSpeedScale ) * direction );
		}
		else
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = pickRay.origin + 2.0f * direction;
			bodyDef.linearVelocity = ( 20.0f * m_launchSpeedScale ) * direction;
			bodyDef.isBullet = true;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3Sphere sphere = { b3Vec3_zero, 0.25f };
			shapeDef.density *= 4.0f;
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		}
	}
}

void Sample::MouseUp( b3Vec2 p, int button )
{
	if ( b3Joint_IsValid( m_mouseJointId ) )
	{
		b3DestroyJoint( m_mouseJointId, true );
	}

	if ( b3Body_IsValid( m_mouseBodyId ) )
	{
		b3DestroyBody( m_mouseBodyId );
	}

	m_mouseJointId = b3_nullJointId;
	m_mouseBodyId = b3_nullBodyId;
	m_mouseFraction = 0.0f;
}

void Sample::MouseMove( b3Vec2 p )
{
	if ( m_camera->m_thirdPerson )
	{
		if ( m_haveMouseLast == false )
		{
			m_mouseLast = p;
			m_haveMouseLast = true;
		}
		else
		{
			m_mouseDelta = { p.x - m_mouseLast.x, p.y - m_mouseLast.y };
			// printf( "mdx = %g, mdy = %g\n", m_mouseDelta.x, m_mouseDelta.y );
			m_mouseLast = p;
		}

		const float sensitivity = 0.1f;
		m_camera->m_yaw -= 2.0f * sensitivity * m_mouseDelta.x;
		m_camera->m_pitch += sensitivity * m_mouseDelta.y;
		m_camera->m_pitch = b3ClampFloat( m_camera->m_pitch, -85.0f, 85.0f );
	}

	PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
	if ( B3_IS_NON_NULL( m_mouseJointId ) )
	{
		m_mousePoint = pickRay.origin + m_mouseFraction * pickRay.translation;
		// b3Transform localFrameA = { target, b3Quat_identity };
		// b3Joint_SetLocalFrameA( m_mouseJointId, localFrameA );
	}
}

void Sample::DrawTextLine( const char* text, ... )
{
	va_list args;
	va_start( args, text );
	char buffer[512];
	vsnprintf( buffer, sizeof( buffer ), text, args );
	DrawText( 5, m_textLine, b3_colorWhite, buffer );
	va_end( args );
	m_textLine += m_textIncrement;
}

SampleEntry SampleManager::sEntries[MAX_SAMPLES];
int SampleManager::sEntryCount = 0;

int SampleManager::Register( const char* category, const char* name, SampleCreateFcn* fcn )
{
	int index = sEntryCount;
	if ( index < MAX_SAMPLES )
	{
		sEntries[index] = { category, name, fcn };
		sEntryCount += 1;
		return index;
	}

	return -1;
}

static int CompareSamples( const void* a, const void* b )
{
	SampleEntry* entryA = (SampleEntry*)a;
	SampleEntry* entryB = (SampleEntry*)b;

	int result = strcmp( entryA->Category, entryB->Category );
	if ( result == 0 )
	{
		result = strcmp( entryA->Name, entryB->Name );
	}

	return result;
}

void SampleManager::Startup( GLFWwindow* window, int width, int height, int bufferWidth, int bufferHeight )
{
	m_context.Load();

	m_context.workerCount = b3MinInt( 8, GetNumberOfCores() / 2 );

	// todo_erin testing
	// m_context.settings.workerCount = 1;

	m_context.arena.Create( 400 * 1024 * 1024 );
	m_context.window = window;
	m_context.camera.Resize( width, height );

	assert( bufferWidth > 0 && bufferHeight > 0 );
	m_context.camera.m_bufferWidth = bufferWidth;
	m_context.camera.m_bufferHeight = bufferHeight;
	m_context.scene = CreateScene( &m_context.camera, m_context.arena );

	if ( m_context.scene == nullptr )
	{
		fprintf( stderr, "Failed to create scene\n" );
		glfwTerminate();
	}

	qsort( sEntries, sEntryCount, sizeof( SampleEntry ), CompareSamples );

	m_sample = sEntries[m_context.sampleIndex].CreateFcn( &m_context );
}

void SampleManager::Step()
{
	const SampleEntry* entry = sEntries + m_context.sampleIndex;
	DrawFormat( 5, m_sample->m_textIncrement, b3_colorCyan, "%s : %s", entry->Category, entry->Name );
	m_sample->m_textLine = 2 * m_sample->m_textIncrement;

	if ( m_context.pause )
	{
		m_sample->DrawTextLine( "****PAUSED****" );
	}

	m_sample->Step();
}

void SampleManager::Resize( int width, int height, int bufferWidth, int bufferHeight )
{
	m_context.minimized = false;
	if ( width == 0 || height == 0 || bufferWidth == 0 || bufferHeight == 0 )
	{
		m_context.minimized = true;
		return;
	}

	m_context.camera.Resize( width, height );
	m_context.camera.m_bufferWidth = bufferWidth;
	m_context.camera.m_bufferHeight = bufferHeight;
	ResizeScene( m_context.scene, &m_context.camera );
}

void SampleManager::Render( GLFWwindow* window, float elapsedTime )
{
	RenderScene( window, elapsedTime );
	UpdateUI( window );
}

void SampleManager::Shutdown( GLFWwindow* window )
{
	// Must destroy sample first because it will destroy debug shapes.
	delete m_sample;
	m_sample = nullptr;

	m_context.Save();

	DestroyScene( m_context.scene );
	m_context.scene = nullptr;

	m_context.arena.Destroy();
}

void SampleManager::Keyboard( int key, int action, int modifiers )
{
	if ( action == GLFW_PRESS )
	{
		switch ( key )
		{
			case GLFW_KEY_ESCAPE:
				glfwSetWindowShouldClose( m_context.window, GL_TRUE );
				break;

			case GLFW_KEY_TAB:
				m_showMenu = !m_showMenu;
				break;

			case GLFW_KEY_O:
				if ( modifiers & GLFW_MOD_SHIFT )
				{
					m_context.singleStep += 5;
				}
				else
				{
					m_context.singleStep += 1;
				}
				break;

			case GLFW_KEY_P:
				m_context.pause = !m_context.pause;
				break;

			case GLFW_KEY_R:
				m_context.restart = true;
				CreateSample();
				break;

			case GLFW_KEY_LEFT_BRACKET:
				m_context.sampleIndex = b3MaxInt( 0, m_context.sampleIndex - 1 );
				m_context.restart = false;
				CreateSample();
				break;

			case GLFW_KEY_RIGHT_BRACKET:
				m_context.sampleIndex = b3MinInt( sEntryCount - 1, m_context.sampleIndex + 1 );
				m_context.restart = false;
				CreateSample();
				break;

			default:
				m_sample->Keyboard( key, action, modifiers );
				break;
		}
	}
}

void SampleManager::CreateSample()
{
	B3_ASSERT( m_sample );

	if (m_context.restart == false)
	{
		EnableGrid( m_context.scene, false );
	}

	delete m_sample;
	m_sample = sEntries[m_context.sampleIndex].CreateFcn( &m_context );
}

void SampleManager::RenderScene( GLFWwindow* window, float elapsedTime )
{
	// Update camera
	m_context.camera.Update( window, elapsedTime );

	m_sample->Render();
	// CheckOpenGL();

	// Actually render
	DrawScene( m_context.scene, &m_context.camera );
	// CheckOpenGL();
}

void SampleManager::UpdateUI( GLFWwindow* window )
{
	if ( m_showMenu == false )
	{
		return;
	}

	int maxWorkers = B3_MAX_WORKERS;

	int savedTestIndex = m_context.sampleIndex;
	float menuWidth = 220.0f;
	b3WorldId worldId = m_sample->m_worldId;

	ImGui::SetNextWindowPos( { m_context.camera.m_width - menuWidth - 10.0f, 10.0f } );
	ImGui::SetNextWindowSize( { menuWidth, m_context.camera.m_height - 20.0f } );

	ImGui::Begin( "Tools", &m_showMenu, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse );

	if ( ImGui::BeginTabBar( "ControlTabs", ImGuiTabBarFlags_None ) )
	{
		if ( ImGui::BeginTabItem( "Controls" ) )
		{
			ImGui::SliderInt( "Sub-steps", &m_context.subStepCount, 1, 50 );
			ImGui::SliderFloat( "Hertz", &m_context.hertz, 5.0f, 2000.0f, "%.0f hz" );

			if ( ImGui::SliderInt( "Workers", &m_context.workerCount, 1, maxWorkers ) )
			{
				m_context.workerCount = b3ClampInt( m_context.workerCount, 1, maxWorkers );
				m_context.restart = true;
				CreateSample();
			}

			ImGui::Separator();

			ImGui::Checkbox( "Sleep", &m_context.enableSleep );
			ImGui::Checkbox( "Warm Starting", &m_context.enableWarmStarting );
			ImGui::Checkbox( "Continuous", &m_context.enableContinuous );

			ImGui::PushItemWidth( 100.0f );
			float recyclingCentimeters = 100.0f * m_context.recycleDistance;
			if ( ImGui::SliderFloat( "Recycle", &recyclingCentimeters, 0.0f, 10.0f, "%.1f cm" ) )
			{
				m_context.recycleDistance = 0.01f * recyclingCentimeters;
				b3World_SetContactRecycleDistance( worldId, m_context.recycleDistance );
			}
			ImGui::PopItemWidth();

			ImGui::Separator();

			ImGui::Checkbox( "Shapes", &m_context.debugDraw.drawShapes );
			ImGui::Checkbox( "Transparent", &m_context.transparentDynamic );
			ImGui::Checkbox( "Joints", &m_context.debugDraw.drawJoints );
			ImGui::Checkbox( "Joint Extras", &m_context.debugDraw.drawJointExtras );
			ImGui::Checkbox( "Bounds", &m_context.debugDraw.drawBounds );

			ImGui::Separator();

			ImGui::Checkbox( "Contact Points", &m_context.debugDraw.drawContacts );
			ImGui::RadioButton( "Anchor A", &m_context.debugDraw.drawAnchorA, 1 );
			ImGui::SameLine();
			ImGui::RadioButton( "Anchor B", &m_context.debugDraw.drawAnchorA, 0 );
			ImGui::Checkbox( "Contact Normals", &m_context.debugDraw.drawContactNormals );
			ImGui::Checkbox( "Contact Forces", &m_context.debugDraw.drawContactForces );
			ImGui::Checkbox( "Contact Features", &m_context.debugDraw.drawContactFeatures );
			ImGui::Checkbox( "Friction Forces", &m_context.debugDraw.drawFrictionForces );

			ImGui::Separator();

			ImGui::Checkbox( "Mass", &m_context.debugDraw.drawMass );
			ImGui::Checkbox( "Body Names", &m_context.debugDraw.drawBodyNames );
			ImGui::Checkbox( "Graph Colors", &m_context.debugDraw.drawGraphColors );
			ImGui::Checkbox( "Draw Islands", &m_context.debugDraw.drawIslands );
			ImGui::Checkbox( "Counters", &m_context.drawCounters );
			ImGui::Checkbox( "Profile", &m_context.drawProfile );
			ImGui::Checkbox( "Frame Time", &m_context.frameTime );

			ImGui::PushItemWidth( 80.0f );
			ImGui::InputFloat( "Joint Scale", &m_context.debugDraw.jointScale );
			ImGui::InputFloat( "Force Scale", &m_context.debugDraw.forceScale );
			ImGui::PopItemWidth();

			ImGui::Separator();

			bool useSSAO = IsSSAOEnabled( m_context.scene );
			if ( ImGui::Checkbox( "SSAO", &useSSAO ) )
			{
				EnableSSAO( m_context.scene, useSSAO );
			}

			bool useShadows = AreShadowsEnabled( m_context.scene );
			if ( ImGui::Checkbox( "Shadows", &useShadows ) )
			{
				EnableShadows( m_context.scene, useShadows );
			}

			SceneAOSettings* ao = GetAOSettings( m_context.scene );
			ImGui::SliderFloat( "AO Radius", &ao->radius, 0.0f, 5.0f );
			ImGui::SliderFloat( "AO Bias", &ao->bias, 0.0f, 0.5f );
			ImGui::SliderFloat( "AO Min", &ao->minScale, 0.0f, 1.0f );
			ImGui::SliderFloat( "AO Power", &ao->power, 0.5f, 8.0f );
			ImGui::SliderFloat( "AO Direct", &ao->direct, 0.0f, 1.0f );
			ImGui::Checkbox( "AO Only", &ao->aoOnly );

			ImGui::Separator();

			if ( ImGui::Button( "Pause", ImVec2( -1, 0 ) ) )
			{
				m_context.pause = !m_context.pause;
			}

			if ( ImGui::Button( "Restart", ImVec2( -1, 0 ) ) )
			{
				m_context.restart = true;
				CreateSample();
			}

			if ( ImGui::Button( "Dump", ImVec2( -1, 0 ) ) )
			{
				b3World_Dump( m_sample->m_worldId );
			}

			if ( ImGui::Button( "Dump Awake", ImVec2( -1, 0 ) ) )
			{
				b3World_DumpAwake( m_sample->m_worldId );
			}

			if ( ImGui::Button( "Quit", ImVec2( -1, 0 ) ) )
			{
				glfwSetWindowShouldClose( window, GL_TRUE );
			}

			ImGui::EndTabItem();
		}

		ImGuiTreeNodeFlags leafNodeFlags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;
		leafNodeFlags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;

		ImGuiTreeNodeFlags nodeFlags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;

		if ( ImGui::BeginTabItem( "Tests" ) )
		{
			const char* category = sEntries[0].Category;
			int i = 0;
			while ( i < sEntryCount )
			{
				bool categorySelected = strcmp( category, sEntries[m_context.sampleIndex].Category ) == 0;
				ImGuiTreeNodeFlags nodeSelectionFlags = categorySelected ? ImGuiTreeNodeFlags_Selected : 0;
				bool nodeOpen = ImGui::TreeNodeEx( category, nodeFlags | nodeSelectionFlags );

				if ( nodeOpen )
				{
					while ( i < sEntryCount && strcmp( category, sEntries[i].Category ) == 0 )
					{
						ImGuiTreeNodeFlags selectionFlags = 0;
						if ( m_context.sampleIndex == i )
						{
							selectionFlags = ImGuiTreeNodeFlags_Selected;
						}
						ImGui::TreeNodeEx( (void*)(intptr_t)i, leafNodeFlags | selectionFlags, "%s", sEntries[i].Name );
						if ( ImGui::IsItemClicked() )
						{
							m_context.sampleIndex = i;
						}
						++i;
					}
					ImGui::TreePop();
				}
				else
				{
					while ( i < sEntryCount && strcmp( category, sEntries[i].Category ) == 0 )
					{
						++i;
					}
				}

				if ( i < sEntryCount )
				{
					category = sEntries[i].Category;
				}
			}
			ImGui::EndTabItem();
		}
		ImGui::EndTabBar();
	}

	ImGui::End();

	m_sample->UpdateUI();

	// todo wrong place for this
	if ( m_context.sampleIndex != savedTestIndex )
	{
		m_context.restart = false;
		CreateSample();
	}
}

void Sample::ToggleThirdPerson()
{
	if ( m_camera->m_thirdPerson )
	{
		glfwSetInputMode( m_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL );
		m_camera->m_thirdPerson = false;
	}
	else
	{
		glfwSetInputMode( m_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED );
		m_camera->m_thirdPerson = true;
	}
}

float CastClosestCallback( b3ShapeId shapeId, b3Vec3 point, b3Vec3 normal, float fraction, uint64_t materialId, int triangleIndex,
						   int childIndex, void* context )
{
	CastClosestContext* rayContext = (CastClosestContext*)context;
	rayContext->shapeId = shapeId;
	rayContext->point = point;
	rayContext->normal = normal;
	rayContext->fraction = fraction;
	rayContext->materialId = materialId;
	rayContext->childIndex = childIndex;
	rayContext->triangleIndex = triangleIndex;
	rayContext->hit = true;
	return fraction;
}

static bool MoverFilterCallback( b3ShapeId shapeId, void* context )
{
	CharacterMover* self = (CharacterMover*)context;
	for ( int i = 0; i < self->m_ignoreCount; ++i )
	{
		if ( B3_ID_EQUALS( shapeId, self->m_ignoreShapeIds[i] ) )
		{
			return false;
		}
	}

	return true;
}

void CharacterMover::Initialize( Sample* sample, b3Vec3 position )
{
	m_sample = sample;
	m_transform.p = position;
	m_transform.q = b3Quat_identity;
	m_velocity = { 0.0f, 0.0f, 0.0f };
	m_capsule = { { 0.0f, -0.5f, 0.0f }, { 0.0f, 0.5f, 0.0f }, 0.3f };

	m_planeCount = 0;
	m_totalIterations = 0;
	m_pogoVelocity = 0.0f;
	m_onGround = false;
	m_sprint = false;

	m_ignoreShapeIds = nullptr;
	m_ignoreCount = 0;
}

static bool PlaneResultFcn( b3ShapeId shapeId, const b3PlaneResult* planeResults, int planeCount, void* context )
{
	if ( MoverFilterCallback( shapeId, context ) == false )
	{
		// ignore these planes but continue looking for more
		return true;
	}

	CharacterMover* self = static_cast<CharacterMover*>( context );
	float maxPush = FLT_MAX;
	bool clipVelocity = true;
	MoverShapeUserData* userData = static_cast<MoverShapeUserData*>( (void*)b3Shape_GetUserData( shapeId ) );
	if ( userData != nullptr )
	{
		maxPush = userData->maxPush;
		clipVelocity = userData->clipVelocity;
	}

	for ( int i = 0; i < planeCount && self->m_planeCount < CharacterMover::m_planeCapacity; ++i )
	{
		assert( b3IsValidPlane( planeResults[i].plane ) );
		self->m_planes[self->m_planeCount] = {
			.plane = planeResults[i].plane,
			.pushLimit = maxPush,
			.push = 0.0f,
			.clipVelocity = clipVelocity,
		};
		self->m_planeExtras[self->m_planeCount] = {
			.point = planeResults[i].point,
			.shapeId = shapeId,
		};
		self->m_planeCount += 1;
	}

	return true;
}

void CharacterMover::SolveMove( float timeStep, b3Vec3 forward, b3Vec3 right, b3Vec2 throttle, bool clipVelocity )
{
	// Friction
	float speed = b3Length( m_velocity );
	if ( speed < m_minSpeed )
	{
		m_velocity.x = 0.0f;
		m_velocity.y = 0.0f;
	}
	else
	{
		// Linear damping above stopSpeed and fixed reduction below stopSpeed
		float control = speed < m_stopSpeed ? m_stopSpeed : speed;

		// friction has units of 1/time
		float drop = control * m_friction * timeStep;
		float newSpeed = b3MaxFloat( 0.0f, speed - drop );
		m_velocity *= newSpeed / speed;
	}

	float maxSpeed = m_sprint ? 1.5f * m_maxSpeed : m_maxSpeed;

	b3Vec3 desiredVelocity = maxSpeed * throttle.x * forward + maxSpeed * throttle.y * right;
	float desiredSpeed;
	b3Vec3 desiredDirection = b3GetLengthAndNormalize( &desiredSpeed, desiredVelocity );

	if ( desiredSpeed > maxSpeed )
	{
		desiredVelocity *= maxSpeed / desiredSpeed;
		desiredSpeed = maxSpeed;
	}

	if ( m_onGround )
	{
		m_velocity.y = 0.0f;
	}

	// Accelerate
	float currentSpeed = b3Dot( m_velocity, desiredDirection );
	float addSpeed = desiredSpeed - currentSpeed;
	if ( addSpeed > 0.0f )
	{
		float accelSpeed = m_accelerate * maxSpeed * timeStep;
		if ( accelSpeed > addSpeed )
		{
			accelSpeed = addSpeed;
		}

		m_velocity += accelSpeed * desiredDirection;
	}

	m_velocity.y -= m_gravity * timeStep;

	b3WorldId worldId = m_sample->m_worldId;
	Scene* scene = m_sample->m_scene;

	float pogoRestLength = 3.0f * m_capsule.radius;
	float rayLength = pogoRestLength + m_capsule.radius;
	b3Vec3 rayOrigin = b3TransformPoint( m_transform, m_capsule.center1 );
	b3Vec3 rayTranslation = -rayLength * b3Vec3_axisY;
	b3QueryFilter skipTeamFilter = { 1, ~2u };
	b3RayResult rayResult = b3World_CastRayClosest( worldId, rayOrigin, rayTranslation, skipTeamFilter );

	if ( rayResult.hit == false )
	{
		m_onGround = false;
		m_pogoVelocity = 0.0f;

		DrawLine( scene, rayOrigin, rayOrigin + rayTranslation, b3_colorGray );
	}
	else
	{
		m_onGround = true;
		float pogoCurrentLength = rayResult.fraction * rayLength;

		float zeta = 0.7f;
		float hertz = 4.0f;
		float omega = 2.0f * B3_PI * hertz;
		float omegaH = omega * timeStep;

		m_pogoVelocity = ( m_pogoVelocity - omega * omegaH * ( pogoCurrentLength - pogoRestLength ) ) /
						 ( 1.0f + 2.0f * zeta * omegaH + omegaH * omegaH );
		DrawLine( scene, rayOrigin, rayResult.point, b3_colorGreen );
	}

	b3Vec3 startPosition = m_transform.p;
	b3Vec3 target = m_transform.p + timeStep * m_velocity + timeStep * m_pogoVelocity * b3Vec3_axisY;

	// Want the mover to collide with allies
	b3QueryFilter moverFilter = { .categoryBits = 1, .maskBits = ~0u };

	// The cast should ignore allies
	b3QueryFilter castFilter = { .categoryBits = 1, .maskBits = ~2u };

	m_totalIterations = 0;
	float tolerance = 0.01f;

	for ( int iteration = 0; iteration < 5; ++iteration )
	{
		m_planeCount = 0;

		b3Capsule mover;
		mover.center1 = b3TransformPoint( m_transform, m_capsule.center1 );
		mover.center2 = b3TransformPoint( m_transform, m_capsule.center2 );
		mover.radius = m_capsule.radius;

		b3World_CollideMover( worldId, &mover, moverFilter, PlaneResultFcn, this );

		b3Vec3 targetDelta = target - m_transform.p;
		b3PlaneSolverResult result = b3SolvePlanes( targetDelta, m_planes, m_planeCount );

		m_totalIterations += result.iterationCount;

		b3Vec3 delta = result.delta;

		float fraction = b3World_CastMover( worldId, &mover, delta, castFilter, MoverFilterCallback, this );

		delta *= fraction;
		m_transform.p += delta;

		if ( b3LengthSquared( delta ) < tolerance * tolerance )
		{
			break;
		}
	}

	for ( int i = 0; i < m_planeCount; ++i )
	{
		b3BodyId bodyId = b3Shape_GetBody( m_planeExtras[i].shapeId );
		b3BodyType bodyType = b3Body_GetType( bodyId );
		if ( bodyType != b3_dynamicBody )
		{
			continue;
		}

		b3Vec3 point = m_planeExtras[i].point;
		b3Vec3 normal = b3Neg( m_planes[i].plane.normal );

		float invMassA = 0.0f;
		float invMassB = b3Body_GetInverseMass( bodyId );
		b3Matrix3 invIB = b3Body_GetWorldInverseRotationalInertia( bodyId );

		b3Vec3 pB = b3Body_GetWorldCenterOfMass( bodyId );
		b3Vec3 rB = point - pB;

		b3Vec3 rnB = b3Cross( rB, normal );
		float kNormal = invMassA + invMassB + b3Dot( rnB, b3MulMV( invIB, rnB ) );
		float normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

		b3Vec3 vB = b3Body_GetLinearVelocity( bodyId );
		b3Vec3 omegaB = b3Body_GetAngularVelocity( bodyId );
		b3Vec3 vrB = b3Add( vB, b3Cross( omegaB, rB ) );
		float vn = b3Dot( b3Sub( vrB, m_velocity ), normal );
		float impulse = b3MaxFloat( -normalMass * vn, 0.0f );

		b3Vec3 P = b3MulSV( impulse, normal );
		m_velocity = b3MulSub( m_velocity, invMassA, P );

		b3Body_ApplyLinearImpulse( bodyId, P, point, true );
	}

	if ( clipVelocity )
	{
		// Using the velocity clipper can avoid picking up velocity from depenetration.
		// This allows the mover to avoid velocity from soft collision depenetration.
		m_velocity = b3ClipVector( m_velocity, m_planes, m_planeCount );
	}
	else if ( timeStep > 0.0f )
	{
		// Using the position delta is more holistic and intuitive in some cases.
		m_velocity = ( 1.0f / timeStep ) * ( m_transform.p - startPosition );
	}
}

void CharacterMover::Step( b3ShapeId* ignoreShapes, int ignoreCount, bool clipVelocity )
{
	m_ignoreShapeIds = ignoreShapes;
	m_ignoreCount = ignoreCount;

	b3Vec2 throttle = { 0.0f, 0.0f };
	b3Vec3 forward = -m_sample->m_camera->GetForward();
	b3Vec3 right = m_sample->m_camera->GetRight();
	forward.y = 0.0f;

	if ( m_sample->m_camera->m_thirdPerson )
	{
		if ( glfwGetKey( m_sample->m_window, GLFW_KEY_W ) )
		{
			throttle.x += 1.0f;
		}

		if ( glfwGetKey( m_sample->m_window, GLFW_KEY_S ) )
		{
			throttle.x -= 1.0f;
		}

		if ( glfwGetKey( m_sample->m_window, GLFW_KEY_A ) )
		{
			throttle.y -= 1.0f;
		}

		if ( glfwGetKey( m_sample->m_window, GLFW_KEY_D ) )
		{
			throttle.y += 1.0f;
		}

		if ( glfwGetKey( m_sample->m_window, GLFW_KEY_SPACE ) && m_onGround == true )
		{
			m_velocity.y = m_jumpSpeed;
			m_onGround = false;
		}

		if ( m_onGround == true )
		{
			m_sprint = glfwGetKey( m_sample->m_window, GLFW_KEY_LEFT_SHIFT ) != 0;
		}
		else
		{
			m_sprint = false;
		}
	}

	float hertz = m_sample->m_context->hertz;
	float timeStep = hertz > 0.0f ? 1.0f / hertz : 0.0f;

	// throttle = { 0.0f, 0.0f, -1.0f };

	SolveMove( timeStep, forward, right, throttle, clipVelocity );

	Scene* scene = m_sample->m_scene;

	int count = m_planeCount;
	for ( int i = 0; i < count; ++i )
	{
		b3Plane plane = m_planes[i].plane;
		b3Vec3 p1 = m_transform.p + ( plane.offset - m_capsule.radius ) * plane.normal;
		b3Vec3 p2 = p1 + 0.1f * plane.normal;
		DrawPoint( scene, p1, 5.0f, b3_colorYellow );
		DrawLine( scene, p1, p2, b3_colorYellow );
	}

	DrawCapsule( scene, m_transform, m_capsule, b3_colorBlue );
	DrawLine( scene, m_transform.p, m_transform.p + m_velocity, b3_colorPurple );

	b3Vec3 p = m_transform.p;
	// m_sample->DrawTextLine( "position %.2f %.2f %.2f", p.x, p.y, p.z );
	// m_sample->DrawTextLine( "forward %.2f %.2f %.2f", forward.x, forward.y, forward.z );
	// m_sample->DrawTextLine( "right %.2f %.2f %.2f", right.x, right.y, right.z );
	// m_sample->DrawTextLine( "velocity %.2f %.2f %.2f", m_velocity.x, m_velocity.y, m_velocity.z );
	// m_sample->DrawTextLine( "iterations %d", m_totalIterations );

	if ( m_sample->m_camera->m_thirdPerson )
	{
		m_sample->m_camera->m_pivot = p;
		m_sample->m_camera->UpdateTransform();
	}

	m_ignoreShapeIds = nullptr;
	m_ignoreCount = 0;
}
