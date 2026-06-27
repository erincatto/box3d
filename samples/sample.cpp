// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "sample.h"

#include "benchmarks.h"
#include "gfx/debug_adapter.h"
#include "gfx/draw.h"
#include "gfx/gtao.h"
#include "gfx/keycodes.h"
#include "gfx/renderer.h"
#include "gfx/shadow.h"
#include "gfx/text.h"
#include "human.h"
#include "imgui.h"
#include "implot.h"
#include "jsmn.h"
#include "sokol_app.h"
#include "utils.h"

#include "box3d/box3d.h"

#include <ctype.h>
#include <nfd.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include <thread>

#define INFO_PANEL_WIDTH 16.0f

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

// Polled key state, fed from the host event loop (see main.cpp). Indexed by
// sokol keycode; samples read it through the KEY_* aliases in gfx/keycodes.h.
static bool s_keyDown[512];

bool IsKeyDown( int key )
{
	return (unsigned)key < 512u ? s_keyDown[key] : false;
}

void SetKeyDown( int key, bool down )
{
	if ( (unsigned)key < 512u )
	{
		s_keyDown[key] = down;
	}
}

void SampleContext::Save()
{
	const b3DebugDraw* gd = GetGuiDraw();
	int gtaoQuality = GetGtaoTraceParams().quality;

	FILE* file = fopen( settingsFileName, "w" );
	if ( file == nullptr )
	{
		return;
	}
	fprintf( file, "{\n" );
	fprintf( file, "  \"sampleIndex\": %d,\n", sampleIndex );
	fprintf( file, "  \"drawShapes\": %s,\n", gd->drawShapes ? "true" : "false" );
	fprintf( file, "  \"drawJoints\": %s,\n", gd->drawJoints ? "true" : "false" );
	fprintf( file, "  \"drawContacts\": %s,\n", gd->drawContacts ? "true" : "false" );
	fprintf( file, "  \"showDiagnostics\": %s,\n", showMetrics ? "true" : "false" );
	fprintf( file, "  \"enableShadows\": %s,\n", enableShadows ? "true" : "false" );
	fprintf( file, "  \"enableGtao\": %s,\n", enableGtao ? "true" : "false" );
	fprintf( file, "  \"gtaoQuality\": %d,\n", gtaoQuality );
	fprintf( file, "  \"enableIbl\": %s,\n", GetIblEnabled() ? "true" : "false" );
	fprintf( file, "  \"exposure\": %g,\n", GetExposure() );
	fprintf( file, "  \"sunStrength\": %g,\n", GetSun().strength );
	fprintf( file, "  \"debugView\": %d,\n", debugView );
	fprintf( file, "  \"showHullEdges\": %s,\n", GetEdgeOverlayParams().showHulls ? "true" : "false" );
	fprintf( file, "  \"showEdgeConvexity\": %s,\n", GetEdgeOverlayParams().showEdgeConvexity ? "true" : "false" );
	fprintf( file, "  \"replayKeyframeBudgetMB\": %d,\n", replayKeyframeBudgetMB );
	fprintf( file, "  \"replayKeyframeMinInterval\": %d\n", replayKeyframeMinInterval );
	fprintf( file, "}\n" );
	fclose( file );
}

#define MAX_TOKENS 64

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
			else if ( sampleIndex >= g_sampleCount )
			{
				sampleIndex = g_sampleCount - 1;
			}
		}
		else if ( jsoneq( data, &tokens[i], "drawShapes" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			GetGuiDraw()->drawShapes = strncmp( s, "true", 4 ) == 0;
		}
		else if ( jsoneq( data, &tokens[i], "drawJoints" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			GetGuiDraw()->drawJoints = strncmp( s, "true", 4 ) == 0;
		}
		else if ( jsoneq( data, &tokens[i], "drawContacts" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			GetGuiDraw()->drawContacts = strncmp( s, "true", 4 ) == 0;
		}
		else if ( jsoneq( data, &tokens[i], "showDiagnostics" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			showMetrics = strncmp( s, "true", 4 ) == 0;
		}
		else if ( jsoneq( data, &tokens[i], "enableShadows" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			enableShadows = strncmp( s, "true", 4 ) == 0;
		}
		else if ( jsoneq( data, &tokens[i], "enableGtao" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			enableGtao = strncmp( s, "true", 4 ) == 0;
		}
		else if ( jsoneq( data, &tokens[i], "gtaoQuality" ) == 0 )
		{
			int count = tokens[i + 1].end - tokens[i + 1].start;
			assert( count < 32 );
			const char* s = data + tokens[i + 1].start;
			strncpy( buffer, s, count );
			buffer[count] = 0;
			int quality = b3ClampInt( (int)strtol( buffer, nullptr, 10 ), 0, 2 );

			GtaoTraceParams p = GetGtaoTraceParams();
			p.quality = (AmbientOcclusionQuality)quality;
			SetGtaoTraceParams( p );
		}
		else if ( jsoneq( data, &tokens[i], "enableIbl" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			SetIblEnabled( strncmp( s, "true", 4 ) == 0 );
		}
		else if ( jsoneq( data, &tokens[i], "exposure" ) == 0 )
		{
			int count = tokens[i + 1].end - tokens[i + 1].start;
			assert( count < 32 );
			const char* s = data + tokens[i + 1].start;
			strncpy( buffer, s, count );
			buffer[count] = 0;
			SetExposure( strtof( buffer, nullptr ) );
		}
		else if ( jsoneq( data, &tokens[i], "sunStrength" ) == 0 )
		{
			int count = tokens[i + 1].end - tokens[i + 1].start;
			assert( count < 32 );
			const char* s = data + tokens[i + 1].start;
			strncpy( buffer, s, count );
			buffer[count] = 0;
			Sun sun = GetSun();
			sun.strength = strtof( buffer, nullptr );
			SetSun( sun );
		}
		else if ( jsoneq( data, &tokens[i], "debugView" ) == 0 )
		{
			int count = tokens[i + 1].end - tokens[i + 1].start;
			assert( count < 32 );
			const char* s = data + tokens[i + 1].start;
			strncpy( buffer, s, count );
			buffer[count] = 0;
			debugView = b3ClampInt( (int)strtol( buffer, nullptr, 10 ), 0, 4 );
		}
		else if ( jsoneq( data, &tokens[i], "showHullEdges" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			EdgeOverlayParams p = GetEdgeOverlayParams();
			p.showHulls = strncmp( s, "true", 4 ) == 0;
			SetEdgeOverlayParams( &p );
		}
		else if ( jsoneq( data, &tokens[i], "showEdgeConvexity" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			EdgeOverlayParams p = GetEdgeOverlayParams();
			p.showEdgeConvexity = strncmp( s, "true", 4 ) == 0;
			SetEdgeOverlayParams( &p );
		}
		else if ( jsoneq( data, &tokens[i], "replayKeyframeBudgetMB" ) == 0 )
		{
			int count = tokens[i + 1].end - tokens[i + 1].start;
			if ( count > (int)sizeof( buffer ) - 1 )
			{
				count = (int)sizeof( buffer ) - 1;
			}
			const char* s = data + tokens[i + 1].start;
			memcpy( buffer, s, count );
			buffer[count] = 0;
			replayKeyframeBudgetMB = b3ClampInt( (int)strtol( buffer, nullptr, 10 ), 64, 4096 );
		}
		else if ( jsoneq( data, &tokens[i], "replayKeyframeMinInterval" ) == 0 )
		{
			int count = tokens[i + 1].end - tokens[i + 1].start;
			if ( count > (int)sizeof( buffer ) - 1 )
			{
				count = (int)sizeof( buffer ) - 1;
			}
			const char* s = data + tokens[i + 1].start;
			memcpy( buffer, s, count );
			buffer[count] = 0;
			replayKeyframeMinInterval = b3ClampInt( (int)strtol( buffer, nullptr, 10 ), 1, 1024 );
		}
	}

	free( data );
}

Sample::Sample( SampleContext* context )
{
	m_context = context;
	m_camera = &m_context->camera;

	if ( m_camera->m_thirdPerson )
	{
		sapp_lock_mouse( false );
		m_camera->m_thirdPerson = false;
	}

	m_worldId = b3_nullWorldId;

	m_recording = nullptr;
	m_recordStartStep = 0;

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
	ResetAdapterPool();
	ResetGroundShapeId();
	if ( B3_IS_NON_NULL( m_worldId ) )
	{
		FinishRecording();
		b3DestroyWorld( m_worldId );
	}
}

void Sample::StartRecording()
{
	if ( m_recording != nullptr )
	{
		return;
	}

	// Snapshot the live world as the seed, so recording can begin at any step boundary.
	m_recording = b3CreateRecording( 0 );
	b3World_StartRecording( m_worldId, m_recording );
	m_recordStartStep = m_stepCount;
}

void Sample::FinishRecording()
{
	if ( m_recording == nullptr )
	{
		return;
	}

	b3World_StopRecording( m_worldId );
	b3SaveRecordingToFile( m_recording, m_context->recordingFile );
	b3DestroyRecording( m_recording );
	m_recording = nullptr;
}

void Sample::CreateWorld( b3Capacity* capacity )
{
	if ( B3_IS_NON_NULL( m_worldId ) )
	{
		FinishRecording();
		b3DestroyWorld( m_worldId );
	}

	m_mouseBodyId = {};
	m_mouseJointId = {};
	m_mousePoint = {};

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = m_context->workerCount;
	worldDef.enableSleep = m_context->enableSleep;
	AttachToWorldDef( &worldDef );
	if ( capacity != nullptr )
	{
		worldDef.capacity = *capacity;
	}
	m_worldId = b3CreateWorld( &worldDef );

	b3World_SetContactRecycleDistance( m_worldId, m_context->recycleDistance );
}

void Sample::ResetText()
{
	// Below the menu bar when the UI shows. Hidden, the minimal HUD owns the
	// top-left, so start lower.
	float fontSize = ImGui::GetFontSize();
	if ( m_context->showUI )
	{
		m_textLine = (int)( ImGui::GetFrameHeight() + 0.5f * fontSize );
	}
	else
	{
		m_textLine = (int)( 3.0f * fontSize );
	}
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

	// Cursor pick feeds the surface type readout only. Highlighting is selection driven.
	if ( m_camera->m_thirdPerson == false )
	{
		PickRay pickRay = m_camera->BuildPickRay( m_context->mouseX, m_context->mouseY );

		b3QueryFilter filter = b3DefaultQueryFilter();
		filter.name = "pick";
		b3RayResult result = b3World_CastRayClosest( m_worldId, pickRay.origin, pickRay.translation, filter );

		if ( result.hit )
		{
			if ( b3Shape_GetType( result.shapeId ) == b3_meshShape )
			{
				m_triangleIndex = result.triangleIndex;
			}

			m_userMaterialId = result.userMaterialId;
		}
	}

	// The frame latched the origin before Step, but a third person follow moves the eye while
	// stepping, so refresh it here. Shapes come back demoted to float against this origin, the same
	// relative frame the translation free view renders, so everything stays precise far from the origin.
	SetDrawOrigin( m_camera->DrawOrigin() );

	b3DebugDraw debugDraw;
	MakeDebugDraw( &debugDraw );

	// Box3D uses this to decide which shapes enter the draw set and lazily fire
	// createDebugShape. The camera derives it from the view distance, in length units
	// around the simulation eye, matching the broad-phase tree and the far plane.
	debugDraw.drawingBounds = m_camera->DrawBounds();

	ApplyGuiFlags( &debugDraw );

	b3World_Draw( m_worldId, &debugDraw, B3_DEFAULT_MASK_BITS );
}

bool Sample::FocusBounds( b3AABB* )
{
	return false;
}

float Sample::InfoPanelWidthEm() const
{
	return INFO_PANEL_WIDTH;
}

b3BodyId Sample::FocusBody() const
{
	return GetSelectedBody();
}

void Sample::FocusHome()
{
	b3AABB aabb = b3World_GetBounds( m_worldId );
	float aspect = m_camera->m_height > 0 ? (float)m_camera->m_width / (float)m_camera->m_height : 1.0f;
	m_camera->Frame( aabb, aspect, 0.75f );
}

void Sample::ResetProfile()
{
	m_stepCount = 0;
	memset( m_profiles, 0, sizeof( m_profiles ) );
	m_currentProfileIndex = 0;
	m_profileReadIndex = 0;
	m_profileWriteIndex = 0;
}

b3BodyId Sample::AddGroundBox( float extent )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.name = "ground";
	bodyDef.position = { 0.0f, -1.0f, 0.0f };
	b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull hull = b3MakeBoxHull( extent, 1.0f, extent );
	b3ShapeId shapeId = b3CreateHullShape( groundId, &shapeDef, &hull.base );

	// Draw the ground with the procedural grid material
	SetGroundShape( shapeId );

	return groundId;
}

struct RowDef
{
	const char* name;
	int indent;
	ImU32 color;
};

float AddSegment( ImDrawList* dl, float availWidth, float t, float stepNow, ImU32 col, float x, ImVec2 cursor, float barHeight )
{
	float w = availWidth * ( t / stepNow );
	if ( w > 0.0f )
	{
		dl->AddRectFilled( ImVec2( x, cursor.y ), ImVec2( x + w, cursor.y + barHeight ), col );
		x += w;
	}

	return x;
}

// Bottom diagnostics drawer (M). Tabs: Profile, Counters, Renderer, and the
// optional ImPlot frame-time chart. Anchored along the window bottom, clear
// of the right info panel.
void Sample::DrawMetrics()
{
	if ( m_context->showMetrics == false )
	{
		return;
	}

	float fontSize = ImGui::GetFontSize();
	float menuWidth = InfoPanelWidthEm() * fontSize;
	float drawerHeight = 16.0f * fontSize;
	float drawerWidth = m_camera->m_width - menuWidth - 1.5f * fontSize;

	ImGui::SetNextWindowPos( { 0.5f * fontSize, m_camera->m_height - drawerHeight - 0.5f * fontSize } );
	ImGui::SetNextWindowSize( { drawerWidth, drawerHeight } );

	ImGui::Begin( "Metrics", nullptr,
				  ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
					  ImGuiWindowFlags_NoTitleBar );

	if ( ImGui::BeginTabBar( "MetricsTabs", ImGuiTabBarFlags_None ) == false )
	{
		ImGui::End();
		return;
	}

	if ( ImGui::BeginTabItem( "Profile" ) )
	{
		int count = m_profileWriteIndex - m_profileReadIndex;

		// Unroll ring buffer into per-field histories.
		constexpr int kRowCount = 22;
		float histories[kRowCount][m_profileCapacity];
		float totals[kRowCount] = {};
		for ( int i = 0; i < count; ++i )
		{
			int idx = ( m_profileReadIndex + i ) & ( m_profileCapacity - 1 );
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

		// Smoothed over the last few frames so bars don't jitter visibly.
		constexpr int kNowWindow = 10;
		float now[kRowCount] = {};
		{
			int n = count < kNowWindow ? count : kNowWindow;
			if ( n > 0 )
			{
				float inv = 1.0f / n;
				for ( int r = 0; r < kRowCount; ++r )
				{
					float sum = 0.0f;
					for ( int i = count - n; i < count; ++i )
					{
						sum += histories[r][i];
					}
					now[r] = sum * inv;
				}
			}
		}

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

		float rowMax[kRowCount] = {};
		for ( int r = 0; r < kRowCount; ++r )
		{
			for ( int i = 0; i < count; ++i )
			{
				if ( histories[r][i] > rowMax[r] )
				{
					rowMax[r] = histories[r][i];
				}
			}
		}

		// Match Frame Time chart's first three colors so rows read with the line plot.
		const ImU32 colorStep = IM_COL32( 102, 153, 255, 255 );
		const ImU32 colorPairs = IM_COL32( 220, 220, 220, 255 );
		const ImU32 colorCollide = IM_COL32( 255, 140, 51, 255 );
		const ImU32 colorSolve = IM_COL32( 102, 204, 102, 255 );
		const ImU32 colorSensors = IM_COL32( 200, 120, 220, 255 );
		const ImU32 colorOther = IM_COL32( 90, 90, 90, 255 );
		const ImU32 colorDefault = IM_COL32( 220, 220, 220, 255 );

		const RowDef rows[kRowCount] = {
			{ "step", 0, colorStep },			{ "pairs", 0, colorPairs },			 { "collide", 0, colorCollide },
			{ "solve", 0, colorSolve },			{ "setup", 1, colorDefault },		 { "constraints", 1, colorDefault },
			{ "prepare", 2, colorDefault },		{ "velocities", 2, colorDefault },	 { "warm start", 2, colorDefault },
			{ "bias", 2, colorDefault },		{ "positions", 2, colorDefault },	 { "relax", 2, colorDefault },
			{ "restitution", 2, colorDefault }, { "store", 2, colorDefault },		 { "split islands", 2, colorDefault },
			{ "transforms", 1, colorDefault },	{ "joint events", 1, colorDefault }, { "hit events", 1, colorDefault },
			{ "refit BVH", 1, colorDefault },	{ "sleep", 1, colorDefault },		 { "bullets", 1, colorDefault },
			{ "sensors", 0, colorSensors },
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
		const float stepNow = b3MaxFloat( now[0], 0.001f );

		if ( ImGui::Button( "Reset" ) )
		{
			ResetProfile();
		}
		ImGui::SameLine();
		ImGui::Checkbox( "Show plots", &s_showPlots );
		ImGui::SameLine();
		ImGui::Text( "   step %.2f ms", now[0] );

		// Flame strip: step subdivided by top-level children.
		{
			float pairsT = now[1];
			float collideT = now[2];
			float solveT = now[3];
			float sensorsT = now[21];
			float otherT = b3MaxFloat( stepNow - pairsT - collideT - solveT - sensorsT, 0.0f );

			float availWidth = ImGui::GetContentRegionAvail().x;
			float barHeight = 1.5f * fontSize;
			ImDrawList* dl = ImGui::GetWindowDrawList();
			ImVec2 cursor = ImGui::GetCursorScreenPos();
			float x = cursor.x;

			x = AddSegment( dl, availWidth, pairsT, stepNow, colorPairs, x, cursor, barHeight );
			x = AddSegment( dl, availWidth, collideT, stepNow, colorCollide, x, cursor, barHeight );
			x = AddSegment( dl, availWidth, solveT, stepNow, colorSolve, x, cursor, barHeight );
			x = AddSegment( dl, availWidth, sensorsT, stepNow, colorSensors, x, cursor, barHeight );
			x = AddSegment( dl, availWidth, otherT, stepNow, colorOther, x, cursor, barHeight );

			ImGui::Dummy( ImVec2( availWidth, barHeight ) );
		}

		const ImGuiTableFlags tableFlags =
			ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_ScrollY;

		const int colCount = s_showPlots ? 6 : 5;
		ImVec2 tableSize = ImGui::GetContentRegionAvail();
		if ( ImGui::BeginTable( "profile", colCount, tableFlags, tableSize ) )
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

				// Hide leaf rows that are entirely zero. Parents stay so structure reads.
				if ( !hasChildren[r] && now[r] == 0.0f && avg[r] == 0.0f && rowMax[r] == 0.0f )
				{
					continue;
				}

				const RowDef& d = rows[r];
				const float* hist = histories[r];

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
				ImGui::Text( "%6.2f", rowMax[r] );

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
						ImGui::PlotLines( id, hist, count, 0, nullptr, 0.0f, rowMax[r] * 1.05f + 0.001f,
										  ImVec2( -FLT_MIN, rowHeight ) );
						ImGui::PopStyleColor();
					}
				}
			}
			ImGui::EndTable();
		}

		ImGui::EndTabItem();
	}

	if ( ImGui::BeginTabItem( "Counters" ) )
	{
		ImGui::BeginChild( "##counters_scroll" );
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
		ImGui::Text( "toi d/p/r = %d/%d/%d", s.distanceIterations, s.pushBackIterations, s.rootIterations );
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
			ImGui::TableSetupColumn( "manifolds", ImGuiTableColumnFlags_WidthFixed, 3.5f * fontSize );
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

		ImGui::EndChild();
		ImGui::EndTabItem();
	}

	if ( ImGui::BeginTabItem( "Renderer" ) )
	{
		const RenderStats st = GetRenderStats();

		// Display-link frame time vs the RenderFrame submit cost, both
		// EMA-smoothed so the readout doesn't flicker.
		const float frameMs = (float)( sapp_frame_duration() * 1000.0 );
		static float smoothedFrameMs = 0.0f;
		static float smoothedCpuMs = 0.0f;
		smoothedFrameMs = ( smoothedFrameMs <= 0.0f ) ? frameMs : smoothedFrameMs * 0.9f + frameMs * 0.1f;
		smoothedCpuMs = ( smoothedCpuMs <= 0.0f ) ? st.frameTimeMs : smoothedCpuMs * 0.9f + st.frameTimeMs * 0.1f;
		const float fps = ( smoothedFrameMs > 0.0f ) ? ( 1000.0f / smoothedFrameMs ) : 0.0f;

		const char* backend = "?";
		switch ( sg_query_backend() )
		{
			case SG_BACKEND_D3D11:
				backend = "D3D11";
				break;
			case SG_BACKEND_METAL_MACOS:
				backend = "Metal";
				break;
			case SG_BACKEND_GLCORE:
				backend = "GLCORE";
				break;
			case SG_BACKEND_GLES3:
				backend = "GLES3";
				break;
			case SG_BACKEND_WGPU:
				backend = "WGPU";
				break;
			default:
				break;
		}

		ImGui::Text( "%s   %.2f ms  (%.0f FPS)   CPU %.2f ms", backend, smoothedFrameMs, fps, smoothedCpuMs );
		ImGui::Text( "draw calls %d  (approx)", st.drawCallCount );
		ImGui::Separator();
		ImGui::Text( "opaque   cubes %d  spheres %d  capsules %d  geom %d (%d spans)", st.cubeCount, st.sphereCount,
					 st.capsuleCount, st.geomInstanceCount, st.geomSpanCount );
		ImGui::Text( "transp.  cubes %d  spheres %d  capsules %d  geom %d", st.cubeCountXp, st.sphereCountXp, st.capsuleCountXp,
					 st.geomInstanceCountXp );
		ImGui::Text( "overlays lines %d  points %d", st.lineCount, st.pointCount );

		ImGui::EndTabItem();
	}

	if ( ImGui::BeginTabItem( "Frame Time" ) )
	{
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

		ImVec2 plotSize = ImGui::GetContentRegionAvail();
		if ( ImPlot::BeginPlot( "Profile", plotSize, ImPlotFlags_NoTitle ) )
		{
			ImPlot::SetupAxes( "t", "ms" );
			ImPlot::SetupAxisLimits( ImAxis_X1, 0.0, m_profileCapacity / 60.0 );
			ImPlot::SetupAxisLimits( ImAxis_Y1, 0.0, b3MaxFloat( maxValue, 1.0f ) * 1.05, ImPlotCond_Always );
			ImPlot::PlotLine( "step", times, stepTimes, count );
			ImPlot::PlotLine( "collide", times, collideTimes, count );
			ImPlot::PlotLine( "solve", times, solveTimes, count );
			ImPlot::EndPlot();
		}

		ImGui::EndTabItem();
	}

	// Sample-specific tabs (the Replay viewer adds Timeline here).
	DrawMetricsTab();

	ImGui::EndTabBar();
	ImGui::End();
}

void Sample::MouseDown( b3Vec2 p, int button, int modifiers )
{
	if ( button == 0 && modifiers == 0 )
	{
		// Plain click selects the body under the cursor, any type. Empty space clears.
		PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );

		b3QueryFilter filter = b3DefaultQueryFilter();
		filter.name = "select";
		b3RayResult result = b3World_CastRayClosest( m_worldId, pickRay.origin, pickRay.translation, filter );

		if ( result.hit )
		{
			SetSelectedBody( b3Shape_GetBody( result.shapeId ) );
		}
		else
		{
			ClearSelection();
		}
	}
	else if ( button == 0 && modifiers == MOD_CTRL )
	{
		// Ctrl click grabs a dynamic body with a kinematic mouse body and a motor joint.
		PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );

		b3QueryFilter filter = b3DefaultQueryFilter();
		filter.name = "grab";
		b3RayResult result = b3World_CastRayClosest( m_worldId, pickRay.origin, pickRay.translation, filter );

		b3BodyId bodyId = result.hit ? b3Shape_GetBody( result.shapeId ) : b3_nullBodyId;
		if ( result.hit && b3Body_GetType( bodyId ) == b3_dynamicBody )
		{
			m_mousePoint = result.point;

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_kinematicBody;
			bodyDef.position = m_mousePoint;
			bodyDef.enableSleep = false;
			m_mouseBodyId = b3CreateBody( m_worldId, &bodyDef );

			// Grabbing also selects so the body stays highlighted while dragged.
			SetSelectedBody( bodyId );

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
	else if ( modifiers & MOD_SHIFT )
	{
		PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
		b3Vec3 direction = b3Normalize( pickRay.translation );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		if ( modifiers & MOD_CTRL )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = pickRay.origin + 2.0f * direction;
			bodyDef.linearVelocity = ( 10.0f * m_launchSpeedScale ) * direction;
			bodyDef.isBullet = true;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3HullData* hull = b3CreateCylinder( 2.0f, 0.15f, 0.0f, 6 );
			b3CreateHullShape( bodyId, &shapeDef, hull );
			b3DestroyHull( hull );
		}
		else if ( modifiers & MOD_ALT )
		{
			b3Pos position = pickRay.origin + 2.0f * direction;
			Human human = {};
			CreateHuman( &human, m_worldId, position, 1.0f, 1.0f, 1.0f, 0, nullptr, true );
			Human_SetBullet( &human, true );
			Human_SetVelocity( &human, ( 10.0f * m_launchSpeedScale ) * direction );
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
		// The cursor is locked in third person, so look uses the host's
		// relative deltas. Camera angles are radians here (host camera).
		m_mouseDelta = { m_context->mouseDX, m_context->mouseDY };

		const float sensitivity = 0.1f * B3_DEG_TO_RAD;
		m_camera->m_yaw -= 2.0f * sensitivity * m_mouseDelta.x;
		m_camera->m_pitch += sensitivity * m_mouseDelta.y;
		m_camera->m_pitch = b3ClampFloat( m_camera->m_pitch, -85.0f * B3_DEG_TO_RAD, 85.0f * B3_DEG_TO_RAD );
	}

	PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
	if ( B3_IS_NON_NULL( m_mouseJointId ) )
	{
		m_mousePoint = pickRay.origin + m_mouseFraction * pickRay.translation;
	}
}

void Sample::DrawTextLine( const char* text, ... )
{
	va_list args;
	va_start( args, text );
	char buffer[512];
	vsnprintf( buffer, sizeof( buffer ), text, args );
	va_end( args );
	DrawScreenString( 5, m_textLine, MakeColor( b3_colorWhite ), buffer );
	m_textLine += m_textIncrement;
}

SampleEntry g_sampleEntries[MAX_SAMPLES];
int g_sampleCount = 0;
int g_replayIndex = -1;

int RegisterSample( const char* category, const char* name, SampleCreateFcn* fcn )
{
	int index = g_sampleCount;
	if ( index < MAX_SAMPLES )
	{
		g_sampleEntries[index] = { category, name, fcn };
		g_sampleCount += 1;
		return index;
	}

	return -1;
}

// Like RegisterSample but remembers the index so the Replay menu can switch to the viewer.
int RegisterReplay( const char* category, const char* name, SampleCreateFcn* fcn )
{
	int index = RegisterSample( category, name, fcn );
	if ( index >= 0 )
	{
		g_replayIndex = index;
	}
	return index;
}

void SelectSample( SampleContext* context, int selection, bool restart )
{
	if ( selection < 0 || selection >= g_sampleCount )
	{
		// Out of range: keep the current sample rather than calling a null factory.
		return;
	}

	// delete tolerates the first selection, before any sample exists.
	delete context->sample;
	context->sample = nullptr;

	context->sampleIndex = selection;
	context->restart = restart;
	context->sample = g_sampleEntries[selection].CreateFcn( context );

	// Fit the shadow cascade range to the world bounds.
	b3AABB bounds = b3World_GetBounds( context->sample->m_worldId );
	float diagonal = b3Distance( bounds.lowerBound, bounds.upperBound );
	SetShadowSplitFar( b3ClampFloat( diagonal, SHADOW_SPLIT_FAR, 200.0f ) );

	// Samples read restart only while constructing, to keep the camera across a
	// restart. Clear it so a later switch starts fresh.
	context->restart = false;
}

// Subsequence fuzzy match. Returns a score (higher is better) or -1 if the
// needle is not a subsequence of the haystack. Bonuses for a prefix match, a
// word start, and a contiguous run, so "stack" ranks Pyramid > Stack sensibly.
static int FuzzyScore( const char* needle, const char* haystack )
{
	if ( needle == nullptr || needle[0] == '\0' )
	{
		return 0;
	}

	int score = 0;
	int hi = 0;
	int prevMatchHi = -2;

	for ( int ni = 0; needle[ni] != '\0'; ++ni )
	{
		int nc = tolower( (unsigned char)needle[ni] );
		while ( haystack[hi] != '\0' && tolower( (unsigned char)haystack[hi] ) != nc )
		{
			++hi;
		}
		if ( haystack[hi] == '\0' )
		{
			return -1;
		}

		int bonus = 1;
		if ( hi == 0 )
		{
			bonus += 10; // prefix
		}
		else if ( !isalnum( (unsigned char)haystack[hi - 1] ) )
		{
			bonus += 5; // word start
		}
		if ( hi == prevMatchHi + 1 )
		{
			bonus += 3; // contiguous run
		}

		score += bonus;
		prevMatchHi = hi;
		++hi;
	}

	return score;
}

struct Scored
{
	int idx;
	int score;
};

// Filter and rank the sample registry against the picker query. Name matches
// outweigh category-only matches. Equal scores keep the sorted registry order.
static void RebuildPicker( const char* q, int* outFiltered, int* outCount )
{
	static Scored scored[MAX_SAMPLES];
	int n = 0;
	for ( int i = 0; i < g_sampleCount; ++i )
	{
		int nameScore = FuzzyScore( q, g_sampleEntries[i].Name );
		int catScore = FuzzyScore( q, g_sampleEntries[i].Category );
		int best = -1;
		if ( nameScore >= 0 )
		{
			best = nameScore * 2;
		}
		if ( catScore >= 0 && catScore > best )
		{
			best = catScore;
		}
		if ( best < 0 )
		{
			continue;
		}
		scored[n].idx = i;
		scored[n].score = best;
		n += 1;
	}

	// Stable insertion sort by score descending.
	for ( int i = 1; i < n; ++i )
	{
		Scored tmp = scored[i];
		int j = i - 1;
		while ( j >= 0 && scored[j].score < tmp.score )
		{
			scored[j + 1] = scored[j];
			--j;
		}
		scored[j + 1] = tmp;
	}

	for ( int i = 0; i < n; ++i )
	{
		outFiltered[i] = scored[i].idx;
	}
	*outCount = n;
}

static void DrawRow( const char* key, const char* desc )
{
	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex( 0 );
	ImGui::TextUnformatted( key );
	ImGui::TableSetColumnIndex( 1 );
	ImGui::TextUnformatted( desc );
}

// b3HexColor (0xRRGGBB) to an opaque ImVec4 for panel text. Distinct from
// gfx/utility.h's MakeColor, which returns the renderer's linear Vec4.
static ImVec4 HexColor( b3HexColor hexColor )
{
	uint32_t h = (uint32_t)hexColor;
	ImU32 color = IM_COL32( ( h >> 16 ) & 0xFF, ( h >> 8 ) & 0xFF, h & 0xFF, 255 );
	return ImGui::ColorConvertU32ToFloat4( color );
}

// Abbreviated render controls: mostly visualization/debug toggles plus a single
// exposure slider. The deep GTAO knobs and most edge/outline parameters keep
// their renderer defaults, only the edge convexity coloring is exposed.
static void DrawRenderMenu( SampleContext& ctx )
{
	ImGui::MenuItem( "Shadows", nullptr, &ctx.enableShadows );

	if ( ImGui::BeginMenu( "GTAO" ) )
	{
		ImGui::MenuItem( "Enable", nullptr, &ctx.enableGtao );

		GtaoTraceParams p = GetGtaoTraceParams();
		int quality = p.quality;
		static const char* qualityNames[] = { "Medium", "High", "Ultra" };
		if ( ImGui::Combo( "Quality", &quality, qualityNames, 3 ) )
		{
			p.quality = (AmbientOcclusionQuality)quality;
			SetGtaoTraceParams( p );
		}
		ImGui::EndMenu();
	}

	bool ibl = GetIblEnabled();
	if ( ImGui::MenuItem( "IBL", nullptr, &ibl ) )
	{
		SetIblEnabled( ibl );
	}

	EdgeOverlayParams edgeParams = GetEdgeOverlayParams();
	if ( ImGui::MenuItem( "Hull Edges", nullptr, &edgeParams.showHulls ) )
	{
		SetEdgeOverlayParams( &edgeParams );
	}
	if ( ImGui::MenuItem( "Edge Convexity", nullptr, &edgeParams.showEdgeConvexity ) )
	{
		SetEdgeOverlayParams( &edgeParams );
	}

	ImGui::Separator();

	ImGui::PushItemWidth( 8.0f * ImGui::GetFontSize() );
	float ev = GetExposure();
	if ( ImGui::SliderFloat( "Exposure", &ev, -8.0f, 4.0f, "%.2f" ) )
	{
		SetExposure( ev );
	}

	Sun sun = GetSun();
	if ( ImGui::SliderFloat( "Sun", &sun.strength, 0.0f, 1.0f, "%.2f" ) )
	{
		SetSun( sun );
	}

	ImGui::PopItemWidth();

	ImGui::Separator();

	if ( ImGui::BeginMenu( "Debug View" ) )
	{
		static const char* viewNames[] = {
			"0 - lit", "1 - view-space distance", "2 - CSM cascade", "3 - view-space normal", "4 - raw GTAO",
		};
		for ( int i = 0; i < 5; ++i )
		{
			if ( ImGui::RadioButton( viewNames[i], ctx.debugView == i ) )
			{
				ctx.debugView = i;
			}
		}
		ImGui::EndMenu();
	}
}

static void DrawMenuBar( SampleContext* context )
{
	b3DebugDraw* gd = GetGuiDraw();
	float fontSize = ImGui::GetFontSize();

	if ( ImGui::BeginMainMenuBar() )
	{
		if ( ImGui::BeginMenu( "Sim" ) )
		{
			ImGui::MenuItem( "Pause", "P", &context->pause );
			if ( ImGui::MenuItem( "Single Step", "O" ) )
			{
				context->singleStep += 1;
			}
			if ( ImGui::MenuItem( "Restart", "R" ) )
			{
				SelectSample( context, context->sampleIndex, true );
			}
			ImGui::Separator();
			if ( ImGui::MenuItem( "Previous Sample", "[" ) )
			{
				SelectSample( context, b3MaxInt( 0, context->sampleIndex - 1 ), false );
			}
			if ( ImGui::MenuItem( "Next Sample", "]" ) )
			{
				SelectSample( context, b3MinInt( g_sampleCount - 1, context->sampleIndex + 1 ), false );
			}
			ImGui::Separator();
			if ( ImGui::MenuItem( "Reset Profile" ) )
			{
				context->sample->ResetProfile();
			}
			if ( ImGui::MenuItem( "Dump Mem Stats" ) )
			{
				b3World_DumpMemoryStats( context->sample->m_worldId );
			}
			ImGui::Separator();
			if ( ImGui::MenuItem( "Quit", "Esc" ) )
			{
				sapp_request_quit();
			}
			ImGui::EndMenu();
		}

		if ( ImGui::BeginMenu( "View" ) )
		{
			if ( ImGui::MenuItem( "Hide UI", "Tab" ) )
			{
				context->showUI = false;
			}
			if ( ImGui::MenuItem( "Frame Camera", "Home" ) )
			{
				b3AABB aabb = b3World_GetBounds( context->sample->m_worldId );
				Camera& cam = context->camera;
				float aspect = cam.m_height > 0 ? (float)cam.m_width / (float)cam.m_height : 1.0f;
				cam.Frame( aabb, aspect, 0.75f );
			}
			ImGui::MenuItem( "Shapes", nullptr, &gd->drawShapes );
			ImGui::MenuItem( "Transparent", nullptr, &context->transparentDynamic );
			ImGui::MenuItem( "Joints", nullptr, &gd->drawJoints );
			ImGui::MenuItem( "Joint Extras", nullptr, &gd->drawJointExtras );
			ImGui::MenuItem( "Bounds", nullptr, &gd->drawBounds );
			ImGui::MenuItem( "Mass", nullptr, &gd->drawMass );
			ImGui::MenuItem( "Body Names", nullptr, &gd->drawBodyNames );
			ImGui::MenuItem( "Graph Colors", nullptr, &gd->drawGraphColors );
			ImGui::MenuItem( "Islands", nullptr, &gd->drawIslands );
			ImGui::Separator();
			ImGui::MenuItem( "Contact Points", nullptr, &gd->drawContacts );
			ImGui::MenuItem( "Contact Normals", nullptr, &gd->drawContactNormals );
			ImGui::MenuItem( "Contact Features", nullptr, &gd->drawContactFeatures );
			ImGui::MenuItem( "Contact Forces", nullptr, &gd->drawContactForces );
			ImGui::MenuItem( "Friction Forces", nullptr, &gd->drawFrictionForces );
			if ( ImGui::BeginMenu( "Anchor" ) )
			{
				if ( ImGui::MenuItem( "Anchor A", nullptr, gd->drawAnchorA != 0 ) )
				{
					gd->drawAnchorA = 1;
				}
				if ( ImGui::MenuItem( "Anchor B", nullptr, gd->drawAnchorA == 0 ) )
				{
					gd->drawAnchorA = 0;
				}
				ImGui::EndMenu();
			}
			ImGui::Separator();
			ImGui::MenuItem( "Diagnostics", "M", &context->showMetrics );
			ImGui::Separator();
			if ( ImGui::BeginMenu( "Scale" ) )
			{
				ImGui::PushItemWidth( 6.0f * fontSize );
				ImGui::InputFloat( "Joint", &gd->jointScale );
				ImGui::InputFloat( "Force", &gd->forceScale, 0, 0, "%.6f", ImGuiInputTextFlags_CharsScientific );
				ImGui::PopItemWidth();
				ImGui::EndMenu();
			}
			ImGui::EndMenu();
		}

		if ( ImGui::BeginMenu( "Render" ) )
		{
			DrawRenderMenu( *context );
			ImGui::EndMenu();
		}

		if ( ImGui::BeginMenu( "Samples" ) )
		{
			int i = 0;
			while ( i < g_sampleCount )
			{
				const char* category = g_sampleEntries[i].Category;
				if ( ImGui::BeginMenu( category ) )
				{
					while ( i < g_sampleCount && strcmp( category, g_sampleEntries[i].Category ) == 0 )
					{
						bool selected = ( i == context->sampleIndex );
						if ( ImGui::MenuItem( g_sampleEntries[i].Name, nullptr, selected ) )
						{
							SelectSample( context, i, false );
						}
						++i;
					}
					ImGui::EndMenu();
				}
				else
				{
					while ( i < g_sampleCount && strcmp( category, g_sampleEntries[i].Category ) == 0 )
					{
						++i;
					}
				}
			}
			ImGui::EndMenu();
		}

		// Only present once the replay viewer is registered. Open pops a native picker, then hands
		// the chosen file to the viewer through replayFile.
		if ( g_replayIndex >= 0 && ImGui::BeginMenu( "Replay" ) )
		{
			if ( ImGui::MenuItem( "Open..." ) )
			{
				NFD_Init();
				nfdu8char_t* outPath = nullptr;
				nfdu8filteritem_t filter[1] = { { "Box3D recording", "b3rec" } };
				if ( NFD_OpenDialogU8( &outPath, filter, 1, nullptr ) == NFD_OKAY )
				{
					snprintf( context->replayFile, sizeof( context->replayFile ), "%s", outPath );
					NFD_FreePathU8( outPath );
					SelectSample( context, g_replayIndex, false );
				}
				NFD_Quit();
			}
			ImGui::EndMenu();
		}

		static bool showHelp = false;
		static bool showAbout = false;
		if ( ImGui::BeginMenu( "Help" ) )
		{
			ImGui::MenuItem( "Controls", nullptr, &showHelp );
			ImGui::MenuItem( "About", nullptr, &showAbout );
			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();

		if ( showHelp )
		{
			ImGui::SetNextWindowPos( { context->camera.m_width * 0.5f, context->camera.m_height * 0.5f }, ImGuiCond_Appearing,
									 { 0.5f, 0.5f } );
			ImGui::SetNextWindowSize( { 26.0f * fontSize, 0.0f }, ImGuiCond_Appearing );

			if ( ImGui::Begin( "Controls", &showHelp, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize ) )
			{
				ImGui::SeparatorText( "Keyboard" );
				if ( ImGui::BeginTable( "keys", 2, ImGuiTableFlags_SizingFixedFit ) )
				{
					DrawRow( "Tab", "Show / hide UI" );
					DrawRow( "M", "Show / hide diagnostics" );
					DrawRow( "P", "Pause / resume" );
					DrawRow( "O", "Single step (Shift: 5)" );
					DrawRow( "R", "Restart sample" );
					DrawRow( "[  ]", "Previous / next sample" );
					DrawRow( "Ctrl+O", "Open sample picker" );
					DrawRow( "F / Home", "Frame selection / world" );
					DrawRow( "Esc", "Quit" );
					ImGui::EndTable();
				}

				ImGui::SeparatorText( "Mouse" );
				if ( ImGui::BeginTable( "mouse", 2, ImGuiTableFlags_SizingFixedFit ) )
				{
					DrawRow( "Left drag", "Move bodies (mouse joint)" );
					DrawRow( "Alt + left drag", "Orbit camera" );
					DrawRow( "Alt + middle drag", "Pan camera" );
					DrawRow( "Right drag", "Fly look (WASD to move)" );
					DrawRow( "Scroll", "Zoom" );
					DrawRow( "Shift + left", "Shoot (Ctrl spin, Alt ragdoll)" );
					ImGui::EndTable();
				}
			}
			ImGui::End();
		}

		if ( showAbout )
		{
			ImGui::SetNextWindowPos( { context->camera.m_width * 0.5f, context->camera.m_height * 0.5f }, ImGuiCond_Appearing,
									 { 0.5f, 0.5f } );
			ImGui::SetNextWindowSize( { 22.0f * fontSize, 0.0f }, ImGuiCond_Appearing );

			if ( ImGui::Begin( "About", &showAbout, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize ) )
			{
				b3Version version = b3GetVersion();
				ImGui::Text( "Box3D %d.%d.%d", version.major, version.minor, version.revision );
				ImGui::Spacing();
				ImGui::TextLinkOpenURL( "github.com/erincatto/box3d", "https://github.com/erincatto/box3d" );
			}
			ImGui::End();
		}
	}
}

static void DrawSamplePicker( SampleContext* context )
{
	float fontSize = ImGui::GetFontSize();

	// Opens a transient popup; type to filter by name or category, Up/Down to
	// navigate, Enter to select, Esc / click-outside to dismiss.
	static char query[64] = {};
	static char prevQuery[64] = {};
	static int highlight = 0;
	static int prevHighlight = 0;
	static int filtered[MAX_SAMPLES];
	static int filteredCount = 0;
	static bool justOpened = false;
	static bool forceScroll = false;

	if ( context->openSamplePicker )
	{
		ImGui::OpenPopup( "##sample_picker" );
		context->openSamplePicker = false;
		query[0] = '\0';
		prevQuery[0] = '\0';
		highlight = 0;
		prevHighlight = 0;
		RebuildPicker( query, filtered, &filteredCount );
		justOpened = true;
		forceScroll = true;
	}

	ImGui::SetNextWindowPos( { context->camera.m_width * 0.5f, context->camera.m_height * 0.35f }, ImGuiCond_Appearing,
							 { 0.5f, 0.5f } );
	ImGui::SetNextWindowSize( { 32.0f * fontSize, 0.0f }, ImGuiCond_Appearing );

	if ( ImGui::BeginPopup( "##sample_picker",
							ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings ) )
	{
		if ( justOpened )
		{
			ImGui::SetKeyboardFocusHere();
			justOpened = false;
		}

		ImGui::PushItemWidth( -1.0f );
		ImGui::InputTextWithHint( "##q", "Search by name or category...", query, sizeof( query ) );
		ImGui::PopItemWidth();

		if ( strcmp( query, prevQuery ) != 0 )
		{
			RebuildPicker( query, filtered, &filteredCount );
			strncpy( prevQuery, query, sizeof( prevQuery ) );
			prevQuery[sizeof( prevQuery ) - 1] = '\0';
			highlight = 0;
			forceScroll = true;
		}

		if ( filteredCount > 0 )
		{
			if ( ImGui::IsKeyPressed( ImGuiKey_DownArrow, true ) )
			{
				highlight = ( highlight + 1 ) % filteredCount;
			}
			if ( ImGui::IsKeyPressed( ImGuiKey_UpArrow, true ) )
			{
				highlight = ( highlight + filteredCount - 1 ) % filteredCount;
			}
		}
		bool commit = ImGui::IsKeyPressed( ImGuiKey_Enter, false ) || ImGui::IsKeyPressed( ImGuiKey_KeypadEnter, false );

		ImGui::BeginChild( "##results", { 0.0f, 14.0f * fontSize }, ImGuiChildFlags_Borders );
		for ( int row = 0; row < filteredCount; ++row )
		{
			int i = filtered[row];
			char label[160];
			snprintf( label, sizeof( label ), "%s  >  %s", g_sampleEntries[i].Category, g_sampleEntries[i].Name );
			bool sel = ( row == highlight );
			if ( ImGui::Selectable( label, sel ) )
			{
				highlight = row;
				commit = true;
			}
			if ( sel && ( forceScroll || highlight != prevHighlight ) )
			{
				ImGui::SetScrollHereY();
			}
		}
		ImGui::EndChild();
		prevHighlight = highlight;
		forceScroll = false;

		if ( commit && filteredCount > 0 )
		{
			SelectSample( context, filtered[highlight], false );
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}
}

static void DrawInfoPanel( SampleContext* context )
{
	const SampleEntry& entry = g_sampleEntries[context->sampleIndex];
	float fontSize = ImGui::GetFontSize();
	float menuWidth = context->sample->InfoPanelWidthEm() * fontSize;
	float menuBarHeight = ImGui::GetFrameHeight();

	// Full-height panel pinned under the menu bar at the right edge, matching Box2D.
	ImGui::SetNextWindowPos( { context->camera.m_width - menuWidth - 0.5f * fontSize, menuBarHeight + 0.5f * fontSize } );
	ImGui::SetNextWindowSize( { menuWidth, context->camera.m_height - menuBarHeight - fontSize } );

	ImGui::Begin( "Info", nullptr,
				  ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
					  ImGuiWindowFlags_NoTitleBar );

	ImGui::TextColored( HexColor( b3_colorGoldenRod ), "%s", entry.Name );
	ImGui::TextColored( HexColor( b3_colorLightGray ), "%s", entry.Category );
	ImGui::Separator();

	const float frameMs = (float)( sapp_frame_duration() * 1000.0 );
	ImGui::TextColored( HexColor( b3_colorSeaGreen ), "%.1f ms", frameMs );
	ImGui::TextColored( HexColor( b3_colorSeaGreen ), "step %d", context->sample->m_stepCount );
	ImGui::Separator();

	// The camera lives in display space, so pivot, radius and speed are in meters
	// regardless of the simulation's length units.
	b3Pos p = context->camera.m_pivot;
	ImGui::TextColored( HexColor( b3_colorSeaGreen ), "pivot m (%.1f, %.1f, %.1f)", p.x, p.y, p.z );
	float yawDeg = B3_RAD_TO_DEG * context->camera.m_yaw;
	float pitchDeg = B3_RAD_TO_DEG * context->camera.m_pitch;
	ImGui::TextColored( HexColor( b3_colorSeaGreen ), "yaw/pitch (%.1f, %.1f)", yawDeg, pitchDeg );
	ImGui::TextColored( HexColor( b3_colorSeaGreen ), "radius m %.1f, speed m/s %.1f", context->camera.m_radius,
						context->camera.m_speed );

	ImGui::Separator();

	ImGui::PushItemWidth( 6.0f * fontSize );
	if ( context->sample->DrawControls() )
	{
		ImGui::Separator();
	}
	ImGui::PopItemWidth();

	if ( context->sample->HasSolverControls() && ImGui::CollapsingHeader( "Solver", ImGuiTreeNodeFlags_DefaultOpen ) )
	{
		ImGui::PushItemWidth( 6.0f * fontSize );
		ImGui::SliderInt( "Sub-steps##Solver", &context->subStepCount, 1, 50 );
		ImGui::SliderFloat( "Hertz##Solver", &context->hertz, 5.0f, 240.0f, "%.0f hz" );

		if ( ImGui::SliderInt( "Workers##Solver", &context->workerCount, 1, B3_MAX_WORKERS ) )
		{
			context->workerCount = b3ClampInt( context->workerCount, 1, B3_MAX_WORKERS );
			SelectSample( context, context->sampleIndex, true );
		}

		float recyclingCentimeters = 100.0f * context->recycleDistance;
		if ( ImGui::SliderFloat( "Recycle##Solver", &recyclingCentimeters, 0.0f, 10.0f, "%.1f cm" ) )
		{
			context->recycleDistance = 0.01f * recyclingCentimeters;
			b3World_SetContactRecycleDistance( context->sample->m_worldId, context->recycleDistance );
		}
		ImGui::PopItemWidth();

		ImGui::Checkbox( "Sleep##Solver", &context->enableSleep );
		ImGui::Checkbox( "Warm Starting##Solver", &context->enableWarmStarting );
		ImGui::Checkbox( "Continuous##Solver", &context->enableContinuous );

		if ( ImGui::Shortcut( ImGuiKey_R ) || ImGui::Button( "Restart" ) )
		{
			SelectSample( context, context->sampleIndex, true );
		}
	}

	if ( context->sample->HasSolverControls() && ImGui::CollapsingHeader( "Recording", ImGuiTreeNodeFlags_DefaultOpen ) )
	{
		ImGui::PushItemWidth( 9.0f * fontSize );
		ImGui::InputText( "File##Recording", context->recordingFile, sizeof( context->recordingFile ) );
		ImGui::PopItemWidth();

		if ( context->sample->m_recording == nullptr )
		{
			// Restart to a clean world then snapshot it at step 0, a whole session capture.
			if ( ImGui::Button( "Record (restart)##Recording" ) )
			{
				SelectSample( context, context->sampleIndex, true );
				context->sample->StartRecording();
			}

			// Snapshot the running world and log from here, the mid simulation case.
			if ( ImGui::Button( "Record Now##Recording" ) )
			{
				context->sample->StartRecording();
			}
		}
		else
		{
			if ( ImGui::Button( "Stop##Recording" ) )
			{
				context->sample->FinishRecording();
			}
			ImGui::TextColored( HexColor( b3_colorSeaGreen ), "recording (from step %d)", context->sample->m_recordStartStep );
		}
	}

	ImGui::End();
}

// Minimal in-world HUD shown only when the UI is hidden: sample identity at the
// top-left, frame time and step count at the bottom-left. Matches Box2D DrawHud.
static void DrawHud( SampleContext* context )
{
	const SampleEntry& entry = g_sampleEntries[context->sampleIndex];
	float fontSize = ImGui::GetFontSize();

	DrawScreenStringFormat( 5, (int)( 1.5f * fontSize ), MakeColor( b3_colorYellow ), "%s : %s", entry.Category, entry.Name );
	DrawScreenStringFormat( 5, context->camera.m_height - (int)( 1.5f * fontSize ), MakeColor( b3_colorSeaGreen ),
							"%.1f ms  step %d", 1000.0f * (float)sapp_frame_duration(), context->sample->m_stepCount );
}

void DrawUI( SampleContext* context )
{
	// Hidden shows only the minimal in-world HUD.
	if ( context->showUI == false )
	{
		DrawHud( context );
		return;
	}

	DrawMenuBar( context );
	DrawSamplePicker( context );
	DrawInfoPanel( context );

	// Extra top-level windows (the Replay viewer adds the Outline panel).
	context->sample->DrawSampleWindows();

	// Bottom diagnostics drawer. Sample controls live in the info panel.
	context->sample->DrawMetrics();
}

void Sample::ToggleThirdPerson()
{
	if ( m_camera->m_thirdPerson )
	{
		sapp_lock_mouse( false );
		m_camera->m_thirdPerson = false;
	}
	else
	{
		sapp_lock_mouse( true );
		m_camera->m_thirdPerson = true;
	}
}

float CastClosestCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t materialId, int triangleIndex,
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

void CharacterMover::Initialize( Sample* sample, b3Pos position )
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
			.point = b3OffsetPos( self->m_transform.p, planeResults[i].point ),
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

	float pogoRestLength = 3.0f * m_capsule.radius;
	float rayLength = pogoRestLength + m_capsule.radius;
	b3Pos rayOrigin = b3TransformWorldPoint( m_transform, m_capsule.center1 );
	b3Vec3 rayTranslation = -rayLength * b3Vec3_axisY;
	b3QueryFilter skipTeamFilter = { 1, ~2u };
	skipTeamFilter.name = "pogo";
	b3RayResult rayResult = b3World_CastRayClosest( worldId, rayOrigin, rayTranslation, skipTeamFilter );

	if ( rayResult.hit == false )
	{
		m_onGround = false;
		m_pogoVelocity = 0.0f;

		DrawLine( rayOrigin, b3OffsetPos( rayOrigin, rayTranslation ), MakeColor( b3_colorGray ) );
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
		DrawLine( rayOrigin, rayResult.point, MakeColor( b3_colorGreen ) );
	}

	b3Pos startPosition = m_transform.p;
	b3Pos target = m_transform.p + timeStep * m_velocity + timeStep * m_pogoVelocity * b3Vec3_axisY;

	// Want the mover to collide with allies
	b3QueryFilter moverFilter = { .categoryBits = 1, .maskBits = ~0u, .id = 1, .name = "mover_collide" };

	// The cast should ignore allies
	b3QueryFilter castFilter = { .categoryBits = 1, .maskBits = ~2u, .id = 1, .name = "mover_cast" };

	m_totalIterations = 0;
	float tolerance = 0.01f;

	for ( int iteration = 0; iteration < 5; ++iteration )
	{
		m_planeCount = 0;

		b3Capsule mover;
		mover.center1 = m_capsule.center1;
		mover.center2 = m_capsule.center2;
		mover.radius = m_capsule.radius;

		b3World_CollideMover( worldId, m_transform.p, &mover, moverFilter, PlaneResultFcn, this );

		b3Vec3 targetDelta = target - m_transform.p;
		b3PlaneSolverResult result = b3SolvePlanes( targetDelta, m_planes, m_planeCount );

		m_totalIterations += result.iterationCount;

		b3Vec3 delta = result.delta;

		float fraction = b3World_CastMover( worldId, m_transform.p, &mover, delta, castFilter, MoverFilterCallback, this );

		delta *= fraction;
		m_transform.p = m_transform.p + delta;

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

		b3Pos point = m_planeExtras[i].point;
		b3Vec3 normal = b3Neg( m_planes[i].plane.normal );

		float invMassA = 0.0f;
		float invMassB = b3Body_GetInverseMass( bodyId );
		b3Matrix3 invIB = b3Body_GetWorldInverseRotationalInertia( bodyId );

		b3Pos pB = b3Body_GetWorldCenterOfMass( bodyId );
		b3Vec3 rB = b3SubPos( point, pB );

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
		if ( IsKeyDown( KEY_W ) )
		{
			throttle.x += 1.0f;
		}

		if ( IsKeyDown( KEY_S ) )
		{
			throttle.x -= 1.0f;
		}

		if ( IsKeyDown( KEY_A ) )
		{
			throttle.y -= 1.0f;
		}

		if ( IsKeyDown( KEY_D ) )
		{
			throttle.y += 1.0f;
		}

		if ( IsKeyDown( KEY_SPACE ) && m_onGround == true )
		{
			m_velocity.y = m_jumpSpeed;
			m_onGround = false;
		}

		if ( m_onGround == true )
		{
			m_sprint = IsKeyDown( KEY_LEFT_SHIFT );
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

	b3Pos position = m_transform.p;

	// Follow the mover and latch the draw origin before drawing, so the overlays below demote
	// against the same eye the view renders from.
	if ( m_sample->m_camera->m_thirdPerson )
	{
		m_sample->m_camera->m_pivot = position;
		m_sample->m_camera->UpdateTransform();
	}

	SetDrawOrigin( m_sample->m_camera->DrawOrigin() );

	int count = m_planeCount;
	for ( int i = 0; i < count; ++i )
	{
		b3Plane plane = m_planes[i].plane;
		b3Pos p1 = position + ( plane.offset - m_capsule.radius ) * plane.normal;
		b3Pos p2 = p1 + 0.1f * plane.normal;
		DrawPoint( p1, 5.0f, MakeColor( b3_colorYellow ) );
		DrawLine( p1, p2, MakeColor( b3_colorYellow ) );
	}

	DrawSolidCapsule( m_transform, m_capsule, MakeColor( b3_colorBlue ) );
	DrawLine( position, position + m_velocity, MakeColor( b3_colorPurple ) );

	m_ignoreShapeIds = nullptr;
	m_ignoreCount = 0;
}
