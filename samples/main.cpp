// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

// Box3D samples host: a sokol_app shell driving render3d's renderer through
// the SampleManager. The four sokol_app callbacks own the window and input;
// everything below the entry points (InitRenderer, RenderFrame, the Draw* API,
// the b3DebugDraw adapter) is host-agnostic. See render3d's HOST_INTEGRATION.md
// for the contract this fills in.
//
//   OnInit:    InitRenderer -> InitUI -> InitAdapter -> SampleManager::Startup
//   OnEvent:   Esc quits; ImGui gate; else feed camera + dispatch to the sample
//   OnFrame:   ResetFrameArena -> Step -> Draw -> RenderFrame -> UI -> commit
//   OnCleanup: SampleManager::Shutdown -> ShutdownUI -> ShutdownRenderer
//
// --frames N runs N frames then exits with status = sokol validation-error
// count, the automated regression net for the port.

#include "sample.h"

#include "gfx/debug_adapter.h"
#include "gfx/keycodes.h"
#include "gfx/renderer.h"

#include "host/gui.h"

#include "box3d/math_functions.h"

#include "sokol_app.h"
#include "sokol_glue.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static SampleManager s_manager;
static int s_frame = 0;
static int s_frameLimit = -1;
static int s_sampleOverride = -1;
static bool s_shadowsOn = true;
static bool s_gtaoOn = true;
static int s_debugView = 0;

// Single host UI callback fired from inside StartUIFrame: the tools panel plus
// the active sample's own panel, both drawn by the manager.
static void DrawUI( void )
{
	s_manager.UpdateUI();
}

static void OnInit( void )
{
	const sg_environment env = sglue_environment();
	InitRenderer( &env );
	InitUI( &env, DrawUI, true );
	InitAdapter();

	constexpr float DEG = 3.14159265358979323846f / 180.0f;
	Camera& camera = s_manager.m_context.camera;
	camera.SetFov( 50.0f * DEG );
	camera.SetClip( 0.1f, 1000.0f );

	s_manager.Startup( sapp_width(), sapp_height() );

	// --sample N selects a registered sample by sorted index, overriding the
	// persisted one. Lets a headless --frames run target a specific sample.
	if ( s_sampleOverride >= 0 && s_sampleOverride < SampleManager::sEntryCount )
	{
		s_manager.m_context.sampleIndex = s_sampleOverride;
		s_manager.m_context.restart = false;
		s_manager.CreateSample();
	}
}

static void OnEvent( const sapp_event* e )
{
	// Esc quits even with ImGui focus so a text field can't trap the app.
	if ( e->type == SAPP_EVENTTYPE_KEY_DOWN && e->key_code == SAPP_KEYCODE_ESCAPE )
	{
		sapp_request_quit();
		return;
	}

	if ( HandleEvent( e ) )
	{
		return;
	}

	Camera& camera = s_manager.m_context.camera;
	camera.OnEvent( e );

	// Keep keyboard mods only. sokol packs the held mouse button into modifiers
	// (SAPP_MODIFIER_LMB == 0x100), which would defeat the sample's modifiers == 0 checks.
	const int mods = e->modifiers & ( SAPP_MODIFIER_SHIFT | SAPP_MODIFIER_CTRL | SAPP_MODIFIER_ALT | SAPP_MODIFIER_SUPER );

	switch ( e->type )
	{
		case SAPP_EVENTTYPE_KEY_DOWN:
			SetKeyDown( e->key_code, true );
			if ( e->key_repeat == false )
			{
				s_manager.Keyboard( e->key_code, ACTION_PRESS, mods );
			}
			break;

		case SAPP_EVENTTYPE_KEY_UP:
			SetKeyDown( e->key_code, false );
			s_manager.Keyboard( e->key_code, ACTION_RELEASE, mods );
			break;

		case SAPP_EVENTTYPE_MOUSE_DOWN:
			s_manager.m_context.mouseX = e->mouse_x;
			s_manager.m_context.mouseY = e->mouse_y;
			s_manager.MouseDown( { e->mouse_x, e->mouse_y }, e->mouse_button, mods );
			break;

		case SAPP_EVENTTYPE_MOUSE_UP:
			s_manager.MouseUp( { e->mouse_x, e->mouse_y }, e->mouse_button );
			break;

		case SAPP_EVENTTYPE_MOUSE_MOVE:
			s_manager.m_context.mouseX = e->mouse_x;
			s_manager.m_context.mouseY = e->mouse_y;
			s_manager.m_context.mouseDX = e->mouse_dx;
			s_manager.m_context.mouseDY = e->mouse_dy;
			s_manager.MouseMove( { e->mouse_x, e->mouse_y } );
			break;

		case SAPP_EVENTTYPE_RESIZED:
			s_manager.Resize( sapp_width(), sapp_height() );
			break;

		default:
			break;
	}
}

static void OnFrame( void )
{
	if ( s_frameLimit >= 0 && s_frame >= s_frameLimit )
	{
		sapp_quit();
		return;
	}

	const float dt = (float)sapp_frame_duration();
	const int W = sapp_width();
	const int H = sapp_height();

	Camera& camera = s_manager.m_context.camera;
	camera.Update( dt, W, H );

	ResetFrameArena();

	// Physics + the sample's debug-draw submission. Step advances the world and
	// queues the HUD text; Draw fills the instance and overlay arenas via the
	// b3DebugDraw adapter and the sample's own Draw* calls.
	s_manager.Step();
	s_manager.Draw();

	FrameInput fi{};
	fi.view = camera.View();
	fi.viewInv = camera.ViewInverse();
	fi.proj = camera.Proj();
	fi.projInv = camera.ProjInverse();
	fi.cameraPosition = camera.Position();
	fi.time = (float)sapp_frame_count() / 60.0f;
	fi.debugMode = s_debugView;
	fi.disableShadows = !s_shadowsOn;
	fi.disableAmbientOcclusion = !s_gtaoOn;

	const sg_swapchain sc = sglue_swapchain();
	RenderFrame( &sc, &fi );

	// StartUIFrame runs after RenderFrame: it drains the text arena with the
	// camera state RenderFrame just latched and runs the UI draw callback.
	GuiToggles tg{};
	tg.shadowsOn = &s_shadowsOn;
	tg.gtaoOn = &s_gtaoOn;
	tg.debugViewMode = &s_debugView;
	StartUIFrame( dt, tg );

	RenderUI( &sc );
	sg_commit();
	++s_frame;
}

static void OnCleanup( void )
{
	s_manager.Shutdown();
	ShutdownUI();
	ShutdownRenderer();

	const int errors = GetRenderErrorCount();
	fprintf( stderr, "samples: %d frames, %d sokol errors\n", s_frame, errors );
	exit( errors == 0 ? 0 : 1 );
}

sapp_desc sokol_main( int argc, char** argv )
{
	for ( int i = 1; i < argc; ++i )
	{
		if ( strcmp( argv[i], "--frames" ) == 0 && i + 1 < argc )
		{
			s_frameLimit = atoi( argv[++i] );
		}
		else if ( strcmp( argv[i], "--sample" ) == 0 && i + 1 < argc )
		{
			s_sampleOverride = atoi( argv[++i] );
		}
	}

	sapp_desc desc{};
	desc.init_cb = OnInit;
	desc.frame_cb = OnFrame;
	desc.event_cb = OnEvent;
	desc.cleanup_cb = OnCleanup;

	// GL 4.5 for glClipControl (reverse-Z). Ignored on D3D11 / Metal.
	desc.gl.major_version = 4;
	desc.gl.minor_version = 5;

	desc.width = 1920;
	desc.height = 1080;

	// No swap-chain MSAA. The renderer runs MSAA in its own scene target.
	desc.sample_count = 1;

	desc.window_title = "Box3D Samples";
	desc.swap_interval = 1;
	desc.high_dpi = true;

	return desc;
}
