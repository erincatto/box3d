// Box3D samples host: sokol_app shell driving render3d's renderer.
//
// Scaffolding milestone. This is the minimal host that proves the ported
// pipeline links and runs: it opens a sokol_app window, drives an orbit
// camera, submits a fixed placeholder scene through the Draw* API, and
// renders it with the full renderer (sky + shadows + GTAO + tone map) plus
// an ImGui panel. No Sample / SampleManager retargeting yet; that lands in
// later phases. The four sokol_app callbacks mirror render3d's app/main.cpp.
//
// --frames N runs N frames then exits with a status equal to the sokol
// validation-error count, the automated regression net for the port.

#include "host/camera.h"
#include "host/gui.h"

#include "gfx/draw.h"
#include "gfx/renderer.h"
#include "gfx/text.h"

#include "box3d/math_functions.h"

#include "imgui.h"
#include "sokol_app.h"
#include "sokol_glue.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static Camera s_camera;
static int s_frame = 0;
static int s_frameLimit = -1;
static bool s_shadowsOn = true;
static bool s_gtaoOn = true;
static int s_debugView = 0;

static b3Transform TransformAt( float x, float y, float z )
{
	b3Transform t = b3Transform_identity;
	t.p = b3Vec3{ x, y, z };
	return t;
}

// Right "Scaffold" panel, registered with InitUI. Renders after the built-in
// left "Renderer" window. Placeholder content until the Box3D sample UI ports.
static void DrawUI( void )
{
	if ( !BeginPanel( "Scaffold", GUI_ANCHOR_RIGHT, 250.0f ) )
	{
		EndPanel();
		return;
	}
	ImGui::TextUnformatted( "render3d renderer on sokol_app" );
	ImGui::Separator();
	ImGui::Text( "frame %d", s_frame );
	ImGui::TextUnformatted( "Sample host scaffold" );
	EndPanel();
}

static void OnInit( void )
{
	const sg_environment env = sglue_environment();
	InitRenderer( &env );
	InitUI( &env, DrawUI, true );

	constexpr float DEG = 3.14159265358979323846f / 180.0f;
	s_camera.SetFov( 50.0f * DEG );
	s_camera.SetClip( 0.1f, 1000.0f );
	s_camera.SetTarget( b3Vec3{ 0.0f, 0.6f, 0.0f } );
	s_camera.SetOrbit( 35.0f * DEG, 18.0f * DEG, 7.0f );
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
	s_camera.OnEvent( e );
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
	s_camera.Update( dt, W, H );

	FrameInput fi{};
	fi.view = s_camera.View();
	fi.viewInv = s_camera.ViewInverse();
	fi.proj = s_camera.Proj();
	fi.projInv = s_camera.ProjInverse();
	fi.cameraPosition = s_camera.Position();
	fi.time = (float)sapp_frame_count() / 60.0f;
	fi.debugMode = s_debugView;
	fi.disableShadows = !s_shadowsOn;
	fi.disableAmbientOcclusion = !s_gtaoOn;

	ResetFrameArena();

	// Placeholder scene: ground plate, one of each impostor shape, an axis
	// triad, and a ground grid. Exercises the cube / sphere / capsule
	// pipelines plus the overlay path.
	DrawCube( TransformAt( 0.0f, -0.25f, 0.0f ), b3Vec3{ 20.0f, 0.5f, 20.0f }, MakeVec4( 0.40f, 0.42f, 0.45f, 1.0f ) );
	DrawCube( TransformAt( -2.0f, 0.5f, 0.0f ), b3Vec3{ 1.0f, 1.0f, 1.0f }, MakeVec4( 0.80f, 0.25f, 0.20f, 1.0f ) );
	DrawSphere( TransformAt( 0.0f, 0.6f, 0.0f ), 0.6f, MakeVec4( 0.25f, 0.55f, 0.80f, 1.0f ) );
	DrawCapsule( TransformAt( 2.0f, 0.5f, 0.0f ), 0.5f, 0.4f, MakeVec4( 0.30f, 0.70f, 0.35f, 1.0f ) );
	DrawAxes( b3Transform_identity, 1.0f );
	DrawGrid( b3Vec3{ 0.0f, 0.0f, 0.0f }, b3Vec3{ 0.0f, 1.0f, 0.0f }, 10.0f, 20, MakeVec4( 0.5f, 0.5f, 0.5f, 0.25f ) );
	DrawScreenStringFormat( 10, 10, MakeVec4( 0.9f, 0.9f, 0.9f, 1.0f ), "Box3D sokol scaffold  frame %d", s_frame );

	const sg_swapchain sc = sglue_swapchain();
	RenderFrame( &sc, &fi );

	// StartUIFrame runs after RenderFrame: it drains the text arena with the
	// camera state RenderFrame just latched and opens the ImGui frame.
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
	ShutdownUI();
	ShutdownRenderer();

	const int errors = GetRenderErrorCount();
	fprintf( stderr, "scaffold: %d frames, %d sokol errors\n", s_frame, errors );
	// sokol_app's own main returns 0; bypass it so a --frames run surfaces
	// the real validation-error count to the caller.
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
	}

	sapp_desc desc{};
	desc.init_cb = OnInit;
	desc.frame_cb = OnFrame;
	desc.event_cb = OnEvent;
	desc.cleanup_cb = OnCleanup;

	// GL 4.5 for glClipControl (reverse-Z). Ignored on D3D11 / Metal.
	desc.gl.major_version = 4;
	desc.gl.minor_version = 5;

	desc.width = 1280;
	desc.height = 720;

	// No swapchain MSAA; the renderer runs MSAA in its own scene target.
	desc.sample_count = 1;

	desc.window_title = "Box3D samples (sokol scaffold)";
	desc.swap_interval = 1;
	desc.high_dpi = true;

	return desc;
}
