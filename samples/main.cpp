// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/debug_adapter.h"
#include "gfx/keycodes.h"
#include "gfx/renderer.h"
#include "host/gui.h"
#include "sample.h"
#include "sokol_app.h"
#include "sokol_glue.h"

#include "box3d/box3d.h"
#include "box3d/math_functions.h"
#include "gfx/draw.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>

#if defined( _WIN32 )
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
// clang-format off
#include <windows.h>
#include <timeapi.h> // timeBeginPeriod for fine sleep granularity
// clang-format on
#pragma comment( lib, "winmm.lib" ) // MSVC auto-link
#endif

static SampleContext s_context;
static int s_frame = 0;
static int s_frameLimit = -1;
static int s_sampleOverride = -1;

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

static void SortSamples()
{
	qsort( g_sampleEntries, g_sampleCount, sizeof( SampleEntry ), CompareSamples );
}

// Single host UI callback fired from inside StartUIFrame: menu bar, panels, and
// the active sample's drawer.
static void OnDrawUI( void )
{
	DrawUI( &s_context );
}

static void OnInit( void )
{
#if defined( _WIN32 )
	timeBeginPeriod( 1 );
#endif

	const sg_environment env = sglue_environment();
	InitRenderer( &env );
	InitUI( &env, OnDrawUI );
	InitAdapter();

	constexpr float DEG = 3.14159265358979323846f / 180.0f;
	s_context.camera.SetFov( 50.0f * DEG );
	s_context.camera.SetClip( 0.1f, Camera::kViewDistance );

	s_context.Load();

	int cores = (int)std::thread::hardware_concurrency();
	s_context.workerCount = b3ClampInt( cores / 2, 1, 8 );
	s_context.windowWidth = sapp_width();
	s_context.windowHeight = sapp_height();

	SortSamples();

	// --sample N selects a registered sample by sorted index, overriding the
	// persisted one. Lets a headless --frames run target a specific sample.
	int index = s_context.sampleIndex;
	if ( s_sampleOverride >= 0 && s_sampleOverride < g_sampleCount )
	{
		index = s_sampleOverride;
	}

	SelectSample( &s_context, index, false );
}

static void OnEvent( const sapp_event* e )
{
	// Esc quits even with ImGui focus so a text field can't trap the app.
	if ( e->type == SAPP_EVENTTYPE_KEY_DOWN && e->key_code == SAPP_KEYCODE_ESCAPE )
	{
		sapp_request_quit();
		return;
	}

	bool uiCaptured = false;
	if (s_context.camera.m_thirdPerson == false)
	{
		uiCaptured = HandleEvent( e );
	}

	// The camera must always see button releases and focus loss, even when the UI
	// captures the event. Otherwise a release over an ImGui panel never clears the
	// drag flag and the camera keeps orbiting.
	const bool releaseOrUnfocus = e->type == SAPP_EVENTTYPE_MOUSE_UP || e->type == SAPP_EVENTTYPE_UNFOCUSED;
	if ( uiCaptured == false || releaseOrUnfocus )
	{
		s_context.camera.OnEvent( e );
	}

	if ( uiCaptured )
	{
		return;
	}

	// Keep keyboard mods only. sokol packs the held mouse button into modifiers
	// (SAPP_MODIFIER_LMB == 0x100), which would defeat the sample's modifiers == 0 checks.
	const int mods = e->modifiers & ( SAPP_MODIFIER_SHIFT | SAPP_MODIFIER_CTRL | SAPP_MODIFIER_ALT | SAPP_MODIFIER_SUPER );

	switch ( e->type )
	{
		case SAPP_EVENTTYPE_KEY_DOWN:
			SetKeyDown( e->key_code, true );

			// Global shortcuts on first press; repeats are ignored. The sokol
			// analog of Box2D's KeyCallback.
			if ( e->key_repeat == false )
			{
				switch ( e->key_code )
				{
					case KEY_TAB:
						s_context.showUI = !s_context.showUI;
						break;

					case KEY_O:
						if ( mods & MOD_CTRL )
						{
							// Ctrl+O opens the fuzzy picker. Force the UI visible so it shows.
							s_context.showUI = true;
							s_context.openSamplePicker = true;
						}
						else
						{
							s_context.singleStep += ( mods & MOD_SHIFT ) ? 5 : 1;
						}
						break;

					case KEY_P:
						s_context.pause = !s_context.pause;
						break;

					case KEY_M:
						s_context.showMetrics = !s_context.showMetrics;
						break;

					case KEY_R:
						SelectSample( &s_context, s_context.sampleIndex, true );
						break;

					case KEY_LEFT_BRACKET:
						SelectSample( &s_context, b3MaxInt( 0, s_context.sampleIndex - 1 ), false );
						break;

					case KEY_RIGHT_BRACKET:
						SelectSample( &s_context, b3MinInt( g_sampleCount - 1, s_context.sampleIndex + 1 ), false );
						break;

					case KEY_F:
					case KEY_HOME:
					{
						// Frame the selection, or let the sample frame its whole scene when nothing is
						// selected. A non-body selection such as a recorded query supplies its own bounds and
						// takes priority over the hovered body. The replay viewer's scene lives in a
						// player-owned world, not the base world, so the whole-scene case routes through
						// FocusHome.
						Camera& cam = s_context.camera;
						float aspect = cam.m_height > 0 ? (float)cam.m_width / (float)cam.m_height : 1.0f;
						b3AABB bounds;
						if ( s_context.sample->FocusBounds( &bounds ) )
						{
							cam.Frame( bounds, aspect, 1.5f );
						}
						else
						{
							b3BodyId bodyId = s_context.sample->FocusBody();
							if ( B3_IS_NON_NULL( bodyId ) )
							{
								cam.Frame( b3Body_ComputeAABB( bodyId ), aspect, 1.5f );
							}
							else
							{
								s_context.sample->FocusHome();
							}
						}
					}
					break;

					default:
						s_context.sample->Keyboard( e->key_code, ACTION_PRESS, mods );
						break;
				}
			}
			break;

		case SAPP_EVENTTYPE_KEY_UP:
			SetKeyDown( e->key_code, false );
			break;

		case SAPP_EVENTTYPE_MOUSE_DOWN:
			s_context.mouseX = e->mouse_x;
			s_context.mouseY = e->mouse_y;
			s_context.sample->MouseDown( { e->mouse_x, e->mouse_y }, e->mouse_button, mods );
			break;

		case SAPP_EVENTTYPE_MOUSE_UP:
			s_context.sample->MouseUp( { e->mouse_x, e->mouse_y }, e->mouse_button );
			break;

		case SAPP_EVENTTYPE_MOUSE_MOVE:
			s_context.mouseX = e->mouse_x;
			s_context.mouseY = e->mouse_y;
			s_context.mouseDX = e->mouse_dx;
			s_context.mouseDY = e->mouse_dy;
			s_context.sample->MouseMove( { e->mouse_x, e->mouse_y } );
			break;

		case SAPP_EVENTTYPE_RESIZED:
		{
			int w = sapp_width();
			int h = sapp_height();
			s_context.minimized = ( w == 0 || h == 0 );
			if ( s_context.minimized == false )
			{
				s_context.windowWidth = w;
				s_context.windowHeight = h;
			}
		}
		break;

		default:
			break;
	}
}

// Pace the loop to 60 Hz so the fixed 1/60 physics step plays at real time on any
// display. Sleep the bulk of the idle time, then spin the last bit since sleep wakes
// are only accurate to about a millisecond.
static void LimitFrameRate( uint64_t frameStart )
{
	const float targetMs = 1000.0f / 60.0f;
	const float spinMs = 2.0f;

	int sleepMs = (int)( targetMs - spinMs - b3GetMilliseconds( frameStart ) );
	if ( sleepMs > 0 )
	{
		b3Sleep( sleepMs );
	}

	while ( b3GetMilliseconds( frameStart ) < targetMs )
	{
		b3Yield();
	}
}

static void OnFrame( void )
{
	if ( s_frameLimit >= 0 && s_frame >= s_frameLimit )
	{
		sapp_quit();
		return;
	}

	const uint64_t frameStart = b3GetTicks();

	// Nothing to draw while minimized. sapp reports a 0x0 framebuffer then, which
	// would drive the swapchain and every render target to zero size. Pace the
	// loop so it doesn't spin and bail.
	if ( s_context.minimized )
	{
		if ( s_frameLimit < 0 )
		{
			LimitFrameRate( frameStart );
		}
		return;
	}

	const float dt = (float)sapp_frame_duration();
	const int W = sapp_width();
	const int H = sapp_height();

	Camera& camera = s_context.camera;
	camera.Update( dt, W, H );

	// Push the current length scale and Z-up choice. The replay player sets length
	// units from its header on load and restores them on close, so querying every
	// frame tracks load/unload without extra wiring. Live samples sit at 1 unit per
	// meter, leaving the transform identity.
	camera.SetRenderTransform( b3GetLengthUnitsPerMeter(), s_context.viewZUp );

	// Sync the draw origin to the camera eye once per frame, before any drawing. This must hold even
	// for samples that drive their own Step without calling Sample::Step. Render re-syncs after Step
	// because a third person follow moves the eye while stepping. The draw origin is in simulation
	// space; the view folds in the scale and up axis.
	SetDrawOrigin( camera.DrawOrigin() );

	ResetFrameArena();

	// Apply the per-frame draw state the UI owns, then advance the sample. Step
	// queues the HUD text; Render fills the instance and overlay arenas via the
	// b3DebugDraw adapter and the sample's own Draw* calls.
	SetTransparentDynamic( s_context.transparentDynamic );
	s_context.sample->ResetText();

	// Pause banner only with the UI up, matching Box2D.
	if ( s_context.pause && s_context.showUI )
	{
		s_context.sample->DrawTextLine( "****PAUSED****" );
		s_context.sample->DrawTextLine( "" );
	}

	s_context.sample->Step();
	s_context.sample->Render();

	FrameInput fi{};
	fi.view = camera.View();
	fi.viewInv = camera.ViewInverse();
	fi.proj = camera.Proj();
	fi.projInv = camera.ProjInverse();
	fi.cameraPosition = camera.Position();
	// Read back the origin Render actually shifted to. A follow-cam sample can
	// move it during Step/Render, so reuse the live value rather than the eye.
	b3Pos drawOrigin = GetDrawOrigin();
	fi.drawOrigin = b3ToVec3( drawOrigin );
	// Wrap the origin to the grid period in double, before it narrows to float.
	// A float can't resolve a 1 m cell at 1e7 m, so feeding the raw origin to
	// the grid would shatter the lines. The pattern repeats every 10 cells, so
	// the wrapped offset draws identical lines at any distance.
	double gridPeriod = 10.0 * BOX3D_GROUND_GRID_CELL_SIZE;
	fi.gridWrap.x = (float)fmod( (double)drawOrigin.x, gridPeriod );
	fi.gridWrap.y = (float)fmod( (double)drawOrigin.z, gridPeriod );
	fi.time = (float)sapp_frame_count() / 60.0f;
	fi.debugMode = s_context.debugView;
	fi.disableShadows = !s_context.enableShadows;
	fi.disableAmbientOcclusion = !s_context.enableGtao;
	fi.zUp = s_context.viewZUp;

	const sg_swapchain sc = sglue_swapchain();
	RenderFrame( &sc, &fi );

	// StartUIFrame runs after RenderFrame: it drains the text arena with the
	// camera state RenderFrame just latched and runs the UI draw callback.
	StartUIFrame( dt );

	RenderUI( &sc );
	sg_commit();
	++s_frame;

	if ( s_frameLimit < 0 )
	{
		LimitFrameRate( frameStart );
	}
}

static void OnCleanup( void )
{
	// Destroy the sample first because it will destroy debug shapes.
	delete s_context.sample;
	s_context.sample = nullptr;
	s_context.Save();

	ShutdownUI();
	ShutdownRenderer();

	const int errors = GetRenderErrorCount();
	fprintf( stderr, "samples: %d frames, %d sokol errors\n", s_frame, errors );

#if defined( _WIN32 )
	timeEndPeriod( 1 );
#endif

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

	// Static so the pointer outlives sokol_main; sokol keeps it for the window lifetime.
	b3Version version = b3GetVersion();
	static char title[64];
	snprintf( title, sizeof( title ), "Box3D %d.%d.%d - %s precision", version.major, version.minor,
			  version.revision, b3IsDoublePrecision() ? "double" : "single" );
	desc.window_title = title;

	// Vsync off: the software limiter in OnFrame owns the cadence. A hard 60 Hz
	// cap under vsync would beat against a non-60 display and pace to the wrong rate.
	desc.swap_interval = 0;
	desc.high_dpi = true;

	return desc;
}
