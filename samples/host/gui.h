// Dear ImGui shell.
//
// Wraps sokol_imgui.h, runs the app's UI draw callback (the DrawUiFcn passed
// to InitUI) on top of the live scene, and drains the in-world text overlay.
// Used only by the renderer binary; the visual-test targets call
// RenderFrameOffscreen directly and never instantiate ImGui.
//
// Lifecycle from main.cpp's sapp callbacks:
//
//   OnInit:    InitRenderer(&env); InitUI(&env, DrawUI);
//   OnEvent:   if (HandleEvent(e)) return;     // skip camera if ImGui ate it
//   OnFrame:   ResetFrameArena();
//              scene draws...
//              RenderFrame(&sc, &frame);
//              StartUIFrame(dt);              // runs the UI draw callback, drains in-world text
//              RenderUI(&sc);                 // separate pass on the swapchain
//   OnCleanup: ShutdownUI(); ShutdownRenderer();
//
// RenderUI opens its own swapchain pass with LOAD action so ImGui sits on
// top of the tonemapped scene without disturbing it. Single-threaded;
// render-thread only.

#pragma once

#include "sokol_app.h"
#include "sokol_gfx.h"

// Screen edge a panel anchors to. Panels sharing a side stack top-down in
// the order they call guiBeginPanel within a frame.
typedef enum GuiPanelAnchor
{
	GUI_ANCHOR_LEFT = 0,
	GUI_ANCHOR_RIGHT = 1,
} GuiPanelAnchor;

typedef void DrawUiFcn( void );

void InitUI( const sg_environment* env, DrawUiFcn* drawGuiFcn );
void ShutdownUI( void );

bool HandleEvent( const sapp_event* e );

void StartUIFrame( float dtSec );
void RenderUI( const sg_swapchain* swapchain );

// Begin an edge-anchored panel. A thin wrapper over ImGui::Begin. Pins the
// window flush against anchor's screen edge, stacked below any panel
// already opened on that side this frame, and re-anchors it there every
// frame so it tracks the edge when the window resizes (unlike a one-shot
// ImGuiCond_FirstUseEver position, which drifts off a shrinking window).
// width is in unscaled pixels. The UI DPI scale is applied internally.
// Height auto-fits the content, capped to the remaining viewport height
// (the panel gains a scrollbar past the cap). Returns ImGui::Begin's value;
// pair every call with EndPanel, including when it returns false.
bool BeginPanel( const char* name, GuiPanelAnchor anchor, float width );
void EndPanel( void );
