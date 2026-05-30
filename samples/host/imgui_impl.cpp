// Single translation unit for sokol_imgui.h's implementation.
//
// Built as C++ so it can call into the C++ Dear ImGui API directly (the C
// path needs cimgui, which we don't pull in). Compiled as part of the
// renderer binary; not linked into tests, which never instantiate the GUI.
//
// The sokol_app/glue/time *_IMPL macros live in src/app/sokol_app_impl.c and
// sokol_gfx's SOKOL_IMPL lives in src/gfx/sokol_impl.c - we only enable
// SOKOL_IMGUI_IMPL here.

#include "imgui.h"

#include "sokol_gfx.h"
#include "sokol_app.h"

#define SOKOL_IMGUI_IMPL
#include "sokol_imgui.h"
