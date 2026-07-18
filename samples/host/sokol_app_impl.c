// Single-TU build of sokol_app + sokol_glue + sokol_time.
//
// sokol_gfx's implementation lives in src/gfx/sokol_impl.c. We only include
// its header here (without SOKOL_IMPL) so that sokol_glue can see the
// sg_environment / sg_swapchain declarations it translates into.
//
// Backend macro (SOKOL_D3D11 / SOKOL_METAL / SOKOL_GLCORE) comes from the
// `sokol` INTERFACE target. On Apple this file is compiled as Objective-C
// with ARC (see src/app/CMakeLists.txt) so sokol_app's Cocoa backend builds.
//
// SOKOL_NO_ENTRY: samples/main.cpp supplies its own main() so it can install
// MSVC CRT leak reporting and dump after sapp_run returns, past sokol's own
// teardown. Our main is console-subsystem, which also keeps stderr flowing to
// ctest for the smoke test.
//
// _POSIX_C_SOURCE: glibc gates clock_gettime / CLOCK_MONOTONIC behind a
// POSIX feature-test macro under -std=c17. Define before any system headers.
#if defined(__linux__) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif

#define SOKOL_APP_IMPL
#define SOKOL_GLUE_IMPL
#define SOKOL_LOG_IMPL
#define SOKOL_NO_ENTRY

#include "sokol_gfx.h"
#include "sokol_app.h"
#include "sokol_glue.h"
#include "sokol_log.h"
