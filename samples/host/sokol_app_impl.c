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
// SOKOL_WIN32_FORCE_MAIN: sokol_app defaults to WinMain (subsystem:windows)
// on Windows. Our binaries are console-subsystem so the smoke test can
// surface stderr to ctest, so force the int main variant.
//
// _POSIX_C_SOURCE: glibc gates clock_gettime / CLOCK_MONOTONIC behind a
// POSIX feature-test macro under -std=c17. Define before any system headers.
#if defined(__linux__) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif

#define SOKOL_APP_IMPL
#define SOKOL_GLUE_IMPL
#define SOKOL_LOG_IMPL
#define SOKOL_WIN32_FORCE_MAIN

#include "sokol_gfx.h"
#include "sokol_app.h"
#include "sokol_glue.h"
#include "sokol_log.h"
