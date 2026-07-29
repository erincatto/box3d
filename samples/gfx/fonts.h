// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// The embedded font, shared by the two things that rasterize text.
//
// Both consumers want the same bytes: world_text.c bakes its glyph atlas from
// them, and the GUI shell hands them to ImGui. Keeping the byte array behind
// this one translation unit means the font exists once in the binary, and the
// panels and the world labels are held to the same face.
//
// Pure C so world_text.c can reach it. See data/fonts/README.md for where the
// face comes from and how to regenerate it.

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

// TTF bytes, valid for the life of the program. Covers printable ASCII, which
// is all the samples emit. Anything outside that draws as '?'.
const unsigned char* GetFontBytes( int* outSize );

// Short name for the ImGui atlas and logs.
const char* GetFontName( void );

#ifdef __cplusplus
} // extern "C"
#endif
