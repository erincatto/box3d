// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

// Renamed thunks over gfx/draw.h and gfx/text.h.
//
// gfx's draw functions have C linkage, so a C++ translation unit can't both
// see them and define same-named overloads. draw_bridge.cpp is the one place
// that includes gfx/draw.h; everything else reaches gfx through these names.

#pragma once

#include "gfx/utility.h"

#include "box3d/math_functions.h"

void BridgeSphere( b3Transform transform, float radius, Vec4 color );
void BridgeCapsule( b3Transform transform, float halfLength, float radius, Vec4 color );
void BridgeCube( b3Transform transform, b3Vec3 scale, Vec4 color );
void BridgePoint( b3Vec3 point, float sizePx, Vec4 color );
void BridgeLine( b3Vec3 p1, b3Vec3 p2, Vec4 color );
void BridgeArrow( b3Vec3 p1, b3Vec3 p2, Vec4 color );
void BridgeCross( b3Vec3 center, float size, Vec4 color );
void BridgeAabb( b3Vec3 lower, b3Vec3 upper, Vec4 color );
void BridgeAxes( b3Transform transform, float size );
void BridgeGrid( b3Vec3 center, b3Vec3 normal, float halfExtent, int divisions, Vec4 color );
void BridgeTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, Vec4 color );
void BridgeString( b3Vec3 worldPos, Vec4 color, const char* text );
