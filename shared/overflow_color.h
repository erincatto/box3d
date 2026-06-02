// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "box3d/id.h"

#ifdef __cplusplus
extern "C"
{
#endif

// One heavy dynamic hub surrounded by enough dynamic neighbors that the hub's
// degree in the dyn-dyn contact graph exceeds B3_DYNAMIC_COLOR_COUNT (= 20).
// The excess contacts land in the overflow color (B3_GRAPH_COLOR_COUNT - 1),
// exercising the b3*_Overflow solver path that no other scene reaches.
typedef struct OverflowColorPileData
{
	b3ShapeId groundShapeId;
	b3BodyId hubId;
	int neighborCount;
} OverflowColorPileData;

OverflowColorPileData CreateOverflowColorPile( b3WorldId worldId );

#ifdef __cplusplus
}
#endif
