// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/types.h"
#include "gfx/utility.h"

#ifdef __cplusplus
extern "C"
{
#endif

// World-space minor-cell size of the procedural ground grid (meters). Major
// lines fire every 10 cells inside the shader. Shared so the host can wrap the
// draw origin to the grid period and keep the grid crisp far from the origin.
#define BOX3D_GROUND_GRID_CELL_SIZE 1.0f

void InitAdapter( void );

// Unconditionally flush the debug-shape pool: release the GPU mesh references
// held by live hull/mesh/heightfield shapes and rebuild the free list. Call
// before b3DestroyWorld (or on scene switch). Defensive even when
// destroyDebugShape already fired for every shape. After this,
// GetDebugShapeCount() == 0 and the next world allocates from the pool bottom.
void ResetAdapterPool( void );

void AttachToWorldDef( b3WorldDef* def );
void MakeDebugDraw( b3DebugDraw* out );
b3DebugDraw* GetGuiDraw( void );
void ApplyGuiFlags( b3DebugDraw* out );
int GetDebugShapeCount( void );

// Tag a Box3D shape so the renderer draws it with the procedural ground grid.
void SetGroundShape( b3ShapeId shapeId );
void SetShapeMaterial( b3ShapeId shapeId, Vec4 color, float metallic, float roughness );
void SetTransparentDynamic( bool enabled );
bool GetTransparentDynamic( void );

void SetSelectedBody( b3BodyId bodyId );
void SetSelectedShape( b3ShapeId shapeId );
void ClearSelection( void );
b3BodyId GetSelectedBody( void );
bool IsBodySelected( b3BodyId bodyId );

#ifdef __cplusplus
} // extern "C"
#endif
