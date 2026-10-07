// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "dynamic_tree.h"

#include "box3d/types.h"

static inline b3TreeView b3GetCompoundTreeView( const b3CompoundData* compound )
{
	b3TreeView view = {
		.nodes = (const b3TreeNode*)( (intptr_t)compound + compound->nodeOffset ),
		.proxies = NULL,
	};
	return view;
}

b3TOIOutput b3CompoundTimeOfImpact( const b3CompoundData* compound, b3Transform transform, const b3ShapeProxy* proxy,
									const b3Sweep* sweep, float maxFraction );

// Transforms a sweep for a compound child shape
b3Sweep b3MakeCompoundChildSweep( b3Transform compoundTransform, b3Transform childTransform );

int b3CollideMoverAndCompound( b3PlaneResult* planes, int capacity, const b3CompoundData* shape, const b3Capsule* mover );
