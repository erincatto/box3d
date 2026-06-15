// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "overflow_color.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <math.h>

#define OVERFLOW_PILE_RING_COUNT 5
#define OVERFLOW_PILE_PER_RING 5

OverflowColorPileData CreateOverflowColorPile( b3WorldId worldId )
{
	OverflowColorPileData data = { 0 };
	data.neighborCount = OVERFLOW_PILE_RING_COUNT * OVERFLOW_PILE_PER_RING;

	// Static ground (top surface at y = 0)
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		data.groundShapeId = b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	// Tall, heavy hub. Tall so neighbors can ring around it in multiple
	// vertical layers; heavy so it stays roughly still under uneven inward
	// pressure from the ring.
	float hubHalfX = 0.5f;
	float hubHalfY = 2.5f;
	float hubHalfZ = 0.5f;
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, hubHalfY, 0.0f };
		data.hubId = b3CreateBody( worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( hubHalfX, hubHalfY, hubHalfZ );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 50.0f;
		b3CreateHullShape( data.hubId, &shapeDef, &box.base );
	}

	// Neighbors: vertical rings around the hub, each box slightly overlapping
	// the hub so a contact exists on the very first step.
	float neighborHalf = 0.2f;
	float ringRadius = hubHalfX + neighborHalf - 0.03f;

	b3BoxHull neighborBox = b3MakeBoxHull( neighborHalf, neighborHalf, neighborHalf );
	b3ShapeDef neighborShape = b3DefaultShapeDef();

	float ringSpacing = 0.5f;
	float baseY = neighborHalf + 0.05f;

	for ( int ring = 0; ring < OVERFLOW_PILE_RING_COUNT; ++ring )
	{
		float y = baseY + ringSpacing * ring;

		// Offset alternate rings by half a slot so neighbors don't sit
		// directly above each other.
		float thetaOffset = ( ring & 1 ) ? ( B3_PI / OVERFLOW_PILE_PER_RING ) : 0.0f;

		for ( int slot = 0; slot < OVERFLOW_PILE_PER_RING; ++slot )
		{
			float theta = thetaOffset + ( 2.0f * B3_PI * slot ) / OVERFLOW_PILE_PER_RING;

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = (b3Pos){ ringRadius * cosf( theta ), y, ringRadius * sinf( theta ) };
			b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

			b3CreateHullShape( bodyId, &neighborShape, &neighborBox.base );
		}
	}

	return data;
}
