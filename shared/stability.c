// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "stability.h"

#include "utils.h"

#include "box3d/box3d.h"

#include <assert.h>
#include <stdlib.h>

#define MESH_DROP_GRID_COUNT 32

MeshDropData CreateMeshDrop( b3WorldId worldId, b3Pos origin )
{
	MeshDropData data = { 0 };
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = origin;
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

		int gridCount = 40;
		float cellWidth = 1.0f;
		float rowHz = 0.1f;
		float columnHz = 0.2f;
		float groundAmplitude = 0.5f;

		data.mesh = b3CreateWaveMesh( gridCount, gridCount, cellWidth, groundAmplitude, rowHz, columnHz );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.filter.categoryBits = 1;
		b3CreateMeshShape( groundId, &shapeDef, data.mesh, b3Vec3_one );
	}

	{
		b3BoxHull box = b3MakeBoxHull( 0.02f, 0.2f, 0.04f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.rollingResistance = 0.1f;

		// Don't allow shapes to collide with each other.
		shapeDef.filter.categoryBits = 2;
		shapeDef.filter.maskBits = 1;

		g_randomSeed = 3963634789;

		int gridCount = MESH_DROP_GRID_COUNT;

		for ( int i = 0; i < gridCount; ++i )
		{
			for ( int j = 0; j < gridCount; ++j )
			{
				b3Vec3 linearVelocity = RandomVec3Uniform( -1.0f, 1.0f );
				b3Vec3 angularVelocity = RandomVec3Uniform( -5.0f, 5.0f );

				bodyDef.position =
					b3OffsetPos( origin, (b3Vec3){ 0.5f * ( i - 0.5f * gridCount ), 5.0f, 0.5f * ( j - 0.5f * gridCount ) } );
				bodyDef.linearVelocity = linearVelocity;
				bodyDef.angularVelocity = angularVelocity;
				b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}
		}
	}

	return data;
}

void DestroyMeshDrop( MeshDropData* data )
{
	b3DestroyMesh( data->mesh );
}
