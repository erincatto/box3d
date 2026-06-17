// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "determinism.h"

#include "human.h"

#include "box3d/box3d.h"

#include <assert.h>
#include <stdlib.h>

#define GRID_SIZE 15.0f

static void CreateGroup( FallingRagdollData* data, b3WorldId worldId, int rowIndex, int columnIndex )
{
	assert( rowIndex < RAGDOLL_GRID_COUNT && columnIndex < RAGDOLL_GRID_COUNT );

	int groupIndex = rowIndex * RAGDOLL_GRID_COUNT + columnIndex;

	float span = RAGDOLL_GRID_COUNT * GRID_SIZE;
	float groupDistance = 1.0f * span / RAGDOLL_GRID_COUNT;

	b3Pos position;
	position.x = -0.5f * span + groupDistance * ( columnIndex + 0.5f );
	position.y = 15.0f;
	position.z = -0.5f * span + groupDistance * ( rowIndex + 0.5f );

	float frictionTorque = 5.0f;
	float hertz = 1.0f;
	float dampingRatio = 0.7f;
	bool colorize = false;

	for ( int i = 0; i < RAGDOLL_GROUP_SIZE; ++i )
	{
		Human* human = data->groups[groupIndex].humans + i;
		CreateHuman( human, worldId, position, frictionTorque, hertz, dampingRatio, groupIndex, NULL, colorize );
		position.x += 0.75f;
	}
}

FallingRagdollData CreateFallingRagdolls( b3WorldId worldId )
{
	FallingRagdollData data = { 0 };

	int halfMeshGridRows = 4;
	float meshGridCellWidth = GRID_SIZE / ( 2.0f * halfMeshGridRows );
	data.gridMesh = b3CreateGridMesh( 2 * halfMeshGridRows, 2 * halfMeshGridRows, meshGridCellWidth, 0, true );
	data.torusMesh = b3CreateTorusMesh( 16, 16, 0.25f * GRID_SIZE, 1.0f );

	float span = GRID_SIZE * RAGDOLL_GRID_COUNT;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	bodyDef.position.x = -0.5f * span + 0.5f * GRID_SIZE;
	for ( int i = 0; i < RAGDOLL_GRID_COUNT; ++i )
	{
		bodyDef.position.z = -0.5f * span + 0.5f * GRID_SIZE;
		for ( int j = 0; j < RAGDOLL_GRID_COUNT; ++j )
		{
			b3BodyId body = b3CreateBody( worldId, &bodyDef );
			b3CreateMeshShape( body, &shapeDef, data.gridMesh, b3Vec3_one );
			b3CreateMeshShape( body, &shapeDef, data.torusMesh, b3Vec3_one );

			CreateGroup( &data, worldId, i, j );

			bodyDef.position.z += GRID_SIZE;
		}

		bodyDef.position.x += GRID_SIZE;
	}

	return data;
}

bool UpdateFallingRagdolls( b3WorldId worldId, FallingRagdollData* data )
{
	if ( data->hash == 0 )
	{
		b3BodyEvents bodyEvents = b3World_GetBodyEvents( worldId );

		if ( bodyEvents.moveCount == 0 )
		{
			int awakeCount = b3World_GetAwakeBodyCount( worldId );
			assert( awakeCount == 0 );

			data->hash = B3_HASH_INIT;
			for ( int i = 0; i < RAGDOLL_GRID_COUNT; ++i )
			{
				for ( int j = 0; j < RAGDOLL_GRID_COUNT; ++j )
				{
					for ( int k = 0; k < RAGDOLL_GROUP_SIZE; ++k )
					{
						int groupIndex = i * RAGDOLL_GRID_COUNT + j;

						Human* human = data->groups[groupIndex].humans + k;

						for ( int b = 0; b < bone_count; ++b )
						{
							b3BodyId bodyId = human->bones[b].bodyId;
							b3WorldTransform xf = b3Body_GetTransform( bodyId );
							data->hash = b3Hash( data->hash, (uint8_t*)( &xf ), sizeof( b3WorldTransform ) );
						}
					}
				}
			}

			data->sleepStep = data->stepCount;
		}
	}

	data->stepCount += 1;

	return data->hash != 0;
}

void DestroyFallingRagdolls( FallingRagdollData* data )
{
	b3DestroyMesh( data->gridMesh );
	b3DestroyMesh( data->torusMesh );

	data->gridMesh = NULL;
	data->torusMesh = NULL;
}
