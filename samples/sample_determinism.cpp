// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "determinism.h"
#include "gfx/debug_adapter.h"
#include "gfx/draw.h"
#include "sample.h"
#include "stability.h"

#include "box3d/box3d.h"

#include <stdio.h>

class FallingRagdolls : public Sample
{
public:
	explicit FallingRagdolls( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 40.0f, b3Pos_zero );
		}

		m_data = CreateFallingRagdolls( m_worldId );
		m_done = false;
	}

	~FallingRagdolls() override
	{
		DestroyFallingRagdolls( &m_data );
	}

	void Step( ) override
	{
		Sample::Step( );

		if ( m_done == false )
		{
			m_done = UpdateFallingRagdolls( m_worldId, &m_data );
			if (m_done)
			{
				printf( "sleep step = %d, hash = 0x%08X\n", m_data.sleepStep, m_data.hash );
			}
		}
		else
		{
			DrawTextLine( "sleep step = %d, hash = 0x%08X", m_data.sleepStep, m_data.hash );

		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new FallingRagdolls( context );
	}

	FallingRagdollData m_data;
	bool m_done;
};

static int sampleFallingRagdolls = RegisterSample( "Determinism", "Falling Ragdolls", FallingRagdolls::Create );

class WavePile : public Sample
{
public:
	explicit WavePile( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 25.0f, 25.0f, b3Pos_zero );
		}

		m_data = CreateWavePile( m_worldId );
		m_done = false;
	}

	~WavePile() override
	{
		DestroyWavePile( &m_data );
	}

	void Step() override
	{
		Sample::Step();

		if ( m_done == false )
		{
			// Only advance the scenario on real steps, else pausing would skew the step count
			if ( m_didStep )
			{
				m_done = UpdateWavePile( m_worldId, &m_data );
				if ( m_done )
				{
					printf( "sleep step = %d, hash = 0x%08X\n", m_data.sleepStep, m_data.hash );
				}
			}
		}
		else
		{
			DrawTextLine( "sleep step = %d, hash = 0x%08X", m_data.sleepStep, m_data.hash );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new WavePile( context );
	}

	WavePileData m_data;
	bool m_done;
};

static int sampleWavePile = RegisterSample( "Determinism", "Wave Pile", WavePile::Create );

class QuerySpawn : public Sample
{
public:
	explicit QuerySpawn( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 25.0f, 30.0f, b3Pos_zero );
		}

		m_data = CreateQuerySpawn( m_worldId );
		m_frameCount = 0;
		m_done = false;
	}

	~QuerySpawn() override
	{
		DestroyQuerySpawn( &m_data );
	}

	void Step() override
	{
		Sample::Step();

		if ( m_done == false )
		{
			// Advance every 10th step so each query lingers on screen. This diverges from
			// the unit test cadence, so the hashes differ from the pinned constants.
			if ( m_didStep )
			{
				m_frameCount += 1;
				if ( m_frameCount % 10 == 1 )
				{
					m_done = UpdateQuerySpawn( m_worldId, &m_data );
					if ( m_done )
					{
						printf( "sleep step = %d, hash = 0x%08X, hits = %d, query hash = 0x%08X\n", m_data.sleepStep,
								m_data.hash, m_data.queryHitCount, m_data.queryHash );
					}
				}
			}

			DrawTextLine( "spawned = %d of %d, query hits = %d", m_data.spawnCount, QUERY_SPAWN_COUNT, m_data.queryHitCount );
			DrawTextLine( "ray = yellow, overlap box = magenta, sphere cast = cyan, spawn = white" );
		}
		else
		{
			DrawTextLine( "sleep step = %d, hash = 0x%08X", m_data.sleepStep, m_data.hash );
			DrawTextLine( "query hits = %d, query hash = 0x%08X", m_data.queryHitCount, m_data.queryHash );
		}
	}

	void Render() override
	{
		Sample::Render();

		if ( m_data.spawnCount == 0 )
		{
			return;
		}

		// Ray with hit point and normal, red end point on a miss
		DrawLine( m_data.rayOrigin, m_data.rayPoint, MakeColor( b3_colorYellow ) );
		DrawPoint( m_data.rayOrigin, 5.0f, MakeColor( b3_colorYellow ) );
		if ( m_data.rayDidHit )
		{
			DrawPoint( m_data.rayPoint, 8.0f, MakeColor( b3_colorGreen ) );
			DrawLine( m_data.rayPoint, b3OffsetPos( m_data.rayPoint, m_data.rayNormal ), MakeColor( b3_colorGreen ) );
		}
		else
		{
			DrawPoint( m_data.rayPoint, 5.0f, MakeColor( b3_colorRed ) );
		}

		DrawBounds( m_data.overlapBounds, 0.0f, MakeColor( b3_colorMagenta ) );

		// Swept sphere stopped at its cast fraction, gray when it reached full length
		b3Pos castCenter = b3OffsetPos( m_data.rayOrigin, b3MulSV( m_data.castFraction, m_data.rayTranslation ) );
		b3Sphere castSphere = { b3Vec3_zero, QUERY_SPAWN_CAST_RADIUS };
		Vec4 castColor = m_data.castFraction < 1.0f ? MakeColor( b3_colorCyan ) : MakeColor( b3_colorGray );
		DrawWireSphere( { castCenter, b3Quat_identity }, &castSphere, 24, castColor );

		DrawCross( m_data.lastSpawnPosition, 0.5f, MakeColor( b3_colorWhite ) );
	}

	static Sample* Create( SampleContext* context )
	{
		return new QuerySpawn( context );
	}

	QuerySpawnData m_data;
	int m_frameCount;
	bool m_done;
};

static int sampleQuerySpawn = RegisterSample( "Determinism", "Query Spawn", QuerySpawn::Create );

class MeshDropDeterminism : public Sample
{
public:
	explicit MeshDropDeterminism( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 20.0f, b3Pos_zero );
			GetGuiDraw()->forceScale = 0.1f;
		}

		m_data = CreateMeshDrop( m_worldId, b3Pos_zero );
		m_done = false;
	}

	~MeshDropDeterminism() override
	{
		DestroyMeshDrop( &m_data );
	}

	void Step() override
	{
		Sample::Step();

		if ( m_done == false )
		{
			if ( m_didStep )
			{
				m_done = UpdateMeshDrop( m_worldId, &m_data );
				if ( m_done )
				{
					printf( "sleep step = %d, hash = 0x%08X\n", m_data.sleepStep, m_data.hash );
				}
			}
		}
		else
		{
			DrawTextLine( "sleep step = %d, hash = 0x%08X", m_data.sleepStep, m_data.hash );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new MeshDropDeterminism( context );
	}

	MeshDropData m_data;
	bool m_done;
};

static int sampleMeshDropDeterminism = RegisterSample( "Determinism", "Mesh Drop", MeshDropDeterminism::Create );
