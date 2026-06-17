// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "determinism.h"
#include "sample.h"

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
