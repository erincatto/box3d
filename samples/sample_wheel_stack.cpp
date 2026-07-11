// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/debug_adapter.h"
#include "sample.h"

#include "box3d/box3d.h"

#include "metal_wheel1_hulls.h"

#include <stdio.h>

// 30 metal_wheel1 props (37-piece convex decompositions) stacked. Body-pair contact merging
// lets them settle and sleep instead of wobbling.
class WheelStack : public Sample
{
public:
	explicit WheelStack( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 12.0f, 5.0f, { 0.0f, 0.85f, 0.0f } );
		}

		AddGroundBox( 10.0f );

		for ( int h = 0; h < s_metalWheel1HullCount; ++h )
		{
			const WheelHullSpan& span = s_metalWheel1Hulls[h];
			m_hulls[h] = b3CreateHull( &s_metalWheel1Verts[span.offset], span.count, span.count );
		}

		const float height = 0.171f;
		const float spacing = height + 0.006f;
		const float startY = 0.5f * height + 0.004f;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.6f;

		for ( int i = 0; i < m_wheelCount; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.name = "wheel";
			bodyDef.position = { 0.0f, startY + i * spacing, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			for ( int h = 0; h < s_metalWheel1HullCount; ++h )
			{
				b3CreateHullShape( bodyId, &shapeDef, m_hulls[h] );
			}
		}

		b3World_SetContactTuning( m_worldId, 240.0f, 10.0f, 3.0f );
	}

	~WheelStack() override
	{
		for ( int h = 0; h < s_metalWheel1HullCount; ++h )
		{
			b3DestroyHull( m_hulls[h] );
		}
	}

	void Step() override
	{
		Sample::Step();

		b3Profile p = b3World_GetProfile( m_worldId );
		float substepRate = m_context->hertz * m_context->subStepCount;

		DrawTextLine( "wheels %d, hull pieces/wheel %d (%d total shapes)", m_wheelCount, s_metalWheel1HullCount,
					  m_wheelCount * s_metalWheel1HullCount );
		DrawTextLine( "step %.0f hz, sub-steps %d -> substep rate %.0f hz", m_context->hertz, m_context->subStepCount,
					  substepRate );
		DrawTextLine( "eff contact hz = min(240, %.0f) = %.0f", 0.125f * substepRate,
					  b3MinFloat( 240.0f, 0.125f * substepRate ) );
		DrawTextLine( "step %.3f ms | collide %.3f ms (%.0f%%) | solve %.3f ms (%.0f%%)", p.step, p.collide,
					  p.step > 0.0f ? 100.0f * p.collide / p.step : 0.0f, p.solve,
					  p.step > 0.0f ? 100.0f * p.solve / p.step : 0.0f );
	}

	static Sample* Create( SampleContext* context )
	{
		return new WheelStack( context );
	}

	b3HullData* m_hulls[s_metalWheel1HullCount];
	static constexpr int m_wheelCount = 30;
};

static int sampleWheelStack = RegisterSample( "Stacking", "Wheel Stack (PHX)", WheelStack::Create );
