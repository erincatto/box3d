// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/debug_adapter.h"
#include "imgui.h"
#include "sample.h"

#include "box3d/box3d.h"

// Builds an identical box stack a long way from the origin to show off double precision world
// positions. With double precision on, a stack at 1e7 settles exactly like one at the origin and
// renders crisply because the debug draw origin tracks the content. A float build of the same
// sample keeps full speed but loses sub-meter resolution far out, so the stack snaps to a coarse
// grid and jitters. The settled height readout stays put at any offset in double, and drifts in float.
class LargeWorld : public Sample
{
public:
	static constexpr float m_maxOffset = 10000.0f;

	explicit LargeWorld( SampleContext* context )
		: Sample( context )
	{
		// Double precision opens at the dramatic offset, float opens at the origin so it is usable
		// out of the box. Either way the slider sweeps the full range.
		m_offsetKilometers = b3IsDoublePrecision() ? m_maxOffset : 0.0f;
		m_columnCount = 6;

		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 8.0f, 16.0f, { 0.0f, 2.0f, 0.0f } );
		}

		BuildScene();
	}

	// Place the ground and stack at the current offset and point the draw origin at them so the
	// float renderer works in a small relative frame.
	void BuildScene()
	{
		b3Pos base = { 1000.0f * m_offsetKilometers, 0.0f, 0.0f };
		m_drawOrigin = base;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.name = "ground";
		bodyDef.position = b3OffsetPos( base, { 0.0f, -1.0f, 0.0f } );
		b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull groundHull = b3MakeBoxHull( 12.0f, 1.0f, 12.0f );
		b3ShapeId groundShapeId = b3CreateHullShape( groundId, &shapeDef, &groundHull.base );
		SetGroundShape( groundShapeId );

		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3BodyDef boxDef = b3DefaultBodyDef();
		boxDef.type = b3_dynamicBody;
		b3ShapeDef boxShape = b3DefaultShapeDef();

		m_topBodyId = b3_nullBodyId;
		for ( int i = 0; i < m_columnCount; ++i )
		{
			// A small alternating skew so a float build visibly drifts rather than balancing by luck.
			float skew = 0.02f * ( i & 1 ? 1.0f : -1.0f );
			boxDef.position = b3OffsetPos( base, { skew, 0.5f + 1.0f * i, 0.0f } );
			b3BodyId body = b3CreateBody( m_worldId, &boxDef );
			b3CreateHullShape( body, &boxShape, &box.base );
			m_topBodyId = body;
		}
	}

	void Rebuild()
	{
		b3Capacity capacity = {};
		CreateWorld( &capacity );
		BuildScene();
	}

	bool DrawControls() override
	{
		const float presets[] = { 0.0f, 10.0f, 100.0f, 1000.0f, m_maxOffset };
		const char* labels[] = { "origin", "10km", "100km", "1000km", "10000km" };
		for ( int i = 0; i < 5; ++i )
		{
			if ( ImGui::Button( labels[i] ) )
			{
				m_offsetKilometers = presets[i];
				Rebuild();
			}
		}

		return true;
	}

	void Step() override
	{
		Sample::Step();

		// Height of the top box above the ground, measured in the offset's own frame. This holds
		// steady at any offset under double precision and drifts once float runs out of resolution.
		b3Vec3 top = b3SubPos( b3Body_GetWorldCenterOfMass( m_topBodyId ), m_drawOrigin );

		DrawTextLine( "double precision: %s", b3IsDoublePrecision() ? "ON" : "OFF" );
		DrawTextLine( "world offset: %.1f km", m_offsetKilometers );
		DrawTextLine( "top box height above ground: %.4f m", top.y );
	}

	static Sample* Create( SampleContext* context )
	{
		return new LargeWorld( context );
	}

	float m_offsetKilometers;
	int m_columnCount;
	b3BodyId m_topBodyId;
};

static int sampleLargeWorld = RegisterSample( "World", "Large World", LargeWorld::Create );
