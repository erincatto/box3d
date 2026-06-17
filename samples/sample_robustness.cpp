// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "overflow_color.h"
#include "sample.h"
#include "gfx/draw.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"
#include "gfx/debug_adapter.h"

#include <imgui.h>
#include <stdlib.h>

// Pyramid with heavy box on top
class HighMassRatio1 : public Sample
{
public:
	explicit HighMassRatio1( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 30.0f, 15.0f, 70.0f, b3Pos_zero );
			
		}

		AddGroundBox( 50.0f );

		float extent = 1.0f;

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			b3BoxHull box = b3MakeBoxHull( extent, extent, extent );
			b3ShapeDef shapeDef = b3DefaultShapeDef();

			for ( int j = 0; j < 3; ++j )
			{
				int count = 10;
				float offset = -20.0f * extent + 2.0f * ( count + 1.0f ) * extent * j;
				float y = extent;
				while ( count > 0 )
				{
					for ( int i = 0; i < count; ++i )
					{
						float coeff = i - 0.5f * count;

						float yy = count == 1 ? y + 2.0f : y;
						bodyDef.position = { 2.0f * coeff * extent + offset, yy, 0.0f };
						b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

						shapeDef.density = count == 1 ? ( j + 1.0f ) * 100.0f : 1.0f;
						b3CreateHullShape( bodyId, &shapeDef, &box.base );
					}

					--count;
					y += 2.0f * extent;
				}
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new HighMassRatio1( context );
	}
};

static int sampleHighMassRatio1 = RegisterSample( "Robustness", "HighMassRatio1", HighMassRatio1::Create );

// A pyramid of 5cm boxes. Stacking tiny objects is challenging for physics engines due to rotational effects.
// This is also challenging for Box3D because of the AABB margin and linear slop are close to the shape size. This
// leads to many collision pairs and some shape overlap.
class TinyPyramid : public Sample
{
public:
	explicit TinyPyramid( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( -30.0f, 20.0f, 10.0f, { 0.0f, 0.5f, 0.0f } );
		}

		AddGroundBox( 20.0f );

		{
			m_extent = 0.025f;
			int baseCount = 30;

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;

			b3ShapeDef shapeDef = b3DefaultShapeDef();

			b3BoxHull box = b3MakeBoxHull( m_extent, m_extent, m_extent );

			for ( int i = 0; i < baseCount; ++i )
			{
				float y = ( 2.0f * i + 1.0f ) * m_extent;

				for ( int j = i; j < baseCount; ++j )
				{
					float x = ( i + 1.0f ) * m_extent + 2.0f * ( j - i ) * m_extent - baseCount * m_extent;
					bodyDef.position = { x, y, 0.0f };

					b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
					b3CreateHullShape( bodyId, &shapeDef, &box.base );
				}
			}
		}
	}

	void Step() override
	{
		DrawTextLine( "%.1fcm boxes", 200.0f * m_extent );
		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new TinyPyramid( context );
	}

	float m_extent;
};

static int sampleTinyPyramid = RegisterSample( "Robustness", "Tiny Pyramid", TinyPyramid::Create );

class OverlapRecovery : public Sample
{
public:
	explicit OverlapRecovery( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 20.0f, 15.0f, b3Pos_zero );
		}

		m_bodyIds = nullptr;
		m_bodyCount = 0;
		m_baseCount = 4;
		m_overlap = 0.25f;
		m_extent = 0.5f;
		m_speed = 3.0f;
		m_hertz = 30.0f;
		m_dampingRatio = 10.0f;

		AddGroundBox( 20.0f );

		CreateScene();
	}

	~OverlapRecovery() override
	{
		free( m_bodyIds );
	}

	void CreateScene()
	{
		for ( int i = 0; i < m_bodyCount; ++i )
		{
			b3DestroyBody( m_bodyIds[i] );
		}

		b3World_SetContactTuning( m_worldId, m_hertz, m_dampingRatio, m_speed );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;

		b3BoxHull box = b3MakeBoxHull( m_extent, m_extent, m_extent );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 1.0f;

		m_bodyCount = m_baseCount * ( m_baseCount + 1 ) / 2;
		m_bodyIds = (b3BodyId*)realloc( m_bodyIds, m_bodyCount * sizeof( b3BodyId ) );

		int bodyIndex = 0;
		float fraction = 1.0f - m_overlap;
		float y = m_extent;
		for ( int i = 0; i < m_baseCount; ++i )
		{
			float x = fraction * m_extent * ( i - m_baseCount );
			for ( int j = i; j < m_baseCount; ++j )
			{
				bodyDef.position = { x, y };
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

				b3CreateHullShape( bodyId, &shapeDef, &box.base );

				m_bodyIds[bodyIndex++] = bodyId;

				x += 2.0f * fraction * m_extent;
			}

			y += 2.0f * fraction * m_extent;
		}

		assert( bodyIndex == m_bodyCount );
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		bool changed = false;
		changed = changed || ImGui::SliderFloat( "Extent", &m_extent, 0.1f, 1.0f, "%.1f" );
		changed = changed || ImGui::SliderInt( "Base Count", &m_baseCount, 1, 10 );
		changed = changed || ImGui::SliderFloat( "Overlap", &m_overlap, 0.0f, 1.0f, "%.2f" );
		changed = changed || ImGui::SliderFloat( "Speed", &m_speed, 0.0f, 10.0f, "%.1f" );
		changed = changed || ImGui::SliderFloat( "Hertz", &m_hertz, 0.0f, 240.0f, "%.f" );
		changed = changed || ImGui::SliderFloat( "Damping Ratio", &m_dampingRatio, 0.0f, 20.0f, "%.1f" );
		changed = changed || ImGui::Button( "Reset Scene" );

		if ( changed )
		{
			CreateScene();
		}

		ImGui::PopItemWidth();
		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new OverlapRecovery( context );
	}

	b3BodyId* m_bodyIds;
	int m_bodyCount;
	int m_baseCount;
	float m_overlap;
	float m_extent;
	float m_speed;
	float m_hertz;
	float m_dampingRatio;
};

static int sampleOverlapRecovery = RegisterSample( "Robustness", "Overlap Recovery", OverlapRecovery::Create );

class Cart : public Sample
{
public:
	explicit Cart( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 15.0f, { 0.0f, 2.0f, 0.0f } );
			//context->settings.subStepCount = 12;
			
		}

		AddGroundBox( 20.0f );

		b3World_SetGravity( m_worldId, { 0, -22, 0 } );

		m_contactHertz = 240.0f;
		m_contactDampingRatio = 10.0f;
		m_contactSpeed = 0.5f;
		b3World_SetContactTuning( m_worldId, m_contactHertz, m_contactDampingRatio, m_contactSpeed );

		m_constraintHertz = 240.0f;
		m_constraintDampingRatio = 0.0f;

		m_chassisId = {};
		m_wheelId1 = {};
		m_wheelId2 = {};
		m_wheelId3 = {};
		m_wheelId4 = {};
		m_jointId1 = {};
		m_jointId2 = {};
		m_jointId3 = {};
		m_jointId4 = {};

		CreateScene();
	}

	void CreateScene()
	{
		if ( B3_IS_NON_NULL( m_chassisId ) )
		{
			b3DestroyBody( m_chassisId );
		}

		if ( B3_IS_NON_NULL( m_wheelId1 ) )
		{
			b3DestroyBody( m_wheelId1 );
		}

		if ( B3_IS_NON_NULL( m_wheelId2 ) )
		{
			b3DestroyBody( m_wheelId2 );
		}

		if ( B3_IS_NON_NULL( m_wheelId3 ) )
		{
			b3DestroyBody( m_wheelId3 );
		}

		if ( B3_IS_NON_NULL( m_wheelId4 ) )
		{
			b3DestroyBody( m_wheelId4 );
		}

		float yBase = 1.0f;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, yBase, 0.0f };
		m_chassisId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 1000.0f;

		b3BoxHull box = b3MakeTransformedBoxHull( 1.0f, 0.25f, 0.5f, { { 0.0f, 0.25f, 0.0f }, b3Quat_identity } );
		b3CreateHullShape( m_chassisId, &shapeDef, &box.base );

		shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.rollingResistance = 0.02f;
		shapeDef.density = 200.0f;

		b3Sphere sphere = { b3Vec3_zero, 0.1f };
		bodyDef.position = { -0.9f, yBase - 0.15f, 0.4f };
		m_wheelId1 = b3CreateBody( m_worldId, &bodyDef );
		b3CreateSphereShape( m_wheelId1, &shapeDef, &sphere );

		bodyDef.position = { 0.9f, yBase - 0.15f, 0.4f };
		m_wheelId2 = b3CreateBody( m_worldId, &bodyDef );
		b3CreateSphereShape( m_wheelId2, &shapeDef, &sphere );

		bodyDef.position = { 0.9f, yBase - 0.15f, -0.4f };
		m_wheelId3 = b3CreateBody( m_worldId, &bodyDef );
		b3CreateSphereShape( m_wheelId3, &shapeDef, &sphere );

		bodyDef.position = { -0.9f, yBase - 0.15f, -0.4f };
		m_wheelId4 = b3CreateBody( m_worldId, &bodyDef );
		b3CreateSphereShape( m_wheelId4, &shapeDef, &sphere );

		b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
		jointDef.base.constraintHertz = m_constraintHertz;
		jointDef.base.constraintDampingRatio = m_constraintDampingRatio;
		jointDef.base.drawScale = 0.5f;

		jointDef.base.bodyIdA = m_chassisId;
		jointDef.base.bodyIdB = m_wheelId1;
		jointDef.base.localFrameA.p = { -0.9f, -0.15f, 0.4f };
		jointDef.base.localFrameB.p = { 0.0f, 0.0f, 0.0f };
		m_jointId1 = b3CreateSphericalJoint( m_worldId, &jointDef );

		jointDef.base.bodyIdA = m_chassisId;
		jointDef.base.bodyIdB = m_wheelId2;
		jointDef.base.localFrameA.p = { 0.9f, -0.15f, 0.4f };
		jointDef.base.localFrameB.p = { 0.0f, 0.0f, 0.0f };
		m_jointId2 = b3CreateSphericalJoint( m_worldId, &jointDef );

		jointDef.base.bodyIdA = m_chassisId;
		jointDef.base.bodyIdB = m_wheelId3;
		jointDef.base.localFrameA.p = { 0.9f, -0.15f, -0.4f };
		jointDef.base.localFrameB.p = { 0.0f, 0.0f, 0.0f };
		m_jointId3 = b3CreateSphericalJoint( m_worldId, &jointDef );

		jointDef.base.bodyIdA = m_chassisId;
		jointDef.base.bodyIdB = m_wheelId4;
		jointDef.base.localFrameA.p = { -0.9f, -0.15f, -0.4f };
		jointDef.base.localFrameB.p = { 0.0f, 0.0f, 0.0f };
		m_jointId4 = b3CreateSphericalJoint( m_worldId, &jointDef );
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		bool changed = false;
		ImGui::Text( "Contact" );
		changed = changed || ImGui::SliderFloat( "Hertz##contact", &m_contactHertz, 0.0f, 240.0f, "%.f" );
		changed = changed || ImGui::SliderFloat( "Damping Ratio##contact", &m_contactDampingRatio, 0.0f, 100.0f, "%.f" );
		changed = changed || ImGui::SliderFloat( "Speed", &m_contactSpeed, 0.0f, 5.0f, "%.1f" );

		if ( changed )
		{
			b3World_SetContactTuning( m_worldId, m_contactHertz, m_contactDampingRatio, m_contactSpeed );
			CreateScene();
		}

		ImGui::Separator();

		changed = false;
		ImGui::Text( "Joint" );
		changed = changed || ImGui::SliderFloat( "Hertz##joint", &m_constraintHertz, 0.0f, 240.0f, "%.f" );
		changed = changed || ImGui::SliderFloat( "Damping Ratio##joint", &m_constraintDampingRatio, 0.0f, 20.0f, "%.f" );

		ImGui::Separator();

		changed = changed || ImGui::Button( "Reset Scene" );

		if ( changed )
		{
			b3Joint_SetConstraintTuning( m_jointId1, m_constraintHertz, m_constraintDampingRatio );
			b3Joint_SetConstraintTuning( m_jointId2, m_constraintHertz, m_constraintDampingRatio );
			b3Joint_SetConstraintTuning( m_jointId3, m_constraintHertz, m_constraintDampingRatio );
			b3Joint_SetConstraintTuning( m_jointId4, m_constraintHertz, m_constraintDampingRatio );
			CreateScene();
		}

		ImGui::PopItemWidth();
		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new Cart( context );
	}

	b3BodyId m_chassisId;
	b3BodyId m_wheelId1;
	b3BodyId m_wheelId2;
	b3BodyId m_wheelId3;
	b3BodyId m_wheelId4;
	b3JointId m_jointId1;
	b3JointId m_jointId2;
	b3JointId m_jointId3;
	b3JointId m_jointId4;

	float m_contactHertz;
	float m_contactDampingRatio;
	float m_contactSpeed;
	float m_constraintHertz;
	float m_constraintDampingRatio;
};

static int sampleCart = RegisterSample( "Robustness", "Cart", Cart::Create );

// This forces a constraint graph color overflow
class OverflowColorPile : public Sample
{
public:
	explicit OverflowColorPile( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 30.0f, 35.0f, 15.0f, b3Pos_zero );
			
		}

		m_data = CreateOverflowColorPile( m_worldId );

		SetGroundShape( m_data.groundShapeId );
	}

	void Step() override
	{
		Sample::Step();

		b3Counters counters = b3World_GetCounters( m_worldId );
		int overflowContacts = counters.colorCounts[B3_GRAPH_COLOR_COUNT - 1];

		DrawTextLine( "neighbors = %d", m_data.neighborCount );
		DrawTextLine( "overflow contacts = %d", overflowContacts );
		DrawTextLine( "total contacts = %d", counters.contactCount );
	}

	static Sample* Create( SampleContext* context )
	{
		return new OverflowColorPile( context );
	}

	OverflowColorPileData m_data;
};

static int sampleOverflowColorPile = RegisterSample( "Robustness", "Overflow Color Pile", OverflowColorPile::Create );
