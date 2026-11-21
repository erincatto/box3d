// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/debug_adapter.h"
#include "human.h"
#include "imgui.h"
#include "sample.h"
#include "stability.h"

#include "box3d/box3d.h"

// Builds an identical box stack a long way from the origin to show off double precision world
// positions. With double precision on, a stack at 1e7 settles exactly like one at the origin and
// renders crisply because the debug draw origin tracks the content. A float build of the same
// sample keeps full speed but loses sub-meter resolution far out, so the stack snaps to a coarse
// grid and jitters. The settled height readout stays put at any offset in double, and drifts in float.
class FarStack : public Sample
{
public:
	static constexpr float m_maxOffset = 10000.0f;

	explicit FarStack( SampleContext* context )
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

	// Place the ground and stack at the current offset and aim the camera at them. The draw origin
	// rides the camera eye, so the float renderer works in a small relative frame near the content
	// no matter how large the offset.
	void BuildScene()
	{
		b3Pos base = { 1000.0f * m_offsetKilometers, 0.0f, 0.0f };
		m_base = base;
		m_camera->m_pivot = b3OffsetPos( base, { 0.0f, 2.0f, 0.0f } );
		m_camera->UpdateTransform();

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
		b3Vec3 top = b3SubPos( b3Body_GetWorldCenter( m_topBodyId ), m_base );

		DrawTextLine( "double precision: %s", b3IsDoublePrecision() ? "ON" : "OFF" );
		DrawTextLine( "world offset: %.1f km", m_offsetKilometers );
		DrawTextLine( "top box height above ground: %.4f m", top.y );
	}

	static Sample* Create( SampleContext* context )
	{
		return new FarStack( context );
	}

	float m_offsetKilometers;
	int m_columnCount;
	b3BodyId m_topBodyId;
	b3Pos m_base; // world position of the offset content, the frame the height readout uses
};

static int sampleLargeWorld = RegisterSample( "World", "Far Stack", FarStack::Create );

// Motion lock joints on bodies far from the static ground body's center. The ground sits at the world
// origin and the locks sit at the offset, so each joint spans the whole distance. The joint keeps the anchor
// separation in double precision and the error readout stays at solver noise at any offset. Subtracting the
// two body centers in float instead lets gravity sag the bodies by up to half a float step of the offset,
// about half a meter at 10000 km.
class FarMotionLock : public Sample
{
public:
	static constexpr float m_maxOffset = 10000.0f;
	static constexpr int e_count = 4;

	explicit FarMotionLock( SampleContext* context )
		: Sample( context )
	{
		m_offsetKilometers = b3IsDoublePrecision() ? m_maxOffset : 0.0f;

		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 20.0f, 30.0f, { 0.0f, 0.0f, 0.0f } );
		}

		BuildScene();
	}

	void BuildScene()
	{
		// Every coordinate is an integer number of meters so the ground's float anchors are exact at 1e7
		float offset = 1000.0f * m_offsetKilometers;
		b3Pos base = { offset, offset, offset };
		m_camera->m_pivot = b3OffsetPos( base, { 0.0f, -2.0f, 0.0f } );
		m_camera->UpdateTransform();

		b3BodyDef groundDef = b3DefaultBodyDef();
		groundDef.name = "ground";
		b3BodyId groundId = b3CreateBody( m_worldId, &groundDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.enableSleep = false;

		struct Preset
		{
			bool x, y, z, rotation;
			b3Vec3 localAnchor;
		};

		Preset presets[e_count] = {
			{ true, true, false, false, { 0.0f, 0.0f, 0.0f } },
			{ false, true, false, false, { 0.0f, 0.0f, 0.0f } },
			{ true, true, true, false, { 0.5f, 0.5f, 0.5f } },
			{ false, false, true, true, { 0.0f, 0.0f, 0.0f } },
		};

		for ( int i = 0; i < e_count; ++i )
		{
			b3Pos anchor = b3OffsetPos( base, { -6.0f + 4.0f * i, 0.0f, 0.0f } );
			bodyDef.position = b3OffsetPos( anchor, b3Neg( presets[i].localAnchor ) );
			m_bodyIds[i] = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( m_bodyIds[i], &shapeDef, &box.base );

			b3MotionLockJointDef jointDef = b3DefaultMotionLockJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = m_bodyIds[i];
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( groundId, anchor );
			jointDef.base.localFrameB.p = presets[i].localAnchor;
			jointDef.lockLinearX = presets[i].x;
			jointDef.lockLinearY = presets[i].y;
			jointDef.lockLinearZ = presets[i].z;
			jointDef.lockAngularXY = presets[i].rotation;
			m_jointIds[i] = b3CreateMotionLockJoint( m_worldId, &jointDef );

			m_maxSeparation[i] = 0.0f;
		}

		m_stepIndex = 0;
	}

	void Rebuild()
	{
		b3Capacity capacity = {};
		CreateWorld( &capacity );
		BuildScene();
	}

	bool DrawControls() override
	{
		const float presets[] = { 0.0f, 10.0f, 1000.0f, m_maxOffset };
		const char* labels[] = { "origin", "10km", "1000km", "10000km" };
		for ( int i = 0; i < 4; ++i )
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
		if ( m_context->pause == false || m_context->singleStep > 0 )
		{
			// Bounded back and forth push along the free axes
			float push = 2.0f * cosf( m_stepIndex / 60.0f );
			for ( int i = 0; i < e_count; ++i )
			{
				b3Body_ApplyForceToCenter( m_bodyIds[i], { push, 0.0f, -push }, true );
			}
			m_stepIndex += 1;
		}

		Sample::Step();

		for ( int i = 0; i < e_count; ++i )
		{
			m_maxSeparation[i] = b3MaxFloat( m_maxSeparation[i], b3Joint_GetLinearSeparation( m_jointIds[i] ) );
		}
	}

	void Render() override
	{
		Sample::Render();

		DrawTextLine( "double precision: %s", b3IsDoublePrecision() ? "ON" : "OFF" );
		DrawTextLine( "locks %.0f km from the ground body center", m_offsetKilometers );

		const char* names[e_count] = { "line", "plane", "point at corner", "planar" };
		for ( int i = 0; i < e_count; ++i )
		{
			DrawTextLine( "%s: max lock error %.4f mm", names[i], 1000.0f * m_maxSeparation[i] );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new FarMotionLock( context );
	}

	float m_offsetKilometers;
	int m_stepIndex;
	b3BodyId m_bodyIds[e_count];
	b3JointId m_jointIds[e_count];
	float m_maxSeparation[e_count];
};

static int sampleFarMotionLock = RegisterSample( "World", "Far Motion Lock", FarMotionLock::Create );

class FarPyramid : public Sample
{
public:
	static constexpr float m_offsetKilometers = 10000.0f;

	explicit FarPyramid( SampleContext* context )
		: Sample( context )
	{
		b3Pos base = { 1000.0f * m_offsetKilometers, 0.0f, 0.0f };

		if ( context->restart == false )
		{
			m_camera->SetView( 40.0f, -10.0f, 60.0f, b3OffsetPos( base, { 0.0f, 20.0f, 0.0f } ) );
		}

		int baseCount = 40;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3OffsetPos( base, { 0.0f, -1.0f, 0.0f } );
		b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull groundHull = b3MakeBoxHull( 400.0f, 1.0f, 400.0f );
		b3ShapeId groundShapeId = b3CreateHullShape( groundId, &shapeDef, &groundHull.base );
		SetGroundShape( groundShapeId );

		float h = 0.5f;
		float shift = h;
		b3BoxHull box = b3MakeBoxHull( h, h, h );
		shapeDef.density = 100.0f;
		bodyDef.type = b3_dynamicBody;
		for ( int i = 0; i < baseCount; ++i )
		{
			float y = ( 2.0f * i + 1.0f ) * shift;
			for ( int j = i; j < baseCount; ++j )
			{
				float x = ( i + 1.0f ) * shift + 2.0f * ( j - i ) * shift - h * baseCount;
				bodyDef.position = b3OffsetPos( base, { x, y, 0.0f } );
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}
		}
	}

	void Step() override
	{
		Sample::Step();

		DrawTextLine( "double precision: %s", b3IsDoublePrecision() ? "ON" : "OFF" );
		DrawTextLine( "pyramid built %.0f km from the world origin", m_offsetKilometers );
	}

	static Sample* Create( SampleContext* context )
	{
		return new FarPyramid( context );
	}
};

static int sampleFarPyramid = RegisterSample( "World", "Far Pyramid", FarPyramid::Create );

class FarRagdolls : public Sample
{
public:
	static constexpr int m_count = 20;
	static constexpr float m_offsetKilometers = 1000.0f;

	explicit FarRagdolls( SampleContext* context )
		: Sample( context )
	{
		b3Pos base = { 1000.0f * m_offsetKilometers, 0.0f, 0.0f };

		if ( context->restart == false )
		{
			m_camera->SetView( 180.0f, 30.0f, 20.0f, base );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3OffsetPos( base, { 0.0f, -1.0f, 0.0f } );
		b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		m_groundMesh = b3CreateGridMesh( 20, 20, 1.0f, 1, true );
		b3CreateMeshShape( groundId, &shapeDef, m_groundMesh, b3Vec3_one );

		for ( int i = 0; i < m_count; ++i )
		{
			b3Vec3 offset = {
				0.15f * ( i - 0.5f * m_count ),
				2.0f + 0.25f * i,
				0.15f * ( 0.5f * m_count - i ),
			};
			b3Pos position = b3OffsetPos( base, offset );
			float torque = 10.0f;
			float hertz = 0.5f;
			float damping = 0.7f;
			CreateHuman( m_humans + i, m_worldId, position, torque, hertz, damping, i, nullptr, false );
		}
	}

	~FarRagdolls() override
	{
		b3DestroyMesh( m_groundMesh );
	}

	void Step() override
	{
		Sample::Step();

		DrawTextLine( "double precision: %s", b3IsDoublePrecision() ? "ON" : "OFF" );
		DrawTextLine( "%d ragdolls piled %.0f km from the world origin", m_count, m_offsetKilometers );
	}

	static Sample* Create( SampleContext* context )
	{
		return new FarRagdolls( context );
	}

	b3MeshData* m_groundMesh;
	Human m_humans[m_count] = {};
};

static int sampleFarRagdolls = RegisterSample( "World", "Far Ragdolls", FarRagdolls::Create );

// The Mesh Drop unit test, run 1000 km out in both x and z. A field of thin boxes rains onto a
// wave mesh and must come to rest. Continuous collision and the contact solver work in delta
// space, so the pile settles far from the origin exactly as it does at the origin. Past 300 steps
// any body still moving trips the failure readout, the same check the unit test uses.
class FarMeshDrop : public Sample
{
public:
	static constexpr float m_offsetKilometers = 1000.0f;

	explicit FarMeshDrop( SampleContext* context )
		: Sample( context )
	{
		b3Pos base = { 1000.0f * m_offsetKilometers, 0.0f, 1000.0f * m_offsetKilometers };

		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 20.0f, base );
			GetGuiDraw()->forceScale = 0.1f;
		}

		m_data = CreateMeshDrop( m_worldId, base );
		m_failed = false;
	}

	~FarMeshDrop() override
	{
		DestroyMeshDrop( &m_data );
	}

	void Step() override
	{
		Sample::Step();

		// Once the pile should have settled, any remaining body motion means it did not come to
		// rest far from the origin.
		if ( m_failed == false && m_stepCount >= 300 )
		{
			b3BodyEvents bodyEvents = b3World_GetBodyEvents( m_worldId );
			if ( bodyEvents.moveCount > 0 )
			{
				m_failed = true;
			}
		}

		DrawTextLine( "double precision: %s", b3IsDoublePrecision() ? "ON" : "OFF" );
		DrawTextLine( "mesh drop running %.0f km from the world origin", m_offsetKilometers );
		if ( m_failed )
		{
			DrawTextLine( "failed!" );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new FarMeshDrop( context );
	}

	MeshDropData m_data;
	bool m_failed;
};

static int sampleFarMeshDrop = RegisterSample( "World", "Far Mesh Drop", FarMeshDrop::Create );
