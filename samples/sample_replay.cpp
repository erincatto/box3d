// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/debug_adapter.h"
#include "gfx/draw.h"
#include "gfx/text.h"
#include "imgui.h"
#include "sample.h"

#include "box3d/box3d.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

// Inspector readout names
static const char* ReplayBodyTypeName( b3BodyType type )
{
	switch ( type )
	{
		case b3_staticBody:
			return "static";
		case b3_kinematicBody:
			return "kinematic";
		case b3_dynamicBody:
			return "dynamic";
		default:
			return "?";
	}
}

static const char* ReplayShapeTypeName( b3ShapeType type )
{
	switch ( type )
	{
		case b3_sphereShape:
			return "sphere";
		case b3_capsuleShape:
			return "capsule";
		case b3_hullShape:
			return "hull";
		case b3_meshShape:
			return "mesh";
		case b3_heightShape:
			return "height field";
		case b3_compoundShape:
			return "compound";
		default:
			return "?";
	}
}

// b3HexColor to an opaque ImVec4 for panel text
static ImVec4 PanelColor( b3HexColor hexColor )
{
	uint32_t h = (uint32_t)hexColor;
	return ImGui::ColorConvertU32ToFloat4( IM_COL32( ( h >> 16 ) & 0xFF, ( h >> 8 ) & 0xFF, h & 0xFF, 255 ) );
}

// Plays back a .b3rec recording by driving the keyframe player one recorded step at a time and
// drawing the replayed world. The player owns the world, so the base sample world is left empty and
// unused. Mouse picking only reads the world (no drag joint), since mutating it would diverge the
// replay from the recording.
class ReplayViewer : public Sample
{
public:
	explicit ReplayViewer( SampleContext* context )
		: Sample( context )
	{
		m_player = nullptr;
		m_replayWorldId = b3_nullWorldId;
		m_frameCount = 0;
		m_speed = 1.0f;
		m_frameAccumulator = 0.0f;
		m_loop = true;
		m_selBody = b3_nullBodyId;
		m_status[0] = '\0';
		snprintf( m_path, sizeof( m_path ), "replay_demo.b3rec" );

		if ( context->restart == false )
		{
			m_camera->SetView( 30.0f, 22.0f, 14.0f, { 0.0f, 2.0f, 0.0f } );
		}

		// Auto-play a loaded recording so the sample shows motion immediately.
		m_context->pause = false;

		// Open the default recording if it already exists, so Restart (R) re-opens it. A missing
		// file is not an error here, just an invitation to generate the demo.
		if ( LoadFromPath() == false )
		{
			snprintf( m_status, sizeof( m_status ), "No recording. Generate a demo or load a .b3rec." );
		}
	}

	~ReplayViewer() override
	{
		// Runs before the base destructor, which destroys the (empty) base world and resets the
		// debug-shape pool. Destroying the player here releases the replay world's pool entries first.
		ClosePlayer();
	}

	void ClosePlayer()
	{
		if ( m_player != nullptr )
		{
			b3RecPlayer_Destroy( m_player );
			m_player = nullptr;
		}
		m_replayWorldId = b3_nullWorldId;
		m_selBody = b3_nullBodyId;
		SetHoveredBody( b3_nullBodyId );
		ClearSelection();
	}

	// Load m_path into a fresh player and adopt its world. Returns false (and sets m_status) on any
	// failure: missing file, bad header, or deserialization error.
	bool LoadFromPath()
	{
		ClosePlayer();

		if ( m_path[0] == '\0' )
		{
			snprintf( m_status, sizeof( m_status ), "No file path." );
			return false;
		}

		b3Recording* recording = b3LoadRecordingFromFile( m_path );
		if ( recording == nullptr )
		{
			snprintf( m_status, sizeof( m_status ), "Could not open %s", m_path );
			return false;
		}

		// The player copies the bytes, so the recording can be freed right away.
		m_player = b3RecPlayer_Create( b3Recording_GetData( recording ), b3Recording_GetSize( recording ), 1 );
		b3DestroyRecording( recording );

		if ( m_player == nullptr )
		{
			snprintf( m_status, sizeof( m_status ), "Bad recording: %s", m_path );
			return false;
		}

		// Wire the renderer's debug-shape callbacks into the player's world. Without them the replay
		// world has no GPU mesh handles and draws nothing. This rebuilds the world, so adopt it after.
		b3WorldDef defTemplate = b3DefaultWorldDef();
		AttachToWorldDef( &defTemplate );
		b3RecPlayer_SetDebugShapeCallbacks( m_player, defTemplate.createDebugShape, defTemplate.destroyDebugShape,
											defTemplate.userDebugShapeContext );

		m_frameCount = b3RecPlayer_GetFrameCount( m_player );
		AdoptPlayerWorld( true );
		snprintf( m_status, sizeof( m_status ), "Loaded %d frames", m_frameCount );
		return true;
	}

	// Build a small settling-pyramid recording and save it to m_path. Public API only, so the sample
	// is self-contained: no external .b3rec needed to exercise the viewer.
	bool GenerateDemoRecording()
	{
		b3Recording* recording = b3CreateRecording( 0 );

		// Serial, deterministic world (no task callbacks). Not drawn, so no adapter wiring.
		b3WorldDef worldDef = b3DefaultWorldDef();
		b3WorldId worldId = b3CreateWorld( &worldDef );

		b3World_StartRecording( worldId, recording );
		b3World_SetGravity( worldId, { 0.0f, -10.0f, 0.0f } );

		b3BodyDef groundDef = b3DefaultBodyDef();
		groundDef.type = b3_staticBody;
		groundDef.name = "ground";
		groundDef.position = { 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = b3CreateBody( worldId, &groundDef );
		b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
		b3ShapeDef groundShape = b3DefaultShapeDef();
		b3CreateHullShape( groundId, &groundShape, &groundBox.base );

		// One unit box shared by every block, so the geometry registry dedups to a single entry.
		const float h = 0.5f;
		b3BoxHull box = b3MakeBoxHull( h, h, h );
		b3ShapeDef boxShape = b3DefaultShapeDef();
		boxShape.density = 1.0f;

		const int baseCount = 6;
		for ( int row = 0; row < baseCount; ++row )
		{
			int count = baseCount - row;
			// Small vertical gap so the stack drops and settles in the first frames.
			float y = h + row * ( 2.0f * h + 0.04f ) + 0.05f;
			float x0 = -( count - 1 ) * h;
			for ( int j = 0; j < count; ++j )
			{
				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.type = b3_dynamicBody;
				bodyDef.position = { x0 + j * 2.0f * h, y, 0.0f };
				b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
				b3CreateHullShape( bodyId, &boxShape, &box.base );
			}
		}

		for ( int i = 0; i < 300; ++i )
		{
			b3World_Step( worldId, 1.0f / 60.0f, 4 );
		}

		b3World_StopRecording( worldId );
		bool ok = b3SaveRecordingToFile( recording, m_path );
		b3DestroyWorld( worldId );
		b3DestroyRecording( recording );

		if ( ok == false )
		{
			snprintf( m_status, sizeof( m_status ), "Failed to write %s", m_path );
		}
		return ok;
	}

	// Point the camera at the recorded motion and reset the frame accumulator. For from-creation
	// recordings the world is empty at frame 0, so step one frame to populate the bodies, frame
	// against those bounds, then rewind. SeekFrame(0) may rebuild the world, so re-read the id after.
	void AdoptPlayerWorld( bool frameCamera )
	{
		if ( frameCamera && m_frameCount > 0 )
		{
			b3RecPlayer_StepFrame( m_player );
			b3AABB bounds = b3World_GetBounds( b3RecPlayer_GetWorldId( m_player ) );
			b3Vec3 extent = b3Sub( bounds.upperBound, bounds.lowerBound );
			if ( extent.x > 0.0f || extent.y > 0.0f || extent.z > 0.0f )
			{
				float aspect = m_camera->m_height > 0 ? (float)m_camera->m_width / (float)m_camera->m_height : 1.0f;
				m_camera->Frame( bounds, aspect, 1.4f );
			}
			b3RecPlayer_SeekFrame( m_player, 0 );
		}
		m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
		m_frameAccumulator = 0.0f;
	}

	// Advance one recorded step and keep the world pointer current (stable forward, refreshed cheaply).
	void AdvanceOne()
	{
		b3RecPlayer_StepFrame( m_player );
		m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
	}

	void Step() override
	{
		if ( m_player != nullptr )
		{
			if ( m_context->pause && m_context->singleStep > 0 )
			{
				m_context->singleStep = b3MaxInt( 0, m_context->singleStep - 1 );
				if ( b3RecPlayer_IsAtEnd( m_player ) == false )
				{
					AdvanceOne();
				}
				m_frameAccumulator = 0.0f;
			}
			else if ( m_context->pause == false )
			{
				// Speed scales recorded steps per display frame.
				m_frameAccumulator += m_speed;
				while ( m_frameAccumulator >= 1.0f )
				{
					m_frameAccumulator -= 1.0f;
					if ( b3RecPlayer_IsAtEnd( m_player ) )
					{
						if ( m_loop )
						{
							b3RecPlayer_SeekFrame( m_player, 0 );
							m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
						}
						else
						{
							m_frameAccumulator = 0.0f;
							break;
						}
					}
					AdvanceOne();
				}
			}

			// Keep the info panel "step N" line on the replay frame.
			m_stepCount = b3RecPlayer_GetFrame( m_player );
		}

		SetDrawOrigin( m_camera->m_worldEye );

		if ( B3_IS_NULL( m_replayWorldId ) )
		{
			DrawScreenStringFormat( 5, m_textLine, MakeColor( b3_colorLightGray ), "%s", m_status );
			return;
		}

		// Highlight the dynamic body under the cursor, matching the live samples.
		PickRay pickRay = m_camera->BuildPickRay( m_context->mouseX, m_context->mouseY );
		b3RayResult hover = b3World_CastRayClosest( m_replayWorldId, pickRay.origin, pickRay.translation, b3DefaultQueryFilter() );
		b3BodyId hovered = b3_nullBodyId;
		if ( hover.hit && b3Body_GetType( b3Shape_GetBody( hover.shapeId ) ) == b3_dynamicBody )
		{
			hovered = b3Shape_GetBody( hover.shapeId );
		}
		SetHoveredBody( hovered );

		// Draw the replay world through the same adapter path the live samples use.
		b3DebugDraw debugDraw;
		MakeDebugDraw( &debugDraw );
		ApplyGuiFlags( &debugDraw );
		b3Vec3 r = { 1000.0f, 1000.0f, 1000.0f };
		debugDraw.drawingBounds = b3OffsetAABB( { b3Neg( r ), r }, m_camera->m_worldEye );
		b3World_Draw( m_replayWorldId, &debugDraw, B3_DEFAULT_MASK_BITS );

		DrawSelectionOverlay();
	}

	// A replay re-runs recorded inputs, so the live solver sliders would do nothing.
	bool HasSolverControls() const override
	{
		return false;
	}

	// Transport row shared by the load state and the playing state.
	void DrawTransport()
	{
		int frame = b3RecPlayer_GetFrame( m_player );

		if ( ImGui::Button( "|<" ) )
		{
			b3RecPlayer_SeekFrame( m_player, 0 );
			m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
			m_frameAccumulator = 0.0f;
		}
		ImGui::SameLine();
		if ( ImGui::Button( "<" ) )
		{
			b3RecPlayer_SeekFrame( m_player, frame - 1 );
			m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
			m_frameAccumulator = 0.0f;
			m_context->pause = true;
		}
		ImGui::SameLine();
		if ( m_context->pause )
		{
			if ( ImGui::Button( "Play" ) )
			{
				m_context->pause = false;
			}
		}
		else
		{
			if ( ImGui::Button( "Pause" ) )
			{
				m_context->pause = true;
			}
		}
		ImGui::SameLine();
		if ( ImGui::Button( ">" ) )
		{
			b3RecPlayer_SeekFrame( m_player, frame + 1 );
			m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
			m_frameAccumulator = 0.0f;
			m_context->pause = true;
		}
		ImGui::SameLine();
		if ( ImGui::Button( ">|" ) )
		{
			b3RecPlayer_SeekFrame( m_player, m_frameCount );
			m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
			m_frameAccumulator = 0.0f;
		}
	}

	bool DrawControls() override
	{
		float fontSize = ImGui::GetFontSize();

		// File row: editable path plus load and generate. Always present so a recording can be opened
		// even when none is loaded.
		ImGui::PushItemWidth( -FLT_MIN );
		ImGui::InputText( "##path", m_path, sizeof( m_path ) );
		ImGui::PopItemWidth();
		if ( ImGui::Button( "Load" ) )
		{
			LoadFromPath();
		}
		ImGui::SameLine();
		if ( ImGui::Button( "Generate demo" ) )
		{
			if ( GenerateDemoRecording() )
			{
				LoadFromPath();
			}
		}
		ImGui::TextWrapped( "%s", m_status );

		if ( m_player == nullptr )
		{
			return true;
		}

		ImGui::Separator();
		DrawTransport();

		// Scrubber seeks both directions; backward uses the keyframe ring.
		int scrub = b3RecPlayer_GetFrame( m_player );
		ImGui::PushItemWidth( -FLT_MIN );
		if ( ImGui::SliderInt( "##frame", &scrub, 0, m_frameCount ) )
		{
			b3RecPlayer_SeekFrame( m_player, scrub );
			m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
			m_frameAccumulator = 0.0f;
			m_context->pause = true;
		}
		ImGui::PopItemWidth();

		const char* speedNames[] = { "0.25x", "0.5x", "1x", "2x", "4x" };
		const float speedValues[] = { 0.25f, 0.5f, 1.0f, 2.0f, 4.0f };
		int speedIndex = 2;
		for ( int i = 0; i < 5; ++i )
		{
			if ( m_speed == speedValues[i] )
			{
				speedIndex = i;
			}
		}
		ImGui::PushItemWidth( 5.0f * fontSize );
		if ( ImGui::Combo( "Speed", &speedIndex, speedNames, 5 ) )
		{
			m_speed = speedValues[speedIndex];
		}
		ImGui::PopItemWidth();
		ImGui::SameLine();
		ImGui::Checkbox( "Loop", &m_loop );

		ImGui::Text( "frame %d / %d%s", b3RecPlayer_GetFrame( m_player ), m_frameCount,
					 b3RecPlayer_IsAtEnd( m_player ) ? "  (end)" : "" );
		if ( b3RecPlayer_HasDiverged( m_player ) )
		{
			ImGui::TextColored( PanelColor( b3_colorRed ), "****DIVERGED****" );
		}

		if ( B3_IS_NON_NULL( m_replayWorldId ) )
		{
			b3Counters c = b3World_GetCounters( m_replayWorldId );
			b3Vec3 g = b3World_GetGravity( m_replayWorldId );
			ImGui::Separator();
			ImGui::Text( "bodies %d  shapes %d", c.bodyCount, c.shapeCount );
			ImGui::Text( "contacts %d  joints %d", c.contactCount, c.jointCount );
			ImGui::Text( "gravity %.1f %.1f %.1f", g.x, g.y, g.z );
		}

		ImGui::Separator();
		DrawInspector();
		return true;
	}

	// Selected-body detail. Selection is an id, so a backward seek that rebuilds the world drops it
	// (the id no longer validates); the panel then prompts for a fresh pick.
	void DrawInspector()
	{
		if ( B3_IS_NULL( m_selBody ) || b3Body_IsValid( m_selBody ) == false )
		{
			if ( B3_IS_NON_NULL( m_selBody ) )
			{
				m_selBody = b3_nullBodyId;
				ClearSelection();
			}
			ImGui::TextDisabled( "Click a body to inspect." );
			return;
		}

		const char* name = b3Body_GetName( m_selBody );
		b3WorldTransform xf = b3Body_GetTransform( m_selBody );
		b3Vec3 v = b3Body_GetLinearVelocity( m_selBody );
		float spin;
		b3GetAxisAngle( &spin, xf.q );

		ImGui::Text( "body %d  %s", m_selBody.index1,
					 ( name != nullptr && name[0] != '\0' ) ? name : ReplayBodyTypeName( b3Body_GetType( m_selBody ) ) );
		ImGui::Text( "pos   %.2f %.2f %.2f", xf.p.x, xf.p.y, xf.p.z );
		ImGui::Text( "spin  %.0f deg", spin * B3_RAD_TO_DEG );
		ImGui::Text( "vel   %.2f %.2f %.2f", v.x, v.y, v.z );
		ImGui::Text( "speed %.2f  omega %.2f", b3Length( v ), b3Length( b3Body_GetAngularVelocity( m_selBody ) ) );
		ImGui::Text( "mass  %.3g  awake %s", b3Body_GetMass( m_selBody ), b3Body_IsAwake( m_selBody ) ? "yes" : "no" );
		ImGui::Text( "shapes %d  joints %d", b3Body_GetShapeCount( m_selBody ), b3Body_GetJointCount( m_selBody ) );

		b3ShapeId shapes[8];
		int shapeCount = b3Body_GetShapes( m_selBody, shapes, 8 );
		if ( shapeCount > 0 )
		{
			b3ShapeId s = shapes[0];
			ImGui::Separator();
			ImGui::Text( "shape %s", ReplayShapeTypeName( b3Shape_GetType( s ) ) );
			ImGui::Text( "friction %.2f  restitution %.2f", b3Shape_GetFriction( s ), b3Shape_GetRestitution( s ) );
			ImGui::Text( "density %.2f  sensor %s", b3Shape_GetDensity( s ), b3Shape_IsSensor( s ) ? "yes" : "no" );
		}
	}

	// World-space overlays for the selection: body axes plus live contact points and normals, the
	// most useful solver readout. Drawn straight to the overlay, the world is never touched.
	void DrawSelectionOverlay()
	{
		if ( B3_IS_NULL( m_selBody ) || b3Body_IsValid( m_selBody ) == false )
		{
			return;
		}

		b3WorldTransform xf = b3Body_GetTransform( m_selBody );
		DrawAxes( xf, 0.75f );

		b3ContactData contacts[32];
		int capacity = b3Body_GetContactCapacity( m_selBody );
		if ( capacity > 32 )
		{
			capacity = 32;
		}
		int count = b3Body_GetContactData( m_selBody, contacts, capacity );
		for ( int i = 0; i < count; ++i )
		{
			b3Pos originA = b3Body_GetWorldCenterOfMass( b3Shape_GetBody( contacts[i].shapeIdA ) );
			for ( int m = 0; m < contacts[i].manifoldCount; ++m )
			{
				const b3Manifold* manifold = &contacts[i].manifolds[m];
				for ( int p = 0; p < manifold->pointCount; ++p )
				{
					b3Pos point = b3OffsetPos( originA, manifold->points[p].anchorA );
					DrawPoint( point, 6.0f, MakeColor( b3_colorOrange ) );
					DrawLine( point, b3OffsetPos( point, b3MulSV( 0.3f, manifold->normal ) ), MakeColor( b3_colorOrange ) );
				}
			}
		}
	}

	// Left click selects a body to inspect. Picking only reads the world, it never creates the drag
	// joint the base sample does, so the replay is not mutated. Dragging stays disabled.
	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button != 0 || modifiers != 0 || B3_IS_NULL( m_replayWorldId ) )
		{
			return;
		}

		PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
		b3RayResult result = b3World_CastRayClosest( m_replayWorldId, pickRay.origin, pickRay.translation, b3DefaultQueryFilter() );
		if ( result.hit )
		{
			m_selBody = b3Shape_GetBody( result.shapeId );
			SetSelectedBody( m_selBody );
		}
		else
		{
			m_selBody = b3_nullBodyId;
			ClearSelection();
		}
	}

	void MouseUp( b3Vec2, int ) override
	{
	}

	void MouseMove( b3Vec2 ) override
	{
	}

	static Sample* Create( SampleContext* context )
	{
		return new ReplayViewer( context );
	}

	b3RecPlayer* m_player;
	b3WorldId m_replayWorldId; // player-owned world we draw and pick; separate from the empty base world
	char m_path[256];
	char m_status[128];
	int m_frameCount;
	float m_speed;
	float m_frameAccumulator;
	bool m_loop;
	b3BodyId m_selBody; // picked body, re-resolved each frame; dropped when it stops validating
};

static int sampleReplayViewer = RegisterSample( "Replay", "Viewer", ReplayViewer::Create );
