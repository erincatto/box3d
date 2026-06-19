// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/debug_adapter.h"
#include "gfx/draw.h"
#include "gfx/text.h"
#include "imgui.h"
#include "sample.h"

#include "box3d/box3d.h"

#include <float.h>
#include <stdint.h>
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

static const char* ReplayJointTypeName( b3JointType type )
{
	switch ( type )
	{
		case b3_parallelJoint:
			return "parallel";
		case b3_distanceJoint:
			return "distance";
		case b3_filterJoint:
			return "filter";
		case b3_motorJoint:
			return "motor";
		case b3_prismaticJoint:
			return "prismatic";
		case b3_revoluteJoint:
			return "revolute";
		case b3_sphericalJoint:
			return "spherical";
		case b3_weldJoint:
			return "weld";
		case b3_wheelJoint:
			return "wheel";
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
//
// The UI is spread across three surfaces, matching Box2D's viewer:
//   - right info panel (DrawControls): Show Timeline button, divergence flag, frame counter
//   - left Outline / Detail window (DrawSampleWindows): the recorded scene tree and selection detail
//   - Timeline tab in the diagnostics drawer (DrawMetricsTab): transport, scrubber, keyframe readout
class ReplayViewer : public Sample
{
public:
	enum SelKind
	{
		kSelNone,
		kSelBody,
		kSelShape,
		kSelJoint,
	};

	explicit ReplayViewer( SampleContext* context )
		: Sample( context )
	{
		m_player = nullptr;
		m_replayWorldId = b3_nullWorldId;
		m_info = { 0 };
		m_frameCount = 0;
		m_speed = 1.0f;
		m_frameAccumulator = 0.0f;
		m_loop = true;
		m_selKind = kSelNone;
		m_selBodyOrdinal = -1;
		m_selSub = -1;
		m_selectTimelineTab = false;
		m_openLoadPopup = false;
		m_status[0] = '\0';

		if ( context->restart == false )
		{
			m_camera->SetView( 30.0f, 22.0f, 14.0f, { 0.0f, 2.0f, 0.0f } );
		}

		// Auto-play a loaded recording so the sample shows motion immediately.
		m_context->pause = false;

		// A path chosen through the Replay menu wins; opening it pops the keyframe-policy dialog.
		if ( context->replayFile[0] != '\0' )
		{
			snprintf( m_path, sizeof( m_path ), "%s", context->replayFile );
			context->replayFile[0] = '\0';
			if ( LoadFromPath() )
			{
				m_openLoadPopup = true;
			}
		}
		else
		{
			snprintf( m_path, sizeof( m_path ), "replay_demo.b3rec" );

			// Open the default recording if it already exists, so Restart (R) re-opens it. A missing
			// file is not an error here, just an invitation to generate the demo.
			if ( LoadFromPath() == false )
			{
				snprintf( m_status, sizeof( m_status ), "No recording. Generate a demo or load a .b3rec." );
			}
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
		SetHoveredBody( b3_nullBodyId );
		ClearSelection();
		m_selKind = kSelNone;
		m_selBodyOrdinal = -1;
		m_selSub = -1;
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

		// Honor the persisted keyframe policy before the ring populates.
		ApplyKeyframePolicy();

		// Wire the renderer's debug-shape callbacks into the player's world. Without them the replay
		// world has no GPU mesh handles and draws nothing. This rebuilds the world, so adopt it after.
		b3WorldDef defTemplate = b3DefaultWorldDef();
		AttachToWorldDef( &defTemplate );
		b3RecPlayer_SetDebugShapeCallbacks( m_player, defTemplate.createDebugShape, defTemplate.destroyDebugShape,
											defTemplate.userDebugShapeContext );

		m_info = b3RecPlayer_GetInfo( m_player );
		m_frameCount = m_info.frameCount;
		AdoptPlayerWorld( true );
		snprintf( m_status, sizeof( m_status ), "loaded" );
		return true;
	}

	void ApplyKeyframePolicy()
	{
		if ( m_player == nullptr )
		{
			return;
		}
		size_t budget = (size_t)m_context->replayKeyframeBudgetMB * 1024u * 1024u;
		b3RecPlayer_SetKeyframePolicy( m_player, budget, m_context->replayKeyframeMinInterval );
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

	// Point the camera at the recorded motion and pre-generate the keyframe ring, so the timeline
	// shows real spacing and backward scrubbing is fast from the first interaction. Bounds come from
	// the trailing RecordingBounds record, so no stepping is needed to frame.
	void AdoptPlayerWorld( bool frameCamera )
	{
		if ( frameCamera )
		{
			b3Vec3 extent = b3Sub( m_info.bounds.upperBound, m_info.bounds.lowerBound );
			if ( extent.x > 0.0f || extent.y > 0.0f || extent.z > 0.0f )
			{
				float aspect = m_camera->m_height > 0 ? (float)m_camera->m_width / (float)m_camera->m_height : 1.0f;
				m_camera->Frame( m_info.bounds, aspect, 1.4f );
			}
		}

		// Walk to the end and back to populate the keyframe ring under the current policy.
		b3RecPlayer_SeekFrame( m_player, m_frameCount );
		b3RecPlayer_SeekFrame( m_player, 0 );
		m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
		m_frameAccumulator = 0.0f;
	}

	// Advance one recorded step and keep the world pointer current (stable forward, refreshed cheaply).
	void AdvanceOne()
	{
		b3RecPlayer_StepFrame( m_player );
		m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
	}

	// Resolve the selected body ordinal to a live id at the current frame, or null if it is gone.
	b3BodyId SelectedBody() const
	{
		if ( m_player == nullptr || m_selKind == kSelNone || m_selBodyOrdinal < 0 )
		{
			return b3_nullBodyId;
		}
		return b3RecPlayer_GetBodyId( m_player, m_selBodyOrdinal );
	}

	int FindOrdinal( b3BodyId id ) const
	{
		int n = b3RecPlayer_GetBodyCount( m_player );
		for ( int i = 0; i < n; ++i )
		{
			if ( B3_ID_EQUALS( b3RecPlayer_GetBodyId( m_player, i ), id ) )
			{
				return i;
			}
		}
		return -1;
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

		// Mirror the ordinal selection into the renderer highlight each frame.
		SetSelectedBody( SelectedBody() );

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

	// Transport row shared by the right panel fallback and the Timeline tab.
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

	// Right info panel: a compact summary. The scene tree lives in the Outline window and the
	// transport in the Timeline tab. When nothing is loaded, fall back to load and generate so the
	// viewer is usable when picked straight from the Samples menu.
	bool DrawControls() override
	{
		if ( m_player == nullptr )
		{
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
			return true;
		}

		if ( ImGui::Button( "Show Timeline" ) )
		{
			m_context->showMetrics = true;
			m_selectTimelineTab = true;
		}

		if ( b3RecPlayer_HasDiverged( m_player ) )
		{
			ImGui::TextColored( PanelColor( b3_colorRed ), "****DIVERGED****" );
		}

		ImGui::Text( "Frame %d / %d%s", b3RecPlayer_GetFrame( m_player ), m_frameCount,
					 b3RecPlayer_IsAtEnd( m_player ) ? "  (end)" : "" );
		return true;
	}

	// Left-edge Outline / Detail window plus the keyframe-policy popup.
	void DrawSampleWindows() override
	{
		DrawLoadPopup();

		if ( m_player == nullptr )
		{
			return;
		}

		float fontSize = ImGui::GetFontSize();
		float panelWidth = 22.0f * fontSize;
		float menuBarHeight = ImGui::GetFrameHeight();
		float top = menuBarHeight + 0.5f * fontSize;

		// Stop above the diagnostics drawer when it is open so the panels do not overlap. The 16 em
		// drawer height mirrors DrawMetrics.
		float bottom = m_context->showMetrics ? ( m_camera->m_height - 16.0f * fontSize - fontSize )
											  : ( m_camera->m_height - 0.5f * fontSize );

		ImGui::SetNextWindowPos( { 0.5f * fontSize, top } );
		ImGui::SetNextWindowSize( { panelWidth, bottom - top } );
		ImGui::Begin( "Outline", nullptr,
					  ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
						  ImGuiWindowFlags_NoTitleBar );

		ImGui::TextColored( PanelColor( b3_colorGoldenRod ), "Outline" );
		float avail = ImGui::GetContentRegionAvail().y;
		float detailHeight = 0.42f * avail;
		ImGui::BeginChild( "OutlineTree", { 0.0f, avail - detailHeight } );
		DrawOutlineTree();
		ImGui::EndChild();

		ImGui::Separator();
		ImGui::TextColored( PanelColor( b3_colorGoldenRod ), "Detail" );
		ImGui::BeginChild( "DetailPane", { 0.0f, 0.0f } );
		DrawDetail();
		ImGui::EndChild();

		ImGui::End();
	}

	// Recorded scene tree: bodies by creation ordinal, expandable to their shapes and joints. Holes
	// (destroyed bodies) keep their ordinal so a stored selection stays put across the playthrough.
	void DrawOutlineTree()
	{
		int bodyCount = b3RecPlayer_GetBodyCount( m_player );
		for ( int i = 0; i < bodyCount; ++i )
		{
			b3BodyId id = b3RecPlayer_GetBodyId( m_player, i );
			if ( B3_IS_NULL( id ) || b3Body_IsValid( id ) == false )
			{
				ImGui::TextDisabled( "Body %d  (destroyed)", i );
				continue;
			}

			ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAvailWidth;
			if ( m_selKind == kSelBody && m_selBodyOrdinal == i )
			{
				flags |= ImGuiTreeNodeFlags_Selected;
			}

			bool open = ImGui::TreeNodeEx( (void*)(intptr_t)i, flags, "Body %d  %s", i,
										   ReplayBodyTypeName( b3Body_GetType( id ) ) );
			if ( ImGui::IsItemClicked() && ImGui::IsItemToggledOpen() == false )
			{
				m_selKind = kSelBody;
				m_selBodyOrdinal = i;
				m_selSub = -1;
			}

			if ( open )
			{
				b3ShapeId shapes[16];
				int shapeCount = b3Body_GetShapes( id, shapes, 16 );
				for ( int s = 0; s < shapeCount; ++s )
				{
					ImGuiTreeNodeFlags leaf =
						ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_NoTreePushOnOpen;
					if ( m_selKind == kSelShape && m_selBodyOrdinal == i && m_selSub == s )
					{
						leaf |= ImGuiTreeNodeFlags_Selected;
					}
					ImGui::TreeNodeEx( (void*)(intptr_t)( 0x1000 + s ), leaf, "Shape %d  %s", s,
									   ReplayShapeTypeName( b3Shape_GetType( shapes[s] ) ) );
					if ( ImGui::IsItemClicked() )
					{
						m_selKind = kSelShape;
						m_selBodyOrdinal = i;
						m_selSub = s;
					}
				}

				b3JointId joints[16];
				int jointCount = b3Body_GetJoints( id, joints, 16 );
				for ( int j = 0; j < jointCount; ++j )
				{
					ImGuiTreeNodeFlags leaf =
						ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_NoTreePushOnOpen;
					if ( m_selKind == kSelJoint && m_selBodyOrdinal == i && m_selSub == j )
					{
						leaf |= ImGuiTreeNodeFlags_Selected;
					}
					ImGui::TreeNodeEx( (void*)(intptr_t)( 0x2000 + j ), leaf, "Joint %d  %s", j,
									   ReplayJointTypeName( b3Joint_GetType( joints[j] ) ) );
					if ( ImGui::IsItemClicked() )
					{
						m_selKind = kSelJoint;
						m_selBodyOrdinal = i;
						m_selSub = j;
					}
				}

				ImGui::TreePop();
			}
		}
	}

	// Detail for the current selection, resolved to live ids each frame. With nothing selected, show
	// the world summary.
	void DrawDetail()
	{
		b3BodyId body = SelectedBody();
		if ( m_selKind == kSelNone || B3_IS_NULL( body ) || b3Body_IsValid( body ) == false )
		{
			b3Vec3 g = b3World_GetGravity( m_replayWorldId );
			b3Counters c = b3World_GetCounters( m_replayWorldId );
			ImGui::Text( "gravity (%.2f, %.2f, %.2f)", g.x, g.y, g.z );
			ImGui::Text( "bodies %d  shapes %d", c.bodyCount, c.shapeCount );
			ImGui::Text( "contacts %d  joints %d", c.contactCount, c.jointCount );
			ImGui::Spacing();
			ImGui::TextDisabled( "Click a node, or a shape in the view." );
			return;
		}

		if ( m_selKind == kSelShape )
		{
			b3ShapeId shapes[16];
			int shapeCount = b3Body_GetShapes( body, shapes, 16 );
			if ( m_selSub >= 0 && m_selSub < shapeCount )
			{
				b3ShapeId s = shapes[m_selSub];
				b3Filter filter = b3Shape_GetFilter( s );
				ImGui::Text( "shape %d  %s", m_selSub, ReplayShapeTypeName( b3Shape_GetType( s ) ) );
				ImGui::Text( "friction %.2f  restitution %.2f", b3Shape_GetFriction( s ), b3Shape_GetRestitution( s ) );
				ImGui::Text( "density %.2f  sensor %s", b3Shape_GetDensity( s ), b3Shape_IsSensor( s ) ? "yes" : "no" );
				ImGui::Text( "category 0x%llx", (unsigned long long)filter.categoryBits );
				ImGui::Text( "mask     0x%llx", (unsigned long long)filter.maskBits );
				return;
			}
		}

		if ( m_selKind == kSelJoint )
		{
			b3JointId joints[16];
			int jointCount = b3Body_GetJoints( body, joints, 16 );
			if ( m_selSub >= 0 && m_selSub < jointCount )
			{
				b3JointId joint = joints[m_selSub];
				b3BodyId a = b3Joint_GetBodyA( joint );
				b3BodyId b = b3Joint_GetBodyB( joint );
				ImGui::Text( "joint %d  %s", m_selSub, ReplayJointTypeName( b3Joint_GetType( joint ) ) );
				ImGui::Text( "bodies %d, %d", a.index1, b.index1 );
				ImGui::Text( "collide %s", b3Joint_GetCollideConnected( joint ) ? "yes" : "no" );
				return;
			}
		}

		// Body detail (also the fallback when a shape/joint sub-index went stale).
		const char* name = b3Body_GetName( body );
		b3WorldTransform xf = b3Body_GetTransform( body );
		b3Vec3 v = b3Body_GetLinearVelocity( body );
		float spin;
		b3GetAxisAngle( &spin, xf.q );

		ImGui::Text( "body %d  %s", body.index1,
					 ( name != nullptr && name[0] != '\0' ) ? name : ReplayBodyTypeName( b3Body_GetType( body ) ) );
		ImGui::Text( "pos   %.2f %.2f %.2f", xf.p.x, xf.p.y, xf.p.z );
		ImGui::Text( "spin  %.0f deg", spin * B3_RAD_TO_DEG );
		ImGui::Text( "vel   %.2f %.2f %.2f", v.x, v.y, v.z );
		ImGui::Text( "speed %.2f  omega %.2f", b3Length( v ), b3Length( b3Body_GetAngularVelocity( body ) ) );
		ImGui::Text( "mass  %.3g  awake %s", b3Body_GetMass( body ), b3Body_IsAwake( body ) ? "yes" : "no" );
		ImGui::Text( "shapes %d  joints %d", b3Body_GetShapeCount( body ), b3Body_GetJointCount( body ) );
	}

	// Timeline tab in the diagnostics drawer: file, transport, keyframe readout, scrubber, divergence.
	void DrawMetricsTab() override
	{
		ImGuiTabItemFlags flags = m_selectTimelineTab ? ImGuiTabItemFlags_SetSelected : 0;
		m_selectTimelineTab = false;
		if ( ImGui::BeginTabItem( "Timeline", nullptr, flags ) == false )
		{
			return;
		}

		if ( m_player == nullptr )
		{
			ImGui::TextDisabled( "No recording loaded. Use Replay > Open or the info panel." );
			ImGui::EndTabItem();
			return;
		}

		float fontSize = ImGui::GetFontSize();

		ImGui::Text( "File: %s  %s", m_path, m_status );

		DrawTransport();

		ImGui::SameLine();
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
		ImGui::SameLine();
		ImGui::Text( "%d Workers", m_info.workerCount );

		double mb = (double)b3RecPlayer_GetKeyframeBytes( m_player ) / ( 1024.0 * 1024.0 );
		ImGui::TextDisabled( "keyframe spacing %d frames, %.1f MB", b3RecPlayer_GetKeyframeInterval( m_player ), mb );

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

		float hz = m_info.timeStep > 0.0f ? 1.0f / m_info.timeStep : 0.0f;
		b3Counters c = b3World_GetCounters( m_replayWorldId );
		ImGui::Text( "frames %d", m_frameCount );
		ImGui::SameLine();
		ImGui::Text( "   %.0f hz, %d sub-steps", hz, m_info.subStepCount );
		ImGui::SameLine();
		ImGui::Text( "   bodies %d  shapes %d  contacts %d  joints %d", c.bodyCount, c.shapeCount, c.contactCount,
					 c.jointCount );

		int divergeFrame = b3RecPlayer_GetDivergeFrame( m_player );
		if ( divergeFrame >= 0 )
		{
			ImGui::SameLine();
			ImGui::TextColored( PanelColor( b3_colorRed ), "   diverged at frame %d", divergeFrame );
		}

		ImGui::EndTabItem();
	}

	// Modal that tunes the keyframe ring on open. Generation is synchronous: the recordings the
	// viewer handles are small enough that a walk to the end is instant.
	void DrawLoadPopup()
	{
		if ( m_openLoadPopup )
		{
			ImGui::OpenPopup( "Replay Load" );
			m_openLoadPopup = false;
		}

		if ( ImGui::BeginPopupModal( "Replay Load", nullptr, ImGuiWindowFlags_AlwaysAutoResize ) == false )
		{
			return;
		}

		ImGui::Text( "%s", m_path );
		ImGui::Separator();
		ImGui::SliderInt( "Memory budget (MB)", &m_context->replayKeyframeBudgetMB, 64, 4096 );
		ImGui::SliderInt( "Min sample interval", &m_context->replayKeyframeMinInterval, 4, 120 );

		if ( m_player != nullptr )
		{
			double mb = (double)b3RecPlayer_GetKeyframeBytes( m_player ) / ( 1024.0 * 1024.0 );
			ImGui::TextDisabled( "keyframe spacing %d frames, %.1f MB", b3RecPlayer_GetKeyframeInterval( m_player ), mb );
		}

		if ( ImGui::Button( "Apply" ) && m_player != nullptr )
		{
			int frame = b3RecPlayer_GetFrame( m_player );
			ApplyKeyframePolicy();
			// Repopulate the ring under the new policy, then return to the current frame.
			b3RecPlayer_SeekFrame( m_player, m_frameCount );
			b3RecPlayer_SeekFrame( m_player, frame );
			m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
		}
		ImGui::SameLine();
		if ( ImGui::Button( "Close" ) )
		{
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}

	// World-space overlays for the selection: body axes plus live contact points and normals, the
	// most useful solver readout. Drawn straight to the overlay, the world is never touched.
	void DrawSelectionOverlay()
	{
		b3BodyId body = SelectedBody();
		if ( B3_IS_NULL( body ) || b3Body_IsValid( body ) == false )
		{
			return;
		}

		b3WorldTransform xf = b3Body_GetTransform( body );
		DrawAxes( xf, 0.75f );

		b3ContactData contacts[32];
		int capacity = b3Body_GetContactCapacity( body );
		if ( capacity > 32 )
		{
			capacity = 32;
		}
		int count = b3Body_GetContactData( body, contacts, capacity );
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

	// Left click selects a body to inspect by its creation ordinal, so the selection survives a
	// backward seek that rebuilds the world. Picking only reads the world, it never creates the drag
	// joint the base sample does, so the replay is not mutated.
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
			int ordinal = FindOrdinal( b3Shape_GetBody( result.shapeId ) );
			if ( ordinal >= 0 )
			{
				m_selKind = kSelBody;
				m_selBodyOrdinal = ordinal;
				m_selSub = -1;
				return;
			}
		}

		m_selKind = kSelNone;
		m_selBodyOrdinal = -1;
		m_selSub = -1;
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
	b3RecPlayerInfo m_info;    // cached at load for the timeline readout and camera framing
	char m_path[256];
	char m_status[128];
	int m_frameCount;
	float m_speed;
	float m_frameAccumulator;
	bool m_loop;

	// Selection by creation ordinal so it survives a backward seek that rebuilds the world.
	SelKind m_selKind;
	int m_selBodyOrdinal;
	int m_selSub; // shape or joint index within the selected body

	bool m_selectTimelineTab; // one-shot: focus the Timeline tab on the next draw
	bool m_openLoadPopup;     // one-shot: open the keyframe-policy modal
};

static int sampleReplayViewer = RegisterReplay( "Replay", "Viewer", ReplayViewer::Create );
