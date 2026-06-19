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
		SelNone,
		SelBody,
		SelShape,
		SelJoint,
	};

	explicit ReplayViewer( SampleContext* context )
		: Sample( context )
	{
		m_player = nullptr;
		m_replayWorldId = b3_nullWorldId;
		m_info = { 0 };
		m_speed = 1.0f;
		m_frameAccumulator = 0.0f;
		m_loop = false;
		m_selKind = SelNone;
		m_selBodyOrdinal = -1;
		m_selSlot = -1;
		m_revealSelection = false;
		m_requestLoadPopup = false;
		m_generating = false;
		m_popupBudgetMB = m_context->replayKeyframeBudgetMB;
		m_popupMinInterval = m_context->replayKeyframeMinInterval;
		m_status[0] = '\0';

		if ( context->restart == false )
		{
			m_camera->SetView( 30.0f, 22.0f, 14.0f, { 0.0f, 2.0f, 0.0f } );
		}

		// The timeline scrubber lives in the diagnostics drawer, so open it and start paused.
		m_prevShowMetrics = m_context->showMetrics;
		m_context->showMetrics = true;
		m_context->pause = true;
		m_selectTimelineTab = true;

		snprintf( m_path, sizeof( m_path ), "%s", m_context->replayFile );

		// A fresh open gathers the keyframe policy through the Load popup, then pre-generates every
		// keyframe behind a progress bar. A restart reuses the persisted policy and fills the ring
		// lazily so R stays quick.
		if ( context->restart == false )
		{
			if ( strlen( m_path ) > 0 )
			{
				m_requestLoadPopup = true;
			}
			else
			{
				snprintf( m_status, sizeof( m_status ), "Open recording from Replay menu" );
			}
		}
		else
		{
			CreatePlayer();
		}
	}

	~ReplayViewer() override
	{
		// Runs before the base destructor, which destroys the (empty) base world and resets the
		// debug-shape pool. Destroying the player here releases the replay world's pool entries first.
		ClosePlayer();
		m_context->showMetrics = m_prevShowMetrics;
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
		SetSelectedBody( b3_nullBodyId );
		m_selKind = SelNone;
		m_selBodyOrdinal = -1;
		m_selSlot = -1;
	}

	// Load m_path into a fresh player and adopt its world. Sets m_status on any failure: missing
	// file, bad header, or deserialization error. Pre-generation of the keyframe ring is done by the
	// Load popup, not here, so the open stays responsive.
	void CreatePlayer()
	{
		ClosePlayer();

		if ( strlen( m_path ) == 0 )
		{
			snprintf( m_status, sizeof( m_status ), "Open recording from Replay menu" );
			return;
		}

		// The player copies the bytes, so the recording can be freed right away. Replaying at the host
		// worker count makes the per-step StateHash check double as a cross-thread determinism test.
		b3Recording* recording = b3LoadRecordingFromFile( m_path );
		if ( recording != nullptr )
		{
			m_player = b3RecPlayer_Create( b3Recording_GetData( recording ), b3Recording_GetSize( recording ),
										   m_context->workerCount );
			b3DestroyRecording( recording );
		}
		else
		{
			m_player = nullptr;
		}

		m_frameAccumulator = 0.0f;

		if ( m_player == nullptr )
		{
			m_info = b3RecPlayerInfo{};
			snprintf( m_status, sizeof( m_status ), "failed to open file" );
			return;
		}

		// Honor the persisted keyframe policy before any stepping captures keyframes.
		size_t bytes = (size_t)m_context->replayKeyframeBudgetMB * 1024u * 1024u;
		b3RecPlayer_SetKeyframePolicy( m_player, bytes, m_context->replayKeyframeMinInterval );

		// Wire the renderer's debug-shape callbacks into the player's world. Without them the replay
		// world has no GPU mesh handles and draws nothing. This rebuilds the world and rewinds to
		// frame 0, so adopt the world id afterward.
		b3WorldDef defTemplate = b3DefaultWorldDef();
		AttachToWorldDef( &defTemplate );
		b3RecPlayer_SetDebugShapeCallbacks( m_player, defTemplate.createDebugShape, defTemplate.destroyDebugShape,
											defTemplate.userDebugShapeContext );

		m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
		m_info = b3RecPlayer_GetInfo( m_player );
		snprintf( m_status, sizeof( m_status ), "loaded" );

		// Frame the recorded motion. Bounds come from the trailing RecordingBounds record; an empty
		// extent means an older recording, so leave the default view.
		if ( m_context->restart == false )
		{
			b3Vec3 extent = b3Sub( m_info.bounds.upperBound, m_info.bounds.lowerBound );
			if ( extent.x > 0.0f || extent.y > 0.0f || extent.z > 0.0f )
			{
				float aspect = m_camera->m_height > 0 ? (float)m_camera->m_width / (float)m_camera->m_height : 1.0f;
				m_camera->Frame( m_info.bounds, aspect, 1.4f );
			}
		}
	}

	// Modal shown after the Replay menu picks a file: choose the keyframe budget and min interval,
	// then Load creates the player and pre-generates the whole ring behind a progress bar. Drawn from
	// DrawSampleWindows, which runs inside the imgui frame (Step does not in Box3D).
	void DrawLoadPopup()
	{
		const char* popupId = "Load Replay";

		if ( m_requestLoadPopup )
		{
			m_requestLoadPopup = false;
			m_popupBudgetMB = m_context->replayKeyframeBudgetMB;
			m_popupMinInterval = m_context->replayKeyframeMinInterval;
			ImGui::OpenPopup( popupId );
		}

		float fontSize = ImGui::GetFontSize();
		ImGui::SetNextWindowPos( { m_camera->m_width * 0.5f, m_camera->m_height * 0.35f }, ImGuiCond_Appearing,
								 { 0.5f, 0.5f } );
		ImGui::SetNextWindowSize( { 26.0f * fontSize, 0.0f }, ImGuiCond_Appearing );

		if ( ImGui::BeginPopupModal( popupId, nullptr, ImGuiWindowFlags_AlwaysAutoResize ) == false )
		{
			return;
		}

		// Show just the file name, paths run long.
		const char* slash = strrchr( m_path, '\\' );
		ImGui::TextDisabled( "File:" );
		ImGui::SameLine();
		ImGui::TextUnformatted( slash != nullptr ? slash + 1 : m_path );
		ImGui::Separator();

		if ( m_generating )
		{
			// Step forward in wall-clock slices so the bar animates. Forward stepping captures
			// keyframes at the interval; a restart then returns to frame 0 with the ring kept.
			uint64_t ticks = b3GetTicks();
			while ( b3RecPlayer_IsAtEnd( m_player ) == false && b3GetMilliseconds( ticks ) < 12.0f )
			{
				b3RecPlayer_StepFrame( m_player );
			}

			int frame = b3RecPlayer_GetFrame( m_player );
			int total = m_info.frameCount > 0 ? m_info.frameCount : 1;
			float frac = frame >= total ? 1.0f : (float)frame / (float)total;
			char overlay[32];
			snprintf( overlay, sizeof( overlay ), "%d / %d", frame, m_info.frameCount );
			ImGui::TextUnformatted( "Generating keyframes" );
			ImGui::ProgressBar( frac, ImVec2( -FLT_MIN, 0.0f ), overlay );

			if ( b3RecPlayer_IsAtEnd( m_player ) )
			{
				b3RecPlayer_Restart( m_player );
				m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
				m_generating = false;
				m_context->pause = true;
				ImGui::CloseCurrentPopup();
			}

			ImGui::EndPopup();
			return;
		}

		ImGui::PushItemWidth( 10.0f * fontSize );
		ImGui::SliderInt( "Memory budget (MB)", &m_popupBudgetMB, 128, 4096 );
		ImGui::SliderInt( "Min sample interval", &m_popupMinInterval, 8, 60 );
		ImGui::PopItemWidth();

		// Surface a failed open inline so Load can be retried.
		if ( m_status[0] != '\0' && strcmp( m_status, "loaded" ) != 0 )
		{
			ImGui::TextColored( PanelColor( b3_colorRed ), "%s", m_status );
		}

		ImGui::Separator();
		if ( ImGui::Button( "Load" ) )
		{
			// Commit the choices so they persist, build the player under that policy, then start
			// pre-generation. An empty recording has nothing to generate.
			m_context->replayKeyframeBudgetMB = m_popupBudgetMB;
			m_context->replayKeyframeMinInterval = m_popupMinInterval;
			CreatePlayer();
			if ( m_player != nullptr )
			{
				m_generating = m_info.frameCount > 0;
				if ( m_generating == false )
				{
					ImGui::CloseCurrentPopup();
				}
			}
		}
		ImGui::SameLine();
		if ( ImGui::Button( "Cancel" ) )
		{
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}

	// Advance one recorded step and keep the world pointer current (stable forward, refreshed cheaply).
	void AdvanceOne()
	{
		b3RecPlayer_StepFrame( m_player );
		m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
	}

	void Step() override
	{
		// Generation runs inside the imgui frame (DrawLoadPopup). While it fast-forwards, the world is
		// mid-replay, so hold off drawing it.
		if ( m_generating )
		{
			m_stepCount = m_player != nullptr ? b3RecPlayer_GetFrame( m_player ) : 0;
			SetDrawOrigin( m_camera->m_worldEye );
			return;
		}

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
							b3RecPlayer_Restart( m_player );
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

		DrawSelectionHighlight();
	}

	// A replay re-runs recorded inputs, so the live solver sliders would do nothing. This also hides
	// the Solver and Recording sections in the right info panel.
	bool HasSolverControls() const override
	{
		return false;
	}

	// Transport row shared by the right panel and the Timeline tab. Play is green, Pause red.
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
			ImGui::PushStyleColor( ImGuiCol_Button, (ImVec4)ImColor::HSV( 2.0f / 7.0f, 0.6f, 0.6f ) );
			ImGui::PushStyleColor( ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV( 2.0f / 7.0f, 0.7f, 0.7f ) );
			ImGui::PushStyleColor( ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV( 2.0f / 7.0f, 0.8f, 0.8f ) );
			if ( ImGui::Button( "Play " ) )
			{
				m_context->pause = false;
			}
			ImGui::PopStyleColor( 3 );
		}
		else
		{
			ImGui::PushStyleColor( ImGuiCol_Button, (ImVec4)ImColor::HSV( 1.0f / 7.0f, 0.6f, 0.6f ) );
			ImGui::PushStyleColor( ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV( 1.0f / 7.0f, 0.7f, 0.7f ) );
			ImGui::PushStyleColor( ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV( 1.0f / 7.0f, 0.8f, 0.8f ) );
			if ( ImGui::Button( "Pause" ) )
			{
				m_context->pause = true;
			}
			ImGui::PopStyleColor( 3 );
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
			b3RecPlayer_SeekFrame( m_player, m_info.frameCount );
			m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
			m_frameAccumulator = 0.0f;
		}
	}

	// Right info panel: a compact summary. The scene tree lives in the Outline window and the
	// transport in the Timeline tab.
	bool DrawControls() override
	{
		if ( m_player == nullptr )
		{
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

		ImGui::TextDisabled( "Frame %d / %d%s", b3RecPlayer_GetFrame( m_player ), m_info.frameCount,
							 b3RecPlayer_IsAtEnd( m_player ) ? "  (end)" : "" );
		return true;
	}

	// Selection resolution. The selection is stored as creation ordinals so it survives a backward
	// seek that rebuilds the world. Each frame the ordinal maps back to a live id, or to null when
	// that object does not exist at the current frame.
	b3BodyId SelectedBody() const
	{
		if ( m_player == nullptr || m_selBodyOrdinal < 0 )
		{
			return b3_nullBodyId;
		}
		return b3RecPlayer_GetBodyId( m_player, m_selBodyOrdinal );
	}

	b3ShapeId SelectedShape() const
	{
		b3BodyId body = SelectedBody();
		if ( m_selKind != SelShape || b3Body_IsValid( body ) == false )
		{
			return b3_nullShapeId;
		}
		b3ShapeId shapes[32];
		int n = b3Body_GetShapes( body, shapes, 32 );
		return ( m_selSlot >= 0 && m_selSlot < n ) ? shapes[m_selSlot] : b3_nullShapeId;
	}

	b3JointId SelectedJoint() const
	{
		b3BodyId body = SelectedBody();
		if ( m_selKind != SelJoint || b3Body_IsValid( body ) == false )
		{
			return b3_nullJointId;
		}
		b3JointId joints[16];
		int n = b3Body_GetJoints( body, joints, 16 );
		return ( m_selSlot >= 0 && m_selSlot < n ) ? joints[m_selSlot] : b3_nullJointId;
	}

	int FindBodyOrdinal( b3BodyId body ) const
	{
		int count = b3RecPlayer_GetBodyCount( m_player );
		for ( int i = 0; i < count; ++i )
		{
			if ( B3_ID_EQUALS( b3RecPlayer_GetBodyId( m_player, i ), body ) )
			{
				return i;
			}
		}
		return -1;
	}

	// Map a picked shape back to its body ordinal and shape slot. A null shape clears the selection.
	void SelectShape( b3ShapeId shape )
	{
		if ( B3_IS_NULL( shape ) )
		{
			m_selKind = SelNone;
			return;
		}
		b3BodyId body = b3Shape_GetBody( shape );
		int ordinal = FindBodyOrdinal( body );
		if ( ordinal < 0 )
		{
			m_selKind = SelNone;
			return;
		}
		b3ShapeId shapes[32];
		int n = b3Body_GetShapes( body, shapes, 32 );
		int slot = -1;
		for ( int i = 0; i < n; ++i )
		{
			if ( B3_ID_EQUALS( shapes[i], shape ) )
			{
				slot = i;
				break;
			}
		}
		m_selKind = SelShape;
		m_selBodyOrdinal = ordinal;
		m_selSlot = slot;
		m_revealSelection = true; // expand and scroll the tree to the picked shape next draw
	}

	// World-space overlay: a body's live contact points and normals, the most useful solver readout.
	void DrawBodyContacts( b3BodyId body )
	{
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

	// Highlight the current selection without touching the world: AABB box, body axes, center of
	// mass, and contacts. Joints mark both connected body centers.
	void DrawSelectionHighlight()
	{
		if ( m_selKind == SelShape )
		{
			b3ShapeId shape = SelectedShape();
			if ( b3Shape_IsValid( shape ) == false )
			{
				return;
			}
			b3BodyId body = b3Shape_GetBody( shape );
			DrawBounds( b3Shape_GetAABB( shape ), 0.0f, MakeColor( b3_colorYellow ) );
			DrawAxes( b3Body_GetTransform( body ), 0.5f );
			DrawPoint( b3Body_GetWorldCenterOfMass( body ), 8.0f, MakeColor( b3_colorYellow ) );
			DrawBodyContacts( body );
		}
		else if ( m_selKind == SelBody )
		{
			b3BodyId body = SelectedBody();
			if ( b3Body_IsValid( body ) == false )
			{
				return;
			}
			DrawBounds( b3Body_ComputeAABB( body ), 0.0f, MakeColor( b3_colorYellow ) );
			DrawAxes( b3Body_GetTransform( body ), 0.5f );
			DrawPoint( b3Body_GetWorldCenterOfMass( body ), 8.0f, MakeColor( b3_colorYellow ) );
			DrawBodyContacts( body );
		}
		else if ( m_selKind == SelJoint )
		{
			b3JointId joint = SelectedJoint();
			if ( b3Joint_IsValid( joint ) == false )
			{
				return;
			}
			b3BodyId a = b3Joint_GetBodyA( joint );
			b3BodyId b = b3Joint_GetBodyB( joint );
			if ( b3Body_IsValid( a ) )
			{
				DrawPoint( b3Body_GetWorldCenterOfMass( a ), 8.0f, MakeColor( b3_colorMagenta ) );
			}
			if ( b3Body_IsValid( b ) )
			{
				DrawPoint( b3Body_GetWorldCenterOfMass( b ), 8.0f, MakeColor( b3_colorMagenta ) );
			}
		}
	}

	// Left-edge Outline / Detail window plus the keyframe-policy popup.
	void DrawSampleWindows() override
	{
		DrawLoadPopup();

		if ( m_player == nullptr || m_generating )
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
		ImGui::BeginChild( "OutlineTree", { 0.0f, 0.55f * avail } );
		DrawOutlineTree();
		ImGui::EndChild();

		ImGui::Separator();
		ImGui::TextColored( PanelColor( b3_colorGoldenRod ), "Detail" );
		ImGui::BeginChild( "DetailPane", { 0.0f, 0.0f } );
		DrawDetail();
		ImGui::EndChild();

		ImGui::End();
	}

	// Recorded scene tree: bodies by creation ordinal, expandable to their shapes and joints.
	// Destroyed bodies keep their ordinal but are not shown, so a stored selection stays put.
	void DrawOutlineTree()
	{
		// A viewport pick asks the tree to reveal its target once: expand the owning body and scroll
		// to the row. Consumed at the end so it never fights the user's own expand/collapse.
		bool reveal = m_revealSelection;

		int count = b3RecPlayer_GetBodyCount( m_player );
		for ( int ord = 0; ord < count; ++ord )
		{
			b3BodyId body = b3RecPlayer_GetBodyId( m_player, ord );
			if ( B3_IS_NULL( body ) || b3Body_IsValid( body ) == false )
			{
				continue;
			}

			bool ownsSelection =
				m_selBodyOrdinal == ord && ( m_selKind == SelBody || m_selKind == SelShape || m_selKind == SelJoint );

			const char* name = b3Body_GetName( body );
			char label[64];
			snprintf( label, sizeof( label ), "Body %d  %s###b%d", ord,
					  ( name != nullptr && name[0] != '\0' ) ? name : ReplayBodyTypeName( b3Body_GetType( body ) ), ord );

			ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAvailWidth;
			if ( m_selKind == SelBody && m_selBodyOrdinal == ord )
			{
				flags |= ImGuiTreeNodeFlags_Selected;
			}
			// Reveal a picked shape or joint by expanding its body.
			if ( reveal && ownsSelection && m_selKind != SelBody )
			{
				ImGui::SetNextItemOpen( true );
			}
			bool open = ImGui::TreeNodeEx( label, flags );
			if ( reveal && ownsSelection && m_selKind == SelBody )
			{
				ImGui::SetScrollHereY( 0.5f );
			}
			if ( ImGui::IsItemClicked() && ImGui::IsItemToggledOpen() == false )
			{
				m_selKind = SelBody;
				m_selBodyOrdinal = ord;
				m_selSlot = -1;
			}
			if ( open == false )
			{
				continue;
			}

			b3ShapeId shapes[32];
			int sn = b3Body_GetShapes( body, shapes, 32 );
			for ( int s = 0; s < sn; ++s )
			{
				char sl[64];
				snprintf( sl, sizeof( sl ), "Shape %d  %s###b%ds%d", s, ReplayShapeTypeName( b3Shape_GetType( shapes[s] ) ),
						  ord, s );
				ImGuiTreeNodeFlags lf =
					ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_NoTreePushOnOpen;
				if ( m_selKind == SelShape && m_selBodyOrdinal == ord && m_selSlot == s )
				{
					lf |= ImGuiTreeNodeFlags_Selected;
				}
				ImGui::TreeNodeEx( sl, lf );
				if ( reveal && m_selKind == SelShape && m_selBodyOrdinal == ord && m_selSlot == s )
				{
					ImGui::SetScrollHereY( 0.5f );
				}
				if ( ImGui::IsItemClicked() )
				{
					m_selKind = SelShape;
					m_selBodyOrdinal = ord;
					m_selSlot = s;
				}
			}

			b3JointId joints[16];
			int jn = b3Body_GetJoints( body, joints, 16 );
			for ( int j = 0; j < jn; ++j )
			{
				char jl[64];
				snprintf( jl, sizeof( jl ), "%s joint###b%dj%d", ReplayJointTypeName( b3Joint_GetType( joints[j] ) ), ord, j );
				ImGuiTreeNodeFlags lf =
					ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_NoTreePushOnOpen;
				if ( m_selKind == SelJoint && m_selBodyOrdinal == ord && m_selSlot == j )
				{
					lf |= ImGuiTreeNodeFlags_Selected;
				}
				ImGui::TreeNodeEx( jl, lf );
				if ( ImGui::IsItemClicked() )
				{
					m_selKind = SelJoint;
					m_selBodyOrdinal = ord;
					m_selSlot = j;
				}
			}

			ImGui::TreePop();
		}

		m_revealSelection = false;
	}

	// Detail pane for the current selection, resolved to live ids each frame. With nothing selected,
	// show the world summary.
	void DrawDetail()
	{
		if ( m_selKind == SelNone )
		{
			ImGui::TextWrapped( "Click a node, or a shape in the view." );
			if ( B3_IS_NON_NULL( m_replayWorldId ) )
			{
				b3Vec3 g = b3World_GetGravity( m_replayWorldId );
				b3Counters c = b3World_GetCounters( m_replayWorldId );
				ImGui::Text( "gravity (%.2f, %.2f, %.2f)", g.x, g.y, g.z );
				ImGui::Text( "bodies %d  shapes %d", c.bodyCount, c.shapeCount );
				ImGui::Text( "contacts %d  joints %d", c.contactCount, c.jointCount );
			}
			return;
		}

		b3BodyId body = SelectedBody();
		if ( b3Body_IsValid( body ) == false )
		{
			ImGui::TextDisabled( "Not present at this frame." );
			return;
		}

		DrawBodyDetail( body );
		if ( m_selKind == SelShape )
		{
			b3ShapeId shape = SelectedShape();
			if ( b3Shape_IsValid( shape ) )
			{
				DrawShapeDetail( shape );
			}
		}
		else if ( m_selKind == SelJoint )
		{
			b3JointId joint = SelectedJoint();
			if ( b3Joint_IsValid( joint ) )
			{
				DrawJointDetail( joint );
			}
		}
		DrawContactDetail( body );
	}

	void DrawBodyDetail( b3BodyId body )
	{
		if ( ImGui::CollapsingHeader( "Body", ImGuiTreeNodeFlags_DefaultOpen ) == false )
		{
			return;
		}

		const char* name = b3Body_GetName( body );
		b3WorldTransform xf = b3Body_GetTransform( body );
		b3Vec3 v = b3Body_GetLinearVelocity( body );
		b3Vec3 w = b3Body_GetAngularVelocity( body );
		float spin;
		b3GetAxisAngle( &spin, xf.q );

		ImGui::Text( "id      %d", body.index1 );
		ImGui::Text( "name    %s", ( name != nullptr && name[0] != '\0' ) ? name : "(none)" );
		ImGui::Text( "type    %s", ReplayBodyTypeName( b3Body_GetType( body ) ) );
		ImGui::Text( "pos     (%.3f, %.3f, %.3f)", xf.p.x, xf.p.y, xf.p.z );
		ImGui::Text( "spin    %.1f deg", spin * B3_RAD_TO_DEG );
		ImGui::Text( "vel     (%.3f, %.3f, %.3f)", v.x, v.y, v.z );
		ImGui::Text( "omega   (%.3f, %.3f, %.3f)", w.x, w.y, w.z );
		ImGui::Text( "speed   %.3f  spin rate %.3f", b3Length( v ), b3Length( w ) );
		ImGui::Text( "mass    %.4g kg", b3Body_GetMass( body ) );
		ImGui::Text( "awake   %s", b3Body_IsAwake( body ) ? "yes" : "no" );
		ImGui::Text( "enabled %s", b3Body_IsEnabled( body ) ? "yes" : "no" );
		ImGui::Text( "bullet  %s", b3Body_IsBullet( body ) ? "yes" : "no" );
		ImGui::Text( "gravity scale %.2f", b3Body_GetGravityScale( body ) );
		ImGui::Text( "shapes %d  joints %d", b3Body_GetShapeCount( body ), b3Body_GetJointCount( body ) );
	}

	void DrawShapeDetail( b3ShapeId shape )
	{
		if ( ImGui::CollapsingHeader( "Shape", ImGuiTreeNodeFlags_DefaultOpen ) == false )
		{
			return;
		}

		ImGui::Text( "id      %d", shape.index1 );
		ImGui::Text( "type     %s", ReplayShapeTypeName( b3Shape_GetType( shape ) ) );
		b3Filter f = b3Shape_GetFilter( shape );
		ImGui::Text( "category 0x%016llx", (unsigned long long)f.categoryBits );
		ImGui::Text( "mask     0x%016llx", (unsigned long long)f.maskBits );
		ImGui::Text( "group    %d", f.groupIndex );
		ImGui::Text( "density  %.3g", b3Shape_GetDensity( shape ) );
		ImGui::Text( "friction %.3g", b3Shape_GetFriction( shape ) );
		ImGui::Text( "restitution %.3g", b3Shape_GetRestitution( shape ) );
		ImGui::Text( "sensor   %s", b3Shape_IsSensor( shape ) ? "yes" : "no" );
		b3SurfaceMaterial mat = b3Shape_GetSurfaceMaterial( shape );
		ImGui::Text( "custom color 0x%06x", (unsigned)mat.customColor );
		b3AABB aabb = b3Shape_GetAABB( shape );
		ImGui::Text( "aabb (%.2f, %.2f, %.2f)", aabb.lowerBound.x, aabb.lowerBound.y, aabb.lowerBound.z );
		ImGui::Text( "     (%.2f, %.2f, %.2f)", aabb.upperBound.x, aabb.upperBound.y, aabb.upperBound.z );
	}

	void DrawContactDetail( b3BodyId body )
	{
		b3ContactData contacts[32];
		int capacity = b3Body_GetContactCapacity( body );
		if ( capacity > 32 )
		{
			capacity = 32;
		}
		int count = b3Body_GetContactData( body, contacts, capacity );

		char header[32];
		snprintf( header, sizeof( header ), "Contacts (%d)###contacts", count );
		if ( ImGui::CollapsingHeader( header ) == false )
		{
			return;
		}

		for ( int i = 0; i < count; ++i )
		{
			ImGui::Text( "shapes %d / %d", contacts[i].shapeIdA.index1, contacts[i].shapeIdB.index1 );
			for ( int m = 0; m < contacts[i].manifoldCount; ++m )
			{
				const b3Manifold* manifold = &contacts[i].manifolds[m];
				ImGui::Text( "normal (%.2f, %.2f, %.2f)", manifold->normal.x, manifold->normal.y, manifold->normal.z );
				ImGui::Text( "points %d", manifold->pointCount );
				for ( int p = 0; p < manifold->pointCount; ++p )
				{
					const b3ManifoldPoint* mp = &manifold->points[p];
					ImGui::Text( "  sep %.3f  Pn %.2g", mp->separation, mp->normalImpulse );
				}
			}

			ImGui::Separator();
		}
	}

	void DrawJointDetail( b3JointId joint )
	{
		if ( ImGui::CollapsingHeader( "Joint", ImGuiTreeNodeFlags_DefaultOpen ) == false )
		{
			return;
		}

		b3JointType type = b3Joint_GetType( joint );
		ImGui::Text( "type     %s", ReplayJointTypeName( type ) );
		ImGui::Text( "body A   %d", b3Joint_GetBodyA( joint ).index1 );
		ImGui::Text( "body B   %d", b3Joint_GetBodyB( joint ).index1 );
		ImGui::Text( "collide  %s", b3Joint_GetCollideConnected( joint ) ? "yes" : "no" );
		ImGui::Text( "force    %.3g", b3Length( b3Joint_GetConstraintForce( joint ) ) );
		ImGui::Text( "torque   %.3g", b3Length( b3Joint_GetConstraintTorque( joint ) ) );

		switch ( type )
		{
			case b3_revoluteJoint:
				ImGui::Text( "angle    %.1f deg", b3RevoluteJoint_GetAngle( joint ) * B3_RAD_TO_DEG );
				break;
			case b3_prismaticJoint:
				ImGui::Text( "translation %.3f", b3PrismaticJoint_GetTranslation( joint ) );
				break;
			case b3_distanceJoint:
				ImGui::Text( "length   %.3f", b3DistanceJoint_GetCurrentLength( joint ) );
				break;
			default:
				break;
		}
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

		// File row, always available so a recording can be loaded even when none is open.
		ImGui::Text( "File: %s  %s", m_path, m_status );

		if ( m_player == nullptr )
		{
			ImGui::EndTabItem();
			return;
		}

		float fontSize = ImGui::GetFontSize();

		// Transport row: buttons, speed, loop, replay worker count.
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

		// Replaying at a different worker count re-partitions the constraint graph, a visual
		// cross-thread determinism check. The setter applies it without rebuilding the world.
		ImGui::PushItemWidth( 6.0f * fontSize );
		if ( ImGui::SliderInt( "Workers", &m_context->workerCount, 1, B3_MAX_WORKERS ) )
		{
			b3RecPlayer_SetWorkerCount( m_player, m_context->workerCount );
		}
		ImGui::PopItemWidth();

		double mb = (double)b3RecPlayer_GetKeyframeBytes( m_player ) / ( 1024.0 * 1024.0 );
		ImGui::TextDisabled( "keyframe spacing %d frames, %.1f MB", b3RecPlayer_GetKeyframeInterval( m_player ), mb );

		// Scrubber seeks both directions; backward uses the keyframe ring.
		int scrub = b3RecPlayer_GetFrame( m_player );
		ImGui::PushItemWidth( -FLT_MIN );
		if ( ImGui::SliderInt( "##frame", &scrub, 0, m_info.frameCount ) )
		{
			b3RecPlayer_SeekFrame( m_player, scrub );
			m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
			m_frameAccumulator = 0.0f;
			m_context->pause = true;
		}
		ImGui::PopItemWidth();

		// Mark where the replay first diverged on the scrubber track.
		int divergeFrame = b3RecPlayer_GetDivergeFrame( m_player );
		if ( divergeFrame >= 0 && m_info.frameCount > 0 )
		{
			ImVec2 lo = ImGui::GetItemRectMin();
			ImVec2 hi = ImGui::GetItemRectMax();
			float t = (float)divergeFrame / (float)m_info.frameCount;
			float x = lo.x + t * ( hi.x - lo.x );
			ImGui::GetWindowDrawList()->AddLine( ImVec2( x, lo.y ), ImVec2( x, hi.y ), IM_COL32( 220, 60, 60, 255 ), 2.0f );
		}

		float hz = m_info.timeStep > 0.0f ? 1.0f / m_info.timeStep : 0.0f;
		b3Counters c = b3World_GetCounters( m_replayWorldId );
		ImGui::Text( "frames %d", m_info.frameCount );
		ImGui::SameLine();
		ImGui::Text( "   %.0f hz, %d sub-steps", hz, m_info.subStepCount );
		ImGui::SameLine();
		ImGui::Text( "   bodies %d  shapes %d  contacts %d  joints %d", c.bodyCount, c.shapeCount, c.contactCount,
					 c.jointCount );

		if ( divergeFrame >= 0 )
		{
			ImGui::SameLine();
			ImGui::TextColored( PanelColor( b3_colorRed ), "   diverged at frame %d", divergeFrame );
		}

		ImGui::EndTabItem();
	}

	// Left click selects a shape to inspect by its creation ordinal, so the selection survives a
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
		SelectShape( result.hit ? result.shapeId : b3_nullShapeId );
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
	float m_speed;
	float m_frameAccumulator;
	bool m_loop;

	bool m_selectTimelineTab; // one-shot: focus the Timeline tab on the next draw
	bool m_prevShowMetrics;   // restore the drawer state on exit

	// Load popup state. A fresh open configures the keyframe policy here, then the popup switches to a
	// progress bar while every keyframe is generated up front. Temporaries hold the in-popup edits so
	// Cancel leaves the persisted settings untouched.
	bool m_requestLoadPopup;
	bool m_generating;
	int m_popupBudgetMB;
	int m_popupMinInterval;

	// Selection by creation ordinal so it survives a backward seek that rebuilds the world.
	SelKind m_selKind;
	int m_selBodyOrdinal;
	int m_selSlot;            // shape or joint slot within the selected body
	bool m_revealSelection;   // one-shot: expand and scroll the tree to a viewport pick
};

static int sampleReplayViewer = RegisterReplay( "Replay", "Viewer", ReplayViewer::Create );
