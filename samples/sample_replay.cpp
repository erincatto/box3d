// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/debug_adapter.h"
#include "gfx/draw.h"
#include "gfx/text.h"
#include "imgui.h"
#include "sample.h"

#include "box3d/box3d.h"

#include <ctype.h>
#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unordered_map>
#include <vector>

// Whole-recording query index, built lazily by scanning the player once. Backs the search box.
struct QueryIndexRow
{
	int frame;
	b3RecQueryType type;
	int kindOrdinal;
	int instance; // occurrence among same-key queries in its frame
	uint64_t key;
	uint64_t id;
	const char* name; // points into the player's tag table, stable for the player's lifetime
	b3QueryFilter filter;
	b3Pos origin;
	int hitCount;
};

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

static const char* ReplayQueryTypeName( b3RecQueryType type )
{
	switch ( type )
	{
		case b3_recQueryOverlapAABB:
			return "overlap AABB";
		case b3_recQueryOverlapShape:
			return "overlap shape";
		case b3_recQueryCastRay:
			return "cast ray";
		case b3_recQueryCastShape:
			return "cast shape";
		case b3_recQueryCastRayClosest:
			return "cast ray closest";
		case b3_recQueryCastMover:
			return "cast mover";
		case b3_recQueryCollideMover:
			return "collide mover";
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

// Number of b3RecQueryType values, for per-kind bookkeeping in the outline and search.
static constexpr int ReplayQueryTypeCount = 7;

// Outline text and the 3D overlay share one color per query kind, so a row reads as the geometry it
// draws. Matches the colors b3RecPlayer_DrawFrameQueries paints with.
static b3HexColor QueryKindColor( b3RecQueryType type )
{
	switch ( type )
	{
		case b3_recQueryCastRay:
		case b3_recQueryCastRayClosest:
			return b3_colorYellow;
		case b3_recQueryCastShape:
			return b3_colorSkyBlue;
		case b3_recQueryCastMover:
			return b3_colorLightSkyBlue;
		case b3_recQueryCollideMover:
			return b3_colorTan;
		default:
			return b3_colorLimeGreen; // overlap AABB and overlap shape
	}
}

// Case-insensitive substring test for the query search box.
static bool ContainsNoCase( const char* haystack, const char* needle )
{
	if ( needle[0] == '\0' )
	{
		return true;
	}
	for ( ; *haystack != '\0'; ++haystack )
	{
		const char* h = haystack;
		const char* n = needle;
		while ( *h != '\0' && *n != '\0' && tolower( (unsigned char)*h ) == tolower( (unsigned char)*n ) )
		{
			++h;
			++n;
		}
		if ( *n == '\0' )
		{
			return true;
		}
	}
	return false;
}

// Compose a query's display label from its caller id and label, falling back to kind and per-kind
// ordinal when untagged. e.g. "bullet (53)", "bullet", "#53", or "cast ray #0".
static void FormatQueryLabel( char* out, int cap, const char* name, uint64_t id, b3RecQueryType type, int kindOrdinal )
{
	if ( name != nullptr && id != 0 )
	{
		snprintf( out, cap, "%s (%llu)", name, (unsigned long long)id );
	}
	else if ( name != nullptr )
	{
		snprintf( out, cap, "%s", name );
	}
	else if ( id != 0 )
	{
		snprintf( out, cap, "#%llu", (unsigned long long)id );
	}
	else
	{
		snprintf( out, cap, "%s #%d", ReplayQueryTypeName( type ), kindOrdinal );
	}
}

enum SelectionKind
{
	SelNone,
	SelBody,
	SelShape,
	SelJoint,
	SelQuery,
};

// Plays back a .b3rec recording by driving the keyframe player one recorded step at a time and
// drawing the replayed world. The player owns the world, so the base sample world is left empty and
// unused. Mouse picking only reads the world (no drag joint), since mutating it would diverge the
// replay from the recording.
//
// The UI is spread across three surfaces, matching Box2D's viewer:
//   - right info panel (DrawControls): Show Timeline button, view toggles, frame counter, and the
//     selection detail
//   - left Outline window (DrawSampleWindows): the recorded scene tree
//   - Timeline tab in the diagnostics drawer (DrawMetricsTab): transport, scrubber, keyframe readout
class ReplayViewer : public Sample
{
public:
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
		m_selQuery = -1;
		m_selQueryKey = 0;
		m_selQueryLabel[0] = '\0';
		m_selQueryKind = b3_recQueryOverlapAABB;
		m_selQueryKindOrdinal = -1;
		m_selQueryInstance = 0;
		m_hoverQuery = -1;
		m_revealSelection = false;
		m_drawAllQueries = false;
		m_queryIndexBuilt = false;
		m_querySearch[0] = '\0';
		m_querySearchKind = 0;
		m_querySearchMinHits = 0;
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

		// Z-up is a replay-only view choice and has no toggle elsewhere, so restore it on exit
		// rather than leak a tilted view into the next sample.
		m_prevViewZUp = m_context->viewZUp;

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
		m_context->viewZUp = m_prevViewZUp;
	}

	void ClosePlayer()
	{
		if ( m_player != nullptr )
		{
			b3RecPlayer_Destroy( m_player );
			m_player = nullptr;
		}
		m_replayWorldId = b3_nullWorldId;
		SetSelectedBody( b3_nullBodyId );
		m_selKind = SelNone;
		m_selBodyOrdinal = -1;
		m_selSlot = -1;
		m_selQuery = -1;
		m_selQueryKey = 0;
		m_selQueryLabel[0] = '\0';
		m_selQueryKindOrdinal = -1;
		m_selQueryInstance = 0;
		m_queryIndexBuilt = false;
		m_queryIndex.clear();
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
			m_player =
				b3RecPlayer_Create( b3Recording_GetData( recording ), b3Recording_GetSize( recording ), m_context->workerCount );
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

		// Frame the recorded motion on a fresh open. A restart keeps the user's current camera.
		if ( m_context->restart == false )
		{
			FrameRecording();
		}
	}

	// Fit the view to the recorded bounds for the current up axis. The player applies the recording's
	// length scale on load, so frame against the same sim->display transform the frame loop uses, or
	// the fit lands at the wrong distance. Reused on a Z-up toggle, where the transform rotates about
	// the sim origin and would otherwise swing the bounds out of view. An empty extent means an older
	// recording with no bounds record, so leave the default view.
	void FrameRecording()
	{
		if ( m_player == nullptr )
		{
			return;
		}

		b3Vec3 extent = b3Sub( m_info.bounds.upperBound, m_info.bounds.lowerBound );
		if ( extent.x > 0.0f || extent.y > 0.0f || extent.z > 0.0f )
		{
			m_camera->SetRenderTransform( b3GetLengthUnitsPerMeter(), m_context->viewZUp );
			float aspect = m_camera->m_height > 0 ? (float)m_camera->m_width / (float)m_camera->m_height : 1.0f;
			m_camera->Frame( m_info.bounds, aspect, 0.75f );
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
		ImGui::SetNextWindowPos( { m_camera->m_width * 0.5f, m_camera->m_height * 0.35f }, ImGuiCond_Appearing, { 0.5f, 0.5f } );
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
		ImGui::SliderInt( "Memory budget (MB)", &m_popupBudgetMB, 128, 8 * 1024 );
		ImGui::SliderInt( "Min sample interval", &m_popupMinInterval, 8, 256 );
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
			SetDrawOrigin( m_camera->DrawOrigin() );
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

		SetDrawOrigin( m_camera->DrawOrigin() );

		if ( B3_IS_NULL( m_replayWorldId ) )
		{
			DrawScreenStringFormat( 5, m_textLine, MakeColor( b3_colorLightGray ), "%s", m_status );
			return;
		}

		// Draw the replay world through the same adapter path the live samples use.
		b3DebugDraw debugDraw;
		MakeDebugDraw( &debugDraw );
		ApplyGuiFlags( &debugDraw );
		debugDraw.drawingBounds = m_camera->DrawBounds(); // view-distance box in length units around the eye

		// Drive the shared outline highlight from the selection. A shape selection outlines that
		// shape alone, a body selection outlines the whole body. Set before the draw so the mask
		// is filled by the draw callback.
		if ( m_selKind == SelShape )
		{
			SetSelectedShape( SelectedShape() );
		}
		else if ( m_selKind == SelBody )
		{
			SetSelectedBody( SelectedBody() );
		}
		else
		{
			ClearSelection();
		}

		b3World_Draw( m_replayWorldId, &debugDraw, B3_DEFAULT_MASK_BITS );

		DrawSelectionHighlight();

		// Overlay query geometry and recorded hits on top of the world. The toggle draws every recorded
		// query, otherwise just the selected one. Re-resolve the pinned query to this frame so a repeated
		// query keeps drawing as the recording steps; it vanishes only on a frame that does not issue it.
		m_selQuery = m_selKind == SelQuery ? ResolveSelectedQuery() : -1;
		if ( m_drawAllQueries )
		{
			// Draw all, emphasizing the selected one so it stands out from the crowd.
			b3RecPlayer_DrawFrameQueries( m_player, &debugDraw, -1, m_selQuery );
		}
		else if ( m_selQuery >= 0 )
		{
			b3RecPlayer_DrawFrameQueries( m_player, &debugDraw, m_selQuery, m_selQuery );
		}

		// Light up the query row under the cursor even when it is not the pinned one. Set during the
		// outline draw, so the highlight trails the cursor by a frame, which is invisible in practice.
		// Keep passing the selected index so hovering the selected query does not repaint over its
		// emphasis color with the kind color.
		if ( m_hoverQuery >= 0 )
		{
			b3RecPlayer_DrawFrameQueries( m_player, &debugDraw, m_hoverQuery, m_selQuery );
		}
	}

	// A replay re-runs recorded inputs, so the live solver sliders would do nothing. This also hides
	// the Solver and Recording sections in the right info panel.
	bool HasSolverControls() const override
	{
		return false;
	}

	// Wider than the default so the detail pane, hosted in the info panel, has room for ids and vectors.
	float InfoPanelWidthEm() const override
	{
		return 24.0f;
	}

	// F frames the selected query by its recorded world-space bounds. Absent this frame, fall through
	// to the body paths.
	bool FocusBounds( b3AABB* bounds ) override
	{
		// A shape selection frames that shape alone, finer than its whole body.
		if ( m_selKind == SelShape )
		{
			b3ShapeId shape = SelectedShape();
			if ( b3Shape_IsValid( shape ) )
			{
				*bounds = b3Shape_GetAABB( shape );
				return true;
			}
		}
		if ( m_selKind != SelQuery || m_player == nullptr )
		{
			return false;
		}
		int sel = ResolveSelectedQuery();
		if ( sel < 0 )
		{
			return false;
		}
		*bounds = b3RecPlayer_GetFrameQuery( m_player, sel ).aabb;
		return true;
	}

	// Frame the selection by body. A body, shape, or joint pick resolves to a body. A shape pick is
	// framed tighter in FocusBounds, so this is its fallback. Null when nothing resolves.
	b3BodyId FocusBody() const override
	{
		if ( m_selKind == SelBody || m_selKind == SelShape || m_selKind == SelJoint )
		{
			b3BodyId body = SelectedBody();
			if ( b3Body_IsValid( body ) )
			{
				return body;
			}
		}
		return b3_nullBodyId;
	}

	// Nothing selected: fit the recording, not the empty base world. Reuses the on-load framing so the
	// current up-axis view transform is applied.
	void FocusHome() override
	{
		FrameRecording();
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
			return false;
		}

		if ( ImGui::Button( "Show Timeline" ) )
		{
			m_context->showMetrics = true;
			m_selectTimelineTab = true;
		}

		// Overlay every recorded query, not just the one selected in the outline.
		ImGui::Checkbox( "Draw All Queries", &m_drawAllQueries );

		// View-only stand-up for recordings authored with +Z as up. The simulation is untouched.
		// Reframe on a toggle so the rotated bounds stay centered, matching the fit done on load.
		if ( ImGui::Checkbox( "Z-up View", &m_context->viewZUp ) )
		{
			FrameRecording();
		}

		if ( b3RecPlayer_HasDiverged( m_player ) )
		{
			ImGui::TextColored( PanelColor( b3_colorRed ), "****DIVERGED****" );
		}

		ImGui::TextDisabled( "Frame %d / %d%s", b3RecPlayer_GetFrame( m_player ), m_info.frameCount,
							 b3RecPlayer_IsAtEnd( m_player ) ? "  (end)" : "" );

		// Selection detail lives here in the info panel, not the Outline window, so the scene tree gets
		// the full left column. The child takes the remaining panel height and scrolls a long detail.
		// Return false so the panel adds no trailing separator below this full-height child.
		ImGui::Separator();
		ImGui::TextColored( PanelColor( b3_colorGoldenRod ), "Detail" );
		ImGui::BeginChild( "DetailPane", { 0.0f, 0.0f } );
		DrawDetail();
		ImGui::EndChild();
		return false;
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

	// Every shape on a body, sized to the real count so a body with many shapes still
	// selects. The slot model breaks under a fixed cap once the count exceeds it.
	void BodyShapes( std::vector<b3ShapeId>& buffer, b3BodyId bodyId ) const
	{
		int count = b3Body_GetShapeCount( bodyId );
		buffer.resize( count );
		if ( count > 0 )
		{
			b3Body_GetShapes( bodyId, buffer.data(), count );
		}
	}

	b3ShapeId SelectedShape()
	{
		b3BodyId body = SelectedBody();
		if ( m_selKind != SelShape || b3Body_IsValid( body ) == false )
		{
			return b3_nullShapeId;
		}
		BodyShapes( m_shapeBuffer, body );
		return ( m_selSlot >= 0 && m_selSlot < (int)m_shapeBuffer.size() ) ? m_shapeBuffer[m_selSlot] : b3_nullShapeId;
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

	// Map the pinned query to this frame's flat query index, or -1 when the frame did not issue it. A
	// tagged query is matched by its caller id and instance, which track the logical query across steps
	// and seeks regardless of call order. An untagged query falls back to the per-kind call position.
	int ResolveSelectedQuery() const
	{
		if ( m_player == nullptr || m_selKind != SelQuery )
		{
			return -1;
		}
		int n = b3RecPlayer_GetFrameQueryCount( m_player );
		if ( m_selQueryKey != 0 )
		{
			int instance = 0;
			for ( int i = 0; i < n; ++i )
			{
				if ( b3RecPlayer_GetFrameQuery( m_player, i ).key == m_selQueryKey )
				{
					if ( instance == m_selQueryInstance )
					{
						return i;
					}
					instance += 1;
				}
			}
			return -1;
		}
		if ( m_selQueryKindOrdinal < 0 )
		{
			return -1;
		}
		int ord = 0;
		for ( int i = 0; i < n; ++i )
		{
			if ( b3RecPlayer_GetFrameQuery( m_player, i ).type == m_selQueryKind && ord++ == m_selQueryKindOrdinal )
			{
				return i;
			}
		}
		return -1;
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
		BodyShapes( m_shapeBuffer, body );
		int slot = -1;
		for ( int i = 0; i < (int)m_shapeBuffer.size(); ++i )
		{
			if ( B3_ID_EQUALS( m_shapeBuffer[i], shape ) )
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

	// Selection overlays drawn on top of the outline highlight: body axes, center of mass, and
	// contacts. Joints mark both connected body centers.
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

	// Left-edge Outline window plus the keyframe-policy popup. The selection detail lives in the right
	// info panel (DrawControls), so the tree owns the whole column.
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
		ImGui::BeginChild( "OutlineTree", { 0.0f, 0.0f } );
		DrawOutlineTree();
		ImGui::EndChild();

		ImGui::End();
	}

	// Scan the whole recording once to index every recorded query. Drives the player frame 0 to end,
	// then seeks back to where the user was so the live view is undisturbed. One full replay pass, so
	// build lazily and cache. The scan replays the same deterministic path as playback, so it must not
	// trip the divergence flag on a clean recording.
	void BuildQueryIndex()
	{
		m_queryIndex.clear();
		m_queryIndexBuilt = true;
		if ( m_player == nullptr )
		{
			return;
		}

		int resume = b3RecPlayer_GetFrame( m_player );
		b3RecPlayer_SeekFrame( m_player, 0 );
		while ( b3RecPlayer_IsAtEnd( m_player ) == false )
		{
			b3RecPlayer_StepFrame( m_player );
			int frame = b3RecPlayer_GetFrame( m_player );
			int qn = b3RecPlayer_GetFrameQueryCount( m_player );
			int kindOrdinal[ReplayQueryTypeCount] = { 0 };
			// Running occurrence count per key, so disambiguating same-key calls stays linear in the frame.
			std::unordered_map<uint64_t, int> keyInstance;
			for ( int i = 0; i < qn; ++i )
			{
				b3RecQueryInfo q = b3RecPlayer_GetFrameQuery( m_player, i );
				QueryIndexRow row;
				row.frame = frame;
				row.type = q.type;
				row.kindOrdinal = kindOrdinal[q.type]++;
				row.instance = q.key != 0 ? keyInstance[q.key]++ : 0;
				row.key = q.key;
				row.id = q.id;
				row.name = q.name;
				row.filter = q.filter;
				row.origin = q.origin;
				row.hitCount = q.hitCount;
				m_queryIndex.push_back( row );
			}
		}

		b3RecPlayer_SeekFrame( m_player, resume );
		m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
	}

	// Search box over the whole-recording query index, drawn above the scene tree. The index is built
	// lazily on first open, then cached until the recording changes. A result jumps to its frame and
	// pins the query, so stepping from there tracks it.
	void DrawQuerySearch()
	{
		if ( ImGui::CollapsingHeader( "Query Search" ) == false )
		{
			return;
		}

		if ( m_queryIndexBuilt == false )
		{
			BuildQueryIndex();
		}

		float fontSize = ImGui::GetFontSize();
		ImGui::PushItemWidth( -1.0f );
		ImGui::InputTextWithHint( "##qsearch", "Filter by kind, frame, or category...", m_querySearch, sizeof( m_querySearch ) );
		ImGui::PopItemWidth();

		// Kind 0 is any, otherwise a specific b3RecQueryType in enum order.
		const char* kindNames[ReplayQueryTypeCount + 1] = {
			"any kind",	  "overlap AABB",	  "overlap shape", "cast ray",
			"cast shape", "cast ray closest", "cast mover",	   "collide mover",
		};
		ImGui::PushItemWidth( 8.0f * fontSize );
		ImGui::Combo( "Kind", &m_querySearchKind, kindNames, ReplayQueryTypeCount + 1 );
		ImGui::PopItemWidth();
		ImGui::SameLine();
		ImGui::PushItemWidth( 4.0f * fontSize );
		ImGui::InputInt( "Min hits", &m_querySearchMinHits, 0, 0 );
		ImGui::PopItemWidth();
		if ( m_querySearchMinHits < 0 )
		{
			m_querySearchMinHits = 0;
		}

		if ( ImGui::SmallButton( "Rebuild" ) )
		{
			BuildQueryIndex();
		}
		ImGui::SameLine();
		ImGui::TextDisabled( "%d recorded", (int)m_queryIndex.size() );

		ImGui::BeginChild( "##qresults", { 0.0f, 11.0f * fontSize }, ImGuiChildFlags_Borders );
		int shown = 0;
		for ( int r = 0; r < (int)m_queryIndex.size(); ++r )
		{
			const QueryIndexRow& row = m_queryIndex[r];
			if ( m_querySearchKind != 0 && (int)row.type != m_querySearchKind - 1 )
			{
				continue;
			}
			if ( row.hitCount < m_querySearchMinHits )
			{
				continue;
			}

			// The label (name + id) is searchable, so typing a label or an entity id narrows results.
			char base[64];
			FormatQueryLabel( base, sizeof( base ), row.name, row.id, row.type, row.kindOrdinal );
			char text[112];
			snprintf( text, sizeof( text ), "f%-5d %s  (%d)  cat 0x%llx", row.frame, base, row.hitCount,
					  (unsigned long long)row.filter.categoryBits );
			if ( ContainsNoCase( text, m_querySearch ) == false )
			{
				continue;
			}

			// Hold the full text plus the ###qr id so a long label cannot truncate the id and collide rows.
			char label[128];
			snprintf( label, sizeof( label ), "%s###qr%d", text, r );
			ImGui::PushStyleColor( ImGuiCol_Text, PanelColor( QueryKindColor( row.type ) ) );
			bool clicked = ImGui::Selectable( label, false );
			ImGui::PopStyleColor();
			if ( clicked )
			{
				b3RecPlayer_SeekFrame( m_player, row.frame );
				m_replayWorldId = b3RecPlayer_GetWorldId( m_player );
				m_context->pause = true;
				m_frameAccumulator = 0.0f;
				m_selKind = SelQuery;
				m_selQueryKey = row.key;
				FormatQueryLabel( m_selQueryLabel, sizeof( m_selQueryLabel ), row.name, row.id, row.type, row.kindOrdinal );
				m_selQueryKind = row.type;
				m_selQueryKindOrdinal = row.kindOrdinal;
				m_selQueryInstance = row.instance;
				m_selQuery = ResolveSelectedQuery();
				m_revealSelection = true; // expand and scroll the Queries node to the pinned row
			}
			shown++;
		}
		if ( shown == 0 )
		{
			ImGui::TextDisabled( "No matching queries." );
		}
		ImGui::EndChild();
		ImGui::Separator();
	}

	// Recorded scene tree: bodies by creation ordinal, expandable to their shapes and joints.
	// Destroyed bodies keep their ordinal but are not shown, so a stored selection stays put.
	void DrawOutlineTree()
	{
		// Cleared each draw, set again if the cursor lands on a query row this frame.
		m_hoverQuery = -1;

		// Whole-recording query search sits above the scene tree. A jump from there can set the reveal
		// flag, so draw it before capturing reveal below.
		DrawQuerySearch();

		// A viewport pick or a search jump asks the tree to reveal its target once: expand the owner and
		// scroll to the row. Consumed at the end so it never fights the user's own expand/collapse.
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

			BodyShapes( m_shapeBuffer, body );
			for ( int s = 0; s < (int)m_shapeBuffer.size(); ++s )
			{
				b3ShapeType shapeType = b3Shape_GetType( m_shapeBuffer[s] );
				char sl[64];
				snprintf( sl, sizeof( sl ), "Shape %d  %s###b%ds%d", s, ReplayShapeTypeName( shapeType ), ord, s );
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

		// Spatial queries recorded for the current frame. A row is colored by kind and numbered within
		// its kind, so the Nth ray cast keeps the same label every frame the app issues it. Selecting one
		// pins that kind and ordinal, and the overlay and detail then track it as the recording steps.
		int qn = b3RecPlayer_GetFrameQueryCount( m_player );
		char ql[32];
		snprintf( ql, sizeof( ql ), "Queries (%d)###queries", qn );
		if ( reveal && m_selKind == SelQuery )
		{
			ImGui::SetNextItemOpen( true );
		}
		if ( ImGui::TreeNodeEx( ql, ImGuiTreeNodeFlags_SpanAvailWidth ) )
		{
			int kindOrdinal[ReplayQueryTypeCount] = { 0 };
			// Count earlier same-key queries as we go so a tag issued several times this frame (the mover
			// slide loop) gets a stable instance number. Untagged queries have no shared identity.
			std::unordered_map<uint64_t, int> keyInstance;
			for ( int i = 0; i < qn; ++i )
			{
				b3RecQueryInfo q = b3RecPlayer_GetFrameQuery( m_player, i );
				int ord = kindOrdinal[q.type]++;
				int instance = q.key != 0 ? keyInstance[q.key]++ : 0;
				char base[64];
				FormatQueryLabel( base, sizeof( base ), q.name, q.id, q.type, ord );
				char qi[96];
				snprintf( qi, sizeof( qi ), "%s  (%d)###q%d", base, q.hitCount, i );
				ImGuiTreeNodeFlags lf =
					ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_NoTreePushOnOpen;
				bool pinned = m_selKind == SelQuery &&
							  ( m_selQueryKey != 0 ? ( q.key == m_selQueryKey && instance == m_selQueryInstance )
												   : ( q.type == m_selQueryKind && ord == m_selQueryKindOrdinal ) );
				if ( pinned )
				{
					lf |= ImGuiTreeNodeFlags_Selected;
				}
				ImGui::PushStyleColor( ImGuiCol_Text, PanelColor( QueryKindColor( q.type ) ) );
				ImGui::TreeNodeEx( qi, lf );
				ImGui::PopStyleColor();
				if ( reveal && pinned )
				{
					ImGui::SetScrollHereY( 0.5f );
				}
				if ( ImGui::IsItemHovered() )
				{
					m_hoverQuery = i;
				}
				if ( ImGui::IsItemClicked() )
				{
					m_selKind = SelQuery;
					m_selQueryKey = q.key;
					FormatQueryLabel( m_selQueryLabel, sizeof( m_selQueryLabel ), q.name, q.id, q.type, ord );
					m_selQueryKind = q.type;
					m_selQueryKindOrdinal = ord;
					m_selQueryInstance = instance;
					m_selQuery = i;
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

		if ( m_selKind == SelQuery )
		{
			DrawQueryDetail();
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

	// Detail for the pinned recorded query: type, filter, origin, sweep, and the recorded hit shapes.
	// The pin is a kind plus per-kind ordinal, so re-resolve it to this frame; absent means the frame
	// did not issue it, while the pin stays put and repopulates when the query returns.
	void DrawQueryDetail()
	{
		m_selQuery = ResolveSelectedQuery();
		if ( m_selQuery < 0 )
		{
			ImGui::PushStyleColor( ImGuiCol_Text, PanelColor( QueryKindColor( m_selQueryKind ) ) );
			ImGui::TextUnformatted( m_selQueryLabel[0] != '\0' ? m_selQueryLabel : "query" );
			ImGui::PopStyleColor();
			ImGui::TextDisabled( "Not present at this frame." );
			return;
		}

		b3RecQueryInfo q = b3RecPlayer_GetFrameQuery( m_player, m_selQuery );
		if ( ImGui::CollapsingHeader( "Query", ImGuiTreeNodeFlags_DefaultOpen ) == false )
		{
			return;
		}

		if ( q.name != nullptr )
		{
			ImGui::Text( "label    %s", q.name );
		}
		if ( q.id != 0 )
		{
			ImGui::Text( "id       %llu", (unsigned long long)q.id );
		}
		if ( q.key != 0 )
		{
			ImGui::Text( "type     %s", ReplayQueryTypeName( q.type ) );
		}
		else
		{
			ImGui::Text( "type     %s #%d", ReplayQueryTypeName( q.type ), m_selQueryKindOrdinal );
		}
		ImGui::Text( "category 0x%016llx", (unsigned long long)q.filter.categoryBits );
		ImGui::Text( "mask     0x%016llx", (unsigned long long)q.filter.maskBits );
		if ( q.type != b3_recQueryOverlapAABB )
		{
			ImGui::Text( "origin   (%.2f, %.2f, %.2f)", q.origin.x, q.origin.y, q.origin.z );
		}
		// Translation is the ray or sweep vector. Overlaps and collide-mover leave it zero.
		if ( q.type == b3_recQueryCastRay || q.type == b3_recQueryCastRayClosest || q.type == b3_recQueryCastShape ||
			 q.type == b3_recQueryCastMover )
		{
			ImGui::Text( "sweep    (%.2f, %.2f, %.2f)", q.translation.x, q.translation.y, q.translation.z );
		}
		ImGui::Text( "hits     %d", q.hitCount );

		// Hit shape ids as one wrapped list so a many-hit query stays compact.
		char line[256];
		int len = 0;
		for ( int h = 0; h < q.hitCount && len < (int)sizeof( line ) - 12; ++h )
		{
			b3RecQueryHit hit = b3RecPlayer_GetFrameQueryHit( m_player, m_selQuery, h );
			len += snprintf( line + len, sizeof( line ) - len, "%d ", hit.shape.index1 );
		}
		if ( q.hitCount > 0 )
		{
			ImGui::TextWrapped( "hit shapes: %s", line );
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
		ImGui::Text( "   bodies %d  shapes %d  contacts %d  joints %d", c.bodyCount, c.shapeCount, c.contactCount, c.jointCount );

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
		b3QueryFilter queryFilter = b3DefaultQueryFilter();
		b3RayResult result = b3World_CastRayClosest( m_replayWorldId, pickRay.origin, pickRay.translation, queryFilter );
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
	b3RecPlayerInfo m_info;	   // cached at load for the timeline readout and camera framing
	char m_path[256];
	char m_status[128];
	float m_speed;
	float m_frameAccumulator;
	bool m_loop;

	bool m_selectTimelineTab; // one-shot: focus the Timeline tab on the next draw
	bool m_prevShowMetrics;	  // restore the drawer state on exit
	bool m_prevViewZUp;		  // restore the up-axis view on exit

	// Load popup state. A fresh open configures the keyframe policy here, then the popup switches to a
	// progress bar while every keyframe is generated up front. Temporaries hold the in-popup edits so
	// Cancel leaves the persisted settings untouched.
	bool m_requestLoadPopup;
	bool m_generating;
	int m_popupBudgetMB;
	int m_popupMinInterval;

	// Selection by creation ordinal so it survives a backward seek that rebuilds the world.
	SelectionKind m_selKind;
	int m_selBodyOrdinal;
	int m_selSlot;			// shape or joint slot within the selected body
	int m_selQuery;			// resolved query index for the current frame, recomputed from the pin below
	bool m_revealSelection; // one-shot: expand and scroll the tree to a viewport pick or search jump
	bool m_drawAllQueries;	// overlay every recorded query, not just the selected one
	int m_hoverQuery;		// outline query row under the cursor, drawn as a transient highlight

	// Re-usable buffer for getting all shapes from a body.
	std::vector<b3ShapeId> m_shapeBuffer;

	// A pinned query prefers its identity key (the hash of the caller id and label), the robust identity
	// that tracks the logical query regardless of call order. A tag issued several times per step (the
	// mover slide loop) shares one key across its calls, so the instance index tells those calls apart.
	// Untagged queries (key 0) fall back to the per-kind call position, stable only while queries keep
	// the same order each frame.
	uint64_t m_selQueryKey;
	char m_selQueryLabel[64]; // pinned query's display label, for the absent-frame readout
	b3RecQueryType m_selQueryKind;
	int m_selQueryKindOrdinal;
	int m_selQueryInstance; // occurrence among same-key queries this frame, 0 unless a tag repeats

	std::vector<QueryIndexRow> m_queryIndex;
	bool m_queryIndexBuilt;
	char m_querySearch[64];
	int m_querySearchKind;	  // 0 any, else b3RecQueryType + 1
	int m_querySearchMinHits; // hide queries with fewer hits
};

static int sampleReplayViewer = RegisterReplay( "Replay", "Viewer", ReplayViewer::Create );
