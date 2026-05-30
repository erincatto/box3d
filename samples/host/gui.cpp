// Dear ImGui shell - implementation.
//
// Owns the simgui setup and renders the built-in left "Renderer" window
// (Stats + Render Settings collapsing sections) plus the app's UI draw callback
// (the right "Scene" window). All renderer state changes flow through the
// public Set* setters; the panel never reads or writes renderer
// internals.
//
// Render path: ImGui draws on a NEW swapchain pass opened in RenderUI,
// LOADed on top of the already-tonemapped swapchain image. The simgui
// pipeline therefore inherits the swapchain's color + depth format and
// sample_count=1.

#include "gui.h"

#include "gfx/draw.h"
#include "gfx/gtao.h"
#include "gfx/projection.h"
#include "gfx/renderer.h"
#include "gfx/text.h"
#include "imgui.h"
#include "sokol_app.h"
#include "sokol_imgui.h"
#include "sokol_log.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

static bool s_uiInitialized = false;

// TAB toggles this. When false we skip the ImGui panels (Stats / Render
// Settings / the UI draw callback) but still run simgui_new_frame and
// RenderText, so scene labels stay visible.
static bool s_panelsVisible = false;

// Display scale (sapp_dpi_scale) latched once in InitUI. ImGui runs in
// physical framebuffer pixels, so panel code multiplies explicit pixel sizes
// by this; the font atlas and the rest of the style are pre-scaled by it too.
static float s_uiScale = 1.0f;

static DrawUiFcn* s_uiCallback;

// Restyle Dear ImGui away from the stock dark theme: a desaturated steel-blue
// accent over neutral charcoal, one consistent rounding system, and roomier
// padding. Sets base (unscaled) metrics - InitUI runs ScaleAllSizes after
// this on hi-DPI displays; colors are scale-independent so they need no pass.
static void guiApplyStyle( void )
{
	ImGuiStyle& style = ImGui::GetStyle();

	// Metrics: containers round at 4px, controls at 3px - one deliberate
	// system instead of the stock mix. Padding gives rows room to breathe.
	style.WindowPadding = ImVec2( 10.0f, 10.0f );
	style.FramePadding = ImVec2( 8.0f, 4.0f );
	style.CellPadding = ImVec2( 6.0f, 4.0f );
	style.ItemSpacing = ImVec2( 8.0f, 7.0f );
	style.ItemInnerSpacing = ImVec2( 7.0f, 4.0f );
	style.IndentSpacing = 18.0f;
	style.ScrollbarSize = 12.0f;
	style.GrabMinSize = 10.0f;

	style.WindowBorderSize = 1.0f;
	style.FrameBorderSize = 0.0f;
	style.PopupBorderSize = 1.0f;
	style.TabBorderSize = 0.0f;
	style.SeparatorTextBorderSize = 1.0f;

	style.WindowRounding = 4.0f;
	style.ChildRounding = 4.0f;
	style.PopupRounding = 4.0f;
	style.FrameRounding = 3.0f;
	style.GrabRounding = 3.0f;
	style.ScrollbarRounding = 3.0f;
	style.TabRounding = 3.0f;

	style.WindowTitleAlign = ImVec2( 0.0f, 0.5f );

	// Palette: neutral charcoal surfaces, one steel-blue accent at three
	// brightnesses. Replaces stock ImGui's saturated cornflower blue.
	const ImVec4 accent = ImVec4( 0.28f, 0.48f, 0.66f, 1.00f );
	const ImVec4 accentHi = ImVec4( 0.38f, 0.60f, 0.80f, 1.00f );
	const ImVec4 accentLo = ImVec4( 0.22f, 0.36f, 0.50f, 1.00f );

	ImVec4* c = style.Colors;
	c[ImGuiCol_Text] = ImVec4( 0.90f, 0.91f, 0.93f, 1.00f );
	c[ImGuiCol_TextDisabled] = ImVec4( 0.49f, 0.51f, 0.55f, 1.00f );
	c[ImGuiCol_WindowBg] = ImVec4( 0.110f, 0.115f, 0.125f, 0.97f );
	c[ImGuiCol_ChildBg] = ImVec4( 0.00f, 0.00f, 0.00f, 0.00f );
	c[ImGuiCol_PopupBg] = ImVec4( 0.100f, 0.105f, 0.115f, 0.98f );
	c[ImGuiCol_Border] = ImVec4( 0.00f, 0.00f, 0.00f, 0.45f );
	c[ImGuiCol_BorderShadow] = ImVec4( 0.00f, 0.00f, 0.00f, 0.00f );
	c[ImGuiCol_FrameBg] = ImVec4( 0.18f, 0.19f, 0.21f, 1.00f );
	c[ImGuiCol_FrameBgHovered] = ImVec4( 0.24f, 0.26f, 0.29f, 1.00f );
	c[ImGuiCol_FrameBgActive] = ImVec4( 0.29f, 0.32f, 0.36f, 1.00f );
	c[ImGuiCol_TitleBg] = ImVec4( 0.090f, 0.095f, 0.105f, 1.00f );
	c[ImGuiCol_TitleBgActive] = ImVec4( 0.14f, 0.16f, 0.19f, 1.00f );
	c[ImGuiCol_TitleBgCollapsed] = ImVec4( 0.090f, 0.095f, 0.105f, 0.75f );
	c[ImGuiCol_MenuBarBg] = ImVec4( 0.13f, 0.14f, 0.16f, 1.00f );
	c[ImGuiCol_ScrollbarBg] = ImVec4( 0.06f, 0.06f, 0.07f, 0.55f );
	c[ImGuiCol_ScrollbarGrab] = ImVec4( 0.28f, 0.30f, 0.33f, 1.00f );
	c[ImGuiCol_ScrollbarGrabHovered] = ImVec4( 0.36f, 0.39f, 0.43f, 1.00f );
	c[ImGuiCol_ScrollbarGrabActive] = accent;
	c[ImGuiCol_CheckMark] = accentHi;
	c[ImGuiCol_SliderGrab] = accent;
	c[ImGuiCol_SliderGrabActive] = accentHi;
	c[ImGuiCol_Button] = ImVec4( 0.22f, 0.24f, 0.27f, 1.00f );
	c[ImGuiCol_ButtonHovered] = accentLo;
	c[ImGuiCol_ButtonActive] = accent;
	c[ImGuiCol_Header] = ImVec4( 0.19f, 0.21f, 0.24f, 1.00f );
	c[ImGuiCol_HeaderHovered] = accentLo;
	c[ImGuiCol_HeaderActive] = accent;
	c[ImGuiCol_Separator] = ImVec4( 1.00f, 1.00f, 1.00f, 0.09f );
	c[ImGuiCol_SeparatorHovered] = accentLo;
	c[ImGuiCol_SeparatorActive] = accent;
	c[ImGuiCol_ResizeGrip] = ImVec4( 1.00f, 1.00f, 1.00f, 0.06f );
	c[ImGuiCol_ResizeGripHovered] = accentLo;
	c[ImGuiCol_ResizeGripActive] = accent;
	c[ImGuiCol_Tab] = ImVec4( 0.15f, 0.16f, 0.18f, 1.00f );
	c[ImGuiCol_TabHovered] = accentLo;
	c[ImGuiCol_TabSelected] = accent;
	c[ImGuiCol_TabSelectedOverline] = accentHi;
	c[ImGuiCol_TabDimmed] = ImVec4( 0.12f, 0.13f, 0.14f, 1.00f );
	c[ImGuiCol_TabDimmedSelected] = accentLo;
	c[ImGuiCol_TextSelectedBg] = ImVec4( accent.x, accent.y, accent.z, 0.40f );
	c[ImGuiCol_DragDropTarget] = accentHi;
	c[ImGuiCol_NavCursor] = accentHi;
	c[ImGuiCol_PlotLines] = ImVec4( 0.70f, 0.72f, 0.75f, 1.00f );
	c[ImGuiCol_PlotLinesHovered] = accentHi;
	c[ImGuiCol_PlotHistogram] = accent;
	c[ImGuiCol_PlotHistogramHovered] = accentHi;
}

void InitUI( const sg_environment* env, DrawUiFcn* drawGuiFcn, bool showInterface )
{
	s_panelsVisible = showInterface;
	s_uiCallback = drawGuiFcn;

	simgui_desc_t desc = {};
	desc.color_format = env->defaults.color_format;
	desc.depth_format = env->defaults.depth_format;
	desc.sample_count = env->defaults.sample_count;
	desc.ini_filename = "render3d_imgui.ini";
	desc.logger.func = slog_func;
	// Skip ImGui's auto-pick so we always get the vector font we add below.
	desc.no_default_font = true;
	simgui_setup( &desc );

	// DPI handling mirrors the Box2D sample app, which gets crisp text at
	// fractional Windows display scales. ImGui runs in physical framebuffer
	// pixels: StartUIFrame feeds simgui dpi_scale=1.0, so DisplaySize is the
	// framebuffer size and DisplayFramebufferScale is 1.0. The display scale
	// is then applied by hand - the font atlas is baked once at an integer
	// pixel size and ScaleAllSizes scales the rest of the style. Drawing
	// ImGui 1:1 against the framebuffer with an integer-sized atlas is what
	// keeps glyphs sharp; letting sokol_imgui scale a logical-pixel UI up by
	// a fractional factor instead leaves every glyph on a sub-pixel boundary.
	s_uiScale = sapp_dpi_scale();

	ImGuiIO& io = ImGui::GetIO();
	io.Fonts->Clear();

	// Restyle before the hi-DPI branch: guiApplyStyle sets base metrics, then
	// ScaleAllSizes (below) multiplies them up. Colors are unscaled.
	guiApplyStyle();

	io.Fonts->AddFontDefaultVector();

	ImGuiStyle& style = ImGui::GetStyle();
	if ( s_uiScale != 1.0f )
	{
		style.ScaleAllSizes( s_uiScale );
	}

	// 13px base for the proportional UI font, scaled to a whole physical
	// pixel. Rasterizer density is 1.0 (DisplayFramebufferScale 1.0), so the
	// atlas bakes at exactly this - floorf keeps it on a whole-pixel size.
	style.FontSizeBase = floorf( 13.0f * s_uiScale );

	s_uiInitialized = true;
}

void ShutdownUI( void )
{
	if ( !s_uiInitialized )
		return;
	simgui_shutdown();
	s_uiInitialized = false;
}

bool HandleEvent( const sapp_event* e )
{
	if ( !s_uiInitialized )
		return false;
	// TAB toggles panel visibility. Handled before simgui_handle_event so
	// ImGui's nav system doesn't also act on the key. Suppressed while a
	// text widget is focused so the user can still TAB between fields.
	if ( e->type == SAPP_EVENTTYPE_KEY_DOWN && e->key_code == SAPP_KEYCODE_TAB && !ImGui::GetIO().WantTextInput )
	{
		s_panelsVisible = !s_panelsVisible;
		return true;
	}
	const bool handled = simgui_handle_event( e );
	const ImGuiIO& io = ImGui::GetIO();
	// simgui_handle_event tracks every event for ImGui's input state, but
	// only returns true for events it consumed exclusively (e.g. clipboard
	// paste). For typical interactions we want the camera/key handler to
	// skip whenever ImGui has the cursor or keyboard focus - that's what
	// io.WantCapture* signals.
	if ( handled )
		return true;
	if ( e->type == SAPP_EVENTTYPE_KEY_DOWN || e->type == SAPP_EVENTTYPE_KEY_UP || e->type == SAPP_EVENTTYPE_CHAR )
	{
		return io.WantCaptureKeyboard;
	}
	if ( e->type == SAPP_EVENTTYPE_MOUSE_DOWN || e->type == SAPP_EVENTTYPE_MOUSE_UP || e->type == SAPP_EVENTTYPE_MOUSE_MOVE ||
		 e->type == SAPP_EVENTTYPE_MOUSE_SCROLL )
	{
		return io.WantCaptureMouse;
	}
	return false;
}

// Gap between a panel and the window edge
static const float PANEL_GAP = 5.0f;

// Running top Y for the next panel on each side, indexed by GuiPanelAnchor.
static float s_panelCursorY[2];
// Side of the panel currently open between guiBeginPanel and guiEndPanel.
static GuiPanelAnchor s_panelOpenSide;
static bool s_panelOpen = false;

static void ResetUIPanelLayout( void )
{
	const ImGuiViewport* vp = ImGui::GetMainViewport();
	const float top = vp->WorkPos.y + PANEL_GAP * s_uiScale;
	s_panelCursorY[GUI_ANCHOR_LEFT] = top;
	s_panelCursorY[GUI_ANCHOR_RIGHT] = top;
	s_panelOpen = false;
}

bool BeginPanel( const char* name, GuiPanelAnchor anchor, float width )
{
	ImGuiViewport* vp = ImGui::GetMainViewport();
	float gap = PANEL_GAP * s_uiScale;
	float w = width * s_uiScale;
	float cursorY = s_panelCursorY[anchor];

	// Left panels pin their top-left corner to the left edge; right panels
	// pin their top-right corner to the right edge (pivot picks the corner).
	float anchorX;
	ImVec2 pivot;
	if ( anchor == GUI_ANCHOR_RIGHT )
	{
		anchorX = vp->WorkPos.x + vp->WorkSize.x - gap;
		pivot = ImVec2( 1.0f, 0.0f );
	}
	else
	{
		anchorX = vp->WorkPos.x + gap;
		pivot = ImVec2( 0.0f, 0.0f );
	}
	ImGui::SetNextWindowPos( ImVec2( anchorX, cursorY ), ImGuiCond_Always, pivot );

	// SetNextWindowSize y=0 auto-fits height to content; the constraint caps
	// it so a tall panel can't run past the window's bottom edge (the panel
	// gains a scrollbar past the cap). Width is locked to w by both calls.
	float maxH = vp->WorkPos.y + vp->WorkSize.y - gap - cursorY;
	if ( maxH < ImGui::GetFrameHeight() )
		maxH = ImGui::GetFrameHeight();
	ImGui::SetNextWindowSizeConstraints( ImVec2( w, 0.0f ), ImVec2( w, maxH ) );
	ImGui::SetNextWindowSize( ImVec2( w, 0.0f ), ImGuiCond_Always );

	s_panelOpenSide = anchor;
	s_panelOpen = true;
	return ImGui::Begin( name, nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );
}

void EndPanel( void )
{
	// Advance this side's cursor past the panel just drawn so the next panel
	// stacks directly below it. GetWindowSize is valid here whether or not
	// Begin returned true - a collapsed panel reports its title-bar height,
	// so the next panel snugs up under the collapsed bar.
	if ( s_panelOpen )
	{
		s_panelCursorY[s_panelOpenSide] += ImGui::GetWindowSize().y + PANEL_GAP * s_uiScale;
		s_panelOpen = false;
	}
	ImGui::End();
}

// One row of the Stats panel: label in the fixed left column, value in the
// right. Replaces space-padded ImGui::Text - that only aligned under the old
// monospace font and goes ragged under a proportional one.
static void StatRow( const char* label, const char* fmt, ... )
{
	ImGui::TableNextColumn();
	ImGui::TextUnformatted( label );
	ImGui::TableNextColumn();
	va_list args;
	va_start( args, fmt );
	ImGui::TextV( fmt, args );
	va_end( args );
}

// The Stats section of the left "Renderer" window - a collapsing header over
// the live frame / instance / overlay counts. Renders no window of its own:
// LeftPanel owns the window and stacks this above SettingsSection.
static void StatsSection( void )
{
	if ( !ImGui::CollapsingHeader( "Stats", ImGuiTreeNodeFlags_DefaultOpen ) )
		return;
	ImGui::Indent();

	const RenderStats st = GetRenderStats();

	// Headline frame time is sokol's display-link-derived value (real
	// present-to-present interval). CPU is the RenderFrame submit cost.
	// Both EMA-smoothed (factor 0.1 -> ~10-frame time constant) to keep the
	// readout from flickering.
	const float frameMs = (float)( sapp_frame_duration() * 1000.0 );
	static float s_smoothedFrameMs = 0.0f;
	static float s_smoothedCpuMs = 0.0f;
	s_smoothedFrameMs = ( s_smoothedFrameMs <= 0.0f ) ? frameMs : s_smoothedFrameMs * 0.9f + frameMs * 0.1f;
	s_smoothedCpuMs = ( s_smoothedCpuMs <= 0.0f ) ? st.frameTimeMs : s_smoothedCpuMs * 0.9f + st.frameTimeMs * 0.1f;
	const float fps = ( s_smoothedFrameMs > 0.0f ) ? ( 1000.0f / s_smoothedFrameMs ) : 0.0f;

	const char* backend = "?";
	switch ( sg_query_backend() )
	{
		case SG_BACKEND_D3D11:
			backend = "D3D11";
			break;
		case SG_BACKEND_METAL_MACOS:
			backend = "Metal";
			break;
		case SG_BACKEND_METAL_IOS:
			backend = "Metal (iOS)";
			break;
		case SG_BACKEND_METAL_SIMULATOR:
			backend = "Metal (Sim)";
			break;
		case SG_BACKEND_GLCORE:
			backend = "GLCORE";
			break;
		case SG_BACKEND_GLES3:
			backend = "GLES3";
			break;
		case SG_BACKEND_WGPU:
			backend = "WGPU";
			break;
		default:
			break;
	}

	// Fixed label-column width so the value column lines up across every
	// table in the panel. GetFontSize is already DPI-scaled.
	const float labelW = ImGui::GetFontSize() * 8.0f;
	const ImGuiTableFlags tableFlags = ImGuiTableFlags_NoSavedSettings;

	if ( ImGui::BeginTable( "##stats_top", 2, tableFlags ) )
	{
		ImGui::TableSetupColumn( "##l", ImGuiTableColumnFlags_WidthFixed, labelW );
		ImGui::TableSetupColumn( "##v", ImGuiTableColumnFlags_WidthStretch );
		StatRow( "Backend", "%s", backend );
		StatRow( "Frame", "%.2f ms  (%.0f FPS)", s_smoothedFrameMs, fps );
		StatRow( "CPU", "%.2f ms", s_smoothedCpuMs );
		StatRow( "Draw calls", "%d  (approx)", st.drawCallCount );
		ImGui::EndTable();
	}

	if ( ImGui::CollapsingHeader( "Opaque instances", ImGuiTreeNodeFlags_DefaultOpen ) )
	{
		if ( ImGui::BeginTable( "##stats_opaque", 2, tableFlags ) )
		{
			ImGui::TableSetupColumn( "##l", ImGuiTableColumnFlags_WidthFixed, labelW );
			ImGui::TableSetupColumn( "##v", ImGuiTableColumnFlags_WidthStretch );
			StatRow( "Cubes", "%d", st.cubeCount );
			StatRow( "Spheres", "%d", st.sphereCount );
			StatRow( "Capsules", "%d", st.capsuleCount );
			StatRow( "Geom", "%d  (%d spans)", st.geomInstanceCount, st.geomSpanCount );
			ImGui::EndTable();
		}
	}
	if ( ImGui::CollapsingHeader( "Transparent instances" ) )
	{
		if ( ImGui::BeginTable( "##stats_xp", 2, tableFlags ) )
		{
			ImGui::TableSetupColumn( "##l", ImGuiTableColumnFlags_WidthFixed, labelW );
			ImGui::TableSetupColumn( "##v", ImGuiTableColumnFlags_WidthStretch );
			StatRow( "Cubes", "%d", st.cubeCountXp );
			StatRow( "Spheres", "%d", st.sphereCountXp );
			StatRow( "Capsules", "%d", st.capsuleCountXp );
			StatRow( "Geom", "%d", st.geomInstanceCountXp );
			ImGui::EndTable();
		}
	}
	if ( ImGui::CollapsingHeader( "Overlays" ) )
	{
		if ( ImGui::BeginTable( "##stats_overlay", 2, tableFlags ) )
		{
			ImGui::TableSetupColumn( "##l", ImGuiTableColumnFlags_WidthFixed, labelW );
			ImGui::TableSetupColumn( "##v", ImGuiTableColumnFlags_WidthStretch );
			StatRow( "Lines", "%d", st.lineCount );
			StatRow( "Points", "%d", st.pointCount );
			ImGui::EndTable();
		}
	}
	ImGui::Unindent();
}

// A dimmed "(?)" placed after a widget; hovering it reveals wrapped help
// text. Keeps long explanatory strings out of the panel body, where as inline
// TextDisabled lines they clipped against the window's right edge.
static void HelpMarker( const char* text )
{
	ImGui::SameLine();
	ImGui::TextDisabled( "(?)" );
	if ( ImGui::BeginItemTooltip() )
	{
		ImGui::PushTextWrapPos( ImGui::GetFontSize() * 26.0f );
		ImGui::TextUnformatted( text );
		ImGui::PopTextWrapPos();
		ImGui::EndTooltip();
	}
}

// The Render Settings section of the left "Renderer" window. Like
// StatsSection, a bare collapsing header with no window of its own.
static void SettingsSection( GuiToggles t )
{
	if ( !ImGui::CollapsingHeader( "Render Settings", ImGuiTreeNodeFlags_DefaultOpen ) )
		return;
	ImGui::Indent();

	// Reserve room on the right for the slider's value label - just enough
	// for the longest GTAO knob name ("Depth-MIP samp offset") to clear the
	// window edge. PushItemWidth with a negative value sets widget width to
	// (available - X); GetFontSize returns already-DPI-scaled pixels so
	// this stays correct at any monitor scale.
	ImGui::PushItemWidth( -ImGui::GetFontSize() * 12.0f );

	if ( ImGui::CollapsingHeader( "Tonemap", ImGuiTreeNodeFlags_DefaultOpen ) )
	{
		float ev = GetExposure();
		if ( ImGui::SliderFloat( "EV", &ev, -8.0f, +4.0f, "%.2f" ) )
		{
			SetExposure( ev );
		}
		float sat = GetToneMapSaturation();
		if ( ImGui::SliderFloat( "Saturation", &sat, 0.0f, 2.0f, "%.2f" ) )
		{
			SetToneMapSaturation( sat );
		}
		HelpMarker( "1.0 = stock AgX; raise to undo desaturation." );
	}

	if ( ImGui::CollapsingHeader( "Sun", ImGuiTreeNodeFlags_DefaultOpen ) )
	{
		Sun sun = GetSun();
		bool changed = false;
		float dir[3] = { sun.dirToSun.x, sun.dirToSun.y, sun.dirToSun.z };
		float color[3] = { sun.color.x, sun.color.y, sun.color.z };
		// %.2f (not %.3f): the three components share one slider width, so
		// at the panel's width a 3-decimal value crowds the slider grab. Two
		// decimals is plenty for a direction the renderer renormalizes anyway.
		if ( ImGui::SliderFloat3( "Direction", dir, -1.0f, 1.0f, "%.2f" ) )
		{
			sun.dirToSun = b3Vec3{ dir[0], dir[1], dir[2] };
			changed = true;
		}
		// HDR color with no upper cap on the slider - sun strength sits in
		// the 7-8 range under AgX. ColorEdit3 with HDR flag gets a wider
		// picker. NoInputs drops the inline RGB fields (just a swatch; click
		// to open the picker) - consistent with the other color editors here.
		if ( ImGui::ColorEdit3( "Color", color,
								ImGuiColorEditFlags_HDR | ImGuiColorEditFlags_Float | ImGuiColorEditFlags_NoInputs ) )
		{
			sun.color = b3Vec3{ color[0], color[1], color[2] };
			changed = true;
		}
		if ( ImGui::SliderFloat( "Ambient", &sun.ambient, 0.0f, 1.0f, "%.3f" ) )
		{
			changed = true;
		}
		if ( changed )
		{
			SetSun( sun );
		}
	}

	if ( ImGui::CollapsingHeader( "Sky", ImGuiTreeNodeFlags_DefaultOpen ) )
	{
		float t_turb = GetSkyTurbidity();
		if ( ImGui::SliderFloat( "Turbidity", &t_turb, 1.5f, 10.0f, "%.2f" ) )
		{
			SetSkyTurbidity( t_turb );
		}
		HelpMarker( "Lower = clearer; higher = hazier." );
	}

	if ( ImGui::CollapsingHeader( "IBL", ImGuiTreeNodeFlags_DefaultOpen ) )
	{
		bool ibl = GetIblEnabled();
		if ( ImGui::Checkbox( "Enable##ibl", &ibl ) )
		{
			SetIblEnabled( ibl );
		}
		HelpMarker( "Off = flat ambient (base color x Sun ambient)." );
	}

	if ( ImGui::CollapsingHeader( "Shadows", ImGuiTreeNodeFlags_DefaultOpen ) )
	{
		if ( t.shadowsOn )
		{
			// "##shadows" label for uniqueness from the GTAO Enable below 
			ImGui::Checkbox( "Enable##shadows", t.shadowsOn );
		}
		else
		{
			ImGui::TextDisabled( "(no toggle wired)" );
		}
	}

	if ( ImGui::CollapsingHeader( "GTAO", ImGuiTreeNodeFlags_DefaultOpen ) )
	{
		if ( t.gtaoOn )
		{
			ImGui::Checkbox( "Enable##gtao", t.gtaoOn );
		}
		GtaoTraceParams p = GetGtaoTraceParams();
		bool changed = false;

		// Quality preset (trace slice/step counts - the dominant ALU cost).
		static const char* k_qualityNames[] = { "Medium", "High", "Ultra" };
		const int k_qualityCount = (int)( sizeof( k_qualityNames ) / sizeof( k_qualityNames[0] ) );
		int quality = AO_QUALITY_HIGH;
		for ( int q = 0; q < k_qualityCount; ++q )
		{
			GtaoTraceParams preset = GetGtaoTraceParamsPreset( (AmbientOcclusionQuality)q );
			if ( preset.sliceCount == p.sliceCount && preset.stepsPerSlice == p.stepsPerSlice )
			{
				quality = q;
				break;
			}
		}
		if ( ImGui::Combo( "Quality", &quality, k_qualityNames, k_qualityCount ) )
		{
			GtaoTraceParams preset = GetGtaoTraceParamsPreset( (AmbientOcclusionQuality)quality );
			p.sliceCount = preset.sliceCount;
			p.stepsPerSlice = preset.stepsPerSlice;
			changed = true;
		}

		changed |= ImGui::SliderFloat( "Radius", &p.radius, 0.05f, 4.0f, "%.3f" );
		changed |= ImGui::SliderFloat( "Falloff range", &p.falloff, 0.05f, 1.0f, "%.3f" );
		changed |= ImGui::SliderFloat( "Radius multiplier", &p.radiusMul, 0.5f, 3.0f, "%.3f" );
		changed |= ImGui::SliderFloat( "Final value power", &p.finalValuePow, 0.5f, 4.0f, "%.3f" );
		changed |= ImGui::SliderFloat( "Denoise blur beta", &p.denoiseBlurBeta, 0.0f, 4.0f, "%.3f" );
		changed |= ImGui::SliderInt( "Denoise passes", &p.denoisePassCount, 1, GTAO_MAX_DENOISE_PASSES );
		changed |= ImGui::SliderFloat( "Sample dist. power", &p.sampleDistPow, 0.5f, 4.0f, "%.3f" );
		changed |= ImGui::SliderFloat( "Thin occluder comp.", &p.thinOcclComp, 0.0f, 1.0f, "%.3f" );
		changed |= ImGui::SliderFloat( "Depth-MIP samp offset", &p.depthMipSampOffset, 0.0f, 5.0f, "%.3f" );
		if ( changed )
		{
			SetGtaoTraceParams( p );
		}
		if ( ImGui::Button( "Reset to defaults (High)" ) )
		{
			SetGtaoTraceParams( GetDefaultGtaoTraceParams() );
		}
	}

	if ( ImGui::CollapsingHeader( "Debug view" ) )
	{
		if ( t.debugViewMode )
		{
			static const char* k_names[] = {
				"0 - lit", "1 - view-space distance", "2 - CSM cascade index", "3 - view-space normal", "4 - raw GTAO",
			};
			for ( int i = 0; i < 5; ++i )
			{
				if ( ImGui::RadioButton( k_names[i], *t.debugViewMode == i ) )
				{
					*t.debugViewMode = i;
				}
			}
		}
		else
		{
			ImGui::TextDisabled( "(no toggle wired)" );
		}
	}

	if ( ImGui::CollapsingHeader( "Edge overlay" ) )
	{
		EdgeOverlayParams ep = GetEdgeOverlayParams();
		bool changed = false;
		changed |= ImGui::Checkbox( "Enable##edges", &ep.enabled );
		changed |= ImGui::Checkbox( "Show hull edges (transparent only)", &ep.showHulls );
		changed |= ImGui::Checkbox( "Show mesh edges", &ep.showMeshes );
		changed |= ImGui::Checkbox( "Show heightfield edges", &ep.showHeightfields );
		changed |= ImGui::SliderFloat( "Thickness (px)", &ep.thicknessPx, 0.5f, 6.0f, "%.1f" );
		changed |= ImGui::SliderFloat( "Z-bias", &ep.zBias, 0.0f, 1.0e-3f, "%.6f" );
		float cv[3] = { ep.convexColor.x, ep.convexColor.y, ep.convexColor.z };
		float cc[3] = { ep.concaveColor.x, ep.concaveColor.y, ep.concaveColor.z };
		if ( ImGui::ColorEdit3( "Convex color", cv, ImGuiColorEditFlags_Float | ImGuiColorEditFlags_NoInputs ) )
		{
			ep.convexColor = MakeVec4( cv[0], cv[1], cv[2], ep.convexColor.w );
			changed = true;
		}
		if ( ImGui::ColorEdit3( "Concave color", cc, ImGuiColorEditFlags_Float | ImGuiColorEditFlags_NoInputs ) )
		{
			ep.concaveColor = MakeVec4( cc[0], cc[1], cc[2], ep.concaveColor.w );
			changed = true;
		}
		changed |= ImGui::SliderFloat( "Convex alpha", &ep.convexColor.w, 0.0f, 1.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Concave alpha", &ep.concaveColor.w, 0.0f, 1.0f, "%.2f" );
		if ( ImGui::Button( "Reset to defaults##edges" ) )
		{
			ep = GetDefaultEdgeParams();
			ep.enabled = true; // Reset keeps the panel useful by leaving the master on.
			changed = true;
		}
		if ( changed )
		{
			SetEdgeOverlayParams( &ep );
		}
	}

	if ( ImGui::CollapsingHeader( "Highlight outline" ) )
	{
		HighlightParams hp = GetHighlightParams();
		bool changed = false;
		changed |= ImGui::Checkbox( "Enable##highlight", &hp.enabled );
		changed |= ImGui::SliderFloat( "Hover thickness (px)", &hp.hoverThicknessPx, 1.0f, 4.0f, "%.1f" );
		changed |= ImGui::SliderFloat( "Select thickness (px)", &hp.selectThicknessPx, 1.0f, 4.0f, "%.1f" );
		float hc[3] = { hp.hoverColor.x, hp.hoverColor.y, hp.hoverColor.z };
		float sc[3] = { hp.selectColor.x, hp.selectColor.y, hp.selectColor.z };
		if ( ImGui::ColorEdit3( "Hover color", hc, ImGuiColorEditFlags_Float | ImGuiColorEditFlags_NoInputs ) )
		{
			hp.hoverColor = MakeVec4( hc[0], hc[1], hc[2], hp.hoverColor.w );
			changed = true;
		}
		if ( ImGui::ColorEdit3( "Select color", sc, ImGuiColorEditFlags_Float | ImGuiColorEditFlags_NoInputs ) )
		{
			hp.selectColor = MakeVec4( sc[0], sc[1], sc[2], hp.selectColor.w );
			changed = true;
		}
		changed |= ImGui::SliderFloat( "Hover alpha", &hp.hoverColor.w, 0.0f, 1.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Select alpha", &hp.selectColor.w, 0.0f, 1.0f, "%.2f" );
		if ( ImGui::Button( "Reset to defaults##highlight" ) )
		{
			hp = GetDefaultHighlightParams();
			changed = true;
		}
		if ( changed )
		{
			SetHighlightParams( &hp );
		}
	}

	ImGui::PopItemWidth();
	ImGui::Unindent();
}

// The left "Renderer" window - Stats stacked over Render Settings, anchored
// to the left edge. The two are collapsing-header sections within one window.
// Width is sized to the Render Settings sliders (the widest content); Stats
// fits comfortably inside it.
static void LeftPanel( GuiToggles t )
{
	if ( !BeginPanel( "Renderer", GUI_ANCHOR_LEFT, 300.0f ) )
	{
		EndPanel();
		return;
	}
	StatsSection();
	SettingsSection( t );
	EndPanel();
}

// Text overlay. Drain the per-frame label arena (filled by DrawString /
// DrawScreenString and the Box3D adapter's DrawString callback). World
// entries project to screen pixels with the last rendered camera, screen
// entries pass through their pixel position, and both emit into ImGui's
// background draw list - labels sit on the scene but under any ImGui
// windows. The drain runs in StartUIFrame because the calling app
// guarantees RenderFrame already ran this frame, so GetCameraState
// returns the matrices the scene was actually rasterized with.
static void RenderText()
{
	const int n = GetTextCount();
	if ( n <= 0 )
		return;

	const CameraState cam = GetCameraState();
	if ( cam.viewportW <= 0 || cam.viewportH <= 0 )
		return;

	// ImGui runs in physical framebuffer pixels (see InitUI), so the main
	// viewport's Size matches cam.viewportW/H. Project against the viewport
	// Size and offset by vp->Pos so this stays correct even if ImGui's space
	// ever gains an origin offset. Aspect is encoded in cam.proj, so any
	// width/height with the scene's ratio yields identical NDC.
	ImGuiViewport* vp = ImGui::GetMainViewport();
	const int vpW = (int)vp->Size.x;
	const int vpH = (int)vp->Size.y;
	if ( vpW <= 0 || vpH <= 0 )
		return;
	ImDrawList* dl = ImGui::GetBackgroundDrawList();
	const ImVec2 origin = vp->Pos;

	for ( int i = 0; i < n; ++i )
	{
		const TextEntry* e = GetTextAt( i );
		float sx, sy;
		if ( !ResolveTextScreenPos( e, cam.view, cam.proj, vpW, vpH, &sx, &sy ) )
		{
			continue;
		}
		// Linear-to-byte: pass straight through (no sRGB encode) - matches
		// how Box3D's b3HexColor literals reach the Draw* path, so a label
		// color reads the same as the SVG hex it came from.
		const auto byte = []( float v ) -> uint32_t {
			if ( v < 0.0f )
				v = 0.0f;
			if ( v > 1.0f )
				v = 1.0f;
			return (uint32_t)( v * 255.0f + 0.5f );
		};
		const uint32_t col = IM_COL32( byte( e->color.x ), byte( e->color.y ), byte( e->color.z ), byte( e->color.w ) );
		dl->AddText( nullptr, 0.0f, ImVec2( origin.x + sx, origin.y + sy ), col, e->text );
	}
}

void StartUIFrame( float dtSec, GuiToggles toggles )
{
	if ( !s_uiInitialized )
	{
		return;
	}

	simgui_frame_desc_t fd = {};
	fd.width = sapp_width();
	fd.height = sapp_height();

	// Clamp the delta we hand to ImGui to at least ~1ms. With swap_interval=0
	// sokol_app can call OnFrame back-to-back fast enough that stm_laptime
	// returns sub-microsecond deltas, and once cast to float ImGui's g.Time
	// fails to advance. That breaks the per-frame reset of the BG/FG draw
	// lists (their reset gate is `(float)BgFgDrawListsLastTimeActive !=
	// (float)g.Time`), so subsequent AddText calls into the background draw
	// list pile on top of the prior frame's vertices instead of replacing
	// them - visible as in-world labels flickering bright/dim as overlapping
	// copies alpha-blend over each other.
	const float kMinDt = 1.0f / 1000.0f;
	fd.delta_time = ( dtSec >= kMinDt ) ? (double)dtSec : (double)kMinDt;

	// dpi_scale 1.0 on purpose: it keeps ImGui in physical framebuffer pixels
	// (DisplaySize = framebuffer, DisplayFramebufferScale = 1.0). The display
	// scale is baked into the font and style in InitUI instead. See there.
	fd.dpi_scale = 1.0f;
	simgui_new_frame( &fd );

	// Reset the edge-anchor cursors before any panel opens - they accumulate
	// per panel within this frame and must start fresh each frame.
	ResetUIPanelLayout();

	if ( s_panelsVisible )
	{
		LeftPanel( toggles );

		if ( s_uiCallback != nullptr )
		{
			s_uiCallback();
		}
	}

	RenderText();
}

void RenderUI( const sg_swapchain* swapchain )
{
	if ( !s_uiInitialized )
	{
		return;
	}

	sg_pass pass = {};
	pass.action.colors[0].load_action = SG_LOADACTION_LOAD; // sit on top of tonemap
	pass.action.depth.load_action = SG_LOADACTION_DONTCARE;
	pass.action.stencil.load_action = SG_LOADACTION_DONTCARE;
	pass.swapchain = *swapchain;
	pass.label = "imgui";
	sg_begin_pass( &pass );
	simgui_render();
	sg_end_pass();
}
