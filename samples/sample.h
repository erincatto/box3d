// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "host/camera.h"

#include "box3d/types.h"

// Polled key state for samples that need continuous input (character movers).
// Fed from the host event loop; read with the KEY_* aliases in gfx/keycodes.h.
bool IsKeyDown( int key );
void SetKeyDown( int key, bool down );

// Key action passed to Keyboard, matching the old GLFW press/release values so
// sample code reads naturally without leaking sokol_app event types.
enum
{
	ACTION_RELEASE = 0,
	ACTION_PRESS = 1,
};

struct SampleContext
{
	void Save();
	void Load();

	Camera camera;
	bool minimized = false;
	class Sample* sample = nullptr;
	b3Capacity capacity;

	// Latest cursor position in framebuffer pixels, fed from the host event
	// loop. Drives picking on the step that follows a mouse move.
	float mouseX = 0.0f;
	float mouseY = 0.0f;

	// Relative cursor motion for the latest move, fed from the host. Used for
	// third-person look while the cursor is locked.
	float mouseDX = 0.0f;
	float mouseDY = 0.0f;

	int windowWidth = 1920;
	int windowHeight = 1080;

	// Used to shift the origin and test round-off problems
	b3Vec3 origin = b3Vec3_zero;

	// Recording output path and the path the Replay viewer opens. Edited in the UI and persisted.
	char recordingFile[256] = "recording.b3rec";
	char replayFile[256] = "";

	// Keyframe ring policy the Replay viewer applies on open, persisted across sessions.
	int replayKeyframeBudgetMB = 512;
	int replayKeyframeMinInterval = 16;

	float hertz = 60.0f;
	float recycleDistance = 0.05f;
	float drawDistance = 100.0f; // meters, view/cull box half extent, persisted
	int subStepCount = 4;
	int workerCount = 1;
	bool transparentDynamic = false;
	bool transparent = false;
	bool enableWarmStarting = true;
	bool enableContinuous = true;
	bool enableSleep = true;
	bool pause = false;
	int singleStep = 0;
	bool restart = false;

	// UI visibility (Tab / View > Hide UI). When hidden only the minimal HUD shows.
	bool showUI = true;

	// Bottom diagnostics drawer (M), set by Ctrl+O for the fuzzy picker.
	bool showMetrics = false;
	bool openSamplePicker = false;

	// Controls help window (Help > Controls, toggled with ?). Promoted to the
	// context so the key handler can reach it. Seeded from newUser after Load.
	bool showControls = false;

	// Deferred request from the Replay > Open menu. The native file dialog
	// blocks on a nested run loop, so it must run outside the frame (top of
	// OnFrame) rather than mid-frame inside the ImGui callback, which would
	// re-enter the render loop. See OpenReplayFileDialog.
	bool openReplayPicker = false;

	// Render toggles fed into FrameInput each frame. They live here, not in
	// main.cpp, so the Render menu and the frame loop share one source.
	bool enableShadows = true;
	bool enableGtao = true;

	// View-only: treat simulation +Z as up when drawing. Does not touch the
	// simulation. The frame loop hands this to the camera's render transform.
	bool viewZUp = false;

	float sunStrength = 1.0f;
	int debugView = 0;

	int sampleIndex = 0;

	// No settings file found yet. The first run opens the replay viewer with the
	// controls window up, so there is something to watch and a key to every action.
	bool newUser = true;
};

class Sample
{
public:
	explicit Sample( SampleContext* context );
	virtual ~Sample();

	void CreateWorld( b3Capacity* capacity );

	// Position the first HUD text line below the menu bar, or near the top when
	// the UI is hidden. Mirrors Box2D ResetText.
	void ResetText();

	// Update and render are split to support pausing the simulation
	virtual void Step();

	virtual void Render() {}

	// Draw sample controls into the shared info panel. Return true if any widget
	// was drawn so the panel can add a separator.
	virtual bool DrawControls()
	{
		return false;
	}

	// Let a debug sample hide the Solver section.
	virtual bool HasSolverControls() const
	{
		return true;
	}

	// Width of the right info panel in em. The bottom drawer clears to the same width. A sample that
	// hosts heavier content there, such as the replay viewer's detail pane, can widen it.
	virtual float InfoPanelWidthEm() const;

	// Bounds the Frame shortcut should fit when the selection is not a body, e.g. a recorded query.
	// Takes priority over FocusBody so an explicit selection wins over a hovered body. Returns false
	// by default, leaving the body and whole-scene paths in charge.
	virtual bool FocusBounds( b3AABB* bounds );

	// Body the Frame shortcut should fit. Defaults to whatever the cursor is over.
	// The replay viewer overrides this to fit its persistent selection instead, so
	// framing works regardless of where the cursor sits.
	virtual b3BodyId FocusBody() const;

	// Frame shortcut with nothing selected: fit the whole scene. Defaults to the live world bounds.
	// The replay viewer overrides this to fit the recording, whose world is player-owned and separate
	// from the empty base world.
	virtual void FocusHome();

	// Arm recording on the live world, snapshotting it as the seed so capture can begin at any
	// step boundary. Stop writes the file named by the context and frees the buffer.
	void StartRecording();
	void FinishRecording();

	// Bottom diagnostics drawer (Profile / Counters / Renderer / Frame Time).
	void DrawMetrics();

	// Append extra tabs to the diagnostics drawer's tab bar (the Replay viewer adds Timeline).
	virtual void DrawMetricsTab()
	{
	}

	// Draw extra top-level windows when the UI is shown (the Replay viewer adds the Outline panel).
	virtual void DrawSampleWindows()
	{
	}

	virtual void Keyboard( int key, int action, int modifiers )
	{
	}

	virtual void MouseDown( b3Vec2 p, int button, int modifiers );
	virtual void MouseUp( b3Vec2 p, int button );
	virtual void MouseMove( b3Vec2 p );

	void ToggleThirdPerson();
	void DrawTextLine( const char* text, ... );
	void ResetProfile();

	// Static ground box at the origin, drawn with the procedural grid material
	b3BodyId AddGroundBox( float extent );

#if defined( NDEBUG )
	static constexpr bool m_isDebug = false;
#else
	static constexpr bool m_isDebug = true;
#endif

	static constexpr int m_maxTasks = 64;
	static constexpr int m_maxThreads = 64;
	static constexpr int m_profileCapacity = 512;

	SampleContext* m_context;
	Camera* m_camera;

	b3WorldId m_worldId;

	b3Pos m_mousePoint;
	b3BodyId m_mouseBodyId;
	b3JointId m_mouseJointId;
	float m_mouseFraction;
	float m_mouseForceScale;
	float m_launchSpeedScale;
	int m_stepCount;
	int m_textLine;
	int m_textIncrement;
	int m_triangleIndex;
	uint64_t m_userMaterialId;

	// Active recording, or null when not recording. The step recording began on drives the status.
	struct b3Recording* m_recording;
	int m_recordStartStep;

	b3Profile m_profiles[m_profileCapacity];
	int m_currentProfileIndex;
	int m_profileReadIndex;
	int m_profileWriteIndex;

	b3Vec2 m_mouseLast;
	b3Vec2 m_mouseDelta;
	bool m_didStep;
	bool m_stepWhilePaused;
	bool m_haveMouseLast;
};

#define MAX_SAMPLES 256

typedef Sample* SampleCreateFcn( SampleContext* context );

struct SampleEntry
{
	const char* Category;
	const char* Name;
	SampleCreateFcn* CreateFcn;
};

extern SampleEntry g_sampleEntries[MAX_SAMPLES];
extern int g_sampleCount;

// Index of the registered Replay viewer, or -1 if none. Gates the Replay menu.
extern int g_replayIndex;

int RegisterSample( const char* category, const char* name, SampleCreateFcn* fcn );

// Register the Replay viewer and record its index in g_replayIndex.
int RegisterReplay( const char* category, const char* name, SampleCreateFcn* fcn );

// Destroy the active sample and build the selected one. restart keeps the camera
// by leaving the restart flag set while the new sample constructs.
void SelectSample( SampleContext* context, int selection, bool restart );

// Run the native "open replay" file picker and, on success, hand the chosen
// file to the Replay viewer. Must be called outside the frame (the dialog spins
// a blocking nested run loop), driven by SampleContext::openReplayPicker.
void OpenReplayFileDialog( SampleContext* context );

// The single host UI callback: menu bar, sample picker, info panel, and the
// bottom diagnostics drawer.
void DrawUI( SampleContext* context );

struct CastClosestContext
{
	b3ShapeId shapeId;
	b3Pos point;
	b3Vec3 normal;
	float fraction;
	uint64_t materialId;
	int childIndex;
	int triangleIndex;
	bool hit;
};

float CastClosestCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t materialId, int triangleIndex,
						   int childIndex, void* context );

struct MoverShapeUserData
{
	float maxPush;
	bool clipVelocity;
};

struct PlaneExtra
{
	b3Pos point;
	b3ShapeId shapeId;
};

struct CharacterMover
{
	static constexpr int m_planeCapacity = 8;
	static constexpr float m_jumpSpeed = 5.0f;
	static constexpr float m_maxSpeed = 6.0f;
	static constexpr float m_minSpeed = 0.01f;
	static constexpr float m_stopSpeed = 1.0f;
	static constexpr float m_accelerate = 30.0f;
	static constexpr float m_friction = 4.0f;
	static constexpr float m_gravity = 15.0f;

	void Initialize( Sample* sample, b3Pos position );
	void SolveMove( float timeStep, b3Vec3 forward, b3Vec3 right, b3Vec2 throttle, bool clipVelocity );
	void Step( b3ShapeId* ignoreShapes, int ignoreCount, bool clipVelocity );

	Sample* m_sample;
	b3WorldTransform m_transform;
	b3Vec3 m_velocity;
	b3Capsule m_capsule;
	b3CollisionPlane m_planes[m_planeCapacity] = {};
	PlaneExtra m_planeExtras[m_planeCapacity] = {};
	int m_planeCount;
	int m_totalIterations;
	float m_pogoVelocity;
	bool m_onGround;
	bool m_sprint;

	// Transient
	b3ShapeId* m_ignoreShapeIds;
	int m_ignoreCount;
};
