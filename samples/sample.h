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

	float hertz = 60.0f;
	float recycleDistance = 0.05f;
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

	// Render toggles fed into FrameInput each frame. They live here, not in
	// main.cpp, so the Render menu and the frame loop share one source.
	bool enableShadows = true;
	bool enableGtao = true;

	float sunStrength = 1.0f;
	int debugView = 0;

	int sampleIndex = 0;
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

	virtual void Render();

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

	// Bottom diagnostics drawer (Profile / Counters / Renderer / Frame Time).
	void DrawMetrics();

	virtual void Keyboard( int key, int action, int modifiers )
	{
	}

	virtual void MouseDown( b3Vec2 p, int button, int modifiers );
	virtual void MouseUp( b3Vec2 p, int button );
	virtual void MouseMove( b3Vec2 p );

	void ToggleThirdPerson();
	void DrawTextLine( const char* text, ... );
	void ResetProfile();

	// Latch the draw origin to the current camera eye and push it to the host overlay system.
	// Call after any camera move (Step, Render, third person follow) so debug shapes and host
	// overlays demote against the same point the translation free view renders from.
	void SyncDrawOrigin();

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

	// World position the scene is drawn and picked relative to: the camera eye, refreshed each
	// frame from the camera so the float render frame stays near the origin wherever it roams.
	b3Pos m_drawOrigin;

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

int RegisterSample( const char* category, const char* name, SampleCreateFcn* fcn );

// Destroy the active sample and build the selected one. restart keeps the camera
// by leaving the restart flag set while the new sample constructs.
void SelectSample( SampleContext* context, int selection, bool restart );

// The single host UI callback: menu bar, sample picker, info panel, and the
// bottom diagnostics drawer.
void DrawUI( SampleContext* context );

struct CastClosestContext
{
	b3ShapeId shapeId;
	b3Vec3 point;
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
	b3Vec3 point;
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

	void Initialize( Sample* sample, b3Vec3 position );
	void SolveMove( float timeStep, b3Vec3 forward, b3Vec3 right, b3Vec2 throttle, bool clipVelocity );
	void Step( b3ShapeId* ignoreShapes, int ignoreCount, bool clipVelocity );

	Sample* m_sample;
	b3Transform m_transform;
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
