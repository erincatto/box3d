// Orbit + fly camera, modeled after Box3D's sample camera.
//
// Two modes share one state. Sticky mouse-button + Alt-modifier flags
// select which mode each gesture drives:
//
//   ORBIT (Alt held)
//     Alt + left-drag    : orbit (yaw/pitch around pivot)
//     Alt + middle-drag  : pan pivot in view-space XY
//     Alt + right-drag-Y : radial zoom (adjust radius)
//
//   FLY (right mouse held, no Alt)
//     right-drag         : FPS look (yaw/pitch the view direction)
//     WASD               : translate eye along view forward/right at m_speed
//     scroll             : tune m_speed
//
//   Always
//     bare scroll        : multiplicative zoom on radius
//
// OnEvent accumulates input deltas; Update consumes them and folds them
// into the camera state. View / Proj produce the matrices the renderer
// consumes.
//
// m_pivot is the look-at pivot, m_radius the radial offset, m_yaw / m_pitch
// the spherical angles in radians. In fly mode the eye is the primary anchor
// and m_pivot is back-derived so re-entering orbit mode preserves the look
// direction. Box3D's sample camera keeps yaw/pitch in degrees, so SetView
// converts at the boundary.

#pragma once

#include "gfx/utility.h"

struct sapp_event;

// World-space ray from a screen pixel: origin sits on the near plane,
// translation spans near -> far. Feeds straight into b3World_CastRay*.
struct PickRay
{
	b3Vec3 origin;
	b3Vec3 translation;
};

class Camera
{
public:
	Camera();

	// Feed each sokol_app event here. Safe to call from event_cb at any
	// time during the frame; deltas are summed and consumed by Update().
	void OnEvent( const sapp_event* e );

	// Width/height are the latched framebuffer pixels; aspect derives from
	// them and they back BuildPickRay and ImGui bottom-anchored panels.
	void Update( float dt, int width, int height );

	Mat4 View() const
	{
		return m_view;
	}
	Mat4 ViewInverse() const
	{
		return m_viewInv;
	}
	Mat4 Proj() const
	{
		return m_proj;
	}
	Mat4 ProjInverse() const
	{
		return m_projInv;
	}

	b3Vec3 Position() const;

	// Cached basis accessors (Box3D parity). Forward is +view-Z (pivot -> eye),
	// so the look direction is -GetForward().
	b3Vec3 GetPosition() const
	{
		return m_position;
	}
	b3Vec3 GetForward() const
	{
		return m_forward;
	}
	b3Vec3 GetRight() const
	{
		return m_right;
	}
	b3Vec3 GetUp() const
	{
		return m_up;
	}

	// Snap to a fixed orientation; used by scene init paths that frame
	// the camera independently of mouse state.
	void SetOrbit( float yawRadians, float pitchRadians, float radius );

	// Box3D's sample signature: angles in degrees, pivot folded in. Mirrors
	// the ~140 sample call sites exactly so they need no edits.
	void SetView( float yawDegrees, float pitchDegrees, float radius, b3Vec3 pivot = b3Vec3_zero );

	void SetPivot( b3Vec3 pivot )
	{
		m_pivot = pivot;
	}
	// Forwarder for existing render3d call sites.
	void SetTarget( b3Vec3 target )
	{
		m_pivot = target;
	}

	// Frame an AABB: keep current yaw/pitch, move pivot to the AABB center,
	// and refit radius so the AABB fits in view at the current FOV+aspect.
	// `padding` (>= 1) scales the fit radius (1.2 leaves a small margin).
	// A degenerate / invalid AABB is a no-op.
	void Frame( b3AABB aabb, float aspect, float padding );
	void SetFov( float fovRadians )
	{
		m_fov = fovRadians;
	}
	void SetClip( float near, float far )
	{
		m_near = near;
		m_far = far;
	}

	// Recomputes the projection. Call after any SetFov / SetClip change, or
	// when the target framebuffer aspect changes outside Update().
	void RebuildProj( float aspect );

	// Recompute the basis + view from yaw/pitch/pivot/radius. Public entry
	// point for sample code that mutates m_pivot directly (third-person follow).
	void UpdateTransform();

	// World-space pick ray from a framebuffer-pixel coordinate, using the
	// latched camera state. Wraps the free function in gfx/picking.h; hands
	// back a zero-translation ray (origin at the eye) if that state is not
	// ready yet, so callers never read uninitialized values.
	PickRay BuildPickRay( float x, float y ) const;

	b3Vec3 m_pivot;
	float m_yaw;	// radians, around Y
	float m_pitch;	// radians, around camera-frame X
	float m_radius; // meters from pivot

	float m_fov; // radians, vertical
	float m_near;
	float m_far;

	// Fly-mode translation speed, m/s. Scroll-tunable while right-mouse held.
	float m_speed;

	// Latched framebuffer pixels, refreshed each Update. Sample UI anchors
	// panels relative to m_height.
	int m_width;
	int m_height;

	// Locks input to wheel-zoom; the sample drives m_pivot to a followed body
	// and calls UpdateTransform. Eye placement stays the orbit formula.
	bool m_thirdPerson;

	// Cached basis (Box3D-style: forward is +view-Z, i.e. pivot->eye), refreshed
	// alongside m_view / m_viewInv whenever yaw/pitch/pivot/radius change.
	// These let consumers (picking, shadow frustum) skip re-inverting m_view.
	b3Vec3 m_position;
	b3Vec3 m_right;
	b3Vec3 m_up;
	b3Vec3 m_forward;

	// All four matrices are produced together — view / viewInv from the basis,
	// proj / projInv from fov/aspect/near/far — so the renderer never inverts
	// at runtime.
	Mat4 m_view;
	Mat4 m_viewInv;
	Mat4 m_proj;
	Mat4 m_projInv;

	// Per-frame input deltas accumulated by OnEvent and zeroed by Update.
	// Routing into the right bucket happens in OnEvent based on which mouse
	// buttons + Alt are held when the event arrives.
	float m_orbitDX = 0.0f; // alt+left-drag, or right-drag (fly look)
	float m_orbitDY = 0.0f;
	float m_panDX = 0.0f; // alt+middle-drag
	float m_panDY = 0.0f;
	float m_radialZoomDY = 0.0f;	 // alt+right-drag, Y component
	float m_scrollAccum = 0.0f;		 // bare scroll: orbit zoom
	float m_speedScrollAccum = 0.0f; // scroll while right held (no alt): tune speed

	// Sticky button/key state, toggled by MOUSE_DOWN/UP and KEY_DOWN/UP.
	bool m_leftDown = false;
	bool m_rightDown = false;
	bool m_middleDown = false;
	bool m_altDown = false;
	bool m_wDown = false;
	bool m_aDown = false;
	bool m_sDown = false;
	bool m_dDown = false;
};
