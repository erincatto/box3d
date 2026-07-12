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
	b3Pos origin;
	b3Vec3 translation;
};

class Camera
{
public:
	Camera();

	// One knob for how far the camera reaches, in meters: the projection far plane,
	// the maximum orbit radius, and (scaled to length units) the draw cull box.
	static constexpr float kViewDistance = 1000.0f;

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

	// Forward is +view-Z (pivot -> eye), so the look direction is -GetForward().
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
	void SetView( float yawDegrees, float pitchDegrees, float radius, b3Pos pivot );

	void SetPivot( b3Pos pivot )
	{
		m_pivot = pivot;
	}
 
	void SetTarget( b3Pos target )
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

	// Draw/cull box half extent around the eye, in meters. Drives DrawBounds.
	void SetDrawDistance( float meters )
	{
		m_drawDistance = meters;
	}

	// Recomputes the projection. Call after any SetFov / SetClip change, or
	// when the target framebuffer aspect changes outside Update().
	void RebuildProj( float aspect );

	// Recompute the basis + view from yaw/pitch/pivot/radius. Public entry
	// point for sample code that mutates m_pivot directly (third-person follow).
	void UpdateTransform();

	// World-space pick ray from a framebuffer-pixel coordinate. Unprojects
	// through the camera's own eye-relative matrices and framebuffer size, then
	// lifts the origin to absolute world with the double eye. Hands back a zero
	// ray if the camera has no size yet, so callers never read uninitialized values.
	PickRay BuildPickRay( float x, float y ) const;

	// Set the simulation->display transform: a uniform scale of 1/lengthUnitsPerMeter
	// so the scene renders in meters, plus an optional rotation taking simulation +Z
	// to view up. Folded into the view matrix, so simulation data and the draw origin
	// stay in length units and only the rendered result is scaled and reoriented. The
	// camera itself then lives in meters, so radius and speeds read in meters.
	void SetRenderTransform( float lengthUnitsPerMeter, bool zUp );

	// Eye in simulation space (length units). The renderer shifts simulation-space
	// geometry against this. The view folds in the sim->display map afterward. With
	// an identity transform this is just m_worldEye.
	b3Pos DrawOrigin() const
	{
		Vec4 e = MulMV4( m_renderXformInv, MakeVec4( m_worldEye.x, m_worldEye.y, m_worldEye.z, 1.0f ) );
		return b3Pos{ e.x, e.y, e.z };
	}

	// Cull box for b3World_Draw, in simulation space (length units): a cube of the
	// draw distance centered on the simulation eye. The broad phase is queried in
	// length units, hence the scale.
	b3AABB DrawBounds() const
	{
		float h = m_drawDistance * m_lengthUnitsPerMeter;
		b3Vec3 r = { h, h, h };
		return b3OffsetAABB( { b3Neg( r ), r }, DrawOrigin() );
	}

	b3Pos m_pivot; // look-at point, display space (meters). Double precision so it stays exact far from origin.
	float m_yaw;	// radians, around Y
	float m_pitch;	// radians, around camera-frame X
	float m_radius; // meters from pivot

	float m_fov; // radians, vertical
	float m_near;
	float m_far;

	// Half extent of the draw/cull box around the eye (meters). Independent of the
	// far plane, so it shrinks the draw set without touching the projection.
	float m_drawDistance;

	// Fly-mode translation speed, m/s. Scroll-tunable while right-mouse held.
	float m_speed;

	// Latched framebuffer pixels, refreshed each Update. Sample UI anchors
	// panels relative to m_height.
	int m_width;
	int m_height;

	// Locks input to wheel-zoom. The sample drives m_pivot to a followed body
	// and calls UpdateTransform. Eye placement stays the orbit formula.
	bool m_thirdPerson;

	// Cached basis: forward is +view-Z, i.e. pivot->eye) refreshed
	// alongside m_view / m_viewInv whenever yaw/pitch/pivot/radius change.
	// These let consumers (picking, shadow frustum) skip re-inverting m_view.
	//
	// The view is built in the eye-relative frame, so m_position (the eye in that
	// frame) is always zero and the view matrix is translation free. m_worldEye is
	// the eye in world space, the draw origin the sample renders and picks against.
	b3Vec3 m_position;
	b3Pos m_worldEye;
	b3Vec3 m_right;
	b3Vec3 m_up;
	b3Vec3 m_forward;

	// All four matrices are produced together: view / viewInv from the basis,
	// proj / projInv from fov/aspect/near/far.
	Mat4 m_view;
	Mat4 m_viewInv;
	Mat4 m_proj;
	Mat4 m_projInv;

	// Simulation -> display transform and its inverse, folded into m_view / m_viewInv
	// by RebuildBasisAndView. Identity until SetRenderTransform runs, so the default
	// is meters and y-up.
	Mat4 m_renderXform;
	Mat4 m_renderXformInv;
	float m_lengthUnitsPerMeter;
	bool m_zUp;

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
