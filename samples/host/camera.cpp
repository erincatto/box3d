#include "camera.h"

#include "sokol_app.h"

#include "box3d/math_functions.h"

#include <math.h>

static constexpr float HALF_PI = B3_PI * 0.5f;
static constexpr float DEG_TO_RAD = B3_PI / 180.0f;
static constexpr float ORBIT_SENS = 0.005f;		 // radians per pixel
static constexpr float FLY_LOOK_SENS = 0.005f;	 // radians per pixel
static constexpr float PAN_SENS = 0.005f;		 // meters per pixel per meter of radius
static constexpr float RADIAL_ZOOM_SENS = 0.02f; // meters per pixel (alt+right-drag)
static constexpr float FLY_SPEED_STEP = 1.0f;	 // m/s per scroll tick
static constexpr float ZOOM_STEP = 0.9f;		 // multiplier per scroll tick
static constexpr float MIN_DIST = 0.1f;
static constexpr float MIN_SPEED = 0.06f;	 // matches box3d's 0.001*60
static constexpr float MAX_SPEED = 30000.0f; // matches box3d's 500*60
static constexpr float PITCH_LIM = HALF_PI - 0.01f;
static constexpr float THIRD_PERSON_MIN_RADIUS = 0.25f; // matches box3d

// Build a reverse-Z RH projection and its inverse together by swapping the
// near/far args. mat4PerspectiveAndInverse normally puts ndc.z=0 at near, 1
// at far. With the arguments swapped you get 1 at near, 0 at far - what the
// project's pipeline expects (compare GREATER, far-clear 0). The inverse is
// closed-form from the same params so the un-project path stays accurate.
static void ReverseZPerspectiveAndInverse( Mat4* outProj, Mat4* outProjInv, float fov, float aspect, float n, float f )
{
	MakePerspectiveAndInverse( outProj, outProjInv, fov, aspect, f, n );
}

// Yaw/pitch -> the "+view-Z" direction (pivot -> camera). The actual looking
// direction is -ForwardFromAngles.
static b3Vec3 ForwardFromAngles( float yaw, float pitch )
{
	const float cp = cosf( pitch );
	return b3Vec3{ sinf( yaw ) * cp, sinf( pitch ), cosf( yaw ) * cp };
}

// Refresh the world-space basis and the view / viewInv pair from yaw / pitch /
// pivot / radius. Called from every state mutator (SetOrbit, Frame, Update)
// so consumers can read m_view / m_viewInv without an inversion step.
static void RebuildBasisAndView( Camera& c )
{
	const b3Vec3 forward = ForwardFromAngles( c.m_yaw, c.m_pitch );
	const b3Vec3 worldUp = { 0.0f, 1.0f, 0.0f };
	// up = worldUp re-orthogonalized against forward. right = up x forward.
	// Matches Box3D's UpdateTransform; produces a right-handed view basis.
	const b3Vec3 upUnnorm = b3Sub( worldUp, b3MulSV( b3Dot( worldUp, forward ), forward ) );
	const b3Vec3 up = b3Normalize( upUnnorm );
	const b3Vec3 right = b3Normalize( b3Cross( up, forward ) );

	// Eye in world space is the draw origin. Building the view in that relative frame puts the eye
	// at the origin, so the view matrix carries no translation and stays exact far from the origin.
	c.m_worldEye = b3OffsetPos( c.m_pivot, b3MulSV( c.m_radius, forward ) );
	c.m_position = b3Vec3_zero;
	c.m_right = right;
	c.m_up = up;
	c.m_forward = forward;

	// The basis view assumes meters, Y-up, eye at the origin. Geometry reaches the
	// GPU shifted against the draw origin in length units, so fold the sim->display
	// map in: view = basis * S maps a length-unit eye-relative point to view space.
	// The inverse mirrors it (viewInv = S^-1 * basisInv) so unproject and the sky's
	// invViewProj recover length-unit points.
	Mat4 viewBasis, viewBasisInv;
	MakeViewAndInverse( &viewBasis, &viewBasisInv, right, up, forward, c.m_position );
	c.m_view = MulMM4( viewBasis, c.m_renderXform );
	c.m_viewInv = MulMM4( c.m_renderXformInv, viewBasisInv );
}

Camera::Camera()
	: m_pivot{ 0.0f, 0.0f, 0.0f }
	, m_yaw( 35.0f * DEG_TO_RAD )
	, m_pitch( -25.0f * DEG_TO_RAD )
	, m_radius( 25.0f )
	, m_fov( 60.0f * DEG_TO_RAD )
	, m_near( 0.1f )
	, m_far( kViewDistance )
	, m_drawDistance( kViewDistance )
	, m_speed( 10.0f )
	, m_width( 0 )
	, m_height( 0 )
	, m_thirdPerson( false )
	, m_position{ 0.0f, 0.0f, 0.0f }
	, m_worldEye{ 0.0f, 0.0f, 0.0f }
	, m_right{ 1.0f, 0.0f, 0.0f }
	, m_up{ 0.0f, 1.0f, 0.0f }
	, m_forward{ 0.0f, 0.0f, 1.0f }
	, m_view( MakeIdentity() )
	, m_viewInv( MakeIdentity() )
	, m_proj( MakeIdentity() )
	, m_projInv( MakeIdentity() )
	, m_renderXform( MakeIdentity() )
	, m_renderXformInv( MakeIdentity() )
	, m_lengthUnitsPerMeter( 1.0f )
	, m_zUp( false )
{
	RebuildBasisAndView( *this );
	RebuildProj( 1.0f );
}

void Camera::RebuildProj( float aspect )
{
	if ( aspect <= 0.0f )
		aspect = 1.0f;
	ReverseZPerspectiveAndInverse( &m_proj, &m_projInv, m_fov, aspect, m_near, m_far );
}

void Camera::SetRenderTransform( float lengthUnitsPerMeter, bool zUp )
{
	m_lengthUnitsPerMeter = lengthUnitsPerMeter > 0.0f ? lengthUnitsPerMeter : 1.0f;
	m_zUp = zUp;

	const float s = 1.0f / m_lengthUnitsPerMeter; // meters per length unit
	const float u = m_lengthUnitsPerMeter;		  // length units per meter

	if ( zUp )
	{
		// S = s * Rx(-90): sim (x,y,z) -> display (x, z, -y), so sim +Z is view up.
		// A proper rotation (det +1), not an axis swap, so winding and normals stay
		// correct and back-face culling is not inverted.
		m_renderXform =
			Mat4{ { s, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, -s, 0.0f }, { 0.0f, s, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 1.0f } };
		m_renderXformInv =
			Mat4{ { u, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, u, 0.0f }, { 0.0f, -u, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 1.0f } };
	}
	else
	{
		m_renderXform =
			Mat4{ { s, 0.0f, 0.0f, 0.0f }, { 0.0f, s, 0.0f, 0.0f }, { 0.0f, 0.0f, s, 0.0f }, { 0.0f, 0.0f, 0.0f, 1.0f } };
		m_renderXformInv =
			Mat4{ { u, 0.0f, 0.0f, 0.0f }, { 0.0f, u, 0.0f, 0.0f }, { 0.0f, 0.0f, u, 0.0f }, { 0.0f, 0.0f, 0.0f, 1.0f } };
	}

	RebuildBasisAndView( *this );
}

// Map a simulation-space AABB into display space (meters, view up axis). S is a
// scaled signed-axis permutation, so transforming the two opposite corners and
// taking the per-axis min/max recovers the display AABB exactly.
static b3AABB TransformAABBToDisplay( const Mat4& s, b3AABB a )
{
	Vec4 lo = MulMV4( s, MakeVec4( a.lowerBound.x, a.lowerBound.y, a.lowerBound.z, 1.0f ) );
	Vec4 hi = MulMV4( s, MakeVec4( a.upperBound.x, a.upperBound.y, a.upperBound.z, 1.0f ) );
	b3AABB out;
	out.lowerBound = { fminf( lo.x, hi.x ), fminf( lo.y, hi.y ), fminf( lo.z, hi.z ) };
	out.upperBound = { fmaxf( lo.x, hi.x ), fmaxf( lo.y, hi.y ), fmaxf( lo.z, hi.z ) };
	return out;
}

void Camera::Frame( b3AABB aabb, float aspect, float padding )
{
	if ( !b3IsValidAABB( aabb ) )
	{
		return;
	}

	// Bounds arrive in simulation space. frame against the display-space box so the
	// fit accounts for the render scale and up axis.
	aabb = TransformAABBToDisplay( m_renderXform, aabb );
	b3Vec3 center = b3AABB_Center( aabb );
	b3Vec3 ext = b3AABB_Extents( aabb ); // half-extents
	float r = sqrtf( ext.x * ext.x + ext.y * ext.y + ext.z * ext.z );
	m_pivot = b3ToPos( center );
	if ( r < 1.0e-6f )
	{
		// Point-like. Preserve current radius, just retarget. The cached
		// position/basis depend on m_pivot, so refresh them before returning.
		RebuildBasisAndView( *this );
		return;
	}
	if ( aspect <= 0.0f )
	{
		aspect = 1.0f;
	}
	float halfFovY = 0.5f * m_fov;
	float invTan = 1.0f / tanf( halfFovY );
	float distV = r * invTan;
	float distH = r * invTan / aspect;
	float d = ( distV > distH ) ? distV : distH;
	d *= padding;
	SetOrbit( m_yaw, m_pitch, d );
}

void Camera::SetOrbit( float yawRadians, float pitchRadians, float radius )
{
	m_yaw = yawRadians;
	m_pitch = b3ClampFloat( pitchRadians, -PITCH_LIM, PITCH_LIM );
	m_radius = b3ClampFloat( radius, MIN_DIST, kViewDistance );
	RebuildBasisAndView( *this );
}

void Camera::SetView( float yawDegrees, float pitchDegrees, float radius, b3Pos pivot )
{
	SetPivot( pivot );
	SetOrbit( yawDegrees * DEG_TO_RAD, pitchDegrees * DEG_TO_RAD, radius );
}

void Camera::UpdateTransform()
{
	RebuildBasisAndView( *this );
}

b3Vec3 Camera::Position() const
{
	return m_position;
}

PickRay Camera::BuildPickRay( float x, float y ) const
{
	// Defined zero ray so callers never read uninitialized values when the
	// camera has no size yet or the unprojection is degenerate.
	PickRay ray;
	ray.origin = b3Pos_zero;
	ray.translation = b3Vec3_zero;

	if ( m_width <= 0 || m_height <= 0 )
	{
		return ray;
	}

	// Pixel -> NDC. Origin top-left, Y down. NDC has Y up, X right.
	float ndcX = ( 2.0f * x / (float)m_width ) - 1.0f;
	float ndcY = 1.0f - ( 2.0f * y / (float)m_height );

	// Reverse-Z: near maps to clip-Z = +w, far to 0. Unproject both clip points
	// through inv(P*V) = viewInv * projInv, no runtime matrix inversion. The
	// matrices are eye-relative, so the recovered points are too.
	Mat4 invVP = MulMM4( m_viewInv, m_projInv );

	Vec4 clipN = MakeVec4( ndcX, ndcY, 1.0f, 1.0f );
	Vec4 clipF = MakeVec4( ndcX, ndcY, 0.0f, 1.0f );

	Vec4 wN = MulMV4( invVP, clipN );
	Vec4 wF = MulMV4( invVP, clipF );

	if ( wN.w == 0.0f || wF.w == 0.0f )
	{
		return ray;
	}

	float invWN = 1.0f / wN.w;
	float invWF = 1.0f / wF.w;

	b3Vec3 nearWorld = { wN.x * invWN, wN.y * invWN, wN.z * invWN };
	b3Vec3 farWorld = { wF.x * invWF, wF.y * invWF, wF.z * invWF };

	// viewInv folds in S^-1, so the unprojected points are in simulation space
	// relative to the eye. Lift the near point with the simulation eye so the ray
	// feeds straight into the world queries, which run in length units.
	ray.origin = b3OffsetPos( DrawOrigin(), nearWorld );
	ray.translation = b3Sub( farWorld, nearWorld );
	return ray;
}

void Camera::OnEvent( const sapp_event* e )
{
	uint32_t mods = e->modifiers & ( SAPP_MODIFIER_SHIFT | SAPP_MODIFIER_CTRL | SAPP_MODIFIER_ALT );
	bool canOrbit = mods == SAPP_MODIFIER_ALT;

	switch ( e->type )
	{
		case SAPP_EVENTTYPE_MOUSE_DOWN:
			if ( e->mouse_button == SAPP_MOUSEBUTTON_LEFT )
				m_leftDown = true;
			if ( e->mouse_button == SAPP_MOUSEBUTTON_RIGHT )
				m_rightDown = true;
			if ( e->mouse_button == SAPP_MOUSEBUTTON_MIDDLE )
				m_middleDown = true;
			break;
		case SAPP_EVENTTYPE_MOUSE_UP:
			if ( e->mouse_button == SAPP_MOUSEBUTTON_LEFT )
				m_leftDown = false;
			if ( e->mouse_button == SAPP_MOUSEBUTTON_RIGHT )
				m_rightDown = false;
			if ( e->mouse_button == SAPP_MOUSEBUTTON_MIDDLE )
				m_middleDown = false;
			break;
		case SAPP_EVENTTYPE_MOUSE_MOVE:
			if ( canOrbit && m_leftDown )
			{
				m_orbitDX += e->mouse_dx;
				m_orbitDY += e->mouse_dy;
			}
			else if ( canOrbit && m_middleDown )
			{
				m_panDX += e->mouse_dx;
				m_panDY += e->mouse_dy;
			}
			else if ( canOrbit && m_rightDown )
			{
				m_radialZoomDY += e->mouse_dy;
			}
			else if ( !canOrbit && m_rightDown )
			{
				// Fly-look: routed into the same orbit accumulators; Update
				// interprets them differently when m_rightDown && !m_altDown.
				m_orbitDX += e->mouse_dx;
				m_orbitDY += e->mouse_dy;
			}
			break;
		case SAPP_EVENTTYPE_MOUSE_SCROLL:
			if ( !canOrbit && m_rightDown )
				m_speedScrollAccum += e->scroll_y;
			else
				m_scrollAccum += e->scroll_y;
			break;
		case SAPP_EVENTTYPE_KEY_DOWN:
			switch ( e->key_code )
			{
				case SAPP_KEYCODE_W:
					m_wDown = true;
					break;
				case SAPP_KEYCODE_A:
					m_aDown = true;
					break;
				case SAPP_KEYCODE_S:
					m_sDown = true;
					break;
				case SAPP_KEYCODE_D:
					m_dDown = true;
					break;
				case SAPP_KEYCODE_LEFT_ALT:
				case SAPP_KEYCODE_RIGHT_ALT:
					m_altDown = true;
					break;
				default:
					break;
			}
			break;
		case SAPP_EVENTTYPE_KEY_UP:
			switch ( e->key_code )
			{
				case SAPP_KEYCODE_W:
					m_wDown = false;
					break;
				case SAPP_KEYCODE_A:
					m_aDown = false;
					break;
				case SAPP_KEYCODE_S:
					m_sDown = false;
					break;
				case SAPP_KEYCODE_D:
					m_dDown = false;
					break;
				case SAPP_KEYCODE_LEFT_ALT:
				case SAPP_KEYCODE_RIGHT_ALT:
					m_altDown = false;
					break;
				default:
					break;
			}
			break;
		case SAPP_EVENTTYPE_UNFOCUSED:
			// Drop every held input on focus loss. Otherwise a drag or a held WASD
			// key survives an alt-tab and the camera keeps moving.
			m_leftDown = false;
			m_rightDown = false;
			m_middleDown = false;
			m_wDown = false;
			m_aDown = false;
			m_sDown = false;
			m_dDown = false;
			m_altDown = false;
			break;
		default:
			break;
	}
}

void Camera::Update( float dt, int width, int height )
{
	m_width = width;
	m_height = height;
	const float aspect = ( height > 0 ) ? (float)width / (float)height : 1.0f;

	if ( m_thirdPerson )
	{
		// Follow cam: the sample drives m_pivot to the tracked body and calls
		// UpdateTransform. The only camera input is wheel zoom on the radius.
		if ( m_scrollAccum != 0.0f )
		{
			m_radius -= m_scrollAccum;
			if ( m_radius < THIRD_PERSON_MIN_RADIUS )
				m_radius = THIRD_PERSON_MIN_RADIUS;
		}
		// Drop the other accumulators so a return to orbit/fly starts clean.
		m_orbitDX = 0.0f;
		m_orbitDY = 0.0f;
		m_panDX = 0.0f;
		m_panDY = 0.0f;
		m_radialZoomDY = 0.0f;
		m_scrollAccum = 0.0f;
		m_speedScrollAccum = 0.0f;
		RebuildBasisAndView( *this );
		RebuildProj( aspect );
		return;
	}

	const bool flyMode = m_rightDown && !m_altDown;

	if ( flyMode )
	{
		// Snapshot eye BEFORE rotating so yaw/pitch pivot around the eye
		// (FPS) instead of around m_pivot (orbit). After rotation we
		// back-derive m_pivot from this preserved eye so the eye stays put
		// regardless of look angle.
		const b3Pos eyeBefore = b3OffsetPos( m_pivot, b3MulSV( m_radius, ForwardFromAngles( m_yaw, m_pitch ) ) );

		// FPS look: drag right -> yaw decreases (turn head right, scene
		// shifts left); drag down -> pitch increases (look down).
		if ( m_orbitDX != 0.0f || m_orbitDY != 0.0f )
		{
			m_yaw -= m_orbitDX * FLY_LOOK_SENS;
			m_pitch += m_orbitDY * FLY_LOOK_SENS;
			if ( m_pitch > PITCH_LIM )
				m_pitch = PITCH_LIM;
			if ( m_pitch < -PITCH_LIM )
				m_pitch = -PITCH_LIM;
		}

		const b3Vec3 forward = ForwardFromAngles( m_yaw, m_pitch );

		float wasdF = 0.0f; // +forward = backwards, since forward = pivot->eye
		float wasdR = 0.0f;
		if ( m_wDown )
			wasdF -= 1.0f;
		if ( m_sDown )
			wasdF += 1.0f;
		if ( m_dDown )
			wasdR += 1.0f;
		if ( m_aDown )
			wasdR -= 1.0f;

		b3Pos eye = eyeBefore;
		if ( wasdF != 0.0f || wasdR != 0.0f )
		{
			// Right = normalize(worldUp x forward), matching Box3D's
			// UpdateTransform. worldUp = (0,1,0).
			const b3Vec3 worldUp = { 0.0f, 1.0f, 0.0f };
			b3Vec3 right = b3Cross( worldUp, forward );
			const float rlen = sqrtf( right.x * right.x + right.y * right.y + right.z * right.z );
			if ( rlen > 1.0e-6f )
			{
				right.x /= rlen;
				right.y /= rlen;
				right.z /= rlen;
			}
			const float step = m_speed * dt;
			eye.x += forward.x * wasdF * step + right.x * wasdR * step;
			eye.y += forward.y * wasdF * step + right.y * wasdR * step;
			eye.z += forward.z * wasdF * step + right.z * wasdR * step;
		}

		// Back-derive the pivot so Position() stays consistent and a return
		// to orbit mode preserves the current look direction.
		m_pivot.x = eye.x - forward.x * m_radius;
		m_pivot.y = eye.y - forward.y * m_radius;
		m_pivot.z = eye.z - forward.z * m_radius;

		if ( m_speedScrollAccum != 0.0f )
		{
			m_speed += m_speedScrollAccum * FLY_SPEED_STEP;
			m_speed = b3ClampFloat( m_speed, MIN_SPEED, MAX_SPEED );
		}
	}
	else
	{
		// Orbit mode.
		if ( m_orbitDX != 0.0f || m_orbitDY != 0.0f )
		{
			m_yaw -= m_orbitDX * ORBIT_SENS;
			m_pitch -= m_orbitDY * ORBIT_SENS;
			m_pitch = b3ClampFloat( m_pitch, -PITCH_LIM, PITCH_LIM );
		}

		if ( m_panDX != 0.0f || m_panDY != 0.0f )
		{
			// Move pivot along view-space right/up. m_right / m_up are still
			// last frame's values (orbit yaw/pitch haven't been rebuilt yet),
			// which matches the screen the user dragged against.
			const float panScale = PAN_SENS * m_radius;
			m_pivot.x -= m_right.x * m_panDX * panScale;
			m_pivot.y -= m_right.y * m_panDX * panScale;
			m_pivot.z -= m_right.z * m_panDX * panScale;
			m_pivot.x += m_up.x * m_panDY * panScale;
			m_pivot.y += m_up.y * m_panDY * panScale;
			m_pivot.z += m_up.z * m_panDY * panScale;
		}

		if ( m_radialZoomDY != 0.0f )
		{
			// Drag down -> zoom in (radius shrinks). Matches Box3D.
			m_radius -= RADIAL_ZOOM_SENS * m_radialZoomDY;
			m_radius = b3ClampFloat( m_radius, MIN_DIST, kViewDistance );
		}

		if ( m_scrollAccum != 0.0f )
		{
			m_radius *= powf( ZOOM_STEP, m_scrollAccum );
			m_radius = b3ClampFloat( m_radius, MIN_DIST, kViewDistance );
		}
	}

	// Keep yaw bounded across long sessions. b3UnwindAngle wraps to (-pi, pi].
	m_yaw = b3UnwindAngle( m_yaw );

	m_orbitDX = 0.0f;
	m_orbitDY = 0.0f;
	m_panDX = 0.0f;
	m_panDY = 0.0f;
	m_radialZoomDY = 0.0f;
	m_scrollAccum = 0.0f;
	m_speedScrollAccum = 0.0f;

	RebuildBasisAndView( *this );
	RebuildProj( aspect );
}
