// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "camera.h"

#include <glad/glad.h>
// prevent clang format sorting glad.h with glfw3.h
#include <GLFW/glfw3.h>

// ImGui
#include <imgui.h>
#include <imgui_internal.h>

Matrix4 MakeOrthographicMatrix( float left, float right, float bottom, float top, float near, float far)
{
	Matrix4 m;
	m.cx = Vector4{ 2.0f / (right - left), 0.0f, 0.0f, 0.0f };
	m.cy = Vector4{ 0.0f, 2.0f / (top - bottom), 0.0f, 0.0f };
	m.cz = Vector4{ 0.0f, 0.0f, -2.0f / (far - near), 0.0f };
	m.cw.x = -(right + left) / (right - left);
	m.cw.y = -(top + bottom) / (top - bottom);
	m.cw.z = -(far + near) / (far - near);
	m.cw.w = 1.0f;
	return m;
}

Camera::Camera()
{
	m_yaw = 0.0f;
	m_pitch = 0.0f;
	m_radius = 1.0f;
	m_speed = 20.0f;
	m_pivot = b3Vec3_zero;
	m_nearPlane = 0.02f;
	m_farPlane = 100000.0f;
	m_fovY = 50.0f;
	m_tanY = tanf( 0.5f * B3_DEG_TO_RAD * m_fovY );
	m_thirdPerson = false;

	m_bufferWidth = 1920;
	m_bufferHeight = 1080;

	Resize(1920, 1080);
	UpdateTransform();
}

void Camera::Resize( int width, int height )
{
	m_width = b3MaxInt(width, 1 );
	m_height = b3MaxInt(height, 1 );
	m_ratio = float( m_width ) / float( m_height );

	m_nearHeight = 2.0f * m_nearPlane * m_tanY;
	m_nearWidth = m_ratio * m_nearHeight;

	m_farHeight = m_farPlane * ( m_nearHeight / m_nearPlane );
	m_farWidth = m_farPlane * ( m_nearWidth / m_nearPlane );
}

b3Vec3 Camera::GetRight() const
{
	return m_right;
}

b3Vec3 Camera::GetUp() const
{
	return m_up;
}

b3Vec3 Camera::GetForward() const
{
	return m_forward;
}

b3Vec3 Camera::GetPosition() const
{
	return m_position;
}

void Camera::Frame( b3AABB bounds)
{
	b3Vec3 center = b3AABB_Center(bounds);
	b3Vec3 extent = b3AABB_Extents(bounds);

	float height = 2.0f * b3MaxFloat(extent.x, b3MaxFloat(extent.y, extent.z ) );
	float radius = 0.5f * height / tanf( 0.5f * B3_DEG_TO_RAD * m_fovY );

	m_pivot = center;
	m_radius = 1.25f * radius;
}

void Camera::SetView( float yaw, float pitch, float radius, b3Vec3 pivot)
{
	m_yaw = yaw;
	m_pitch = pitch;
	m_radius = radius;
	m_pivot = pivot;
	UpdateTransform();
}

Matrix4 Camera::GetWorldMatrix() const
{
	Matrix4 m;
	m.cx = { m_right.x, m_right.y, m_right.z, 0.0f };
	m.cy = { m_up.x, m_up.y, m_up.z, 0.0f };
	m.cz = { m_forward.x, m_forward.y, m_forward.z, 0.0f };
	m.cw = { m_position.x, m_position.y, m_position.z, 1.0f };
	return m;
}

static Matrix4 InvertOrthonormal( const Matrix4& m)
{
	//     [ R | t ]
	// M = [ --+-- ]
	//     [ 0 | 1 ]
	//
	//  y = M * x  ->  y = R * x + t  ->  x = R^-1 * (y - t)  ->  x = R^-1 * y - R^-1 * t
	//
	//         [ R | t ]-1   [ R^-1 | -R^-1 * t ]
	//  M^-1 = [ --+-- ]   = [ -----+---------- ]
	//         [ 0 | 1 ]     [  0   |     1     ]
	b3Matrix3 r;
	r.cx = {m.cx.x, m.cx.y, m.cx.z };
	r.cy = {m.cy.x, m.cy.y, m.cy.z };
	r.cz = {m.cz.x, m.cz.y, m.cz.z };

	// todo just transpose in place into out matrix
	b3Matrix3 invR = b3Transpose(r);

	b3Vec3 t{m.cw.x, m.cw.y, m.cw.z };
	b3Vec3 invRXT = invR * t;

	Matrix4 out;
	out.cx = {invR.cx.x, invR.cx.y, invR.cx.z, 0.0f };
	out.cy = {invR.cy.x, invR.cy.y, invR.cy.z, 0.0f };
	out.cz = {invR.cz.x, invR.cz.y, invR.cz.z, 0.0f };
	out.cw = { -invRXT.x, -invRXT.y, -invRXT.z, 1.0f };

	return out;
}

Matrix4 Camera::GetViewMatrix() const
{
	Matrix4 v = GetWorldMatrix();
	v = InvertOrthonormal(v);
	return v;
}

// http://www.songho.ca/opengl/gl_projectionmatrix.html
// https://unspecified.wordpress.com/2012/06/21/calculating-the-gluperspective-matrix-and-other-opengl-matrix-maths/
// https://software.intel.com/en-us/articles/alternatives-to-using-z-bias-to-fix-z-fighting-issues
static Matrix4 Perspective( float fieldOfViewY, float aspectRatio, float nearPlane, float farPlane)
{
	float cot = 1.0f / tanf( 0.5f * fieldOfViewY);
	float depth = farPlane - nearPlane;

	Matrix4 out;
	out.cx = {cot / aspectRatio, 0.0f, 0.0f, 0.0f };
	out.cy = { 0.0f, cot, 0.0f, 0.0f };
	out.cz = { 0.0f, 0.0f, -(nearPlane + farPlane) / depth, -1.0f };
	out.cw = { 0.0f, 0.0f, -2.0f * nearPlane * farPlane / depth, 0.0f };
	return out;
}

Matrix4 Camera::GetProjectionMatrix() const
{
	return Perspective( B3_DEG_TO_RAD * m_fovY, m_ratio, m_nearPlane, m_farPlane );
}

PickRay Camera::BuildPickRay( float x, float y) const
{
	// Use similar triangles to build pick ray

	// Scale, flip, shift into frustum coordinates
	float xRatio = x / m_width - 0.5f;
	float yRatio = 0.5f - y / m_height;

	// Near plane coordinates using ratio
	float xNear = xRatio * m_nearWidth;
	float yNear = yRatio * m_nearHeight;

	// Far plane coordinates using ratio
	float xFar = xRatio * m_farWidth;
	float yFar = yRatio * m_farHeight;

	// Coordinates of near and far plane points in camera space
	Vector4 nearPoint{xNear, yNear, -m_nearPlane, 1.0f };
	Vector4 farPoint{xFar, yFar, -m_farPlane, 1.0f };

	// Convert to world space
	Matrix4 m = GetWorldMatrix();
	nearPoint = MulMV(m, nearPoint);
	farPoint = MulMV(m, farPoint);

	PickRay ray;
	ray.origin = {nearPoint.x, nearPoint.y, nearPoint.z };
	ray.translation = b3Vec3{farPoint.x, farPoint.y, farPoint.z } - ray.origin;
	return ray;
}

bool Camera::WorldToScreen( float& xScreen, float& yScreen, b3Vec3 worldPoint) const
{
	Matrix4 view = GetViewMatrix();
	Vector4 cameraPoint = MulMV(view, Vector4{ worldPoint.x, worldPoint.y, worldPoint.z, 1.0f });

	float x = cameraPoint.x;
	float y = cameraPoint.y;
	float z = cameraPoint.z;

	// Frustum comes out the back of the camera
	if (z > -m_nearPlane || z < -m_farPlane )
	{
		return false;
	}

	// Find the frustum size at depth Z
	float heightAtZ = -2.0f * z * m_tanY;
	float widthAtZ = m_ratio * heightAtZ;

	// Compute ratios of the point within the frustum rectangle at Z
	float xRatio = x / widthAtZ;
	float yRatio = y / heightAtZ;

	// Shift, flip, and scale
	xScreen = (xRatio + 0.5f ) * m_width;
	yScreen = ( 0.5f - yRatio) * m_height;

	return true;
}

void Camera::Update( GLFWwindow* window, float elapsedTime)
{
	ImGuiIO& io = ImGui::GetIO();

	if ( m_thirdPerson )
	{
		const float sensitivity = 1.0f;
		m_radius -= sensitivity * io.MouseWheel;
		m_radius = b3MaxFloat( 0.25f, m_radius );
		return;
	}

	if (io.KeyAlt )
	{
		// Orbit
		if ( ImGui::IsMouseDragging( ImGuiMouseButton_Left ) )
		{
			const float sensitivity = 0.2f;
			m_yaw -= sensitivity * io.MouseDelta.x;
			m_pitch += sensitivity * io.MouseDelta.y;
			m_pitch = b3ClampFloat( m_pitch, -85.0f, 85.0f );
		}

		if ( ImGui::IsMouseDragging( ImGuiMouseButton_Middle ) )
		{
			const float sensitivity = 0.01f;
			m_pivot -= sensitivity * io.MouseDelta.x * m_right;
			m_pivot += sensitivity * io.MouseDelta.y * m_up;
		}

		if ( ImGui::IsMouseDragging( ImGuiMouseButton_Right ) )
		{
			const float sensitivity = 0.02f;
			m_radius -= sensitivity * io.MouseDelta.y;
			m_radius = b3MaxFloat( 0.001f, m_radius );
		}

		const float sensitivity = 1.0f;
		m_radius -= sensitivity * io.MouseWheel;
		m_radius = b3MaxFloat( 0.001f, m_radius );

		UpdateTransform();
	}
	else if ( ImGui::IsMouseDown( ImGuiMouseButton_Right ) )
	{
		m_speed += io.MouseWheel;
		m_speed = b3ClampFloat( m_speed, 0.001f * 60.0f, 500.0f * 60.0f );

		if ( ImGui::IsMouseDragging( ImGuiMouseButton_Right ) )
		{
			const float sensitivity = 0.2f;
			m_yaw -= sensitivity * io.MouseDelta.x;
			m_pitch += sensitivity * io.MouseDelta.y;
			m_pitch = b3ClampFloat( m_pitch, -85.0f, 85.0f );

			float sinYaw = sinf( B3_DEG_TO_RAD * m_yaw ), cosYaw = cosf( B3_DEG_TO_RAD * m_yaw );
			float sinPitch = sinf( B3_DEG_TO_RAD * m_pitch ), cosPitch = cosf( B3_DEG_TO_RAD * m_pitch );

			m_forward = {sinYaw * cosPitch, sinPitch, cosYaw * cosPitch};
			m_up = { 0.0f, 1.0f, 0.0f };
			m_up = b3Normalize( m_up - b3Dot( m_forward, m_up ) * m_forward );
			m_right = b3Normalize( b3Cross( m_up, m_forward ) );
		}

		// Smoother with GLFW
		if ( glfwGetKey(window, GLFW_KEY_W ) )
		{
			m_position -= m_speed * elapsedTime * m_forward;
		}

		if ( glfwGetKey(window, GLFW_KEY_S ) )
		{
			m_position += m_speed * elapsedTime * m_forward;
		}

		if ( glfwGetKey(window, GLFW_KEY_A ) )
		{
			m_position -= m_speed * elapsedTime * m_right;
		}

		if ( glfwGetKey(window, GLFW_KEY_D ) )
		{
			m_position += m_speed * elapsedTime * m_right;
		}

		m_pivot = m_position - m_radius * m_forward;
	}
}

void Camera::UpdateTransform()
{
	m_yaw = B3_RAD_TO_DEG * b3UnwindAngle( B3_DEG_TO_RAD * m_yaw );
	float sinYaw = sinf( B3_DEG_TO_RAD * m_yaw ), cosYaw = cosf( B3_DEG_TO_RAD * m_yaw );
	float sinPitch = sinf( B3_DEG_TO_RAD * m_pitch ), cosPitch = cosf( B3_DEG_TO_RAD * m_pitch );

	m_forward = {sinYaw * cosPitch, sinPitch, cosYaw * cosPitch};
	m_up = { 0.0f, 1.0f, 0.0f };
	m_up = b3Normalize( m_up - b3Dot( m_forward, m_up ) * m_forward );
	m_right = b3Normalize( b3Cross( m_up, m_forward ) );

	m_position = m_pivot + m_radius * m_forward;
}
