// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"
#include "gfx/draw.h"
#include "utils.h"

#include "box3d/box3d.h"

#include <imgui.h>

class BoxHull : public Sample
{
public:
	explicit BoxHull( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 5.0f, b3Pos_zero );
		}

		m_hull = nullptr;

		m_halfWidths = { 1.0f, 0.5f, 0.25f };
		m_postScale = { 1.0f, 1.0f, 1.0f };
		m_rotation = { 0.0f, 0.0f, 0.0f };
		m_transform = b3Transform_identity;

		UpdateRotation();
		CreateHulls( m_halfWidths, m_transform, m_postScale );
	}

	~BoxHull() override
	{
		b3DestroyHull( m_hull );
	}

	void UpdateRotation()
	{
		b3Quat qx = b3MakeQuatFromAxisAngle( b3Vec3_axisX, B3_DEG_TO_RAD * m_rotation.x );
		b3Quat qy = b3MakeQuatFromAxisAngle( b3Vec3_axisY, B3_DEG_TO_RAD * m_rotation.y );
		b3Quat qz = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, B3_DEG_TO_RAD * m_rotation.z );

		m_transform.q = b3MulQuat( qz, b3MulQuat( qy, qx ) );
	}

	void CreateHulls( b3Vec3 h, b3Transform transform, b3Vec3 postScale )
	{
		if ( m_hull != nullptr )
		{
			b3DestroyHull( m_hull );
			m_hull = nullptr;
		}

		b3Vec3 scale = b3SafeScale( postScale );
		b3Vec3 points[8];

		points[0] = scale * b3TransformPoint( transform, { h.x, h.y, h.z } );
		points[1] = scale * b3TransformPoint( transform, { h.x, h.y, -h.z } );
		points[2] = scale * b3TransformPoint( transform, { h.x, -h.y, h.z } );
		points[3] = scale * b3TransformPoint( transform, { h.x, -h.y, -h.z } );
		points[4] = scale * b3TransformPoint( transform, { -h.x, h.y, h.z } );
		points[5] = scale * b3TransformPoint( transform, { -h.x, h.y, -h.z } );
		points[6] = scale * b3TransformPoint( transform, { -h.x, -h.y, h.z } );
		points[7] = scale * b3TransformPoint( transform, { -h.x, -h.y, -h.z } );

		m_hull = b3CreateHull( points, 8, 8 );
		m_box = b3MakeScaledBoxHull( h, transform, postScale );
	}

	bool HasSolverControls() const override
	{
		return false;
	}

	bool DrawControls() override
	{
		float fontSize = ImGui::GetFontSize();
		ImGui::PushItemWidth( 10.0f * fontSize );

		if ( ImGui::SliderFloat3( "h", &m_halfWidths.x, 0.1f, 2.0f, "%.1f" ) )
		{
			CreateHulls( m_halfWidths, m_transform, m_postScale );
		}

		if ( ImGui::SliderFloat3( "c", &m_transform.p.x, -2.0f, 2.0f, "%.1f" ) )
		{
			CreateHulls( m_halfWidths, m_transform, m_postScale );
		}

		if ( ImGui::SliderFloat3( "r", &m_rotation.x, -180.0f, 180.0f, "%.0f" ) )
		{
			UpdateRotation();
			CreateHulls( m_halfWidths, m_transform, m_postScale );
		}

		if ( ImGui::SliderFloat3( "s", &m_postScale.x, -2.0f, 2.0f, "%.1f" ) )
		{
			CreateHulls( m_halfWidths, m_transform, m_postScale );
		}

		if ( ImGui::Button( "Refresh" ) )
		{
			CreateHulls( m_halfWidths, m_transform, m_postScale );
		}

		ImGui::PopItemWidth();

		return true;
	}

	void Render() override
	{
		DrawHull( b3WorldTransform_identity, m_hull, MakeColor( b3_colorYellow ) );
		DrawHull( b3WorldTransform_identity, &m_box.base, MakeColor( b3_colorCyan ) );

		DrawAxes( b3WorldTransform_identity, 1.0f );

		Sample::Render();
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new BoxHull( sampleContext );
	}

	b3Vec3 m_halfWidths;
	b3Vec3 m_rotation;
	b3Transform m_transform;
	b3Vec3 m_postScale;
	b3BoxHull m_box;
	b3HullData* m_hull;
};

static int sampleBoxHull = RegisterSample( "Geometry", "Box Hull", BoxHull::Create );

class Hull : public Sample
{
public:
	explicit Hull( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 5.0f, b3Pos_zero );
		}

		m_hull = nullptr;

		// this fails because it generates too many edges
		b3Vec3 points[] = {
			{ -3.9866004, 75.4595108, 28.3783073 },	  { -13.1079493, 73.080368, 28.296587 },
			{ -18.6611958, 72.0040894, 16.9292431 },  { 4.82537603, 79.2908554, 22.2369995 },
			{ -12.7315464, 79.2187576, 2.94275379 },  { -21.806488, 78.7758865, 0.985544085 },
			{ -27.7619209, 73.3481522, 11.9647141 },  { -22.3994541, 72.2203826, 21.4116211 },
			{ -25.3797474, 76.7417755, 27.9124985 },  { -22.7552319, 77.0559006, 29.4733639 },
			{ -6.81736374, 78.3484726, 36.8649979 },  { 3.62397718, 85.5270843, 29.2077713 },
			{ 7.90363788, 84.121231, 18.2612896 },	  { -12.3809223, 84.5280533, -0.43230924 },
			{ 5.83599472, 95.2908325, 4.4423275 },	  { -22.5541401, 89.9094467, -4.87791252 },
			{ -43.9060402, 78.5287094, 1.32877088 },  { -42.6015129, 76.7829742, 7.67437983 },
			{ -25.735527, 78.1218796, 27.908411 },	  { -23.5183544, 77.6326675, 29.1178799 },
			{ 2.0977366, 100.430191, 34.3929482 },	  { 1.09743047, 103.952553, 35.5656395 },
			{ 8.50175952, 96.0529861, 8.73674774 },	  { 2.52570295, 103.303696, 32.2314339 },
			{ -20.099781, 89.4923248, -4.15468454 },  { 2.8092947, 123.516098, -1.12693477 },
			{ -43.9318161, 79.1106186, 1.39006138 },  { -23.358511, 90.9599686, -4.25683546 },
			{ 2.10804915, 123.603645, -1.38435471 },  { -44.1329117, 78.7192383, 1.54941654 },
			{ -42.4365158, 77.725357, 8.14835929 },	  { -43.204792, 77.5811691, 7.14319515 },
			{ -44.17416, 78.7810363, 2.50146222 },	  { -32.8975143, 99.1221771, 7.55588436 },
			{ -0.624746263, 110.070351, 32.7381058 }, { 0.00431228895, 109.14341, 33.6411133 },
			{ -0.58865279, 122.980537, 16.6554794 },  { 2.18539238, 124.324593, -0.620266676 },
			{ -1.02177501, 123.881721, 16.8230057 },  { 1.9842999, 124.571777, -0.321986318 },
			{ 1.86570692, 124.365791, -0.599836588 }, { -43.591507, 78.1373291, 6.1135149 },
			{ -43.8235397, 79.2239074, 3.48619604 },  { -43.591507, 78.50811, 5.54555655 },
			{ 1.21086729, 124.49453, 1.07543683 },	  { -1.86223853, 124.195847, 15.6257992 },
			{ -1.46520972, 124.355492, 16.9864483 },  { 1.654302, 124.612976, 0.621887207 },
		};

		static_assert( sizeof( points ) / sizeof( points[0] ) < m_capacity, "bad" );

		m_count = sizeof( points ) / sizeof( points[0] );
		for ( int i = 0; i < m_count; ++i )
		{
			m_points[i] = 0.01f * points[i];
		}

		// This shift shouldn't be necessary but I'm doing it so the hull
		// appears on the screen.
		// for ( int i = 0; i < m_count; ++i )
		//{
		//	m_points[i] -= m_points[0];
		//	m_points[i] *= 0.01f;
		//}

		m_hull = b3CreateHull( m_points, m_count, 16 );

		(void)m_hull;
	}

	~Hull() override
	{
		if ( m_hull != nullptr )
		{
			b3DestroyHull( m_hull );
		}
	}

	void Render() override
	{
		if ( m_hull != nullptr )
		{
			DrawHull( b3WorldTransform_identity, m_hull, MakeColor( b3_colorYellow ) );
		}

		DrawAxes( b3WorldTransform_identity, 1.0f );

		Sample::Render();
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new Hull( sampleContext );
	}

	static constexpr int m_capacity = 64;
	b3HullData* m_hull;
	b3Vec3 m_points[m_capacity];
	int m_count;
};

static int sampleHull = RegisterSample( "Geometry", "Hull", Hull::Create );

class HullReduction : public Sample
{
public:
	enum Type
	{
		e_box,
		e_sphere,
	};

	explicit HullReduction( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 5.0f, b3Pos_zero );
		}

		m_hull = nullptr;

		m_type = e_sphere;
		m_count = 16;
		GeneratePoints();
		GenerateHull();
	}

	void GeneratePoints()
	{
		g_randomSeed = 42;

		if ( m_type == e_box )
		{
			b3Vec3 lower = { -2.0f, -2.0f, -2.0f };
			b3Vec3 upper = { 2.0f, 2.0f, 2.0f };
			b3AABB box = { { -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f } };
			float a = 0.001f;
			b3AABB noise = { { -a, -a, -a }, { a, a, a } };

			for ( int i = 0; i < m_capacity; ++i )
			{
				b3Vec3 p = RandomVec3( lower, upper );
				p = b3Clamp( p, box.lowerBound, box.upperBound );
				b3Vec3 f = RandomVec3( noise.lowerBound, noise.upperBound );
				m_points[i] = b3Add(p, f);
			}
		}
		else
		{
			for ( int i = 0; i < m_capacity; ++i )
			{
				m_points[i] = RandomUnitVector();
			}
		}
	}

	void GenerateHull()
	{
		if ( m_hull != nullptr )
		{
			b3DestroyHull( m_hull );
		}

		m_hull = b3CreateHull( m_points, m_capacity, m_count );
	}

	~HullReduction() override
	{
		if ( m_hull != nullptr )
		{
			b3DestroyHull( m_hull );
		}
	}

	bool HasSolverControls() const override
	{
		return false;
	}

	bool DrawControls() override
	{
		if ( ImGui::RadioButton( "Box", m_type == e_box ) )
		{
			m_type = e_box;
			GeneratePoints();
			GenerateHull();
		}

		if ( ImGui::RadioButton( "Sphere", m_type == e_sphere ) )
		{
			m_type = e_sphere;
			GeneratePoints();
			GenerateHull();
		}

		if ( ImGui::SliderInt( "count", &m_count, 4, m_capacity ) )
		{
			GenerateHull();
		}

		return true;
	}

	void Render() override
	{
		if ( m_hull != nullptr )
		{
			DrawHull( b3WorldTransform_identity, m_hull, MakeColor( b3_colorYellow ) );

			DrawTextLine( "v/f/e = %d/%d/%d", m_hull->vertexCount, m_hull->faceCount, m_hull->edgeCount / 2 );
		}

		DrawAxes( b3WorldTransform_identity, 1.0f );

		Sample::Render();
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new HullReduction( sampleContext );
	}

	static constexpr int m_capacity = 128;
	Type m_type;
	b3HullData* m_hull;
	b3Vec3 m_points[m_capacity];
	int m_count;
};

static int sampleHullReduction = RegisterSample( "Geometry", "Hull Reduction", HullReduction::Create );

class HullTransform : public Sample
{
public:
	explicit HullTransform( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 5.0f, b3Pos_zero );
		}

		m_original = b3CreateCylinder( 1.0f, 0.5f, 0.0f, 9 );
		m_box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		// m_original = &m_box.base;
		m_scale = { 1.0f, 1.0f, 1.0f };
		m_angles = b3Vec3_zero;
		m_offset = b3Vec3_zero;
		m_hull = b3CloneAndTransformHull( m_original, b3Transform_identity, m_scale );
	}

	~HullTransform() override
	{
		if ( m_original != &m_box.base )
		{
			b3DestroyHull( m_original );
		}

		b3DestroyHull( m_hull );
	}

	void UpdateHull()
	{
		b3DestroyHull( m_hull );

		b3Quat qx = b3MakeQuatFromAxisAngle( b3Vec3_axisX, m_angles.x * B3_PI / 180.0f );
		b3Quat qy = b3MakeQuatFromAxisAngle( b3Vec3_axisY, m_angles.y * B3_PI / 180.0f );
		b3Quat qz = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, m_angles.z * B3_PI / 180.0f );
		b3Quat q = b3MulQuat( qz, b3MulQuat( qy, qx ) );

		m_hull = b3CloneAndTransformHull( m_original, { m_offset, q }, m_scale );
	}

	void Render() override
	{
		b3Transform transform1 = { { -2.0f, 0.0f, 0.0f }, b3Quat_identity };
		b3Transform transform2 = { { 2.0f, 0.0f, 0.0f }, b3Quat_identity };

		DrawHull( b3MakeWorldTransform( transform1 ), m_original, MakeColor( b3_colorGreen ) );
		DrawHull( b3MakeWorldTransform( transform2 ), m_hull, MakeColor( b3_colorYellow ) );
		DrawAxes( b3WorldTransform_identity, 1.0f );

		DrawTextLine( "hull 1: area = %g, volume = %g, radius = %g", m_original->surfaceArea, m_original->volume,
					  m_original->innerRadius );
		DrawTextLine( "hull 2: area = %g, volume = %g, radius = %g", m_hull->surfaceArea, m_hull->volume, m_hull->innerRadius );

		Sample::Render();
	}

	bool HasSolverControls() const override
	{
		return false;
	}

	bool DrawControls() override
	{
		if ( ImGui::SliderFloat( "sx", &m_scale.x, -2.0f, 2.0f, "%.1f" ) )
		{
			UpdateHull();
		}

		if ( ImGui::SliderFloat( "sy", &m_scale.y, -2.0f, 2.0f, "%.1f" ) )
		{
			UpdateHull();
		}

		if ( ImGui::SliderFloat( "sz", &m_scale.z, -2.0f, 2.0f, "%.1f" ) )
		{
			UpdateHull();
		}

		if ( ImGui::SliderFloat( "rx", &m_angles.x, -180.0f, 180.0f, "%.0f" ) )
		{
			UpdateHull();
		}

		if ( ImGui::SliderFloat( "ry", &m_angles.y, -180.0f, 180.0f, "%.0f" ) )
		{
			UpdateHull();
		}

		if ( ImGui::SliderFloat( "rz", &m_angles.z, -180.0f, 180.0f, "%.0f" ) )
		{
			UpdateHull();
		}

		if ( ImGui::SliderFloat( "px", &m_offset.x, -1.0f, 1.0f, "%.1f" ) )
		{
			UpdateHull();
		}

		if ( ImGui::SliderFloat( "py", &m_offset.y, -1.0f, 1.0f, "%.1f" ) )
		{
			UpdateHull();
		}

		if ( ImGui::SliderFloat( "pz", &m_offset.z, -1.0f, 1.0f, "%.1f" ) )
		{
			UpdateHull();
		}

		return true;
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new HullTransform( sampleContext );
	}

	b3BoxHull m_box;
	b3HullData* m_original;
	b3HullData* m_hull;
	b3Vec3 m_angles;
	b3Vec3 m_scale;
	b3Vec3 m_offset;
};

static int sampleHullTransform = RegisterSample( "Geometry", "Hull Transform", HullTransform::Create );

class CapsuleMass : public Sample
{
public:
	explicit CapsuleMass( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 5.0f, b3Pos_zero );
		}

		m_radius = 1.0f;
		m_length = 2.0f;
		m_sides = 6;
		m_capsule = { { -0.5f * m_length, 0.0f, 0.0f }, { 0.5f * m_length, 0.0f, 0.0f }, m_radius };
		m_box = b3MakeBoxHull( m_radius + 0.5f * m_length, m_radius, m_radius );

		m_hull = nullptr;
		CreateCapsuleHull( m_sides );
	}

	~CapsuleMass() override
	{
		if ( m_hull != nullptr )
		{
			b3DestroyHull( m_hull );
		}
	}

	void CreateCapsuleHull( int sides )
	{
		if ( m_hull != nullptr )
		{
			b3DestroyHull( m_hull );
			m_hull = nullptr;
		}

		constexpr int N = m_maxSides;
		constexpr int M = 2 * N * N;
		b3Vec3 points[M];

		if ( sides > N )
		{
			return;
		}

		int count = 2 * sides * sides;
		float d = B3_PI / ( sides - 1.0f );
		float angle1 = -0.5f * B3_PI;
		int index = 0;
		for ( int i = 0; i < sides; ++i )
		{
			float s1 = sinf( angle1 );
			float c1 = cosf( angle1 );
			float angle2 = -0.5f * B3_PI;
			for ( int j = 0; j < sides; ++j )
			{
				points[index].x = 1.0f + m_radius * c1;
				points[index].y = m_radius * s1 * cosf( angle2 );
				points[index].z = m_radius * s1 * sinf( angle2 );
				angle2 += d;
				index += 1;
			}

			angle1 += d;
		}

		angle1 = 0.5f * B3_PI;
		for ( int i = 0; i < sides; ++i )
		{
			float s1 = sinf( angle1 );
			float c1 = cosf( angle1 );
			float angle2 = -0.5f * B3_PI;
			for ( int j = 0; j < sides; ++j )
			{
				points[index].x = -1.0f + m_radius * c1;
				points[index].y = m_radius * s1 * cosf( angle2 );
				points[index].z = m_radius * s1 * sinf( angle2 );
				angle2 += d;
				index += 1;
			}

			angle1 += d;
		}

		assert( index == count );

		m_hull = b3CreateHull( points, count, count );
	}

	void Render() override
	{
		DrawSolidCapsule( b3WorldTransform_identity, m_capsule, MakeColorAlpha( b3_colorAqua, 0.8f ) );
		DrawHull( b3WorldTransform_identity, &m_box.base, MakeColor( b3_colorBlueViolet ) );

		if ( m_hull != nullptr )
		{
			DrawHull( b3WorldTransform_identity, m_hull, MakeColor( b3_colorYellow ) );
		}

		DrawAxes( b3WorldTransform_identity, 1.0f );

		if ( m_hull != nullptr )
		{
			b3MassData lowerMassData = b3ComputeHullMass( m_hull, 1.0f );
			b3MassData massData = b3ComputeCapsuleMass( &m_capsule, 1.0f );
			b3MassData upperMassData = b3ComputeHullMass( &m_box.base, 1.0f );

			DrawTextLine( "mass hull:    %g", lowerMassData.mass );
			DrawTextLine( "mass capsule: %g", massData.mass );
			DrawTextLine( "mass box:     %g", upperMassData.mass );
			m_textLine += m_textIncrement;
			DrawTextLine( "Ixx hull:    %g", lowerMassData.inertia.cx.x );
			DrawTextLine( "Ixx capsule: %g", massData.inertia.cx.x );
			DrawTextLine( "Ixx box:     %g", upperMassData.inertia.cx.x );
			m_textLine += m_textIncrement;
			DrawTextLine( "Iyy hull:    %g", lowerMassData.inertia.cy.y );
			DrawTextLine( "Iyy capsule: %g", massData.inertia.cy.y );
			DrawTextLine( "Iyy box:     %g", upperMassData.inertia.cy.y );
			m_textLine += m_textIncrement;
			DrawTextLine( "Izz hull:    %g", lowerMassData.inertia.cz.z );
			DrawTextLine( "Izz capsule: %g", massData.inertia.cz.z );
			DrawTextLine( "Izz box:     %g", upperMassData.inertia.cz.z );
		}

		Sample::Render();
	}

	bool HasSolverControls() const override
	{
		return false;
	}

	bool DrawControls() override
	{
		if ( ImGui::SliderInt( "sides", &m_sides, 3, m_maxSides ) )
		{
			CreateCapsuleHull( m_sides );
		}

		return true;
	}

	static Sample* Create( SampleContext* sampleContext )
	{
		return new CapsuleMass( sampleContext );
	}

	// Limited by edge count
	static constexpr int m_maxSides = 6;
	b3Capsule m_capsule;
	b3BoxHull m_box;
	b3HullData* m_hull;
	float m_radius;
	float m_length;
	int m_sides;
};

static int sampleCapsuleMass = RegisterSample( "Geometry", "Capsule Mass", CapsuleMass::Create );
