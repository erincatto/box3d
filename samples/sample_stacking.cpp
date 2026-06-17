// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/debug_adapter.h"
#include "gfx/keycodes.h"
#include "imgui.h"
#include "mesh_loader.h"
#include "sample.h"
#include "gfx/draw.h"

#include "box3d/box3d.h"

class CardHouseThick : public Sample
{
public:
	explicit CardHouseThick( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 25.0f, 10.0f, { 0.0f, 2.0f, 0.0f } );
		}

		AddGroundBox( 10.0f );

		const float alpha = 25.0f * B3_DEG_TO_RAD;
		const float width = 0.38f;
		const float height = 0.98f;
		const float depth = 0.08f;

		float offsetX = 0.5f * height * b3Sin( alpha ) + 0.045f;
		float offsetY = 0.5f * height * b3Cos( alpha ) + 0.035f;

		b3BoxHull box = b3MakeBoxHull( 0.5f * depth, 0.5f * height, 0.5f * width );
		AddVerticalRow( 4, -6.0f * offsetX, offsetX, offsetY, alpha, box );
		AddHorizontalRow( 3, -4.0f * offsetX, 4.0f * offsetX, 2.0f * offsetY + 0.04f, box );
		AddVerticalRow( 3, -4.0f * offsetX, offsetX, 3.0f * offsetY + 0.08f, alpha, box );
		AddHorizontalRow( 2, -2.0f * offsetX, 4.0f * offsetX, 4.0f * offsetY + 0.12f, box );
		AddVerticalRow( 2, -2.0f * offsetX, offsetX, 5.0f * offsetY + 0.16f, alpha, box );
		AddHorizontalRow( 1, -0.0f * offsetX, 4.0f * offsetX, 6.0f * offsetY + 0.20f, box );
		AddVerticalRow( 1, -0.0f * offsetX, offsetX, 7.0f * offsetY + 0.24f, alpha, box );
	}

	void AddVerticalRow( int n, float startX, float offsetX, float startY, float alpha, const b3BoxHull& box )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.8f;

		for ( int index = 0; index < n; ++index )
		{
			bodyDef.position = { startX - offsetX, startY, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, -alpha );
			b3BodyId body1 = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( body1, &shapeDef, &box.base );

			bodyDef.position = { startX + offsetX, startY, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, alpha );
			b3BodyId body2 = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( body2, &shapeDef, &box.base );

			startX += 4.0f * offsetX;
		}
	}

	void AddHorizontalRow( int n, float startX, float offsetX, float startY, const b3BoxHull& box )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.8f;

		for ( int index = 0; index < n; ++index )
		{
			bodyDef.position = { startX + index * offsetX, startY, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI );
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( body, &shapeDef, &box.base );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new CardHouseThick( context );
	}
};

static int sampleCardHouseThick = RegisterSample( "Stacking", "Card House Thick", CardHouseThick::Create );

// From PEEL
class CardHouse : public Sample
{
public:
	explicit CardHouse( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 30.0f, 10.0f, 3.0f, { 0.75, 1.0, 0.4f } );
		}

		AddGroundBox( 10.0f );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.7f;

		float cardHeight = 0.2f;
		float cardThickness = 0.001f;
		float cardDepth = 0.1f;

		float angle0 = 25.0f * B3_PI / 180.0f;
		float angle1 = -25.0f * B3_PI / 180.0f;
		float angle2 = 0.5f * B3_PI;

		// todo box hull is limiting the minimum thickness, breaking this test
		b3BoxHull cardBox = b3MakeBoxHull( cardThickness, cardHeight, cardDepth );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;

		int Nb = 5;
		float z0 = 0.0f;
		float y = cardHeight - 0.02f;
		while ( Nb )
		{
			float z = z0;
			for ( int i = 0; i < Nb; i++ )
			{
				if ( i != Nb - 1 )
				{
					bodyDef.position = { z + 0.25f, y + cardHeight - 0.015f, 0.0f };
					bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, angle2 );
					b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
					b3CreateHullShape( bodyId, &shapeDef, &cardBox.base );
				}

				bodyDef.position = { z, y, 0.0f };
				bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, angle1 );
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
				b3CreateHullShape( bodyId, &shapeDef, &cardBox.base );

				z += 0.175f;

				bodyDef.position = { z, y, 0.0f };
				bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, angle0 );
				bodyId = b3CreateBody( m_worldId, &bodyDef );
				b3CreateHullShape( bodyId, &shapeDef, &cardBox.base );

				z += 0.175f;
			}
			y += cardHeight * 2.0f - 0.03f;
			z0 += 0.175f;
			Nb--;
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new CardHouse( context );
	}
};

static int sampleCardHouse = RegisterSample( "Stacking", "Card House", CardHouse::Create );

class SphereStack : public Sample
{
public:
	explicit SphereStack( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 50.0f, { 0.0f, 10.0f, 0.0f } );
		}

		AddGroundBox( 15.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;

		float r = 0.5f;
		b3Sphere sphere = { b3Vec3_zero, r };
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.rollingResistance = 0.1f;

		float y = 1.5f * r;

		for ( int i = 0; i < 30; ++i )
		{
			bodyDef.name = "sphere";
			// bodyDef.position.x = 0.1f * i;
			bodyDef.position.y = y;
			bodyDef.angularVelocity = { 0.0f, 0.0f, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			if ( i == 0 )
			{
				m_bodyId = bodyId;
			}

			// shapeDef.density = 1.0f + 4.0f * i;
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );

			y += 3.0f * r;
		}
	}

	void Step() override
	{
		// b3Vec3 p = b3Body_GetPosition( m_bodyId );
		// printf( "%g %g %g\n", p.x, p.y, p.z );

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new SphereStack( context );
	}

	b3BodyId m_bodyId;
};

static int sampleSphereStack = RegisterSample( "Stacking", "Sphere Stack", SphereStack::Create );

class CapsuleStack : public Sample
{
public:
	explicit CapsuleStack( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 50.0f, { 0.0f, 10.0f, 0.0f } );
		}

		AddGroundBox( 40.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.motionLocks.linearZ = true;
		bodyDef.motionLocks.angularX = true;
		bodyDef.motionLocks.angularY = true;
		bodyDef.motionLocks.angularZ = true;

		float r = 0.5f;
		b3Capsule capsule = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, r };
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		float y = 1.5f * r;

		for ( int i = 0; i < 20; ++i )
		{
			bodyDef.position.y = y;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );

			y += 2.0f * r;
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new CapsuleStack( context );
	}
};

static int sampleCapsuleStack = RegisterSample( "Stacking", "Capsule Stack", CapsuleStack::Create );

class SingleBox : public Sample
{
public:
	explicit SingleBox( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 25.0f, 10.0f, b3Pos_zero );
		}

		AddGroundBox( 20.0f );

		{
			b3BoxHull cube = b3MakeCubeHull( 0.5f );
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "cube";
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 0.0f, 0.5f, 0.0f };
			bodyDef.angularVelocity = { 0.0f, 10.0f, 0.0f };
			m_bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( m_bodyId, &shapeDef, &cube.base );
		}
	}

	void Step() override
	{
		Sample::Step();

		b3Pos position = b3Body_GetPosition( m_bodyId );
		DrawTextLine( "(x, y, z) = (%.2g, %.2g, %.2g)", position.x, position.y, position.z );
	}

	static Sample* Create( SampleContext* context )
	{
		return new SingleBox( context );
	}

	b3BodyId m_bodyId;
};

static int sampleSingleBox = RegisterSample( "Stacking", "Single Box", SingleBox::Create );

class Cylinder : public Sample
{
public:
	explicit Cylinder( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 10.0f, b3Pos_zero );
		}

		AddGroundBox( 10.0f );

		{
			m_hull = b3CreateCylinder( 1.0f, 0.25f, 0.0f, 12 );
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "cylinder";
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 0.0f, 2.0f, 0.0f };
			bodyDef.linearVelocity = { 0.0f, 0.0f, 0.0f };
			// bodyDef.angularVelocity = { 0.0f, 5.0f, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.baseMaterial.rollingResistance = 0.05f;
			b3CreateHullShape( bodyId, &shapeDef, m_hull );
		}

		GetGuiDraw()->forceScale = 0.01f;
	}

	~Cylinder() override
	{
		b3DestroyHull( m_hull );
	}

	void Step() override
	{
		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new Cylinder( context );
	}

	b3HullData* m_hull;
};

static int sampleCylinder = RegisterSample( "Stacking", "Cylinder", Cylinder::Create );

class CylinderStack : public Sample
{
public:
	explicit CylinderStack( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 15.0f, { 0.0f, 5.0f, 0.0f } );
		}

		AddGroundBox( 10.0f );

		m_hull = b3CreateCylinder( 1.0f, 0.5f, 0.0f, 15 );

		b3Vec3 scales[4] = {
			b3Vec3_one,
			{ -0.75f, 1.0f, 1.0f },
			{ 1.2f, 1.0f, -0.9f },
			{ 0.9f, 0.9f, 0.9f },
		};

		for ( int i = 0; i < 10; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "cylinder";
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 0.0f, 0.0f + 1.1f * i, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			// shapeDef.baseMaterial.rollingResistance = 0.1f;
			b3CreateTransformedHullShape( bodyId, &shapeDef, m_hull, b3Transform_identity, scales[i % 4] );
		}

		GetGuiDraw()->forceScale = 0.001f;
	}

	~CylinderStack() override
	{
		b3DestroyHull( m_hull );
	}

	void Step() override
	{
		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new CylinderStack( context );
	}

	b3HullData* m_hull;
};

static int sampleCylinderStack = RegisterSample( "Stacking", "Cylinder Stack", CylinderStack::Create );

class BoxStack : public Sample
{
public:
	explicit BoxStack( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 50.0f, { 0.0f, 20.0f, 0.0f } );
		}

		AddGroundBox( 40.0f );

		float a = 0.5f;
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.name = "cube";
		bodyDef.type = b3_dynamicBody;

		b3BoxHull cube = b3MakeBoxHull( a, a, a );
		// b3Quat q = b3MakeQuatFromAxisAngle( { 1.0f, 0.0f, 0.0f }, 0.5f * B3_PI );

		for ( int i = 0; i < 40; ++i )
		{
			bodyDef.position = { 0.0f, 1.5f * a + 2.5f * a * i, 0.0f };
			// bodyDef.rotation = b3MulQuat( q, bodyDef.rotation );

			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.baseMaterial.rollingResistance = 0.1f;

			b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		}

		//{
		//	b3BoxHull wide = b3MakeBoxHull( 2.0f, 0.6f, 0.6f );
		//	b3BodyDef bodyDef = b3DefaultBodyDef();
		//	bodyDef.name = "wide";
		//	bodyDef.type = b3_dynamicBody;
		//	bodyDef.position = { 0.0f, 0.6f + 1.0f * m_count, 0.0f };
		//	b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		//	b3ShapeDef shapeDef = b3DefaultShapeDef();
		//	b3CreateHullShape( bodyId, &shapeDef, &wide.base );
		//}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BoxStack( context );
	}
};

static int sampleBoxStack = RegisterSample( "Stacking", "Box Stack", BoxStack::Create );

class JengaStack : public Sample
{
public:
	explicit JengaStack( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 35.0f, 15.0f, 30.0f, { 0.0f, 10.0f, 0.0f } );
		}

		m_shapeType = b3_hullShape;
		CreateStack();
	}

	void CreateStack()
	{
		AddGroundBox( 60.0f );

		{
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.baseMaterial.rollingResistance = m_shapeType == b3_capsuleShape ? 0.1f : 0.01f;
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;

			b3BoxHull box = b3MakeBoxHull( 2.5f, 0.25f, 0.25f );
			b3Capsule capsule = { { -2.5f, 0.0f, 0.0f }, { 2.5f, 0.0f, 0.0f }, 0.25f };

			for ( int i = 0; i < m_size; ++i )
			{
				float alpha = ( i & 1 ) == 1 ? 0.0f : 0.5f * B3_PI;

				float x = ( i & 1 ) == 0 ? 1.75f : 0.0f;
				float z = ( i & 1 ) == 0 ? 0.0f : 1.75f;

				bodyDef.position = { x, 0.5f * i + 0.25f, z };
				bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisY, alpha );
				b3BodyId boxBody1 = b3CreateBody( m_worldId, &bodyDef );

				bodyDef.position = { -x, 0.5f * i + 0.25f, -z };
				bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisY, alpha );
				b3BodyId boxBody2 = b3CreateBody( m_worldId, &bodyDef );

				if ( m_shapeType == b3_capsuleShape )
				{
					b3CreateCapsuleShape( boxBody1, &shapeDef, &capsule );
					b3CreateCapsuleShape( boxBody2, &shapeDef, &capsule );
				}
				else
				{
					b3CreateHullShape( boxBody1, &shapeDef, &box.base );
					b3CreateHullShape( boxBody2, &shapeDef, &box.base );
				}
			}
		}
	}

	bool DrawControls() override
	{
		b3Capacity capacity = {};

		if ( ImGui::RadioButton( "Capsule", m_shapeType == b3_capsuleShape ) )
		{
			m_shapeType = b3_capsuleShape;
			CreateWorld( &capacity );
			CreateStack();
		}

		if ( ImGui::RadioButton( "Hull", m_shapeType == b3_hullShape ) )
		{
			m_shapeType = b3_hullShape;
			CreateWorld( &capacity );
			CreateStack();
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new JengaStack( context );
	}

	static constexpr int m_size = 40;
	b3ShapeType m_shapeType;
};

static int sampleJengaStack = RegisterSample( "Stacking", "Jenga Stack", JengaStack::Create );

class Dominoes : public Sample
{
public:
	explicit Dominoes( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			if (m_isDebug)
			{
				m_camera->SetView( 0.0f, 15.0f, 25.0f, b3Pos_zero );
			}
			else
			{
				m_camera->SetView( 0.0f, 15.0f, 75.0f, b3Pos_zero );
			}
		}

		AddGroundBox( 80.0f );

		constexpr int n = m_isDebug ? 2 : 30;

		b3BoxHull box = b3MakeBoxHull( 0.2f, 0.8f, 0.05f );
		for ( int ring = 0; ring < n; ++ring )
		{
			float radius = 7.0f + 1.1f * ring;
			CreateRing( radius, box );
		}
	}

	void CreateRing( float radius, b3BoxHull& box )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		for ( float alpha = 0; alpha <= 360.0f; alpha += 2.0f )
		// for (float Alpha = 0; Alpha <= 4.0f; Alpha += 4.0f)
		{
			b3CosSin cs = b3ComputeCosSin( B3_DEG_TO_RAD * alpha );
			b3Pos position = { radius * cs.cosine, 0.8f, radius * cs.sine };
			b3Vec3 normal = { cs.cosine, 0.0f, cs.sine };
			position = position - alpha / 630.0f * normal;

			b3Quat orientation = b3MakeQuatFromAxisAngle( b3Vec3_axisY, -B3_DEG_TO_RAD * alpha );

			bodyDef.position = position;
			bodyDef.rotation = orientation;
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( body, &shapeDef, &box.base );

			if ( alpha == 0.0f )
			{
				b3Body_ApplyLinearImpulse( body, { 0.0f, 0.0f, 25.0f }, position + b3Vec3{ 0.0f, 0.8f, 0.0f }, true );
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new Dominoes( context );
	}
};

static int sampleDominoes = RegisterSample( "Stacking", "Dominoes", Dominoes::Create );

// This wedge shape can have an incorrect manifold if not handled correctly
class Wedge : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new Wedge( context );
	}

	explicit Wedge( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 75.0f, 10.0f, 10.0f, b3Pos_zero );
		}

		AddGroundBox( 20.0f );

		b3Vec3 vertices[] = {
			{ -1.0, 1.0f, -0.1f }, { 1.0, 1.0f, -0.1f }, { -1.0, 1.0f, 0.1f },
			{ 1.0, 1.0f, 0.1f },   { -0.5, 0.5f, 0.0f }, { 0.5, 0.5f, 0.0f },
		};

		m_wedgeHull = b3CreateHull( vertices, 6, 6 );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 1.0f, 0.0f };
		b3BodyId wedgeBody = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHullShape( wedgeBody, &shapeDef, m_wedgeHull );
	}

	~Wedge() override
	{
		b3DestroyHull( m_wedgeHull );
	}

	b3HullData* m_wedgeHull;
};

static int sampleWedge = RegisterSample( "Stacking", "Wedge", Wedge::Create );

// jitter
/*
 *+		position	[ -459.292877, 217.398331, 1.00115335 ]	b3Vec3
+		rotation	[ [ -0.00000000, 0.00000000, -0.707106769 ], 0.707106769 ]	b3Quat

points[0]	= { -44.8770714, -91.6598053, -1.92012548 };
points[1]	= { -92.5001831, 51.0151291, 15.8006573 };
points[2]	= { -91.0282211, -9.44371605, 15.6148796 };
points[3]	= { 90.2375641, 77.3870087, 15.9356089 };
points[4]	= { -85.5353241, 91.3750992, -1.36629653 };
points[5]	= { 88.9092178, -87.2975464, -1.86754704 };
points[6]	= { 83.7932816, -89.8572235, 15.4168339 };
points[7]	= { 87.0243988, 88.9776535, -1.32423306 };
points[8]	= { -91.6564941, -85.4949493, 15.3782759 };
points[9]	= { -90.2922516, -87.2074127, -1.92012548 };
points[10]	= { -87.2944870, 89.9510498, 15.9215889 };
points[11]	= { 79.2338104, 89.9690781, 15.9724140 };
points[12]	= { -91.6744461, 81.0823212, -1.39959598 };
points[13]	= { 90.3452759, -76.4459610, 15.4588966 };
points[14]	= { -87.4021912, -89.2263107, 15.3677588 };
points[15]	= { 76.3258057, 92.0059967, 1.82873762 };
*/

/*
 *
 *+		position	[ -402.321838, 157.310364, 16.8169250 ]	b3Vec3
+		rotation	[ [ -0.00000000, 0.00000000, -0.00152086187 ], 0.999998868 ]	b3Quat

 *
points[0]	= { 29.5000000, 17.1488495, 0.175081104 };
points[1]	= { 29.5000000, -17.2990532, 0.125000000 };
points[2]	= { 29.4840164, -17.3057766, 24.0200863 };
points[3]	= { 29.4840164, 17.1648350, 24.1781254 };
points[4]	= { -29.1345520, 17.5529804, 0.125000000 };
points[5]	= { -29.1345520, 17.5529804, 23.7899799 };
points[6]	= { -29.1441040, 16.9679585, 24.3750000 };
points[7]	= { -29.1345520, -17.2990532, 24.3750000 };
points[8]	= { -29.1345520, -17.2990532, 0.175081253 };
points[9]	= { 29.0720215, 17.5529785, 0.125000000 };
points[10]	= { 29.0859070, 17.5629406, 23.8120594 };
points[11]	= { 29.1401348, -17.2990532, 24.3750000 };
points[12]	= { 29.1123581, 16.9722290, 24.4027710 };
points[13]	= { 29.3944912, 17.2543602, 24.1206398 };
points[14]	= { -29.1345520, -17.2990532, 24.0759430 };
points[15]	= { -29.1345520, -16.9722252, 24.4027710 };
points[16]	= { 29.1123619, -16.9722271, 24.4027729 };
points[17]	= { 29.5000000, 17.3429642, 24.0000000 };
*/

class Arch : public Sample
{
public:
	explicit Arch( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 25.0f, 10.0f, 30.0f, { 0.0f, 5.0f, 0.0f } );
		}

		AddGroundBox( 40.0f );

		b3Vec3 ps1[9] = { { 16.0f, 0.0f, 0.0f },
						  { 14.93803712795643f, 5.133601056842984f, 0.0f },
						  { 13.79871746027416f, 10.24928069555078f, 0.0f },
						  { 12.56252963284711f, 15.34107019122473f, 0.0f },
						  { 11.20040987372525f, 20.39856541571217f, 0.0f },
						  { 9.66521217819836f, 25.40369899225096f, 0.0f },
						  { 7.87179930638133f, 30.3179337000085f, 0.0f },
						  { 5.635199558196225f, 35.03820717801641f, 0.0f },
						  { 2.405937953536585f, 39.09554102558315f, 0.0f } };

		b3Vec3 ps2[9] = { { 24.0f, 0.0f, 0.0f },
						  { 22.33619528222415f, 6.02299846205841f, 0.0f },
						  { 20.54936888969905f, 12.00964361211476f, 0.0f },
						  { 18.60854610798073f, 17.9470321677465f, 0.0f },
						  { 16.46769273811807f, 23.81367936585418f, 0.0f },
						  { 14.05325025774858f, 29.57079353071012f, 0.0f },
						  { 11.23551045834022f, 35.13775818285372f, 0.0f },
						  { 7.752568160730571f, 40.30450679009583f, 0.0f },
						  { 3.016931552701656f, 44.28891593799322f, 0.0f } };

		float scale = 0.25f;
		for ( int i = 0; i < 9; ++i )
		{
			ps1[i].x *= scale;
			ps1[i].y *= scale;
			ps2[i].x *= scale;
			ps2[i].y *= scale;
		}

		const float halfDepth = 0.5f;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 200.0f;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;

		for ( int i = 0; i < 8; ++i )
		{
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3Vec3 ps[8] = {
				{ ps1[i].x, ps1[i].y, -halfDepth },			{ ps2[i].x, ps2[i].y, -halfDepth },
				{ ps2[i + 1].x, ps2[i + 1].y, -halfDepth }, { ps1[i + 1].x, ps1[i + 1].y, -halfDepth },
				{ ps1[i].x, ps1[i].y, halfDepth },			{ ps2[i].x, ps2[i].y, halfDepth },
				{ ps2[i + 1].x, ps2[i + 1].y, halfDepth },	{ ps1[i + 1].x, ps1[i + 1].y, halfDepth },
			};
			b3HullData* hull = b3CreateHull( ps, 8, 8 );
			b3CreateHullShape( bodyId, &shapeDef, hull );
			b3DestroyHull( hull );
		}

		for ( int i = 0; i < 8; ++i )
		{
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3Vec3 ps[8] = {
				{ -ps2[i].x, ps2[i].y, -halfDepth },		 { -ps1[i].x, ps1[i].y, -halfDepth },
				{ -ps1[i + 1].x, ps1[i + 1].y, -halfDepth }, { -ps2[i + 1].x, ps2[i + 1].y, -halfDepth },
				{ -ps2[i].x, ps2[i].y, halfDepth },			 { -ps1[i].x, ps1[i].y, halfDepth },
				{ -ps1[i + 1].x, ps1[i + 1].y, halfDepth },	 { -ps2[i + 1].x, ps2[i + 1].y, halfDepth },
			};
			b3HullData* hull = b3CreateHull( ps, 8, 8 );
			b3CreateHullShape( bodyId, &shapeDef, hull );
			b3DestroyHull( hull );
		}

		{
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3Vec3 ps[8] = {
				{ ps1[8].x, ps1[8].y, -halfDepth },	 { ps2[8].x, ps2[8].y, -halfDepth }, { -ps2[8].x, ps2[8].y, -halfDepth },
				{ -ps1[8].x, ps1[8].y, -halfDepth }, { ps1[8].x, ps1[8].y, halfDepth },	 { ps2[8].x, ps2[8].y, halfDepth },
				{ -ps2[8].x, ps2[8].y, halfDepth },	 { -ps1[8].x, ps1[8].y, halfDepth },
			};
			b3HullData* hull = b3CreateHull( ps, 8, 8 );
			b3CreateHullShape( bodyId, &shapeDef, hull );
			b3DestroyHull( hull );
		}

		for ( int i = 0; i < 4; ++i )
		{
			b3BoxHull box = b3MakeBoxHull( 2.0f, 0.5f, halfDepth );
			bodyDef.position = { 0.0f, 0.5f + ps2[8].y + 1.0f * i, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new Arch( context );
	}
};

static int sampleArch = RegisterSample( "Stacking", "Arch", Arch::Create );

class DoubleDomino : public Sample
{
public:
	explicit DoubleDomino( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 15.0f, 15.0f, { 0.0f, 0.5f, 1.0f } );
		}

		AddGroundBox( 20.0f );

		b3BoxHull box = b3MakeBoxHull( 0.125f, 0.5f, 0.25f );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.6f;
		shapeDef.density = 4.0f;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;

		int count = 15;
		float x = -0.5f * count;
		for ( int i = 0; i < count; ++i )
		{
			bodyDef.position = { x, 0.5f, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
			if ( i == 0 )
			{
				b3Body_ApplyLinearImpulse( bodyId, { 0.2f, 0.0f, 0.0f }, { x, 1.0f, 0.0f }, true );
			}

			x += 1.01f;
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new DoubleDomino( context );
	}
};

static int sampleDoubleDomino = RegisterSample( "Stacking", "Double Domino", DoubleDomino::Create );

class Pyramid2D : public Sample
{
public:
	explicit Pyramid2D( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 30.0f, 50.0f, { 0.0f, 5.0f, 0.0f } );
		}

		AddGroundBox( 40.0f );

		float a = 1.0f;
		b3BoxHull box = b3MakeBoxHull( a, a, a );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.motionLocks.linearZ = true;
		bodyDef.motionLocks.angularX = true;
		bodyDef.motionLocks.angularY = true;

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		for ( int row = 0; row < m_size; ++row )
		{
			for ( int column = 0; column < m_size - row; ++column )
			{
				bodyDef.position = { ( -10.0f + 2.0f * column + row ) * a, ( 1.5f + 2.5f * row ) * a, 0.0f };
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new Pyramid2D( context );
	}

	static constexpr int m_size = 12;
};

static int samplePyramid2D = RegisterSample( "Stacking", "Pyramid2D", Pyramid2D::Create );
