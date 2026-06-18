// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/draw.h"
#include "human.h"
#include "imgui.h"
#include "mesh_loader.h"
#include "sample.h"
#include "utils.h"

#include "box3d/box3d.h"

class InclinedPlane : public Sample
{
public:
	explicit InclinedPlane( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -55.0f, 30.0f, 60.0f, { 0.0f, 7.5f, 0.0f } );
		}

		AddGroundBox( 50.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { 0.0f, 7.5f, -5.0f };
		bodyDef.rotation = b3MakeQuatFromAxisAngle( { 1.0f, 0.0f, 0.0f }, 40.0f * B3_DEG_TO_RAD );
		b3BodyId planeBody = b3CreateBody( m_worldId, &bodyDef );
		b3BoxHull planeBox = b3MakeBoxHull( 16.0f, 0.5f, 10.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 1.0f;
		b3CreateHullShape( planeBody, &shapeDef, &planeBox.base );

		b3BoxHull box = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
		bodyDef.type = b3_dynamicBody;
		for ( int index = 0; index < m_boxCount; ++index )
		{
			bodyDef.position = { -10.0f + 5.0f * index, 15.75f, -10.6f };
			b3BodyId boxBody = b3CreateBody( m_worldId, &bodyDef );
			shapeDef.baseMaterial.friction = ( index + 1 ) * ( index + 1 ) * 0.04f;
			b3CreateHullShape( boxBody, &shapeDef, &box.base );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new InclinedPlane( context );
	}

	static constexpr int m_boxCount = 5;
};

static int sampleInclinedPlane = RegisterSample( "Shapes", "Inclined Plane", InclinedPlane::Create );

class RollingResistance : public Sample
{
public:
	explicit RollingResistance( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( -140.0f, 17.0f, 60.0f, { 0.0f, 7.5f, 0.0f } );
		}

		AddGroundBox( 50.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		bodyDef.position = { 0.0f, 2.0f, -20.0f };
		bodyDef.rotation = b3MakeQuatFromAxisAngle( { 1.0f, 0.0f, 0.0f }, 10.0f * B3_DEG_TO_RAD );
		b3BodyId planeBody = b3CreateBody( m_worldId, &bodyDef );

		b3BoxHull plane = b3MakeBoxHull( 32.0f, 0.5f, 15.0f );
		b3CreateHullShape( planeBody, &shapeDef, &plane.base );

		b3Sphere sphere = { b3Vec3_zero, 1.0f };
		bodyDef.type = b3_dynamicBody;
		for ( int index = 0; index < m_count; ++index )
		{
			bodyDef.position = { -25.0f + 5.0f * index, 8.0f, -24.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			shapeDef.baseMaterial.rollingResistance = 0.05f * index;
			b3CreateSphereShape( body, &shapeDef, &sphere );
		}

		b3Capsule capsule = { { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 0.5f };
		bodyDef.type = b3_dynamicBody;
		for ( int index = 0; index < m_count; ++index )
		{
			bodyDef.position = { 2.0f + 5.0f * index, 8.0f, -24.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			shapeDef.baseMaterial.rollingResistance = 0.05f * index;
			b3CreateCapsuleShape( body, &shapeDef, &capsule );
		}
	}

	static constexpr int m_count = 5;

	static Sample* Create( SampleContext* context )
	{
		return new RollingResistance( context );
	}
};

static int sampleRollingResistance = RegisterSample( "Shapes", "Rolling Resistance", RollingResistance::Create );

class HighResistance : public Sample
{
public:
	explicit HighResistance( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 0.0f, 5.0f, 40.0f, { 0.0f, 7.5f, 0.0f } );
		}

		AddGroundBox( 50.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		b3Capsule capsule = { { 0.0f, -1.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 0.5f };
		bodyDef.type = b3_dynamicBody;
		bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, B3_DEG_TO_RAD * 30.0f );
		for ( int index = 0; index < m_count; ++index )
		{
			bodyDef.position = { -22.0f + 5.0f * index, 1.5f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			shapeDef.baseMaterial.rollingResistance = 0.2f * index;
			b3CreateCapsuleShape( body, &shapeDef, &capsule );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new HighResistance( context );
	}

	static constexpr int m_count = 10;
};

static int sampleHighResistance = RegisterSample( "Shapes", "High Resistance", HighResistance::Create );

class IsotropicFriction : public Sample
{
public:
	explicit IsotropicFriction( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 150.0f, b3Pos_zero );
		}

		AddGroundBox( 100.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BoxHull box = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
		bodyDef.type = b3_dynamicBody;
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.6f;
		for ( int index = 0; index < m_boxCount; ++index )
		{
			float alpha = B3_PI / 16.0f * index;
			b3CosSin cs = b3ComputeCosSin( alpha );
			b3Pos position = { 15.0f * cs.cosine, 1.0f, 15.0f * cs.sine };
			b3Quat orientation = b3MakeQuatFromAxisAngle( { 0.0f, 1.0f, 0.0f }, -alpha );
			b3Vec3 velocity = { 25.0f * cs.cosine, 0.0f, 25.0f * cs.sine };

			bodyDef.position = position;
			bodyDef.rotation = orientation;
			bodyDef.linearVelocity = velocity;
			b3BodyId boxBody = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( boxBody, &shapeDef, &box.base );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new IsotropicFriction( context );
	}

	static constexpr int m_boxCount = 32;
};

static int sampleIsotropicFriction = RegisterSample( "Shapes", "Isotropic Friction", IsotropicFriction::Create );

// todo what is the point of this?
class SlideTwist : public Sample
{
public:
	explicit SlideTwist( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -30.0f, 17.0f, 30.0f, { 0.0f, 5.0f, 0.0f } );
		}

		AddGroundBox( 50.0f );

		b3Quat orientation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 20.0f * B3_DEG_TO_RAD );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 1.0f;

		bodyDef.position = { 0.0f, 4.0f, 0.0f };
		bodyDef.rotation = orientation;
		b3BodyId planeBody = b3CreateBody( m_worldId, &bodyDef );

		b3BoxHull plane = b3MakeBoxHull( 10.0f, 0.5f, 10.0f );
		shapeDef.baseMaterial.friction = 0.6f;
		b3CreateHullShape( planeBody, &shapeDef, &plane.base );

		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 5.0f, 0.0f };
		bodyDef.rotation = orientation;
		bodyDef.angularVelocity = 25.0f * b3RotateVector( orientation, b3Vec3_axisY );
		b3BodyId boxBody = b3CreateBody( m_worldId, &bodyDef );
		b3BoxHull mBox = b3MakeBoxHull( 1.0f, 0.5f, 1.0f );
		shapeDef.baseMaterial.friction = 0.3f;
		b3CreateHullShape( boxBody, &shapeDef, &mBox.base );
	}

	static Sample* Create( SampleContext* context )
	{
		return new SlideTwist( context );
	}
};

static int sampleSlideTwist = RegisterSample( "Shapes", "Slide Twist", SlideTwist::Create );

class Restitution : public Sample
{
public:
	enum ShapeType
	{
		e_sphereShape = 0,
		e_boxShape
	};

	explicit Restitution( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 25.0f, 85.0f, { 0.0f, 20.0f, 0.0f } );
		}

		AddGroundBox( 50.0f );

		m_shapeType = e_sphereShape;

		CreateBodies();
	}

	void CreateBodies()
	{
		for ( int i = 0; i < m_count; ++i )
		{
			if ( B3_IS_NON_NULL( m_bodyIds[i] ) )
			{
				b3DestroyBody( m_bodyIds[i] );
				m_bodyIds[i] = b3_nullBodyId;
			}
		}

		b3Sphere sphere = { b3Vec3_zero, 0.5f };
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;

		float dr = 1.0f / ( m_count > 1 ? m_count - 1 : 1 );
		float x = -1.0f * ( m_count - 1 );
		float dx = 2.0f;

		for ( int i = 0; i < m_count; ++i )
		{
			bodyDef.position = { x, 40.0f, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			m_bodyIds[i] = bodyId;

			if ( m_shapeType == e_sphereShape )
			{
				b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			}
			else
			{
				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}

			shapeDef.baseMaterial.restitution += dr;
			x += dx;
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::RadioButton( "Sphere", m_shapeType == e_sphereShape ) )
		{
			m_shapeType = e_sphereShape;
			CreateBodies();
		}

		if ( ImGui::RadioButton( "Box", m_shapeType == e_boxShape ) )
		{
			m_shapeType = e_boxShape;
			CreateBodies();
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new Restitution( context );
	}

	static constexpr int m_count = 40;

	b3BodyId m_bodyIds[m_count] = {};
	ShapeType m_shapeType;
};

static int sampleRestitution = RegisterSample( "Shapes", "Restitution", Restitution::Create );

// This shows an optimization when creating many static shapes you can skip having them invoke collision, assuming
// dynamic bodies are added after the static bodies.
class StaticInvoke : public Sample
{
public:
	explicit StaticInvoke( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 25.0f, 10.0f, { 0.0f, 1.0f, 0.0f } );
		}

		AddGroundBox( 20.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.25f, 1.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.rollingResistance = 0.2f;
		b3Sphere sphere = { b3Vec3_zero, 0.5f };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );

		m_invoke = false;
		m_bodyId = {};
	}

	void CreateStatic()
	{
		if ( B3_IS_NON_NULL( m_bodyId ) )
		{
			b3DestroyBody( m_bodyId );
			m_bodyId = b3_nullBodyId;
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { 0.0f, 0.5f, 0.0f };
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );
		b3Sphere sphere = { b3Vec3_zero, 0.5f };
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.invokeContactCreation = m_invoke;

		b3CreateSphereShape( m_bodyId, &shapeDef, &sphere );
	}

	bool DrawControls() override
	{
		if ( ImGui::RadioButton( "Invoke", m_invoke == true ) )
		{
			m_invoke = true;
		}

		if ( ImGui::RadioButton( "Passive", m_invoke == false ) )
		{
			m_invoke = false;
		}

		if ( B3_IS_NULL( m_bodyId ) )
		{
			if ( ImGui::Button( "Create" ) )
			{
				CreateStatic();
			}
		}
		else
		{
			if ( ImGui::Button( "Destroy" ) )
			{
				b3DestroyBody( m_bodyId );
				m_bodyId = {};
			}
		}

		return true;
	}

	void Step() override
	{
		Sample::Step();

		if ( m_stepCount == 20 )
		{
			CreateStatic();
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new StaticInvoke( context );
	}

	b3BodyId m_bodyId;
	bool m_invoke;
};

static int sampleStaticInvoke = RegisterSample( "Shapes", "Static Invoke", StaticInvoke::Create );

class ConveyorBelt : public Sample
{
public:
	explicit ConveyorBelt( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 25.0f, 40.0f, { 0.0f, 1.0f, 0.0f } );
		}

		AddGroundBox( 20.0f );

		// Platform
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { -5.0f, 5.0f, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.2f );
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( 10.0f, 0.25f, 2.0f );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.baseMaterial.friction = 0.8f;
			shapeDef.baseMaterial.tangentVelocity = { 2.0f, 0.0f, 0.0f };
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}

		// Boxes
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull cube = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		for ( int i = 0; i < 5; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -10.0f + 2.0f * i, 7.0f, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new ConveyorBelt( context );
	}
};

static int sampleConveyorBelt = RegisterSample( "Shapes", "Conveyor Belt", ConveyorBelt::Create );

class ConveyorMesh : public Sample
{
public:
	explicit ConveyorMesh( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 65.0f, 25.0f, 28.0f, { 0.0f, 1.0f, 0.0f } );
		}

		AddGroundBox( 20.0f );

		// Mesh
		{
			m_meshData = CreateMeshData( "data/meshes/conveyor.obj", 1.0f, false, true, true, true );

			int triangleCount = m_meshData->triangleCount;

			uint8_t* materialIndices = (uint8_t*)( (intptr_t)m_meshData + m_meshData->materialOffset );

			memset( materialIndices, 0, triangleCount * sizeof( uint8_t ) );
			m_velocities[0] = { 0.0f, 0.0f, 0.0f };

			// +x -z
			materialIndices[0] = 1;
			materialIndices[4] = 1;
			m_velocities[1] = { 0.7f, 0.0f, -0.2f };

			// +x +z
			materialIndices[9] = 2;
			materialIndices[12] = 2;
			m_velocities[2] = { 0.6f, 0.0f, 0.4f };

			// +z
			materialIndices[21] = 3;
			materialIndices[38] = 3;
			m_velocities[3] = { 0.0f, 0.0f, 1.3f };

			// -x +z
			materialIndices[43] = 4;
			materialIndices[46] = 4;
			m_velocities[4] = { -0.6f, 0.0f, 0.4f };

			// -x -z
			materialIndices[30] = 5;
			materialIndices[33] = 5;
			m_velocities[5] = { -0.75f, 0.0f, -0.4f };

			// -z
			materialIndices[18] = 6;
			materialIndices[24] = 6;
			m_velocities[6] = { 0.0f, 0.0f, -1.3f };

			b3HexColor colors[7] = {
				b3_colorGreen,	   b3_colorGreenYellow, b3_colorHoneyDew, b3_colorHotPink,
				b3_colorIndianRed, b3_colorIndigo,		b3_colorIvory,
			};

			b3SurfaceMaterial materials[7];
			for ( int i = 0; i < 7; ++i )
			{
				materials[i] = b3DefaultSurfaceMaterial();
				materials[i].friction = 0.8f;
				materials[i].tangentVelocity = 2.0f * m_velocities[i];
				materials[i].customColor = colors[i];
			}

			m_meshTransform.p = { 0.0f, 0.5f, 6.0f };
			m_meshTransform.q = b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.5f * B3_PI );
			// m_meshTransform.q = b3Quat_identity;

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = m_meshTransform.p;
			bodyDef.rotation = m_meshTransform.q;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.materials = materials;
			shapeDef.materialCount = 7;
			b3CreateMeshShape( bodyId, &shapeDef, m_meshData, b3Vec3_one );
		}

		// High number of sides to stress the collision code
		// Normally the number of sides should be 16 or less.
		m_cylinderHull = b3CreateCylinder( 0.3f, 0.15f, 0.0f, 32 );
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		// Cylinders
		for ( int i = 0; i < 20; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -8.5f + 0.9f * i, 1.5f, -5.5f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3CreateHullShape( bodyId, &shapeDef, m_cylinderHull );
		}

#if 0
		// Boxes
		b3BoxHull cube = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		for ( int i = 0; i < 5; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -8.5f + 3.0f * i, 2.0f, -7.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3CreateHullShape( bodyId, &shapeDef, &cube.base );
		}
		b3Sphere sphere = { b3Vec3_zero, 0.25f };
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		for ( int i = 0; i < 10; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -8.5f + 2.0f * i, 2.0f, -6.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		}
#endif

#if 0
		for ( int i = 0; i < 10; ++i )
		{
			Human human = {};
			CreateHuman( &human, m_worldId, { -8.0f + 2.0f * i, 3.0f, -5.5f }, 10.0f, 1.0f, 1.0f, 0, nullptr, true );
		}
#endif
	}

	~ConveyorMesh() override
	{
		b3DestroyMesh( m_meshData );
		b3DestroyHull( m_cylinderHull );
	}

	void Render() override
	{
		Sample::Render();

		int triangleCount = m_meshData->triangleCount;
		const b3MeshTriangle* triangles = b3GetMeshTriangles( m_meshData );
		const b3Vec3* vertices = b3GetMeshVertices( m_meshData );
		const uint8_t* materialIndices = b3GetMeshMaterialIndices( m_meshData );

		for ( int i = 0; i < triangleCount; ++i )
		{
			const b3MeshTriangle* t = triangles + i;
			b3Vec3 v1 = vertices[t->index1];
			b3Vec3 v2 = vertices[t->index2];
			b3Vec3 v3 = vertices[t->index3];

			b3Vec3 n = b3Cross( v2 - v1, v3 - v1 );
			n = b3Normalize( n );

			if ( n.y < 0.9f )
			{
				continue;
			}

			b3Pos p = b3TransformWorldPoint( m_meshTransform, 1.0f / 3.0f * ( v1 + v2 + v3 ) );

			DrawString3D( p, MakeColor( b3_colorAqua ), "%d", i );

			int materialIndex = materialIndices[i];

			b3Vec3 v = b3RotateVector( m_meshTransform.q, m_velocities[materialIndex] );
			DrawLine( p, p + v, MakeColor( b3_colorBlueViolet ) );
		}

		DrawAxes( b3WorldTransform_identity, 0.5f );
	}

	static Sample* Create( SampleContext* context )
	{
		return new ConveyorMesh( context );
	}

	b3WorldTransform m_meshTransform;
	b3MeshData* m_meshData;
	b3HullData* m_cylinderHull;
	b3Vec3 m_velocities[7];
};

static int sampleConveyorMesh = RegisterSample( "Shapes", "Conveyor Mesh", ConveyorMesh::Create );

class Wind : public Sample
{
public:
	enum ShapeType
	{
		e_sphereShape = 0,
		e_capsuleShape,
		e_boxShape
	};

	explicit Wind( SampleContext* context )
		: Sample( context )
		, m_bodyIds{}
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 0.0f, 0.0f, 5.0f, { 0.0f, 1.0f, 0.0f } );
		}

		AddGroundBox( 20.0f );

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			m_groundId = b3CreateBody( m_worldId, &bodyDef );
		}

		m_shapeType = e_boxShape;
		m_wind = { 6.0f, 0.0f, 0.0f };
		m_drag = 1.0f;
		m_lift = 0.75f;
		m_count = 10;
		m_noise = { 0.0f, 0.0f, 0.0f };

		// Need this to be false for debug draw to work
		m_stepWhilePaused = false;

		CreateScene();
	}

	void CreateScene()
	{
		for ( int i = 0; i < m_maxCount; ++i )
		{
			if ( B3_IS_NON_NULL( m_bodyIds[i] ) )
			{
				b3DestroyBody( m_bodyIds[i] );
				m_bodyIds[i] = b3_nullBodyId;
			}
		}

		float radius = 0.1f;
		float verticalOffset = 2.0f;

		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, radius };
		b3Capsule capsule = { { -radius, 0.0f, 0.0f }, { radius, 0.0f, 0.0f }, 0.5f * radius };
		b3BoxHull box = b3MakeBoxHull( 1.25f * radius, 0.75f * radius, 0.125f * radius );

		b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
		jointDef.base.bodyIdA = m_groundId;
		jointDef.base.localFrameA.p = { 0.0f, verticalOffset, 0.0f };
		jointDef.base.drawScale = 0.1f;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 20.0f;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.gravityScale = 0.5f;
		bodyDef.enableSleep = false;

		for ( int i = 0; i < m_count; ++i )
		{
			bodyDef.position = { ( 2.0f * i + 1.0f ) * radius, verticalOffset, 0.0f };
			m_bodyIds[i] = b3CreateBody( m_worldId, &bodyDef );

			if ( m_shapeType == e_sphereShape )
			{
				b3CreateSphereShape( m_bodyIds[i], &shapeDef, &sphere );
			}
			else if ( m_shapeType == e_capsuleShape )
			{
				b3CreateCapsuleShape( m_bodyIds[i], &shapeDef, &capsule );
			}
			else
			{
				b3CreateHullShape( m_bodyIds[i], &shapeDef, &box.base );
			}

			jointDef.base.bodyIdB = m_bodyIds[i];
			jointDef.base.localFrameB.p = { -radius, 0.0f, 0.0f };
			b3CreateSphericalJoint( m_worldId, &jointDef );

			jointDef.base.bodyIdA = m_bodyIds[i];
			jointDef.base.localFrameA.p = { radius, 0.0f, 0.0f };
		}
	}

	bool DrawControls() override
	{
		const char* shapeTypes[] = { "Circle", "Capsule", "Box" };
		int shapeType = int( m_shapeType );
		if ( ImGui::Combo( "Shape", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) ) )
		{
			m_shapeType = ShapeType( shapeType );
			CreateScene();
		}

		ImGui::SliderFloat( "Wind", &m_wind.x, -50.0f, 50.0f, "%.1f" );
		ImGui::SliderFloat( "Drag", &m_drag, 0.0f, 1.0f, "%.2f" );
		ImGui::SliderFloat( "Lift", &m_lift, 0.0f, 4.0f, "%.2f" );
		if ( ImGui::SliderInt( "Count", &m_count, 1, m_maxCount, "%d" ) )
		{
			CreateScene();
		}

		return true;
	}

	void Step() override
	{
		bool shouldStep = m_context->pause == false || m_context->singleStep > 0;
		Sample::Step();

		if ( shouldStep == true )
		{
			float speed;
			b3Vec3 direction = b3GetLengthAndNormalize( &speed, m_wind );
			b3Vec3 wind = b3MulSV( speed, b3Add( direction, m_noise ) );

			for ( int i = 0; i < m_count; ++i )
			{
				b3ShapeId shapeIds[1];
				int count = b3Body_GetShapes( m_bodyIds[i], shapeIds, 1 );
				for ( int j = 0; j < count; ++j )
				{
					b3Shape_ApplyWind( shapeIds[j], wind, m_drag, m_lift, 10.0f, true );
				}
			}

			b3Vec3 rand = RandomVec3( { -0.3f, -0.3f, -0.3f }, { 0.3f, 0.3f, 0.3f } );
			m_noise = b3Lerp( m_noise, rand, 0.05f );

			b3Pos p1 = { 0.0f, 0.5f, 0.0f };
			b3Pos p2 = p1 + b3MulSV( 0.2f, wind );
			DrawArrow( p1, p2, MakeColor( b3_colorFuchsia ) );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new Wind( context );
	}

	static constexpr int m_maxCount = 60;

	ShapeType m_shapeType;
	b3Vec3 m_wind;
	float m_drag;
	float m_lift;
	b3Vec3 m_noise;
	b3BodyId m_groundId;
	b3BodyId m_bodyIds[m_maxCount];
	int m_count;
};

static int sampleWind = RegisterSample( "Shapes", "Wind", Wind::Create );

class WindDrop : public Sample
{
public:
	explicit WindDrop( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -45.0f, 15.0f, 20.0f, { 0.0f, 5.0f, 0.0f } );
		}

		AddGroundBox( 15.0f );

		m_drag = 1.0f;
		m_lift = 4.0f;

		// Need this to be false for debug draw to work
		m_stepWhilePaused = false;

		float radius = 0.1f;
		// b3BoxHull box = b3MakeBoxHull( 0.25f * radius, 1.25f * radius, 0.25f * radius );
		b3BoxHull box = b3MakeBoxHull( 4.0f * radius, 0.1f * radius, 4.0f * radius );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 2.0f;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.linearVelocity = { 0.0f, 0.0f, 0.0f };
		bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.25f );
		bodyDef.gravityScale = 0.5f;

		bodyDef.position = { 0.0f, 10.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		m_shapeId = b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	void Step() override
	{
		bool shouldStep = m_context->pause == false || m_context->singleStep > 0;
		Sample::Step();

		if ( shouldStep == true )
		{
			b3Shape_ApplyWind( m_shapeId, b3Vec3_zero, m_drag, m_lift, 10.0f, true );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new WindDrop( context );
	}

	float m_drag;
	float m_lift;
	b3ShapeId m_shapeId;
};

static int sampleWindDrop = RegisterSample( "Shapes", "Wind Drop", WindDrop::Create );

class WindFlap : public Sample
{
public:
	explicit WindFlap( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -35.0f, 15.0f, 65.0f, { 0.0f, 5.0f, 10.0f } );
		}

		AddGroundBox( 50.0f );

		m_drag = 1.0f;
		m_lift = 2.0f;

		// Need this to be false for debug draw to work
		m_stepWhilePaused = false;

		float a = 0.4f;
		// b3BoxHull box = b3MakeBoxHull( 0.25f * radius, 1.25f * radius, 0.25f * radius );
		b3Capsule capsule = { { 0.0f, 0.0f, -a }, { 0.0f, 0.0f, a }, 0.25f * a };
		// b3BoxHull box = b3MakeBoxHull( 2.0f * a, 0.01f, a );
		b3Transform wingTransform1 = { b3Vec3_zero, b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.1f ) };
		b3BoxHull box1 = b3MakeTransformedBoxHull( 2.0f * a, 0.01f, a, wingTransform1 );
		b3Transform wingTransform2 = { b3Vec3_zero, b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.1f ) };
		b3BoxHull box2 = b3MakeTransformedBoxHull( 2.0f * a, 0.01f, a, wingTransform2 );

		float y = 20.0f;

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		// bodyDef.gravityScale = 0.5f;

		shapeDef.density = 5.0f;
		bodyDef.position = { -2.0f * a, y, 0.0f };
		b3BodyId wingBodyId1 = b3CreateBody( m_worldId, &bodyDef );
		m_shapeId1 = b3CreateHullShape( wingBodyId1, &shapeDef, &box1.base );

		bodyDef.position = { 2.0f * a, y, 0.0f };
		b3BodyId wingBodyId2 = b3CreateBody( m_worldId, &bodyDef );
		m_shapeId2 = b3CreateHullShape( wingBodyId2, &shapeDef, &box2.base );

		bodyDef.position = { 0.0f, y, 0.0f };
		// bodyDef.type = b3_staticBody;
		b3BodyId torsoBodyId = b3CreateBody( m_worldId, &bodyDef );

		shapeDef.density = 10.0f;
		m_torsoShapeId = b3CreateCapsuleShape( torsoBodyId, &shapeDef, &capsule );

		b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
		jointDef.base.drawScale = 0.1f;
		jointDef.base.bodyIdA = torsoBodyId;
		jointDef.base.localFrameA.p = { 0.0f, 0.0f, 0.0f };
		jointDef.base.bodyIdB = wingBodyId1;
		jointDef.base.localFrameB.p = { 2.0f * a, 0.0f, 0.0f };
		jointDef.enableSpring = true;
		jointDef.hertz = 6.0f;
		jointDef.dampingRatio = 0.5f;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = -30.0f * B3_PI / 180.0f;
		jointDef.upperAngle = 30.0f * B3_PI / 180.0f;
		m_jointId1 = b3CreateRevoluteJoint( m_worldId, &jointDef );

		jointDef.base.bodyIdB = wingBodyId2;
		jointDef.base.localFrameB.p = { -2.0f * a, 0.0f, 0.0f };
		m_jointId2 = b3CreateRevoluteJoint( m_worldId, &jointDef );

		b3FilterJointDef filterDef = b3DefaultFilterJointDef();
		filterDef.base.bodyIdA = wingBodyId1;
		filterDef.base.bodyIdB = wingBodyId2;
		b3CreateFilterJoint( m_worldId, &filterDef );

		m_time = 0.0f;
	}

	void Step() override
	{
		bool shouldStep = m_context->pause == false || m_context->singleStep > 0;
		Sample::Step();

		if ( shouldStep == true )
		{
			float maxSpeed = 10.0f;
			bool wake = false;
			b3Shape_ApplyWind( m_shapeId1, b3Vec3_zero, m_drag, m_lift, maxSpeed, wake );
			b3Shape_ApplyWind( m_shapeId2, b3Vec3_zero, m_drag, m_lift, maxSpeed, wake );
			// b3Shape_ApplyWind( m_torsoShapeId, b3Vec3_zero, m_drag, m_lift, maxSpeed, wake );

			float angle = b3Sin( 10.0f * m_time );
			b3RevoluteJoint_SetTargetAngle( m_jointId1, angle );
			b3RevoluteJoint_SetTargetAngle( m_jointId2, -angle );

			m_time += m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new WindFlap( context );
	}

	float m_time;
	float m_drag;
	float m_lift;
	b3ShapeId m_shapeId1;
	b3ShapeId m_shapeId2;
	b3ShapeId m_torsoShapeId;
	b3JointId m_jointId1;
	b3JointId m_jointId2;
};

static int sampleWindFlap = RegisterSample( "Shapes", "Wind Flap", WindFlap::Create );
