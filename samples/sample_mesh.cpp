// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "human.h"
#include "mesh_loader.h"
#include "sample.h"
#include "gfx/draw.h"

#include "gfx/debug_adapter.h"

#include "box3d/box3d.h"

#include <imgui.h>
#include <queue>
#include <stdio.h>

enum class ShapeType
{
	sphere,
	capsule,
	box,
	cylinder
};

class GridMesh : public Sample
{
public:
	explicit GridMesh( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 6.0f, b3Pos_zero );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

		m_gridMesh = b3CreateGridMesh( 20, 20, 1.0f, 0, true );

		m_scale = b3Vec3_one;
		m_scale = { 2.0f, 2.0f, 2.0f };
		// m_scale = { -1.2f, 2.0f, 4.0f };
		// m_scale = {  8.0f, 8.0f, 8.0f };
		// m_scale = {  0.5f, 0.5f, 0.5f };
		// m_scale = { 0.25f, 0.25f, 0.25f };

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		m_gridShapeId = b3CreateMeshShape( groundId, &shapeDef, m_gridMesh, m_scale );

		m_shapeType = ShapeType::cylinder;
		m_bodyId = b3_nullBodyId;
		m_cylinderHull = b3CreateCylinder( 1.0f, 0.25f, 0.0f, 15 );
		// m_cylinderHull = b3CreateCylinder( 0.5f, 0.2f, 0.0f, 15 );

		Spawn();

		GetGuiDraw()->forceScale = 0.01f;
	}

	~GridMesh() override
	{
		b3DestroyHull( m_cylinderHull );
		b3DestroyMesh( m_gridMesh );
	}

	void Spawn()
	{
		if ( B3_IS_NON_NULL( m_bodyId ) )
		{
			b3DestroyBody( m_bodyId );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		// bodyDef.position = { 0.5f, 2.0f, -0.5f };
		bodyDef.position = { 0.1f, 1.0f, -0.1f };
		// bodyDef.enableContactRecycling = m_shapeType != ShapeType::cylinder;
		bodyDef.angularDamping = m_shapeType == ShapeType::cylinder ? 0.1f : 0.0f;

		// bodyDef.linearVelocity = { 2.0f, 0.0f, 0.0f };
		//  bodyDef.position = { 0.208701894, 0.0791450217, -0.298710823 };
		//  bodyDef.rotation = { { 0.0808477178, -0.00305216131, 0.000387475739 }, 0.996721804 };
		//  bodyDef.linearVelocity = { 7.28029263e-05, 0.000761006493, 0.0104003791 };
		//  bodyDef.angularVelocity = { 0.116714634, -0.00774557842, -0.000242586044 };
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		switch ( m_shapeType )
		{
			case ShapeType::sphere:
			{
				shapeDef.baseMaterial.rollingResistance = 0.05f;
				b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
				b3CreateSphereShape( m_bodyId, &shapeDef, &sphere );
			}
			break;

			case ShapeType::capsule:
			{
				shapeDef.baseMaterial.rollingResistance = 0.05f;
				b3Capsule capsule = { { 0.0f, 0.0f, 1.276f }, { 0.0f, 0.0f, 0.476f }, 0.15f };
				b3CreateCapsuleShape( m_bodyId, &shapeDef, &capsule );
			}
			break;

			case ShapeType::box:
			{
				b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
				// shapeDef.baseMaterial.friction = 0.01f;
				b3CreateHullShape( m_bodyId, &shapeDef, &box.base );
			}
			break;

			case ShapeType::cylinder:
			{
				shapeDef.baseMaterial.rollingResistance = 0.02f;
				b3CreateHullShape( m_bodyId, &shapeDef, m_cylinderHull );
			}
			break;

			default:
				assert( false );
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::RadioButton( "Sphere", m_shapeType == ShapeType::sphere ) )
		{
			m_shapeType = ShapeType::sphere;
			Spawn();
		}

		if ( ImGui::RadioButton( "Capsule", m_shapeType == ShapeType::capsule ) )
		{
			m_shapeType = ShapeType::capsule;
			Spawn();
		}

		if ( ImGui::RadioButton( "Box", m_shapeType == ShapeType::box ) )
		{
			m_shapeType = ShapeType::box;
			Spawn();
		}

		if ( ImGui::RadioButton( "Cylinder", m_shapeType == ShapeType::cylinder ) )
		{
			m_shapeType = ShapeType::cylinder;
			Spawn();
		}

		b3Vec3 scale = m_scale;
		bool changed = false;
		changed = changed || ImGui::SliderFloat( "Scale X", &scale.x, -2.0f, 2.0f, "%.1f" );
		changed = changed || ImGui::SliderFloat( "Scale Z", &scale.z, -2.0f, 2.0f, "%.1f" );

		if ( changed )
		{
			m_scale = scale;
			b3Shape_SetMesh( m_gridShapeId, m_gridMesh, m_scale );
		}

		return true;
	}

	void Render() override
	{
		Sample::Render();

		DrawTextLine( "triangle count = %d, bytes = %d", m_gridMesh->triangleCount, m_gridMesh->byteCount );
		b3Transform transform = { { 0.0f, 0.01f, 0.0f }, b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( transform ), 1.0f );
	}

	void Step() override
	{
		if ( m_stepCount == 83 )
		{
			// m_context->settings.pause = true;
		}

		if ( m_stepCount < 100 )
		{
			// b3Vec3 p = b3Body_GetPosition( m_bodyId );
			// printf( "%d : %g %g %g\n", m_stepCount, p.x, p.y, p.z );
			// b3Vec3 v = b3Body_GetLinearVelocity( m_bodyId );
			// printf( "%d : %g %g %g\n", m_stepCount, v.x, v.y, v.z );
		}

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new GridMesh( context );
	}

	b3HullData* m_cylinderHull;
	ShapeType m_shapeType;
	b3BodyId m_bodyId;
	b3MeshData* m_gridMesh;
	b3ShapeId m_gridShapeId;
	b3Vec3 m_scale;
};

static int sampleGridMesh = RegisterSample( "Mesh", "Grid", GridMesh::Create );

class BigBoxMesh : public Sample
{
public:
	explicit BigBoxMesh( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 6.0f, b3Pos_zero );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );
		m_boxMesh = b3CreateBoxMesh( {0.0f, -1.0f, 0.0f}, { 50.0f, 1.0f, 50.0f }, true );
		m_scale = b3Vec3_one;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.5f;
		// shapeDef.baseMaterial.restitution = 0.5f;
		// shapeDef.baseMaterial.rollingResistance = 0.5f;
		m_gridShapeId = b3CreateMeshShape( groundId, &shapeDef, m_boxMesh, m_scale );

		m_shapeType = ShapeType::cylinder;
		m_bodyId = b3_nullBodyId;
		//m_cylinderHull = b3CreateCylinder( 0.5f, 0.2f, 0.0f, 15 );
		m_cylinderHull = b3CreateCylinder( 0.3f, 0.15f, 0.0f, 32 );

		m_stepWhilePaused = false;

		Spawn();
	}

	~BigBoxMesh() override
	{
		b3DestroyHull( m_cylinderHull );
		b3DestroyMesh( m_boxMesh );
	}

	void Spawn()
	{
		if ( B3_IS_NON_NULL( m_bodyId ) )
		{
			b3DestroyBody( m_bodyId );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.5f, 0.0f, 0.0f };
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.rollingResistance = 0.05f;

		switch ( m_shapeType )
		{
			case ShapeType::sphere:
			{
				b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
				b3CreateSphereShape( m_bodyId, &shapeDef, &sphere );
			}
			break;

			case ShapeType::capsule:
			{
				b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
				// material.friction = 0.0f;
				// material.restitution = 1.0f;
				material.rollingResistance = 0.1f;
				shapeDef.materials = &material;
				shapeDef.materialCount = 1;
				b3Capsule capsule = { { 0.0f, 0.0f, 1.276f }, { 0.0f, 0.0f, 0.476f }, 0.15f };
				b3CreateCapsuleShape( m_bodyId, &shapeDef, &capsule );
			}
			break;

			case ShapeType::box:
			{
				b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
				// shapeDef.baseMaterial.friction = 0.01f;
				b3CreateHullShape( m_bodyId, &shapeDef, &box.base );
			}
			break;

			case ShapeType::cylinder:
			{
				b3CreateHullShape( m_bodyId, &shapeDef, m_cylinderHull );
			}
			break;

			default:
				assert( false );
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::RadioButton( "Sphere", m_shapeType == ShapeType::sphere ) )
		{
			m_shapeType = ShapeType::sphere;
			Spawn();
		}

		if ( ImGui::RadioButton( "Capsule", m_shapeType == ShapeType::capsule ) )
		{
			m_shapeType = ShapeType::capsule;
			Spawn();
		}

		if ( ImGui::RadioButton( "Box", m_shapeType == ShapeType::box ) )
		{
			m_shapeType = ShapeType::box;
			Spawn();
		}

		if ( ImGui::RadioButton( "Cylinder", m_shapeType == ShapeType::cylinder ) )
		{
			m_shapeType = ShapeType::cylinder;
			Spawn();
		}

		b3Vec3 scale = m_scale;
		bool changed = false;
		changed = changed || ImGui::SliderFloat( "Scale X", &scale.x, -2.0f, 2.0f, "%.1f" );
		changed = changed || ImGui::SliderFloat( "Scale Z", &scale.z, -2.0f, 2.0f, "%.1f" );

		if ( changed )
		{
			m_scale = scale;
			b3Shape_SetMesh( m_gridShapeId, m_boxMesh, m_scale );
		}

		return true;
	}

	void Render() override
	{
		Sample::Render();
		b3Transform transform = { { 0.0f, 0.01f, 0.0f }, b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( transform ), 1.0f );
	}

	void Step() override
	{
		if ( m_stepCount == 83 )
		{
			// m_context->settings.pause = true;
		}

		if ( m_stepCount < 100 )
		{
			// b3Vec3 p = b3Body_GetPosition( m_bodyId );
			// printf( "%d : %g %g %g\n", m_stepCount, p.x, p.y, p.z );
			// b3Vec3 v = b3Body_GetLinearVelocity( m_bodyId );
			// printf( "%d : %g %g %g\n", m_stepCount, v.x, v.y, v.z );
		}

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BigBoxMesh( context );
	}

	b3HullData* m_cylinderHull;
	ShapeType m_shapeType;
	b3BodyId m_bodyId;
	b3MeshData* m_boxMesh;
	b3ShapeId m_gridShapeId;
	b3Vec3 m_scale;
};

static int sampleBigBoxMesh = RegisterSample( "Mesh", "Big Box", BigBoxMesh::Create );

class BoxMesh : public Sample
{
public:
	explicit BoxMesh( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 6.0f, b3Pos_zero );
			
		}

		AddGroundBox( 20.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { 0.0f, -1.0f, 0.0f };
		bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.25f * B3_PI );
		b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		m_boxMesh = b3CreateBoxMesh( { 0.0f, 1.0f, 0.0f }, { 1.0f, 1.0f, 1.0f }, true );
		// m_scale = { -0.6f, 1.0f, 2.0f };
		m_scale = b3Vec3_one;
		m_boxShapeId = b3CreateMeshShape( groundId, &shapeDef, m_boxMesh, m_scale );

		m_cylinderHull = b3CreateCylinder( 1.0f, 0.75f, 0.0f, 8 );

		m_shapeType = ShapeType::box;
		m_bodyId = b3_nullBodyId;

		Spawn();
	}

	~BoxMesh() override
	{
		b3DestroyMesh( m_boxMesh );
		b3DestroyHull( m_cylinderHull );
	}

	void Spawn()
	{
		if ( B3_IS_NON_NULL( m_bodyId ) )
		{
			b3DestroyBody( m_bodyId );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 1.5f, 0.0f };
		// bodyDef.position = { -0.2f, 1.5f, -1.25f };

		if ( m_shapeType == ShapeType::cylinder )
		{
			bodyDef.position.y -= 0.5f;
		}

		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		switch ( m_shapeType )
		{
			case ShapeType::sphere:
			{
				b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
				b3CreateSphereShape( m_bodyId, &shapeDef, &sphere );
			}
			break;

			case ShapeType::capsule:
			{
				b3Capsule capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.1f };
				b3CreateCapsuleShape( m_bodyId, &shapeDef, &capsule );
			}
			break;

			case ShapeType::box:
			{
				b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
				// b3BoxHull box = b3MakeBoxHull( 0.1f, 1.0f, 0.1f );
				b3CreateHullShape( m_bodyId, &shapeDef, &box.base );
			}
			break;

			case ShapeType::cylinder:
			{
				b3CreateHullShape( m_bodyId, &shapeDef, m_cylinderHull );
			}
			break;

			default:
				assert( false );
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::RadioButton( "Sphere", m_shapeType == ShapeType::sphere ) )
		{
			m_shapeType = ShapeType::sphere;
			Spawn();
		}

		if ( ImGui::RadioButton( "Capsule", m_shapeType == ShapeType::capsule ) )
		{
			m_shapeType = ShapeType::capsule;
			Spawn();
		}

		if ( ImGui::RadioButton( "Box", m_shapeType == ShapeType::box ) )
		{
			m_shapeType = ShapeType::box;
			Spawn();
		}

		if ( ImGui::RadioButton( "Cylinder", m_shapeType == ShapeType::cylinder ) )
		{
			m_shapeType = ShapeType::cylinder;
			Spawn();
		}

		b3Vec3 scale = m_scale;
		bool changed = false;
		changed = changed || ImGui::SliderFloat( "Scale X", &scale.x, -2.0f, 2.0f, "%.1f" );
		changed = changed || ImGui::SliderFloat( "Scale Z", &scale.z, -2.0f, 2.0f, "%.1f" );

		if ( changed )
		{
			m_scale = scale;
			b3Shape_SetMesh( m_boxShapeId, m_boxMesh, m_scale );
		}

		return true;
	}

	void Step() override
	{
		if ( m_stepCount == 83 )
		{
			// m_context->settings.pause = true;
		}

		if ( m_stepCount < 100 )
		{
			// b3Vec3 p = b3Body_GetPosition( m_bodyId );
			// printf( "%d : %g %g %g\n", m_stepCount, p.x, p.y, p.z );
			// b3Vec3 v = b3Body_GetLinearVelocity( m_bodyId );
			// printf( "%d : %g %g %g\n", m_stepCount, v.x, v.y, v.z );
		}

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BoxMesh( context );
	}

	ShapeType m_shapeType;
	b3BodyId m_bodyId;
	b3MeshData* m_boxMesh;
	b3HullData* m_cylinderHull;
	b3ShapeId m_boxShapeId;
	b3Vec3 m_scale;
};

static int sampleBoxMesh = RegisterSample( "Mesh", "Box", BoxMesh::Create );

class MeshReflection : public Sample
{
public:
	enum
	{
		e_humanCount = 20
	};

	explicit MeshReflection( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 40.0f, b3Pos_zero );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		{
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			m_gridMesh = b3CreateGridMesh( 20, 20, 2.0f, 2, true );
			b3CreateMeshShape( body, &shapeDef, m_gridMesh, b3Vec3_one );
		}

		{
			m_buildingMesh = CreateMeshData( "data/meshes/building.obj", 1.0f, false, false, true, true );

			b3ShapeDef meshShapeDef = b3DefaultShapeDef();
			b3SurfaceMaterial materials[3] = {
				{
					.friction = 0.6f,
				},
				{
					.friction = 0.0f,
					.restitution = 0.95f,
					.userMaterialId = 1,
				},
				{
					.friction = 0.2f,
					.restitution = 0.2f,
					.userMaterialId = 2,
				},
			};
			meshShapeDef.materials = materials;
			meshShapeDef.materialCount = 3;

			bodyDef.position = { -10.0f, 0.0f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3CreateMeshShape( body, &meshShapeDef, m_buildingMesh, b3Vec3_one );

			m_scale = { -1.0f, 1.0f, 1.0f };
			bodyDef.position = { 10.0f, 0.0f, 0.0f };
			m_meshBodyId = b3CreateBody( m_worldId, &bodyDef );
			m_meshShapeId = b3CreateMeshShape( m_meshBodyId, &meshShapeDef, m_buildingMesh, m_scale );
		}

		bodyDef.type = b3_dynamicBody;
		{
			bodyDef.position = { 6.0f, 15.0f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3Sphere sphere = { b3Vec3_zero, 0.5f };
			shapeDef.baseMaterial.rollingResistance = 0.2f;
			shapeDef.baseMaterial.userMaterialId = 42;
			b3CreateSphereShape( body, &shapeDef, &sphere );
		}

		{
			bodyDef.position = { 9.0f, 15.0f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3Capsule capsule = { { -0.5f, 0.5f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.25f };
			shapeDef.baseMaterial.rollingResistance = 0.2f;
			shapeDef.baseMaterial.userMaterialId = 11;
			b3CreateCapsuleShape( body, &shapeDef, &capsule );
		}

		{
			bodyDef.position = { 12.0f, 15.0f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3BoxHull hull = b3MakeBoxHull( 0.25f, 0.5f, 0.75f );
			shapeDef.baseMaterial.userMaterialId = 555;
			b3CreateHullShape( body, &shapeDef, &hull.base );
		}

		float frictionTorque = 5.0f;
		float hertz = 1.0f;
		float dampingRatio = 0.7f;
		bool colorize = false;
		for ( int humanIndex = 0; humanIndex < e_humanCount; humanIndex += 1 )
		{
			m_humans[humanIndex] = {};

			int groupIndex = humanIndex;
			b3Pos position = { -14.0f + 1.5f * humanIndex, 8.0f, 0.0f };
			CreateHuman( &m_humans[humanIndex], m_worldId, position, frictionTorque, hertz, dampingRatio, groupIndex, nullptr,
						 colorize );
		}
	}

	~MeshReflection() override
	{
		b3DestroyMesh( m_gridMesh );
		b3DestroyMesh( m_buildingMesh );

		for ( int i = 0; i < e_humanCount; ++i )
		{
			DestroyHuman( m_humans + i );
		}
	}

	bool DrawControls() override
	{
		bool changed = false;
		b3Vec3 scale = m_scale;
		if ( ImGui::RadioButton( "Neg X", scale.x < 0.0f ) )
		{
			scale.x = -1.0f;
			changed = true;
		}

		ImGui::SameLine();

		if ( ImGui::RadioButton( "Pos X", scale.x > 0.0f ) )
		{
			scale.x = 1.0f;
			changed = true;
		}

		if ( ImGui::RadioButton( "Neg Y", scale.y < 0.0f ) )
		{
			scale.y = -1.0f;
			changed = true;
		}

		ImGui::SameLine();

		if ( ImGui::RadioButton( "Pos Y", scale.y > 0.0f ) )
		{
			scale.y = 1.0f;
			changed = true;
		}

		if ( ImGui::RadioButton( "Neg Z", scale.z < 0.0f ) )
		{
			scale.z = -1.0f;
			changed = true;
		}

		ImGui::SameLine();

		if ( ImGui::RadioButton( "Pos Z", scale.z > 0.0f ) )
		{
			scale.z = 1.0f;
			changed = true;
		}

		if ( changed )
		{
			m_scale = scale;

			b3Shape_SetMesh( m_meshShapeId, m_buildingMesh, m_scale );
		}

		return true;
	}

	void Render() override
	{
		DrawTextLine( "scale = (%.2g, %.2g, %.2g)", m_scale.x, m_scale.y, m_scale.z );
		DrawTextLine( "surface type = %d", m_userMaterialId );

		Sample::Render();
	}

	static Sample* Create( SampleContext* context )
	{
		return new MeshReflection( context );
	}

	b3Vec3 m_scale;
	b3MeshData* m_gridMesh;
	b3MeshData* m_buildingMesh;
	b3BodyId m_meshBodyId;
	b3ShapeId m_meshShapeId;
	Human m_humans[e_humanCount];
};

static int sampleMeshReflection = RegisterSample( "Mesh", "Reflection", MeshReflection::Create );

struct CastContext
{
	b3Pos point;
	b3Vec3 normal;
	float fraction;
	bool hit;
};

class HeightField : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new HeightField( context );
	}

	explicit HeightField( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 40.0f, b3Pos_zero );
		}

#if defined( NDEBUG )
		m_rowCount = 400;
		m_columnCount = 400;
#else
		m_rowCount = 10;
		m_columnCount = 10;
#endif

		m_groundId = {};
		m_bodyId1 = {};
		m_bodyId2 = {};
		m_amplitude = 0.75f;
		m_holes = false;

		// m_rayOrigin = { 1.4f, 2.0f, 1.5f };
		// m_rayTranslation = { -3.0f, -4.0f, 3.0f };
		// m_rayOrigin = { 30.0f, 2.0f, 0.0f };
		// m_rayTranslation = { -60.0f, -4.0f, 0.0f };

		m_rayOrigin = { 5.5f, 4.0f, 1.01f };
		m_rayTranslation = { 0.0f, -8.0f, 0.0f };
		m_radius = 0.2f;

		m_heightField = nullptr;

		CreateScene();
	}

	~HeightField() override
	{
		b3DestroyHeightField( m_heightField );
	}

	void CreateScene()
	{
		if ( B3_IS_NULL( m_groundId ) == false )
		{
			b3DestroyBody( m_groundId );
			m_groundId = {};
		}

		if ( B3_IS_NULL( m_bodyId1 ) == false )
		{
			b3DestroyBody( m_bodyId1 );
			m_bodyId1 = {};
		}

		if ( B3_IS_NULL( m_bodyId2 ) == false )
		{
			b3DestroyBody( m_bodyId2 );
			m_bodyId2 = {};
		}

		if ( m_heightField != nullptr )
		{
			b3DestroyHeightField( m_heightField );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		m_scale = { 2.0f, 2.0f * m_amplitude, 2.0f };

		if ( m_amplitude == 0.0f )
		{
			m_heightField = b3CreateGrid( m_rowCount, m_columnCount, m_scale, m_holes );
		}
		else
		{
			m_heightField = b3CreateWave( m_rowCount, m_columnCount, m_scale, 0.1f, 0.03333f, m_holes );
		}

		bodyDef.position = { -0.5f * m_heightField->scale.x * ( m_columnCount - 1 ), 0.0f,
							 -0.5f * m_heightField->scale.z * ( m_rowCount - 1 ) };
		// bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.25f * B3_PI );
		m_groundId = b3CreateBody( m_worldId, &bodyDef );
		b3CreateHeightFieldShape( m_groundId, &shapeDef, m_heightField );

#if 0
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = { 0.0f, 4.0f, 1.0f };
		m_bodyId1 = b3CreateBody( m_worldId, &bodyDef );
		b3Sphere sphere = { b3Vec3_zero, 0.5f };
		b3CreateSphereShape( m_bodyId1, &shapeDef, &sphere );

		bodyDef.position = { 0.0f, 0.5f, -0.5f };
		bodyDef.linearVelocity = { 5.0f, 0.0f, 0.0f };
		m_bodyId2 = b3CreateBody( m_worldId, &bodyDef );
		b3BoxHull box = b3MakeBoxHull( 0.3f, 0.3f, 0.3f );
		shapeDef.baseMaterial.friction = 0.05f;
		b3CreateHullShape( m_bodyId2, &shapeDef, &box.base );
#endif
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		if ( ImGui::SliderInt( "columns", &m_columnCount, 1, 500 ) )
		{
			CreateScene();
		}

		if ( ImGui::SliderInt( "rows", &m_rowCount, 1, 500 ) )
		{
			CreateScene();
		}

		if ( ImGui::SliderFloat( "amplitude", &m_amplitude, 0.0f, 2.0f ) )
		{
			CreateScene();
		}

		if ( ImGui::Checkbox( "holes", &m_holes ) )
		{
			CreateScene();
		}

		ImGui::SliderFloat( "ray x", &m_rayOrigin.x, -2.0f * m_radius - 0.5f * m_scale.x * ( m_columnCount - 1 ),
							2.0f * m_radius + 0.5f * m_scale.x * ( m_columnCount - 1 ) );
		ImGui::SliderFloat( "ray z", &m_rayOrigin.z, -2.0f * m_radius - 0.5f * m_scale.z * ( m_rowCount - 1 ),
							2.0f * m_radius + 0.5f * m_scale.z * ( m_rowCount - 1 ) );
		ImGui::SliderFloat( "delta x", &m_rayTranslation.x, -2.0f * m_scale.x * ( m_columnCount - 1 ),
							2.0f * m_scale.x * ( m_columnCount - 1 ) );
		ImGui::SliderFloat( "delta z", &m_rayTranslation.z, -2.0f * m_scale.z * ( m_rowCount - 1 ),
							2.0f * m_scale.z * ( m_rowCount - 1 ) );
		ImGui::SliderFloat( "radius", &m_radius, 0.0f, 1.0f );

		ImGui::PopItemWidth();
		return true;
	}

	void Render() override
	{
		Sample::Render();
		b3Transform transform = { { 0.0f, 0.1f, 0.0f }, b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( transform ), 0.5f );
	}

	// This callback finds the closest hit.
	static float CastCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t surfaceType,
							   int triangleIndex, int childIndex, void* context )
	{
		(void)shapeId;
		(void)surfaceType;
		(void)triangleIndex;
		(void)childIndex;

		CastContext* castContext = (CastContext*)context;

		castContext->point = point;
		castContext->normal = normal;
		castContext->fraction = fraction;
		castContext->hit = true;

		// By returning the current fraction, we instruct the calling code to clip the ray and
		// continue the ray-cast to the next shape. WARNING: do not assume that shapes
		// are reported in order. However, by clipping, we can always get the closest shape.
		return fraction;
	}

	void Step() override
	{
		Sample::Step();

		if ( m_radius == 0.0f )
		{
			// m_rayOrigin = { 0.0f, -FLT_EPSILON, 0.0f };
			// m_rayTranslation = { -1000.0f, 0.0f, 0.0 };

			b3Pos origin = b3ToPos( m_rayOrigin );
			b3RayResult result = b3World_CastRayClosest( m_worldId, origin, m_rayTranslation, b3DefaultQueryFilter() );

			DrawPoint( origin, 6.0f, MakeColor( b3_colorGreenYellow ) );
			DrawPoint( b3OffsetPos( origin, m_rayTranslation ), 6.0f, MakeColor( b3_colorRed ) );
			DrawLine( origin, b3OffsetPos( origin, m_rayTranslation ), MakeColor( b3_colorGray ) );

			if ( result.hit )
			{
				b3Pos point = result.point;
				DrawLine( point, b3OffsetPos( point, 0.5f * result.normal ), MakeColor( b3_colorGray ) );
				DrawPoint( point, 10.0f, MakeColor( b3_colorOrange ) );
			}
		}
		else
		{
			b3ShapeProxy proxy = {
				.points = &m_rayOrigin,
				.count = 1,
				.radius = m_radius,
			};

			CastContext result = {};
			result.fraction = 1.0f;

			b3World_CastShape( m_worldId, b3Pos_zero, &proxy, m_rayTranslation, b3DefaultQueryFilter(), CastCallback, &result );

			b3Pos origin = b3ToPos( m_rayOrigin );
			DrawPoint( origin, 2.0f, MakeColor( b3_colorGreen ) );
			DrawPoint( b3OffsetPos( origin, m_rayTranslation ), 2.0f, MakeColor( b3_colorRed ) );
			DrawLine( origin, b3OffsetPos( origin, m_rayTranslation ), MakeColor( b3_colorYellow ) );

			b3Sphere sphere = { b3Vec3_zero, m_radius };
			b3Transform sphereXf = { m_rayOrigin + result.fraction * m_rayTranslation, b3Quat_identity };
			DrawSolidSphere( b3MakeWorldTransform( sphereXf ), sphere, MakeColor( b3_colorOrange ) );

			if ( result.hit )
			{
				b3Pos point = result.point;
				DrawLine( point, b3OffsetPos( point, 0.5f * result.normal ), MakeColor( b3_colorGreen ) );
				DrawPoint( point, 6.0f, MakeColor( b3_colorPurple ) );
			}
		}
	}

	b3BodyId m_groundId;
	b3BodyId m_bodyId1;
	b3BodyId m_bodyId2;
	float m_amplitude;
	b3Vec3 m_scale;
	b3Vec3 m_rayOrigin;
	b3Vec3 m_rayTranslation;
	b3HeightFieldData* m_heightField;
	float m_radius;
	int m_rowCount;
	int m_columnCount;
	bool m_holes;
};

static int sampleHeightField = RegisterSample( "Mesh", "Height Field", HeightField::Create );

static float ComputeInternalSurfaceArea( const b3MeshData* data )
{
	const b3MeshNode* nodes = b3GetMeshNodes( data );
	int nodeCount = data->nodeCount;
	float area = 0.0f;
	for ( int i = 1; i < nodeCount; ++i )
	{
		const b3MeshNode* node = nodes + i;
		if ( node->data.asLeaf.type == 3 )
		{
			continue;
		}

		b3AABB box = {
			node->lowerBound,
			node->upperBound,
		};

		area += b3AABB_Area( box );
	}

	return area;
}

class MeshViewer : public Sample
{
public:
	explicit MeshViewer( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 50.0f, b3Pos_zero );
		}

		m_meshIndex = 0;
		m_mesh = nullptr;
		m_bodyId = b3_nullBodyId;
		m_medianSplit = true;
		m_concaveEdges = true;
		m_weldVertices = true;
		m_weldToleranceMillimeters = 1.5f;
		m_area = 0.0f;
		m_drawLevel = -1;
		m_height = 0;
		m_buildTime = 0.0f;
		m_launchSpeedScale = 1.0f;

		LoadMesh();
	}

	~MeshViewer() override
	{
		b3DestroyMesh( m_mesh );
	}

	void LoadMesh()
	{
		if ( B3_IS_NON_NULL( m_bodyId ) )
		{
			b3DestroyBody( m_bodyId );
			m_bodyId = b3_nullBodyId;
		}

		if ( m_mesh != nullptr )
		{
			b3DestroyMesh( m_mesh );
			m_mesh = nullptr;
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );
		float scale = 0.01f;

		const char* filesNames[m_meshCount] = {
			"voxel_mesh_01.obj",
			"voxel_mesh_02.obj",
			"voxel_mesh_03.obj",
			"voxel_mesh_04.obj",
		};

		char buffer[64] = {};
		snprintf( buffer, 64, "data/meshes/%s", filesNames[m_meshIndex] );

		LoadTempMesh( buffer, &m_tempMesh, scale, true );

		b3MeshDef def = {};
		def.vertices = m_tempMesh.vertices.data();
		def.vertexCount = (int)m_tempMesh.vertices.size();
		def.indices = m_tempMesh.indices.data();
		def.triangleCount = (int)m_tempMesh.indices.size() / 3;
		def.materialIndices = m_tempMesh.materialIndices.data();
		def.useMedianSplit = m_medianSplit;
		def.identifyEdges = m_concaveEdges;
		def.weldVertices = m_weldVertices;
		def.weldTolerance = 0.001f * m_weldToleranceMillimeters;

		uint64_t startTicks = b3GetTicks();
		m_mesh = b3CreateMesh( &def, m_degenerateTriangles, m_degenerateCapacity );
		m_buildTime = b3GetMilliseconds( startTicks );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateMeshShape( m_bodyId, &shapeDef, m_mesh, b3Vec3_one );

		m_area = ComputeInternalSurfaceArea( m_mesh );
		m_height = b3GetHeight( m_mesh );
		m_drawLevel = b3ClampInt( m_drawLevel, -1, m_height );
	}

	void DrawNodes()
	{
		if ( m_drawLevel < 0 || m_mesh == nullptr )
		{
			return;
		}

		int count = m_mesh->nodeCount;
		if ( count == 0 )
		{
			return;
		}

		constexpr int colorCount = 20;
		b3HexColor colors[colorCount] = {
			b3_colorAliceBlue, b3_colorAntiqueWhite,   b3_colorAqua,		   b3_colorAquamarine, b3_colorAzure,
			b3_colorBeige,	   b3_colorBisque,		   b3_colorBlanchedAlmond, b3_colorBlue,	   b3_colorBlueViolet,
			b3_colorBrown,	   b3_colorBurlywood,	   b3_colorCadetBlue,	   b3_colorChartreuse, b3_colorChocolate,
			b3_colorCoral,	   b3_colorCornflowerBlue, b3_colorCornsilk,	   b3_colorCrimson,	   b3_colorCyan,
		};

		const b3MeshNode* nodes = b3GetMeshNodes( m_mesh );
		const b3MeshNode* root = nodes + 0;

		// Queue to store nodes along with their levels
		std::queue<std::pair<const b3MeshNode*, int>> q;

		// Start with the root at level 0
		q.push( { root, 0 } );

		while ( !q.empty() )
		{
			std::pair<const b3MeshNode*, int> pair = q.front();
			q.pop();

			const b3MeshNode* node = pair.first;
			int level = pair.second;

			// If the current level matches the target level, add the node's data to the result
			if ( level == m_drawLevel )
			{
				b3AABB box = {
					node->lowerBound,
					node->upperBound,
				};
				DrawBounds( box, 0.0f, MakeColor( colors[level % colorCount] ) );

				int axis = node->data.asNode.axis;
				b3Vec3 center = b3AABB_Center( box );
				b3Pos centerPos = b3ToPos( center );
				if ( axis == 0 )
				{
					DrawArrow( centerPos, b3OffsetPos( centerPos, 0.1f * b3Vec3_axisX ), MakeColor( b3_colorRed ) );
				}
				else if ( axis == 1 )
				{
					DrawArrow( centerPos, b3OffsetPos( centerPos, 0.1f * b3Vec3_axisY ), MakeColor( b3_colorGreen ) );
				}
				else if ( axis == 2 )
				{
					DrawArrow( centerPos, b3OffsetPos( centerPos, 0.1f * b3Vec3_axisZ ), MakeColor( b3_colorBlue ) );
				}
				else
				{
					DrawSolidSphere( b3WorldTransform_identity, { center, 0.03f }, MakeColor( b3_colorOrange ) );
				}
			}

			// If the current level exceeds the target level, stop processing
			if ( level > m_drawLevel )
			{
				break;
			}

			bool isLeaf = node->data.asNode.axis == 3;
			if ( isLeaf == false )
			{
				// Enqueue the left and right children with their respective levels
				// I'm using some internal knowledge of how the node data is organized.
				q.push( { node + 1, level + 1 } );
				q.push( { node + node->data.asNode.childOffset, level + 1 } );
			}
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::SliderInt( "index", &m_meshIndex, 0, m_meshCount - 1 ) )
		{
			LoadMesh();
		}

		if ( ImGui::RadioButton( "median split", m_medianSplit ) )
		{
			m_medianSplit = true;
			LoadMesh();
		}

		if ( ImGui::RadioButton( "sah binning", m_medianSplit == false ) )
		{
			m_medianSplit = false;
			LoadMesh();
		}

		if ( ImGui::Checkbox( "concave edges", &m_concaveEdges ) )
		{
			LoadMesh();
		}

		if ( ImGui::Checkbox( "weld vertices", &m_weldVertices ) )
		{
			LoadMesh();
		}

		if ( ImGui::SliderFloat( "tolerance", &m_weldToleranceMillimeters, 0.0f, 10.0f, "%.1f" ) )
		{
			LoadMesh();
		}

		ImGui::SliderInt( "draw level", &m_drawLevel, -1, m_height );
		return true;
	}

	void Render() override
	{
		Sample::Render();
		DrawAxes( b3WorldTransform_identity, 1.0f );

		DrawTextLine( "triangle count = %d", m_mesh->triangleCount );
		DrawTextLine( "vertex count = %d", m_mesh->vertexCount );
		DrawTextLine( "degenerate count = %d", m_mesh->degenerateCount );
		DrawTextLine( "node area = %g", m_area );
		DrawTextLine( "height = %d", m_height );
		DrawTextLine( "build time (ms) = %g", m_buildTime );

		DrawNodes();

		b3Vec3 offset = { 0.01f, 0.01f, 0.01f };
		for ( int i = 0; i < m_mesh->degenerateCount; ++i )
		{
			int triangleIndex = m_degenerateTriangles[i];
			int i1 = m_tempMesh.indices[3 * triangleIndex + 0];
			int i2 = m_tempMesh.indices[3 * triangleIndex + 1];
			int i3 = m_tempMesh.indices[3 * triangleIndex + 2];
			b3Vec3 v1 = m_tempMesh.vertices[i1];
			b3Vec3 v2 = m_tempMesh.vertices[i2];
			b3Vec3 v3 = m_tempMesh.vertices[i3];

			float area = b3Length( b3Cross( v2 - v1, v3 - v1 ) );
			(void)area;

			b3Vec3 p = ( 1.0f / 3.0f ) * ( v1 + v2 + v3 );
			b3Pos pPos = b3ToPos( p );
			DrawPoint( pPos, 10.0f, MakeColor( b3_colorCyan ) );
			DrawString3D( b3OffsetPos( pPos, offset ), MakeColor( b3_colorOrange ), "%d", triangleIndex );

			{
				b3Pos p1 = b3ToPos( v1 );
				b3Pos p2 = b3ToPos( v2 );
				b3Pos p3 = b3ToPos( v3 );
				DrawPoint( p1, 10.0f, MakeColor( b3_colorRed ) );
				DrawPoint( p2, 10.0f, MakeColor( b3_colorGreen ) );
				DrawPoint( p3, 10.0f, MakeColor( b3_colorBlue ) );
				DrawString3D( b3OffsetPos( p1, offset ), MakeColor( b3_colorRed ), "%d", i1 );
				DrawString3D( b3OffsetPos( p2, offset ), MakeColor( b3_colorGreen ), "%d", i2 );
				DrawString3D( b3OffsetPos( p3, offset ), MakeColor( b3_colorBlue ), "%d", i3 );
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new MeshViewer( context );
	}

	static constexpr int m_meshCount = 4;
	static constexpr int m_degenerateCapacity = 64;
	TempMesh m_tempMesh;
	b3MeshData* m_mesh;
	b3BodyId m_bodyId;
	float m_area;
	float m_weldToleranceMillimeters;
	float m_buildTime;
	int m_meshIndex;
	int m_degenerateTriangles[m_degenerateCapacity];
	int m_drawLevel;
	int m_height;
	bool m_medianSplit;
	bool m_concaveEdges;
	bool m_weldVertices;
};

static int sampleMeshViewer = RegisterSample( "Mesh", "Viewer", MeshViewer::Create );

// Results 9/29/25
// base median split: 0.222 ms per mesh
// weld: 0.526
// weld + edge: 0.755
class MeshCreationBenchmark : public Sample
{
public:
	explicit MeshCreationBenchmark( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 40.0f, b3Pos_zero );
		}

		LoadTempMesh( "data/meshes/voxel_mesh_01.obj", m_tempMeshes + 0, 0.01f, true );
		LoadTempMesh( "data/meshes/voxel_mesh_02.obj", m_tempMeshes + 1, 0.01f, true );
		LoadTempMesh( "data/meshes/voxel_mesh_03.obj", m_tempMeshes + 2, 0.01f, true );
		LoadTempMesh( "data/meshes/voxel_mesh_04.obj", m_tempMeshes + 3, 0.01f, true );

		m_time = FLT_MAX;
	}

	void Step() override
	{
		bool computeArea = false;
		int iterations = m_isDebug ? 1 : 10;
		int meshCount = m_meshCount;
		int triangleCount = 0;
		float area = 0.0f;

		for ( int i = 0; i < iterations; ++i )
		{
			uint64_t startTicks = b3GetTicks();

			for ( int j = 0; j < meshCount; ++j )
			{
				TempMesh& mesh = m_tempMeshes[j];
				b3MeshDef def = {};
				def.vertices = mesh.vertices.data();
				def.vertexCount = (int)mesh.vertices.size();
				def.indices = mesh.indices.data();
				def.triangleCount = (int)mesh.indices.size() / 3;
				def.materialIndices = mesh.materialIndices.data();
				def.useMedianSplit = true;
				def.identifyEdges = false;
				def.weldVertices = true;
				def.weldTolerance = 0.0015f;

				b3MeshData* meshData = b3CreateMesh( &def, nullptr, 0 );
				triangleCount += i == 0 ? meshData->triangleCount : 0;

				if ( computeArea && i == 0 )
				{
					area += ComputeInternalSurfaceArea( meshData );
				}

				b3DestroyMesh( meshData );
			}

			float ms = b3GetMilliseconds( startTicks );
			m_time = b3MinFloat( m_time, ms );
		}

		DrawTextLine( "triangle count = %d, area = %g", triangleCount, area );
		DrawTextLine( "total time = %.4f ms", m_time );
		DrawTextLine( "time per mesh = %.4f ms", m_time / meshCount );
	}

	static Sample* Create( SampleContext* context )
	{
		return new MeshCreationBenchmark( context );
	}

	static constexpr int m_meshCount = 4;
	TempMesh m_tempMeshes[m_meshCount];
	float m_time;
};

static int sampleMeshCreationBenchmark = RegisterSample( "Mesh", "Creation Benchmark", MeshCreationBenchmark::Create );

class VoxelMesh : public Sample
{
public:
	explicit VoxelMesh( SampleContext* context )
		: Sample( context )
	{
		b3Pos origin = { 5000.0f, 3500.0f, -7000.0f };

		if ( context->restart == false )
		{
			m_camera->SetView( -115.0f, 5.0f, 5.0f, b3OffsetPos( origin, b3Vec3{ 0.0f, 10.0f, 0.0f } ) );
		}

		m_launchSpeedScale = 1.0f;

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "ground";
			bodyDef.position = origin;
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			float scale = 0.01f;
			TempMesh tempMesh;
			const char* fileName = "data/meshes/collision_mesh_01.obj";
			LoadTempMesh( fileName, &tempMesh, scale, true );

			b3MeshDef def = {};
			def.vertices = tempMesh.vertices.data();
			def.vertexCount = (int)tempMesh.vertices.size();
			def.indices = tempMesh.indices.data();
			def.triangleCount = (int)tempMesh.indices.size() / 3;
			def.materialIndices = tempMesh.materialIndices.data();
			def.useMedianSplit = true;

			// this has a big impact on stability because the faces are nearly coplanar
			def.identifyEdges = true;

			def.weldVertices = true;
			def.weldTolerance = 0.002f;

			m_meshData = b3CreateMesh( &def, nullptr, 0 );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateMeshShape( groundId, &shapeDef, m_meshData, b3Vec3_one );
		}

		{
			b3Vec3 points[16];
			points[0] = { -3.13548756, 3.81141949, 237.289047 };
			points[1] = { -16.2333279, -23.4977913, 235.486603 };
			points[2] = { -13.8834839, 6.20244455, 23.7760544 };
			points[3] = { 14.0794125, 4.63170528, 24.9530792 };
			points[4] = { 3.98322797, -16.4192238, 236.704071 };
			points[5] = { -23.3520412, -3.26714420, 236.071594 };
			points[6] = { 13.4517860, -6.94963741, 24.4085312 };
			points[7] = { -5.24953651, 13.9316301, 24.5058060 };
			points[8] = { -4.65071201, -24.1484108, 235.974121 };
			points[9] = { -14.5111103, -5.37889385, 23.2315063 };
			points[10] = { 6.33307076, 13.2810068, 24.9935150 };
			points[11] = { 4.81784487, -14.6788225, 23.6787796 };
			points[12] = { -14.7180958, 4.46204281, 236.801331 };
			points[13] = { -23.9796677, -14.8484812, 235.527039 };
			points[14] = { 4.61085415, -4.83788204, 237.248611 };
			points[15] = { -6.76476669, -14.0281992, 23.1910706 };

			for ( int i = 0; i < 16; ++i )
			{
				b3Vec3 p = points[i];
				points[i] = b3MulSV( 0.01f, b3Vec3{ p.y, p.z, p.x } );
			}

			b3HullData* hull = b3CreateHull( points, 16, 16 );
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "cylinder";
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 5020.27734f, 3506.22559f, -6986.48584f };
			bodyDef.rotation = { { 0.664546967, 0.669287264, 0.135021493 }, 0.303646326 };

			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.baseMaterial.rollingResistance = 0.1f;
			b3CreateHullShape( bodyId, &shapeDef, hull );
			b3DestroyHull( hull );
		}

		// float jointFrictionTorque = 5.0f;
		// float jointHertz = 1.0f;
		// float jointDampingRatio = 0.7f;

		m_human = {};
		// CreateHuman( &m_human, m_worldId, b3Vec3{ 15.0f, 10.0f, 15.0f } + origin, jointFrictionTorque, jointHertz,
		// jointDampingRatio, 1, nullptr, 			 false );
	}

	~VoxelMesh() override
	{
		b3DestroyMesh( m_meshData );
	}

	void Step() override
	{
		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new VoxelMesh( context );
	}

	b3MeshData* m_meshData;
	Human m_human;
};

static int sampleVoxelMesh = RegisterSample( "Mesh", "Voxel", VoxelMesh::Create );

// Pause this to check contact manifolds for axis aligned collisions.
class HollowBox : public Sample
{
public:
	explicit HollowBox( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 30.0f, b3Pos_zero );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			m_mesh = b3CreateHollowBoxMesh( { 0.0f, 0.0f, 0.0f }, { 10.0f, 10.0f, 10.0f } );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateMeshShape( groundId, &shapeDef, m_mesh, b3Vec3_one );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.gravityScale = 0.0f;
		bodyDef.enableSleep = false;
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		{
			b3HullData* cylinderHull = b3CreateCylinder( 1.0f, 0.25f, 0.0f, 8 );

			b3Pos positions[6] = {
				{ 0.0f, -10.2f, 0.0f }, { 0.0f, 9.2f, 0.0f }, { -9.8f, 0.0f, 0.0f },
				{ 9.8f, 0.0f, 0.0f }, { 0.0f, 0.0f, -9.8f }, { 0.0f, 0.0f, 9.8f },
			};

			for (int i = 0; i < 6; ++i)
			{
				bodyDef.position = positions[i];
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
				b3CreateHullShape( bodyId, &shapeDef, cylinderHull );
			}

			b3DestroyHull( cylinderHull );
		}

		{
			b3Capsule capsule ={{0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 0.25f};
			b3Pos positions[8] = {
				{ 0.0f, -10.2f, 2.0f }, { 0.0f, 9.2f, 2.0f }, 
				{ 0.0f, -9.9f, 4.0f }, { 0.0f, 8.9f, 4.0f }, 
				{ -9.8f, 2.0f, 0.0f },
				{ 9.8f, 2.0f, 0.0f }, { 0.0f, 2.0f, -9.8f }, { 0.0f, 2.0f, 9.8f },
			};

			for (int i = 0; i < 8; ++i)
			{
				bodyDef.position = positions[i];
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
				b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			}
		}
	}

	~HollowBox() override
	{
		b3DestroyMesh( m_mesh );
	}

	void Render() override
	{
		Sample::Render();

		b3Transform transform = { { 0.0f, 0.01f, 0.0f }, b3Quat_identity };
		DrawAxes( b3MakeWorldTransform( transform ), 1.0f );
	}

	void Step() override
	{
		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new HollowBox( context );
	}

	b3MeshData* m_mesh;
};

static int sampleHollowBox = RegisterSample( "Mesh", "Hollow Box", HollowBox::Create );
