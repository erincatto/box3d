// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "ghost_collisions.h"
#include "human.h"
#include "mesh_loader.h"
#include "sample.h"
#include "utils.h"
#include "gfx/draw.h"

#include "gfx/debug_adapter.h"

#include "box3d/box3d.h"

#include <algorithm>
#include <float.h>
#include <imgui.h>
#include <queue>
#include <stdio.h>
#include <string.h>
#include <vector>

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

			m_scale = { -1.0f, 1.0f, 1.0f };
			bodyDef.position = { 10.0f, 0.0f, 0.0f };
			m_meshBodyId = b3CreateBody( m_worldId, &bodyDef );

			m_meshShapeId = b3_nullShapeId;
			if ( m_buildingMesh != nullptr )
			{
				b3CreateMeshShape( body, &meshShapeDef, m_buildingMesh, b3Vec3_one );
				m_meshShapeId = b3CreateMeshShape( m_meshBodyId, &meshShapeDef, m_buildingMesh, m_scale );
			}
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
		DestroyMeshData( m_buildingMesh );

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

			if ( m_buildingMesh != nullptr )
			{
				b3Shape_SetMesh( m_meshShapeId, m_buildingMesh, m_scale );
			}
		}

		return true;
	}

	void Render() override
	{
		DrawTextLine( "scale = (%.2g, %.2g, %.2g)", m_scale.x, m_scale.y, m_scale.z );
		DrawTextLine( "surface type = %d", (int)m_userMaterialId );

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
		DestroyMeshData( m_mesh );
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

		if ( LoadTempMesh( buffer, &m_tempMesh, scale, true ) == false )
		{
			m_buildTime = 0.0f;
			m_area = 0.0f;
			m_height = 0;
			m_drawLevel = -1;
			return;
		}

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

		if ( m_mesh == nullptr )
		{
			m_area = 0.0f;
			m_height = 0;
			m_drawLevel = -1;
			return;
		}

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

		if ( m_mesh == nullptr )
		{
			DrawTextLine( "mesh failed to load" );
			return;
		}

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
				if ( meshData == nullptr )
				{
					continue;
				}

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

			if ( m_meshData != nullptr )
			{
				b3ShapeDef shapeDef = b3DefaultShapeDef();
				b3CreateMeshShape( groundId, &shapeDef, m_meshData, b3Vec3_one );
			}
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
		DestroyMeshData( m_meshData );
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

// Ghost collision scenes. The scenes, launch and ghost test live in shared/ghost_collisions.c so the
// headless ghost scan measures the same thing.
class GhostCollisions : public Sample
{
public:
	static constexpr int m_markerCapacity = 256;

	explicit GhostCollisions( SampleContext* context )
		: Sample(context), m_markers{}
	{
		if (m_context->restart == false)
		{
			m_camera->SetView(90.0f, 20.0f, 30.0f, {0.0f, 1.0f, 0.0f});
		}

		m_data = CreateGhostCollisions(m_worldId, GetDefaultGhostSettings());
	}

	~GhostCollisions() override
	{
		DestroyGhostCollisions( &m_data );
	}

	void Step() override
	{
		Sample::Step();

		if ( m_didStep )
		{
			GhostReport reports[16];
			int ghostCount = UpdateGhostCollisions( m_worldId, &m_data, reports, IM_ARRAYSIZE( reports ) );
			ghostCount = b3MinInt( ghostCount, IM_ARRAYSIZE( reports ) );
			for ( int i = 0; i < ghostCount; ++i )
			{
				m_markers[m_markerIndex] = { reports[i].point, reports[i].normal };
				m_markerIndex = ( m_markerIndex + 1 ) % m_markerCapacity;
				m_markerCount = b3MinInt( m_markerCount + 1, m_markerCapacity );
			}
		}

		for ( int i = 0; i < m_markerCount; ++i )
		{
			b3Pos p = m_markers[i].point;
			DrawPoint( p, 8.0f, MakeColor( b3_colorRed ) );
			DrawLine( p, p + 0.5f * m_markers[i].normal, MakeColor( b3_colorRed ) );
		}

		b3Vec3 velocity = b3Body_GetLinearVelocity( m_data.bodyId );
		DrawTextLine( "ghost manifolds: %d this run, %d total over %d runs", m_data.runGhostCount, m_data.ghostCount,
					  m_data.runCount + 1 );
		DrawTextLine( "speed: %.2f m/s, vertical velocity: %.2f m/s", b3Length( velocity ), velocity.y );
	}

	bool DrawControls() override
	{
		GhostSettings& settings = m_data.settings;
		bool rebuild = false;
		bool relaunch = false;

		int scene = (int)settings.scene;
		if ( ImGui::Combo( "Scene", &scene, g_ghostSceneNames, e_ghostSceneCount ) )
		{
			settings.scene = (GhostScene)scene;
			rebuild = true;

			// The chamfer ghost needs the s&box character, a fixed rotation hull without speculative contact
			if ( settings.scene == e_ghostChamfer )
			{
				settings.fixedRotation = true;
				settings.speculative = false;
				if ( settings.body != e_ghostBox && settings.body != e_ghostCylinder )
				{
					settings.body = e_ghostBox;
				}
				relaunch = true;
			}

			if ( settings.scene == e_ghostInternal )
			{
				m_camera->SetView( 60.0f, 20.0f, 160.0f, { 0.0f, 30.0f, 0.0f } );
			}
			else
			{
				m_camera->SetView( 90.0f, 20.0f, 30.0f, { 0.0f, 1.0f, 0.0f } );
			}
		}

		if ( settings.scene == e_ghostInternal )
		{
			rebuild |= ImGui::Combo( "Preset", &settings.peelPreset, g_ghostPeelPresetNames, GHOST_PEEL_PRESET_COUNT );
		}

		int body = (int)settings.body;
		if ( ImGui::Combo( "Body", &body, g_ghostBodyNames, e_ghostBodyCount ) )
		{
			settings.body = (GhostBody)body;
			relaunch = true;
		}

		rebuild |= ImGui::Checkbox( "Mirror", &settings.mirror );
		rebuild |= ImGui::Checkbox( "Height Field", &settings.heightField );
		rebuild |= ImGui::SliderFloat( "Cell Size", &settings.cellSize, 0.1f, 1.0f, "%.2f m" );
		relaunch |= ImGui::SliderFloat( "Speed", &settings.speed, 0.0f, 15.0f, "%.1f m/s" );
		relaunch |= ImGui::SliderFloat( "Yaw", &settings.yaw, 0.0f, 45.0f, "%.0f deg" );
		relaunch |= ImGui::SliderFloat( "Friction", &settings.friction, 0.0f, 1.0f, "%.2f" );
		relaunch |= ImGui::Checkbox( "Fixed Rotation", &settings.fixedRotation );
		relaunch |= ImGui::Checkbox( "Speculative", &settings.speculative );
		relaunch |= ImGui::Button( "Launch" );

		if ( ImGui::Button( "Reset Counters" ) )
		{
			m_data.ghostCount = 0;
			m_data.runGhostCount = 0;
			m_data.runCount = 0;
			m_markerCount = 0;
			m_markerIndex = 0;
		}

		if ( rebuild )
		{
			CreateGhostGround( m_worldId, &m_data );
			m_markerCount = 0;
			m_markerIndex = 0;
		}

		if ( rebuild || relaunch )
		{
			LaunchGhostBody( m_worldId, &m_data );
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new GhostCollisions( context );
	}

	struct Marker
	{
		b3Pos point;
		b3Vec3 normal;
	};

	GhostCollisionsData m_data;
	Marker m_markers[m_markerCapacity];
	int m_markerCount = 0;
	int m_markerIndex = 0;
};

static int sampleGhostCollisions = RegisterSample( "Mesh", "Ghost Collisions", GhostCollisions::Create );

// A closed conveyor circuit swept from a channel profile and finely tessellated, so every body slides over
// thousands of internal edges. The loop runs over a washboard, up a helix, through a chicane and down a
// sharp ramp.
// A pushing manifold is counted as a ghost when its normal matches no nearby triangle and does not lie
// on the arc between two triangles that meet at a convex crease.
class ConveyorCircuit : public Sample
{
public:
	enum BodyType
	{
		e_mixedBody,
		e_boxBody,
		e_capsuleBody,
		e_sphereBody,
		e_cylinderBody,
		e_bodyTypeCount
	};

	static constexpr int m_maxBodies = 300;
	static constexpr int m_markerCapacity = 256;

	// Material 0 is the rails and skirts, the rest are belt directions
	static constexpr int m_yawBinCount = 128;
	static constexpr int m_materialCount = m_yawBinCount + 1;

	static constexpr int m_segmentCount = 8;
	static constexpr int m_profileCount = 8;

	static constexpr float m_radius = 5.0f;
	static constexpr float m_straightLength = 16.0f;
	static constexpr float m_chicaneRadius = 3.0f;
	static constexpr float m_chicaneAngle = 25.0f * B3_DEG_TO_RAD;
	static constexpr float m_halfWidth = 1.0f;
	static constexpr float m_railHeight = 0.35f;
	static constexpr float m_railThickness = 0.08f;
	static constexpr float m_skirtDepth = 0.4f;
	static constexpr float m_baseHeight = 1.0f;
	static constexpr float m_liftHeight = 1.5f;
	static constexpr float m_bumpStart = 5.0f;
	static constexpr float m_bumpEnd = 11.0f;
	static constexpr float m_bumpLength = 1.0f;
	static constexpr float m_bumpHeight = 0.06f;
	static constexpr float m_rampLength = 6.0f;
	static constexpr float m_spawnDistance = 1.0f;

	struct Segment
	{
		float length;
		float curvature;
	};

	enum GhostClass
	{
		e_faceGhost,
		e_edgeGhost,
		e_vertexGhost,
		e_otherGhost,
		e_ghostClassCount
	};

	struct Marker
	{
		b3Pos point;
		b3Vec3 normal;
	};

	explicit ConveyorCircuit( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 35.0f, 40.0f, 38.0f, { 0.0f, 1.5f, 0.0f } );
		}

		AddGroundBox( 30.0f );

		float chicaneArc = m_chicaneRadius * m_chicaneAngle;
		float chicaneAdvance = 4.0f * m_chicaneRadius * sinf( m_chicaneAngle );
		float lead = 2.0f;

		// The chicane returns to the original heading and line, so the loop still closes
		m_segments[0] = { m_straightLength, 0.0f };
		m_segments[1] = { B3_PI * m_radius, 1.0f / m_radius };
		m_segments[2] = { lead, 0.0f };
		m_segments[3] = { chicaneArc, 1.0f / m_chicaneRadius };
		m_segments[4] = { 2.0f * chicaneArc, -1.0f / m_chicaneRadius };
		m_segments[5] = { chicaneArc, 1.0f / m_chicaneRadius };
		m_segments[6] = { m_straightLength - lead - chicaneAdvance, 0.0f };
		m_segments[7] = { B3_PI * m_radius, 1.0f / m_radius };

		m_totalLength = 0.0f;
		for ( int i = 0; i < m_segmentCount; ++i )
		{
			m_segmentStart[i] = m_totalLength;
			m_totalLength += m_segments[i].length;
		}

		// Channel cross section in lateral and up coordinates, traversed with air on the left
		m_profile[0] = { -m_halfWidth - m_railThickness, -m_skirtDepth };
		m_profile[1] = { -m_halfWidth - m_railThickness, m_railHeight };
		m_profile[2] = { -m_halfWidth, m_railHeight };
		m_profile[3] = { -m_halfWidth, 0.0f };
		m_profile[4] = { m_halfWidth, 0.0f };
		m_profile[5] = { m_halfWidth, m_railHeight };
		m_profile[6] = { m_halfWidth + m_railThickness, m_railHeight };
		m_profile[7] = { m_halfWidth + m_railThickness, -m_skirtDepth };

		m_cylinderHull = b3CreateCylinder( 0.4f, 0.25f, -0.2f, 16 );

		m_cellSize = 0.25f;
		m_beltSpeed = 2.5f;
		m_targetCount = 80;
		m_spawnInterval = 15;
		m_bodyType = e_mixedBody;
		m_toleranceDegrees = 10.0f;
		m_contactRecycling = true;
		m_drawFlow = true;
		m_drawGhosts = true;

		m_groundId = b3_nullBodyId;
		m_bodyCount = 0;
		m_spawnIndex = 0;
		m_spawnTimer = 0;

		ResetCounters();
		CreateCircuit();
	}

	~ConveyorCircuit() override
	{
		DestroyCircuit();
		b3DestroyHull( m_cylinderHull );
	}

	float EvaluateHeight( float s ) const
	{
		float helix0 = m_segmentStart[1];
		float helix1 = m_segmentStart[2];
		float ramp0 = m_segmentStart[6] + 1.0f;
		float ramp1 = ramp0 + m_rampLength;

		if ( s < helix0 )
		{
			if ( m_bumpStart <= s && s <= m_bumpEnd )
			{
				float phase = 2.0f * B3_PI * ( s - m_bumpStart ) / m_bumpLength;
				return 0.5f * m_bumpHeight * ( 1.0f - cosf( phase ) );
			}

			return 0.0f;
		}

		if ( s < helix1 )
		{
			float u = ( s - helix0 ) / ( helix1 - helix0 );
			return m_liftHeight * u * u * ( 3.0f - 2.0f * u );
		}

		if ( s < ramp0 )
		{
			return m_liftHeight;
		}

		// Linear with sharp creases at both ends
		if ( s < ramp1 )
		{
			return m_liftHeight * ( ramp1 - s ) / m_rampLength;
		}

		return 0.0f;
	}

	b3Vec3 EvaluateCenter( float s ) const
	{
		float x = -0.5f * m_straightLength;
		float z = -m_radius;
		float yaw = 0.0f;
		float remaining = s;

		for ( int i = 0; i < m_segmentCount && remaining > 0.0f; ++i )
		{
			float ds = b3MinFloat( remaining, m_segments[i].length );
			float k = m_segments[i].curvature;
			if ( k == 0.0f )
			{
				x += ds * cosf( yaw );
				z += ds * sinf( yaw );
			}
			else
			{
				float yaw1 = yaw + k * ds;
				x += ( sinf( yaw1 ) - sinf( yaw ) ) / k;
				z -= ( cosf( yaw1 ) - cosf( yaw ) ) / k;
				yaw = yaw1;
			}

			remaining -= ds;
		}

		return { x, m_baseHeight + EvaluateHeight( s ), z };
	}

	void BuildStations()
	{
		int count = b3MaxInt( 32, (int)ceilf( m_totalLength / m_cellSize ) );
		m_stations.resize( count );
		m_tangents.resize( count );
		m_laterals.resize( count );
		m_ups.resize( count );

		for ( int k = 0; k < count; ++k )
		{
			m_stations[k] = EvaluateCenter( m_totalLength * (float)k / (float)count );
		}

		// The chord across each station bisects a crease, so the rails stay square to the belt
		for ( int k = 0; k < count; ++k )
		{
			b3Vec3 next = m_stations[( k + 1 ) % count];
			b3Vec3 prev = m_stations[( k + count - 1 ) % count];
			b3Vec3 t = b3Normalize( next - prev );
			b3Vec3 l = b3Normalize( b3Cross( t, b3Vec3_axisY ) );
			m_tangents[k] = t;
			m_laterals[k] = l;
			m_ups[k] = b3Cross( l, t );
		}
	}

	b3SurfaceMaterial MakeMaterial( int index ) const
	{
		b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
		if ( index == 0 )
		{
			material.friction = 0.2f;
			material.customColor = b3_colorGoldenRod;
			return material;
		}

		// Horizontal is enough, the solver projects tangent velocity onto the contact plane
		float yaw = -B3_PI + 2.0f * B3_PI * (float)( index - 1 ) / (float)m_yawBinCount;
		material.friction = 0.8f;
		material.tangentVelocity = { m_beltSpeed * cosf( yaw ), 0.0f, m_beltSpeed * sinf( yaw ) };
		material.customColor = b3_colorSlateGray;
		return material;
	}

	void CreateCircuit()
	{
		BuildStations();

		// The last row repeats the first station and welds to it, closing the loop
		int stationCount = (int)m_stations.size();

		std::vector<b3Vec3> vertices;
		std::vector<int32_t> indices;
		std::vector<uint8_t> materialIndices;

		for ( int j = 0; j + 1 < m_profileCount; ++j )
		{
			b3Vec2 pa = m_profile[j];
			b3Vec2 delta = { m_profile[j + 1].x - pa.x, m_profile[j + 1].y - pa.y };
			float length = sqrtf( delta.x * delta.x + delta.y * delta.y );
			int columnCount = b3MaxInt( 1, (int)ceilf( length / m_cellSize - 0.01f ) );
			bool floor = j == 3;

			int base = (int)vertices.size();
			int stride = columnCount + 1;
			for ( int k = 0; k <= stationCount; ++k )
			{
				int station = k % stationCount;
				for ( int i = 0; i <= columnCount; ++i )
				{
					float alpha = (float)i / (float)columnCount;
					float u = pa.x + alpha * delta.x;
					float v = pa.y + alpha * delta.y;
					vertices.push_back( m_stations[station] + u * m_laterals[station] + v * m_ups[station] );
				}
			}

			for ( int k = 0; k < stationCount; ++k )
			{
				int station = k;
				b3Vec3 expected = -delta.y * m_laterals[station] + delta.x * m_ups[station];

				int material = 0;
				if ( floor )
				{
					b3Vec3 direction = m_stations[( k + 1 ) % stationCount] - m_stations[station];
					float yaw = atan2f( direction.z, direction.x );
					int bin = (int)floorf( ( yaw + B3_PI ) / ( 2.0f * B3_PI ) * m_yawBinCount + 0.5f );
					material = 1 + bin % m_yawBinCount;
				}

				for ( int i = 0; i < columnCount; ++i )
				{
					int a = base + k * stride + i;
					int b = a + 1;
					int c = a + stride + 1;
					int d = a + stride;

					int corners[6] = { a, b, c, c, d, a };
					if ( ( k + i ) & 1 )
					{
						int flipped[6] = { a, b, d, b, c, d };
						memcpy( corners, flipped, sizeof( corners ) );
					}

					for ( int n = 0; n < 6; n += 3 )
					{
						b3Vec3 v1 = vertices[corners[n]];
						b3Vec3 v2 = vertices[corners[n + 1]];
						b3Vec3 v3 = vertices[corners[n + 2]];
						bool reversed = b3Dot( b3Cross( v2 - v1, v3 - v1 ), expected ) < 0.0f;

						indices.push_back( corners[n] );
						indices.push_back( reversed ? corners[n + 2] : corners[n + 1] );
						indices.push_back( reversed ? corners[n + 1] : corners[n + 2] );
						materialIndices.push_back( (uint8_t)material );
					}
				}
			}
		}

		b3MeshDef meshDef = {};
		meshDef.vertices = vertices.data();
		meshDef.indices = indices.data();
		meshDef.materialIndices = materialIndices.data();
		meshDef.vertexCount = (int)vertices.size();
		meshDef.triangleCount = (int)( indices.size() / 3 );
		meshDef.weldVertices = true;
		meshDef.weldTolerance = 0.001f;
		meshDef.identifyEdges = true;

		m_mesh = b3CreateMesh( &meshDef, nullptr, 0 );

		b3SurfaceMaterial materials[m_materialCount];
		for ( int i = 0; i < m_materialCount; ++i )
		{
			materials[i] = MakeMaterial( i );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.name = "circuit";
		m_groundId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.materials = materials;
		shapeDef.materialCount = m_materialCount;
		m_shapeId = b3CreateMeshShape( m_groundId, &shapeDef, m_mesh, b3Vec3_one );

		BuildAdjacency();

		m_markerCount = 0;
		m_markerIndex = 0;
	}

	// The body holding the mesh shapes must go before the mesh data
	void DestroyCircuit()
	{
		if ( B3_IS_NON_NULL( m_groundId ) )
		{
			b3DestroyBody( m_groundId );
			m_groundId = b3_nullBodyId;
		}

		if ( m_mesh != nullptr )
		{
			b3DestroyMesh( m_mesh );
			m_mesh = nullptr;
		}
	}

	void UpdateBeltSpeed()
	{
		for ( int i = 1; i < m_materialCount; ++i )
		{
			b3Shape_SetMeshMaterial( m_shapeId, MakeMaterial( i ), i );
		}
	}

	b3Pos GetSpawnPosition() const
	{
		b3Vec3 center = EvaluateCenter( m_spawnDistance );
		float lateral = RandomFloatRange( -0.6f, 0.6f );
		return b3ToPos( { center.x, center.y + 1.2f, center.z + lateral } );
	}

	void SpawnBody()
	{
		int type = m_bodyType == e_mixedBody ? 1 + m_spawnIndex % ( e_bodyTypeCount - 1 ) : m_bodyType;
		m_spawnIndex += 1;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = GetSpawnPosition();
		bodyDef.rotation = RandomQuat();

		// A jammed body would fall asleep on a moving belt
		bodyDef.enableSleep = false;
		bodyDef.enableContactRecycling = m_contactRecycling;
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.6f;

		// Round bodies would roll back down the helix without it
		shapeDef.baseMaterial.rollingResistance = 0.3f;

		switch ( type )
		{
			case e_boxBody:
			{
				float halfLength = ( m_spawnIndex & 1 ) ? 0.4f : 0.25f;
				b3BoxHull box = b3MakeBoxHull( halfLength, 0.15f, 0.25f );
				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}
			break;

			case e_capsuleBody:
			{
				b3Capsule capsule = { { -0.3f, 0.0f, 0.0f }, { 0.3f, 0.0f, 0.0f }, 0.15f };
				b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			}
			break;

			case e_sphereBody:
			{
				b3Sphere sphere = { b3Vec3_zero, 0.25f };
				b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			}
			break;

			default:
				b3CreateHullShape( bodyId, &shapeDef, m_cylinderHull );
				break;
		}

		m_bodyIds[m_bodyCount++] = bodyId;
	}

	void DestroyBodies()
	{
		for ( int i = 0; i < m_bodyCount; ++i )
		{
			b3DestroyBody( m_bodyIds[i] );
		}

		m_bodyCount = 0;
		m_spawnIndex = 0;
		m_spawnTimer = 0;
	}

	void ResetCounters()
	{
		memset( m_classCounts, 0, sizeof( m_classCounts ) );
		memset( m_flagCounts, 0, sizeof( m_flagCounts ) );
		m_ghostCount = 0;
		m_manifoldCount = 0;
		m_stepGhostCount = 0;
		m_recycleCount = 0;
		m_markerCount = 0;
		m_markerIndex = 0;
	}

	void BuildAdjacency()
	{
		int triangleCount = m_mesh->triangleCount;
		int vertexCount = m_mesh->vertexCount;
		const b3MeshTriangle* triangles = b3GetMeshTriangles( m_mesh );
		const b3Vec3* vertices = b3GetMeshVertices( m_mesh );

		m_triangleNormals.resize( triangleCount );
		m_edgeNeighbors.assign( 3 * triangleCount, B3_NULL_INDEX );
		m_vertexRings.assign( vertexCount, {} );
		m_vertexTriangles.assign( vertexCount, {} );

		// Sorting by vertex pair puts the two sides of a shared edge next to each other
		std::vector<std::pair<uint64_t, int>> edges;
		edges.reserve( 3 * triangleCount );

		for ( int t = 0; t < triangleCount; ++t )
		{
			int v[3] = { triangles[t].index1, triangles[t].index2, triangles[t].index3 };
			m_triangleNormals[t] = b3Normalize( b3Cross( vertices[v[1]] - vertices[v[0]], vertices[v[2]] - vertices[v[0]] ) );

			for ( int e = 0; e < 3; ++e )
			{
				int a = v[e];
				int b = v[e < 2 ? e + 1 : 0];
				uint64_t key = ( (uint64_t)b3MinInt( a, b ) << 32 ) | (uint64_t)b3MaxInt( a, b );
				edges.push_back( { key, 3 * t + e } );
				m_vertexRings[a].push_back( b );
				m_vertexRings[b].push_back( a );
				m_vertexTriangles[a].push_back( t );
			}
		}

		std::sort( edges.begin(), edges.end() );
		for ( size_t i = 0; i + 1 < edges.size(); ++i )
		{
			// A non-manifold edge leaves every side without a neighbor
			uint64_t key = edges[i].first;
			bool pair = edges[i + 1].first == key;
			bool single = ( i + 2 >= edges.size() || edges[i + 2].first != key ) && ( i == 0 || edges[i - 1].first != key );
			if ( pair && single )
			{
				m_edgeNeighbors[edges[i].second] = edges[i + 1].second / 3;
				m_edgeNeighbors[edges[i + 1].second] = edges[i].second / 3;
			}
		}

		for ( std::vector<int>& ring : m_vertexRings )
		{
			std::sort( ring.begin(), ring.end() );
			ring.erase( std::unique( ring.begin(), ring.end() ), ring.end() );
		}
	}

	bool MatchesFace( b3Vec3 normal, int triangleIndex ) const
	{
		return b3Dot( normal, m_triangleNormals[triangleIndex] ) >= m_cosTolerance;
	}

	// Direction in the face plane pointing away from the edge into the face
	static b3Vec3 GetInwardDirection( b3Vec3 edgeStart, b3Vec3 axis, b3Vec3 opposite )
	{
		b3Vec3 d = opposite - edgeStart;
		return b3Normalize( d - b3Dot( d, axis ) * axis );
	}

	// The normal cone of a convex edge is the arc between its two face normals. Neither face may rise
	// above the edge along the normal, so a concave edge only admits its face normals.
	bool AdmitsEdge( b3Vec3 normal, int triangleIndex, int edgeIndex ) const
	{
		int neighbor = m_edgeNeighbors[3 * triangleIndex + edgeIndex];
		if ( MatchesFace( normal, triangleIndex ) || ( neighbor != B3_NULL_INDEX && MatchesFace( normal, neighbor ) ) )
		{
			return true;
		}

		const b3MeshTriangle* triangles = b3GetMeshTriangles( m_mesh );
		const b3Vec3* vertices = b3GetMeshVertices( m_mesh );
		int v[3] = { triangles[triangleIndex].index1, triangles[triangleIndex].index2, triangles[triangleIndex].index3 };
		int ia = v[edgeIndex];
		int ib = v[edgeIndex < 2 ? edgeIndex + 1 : 0];
		int ic = v[edgeIndex > 0 ? edgeIndex - 1 : 2];

		b3Vec3 a = vertices[ia];
		b3Vec3 axis = b3Normalize( vertices[ib] - a );
		if ( b3AbsFloat( b3Dot( normal, axis ) ) > m_sinTolerance )
		{
			return false;
		}

		if ( b3Dot( normal, GetInwardDirection( a, axis, vertices[ic] ) ) > m_sinTolerance )
		{
			return false;
		}

		if ( neighbor != B3_NULL_INDEX )
		{
			int w[3] = { triangles[neighbor].index1, triangles[neighbor].index2, triangles[neighbor].index3 };
			for ( int k = 0; k < 3; ++k )
			{
				if ( w[k] != ia && w[k] != ib && b3Dot( normal, GetInwardDirection( a, axis, vertices[w[k]] ) ) > m_sinTolerance )
				{
					return false;
				}
			}
		}

		return true;
	}

	// A vertex normal is valid when the vertex is a local support point in that direction
	bool AdmitsVertex( b3Vec3 normal, int vertexIndex ) const
	{
		for ( int t : m_vertexTriangles[vertexIndex] )
		{
			if ( MatchesFace( normal, t ) )
			{
				return true;
			}
		}

		const b3Vec3* vertices = b3GetMeshVertices( m_mesh );
		b3Vec3 vertex = vertices[vertexIndex];
		for ( int other : m_vertexRings[vertexIndex] )
		{
			if ( b3Dot( normal, b3Normalize( vertices[other] - vertex ) ) > m_sinTolerance )
			{
				return false;
			}
		}

		return true;
	}

	bool AdmitsPoint( b3Vec3 normal, const b3ManifoldPoint& point ) const
	{
		const b3MeshTriangle& triangle = b3GetMeshTriangles( m_mesh )[point.triangleIndex];
		int v[3] = { triangle.index1, triangle.index2, triangle.index3 };

		// The normal was replaced by the face normal of a nearby triangle
		if ( point.meshFlags & b3_meshContactBlocked )
		{
			for ( int k = 0; k < 3; ++k )
			{
				for ( int other : m_vertexTriangles[v[k]] )
				{
					if ( MatchesFace( normal, other ) )
					{
						return true;
					}
				}
			}

			return false;
		}

		switch ( point.triangleFeature )
		{
			case b3_featureTriangleFace:
				return MatchesFace( normal, point.triangleIndex );

			case b3_featureEdge1:
			case b3_featureEdge2:
			case b3_featureEdge3:
				return AdmitsEdge( normal, point.triangleIndex, point.triangleFeature - b3_featureEdge1 );

			case b3_featureVertex1:
			case b3_featureVertex2:
			case b3_featureVertex3:
				return AdmitsVertex( normal, v[point.triangleFeature - b3_featureVertex1] );

			default:
				// No feature was resolved, so any feature of the triangle may own the normal
				if ( MatchesFace( normal, point.triangleIndex ) )
				{
					return true;
				}

				for ( int k = 0; k < 3; ++k )
				{
					if ( AdmitsEdge( normal, point.triangleIndex, k ) || AdmitsVertex( normal, v[k] ) )
					{
						return true;
					}
				}

				return false;
		}
	}

	static GhostClass Classify( const b3ManifoldPoint& point )
	{
		switch ( point.triangleFeature )
		{
			case b3_featureTriangleFace:
				return e_faceGhost;

			case b3_featureEdge1:
			case b3_featureEdge2:
			case b3_featureEdge3:
				return e_edgeGhost;

			case b3_featureVertex1:
			case b3_featureVertex2:
			case b3_featureVertex3:
				return e_vertexGhost;

			default:
				return e_otherGhost;
		}
	}

	void CountGhosts( b3BodyId bodyId, b3Pos center )
	{
		// Clustering merges manifolds whose normals agree within its threshold and hands them one normal,
		// so a point may carry a normal that far from its own feature
		float angle = m_toleranceDegrees * B3_DEG_TO_RAD + acosf( 0.996f );
		m_cosTolerance = cosf( angle );
		m_sinTolerance = sinf( angle );

		b3ContactData contacts[16];
		int contactCount = b3Body_GetContactData( bodyId, contacts, 16 );

		for ( int i = 0; i < contactCount; ++i )
		{
			bool bodyIsA = B3_ID_EQUALS( b3Shape_GetBody( contacts[i].shapeIdA ), bodyId );
			b3ShapeId otherId = bodyIsA ? contacts[i].shapeIdB : contacts[i].shapeIdA;
			if ( b3Shape_GetType( otherId ) != b3_meshShape )
			{
				continue;
			}

			for ( int j = 0; j < contacts[i].manifoldCount; ++j )
			{
				const b3Manifold* manifold = contacts[i].manifolds + j;
				b3Vec3 normal = bodyIsA ? b3Neg( manifold->normal ) : manifold->normal;

				// Speculative points that never pushed are harmless. Clustering hands every point the normal
				// of one member, so the manifold is sound if any pushing point owns that normal.
				int pushCount = 0;
				int firstPush = 0;
				bool admitted = false;
				for ( int k = 0; k < manifold->pointCount && admitted == false; ++k )
				{
					const b3ManifoldPoint& point = manifold->points[k];
					if ( point.totalNormalImpulse <= 0.0f )
					{
						continue;
					}

					firstPush = pushCount == 0 ? k : firstPush;
					pushCount += 1;
					admitted = AdmitsPoint( normal, point );
				}

				if ( pushCount == 0 )
				{
					continue;
				}

				m_manifoldCount += 1;

				if ( admitted )
				{
					continue;
				}

				const b3ManifoldPoint& point = manifold->points[firstPush];
				m_ghostCount += 1;
				m_stepGhostCount += 1;
				m_classCounts[Classify( point )] += 1;
				for ( int bit = 0; bit < m_flagCount; ++bit )
				{
					m_flagCounts[bit] += ( point.meshFlags >> bit ) & 1;
				}

				b3Vec3 anchor = bodyIsA ? point.anchorA : point.anchorB;
				m_markers[m_markerIndex] = { b3OffsetPos( center, anchor ), normal };
				m_markerIndex = ( m_markerIndex + 1 ) % m_markerCapacity;
				m_markerCount = b3MinInt( m_markerCount + 1, m_markerCapacity );
			}
		}
	}

	void Step() override
	{
		// Contacts are computed at the start of the step and anchors stay relative to that body position.
		// A fast body moves past the margin within a step, so the end position would query the wrong triangles.
		for ( int i = 0; i < m_bodyCount; ++i )
		{
			m_contactCenters[i] = b3Body_GetWorldCenter( m_bodyIds[i] );
		}

		Sample::Step();

		if ( m_didStep )
		{
			m_stepGhostCount = 0;

			for ( int i = 0; i < m_bodyCount; ++i )
			{
				b3BodyId bodyId = m_bodyIds[i];
				b3Pos position = b3Body_GetPosition( bodyId );

				// Off the belt and on the ground, so drop it back in at the start
				if ( position.y < m_baseHeight - 0.5f )
				{
					b3Body_SetTransform( bodyId, GetSpawnPosition(), RandomQuat() );
					b3Body_SetLinearVelocity( bodyId, b3Vec3_zero );
					b3Body_SetAngularVelocity( bodyId, b3Vec3_zero );
					m_recycleCount += 1;
					continue;
				}

				CountGhosts( bodyId, m_contactCenters[i] );
			}

			m_spawnTimer += 1;
			if ( m_bodyCount < m_targetCount && m_spawnTimer >= m_spawnInterval )
			{
				SpawnBody();
				m_spawnTimer = 0;
			}
		}

		float rate = m_manifoldCount > 0 ? 1.0e6f * (float)m_ghostCount / (float)m_manifoldCount : 0.0f;
		DrawTextLine( "ghost manifolds: %d this step, %d of %d pushing (%.0f ppm)", m_stepGhostCount, m_ghostCount,
					  m_manifoldCount, rate );
		DrawTextLine( "ghost feature: face %d, edge %d, vertex %d, unresolved %d", m_classCounts[e_faceGhost],
					  m_classCounts[e_edgeGhost], m_classCounts[e_vertexGhost], m_classCounts[e_otherGhost] );
		DrawTextLine( "ghost origin: hull face %d, snapped %d, unverified %d, deep %d, blocked %d", m_flagCounts[0],
					  m_flagCounts[1], m_flagCounts[2], m_flagCounts[3], m_flagCounts[4] );
		DrawTextLine( "bodies: %d, recycled: %d", m_bodyCount, m_recycleCount );
		DrawTextLine( "triangles: %d, stations: %d", m_mesh->triangleCount, (int)m_stations.size() );
	}

	void Render() override
	{
		Sample::Render();

		int stationCount = (int)m_stations.size();

		if ( m_drawFlow )
		{
			int stride = b3MaxInt( 1, stationCount / 48 );
			for ( int k = 0; k < stationCount; k += stride )
			{
				b3Vec3 p = m_stations[k] + 0.05f * m_ups[k];
				b3Vec3 t = m_tangents[k];
				DrawArrow( b3ToPos( p ), b3ToPos( p + 0.25f * m_beltSpeed * t ), MakeColor( b3_colorBlueViolet ) );
			}
		}

		if ( m_drawGhosts )
		{
			for ( int i = 0; i < m_markerCount; ++i )
			{
				b3Pos p = m_markers[i].point;
				DrawPoint( p, 8.0f, MakeColor( b3_colorRed ) );
				DrawLine( p, p + 0.5f * m_markers[i].normal, MakeColor( b3_colorRed ) );
			}
		}
	}

	bool DrawControls() override
	{
		bool rebuild = false;
		bool respawn = false;

		ImGui::SliderFloat( "Cell Size", &m_cellSize, 0.1f, 0.5f, "%.2f m" );
		rebuild |= ImGui::IsItemDeactivatedAfterEdit();

		if ( ImGui::SliderFloat( "Belt Speed", &m_beltSpeed, 0.0f, 6.0f, "%.1f m/s" ) )
		{
			UpdateBeltSpeed();
		}

		ImGui::SliderInt( "Body Count", &m_targetCount, 1, m_maxBodies );
		ImGui::SliderInt( "Spawn Interval", &m_spawnInterval, 1, 60 );

		const char* bodyNames[e_bodyTypeCount] = { "Mixed", "Box", "Capsule", "Sphere", "Cylinder" };
		int bodyType = m_bodyType;
		if ( ImGui::Combo( "Bodies", &bodyType, bodyNames, e_bodyTypeCount ) )
		{
			m_bodyType = (BodyType)bodyType;
			respawn = true;
		}

		if ( ImGui::Checkbox( "Contact Recycling", &m_contactRecycling ) )
		{
			for ( int i = 0; i < m_bodyCount; ++i )
			{
				b3Body_EnableContactRecycling( m_bodyIds[i], m_contactRecycling );
			}
		}

		ImGui::SliderFloat( "Ghost Tolerance", &m_toleranceDegrees, 1.0f, 20.0f, "%.0f deg" );
		ImGui::Checkbox( "Draw Flow", &m_drawFlow );
		ImGui::Checkbox( "Draw Ghosts", &m_drawGhosts );

		respawn |= ImGui::Button( "Respawn" );
		if ( ImGui::Button( "Reset Counters" ) )
		{
			ResetCounters();
		}

		if ( rebuild )
		{
			DestroyCircuit();
			CreateCircuit();
			ResetCounters();
		}

		if ( respawn )
		{
			DestroyBodies();
			ResetCounters();
		}

		// Shrinking the count drops the newest bodies
		while ( m_bodyCount > m_targetCount )
		{
			m_bodyCount -= 1;
			b3DestroyBody( m_bodyIds[m_bodyCount] );
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new ConveyorCircuit( context );
	}

	Segment m_segments[m_segmentCount];
	float m_segmentStart[m_segmentCount];
	float m_totalLength;
	b3Vec2 m_profile[m_profileCount];

	std::vector<b3Vec3> m_stations;
	std::vector<b3Vec3> m_tangents;
	std::vector<b3Vec3> m_laterals;
	std::vector<b3Vec3> m_ups;

	std::vector<b3Vec3> m_triangleNormals;
	std::vector<int> m_edgeNeighbors;
	std::vector<std::vector<int>> m_vertexRings;
	std::vector<std::vector<int>> m_vertexTriangles;

	b3BodyId m_groundId;
	b3MeshData* m_mesh = nullptr;
	b3ShapeId m_shapeId = {};
	b3HullData* m_cylinderHull;

	b3BodyId m_bodyIds[m_maxBodies];
	b3Pos m_contactCenters[m_maxBodies];
	int m_bodyCount;
	int m_spawnIndex;
	int m_spawnTimer;

	float m_cellSize;
	float m_beltSpeed;
	int m_targetCount;
	int m_spawnInterval;
	BodyType m_bodyType;
	float m_toleranceDegrees;
	float m_cosTolerance = 1.0f;
	float m_sinTolerance = 0.0f;
	bool m_contactRecycling;
	bool m_drawFlow;
	bool m_drawGhosts;

	static constexpr int m_flagCount = 5;
	int m_classCounts[e_ghostClassCount];
	int m_flagCounts[m_flagCount];
	int m_ghostCount;
	int m_manifoldCount;
	int m_stepGhostCount;
	int m_recycleCount;
	Marker m_markers[m_markerCapacity];
	int m_markerCount;
	int m_markerIndex;
};

static int sampleConveyorCircuit = RegisterSample( "Mesh", "Conveyor Circuit", ConveyorCircuit::Create );
