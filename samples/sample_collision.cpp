// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "GLFW/glfw3.h"
#include "camera.h"
#include "utils.h"
#include "renderer.h"
#include "sample.h"
#include "scene.h"

#include "box3d/box3d.h"

#include <imgui.h>

class RayCurtain : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new RayCurtain( context );
	}

	explicit RayCurtain( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 20.0f );
		}

		b3BoxHull box = b3MakeBoxHull( 0.6f, 0.6f, 0.6f );

		m_mesh = b3CreateTorusMesh( 10, 12, 0.65f, 0.35f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_kinematicBody;

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		bodyDef.position = { -6.0f, 3.0f, 0.0f };
		bodyDef.angularVelocity = { 0.8f, 0.4f, 0.8f };
		b3BodyId sphereBody = b3CreateBody( m_worldId, &bodyDef );
		b3Sphere sphere = { b3Vec3_zero, 0.9f };
		b3CreateSphereShape( sphereBody, &shapeDef, &sphere );

		bodyDef.position = { -2.0f, 3.0f, 0.0f };
		bodyDef.angularVelocity = { 0.8f, 0.4f, 0.8f };
		b3BodyId capsuleBody = b3CreateBody( m_worldId, &bodyDef );
		b3Capsule capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.8f };
		b3CreateCapsuleShape( capsuleBody, &shapeDef, &capsule );

		bodyDef.position = { 2.0f, 3.0f, 0.0f };
		bodyDef.angularVelocity = { 0.8f, 0.4f, 0.8f };
		b3BodyId hullBody = b3CreateBody( m_worldId, &bodyDef );
		b3CreateHullShape( hullBody, &shapeDef, &box.base );

		bodyDef.position = { 6.0f, 3.0f, 0.0f };
		bodyDef.angularVelocity = { 0.8f, 0.4f, 0.8f };
		b3BodyId meshBody = b3CreateBody( m_worldId, &bodyDef );
		b3CreateMeshShape( meshBody, &shapeDef, m_mesh, b3Vec3_one );

		m_absSpeed = 0.015f;
		m_offset = 2.0f;
		m_speed = -m_absSpeed;
	}

	~RayCurtain() override
	{
		b3DestroyMesh( m_mesh );
	}

	void Render() override
	{
		Sample::Render();

		DrawGrid( m_scene, 10 );
		DrawLine( m_scene, b3Vec3_zero, 0.4f * b3Vec3_axisX, b3_colorRed );
		DrawLine( m_scene, b3Vec3_zero, 0.4f * b3Vec3_axisY, b3_colorGreen );
		DrawLine( m_scene, b3Vec3_zero, 0.4f * b3Vec3_axisZ, b3_colorBlue );

		for ( float x = -8.0f; x <= 8.0f; x += 0.1f )
		{
			b3Vec3 rayOrigin = { x, 8.0f, m_offset };
			b3Vec3 rayTranslation = b3Vec3{ x, 0.0f, m_offset } - rayOrigin;

			b3RayResult result = b3World_CastRayClosest( m_worldId, rayOrigin, rayTranslation, b3DefaultQueryFilter() );
			if ( result.hit )
			{
				b3Vec3 point = result.point;

				DrawLine( m_scene, point, point + 0.5f * result.normal, b3_colorGreen );
			}

			DrawPoint( m_scene, rayOrigin, 4.0f, b3_colorGreen );
			DrawPoint( m_scene, rayOrigin + rayTranslation, 4.0f, b3_colorRed );
			DrawLine( m_scene, rayOrigin, rayOrigin + rayTranslation, b3_colorYellow );
		}

		if ( m_offset > 2.0f )
		{
			m_speed = -m_absSpeed;
		}
		else if ( m_offset < -2.0f )
		{
			m_speed = m_absSpeed;
		}

		if ( m_context->pause == false )
		{
			m_offset += m_speed;
		}
	}

	float m_offset;
	float m_absSpeed;
	float m_speed;
	b3MeshData* m_mesh;
};

static int sampleRayCurtain = SampleManager::Register( "Collision", "Ray Curtain", RayCurtain::Create );

// Context for ray cast callbacks. Do what you want with this.
struct CastContext
{
	b3Vec3 points[3];
	b3Vec3 normals[3];
	float fractions[3];
	uint64_t materialIds[3];
	int triangleIndices[3];
	int childIndices[3];
	int count;
	bool initialOverlap;
};

// This callback finds the closest hit. This is the most common callback used in games.
static float RayCastClosestCallback( b3ShapeId shapeId, b3Vec3 point, b3Vec3 normal, float fraction, uint64_t materialId,
									 int triangleIndex, int childIndex, void* context )
{
	CastContext* rayContext = (CastContext*)context;

	// Check for initial overlap
	if ( rayContext->initialOverlap == false && fraction == 0.0f )
	{
		// By returning -1, we instruct the calling code to ignore this shape and
		// continue the ray-cast to the next shape.
		return -1.0f;
	}

	// Ignore a specific shape. Also ignore initial overlap.
	intptr_t ignoreFlag = intptr_t( b3Shape_GetUserData( shapeId ) );
	if ( ignoreFlag == 1 )
	{
		// By returning -1, we instruct the calling code to ignore this shape and
		// continue the ray-cast to the next shape.
		return -1.0f;
	}

	rayContext->points[0] = point;
	rayContext->normals[0] = normal;
	rayContext->fractions[0] = fraction;
	rayContext->materialIds[0] = materialId;
	rayContext->triangleIndices[0] = triangleIndex;
	rayContext->childIndices[0] = childIndex;
	rayContext->count = 1;

	// By returning the current fraction, we instruct the calling code to clip the ray and
	// continue the ray-cast to the next shape. WARNING: do not assume that shapes
	// are reported in order. However, by clipping, we can always get the closest shape.
	return fraction;
}

// This callback finds any hit. For this type of query we are usually just checking for obstruction,
// so the hit data is not relevant.
// NOTE: shape hits are not ordered, so this may not return the closest hit
static float RayCastAnyCallback( b3ShapeId shapeId, b3Vec3 point, b3Vec3 normal, float fraction, uint64_t materialId,
								 int triangleIndex, int childIndex, void* context )
{
	CastContext* rayContext = (CastContext*)context;

	// Check for initial overlap
	if ( rayContext->initialOverlap == false && fraction == 0.0f )
	{
		// By returning -1, we instruct the calling code to ignore this shape and
		// continue the ray-cast to the next shape.
		return -1.0f;
	}

	intptr_t ignoreFlag = intptr_t( b3Shape_GetUserData( shapeId ) );
	if ( ignoreFlag == 1 )
	{
		// By returning -1, we instruct the calling code to ignore this shape and
		// continue the ray-cast to the next shape.
		return -1.0f;
	}

	rayContext->points[0] = point;
	rayContext->normals[0] = normal;
	rayContext->fractions[0] = fraction;
	rayContext->materialIds[0] = materialId;
	rayContext->triangleIndices[0] = triangleIndex;
	rayContext->childIndices[0] = childIndex;
	rayContext->count = 1;

	// At this point we have a hit, so we know the ray is obstructed.
	// By returning 0, we instruct the calling code to terminate the ray-cast.
	return 0.0f;
}

// This ray cast collects multiple hits along the ray.
// The shapes are not necessary reported in order, so we might not capture
// the closest shape.
// NOTE: shape hits are not ordered, so this may return hits in any order. This means that
// if you limit the number of results, you may discard the closest hit. You can see this
// behavior in the sample.
static float RayCastMultipleCallback( b3ShapeId shapeId, b3Vec3 point, b3Vec3 normal, float fraction, uint64_t materialId,
									  int triangleIndex, int childIndex, void* context )
{
	CastContext* rayContext = (CastContext*)context;

	// Check for initial overlap
	if ( rayContext->initialOverlap == false && fraction == 0.0f )
	{
		// By returning -1, we instruct the calling code to ignore this shape and
		// continue the ray-cast to the next shape.
		return -1.0f;
	}

	intptr_t ignoreFlag = intptr_t( b3Shape_GetUserData( shapeId ) );
	if ( ignoreFlag == 1 )
	{
		// By returning -1, we instruct the calling code to ignore this shape and
		// continue the ray-cast to the next shape.
		return -1.0f;
	}

	int count = rayContext->count;
	assert( count < 3 );

	rayContext->points[count] = point;
	rayContext->normals[count] = normal;
	rayContext->fractions[count] = fraction;
	rayContext->materialIds[count] = materialId;
	rayContext->triangleIndices[count] = triangleIndex;
	rayContext->childIndices[count] = childIndex;
	rayContext->count = count + 1;

	if ( rayContext->count == 3 )
	{
		// At this point the buffer is full.
		// By returning 0, we instruct the calling code to terminate the ray-cast.
		return 0.0f;
	}

	// By returning 1, we instruct the caller to continue without clipping the ray.
	return 1.0f;
}

// This ray cast collects multiple hits along the ray and sorts them.
static float RayCastSortedCallback( b3ShapeId shapeId, b3Vec3 point, b3Vec3 normal, float fraction, uint64_t materialId,
									int triangleIndex, int childIndex, void* context )
{
	CastContext* rayContext = (CastContext*)context;

	// Check for initial overlap
	if ( rayContext->initialOverlap == false && fraction == 0.0f )
	{
		// By returning -1, we instruct the calling code to ignore this shape and
		// continue the ray-cast to the next shape.
		return -1.0f;
	}

	intptr_t ignoreFlag = intptr_t( b3Shape_GetUserData( shapeId ) );
	if ( ignoreFlag == 1 )
	{
		// By returning -1, we instruct the calling code to ignore this shape and
		// continue the ray-cast to the next shape.
		return -1.0f;
	}

	int count = rayContext->count;
	assert( count <= 3 );

	int index = 3;
	while ( fraction < rayContext->fractions[index - 1] )
	{
		index -= 1;

		if ( index == 0 )
		{
			break;
		}
	}

	if ( index == 3 )
	{
		// not closer, continue but tell the caller not to consider fractions further than the largest fraction acquired
		// this only happens once the buffer is full
		assert( rayContext->count == 3 );
		assert( rayContext->fractions[2] <= 1.0f );
		return rayContext->fractions[2];
	}

	for ( int j = 2; j > index; --j )
	{
		rayContext->points[j] = rayContext->points[j - 1];
		rayContext->normals[j] = rayContext->normals[j - 1];
		rayContext->fractions[j] = rayContext->fractions[j - 1];
		rayContext->materialIds[j] = rayContext->materialIds[j - 1];
		rayContext->triangleIndices[j] = rayContext->triangleIndices[j - 1];
		rayContext->childIndices[j] = rayContext->childIndices[j - 1];
	}

	rayContext->points[index] = point;
	rayContext->normals[index] = normal;
	rayContext->fractions[index] = fraction;
	rayContext->materialIds[index] = materialId;
	rayContext->triangleIndices[index] = triangleIndex;
	rayContext->childIndices[index] = childIndex;
	rayContext->count = count < 3 ? count + 1 : 3;

	if ( rayContext->count == 3 )
	{
		return rayContext->fractions[2];
	}

	// By returning 1, we instruct the caller to continue without clipping the ray.
	return 1.0f;
}

class CastWorld : public Sample
{
public:
	enum
	{
		e_any = 0,
		e_closest = 1,
		e_multiple = 2,
		e_sorted = 3
	};

	enum
	{
		e_rayCast = 0,
		e_sphereCast = 1,
		e_capsuleCast = 2,
		e_boxCast = 3
	};

	enum
	{
		e_maxCount = 64
	};

	static Sample* Create( SampleContext* context )
	{
		return new CastWorld( context );
	}

	explicit CastWorld( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 20.0f );
		}

		m_sphere = { b3Vec3_zero, 0.9f };
		m_capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.8f };
		m_box = b3MakeBoxHull( 0.6f, 0.6f, 0.6f );
		m_mesh = b3CreateTorusMesh( 10, 12, 0.65f, 0.35f );

		m_heightField = b3CreateWave( 10, 10, 0.5f * b3Vec3_one, 0.03f, 0.09f, false );

		m_bodyIndex = 0;

		for ( int i = 0; i < e_maxCount; ++i )
		{
			m_bodies[i] = {};
		}

		m_mode = e_closest;

		m_castType = e_rayCast;
		m_castRadius = 0.5f;

		m_origin = { -20.0f, 10.0f, 0.0f };
		m_translation = { 20.0f, 10.0f, 0.0f };

		m_castContext = {};
		m_initialOverlap = false;
	}

	~CastWorld() override
	{
		b3DestroyMesh( m_mesh );
		b3DestroyHeightField( m_heightField );
	}

	void CreateShapes( b3ShapeType shapeType, int count )
	{
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.gravityScale = 0.0f;

		for ( int i = 0; i < count; ++i )
		{
			if ( B3_IS_NULL( m_bodies[m_bodyIndex] ) == false )
			{
				b3DestroyBody( m_bodies[m_bodyIndex] );
				m_bodies[m_bodyIndex] = b3_nullBodyId;
			}

			if ( m_bodyIndex % 3 == 0 )
			{
				bodyDef.type = b3_kinematicBody;
			}
			else if ( m_bodyIndex % 2 == 0 )
			{
				bodyDef.type = b3_dynamicBody;
			}
			else
			{
				bodyDef.type = b3_staticBody;
			}

			// Heightfield must be a static body
			if (shapeType == b3_heightShape)
			{
				bodyDef.type = b3_staticBody;
			}

			bodyDef.position = RandomVec3Uniform( -20.0f, 20.0f );

			b3Vec3 axis = RandomVec3Uniform( -1.0f, 1.0f );
			axis = b3Normalize( axis );
			float angle = RandomFloatRange( -B3_PI, B3_PI );
			bodyDef.rotation = b3MakeQuatFromAxisAngle( axis, angle );

			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			int flag = ( m_bodyIndex & m_ignoreBase ) == m_ignoreBase ? 1 : 0;
			shapeDef.userData = (void*)(intptr_t)flag;

			switch ( shapeType )
			{
				case b3_sphereShape:
					shapeDef.baseMaterial.userMaterialId = 11;
					b3CreateSphereShape( bodyId, &shapeDef, &m_sphere );
					break;

				case b3_capsuleShape:
					shapeDef.baseMaterial.userMaterialId = 22;
					b3CreateCapsuleShape( bodyId, &shapeDef, &m_capsule );
					break;

				case b3_hullShape:
					shapeDef.baseMaterial.userMaterialId = 33;
					b3CreateHullShape( bodyId, &shapeDef, &m_box.base );
					break;

				case b3_meshShape:
					shapeDef.baseMaterial.userMaterialId = 44;
					b3CreateMeshShape( bodyId, &shapeDef, m_mesh, { 4.0f, 3.0f, -2.0f } );
					break;

				case b3_heightShape:
				{
					shapeDef.baseMaterial.userMaterialId = 55;
					b3SurfaceMaterial materials[3] = {
						{
							.userMaterialId = 111,
						},
						{
							.userMaterialId = 222,
						},
						{
							.userMaterialId = 333,
						},
					};
					shapeDef.materials = materials;
					shapeDef.materialCount = 3;

					b3CreateHeightFieldShape( bodyId, &shapeDef, m_heightField );
				}
				break;

				default:
					assert( false );
					break;
			}

			m_bodies[m_bodyIndex] = bodyId;

			m_bodyIndex = ( m_bodyIndex + 1 ) % e_maxCount;
		}
	}

	void DestroyBody()
	{
		for ( int i = 0; i < e_maxCount; ++i )
		{
			if ( B3_IS_NULL( m_bodies[i] ) == false )
			{
				b3DestroyBody( m_bodies[i] );
				m_bodies[i] = b3_nullBodyId;
				return;
			}
		}
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 && modifiers == GLFW_MOD_CONTROL )
		{
			PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
			m_origin = pickRay.origin;
			m_translation = 100.0f * b3Normalize( pickRay.translation );
		}
	}

	void UpdateUI() override
	{
		float height = 360.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_camera->m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 210.0f, height ) );

		ImGui::Begin( "Options", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		const char* castTypes[] = { "Ray", "Sphere", "Capsule", "Box" };
		if ( ImGui::Combo( "Cast Type", &m_castType, castTypes, IM_ARRAYSIZE( castTypes ) ) )
		{
		}

		if ( m_castType != e_rayCast && m_castType != e_boxCast )
		{
			ImGui::SliderFloat( "Radius", &m_castRadius, 0.1f, 2.0f, "%.1f" );
		}

		const char* modes[] = { "Any", "Closest", "Multiple", "Sorted" };
		if ( ImGui::Combo( "Mode", &m_mode, modes, IM_ARRAYSIZE( modes ) ) )
		{
		}

		ImGui::Checkbox( "Initial Overlap", &m_initialOverlap );

		if ( ImGui::Button( "Spheres" ) )
		{
			CreateShapes( b3_sphereShape, 10 );
		}

		if ( ImGui::Button( "Capsules" ) )
		{
			CreateShapes( b3_capsuleShape, 10 );
		}

		if ( ImGui::Button( "Hulls" ) )
		{
			CreateShapes( b3_hullShape, 10 );
		}

		if ( ImGui::Button( "Meshes" ) )
		{
			CreateShapes( b3_meshShape, 1 );
		}

		if ( ImGui::Button( "Height Field" ) )
		{
			CreateShapes( b3_heightShape, 1 );
		}

		if ( ImGui::Button( "Destroy Shape" ) )
		{
			DestroyBody();
		}

		ImGui::End();

		for ( int i = 0; i < m_maxCount; ++i )
		{
			if ( ( i & m_ignoreBase ) == m_ignoreBase && B3_IS_NULL( m_bodies[i] ) == false )
			{
				b3AABB bounds = b3Body_ComputeAABB( m_bodies[i] );
				DrawBounds( m_scene, bounds, 0.0f, b3_colorYellow );
			}
		}
	}

	void Render() override
	{
		DrawTextLine( "Ctrl + left mouse to cast through cursor" );
		DrawTextLine( "Shapes drawn in yellow boxes are ignored by the ray" );

		switch ( m_mode )
		{
			case e_any:
				DrawTextLine( "Cast mode: any - check for obstruction - unsorted" );
				break;

			case e_closest:
				DrawTextLine( "Cast mode: closest - find closest shape along the cast" );
				break;

			case e_multiple:
				DrawTextLine( "Cast mode: multiple - gather multiple shapes - unsorted" );
				break;

			case e_sorted:
				DrawTextLine( "Cast mode: sorted - gather multiple shapes sorted by closeness" );
				break;

			default:
				assert( false );
				break;
		}

		for ( int i = 0; i < m_castContext.count; ++i )
		{
			DrawTextLine( "material = %d, triangle = %d", m_castContext.materialIds[i], m_castContext.triangleIndices[i] );
		}

		DrawGrid( m_scene, 10 );
		DrawTransform( m_scene, b3Transform_identity, 1.0f );

		Sample::Render();
	}

	void Step() override
	{
		Sample::Step();

		b3HexColor color1 = b3_colorOrange;
		b3HexColor color2 = b3_colorAqua;
		b3HexColor green = b3_colorGreen;
		b3HexColor gray = b3_colorGray;

		b3CastResultFcn* functions[] = { RayCastAnyCallback, RayCastClosestCallback, RayCastMultipleCallback,
										 RayCastSortedCallback };
		b3CastResultFcn* modeFcn = functions[m_mode];

		m_castContext = {};
		m_castContext.initialOverlap = m_initialOverlap;

		// Must initialize fractions for sorting
		m_castContext.fractions[0] = FLT_MAX;
		m_castContext.fractions[1] = FLT_MAX;
		m_castContext.fractions[2] = FLT_MAX;

		b3Sphere sphere = { m_origin, m_castRadius };
		b3Capsule capsule = { m_origin, m_origin + b3Vec3{ 0.0f, 1.0f, 0.0f }, m_castRadius };
		b3BoxHull box = {};

		b3Vec3 pointBuffer[2];

		b3ShapeProxy proxy = {};

		switch ( m_castType )
		{
			case e_rayCast:
				proxy.count = 0;
				break;

			case e_sphereCast:
				proxy.count = 1;
				proxy.radius = m_castRadius;
				proxy.points = &sphere.center;
				break;

			case e_capsuleCast:
				proxy.count = 2;
				proxy.radius = m_castRadius;
				pointBuffer[0] = capsule.center1;
				pointBuffer[1] = capsule.center2;
				proxy.points = pointBuffer;
				break;

			case e_boxCast:
			{
				b3Vec3 extent = { m_castRadius, 0.5f * m_castRadius, 0.25f * m_castRadius };
				box = b3MakeTransformedBoxHull( extent.x, extent.y, extent.z, { m_origin, b3Quat_identity } );
				proxy.points = box.boxPoints;
				proxy.count = box.base.vertexCount;
			}
			break;

			default:
				assert( false );
				break;
		}

		if ( m_castType == e_rayCast )
		{
			b3World_CastRay( m_worldId, m_origin, m_translation, b3DefaultQueryFilter(), modeFcn, &m_castContext );
		}
		else
		{
			b3World_CastShape( m_worldId, &proxy, m_translation, b3DefaultQueryFilter(), modeFcn, &m_castContext );
		}

		if ( m_castContext.count > 0 )
		{
			assert( m_castContext.count <= 3 );
			b3HexColor colors[3] = { b3_colorRed, b3_colorGreen, b3_colorBlue };
			DrawLine( m_scene, m_origin, m_origin + m_translation, color2 );

			for ( int i = 0; i < m_castContext.count; ++i )
			{
				b3Vec3 point = m_castContext.points[i];
				b3Vec3 normal = m_castContext.normals[i];
				DrawPoint( m_scene, point, 10.0f, colors[i] );
				b3Vec3 head = point + 0.5f * normal;

				b3Transform transform = { m_castContext.fractions[i] * m_translation, b3Quat_identity };

				if ( m_castType == e_rayCast )
				{
					DrawLine( m_scene, point, head, colors[i] );
				}
				else if ( m_castType == e_sphereCast )
				{
					DrawLine( m_scene, point, head, color1 );
					DrawSphere( m_scene, transform, sphere, colors[i] );
				}
				else if ( m_castType == e_capsuleCast )
				{
					DrawLine( m_scene, point, head, color1 );
					DrawCapsule( m_scene, transform, capsule, colors[i] );
				}
				else if ( m_castType == e_boxCast )
				{
					DrawLine( m_scene, point, head, color1 );
					DrawHull( m_scene, transform, &box.base, colors[i], false );
				}
			}
		}
		else
		{
			DrawLine( m_scene, m_origin, m_origin + m_translation, color2 );

			b3Transform transform = { m_translation, b3Quat_identity };

			if ( m_castType == e_sphereCast )
			{
				DrawSphere( m_scene, transform, sphere, gray );
			}
			else if ( m_castType == e_capsuleCast )
			{
				DrawCapsule( m_scene, transform, capsule, gray );
			}
			else if ( m_castType == e_boxCast )
			{
				DrawHull( m_scene, transform, &box.base, gray, false );
			}
		}

		DrawPoint( m_scene, m_origin, 2.0f, green );
	}

	static constexpr int m_maxCount = 64;
	static constexpr int m_ignoreBase = 0x3;

	b3Capsule m_capsule;
	b3Sphere m_sphere;
	b3BoxHull m_box;
	b3MeshData* m_mesh;
	b3HeightField* m_heightField;

	b3BodyId m_bodies[m_maxCount];
	int m_bodyIndex;
	int m_mode;
	int m_castType;
	float m_castRadius;
	bool m_initialOverlap;

	b3Vec3 m_origin;
	b3Vec3 m_translation;

	CastContext m_castContext;
};

static int sampleCastWorld = SampleManager::Register( "Collision", "Cast World", CastWorld::Create );

class MeshScale : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new MeshScale( context );
	}

	explicit MeshScale( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 20.0f );
		}

		m_mesh = b3CreateBoxMesh( { 0.0f, 0.0f, 0.0f }, { 0.5f, 0.5f, 0.5f }, true );
		m_scale = { 1.0f, 1.0f, 1.0f };

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		m_meshBodyId = b3CreateBody( m_worldId, &bodyDef );
		m_meshShapeId = b3CreateMeshShape( m_meshBodyId, &shapeDef, m_mesh, m_scale );
		m_start = { -2.0f, 0.0f, 0.0f };
		m_sphereCast = true;
	}

	~MeshScale() override
	{
		b3DestroyMesh( m_mesh );
	}

	void UpdateUI() override
	{
		float height = 200.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_camera->m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 200.0f, height ) );

		ImGui::Begin( "Mesh Scale", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		b3Vec3 scale = m_scale;
		bool changed = false;
		changed = changed || ImGui::SliderFloat( "Scale X", &scale.x, -2.0f, 2.0f, "%.1f" );
		changed = changed || ImGui::SliderFloat( "Scale Y", &scale.y, -2.0f, 2.0f, "%.1f" );
		changed = changed || ImGui::SliderFloat( "Scale Z", &scale.z, -2.0f, 2.0f, "%.1f" );

		if ( changed )
		{
			m_scale = scale;
			b3Shape_SetMesh( m_meshShapeId, m_mesh, m_scale );
		}

		ImGui::SliderFloat( "Start Y", &m_start.y, -2.0f, 2.0f, "%.1f" );
		ImGui::SliderFloat( "Start Z", &m_start.z, -2.0f, 2.0f, "%.1f" );

		ImGui::Checkbox( "sphere Cast", &m_sphereCast );

		ImGui::End();
	}

	void Step() override
	{
		b3Vec3 rayOrigin = m_start;
		b3Vec3 rayTranslation = { 4.0f, 0.0f, 0.0f };

		DrawPoint( m_scene, rayOrigin, 8.0f, b3_colorGreen );
		DrawPoint( m_scene, rayOrigin + rayTranslation, 8.0f, b3_colorRed );
		DrawLine( m_scene, rayOrigin, rayOrigin + rayTranslation, b3_colorWhite );

		if ( m_sphereCast )
		{
			b3Sphere sphere = { rayOrigin, 0.25f };

			CastContext context = {};
			b3ShapeProxy proxy = { &sphere.center, 1, sphere.radius };
			b3World_CastShape( m_worldId, &proxy, rayTranslation, b3DefaultQueryFilter(), RayCastClosestCallback, &context );

			if ( context.count > 0 )
			{
				b3Transform transform = { context.fractions[0] * rayTranslation, b3Quat_identity };
				DrawSphere( m_scene, transform, sphere, b3_colorYellow );

				b3Vec3 point = context.points[0];
				DrawLine( m_scene, point, point + 0.5f * context.normals[0], b3_colorGreen );
				DrawPoint( m_scene, point, 5.0f, b3_colorYellow );
			}
			else
			{
				b3Transform transform = { rayTranslation, b3Quat_identity };
				DrawSphere( m_scene, transform, sphere, b3_colorGray );
			}
		}
		else
		{
			b3RayResult result = b3World_CastRayClosest( m_worldId, rayOrigin, rayTranslation, b3DefaultQueryFilter() );
			if ( result.hit )
			{
				b3Vec3 point = result.point;
				DrawLine( m_scene, point, point + 0.5f * result.normal, b3_colorGreen );
				DrawPoint( m_scene, point, 5.0f, b3_colorYellow );
			}
		}
	}

	b3Vec3 m_scale;
	b3MeshData* m_mesh;
	b3BodyId m_meshBodyId;
	b3ShapeId m_meshShapeId;
	b3Vec3 m_start;
	bool m_sphereCast;
};

static int sampleMeshScale = SampleManager::Register( "Collision", "Mesh Scale", MeshScale::Create );

class ShapeCast : public Sample
{
public:
	explicit ShapeCast( SampleContext* context )
		: Sample( context )
	{
		m_camera->SetView( 120.0f, 30.0f, 20.0f, { 0.0f, 1.5f, 0.0f } );

		b3BoxHull box = b3MakeBoxHull( 0.6f, 0.6f, 0.6f );
		m_mesh = b3CreateTorusMesh( 10, 12, 0.65f, 0.35f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		for ( int index = 0; index < 3; ++index )
		{
			bodyDef.position = { -6.0f, 3.0f + 2.0f * index, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
			b3BodyId sphereBody = b3CreateBody( m_worldId, &bodyDef );
			b3Sphere sphere = { b3Vec3_zero, 0.9f };
			b3CreateSphereShape( sphereBody, &shapeDef, &sphere );

			bodyDef.position = { -2.0f, 3.0f + 2.0f * index, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.25f * B3_PI );
			b3BodyId capsuleBody = b3CreateBody( m_worldId, &bodyDef );
			b3Capsule capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.7f };
			b3CreateCapsuleShape( capsuleBody, &shapeDef, &capsule );

			bodyDef.position = { 2.0f, 3.0f + 2.0f * index, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.25f * B3_PI );
			b3BodyId hullBody = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( hullBody, &shapeDef, &box.base );

			bodyDef.position = { 6.0f, 3.0f + 2.0f * index, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
			b3BodyId meshBody = b3CreateBody( m_worldId, &bodyDef );
			b3CreateMeshShape( meshBody, &shapeDef, m_mesh, b3Vec3_one );

			// todo add height field
		}

		m_baseX = 0;
		m_baseY = 0;
		m_castOffset = {};
		m_trackingX = false;
		m_trackingY = false;
		m_initialOverlap = false;
	}

	~ShapeCast() override
	{
		b3DestroyMesh( m_mesh );
	}

	void CastSpheres() const
	{
		for ( int castIndex = 0; castIndex < 4; ++castIndex )
		{
			// World space sweep
			b3Sphere sphere = { b3Vec3_zero, 0.3f };
			b3Vec3 offset = b3Vec3{ -6.0f + 4.0f * castIndex, 3.0f, -5.0f } + m_castOffset;
			sphere.center += offset;
			b3Vec3 translation = 10.0f * b3Vec3_axisZ;
			b3ShapeProxy proxy = { &sphere.center, 1, sphere.radius };

			CastContext context = {};
			context.initialOverlap = m_initialOverlap;

			b3World_CastShape( m_worldId, &proxy, translation, b3DefaultQueryFilter(), RayCastClosestCallback, &context );

			DrawSphere( m_scene, b3Transform_identity, sphere, b3_colorGreen );

			if ( context.count > 0 )
			{
				b3Vec3 point = context.points[0];
				b3Vec3 normal = context.normals[0];
				float fraction = context.fractions[0];

				// final position with overlap resolution
				DrawSphere( m_scene, { fraction * translation, b3Quat_identity }, sphere, b3_colorRed );

				DrawPoint( m_scene, point, 2.0f, b3_colorRed );
				DrawLine( m_scene, point, point + 0.2f * normal, b3_colorYellow );
			}
			else
			{
				DrawSphere( m_scene, { translation, b3Quat_identity }, sphere, b3_colorGray );
			}
		}
	}

	void CastCapsules() const
	{
		for ( int castIndex = 0; castIndex < 4; ++castIndex )
		{
			b3Capsule capsule = { { -0.2f, -0.2f, -0.2f }, { 0.2f, 0.2f, 0.2f }, 0.2f };
			b3Vec3 offset = b3Vec3{ -6.0f + 4.0f * castIndex, 5.0f, -5.0f } + m_castOffset;
			capsule.center1 += offset;
			capsule.center2 += offset;
			b3Vec3 translation = 10.0f * b3Vec3_axisZ;
			b3ShapeProxy proxy = { &capsule.center1, 2, capsule.radius };

			CastContext context = {};
			context.initialOverlap = m_initialOverlap;

			b3World_CastShape( m_worldId, &proxy, translation, b3DefaultQueryFilter(), RayCastClosestCallback, &context );

			DrawCapsule( m_scene, b3Transform_identity, capsule, b3_colorGreen );

			if ( context.count > 0 )
			{
				b3Vec3 point = context.points[0];
				b3Vec3 normal = context.normals[0];
				float fraction = context.fractions[0];

				DrawCapsule( m_scene, { fraction * translation, b3Quat_identity }, capsule, b3_colorRed );

				DrawPoint( m_scene, point, 2.0f, b3_colorRed );
				DrawLine( m_scene, point, point + 0.2f * normal, b3_colorYellow );
			}
			else
			{
				DrawCapsule( m_scene, { translation, b3Quat_identity }, capsule, b3_colorGray );
			}
		}
	}

	void CastHulls() const
	{
		for ( int castIndex = 0; castIndex < 4; ++castIndex )
		{
			b3Vec3 offset = b3Vec3{ -6.0f + 4.0f * castIndex, 7.0f, -5.0f } + m_castOffset;
			b3Quat qx = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.25f * B3_PI );
			b3Quat qy = b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.25f * B3_PI );
			b3Quat qz = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.25f * B3_PI );

			b3Quat q = b3MulQuat( qx, b3MulQuat( qy, qz ) );

			b3Transform transform = { offset, q };

			b3BoxHull box = b3MakeTransformedBoxHull( 0.3f, 0.3f, 0.3f, transform );

			b3Vec3 translation = 10.0f * b3Vec3_axisZ;
			b3ShapeProxy proxy = { box.boxPoints, box.base.vertexCount, 0.0f };

			CastContext context = {};
			context.initialOverlap = m_initialOverlap;

			b3World_CastShape( m_worldId, &proxy, translation, b3DefaultQueryFilter(), RayCastClosestCallback, &context );

			DrawHull( m_scene, b3Transform_identity, &box.base, b3_colorGreen, false );

			if ( context.count > 0 )
			{
				b3Vec3 point = context.points[0];
				b3Vec3 normal = context.normals[0];
				float fraction = context.fractions[0];

				// final position with overlap resolution
				DrawHull( m_scene, { fraction * translation, b3Quat_identity }, &box.base, b3_colorRed, false );

				DrawPoint( m_scene, point, 2.0f, b3_colorRed );
				DrawLine( m_scene, point, point + 0.2f * normal, b3_colorYellow );
			}
			else
			{
				DrawHull( m_scene, { translation, b3Quat_identity }, &box.base, b3_colorGray, false );
			}
		}
	}

	void UpdateUI() override
	{
		float height = 80.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_camera->m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 210.0f, height ) );
		ImGui::Begin( "Shape Cast", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );
		ImGui::Checkbox( "Initial Overlap", &m_initialOverlap );
		ImGui::End();
	}

	void Render() override
	{
		Sample::Render();

		DrawTextLine( "Shift + LMB and drag to shift start position" );

		DrawGrid( m_scene, 10 );
		DrawTransform( m_scene, b3Transform_identity, 1.0f );
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 )
		{
			if ( modifiers == GLFW_MOD_SHIFT )
			{
				m_trackingX = true;
				m_baseX = p.x;
			}
			else if ( modifiers == GLFW_MOD_CONTROL )
			{
				m_trackingY = true;
				m_baseY = p.y;
			}
		}
	}

	void MouseUp( b3Vec2 p, int button ) override
	{
		if ( button == 0 )
		{
			m_trackingX = false;
			m_trackingY = false;
		}
	}

	void MouseMove( b3Vec2 p ) override
	{
		if ( m_trackingX )
		{
			m_castOffset.z = 0.05f * ( m_baseX - p.x );
		}

		if ( m_trackingY )
		{
			m_castOffset.y = 0.05f * ( m_baseY - p.y );
		}
	}

	void Step() override
	{
		CastSpheres();
		CastCapsules();
		CastHulls();
	}

	static Sample* Create( SampleContext* context )
	{
		return new ShapeCast( context );
	}

	b3MeshData* m_mesh;
	int m_baseX;
	int m_baseY;
	b3Vec3 m_castOffset;
	bool m_trackingX;
	bool m_trackingY;
	bool m_initialOverlap;
};

static int sampleShapeCast = SampleManager::Register( "Collision", "Shape Cast", ShapeCast::Create );

// Tests shape overlap versus world
class OverlapWorld : public Sample
{
public:
	enum
	{
		BODY_COUNT = 12
	};

	static Sample* Create( SampleContext* context )
	{
		return new OverlapWorld( context );
	}

	explicit OverlapWorld( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{

			m_camera->SetView( 120.0f, 30.0f, 20.0f, { 0.0f, 1.5f, 0.0f } );
		}

		b3World_SetGravity( m_worldId, b3Vec3_zero );

		m_box = b3MakeBoxHull( 0.6f, 0.6f, 0.6f );
		m_mesh = b3CreateTorusMesh( 10, 12, 0.65f, 0.35f );

		m_heightField = b3CreateWave( 10, 10, 0.2f * b3Vec3_one, 0.03f, 0.09f, false );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3ShapeDef shapeDef = b3DefaultShapeDef();

		for ( int index = 0; index < 3; ++index )
		{
			b3BodyType type = b3BodyType( index );

			bodyDef.type = type;
			bodyDef.position = { -6.0f, 3.0f + 2.0f * index, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
			b3BodyId sphereBody = b3CreateBody( m_worldId, &bodyDef );
			b3Sphere sphere = { b3Vec3_zero, 0.8f };
			b3CreateSphereShape( sphereBody, &shapeDef, &sphere );

			bodyDef.type = type;
			bodyDef.position = { -3.0f, 3.0f + 2.0f * index, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.25f * B3_PI );
			b3BodyId capsuleBody = b3CreateBody( m_worldId, &bodyDef );
			b3Capsule capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.5f };
			b3CreateCapsuleShape( capsuleBody, &shapeDef, &capsule );

			bodyDef.type = type;
			bodyDef.position = { 0.0f, 3.0f + 2.0f * index, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.25f * B3_PI );
			b3BodyId hullBody = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( hullBody, &shapeDef, &m_box.base );

			bodyDef.type = type;
			bodyDef.position = { 3.0f, 3.0f + 2.0f * index, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
			b3BodyId meshBody = b3CreateBody( m_worldId, &bodyDef );
			b3CreateMeshShape( meshBody, &shapeDef, m_mesh, { -0.5f, 1.5f, -1.0f } );

			// Height fields only on static bodies
			bodyDef.type = b3_staticBody;
			bodyDef.position = { 5.0f, 2.0f + 2.0f * index, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, -0.5f * B3_PI );
			b3BodyId heightBody = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHeightFieldShape( heightBody, &shapeDef, m_heightField );
		}

		m_baseX = 0;
		m_castOffset = 0.0f;
		m_tracking = false;
	}

	~OverlapWorld() override
	{
		b3DestroyHeightField( m_heightField );
		b3DestroyMesh( m_mesh );
	}

	static bool OverlapResultFcn( b3ShapeId shapeId, void* context )
	{
		b3HexColor* color = static_cast<b3HexColor*>( context );
		*color = b3_colorRed;

		// terminate the query
		return false;
	}

	void OverlapSpheres() const
	{
		for ( int i = 0; i < 5; ++i )
		{
			b3Sphere sphere = { { -6.0f + 3.0f * i, 3.0f, -5.0f + m_castOffset }, 0.3f };
			b3ShapeProxy proxy = { &sphere.center, 1, sphere.radius };
			b3HexColor color = b3_colorGreen;
			b3World_OverlapShape( m_worldId, &proxy, b3DefaultQueryFilter(), OverlapResultFcn, &color );
			DrawSphere( m_scene, b3Transform_identity, sphere, color );
		}
	}

	void OverlapCapsules() const
	{

		for ( int i = 0; i < 5; ++i )
		{
			b3Vec3 offset = { -6.0f + 3.0f * i, 5.0f, -5.0f + m_castOffset };
			b3Capsule capsule = { { -0.2f, -0.2f, -0.2f }, { 0.2f, 0.2f, 0.2f }, 0.2f };
			capsule.center1 += offset;
			capsule.center2 += offset;
			b3ShapeProxy proxy = { &capsule.center1, 2, capsule.radius };
			b3HexColor color = b3_colorGreen;
			b3World_OverlapShape( m_worldId, &proxy, b3DefaultQueryFilter(), OverlapResultFcn, &color );
			DrawCapsule( m_scene, b3Transform_identity, capsule, color );
		}
	}

	void OverlapHulls() const
	{
		for ( int i = 0; i < 5; ++i )
		{
			b3Transform transform{ { -6.0f + 3.0f * i, 7.0f, -5.0f + m_castOffset }, b3Quat_identity };
			b3BoxHull box = b3MakeTransformedBoxHull( 0.3f, 0.3f, 0.3f, transform );
			b3ShapeProxy proxy = { box.boxPoints, box.base.vertexCount, 0.0f };
			b3HexColor color = b3_colorGreen;
			b3World_OverlapShape( m_worldId, &proxy, b3DefaultQueryFilter(), OverlapResultFcn, &color );
			DrawHull( m_scene, b3Transform_identity, &box.base, color, false );
		}
	}

	void Render() override
	{
		Sample::Render();

		DrawTextLine( "Shift + LMB and drag to move shapes" );

		DrawGrid( m_scene, 10 );
		DrawLine( m_scene, b3Vec3_zero, 0.4f * b3Vec3_axisX, b3_colorRed );
		DrawLine( m_scene, b3Vec3_zero, 0.4f * b3Vec3_axisY, b3_colorGreen );
		DrawLine( m_scene, b3Vec3_zero, 0.4f * b3Vec3_axisZ, b3_colorBlue );
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 )
		{
			if ( modifiers == GLFW_MOD_SHIFT )
			{
				m_tracking = true;
				m_baseX = p.x;
			}
		}
	}

	void MouseUp( b3Vec2 p, int button ) override
	{
		if ( button == 0 )
		{
			m_tracking = false;
		}
	}

	void MouseMove( b3Vec2 p ) override
	{
		if ( m_tracking )
		{
			m_castOffset = 0.05f * ( m_baseX - p.x );
		}
	}

	void Step() override
	{
		OverlapSpheres();
		OverlapCapsules();
		OverlapHulls();
	}

	b3BoxHull m_box;
	b3MeshData* m_mesh;
	b3HeightField* m_heightField;
	int m_baseX;
	float m_castOffset;
	bool m_tracking;
};

static int sampleOverlapWorld = SampleManager::Register( "Collision", "Overlap World", OverlapWorld::Create );

class InitialOverlap : public Sample
{
public:
	explicit InitialOverlap( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -140.0f, 10.0f, 10.0f, { 0.0f, 0.0f, 0.0f } );
		}

		int indices[] = { 0, 1, 2, 2, 3, 0 };

		b3Vec3 vertices[] = { { -0.5f, 0.5f, 0.5f }, { -0.5f, 0.5f, -0.5f }, { -0.5f, -0.5f, -0.5f }, { -0.5f, -0.5f, 0.5f } };

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { 0.0f, 0.0f, 0.0f };
		bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 10.0f * B3_DEG_TO_RAD );
		b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		// m_mesh = b3CreateBoxMesh( { 0.0f, 0.0f, 0.0f }, { 0.5f, 0.5f, 0.5f } );
		b3MeshDef def = {};
		def.triangleCount = 2;
		def.indices = indices;
		def.vertexCount = 4;
		def.vertices = vertices;
		def.materialIndices = nullptr;
		def.useMedianSplit = false;

		m_mesh = b3CreateMesh( &def, nullptr, 0 );
		b3CreateMeshShape( body, &shapeDef, m_mesh, 4.0f * b3Vec3_one );

		m_initialOverlap = true;
	}

	~InitialOverlap() override
	{
		b3DestroyMesh( m_mesh );
	}

	void Render() override
	{
		Sample::Render();

		DrawTransform( m_scene, b3Transform_identity, 1.0f );
	}

	void UpdateUI() override
	{
		float height = 80.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_camera->m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 200.0f, height ) );
		ImGui::Begin( "Initial Overlap", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );
		ImGui::Checkbox( "initial overlap", &m_initialOverlap );
		ImGui::End();
	}

	void Step() override
	{
		Sample::Step();

		b3Vec3 offset = { -2.1f, -0.8f, 0.95f };

		CastContext context = {};
		context.initialOverlap = m_initialOverlap;

		b3Capsule capsule = {
			.center1 = offset,
			.center2 = offset + b3Vec3{ 0.0f, 1.0f, 0.0f },
			.radius = 0.25f,
		};

		// zero length cast
		b3Vec3 translation = { 0.0f, 0.0f, 0.0f };

		b3ShapeProxy proxy = { &capsule.center1, 2, capsule.radius };

		DrawCapsule( m_scene, b3Transform_identity, capsule, b3_colorGreen );

		b3World_CastShape( m_worldId, &proxy, translation, b3DefaultQueryFilter(), RayCastClosestCallback, &context );

		float fraction = context.count > 0 ? context.fractions[0] : 1.0f;
		b3Transform shapeEnd = { fraction * translation, b3Quat_identity };
		DrawCapsule( m_scene, shapeEnd, capsule, context.count > 0 ? b3_colorRed : b3_colorGreen );

		if ( context.count > 0 )
		{
			b3Vec3 p = context.points[0];
			b3Vec3 n = context.normals[0];
			DrawLine( m_scene, p, p + 0.5f * n, b3_colorAliceBlue );
			DrawPoint( m_scene, p, 8.0f, b3_colorAliceBlue );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new InitialOverlap( context );
	}

	b3MeshData* m_mesh;
	bool m_initialOverlap;
};

static int sampleInitialOverlap = SampleManager::Register( "Collision", "Initial Overlap", InitialOverlap::Create );

// pointA[0] = {0.000000000, 0.000000000, 0.000000000}
// pointA[1] = {0.000000000, -6400.000000000, 0.000000000}
// pointA[2] = {6400.000000000, 0.000000000, 22.609375000}
// pointB[i] = {43616.210937500, -100213.000000000, 132631.812500000}
// pointB[i] = {342231.968750000, 359711.687500000, 132631.812500000}
// radiusB = 40.0f
// xfA = {{0.000000000, 0.000000000, 0.000000000}, {{0.000000000, 0.000000000, 0.000000000}, 1.000000000}
// xfB = {{-115200.000000000, -19200.000000000, -202755.000000000}, {{0.000000000, 0.000000000, 0.000000000}, 1.000000000}
// t = {0.008614914, 0.000000000, 72267.117187500}
// maxFraction = 0.970617533, canEncroach = 0
class ShapeCastDebug : public Sample
{
public:
	explicit ShapeCastDebug( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 120.0f, 30.0f, 20.0f, { 0.0f, 1.5f, 0.0f } );
		}

		// this triggers an assert with scale of 1 and b3_lengthUnitsPerMeter of 100
		float scale = 0.01f;
		m_triangle[0] = scale * b3Vec3{ 0.000000000, 0.000000000, 0.000000000 };
		m_triangle[1] = scale * b3Vec3{ 0.000000000, -6400.000000000, 0.000000000 };
		m_triangle[2] = scale * b3Vec3{ 6400.000000000, 0.000000000, 22.609375000 };

		b3Vec3 origin = m_triangle[0];
		m_triangle[0] = b3Vec3_zero;
		m_triangle[1] -= origin;
		m_triangle[2] -= origin;

		m_points[0] = scale * b3Vec3{ 200.305283, 200.460999, 9.53760529 };
		m_points[1] = scale * b3Vec3{ -200.305283, 200.460999, 9.53760529 };
		m_points[2] = scale * b3Vec3{ -200.305283, -200.460999, 9.53760529 };
		m_points[3] = scale * b3Vec3{ 200.305283, -200.460999, 9.53760529 };
		m_points[4] = scale * b3Vec3{ 200.305283, 200.460999, -9.53760529 };
		m_points[5] = scale * b3Vec3{ -200.305283, 200.460999, -9.53760529 };
		m_points[6] = scale * b3Vec3{ -200.305283, -200.460999, -9.53760529 };
		m_points[7] = scale * b3Vec3{ 200.305283, -200.460999, -9.53760529 };

		m_capsule.center1 = scale * b3Vec3{ 43616.210937500, -100213.000000000, 132631.812500000 };
		m_capsule.center2 = scale * b3Vec3{ 342231.968750000, 359711.687500000, 132631.812500000 };
		m_capsule.radius = scale * 1.0f;

		float length = b3Distance( m_capsule.center1, m_capsule.center2 );
		(void)length;

		m_transform = { scale * b3Vec3{ -115200.000000000, -19200.000000000, -202755.000000000 } - origin,
						{ { 0.00000000, 0.00000000, 0.0f }, 1.0f } };

		m_translation = scale * b3Vec3{ 0.008614914, 0.000000000, 72267.117187500 };

		m_box = b3CreateHull( m_points, 8, 8 );
	}

	~ShapeCastDebug() override
	{
		b3DestroyHull( m_box );
	}

	void Render() override
	{
		Sample::Render();

		DrawGrid( m_scene, 10 );
		DrawTransform( m_scene, b3Transform_identity, 1.0f );

		b3ShapeCastPairInput input;
		input.proxyA = { m_triangle, 3, 0.0f };
		// input.proxyB = { m_points, 8, 0.0f };
		input.proxyB = { &m_capsule.center1, 2, m_capsule.radius };
		input.transformA = b3Transform_identity;
		input.transformB = m_transform;
		input.translationB = m_translation;
		input.maxFraction = 0.970617533;
		input.canEncroach = false;

		b3CastOutput output = b3ShapeCast( &input );

		DrawFace( m_scene, m_triangle[0], m_triangle[1], m_triangle[2], b3_colorCyan );
		// DrawHull( m_scene, m_transform, m_box, b3_colorGreen, false );
		DrawCapsule( m_scene, m_transform, m_capsule, b3_colorGreen );

		if ( output.hit )
		{
			// final position with overlap resolution
			b3Vec3 shapeEnd = m_transform.p + output.fraction * m_translation;
			// DrawHull( m_scene, { shapeEnd, m_transform.q }, m_box, b3_colorRed, false );
			DrawCapsule( m_scene, { shapeEnd, m_transform.q }, m_capsule, b3_colorRed );
		}

		b3Vec3 shapeEnd = m_transform.p + m_translation;
		// DrawHull( m_scene, { shapeEnd, m_transform.q }, m_box, b3_colorGray, false );
		DrawCapsule( m_scene, { shapeEnd, m_transform.q }, m_capsule, b3_colorGray );
	}

	static Sample* Create( SampleContext* context )
	{
		return new ShapeCastDebug( context );
	}

	b3Transform m_transform;
	b3Vec3 m_translation;
	b3Capsule m_capsule;
	b3Hull* m_box;
	b3Vec3 m_points[8];
	b3Vec3 m_triangle[3];
};

static int sampleShapeCastDebug = SampleManager::Register( "Collision", "Shape Cast Debug", ShapeCastDebug::Create );

class DistanceDebug : public Sample
{
public:
	explicit DistanceDebug( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 120.0f, 30.0f, 20.0f, { 0.0f, 1.5f, 0.0f } );
		}

		// this triggers an assert with scale of 1 and b3_lengthUnitsPerMeter of 100
		float scale = 0.01f;
		m_triangle[0] = scale * b3Vec3{ 1400.00000, 1600.00000, 70.1534424 };
		m_triangle[1] = scale * b3Vec3{ 1400.00000, 1500.00000, 66.1250000 };
		m_triangle[2] = scale * b3Vec3{ 1500.00000, 1600.00000, 72.3507080 };

		// shifting to the origin give more accuracy
		b3Vec3 origin = m_triangle[0];
		m_triangle[0] = b3Vec3_zero;
		m_triangle[1] -= origin;
		m_triangle[2] -= origin;

		m_points[0] = scale * b3Vec3{ 200.305283, 200.460999, 9.53760529 };
		m_points[1] = scale * b3Vec3{ -200.305283, 200.460999, 9.53760529 };
		m_points[2] = scale * b3Vec3{ -200.305283, -200.460999, 9.53760529 };
		m_points[3] = scale * b3Vec3{ 200.305283, -200.460999, 9.53760529 };
		m_points[4] = scale * b3Vec3{ 200.305283, 200.460999, -9.53760529 };
		m_points[5] = scale * b3Vec3{ -200.305283, 200.460999, -9.53760529 };
		m_points[6] = scale * b3Vec3{ -200.305283, -200.460999, -9.53760529 };
		m_points[7] = scale * b3Vec3{ 200.305283, -200.460999, -9.53760529 };

		m_boxA = b3MakeBoxHull( 40.0f, 1.0f, 40.0f );
		m_boxB = b3MakeTransformedBoxHull( 0.5f, 10.0f, 0.5f, { { 0.0f, 10.0f, 0.0f }, b3Quat_identity } );

		// m_transformA = {
		//	.p = { 0.0f, -1.0f, 0.0f },
		//	.q = b3Quat_identity,
		// };

		m_transformB = {
			.p = { -1.64657831e-06, 1.00989532471, 0.0f },
			.q = { { 0.00000000, 0.00000000, 0.00494779600 }, 0.999987781 },
		};

		m_box = b3CreateHull( m_points, 8, 8 );

		m_simplexCount = 0;
		m_simplexIndex = 0;

		memset( m_simplexes, 0, sizeof( m_simplexes ) );
	}

	~DistanceDebug() override
	{
		b3DestroyHull( m_box );
	}

	void Render() override
	{
		Sample::Render();

		DrawGrid( m_scene, 10 );
		DrawTransform( m_scene, b3Transform_identity, 1.0f );
	}

	void UpdateUI() override
	{
		float height = 80.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_camera->m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Distance Debug", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		ImGui::SliderInt( "simplex index", &m_simplexIndex, 0, m_simplexCount - 1 );

		ImGui::End();
	}

	static void BuildWitnessPoints( const b3Simplex* simplex, b3Vec3& vertexA, b3Vec3& vertexB )
	{
		const b3SimplexVertex* vs = simplex->vertices;
		int count = simplex->count;
		B3_ASSERT( 1 <= count && count <= 4 );

		switch ( count )
		{
			case 1:
				vertexA = vs[0].wA;
				vertexB = vs[0].wB;
				break;

			case 2:
				vertexA = vs[0].a * vs[0].wA + vs[1].a * vs[1].wA;
				vertexB = vs[0].a * vs[0].wB + vs[1].a * vs[1].wB;
				break;

			case 3:
				vertexA = vs[0].a * vs[0].wA + vs[1].a * vs[1].wA + vs[2].a * vs[2].wA;
				vertexB = vs[0].a * vs[0].wB + vs[1].a * vs[1].wB + vs[2].a * vs[2].wB;
				break;

			case 4:
				// Force identical points and *zero* distance
				vertexA = vertexB = vs[0].a * vs[0].wA + vs[1].a * vs[1].wA + vs[2].a * vs[2].wA + vs[3].a * vs[3].wA;
				break;

			default:
				B3_ASSERT( !"Should never get here!" );
				break;
		}
	}

	static b3Vec3 GetClosestPoint( const b3Simplex* simplex )
	{
		int count = simplex->count;
		B3_ASSERT( 1 <= count && count <= 4 );

		const b3SimplexVertex* vs = simplex->vertices;

		switch ( count )
		{
			case 1:
				return simplex->vertices[0].w;

			case 2:
				return vs[0].a * vs[0].w + vs[1].a * vs[1].w;

			case 3:
				return vs[0].a * vs[0].w + vs[1].a * vs[1].w + vs[2].a * vs[2].w;

			case 4:
				return vs[0].a * vs[0].w + vs[1].a * vs[1].w + vs[2].a * vs[2].w + vs[3].a * vs[3].w;

			default:
				B3_ASSERT( !"Should never get here!" );
				break;
		}

		return b3Vec3_zero;
	}

	void Step() override
	{
		b3DistanceInput input;
		input.proxyA = { m_boxA.boxPoints, 8, 0.0f };
		input.proxyB = { m_boxB.boxPoints, 8, 0.0f };
		input.transformA = b3Transform_identity;
		input.transformB = m_transformB;
		input.useRadii = false;

		b3SimplexCache cache = {};
		b3DistanceOutput output = b3ShapeDistance( &input, &cache, m_simplexes, 32 );
		m_simplexCount = output.simplexCount;

		// DrawFace( m_scene, m_triangle[0], m_triangle[1], m_triangle[2], b3_colorCyan );
		// DrawHull( m_scene, m_transformB, m_box, b3_colorGreen, false );

		DrawHull( m_scene, b3Transform_identity, &m_boxA.base, b3_colorGreen, false );
		DrawHull( m_scene, m_transformB, &m_boxB.base, b3_colorCyan, false );

		for ( int i = 0; i < m_boxB.base.vertexCount; ++i )
		{
			b3Vec3 p = b3TransformPoint( m_transformB, m_boxB.boxPoints[i] );
			DrawWorldString( m_camera, p, b3_colorAliceBlue, " %d", i );
		}

		DrawPoint( m_scene, output.pointA, 5.0f, b3_colorWhite );
		DrawPoint( m_scene, output.pointB, 5.0f, b3_colorWhite );
		DrawLine( m_scene, output.pointA, output.pointA + 1.0f * output.normal, b3_colorWhite );

		b3Vec3 n = output.normal;
		DrawTextLine( "distance = %g, normal = %g, %g, %g", output.distance, n.x, n.y, n.z );

		B3_ASSERT( output.distance > 0.0f && b3IsNormalized( output.normal ) );

		if ( m_simplexCount > 0 )
		{
			b3Simplex* simplex = m_simplexes + m_simplexIndex;

			b3Vec3 v1, v2;
			BuildWitnessPoints( simplex, v1, v2 );

			DrawPoint( m_scene, v1, 10.0f, b3_colorGreen );
			DrawPoint( m_scene, v2, 10.0f, b3_colorGreen );

			for ( int i = 0; i < simplex->count; ++i )
			{
				v1 = simplex->vertices[i].wA;
				v2 = simplex->vertices[i].wB;
				DrawPoint( m_scene, v1, 5.0f, b3_colorRed );
				DrawPoint( m_scene, v2, 5.0f, b3_colorRed );
			}

			b3Vec3 p = GetClosestPoint( simplex );
			float distance = b3Length( p );

			DrawTextLine( "current distance = %.5f", distance );
		}

		for ( int i = 0; i < m_simplexCount; ++i )
		{
			b3Simplex* simplex = m_simplexes + i;

			if ( simplex->count == 1 )
			{
				b3SimplexVertex* v1 = simplex->vertices + 0;
				DrawTextLine( "v1s : %d, v2s : %d", v1->indexA, v1->indexB );
			}
			else if ( simplex->count == 2 )
			{
				b3SimplexVertex* v1 = simplex->vertices + 0;
				b3SimplexVertex* v2 = simplex->vertices + 1;
				DrawTextLine( "v1s : %d/%d, v2s : %d/%d", v1->indexA, v2->indexA, v1->indexB, v2->indexB );
			}
			else if ( simplex->count == 3 )
			{
				b3SimplexVertex* v1 = simplex->vertices + 0;
				b3SimplexVertex* v2 = simplex->vertices + 1;
				b3SimplexVertex* v3 = simplex->vertices + 2;
				DrawTextLine( "v1s : %d/%d/%d, v2s : %d/%d/%d", v1->indexA, v2->indexA, v3->indexA, v1->indexB, v2->indexB,
							  v3->indexB );
			}
			else if ( simplex->count == 3 )
			{
				b3SimplexVertex* v1 = simplex->vertices + 0;
				b3SimplexVertex* v2 = simplex->vertices + 1;
				b3SimplexVertex* v3 = simplex->vertices + 2;
				b3SimplexVertex* v4 = simplex->vertices + 3;
				DrawTextLine( "v1s : %d/%d/%d/%d, v2s : %d/%d/%d/%d", v1->indexA, v2->indexA, v3->indexA, v4->indexA, v1->indexB,
							  v2->indexB, v3->indexB, v4->indexB );
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new DistanceDebug( context );
	}

	b3Transform m_transformB;
	b3Hull* m_box;
	b3BoxHull m_boxA;
	b3BoxHull m_boxB;
	b3Vec3 m_points[8];
	b3Vec3 m_triangle[3];
	b3Simplex m_simplexes[32];
	int m_simplexIndex;
	int m_simplexCount;
};

static int sampleDistanceDebug = SampleManager::Register( "Collision", "Distance Debug", DistanceDebug::Create );

class ShapeDistance : public Sample
{
public:
	enum ShapeType
	{
		e_point,
		e_segment,
		e_triangle,
		e_box
	};

	explicit ShapeDistance( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( -90.0f, 0.0f, 10.0f, { 0.0f, 0.0f, 0.0f } );
		}

		m_point = b3Vec3_zero;
		m_segment[0] = { -0.5f, 0.0f };
		m_segment[1] = { 0.5f, 0.0f };

		m_triangle[0] = { -0.5f, 0.0f };
		m_triangle[1] = { 0.5f, 0.0f };
		m_triangle[2] = { 0.0f, 1.0f };
		m_box = b3MakeBoxHull( 0.25f, 0.5f, 1.0f );

		m_triangle[0] = { 0.0f, 0.0f, -4.0f };
		m_triangle[1] = { 0.0f, 0.0f, -8.0f };
		m_triangle[2] = { -4.0f, 0.0f, -8.0f };
		m_box = b3MakeBoxHull( 0.02f, 0.2f, 0.04f );

		m_transform = { { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
		m_transform = {
			.p = { 0.00000000, 0.796873450, -4.50000000 },
			.q = { { -0.964603841, -0.0856034234, 0.248849452 }, 0.0168983955 },
		};

		m_cache = {};
		m_simplexCount = 0;
		m_simplexIndex = 0;
		m_dragStart = { 0.0f, 0.0f };
		m_basePosition = { 0.0f, 0.0f, 0.0f };
		m_baseQuat = { .v = { .z = 1.0f } };

		m_dragging = false;
		m_rotating = false;
		m_showIndices = false;
		m_useCache = false;
		m_drawSimplex = false;

		m_typeA = e_triangle;
		m_typeB = e_box;
		m_radiusA = 0.0f;
		m_radiusB = 0.0f;

		m_proxyA = MakeProxy( m_typeA, m_radiusA );
		m_proxyB = MakeProxy( m_typeB, m_radiusB );
	}

	b3ShapeProxy MakeProxy( ShapeType type, float radius )
	{
		b3ShapeProxy proxy = {};
		proxy.radius = radius;

		switch ( type )
		{
			case e_point:
				proxy.points = &m_point;
				proxy.count = 1;
				break;

			case e_segment:
				proxy.points = m_segment;
				proxy.count = 2;
				break;

			case e_triangle:
				proxy.points = m_triangle;
				proxy.count = 3;
				break;

			case e_box:
				proxy.points = b3GetHullPoints( &m_box.base );
				proxy.count = m_box.base.vertexCount;
				break;

			default:
				assert( false );
		}

		return proxy;
	}

	void DrawShape( ShapeType type, b3Transform transform, float radius, b3HexColor color )
	{
		switch ( type )
		{
			case e_point:
			{
				if ( radius > 0.0f )
				{
					b3Sphere sphere = {
						.center = m_point,
						.radius = radius,
					};
					DrawSphere( m_scene, transform, sphere, color );
				}
				else
				{
					b3Vec3 p = b3TransformPoint( transform, m_point );
					DrawPoint( m_scene, p, 5.0f, color );
				}
			}
			break;

			case e_segment:
			{
				if ( radius > 0.0f )
				{
					b3Capsule capsule = {
						.center1 = m_segment[0],
						.center2 = m_segment[1],
						.radius = radius,
					};

					DrawCapsule( m_scene, transform, capsule, color );
				}
				else
				{
					b3Vec3 p1 = b3TransformPoint( transform, m_segment[0] );
					b3Vec3 p2 = b3TransformPoint( transform, m_segment[1] );
					DrawLine( m_scene, p1, p2, color );
				}
			}
			break;

			case e_triangle:
			{
				b3Vec3 p1 = b3TransformPoint( transform, m_triangle[0] );
				b3Vec3 p2 = b3TransformPoint( transform, m_triangle[1] );
				b3Vec3 p3 = b3TransformPoint( transform, m_triangle[2] );
				DrawLine( m_scene, p1, p2, color );
				DrawLine( m_scene, p2, p3, color );
				DrawLine( m_scene, p3, p1, color );
			}
			break;

			case e_box:
				DrawHull( m_scene, transform, &m_box.base, color, false );
				break;

			default:
				assert( false );
		}
	}

	void UpdateUI() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 16.0f * fontSize;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->m_height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 19.0f * fontSize, height ) );

		ImGui::Begin( "Shape Distance", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		const char* shapeTypes[] = { "point", "segment", "triangle", "box" };
		int shapeType = int( m_typeA );
		if ( ImGui::Combo( "shape A", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) ) )
		{
			m_typeA = ShapeType( shapeType );
			m_proxyA = MakeProxy( m_typeA, m_radiusA );
		}

		if ( ( m_typeA == e_point || m_typeA == e_segment ) && ImGui::SliderFloat( "radius A", &m_radiusA, 0.0f, 0.5f, "%.2f" ) )
		{
			m_proxyA.radius = m_radiusA;
		}

		shapeType = int( m_typeB );
		if ( ImGui::Combo( "shape B", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) ) )
		{
			m_typeB = ShapeType( shapeType );
			m_proxyB = MakeProxy( m_typeB, m_radiusB );
		}

		if ( ( m_typeB == e_point || m_typeB == e_segment ) && ImGui::SliderFloat( "radius B", &m_radiusB, 0.0f, 0.5f, "%.2f" ) )
		{
			m_proxyB.radius = m_radiusB;
		}

		ImGui::Separator();

		ImGui::Checkbox( "show indices", &m_showIndices );
		ImGui::Checkbox( "use cache", &m_useCache );

		ImGui::Separator();

		if ( ImGui::Checkbox( "draw simplex", &m_drawSimplex ) )
		{
			m_simplexIndex = 0;
		}

		if ( m_drawSimplex && m_simplexCount > 0 )
		{
			ImGui::SliderInt( "index", &m_simplexIndex, 0, m_simplexCount - 1 );
			m_simplexIndex = b3ClampInt( m_simplexIndex, 0, m_simplexCount - 1 );
		}

		ImGui::End();
	}

	void MouseDown( b3Vec2 ps, int button, int mods ) override
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			if ( mods == 0 && m_rotating == false )
			{
				m_dragging = true;
				PickRay ray = m_camera->BuildPickRay( ps.x, ps.y );
				b3Vec3 d = b3Normalize( ray.translation );

				// Intersect with plane going through origin
				// p = c + alpha * f
				// dot(p, f) = 0
				// dot(c, f) + alpha = 0
				// p = c - dot(c, f) * f
				m_dragStart = b3MulSub( ray.origin, b3Dot( ray.origin, d ), d );
				m_basePosition = m_transform.p;
			}
			else if ( mods == GLFW_MOD_SHIFT && m_dragging == false )
			{
				m_rotating = true;
				m_rotateStart = ps.x;
				m_baseQuat = m_transform.q;
			}
		}
	}

	void MouseUp( b3Vec2, int button ) override
	{
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			m_dragging = false;
			m_rotating = false;
		}
	}

	void MouseMove( b3Vec2 ps ) override
	{
		if ( m_dragging )
		{
			PickRay ray = m_camera->BuildPickRay( ps.x, ps.y );
			b3Vec3 d = b3Normalize( ray.translation );
			b3Vec3 p = b3MulSub( ray.origin, b3Dot( ray.origin, d ), d );
			m_transform.p = m_basePosition + ( p - m_dragStart );
		}
		else if ( m_rotating && m_camera->m_width > 0.0f )
		{
			float dx = ( ps.x - m_rotateStart ) / m_camera->m_width;
			float angle = b3ClampFloat( 2.0f * dx, -B3_PI, B3_PI );
			b3Vec3 axis = m_camera->GetForward();
			b3Quat deltaQuat = b3MakeQuatFromAxisAngle( axis, angle );
			m_transform.q = b3MulQuat( deltaQuat, m_baseQuat );
		}
	}

	static b3Vec3 Weight2( float a1, b3Vec3 w1, float a2, b3Vec3 w2 )
	{
		return { a1 * w1.x + a2 * w2.x, a1 * w1.y + a2 * w2.y };
	}

	static b3Vec3 Weight3( float a1, b3Vec3 w1, float a2, b3Vec3 w2, float a3, b3Vec3 w3 )
	{
		return { a1 * w1.x + a2 * w2.x + a3 * w3.x, a1 * w1.y + a2 * w2.y + a3 * w3.y };
	}

	static void ComputeWitnessPoints( b3Vec3* a, b3Vec3* b, const b3Simplex* s )
	{
		const b3SimplexVertex* vs = s->vertices;
		int count = s->count;
		B3_ASSERT( 1 <= count && count <= 4 );

		switch ( count )
		{
			case 1:
				*a = vs[0].wA;
				*b = vs[0].wB;
				break;

			case 2:
				*a = vs[0].a * vs[0].wA + vs[1].a * vs[1].wA;
				*b = vs[0].a * vs[0].wB + vs[1].a * vs[1].wB;
				break;

			case 3:
				*a = vs[0].a * vs[0].wA + vs[1].a * vs[1].wA + vs[2].a * vs[2].wA;
				*b = vs[0].a * vs[0].wB + vs[1].a * vs[1].wB + vs[2].a * vs[2].wB;
				break;

			case 4:
				// Force identical points and *zero* distance
				*a = *b = vs[0].a * vs[0].wA + vs[1].a * vs[1].wA + vs[2].a * vs[2].wA + vs[3].a * vs[3].wA;
				break;

			default:
				B3_ASSERT( !"Should never get here!" );
				break;
		}
	}

	void Render() override
	{
		DrawTransform( m_scene, b3Transform_identity, 0.5f );
		Sample::Render();
	}

	void Step() override
	{
		b3DistanceInput input;
		input.proxyA = m_proxyA;
		input.proxyB = m_proxyB;
		input.transformA = b3Transform_identity;
		input.transformB = m_transform;
		input.useRadii = m_radiusA > 0.0f || m_radiusB > 0.0f;

		if ( m_useCache == false )
		{
			m_cache.count = 0;
		}

		b3DistanceOutput output = b3ShapeDistance( &input, &m_cache, m_simplexes, m_simplexCapacity );

		if ( output.distance > 0.0f )
		{
			assert( b3IsNormalized( output.normal ) );
			b3Vec3 d = b3Sub( output.pointB, output.pointA );
			float length = b3Length( d );
			assert( b3AbsFloat( length - output.distance ) < 10.0f * FLT_EPSILON );
			b3Vec3 n = b3Normalize( d );
			assert( b3AbsFloat( n.x - output.normal.x ) < 100.0f * FLT_EPSILON );
			assert( b3AbsFloat( n.y - output.normal.y ) < 100.0f * FLT_EPSILON );
			assert( b3AbsFloat( n.z - output.normal.z ) < 100.0f * FLT_EPSILON );
		}

		m_simplexCount = output.simplexCount;

		DrawShape( m_typeA, b3Transform_identity, m_radiusA, b3_colorCyan );
		DrawShape( m_typeB, m_transform, m_radiusB, b3_colorBisque );

		if ( m_drawSimplex )
		{
			b3Simplex* simplex = m_simplexes + m_simplexIndex;
			b3SimplexVertex* vertices = simplex->vertices;

			if ( m_simplexIndex > 0 )
			{
				// The first recorded simplex does not have valid barycentric coordinates
				b3Vec3 pointA, pointB;
				ComputeWitnessPoints( &pointA, &pointB, simplex );

				DrawLine( m_scene, pointA, pointB, b3_colorWhite );
				DrawPoint( m_scene, pointA, 10.0f, b3_colorLightGreen );
				DrawPoint( m_scene, pointB, 10.0f, b3_colorLightBlue );
			}

			b3HexColor colors[3] = { b3_colorRed, b3_colorGreen, b3_colorBlue };

			for ( int i = 0; i < simplex->count; ++i )
			{
				b3SimplexVertex* vertex = vertices + i;
				DrawPoint( m_scene, vertex->wA, 10.0f, colors[i] );
				DrawPoint( m_scene, vertex->wB, 10.0f, colors[i] );
			}
		}
		else
		{
			DrawLine( m_scene, output.pointA, output.pointB, b3_colorDimGray );
			DrawPoint( m_scene, output.pointA, 10.0f, b3_colorLightGreen );
			DrawPoint( m_scene, output.pointB, 10.0f, b3_colorLightBlue );
			DrawLine( m_scene, output.pointA, output.pointA + 0.5f * output.normal, b3_colorYellow );
		}

		if ( m_showIndices )
		{
			for ( int i = 0; i < m_proxyA.count; ++i )
			{
				b3Vec3 p = m_proxyA.points[i];
				DrawWorldString( m_camera, p, b3_colorWhite, " %d", i );
			}

			for ( int i = 0; i < m_proxyB.count; ++i )
			{
				b3Vec3 p = b3TransformPoint( m_transform, m_proxyB.points[i] );
				DrawWorldString( m_camera, p, b3_colorWhite, " %d", i );
			}
		}

		DrawTextLine( "mouse button 1: drag" );
		DrawTextLine( "mouse button 1 + shift: rotate" );
		DrawTextLine( "distance = %.4f, iterations = %d", output.distance, output.iterations );

		if ( m_cache.count == 1 )
		{
			DrawTextLine( "cache = {%d}, {%d}", m_cache.indexA[0], m_cache.indexB[0] );
		}
		else if ( m_cache.count == 2 )
		{
			DrawTextLine( "cache = {%d, %d}, {%d, %d}", m_cache.indexA[0], m_cache.indexA[1], m_cache.indexB[0],
						  m_cache.indexB[1] );
		}
		else if ( m_cache.count == 3 )
		{
			DrawTextLine( "cache = {%d, %d, %d}, {%d, %d, %d}", m_cache.indexA[0], m_cache.indexA[1], m_cache.indexA[2],
						  m_cache.indexB[0], m_cache.indexB[1], m_cache.indexB[2] );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new ShapeDistance( context );
	}

	static constexpr int m_simplexCapacity = 20;

	b3BoxHull m_box;
	b3Vec3 m_triangle[3];
	b3Vec3 m_point;
	b3Vec3 m_segment[2];

	ShapeType m_typeA;
	ShapeType m_typeB;
	float m_radiusA;
	float m_radiusB;
	b3ShapeProxy m_proxyA;
	b3ShapeProxy m_proxyB;

	b3SimplexCache m_cache;
	b3Simplex m_simplexes[m_simplexCapacity];
	int m_simplexCount;
	int m_simplexIndex;

	b3Transform m_transform;

	b3Vec3 m_basePosition;
	b3Vec3 m_dragStart;

	b3Quat m_baseQuat;
	float m_rotateStart;

	bool m_dragging;
	bool m_rotating;
	bool m_showIndices;
	bool m_useCache;
	bool m_drawSimplex;
};

static int sampleShapeDistance = SampleManager::Register( "Collision", "Shape Distance", ShapeDistance::Create );

class TimeOfImpact : public Sample
{
public:
	enum ShapeType
	{
		e_box,
		e_capsule,
		e_triangle,
	};

	explicit TimeOfImpact( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( -90.0f, 0.0f, 10.0f, { 0.0f, 0.0f, 0.0f } );
		}

		m_box = b3MakeBoxHull( 0.02f, 0.2f, 0.04f );
		m_capsule = { { 0.0f, -0.2f, 0.0f }, { 0.0f, 0.2f, 0.0f }, 0.02f };
		m_triangle[0] = { -4.0f, 0.0f, -4.0f };
		m_triangle[1] = { -4.0f, 0.0f, -8.0f };
		m_triangle[2] = { -8.0f, 0.0f, -8.0f };

		m_typeA = e_triangle;
		m_typeB = e_capsule;

		m_proxyA = MakeProxy( m_typeA );
		m_proxyB = MakeProxy( m_typeB );

		m_sweepA = {};
		m_sweepA.q1.s = 1.0f;
		m_sweepA.q2.s = 1.0f;

		m_sweepB = {};
		m_sweepB.c1 = { -4.06512070, 0.101333618, -7.87591267 };
		m_sweepB.c2 = { -4.15895557, 0.0356027633, -7.69682646 };
		m_sweepB.q1 = { { -0.860495985, -0.272824734, 0.0724888667 }, 0.424097389 };
		m_sweepB.q2 = { { -0.604184389, -0.424355596, 0.0457959622 }, 0.672894001 };
	}

	b3ShapeProxy MakeProxy( ShapeType type )
	{
		b3ShapeProxy proxy = {};

		switch ( type )
		{
			case e_capsule:
				proxy.points = &m_capsule.center1;
				proxy.radius = m_capsule.radius;
				proxy.count = 2;
				break;

			case e_triangle:
				proxy.points = m_triangle;
				proxy.count = 3;
				break;

			case e_box:
				proxy.points = b3GetHullPoints( &m_box.base );
				proxy.count = m_box.base.vertexCount;
				break;

			default:
				assert( false );
		}

		return proxy;
	}

	void DrawShape( ShapeType type, b3Transform transform, b3HexColor color )
	{
		switch ( type )
		{
			case e_capsule:
			{
				DrawCapsule( m_scene, transform, m_capsule, color );
				b3Vec3 center = b3Lerp( m_capsule.center1, m_capsule.center2, 0.5f );
				center = b3TransformPoint( transform, center );
				b3Transform xf = {
					.p = center,
					.q = transform.q,
				};
				DrawTransform( m_scene, xf, 0.025f );
			}
			break;

			case e_triangle:
			{
				b3Vec3 p1 = b3TransformPoint( transform, m_triangle[0] );
				b3Vec3 p2 = b3TransformPoint( transform, m_triangle[1] );
				b3Vec3 p3 = b3TransformPoint( transform, m_triangle[2] );
				DrawLine( m_scene, p1, p2, color );
				DrawLine( m_scene, p2, p3, color );
				DrawLine( m_scene, p3, p1, color );
				DrawWorldString( m_camera, p1, b3_colorWhite, "0" );
				DrawWorldString( m_camera, p2, b3_colorWhite, "1" );
				DrawWorldString( m_camera, p3, b3_colorWhite, "2" );
			}
			break;

			case e_box:
				DrawHull( m_scene, transform, &m_box.base, color, false );
				break;

			default:
				assert( false );
		}
	}

	void UpdateUI() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 16.0f * fontSize;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->m_height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 19.0f * fontSize, height ) );

		ImGui::Begin( "Time of Impact", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		const char* shapeTypes[] = { "point", "segment", "triangle", "box" };
		int shapeType = int( m_typeA );
		if ( ImGui::Combo( "shape A", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) ) )
		{
			m_typeA = ShapeType( shapeType );
			m_proxyA = MakeProxy( m_typeA );
		}

		shapeType = int( m_typeB );
		if ( ImGui::Combo( "shape B", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) ) )
		{
			m_typeB = ShapeType( shapeType );
			m_proxyB = MakeProxy( m_typeB );
		}

		ImGui::Separator();

		ImGui::End();
	}

	void Render() override
	{
		DrawTransform( m_scene, b3Transform_identity, 0.5f );
		Sample::Render();
	}

	void Step() override
	{
		b3TOIInput input = {};
		input.proxyA = m_proxyA;
		input.proxyB = m_proxyB;
		input.sweepA = m_sweepA;
		input.sweepB = m_sweepB;
		input.maxFraction = 1.0f;

		b3TOIOutput output = b3TimeOfImpact( &input );

		DrawShape( m_typeA, b3Transform_identity, b3_colorCyan );

		b3Transform transform1 = b3GetSweepTransform( &m_sweepB, 0.0f );
		b3Transform transform2 = b3GetSweepTransform( &m_sweepB, 1.0f );

		// qr = inv(q1) * q2
		b3Quat qr = b3InvMulQuat( transform1.q, transform2.q );
		float angle;
		(void)b3GetAxisAngle( &angle, qr );
		DrawTextLine( "angle = %g", 180.0f * angle / B3_PI );

		DrawShape( m_typeB, transform1, b3_colorLightGreen );
		DrawShape( m_typeB, transform2, b3_colorLightCoral );

		if ( output.fraction < 1.0f )
		{
			b3Transform transformHit = b3GetSweepTransform( &m_sweepB, output.fraction );
			DrawShape( m_typeB, transformHit, b3_colorLightCyan );
		}

		if ( output.state == b3_toiStateHit || output.state == b3_toiStateFailed )
		{
			DrawLine( m_scene, output.point, output.point + 0.5f * output.normal, b3_colorDimGray );
			DrawPoint( m_scene, output.point, 10.0f, b3_colorLightGreen );
		}

		switch ( output.state )
		{
			case b3_toiStateUnknown:
				DrawTextLine( "unknown" );
				break;

			case b3_toiStateFailed:
				DrawTextLine( "failed" );
				break;

			case b3_toiStateOverlapped:
				DrawTextLine( "overlapped" );
				break;

			case b3_toiStateHit:
				DrawTextLine( "hit %g", output.fraction );
				break;

			case b3_toiStateSeparated:
				DrawTextLine( "separated" );
				break;

			default:
				assert( false );
				break;
		}

		DrawTextLine( "iterations / push / root = %d / %d / %d", output.distanceIterations, output.pushBackIterations,
					  output.rootIterations );
	}

	static Sample* Create( SampleContext* context )
	{
		return new TimeOfImpact( context );
	}

	b3BoxHull m_box;
	b3Vec3 m_triangle[3];
	b3Capsule m_capsule;

	ShapeType m_typeA;
	ShapeType m_typeB;
	b3ShapeProxy m_proxyA;
	b3ShapeProxy m_proxyB;
	b3Sweep m_sweepA;
	b3Sweep m_sweepB;
};

static int sampleTimeOfImpact = SampleManager::Register( "Collision", "Time of Impact", TimeOfImpact::Create );

class CapsuleCastRay : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new CapsuleCastRay( context );
	}

	explicit CapsuleCastRay( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 120.0f, 30.0f, 20.0f, { 0.0f, 1.5f, 0.0f } );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_kinematicBody;
		m_bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		m_scale = 1.0f;

		// b3Capsule Capsule = {
		//	{ 1.88094640f, 1.13457823f, 30.2966614f }, { -1.37514091f, -2.26233172f, 848.671997f }, 51.7184296f
		// };

		b3Capsule capsule = {
			.center1 = { 0.0f, 0.0f, 0.0f },
			.center2 = { 0.0f, 1.0f, 0.0f },
			.radius = 0.5f,
		};

		m_offset = capsule.center1;

		b3CreateCapsuleShape( m_bodyId, &shapeDef, &capsule );
	}

	void Render() override
	{
		Sample::Render();

		DrawGrid( m_scene, 10 );
		DrawLine( m_scene, b3Vec3_zero, 0.4f * b3Vec3_axisX, b3_colorRed );
		DrawLine( m_scene, b3Vec3_zero, 0.4f * b3Vec3_axisY, b3_colorGreen );
		DrawLine( m_scene, b3Vec3_zero, 0.4f * b3Vec3_axisZ, b3_colorBlue );

		b3BodyRayCastInput input;
		input.origin = { -1.0f, 0.5f, 0.0f };
		input.translation = { 2.0f, 0.0f, 0.0f };
		input.filter = b3DefaultQueryFilter();
		input.maxFraction = 1.0f;

		b3BodyCastResult result = b3Body_CastRay( m_bodyId, &input, b3Transform_identity );

		DrawLine( m_scene, input.origin, input.origin + input.translation, b3_colorGray );
		DrawPoint( m_scene, input.origin, 4.0f, b3_colorGreen );
		DrawPoint( m_scene, input.origin + input.translation, 4.0f, b3_colorRed );

		if ( result.hit )
		{
			DrawPoint( m_scene, result.point, 4.0f, b3_colorOrange );
		}
	}

	b3BodyId m_bodyId;
	b3Vec3 m_offset;
	float m_scale;
};

static int sampleCapsuleCastRay = SampleManager::Register( "Collision", "Capsule Cast Ray", CapsuleCastRay::Create );
