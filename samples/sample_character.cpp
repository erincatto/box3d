// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/draw.h"
#include "gfx/keycodes.h"
#include "mesh_loader.h"
#include "sample.h"

#include "box3d/box3d.h"

#include <imgui.h>

class CapsulePlane : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new CapsulePlane( context );
	}

	explicit CapsulePlane( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 120.0f, 30.0f, 20.0f, { 0.0f, 1.5f, 0.0f } );
		}

		m_transform = { { 0.0f, 1.0f, 0.4f }, b3Quat_identity };
		m_capsule = { { 0.0f, -0.5f, 0.0f }, { 0.0f, 0.5f, 0.0f }, 0.25f };

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { 0.0f, 1.0f, 1.0f };
		b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3CreateHullShape( body, &shapeDef, &box.base );

		m_baseTranslation = b3Pos_zero;
		m_baseX = 0;
		m_baseY = 0;
		m_origin = b3Pos_zero;
		m_tracking = false;
		m_planeCount = 0;
	}

	void Solve()
	{
		b3PlaneSolverResult result = b3SolvePlanes( b3Vec3_zero, m_planes, m_planeCount );
		m_transform.p = m_transform.p + result.delta;
	}

	void Render() override
	{
		Sample::Render();

		DrawGroundGrid( 10 );
		DrawLine( b3Pos_zero, b3OffsetPos( b3Pos_zero, 2.0f * b3Vec3_axisX ), MakeColor( b3_colorRed ) );
		DrawLine( b3Pos_zero, b3OffsetPos( b3Pos_zero, 2.0f * b3Vec3_axisY ), MakeColor( b3_colorGreen ) );
		DrawLine( b3Pos_zero, b3OffsetPos( b3Pos_zero, 2.0f * b3Vec3_axisZ ), MakeColor( b3_colorBlue ) );
	}

	bool DrawControls() override
	{
		if ( ImGui::Button( "Solve" ) )
		{
			Solve();
		}

		return true;
	}

	static bool PlaneResultFcn( b3ShapeId shape, const b3PlaneResult* results, int planeCount, void* context )
	{
		CapsulePlane* self = static_cast<CapsulePlane*>( context );
		for ( int i = 0; i < planeCount && self->m_planeCount < 3; ++i )
		{
			self->m_planes[self->m_planeCount] = { results[i].plane, FLT_MAX, 0.0f, true };
			self->m_planeCount += 1;
		}
		return true;
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 && ( modifiers & MOD_ALT ) == 0 )
		{
			PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
			m_origin = pickRay.origin + 10.0f * b3Normalize( pickRay.translation );
			m_baseTranslation = m_transform.p;
			m_tracking = true;
		}
	}

	void MouseUp( b3Vec2 p, int button ) override
	{
		m_tracking = false;
	}

	void MouseMove( b3Vec2 p ) override
	{
		if ( m_tracking )
		{
			PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
			b3Pos origin = pickRay.origin + 10.0f * b3Normalize( pickRay.translation );
			m_transform.p = m_baseTranslation + b3SubPos( origin, m_origin );
		}
	}

	void Step() override
	{
		m_planeCount = 0;

		DrawSolidCapsule( m_transform, m_capsule, MakeColor( b3_colorGreen ) );
		b3QueryFilter filter = b3DefaultQueryFilter();

		b3Capsule capsule = { m_capsule.center1, m_capsule.center2, m_capsule.radius };
		b3World_CollideMover( m_worldId, m_transform.p, &capsule, filter, PlaneResultFcn, this );

		for ( int i = 0; i < m_planeCount; ++i )
		{
			b3Plane plane = m_planes[i].plane;
			b3Pos p1 = m_transform.p + ( plane.offset - m_capsule.radius ) * plane.normal;
			b3Pos p2 = p1 + 0.1f * plane.normal;
			DrawPoint( p1, 5.0f, MakeColor( b3_colorYellow ) );
			DrawLine( p1, p2, MakeColor( b3_colorYellow ) );
		}
	}

	static constexpr int m_planeCapacity = 3;

	b3WorldTransform m_transform;
	b3Capsule m_capsule;
	b3CollisionPlane m_planes[m_planeCapacity] = {};
	int m_planeCount;

	b3Pos m_baseTranslation;
	b3Pos m_origin;
	int m_baseX;
	int m_baseY;
	bool m_tracking;
};

static int sampleCapsulePlane = RegisterSample( "Character", "CapsulePlane", CapsulePlane::Create );

// Exercises the deep-overlap path of b3World_CollideMover against each primitive
// shape type: drag the mover capsule (yellow) into the static sphere, capsule,
// and box hull and watch the returned planes (lime arrows) and the solved
// push-out position (cyan). The HUD reports the count of planes and the count
// of degenerate (zero) normals; the latter must stay at 0 even when the mover
// is buried inside a shape.
class MoverOverlap : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new MoverOverlap( context );
	}

	explicit MoverOverlap( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 120.0f, 25.0f, 10.0f, { 0.0f, 1.0f, 0.0f } );
		}

		m_capsule = { { 0.0f, -0.5f, 0.0f }, { 0.0f, 0.5f, 0.0f }, 0.35f };
		m_transform = { { 0.0f, 3.5f, 0.0f }, b3Quat_identity };

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		// Static sphere
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { -3.0f, 1.0f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3Sphere sphere = { b3Vec3_zero, 0.6f };
			b3CreateSphereShape( body, &shapeDef, &sphere );
		}

		// Static capsule
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, 1.0f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3Capsule capsule = { { 0.0f, 0.0f, -0.7f }, { 0.0f, 0.0f, 0.7f }, 0.4f };
			b3CreateCapsuleShape( body, &shapeDef, &capsule );
		}

		// Static box hull
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 3.0f, 1.0f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3BoxHull box = b3MakeBoxHull( 0.6f, 0.6f, 0.6f );
			b3CreateHullShape( body, &shapeDef, &box.base );
		}

		m_baseTranslation = b3Pos_zero;
		m_origin = b3Pos_zero;
		m_tracking = false;
		m_planeCount = 0;
		m_zeroNormalCount = 0;
	}

	void Render() override
	{
		Sample::Render();
		DrawGroundGrid( 12 );
	}

	static bool PlaneResultFcn( b3ShapeId /*shape*/, const b3PlaneResult* results, int planeCount, void* context )
	{
		MoverOverlap* self = static_cast<MoverOverlap*>( context );
		for ( int i = 0; i < planeCount && self->m_planeCount < m_planeCapacity; ++i )
		{
			self->m_results[self->m_planeCount] = results[i];
			self->m_planeCount += 1;
		}
		return true;
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == 0 && ( modifiers & MOD_ALT ) == 0 )
		{
			PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
			m_origin = pickRay.origin + 10.0f * b3Normalize( pickRay.translation );
			m_baseTranslation = m_transform.p;
			m_tracking = true;
		}
	}

	void MouseUp( b3Vec2 /*p*/, int /*button*/ ) override
	{
		m_tracking = false;
	}

	void MouseMove( b3Vec2 p ) override
	{
		if ( m_tracking )
		{
			PickRay pickRay = m_camera->BuildPickRay( p.x, p.y );
			b3Pos origin = pickRay.origin + 10.0f * b3Normalize( pickRay.translation );
			m_transform.p = m_baseTranslation + b3SubPos( origin, m_origin );
		}
	}

	void Step() override
	{
		// Query overlapping shapes. The mover capsule is relative to the body origin.
		b3Capsule mover = { m_capsule.center1, m_capsule.center2, m_capsule.radius };

		m_planeCount = 0;
		m_zeroNormalCount = 0;
		b3QueryFilter filter = b3DefaultQueryFilter();
		b3World_CollideMover( m_worldId, m_transform.p, &mover, filter, PlaneResultFcn, this );

		// Mover at the queried position.
		DrawSolidCapsule( m_transform, m_capsule, MakeColor( b3_colorYellow ) );

		// One arrow per returned plane, drawn from the contact point along the
		// normal. A degenerate (zero) normal is drawn red to surface the bug.
		b3CollisionPlane solverPlanes[m_planeCapacity];
		for ( int i = 0; i < m_planeCount; ++i )
		{
			b3PlaneResult r = m_results[i];
			bool valid = b3IsNormalized( r.plane.normal );
			b3HexColor color = valid ? b3_colorLimeGreen : b3_colorRed;
			b3Pos rp = b3OffsetPos( m_transform.p, r.point );
			DrawPoint( rp, 6.0f, MakeColor( color ) );
			DrawArrow( rp, b3OffsetPos( rp, 0.5f * r.plane.normal ), MakeColor( color ) );
			if ( valid == false )
			{
				m_zeroNormalCount += 1;
			}
			solverPlanes[i] = { r.plane, FLT_MAX, 0.0f, true };
		}

		// Solve the planes and show the pushed-out capsule pose.
		b3PlaneSolverResult solved = b3SolvePlanes( b3Vec3_zero, solverPlanes, m_planeCount );
		b3WorldTransform pushed = { m_transform.p + solved.delta, m_transform.q };
		DrawSolidCapsule( pushed, m_capsule, MakeColor( b3_colorCyan ) );

		DrawTextLine( "drag the capsule with the left mouse to push it into the shapes" );
		DrawTextLine( "yellow = queried pose, cyan = solved push-out, lime = valid plane normals" );
		DrawTextLine( "planes: %d   degenerate normals: %d", m_planeCount, m_zeroNormalCount );
	}

	bool DrawControls() override
	{
		ImGui::Text( "planes: %d", m_planeCount );
		ImGui::Text( "degenerate normals: %d", m_zeroNormalCount );
		return true;
	}

	static constexpr int m_planeCapacity = 32;

	b3WorldTransform m_transform;
	b3Capsule m_capsule;
	b3PlaneResult m_results[m_planeCapacity] = {};
	int m_planeCount;
	int m_zeroNormalCount;

	b3Pos m_baseTranslation;
	b3Pos m_origin;
	bool m_tracking;
};

static int sampleMoverOverlap = RegisterSample( "Character", "MoverOverlap", MoverOverlap::Create );

class BasicMover : public Sample
{
public:
	explicit BasicMover( SampleContext* context )
		: Sample( context )
	{
		b3Pos moverPosition = { 7.5f, 0.75f, 9.0f };

		if ( m_context->restart == false )
		{
			m_camera->SetView( 120.0f, 30.0f, 5.0f, moverPosition );
		}

		m_mover.Initialize( this, moverPosition );

		{
			m_levelMesh = CreateMeshData( "data/meshes/test_map01.obj", 1.0f, false, false, true, true );
			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3SurfaceMaterial materials[3];
			materials[0] = { 0.6f, 0.0f, 0 };
			materials[1] = { 0.6f, 1.0f, 1 };
			materials[2] = { 0.1f, 0.0f, 2 };
			shapeDef.materials = materials;
			shapeDef.materialCount = 3;

			b3CreateMeshShape( body, &shapeDef, m_levelMesh, b3Vec3_one );

			// b3Transform transform = { { 0.0f, 1.0f, 14.0f }, b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.75f * B3_PI ) };
			// b3BoxHull box = b3MakeTransformedBoxHull( 5.0f, 1.0f, 0.5f, transform );
			// b3CreateHullShape( body, &shapeDef, &box.base );

			{
				b3Transform transform = { { 4.0f, 1.0f, 14.0f }, b3Quat_identity };
				b3BoxHull box = b3MakeTransformedBoxHull( 1.0f, 1.0f, 1.0f, transform );
				b3CreateHullShape( body, &shapeDef, &box.base );
			}
			{
				b3Transform transform = { { 4.0f, 1.0f, 13.95f }, b3Quat_identity };
				b3BoxHull box = b3MakeTransformedBoxHull( 1.0f, 1.0f, 1.0f, transform );
				b3CreateHullShape( body, &shapeDef, &box.base );
			}
			{
				b3Transform transform = { { 5.8f, 1.0f, 13.7f }, b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.1f * B3_PI ) };
				b3BoxHull box = b3MakeTransformedBoxHull( 1.0f, 1.0f, 1.0f, transform );
				b3CreateHullShape( body, &shapeDef, &box.base );
			}
		}

		{
			m_stairs = CreateMeshData( "data/meshes/stairs.obj", 1.0f, false, false, true, true );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { -10.0f, 0.0f, 0.0f };

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3CreateMeshShape( body, &shapeDef, m_stairs, { 0.75f, 0.75f, -1.5f } );
		}

		{
			m_torus = b3CreateTorusMesh( 10, 12, 2.0f, 1.0f );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { -10.0f, 1.0f, -8.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.5f * B3_PI );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3CreateMeshShape( body, &shapeDef, m_torus, { -0.75f, 1.5f, 0.5f } );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 20.0f, 0.0f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			m_heightField = b3CreateWave( 50.0f, 50.0f, b3Vec3_one, 0.02f, 0.04f, true );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3SurfaceMaterial materials[3];
			materials[0] = { 0.6f, 0.0f, 0 };
			materials[1] = { 0.6f, 1.0f, 1 };
			materials[2] = { 0.1f, 0.0f, 2 };
			shapeDef.materials = materials;
			shapeDef.materialCount = 3;

			b3CreateHeightFieldShape( body, &shapeDef, m_heightField );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, 1.4f, 6.0f };

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			m_enemyShape.maxPush = 1.0f;
			m_enemyShape.clipVelocity = true;

			b3Capsule capsule = {
				{ 0.0f, -0.5f, 0.0f },
				{ 0.0f, 0.5f, 0.0f },
				0.3f,
			};

			// shapeDef.filter = { 2u, ~0u, 0 };
			shapeDef.userData = &m_enemyShape;
			shapeDef.baseMaterial.customColor = b3_colorMediumVioletRed;
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3CreateCapsuleShape( body, &shapeDef, &capsule );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, 1.4f, 5.0f };

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			m_friendlyShape.maxPush = 0.01f;
			m_friendlyShape.clipVelocity = false;

			b3Capsule capsule = {
				{ 0.0f, -0.5f, 0.0f },
				{ 0.0f, 0.5f, 0.0f },
				0.3f,
			};

			shapeDef.filter = { 2u, ~0u, 0 };
			shapeDef.userData = &m_friendlyShape;
			shapeDef.baseMaterial.customColor = b3_colorLimeGreen;
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			b3CreateCapsuleShape( body, &shapeDef, &capsule );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 7.0f, 5.0f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3Sphere sphere = { b3Vec3_zero, 0.5f };
			b3CreateSphereShape( body, &shapeDef, &sphere );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 7.0f, 2.0f, -3.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.baseMaterial.customColor = b3_colorFloralWhite;
			b3BoxHull box = b3MakeBoxHull( 0.5f, 0.25f, 0.5f );
			m_ignoreShapeId = b3CreateHullShape( body, &shapeDef, &box.base );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -2.0f, 1.6f, 0.0f };
			bodyDef.gravityScale = 2.0f;
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 1000.0f;

			b3BoxHull box = b3MakeBoxHull( 0.75f, 1.5f, 0.1f );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );

			b3Quat axisQuat = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, b3Vec3_axisY );
			b3Vec3 offset = { -0.75f, 0.0f, 0.0f };

			b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b3ToVec3( bodyDef.position + offset );
			jointDef.base.localFrameA.q = axisQuat;
			jointDef.base.localFrameB.p = offset;
			jointDef.base.localFrameB.q = axisQuat;

			jointDef.enableLimit = true;
			jointDef.lowerAngle = B3_DEG_TO_RAD * -90.0f;
			jointDef.upperAngle = B3_DEG_TO_RAD * 90.0f;
			jointDef.enableSpring = true;
			jointDef.hertz = 1.0f;
			jointDef.dampingRatio = 0.5f;
			jointDef.enableMotor = false;
			jointDef.maxMotorTorque = 100.0f;
			jointDef.base.drawScale = 2.0f;

			b3CreateRevoluteJoint( m_worldId, &jointDef );
		}

		//{
		//	b3BodyDef BodyDef = b3DefaultBodyDef();
		//	BodyDef.type = b3_dynamicBody;
		//	BodyDef.position = { 25.0f, 5.0f, 10.0f };
		//	b3BodyId Body = b3CreateBody(m_worldId, &bodyDef );

		//	b3ShapeDef ShapeDef = b3DefaultShapeDef();
		//	b3Sphere Sphere = { b3Vec3_zero, 0.5f };
		//	Body->AddSphere( &ShapeDef, Sphere );
		//}

		m_camera->m_thirdPerson = false;
		m_clipVelocity = true;
		// sapp_lock_mouse( true );

		// m_haveMouseLast = false;
		// m_mouseLast = { 0.0f, 0.0f };
		// m_mouseDelta = { 0.0f, 0.0f };
	}

	~BasicMover() override
	{
		m_camera->m_thirdPerson = false;
		sapp_lock_mouse( false );
		b3DestroyMesh( m_levelMesh );
		b3DestroyMesh( m_stairs );
		b3DestroyMesh( m_torus );
		b3DestroyHeightField( m_heightField );
	}

	void Render() override
	{
		Sample::Render();
		DrawAxes( { { 0.0f, 0.0f, 0.02f }, b3Quat_identity }, 2.0f );
	}

	bool DrawControls() override
	{
		bool thirdPerson = m_camera->m_thirdPerson;
		if ( ImGui::Checkbox( "Third Person (T)", &thirdPerson ) )
		{
			ToggleThirdPerson();
		}

		ImGui::Checkbox( "Clip Velocity", &m_clipVelocity );

		return true;
	}

	void Keyboard( int key, int action, int mods ) override
	{
		if ( key == KEY_T && action == ACTION_PRESS )
		{
			ToggleThirdPerson();
		}
	}

	void Step() override
	{
		m_mover.Step( &m_ignoreShapeId, 1, m_clipVelocity );
		DrawTextLine( "third person (T) = %d", m_camera->m_thirdPerson );
		DrawTextLine( "deltaX = %g, deltaY = %g", m_mouseDelta.x, m_mouseDelta.y );

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BasicMover( context );
	}

	CharacterMover m_mover;
	MoverShapeUserData m_enemyShape;
	MoverShapeUserData m_friendlyShape;
	b3MeshData* m_levelMesh;
	b3MeshData* m_stairs;
	b3MeshData* m_torus;
	b3HeightFieldData* m_heightField;
	b3ShapeId m_ignoreShapeId;
	bool m_clipVelocity;
};

static int sampleMover = RegisterSample( "Character", "Mover", BasicMover::Create );

struct ClosestShapeCastContext
{
	b3ShapeId ignoreShapes[16];
	int ignoreCount;
	float closestFraction;
	b3Vec3 closestNormal;
	b3Pos closestPoint;
	b3ShapeId closestShape;
	bool hit;
	bool startedSolid;
};

static float ClosestShapeCastCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
									   int triangleIndex, int childIndex, void* context )
{
	auto* ctx = static_cast<ClosestShapeCastContext*>( context );

	for ( int i = 0; i < ctx->ignoreCount; ++i )
	{
		if ( B3_ID_EQUALS( shapeId, ctx->ignoreShapes[i] ) )
		{
			return -1.0f;
		}
	}

	if ( fraction == 0.0f )
	{
		ctx->startedSolid = true;
		return -1.0f;
	}

	if ( fraction < ctx->closestFraction )
	{
		ctx->closestFraction = fraction;
		ctx->closestNormal = normal;
		ctx->closestPoint = point;
		ctx->closestShape = shapeId;
		ctx->hit = true;
	}

	return ctx->closestFraction;
}

// --- Trace result matching s&box's SceneTraceResult ---

struct TraceResult
{
	b3Pos endPosition;
	b3Vec3 normal;
	b3Pos hitPoint;
	float fraction;
	bool hit;
	bool startedSolid;
};

// --- RigidbodyCharacter ---

struct RigidbodyCharacter
{
	// s&box unit conversion: 1 unit = 1 inch = 0.0254m (40 units per meter)
	static constexpr float SRC = 0.0254f;

	// Tuning — s&box defaults (converted to meters)
	static constexpr float m_walkSpeed = 230.0f * SRC;
	static constexpr float m_runSpeed = 350.0f * SRC;
	static constexpr float m_jumpSpeed = 300.0f * SRC;
	static constexpr float m_maxSlopeAngle = 45.0f;
	static constexpr float m_characterGravity = 15.0f;
	static constexpr float m_characterMass = 500.0f;
	static constexpr float m_jumpCooldownTime = 0.2f;

	// s&box PlayerController parameters
	static constexpr float m_stepUpHeight = 18.0f * SRC;
	static constexpr float m_stepDownHeight = 18.0f * SRC;
	static constexpr float m_skin = 0.095f * SRC;
	static constexpr float m_brakePower = 0.2f;
	static constexpr float m_surfaceFriction = 0.6f;
	static constexpr float m_airFriction = 0.1f;

	// Character dimensions (radius=16, height=72 in s&box units)
	static constexpr float m_bodyRadius = 16.0f * SRC;
	static constexpr float m_totalHeight = 72.0f * SRC;
	static constexpr float m_feetHeight = m_totalHeight * 0.5f;

	// Physics handles
	b3BodyId m_bodyId;
	b3ShapeId m_bodyCapsuleId;
	b3ShapeId m_feetBoxId;

	// All shapes on this body (for ignore list in traces)
	b3ShapeId m_ownShapes[4];
	int m_ownShapeCount;

	// State
	b3Vec3 m_groundNormal;
	b3Vec3 m_groundVelocity;
	float m_jumpCooldown;
	bool m_onGround;
	bool m_sprint;

	// Step-up state: position to restore after physics step
	bool m_didStep;
	b3Pos m_stepPosition;

	// Debug readouts
	b3Vec3 m_lastWishVelocity;
	b3Pos m_massCenterWorld;

	Sample* m_sample;

	void Initialize( Sample* sample, b3Pos position )
	{
		m_sample = sample;
		m_onGround = false;
		m_sprint = false;
		m_jumpCooldown = 0.0f;
		m_groundNormal = b3Vec3_axisY;
		m_groundVelocity = b3Vec3_zero;
		m_lastWishVelocity = b3Vec3_zero;
		m_didStep = false;
		m_stepPosition = b3Pos_zero;

		// Create dynamic body with all rotation locked
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = position;
		m_massCenterWorld = bodyDef.position;
		bodyDef.motionLocks.angularX = true;
		bodyDef.motionLocks.angularY = true;
		bodyDef.motionLocks.angularZ = true;
		bodyDef.enableSleep = false;
		bodyDef.enableContactRecycling = false;
		bodyDef.name = "character";

		bodyDef.gravityScale = m_characterGravity / 10.0f;

		m_bodyId = b3CreateBody( sample->m_worldId, &bodyDef );

		// --- Feet box (lower half): dynamic friction for braking/sliding ---
		// s&box BodyBox: half-extent = BodyRadius * 0.5 (box width = BodyRadius)
		{
			float halfExtX = m_bodyRadius * 0.5f;
			float halfExtY = m_feetHeight * 0.5f;
			float halfExtZ = m_bodyRadius * 0.5f;

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.baseMaterial.friction = 0.0f;
			shapeDef.baseMaterial.restitution = 0.0f;
			shapeDef.baseMaterial.customColor = b3_colorLimeGreen;

			float feetVolume = 8.0f * halfExtX * halfExtY * halfExtZ;
			shapeDef.density = ( m_characterMass * 0.4f ) / feetVolume;

			b3Transform feetTransform = { { 0.0f, -m_totalHeight * 0.5f + halfExtY, 0.0f }, b3Quat_identity };
			b3BoxHull feetBox = b3MakeTransformedBoxHull( halfExtX, halfExtY, halfExtZ, feetTransform );
			m_feetBoxId = b3CreateHullShape( m_bodyId, &shapeDef, &feetBox.base );
		}

		// --- Body capsule (upper half): zero friction so we slide on walls ---
		{
			float capsuleRadius = m_bodyRadius * 0.707f;
			float capsuleBottom = -m_totalHeight * 0.5f + m_feetHeight * 0.5f + capsuleRadius;
			float capsuleTop = m_totalHeight * 0.5f - capsuleRadius;

			if ( capsuleTop > capsuleBottom )
			{
				b3Capsule capsule = {
					{ 0.0f, capsuleBottom, 0.0f },
					{ 0.0f, capsuleTop, 0.0f },
					capsuleRadius,
				};

				b3ShapeDef shapeDef = b3DefaultShapeDef();
				shapeDef.baseMaterial.friction = 0.0f;
				shapeDef.baseMaterial.restitution = 0.0f;
				shapeDef.baseMaterial.customColor = b3_colorCornflowerBlue;

				float h = capsuleTop - capsuleBottom;
				float r = capsuleRadius;
				float capsuleVolume = B3_PI * r * r * ( h + 4.0f * r / 3.0f );
				shapeDef.density = ( m_characterMass * 0.6f ) / capsuleVolume;

				m_bodyCapsuleId = b3CreateCapsuleShape( m_bodyId, &shapeDef, &capsule );
			}
		}

		// Cache own shapes for ignore list
		m_ownShapeCount = b3Body_GetShapes( m_bodyId, m_ownShapes, 4 );

		UpdateMassCenter( 0.0f );
	}

	// --- TraceBody: box shape cast matching s&box's TraceBody ---
	// Casts a box from `from` to `to` with given radius and height scale.
	TraceResult TraceBody( b3Pos from, b3Pos to, float radiusScale = 1.0f, float heightScale = 1.0f ) const
	{
		TraceResult result = {};
		result.endPosition = to;
		result.normal = b3Vec3_axisY;
		result.hitPoint = to;
		result.fraction = 1.0f;
		result.hit = false;
		result.startedSolid = false;

		b3Vec3 translation = to - from;
		float translationLen = b3Length( translation );
		if ( translationLen < 1e-6f )
		{
			return result;
		}

		// Build box half extents matching s&box's BodyBox
		float halfW = m_bodyRadius * 0.5f * radiusScale;
		float halfH = m_totalHeight * heightScale * 0.5f;
		float halfD = m_bodyRadius * 0.5f * radiusScale;

		// 8 corners of the box centered at `from`, with box bottom at from.y
		// s&box BodyBox: min=(−r*0.5*s, −r*0.5*s, 0), max=(r*0.5*s, r*0.5*s, height*hScale)
		// In box3d Y is up, so min.y=0 (feet), max.y=height*hScale (top)
		float boxMinY = 0.0f;
		float boxMaxY = m_totalHeight * heightScale;
		float boxCenterY = ( boxMinY + boxMaxY ) * 0.5f;

		b3Vec3 points[8];
		for ( int i = 0; i < 8; ++i )
		{
			float sx = ( i & 1 ) ? halfW : -halfW;
			float sy = ( i & 2 ) ? halfH : -halfH;
			float sz = ( i & 4 ) ? halfD : -halfD;
			points[i] = { sx, boxCenterY + sy, sz };
		}

		b3ShapeProxy proxy;
		proxy.points = points;
		proxy.count = 8;
		proxy.radius = 0.0f;

		ClosestShapeCastContext ctx = {};
		for ( int i = 0; i < m_ownShapeCount; ++i )
		{
			ctx.ignoreShapes[i] = m_ownShapes[i];
		}
		ctx.ignoreCount = m_ownShapeCount;
		ctx.closestFraction = 1.0f;
		ctx.hit = false;
		ctx.startedSolid = false;
		ctx.closestShape = b3_nullShapeId;

		b3QueryFilter filter = b3DefaultQueryFilter();
		b3World_CastShape( m_sample->m_worldId, from, &proxy, translation, filter, ClosestShapeCastCallback, &ctx );

		result.startedSolid = ctx.startedSolid;
		if ( ctx.hit )
		{
			result.hit = true;
			result.fraction = ctx.closestFraction;
			result.normal = ctx.closestNormal;
			result.hitPoint = ctx.closestPoint;
			result.endPosition = from + ctx.closestFraction * translation;
		}

		return result;
	}

	bool IsStandableSurface( b3Vec3 normal ) const
	{
		float maxSlopeCos = cosf( m_maxSlopeAngle * B3_PI / 180.0f );
		return b3Dot( normal, b3Vec3_axisY ) >= maxSlopeCos;
	}

	// Get feet position from body center position
	b3Pos GetFeetPosition() const
	{
		b3Pos pos = b3Body_GetPosition( m_bodyId );
		return { pos.x, pos.y - m_totalHeight * 0.5f, pos.z };
	}

	// --- CategorizeGround: s&box-style box cast with radius shrinking ---
	void CategorizeGround()
	{
		b3Pos feet = GetFeetPosition();

		// s&box: from = WorldPosition + Up*4, to = WorldPosition + Down*2 (Source units)
		b3Pos from = { feet.x, feet.y + 4.0f * SRC, feet.z };
		b3Pos to = { feet.x, feet.y - 2.0f * SRC, feet.z };

		float radiusScale = 1.0f;
		TraceResult tr = TraceBody( from, to, radiusScale, 0.5f );

		// Shrink radius if started solid or hit non-standable surface
		while ( tr.startedSolid || ( tr.hit && !IsStandableSurface( tr.normal ) ) )
		{
			radiusScale -= 0.1f;
			if ( radiusScale < 0.7f )
			{
				UpdateGround( false, b3Vec3_axisY );
				DrawLine( from, to, MakeColor( b3_colorRed ) );
				return;
			}
			tr = TraceBody( from, to, radiusScale, 0.5f );
		}

		if ( !tr.startedSolid && tr.hit && IsStandableSurface( tr.normal ) && m_jumpCooldown <= 0.0f )
		{
			UpdateGround( true, tr.normal );
			DrawLine( from, tr.hitPoint, MakeColor( b3_colorGreen ) );
			DrawPoint( tr.hitPoint, 5.0f, MakeColor( b3_colorGreen ) );
		}
		else
		{
			UpdateGround( false, b3Vec3_axisY );
			DrawLine( from, to, MakeColor( b3_colorGray ) );
		}
	}

	void UpdateGround( bool onGround, b3Vec3 normal )
	{
		m_onGround = onGround;
		m_groundNormal = normal;
		if ( !onGround )
		{
			m_groundVelocity = b3Vec3_zero;
		}
	}

	// --- Reground / StickToGround: snap character to surface when on ground ---
	void Reground( float stepSize )
	{
		if ( !m_onGround )
		{
			return;
		}

		b3Pos pos = b3Body_GetPosition( m_bodyId );

		b3Pos from = { pos.x, pos.y + 0.05f, pos.z };
		b3Pos to = { pos.x, pos.y - stepSize, pos.z };

		float radiusScale = 1.0f;
		TraceResult tr = TraceBody( from, to, radiusScale, 0.5f );

		while ( tr.startedSolid )
		{
			radiusScale -= 0.1f;
			if ( radiusScale < 0.7f )
			{
				return;
			}
			tr = TraceBody( from, to, radiusScale, 0.5f );
		}

		if ( tr.hit )
		{
			b3Pos targetPos = { tr.endPosition.x, tr.endPosition.y + 0.01f, tr.endPosition.z };
			float deltaY = targetPos.y - pos.y;

			b3Quat rot = b3Body_GetRotation( m_bodyId );
			b3Body_SetTransform( m_bodyId, targetPos, rot );

			// If we moved upward, kill vertical velocity to prevent bouncing
			if ( deltaY > 0.01f )
			{
				b3Vec3 vel = b3Body_GetLinearVelocity( m_bodyId );
				vel.y = 0.0f;
				b3Body_SetLinearVelocity( m_bodyId, vel );
			}

			DrawLine( from, tr.endPosition, MakeColor( b3_colorCyan ) );
		}
	}

	// --- TryStep: 4-phase trace-based step-up algorithm ---
	// Returns true if a step was taken and m_stepPosition was set.
	bool TryStep( float maxStepHeight )
	{
		b3Pos pos = b3Body_GetPosition( m_bodyId );
		b3Vec3 vel = b3Body_GetLinearVelocity( m_bodyId );

		// Only step when on ground and moving
		if ( !m_onGround )
		{
			return false;
		}

		// Horizontal velocity direction
		b3Vec3 hVel = { vel.x, 0.0f, vel.z };
		float hSpeed = b3Length( hVel );
		if ( hSpeed < 0.01f )
		{
			return false;
		}
		b3Vec3 moveDir = { hVel.x / hSpeed, 0.0f, hVel.z / hSpeed };

		// Phase 1 — FORWARD: trace body forward in velocity direction
		// Start slightly behind (offset by skin)
		float forwardDist = hSpeed * ( 1.0f / 60.0f ) + m_bodyRadius; // one frame of movement + radius
		b3Pos forwardFrom = pos - m_skin * moveDir;
		b3Pos forwardTo = pos + forwardDist * moveDir;

		float radiusScale = 1.0f;
		TraceResult trForward = TraceBody( forwardFrom, forwardTo, radiusScale );

		while ( trForward.startedSolid )
		{
			radiusScale -= 0.1f;
			if ( radiusScale < 0.6f )
			{
				DrawLine( forwardFrom, forwardTo, MakeColor( b3_colorRed ) );
				return false;
			}
			trForward = TraceBody( forwardFrom, forwardTo, radiusScale );
		}

		if ( !trForward.hit )
		{
			// No obstacle ahead, no step needed
			return false;
		}

		DrawLine( forwardFrom, trForward.endPosition, MakeColor( b3_colorYellow ) );

		// Remaining velocity direction after hit
		b3Pos hitPos = trForward.endPosition;

		// Phase 2 — UP: trace straight up from hit position
		b3Pos upFrom = hitPos;
		b3Pos upTo = { hitPos.x, hitPos.y + maxStepHeight, hitPos.z };
		TraceResult trUp = TraceBody( upFrom, upTo, radiusScale );

		if ( trUp.startedSolid )
		{
			DrawLine( upFrom, upTo, MakeColor( b3_colorRed ) );
			return false;
		}

		b3Pos topPos = trUp.hit ? trUp.endPosition : upTo;
		float upDistance = topPos.y - upFrom.y;
		if ( upDistance < 0.005f )
		{
			// Too tight to step up
			DrawLine( upFrom, topPos, MakeColor( b3_colorRed ) );
			return false;
		}

		DrawLine( upFrom, topPos, MakeColor( b3_colorYellow ) );

		// Phase 3 — ACROSS: from top position, trace in move direction
		float acrossDist = forwardDist * ( 1.0f - trForward.fraction ) + m_bodyRadius * 0.5f;
		b3Pos acrossFrom = topPos;
		b3Pos acrossTo = topPos + acrossDist * moveDir;
		TraceResult trAcross = TraceBody( acrossFrom, acrossTo, radiusScale );

		if ( trAcross.startedSolid )
		{
			DrawLine( acrossFrom, acrossTo, MakeColor( b3_colorRed ) );
			return false;
		}

		b3Pos acrossPos = trAcross.hit ? trAcross.endPosition : acrossTo;
		DrawLine( acrossFrom, acrossPos, MakeColor( b3_colorYellow ) );

		// Phase 4 — DOWN: from across position, trace straight down
		b3Pos downFrom = acrossPos;
		b3Pos downTo = { acrossPos.x, acrossPos.y - maxStepHeight, acrossPos.z };
		TraceResult trDown = TraceBody( downFrom, downTo, radiusScale );

		if ( !trDown.hit )
		{
			DrawLine( downFrom, downTo, MakeColor( b3_colorRed ) );
			return false;
		}

		if ( !IsStandableSurface( trDown.normal ) )
		{
			DrawLine( downFrom, trDown.endPosition, MakeColor( b3_colorRed ) );
			return false;
		}

		// Check we actually stepped up (not just laterally)
		float stepHeight = trDown.endPosition.y - pos.y;
		if ( stepHeight < 0.01f )
		{
			return false;
		}

		DrawLine( downFrom, trDown.endPosition, MakeColor( b3_colorYellow ) );
		DrawPoint( trDown.endPosition, 8.0f, MakeColor( b3_colorYellow ) );

		// Teleport body to step position
		b3Pos stepPos = { trDown.endPosition.x, trDown.endPosition.y + 0.01f, trDown.endPosition.z };
		b3Quat rot = b3Body_GetRotation( m_bodyId );
		b3Body_SetTransform( m_bodyId, stepPos, rot );

		// Kill vertical velocity, scale horizontal by 0.9
		b3Vec3 newVel = b3Body_GetLinearVelocity( m_bodyId );
		newVel.x *= 0.9f;
		newVel.y = 0.0f;
		newVel.z *= 0.9f;
		b3Body_SetLinearVelocity( m_bodyId, newVel );

		m_stepPosition = stepPos;
		return true;
	}

	void RestoreStep()
	{
		if ( !m_didStep )
		{
			return;
		}

		// After physics, restore to step position to prevent double-velocity
		b3Quat rot = b3Body_GetRotation( m_bodyId );
		b3Body_SetTransform( m_bodyId, m_stepPosition, rot );
		m_didStep = false;
	}

	// --- AddClamped: add vector but cap the addition's magnitude ---
	static b3Vec3 AddClamped( b3Vec3 current, b3Vec3 add, float maxAddLength )
	{
		float addLen = b3Length( add );
		if ( addLen > maxAddLength && addLen > 0.0f )
		{
			add = ( maxAddLength / addLen ) * add;
		}
		return current + add;
	}

	// --- UpdateMassCenter: s&box formula ---
	void UpdateMassCenter( float wishSpeed )
	{
		b3MassData massData = b3Body_GetMassData( m_bodyId );
		float halfHeight = m_totalHeight * 0.5f;

		if ( m_onGround )
		{
			// s&box: massCenter = clamp(WishSpeed, 0, halfHeight)
			float centerOffset = b3ClampFloat( wishSpeed, 0.0f, halfHeight );
			massData.center = { 0.0f, centerOffset - halfHeight, 0.0f };
		}
		else
		{
			massData.center = { 0.0f, 0.0f, 0.0f };
		}

		b3Body_SetMassData( m_bodyId, massData );
	}

	// --- UpdateBody: set friction, gravity, damping per s&box ---
	void UpdateBody( b3Vec3 wishVelocity )
	{
		float wishLen = b3Length( wishVelocity );
		b3Vec3 vel = b3Body_GetLinearVelocity( m_bodyId );
		float velLen = b3Length( vel );

		// Feet friction — s&box: wantsBrakes when wish < 5 Source/s or wish < vel * 0.9
		float feetFriction = 0.0f;
		if ( m_onGround )
		{
			bool wantsBrakes = wishLen < ( 5.0f * SRC ) || wishLen < velLen * 0.9f;
			if ( wantsBrakes )
			{
				// s&box: 1 + 100 * BrakePower * GroundFriction = 1 + 100 * 0.2 * 0.6 = 13.0
				feetFriction = 1.0f + 100.0f * m_brakePower * m_surfaceFriction;
			}
		}
		b3Shape_SetFriction( m_feetBoxId, feetFriction );

		// Mass center
		UpdateMassCenter( wishLen );

		// Gravity: s&box disables gravity when stationary on stable ground
		// s&box: wantsGravity if !onGround || velocity > 1 Source/s || groundVel > 1 Source/s
		bool wantsGravity = false;
		if ( !m_onGround )
			wantsGravity = true;
		if ( velLen > ( 1.0f * SRC ) )
			wantsGravity = true;
		if ( b3Length( m_groundVelocity ) > ( 1.0f * SRC ) )
			wantsGravity = true;
		b3Body_SetGravityScale( m_bodyId, wantsGravity ? ( m_characterGravity / 10.0f ) : 0.0f );

		// Linear damping — s&box: brakes when wish < 1 unit/s && groundVel < 1 unit/s
		bool wantsDamping = m_onGround && wishLen < ( 1.0f * SRC ) && b3Length( m_groundVelocity ) < ( 1.0f * SRC );
		b3Body_SetLinearDamping( m_bodyId, wantsDamping ? 10.0f * m_brakePower : m_airFriction );
	}

	// --- AddVelocity: s&box's MoveMode.Walk velocity model ---
	void AddVelocity( b3Vec3 wishVelocity )
	{
		// Walk mode strips vertical component
		b3Vec3 wish = { wishVelocity.x, 0.0f, wishVelocity.z };
		float wishLen = b3Length( wish );
		if ( wishLen < 0.001f )
		{
			return;
		}

		float groundFrictionFactor = 0.25f + m_surfaceFriction * 10.0f;
		b3Vec3 vel = b3Body_GetLinearVelocity( m_bodyId );
		float savedY = vel.y;

		b3Vec3 velocity = vel - m_groundVelocity;
		float speed = b3Length( velocity );
		float maxSpeed = b3MaxFloat( wishLen, speed );

		if ( m_onGround )
		{
			float amount = 1.0f * groundFrictionFactor;
			velocity = AddClamped( velocity, amount * wish, wishLen * amount );
		}
		else
		{
			float amount = 0.05f;
			velocity = AddClamped( velocity, amount * wish, wishLen );
		}

		// Cap at max speed
		float newSpeed = b3Length( velocity );
		if ( newSpeed > maxSpeed && newSpeed > 0.0f )
		{
			velocity = ( maxSpeed / newSpeed ) * velocity;
		}

		velocity = velocity + m_groundVelocity;
		if ( m_onGround )
		{
			velocity.y = savedY;
		}

		b3Body_SetLinearVelocity( m_bodyId, velocity );
	}

	// --- PreStep: UpdateBody + AddVelocity + TryStep ---
	void PreStep( float timeStep, b3Vec3 forward, b3Vec3 right, b3Vec2 throttle )
	{
		if ( m_jumpCooldown > 0.0f )
		{
			m_jumpCooldown -= timeStep;
		}

		// Compute wish velocity from input
		float maxSpeed = m_sprint ? m_runSpeed : m_walkSpeed;
		b3Vec3 wishVelocity = maxSpeed * throttle.x * forward + maxSpeed * throttle.y * right;
		float wishSpeed = b3Length( wishVelocity );
		if ( wishSpeed > maxSpeed )
		{
			wishVelocity = ( maxSpeed / wishSpeed ) * wishVelocity;
		}
		m_lastWishVelocity = wishVelocity;

		// 1. UpdateBody — set friction, mass center, gravity, damping
		UpdateBody( wishVelocity );

		// 2. AddVelocity — apply wish velocity (s&box model)
		AddVelocity( wishVelocity );

		// 3. TryStep — 4-phase step-up
		m_didStep = TryStep( m_stepUpHeight );

		m_massCenterWorld = b3Body_GetWorldCenterOfMass( m_bodyId );
	}

	// --- PostStep: RestoreStep + Reground + CategorizeGround ---
	void PostStep( float timeStep )
	{
		// 1. RestoreStep — teleport back to step position if we stepped
		RestoreStep();

		// 2. Reground / StickToGround
		Reground( m_stepDownHeight );

		// 3. CategorizeGround
		CategorizeGround();
	}

	void Jump()
	{
		if ( m_onGround && m_jumpCooldown <= 0.0f )
		{
			b3Vec3 velocity = b3Body_GetLinearVelocity( m_bodyId );
			velocity.y = m_jumpSpeed;
			b3Body_SetLinearVelocity( m_bodyId, velocity );
			m_onGround = false;
			m_jumpCooldown = m_jumpCooldownTime;
		}
	}

	void Step( float timeStep, b3Vec3 forward, b3Vec3 right, b3Vec2 throttle )
	{
		PreStep( timeStep, forward, right, throttle );
	}

	void LateStep( float timeStep )
	{
		PostStep( timeStep );
	}

	void DrawDebug() const
	{
		b3Pos pos = b3Body_GetPosition( m_bodyId );
		b3Vec3 vel = b3Body_GetLinearVelocity( m_bodyId );

		// Draw velocity vector (purple)
		DrawLine( pos, b3OffsetPos( pos, vel ), MakeColor( b3_colorPurple ) );

		// Draw wish velocity (orange)
		DrawLine( pos, b3OffsetPos( pos, m_lastWishVelocity ), MakeColor( b3_colorOrange ) );

		// Draw mass center (yellow dot)
		DrawPoint( m_massCenterWorld, 8.0f, MakeColor( b3_colorYellow ) );

		// Draw ground indicator
		if ( m_onGround )
		{
			b3Pos bottom = { pos.x, pos.y - m_totalHeight * 0.5f, pos.z };
			DrawLine( bottom, b3OffsetPos( bottom, 0.3f * m_groundNormal ), MakeColor( b3_colorGreen ) );
		}
	}
};

class RigidBodyCharacter : public Sample
{
public:
	explicit RigidBodyCharacter( SampleContext* context )
		: Sample( context )
	{
		b3Pos startPosition = { 7.5f, 2.0f, 9.0f };

		if ( m_context->restart == false )
		{
			m_camera->SetView( 120.0f, 30.0f, 5.0f, startPosition );
		}

		m_character.Initialize( this, startPosition );

		// --- Main level mesh (test_map01) ---
		{
			m_levelMesh = CreateMeshData( "data/meshes/test_map01.obj", 1.0f, false, false, true, true );
			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3SurfaceMaterial materials[3];
			materials[0] = { 0.6f, 0.0f, 0 };
			materials[1] = { 0.6f, 1.0f, 1 };
			materials[2] = { 0.1f, 0.0f, 2 };
			shapeDef.materials = materials;
			shapeDef.materialCount = 3;

			b3CreateMeshShape( body, &shapeDef, m_levelMesh, b3Vec3_one );
		}

		// --- Stairs mesh ---
		{
			m_stairs = CreateMeshData( "data/meshes/stairs.obj", 1.0f, false, false, true, true );
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { -10.0f, 0.0f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateMeshShape( body, &shapeDef, m_stairs, { 0.75f, 0.75f, -1.5f } );
		}

		// --- High-poly building mesh ---
		{
			m_building = CreateMeshData( "data/meshes/building.obj", 1.0f, false, false, true, true );
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { -5.0f, 0.0f, -10.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateMeshShape( body, &shapeDef, m_building, b3Vec3_one );
		}

		// --- Voxel meshes (dense tri terrain) ---
		{
			m_voxel01 = CreateMeshData( "data/meshes/voxel_mesh_01.obj", 1.0f, false, false, true, true );
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 10.0f, 0.0f, -10.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateMeshShape( body, &shapeDef, m_voxel01, b3Vec3_one );
		}
		{
			m_voxel02 = CreateMeshData( "data/meshes/voxel_mesh_02.obj", 1.0f, false, false, true, true );
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 10.0f, 0.0f, 10.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateMeshShape( body, &shapeDef, m_voxel02, b3Vec3_one );
		}

		// --- Height field terrain ---
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 20.0f, 0.0f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );

			m_heightField = b3CreateWave( 50.0f, 50.0f, b3Vec3_one, 0.02f, 0.04f, true );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3SurfaceMaterial materials[3];
			materials[0] = { 0.6f, 0.0f, 0 };
			materials[1] = { 0.6f, 1.0f, 1 };
			materials[2] = { 0.1f, 0.0f, 2 };
			shapeDef.materials = materials;
			shapeDef.materialCount = 3;

			b3CreateHeightFieldShape( body, &shapeDef, m_heightField );
		}

		// --- Hull obstacles on top of mesh level ---
		b3ShapeDef hullShapeDef = b3DefaultShapeDef();
		hullShapeDef.baseMaterial.friction = 0.6f;

		// Ramp (tilted box)
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 6.0f, 1.0f, 4.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, -20.0f * B3_DEG_TO_RAD );
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			hullShapeDef.baseMaterial.customColor = b3_colorOliveDrab;
			b3BoxHull box = b3MakeBoxHull( 3.0f, 0.15f, 1.5f );
			b3CreateHullShape( body, &hullShapeDef, &box.base );
		}

		// Steep ramp (too steep to stand on)
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 6.0f, 2.0f, -4.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, -50.0f * B3_DEG_TO_RAD );
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			hullShapeDef.baseMaterial.customColor = b3_colorIndianRed;
			b3BoxHull box = b3MakeBoxHull( 2.5f, 0.15f, 1.5f );
			b3CreateHullShape( body, &hullShapeDef, &box.base );
		}

		// Elevated platforms with gaps
		for ( int i = 0; i < 3; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { -4.0f + 3.5f * i, 1.2f, -5.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			hullShapeDef.baseMaterial.customColor = b3_colorSlateGray;
			b3BoxHull box = b3MakeBoxHull( 1.2f, 0.15f, 1.2f );
			b3CreateHullShape( body, &hullShapeDef, &box.base );
		}

		// Step-height test (increasing lip heights)
		for ( int i = 0; i < 5; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			float lipHeight = 0.05f + 0.08f * i;
			bodyDef.position = { -8.0f, lipHeight, -1.0f + 2.0f * i };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			hullShapeDef.baseMaterial.customColor = b3_colorCornflowerBlue;
			b3BoxHull box = b3MakeBoxHull( 1.0f, lipHeight, 0.6f );
			b3CreateHullShape( body, &hullShapeDef, &box.base );
		}

		// Wall
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, 1.5f, 10.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			hullShapeDef.baseMaterial.customColor = b3_colorDarkSlateGray;
			b3BoxHull box = b3MakeBoxHull( 4.0f, 1.5f, 0.2f );
			b3CreateHullShape( body, &hullShapeDef, &box.base );
		}

		// Dynamic boxes to push around
		for ( int i = 0; i < 3; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 3.0f + 1.5f * i, 0.5f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			hullShapeDef.baseMaterial.customColor = b3_colorGold;
			b3BoxHull box = b3MakeBoxHull( 0.4f, 0.4f, 0.4f );
			b3CreateHullShape( body, &hullShapeDef, &box.base );
		}

		// Dynamic sphere
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { -3.0f, 1.0f, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			hullShapeDef.baseMaterial.customColor = b3_colorOrange;
			b3Sphere sphere = { b3Vec3_zero, 0.5f };
			b3CreateSphereShape( body, &hullShapeDef, &sphere );
		}

		m_camera->m_thirdPerson = true;
		m_showDebug = true;
		sapp_lock_mouse( true );
	}

	~RigidBodyCharacter() override
	{
		m_camera->m_thirdPerson = false;
		sapp_lock_mouse( false );
		b3DestroyMesh( m_levelMesh );
		b3DestroyMesh( m_stairs );
		b3DestroyMesh( m_building );
		b3DestroyMesh( m_voxel01 );
		b3DestroyMesh( m_voxel02 );
		b3DestroyHeightField( m_heightField );
	}

	void Keyboard( int key, int action, int mods ) override
	{
		if ( key == KEY_T && action == ACTION_PRESS )
		{
			ToggleThirdPerson();
		}

		if ( key == KEY_V && action == ACTION_PRESS )
		{
			m_showDebug = !m_showDebug;
		}
	}

	void Step() override
	{
		float hertz = m_context->hertz;
		float timeStep = hertz > 0.0f ? 1.0f / hertz : 0.0f;

		// Read input
		b3Vec2 throttle = { 0.0f, 0.0f };
		b3Vec3 forward = -m_camera->GetForward();
		b3Vec3 right = m_camera->GetRight();
		forward.y = 0.0f;

		// Normalize forward to horizontal plane
		float forwardLen = b3Length( forward );
		if ( forwardLen > 0.001f )
		{
			forward *= 1.0f / forwardLen;
		}

		if ( m_camera->m_thirdPerson )
		{
			if ( IsKeyDown( KEY_W ) )
			{
				throttle.x += 1.0f;
			}
			if ( IsKeyDown( KEY_S ) )
			{
				throttle.x -= 1.0f;
			}
			if ( IsKeyDown( KEY_A ) )
			{
				throttle.y -= 1.0f;
			}
			if ( IsKeyDown( KEY_D ) )
			{
				throttle.y += 1.0f;
			}

			if ( IsKeyDown( KEY_SPACE ) )
			{
				m_character.Jump();
			}

			m_character.m_sprint = m_character.m_onGround && IsKeyDown( KEY_LEFT_SHIFT );
		}

		// Pre-step: manipulate velocity before physics
		m_character.Step( timeStep, forward, right, throttle );

		// Run physics
		Sample::Step();

		// Post-step: re-categorize ground and apply corrections
		m_character.LateStep( timeStep );

		// Follow the character with the camera.
		b3Pos charPos = b3Body_GetPosition( m_character.m_bodyId );
		b3Pos pos = charPos;
		if ( m_camera->m_thirdPerson )
		{
			m_camera->m_pivot = charPos;
			m_camera->UpdateTransform();

			// Keep the eye from clipping through geometry. Cast from the character toward the eye
			// and, on a hit, shorten the boom for this frame only. Restoring the radius lets the
			// next frame start from the user's chosen distance instead of ratcheting inward.
			float cameraRadius = 0.15f;
			b3Vec3 translation = b3SubPos( m_camera->m_worldEye, charPos );
			float desiredDist = b3Length( translation );

			if ( desiredDist > 0.01f )
			{
				b3QueryFilter filter = b3DefaultQueryFilter();
				b3RayResult rayResult = b3World_CastRayClosest( m_worldId, charPos, translation, filter );

				if ( rayResult.hit )
				{
					float clampedDist = rayResult.fraction * desiredDist - cameraRadius;
					if ( clampedDist < 0.1f )
						clampedDist = 0.1f;

					float savedRadius = m_camera->m_radius;
					m_camera->m_radius = b3MinFloat( savedRadius, clampedDist );
					m_camera->UpdateTransform();
					m_camera->m_radius = savedRadius;
				}
			}
		}

		// Latch the draw origin to the followed eye so the debug overlays demote against the same
		// point the view renders from.
		SetDrawOrigin( m_camera->DrawOrigin() );

		// Debug visualization
		if ( m_showDebug )
		{
			m_character.DrawDebug();
		}

		// HUD text
		b3Vec3 vel = b3Body_GetLinearVelocity( m_character.m_bodyId );
		float speed = sqrtf( vel.x * vel.x + vel.z * vel.z );
		DrawTextLine( "Rigid Body Character (s&box-style)" );
		DrawTextLine( "position: %.2f %.2f %.2f", pos.x, pos.y, pos.z );
		DrawTextLine( "velocity: %.2f %.2f %.2f (horizontal: %.2f)", vel.x, vel.y, vel.z, speed );
		DrawTextLine( "on ground: %s | sprint: %s", m_character.m_onGround ? "yes" : "no", m_character.m_sprint ? "yes" : "no" );
		DrawTextLine( "WASD=move Space=jump Shift=sprint T=camera V=debug" );
	}

	void Render() override
	{
		Sample::Render();
		DrawAxes( { { 0.0f, 0.0f, 0.02f }, b3Quat_identity }, 2.0f );
	}

	bool DrawControls() override
	{
		bool thirdPerson = m_camera->m_thirdPerson;
		if ( ImGui::Checkbox( "Third Person (T)", &thirdPerson ) )
		{
			ToggleThirdPerson();
		}

		ImGui::Checkbox( "Debug (V)", &m_showDebug );

		ImGui::Separator();
		ImGui::Text( "Ground: %s", m_character.m_onGround ? "YES" : "NO" );

		b3Vec3 vel = b3Body_GetLinearVelocity( m_character.m_bodyId );
		float hSpeed = sqrtf( vel.x * vel.x + vel.z * vel.z );
		ImGui::Text( "Speed: %.2f m/s", hSpeed );
		ImGui::Text( "Vertical: %.2f m/s", vel.y );

		b3Pos mc = m_character.m_massCenterWorld;
		b3Pos pos = b3Body_GetPosition( m_character.m_bodyId );
		ImGui::Text( "Mass center offset: %.2f", mc.y - pos.y );

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new RigidBodyCharacter( context );
	}

	RigidbodyCharacter m_character;
	b3MeshData* m_levelMesh;
	b3MeshData* m_stairs;
	b3MeshData* m_building;
	b3MeshData* m_voxel01;
	b3MeshData* m_voxel02;
	b3HeightFieldData* m_heightField;
	bool m_showDebug;
};

static int sampleRBCharacter = RegisterSample( "Character", "Rigid Body", RigidBodyCharacter::Create );
