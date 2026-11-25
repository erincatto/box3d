// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/draw.h"
#include "gfx/keycodes.h"
#include "gfx/utility.h"
#include "imgui.h"
#include "sample.h"
#include "utils.h"
#include "wagon.h"

#include "box3d/box3d.h"

#include <math.h>
#include <vector>

// A horse drawn wagon on tire joints. The chassis is the only body. Each wheel casts a disc at the
// ground and, where it finds some, holds a tire joint between that ground body and the chassis. The
// front axle is a turntable that swings about the kingbolt. The horse is a box standing on the ground
// between the shafts, which hinge at the front gear and rest on its back. It pulls with a capped force
// at the kingbolt along the shafts.
//
// W/S change gait, A/D steer, Space brakes. With the gait at halt the brake is on. S at halt backs the
// wagon. T toggles a camera that follows the wagon.
//
// The axles are capsules on the chassis, so a tipped wagon lies on its running gear. The front axle
// capsule stays straight ahead on the body and does not swing with the turntable. Rebuilding it
// while steering would churn the shapes every step.
//
// The wagon itself lives in shared/wagon.c so the vehicle unit tests run the same code.
class WagonSample : public Sample
{
public:
	static constexpr uint64_t m_wagonBit = 0x4000000000000000ull;

	explicit WagonSample( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 20.0f, 20.0f, 12.0f, { 0.0f, 1.0f, 0.0f } );
		}

		m_tuning = GetDefaultWagonTuning();
		m_gait = e_wagonHalt;
		m_steerInput = 0.0f;
		m_mass = 700.0f;

		AddGroundBox( 200.0f );
		CreateSlopes();
		CreateCurb();
		CreateRockField();
		CreateLooseBoxes();
		CreateWaveTerrain();
		CreateLogs();

		CreateWagonBody();

		// Cylinders along their y axis, which the wheel transform turns onto the axle
		const WagonWheel& front = m_wagon.wheels[e_wagonFrontLeft];
		const WagonWheel& rear = m_wagon.wheels[e_wagonRearLeft];
		m_frontWheelHull = b3CreateCylinder( 2.0f * front.halfWidth, front.radius, -front.halfWidth, 16 );
		m_rearWheelHull = b3CreateCylinder( 2.0f * rear.halfWidth, rear.radius, -rear.halfWidth, 16 );
	}

	~WagonSample() override
	{
		b3DestroyHull( m_frontWheelHull );
		b3DestroyHull( m_rearWheelHull );
		b3DestroyMesh( m_waveMesh );

		if ( m_camera->m_thirdPerson )
		{
			ToggleThirdPerson();
		}
	}

	void CreateWagonBody()
	{
		WagonDef def = GetDefaultWagonDef();
		def.position = { 0.0f, 0.1f, 0.0f };
		def.mass = m_mass;
		def.categoryBits = m_wagonBit;
		def.tuning = m_tuning;
		m_wagon = CreateWagon( m_worldId, &def );
	}

	// Ramps up to a plateau and back down, side by side. A 10 percent grade, then steeper up to 45 degrees.
	void CreateSlopes()
	{
		float degrees = B3_PI / 180.0f;
		float angles[] = { atanf( 0.1f ), 15.0f * degrees, 25.0f * degrees, 35.0f * degrees, 45.0f * degrees };
		for ( int i = 0; i < 5; ++i )
		{
			CreateSlope( angles[i], 20.0f + 14.0f * (float)i );
		}
	}

	void CreateSlope( float angle, float z )
	{
		float rise = 4.0f;
		float run = rise / tanf( angle );
		float halfSlope = 0.5f * sqrtf( run * run + rise * rise );
		float halfDepth = 0.5f;
		float halfWidth = 6.0f;
		float plateau = 20.0f;
		float start = 25.0f;

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		// The slab's top face runs from the ground to the plateau
		b3Vec3 up = { -sinf( angle ), cosf( angle ), 0.0f };
		b3BoxHull slab = b3MakeBoxHull( halfSlope, halfDepth, halfWidth );

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "slope up";
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, angle );
			b3Vec3 center = b3MulSub( { start + 0.5f * run, 0.5f * rise, z }, halfDepth, up );
			bodyDef.position = { center.x, center.y, center.z };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &slab.base );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "plateau";
			bodyDef.position = { start + run + 0.5f * plateau, 0.5f * rise, z };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3BoxHull box = b3MakeBoxHull( 0.5f * plateau, 0.5f * rise, halfWidth );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}

		{
			b3Vec3 downUp = { sinf( angle ), cosf( angle ), 0.0f };
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "slope down";
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, -angle );
			b3Vec3 center = b3MulSub( { start + 1.5f * run + plateau, 0.5f * rise, z }, halfDepth, downUp );
			bodyDef.position = { center.x, center.y, center.z };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &slab.base );
		}
	}

	// A 10 cm step up across the road
	void CreateCurb()
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.name = "curb";
		bodyDef.position = { 15.0f, 0.05f, -15.0f };
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( 10.0f, 0.05f, 6.0f );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	// Small static rocks, about the size a wagon wheel rolls over
	void CreateRockField()
	{
		g_randomSeed = 42;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		for ( int i = 0; i < 80; ++i )
		{
			float x = RandomFloatRange( -45.0f, -25.0f );
			float z = RandomFloatRange( -6.0f, 6.0f );
			float size = RandomFloatRange( 0.06f, 0.14f );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "rock";
			bodyDef.position = { x, 0.25f * size, z };
			b3Vec3 axis = b3Normalize( { RandomFloat(), RandomFloat() + 1.5f, RandomFloat() } );
			bodyDef.rotation = b3MakeQuatFromAxisAngle( axis, RandomFloatRange( -B3_PI, B3_PI ) );
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3BoxHull box = b3MakeBoxHull( size, 0.5f * size, 0.8f * size );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}

	// Light crates a wheel can rest on or push aside
	void CreateLooseBoxes()
	{
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 150.0f;
		b3BoxHull box = b3MakeBoxHull( 0.4f, 0.1f, 0.4f );

		for ( int i = 0; i < 4; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.name = "crate";
			bodyDef.position = { 8.0f + 2.5f * (float)i, 0.1f, ( i & 1 ) ? 0.725f : -0.725f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}

	// Rolling ground south of the start. Two crossed waves on a plateau that ramps up from the flat
	// ground at its border, so the wagon can drive on and off anywhere.
	static constexpr float m_waveLengthX = 60.0f;
	static constexpr float m_waveLengthZ = 30.0f;
	static constexpr float m_waveCell = 0.5f;
	static constexpr float m_waveAmplitude = 1.2f;

	static float WaveHeight( float u, float v )
	{
		float ramp = 5.0f;
		float rx = b3ClampFloat( b3MinFloat( u, m_waveLengthX - u ) / ramp, 0.0f, 1.0f );
		float rz = b3ClampFloat( b3MinFloat( v, m_waveLengthZ - v ) / ramp, 0.0f, 1.0f );
		float edge = rx * rx * ( 3.0f - 2.0f * rx ) * rz * rz * ( 3.0f - 2.0f * rz );

		float twoPi = 2.0f * B3_PI;
		float swell = sinf( twoPi * u / 26.0f ) * sinf( twoPi * v / 18.0f );
		float chop = sinf( twoPi * ( u / 11.0f + v / 14.0f ) );
		return m_waveAmplitude * edge * ( 0.5f + 0.3f * swell + 0.2f * chop );
	}

	void CreateWaveTerrain()
	{
		int xCount = (int)( m_waveLengthX / m_waveCell );
		int zCount = (int)( m_waveLengthZ / m_waveCell );

		std::vector<b3Vec3> vertices;
		vertices.reserve( ( xCount + 1 ) * ( zCount + 1 ) );
		for ( int ix = 0; ix <= xCount; ++ix )
		{
			for ( int iz = 0; iz <= zCount; ++iz )
			{
				float u = m_waveCell * ix;
				float v = m_waveCell * iz;
				vertices.push_back( { u, WaveHeight( u, v ), v } );
			}
		}

		std::vector<int32_t> indices;
		indices.reserve( 6 * xCount * zCount );
		for ( int ix = 0; ix < xCount; ++ix )
		{
			for ( int iz = 0; iz < zCount; ++iz )
			{
				int i00 = ix * ( zCount + 1 ) + iz;
				int i01 = i00 + 1;
				int i10 = i00 + zCount + 1;
				int i11 = i10 + 1;
				indices.insert( indices.end(), { i00, i01, i10, i10, i01, i11 } );
			}
		}

		b3MeshDef meshDef = {};
		meshDef.vertices = vertices.data();
		meshDef.indices = indices.data();
		meshDef.vertexCount = (int)vertices.size();
		meshDef.triangleCount = (int)indices.size() / 3;
		meshDef.useMedianSplit = true;
		m_waveMesh = b3CreateMesh( &meshDef, nullptr, 0 );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.name = "waves";
		bodyDef.position = { -30.0f, 0.0f, -65.0f };
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateMeshShape( bodyId, &shapeDef, m_waveMesh, b3Vec3_one );

		// Rocks on the same body, each centered on the surface so about half of it is buried
		g_randomSeed = 7;
		for ( int i = 0; i < 75; ++i )
		{
			float u = RandomFloatRange( 6.0f, m_waveLengthX - 6.0f );
			float v = RandomFloatRange( 6.0f, m_waveLengthZ - 6.0f );
			float size = RandomFloatRange( 0.1f, 0.35f );

			// About a third are boulders at twice the size
			if ( RandomFloatRange( 0.0f, 1.0f ) < 0.3f )
			{
				size *= 2.0f;
			}
			b3Vec3 radii = { size, RandomFloatRange( 0.5f, 0.9f ) * size, RandomFloatRange( 0.6f, 1.0f ) * size };

			b3Vec3 points[16];
			for ( int j = 0; j < 16; ++j )
			{
				b3Vec3 d = b3Normalize( RandomVec3Uniform( -1.0f, 1.0f ) );
				points[j] = { radii.x * d.x, radii.y * d.y, radii.z * d.z };
			}

			b3HullData* hull = b3CreateHull( points, 16, 16 );
			if ( hull == nullptr )
			{
				continue;
			}

			b3Vec3 axis = b3Normalize( RandomVec3Uniform( -1.0f, 1.0f ) );
			b3Quat rotation = b3MakeQuatFromAxisAngle( axis, RandomFloatRange( -0.4f, 0.4f ) );
			rotation = b3MulQuat( b3MakeQuatFromAxisAngle( b3Vec3_axisY, RandomFloatRange( -B3_PI, B3_PI ) ), rotation );
			b3Transform transform = { { u, WaveHeight( u, v ), v }, rotation };

			b3CreateTransformedHullShape( bodyId, &shapeDef, hull, transform, b3Vec3_one );
			b3DestroyHull( hull );
		}
	}

	// Felled logs left across the road ahead of the crates, thin trimmings up to trunks
	void CreateLogs()
	{
		g_randomSeed = 11;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		for ( int i = 0; i < 10; ++i )
		{
			float radius = RandomFloatRange( 0.06f, 0.22f );
			float halfLength = RandomFloatRange( 1.5f, 3.0f );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.name = "log";
			bodyDef.position = { 30.0f + 3.0f * (float)i + RandomFloatRange( -0.5f, 0.5f ), radius, RandomFloatRange( -0.8f, 0.8f ) };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisY, RandomFloatRange( -0.35f, 0.35f ) );
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

			b3Capsule capsule = { { 0.0f, 0.0f, -halfLength }, { 0.0f, 0.0f, halfLength }, radius };
			b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
		}
	}

	void Keyboard( int key, int action, int mods ) override
	{
		if ( action != ACTION_PRESS )
		{
			return;
		}

		if ( key == KEY_T )
		{
			ToggleThirdPerson();
		}

		if (m_camera->m_thirdPerson)
		{
			if ( key == KEY_W )
			{
				m_gait = (WagonGait)b3MinInt( m_gait + 1, e_wagonGaitCount - 1 );
			}
			else if ( key == KEY_S )
			{
				m_gait = (WagonGait)b3MaxInt( m_gait - 1, e_wagonBack );
			}
		}
		else
		{
			m_gait = e_wagonHalt;
		}
	}

	bool DrawControls() override
	{
		bool changed = false;
		bool rebuild = false;

		// The wheel mounts and axles are placed from the travel, so a new travel rebuilds the wagon
		rebuild |= ImGui::SliderFloat( "Travel", &m_tuning.suspensionTravel, 0.01f, 0.4f, "%.3f m" );
		ImGui::SetItemTooltip( "Suspension travel from full compression to full extension. Rebuilds the wagon." );
		changed |= ImGui::SliderFloat( "Rest Length", &m_tuning.suspensionRestLength, 0.0f, 0.5f, "%.3f m" );
		ImGui::SetItemTooltip( "Spring length at zero force, measured like the travel. Longer than the travel preloads "
							   "the spring, so the wagon rides higher on a softer spring." );
		changed |= ImGui::SliderFloat( "Spring Hertz", &m_tuning.suspensionHertz, 0.5f, 10.0f, "%.1f" );
		ImGui::SetItemTooltip( "Suspension spring stiffness as a natural frequency. Lower is softer over bumps." );
		changed |= ImGui::SliderFloat( "Rebound Damping", &m_tuning.suspensionDampingRatio, 0.0f, 2.0f, "%.2f" );
		ImGui::SetItemTooltip( "Damping ratio while the wheel extends. Higher stops the wagon bouncing after a bump." );
		changed |= ImGui::SliderFloat( "Bump Damping", &m_tuning.bumpDampingRatio, 0.0f, 2.0f, "%.2f" );
		ImGui::SetItemTooltip( "Damping ratio while the wheel compresses. Lower lets a wheel climb an edge without "
							   "slamming the chassis up." );
		changed |= ImGui::SliderFloat( "Max Spring Force", &m_tuning.maxSuspensionForce, 0.1f, 4.0f, "%.2f W" );
		ImGui::SetItemTooltip( "Cap on each wheel's spring and damper force, as a multiple of the wagon weight." );
		changed |= ImGui::SliderFloat( "Stop Hertz", &m_tuning.bumpStopHertz, 0.0f, 30.0f, "%.1f" );
		ImGui::SetItemTooltip( "Bump stop stiffness past full compression. Zero is a rigid stop. Softer lets a wheel "
							   "sink into a rock on a hard hit instead of throwing the wagon." );
		changed |= ImGui::SliderFloat( "Stop Damping", &m_tuning.bumpStopDampingRatio, 0.0f, 2.0f, "%.2f" );
		ImGui::SetItemTooltip( "Damping ratio of the soft bump stop. Unused when Stop Hertz is zero." );
		changed |= ImGui::SliderFloat( "Lateral Friction", &m_tuning.lateralFriction, 0.0f, 1.5f, "%.2f" );
		ImGui::SetItemTooltip( "Tire grip along the axle. Lower slides sooner in a turn instead of tipping." );
		changed |= ImGui::SliderFloat( "Rolling Friction", &m_tuning.rollingFriction, 0.0f, 1.5f, "%.2f" );
		ImGui::SetItemTooltip( "Tire grip along the rolling direction. Limits braking." );
		changed |= ImGui::SliderFloat( "Rolling Resistance", &m_tuning.rollingResistance, 0.0f, 0.2f, "%.3f" );
		ImGui::SetItemTooltip( "Drag of a free rolling wheel as a fraction of its load." );
		changed |= ImGui::SliderFloat( "Brake Force", &m_tuning.brakeForce, 0.0f, 1.0f, "%.2f W" );
		ImGui::SetItemTooltip( "Brake on each rear wheel, as a multiple of the wagon weight. The tire grip still caps it." );
		changed |= ImGui::SliderFloat( "Tire Force Height", &m_tuning.tireForceHeight, 0.0f, 0.6f, "%.2f m" );
		ImGui::SetItemTooltip( "Height above the contact where tire friction acts on the chassis. Zero is physical. "
							   "Raising it reduces body roll in turns." );
		changed |= ImGui::SliderFloat( "Draft Gain", &m_tuning.draftGain, 0.5f, 20.0f, "%.1f" );
		ImGui::SetItemTooltip( "How hard the horse pulls per m/s of speed short of the gait speed." );
		changed |= ImGui::SliderFloat( "Holdback", &m_tuning.holdback, 0.0f, 0.5f, "%.2f W" );
		ImGui::SetItemTooltip( "Most the horse can hold back or push the wagon backward, as a multiple of the wagon "
							   "weight." );
		changed |= ImGui::SliderFloat( "Strain Draft", &m_tuning.strainDraft, 0.0f, 1.5f, "%.2f W" );
		ImGui::SetItemTooltip( "Pull of a horse that digs in when the wagon stalls, as a multiple of the wagon weight. "
							   "Below the gait draft it does nothing." );
		changed |= ImGui::SliderFloat( "Strain Speed", &m_tuning.strainSpeed, 0.0f, 3.0f, "%.2f m/s" );
		ImGui::SetItemTooltip( "Pulling forward below this speed counts as stalled and the horse strains." );
		changed |= ImGui::SliderFloat( "Strain Duration", &m_tuning.strainDuration, 0.0f, 10.0f, "%.1f s" );
		ImGui::SetItemTooltip( "How long the horse can strain before it is spent. Zero turns straining off." );
		changed |= ImGui::SliderFloat( "Strain Recovery", &m_tuning.strainRecovery, 0.0f, 30.0f, "%.1f s" );
		ImGui::SetItemTooltip( "Rest time to recover a full strain. A spent horse cannot strain again until fully "
							   "recovered." );
		changed |= ImGui::SliderFloat( "Max Lateral Accel", &m_tuning.maxLateralAcceleration, 0.05f, 2.0f, "%.2f g" );
		ImGui::SetItemTooltip( "Hardest turn the horse will take. Narrows the steering limit as speed rises." );

		float degrees = 180.0f / B3_PI;
		float steerRate = m_tuning.steerRate * degrees;
		if ( ImGui::SliderFloat( "Steer Rate", &steerRate, 5.0f, 180.0f, "%.0f deg/s" ) )
		{
			m_tuning.steerRate = steerRate / degrees;
			changed = true;
		}
		ImGui::SetItemTooltip( "How fast the front gear turns toward the steering input." );

		float maxSteer = m_tuning.maxSteer * degrees;
		if ( ImGui::SliderFloat( "Cramp Limit", &maxSteer, 5.0f, 80.0f, "%.0f deg" ) )
		{
			m_tuning.maxSteer = maxSteer / degrees;
			changed = true;
		}
		ImGui::SetItemTooltip( "Most the front gear can turn about the kingbolt, at low speed." );

		float groundAngle = m_tuning.maxGroundAngle * degrees;
		if ( ImGui::SliderFloat( "Max Ground Angle", &groundAngle, 10.0f, 89.0f, "%.0f deg" ) )
		{
			m_tuning.maxGroundAngle = groundAngle / degrees;
			changed = true;
		}
		ImGui::SetItemTooltip( "Steepest wheel hit treated as ground the wheel can climb. Steeper hits are walls." );

		changed |= ImGui::SliderFloat( "Anti Roll Torque", &m_tuning.maxRollTorque, 0.0f, 4.0f, "%.2f W*T" );
		ImGui::SetItemTooltip( "Cap on the parallel joint that holds the wagon level across the ground, as a multiple "
							   "of weight times half track, the moment gravity gives against tipping. Zero is off." );
		changed |= ImGui::SliderFloat( "Anti Roll Hertz", &m_tuning.rollHertz, 0.1f, 10.0f, "%.1f" );
		ImGui::SetItemTooltip( "Stiffness of the anti roll joint as a natural frequency." );
		changed |= ImGui::SliderFloat( "Anti Roll Damping", &m_tuning.rollDampingRatio, 0.0f, 2.0f, "%.2f" );
		ImGui::SetItemTooltip( "Damping ratio of the anti roll joint. It damps pitch rate as well as roll." );

		changed |= ImGui::SliderFloat( "Horse Pitch Hertz", &m_tuning.horsePitchHertz, 0.0f, 10.0f, "%.1f" );
		ImGui::SetItemTooltip( "How fast the shafts and the horse's body follow the ground under its hooves. "
							   "Zero snaps to the ground." );
		changed |= ImGui::SliderFloat( "Horse Pitch Damping", &m_tuning.horsePitchDampingRatio, 0.0f, 2.0f, "%.2f" );
		ImGui::SetItemTooltip( "Damping ratio of the horse pitch. Below one the shafts bob after a step." );

		for ( int gait = e_wagonBack; gait < e_wagonGaitCount; ++gait )
		{
			if ( gait == e_wagonHalt )
			{
				continue;
			}

			ImGui::PushID( gait );
			ImGui::Text( "%s", g_wagonGaitNames[gait] );
			float minSpeed = gait == e_wagonBack ? -3.0f : 0.5f;
			float maxSpeed = gait == e_wagonBack ? -0.2f : 20.0f;
			changed |= ImGui::SliderFloat( "Speed", &m_tuning.gaitSpeed[gait], minSpeed, maxSpeed, "%.1f m/s" );
			ImGui::SetItemTooltip( "Speed the horse holds at this gait." );
			changed |= ImGui::SliderFloat( "Draft", &m_tuning.draft[gait], 0.0f, 1.0f, "%.2f W" );
			ImGui::SetItemTooltip( "Most the horse pulls forward at this gait, as a multiple of the wagon weight." );
			ImGui::PopID();
		}

		if ( changed )
		{
			SetWagonTuning( &m_wagon, &m_tuning );
		}

		// The mass is fixed at creation, so a new mass rebuilds the wagon where it stands
		rebuild |= ImGui::SliderFloat( "Mass", &m_mass, 200.0f, 4000.0f, "%.0f kg" );
		ImGui::SetItemTooltip( "Wagon mass. Forces given in W scale with it. Rebuilds the wagon where it stands." );
		if ( rebuild )
		{
			b3WorldTransform transform = b3Body_GetTransform( m_wagon.chassisId );
			DestroyWagon( &m_wagon );
			CreateWagonBody();
			b3Body_SetTransform( m_wagon.chassisId, transform.p, transform.q );
		}

		int gait = m_gait;
		if ( ImGui::Combo( "Gait", &gait, g_wagonGaitNames, e_wagonGaitCount ) )
		{
			m_gait = (WagonGait)gait;
		}
		ImGui::SetItemTooltip( "Horse gait. W and S step it in third person. Halt sets the brake." );

		bool thirdPerson = m_camera->m_thirdPerson;
		if ( ImGui::Checkbox( "Third Person (T)", &thirdPerson ) )
		{
			ToggleThirdPerson();
		}
		ImGui::SetItemTooltip( "Camera follows the wagon and the keys drive it." );

		return true;
	}

	void Step() override
	{
		m_steerInput = 0.0f;

		if ( m_camera->m_thirdPerson )
		{
			m_steerInput += IsKeyDown( KEY_A ) ? m_tuning.maxSteer : 0.0f;
			m_steerInput -= IsKeyDown( KEY_D ) ? m_tuning.maxSteer : 0.0f;
		}

		WagonInput input;
		input.gait = m_gait;
		input.steer = m_steerInput;
		input.brake = m_gait == e_wagonHalt || IsKeyDown( KEY_SPACE );

		// The draft force accumulates until the world steps, so only update on a step that runs
		bool stepping = m_context->pause == false || m_context->singleStep > 0;
		float timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
		if ( stepping && timeStep > 0.0f )
		{
			UpdateWagon( &m_wagon, input, timeStep );
		}

		Sample::Step();

		if ( stepping && timeStep > 0.0f )
		{
			UpdateWagonResults( &m_wagon, timeStep );
		}

		if ( m_camera->m_thirdPerson )
		{
			m_camera->m_pivot = b3Body_GetPosition( m_wagon.chassisId );
			m_camera->UpdateTransform();
		}
	}

	void DrawWagon()
	{
		for ( int i = 0; i < e_wagonWheelCount; ++i )
		{
			const WagonWheel& wheel = m_wagon.wheels[i];
			b3WorldTransform wheelTransform = GetWagonWheelTransform( &m_wagon, i );
			const b3HullData* hull = wheel.steered ? m_frontWheelHull : m_rearWheelHull;

			b3HexColor color = b3_colorPeru;
			if ( wheel.grounded == false )
			{
				color = b3_colorGray;
			}
			else if ( wheel.saturated )
			{
				color = b3_colorRed;
			}

			DrawHull( wheelTransform, hull, MakeColor( color ) );

			if ( wheel.grounded )
			{
				DrawPoint( wheel.contactPoint, 6.0f, MakeColor( b3_colorYellow ) );
			}
		}

		// Front gear: the axle between the front hubs and a cross at the kingbolt
		b3Pos left = GetWagonWheelTransform( &m_wagon, e_wagonFrontLeft ).p;
		b3Pos right = GetWagonWheelTransform( &m_wagon, e_wagonFrontRight ).p;
		DrawLine( left, right, MakeColor( b3_colorSaddleBrown ) );
		b3Pos kingbolt = GetWagonKingbolt( &m_wagon );
		DrawCross( kingbolt, 0.15f, MakeColor( b3_colorOrange ) );

		// Shafts hinge at the front gear and rest in the tugs on the horse's back
		b3WorldTransform horse = m_wagon.horseTransform;
		b3Vec3 side = b3RotateVector( horse.q, b3Vec3_axisZ );
		float shaftSpacing = 0.35f;
		for ( int i = 0; i < 2; ++i )
		{
			float s = i == 0 ? shaftSpacing : -shaftSpacing;
			b3Pos a = b3OffsetPos( kingbolt, b3MulSV( s, side ) );
			b3Pos b = b3OffsetPos( a, b3MulSV( m_wagon.shaftLength, m_wagon.shaftDirection ) );
			DrawLine( a, b, MakeColor( b3_colorSienna ) );
		}

		b3Pos tug = GetWagonTug( &m_wagon );
		DrawCross( tug, 0.1f, MakeColor( b3_colorOrange ) );

		// The horse's body on legs about 0.8 m long, pitched to its hooves
		b3Vec3 horseSize = { 2.2f, 1.5f, 0.6f };
		b3Pos horseCenter = b3TransformWorldPoint( horse, { 0.0f, 0.8f + 0.5f * horseSize.y, 0.0f } );
		DrawCube( { horseCenter, horse.q }, horseSize, MakeColorAlpha( b3_colorTan, 0.6f ) );

		for ( int i = 0; i < 2; ++i )
		{
			b3HexColor color = m_wagon.hoofGrounded[i] ? b3_colorYellow : b3_colorGray;
			DrawPoint( m_wagon.hooves[i], 6.0f, MakeColor( color ) );
		}

		// Draft force at the kingbolt, scaled by the wagon weight
		if ( m_wagon.draftForce != 0.0f )
		{
			b3Vec3 draft = b3MulSV( 2.0f * m_wagon.draftForce / m_wagon.weight, m_wagon.shaftDirection );
			DrawArrow( kingbolt, b3OffsetPos( kingbolt, draft ), MakeColor( b3_colorCyan ) );
		}
	}

	void Render() override
	{
		Sample::Render();

		DrawWagon();

		float degrees = 180.0f / B3_PI;
		bool awake = b3Body_IsAwake( m_wagon.chassisId );
		DrawTextLine( "gait = %s, speed = %.2f m/s, steer = %.1f deg (limit %.1f), draft = %.2f W, brake = %s, %s",
					  g_wagonGaitNames[m_wagon.gait], m_wagon.speed, m_wagon.steer * degrees, m_wagon.steerLimit * degrees,
					  m_wagon.draftForce / m_wagon.weight, m_wagon.brake ? "on" : "off", awake ? "awake" : "asleep" );
		DrawTextLine( "strain = %.0f%%, used %.1f of %.1f s%s", 100.0f * m_wagon.strain, m_wagon.strainTime,
					  m_tuning.strainDuration, m_wagon.strainSpent ? ", spent" : "" );
		DrawTextLine( "shaft pitch = %.1f deg (target %.1f)%s", m_wagon.shaftPitch * degrees,
					  m_wagon.targetShaftPitch * degrees,
					  m_wagon.hoofGrounded[0] || m_wagon.hoofGrounded[1] ? "" : ", horse off the ground" );
		if ( b3Joint_IsValid( m_wagon.rollJointId ) )
		{
			float maxTorque = m_tuning.maxRollTorque * m_wagon.weight * m_wagon.halfTrack;
			DrawTextLine( "anti roll torque = %.0f of %.0f N*m", m_wagon.rollTorque, maxTorque );
		}

		const char* names[e_wagonWheelCount] = { "front left", "front right", "rear left", "rear right" };
		for ( int i = 0; i < e_wagonWheelCount; ++i )
		{
			const WagonWheel& wheel = m_wagon.wheels[i];
			if ( wheel.grounded == false )
			{
				DrawTextLine( "%-11s  in the air", names[i] );
				continue;
			}

			float compression = m_tuning.suspensionTravel - wheel.suspensionLength;
			DrawTextLine( "%-11s  compression %5.1f cm  N %6.0f  lateral %6.0f  rolling %6.0f  %s", names[i],
						  100.0f * compression, wheel.normalForce, wheel.lateralForce, wheel.rollingForce,
						  wheel.saturated ? "saturated" : "" );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new WagonSample( context );
	}

	Wagon m_wagon;
	WagonTuning m_tuning;
	WagonGait m_gait;
	float m_steerInput;
	float m_mass;
	b3HullData* m_frontWheelHull;
	b3HullData* m_rearWheelHull;
	b3MeshData* m_waveMesh;
};

static int sampleWagon = RegisterSample( "Vehicle", "Wagon", WagonSample::Create );
