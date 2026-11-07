// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"

#include "gfx/draw.h"

#include "box3d/box3d.h"

#include <algorithm>
#include <float.h>
#include <functional>
#include <imgui.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <utility>
#include <vector>

// Scenes for comparing ghost collision handling across physics engines. Each one isolates a mechanism
// where engines differ: crest angles around the active edge cutoffs, high valence vertex fans, long runs
// through grid vertices, and large bodies on fine meshes. Energy, vertical velocity and drift only need
// body state, so the same scenes can be measured in any engine. The ghost count also needs contact
// normals and the triangle feature behind each point.

enum ProbeShape
{
	e_probeBox,
	e_probeCylinder,
	e_probeSphere,
	e_probeCapsule,
	e_probeShapeCount
};

static const char* s_probeShapeNames[e_probeShapeCount] = { "Box", "Cylinder", "Sphere", "Capsule" };

// Box and cylinder slide without friction. Sphere and capsule roll.
static bool IsRoller( int shape )
{
	return shape == e_probeSphere || shape == e_probeCapsule;
}

static float GetProbeRadius( int shape )
{
	return shape == e_probeSphere ? 0.25f : 0.2f;
}

enum DiagonalMode
{
	e_diagonalForward,
	e_diagonalBackward,
	e_diagonalAlternate,
	e_diagonalModeCount
};

static const char* s_diagonalNames[e_diagonalModeCount] = { "Along +x+z", "Along +x-z", "Alternate" };

static std::vector<float> MakeSamples( float lower, float upper, int count )
{
	std::vector<float> samples( count + 1 );
	for ( int i = 0; i <= count; ++i )
	{
		samples[i] = lower + ( upper - lower ) * (float)i / (float)count;
	}
	return samples;
}

// Rows run along x. Triangles wind counter clockwise seen from above.
static void AddGrid( std::vector<b3Vec3>& vertices, std::vector<int32_t>& indices, const std::vector<float>& xs,
					 const std::vector<float>& zs, const std::function<float( int, int )>& height, int mode )
{
	int base = (int)vertices.size();
	int stride = (int)zs.size();
	for ( int i = 0; i < (int)xs.size(); ++i )
	{
		for ( int j = 0; j < stride; ++j )
		{
			vertices.push_back( { xs[i], height( i, j ), zs[j] } );
		}
	}

	int xCount = (int)xs.size() - 1;
	int zCount = stride - 1;
	for ( int i = 0; i < xCount; ++i )
	{
		for ( int j = 0; j < zCount; ++j )
		{
			int a = base + i * stride + j;
			int b = a + 1;
			int c = a + stride + 1;
			int d = a + stride;

			bool forward = mode == e_diagonalForward || ( mode == e_diagonalAlternate && ( ( i + j ) & 1 ) == 0 );
			if ( forward )
			{
				int32_t corners[6] = { a, b, c, c, d, a };
				indices.insert( indices.end(), corners, corners + 6 );
			}
			else
			{
				int32_t corners[6] = { a, b, d, b, c, d };
				indices.insert( indices.end(), corners, corners + 6 );
			}
		}
	}
}

static void AddUpTriangle( const std::vector<b3Vec3>& vertices, std::vector<int32_t>& indices, int a, int b, int c )
{
	b3Vec3 normal = b3Cross( vertices[b] - vertices[a], vertices[c] - vertices[a] );
	if ( normal.y < 0.0f )
	{
		std::swap( b, c );
	}

	indices.push_back( a );
	indices.push_back( b );
	indices.push_back( c );
}

// Linear and rotational kinetic energy per unit mass
static float GetSpecificKinetic( b3BodyId bodyId )
{
	b3Vec3 v = b3Body_GetLinearVelocity( bodyId );
	b3Vec3 w = b3InvRotateVector( b3Body_GetRotation( bodyId ), b3Body_GetAngularVelocity( bodyId ) );
	b3Matrix3 inertia = b3Body_GetLocalRotationalInertia( bodyId );
	return 0.5f * b3Dot( v, v ) + 0.5f * b3Dot( w, b3MulMV( inertia, w ) ) / b3Body_GetMass( bodyId );
}

static float GetSpecificEnergy( b3BodyId bodyId, b3Vec3 gravity )
{
	b3Vec3 center = b3ToVec3( b3Body_GetWorldCenter( bodyId ) );
	return GetSpecificKinetic( bodyId ) - b3Dot( gravity, center );
}

static float NextUnit( uint32_t& state )
{
	state = 1664525u * state + 1013904223u;
	return (float)( state >> 8 ) / 16777216.0f;
}

// Owns the ground mesh and counts ghost manifolds on it. A pushing manifold is a ghost when its normal
// matches no nearby triangle and does not lie in the normal cone of the edge or vertex that produced it.
class GhostSample : public Sample
{
public:
	static constexpr int m_markerCapacity = 256;
	static constexpr int m_historyCapacity = 600;

	enum GhostClass
	{
		e_faceGhost,
		e_edgeGhost,
		e_vertexGhost,
		e_otherGhost,
		e_ghostClassCount
	};

	explicit GhostSample( SampleContext* context )
		: Sample( context )
	{
		m_mesh = nullptr;
		m_groundId = b3_nullBodyId;
		m_followId = b3_nullBodyId;
		m_cylinder = b3CreateCylinder( 0.4f, 0.4f, -0.2f, 16 );
		m_toleranceDegrees = 5.0f;
		m_cosTolerance = 1.0f;
		m_sinTolerance = 0.0f;
		m_drawGhosts = true;
		m_follow = true;
		m_showHistory = true;
		m_plotScale = 1.0f;
		m_lastHeight = 0.0f;
		ResetGhostCounts();
		ClearHistory();
	}

	~GhostSample() override
	{
		b3DestroyHull( m_cylinder );
		if ( m_mesh != nullptr )
		{
			b3DestroyMesh( m_mesh );
		}
	}

protected:
	void CreateGround( std::vector<b3Vec3>& vertices, std::vector<int32_t>& indices )
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

		b3MeshDef def = {};
		def.vertices = vertices.data();
		def.vertexCount = (int)vertices.size();
		def.indices = indices.data();
		def.triangleCount = (int)indices.size() / 3;
		def.useMedianSplit = true;
		def.identifyEdges = true;
		m_mesh = b3CreateMesh( &def, nullptr, 0 );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.name = "ground";
		m_groundId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateMeshShape( m_groundId, &shapeDef, m_mesh, b3Vec3_one );

		BuildAdjacency();
		ResetGhostCounts();
	}

	// The direction is horizontal. Rollers start with the spin that matches their speed. The yaw turns a
	// box or cylinder about the vertical axis relative to the direction of travel.
	b3BodyId CreateProbe( int shape, b3Pos position, b3Vec3 direction, float yawDegrees, float speed )
	{
		b3Quat heading = b3MakeQuatFromAxisAngle( b3Vec3_axisY, atan2f( -direction.z, direction.x ) );
		b3Quat yaw = b3MakeQuatFromAxisAngle( b3Vec3_axisY, yawDegrees * B3_DEG_TO_RAD );
		bool roller = IsRoller( shape );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = position;
		bodyDef.rotation = roller ? heading : b3MulQuat( heading, yaw );
		bodyDef.linearVelocity = speed * direction;
		if ( roller )
		{
			bodyDef.angularVelocity = ( speed / GetProbeRadius( shape ) ) * b3Cross( b3Vec3_axisY, direction );
		}

		// Recycling reuses a manifold while the body barely moves, which hides the collision under test
		bodyDef.enableContactRecycling = false;
		bodyDef.enableSleep = false;
		bodyDef.name = "probe";
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = roller ? 0.6f : 0.0f;

		switch ( shape )
		{
			case e_probeBox:
			{
				b3BoxHull box = b3MakeBoxHull( 0.4f, 0.2f, 0.4f );
				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}
			break;

			case e_probeCylinder:
				b3CreateHullShape( bodyId, &shapeDef, m_cylinder );
				break;

			case e_probeSphere:
			{
				b3Sphere sphere = { b3Vec3_zero, GetProbeRadius( shape ) };
				b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			}
			break;

			default:
			{
				// Axis across the direction of travel so it rolls
				b3Capsule capsule = { { 0.0f, 0.0f, -0.4f }, { 0.0f, 0.0f, 0.4f }, GetProbeRadius( shape ) };
				b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			}
			break;
		}

		return bodyId;
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

	// The center is the body center at the start of the step, where the anchors were computed
	int CountGhosts( b3BodyId bodyId, b3Pos center )
	{
		// Clustering merges manifolds whose normals agree within its threshold and hands them one normal,
		// so a point may carry a normal that far from its own feature
		float angle = m_toleranceDegrees * B3_DEG_TO_RAD + acosf( 0.996f );
		m_cosTolerance = cosf( angle );
		m_sinTolerance = sinf( angle );

		memset( m_stepClassCounts, 0, sizeof( m_stepClassCounts ) );
		int ghostCount = 0;

		// A pile box touches many neighbors besides the ground
		m_contacts.resize( b3MaxInt( 1, b3Body_GetContactCapacity( bodyId ) ) );
		b3ContactData* contacts = m_contacts.data();
		int contactCount = b3Body_GetContactData( bodyId, contacts, (int)m_contacts.size() );
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
				b3Vec3 normal = bodyIsA ? -manifold->normal : manifold->normal;

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
				GhostClass ghostClass = Classify( point );
				m_classCounts[ghostClass] += 1;
				m_stepClassCounts[ghostClass] += 1;
				m_ghostCount += 1;
				ghostCount += 1;

				b3Vec3 anchor = bodyIsA ? point.anchorA : point.anchorB;
				m_markers[m_markerIndex] = { b3OffsetPos( center, anchor ), normal };
				m_markerIndex = ( m_markerIndex + 1 ) % m_markerCapacity;
				m_markerCount = b3MinInt( m_markerCount + 1, m_markerCapacity );
			}
		}

		return ghostCount;
	}

	void ResetGhostCounts()
	{
		memset( m_classCounts, 0, sizeof( m_classCounts ) );
		memset( m_stepClassCounts, 0, sizeof( m_stepClassCounts ) );
		m_ghostCount = 0;
		m_manifoldCount = 0;
		m_markerCount = 0;
		m_markerIndex = 0;
	}

	virtual void ResetCounters()
	{
		ResetGhostCounts();
	}

	void ClearHistory()
	{
		m_historyCount = 0;
		m_historyIndex = 0;
	}

	// From the height change over the step, so push out counts and every engine is measured the same way.
	// A fast rolling body can report a steady downward velocity that never moves it.
	float GetVerticalSpeed( float height )
	{
		float speed = ( height - m_lastHeight ) * m_context->hertz;
		m_lastHeight = height;
		return speed;
	}

	void PushHistory( float value )
	{
		m_history[m_historyIndex] = value;
		m_historyIndex = ( m_historyIndex + 1 ) % m_historyCapacity;
		m_historyCount = b3MinInt( m_historyCount + 1, m_historyCapacity );
	}

	void DrawGhosts()
	{
		if ( m_drawGhosts )
		{
			for ( int i = 0; i < m_markerCount; ++i )
			{
				b3Pos p = m_markers[i].point;
				DrawPoint( p, 8.0f, MakeColor( b3_colorRed ) );
				DrawLine( p, p + 0.5f * m_markers[i].normal, MakeColor( b3_colorRed ) );
			}
		}

		if ( m_follow && B3_IS_NON_NULL( m_followId ) )
		{
			m_camera->SetPivot( b3Body_GetPosition( m_followId ) );
		}
	}

	void DrawGhostSummary()
	{
		DrawTextLine( "ghost manifolds: %d of %d pushing, face %d, edge %d, vertex %d, unresolved %d", m_ghostCount,
					  m_manifoldCount, m_classCounts[e_faceGhost], m_classCounts[e_edgeGhost], m_classCounts[e_vertexGhost],
					  m_classCounts[e_otherGhost] );
		DrawTextLine( "triangles: %d, mesh bytes: %d", m_mesh->triangleCount, m_mesh->byteCount );
	}

	void DrawGhostControls()
	{
		ImGui::Separator();
		ImGui::SliderFloat( "Ghost Tolerance", &m_toleranceDegrees, 1.0f, 20.0f, "%.0f deg" );
		ImGui::Checkbox( "Draw Ghosts", &m_drawGhosts );
		ImGui::Checkbox( "Follow", &m_follow );
		if ( ImGui::Button( "Reset Counters" ) )
		{
			ResetCounters();
		}

		if ( m_showHistory )
		{
			ImGui::SliderFloat( "Plot Scale", &m_plotScale, 0.1f, 5.0f, "%.1f m/s" );
			int offset = m_historyCount < m_historyCapacity ? 0 : m_historyIndex;
			ImGui::PlotLines( "##vy", m_history, m_historyCount, offset, "vertical velocity", -m_plotScale, m_plotScale,
							  ImVec2( 0.0f, 80.0f ) );
		}
	}

	struct Marker
	{
		b3Pos point;
		b3Vec3 normal;
	};

	b3MeshData* m_mesh;
	b3BodyId m_groundId;
	b3BodyId m_followId;
	b3HullData* m_cylinder;

	std::vector<b3Vec3> m_triangleNormals;
	std::vector<int> m_edgeNeighbors;
	std::vector<std::vector<int>> m_vertexRings;
	std::vector<std::vector<int>> m_vertexTriangles;
	std::vector<b3ContactData> m_contacts;

	float m_toleranceDegrees;
	float m_cosTolerance;
	float m_sinTolerance;
	int m_classCounts[e_ghostClassCount];
	int m_stepClassCounts[e_ghostClassCount];
	int m_ghostCount;
	int m_manifoldCount;

	Marker m_markers[m_markerCapacity];
	int m_markerCount;
	int m_markerIndex;

	float m_history[m_historyCapacity];
	int m_historyCount;
	int m_historyIndex;
	float m_plotScale;
	float m_lastHeight;

	bool m_drawGhosts;
	bool m_follow;
	bool m_showHistory;
};

// Roof ridges with crest angles from 1 to 20 degrees, separated by flat runs. The angles straddle the
// cutoffs of engines that keep one convex bit per edge, about 5 degrees in Jolt and PhysX, while Box3D
// stores the angle in 1.4 degree steps. Each crest reports, from its foot to the next foot, the energy
// lost and the peak vertical velocity. The slope alone lifts the body at its speed times the sine of
// half the crest angle.
class CrestLadder : public GhostSample
{
public:
	static constexpr int m_crestCount = 9;
	static constexpr float m_slopeRun = 1.5f;
	static constexpr float m_gap = 2.5f;
	static constexpr float m_leadIn = 5.0f;
	static constexpr float m_runOut = 5.0f;
	static constexpr float m_halfWidth = 2.0f;

	struct CrestStats
	{
		float startEnergy;
		float startKinetic;
		float slopeVelocity;
		float peakVelocity;
		float loss;
		int ghosts;
		bool valid;
	};

	explicit CrestLadder( SampleContext* context )
		: GhostSample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 60.0f, 20.0f, 12.0f, { 2.0f, 0.5f, 0.0f } );
		}

		m_probe = e_probeBox;
		m_speed = 4.0f;
		m_yaw = 0.0f;
		m_cellSize = 0.25f;
		m_diagonal = e_diagonalAlternate;
		m_probeId = b3_nullBodyId;

		BuildGround();
		ResetCounters();
		Launch();
	}

	void ResetCounters() override
	{
		GhostSample::ResetCounters();
		memset( m_current, 0, sizeof( m_current ) );
		memset( m_last, 0, sizeof( m_last ) );
		memset( m_crestGhosts, 0, sizeof( m_crestGhosts ) );
		m_runCount = 0;
	}

	void BuildGround()
	{
		std::vector<b3Vec2> corners;
		corners.push_back( { 0.0f, 0.0f } );

		float x = m_leadIn;
		for ( int i = 0; i < m_crestCount; ++i )
		{
			float rise = m_slopeRun * tanf( 0.5f * m_angles[i] * B3_DEG_TO_RAD );
			m_windowStart[i] = x;
			m_crestPoints[i] = { x + m_slopeRun, rise };
			corners.push_back( { x, 0.0f } );
			corners.push_back( { x + m_slopeRun, rise } );
			corners.push_back( { x + 2.0f * m_slopeRun, 0.0f } );
			x += 2.0f * m_slopeRun + m_gap;
		}

		m_windowEnd = x;
		m_length = x + m_runOut;
		corners.push_back( { m_length, 0.0f } );

		// Every corner lands on a vertex row
		std::vector<float> xs;
		std::vector<float> ys;
		for ( size_t k = 0; k + 1 < corners.size(); ++k )
		{
			b3Vec2 a = corners[k];
			b3Vec2 b = corners[k + 1];
			int count = b3MaxInt( 1, (int)ceilf( ( b.x - a.x ) / m_cellSize - 0.001f ) );
			for ( int s = 0; s < count; ++s )
			{
				float t = (float)s / (float)count;
				xs.push_back( a.x + t * ( b.x - a.x ) );
				ys.push_back( a.y + t * ( b.y - a.y ) );
			}
		}
		xs.push_back( corners.back().x );
		ys.push_back( corners.back().y );

		int zCount = b3MaxInt( 1, (int)ceilf( 2.0f * m_halfWidth / m_cellSize - 0.001f ) );
		std::vector<float> zs = MakeSamples( -m_halfWidth, m_halfWidth, zCount );

		std::vector<b3Vec3> vertices;
		std::vector<int32_t> indices;
		AddGrid( vertices, indices, xs, zs, [&]( int i, int ) { return ys[i]; }, m_diagonal );
		CreateGround( vertices, indices );
	}

	void Launch()
	{
		if ( B3_IS_NON_NULL( m_probeId ) )
		{
			b3DestroyBody( m_probeId );
		}

		b3Pos position = { 2.0f, GetProbeRadius( m_probe ) + 0.01f, 0.0f };
		m_probeId = CreateProbe( m_probe, position, b3Vec3_axisX, m_yaw, m_speed );
		m_followId = m_probeId;
		m_lastHeight = (float)position.y;
		m_window = -1;
		m_runSteps = 0;
		memset( m_current, 0, sizeof( m_current ) );
		ClearHistory();
	}

	int FindWindow( float x ) const
	{
		for ( int i = 0; i < m_crestCount; ++i )
		{
			float end = i + 1 < m_crestCount ? m_windowStart[i + 1] : m_windowEnd;
			if ( m_windowStart[i] <= x && x < end )
			{
				return i;
			}
		}

		return -1;
	}

	void OpenWindow( int index )
	{
		b3Vec3 gravity = b3World_GetGravity( m_worldId );
		float speed = b3Length( b3Body_GetLinearVelocity( m_probeId ) );
		CrestStats& stats = m_current[index];
		stats = {};
		stats.startEnergy = GetSpecificEnergy( m_probeId, gravity );
		stats.startKinetic = GetSpecificKinetic( m_probeId );
		stats.slopeVelocity = speed * sinf( 0.5f * m_angles[index] * B3_DEG_TO_RAD );
		stats.peakVelocity = -FLT_MAX;
	}

	void CloseWindow( int index )
	{
		b3Vec3 gravity = b3World_GetGravity( m_worldId );
		CrestStats& stats = m_current[index];
		float lost = stats.startEnergy - GetSpecificEnergy( m_probeId, gravity );
		stats.loss = stats.startKinetic > 0.0f ? 100.0f * lost / stats.startKinetic : 0.0f;
		stats.valid = true;
	}

	void Step() override
	{
		b3Pos center = b3Body_GetWorldCenter( m_probeId );

		Sample::Step();

		if ( m_didStep )
		{
			int ghostCount = CountGhosts( m_probeId, center );

			b3Pos position = b3Body_GetWorldCenter( m_probeId );
			float verticalSpeed = GetVerticalSpeed( (float)position.y );
			PushHistory( verticalSpeed );

			int window = FindWindow( (float)position.x );
			if ( window != m_window )
			{
				if ( m_window >= 0 )
				{
					CloseWindow( m_window );
				}

				if ( window >= 0 )
				{
					OpenWindow( window );
				}

				m_window = window;
			}

			if ( m_window >= 0 )
			{
				CrestStats& stats = m_current[m_window];
				stats.ghosts += ghostCount;
				stats.peakVelocity = b3MaxFloat( stats.peakVelocity, verticalSpeed );
				m_crestGhosts[m_window] += ghostCount;
			}

			m_runSteps += 1;
			bool done = (float)position.x > m_length - 3.0f || (float)position.y < -2.0f || m_runSteps > 1800;
			if ( done )
			{
				if ( m_window >= 0 )
				{
					CloseWindow( m_window );
				}

				memcpy( m_last, m_current, sizeof( m_last ) );
				m_runCount += 1;
				Launch();
			}
		}

		DrawGhosts();

		for ( int i = 0; i < m_crestCount; ++i )
		{
			b3Pos p = { m_crestPoints[i].x, m_crestPoints[i].y + 0.6f, m_halfWidth };
			DrawString3D( p, MakeColor( b3_colorWhite ), "%.0f deg", m_angles[i] );
		}

		DrawTextLine( "completed runs: %d, crest values from the last run, ghosts over all runs", m_runCount );
		DrawGhostSummary();

		const CrestStats* stats = m_runCount > 0 ? m_last : m_current;
		for ( int i = 0; i < m_crestCount; ++i )
		{
			if ( stats[i].valid )
			{
				DrawTextLine( "crest %2.0f deg: ghosts %d, energy loss %5.1f%%, peak vy %.2f m/s, slope vy %.2f m/s", m_angles[i],
							  m_crestGhosts[i], stats[i].loss, stats[i].peakVelocity, stats[i].slopeVelocity );
			}
			else
			{
				DrawTextLine( "crest %2.0f deg: ghosts %d", m_angles[i], m_crestGhosts[i] );
			}
		}
	}

	bool DrawControls() override
	{
		bool rebuild = false;
		bool relaunch = false;

		relaunch |= ImGui::Combo( "Probe", &m_probe, s_probeShapeNames, e_probeShapeCount );
		relaunch |= ImGui::SliderFloat( "Speed", &m_speed, 0.5f, 15.0f, "%.1f m/s" );
		relaunch |= ImGui::SliderFloat( "Yaw", &m_yaw, 0.0f, 45.0f, "%.0f deg" );
		rebuild |= ImGui::SliderFloat( "Cell Size", &m_cellSize, 0.1f, 1.0f, "%.2f m" );
		rebuild |= ImGui::Combo( "Diagonals", &m_diagonal, s_diagonalNames, e_diagonalModeCount );
		relaunch |= ImGui::Button( "Launch" );

		DrawGhostControls();

		if ( rebuild )
		{
			BuildGround();
			ResetCounters();
		}

		if ( rebuild || relaunch )
		{
			Launch();
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new CrestLadder( context );
	}

	static constexpr float m_angles[m_crestCount] = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 8.0f, 12.0f, 20.0f };

	b3BodyId m_probeId;
	int m_probe;
	float m_speed;
	float m_yaw;
	float m_cellSize;
	int m_diagonal;

	float m_windowStart[m_crestCount];
	b3Vec2 m_crestPoints[m_crestCount];
	float m_windowEnd;
	float m_length;

	CrestStats m_current[m_crestCount];
	CrestStats m_last[m_crestCount];
	int m_crestGhosts[m_crestCount];
	int m_window;
	int m_runSteps;
	int m_runCount;
};

static int sampleCrestLadder = RegisterSample( "Ghost", "Crest Ladder", CrestLadder::Create );

// A cone apex shared by 6 to 48 triangles with uneven spacing, so some triangles near the apex are
// slivers. Every spoke is a crest whose dihedral angle shrinks as the fan grows, so each engine's edge
// cutoff falls somewhere in the range. Vertex handling is where engines differ most. The probe crosses
// the apex along eight headings in turn.
class HubPeak : public GhostSample
{
public:
	static constexpr int m_headingCount = 8;
	static constexpr int m_ringCount = 10;
	static constexpr float m_ringSpacing = 1.0f;
	static constexpr float m_coneRadius = 3.0f;
	static constexpr float m_launchDistance = 6.5f;

	enum HubShape
	{
		e_hubPeak,
		e_hubPit,
		e_hubSaddle,
		e_hubShapeCount
	};

	struct RunStats
	{
		float startEnergy;
		float startKinetic;
		float peakVelocity;
		float loss;
		int ghosts;
		int vertexGhosts;
		bool valid;
	};

	explicit HubPeak( SampleContext* context )
		: GhostSample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 16.0f, { 0.0f, 0.5f, 0.0f } );
		}

		m_follow = false;
		m_hubShape = e_hubPeak;
		m_fanIndex = 1;
		m_slopeDegrees = 10.0f;
		m_jitter = 0.3f;
		m_probe = e_probeBox;
		m_speed = 5.0f;
		m_yaw = 0.0f;
		m_offset = 0.0f;
		m_heading = 0;
		m_probeId = b3_nullBodyId;

		BuildGround();
		ResetCounters();
		Launch();
	}

	void ResetCounters() override
	{
		GhostSample::ResetCounters();
		memset( m_last, 0, sizeof( m_last ) );
		m_runCount = 0;
	}

	float GetHeight( float radius, float angle ) const
	{
		if ( radius >= m_coneRadius )
		{
			return 0.0f;
		}

		float slope = tanf( m_slopeDegrees * B3_DEG_TO_RAD );
		switch ( m_hubShape )
		{
			case e_hubPeak:
				return slope * ( m_coneRadius - radius );

			case e_hubPit:
				return -slope * ( m_coneRadius - radius );

			default:
				// Three ridges and three valleys meet at the center, the slope there is set by the angle
				return slope * radius * ( 1.0f - radius / m_coneRadius ) * cosf( 3.0f * angle );
		}
	}

	void BuildGround()
	{
		int fanCount = m_fanCounts[m_fanIndex];

		// Fixed seed so a fan count always gets the same slivers
		uint32_t state = 12345;
		std::vector<float> angles( fanCount );
		for ( int i = 0; i < fanCount; ++i )
		{
			float u = NextUnit( state ) - 0.5f;
			angles[i] = 2.0f * B3_PI * ( (float)i + m_jitter * u ) / (float)fanCount;
		}

		std::vector<b3Vec3> vertices;
		std::vector<int32_t> indices;
		vertices.push_back( { 0.0f, GetHeight( 0.0f, 0.0f ), 0.0f } );
		for ( int k = 1; k <= m_ringCount; ++k )
		{
			float radius = m_ringSpacing * (float)k;
			for ( int i = 0; i < fanCount; ++i )
			{
				float angle = angles[i];
				vertices.push_back( { radius * cosf( angle ), GetHeight( radius, angle ), radius * sinf( angle ) } );
			}
		}

		for ( int i = 0; i < fanCount; ++i )
		{
			AddUpTriangle( vertices, indices, 0, 1 + i, 1 + ( i + 1 ) % fanCount );
		}

		// Dihedral angles between neighboring apex triangles
		m_minDihedral = FLT_MAX;
		m_maxDihedral = 0.0f;
		for ( int i = 0; i < fanCount; ++i )
		{
			const int32_t* t1 = indices.data() + 3 * i;
			const int32_t* t2 = indices.data() + 3 * ( ( i + 1 ) % fanCount );
			b3Vec3 n1 = b3Normalize( b3Cross( vertices[t1[1]] - vertices[t1[0]], vertices[t1[2]] - vertices[t1[0]] ) );
			b3Vec3 n2 = b3Normalize( b3Cross( vertices[t2[1]] - vertices[t2[0]], vertices[t2[2]] - vertices[t2[0]] ) );
			float dihedral = acosf( b3ClampFloat( b3Dot( n1, n2 ), -1.0f, 1.0f ) ) / B3_DEG_TO_RAD;
			m_minDihedral = b3MinFloat( m_minDihedral, dihedral );
			m_maxDihedral = b3MaxFloat( m_maxDihedral, dihedral );
		}

		for ( int k = 1; k < m_ringCount; ++k )
		{
			int inner = 1 + ( k - 1 ) * fanCount;
			int outer = inner + fanCount;
			for ( int i = 0; i < fanCount; ++i )
			{
				int next = ( i + 1 ) % fanCount;
				int a = inner + i;
				int b = inner + next;
				int c = outer + next;
				int d = outer + i;

				if ( ( i + k ) & 1 )
				{
					AddUpTriangle( vertices, indices, a, b, c );
					AddUpTriangle( vertices, indices, a, c, d );
				}
				else
				{
					AddUpTriangle( vertices, indices, a, b, d );
					AddUpTriangle( vertices, indices, b, c, d );
				}
			}
		}

		CreateGround( vertices, indices );
	}

	b3Vec3 GetDirection() const
	{
		float angle = (float)m_heading * 2.0f * B3_PI / (float)m_headingCount;
		return { cosf( angle ), 0.0f, sinf( angle ) };
	}

	void Launch()
	{
		if ( B3_IS_NON_NULL( m_probeId ) )
		{
			b3DestroyBody( m_probeId );
		}

		b3Vec3 d = GetDirection();
		b3Vec3 perp = { -d.z, 0.0f, d.x };
		b3Vec3 start = -m_launchDistance * d + m_offset * perp;
		start.y = GetProbeRadius( m_probe ) + 0.01f;
		m_start = start;

		m_probeId = CreateProbe( m_probe, b3ToPos( start ), d, m_yaw, m_speed );
		m_followId = m_probeId;
		m_lastHeight = start.y;
		m_runSteps = 0;

		m_current = {};
		m_current.startEnergy = GetSpecificEnergy( m_probeId, b3World_GetGravity( m_worldId ) );
		m_current.startKinetic = GetSpecificKinetic( m_probeId );
		m_current.peakVelocity = -FLT_MAX;
		ClearHistory();
	}

	void Step() override
	{
		b3Pos center = b3Body_GetWorldCenter( m_probeId );

		Sample::Step();

		if ( m_didStep )
		{
			int ghostCount = CountGhosts( m_probeId, center );
			m_current.ghosts += ghostCount;
			m_current.vertexGhosts += m_stepClassCounts[e_vertexGhost];

			b3Vec3 position = b3ToVec3( b3Body_GetWorldCenter( m_probeId ) );
			float verticalSpeed = GetVerticalSpeed( position.y );
			m_current.peakVelocity = b3MaxFloat( m_current.peakVelocity, verticalSpeed );
			PushHistory( verticalSpeed );

			b3Vec3 d = GetDirection();
			float along = b3Dot( position - m_start, d );
			float radius = sqrtf( position.x * position.x + position.z * position.z );

			m_runSteps += 1;
			bool done = along > 2.0f * m_launchDistance || position.y < -4.0f || radius > 9.0f || m_runSteps > 900;
			if ( done )
			{
				float lost = m_current.startEnergy - GetSpecificEnergy( m_probeId, b3World_GetGravity( m_worldId ) );
				m_current.loss = m_current.startKinetic > 0.0f ? 100.0f * lost / m_current.startKinetic : 0.0f;
				m_current.valid = true;
				m_last[m_heading] = m_current;
				m_runCount += 1;
				m_heading = ( m_heading + 1 ) % m_headingCount;
				Launch();
			}
		}

		DrawGhosts();

		b3Vec3 d = GetDirection();
		b3Vec3 tail = m_start + 0.5f * b3Vec3_axisY;
		DrawArrow( b3ToPos( tail ), b3ToPos( tail + 1.5f * d ), MakeColor( b3_colorYellow ) );

		DrawTextLine( "fan %d, apex dihedral angles %.1f to %.1f deg, completed runs %d", m_fanCounts[m_fanIndex], m_minDihedral,
					  m_maxDihedral, m_runCount );
		DrawGhostSummary();

		for ( int i = 0; i < m_headingCount; ++i )
		{
			int degrees = i * 360 / m_headingCount;
			const RunStats& stats = m_last[i];
			if ( stats.valid )
			{
				DrawTextLine( "heading %3d deg: ghosts %d (vertex %d), energy loss %5.1f%%, peak vy %.2f m/s", degrees, stats.ghosts,
							  stats.vertexGhosts, stats.loss, stats.peakVelocity );
			}
			else
			{
				DrawTextLine( "heading %3d deg: pending", degrees );
			}
		}
	}

	bool DrawControls() override
	{
		bool rebuild = false;
		bool relaunch = false;

		const char* shapeNames[e_hubShapeCount] = { "Peak", "Pit", "Saddle" };
		const char* fanNames[] = { "6", "12", "24", "48" };
		rebuild |= ImGui::Combo( "Hub", &m_hubShape, shapeNames, e_hubShapeCount );
		rebuild |= ImGui::Combo( "Fan", &m_fanIndex, fanNames, IM_ARRAYSIZE( fanNames ) );
		rebuild |= ImGui::SliderFloat( "Slope", &m_slopeDegrees, 1.0f, 20.0f, "%.0f deg" );
		rebuild |= ImGui::SliderFloat( "Jitter", &m_jitter, 0.0f, 0.45f, "%.2f" );
		relaunch |= ImGui::Combo( "Probe", &m_probe, s_probeShapeNames, e_probeShapeCount );
		relaunch |= ImGui::SliderFloat( "Speed", &m_speed, 0.5f, 15.0f, "%.1f m/s" );
		relaunch |= ImGui::SliderFloat( "Yaw", &m_yaw, 0.0f, 45.0f, "%.0f deg" );
		relaunch |= ImGui::SliderFloat( "Offset", &m_offset, 0.0f, 0.5f, "%.2f m" );
		relaunch |= ImGui::Button( "Launch" );

		DrawGhostControls();

		if ( rebuild )
		{
			BuildGround();
		}

		if ( rebuild || relaunch )
		{
			ResetCounters();
			m_heading = 0;
			Launch();
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new HubPeak( context );
	}

	static constexpr int m_fanCounts[4] = { 6, 12, 24, 48 };

	b3BodyId m_probeId;
	int m_hubShape;
	int m_fanIndex;
	float m_slopeDegrees;
	float m_jitter;
	int m_probe;
	float m_speed;
	float m_yaw;
	float m_offset;
	float m_minDihedral;
	float m_maxDihedral;

	b3Vec3 m_start;
	int m_heading;
	int m_runSteps;
	int m_runCount;
	RunStats m_current;
	RunStats m_last[m_headingCount];
};

static int sampleHubPeak = RegisterSample( "Ghost", "Hub Peak", HubPeak::Create );

// A long run through grid vertices. At the 45 degree heading the path meets a vertex in every cell, so
// rare vertex misses add up instead of hiding in one crossing. The wave crests and the default diagonals
// both cross that heading, so the path is a mirror line of the mesh and a clean run keeps its line and
// heading. Diagonals along the path fold the wave cells under it into ridges the body rolls off.
class DiagonalRoll : public GhostSample
{
public:
	static constexpr float m_halfExtent = 20.0f;
	static constexpr float m_runLength = 50.0f;
	static constexpr int m_settleSteps = 10;

	struct RunStats
	{
		float startEnergy;
		float startKinetic;
		float sumSquaredVelocity;
		float distance;
		float loss;
		float lateral;
		float drift;
		int steps;
		int ghosts;
		bool valid;
	};

	explicit DiagonalRoll( SampleContext* context )
		: GhostSample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 20.0f, 25.0f, 8.0f, { -17.0f, 0.0f, -17.0f } );
		}

		// Gentle enough that a sphere at 5 m/s keeps contact over the crests
		m_wave = true;
		m_amplitude = 0.1f;
		m_wavelength = 6.0f;
		m_cellSize = 0.25f;
		m_diagonal = e_diagonalBackward;
		m_probe = e_probeSphere;
		m_speed = 5.0f;
		m_headingDegrees = 45.0f;
		m_offset = 0.0f;
		m_probeId = b3_nullBodyId;

		BuildGround();
		ResetCounters();
		Launch();
	}

	void ResetCounters() override
	{
		GhostSample::ResetCounters();
		m_last = {};
		m_runCount = 0;
	}

	// Crests run along x + z = constant
	float GetHeight( float x, float z ) const
	{
		if ( m_wave == false )
		{
			return 0.0f;
		}

		return m_amplitude * sinf( 2.0f * B3_PI * ( x + z ) / ( sqrtf( 2.0f ) * m_wavelength ) );
	}

	void BuildGround()
	{
		// An even count keeps a vertex at the origin, so the heading line through it meets the vertices
		int halfCount = b3MaxInt( 1, (int)( m_halfExtent / m_cellSize + 0.5f ) );
		m_actualCellSize = m_halfExtent / (float)halfCount;
		std::vector<float> samples = MakeSamples( -m_halfExtent, m_halfExtent, 2 * halfCount );

		std::vector<b3Vec3> vertices;
		std::vector<int32_t> indices;
		AddGrid( vertices, indices, samples, samples, [&]( int i, int j ) { return GetHeight( samples[i], samples[j] ); },
				 m_diagonal );
		CreateGround( vertices, indices );
	}

	b3Vec3 GetDirection() const
	{
		float angle = m_headingDegrees * B3_DEG_TO_RAD;
		return { cosf( angle ), 0.0f, sinf( angle ) };
	}

	void Launch()
	{
		if ( B3_IS_NON_NULL( m_probeId ) )
		{
			b3DestroyBody( m_probeId );
		}

		b3Vec3 d = GetDirection();
		b3Vec3 perp = { -d.z, 0.0f, d.x };
		b3Vec3 start = -0.5f * m_runLength * d + m_offset * perp;
		start.y = GetHeight( start.x, start.z ) + GetProbeRadius( m_probe ) + 0.01f;
		m_start = start;

		m_probeId = CreateProbe( m_probe, b3ToPos( start ), d, 0.0f, m_speed );
		m_followId = m_probeId;
		m_lastHeight = start.y;

		m_current = {};
		m_current.startEnergy = GetSpecificEnergy( m_probeId, b3World_GetGravity( m_worldId ) );
		m_current.startKinetic = GetSpecificKinetic( m_probeId );
		ClearHistory();
	}

	void Finish( RunStats& stats ) const
	{
		b3Vec3 d = GetDirection();
		b3Vec3 perp = { -d.z, 0.0f, d.x };
		b3Vec3 position = b3ToVec3( b3Body_GetWorldCenter( m_probeId ) );
		b3Vec3 velocity = b3Body_GetLinearVelocity( m_probeId );

		float lost = stats.startEnergy - GetSpecificEnergy( m_probeId, b3World_GetGravity( m_worldId ) );
		stats.loss = stats.startKinetic > 0.0f ? 100.0f * lost / stats.startKinetic : 0.0f;
		stats.lateral = b3Dot( position - m_start, perp );
		stats.drift = atan2f( b3Dot( velocity, perp ), b3Dot( velocity, d ) ) / B3_DEG_TO_RAD;
	}

	void Step() override
	{
		b3Pos center = b3Body_GetWorldCenter( m_probeId );

		Sample::Step();

		if ( m_didStep )
		{
			m_current.ghosts += CountGhosts( m_probeId, center );

			b3Vec3 position = b3ToVec3( b3Body_GetWorldCenter( m_probeId ) );
			float verticalSpeed = GetVerticalSpeed( position.y );
			m_current.steps += 1;

			// Skip the drop onto the ground
			if ( m_current.steps > m_settleSteps )
			{
				m_current.sumSquaredVelocity += verticalSpeed * verticalSpeed;
			}
			m_current.distance = b3Dot( position - m_start, GetDirection() );
			PushHistory( verticalSpeed );

			float edge = m_halfExtent - 0.5f;
			bool done = m_current.distance > m_runLength || b3AbsFloat( position.x ) > edge || b3AbsFloat( position.z ) > edge ||
						position.y < -3.0f || m_current.steps > 1800;
			if ( done )
			{
				Finish( m_current );
				m_current.valid = true;
				m_last = m_current;
				m_runCount += 1;
				Launch();
			}
		}

		DrawGhosts();

		b3Vec3 d = GetDirection();
		b3Vec3 lift = { 0.0f, m_wave ? m_amplitude + 0.02f : 0.02f, 0.0f };
		DrawLine( b3ToPos( m_start + lift ), b3ToPos( m_start + m_runLength * d + lift ), MakeColor( b3_colorYellow ) );

		DrawTextLine( "cell %.3f m, completed runs %d", m_actualCellSize, m_runCount );
		DrawGhostSummary();

		RunStats current = m_current;
		Finish( current );
		DrawRun( "current", current );
		if ( m_last.valid )
		{
			DrawRun( "last", m_last );
		}
	}

	void DrawRun( const char* label, const RunStats& stats )
	{
		int sampleCount = stats.steps - m_settleSteps;
		float rms = sampleCount > 0 ? sqrtf( stats.sumSquaredVelocity / (float)sampleCount ) : 0.0f;
		float perMeter = stats.distance > 0.0f ? (float)stats.ghosts / stats.distance : 0.0f;
		DrawTextLine( "%s: %.1f m, ghosts %d (%.2f per m), energy loss %.1f%%, rms vy %.3f m/s, lateral %.3f m, drift %.2f deg",
					  label, stats.distance, stats.ghosts, perMeter, stats.loss, rms, stats.lateral, stats.drift );
	}

	bool DrawControls() override
	{
		bool rebuild = false;
		bool relaunch = false;

		rebuild |= ImGui::Checkbox( "Wave", &m_wave );
		rebuild |= ImGui::SliderFloat( "Amplitude", &m_amplitude, 0.0f, 0.5f, "%.2f m" );
		rebuild |= ImGui::SliderFloat( "Wavelength", &m_wavelength, 1.0f, 10.0f, "%.1f m" );
		rebuild |= ImGui::SliderFloat( "Cell Size", &m_cellSize, 0.2f, 1.0f, "%.2f m" );
		rebuild |= ImGui::Combo( "Diagonals", &m_diagonal, s_diagonalNames, e_diagonalModeCount );
		relaunch |= ImGui::Combo( "Probe", &m_probe, s_probeShapeNames, e_probeShapeCount );
		relaunch |= ImGui::SliderFloat( "Speed", &m_speed, 0.5f, 15.0f, "%.1f m/s" );
		relaunch |= ImGui::SliderFloat( "Heading", &m_headingDegrees, 0.0f, 45.0f, "%.0f deg" );
		relaunch |= ImGui::SliderFloat( "Offset", &m_offset, 0.0f, 0.5f, "%.2f m" );
		relaunch |= ImGui::Button( "Launch" );

		DrawGhostControls();

		if ( rebuild )
		{
			BuildGround();
		}

		if ( rebuild || relaunch )
		{
			ResetCounters();
			Launch();
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new DiagonalRoll( context );
	}

	b3BodyId m_probeId;
	bool m_wave;
	float m_amplitude;
	float m_wavelength;
	float m_cellSize;
	float m_actualCellSize;
	int m_diagonal;
	int m_probe;
	float m_speed;
	float m_headingDegrees;
	float m_offset;

	b3Vec3 m_start;
	RunStats m_current;
	RunStats m_last;
	int m_runCount;
};

static int sampleDiagonalRoll = RegisterSample( "Ghost", "Diagonal Roll", DiagonalRoll::Create );

// Large bodies over thousands of triangles of a fine mesh, and a pile of small boxes for the everyday
// cost. Engines that cap the triangles or features they consider per pair can let ghosts back in where
// the cost is highest. Step time comes from the world profile, so the ghost count is not in it.
class FineMeshLoad : public GhostSample
{
public:
	static constexpr int m_timeCapacity = 60;
	static constexpr int m_pileCount = 64;

	enum LoadMode
	{
		e_largeBodies,
		e_pile,
		e_loadModeCount
	};

	struct LargeStats
	{
		int manifolds;
		int points;
		int ghosts;
		float peakSpeed;
	};

	explicit FineMeshLoad( SampleContext* context )
		: GhostSample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 30.0f, 30.0f, 30.0f, { 0.0f, 0.0f, 0.0f } );
		}

		m_follow = false;
		m_showHistory = false;
		m_mode = e_largeBodies;
		m_cellIndex = 0;
		m_speed = 3.0f;
		m_countGhosts = true;
		m_wallsId = b3_nullBodyId;

		Build();
	}

	void ResetCounters() override
	{
		GhostSample::ResetCounters();
		m_timeCount = 0;
		m_timeIndex = 0;
		m_maxStepTime = 0.0f;
		m_stepGhosts = 0;
		memset( m_largeStats, 0, sizeof( m_largeStats ) );
	}

	void DestroyBodies()
	{
		for ( b3BodyId bodyId : m_bodyIds )
		{
			b3DestroyBody( bodyId );
		}
		m_bodyIds.clear();

		if ( B3_IS_NON_NULL( m_wallsId ) )
		{
			b3DestroyBody( m_wallsId );
			m_wallsId = b3_nullBodyId;
		}
	}

	void Build()
	{
		DestroyBodies();

		std::vector<b3Vec3> vertices;
		std::vector<int32_t> indices;

		if ( m_mode == e_largeBodies )
		{
			float cellSize = m_cellSizes[m_cellIndex];
			int xCount = (int)( 32.0f / cellSize + 0.5f );
			int zCount = (int)( 20.0f / cellSize + 0.5f );
			std::vector<float> xs = MakeSamples( -16.0f, 16.0f, xCount );
			std::vector<float> zs = MakeSamples( -10.0f, 10.0f, zCount );
			AddGrid( vertices, indices, xs, zs, []( int, int ) { return 0.0f; }, e_diagonalAlternate );
			CreateGround( vertices, indices );
			LaunchLarge();
		}
		else
		{
			std::vector<float> samples = MakeSamples( -10.0f, 10.0f, 80 );
			auto height = [&]( int i, int j ) {
				float k = 2.0f * B3_PI / 4.0f;
				return 0.3f * sinf( k * samples[i] ) * sinf( k * samples[j] );
			};
			AddGrid( vertices, indices, samples, samples, height, e_diagonalAlternate );
			CreateGround( vertices, indices );
			CreatePile();
		}

		ResetCounters();
	}

	void LaunchLarge()
	{
		DestroyBodies();

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.linearVelocity = { m_speed, 0.0f, 0.0f };
		bodyDef.enableSleep = false;

		// Recycling would skip the collision under test on some steps and make the step time bimodal
		bodyDef.enableContactRecycling = false;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.0f;

		// A 4 m box over about 3200 triangles at the finest cells
		bodyDef.position = { -13.0f, 0.5f, -5.0f };
		bodyDef.name = "big box";
		b3BodyId boxId = b3CreateBody( m_worldId, &bodyDef );
		b3BoxHull box = b3MakeBoxHull( 2.0f, 0.5f, 2.0f );
		b3CreateHullShape( boxId, &shapeDef, &box.base );
		m_bodyIds.push_back( boxId );

		// A 6 m capsule lying across the direction of travel
		bodyDef.position = { -13.0f, 0.5f, 5.0f };
		bodyDef.name = "long capsule";
		b3BodyId capsuleId = b3CreateBody( m_worldId, &bodyDef );
		b3Capsule capsule = { { 0.0f, 0.0f, -2.5f }, { 0.0f, 0.0f, 2.5f }, 0.5f };
		b3CreateCapsuleShape( capsuleId, &shapeDef, &capsule );
		m_bodyIds.push_back( capsuleId );

		// Ghosts accumulate over passes, the peak is per pass
		m_largeStats[0].peakSpeed = 0.0f;
		m_largeStats[1].peakSpeed = 0.0f;
		m_passSteps = 0;
	}

	void CreatePile()
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.name = "walls";
		m_wallsId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull walls[4] = {
			b3MakeOffsetBoxHull( 10.5f, 1.5f, 0.5f, { 0.0f, 1.0f, -10.5f } ),
			b3MakeOffsetBoxHull( 10.5f, 1.5f, 0.5f, { 0.0f, 1.0f, 10.5f } ),
			b3MakeOffsetBoxHull( 0.5f, 1.5f, 10.5f, { -10.5f, 1.0f, 0.0f } ),
			b3MakeOffsetBoxHull( 0.5f, 1.5f, 10.5f, { 10.5f, 1.0f, 0.0f } ),
		};
		for ( b3BoxHull& wall : walls )
		{
			b3CreateHullShape( m_wallsId, &shapeDef, &wall.base );
		}

		b3BoxHull box = b3MakeBoxHull( 0.1f, 0.1f, 0.1f );
		bodyDef.type = b3_dynamicBody;
		bodyDef.name = "pile box";
		float spacing = 0.3f;
		float start = -0.5f * spacing * ( m_pileCount - 1 );
		for ( int i = 0; i < m_pileCount; ++i )
		{
			for ( int j = 0; j < m_pileCount; ++j )
			{
				int index = m_pileCount * i + j;
				bodyDef.position = { start + spacing * i, 1.5f, start + spacing * j };
				bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Normalize( { 1.0f, 2.0f, 3.0f } ), 0.37f * (float)index );
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
				b3CreateHullShape( bodyId, &shapeDef, &box.base );
				m_bodyIds.push_back( bodyId );
			}
		}
	}

	void Step() override
	{
		int bodyCount = (int)m_bodyIds.size();
		m_centers.resize( bodyCount );
		for ( int i = 0; i < bodyCount; ++i )
		{
			m_centers[i] = b3Body_GetWorldCenter( m_bodyIds[i] );
		}

		Sample::Step();

		if ( m_didStep )
		{
			b3Profile profile = b3World_GetProfile( m_worldId );
			m_stepTimes[m_timeIndex] = profile.step;
			m_collideTimes[m_timeIndex] = profile.collide;
			m_timeIndex = ( m_timeIndex + 1 ) % m_timeCapacity;
			m_timeCount = b3MinInt( m_timeCount + 1, m_timeCapacity );
			m_maxStepTime = b3MaxFloat( m_maxStepTime, profile.step );

			m_stepGhosts = 0;
			if ( m_countGhosts )
			{
				for ( int i = 0; i < bodyCount; ++i )
				{
					int ghostCount = CountGhosts( m_bodyIds[i], m_centers[i] );
					m_stepGhosts += ghostCount;
					if ( m_mode == e_largeBodies )
					{
						m_largeStats[i].ghosts += ghostCount;
					}
				}
			}

			if ( m_mode == e_largeBodies )
			{
				UpdateLarge();
			}
		}

		DrawGhosts();

		float stepTime = 0.0f;
		float collideTime = 0.0f;
		for ( int i = 0; i < m_timeCount; ++i )
		{
			stepTime += m_stepTimes[i];
			collideTime += m_collideTimes[i];
		}
		float scale = m_timeCount > 0 ? 1.0f / (float)m_timeCount : 0.0f;

		DrawTextLine( "step %.2f ms, collide %.2f ms, averaged over %d steps, max step %.2f ms", scale * stepTime,
					  scale * collideTime, m_timeCount, m_maxStepTime );
		DrawGhostSummary();

		if ( m_mode == e_largeBodies )
		{
			float cellSize = m_cellSizes[m_cellIndex];
			int under = (int)( 16.0f / ( 0.5f * cellSize * cellSize ) );
			DrawTextLine( "triangles under the box: about %d", under );
			const char* names[2] = { "box", "capsule" };
			for ( int i = 0; i < 2; ++i )
			{
				const LargeStats& stats = m_largeStats[i];
				DrawTextLine( "%s: manifolds %d, points %d, ghosts %d, peak off plane speed %.3f m/s", names[i], stats.manifolds,
							  stats.points, stats.ghosts, stats.peakSpeed );
			}
		}
		else
		{
			b3Counters counters = b3World_GetCounters( m_worldId );
			DrawTextLine( "bodies %d, awake %d, contacts %d, ghosts this step %d", (int)m_bodyIds.size(),
						  b3World_GetAwakeBodyCount( m_worldId ), counters.contactCount, m_stepGhosts );
		}
	}

	// Off plane speed includes spin, since a ghost on flat ground tips or lifts the body. The first steps
	// settle the bodies onto the grid.
	void UpdateLarge()
	{
		m_passSteps += 1;
		bool settled = m_passSteps > 10;
		bool done = true;
		for ( int i = 0; i < 2; ++i )
		{
			b3BodyId bodyId = m_bodyIds[i];
			LargeStats& stats = m_largeStats[i];

			b3ContactData contacts[4];
			int contactCount = b3Body_GetContactData( bodyId, contacts, 4 );
			stats.manifolds = 0;
			stats.points = 0;
			for ( int j = 0; j < contactCount; ++j )
			{
				stats.manifolds += contacts[j].manifoldCount;
				for ( int k = 0; k < contacts[j].manifoldCount; ++k )
				{
					stats.points += contacts[j].manifolds[k].pointCount;
				}
			}

			b3Vec3 v = b3Body_GetLinearVelocity( bodyId );
			b3Vec3 w = b3Body_GetAngularVelocity( bodyId );
			float offPlane = b3AbsFloat( v.y ) + b3Length( w );
			stats.peakSpeed = settled ? b3MaxFloat( stats.peakSpeed, offPlane ) : 0.0f;

			done = done && (float)b3Body_GetPosition( bodyId ).x > 13.0f;
		}

		if ( done )
		{
			LaunchLarge();
		}
	}

	bool DrawControls() override
	{
		bool rebuild = false;

		const char* modeNames[e_loadModeCount] = { "Large Bodies", "Pile" };
		const char* cellNames[] = { "0.1 m", "0.25 m", "0.5 m" };
		rebuild |= ImGui::Combo( "Mode", &m_mode, modeNames, e_loadModeCount );
		if ( m_mode == e_largeBodies )
		{
			rebuild |= ImGui::Combo( "Cell Size", &m_cellIndex, cellNames, IM_ARRAYSIZE( cellNames ) );
			if ( ImGui::SliderFloat( "Speed", &m_speed, 0.0f, 10.0f, "%.1f m/s" ) )
			{
				LaunchLarge();
			}
		}

		rebuild |= ImGui::Button( "Restart" );
		ImGui::Checkbox( "Count Ghosts", &m_countGhosts );

		DrawGhostControls();

		if ( rebuild )
		{
			Build();
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new FineMeshLoad( context );
	}

	static constexpr float m_cellSizes[3] = { 0.1f, 0.25f, 0.5f };

	int m_mode;
	int m_cellIndex;
	float m_speed;
	bool m_countGhosts;

	std::vector<b3BodyId> m_bodyIds;
	std::vector<b3Pos> m_centers;
	b3BodyId m_wallsId;

	LargeStats m_largeStats[2];
	int m_passSteps;
	float m_stepTimes[m_timeCapacity];
	float m_collideTimes[m_timeCapacity];
	int m_timeCount;
	int m_timeIndex;
	float m_maxStepTime;
	int m_stepGhosts;
};

static int sampleFineMeshLoad = RegisterSample( "Ghost", "Fine Mesh Load", FineMeshLoad::Create );
