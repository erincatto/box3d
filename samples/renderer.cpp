// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "renderer.h"

#include "camera.h"
#include "font.h"
#include "sample.h"
#include "scene.h"
#include "shader.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

// clang-format off
#include <glad/glad.h>
#include <GLFW/glfw3.h>
// clang-format on

#include "box3d/box3d.h"

#include <algorithm>
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <vector>

struct PointData
{
	b3Vec3 position;
	float size;
	RGBA8 rgba;
};

struct PointDraw
{
	enum
	{
		e_batchSize = 2048
	};

	std::vector<PointData> Points;

	uint32_t programId;

	uint32_t vaoId;
	uint32_t vboId;

	int ProjectionUniform;
	int ViewUniform;
	int SizeUniform;
};

PointDraw* CreateDebugPoints()
{
	const char* vs = R"(
		#version 330 core
		uniform mat4 projection;
		uniform mat4 view;
		layout(location = 0) in vec3 point;
		layout(location = 1) in vec4 color;
		layout(location = 2) in float size;
		out vec4 f_color;
		void main(void)
		{
			gl_Position = projection * view * vec4(point, 1.0f);
			f_color = color;
			gl_PointSize = size;
		}
		)";

	const char* fs = R"(
		#version 330 core
		in vec4 f_color;
		out vec4 color;
		void main(void)
		{
			color = f_color;
		}
	)";

	PointDraw* draw = new PointDraw;

	draw->programId = CreateProgramFromStrings( vs, fs );
	draw->ProjectionUniform = GetUniformLocation( draw->programId, "projection" );
	draw->ViewUniform = GetUniformLocation( draw->programId, "view" );

	constexpr int vertexAttribute = 0;
	constexpr int sizeAttribute = 1;
	constexpr int colorAttribute = 2;

	// Generate
	glGenVertexArrays( 1, &draw->vaoId );
	glGenBuffers( 1, &draw->vboId );

	glBindVertexArray( draw->vaoId );
	glEnableVertexAttribArray( vertexAttribute );
	glEnableVertexAttribArray( sizeAttribute );
	glEnableVertexAttribArray( colorAttribute );

	// Vertex buffer
	glBindBuffer( GL_ARRAY_BUFFER, draw->vboId );

	glBufferData( GL_ARRAY_BUFFER, PointDraw::e_batchSize * sizeof( PointData ), nullptr, GL_DYNAMIC_DRAW );

	glVertexAttribPointer( vertexAttribute, 3, GL_FLOAT, GL_FALSE, sizeof( PointData ), (void*)offsetof( PointData, position ) );
	glVertexAttribPointer( colorAttribute, 4, GL_FLOAT, GL_FALSE, sizeof( PointData ), (void*)offsetof( PointData, size ) );
	// save bandwidth by expanding color to floats in the shader
	glVertexAttribPointer( sizeAttribute, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( PointData ), (void*)offsetof( PointData, rgba ) );

	CheckOpenGL();

	// Cleanup
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );

	return draw;
}

void DestroyDebugPoints( PointDraw* draw )
{
	if ( draw->vaoId )
	{
		glDeleteVertexArrays( 1, &draw->vaoId );
		glDeleteBuffers( 1, &draw->vboId );
		draw->vaoId = 0;
	}

	DestroyShader( draw->programId );
	draw->programId = 0;

	delete draw;
}

void AddPoint( PointDraw* draw, b3Vec3 point, float size, b3HexColor color )
{
	draw->Points.push_back( { point, size, MakeRGBA8( color ) } );
}

void FlushPoints( PointDraw* draw, Camera* camera )
{
	if ( draw->Points.size() == 0 )
	{
		return;
	}

	glDepthFunc( GL_LEQUAL );

	glUseProgram( draw->programId );

	Matrix4 projection = camera->GetProjectionMatrix();
	Matrix4 view = camera->GetViewMatrix();

	SetUniformMatrix4( draw->ProjectionUniform, projection );
	SetUniformMatrix4( draw->ViewUniform, view );

	glBindVertexArray( draw->vaoId );

	int totalCount = (int)draw->Points.size();
	PointData* points = draw->Points.data();

	while ( totalCount > 0 )
	{
		int count = b3MinInt( totalCount, PointDraw::e_batchSize );

		glBindBuffer( GL_ARRAY_BUFFER, draw->vboId );
		// todo macOS optimization?
		// glBufferData( GL_ARRAY_BUFFER, PointDraw::e_batchSize * sizeof( PointData ), nullptr, GL_DYNAMIC_DRAW );
		glBufferSubData( GL_ARRAY_BUFFER, 0, count * sizeof( PointData ), points );

		glEnable( GL_PROGRAM_POINT_SIZE );
		glDrawArrays( GL_POINTS, 0, count );
		glDisable( GL_PROGRAM_POINT_SIZE );

		points += count;
		totalCount -= count;
	}

	CheckOpenGL();

	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	glUseProgram( 0 );

	draw->Points.clear();
}

struct LinePoint
{
	b3Vec3 position;
	RGBA8 rgba;
};

struct LineDraw
{
	enum
	{
		e_batchSize = 2048
	};

	std::vector<LinePoint> points;

	uint32_t programId;

	uint32_t VaoId;
	uint32_t VboId;

	int ProjectionUniform;
	int ViewUniform;
};

LineDraw* CreateDebugLines()
{
	const char* vs = R"(
		#version 330 core
		uniform mat4 projection;
		uniform mat4 view;
		layout(location = 0) in vec3 point;
		layout(location = 1) in vec4 color;
		out vec4 f_color;
		void main(void)
		{
			gl_Position = projection * view * vec4(point, 1.0f);
			f_color = color;
		}
	)";

	const char* fs = R"(
		#version 330 core
		in vec4 f_color;
		out vec4 color;
		void main(void)
		{
			color = f_color;
		}
	)";

	LineDraw* draw = new LineDraw;

	draw->programId = CreateProgramFromStrings( vs, fs );
	draw->ProjectionUniform = GetUniformLocation( draw->programId, "projection" );
	draw->ViewUniform = GetUniformLocation( draw->programId, "view" );

	constexpr int vertexAttribute = 0;
	constexpr int colorAttribute = 1;

	// Generate
	glGenVertexArrays( 1, &draw->VaoId );
	glGenBuffers( 1, &draw->VboId );

	glBindVertexArray( draw->VaoId );
	glEnableVertexAttribArray( vertexAttribute );
	glEnableVertexAttribArray( colorAttribute );

	// Vertex buffer
	glBindBuffer( GL_ARRAY_BUFFER, draw->VboId );
	glBufferData( GL_ARRAY_BUFFER, LineDraw::e_batchSize * sizeof( LinePoint ), nullptr, GL_DYNAMIC_DRAW );

	glVertexAttribPointer( vertexAttribute, 3, GL_FLOAT, GL_FALSE, sizeof( LinePoint ),
						   (void*)offsetof( LinePoint, position ) );
	// save bandwidth by expanding color to floats in the shader
	glVertexAttribPointer( colorAttribute, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( LinePoint ),
						   (void*)offsetof( LinePoint, rgba ) );

	CheckOpenGL();

	// Cleanup
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );

	return draw;
}

void DestroyDebugLines( LineDraw* draw )
{
	if ( draw->VaoId )
	{
		glDeleteVertexArrays( 1, &draw->VaoId );
		glDeleteBuffers( 1, &draw->VboId );
		draw->VaoId = 0;
	}

	DestroyShader( draw->programId );
	draw->programId = 0;

	delete draw;
}

void AddLine( LineDraw* draw, b3Vec3 point1, b3Vec3 point2, b3HexColor color )
{
	draw->points.push_back( { point1, MakeRGBA8( color ) } );
	draw->points.push_back( { point2, MakeRGBA8( color ) } );
}

void FlushLines( LineDraw* draw, Camera* camera )
{
	if ( draw->points.size() == 0 )
	{
		return;
	}

	// WARNING: very slow
	//glEnable( GL_LINE_SMOOTH );

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	glUseProgram( draw->programId );

	Matrix4 projection = camera->GetProjectionMatrix();
	Matrix4 view = camera->GetViewMatrix();

	SetUniformMatrix4( draw->ProjectionUniform, projection );
	SetUniformMatrix4( draw->ViewUniform, view );

	glBindVertexArray( draw->VaoId );
	glBindBuffer( GL_ARRAY_BUFFER, draw->VboId );

	int count = (int)draw->points.size();
	LinePoint* points = draw->points.data();

	while ( count > 0 )
	{
		int batchCount = b3MinInt( count, LineDraw::e_batchSize );

		// todo buffer orphaning for macOS?
		// glBufferData( GL_ARRAY_BUFFER, LineDraw::e_batchSize * sizeof( LinePoint ), nullptr, GL_DYNAMIC_DRAW );
		glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( LinePoint ), points );
		glDrawArrays( GL_LINES, 0, batchCount );

		points += batchCount;
		count -= batchCount;
	}

	CheckOpenGL();

	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	glUseProgram( 0 );
	glDisable( GL_BLEND );

	draw->points.clear();
}

static DebugShape* CreateDebugSphere( const b3Sphere* sphere )
{
	DebugShape* s = new DebugShape;
	s->type = DebugShapeType::sphere;
	s->sphere.center = sphere->center;
	s->sphere.radius = sphere->radius;
	return s;
}

static DebugShape* CreateDebugCapsule( const b3Capsule* capsule, Arena arena )
{
	DebugShape* s = new DebugShape;
	s->type = DebugShapeType::capsule;
	s->capsule.center1 = capsule->center1;
	s->capsule.center2 = capsule->center2;
	s->capsule.radius = capsule->radius;
	return s;
}

static DebugShape* CreateDebugCompound(Scene* scene, const b3Compound* compound, Arena arena )
{
	DebugShape* s = new DebugShape;
	memset( s, 0, sizeof( DebugShape ) );
	s->type = DebugShapeType::compound;

	// Capsules
	s->compound.capsuleCount = compound->capsuleCount;
	DebugCapsule* debugCapsules = new DebugCapsule[s->compound.capsuleCount];
	for (int i = 0; i < s->compound.capsuleCount; ++i)
	{
		b3CompoundCapsule element = b3GetCompoundCapsule( compound, i );

		debugCapsules[i].center1 = element.capsule.center1;
		debugCapsules[i].center2 = element.capsule.center2;
		debugCapsules[i].radius = element.capsule.radius;
	}
	s->compound.capsules = debugCapsules;

	// Hulls
	s->compound.hullCount = compound->hullCount;
	DebugHull* debugHulls = new DebugHull[s->compound.hullCount];
	for (int i = 0; i < s->compound.hullCount; ++i)
	{
		b3CompoundHull hull = b3GetCompoundHull( compound, i );

		uint32_t id = CreateInstanceHull( scene, hull.hull, arena );
		debugHulls[i].transform = hull.transform;
		debugHulls[i].id = id;
	}
	s->compound.hulls = debugHulls;

	// Meshes
	s->compound.meshCount = compound->meshCount;
	DebugMesh* debugMeshes = new DebugMesh[s->compound.meshCount];
	for ( int i = 0; i < s->compound.meshCount; ++i )
	{
		b3CompoundMesh mesh = b3GetCompoundMesh( compound, i );

		uint32_t id = CreateInstanceMesh( scene, mesh.meshData, arena );
		debugMeshes[i].transform = mesh.transform;
		debugMeshes[i].scale = mesh.scale;
		debugMeshes[i].id = id;
	}
	s->compound.meshes = debugMeshes;

	// Spheres
	s->compound.sphereCount = compound->sphereCount;
	DebugSphere* debugSpheres = new DebugSphere[s->compound.sphereCount];
	for (int i = 0; i < s->compound.sphereCount; ++i)
	{
		b3CompoundSphere sphere = b3GetCompoundSphere( compound, i );
		debugSpheres[i].center = sphere.sphere.center;
		debugSpheres[i].radius = sphere.sphere.radius;
	}
	s->compound.spheres = debugSpheres;

	return s;
}

static void DestroyDebugCompound( DebugCompound* compound )
{
	delete[] compound->capsules;
	delete[] compound->hulls;
	delete[] compound->meshes;
	delete[] compound->spheres;
}

static DebugShape* CreateDebugHull( Scene* scene, const b3Hull* hull, Arena arena )
{
	uint32_t id = CreateInstanceHull( scene, hull, arena );

	DebugShape* s = new DebugShape;
	s->type = DebugShapeType::hull;
	s->hull.id = id;
	s->hull.transform = b3Transform_identity;
	return s;
}

static DebugShape* CreateDebugMesh( Scene* scene, const b3Mesh* mesh, Arena arena )
{
	uint32_t id = CreateInstanceMesh( scene, mesh->data, arena );

	DebugShape* s = new DebugShape;
	s->type = DebugShapeType::mesh;
	s->mesh.id = id;
	s->mesh.transform = b3Transform_identity;
	s->mesh.scale = mesh->scale;
	return s;
}

static DebugShape* CreateDebugHeightField( Scene* scene, const b3HeightField* heightField, Arena arena )
{
	uint32_t id = CreateInstanceHeightField( scene, heightField, arena );

	DebugShape* s = new DebugShape;
	s->type = DebugShapeType::heightField;
	s->heightField.id = id;
	return s;
}

void* CreateDebugShape( const b3DebugShape* debugShape, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DebugShape* s = nullptr;
	switch ( debugShape->type )
	{
		case b3_capsuleShape:
			s = CreateDebugCapsule( debugShape->capsule, sampleContext->arena );
			break;

		case b3_compoundShape:
			s = CreateDebugCompound( sampleContext->scene, debugShape->compound, sampleContext->arena );
			break;

		case b3_hullShape:
			s = CreateDebugHull( sampleContext->scene, debugShape->hull, sampleContext->arena );
			break;

		case b3_meshShape:
			s = CreateDebugMesh( sampleContext->scene, debugShape->mesh, sampleContext->arena );
			break;

		case b3_heightShape:
			s = CreateDebugHeightField( sampleContext->scene, debugShape->heightField, sampleContext->arena );
			break;

		case b3_sphereShape:
			s = CreateDebugSphere( debugShape->sphere );
			break;

		default:
			break;
	}

	if (s != nullptr)
	{
		b3BodyId bodyId = b3Shape_GetBody( debugShape->shapeId );
		s->bodyType = b3Body_GetType( bodyId );
	}
	else
	{
		s->bodyType = b3_staticBody;
	}

	return s;
}

void DestroyDebugShape( void* userShape, void* context )
{
	DebugShape* s = static_cast<DebugShape*>( userShape );
	switch ( s->type )
	{
		case DebugShapeType::compound:
			DestroyDebugCompound( &s->compound );
			break;

		default:
			break;
	}

	delete s;
}

extern Font* s_font;
void DrawText( int x, int y, b3HexColor color, const char* text )
{
	s_font->AddText( x, y, color, text );
}

void DrawFormat( int x, int y, b3HexColor color, const char* format, ... )
{
	va_list args;
	va_start( args, format );
	char text[512];
	vsnprintf( text, sizeof( text ), format, args );
	DrawText( x, y, color, text );
	va_end( args );
}

void DrawWorldString( Camera* camera, b3Vec3 point, b3HexColor color, const char* format, ... )
{
	float x, y;
	bool valid = camera->WorldToScreen( x, y, point );
	if ( valid )
	{
		va_list args;
		va_start( args, format );
		char text[512];
		vsnprintf( text, sizeof( text ), format, args );
		DrawText( (int)x, (int)y, color, text );
		va_end( args );
	}
}

static void DrawDisc( Scene* scene, b3Vec3 center, b3Vec3 normal, float radius, b3HexColor color )
{
	b3Vec3 tangent1 = b3Perp( normal );
	b3Vec3 tangent2 = b3Cross( normal, tangent1 );

	int n = 9;
	float delta = B3_PI / (float)n;
	b3CosSin cs = b3ComputeCosSin( delta );
	float x1 = radius, y1 = 0.0f;
	b3Vec3 vertex1 = center + x1 * tangent1 + y1 * tangent2;

	for ( int i = 0; i < 2 * n; ++i )
	{
		float x2 = cs.cosine * x1 - cs.sine * y1;
		float y2 = cs.sine * x1 + cs.cosine * y1;
		b3Vec3 vertex2 = center + x2 * tangent1 + y2 * tangent2;

		DrawLine( scene, vertex1, vertex2, color );

		x1 = x2;
		y1 = y2;
		vertex1 = vertex2;
	}
}

static void DrawArc( Scene* scene, b3Vec3 center, b3Vec3 normal, float radius, b3Vec3 start, float maxAngle, b3HexColor color )
{
	b3Vec3 tangent1 = b3Normalize( start );
	b3Vec3 tangent2 = b3Cross( normal, tangent1 );

	for ( float angle = 0.0f; angle < maxAngle; angle += 5.0f )
	{
		float alpha1 = B3_DEG_TO_RAD * ( angle + 0.0f );
		b3Vec3 vertex1 = center + b3Cos( alpha1 ) * radius * tangent1 + b3Sin( alpha1 ) * radius * tangent2;
		float alpha2 = B3_DEG_TO_RAD * ( angle + 5.0f );
		b3Vec3 vertex2 = center + b3Cos( alpha2 ) * radius * tangent1 + b3Sin( alpha2 ) * radius * tangent2;

		DrawLine( scene, vertex1, vertex2, color );
	}
}

void DrawGrid( Scene* scene, int size )
{
	float grid = float( size );

	for ( int x = -size; x <= size; ++x )
	{
		if ( x == 0 )
		{
			DrawLine( scene, { 0.0f, 0.0f, -grid }, { 0.0f, 0.0f, 0.0f }, b3_colorLightGray );
			DrawLine( scene, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, grid }, b3_colorBlue );
		}
		else if ( ( x % 10 ) == 0 )
		{
			DrawLine( scene, { float( x ), 0.0f, -grid }, { float( x ), 0.0f, grid }, b3_colorDarkGray );
		}
		else
		{
			DrawLine( scene, { float( x ), 0.0f, -grid }, { float( x ), 0.0f, grid }, b3_colorGray );
		}
	}

	for ( int z = -size; z <= size; ++z )
	{
		if ( z == 0 )
		{
			DrawLine( scene, { -grid, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, b3_colorLightGray );
			DrawLine( scene, { 0.0f, 0.0f, 0.0f }, { grid, 0.0f, 0.0f }, b3_colorRed );
		}
		else if ( ( z % 10 ) == 0 )
		{
			DrawLine( scene, { -grid, 0.0f, float( z ) }, { grid, 0.0f, float( z ) }, b3_colorDarkGray );
		}
		else
		{
			DrawLine( scene, { -grid, 0.0f, float( z ) }, { grid, 0.0f, float( z ) }, b3_colorGray );
		}
	}
}

void DrawCross( Scene* scene, b3Vec3 center, float radius, b3HexColor color )
{
	DrawLine( scene, center - b3Vec3{ radius, 0.0, 0.0f }, center + b3Vec3{ radius, 0.0f, 0.0f }, color );
	DrawLine( scene, center - b3Vec3{ 0.0f, radius, 0.0f }, center + b3Vec3{ 0.0f, radius, 0.0f }, color );
	DrawLine( scene, center - b3Vec3{ 0.0f, 0.0f, radius }, center + b3Vec3{ 0.0f, 0.0f, radius }, color );
}

void DrawSphere( Scene* scene, const b3Transform& transform, const b3Sphere& sphere, b3HexColor color )
{
	b3Vec3 center = b3TransformPoint( transform, sphere.center );
	float radius = sphere.radius;

	b3Vec3 axisX = b3RotateVector( transform.q, { 1.0f, 0.0f, 0.0f } );
	b3Vec3 axisY = b3RotateVector( transform.q, { 0.0f, 1.0f, 0.0f } );
	b3Vec3 axisZ = b3RotateVector( transform.q, { 0.0f, 0.0f, 1.0f } );

	DrawDisc( scene, center, axisX, radius, color );
	DrawDisc( scene, center, axisY, radius, color );
	DrawDisc( scene, center, axisZ, radius, color );
}

void DrawCapsule( Scene* scene, const b3Transform& transform, const b3Capsule& capsule, b3HexColor color )
{
	b3Vec3 center1 = b3TransformPoint( transform, capsule.center1 );
	b3Vec3 center2 = b3TransformPoint( transform, capsule.center2 );
	float radius = capsule.radius;

	b3Vec3 normal = b3Normalize( center2 - center1 );
	b3Vec3 tangent1 = b3Perp( normal );
	b3Vec3 tangent2 = b3Cross( normal, tangent1 );

	DrawLine( scene, center1 + radius * tangent1, center2 + radius * tangent1, color );
	DrawLine( scene, center1 + radius * tangent2, center2 + radius * tangent2, color );
	DrawLine( scene, center1 - radius * tangent1, center2 - radius * tangent1, color );
	DrawLine( scene, center1 - radius * tangent2, center2 - radius * tangent2, color );

	DrawArc( scene, center1, -tangent1, radius, tangent2, 180.0f, color );
	DrawArc( scene, center1, tangent2, radius, tangent1, 180.0f, color );
	DrawArc( scene, center2, tangent1, radius, tangent2, 180.0f, color );
	DrawArc( scene, center2, -tangent2, radius, tangent1, 180.0f, color );

	DrawDisc( scene, center1, normal, radius, color );
	DrawDisc( scene, center2, normal, radius, color );
}

void DrawHull( Scene* scene, const b3Transform& transform, const b3Hull* hull, b3HexColor color, bool drawNormals )
{
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );

	for ( int edgeIndex = 0; edgeIndex < hull->edgeCount; edgeIndex += 2 )
	{
		const b3HullHalfEdge* edge = edges + edgeIndex + 0;
		const b3HullHalfEdge* twin = edges + edgeIndex + 1;

		b3Vec3 vertex1 = b3TransformPoint( transform, points[edge->origin] );
		b3Vec3 vertex2 = b3TransformPoint( transform, points[twin->origin] );

		DrawLine( scene, vertex1, vertex2, color );
	}

	if (drawNormals)
	{
		const b3HullFace* faces = b3GetHullFaces( hull );
		const b3Plane* planes = b3GetHullPlanes( hull );
		int faceCount = hull->faceCount;

		for (int i = 0; i < faceCount; ++i)
		{
			uint8_t startEdgeIndex = faces[i].edge;
			uint8_t edgeIndex = startEdgeIndex;

			int count = 0;
			b3Vec3 p1 = b3Vec3_zero;
			do
			{
				const b3HullHalfEdge* edge = edges + edgeIndex;
				p1 += points[edge->origin];
				count += 1;
				edgeIndex = edge->next;
			}
			while ( edgeIndex != startEdgeIndex );

			assert( count > 0 );
			p1 = b3MulSV( 1.0f / count, p1 );
			b3Vec3 p2 = p1 + 0.2f * planes[i].normal;

			p1 = b3TransformPoint( transform, p1 );
			p2 = b3TransformPoint( transform, p2 );

			DrawLine( scene, p1, p2, b3_colorWhite );
			DrawPoint( scene, p1, 6.0f, b3_colorLime );
		}
	}
}

void DrawTransform( Scene* scene, const b3Transform& transform, float length )
{
	b3Vec3 p = transform.p;
	b3Vec3 ax = b3RotateVector( transform.q, { 1.0f, 0.0f, 0.0f } );
	b3Vec3 ay = b3RotateVector( transform.q, { 0.0f, 1.0f, 0.0f } );
	b3Vec3 az = b3RotateVector( transform.q, { 0.0f, 0.0f, 1.0f } );

	DrawLine( scene, p, p + length * ax, b3_colorRed );
	DrawLine( scene, p, p + length * ay, b3_colorGreen );
	DrawLine( scene, p, p + length * az, b3_colorBlue );
}
