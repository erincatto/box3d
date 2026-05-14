// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "scene.h"

#include "../src/core.h"
#include "camera.h"
#include "geo_buffer.h"
#include "renderer.h"
#include "shader.h"
#include "ssao_buffer.h"
#include "utility.h"

#include "box3d/math_functions.h"

#include <algorithm>
#include <assert.h>
#include <glad/glad.h>
#include <stdio.h>
#include <string.h>
#include <unordered_map>
#include <vector>

static const int shadowWidth = 4096;
static const int shadowHeight = 4096;
static constexpr int s_batchSize = 256;

// The number of sides of the base circle. For spheres and capsules.
static constexpr int s_sideCount = 16;

// The number of quad rings going from the middle to the sphere end point
// for example, a value of 2 has 2 rings plus the end cone. For spheres and capsules.
static constexpr int s_ringCount = 8;

struct Cube
{
	b3Transform transform;
	RGBAf color;
	bool invert;
	bool shadowCaster;
};

struct DebugCube
{
	b3Transform transform;
	RGBAf color;
};

struct Edge
{
	int index1;
	int index2;
	int flags;
};

struct EdgeData
{
	b3Vec3 position;
	int flags;
};

struct InstanceShape
{
	int triangleCount;
	uint32_t triangleArrayId;

	// Static and dynamic buffers
	uint32_t triangleBufferIds[2];

	int edgeCount;
	uint32_t edgeArrayId;

	// Static and dynamic buffers
	uint32_t edgeBufferIds[2];
};

struct InstanceData
{
	RGBA8 rgba8;
	Matrix4 matrix;
};

struct Scene
{
	Scene() = default;

	Arena arena;

	std::vector<DebugCube> debugCubes;
	std::vector<InstanceData> sphereInstances;
	std::vector<InstanceData> openHemisphereInstances;
	std::vector<InstanceData> openCylinderInstances;

	// Instances organized by hash
	std::unordered_map<uint32_t, std::vector<InstanceData>> hullInstances;
	std::unordered_map<uint32_t, std::vector<InstanceData>> meshInstances;
	std::unordered_map<uint32_t, std::vector<InstanceData>> heightFieldInstances;

	std::vector<InstanceData> transparentSphereInstances;
	std::vector<InstanceData> transparentOpenHemisphereInstances;
	std::vector<InstanceData> transparentOpenCylinderInstances;
	std::unordered_map<uint32_t, std::vector<InstanceData>> transparentHullInstances;

	uint32_t shadowShader = 0;
	uint32_t meshShader = 0;
	uint32_t meshShadowShader = 0;
	uint32_t meshEdgeShader = 0;
	uint32_t geometryShader = 0;
	uint32_t ssaoShader = 0;
	uint32_t ssaoBlurShader = 0;
	uint32_t deferredShader = 0;
	uint32_t meshForwardShader = 0;
	uint32_t forwardShader = 0;
	uint32_t depthCopyShader = 0;

	uint32_t depthMapFBO = 0;
	uint32_t depthMap = 0;

	PointDraw* pointDraw = nullptr;
	LineDraw* lineDraw = nullptr;
	GeoBuffer* geoBuffer = nullptr;
	SsaoBuffer* ssaoBuffer = nullptr;

	InstanceShape sphere;

	// Building blocks for capsules
	InstanceShape openHemisphere;
	InstanceShape openCylinder;

	std::unordered_map<uint32_t, InstanceShape> sharedHulls;
	std::unordered_map<uint32_t, InstanceShape> sharedMeshes;
	std::unordered_map<uint32_t, InstanceShape> sharedHeightFields;

	SceneAOSettings ao = {};

	b3Vec3 lightPosition{ -16.0f, 40.0f, 10.0f };

	bool useSSAO = false;
	bool useShadow = false;
	bool useGrid = false;
};

// Renders a 1x1 XY quad in NDC
static uint32_t quadVAO = 0;
static uint32_t quadVBO = 0;
static void DrawQuad()
{
	if ( quadVAO == 0 )
	{
		float quadVertices[] = {
			// positions        texture Coords
			// (x, y, z, u, v)
			-1.0f, 1.0f, 0.0f, 0.0f, 1.0f, -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
			1.0f,  1.0f, 0.0f, 1.0f, 1.0f, 1.0f,  -1.0f, 0.0f, 1.0f, 0.0f,
		};

		glGenVertexArrays( 1, &quadVAO );
		glGenBuffers( 1, &quadVBO );
		glBindVertexArray( quadVAO );
		glBindBuffer( GL_ARRAY_BUFFER, quadVBO );
		glBufferData( GL_ARRAY_BUFFER, sizeof( quadVertices ), &quadVertices, GL_STATIC_DRAW );
		glEnableVertexAttribArray( 0 );
		glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof( float ), (void*)0 );
		glEnableVertexAttribArray( 1 );
		glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof( float ), (void*)( 3 * sizeof( float ) ) );
	}

	glBindVertexArray( quadVAO );
	glDrawArrays( GL_TRIANGLE_STRIP, 0, 4 );
	glBindVertexArray( 0 );
}

static uint32_t cubeArrayId = 0;
static uint32_t cubeBufferId = 0;
void DrawCube( Scene* scene, b3Transform transform, b3HexColor color, const Matrix4& view, const Matrix4& projection )
{
	// initialize (if necessary)
	if ( cubeArrayId == 0 )
	{
		// position / normal / uv

		float vertices[] = {
			// back face
			-1.0f, -1.0f, -1.0f, 0.0f, 0.0f, -1.0f, // bottom-left
			1.0f, 1.0f, -1.0f, 0.0f, 0.0f, -1.0f,	// top-right
			1.0f, -1.0f, -1.0f, 0.0f, 0.0f, -1.0f,	// bottom-right
			1.0f, 1.0f, -1.0f, 0.0f, 0.0f, -1.0f,	// top-right
			-1.0f, -1.0f, -1.0f, 0.0f, 0.0f, -1.0f, // bottom-left
			-1.0f, 1.0f, -1.0f, 0.0f, 0.0f, -1.0f,	// top-left
													// front face
			-1.0f, -1.0f, 1.0f, 0.0f, 0.0f, 1.0f,	// bottom-left
			1.0f, -1.0f, 1.0f, 0.0f, 0.0f, 1.0f,	// bottom-right
			1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f,		// top-right
			1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f,		// top-right
			-1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f,	// top-left
			-1.0f, -1.0f, 1.0f, 0.0f, 0.0f, 1.0f,	// bottom-left
													// left face
			-1.0f, 1.0f, 1.0f, -1.0f, 0.0f, 0.0f,	// top-right
			-1.0f, 1.0f, -1.0f, -1.0f, 0.0f, 0.0f,	// top-left
			-1.0f, -1.0f, -1.0f, -1.0f, 0.0f, 0.0f, // bottom-left
			-1.0f, -1.0f, -1.0f, -1.0f, 0.0f, 0.0f, // bottom-left
			-1.0f, -1.0f, 1.0f, -1.0f, 0.0f, 0.0f,	// bottom-right
			-1.0f, 1.0f, 1.0f, -1.0f, 0.0f, 0.0f,	// top-right
													// right face
			1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f,		// top-left
			1.0f, -1.0f, -1.0f, 1.0f, 0.0f, 0.0f,	// bottom-right
			1.0f, 1.0f, -1.0f, 1.0f, 0.0f, 0.0f,	// top-right
			1.0f, -1.0f, -1.0f, 1.0f, 0.0f, 0.0f,	// bottom-right
			1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f,		// top-left
			1.0f, -1.0f, 1.0f, 1.0f, 0.0f, 0.0f,	// bottom-left
													// bottom face
			-1.0f, -1.0f, -1.0f, 0.0f, -1.0f, 0.0f, // top-right
			1.0f, -1.0f, -1.0f, 0.0f, -1.0f, 0.0f,	// top-left
			1.0f, -1.0f, 1.0f, 0.0f, -1.0f, 0.0f,	// bottom-left
			1.0f, -1.0f, 1.0f, 0.0f, -1.0f, 0.0f,	// bottom-left
			-1.0f, -1.0f, 1.0f, 0.0f, -1.0f, 0.0f,	// bottom-right
			-1.0f, -1.0f, -1.0f, 0.0f, -1.0f, 0.0f, // top-right
													// top face
			-1.0f, 1.0f, -1.0f, 0.0f, 1.0f, 0.0f,	// top-left
			1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f,		// bottom-right
			1.0f, 1.0f, -1.0f, 0.0f, 1.0f, 0.0f,	// top-right
			1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f,		// bottom-right
			-1.0f, 1.0f, -1.0f, 0.0f, 1.0f, 0.0f,	// top-left
			-1.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f		// bottom-left
		};

		glGenVertexArrays( 1, &cubeArrayId );
		glGenBuffers( 1, &cubeBufferId );

		// fill buffer
		glBindBuffer( GL_ARRAY_BUFFER, cubeBufferId );
		glBufferData( GL_ARRAY_BUFFER, sizeof( vertices ), vertices, GL_STATIC_DRAW );

		// link vertex attributes
		glBindVertexArray( cubeArrayId );

		// position
		glEnableVertexAttribArray( 0 );
		glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof( float ), (void*)0 );

		// normal
		glEnableVertexAttribArray( 1 );
		glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof( float ), (void*)( 3 * sizeof( float ) ) );

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
	}

	glUseProgram( scene->forwardShader );
	SetUniformMatrix4( scene->forwardShader, "projection", projection );
	SetUniformMatrix4( scene->forwardShader, "view", view );

	Matrix4 m = MakeMatrix4( transform, 0.25f );
	SetUniformMatrix4( scene->forwardShader, "model", m );

	RGBAf c = MakeRGBAf( color );
	SetUniformVec3( scene->forwardShader, "color", { c.r, c.g, c.b } );

	glBindVertexArray( cubeArrayId );
	glDrawArrays( GL_TRIANGLES, 0, 36 );
	glBindVertexArray( 0 );
}

struct VertexData
{
	b3Vec3 position;
	b3Vec3 normal;
	int material;
};

// Creates a shape that can be instanced
InstanceShape CreateInstanceShape( const b3Vec3* vertexPositions, const b3Vec3* vertexNormals, const int* triangleIndices,
								   int triangleCount, const Edge* edges, int edgeCount, const int* materials, Arena arena )
{
	InstanceShape shape = {};

	// Vertex buffer
	{
		int vertexCount = 3 * triangleCount;
		VertexData* vertexData = (VertexData*)arena.Allocate( vertexCount * sizeof( VertexData ) );

		int index = 0;
		for ( int triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex )
		{
			int vertexIndex1 = triangleIndices[3 * triangleIndex + 0];
			int vertexIndex2 = triangleIndices[3 * triangleIndex + 1];
			int vertexIndex3 = triangleIndices[3 * triangleIndex + 2];

			b3Vec3 vertexPosition1 = vertexPositions[vertexIndex1];
			b3Vec3 vertexPosition2 = vertexPositions[vertexIndex2];
			b3Vec3 vertexPosition3 = vertexPositions[vertexIndex3];

			vertexData[index + 0].position = vertexPosition1;
			vertexData[index + 1].position = vertexPosition2;
			vertexData[index + 2].position = vertexPosition3;

			b3Vec3 vertexNormal1, vertexNormal2, vertexNormal3;
			if ( vertexNormals )
			{
				vertexNormal1 = vertexNormals[vertexIndex1];
				vertexNormal2 = vertexNormals[vertexIndex2];
				vertexNormal3 = vertexNormals[vertexIndex3];
			}
			else
			{
				b3Vec3 edge1 = vertexPosition2 - vertexPosition1;
				b3Vec3 edge2 = vertexPosition3 - vertexPosition1;
				b3Vec3 normal = b3Cross( edge1, edge2 );
				normal = b3Normalize( normal );

				vertexNormal1 = normal;
				vertexNormal2 = normal;
				vertexNormal3 = normal;
			}

			vertexData[index + 0].normal = vertexNormal1;
			vertexData[index + 1].normal = vertexNormal2;
			vertexData[index + 2].normal = vertexNormal3;

			if ( materials != nullptr )
			{
				vertexData[index + 0].material = materials[triangleIndex];
				vertexData[index + 1].material = materials[triangleIndex];
				vertexData[index + 2].material = materials[triangleIndex];
			}
			else
			{
				vertexData[index + 0].material = -1;
				vertexData[index + 1].material = -1;
				vertexData[index + 2].material = -1;
			}

			index += 3;
		}

		assert( index == vertexCount );

		// Fill vertex buffers
		shape.triangleCount = triangleCount;

		glGenVertexArrays( 1, &shape.triangleArrayId );
		glGenBuffers( 2, shape.triangleBufferIds );

		// Copy static vertex data
		glBindVertexArray( shape.triangleArrayId );
		glBindBuffer( GL_ARRAY_BUFFER, shape.triangleBufferIds[0] );
		glBufferData( GL_ARRAY_BUFFER, vertexCount * sizeof( VertexData ), vertexData, GL_STATIC_DRAW );

		// positions
		uint32_t positionAttribute = 0;
		glEnableVertexAttribArray( positionAttribute );
		glVertexAttribPointer( positionAttribute, 3, GL_FLOAT, GL_FALSE, sizeof( VertexData ),
							   (void*)offsetof( VertexData, position ) );

		// normals
		uint32_t normalAttribute = 1;
		glEnableVertexAttribArray( normalAttribute );
		glVertexAttribPointer( normalAttribute, 3, GL_FLOAT, GL_FALSE, sizeof( VertexData ),
							   (void*)offsetof( VertexData, normal ) );

		// materials
		uint32_t materialAttribute = 2;
		glEnableVertexAttribArray( materialAttribute );
		glVertexAttribIPointer( materialAttribute, 1, GL_INT, sizeof( VertexData ), (void*)offsetof( VertexData, material ) );

		// Instance data setup
		glBindBuffer( GL_ARRAY_BUFFER, shape.triangleBufferIds[1] );
		glBufferData( GL_ARRAY_BUFFER, s_batchSize * sizeof( InstanceData ), nullptr, GL_DYNAMIC_DRAW );

		uint32_t colorInstance = 3;
		glEnableVertexAttribArray( colorInstance );
		// Save bandwidth by expanding color to floats in the shader
		glVertexAttribPointer( colorInstance, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( InstanceData ),
							   (void*)offsetof( InstanceData, rgba8 ) );

		// This makes the color an instance
		glVertexAttribDivisor( colorInstance, 1 );

		// Matrix has to use 4 attributes each 16 bytes
		uint32_t matrixAttribute = 4;
		glEnableVertexAttribArray( matrixAttribute + 0 );
		glEnableVertexAttribArray( matrixAttribute + 1 );
		glEnableVertexAttribArray( matrixAttribute + 2 );
		glEnableVertexAttribArray( matrixAttribute + 3 );

		glVertexAttribPointer( matrixAttribute + 0, 4, GL_FLOAT, GL_FALSE, sizeof( InstanceData ),
							   (void*)offsetof( InstanceData, matrix.cx ) );
		glVertexAttribPointer( matrixAttribute + 1, 4, GL_FLOAT, GL_FALSE, sizeof( InstanceData ),
							   (void*)offsetof( InstanceData, matrix.cy ) );
		glVertexAttribPointer( matrixAttribute + 2, 4, GL_FLOAT, GL_FALSE, sizeof( InstanceData ),
							   (void*)offsetof( InstanceData, matrix.cz ) );
		glVertexAttribPointer( matrixAttribute + 3, 4, GL_FLOAT, GL_FALSE, sizeof( InstanceData ),
							   (void*)offsetof( InstanceData, matrix.cw ) );

		// This makes the matrix an instance
		glVertexAttribDivisor( matrixAttribute + 0, 1 );
		glVertexAttribDivisor( matrixAttribute + 1, 1 );
		glVertexAttribDivisor( matrixAttribute + 2, 1 );
		glVertexAttribDivisor( matrixAttribute + 3, 1 );

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
	}

	CheckOpenGL();

	// Edges are optional
	if ( edgeCount > 0 )
	{
		// Create edge buffer
		EdgeData* edgeBuffer = (EdgeData*)arena.Allocate( 2 * edgeCount * sizeof( EdgeData ) );
		for ( int edgeIndex = 0; edgeIndex < edgeCount; ++edgeIndex )
		{
			int vertexIndex1 = edges[edgeIndex].index1;
			int vertexIndex2 = edges[edgeIndex].index2;
			int flags = edges[edgeIndex].flags;

			const b3Vec3* vb = vertexPositions;
			b3Vec3 vertexPosition1 = vb[vertexIndex1];
			b3Vec3 vertexPosition2 = vb[vertexIndex2];

			edgeBuffer[2 * edgeIndex + 0].position = vertexPosition1;
			edgeBuffer[2 * edgeIndex + 0].flags = flags;
			edgeBuffer[2 * edgeIndex + 1].position = vertexPosition2;
			edgeBuffer[2 * edgeIndex + 1].flags = flags;
		}

		shape.edgeCount = edgeCount;

		glGenVertexArrays( 1, &shape.edgeArrayId );
		glGenBuffers( 2, shape.edgeBufferIds );

		// Copy static vertex data
		glBindVertexArray( shape.edgeArrayId );
		glBindBuffer( GL_ARRAY_BUFFER, shape.edgeBufferIds[0] );
		glBufferData( GL_ARRAY_BUFFER, 2 * edgeCount * sizeof( EdgeData ), edgeBuffer, GL_STATIC_DRAW );

		uint32_t positionIndex = 0;
		glEnableVertexAttribArray( positionIndex );
		glVertexAttribPointer( positionIndex, 3, GL_FLOAT, GL_FALSE, sizeof( EdgeData ), (void*)offsetof( EdgeData, position ) );

		uint32_t flagsIndex = 1;
		glEnableVertexAttribArray( flagsIndex );
		glVertexAttribIPointer( flagsIndex, 1, GL_INT, sizeof( EdgeData ), (void*)offsetof( EdgeData, flags ) );

		// Instance data setup
		glBindBuffer( GL_ARRAY_BUFFER, shape.edgeBufferIds[1] );
		glBufferData( GL_ARRAY_BUFFER, s_batchSize * sizeof( InstanceData ), nullptr, GL_DYNAMIC_DRAW );

		uint32_t colorInstance = 2;
		glEnableVertexAttribArray( colorInstance );
		// Save bandwidth by expanding color to floats in the shader
		glVertexAttribPointer( colorInstance, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( InstanceData ),
							   (void*)offsetof( InstanceData, rgba8 ) );

		// This makes the color an instance
		glVertexAttribDivisor( colorInstance, 1 );

		// Matrix has to use 4 attributes each 16 bytes
		uint32_t matrixAttribute = 3;
		glEnableVertexAttribArray( matrixAttribute + 0 );
		glEnableVertexAttribArray( matrixAttribute + 1 );
		glEnableVertexAttribArray( matrixAttribute + 2 );
		glEnableVertexAttribArray( matrixAttribute + 3 );

		glVertexAttribPointer( matrixAttribute + 0, 4, GL_FLOAT, GL_FALSE, sizeof( InstanceData ),
							   (void*)offsetof( InstanceData, matrix.cx ) );
		glVertexAttribPointer( matrixAttribute + 1, 4, GL_FLOAT, GL_FALSE, sizeof( InstanceData ),
							   (void*)offsetof( InstanceData, matrix.cy ) );
		glVertexAttribPointer( matrixAttribute + 2, 4, GL_FLOAT, GL_FALSE, sizeof( InstanceData ),
							   (void*)offsetof( InstanceData, matrix.cz ) );
		glVertexAttribPointer( matrixAttribute + 3, 4, GL_FLOAT, GL_FALSE, sizeof( InstanceData ),
							   (void*)offsetof( InstanceData, matrix.cw ) );

		// This makes the matrix an instance
		glVertexAttribDivisor( matrixAttribute + 0, 1 );
		glVertexAttribDivisor( matrixAttribute + 1, 1 );
		glVertexAttribDivisor( matrixAttribute + 2, 1 );
		glVertexAttribDivisor( matrixAttribute + 3, 1 );

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
	}

	CheckOpenGL();

	return shape;
}

void DestroySharedShape( InstanceShape shape )
{
	glDeleteBuffers( 2, shape.triangleBufferIds );
	glDeleteVertexArrays( 1, &shape.triangleArrayId );

	if ( shape.edgeCount > 0 )
	{
		glDeleteBuffers( 2, shape.edgeBufferIds );
		glDeleteVertexArrays( 1, &shape.edgeArrayId );
	}
}

static InstanceShape CreateSphere( Arena arena )
{
	float radius = 1.0f;

	int vertexCount = s_sideCount * s_ringCount + 2;
	b3Vec3* vertices = (b3Vec3*)arena.Allocate( vertexCount * sizeof( b3Vec3 ) );
	b3Vec3* normals = (b3Vec3*)arena.Allocate( vertexCount * sizeof( b3Vec3 ) );

	int triangleCount = s_sideCount * 2 * s_ringCount;
	int* indices = (int*)arena.Allocate( 3 * triangleCount * sizeof( int ) );

	// poles
	vertices[s_sideCount * s_ringCount + 0] = b3Vec3{ -radius, 0.0f, 0.0f };
	normals[s_sideCount * s_ringCount + 0] = { -1.0f, 0.0f, 0.0f };
	vertices[s_sideCount * s_ringCount + 1] = b3Vec3{ radius, 0.0f, 0.0f };
	normals[s_sideCount * s_ringCount + 1] = { 1.0f, 0.0f, 0.0f };

	int j = 0;

	for ( int i = 0; i < s_sideCount; ++i )
	{
		float theta = ( -2 * B3_PI * i ) / s_sideCount;
		float s0 = b3Sin( theta );
		float c0 = b3Cos( theta );

		int i1 = ( i + 1 ) % s_sideCount;

		// Sides
		for ( int k = 0; k <= s_ringCount; ++k )
		{
			float phi = ( k + 1 ) * B3_PI / ( s_ringCount + 1 ) - 0.5f * B3_PI;
			float x0 = b3Sin( phi );
			float y0 = b3Cos( phi );

			b3Vec3 normal = { x0, c0 * y0, s0 * y0 };
			vertices[s_ringCount * i + k] = radius * normal;
			normals[s_ringCount * i + k] = normal;

			if ( k + 1 < s_ringCount )
			{
				indices[j++] = s_ringCount * i1 + k;
				indices[j++] = s_ringCount * i + k;
				indices[j++] = s_ringCount * i1 + k + 1;

				indices[j++] = s_ringCount * i1 + k + 1;
				indices[j++] = s_ringCount * i + k;
				indices[j++] = s_ringCount * i + k + 1;
			}
			else
			{
				// Needed the vertices, but not the indices
				break;
			}

			B3_ASSERT( s_ringCount * i + k + 1 < vertexCount );
		}

		// Caps
		indices[j++] = s_ringCount * s_sideCount;
		indices[j++] = s_ringCount * i;
		indices[j++] = s_ringCount * i1;

		indices[j++] = s_ringCount * s_sideCount + 1;
		indices[j++] = s_ringCount * i1 + s_ringCount - 1;
		indices[j++] = s_ringCount * i + s_ringCount - 1;
	}

	B3_ASSERT( j == 3 * triangleCount );

	// todo add line discs for transparent view

	return CreateInstanceShape( vertices, normals, indices, triangleCount, nullptr, 0, nullptr, arena );
}

// Open hemisphere along x-axis. Radius is 1.
static InstanceShape CreateOpenHemisphere( Arena arena )
{
	float radius = 1.0f;

	const int sideVertexCount = s_ringCount + 1;

	// Vertex count including pole
	int vertexCount = s_sideCount * sideVertexCount + 1;
	b3Vec3* vertices = (b3Vec3*)arena.Allocate( vertexCount * sizeof( b3Vec3 ) );
	b3Vec3* normals = (b3Vec3*)arena.Allocate( vertexCount * sizeof( b3Vec3 ) );

	// Hemisphere quad rings and a cone at the pole
	int triangleCount = 2 * s_sideCount * s_ringCount + s_sideCount;
	int* indices = (int*)arena.Allocate( 3 * triangleCount * sizeof( int ) );

	// Pole at the end of the array
	vertices[s_sideCount * sideVertexCount] = b3Vec3{ radius, 0.0f, 0.0f };
	normals[s_sideCount * sideVertexCount] = b3Vec3{ 1.0f, 0.0f, 0.0f };

	int j = 0;

	// loop sides
	for ( int i = 0; i < s_sideCount; ++i )
	{
		int i1 = ( i + 1 ) % s_sideCount;

		float theta = ( -2.0f * B3_PI * i ) / s_sideCount;
		b3CosSin csTheta = b3ComputeCosSin( theta );
		float c0 = csTheta.cosine;
		float s0 = csTheta.sine;

		// loop rings
		for ( int k = 0;; ++k )
		{
			int k1 = k + 1;

			float phi = ( 0.5f * k * B3_PI ) / ( s_ringCount + 1 );
			b3CosSin csPhi = b3ComputeCosSin( phi );
			float y0 = csPhi.cosine;
			float x0 = csPhi.sine;

			b3Vec3 normal = { x0, c0 * y0, s0 * y0 };
			b3Vec3 vertex = radius * normal;
			vertices[sideVertexCount * i + k] = vertex;
			normals[sideVertexCount * i + k] = normal;

			if ( k < s_ringCount )
			{
				// ring quad
				indices[j++] = sideVertexCount * i1 + k;
				indices[j++] = sideVertexCount * i + k;
				indices[j++] = sideVertexCount * i1 + k1;

				indices[j++] = sideVertexCount * i1 + k1;
				indices[j++] = sideVertexCount * i + k;
				indices[j++] = sideVertexCount * i + k1;
			}
			else
			{
				// cone
				indices[j++] = s_sideCount * sideVertexCount;
				indices[j++] = sideVertexCount * i1 + k;
				indices[j++] = sideVertexCount * i + k;
				break;
			}
		}
	}

	B3_ASSERT( j == 3 * triangleCount );

	return CreateInstanceShape( vertices, normals, indices, triangleCount, nullptr, 0, nullptr, arena );
}

static InstanceShape CreateOpenCylinder( Arena arena )
{
	float radius = 1.0f;
	float height = 1.0f;

	int vertexCount = 2 * s_sideCount;
	b3Vec3* vertices = (b3Vec3*)arena.Allocate( vertexCount * sizeof( b3Vec3 ) );
	b3Vec3* normals = (b3Vec3*)arena.Allocate( vertexCount * sizeof( b3Vec3 ) );

	int triangleCount = 2 * s_sideCount;
	int* indices = (int*)arena.Allocate( 3 * triangleCount * sizeof( int ) );

	int j = 0;

	// loop sides
	for ( int i = 0; i < s_sideCount; ++i )
	{
		int i1 = ( i + 1 ) % s_sideCount;

		float theta = ( -2.0f * B3_PI * i ) / s_sideCount;
		b3CosSin csTheta = b3ComputeCosSin( theta );
		float c0 = csTheta.cosine;
		float s0 = csTheta.sine;

		b3Vec3 normal = { 0.0f, c0, s0 };
		b3Vec3 vertex1 = radius * normal - b3Vec3{ 0.5f * height, 0.0f, 0.0f };
		vertices[2 * i + 0] = vertex1;
		normals[2 * i + 0] = normal;

		b3Vec3 vertex2 = radius * normal + b3Vec3{ 0.5f * height, 0.0f, 0.0f };
		vertices[2 * i + 1] = vertex2;
		normals[2 * i + 1] = normal;

		// side quad
		indices[j++] = 2 * i1;
		indices[j++] = 2 * i;
		indices[j++] = 2 * i1 + 1;

		indices[j++] = 2 * i1 + 1;
		indices[j++] = 2 * i;
		indices[j++] = 2 * i + 1;
	}

	B3_ASSERT( j == 3 * triangleCount );

	return CreateInstanceShape( vertices, normals, indices, triangleCount, nullptr, 0, nullptr, arena );
}

Scene* CreateScene( const Camera* camera, Arena arena )
{
	Scene* scene = new Scene;

	scene->ao.radius = 1.0f;
	scene->ao.bias = 0.25f;
	scene->ao.minScale = 0.0f;
	scene->ao.power = 2.0f;
	scene->ao.direct = 0.5f;
	scene->ao.aoOnly = false;
	scene->useSSAO = true;
	scene->useShadow = true;
	scene->useGrid = false;

	scene->pointDraw = CreateDebugPoints();
	scene->lineDraw = CreateDebugLines();

	scene->shadowShader = CreateProgramFromFiles( "shaders/shadow_map.vs", "shaders/empty.fs" );
	scene->meshShader = CreateProgramFromFiles( "shaders/mesh.vs", "shaders/mesh.fs" );
	scene->meshShadowShader = CreateProgramFromFiles( "shaders/mesh_shadow.vs", "shaders/empty.fs" );
	scene->meshEdgeShader = CreateProgramFromFiles( "shaders/mesh_edge.vs", "shaders/forward.fs" );
	scene->geometryShader = CreateProgramFromFiles( "shaders/geometry_vs.glsl", "shaders/geometry_fs.glsl" );
	scene->ssaoShader = CreateProgramFromFiles( "shaders/quad_vs.glsl", "shaders/ssao_fs.glsl" );
	scene->ssaoBlurShader = CreateProgramFromFiles( "shaders/quad_vs.glsl", "shaders/ssao_blur_fs.glsl" );
	scene->deferredShader = CreateProgramFromFiles( "shaders/quad_vs.glsl", "shaders/deferred_lighting_fs.glsl" );
	scene->meshForwardShader = CreateProgramFromFiles( "shaders/mesh_forward.vs", "shaders/mesh_forward.fs" );
	scene->forwardShader = CreateProgramFromFiles( "shaders/forward.vs", "shaders/forward.fs" );
	scene->depthCopyShader = CreateProgramFromFiles( "shaders/quad_vs.glsl", "shaders/depth_copy_fs.glsl" );

	scene->sphere = CreateSphere( arena );
	scene->openHemisphere = CreateOpenHemisphere( arena );
	scene->openCylinder = CreateOpenCylinder( arena );

	if ( scene->shadowShader == 0 || scene->meshShader == 0 || scene->meshEdgeShader == 0 || scene->meshEdgeShader == 0 ||
		 scene->geometryShader == 0 || scene->ssaoShader == 0 || scene->ssaoBlurShader == 0 || scene->deferredShader == 0 ||
		 scene->forwardShader == 0 || scene->depthCopyShader == 0 )
	{
		fprintf( stderr, "Failed to create shaders\n" );
		return nullptr;
	}

	// configure depth map FBO for shadow map
	glGenFramebuffers( 1, &scene->depthMapFBO );

	// create depth texture
	glGenTextures( 1, &scene->depthMap );
	glBindTexture( GL_TEXTURE_2D, scene->depthMap );
	glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, shadowWidth, shadowHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr );
	// Linear filtering combined with comparison mode gives free hardware 2x2 PCF
	// inside each tap when sampled via sampler2DShadow.
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL );
	float borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	glTexParameterfv( GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor );

	// attach depth texture as FBO's depth buffer
	glBindFramebuffer( GL_FRAMEBUFFER, scene->depthMapFBO );
	glFramebufferTexture2D( GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, scene->depthMap, 0 );
	glDrawBuffer( GL_NONE );
	glReadBuffer( GL_NONE );
	glBindFramebuffer( GL_FRAMEBUFFER, 0 );

	scene->geoBuffer = CreateGeoBuffer( camera->m_bufferWidth, camera->m_bufferHeight );
	scene->ssaoBuffer = CreateSsaoBuffer( camera->m_bufferWidth, camera->m_bufferHeight );

	glUseProgram( scene->ssaoShader );
	SetUniformInt( scene->ssaoShader, "gPosition", 0 );
	SetUniformInt( scene->ssaoShader, "gNormal", 1 );
	SetUniformInt( scene->ssaoShader, "texNoise", 2 );

	glUseProgram( scene->ssaoBlurShader );
	SetUniformInt( scene->ssaoBlurShader, "ssaoInput", 0 );
	SetUniformInt( scene->ssaoBlurShader, "gPosition", 1 );

	glUseProgram( scene->deferredShader );
	SetUniformInt( scene->deferredShader, "gPosition", 0 );
	SetUniformInt( scene->deferredShader, "gNormal", 1 );
	SetUniformInt( scene->deferredShader, "gAlbedoSpec", 2 );
	SetUniformInt( scene->deferredShader, "ssao", 3 );
	SetUniformInt( scene->deferredShader, "shadowMap", 4 );

	glUseProgram( scene->depthCopyShader );
	SetUniformInt( scene->depthCopyShader, "depthTexture", 0 );

	return scene;
}

void DestroySharedMeshes( Scene* scene )
{
	for ( std::unordered_map<uint32_t, InstanceShape>::iterator shared = scene->sharedHeightFields.begin();
		  shared != scene->sharedHeightFields.end(); ++shared )
	{
		DestroySharedShape( shared->second );
	}

	for ( std::unordered_map<uint32_t, InstanceShape>::iterator shared = scene->sharedHulls.begin();
		  shared != scene->sharedHulls.end(); ++shared )
	{
		DestroySharedShape( shared->second );
	}

	for ( std::unordered_map<uint32_t, InstanceShape>::iterator shared = scene->sharedMeshes.begin();
		  shared != scene->sharedMeshes.end(); ++shared )
	{
		DestroySharedShape( shared->second );
	}

	scene->sharedHeightFields.clear();
	scene->sharedHulls.clear();
	scene->sharedMeshes.clear();
	scene->heightFieldInstances.clear();
	scene->hullInstances.clear();
	scene->meshInstances.clear();
	scene->transparentHullInstances.clear();
}

void DestroyScene( Scene* scene )
{
	DestroyDebugPoints( scene->pointDraw );
	DestroyDebugLines( scene->lineDraw );
	DestroyGeoBuffer( scene->geoBuffer );
	DestroySsaoBuffer( scene->ssaoBuffer );

	DestroySharedShape( scene->sphere );
	DestroySharedShape( scene->openHemisphere );
	DestroySharedShape( scene->openCylinder );

	DestroySharedMeshes( scene );

	DestroyShader( scene->shadowShader );
	DestroyShader( scene->meshShader );
	DestroyShader( scene->meshShadowShader );
	DestroyShader( scene->meshEdgeShader );
	DestroyShader( scene->geometryShader );
	DestroyShader( scene->ssaoShader );
	DestroyShader( scene->ssaoBlurShader );
	DestroyShader( scene->deferredShader );
	DestroyShader( scene->meshForwardShader );
	DestroyShader( scene->forwardShader );
	DestroyShader( scene->depthCopyShader );

	delete scene;
}

uint32_t CreateInstanceHull( Scene* scene, const b3Hull* hull, Arena arena )
{
	uint32_t hash = hull->hash;

	std::unordered_map<uint32_t, InstanceShape>::iterator result = scene->sharedHulls.find( hash );
	if ( result != scene->sharedHulls.end() )
	{
		// Already created
		return hash;
	}

	int vertexCount = hull->vertexCount;
	b3Vec3* vertices = (b3Vec3*)arena.Allocate( hull->vertexCount * sizeof( b3Vec3 ) );

	const b3Vec3* hullPoints = b3GetHullPoints( hull );
	memcpy( vertices, hullPoints, vertexCount * sizeof( b3Vec3 ) );

	int triangleCount = 0;
	int faceCount = hull->faceCount;
	const b3HullFace* hullFaces = b3GetHullFaces( hull );
	const b3HullHalfEdge* hullEdges = b3GetHullEdges( hull );

	// Count triangles
	for ( int i = 0; i < faceCount; ++i )
	{
		const b3HullFace* face = hullFaces + i;

		const b3HullHalfEdge* edge1 = hullEdges + face->edge;
		const b3HullHalfEdge* edge2 = hullEdges + edge1->next;
		const b3HullHalfEdge* edge3 = hullEdges + edge2->next;
		B3_ASSERT( edge1 != edge3 );

		do
		{
			triangleCount += 1;
			edge3 = hullEdges + edge3->next;
		}
		while ( edge1 != edge3 );
	}

	int* indices = (int*)arena.Allocate( 3 * triangleCount * sizeof( int ) );
	int triangleIndex = 0;

	for ( int i = 0; i < faceCount; ++i )
	{
		const b3HullFace* face = hullFaces + i;

		const b3HullHalfEdge* edge1 = hullEdges + face->edge;
		const b3HullHalfEdge* edge2 = hullEdges + edge1->next;
		const b3HullHalfEdge* edge3 = hullEdges + edge2->next;
		B3_ASSERT( edge1 != edge3 );

		do
		{
			B3_ASSERT( triangleIndex < triangleCount );
			indices[3 * triangleIndex + 0] = edge1->origin;
			indices[3 * triangleIndex + 1] = edge2->origin;
			indices[3 * triangleIndex + 2] = edge3->origin;
			triangleIndex += 1;

			edge2 = edge3;
			edge3 = hullEdges + edge3->next;
		}
		while ( edge1 != edge3 );
	}

	B3_ASSERT( triangleIndex == triangleCount );

	int edgeCount = hull->edgeCount / 2;
	Edge* edges = (Edge*)arena.Allocate( edgeCount * sizeof( Edge ) );

	for ( int i = 0; i < edgeCount; ++i )
	{
		const b3HullHalfEdge* edge = hullEdges + 2 * i + 0;
		const b3HullHalfEdge* twin = hullEdges + 2 * i + 1;

		edges[i].index1 = edge->origin;
		edges[i].index2 = twin->origin;
		edges[i].flags = 0;
	}

	InstanceShape shape = CreateInstanceShape( vertices, nullptr, indices, triangleCount, edges, edgeCount, nullptr, arena );
	scene->sharedHulls[hash] = shape;

	// reserve a slot
	scene->hullInstances[hash] = {};
	scene->transparentHullInstances[hash] = {};

	return hash;
}

static inline bool EdgeLess( Edge e1, Edge e2 )
{
	if ( e1.index1 == e2.index1 )
	{
		return e1.index2 < e2.index2;
	}

	return e1.index1 < e2.index1;
}

static bool EdgeEquals( Edge e1, Edge e2 )
{
	return e1.index1 == e2.index1 && e1.index2 == e2.index2;
}

uint32_t CreateInstanceMesh( Scene* scene, const b3MeshData* meshData, Arena arena )
{
	uint32_t hash = meshData->hash;

	std::unordered_map<uint32_t, InstanceShape>::iterator result = scene->sharedMeshes.find( hash );
	if ( result != scene->sharedMeshes.end() )
	{
		// Already created
		return hash;
	}

	int vertexCount = meshData->vertexCount;
	const b3Vec3* meshVertices = b3GetMeshVertices( meshData );

	int triangleCount = meshData->triangleCount;
	std::vector<int> indices;
	indices.resize( 3 * triangleCount );

	int edgeCount = 3 * triangleCount;
	std::vector<Edge> edges;
	edges.resize( edgeCount );

	std::vector<int> materials;
	materials.resize( triangleCount );

	const b3MeshTriangle* meshTriangles = b3GetMeshTriangles( meshData );
	const uint8_t* materialIndices = b3GetMeshMaterialIndices( meshData );
	const uint8_t* meshFlags = b3GetMeshFlags( meshData );

	for ( int i = 0; i < triangleCount; ++i )
	{
		b3MeshTriangle triangle = meshTriangles[i];

		int i1 = triangle.index1;
		int i2 = triangle.index2;
		int i3 = triangle.index3;

		assert( i1 < vertexCount && i2 < vertexCount && i3 < vertexCount );

		indices[3 * i + 0] = i1;
		indices[3 * i + 1] = i2;
		indices[3 * i + 2] = i3;

		uint8_t triangleFlags = meshFlags[i];

		int flag1 = ( triangleFlags & b3_concaveEdge1 ) != 0 ? 0 : 1;
		int flag2 = ( triangleFlags & b3_concaveEdge2 ) != 0 ? 0 : 1;
		int flag3 = ( triangleFlags & b3_concaveEdge3 ) != 0 ? 0 : 1;

		edges[3 * i + 0] = { i1, i2, flag1 };
		edges[3 * i + 1] = { i2, i3, flag2 };
		edges[3 * i + 2] = { i3, i1, flag3 };

		if ( materialIndices != nullptr )
		{
			materials[i] = materialIndices[i];
		}
		else
		{
			materials[i] = 0;
		}
	}

	// Remove duplicate edges
	std::sort( edges.data(), edges.data() + edges.size(), EdgeLess );
	auto last = std::ranges::unique( edges, EdgeEquals ).begin();
	edges.erase( last, edges.end() );
	edgeCount = (int)edges.size();

	InstanceShape shape = CreateInstanceShape( meshVertices, nullptr, indices.data(), triangleCount, edges.data(), edgeCount,
											   materials.data(), arena );
	scene->sharedMeshes[hash] = shape;

	// reserve a slot
	scene->meshInstances[hash] = {};

	return hash;
}

uint32_t CreateInstanceHeightField( Scene* scene, const b3HeightField* heightField, Arena arena )
{
	uint32_t hash = heightField->hash;

	std::unordered_map<uint32_t, InstanceShape>::iterator result = scene->sharedHeightFields.find( hash );
	if ( result != scene->sharedHeightFields.end() )
	{
		// Already created
		return hash;
	}

	int rowCount = heightField->rowCount;
	int columnCount = heightField->columnCount;
	uint16_t* compressedHeights = heightField->compressedHeights;
	uint8_t* materialIndices = heightField->materialIndices;
	uint8_t* flags = heightField->flags;
	float minHeight = heightField->minHeight;
	float heightScale = heightField->heightScale;
	b3Vec3 scale = heightField->scale;

	int vertexCount = rowCount * columnCount;
	int cellCount = ( rowCount - 1 ) * ( columnCount - 1 );

	std::vector<b3Vec3> vertices;
	vertices.resize( vertexCount );

	int vertexIndex = 0;
	for ( int row = 0; row < rowCount; ++row )
	{
		for ( int column = 0; column < columnCount; ++column )
		{
			int heightIndex = row * columnCount + column;

			float x = float( column );
			float y = minHeight + heightScale * compressedHeights[heightIndex];
			float z = float( row );

			vertices[vertexIndex] = scale * b3Vec3{ x, y, z };
			vertexIndex += 1;
		}
	}
	assert( vertexIndex == vertexCount );

	int triangleCount = 2 * ( rowCount - 1 ) * ( columnCount - 1 );

	std::vector<int> materials;
	materials.resize( triangleCount );

	// int cellCount = triangleCount >> 1;
	int materialIndex = 0;
	for ( int i = 0; i < cellCount; ++i )
	{
		uint8_t index = materialIndices[i];
		if ( index != B3_HEIGHT_FIELD_HOLE )
		{
			materials[materialIndex + 0] = index;
			materials[materialIndex + 1] = index;
			materialIndex += 2;
		}
	}

	int indexCount = 3 * triangleCount;
	std::vector<int> indices;
	indices.resize( indexCount );

	// Edge along each column, row, and diagonal
	int edgeCount = columnCount * ( rowCount - 1 ) + rowCount * ( columnCount - 1 ) + ( rowCount - 1 ) * ( columnCount - 1 );
	std::vector<Edge> edges;
	edges.resize( edgeCount );

	int index = 0;
	int edgeIndex = 0;
	for ( int row = 0; row < rowCount - 1; ++row )
	{
		for ( int column = 0; column < columnCount - 1; ++column )
		{
			int cellIndex = row * ( columnCount - 1 ) + column;
			int triangleIndex = 2 * cellIndex;
			assert( triangleIndex < triangleCount );

			assert( cellIndex < cellCount );
			if ( materialIndices[cellIndex] == B3_HEIGHT_FIELD_HOLE )
			{
				continue;
			}

			int index11 = row * columnCount + column;
			int index12 = index11 + 1;
			int index21 = ( row + 1 ) * columnCount + column;
			int index22 = index21 + 1;

			indices[index + 0] = index11;
			indices[index + 1] = index21;
			indices[index + 2] = index12;

			indices[index + 3] = index22;
			indices[index + 4] = index12;
			indices[index + 5] = index21;

			index += 6;

			int flag = ( flags[triangleIndex] & b3_concaveEdge2 ) != 0 ? 0 : 1;
			edges[edgeIndex] = { index12, index21, flag };
			edgeIndex += 1;
		}
	}
	assert( index <= indexCount );
	indexCount = index;

	// Edge rows
	for ( int row = 0; row < rowCount; ++row )
	{
		for ( int column = 0; column < columnCount - 1; ++column )
		{
			int index1 = row * columnCount + column;
			int index2 = index1 + 1;

			int flag;
			if ( row < rowCount - 1 )
			{
				int cellIndex = row * ( columnCount - 1 ) + column;
				int triangleIndex = 2 * cellIndex;

				assert( triangleIndex < triangleCount );
				flag = ( flags[triangleIndex] & b3_concaveEdge3 ) != 0 ? 0 : 1;
			}
			else
			{
				int cellIndex = ( row - 1 ) * ( columnCount - 1 ) + column;
				int triangleIndex = 2 * cellIndex + 1;
				assert( triangleIndex < triangleCount );
				flag = ( flags[triangleIndex] & b3_concaveEdge3 ) != 0 ? 0 : 1;
			}

			edges[edgeIndex + 0] = { index1, index2, flag };
			edgeIndex += 1;
		}
	}

	// Edge columns
	for ( int column = 0; column < columnCount; ++column )
	{
		for ( int row = 0; row < rowCount - 1; ++row )
		{
			int index1 = row * columnCount + column;
			int index2 = ( row + 1 ) * columnCount + column;

			int flag;
			if ( column < columnCount - 1 )
			{
				int cellIndex = row * ( columnCount - 1 ) + column;
				int triangleIndex = 2 * cellIndex;
				assert( triangleIndex < triangleCount );
				flag = ( flags[triangleIndex] & b3_concaveEdge1 ) != 0 ? 0 : 1;
			}
			else
			{
				int cellIndex = row * ( columnCount - 1 ) + column - 1;
				int triangleIndex = 2 * cellIndex + 1;
				assert( triangleIndex < triangleCount );
				flag = ( flags[triangleIndex] & b3_concaveEdge1 ) != 0 ? 0 : 1;
			}

			edges[edgeIndex + 0] = { index1, index2, flag };
			edgeIndex += 1;
		}
	}
	assert( edgeIndex <= edgeCount );
	edgeCount = edgeIndex;

	triangleCount = indexCount / 3;

	assert( materialIndex == triangleCount );

	InstanceShape shape = CreateInstanceShape( vertices.data(), nullptr, indices.data(), triangleCount, edges.data(), edgeCount,
											   materials.data(), arena );
	scene->sharedHeightFields[hash] = shape;

	// reserve a slot
	scene->heightFieldInstances[hash] = {};

	return hash;
}

void ResizeScene( Scene* scene, const Camera* camera )
{
	DestroyGeoBuffer( scene->geoBuffer );
	scene->geoBuffer = CreateGeoBuffer( camera->m_bufferWidth, camera->m_bufferHeight );

	DestroySsaoBuffer( scene->ssaoBuffer );
	scene->ssaoBuffer = CreateSsaoBuffer( camera->m_bufferWidth, camera->m_bufferHeight );

	CheckOpenGL();
}

void DrawDebugCapsule( Scene* scene, DebugCapsule* capsule, b3Transform transform, b3HexColor color, float alpha )
{
	float radius = capsule->radius;
	b3Vec3 c1 = capsule->center1;
	b3Vec3 c2 = capsule->center2;
	b3Vec3 origin = 0.5f * ( c1 + c2 );
	b3Vec3 d = c2 - c1;
	float length = b3Length( d );

	assert( length > 0.005f );

	RGBA8 rgba8 = MakeRGBA8( color, alpha );
	b3Vec3 axis = b3Normalize( d );

	// Capsule mesh is along x-axis. Build from two open hemispheres and an open cylinder.

	// Left cap
	b3Quat qc1 = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisX, -axis );

	b3Transform xfCap1 = {
		.p = b3TransformPoint( transform, c1 ),
		.q = b3MulQuat( transform.q, qc1 ),
	};

	Matrix4 matrixCap1 = MakeMatrix4( xfCap1, radius );

	// Right cap
	b3Quat qc2 = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisX, axis );

	b3Transform xfCap2 = {
		.p = b3TransformPoint( transform, c2 ),
		.q = b3MulQuat( transform.q, qc2 ),
	};

	Matrix4 matrixCap2 = MakeMatrix4( xfCap2, radius );

	// Middle
	b3Transform xfMiddle = {
		.p = b3TransformPoint( transform, origin ),
		.q = b3MulQuat( transform.q, qc2 ),
	};

	Matrix4 matrixMiddle = MakeMatrix4( xfMiddle, { length, radius, radius } );

	if ( alpha < 1.0f )
	{
		scene->transparentOpenHemisphereInstances.push_back( {
			.rgba8 = rgba8,
			.matrix = matrixCap1,
		} );

		scene->transparentOpenHemisphereInstances.push_back( {
			.rgba8 = rgba8,
			.matrix = matrixCap2,
		} );

		scene->transparentOpenCylinderInstances.push_back( {
			.rgba8 = rgba8,
			.matrix = matrixMiddle,
		} );
	}
	else
	{
		scene->openHemisphereInstances.push_back( {
			.rgba8 = rgba8,
			.matrix = matrixCap1,
		} );

		scene->openHemisphereInstances.push_back( {
			.rgba8 = rgba8,
			.matrix = matrixCap2,
		} );

		scene->openCylinderInstances.push_back( {
			.rgba8 = rgba8,
			.matrix = matrixMiddle,
		} );
	}
}

void DrawDebugHull( Scene* scene, DebugHull* hull, b3Transform transform, b3HexColor color, float alpha )
{
	b3Transform worldTransform = b3MulTransforms( transform, hull->transform );
	Matrix4 matrix = MakeMatrix4( worldTransform, 1.0f );
	InstanceData instanceData = {
		.rgba8 = MakeRGBA8( color, alpha ),
		.matrix = matrix,
	};

	uint32_t id = hull->id;
	if ( alpha < 1.0f )
	{
		assert( scene->transparentHullInstances.contains( id ) );
		scene->transparentHullInstances[id].push_back( instanceData );
	}
	else
	{
		assert( scene->hullInstances.contains( id ) );
		scene->hullInstances[id].push_back( instanceData );
	}
}

void DrawDebugMesh( Scene* scene, DebugMesh* mesh, b3Transform transform, b3HexColor color, float alpha )
{
	b3Transform worldTransform = b3MulTransforms( transform, mesh->transform );
	Matrix4 matrix = MakeMatrix4( worldTransform, mesh->scale );
	InstanceData instanceData = {
		.rgba8 = MakeRGBA8( color, alpha ),
		.matrix = matrix,
	};

	uint32_t id = mesh->id;
	assert( scene->meshInstances.contains( id ) );
	scene->meshInstances[id].push_back( instanceData );
}

void DrawDebugSphere( Scene* scene, DebugSphere* sphere, b3Transform transform, b3HexColor color, float alpha )
{
	b3Vec3 center = sphere->center;
	b3Transform xf = {
		.p = b3TransformPoint( transform, center ),
		.q = transform.q,
	};
	Matrix4 matrix = MakeMatrix4( xf, sphere->radius );
	InstanceData instanceData = {
		.rgba8 = MakeRGBA8( color, alpha ),
		.matrix = matrix,
	};

	if ( alpha < 1.0f )
	{
		scene->transparentSphereInstances.push_back( instanceData );
	}
	else
	{
		scene->sphereInstances.push_back( instanceData );
	}
}

void DrawDebugShape( Scene* scene, DebugShape* debugShape, b3Transform transform, b3HexColor color, float alpha )
{
	switch ( debugShape->type )
	{
		case DebugShapeType::capsule:
			DrawDebugCapsule( scene, &debugShape->capsule, transform, color, alpha );
			break;

		case DebugShapeType::compound:
		{
			DebugCompound* compound = &debugShape->compound;
			for ( int i = 0; i < compound->capsuleCount; ++i )
			{
				DrawDebugCapsule( scene, compound->capsules + i, transform, color, alpha );
			}

			for ( int i = 0; i < compound->hullCount; ++i )
			{
				DrawDebugHull( scene, compound->hulls + i, transform, color, alpha );
			}

			for ( int i = 0; i < compound->meshCount; ++i )
			{
				DrawDebugMesh( scene, compound->meshes + i, transform, color, alpha );
			}

			for ( int i = 0; i < compound->sphereCount; ++i )
			{
				DrawDebugSphere( scene, compound->spheres + i, transform, color, alpha );
			}
		}
		break;

		case DebugShapeType::hull:
			DrawDebugHull( scene, &debugShape->hull, transform, color, alpha );
			break;

		case DebugShapeType::heightField:
		{
			Matrix4 matrix = MakeMatrix4( transform, 1.0f );
			InstanceData instanceData = {
				.rgba8 = MakeRGBA8( color, alpha ),
				.matrix = matrix,
			};

			uint32_t id = debugShape->heightField.id;
			assert( scene->heightFieldInstances.contains( id ) );
			scene->heightFieldInstances[id].push_back( instanceData );
		}
		break;

		case DebugShapeType::mesh:
		{
			b3Transform xf = b3MulTransforms( transform, debugShape->hull.transform );
			Matrix4 matrix = MakeMatrix4( xf, debugShape->mesh.scale );
			InstanceData instanceData = {
				.rgba8 = MakeRGBA8( color, alpha ),
				.matrix = matrix,
			};

			uint32_t id = debugShape->mesh.id;
			assert( scene->meshInstances.contains( id ) );
			scene->meshInstances[id].push_back( instanceData );
		}
		break;

		case DebugShapeType::sphere:
			DrawDebugSphere( scene, &debugShape->sphere, transform, color, alpha );
			break;

		default:
			assert( false );
			break;
	}
}

static void DrawMeshInstances( InstanceShape shape, const InstanceData* instances, int count, bool cull )
{
	if ( count == 0 )
	{
		return;
	}

	if ( cull == false )
	{
		glDisable( GL_CULL_FACE );
	}

	glBindVertexArray( shape.triangleArrayId );
	glBindBuffer( GL_ARRAY_BUFFER, shape.triangleBufferIds[1] );

	int base = 0;
	while ( count > 0 )
	{
		int batchCount = b3MinInt( count, s_batchSize );

		glBufferData( GL_ARRAY_BUFFER, s_batchSize * sizeof( InstanceData ), nullptr, GL_DYNAMIC_DRAW );
		glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( InstanceData ), &instances[base] );
		glDrawArraysInstanced( GL_TRIANGLES, 0, 3 * shape.triangleCount, batchCount );

		// CheckOpenGL();

		count -= s_batchSize;
		base += s_batchSize;
	}

	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );

	if ( cull == false )
	{
		glEnable( GL_CULL_FACE );
	}
}

static void DrawEdgeInstances( InstanceShape shape, const InstanceData* instances, int count )
{
	if ( count == 0 )
	{
		return;
	}

	glBindVertexArray( shape.edgeArrayId );
	glBindBuffer( GL_ARRAY_BUFFER, shape.edgeBufferIds[1] );

	int base = 0;
	while ( count > 0 )
	{
		int batchCount = b3MinInt( count, s_batchSize );

		glBufferData( GL_ARRAY_BUFFER, s_batchSize * sizeof( InstanceData ), nullptr, GL_DYNAMIC_DRAW );
		glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( InstanceData ), &instances[base] );
		glDrawArraysInstanced( GL_LINES, 0, 2 * shape.edgeCount, batchCount );

		// CheckOpenGL();

		count -= s_batchSize;
		base += s_batchSize;
	}

	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
}

static void DrawMeshes( Scene* scene, const Matrix4& view, const Matrix4& projection, bool forShadow )
{
	if ( forShadow == true )
	{
		// Receiver-plane depth bias in the deferred shader handles per-pixel bias
		// precisely, so the shadow pass needs no polygon offset. Adding one back
		// reintroduces "Peter Panning" — a visible gap between objects and their
		// contact shadows.
		glDisable( GL_POLYGON_OFFSET_FILL );

		glUseProgram( scene->meshShadowShader );
		SetUniformMatrix4( scene->meshShadowShader, "lightSpaceMatrix", view );
	}
	else
	{
		glEnable( GL_POLYGON_OFFSET_FILL );
		glPolygonOffset( 2.0f, 2.0f );

		glUseProgram( scene->meshShader );
		SetUniformMatrix4( scene->meshShader, "view", view );
		SetUniformMatrix4( scene->meshShader, "projection", projection );
	}

	DrawMeshInstances( scene->sphere, scene->sphereInstances.data(), (int)scene->sphereInstances.size(), true );
	DrawMeshInstances( scene->openHemisphere, scene->openHemisphereInstances.data(), (int)scene->openHemisphereInstances.size(),
					   true );
	DrawMeshInstances( scene->openCylinder, scene->openCylinderInstances.data(), (int)scene->openCylinderInstances.size(), true );

	for ( std::unordered_map<uint32_t, std::vector<InstanceData>>::iterator instances = scene->heightFieldInstances.begin();
		  instances != scene->heightFieldInstances.end(); ++instances )
	{
		uint32_t id = instances->first;
		InstanceShape shape = scene->sharedHeightFields.at( id );
		DrawMeshInstances( shape, instances->second.data(), (int)instances->second.size(), true );
	}

	for ( std::unordered_map<uint32_t, std::vector<InstanceData>>::iterator instances = scene->hullInstances.begin();
		  instances != scene->hullInstances.end(); ++instances )
	{
		uint32_t id = instances->first;
		InstanceShape shape = scene->sharedHulls.at( id );
		DrawMeshInstances( shape, instances->second.data(), (int)instances->second.size(), true );
	}

	for ( std::unordered_map<uint32_t, std::vector<InstanceData>>::iterator instances = scene->meshInstances.begin();
		  instances != scene->meshInstances.end(); ++instances )
	{
		uint32_t id = instances->first;
		InstanceShape shape = scene->sharedMeshes.at( id );
		DrawMeshInstances( shape, instances->second.data(), (int)instances->second.size(), false );
	}

	glDisable( GL_POLYGON_OFFSET_FILL );
}

static void DrawTransparentShapes( Scene* scene, const Matrix4& view, const Matrix4& projection, b3Vec3 sunDirWorld,
								   b3Vec3 cameraPos )
{
	glUseProgram( scene->meshForwardShader );
	SetUniformVec3( scene->meshForwardShader, "sunDirWorld", sunDirWorld );
	SetUniformVec3( scene->meshForwardShader, "cameraPos", cameraPos );

	SetUniformMatrix4( scene->meshForwardShader, "view", view );
	SetUniformMatrix4( scene->meshForwardShader, "projection", projection );

	glEnable( GL_POLYGON_OFFSET_FILL );
	glPolygonOffset( 2.0, 2.0 );

	glCullFace( GL_BACK );
	glEnable( GL_CULL_FACE );
	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	glDepthMask( GL_FALSE );

	DrawMeshInstances( scene->sphere, scene->transparentSphereInstances.data(), (int)scene->transparentSphereInstances.size(),
					   true );
	DrawMeshInstances( scene->openHemisphere, scene->transparentOpenHemisphereInstances.data(),
					   (int)scene->transparentOpenHemisphereInstances.size(), true );
	DrawMeshInstances( scene->openCylinder, scene->transparentOpenCylinderInstances.data(),
					   (int)scene->transparentOpenCylinderInstances.size(), true );

	for ( std::unordered_map<uint32_t, std::vector<InstanceData>>::iterator hulls = scene->transparentHullInstances.begin();
		  hulls != scene->transparentHullInstances.end(); ++hulls )
	{
		uint32_t id = hulls->first;
		InstanceShape shape = scene->sharedHulls.at( id );
		DrawMeshInstances( shape, hulls->second.data(), (int)hulls->second.size(), true );
	}

	glDisable( GL_POLYGON_OFFSET_FILL );
	glDisable( GL_BLEND );
	glDepthMask( GL_TRUE );
}

#if 0
static void DrawObjects( Scene* scene, uint32_t shader, bool isShadow )
{
	CheckOpenGL();

	// glFrontFace( GL_CCW );
	// glDepthFunc( GL_LESS );
	// glCullFace( GL_BACK );
	glEnable( GL_POLYGON_OFFSET_FILL );
	glPolygonOffset( 2.0, 2.0 );

	for ( int i = 0; i < scene->uniqueShapes.size(); ++i )
	{
		UniqueShape& shape = scene->uniqueShapes[i];
		if ( isShadow && shape.mesh->shadowCaster == false )
		{
			continue;
		}

		if ( shape.color.a < 1.0f )
		{
			continue;
		}

		SetUniformMatrix4( shader, "model", shape.matrix );
		SetUniformColor( shader, "objectColor", shape.color );
		SetUniformBool( shader, "invertedNormals", false );
		CheckOpenGL();

		glBindVertexArray( shape.mesh->vertexArrayId );
		glDrawArrays( GL_TRIANGLES, 0, 3 * shape.mesh->triangleCount );
		glBindVertexArray( 0 );

		CheckOpenGL();
	}

	glDisable( GL_POLYGON_OFFSET_FILL );
}
#endif

static void DrawLines( Scene* scene, const Matrix4& view, const Matrix4& projection )
{
	// WARNING: very slow
	// glEnable( GL_LINE_SMOOTH );

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	glUseProgram( scene->meshEdgeShader );
	SetUniformMatrix4( scene->meshEdgeShader, "view", view );
	SetUniformMatrix4( scene->meshEdgeShader, "projection", projection );

	for ( std::unordered_map<uint32_t, std::vector<InstanceData>>::iterator instances = scene->heightFieldInstances.begin();
		  instances != scene->heightFieldInstances.end(); ++instances )
	{
		uint32_t id = instances->first;
		InstanceShape shape = scene->sharedHeightFields.at( id );
		DrawEdgeInstances( shape, instances->second.data(), (int)instances->second.size() );
	}

	for ( std::unordered_map<uint32_t, std::vector<InstanceData>>::iterator instances = scene->hullInstances.begin();
		  instances != scene->hullInstances.end(); ++instances )
	{
		uint32_t id = instances->first;
		InstanceShape shape = scene->sharedHulls.at( id );
		DrawEdgeInstances( shape, instances->second.data(), (int)instances->second.size() );
	}

	for ( std::unordered_map<uint32_t, std::vector<InstanceData>>::iterator instances = scene->meshInstances.begin();
		  instances != scene->meshInstances.end(); ++instances )
	{
		uint32_t id = instances->first;
		InstanceShape shape = scene->sharedMeshes.at( id );
		DrawEdgeInstances( shape, instances->second.data(), (int)instances->second.size() );
	}

	glDisable( GL_BLEND );
}

static Matrix4 LookAt( b3Vec3 eye, b3Vec3 center, b3Vec3 up )
{
	// Build view transform
	b3Vec3 axisZ = b3Normalize( center - eye );
	b3Vec3 axisX = b3Normalize( b3Cross( axisZ, up ) );
	b3Vec3 axisY = b3Cross( axisX, axisZ );

	// Negate
	axisZ = -axisZ;

	Matrix4 out;
	out.cx = { axisX.x, axisY.x, axisZ.x, 0.0f };
	out.cy = { axisX.y, axisY.y, axisZ.y, 0.0f };
	out.cz = { axisX.z, axisY.z, axisZ.z, 0.0f };
	out.cw = { -b3Dot( axisX, eye ), -b3Dot( axisY, eye ), -b3Dot( axisZ, eye ), 1.0f };

	return out;
}

void DrawScene( Scene* scene, Camera* camera )
{
	// float temp1[2] = {};
	// glGetFloatv( GL_ALIASED_LINE_WIDTH_RANGE, temp1 );

	// float temp2[2] = {};
	// glGetFloatv( GL_SMOOTH_LINE_WIDTH_RANGE, temp2 );

	// CheckOpenGL();
	//  glDisable( GL_LINE_SMOOTH );

	// GLboolean test = glIsEnabled( GL_LINE_SMOOTH );
	// B3_UNUSED( test );

	// todo wirefame
	// glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

	glDepthFunc( GL_LESS );

	// Get camera matrices
	Matrix4 projection = camera->GetProjectionMatrix();
	Matrix4 view = camera->GetViewMatrix();
	Matrix4 cameraMatrix = camera->GetWorldMatrix();

	// 1. render shadow map — fit the ortho frustum to the camera's view frustum
	// each frame so all shadow-map texels go to what the camera can actually see.
	Matrix4 lightView = LookAt( scene->lightPosition, { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f } );

	// Scale the shadow region with camera radius so large scenes still get
	// covered. Trades off texel density at long camera distances — true large
	// scenes want cascaded shadow maps, but a single fitted cascade plus an edge
	// fade in the shader keeps the cutoff invisible at this scale.
	//
	// In third-person the radius is the orbit distance to the pivot, so
	// 1.5 * radius is a tight cover of the actual scene of interest and a small
	// floor (8) gives great texel density at close orbits. In first-person the
	// radius is a stale value — the camera moves m_position directly while
	// m_radius keeps whatever it was last set to. Fall back to a wider fixed
	// floor there so the shadow frustum still covers the scene the camera can
	// see, at the cost of some texel density.
	float minShadowDistance = camera->m_thirdPerson ? 8.0f : 60.0f;
	const float shadowDistance = b3ClampFloat( 1.5f * camera->m_radius, minShadowDistance, 500.0f );
	float shadowNear = camera->m_nearPlane;
	float shadowFar = b3MinFloat( shadowDistance, camera->m_farPlane );

	b3Vec3 lookDir = -camera->m_forward; // m_forward is the camera-z axis (away from view)
	b3Vec3 nearC = camera->m_position + shadowNear * lookDir;
	b3Vec3 farC = camera->m_position + shadowFar * lookDir;
	float nearH = shadowNear * camera->m_tanY;
	float nearW = camera->m_ratio * nearH;
	float farH = shadowFar * camera->m_tanY;
	float farW = camera->m_ratio * farH;

	b3Vec3 frustumCorners[8] = {
		nearC + ( -nearW ) * camera->m_right + ( -nearH ) * camera->m_up,
		nearC + (nearW)*camera->m_right + ( -nearH ) * camera->m_up,
		nearC + (nearW)*camera->m_right + (nearH)*camera->m_up,
		nearC + ( -nearW ) * camera->m_right + (nearH)*camera->m_up,
		farC + ( -farW ) * camera->m_right + ( -farH ) * camera->m_up,
		farC + (farW)*camera->m_right + ( -farH ) * camera->m_up,
		farC + (farW)*camera->m_right + (farH)*camera->m_up,
		farC + ( -farW ) * camera->m_right + (farH)*camera->m_up,
	};

	Vector4 c0 = MulMV( lightView, { frustumCorners[0].x, frustumCorners[0].y, frustumCorners[0].z, 1.0f } );
	float minX = c0.x, maxX = c0.x;
	float minY = c0.y, maxY = c0.y;
	float minZ = c0.z, maxZ = c0.z;
	for ( int i = 1; i < 8; ++i )
	{
		Vector4 c = MulMV( lightView, { frustumCorners[i].x, frustumCorners[i].y, frustumCorners[i].z, 1.0f } );
		minX = b3MinFloat( minX, c.x );
		maxX = b3MaxFloat( maxX, c.x );
		minY = b3MinFloat( minY, c.y );
		maxY = b3MaxFloat( maxY, c.y );
		minZ = b3MinFloat( minZ, c.z );
		maxZ = b3MaxFloat( maxZ, c.z );
	}

	// Texel-snap the fitted ortho origin so the shadow texel grid stays
	// world-locked frame-to-frame. Without this, sub-texel drift as the camera
	// moves creates a wandering pattern on lit surfaces that reads as banding.
	// Span is held constant within a frame, so each shadow texel maps to exactly
	// (texelX, texelY) world-space units perpendicular to the light direction.
	float texelX = ( maxX - minX ) / float( shadowWidth );
	float texelY = ( maxY - minY ) / float( shadowHeight );
	minX = floorf( minX / texelX ) * texelX;
	minY = floorf( minY / texelY ) * texelY;
	maxX = minX + texelX * float( shadowWidth );
	maxY = minY + texelY * float( shadowHeight );

	// Pull the near plane back so off-camera geometry between the light and the
	// view frustum still casts shadows into the visible region.
	const float zPad = 50.0f;
	float lightNearPlane = -maxZ - zPad;
	float lightFarPlane = -minZ;

	Matrix4 lightProjection = MakeOrthographicMatrix( minX, maxX, minY, maxY, lightNearPlane, lightFarPlane );
	Matrix4 lightSpaceMatrix = MulMM( lightProjection, lightView );

	// World-space size of one shadow texel (light view is rigid, so light-space
	// distances equal world-space distances). The deferred shader uses this for
	// normal-offset bias; max() keeps the offset conservative when texels are
	// non-square.
	float worldTexelSize = b3MaxFloat( texelX, texelY );

	// render scene from light's point of view
	glUseProgram( scene->shadowShader );
	SetUniformMatrix4( scene->shadowShader, "lightSpaceMatrix", lightSpaceMatrix );
	CheckOpenGL();

	glViewport( 0, 0, shadowWidth, shadowHeight );
	glBindFramebuffer( GL_FRAMEBUFFER, scene->depthMapFBO );
	glClear( GL_DEPTH_BUFFER_BIT );
	glActiveTexture( GL_TEXTURE0 );

	// Back-face cull during the shadow pass: combined with slope-scaled polygon
	// offset (applied in DrawMeshes) this eliminates self-shadow acne on solid
	// hulls without the gap that front-face culling introduces.
	glCullFace( GL_BACK );
	glEnable( GL_CULL_FACE );

	CheckOpenGL();

	// DrawObjects( scene, scene->shadowShader, true );
	DrawMeshes( scene, lightSpaceMatrix, Matrix4{}, true );

	glBindFramebuffer( GL_FRAMEBUFFER, 0 );

	// reset viewport
	glViewport( 0, 0, camera->m_bufferWidth, camera->m_bufferHeight );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	// 2. geometry pass: render scene's geometry/color data into geoBuffer
	BindGeoBuffer( scene->geoBuffer );
	glUseProgram( scene->geometryShader );
	SetUniformMatrix4( scene->geometryShader, "view", view );
	SetUniformMatrix4( scene->geometryShader, "projection", projection );

	glCullFace( GL_BACK );
	glEnable( GL_CULL_FACE );

	// DrawObjects( scene, scene->geometryShader, false );
	DrawMeshes( scene, view, projection, false );

	glBindFramebuffer( GL_FRAMEBUFFER, 0 );

	// 3. generate SSAO texture (full resolution — sharp silhouettes)
	BindSsaoBuffer( scene->ssaoBuffer );
	glUseProgram( scene->ssaoShader );
	SetUniformMatrix4( scene->ssaoShader, "projection", projection );
	SetUniformFloat( scene->ssaoShader, "radius", scene->ao.radius );
	SetUniformFloat( scene->ssaoShader, "bias", scene->ao.bias );
	SetUniformFloat( scene->ssaoShader, "minScale", scene->ao.minScale );
	{
		int loc = glGetUniformLocation( scene->ssaoShader, "noiseScale" );
		glUniform2f( loc, float( camera->m_bufferWidth ) / 4.0f, float( camera->m_bufferHeight ) / 4.0f );
	}
	SetSsaoSamples( scene->ssaoBuffer, scene->ssaoShader );
	BindGeoTextures( scene->geoBuffer );
	BindSsaoTextures( scene->ssaoBuffer );
	DrawQuad();
	glBindFramebuffer( GL_FRAMEBUFFER, 0 );

	CheckOpenGL();

	// 4. bilateral blur of SSAO — depth-weighted so the blur doesn't leak across silhouettes.
	BindSsaoBlurBuffer( scene->ssaoBuffer );
	glUseProgram( scene->ssaoBlurShader );
	BindSsaoColorTexture( scene->ssaoBuffer );
	BindGeoPosition( scene->geoBuffer, 1 );
	DrawQuad();
	glBindFramebuffer( GL_FRAMEBUFFER, 0 );

	CheckOpenGL();

	// 5. lighting pass: calculate lighting by iterating over a screen filled quad pixel-by-pixel using the geoBuffer's content.
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glUseProgram( scene->deferredShader );
	BindGeoTextures( scene->geoBuffer );
	BindSsaoBlurTexture( scene->ssaoBuffer );

	glActiveTexture( GL_TEXTURE4 );
	glBindTexture( GL_TEXTURE_2D, scene->depthMap );

	SetUniformBool( scene->deferredShader, "useSSAO", scene->useSSAO );
	SetUniformBool( scene->deferredShader, "useShadow", scene->useShadow );
	SetUniformBool( scene->deferredShader, "useGrid", scene->useGrid );
	SetUniformFloat( scene->deferredShader, "aoPower", scene->ao.power );
	SetUniformFloat( scene->deferredShader, "aoDirect", scene->ao.direct );
	SetUniformBool( scene->deferredShader, "aoOnly", scene->ao.aoOnly );

	// Sun direction in world space (from scene toward sun) — shared with the shadow projection,
	// which already uses scene->lightPosition as the light source.
	b3Vec3 sunDirWorld = b3Normalize( scene->lightPosition );
	SetUniformVec3( scene->deferredShader, "sunDirWorld", sunDirWorld );

	// View-ray reconstruction parameters for the sky/fog path.
	SetUniformFloat( scene->deferredShader, "tanHalfFovY", camera->m_tanY );
	SetUniformFloat( scene->deferredShader, "aspectRatio", camera->m_ratio );

	SetUniformMatrix4( scene->deferredShader, "lightSpaceMatrix", lightSpaceMatrix );
	SetUniformMatrix4( scene->deferredShader, "cameraMatrix", cameraMatrix );
	SetUniformFloat( scene->deferredShader, "worldTexelSize", worldTexelSize );
	SetUniformFloat( scene->deferredShader, "lightZRange", lightFarPlane - lightNearPlane );

	// finally render quad
	DrawQuad();

	CheckOpenGL();

	// 5.5. Copy geometry depth to the default framebuffer using a depth-only draw pass.
	// This avoids glBlitFramebuffer which is very slow on Apple Silicon.
	glUseProgram( scene->depthCopyShader );
	BindGeoDepthTexture( scene->geoBuffer, 0 );
	glColorMask( GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE );
	glDepthFunc( GL_ALWAYS );
	glDepthMask( GL_TRUE );
	DrawQuad();
	glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );
	glDepthFunc( GL_LESS );

	CheckOpenGL();

	// 6. forward render lights on top of scene
	// glUseProgram( scene->forwardShader );
	// SetUniformMatrix4( scene->forwardShader, "projection", projection );
	// SetUniformMatrix4( scene->forwardShader, "view", view );

	// b3Transform transform;
	// transform.p = scene->lightPosition;
	// transform.q = b3Quat_identity;
	//  Matrix4 m = MakeMatrix4( transform, 0.25f );
	//  SetUniformMatrix4( scene->forwardShader, "model", m );
	//  SetUniformVec3( scene->forwardShader, "color", { 1.0f, 1.0f, 1.0f } );
	//  DrawCube( scene, transform, b3_colorWhite, view, projection );

	// Draw mesh edges
	DrawLines( scene, view, projection );

	// Draw transparent hulls
	DrawTransparentShapes( scene, view, projection, b3Normalize( scene->lightPosition ), camera->GetPosition() );

	// 7. Debug draw lines and points
	FlushLines( scene->lineDraw, camera );
	FlushPoints( scene->pointDraw, camera );

	// 8. Clear instances
	// scene->uniqueShapes.clear();
	scene->sphereInstances.clear();
	scene->openHemisphereInstances.clear();
	scene->openCylinderInstances.clear();

	// Clear instances
	for ( std::unordered_map<uint32_t, std::vector<InstanceData>>::iterator instance = scene->heightFieldInstances.begin();
		  instance != scene->heightFieldInstances.end(); ++instance )
	{
		instance->second.clear();
	}

	for ( std::unordered_map<uint32_t, std::vector<InstanceData>>::iterator instance = scene->hullInstances.begin();
		  instance != scene->hullInstances.end(); ++instance )
	{
		instance->second.clear();
	}

	for ( std::unordered_map<uint32_t, std::vector<InstanceData>>::iterator instance = scene->meshInstances.begin();
		  instance != scene->meshInstances.end(); ++instance )
	{
		instance->second.clear();
	}

	scene->transparentSphereInstances.clear();
	scene->transparentOpenHemisphereInstances.clear();
	scene->transparentOpenCylinderInstances.clear();

	for ( std::unordered_map<uint32_t, std::vector<InstanceData>>::iterator hulls = scene->transparentHullInstances.begin();
		  hulls != scene->transparentHullInstances.end(); ++hulls )
	{
		hulls->second.clear();
	}
}

void DrawDebugCube( Scene* scene, b3Transform transform, RGBAf color )
{
	int index = (int)scene->debugCubes.size();
	scene->debugCubes[index] = { transform, color };
}

void EnableShadows( Scene* scene, bool useShadows )
{
	scene->useShadow = useShadows;
}

bool AreShadowsEnabled( Scene* scene )
{
	return scene->useShadow;
}

void EnableSSAO( Scene* scene, bool enable )
{
	scene->useSSAO = enable;
}

bool IsSSAOEnabled( Scene* scene )
{
	return scene->useSSAO;
}

void EnableGrid( Scene* scene, bool enable )
{
	scene->useGrid = enable;
}

bool IsGridEnabled( Scene* scene )
{
	return scene->useGrid;
}

SceneAOSettings* GetAOSettings( Scene* scene )
{
	return &scene->ao;
}

void DrawPoint( Scene* scene, b3Vec3 point, float size, b3HexColor color )
{
	AddPoint( scene->pointDraw, point, size, color );
}

void DrawLine( Scene* scene, b3Vec3 point1, b3Vec3 point2, b3HexColor color )
{
	AddLine( scene->lineDraw, point1, point2, color );
}

void DrawPlane( Scene* scene, b3Vec3 normal, b3Vec3 point, b3HexColor color )
{
	b3Vec3 perp1 = b3Perp( normal );
	b3Vec3 perp2 = b3Cross( perp1, normal );
	b3Vec3 p = point;
	float a = 1.0f;
	b3Vec3 p1 = p + a * perp1 + a * perp2;
	b3Vec3 p2 = p - a * perp1 + a * perp2;
	b3Vec3 p3 = p - a * perp1 - a * perp2;
	b3Vec3 p4 = p + a * perp1 - a * perp2;

	AddLine( scene->lineDraw, p1, p2, color );
	AddLine( scene->lineDraw, p2, p3, color );
	AddLine( scene->lineDraw, p3, p4, color );
	AddLine( scene->lineDraw, p4, p1, color );
	AddLine( scene->lineDraw, p, p + 0.5f * normal, color );
	AddPoint( scene->pointDraw, p, 10.0f, color );
}

void DrawArrow( Scene* scene, b3Vec3 point1, b3Vec3 point2, float size, b3HexColor color )
{
	AddLine( scene->lineDraw, point1, point2, color );
	b3Vec3 axis = b3Normalize( point2 - point1 );
	b3Vec3 perp = b3Perp( axis );
	AddLine( scene->lineDraw, point2, point2 - size * axis + size * perp, color );
	AddLine( scene->lineDraw, point2, point2 - size * axis - size * perp, color );
}

void DrawBounds( Scene* scene, b3AABB bounds, float extension, b3HexColor color )
{
	LineDraw* draw = scene->lineDraw;

	b3Vec3 extents = b3AABB_Extents( bounds );
	b3Vec3 center = b3AABB_Center( bounds );
	float hx = extents.x + extension;
	float hy = extents.y + extension;
	float hz = extents.z + extension;

	b3Vec3 p1 = b3Vec3{ -hx, -hy, hz } + center;
	b3Vec3 p2 = b3Vec3{ -hx, hy, hz } + center;
	b3Vec3 p3 = b3Vec3{ -hx, hy, -hz } + center;
	b3Vec3 p4 = b3Vec3{ -hx, -hy, -hz } + center;

	b3Vec3 p5 = b3Vec3{ hx, -hy, hz } + center;
	b3Vec3 p6 = b3Vec3{ hx, hy, hz } + center;
	b3Vec3 p7 = b3Vec3{ hx, hy, -hz } + center;
	b3Vec3 p8 = b3Vec3{ hx, -hy, -hz } + center;

	// negative x loop
	AddLine( draw, p1, p2, color );
	AddLine( draw, p2, p3, color );
	AddLine( draw, p3, p4, color );
	AddLine( draw, p4, p1, color );

	// positive x loop
	AddLine( draw, p5, p6, color );
	AddLine( draw, p6, p7, color );
	AddLine( draw, p7, p8, color );
	AddLine( draw, p8, p5, color );

	// connect loops
	AddLine( draw, p1, p5, color );
	AddLine( draw, p2, p6, color );
	AddLine( draw, p3, p7, color );
	AddLine( draw, p4, p8, color );
}

void DrawBox( Scene* scene, b3Vec3 extents, b3Transform transform, b3HexColor color )
{
	LineDraw* draw = scene->lineDraw;

	float hx = extents.x;
	float hy = extents.y;
	float hz = extents.z;

	b3Vec3 p1 = b3TransformPoint( transform, { -hx, -hy, hz } );
	b3Vec3 p2 = b3TransformPoint( transform, { -hx, hy, hz } );
	b3Vec3 p3 = b3TransformPoint( transform, { -hx, hy, -hz } );
	b3Vec3 p4 = b3TransformPoint( transform, { -hx, -hy, -hz } );

	b3Vec3 p5 = b3TransformPoint( transform, { hx, -hy, hz } );
	b3Vec3 p6 = b3TransformPoint( transform, { hx, hy, hz } );
	b3Vec3 p7 = b3TransformPoint( transform, { hx, hy, -hz } );
	b3Vec3 p8 = b3TransformPoint( transform, { hx, -hy, -hz } );

	// negative x loop
	AddLine( draw, p1, p2, color );
	AddLine( draw, p2, p3, color );
	AddLine( draw, p3, p4, color );
	AddLine( draw, p4, p1, color );

	// positive x loop
	AddLine( draw, p5, p6, color );
	AddLine( draw, p6, p7, color );
	AddLine( draw, p7, p8, color );
	AddLine( draw, p8, p5, color );

	// connect loops
	AddLine( draw, p1, p5, color );
	AddLine( draw, p2, p6, color );
	AddLine( draw, p3, p7, color );
	AddLine( draw, p4, p8, color );
}

void DrawFace( Scene* scene, b3Vec3 vertex1, b3Vec3 vertex2, b3Vec3 vertex3, b3HexColor color )
{
	LineDraw* draw = scene->lineDraw;

	AddLine( draw, vertex1, vertex2, color );
	AddLine( draw, vertex2, vertex3, color );
	AddLine( draw, vertex3, vertex1, color );
}
