// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "mesh_loader.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#define TINYOBJLOADER_IMPLEMENTATION
#define TINYOBJLOADER_USE_MAPBOX_EARCUT
#include "tiny_obj_loader.h"

#include <stdio.h>

// A missing obj is refused rather than fatal, so the samples still run from a
// directory with no data beside them.
static bool ParseObjFile( tinyobj::ObjReader& reader, const char* path )
{
	tinyobj::ObjReaderConfig readerConfig;
	readerConfig.triangulate = true;
	readerConfig.mtl_search_path = "./"; // Path to material files

	if ( reader.ParseFromFile( path, readerConfig ) == false )
	{
		fprintf( stderr, "Failed to load mesh '%s'", path );
		if ( reader.Error().empty() == false )
		{
			fprintf( stderr, ": %s", reader.Error().c_str() );
		}
		fprintf( stderr, "\n" );
		return false;
	}

	if ( reader.Warning().empty() == false )
	{
		fprintf( stderr, "TinyObjReader: %s\n", reader.Warning().c_str() );
	}

	return true;
}

// Load a wavefront obj file using tiny obj loader and earcut for triangulation.
bool LoadTempMesh( const char* path, TempMesh* tempMesh, float scale, bool zUp )
{
	tempMesh->vertices.clear();
	tempMesh->indices.clear();
	tempMesh->materialIndices.clear();

	tinyobj::ObjReader reader;
	if ( ParseObjFile( reader, path ) == false )
	{
		return false;
	}

	const tinyobj::attrib_t& attrib = reader.GetAttrib();
	const std::vector<tinyobj::shape_t>& shapes = reader.GetShapes();
	// const std::vector<tinyobj::material_t>& objMaterials = reader.GetMaterials();

	int vertexCount = int( attrib.vertices.size() / 3 );
	for ( int i = 0; i < vertexCount; i++ )
	{
		float x = scale * attrib.vertices[3 * i + 0];
		float y = scale * attrib.vertices[3 * i + 1];
		float z = scale * attrib.vertices[3 * i + 2];
		b3Vec3 vertex = zUp ? b3Vec3{ y, z, x } : b3Vec3{ x, y, z };
		tempMesh->vertices.push_back( vertex );
	}

	// Loop over shapes
	int shapeCount = (int)shapes.size();
	int materialIndex = 0;
	for ( int shapeIndex = 0; shapeIndex < shapeCount; ++shapeIndex )
	{
		int faceCount = (int)shapes[shapeIndex].mesh.num_face_vertices.size();
		int baseIndex = 0;
		for ( int faceIndex = 0; faceIndex < faceCount; ++faceIndex )
		{
			int faceVertexCount = (int)shapes[shapeIndex].mesh.num_face_vertices[faceIndex];

			// todo only triangles for now
			if ( faceVertexCount != 3 )
			{
				baseIndex += faceVertexCount;
				continue;
			}

			for ( int vertexIndex = 0; vertexIndex < faceVertexCount; ++vertexIndex )
			{
				tinyobj::index_t idx = shapes[shapeIndex].mesh.indices[baseIndex + vertexIndex];
				tempMesh->indices.push_back( idx.vertex_index );
			}

			tempMesh->materialIndices.push_back( materialIndex );
			materialIndex = ( materialIndex + 1 ) % 3;

			baseIndex += faceVertexCount;
		}
	}

	return true;
}

b3MeshData* CreateMeshData( const char* path, float scale, bool zUp, bool useMedianSplit, bool identifyConvexEdges, bool weldVertices  )
{
	tinyobj::ObjReader reader;
	if ( ParseObjFile( reader, path ) == false )
	{
		return nullptr;
	}

	const tinyobj::attrib_t& attrib = reader.GetAttrib();
	const std::vector<tinyobj::shape_t>& shapes = reader.GetShapes();
	// const std::vector<tinyobj::material_t>& objMaterials = reader.GetMaterials();

	TempMesh mesh;

	int vertexCount = int( attrib.vertices.size() / 3 );
	for ( int i = 0; i < vertexCount; i++ )
	{
		float x = scale * attrib.vertices[3 * i + 0];
		float y = scale * attrib.vertices[3 * i + 1];
		float z = scale * attrib.vertices[3 * i + 2];
		b3Vec3 vertex = zUp ? b3Vec3{ y, z, x } : b3Vec3{ x, y, z };
		mesh.vertices.push_back( vertex );
	}

	// Loop over shapes
	int shapeCount = (int)shapes.size();
	int materialIndex = 0;
	for ( int shapeIndex = 0; shapeIndex < shapeCount; ++shapeIndex )
	{
		int faceCount = (int)shapes[shapeIndex].mesh.num_face_vertices.size();
		int baseIndex = 0;
		for ( int faceIndex = 0; faceIndex < faceCount; ++faceIndex )
		{
			int faceVertexCount = (int)shapes[shapeIndex].mesh.num_face_vertices[faceIndex];

			// todo only triangles for now
			if ( faceVertexCount != 3 )
			{
				baseIndex += faceVertexCount;
				continue;
			}

			for ( int vertexIndex = 0; vertexIndex < faceVertexCount; ++vertexIndex )
			{
				tinyobj::index_t idx = shapes[shapeIndex].mesh.indices[baseIndex + vertexIndex];
				mesh.indices.push_back( idx.vertex_index );
			}

			mesh.materialIndices.push_back( materialIndex );
			materialIndex = ( materialIndex + 1 ) % 3;

			baseIndex += faceVertexCount;
		}
	}

	b3MeshDef def = {};
	def.vertices = mesh.vertices.data();
	def.vertexCount = (int)mesh.vertices.size();
	def.indices = mesh.indices.data();
	def.triangleCount = (int)mesh.indices.size() / 3;
	def.materialIndices = mesh.materialIndices.data();
	def.useMedianSplit = useMedianSplit;
	def.identifyEdges = identifyConvexEdges;
	def.weldVertices = weldVertices;
	def.weldTolerance = 0.002f;

	b3MeshData* meshData = b3CreateMesh( &def, nullptr, 0 );
	if ( meshData == nullptr )
	{
		fprintf( stderr, "Mesh '%s' has no usable triangles\n", path );
	}

	return meshData;
}

void DestroyMeshData( b3MeshData* meshData )
{
	if ( meshData != nullptr )
	{
		b3DestroyMesh( meshData );
	}
}
