// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "utility.h"

struct b3Capsule;
struct b3DebugShape;
struct b3Hull;
struct b3MeshData;
struct b3Sphere;

struct Camera;
struct LineDraw;
struct PointDraw;
struct Scene;

void* CreateDebugShape( const b3DebugShape* debugShape, void* context );
void DestroyDebugShape( void* userShape, void* userContext );

void DrawText( int x, int y, b3HexColor color, const char* text );
void DrawFormat( int x, int y, b3HexColor color, const char* format, ... );
void DrawWorldString( Camera* camera, b3Vec3 point, b3HexColor color, const char* format, ... );

void DrawGrid( Scene* scene, int size );
void DrawCross( Scene* scene, b3Vec3 center, float radius, b3HexColor color );
void DrawSphere( Scene* scene, const b3Transform& transform, const b3Sphere& sphere, b3HexColor color );
void DrawCapsule( Scene* scene, const b3Transform& transform, const b3Capsule& capsule, b3HexColor color );
void DrawHull( Scene* scene, const b3Transform& transform, const b3Hull* hull, b3HexColor color, bool drawNormals );
void DrawTransform( Scene* scene, const b3Transform& transform, float length );

// Debug points
PointDraw* CreateDebugPoints();
void DestroyDebugPoints( PointDraw* draw );
void AddPoint( PointDraw* draw, b3Vec3 point, float size, b3HexColor color );
void FlushPoints( PointDraw* draw, Camera* camera );

// Debug lines
LineDraw* CreateDebugLines();
void DestroyDebugLines( LineDraw* draw );
void AddLine( LineDraw* draw, b3Vec3 point1, b3Vec3 point2, b3HexColor color );
void FlushLines( LineDraw* draw, Camera* camera );
