// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/shadow.h"

#include "gfx/draw.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

// Cascade fitting parameters. The split range defaults (SHADOW_SPLIT_NEAR /
// SHADOW_SPLIT_FAR, in shadow.h) are *view-space* distances from the camera
// (positive). CSM quality at distance falls off fast, so cascades focus on
// near content.
//
// PSSM_LAMBDA blends the uniform split scheme (each cascade covers an
// equal slice of view-space distance) and the logarithmic scheme (each
// cascade covers an equal multiplicative range). 0 = pure uniform,
// 1 = pure log, 0.5 is the standard sweet spot. Raising it tightens the
// near cascades but drags the boundaries inward, so the last cascade
// takes over closer to the camera where its coarser texels show. This is
// the starting value, SetShadowSplitLambda overrides it.
//
// CASTER_MARGIN_M expands the orthographic light frustum's near edge
// toward the sun so casters above the camera frustum slice (e.g. the
// building roof at y=3 when the slice itself only reaches y=1) still cast
// onto receivers inside the slice.
// DEPTH_BIAS_TEXELS is what remains of the comparison bias once the lit
// shaders offset the lookup along the surface normal. It only has to
// cover depth quantization, so it is measured in texels of the cascade
// that owns it rather than in a fixed world size that would be wrong at
// one end or the other of a wide split range.
#define PSSM_LAMBDA 0.5f
#define CASTER_MARGIN_M 50.0f
#define DEPTH_BIAS_TEXELS 0.5f

static struct
{
	b3Vec3 sunDir;										  // world-space dir TO sun, normalized
	float splitNear;									  // PSSM split range near (positive view-Z)
	float splitFar;										  // PSSM split range far (positive view-Z)
	float splitLambda;									  // uniform/log blend for the split scheme
	sg_image depthArray;								  // D32F, cascade layers
	sg_view cascadeAttachmentViews[SHADOW_CASCADE_COUNT]; // depth-stencil view per cascade
	sg_view sampleView;									  // full-array texture-sample view
	sg_sampler sampler;									  // comparison sampler (LESS_EQUAL, linear PCF)
	Mat4 cascadeMatrices[SHADOW_CASCADE_COUNT];			  // light-space view-proj per cascade
	float cascadeFarViewZ[SHADOW_CASCADE_COUNT];		  // far end (positive view-Z) per cascade
	float cascadeTexelWorld[SHADOW_CASCADE_COUNT];		  // one shadow texel in world length units
	float cascadeDepthBias[SHADOW_CASCADE_COUNT];		  // residual bias in shadow clip-Z units
	bool inPass;										  // safety against unbalanced begin/end
} s_shadow;

// PSSM split scheme: blend uniform and logarithmic. Returns the *positive*
// view-space distance at the boundary of cascade `i` (i in [0..N], where
// split[0] = near and split[N] = far).
static inline float ComputeSplitDistance( int i, int cascadeCount, float nearZ, float farZ )
{
	const float fi = (float)i / (float)cascadeCount;
	const float u = nearZ + ( farZ - nearZ ) * fi;
	const float l = nearZ * powf( farZ / nearZ, fi );
	const float lambda = s_shadow.splitLambda;
	return lambda * l + ( 1.0f - lambda ) * u;
}

// MakePerspectiveAndInverse puts proj[1][1] = 1/tan(fovY/2) and
// proj[0][0] = proj[1][1] / aspect. The analytic inverse stores both
// quantities directly, so callers holding either matrix can recover the
// frustum shape.
static inline void PerspectiveShape( const Mat4* proj, float* tanHalfFovY, float* aspect )
{
	*tanHalfFovY = ( proj->cy.y != 0.0f ) ? ( 1.0f / proj->cy.y ) : 1.0f;
	*aspect = ( proj->cx.x != 0.0f ) ? ( proj->cy.y / proj->cx.x ) : 1.0f;
}

static inline void PerspectiveShapeFromInverse( const Mat4* projInv, float* tanHalfFovY, float* aspect )
{
	*tanHalfFovY = projInv->cy.y;
	*aspect = ( projInv->cy.y != 0.0f ) ? ( projInv->cx.x / projInv->cy.y ) : 1.0f;
}

// Fill cornersOut with the 8 world-space corners of the camera frustum
// slice between view-space distances nearZ and farZ (both positive, the
// camera looks down its own -Z axis in view space).
//
// Camera basis comes straight from `viewInv` (the camera's world matrix
// produced by mat4ViewAndInverse): column 0 is right, column 1 is up,
// column 2 is back (i.e., -forward), column 3 is eye, no inversion.
static void FrustumSliceCornersWorld( const Mat4* viewInv, float tanHalfFovY, float aspect, float nearZ, float farZ,
									  b3Vec3 cornersOut[8] )
{
	const b3Vec3 eye = { viewInv->cw.x, viewInv->cw.y, viewInv->cw.z };
	const b3Vec3 right = { viewInv->cx.x, viewInv->cx.y, viewInv->cx.z };
	const b3Vec3 up = { viewInv->cy.x, viewInv->cy.y, viewInv->cy.z };
	const b3Vec3 forward = { -viewInv->cz.x, -viewInv->cz.y, -viewInv->cz.z };

	const float distances[2] = { nearZ, farZ };
	for ( int s = 0; s < 2; ++s )
	{
		const float d = distances[s];
		const float h = d * tanHalfFovY;
		const float w = h * aspect;
		const b3Vec3 center = b3MulAdd( eye, d, forward );
		for ( int j = 0; j < 4; ++j )
		{
			const float sx = ( j & 1 ) ? +w : -w;
			const float sy = ( j & 2 ) ? +h : -h;
			b3Vec3 c = b3MulAdd( center, sx, right );
			c = b3MulAdd( c, sy, up );
			cornersOut[s * 4 + j] = c;
		}
	}
}

// Bound a frustum slice by a world-space sphere: centroid of the 8
// corners, plus the max distance from it to any of them. The sphere is
// rotation-invariant. Rotating the camera around its eye doesn't change
// the radius, which is the key property that lets the texel-snap
// stabilization avoid swimming edges.
static void SliceBoundingSphere( const Mat4* viewInv, float tanHalfFovY, float aspect, float nearZ, float farZ,
								 b3Vec3* centerOut, float* radiusOut )
{
	b3Vec3 corners[8];
	FrustumSliceCornersWorld( viewInv, tanHalfFovY, aspect, nearZ, farZ, corners );

	b3Vec3 center = b3Vec3_zero;
	for ( int i = 0; i < 8; ++i )
	{
		center = b3Add( center, corners[i] );
	}
	center = b3MulSV( 1.0f / 8.0f, center );

	float radius = 0.0f;
	for ( int i = 0; i < 8; ++i )
	{
		const b3Vec3 d = b3Sub( corners[i], center );
		const float r = sqrtf( d.x * d.x + d.y * d.y + d.z * d.z );
		if ( r > radius )
		{
			radius = r;
		}
	}
	if ( radius < 0.01f )
	{
		radius = 0.01f;
	}

	*centerOut = center;
	*radiusOut = radius;
}

// Fit one cascade's light-space view-projection matrix to the camera
// frustum slice [nearZ, farZ] for the configured sun direction.
//
// Light view: orthonormal frame whose -Z axis points toward the sun.
// Light proj: orthographic, sized to enclose the sphere. The near plane
// is pushed back along +sun by CASTER_MARGIN_M so casters between the
// slice and the sun are not clipped.
//
// Texel-snap: quantize the sphere center onto a world-anchored lattice
// aligned with the light's own axes, one texel of world per step, then
// build the light matrices around the snapped center. Camera motion now
// slides the cascade in whole texels, so shadow edges step instead of
// crawling.
//
// It has to happen here, on the center, and not by projecting the center
// through the finished lightVP: the center is the look-at target, so it
// lands on clip (0, 0) by construction and rounding it is a no-op.
static Mat4 FitCascade( const Mat4* viewInv, float tanHalfFovY, float aspect, b3Vec3 dirToSun, float nearZ, float farZ,
						float* radiusOut )
{
	b3Vec3 center;
	float radius;
	SliceBoundingSphere( viewInv, tanHalfFovY, aspect, nearZ, farZ, &center, &radius );

	// Avoid a degenerate up vector when the sun is straight up/down.
	b3Vec3 up = b3Vec3_axisY;
	if ( fabsf( dirToSun.y ) > 0.999f )
	{
		up = b3Vec3_axisZ;
	}

	// Light axes, matching the basis MakeLookAt derives below. The eye sits
	// along +sun from the target, so the light looks along -sun. These
	// depend only on the sun and the up choice, never on the camera, which
	// is what makes the lattice hold still.
	const b3Vec3 forward = { -dirToSun.x, -dirToSun.y, -dirToSun.z };
	const b3Vec3 lightRight = b3Normalize( b3Cross( forward, up ) );
	const b3Vec3 lightUp = b3Cross( lightRight, forward );

	// Radius is fixed for a given split, field of view and aspect, so the
	// lattice step is fixed too and the same world points keep landing on
	// the same texels frame to frame.
	const float texelWorld = 2.0f * radius / (float)SHADOW_RESOLUTION;

	// Geometry reaches the GPU shifted against the draw origin, which
	// tracks the camera, so the eye sits at this frame's origin and the
	// frame slides whenever the camera does. Anchoring the lattice here
	// would drag it along and quantize against a moving reference, which
	// is the crawl the snap exists to remove. Fold the draw origin back in
	// and snap the absolute coordinate, in double so the fractional part
	// within a texel survives far from the origin.
	const b3Pos drawOrigin = GetDrawOrigin();
	const double ox = (double)lightRight.x * (double)drawOrigin.x + (double)lightRight.y * (double)drawOrigin.y +
					  (double)lightRight.z * (double)drawOrigin.z;
	const double oy = (double)lightUp.x * (double)drawOrigin.x + (double)lightUp.y * (double)drawOrigin.y +
					  (double)lightUp.z * (double)drawOrigin.z;

	const double step = (double)texelWorld;
	const double ax = ox + (double)b3Dot( lightRight, center );
	const double ay = oy + (double)b3Dot( lightUp, center );
	center = b3MulAdd( center, (float)( round( ax / step ) * step - ax ), lightRight );
	center = b3MulAdd( center, (float)( round( ay / step ) * step - ay ), lightUp );

	// Place the light eye behind the sphere along +sun. Far clip extends
	// past the sphere's back face. Near clip pulls forward by
	// CASTER_MARGIN_M to admit casters above the slice.
	const b3Vec3 eyeWorld = b3MulAdd( center, radius + CASTER_MARGIN_M, dirToSun );
	const Mat4 lightView = MakeLookAt( eyeWorld, center, up );
	const Mat4 lightProj = MakeOrthographic( -radius, radius, -radius, radius, 0.0f, 2.0f * radius + CASTER_MARGIN_M );

	*radiusOut = radius;
	return MulMM4( lightProj, lightView );
}

void InitShadows( void )
{
	s_shadow.sunDir = b3Normalize( (b3Vec3){ 0.5f, 0.8f, 0.4f } );
	s_shadow.splitNear = SHADOW_SPLIT_NEAR;
	s_shadow.splitFar = SHADOW_SPLIT_FAR;
	s_shadow.splitLambda = PSSM_LAMBDA;

	// The lit shaders divide by the texel size to scale the filter against
	// cascade zero, so leaving these zero until the first FitShadows would
	// render a frame of NaN. This is what a unit cascade radius works out to,
	// which makes every ratio one.
	for ( int i = 0; i < SHADOW_CASCADE_COUNT; ++i )
	{
		s_shadow.cascadeTexelWorld[i] = 2.0f / (float)SHADOW_RESOLUTION;
	}

	sg_image_desc desc = { 0 };
	desc.type = SG_IMAGETYPE_ARRAY;
	desc.usage.depth_stencil_attachment = true;
	desc.width = SHADOW_RESOLUTION;
	desc.height = SHADOW_RESOLUTION;
	desc.num_slices = SHADOW_CASCADE_COUNT;
	desc.pixel_format = SG_PIXELFORMAT_DEPTH;
	desc.sample_count = 1;
	desc.label = "shadow_depth_array";
	s_shadow.depthArray = sg_make_image( &desc );

	// Per-cascade depth attachment view (slice = cascade index).
	for ( int i = 0; i < SHADOW_CASCADE_COUNT; ++i )
	{
		sg_view_desc vdesc = { 0 };
		vdesc.depth_stencil_attachment.image = s_shadow.depthArray;
		vdesc.depth_stencil_attachment.slice = i;
		vdesc.label = "shadow_cascade_attachment";
		s_shadow.cascadeAttachmentViews[i] = sg_make_view( &vdesc );
	}

	// Whole-array texture sample view: layer index becomes the .z arg in
	// the shader's sampler2DArrayShadow lookup. Explicitly range over all
	// cascade slices so the view binds the full array
	// (careful: sokol's "count = 0 means all remaining" default).
	sg_view_desc svdesc = { 0 };
	svdesc.texture.image = s_shadow.depthArray;
	svdesc.texture.slices.base = 0;
	svdesc.texture.slices.count = SHADOW_CASCADE_COUNT;
	svdesc.label = "shadow_sample_view";
	s_shadow.sampleView = sg_make_view( &svdesc );

	// Comparison sampler: LESS_EQUAL means the test returns 1.0 (lit) when
	// the receiver's reference depth is <= the stored depth, i.e., the
	// receiver is the closest surface to the light. Linear PCF: each
	// texture call returns a 2x2 weighted compare-result mix.
	//
	// White border means taps past a cascade edge compare against the far
	// plane and come back lit, which dissolves the boundary across the
	// filter kernel. That is what lets the lit shaders drop their hard
	// bounds test, and the hard test is what used to flip whole pixels
	// between sampled and lit as the texel snap stepped the map. Clamping
	// to edge instead would smear the border texel outward and streak
	// shadows across everything past the cascade.
	sg_sampler_desc sdesc = { 0 };
	sdesc.min_filter = SG_FILTER_LINEAR;
	sdesc.mag_filter = SG_FILTER_LINEAR;
	sdesc.mipmap_filter = SG_FILTER_NEAREST;
	sdesc.wrap_u = SG_WRAP_CLAMP_TO_BORDER;
	sdesc.wrap_v = SG_WRAP_CLAMP_TO_BORDER;
	sdesc.wrap_w = SG_WRAP_CLAMP_TO_EDGE;
	sdesc.border_color = SG_BORDERCOLOR_OPAQUE_WHITE;
	sdesc.compare = SG_COMPAREFUNC_LESS_EQUAL;
	sdesc.label = "shadow_compare_sampler";
	s_shadow.sampler = sg_make_sampler( &sdesc );

	s_shadow.inPass = false;
}

void ShutdownShadows( void )
{
	sg_destroy_sampler( s_shadow.sampler );
	sg_destroy_view( s_shadow.sampleView );
	for ( int i = 0; i < SHADOW_CASCADE_COUNT; ++i )
	{
		sg_destroy_view( s_shadow.cascadeAttachmentViews[i] );
	}
	sg_destroy_image( s_shadow.depthArray );
}

void SetShadowSplits( float nearViewZ, float farViewZ )
{
	if ( nearViewZ <= 0.0f || farViewZ <= nearViewZ )
	{
		// Restore defaults on (0, 0) or any nonsense input.
		nearViewZ = SHADOW_SPLIT_NEAR;
		farViewZ = SHADOW_SPLIT_FAR;
	}

	s_shadow.splitNear = nearViewZ;
	s_shadow.splitFar = farViewZ;
}

void SetShadowSplitFar( float farViewZ )
{
	SetShadowSplits( s_shadow.splitNear, farViewZ );
}

float GetShadowSplitNear( void )
{
	return s_shadow.splitNear;
}

float GetShadowSplitFar( void )
{
	return s_shadow.splitFar;
}

void SetShadowSplitLambda( float lambda )
{
	s_shadow.splitLambda = b3ClampFloat( lambda, 0.0f, 1.0f );
}

float GetShadowSplitLambda( void )
{
	return s_shadow.splitLambda;
}

void SetShadowSunDir( b3Vec3 dirToSun )
{
	const float lenSq = dirToSun.x * dirToSun.x + dirToSun.y * dirToSun.y + dirToSun.z * dirToSun.z;
	if ( lenSq > 0.0f )
	{
		s_shadow.sunDir = b3Normalize( dirToSun );
	}
}

void FitShadows( const Mat4* viewInv, const Mat4* proj )
{
	// PSSM splits across [splitNear, splitFar]. split[0] = near,
	// split[N] = far. Cascade i covers (split[i], split[i+1]].
	float splits[SHADOW_CASCADE_COUNT + 1];
	for ( int i = 0; i <= SHADOW_CASCADE_COUNT; ++i )
	{
		splits[i] = ComputeSplitDistance( i, SHADOW_CASCADE_COUNT, s_shadow.splitNear, s_shadow.splitFar );
	}

	float tanHalfFovY, aspect;
	PerspectiveShape( proj, &tanHalfFovY, &aspect );

	for ( int i = 0; i < SHADOW_CASCADE_COUNT; ++i )
	{
		float radius = 0.0f;
		s_shadow.cascadeMatrices[i] =
			FitCascade( viewInv, tanHalfFovY, aspect, s_shadow.sunDir, splits[i], splits[i + 1], &radius );
		s_shadow.cascadeFarViewZ[i] = splits[i + 1];

		// The ortho half-extent is the sphere radius, so the map covers
		// 2 * radius of world across its resolution.
		const float texelWorld = 2.0f * radius / (float)SHADOW_RESOLUTION;
		s_shadow.cascadeTexelWorld[i] = texelWorld;

		// Shadow clip-Z spans the whole orthographic depth range, and the
		// caster margin inflates that range without covering any more
		// receiver. Sizing the bias in world units here decouples it, so
		// the margin costs caster headroom and nothing else.
		const float depthRange = 2.0f * radius + CASTER_MARGIN_M;
		s_shadow.cascadeDepthBias[i] = ( DEPTH_BIAS_TEXELS * texelWorld ) / depthRange;
	}
}

void GetShadowCasterBounds( const Mat4* viewInv, const Mat4* projInv, b3Vec3* loOut, b3Vec3* hiOut )
{
	float tanHalfFovY, aspect;
	PerspectiveShapeFromInverse( projInv, &tanHalfFovY, &aspect );

	b3Vec3 lo = { FLT_MAX, FLT_MAX, FLT_MAX };
	b3Vec3 hi = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

	for ( int i = 0; i < SHADOW_CASCADE_COUNT; ++i )
	{
		const float nearZ = ComputeSplitDistance( i, SHADOW_CASCADE_COUNT, s_shadow.splitNear, s_shadow.splitFar );
		const float farZ = ComputeSplitDistance( i + 1, SHADOW_CASCADE_COUNT, s_shadow.splitNear, s_shadow.splitFar );

		b3Vec3 center;
		float radius;
		SliceBoundingSphere( viewInv, tanHalfFovY, aspect, nearZ, farZ, &center, &radius );

		// A caster can only darken a receiver in this sphere if it sits
		// between the two, so sweep the sphere toward the sun rather than
		// growing it in every direction. Sweeping by the caster margin
		// lands the front face exactly on the cascade's near plane.
		const b3Vec3 swept = b3MulAdd( center, CASTER_MARGIN_M, s_shadow.sunDir );

		lo.x = b3MinFloat( lo.x, b3MinFloat( center.x, swept.x ) - radius );
		lo.y = b3MinFloat( lo.y, b3MinFloat( center.y, swept.y ) - radius );
		lo.z = b3MinFloat( lo.z, b3MinFloat( center.z, swept.z ) - radius );
		hi.x = b3MaxFloat( hi.x, b3MaxFloat( center.x, swept.x ) + radius );
		hi.y = b3MaxFloat( hi.y, b3MaxFloat( center.y, swept.y ) + radius );
		hi.z = b3MaxFloat( hi.z, b3MaxFloat( center.z, swept.z ) + radius );
	}

	*loOut = lo;
	*hiOut = hi;
}

void BeginShadowPass( int cascade )
{
	if ( cascade < 0 || cascade >= SHADOW_CASCADE_COUNT )
	{
		fprintf( stderr, "cascade index out of range: %d\n", cascade );
		return;
	}
	sg_pass pass = { 0 };
	pass.action.depth.load_action = SG_LOADACTION_CLEAR;
	pass.action.depth.clear_value = 1.0f; // standard Z far
	pass.action.depth.store_action = SG_STOREACTION_STORE;
	pass.attachments.depth_stencil = s_shadow.cascadeAttachmentViews[cascade];
	sg_begin_pass( &pass );
	s_shadow.inPass = true;
}

void EndShadowPass( void )
{
	if ( !s_shadow.inPass )
	{
		return;
	}
	sg_end_pass();
	s_shadow.inPass = false;
}

Mat4 GetCascadeMatrix( int cascade )
{
	if ( cascade < 0 || cascade >= SHADOW_CASCADE_COUNT )
	{
		return MakeIdentity();
	}
	return s_shadow.cascadeMatrices[cascade];
}

float GetCascadeFarViewZ( int cascade )
{
	if ( cascade < 0 || cascade >= SHADOW_CASCADE_COUNT )
	{
		return 0.0f;
	}
	return s_shadow.cascadeFarViewZ[cascade];
}

float GetCascadeTexelWorld( int cascade )
{
	if ( cascade < 0 || cascade >= SHADOW_CASCADE_COUNT )
	{
		return 0.0f;
	}
	return s_shadow.cascadeTexelWorld[cascade];
}

float GetCascadeDepthBias( int cascade )
{
	if ( cascade < 0 || cascade >= SHADOW_CASCADE_COUNT )
	{
		return 0.0f;
	}
	return s_shadow.cascadeDepthBias[cascade];
}

sg_view GetShadowSampleView( void )
{
	return s_shadow.sampleView;
}

sg_sampler GetShadowSampler( void )
{
	return s_shadow.sampler;
}

float GetShadowTexelSize( void )
{
	return 1.0f / (float)SHADOW_RESOLUTION;
}
