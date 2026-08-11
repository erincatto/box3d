// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Cascade count plus the parts of the cascaded lookup that touch no
// uniform block, shared by the lit shape shaders via @include. Each
// shader declares its own ub_pass, so the per-cascade values stay there
// and arrive as parameters.

// Must match SHADOW_CASCADE_COUNT in gfx/shadow.h. The renderer static
// asserts the two agree, so a mismatch fails the build instead of
// rendering garbage. Changing it here only reaches the committed headers
// after a shader regen, which needs BOX3D_BUILD_SHADERS=ON.
//
// Four is the ceiling. Per-cascade values ride in vec4 lanes and a fifth
// cascade would need a second vec4 for each of them.
#define SHADOW_CASCADE_COUNT 4

// Cascade whose slice holds this view depth. The last one is a fallback
// rather than a range: its map footprint runs well past the final split,
// so content past the split range still gets shadowed.
int selectShadowCascade( vec4 far_view_z, float view_z )
{
	int cascade = SHADOW_CASCADE_COUNT - 1;
	for ( int i = 0; i < SHADOW_CASCADE_COUNT - 1; ++i )
	{
		if ( view_z < far_view_z[i] )
		{
			cascade = i;
			break;
		}
	}
	return cascade;
}

// Debug view 2.
vec3 shadowCascadeTint( int cascade )
{
	if ( cascade == 0 )
	{
		return vec3( 1.0, 0.4, 0.4 );
	}
	if ( cascade == 1 )
	{
		return vec3( 0.4, 1.0, 0.4 );
	}
	if ( cascade == 2 )
	{
		return vec3( 0.4, 0.4, 1.0 );
	}
	return vec3( 1.0, 0.9, 0.3 );
}
