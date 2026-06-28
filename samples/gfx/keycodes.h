// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// These mappings exist to make porting samples from Box2D easier.

#pragma once

#include "sokol_app.h"

#include <assert.h>

enum
{
	KEY_A = SAPP_KEYCODE_A,
	KEY_B = SAPP_KEYCODE_B,
	KEY_D = SAPP_KEYCODE_D,
	KEY_F = SAPP_KEYCODE_F,
	KEY_L = SAPP_KEYCODE_L,
	KEY_M = SAPP_KEYCODE_M,
	KEY_O = SAPP_KEYCODE_O,
	KEY_P = SAPP_KEYCODE_P,
	KEY_Q = SAPP_KEYCODE_Q,
	KEY_R = SAPP_KEYCODE_R,
	KEY_S = SAPP_KEYCODE_S,
	KEY_T = SAPP_KEYCODE_T,
	KEY_V = SAPP_KEYCODE_V,
	KEY_W = SAPP_KEYCODE_W,
	KEY_SPACE = SAPP_KEYCODE_SPACE,
	KEY_ESCAPE = SAPP_KEYCODE_ESCAPE,
	KEY_TAB = SAPP_KEYCODE_TAB,
	KEY_LEFT_SHIFT = SAPP_KEYCODE_LEFT_SHIFT,
	KEY_LEFT_BRACKET = SAPP_KEYCODE_LEFT_BRACKET,
	KEY_RIGHT_BRACKET = SAPP_KEYCODE_RIGHT_BRACKET,

	MOD_SHIFT = SAPP_MODIFIER_SHIFT,
	MOD_CTRL = SAPP_MODIFIER_CTRL,
	MOD_ALT = SAPP_MODIFIER_ALT,

	MOUSE_LEFT = SAPP_MOUSEBUTTON_LEFT,
	MOUSE_RIGHT = SAPP_MOUSEBUTTON_RIGHT,
	MOUSE_MIDDLE = SAPP_MOUSEBUTTON_MIDDLE,
};

// Spot-check that the aliases still track sokol_app at compile time. One per
// enum source (keycode, modifier, mouse button) is enough to catch a drift.
// Cast to int so the compare doesn't trip C++'s deprecated cross-enum-type
// comparison warning under -Werror.
static_assert( (int)KEY_W == (int)SAPP_KEYCODE_W, "keycode alias drift" );
static_assert( (int)KEY_ESCAPE == (int)SAPP_KEYCODE_ESCAPE, "keycode alias drift" );
static_assert( (int)MOD_SHIFT == (int)SAPP_MODIFIER_SHIFT, "modifier alias drift" );
static_assert( (int)MOUSE_LEFT == (int)SAPP_MOUSEBUTTON_LEFT, "mouse button alias drift" );
