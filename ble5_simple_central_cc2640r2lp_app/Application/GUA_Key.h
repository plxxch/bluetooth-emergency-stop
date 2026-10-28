/*
 * GUA_Key.h
 *
 * Project-local key scanner for the Bluetooth emergency-stop firmware.
 * This replaces the third-party tutorial library originally used by the
 * prototype. The pin mapping is unchanged:
 *
 *   UP = DIO21, DOWN = DIO16, LEFT = DIO17, RIGHT = DIO18, SELECT = DIO30
 *
 * Copyright (c) 2020 Pengyu Li
 * SPDX-License-Identifier: MIT
 */

#ifndef _GUA_KEY_H_
#define _GUA_KEY_H_

#ifndef GUA_U8
typedef unsigned char GUA_U8;
#endif

#define GUA_KEY_UP_VALUE                0x0001
#define GUA_KEY_DOWN_VALUE              0x0002
#define GUA_KEY_LEFT_VALUE              0x0004
#define GUA_KEY_RIGHT_VALUE             0x0008
#define GUA_KEY_SELECT_VALUE            0x0010

#define GUA_KEY_DEBOUNCE_TIMEOUT        20

typedef void (*GUA_keysPressedCB_t)(GUA_U8 GUA_keysPressed);

extern void GUA_initKeys(GUA_keysPressedCB_t GUA_appKeyCB);

#endif /* _GUA_KEY_H_ */
