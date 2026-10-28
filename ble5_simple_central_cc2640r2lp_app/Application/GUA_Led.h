/*
 * GUA_Led.h
 *
 * Project-local LED / relay helper for the Bluetooth emergency-stop firmware.
 * This replaces the third-party tutorial library originally used by the
 * prototype. The pin mapping is unchanged:
 *
 *   LED1 = DIO7, LED2 = DIO9, LED3 = DIO11, LED4/RELAY = DIO0, LED5 = DIO20
 *
 * The outputs are active-low.
 *
 * Copyright (c) 2020 Pengyu Li
 * SPDX-License-Identifier: MIT
 */

#ifndef _GUA_LED_H_
#define _GUA_LED_H_

#ifndef GUA_U8
typedef unsigned char GUA_U8;
#endif

/* LED bit masks */
#define GUA_LED_NO_1            0x01
#define GUA_LED_NO_2            0x02
#define GUA_LED_NO_3            0x04
#define GUA_LED_NO_4            0x08
#define GUA_LED_NO_5            0x10
#define GUA_LED_NO_ALL          (GUA_LED_NO_1 | GUA_LED_NO_2 | GUA_LED_NO_3 | \
                                 GUA_LED_NO_4 | GUA_LED_NO_5)

/* Output modes */
#define GUA_LED_MODE_OFF        0x00
#define GUA_LED_MODE_ON         0x01
#define GUA_LED_MODE_FLASH      0x02
#define GUA_LED_MODE_TOGGLE     0x04

extern void GUA_Led_Set(GUA_U8 nGUA_Led_No, GUA_U8 nGUA_Mode);

#endif /* _GUA_LED_H_ */
