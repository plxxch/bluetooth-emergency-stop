/*
 * GUA_Key.c
 *
 * Project-local key scanner for the Bluetooth emergency-stop firmware.
 * Five active-low keys with pull-ups, both-edge interrupts and a 20 ms
 * debounce clock. The application callback receives the pressed-key bitmask.
 *
 * Pin mapping: UP = DIO21, DOWN = DIO16, LEFT = DIO17, RIGHT = DIO18,
 *              SELECT = DIO30
 *
 * Copyright (c) 2020 Pengyu Li
 * SPDX-License-Identifier: MIT
 */

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/sysbios/knl/Clock.h>

#include "util.h"
#include "Board.h"
#include "GUA_Key.h"

#define GUA_KEY_UP          IOID_21
#define GUA_KEY_DOWN        IOID_16
#define GUA_KEY_LEFT        IOID_17
#define GUA_KEY_RIGHT       IOID_18
#define GUA_KEY_SELECT      IOID_30

static GUA_U8 GUA_keysPressed;
static Clock_Struct GUA_keyChangeClock;
static GUA_keysPressedCB_t GUA_appKeyChangeHandler = NULL;

static PIN_State  GUA_keyPins;
static PIN_Handle GUA_hKeyPins;

static PIN_Config GUA_keyPinsCfg[] =
{
    GUA_KEY_UP     | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP,
    GUA_KEY_DOWN   | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP,
    GUA_KEY_LEFT   | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP,
    GUA_KEY_RIGHT  | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP,
    GUA_KEY_SELECT | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP,
    PIN_TERMINATE
};

static GUA_U8 GUA_readKeys(void)
{
    GUA_U8 keys = 0;

    if (PIN_getInputValue(GUA_KEY_UP) == 0)     keys |= GUA_KEY_UP_VALUE;
    if (PIN_getInputValue(GUA_KEY_DOWN) == 0)   keys |= GUA_KEY_DOWN_VALUE;
    if (PIN_getInputValue(GUA_KEY_LEFT) == 0)   keys |= GUA_KEY_LEFT_VALUE;
    if (PIN_getInputValue(GUA_KEY_RIGHT) == 0)  keys |= GUA_KEY_RIGHT_VALUE;
    if (PIN_getInputValue(GUA_KEY_SELECT) == 0) keys |= GUA_KEY_SELECT_VALUE;

    return keys;
}

static void GUA_keyChangeHandler(UArg a0)
{
    if (GUA_appKeyChangeHandler != NULL)
    {
        GUA_keysPressed = GUA_readKeys();
        (*GUA_appKeyChangeHandler)(GUA_keysPressed);
    }
}

static void GUA_keyCallback(PIN_Handle hPin, PIN_Id pinId)
{
    GUA_keysPressed = GUA_readKeys();
    Util_startClock(&GUA_keyChangeClock);
}

void GUA_initKeys(GUA_keysPressedCB_t GUA_appKeyCB)
{
    GUA_hKeyPins = PIN_open(&GUA_keyPins, GUA_keyPinsCfg);

    PIN_registerIntCb(GUA_hKeyPins, GUA_keyCallback);

    PIN_setConfig(GUA_hKeyPins, PIN_BM_IRQ, GUA_KEY_UP     | PIN_IRQ_BOTHEDGES);
    PIN_setConfig(GUA_hKeyPins, PIN_BM_IRQ, GUA_KEY_DOWN   | PIN_IRQ_BOTHEDGES);
    PIN_setConfig(GUA_hKeyPins, PIN_BM_IRQ, GUA_KEY_LEFT   | PIN_IRQ_BOTHEDGES);
    PIN_setConfig(GUA_hKeyPins, PIN_BM_IRQ, GUA_KEY_RIGHT  | PIN_IRQ_BOTHEDGES);
    PIN_setConfig(GUA_hKeyPins, PIN_BM_IRQ, GUA_KEY_SELECT | PIN_IRQ_BOTHEDGES);

#ifdef POWER_SAVING
    PIN_setConfig(GUA_hKeyPins, PINCC26XX_BM_WAKEUP, GUA_KEY_UP     | PINCC26XX_WAKEUP_NEGEDGE);
    PIN_setConfig(GUA_hKeyPins, PINCC26XX_BM_WAKEUP, GUA_KEY_DOWN   | PINCC26XX_WAKEUP_NEGEDGE);
    PIN_setConfig(GUA_hKeyPins, PINCC26XX_BM_WAKEUP, GUA_KEY_LEFT   | PINCC26XX_WAKEUP_NEGEDGE);
    PIN_setConfig(GUA_hKeyPins, PINCC26XX_BM_WAKEUP, GUA_KEY_RIGHT  | PINCC26XX_WAKEUP_NEGEDGE);
    PIN_setConfig(GUA_hKeyPins, PINCC26XX_BM_WAKEUP, GUA_KEY_SELECT | PINCC26XX_WAKEUP_NEGEDGE);
#endif

    Util_constructClock(&GUA_keyChangeClock, GUA_keyChangeHandler,
                        GUA_KEY_DEBOUNCE_TIMEOUT, 0, false, 0);

    GUA_appKeyChangeHandler = GUA_appKeyCB;
}
