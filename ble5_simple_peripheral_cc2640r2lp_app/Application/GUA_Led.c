/*
 * GUA_Led.c
 *
 * Project-local LED / relay helper for the Bluetooth emergency-stop firmware.
 *
 * Pin mapping (active-low):
 *   LED1 = DIO7, LED2 = DIO9, LED3 = DIO11, LED4/RELAY = DIO0, LED5 = DIO20
 *
 * Copyright (c) 2020 Pengyu Li
 * SPDX-License-Identifier: MIT
 */

#include <ti/drivers/PIN.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include "GUA_Led.h"

#define GUA_LED1     PIN_ID(7)
#define GUA_LED2     PIN_ID(9)
#define GUA_LED3     PIN_ID(11)
#define GUA_LED4     PIN_ID(0)     /* relay output */
#define GUA_LED5     PIN_ID(20)

static PIN_State  sGUA_LedPins;
static PIN_Handle sGUA_HledPins = NULL;

static PIN_Config bGUA_LedPinsCfg[] =
{
    GUA_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    GUA_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    GUA_LED3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    GUA_LED4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    GUA_LED5 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

void GUA_Led_Set(GUA_U8 nGUA_Led_No, GUA_U8 nGUA_Mode)
{
    static const PIN_Id pins[5] = { GUA_LED1, GUA_LED2, GUA_LED3, GUA_LED4, GUA_LED5 };
    GUA_U8 i;

    if (sGUA_HledPins == NULL)
    {
        sGUA_HledPins = PIN_open(&sGUA_LedPins, bGUA_LedPinsCfg);
    }

    for (i = 0; i < 5; i++)
    {
        if (nGUA_Led_No & (1 << i))
        {
            switch (nGUA_Mode)
            {
                case GUA_LED_MODE_OFF:
                    PIN_setOutputValue(sGUA_HledPins, pins[i], 1);
                    break;

                case GUA_LED_MODE_ON:
                    PIN_setOutputValue(sGUA_HledPins, pins[i], 0);
                    break;

                case GUA_LED_MODE_FLASH:
                    PIN_setOutputValue(sGUA_HledPins, pins[i], 0);
                    Task_sleep(10 * 1000 / Clock_tickPeriod);
                    PIN_setOutputValue(sGUA_HledPins, pins[i], 1);
                    break;

                case GUA_LED_MODE_TOGGLE:
                    PIN_setOutputValue(sGUA_HledPins, pins[i],
                                       !PIN_getOutputValue(pins[i]));
                    break;

                default:
                    break;
            }
        }
    }
}
