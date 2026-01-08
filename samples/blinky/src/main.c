
/*
 * Copyright (c) 2024 Circuit Dojo LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

static const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(npm1300_leds));

int main(void)
{
    LOG_INF("Blinky Sample");

    while (1)
    {
        led_on(leds, 2U);
        k_sleep(K_MSEC(500));
        led_off(leds, 2U);
        k_sleep(K_MSEC(500));
    }

    return 0;
}