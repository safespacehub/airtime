
/*
 * Copyright (c) 2024 Circuit Dojo LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

/* nRF Libraries */
#include <modem/lte_lc.h>
#include <modem/modem_info.h>
#include <modem/nrf_modem_lib.h>

int main(void)
{
    int err;

    LOG_INF("Battery Voltage Sample");

    /* Init modem library */
    err = nrf_modem_lib_init();
    if (err < 0)
        LOG_ERR("Unable to initialize modem lib. (err: %i)", err);

    /* Set up modem info */
    err = modem_info_init();
    if (err < 0)
        LOG_ERR("Unable to initialize modem info. (err: %i)", err);

    /* Get battery voltage */
    int voltage = 0;
    err = modem_info_get_batt_voltage(&voltage);

    /* Get battery voltage */
    LOG_INF("Battery Voltage: %d mV", voltage);

    return 0;
}