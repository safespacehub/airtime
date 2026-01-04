/*
 * SUPL A-GNSS Assistance Module
 * Based on Nordic Semiconductor GPS sample
 *
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef SUPL_ASSIST_H
#define SUPL_ASSIST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <nrf_modem_gnss.h>
#include <zephyr/kernel.h>

/**
 * @brief Initializes the SUPL assistance module.
 *
 * @param[in] assistance_work_q Work queue that can be used by the module (unused).
 *
 * @retval 0 on success.
 * @retval -1 in case of an error.
 */
int supl_assist_init(struct k_work_q *assistance_work_q);

/**
 * @brief Handles an A-GNSS data request.
 *
 * @details Fetches and injects A-GNSS data to the GNSS module via SUPL.
 *
 * @param[in] agnss_request A-GNSS data requested by GNSS.
 *
 * @retval 0 on success.
 * @retval <0 in case of an error.
 */
int supl_assist_request(struct nrf_modem_gnss_agnss_data_frame *agnss_request);

/**
 * @brief Returns SUPL assistance module state.
 *
 * @retval true if assistance module is downloading data.
 * @retval false if assistance module is idle.
 */
bool supl_assist_is_active(void);

#ifdef __cplusplus
}
#endif

#endif /* SUPL_ASSIST_H */
