/*
 * Flight Timer Module
 * Coordinates state machines and manages flight timing
 */

#ifndef FLIGHT_TIMER_H
#define FLIGHT_TIMER_H

#include <stdint.h>
#include <stdbool.h>
#include "gps_state.h"
#include "baro_state.h"

/**
 * @brief Combined flight state
 */
typedef enum {
	FLIGHT_STATUS_GROUND = 0,   /* Both GPS and baro indicate ground */
	FLIGHT_STATUS_TAXIING,      /* GPS indicates taxi, baro ground */
	FLIGHT_STATUS_AIRBORNE,     /* Either GPS or baro indicates flight */
} flight_status_t;

/**
 * @brief Flight timer data
 */
struct flight_timer_data {
	flight_status_t status;
	int64_t session_start_ms;
	int64_t flight_start_ms;      /* When airborne started */
	int64_t total_flight_time_ms; /* Accumulated flight time */
	bool currently_airborne;
};

/**
 * @brief Initialize flight timer
 * @return 0 on success, negative errno on failure
 */
int flight_timer_init(void);

/**
 * @brief Start flight timer (new session)
 * @return 0 on success, negative errno on failure
 */
int flight_timer_start(void);

/**
 * @brief Process a GPS PVT update
 * @param pvt Pointer to PVT data frame
 */
void flight_timer_process_gps(const struct nrf_modem_gnss_pvt_data_frame *pvt);

/**
 * @brief Process a barometer sample
 * @param pressure_pa Pressure in Pascals
 * @param temperature_c Temperature in Celsius
 */
void flight_timer_process_baro(float pressure_pa, float temperature_c);

/**
 * @brief Get current flight status
 * @return Current flight status
 */
flight_status_t flight_timer_get_status(void);

/**
 * @brief Get flight timer data
 * @return Pointer to flight timer data
 */
const struct flight_timer_data *flight_timer_get_data(void);

/**
 * @brief Get total flight time in current session
 * @return Flight time in milliseconds
 */
int64_t flight_timer_get_flight_time_ms(void);

/**
 * @brief Get session elapsed time
 * @return Elapsed time in milliseconds
 */
int64_t flight_timer_get_session_time_ms(void);

/**
 * @brief Periodic update task (call from main loop)
 * Saves session state periodically
 */
void flight_timer_periodic_update(void);

/**
 * @brief Check if periodic save is needed
 * @return true if save needed
 */
bool flight_timer_needs_save(void);

/**
 * @brief Force save session state
 * @return 0 on success, negative errno on failure
 */
int flight_timer_save(void);

#endif /* FLIGHT_TIMER_H */

