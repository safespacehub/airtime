/*
 * Barometer State Machine
 * Tracks flight state based on pressure altitude changes
 */

#ifndef BARO_STATE_H
#define BARO_STATE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Barometer-based flight states
 */
typedef enum {
	BARO_STATE_GROUND = 0,   /* On ground, stable altitude */
	BARO_STATE_CLIMBING,     /* Ascending > 200 fpm */
	BARO_STATE_CRUISING,     /* Level flight */
	BARO_STATE_DESCENDING,   /* Descending > 200 fpm */
} baro_state_t;

/**
 * @brief Barometer state data structure
 */
struct baro_state_data {
	baro_state_t state;
	baro_state_t prev_state;
	int64_t state_enter_time_ms;
	int64_t cruise_start_time_ms;  /* Track cruise duration for guards */
	float pressure_pa;
	float temperature_c;
	float altitude_ft;
	float altitude_std_dev_ft;
	float vertical_rate_fpm;
	bool in_flight;  /* Combined flight indicator */
};

/**
 * @brief Initialize barometer state machine
 * @return 0 on success, negative errno on failure
 */
int baro_state_init(void);

/**
 * @brief Add a new pressure sample and update state
 * @param pressure_pa Pressure in Pascals
 * @param temperature_c Temperature in Celsius
 * @return true if state changed, false otherwise
 */
bool baro_state_update(float pressure_pa, float temperature_c);

/**
 * @brief Get current barometer state
 * @return Current state
 */
baro_state_t baro_state_get(void);

/**
 * @brief Get barometer state data
 * @return Pointer to state data (read-only)
 */
const struct baro_state_data *baro_state_get_data(void);

/**
 * @brief Get state name string
 * @param state State enum value
 * @return State name string
 */
const char *baro_state_name(baro_state_t state);

/**
 * @brief Get single-char state code for logging
 * @param state State enum value
 * @return Single character code
 */
char baro_state_code(baro_state_t state);

/**
 * @brief Check if barometer indicates in-flight
 * @return true if in flight, false if on ground
 */
bool baro_state_is_in_flight(void);

/**
 * @brief Convert pressure to altitude
 * @param pressure_pa Pressure in Pascals
 * @return Altitude in feet (standard atmosphere)
 */
float baro_pressure_to_altitude_ft(float pressure_pa);

#endif /* BARO_STATE_H */

