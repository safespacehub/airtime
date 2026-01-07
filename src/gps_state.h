/*
 * GPS State Machine
 * Tracks flight state based on GPS speed with hysteresis
 */

#ifndef GPS_STATE_H
#define GPS_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include <nrf_modem_gnss.h>

/* Minimum number of valid fixes required before allowing state transitions */
/* Prevents false state changes from inaccurate initial GPS fixes */
#define GPS_STATE_MIN_FIXES_FOR_STATE_CHANGE 5

/**
 * @brief GPS-based flight states
 */
typedef enum {
	GPS_STATE_UNKNOWN = 0,  /* No valid fix or stale data */
	GPS_STATE_STOPPED,      /* Speed < 2 kts */
	GPS_STATE_TAXI,         /* Speed 2-50 kts */
	GPS_STATE_FLIGHT,       /* Speed > 50 kts */
} gps_state_t;

/**
 * @brief GPS state data structure
 */
struct gps_state_data {
	gps_state_t state;
	gps_state_t prev_state;
	int64_t last_fix_time_ms;
	int64_t state_enter_time_ms;
	double latitude;
	double longitude;
	float altitude_m;
	float speed_mps;
	float speed_kts;
	bool has_fix;
};

/**
 * @brief Initialize GPS state machine
 * @return 0 on success, negative errno on failure
 */
int gps_state_init(void);

/**
 * @brief Update GPS state from PVT data
 * @param pvt Pointer to PVT data frame from modem
 * @return true if state changed, false otherwise
 */
bool gps_state_update(const struct nrf_modem_gnss_pvt_data_frame *pvt);

/**
 * @brief Get current GPS state
 * @return Current state
 */
gps_state_t gps_state_get(void);

/**
 * @brief Get GPS state data
 * @return Pointer to state data (read-only)
 */
const struct gps_state_data *gps_state_get_data(void);

/**
 * @brief Get state name string
 * @param state State enum value
 * @return State name string
 */
const char *gps_state_name(gps_state_t state);

/**
 * @brief Get single-char state code for logging
 * @param state State enum value
 * @return Single character code
 */
char gps_state_code(gps_state_t state);

/**
 * @brief Check if GPS data is stale (>20 seconds old)
 * @return true if stale, false otherwise
 */
bool gps_state_is_stale(void);

/**
 * @brief Get the count of valid fixes received (for debugging)
 * @return Number of valid fixes received since initialization or last reset
 */
uint32_t gps_state_get_fix_count(void);

#endif /* GPS_STATE_H */

