/*
 * Session Manager Module
 * UUID generation, state history tracking, and file rotation
 */

#ifndef SESSION_MGR_H
#define SESSION_MGR_H

#include <stdint.h>
#include <stdbool.h>
#include "gps_state.h"
#include "baro_state.h"

#define UUID_STRING_LEN 37  /* 36 chars + null terminator */

/**
 * @brief Session data structure
 */
struct session_data {
	char uuid[UUID_STRING_LEN];
	int64_t start_time_ms;
	int64_t elapsed_ms;
	gps_state_t gps_state;
	baro_state_t baro_state;
	double last_lat;
	double last_lon;
	float last_altitude_ft;
	uint32_t samples_logged;
};

/**
 * @brief Initialize session manager
 * @return 0 on success, negative errno on failure
 */
int session_mgr_init(void);

/**
 * @brief Start a new session
 * Rotates any existing active session files
 * @return 0 on success, negative errno on failure
 */
int session_mgr_start_session(void);

/**
 * @brief Update session with current data
 * @param gps_state Current GPS state
 * @param baro_state Current barometer state
 * @param lat Current latitude
 * @param lon Current longitude
 * @param altitude_ft Current altitude
 * @param samples Number of samples logged
 */
void session_mgr_update(gps_state_t gps_state,
			baro_state_t baro_state,
			double lat,
			double lon,
			float altitude_ft,
			uint32_t samples);

/**
 * @brief Save session to active.json
 * @return 0 on success, negative errno on failure
 */
int session_mgr_save(void);

/**
 * @brief Get current session UUID
 * @return Pointer to UUID string
 */
const char *session_mgr_get_uuid(void);

/**
 * @brief Get session data
 * @return Pointer to session data
 */
const struct session_data *session_mgr_get_data(void);

/**
 * @brief Get session elapsed time in milliseconds
 * @return Elapsed time
 */
int64_t session_mgr_get_elapsed_ms(void);

/**
 * @brief Generate a new UUID v4
 * @param buf Buffer to store UUID string (must be at least UUID_STRING_LEN)
 */
void session_mgr_generate_uuid(char *buf);

#endif /* SESSION_MGR_H */

