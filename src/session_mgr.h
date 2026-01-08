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
#include "flight_timer.h"

#define UUID_STRING_LEN 37  /* 36 chars + null terminator */
#define MAX_STATE_CHANGES 200  /* Maximum state changes to store (matches flight_timer buffer) */

/* Forward declaration - defined in flight_timer.h */
struct state_change_entry;

/**
 * @brief Session data structure
 * Stores only: uuid, start/stop lat/long, timestamps, and state changes
 */
struct session_data {
	char uuid[UUID_STRING_LEN];
	int64_t start_time_ms;  /* Session start timestamp */
	int64_t stop_time_ms;   /* Most recent update timestamp */
	double start_lat;
	double start_lon;
	double stop_lat;
	double stop_lon;
	struct state_change_entry state_changes[MAX_STATE_CHANGES];
	uint16_t state_change_count;
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
 * @brief Set session start position (called once at session start)
 * @param lat Start latitude
 * @param lon Start longitude
 * @return true if start position was set (first time), false if already set
 */
bool session_mgr_set_start_position(double lat, double lon);

/**
 * @brief Update session stop position (most recent position)
 * @param lat Current latitude
 * @param lon Current longitude
 */
void session_mgr_update_position(double lat, double lon);

/**
 * @brief Save session to session.json
 * @param state_changes Array of state changes from flight timer
 * @param state_change_count Number of state changes
 * @return 0 on success, negative errno on failure
 */
int session_mgr_save(const struct state_change_entry *state_changes, uint16_t state_change_count);

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

/**
 * @brief Rotate existing session to old-{uuid}.json
 * Marks current session as old without starting a new session
 * @return 0 on success, negative errno on failure
 */
int session_mgr_rotate_session(void);

#endif /* SESSION_MGR_H */

