/*
 * Session Upload Module
 * Handles periodic and state-change triggered uploads of session data over LTE
 */

#ifndef SESSION_UPLOAD_H
#define SESSION_UPLOAD_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize session upload module
 * @return 0 on success, negative errno on failure
 */
int session_upload_init(void);

/**
 * @brief Trigger an upload (called on state changes or periodically)
 * @param is_state_change true if triggered by ground/flight state change
 * @return 0 on success, negative errno on failure
 */
int session_upload_trigger(bool is_state_change);

/**
 * @brief Periodic update (call from main loop)
 * Handles scheduling of periodic uploads based on ground/flight state
 */
void session_upload_periodic_update(void);

/**
 * @brief Check if LTE is connected and ready for uploads
 * @return true if LTE is ready, false otherwise
 */
bool session_upload_is_lte_ready(void);

/**
 * @brief Upload and delete all prior session files (old-*.json)
 * Finds all old-*.json files, uploads them with hw_id, and deletes them after successful upload
 * @return 0 on success, negative errno on failure
 */
int session_upload_prior_sessions(void);

/**
 * @brief Upload both active session and all prior sessions
 * Uploads current active session first, then all prior sessions (old-*.json)
 * @return 0 on success, negative errno on failure
 */
int session_upload_all(void);

/**
 * @brief Check if an upload is currently in progress
 * @return true if upload is in progress, false otherwise
 */
bool session_upload_is_in_progress(void);

#endif /* SESSION_UPLOAD_H */

