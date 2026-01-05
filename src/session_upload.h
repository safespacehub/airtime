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

#endif /* SESSION_UPLOAD_H */

