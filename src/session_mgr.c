/*
 * Session Manager Module
 * UUID generation, state history tracking, and file rotation
 */

#include "session_mgr.h"
#include "rtc_sync.h"
#include "sd_logger.h"
#include "flight_timer.h"
#include "gps_state.h"
#include <hw_id.h>

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

LOG_MODULE_REGISTER(session_mgr, CONFIG_LOG_DEFAULT_LEVEL);

static struct session_data session;
static bool session_active = false;
static int64_t session_uptime_start = 0;  /* Uptime at session start */

void session_mgr_generate_uuid(char *buf)
{
	uint8_t uuid[16];
	sys_rand_get(uuid, sizeof(uuid));
	
	/* Set version 4 (random) */
	uuid[6] = (uuid[6] & 0x0f) | 0x40;
	/* Set variant (RFC 4122) */
	uuid[8] = (uuid[8] & 0x3f) | 0x80;
	
	snprintf(buf, UUID_STRING_LEN,
		 "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
		 uuid[0], uuid[1], uuid[2], uuid[3],
		 uuid[4], uuid[5],
		 uuid[6], uuid[7],
		 uuid[8], uuid[9],
		 uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15]);
}

static int rotate_existing_session(void)
{
	char old_uuid[UUID_STRING_LEN] = {0};
	char new_filename[64];
	static char json_buffer[6144];  /* Static to avoid stack overflow */
	char hw_id[HW_ID_LEN];
	int ret;
	
	/* Check if session.json exists */
	if (!sd_logger_file_exists("session.json")) {
		/* No existing session to rotate */
		return 0;
	}
	
	/* Read the full session file */
	ret = sd_logger_read_file("session.json", json_buffer, sizeof(json_buffer) - 1);
	if (ret <= 0) {
		LOG_WRN("Failed to read session file for rotation: %d", ret);
		/* Still try to rotate with fallback name */
	} else {
		json_buffer[ret] = '\0';
		
		/* Parse UUID from JSON: {"uuid":"...",...} */
		const char *uuid_start = strstr(json_buffer, "\"uuid\":\"");
		if (uuid_start != NULL) {
			uuid_start += 8; /* Skip past "uuid":" */
			const char *uuid_end = strchr(uuid_start, '"');
			if (uuid_end != NULL && (uuid_end - uuid_start) < UUID_STRING_LEN) {
				size_t uuid_len = uuid_end - uuid_start;
				memcpy(old_uuid, uuid_start, uuid_len);
				old_uuid[uuid_len] = '\0';
			}
		}
		
		/* Get hardware ID */
		memset(hw_id, 0, sizeof(hw_id));
		if (hw_id_get(hw_id, sizeof(hw_id)) == 0 && hw_id[0] != '\0') {
			static char json_temp[6144];  /* Static buffer for transformations */
			
			/* Add hw_id if not present */
			if (strstr(json_buffer, "\"hw_id\"") == NULL) {
				const char *first_brace = strchr(json_buffer, '{');
				if (first_brace != NULL) {
					int pos = 0;
					pos += snprintf(json_temp + pos, sizeof(json_temp) - pos, "{\"hw_id\":\"%s\"", hw_id);
					pos += snprintf(json_temp + pos, sizeof(json_temp) - pos, ",%s", first_brace + 1);
					memcpy(json_buffer, json_temp, sizeof(json_buffer) - 1);
					json_buffer[sizeof(json_buffer) - 1] = '\0';
					ret = strlen(json_buffer);
				}
			}
			
			/* Add "closed": true if not present */
			if (strstr(json_buffer, "\"closed\"") == NULL) {
				char *closing_brace = strrchr(json_buffer, '}');
				if (closing_brace != NULL) {
					size_t insert_pos = closing_brace - json_buffer;
					memcpy(json_temp, json_buffer, insert_pos);
					json_temp[insert_pos] = '\0';
					
					/* Add comma if needed */
					char *before = json_temp + insert_pos - 1;
					while (before > json_temp && (*before == ' ' || *before == '\t' || *before == '\n' || *before == '\r')) {
						before--;
					}
					if (before > json_temp && *before != '{' && *before != '[') {
						strcat(json_temp, ", ");
					} else if (before > json_temp) {
						strcat(json_temp, " ");
					}
					
					strcat(json_temp, "\"closed\": true}");
					memcpy(json_buffer, json_temp, sizeof(json_buffer) - 1);
					json_buffer[sizeof(json_buffer) - 1] = '\0';
					ret = strlen(json_buffer);
				}
			}
			
			/* Write updated JSON back to file before renaming */
			sd_logger_write_file("session.json", json_buffer, ret);
		}
	}
	
	/* Generate new filename: either old-{uuid}.json or timestamp-based */
	if (old_uuid[0] != '\0') {
		/* Use UUID from existing file with "old-" prefix */
		snprintf(new_filename, sizeof(new_filename), "old-%s.json", old_uuid);
	} else {
		/* Fallback: use timestamp */
		int64_t timestamp_ms = rtc_get_timestamp_ms();
		snprintf(new_filename, sizeof(new_filename), "old-session_%" PRId64 ".json", timestamp_ms);
	}
	
	/* Rename session.json to the new filename */
	ret = sd_logger_rename("session.json", new_filename);
	if (ret != 0) {
		LOG_ERR("Failed to rotate session file: %d", ret);
		return ret;
	}
	
	LOG_INF("Session rotated: session.json -> %s", new_filename);
	return 0;
}

int session_mgr_init(void)
{
	memset(&session, 0, sizeof(session));
	session_active = false;
	
	LOG_INF("Session manager initialized");
	return 0;
}

int session_mgr_start_session(void)
{
	int ret;

	/* Rotate any existing session */
	ret = rotate_existing_session();
	if (ret != 0) {
		LOG_WRN("Session rotation failed: %d", ret);
	}

	/* Generate new UUID */
	session_mgr_generate_uuid(session.uuid);
	
	/* Initialize session data */
	session_uptime_start = k_uptime_get();
	session.start_time_ms = rtc_get_timestamp_ms();
	session.stop_time_ms = session.start_time_ms;
	session.start_lat = 0.0;
	session.start_lon = 0.0;
	session.stop_lat = 0.0;
	session.stop_lon = 0.0;
	session.state_change_count = 0;
	memset(session.state_changes, 0, sizeof(session.state_changes));

	session_active = true;
	LOG_INF("New session started: %s", session.uuid);

	/* Write initial session file immediately */
	struct state_change_entry empty_changes[1] = {0};
	ret = session_mgr_save(empty_changes, 0);
	if (ret != 0) {
		LOG_WRN("Failed to write initial session file: %d", ret);
		/* Don't fail session start if initial save fails */
	}

	return 0;
}


bool session_mgr_set_start_position(double lat, double lon)
{
	if (!session_active) {
		return false;
	}
	
	/* Only set start position if not already set (first valid GPS fix) */
	if (session.start_lat == 0.0 && session.start_lon == 0.0 && lat != 0.0 && lon != 0.0) {
		session.start_lat = lat;
		session.start_lon = lon;
		LOG_INF("Session start position: %.6f, %.6f", lat, lon);
		return true;  /* Start position was just set */
	}
	
	return false;  /* Start position already set or invalid */
}

void session_mgr_update_position(double lat, double lon)
{
	if (!session_active) {
		return;
	}
	
	/* Update stop position (most recent position) */
	if (lat != 0.0 && lon != 0.0) {
		session.stop_lat = lat;
		session.stop_lon = lon;
	}
}

int session_mgr_save(const struct state_change_entry *state_changes, uint16_t state_change_count)
{
	if (!session_active) {
		LOG_ERR("session_mgr_save: session not active");
		return -EINVAL;
	}

	/* Update stop timestamp */
	session.stop_time_ms = rtc_get_timestamp_ms();

	/* Copy state changes to session data */
	uint16_t copy_count = state_change_count;
	if (copy_count > MAX_STATE_CHANGES) {
		LOG_WRN("Truncating from %u to %u state changes", copy_count, MAX_STATE_CHANGES);
		copy_count = MAX_STATE_CHANGES;
	}
	memcpy(session.state_changes, state_changes, copy_count * sizeof(struct state_change_entry));
	session.state_change_count = copy_count;

	/* Build compact JSON string */
	/* Format: {"uuid":"...","start_time":...,"stop_time":...,"start":{"lat":...,"lon":...},"stop":{"lat":...,"lon":...},"changes":[...]} */
	/* Estimate: UUID(40) + timestamps(40) + coords(60) + 200 changes(~4000) = ~4140 bytes, use 6KB for safety */
	char json[6144];  /* Size limit: handles up to 200 state changes */
	int pos = 0;
	
	pos += snprintf(json + pos, sizeof(json) - pos, "{\"uuid\":\"%s\"", session.uuid);
	/* Use explicit format for int64_t - newlib supports %lld */
	pos += snprintf(json + pos, sizeof(json) - pos, ",\"start_time\":%lld", (long long)session.start_time_ms);
	pos += snprintf(json + pos, sizeof(json) - pos, ",\"stop_time\":%lld", (long long)session.stop_time_ms);
	pos += snprintf(json + pos, sizeof(json) - pos, ",\"start\":{\"lat\":%.6f,\"lon\":%.6f}",
		session.start_lat, session.start_lon);
	pos += snprintf(json + pos, sizeof(json) - pos, ",\"stop\":{\"lat\":%.6f,\"lon\":%.6f}",
		session.stop_lat, session.stop_lon);
	
	/* Add state changes array */
	pos += snprintf(json + pos, sizeof(json) - pos, ",\"changes\":[");
	for (uint16_t i = 0; i < session.state_change_count && pos < (int)(sizeof(json) - 50); i++) {
		if (i > 0) {
			pos += snprintf(json + pos, sizeof(json) - pos, ",");
		}
		pos += snprintf(json + pos, sizeof(json) - pos, "{\"s\":%d,\"t\":%lld}",
			session.state_changes[i].status,
			(long long)session.state_changes[i].timestamp_ms);
	}
	pos += snprintf(json + pos, sizeof(json) - pos, "]}");
	
	if (pos >= (int)sizeof(json)) {
		LOG_WRN("JSON buffer overflow, truncating");
		pos = sizeof(json) - 1;
		json[pos] = '\0';
	}

	/* Write to SD card */
	LOG_INF("Beginning session.json update");
	int ret = sd_logger_write_file("session.json", json, pos);
	if (ret != 0) {
		LOG_ERR("Failed to write session file: %d", ret);
		return ret;
	}

	/* Check if we've reached the maximum state changes - rotate session if so */
	if (session.state_change_count >= MAX_STATE_CHANGES) {
		LOG_INF("Session reached maximum state changes (%u) - rotating to new session", MAX_STATE_CHANGES);
		
		/* Rotate the current session (saves it as old-{uuid}.json) */
		ret = rotate_existing_session();
		if (ret != 0) {
			LOG_ERR("Failed to rotate session at max state changes: %d", ret);
			/* Continue anyway - try to start new session */
		}
		
		/* Start a new session */
		ret = session_mgr_start_session();
		if (ret != 0) {
			LOG_ERR("Failed to start new session after rotation: %d", ret);
			return ret;
		}
		
		/* Restart flight timer for the new session */
		ret = flight_timer_start();
		if (ret != 0) {
			LOG_ERR("Failed to restart flight timer after session rotation: %d", ret);
			/* Continue anyway - session is started */
		}
		
		LOG_INF("New session started after reaching max state changes: %s", session.uuid);
	}

	return 0;
}

const char *session_mgr_get_uuid(void)
{
	return session.uuid;
}

const struct session_data *session_mgr_get_data(void)
{
	return &session;
}

int64_t session_mgr_get_elapsed_ms(void)
{
	if (!session_active) {
		return 0;
	}
	return k_uptime_get() - session_uptime_start;
}

int session_mgr_rotate_session(void)
{
	/* First, save the current session state to ensure it's up to date */
	/* Get current state changes from flight timer */
	struct state_change_entry state_changes[MAX_STATE_CHANGES];
	uint16_t count = flight_timer_get_status_changes(state_changes, MAX_STATE_CHANGES);
	
	if (count > 0) {
		/* Update stop position with current GPS position */
		const struct gps_state_data *gps_data = gps_state_get_data();
		if (gps_data->latitude != 0.0 && gps_data->longitude != 0.0) {
			session_mgr_update_position(gps_data->latitude, gps_data->longitude);
		}
		
		int err = session_mgr_save(state_changes, count);
		if (err != 0) {
			LOG_WRN("Failed to save session before rotation: %d", err);
			/* Continue with rotation anyway */
		}
	}
	
	/* Rotate the session */
	return rotate_existing_session();
}

