/*
 * Session Manager Module
 * UUID generation, state history tracking, and file rotation
 */

#include "session_mgr.h"
#include "rtc_sync.h"

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

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
	/* Session rotation disabled - logging backend handles all file management */
	LOG_INF("Session rotation skipped (logging backend handles files)");
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
	session.start_time_ms = rtc_get_timestamp_ms();  /* Wall clock for JSON */
	session.elapsed_ms = 0;
	session.gps_state = GPS_STATE_UNKNOWN;
	session.baro_state = BARO_STATE_GROUND;
	session.last_lat = 0.0;
	session.last_lon = 0.0;
	session.last_altitude_ft = 0.0f;
	session.samples_logged = 0;

	/* CSV logging now handled via LOG_BACKEND_FS - no file to open */
	/* Session data is tracked in memory only - no SD card writes */

	session_active = true;
	LOG_INF("New session started: %s", session.uuid);

	return 0;
}


void session_mgr_update(gps_state_t gps_state,
			baro_state_t baro_state,
			double lat,
			double lon,
			float altitude_ft,
			uint32_t samples)
{
	if (!session_active) {
		return;
	}
	
	session.elapsed_ms = k_uptime_get() - session_uptime_start;
	session.gps_state = gps_state;
	session.baro_state = baro_state;
	session.last_lat = lat;
	session.last_lon = lon;
	session.last_altitude_ft = altitude_ft;
	session.samples_logged = samples;
}

int session_mgr_save(void)
{
	/* Session data is tracked in memory only - no SD card writes */
	/* All data is logged via LOG_BACKEND_FS in the CSV and STATUS messages */
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

