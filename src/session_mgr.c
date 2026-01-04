/*
 * Session Manager Module
 * UUID generation, state history tracking, and file rotation
 */

#include "session_mgr.h"
#include "sd_logger.h"
#include "rtc_sync.h"

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(session_mgr, CONFIG_LOG_DEFAULT_LEVEL);

#define ACTIVE_JSON_FILE    "active.json"

static struct session_data session;
static bool session_active = false;
static int64_t session_uptime_start = 0;  /* Uptime at session start */

/* Static buffer for JSON serialization */
static char session_json_buf[1024];

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

static int parse_uuid_from_json(const char *json, char *uuid_out)
{
	/* Simple JSON parser - find "uuid": "..." (handles whitespace) */
	const char *uuid_key = "\"uuid\"";
	const char *ptr = strstr(json, uuid_key);
	if (ptr == NULL) {
		return -EINVAL;
	}
	
	/* Skip past "uuid" */
	ptr += strlen(uuid_key);
	
	/* Skip whitespace and colon */
	while (*ptr == ' ' || *ptr == '\t' || *ptr == '\n' || *ptr == '\r') {
		ptr++;
	}
	if (*ptr != ':') {
		return -EINVAL;
	}
	ptr++; /* Skip colon */
	
	/* Skip whitespace after colon */
	while (*ptr == ' ' || *ptr == '\t' || *ptr == '\n' || *ptr == '\r') {
		ptr++;
	}
	
	/* Expect opening quote */
	if (*ptr != '"') {
		return -EINVAL;
	}
	ptr++; /* Skip opening quote */
	
	/* Copy until closing quote */
	int i = 0;
	while (*ptr != '"' && *ptr != '\0' && i < UUID_STRING_LEN - 1) {
		uuid_out[i++] = *ptr++;
	}
	if (*ptr != '"') {
		return -EINVAL; /* Didn't find closing quote */
	}
	uuid_out[i] = '\0';
	
	return 0;
}

static int rotate_existing_session(void)
{
	if (!sd_logger_file_exists(ACTIVE_JSON_FILE)) {
		LOG_INF("No existing session to rotate");
		return 0;
	}

	/* Read existing active.json to get UUID */
	char json_buf[256];
	int bytes_read = sd_logger_read_file(ACTIVE_JSON_FILE, json_buf, sizeof(json_buf) - 1);
	if (bytes_read <= 0) {
		LOG_WRN("Failed to read existing active.json, deleting");
		/* Can't read, just continue with new session */
		return 0;
	}
	json_buf[bytes_read] = '\0';

	/* Parse UUID from JSON */
	char old_uuid[UUID_STRING_LEN];
	int ret = parse_uuid_from_json(json_buf, old_uuid);
	if (ret != 0) {
		LOG_WRN("Failed to parse UUID from active.json");
		return 0;
	}

	LOG_INF("Rotating previous session: %s", old_uuid);

	/* Rename files with UUID prefix */
	char new_name[64];
	
	snprintf(new_name, sizeof(new_name), "%s.json", old_uuid);
	sd_logger_rename(ACTIVE_JSON_FILE, new_name);

	/* CSV data is now in log files via LOG_BACKEND_FS - no separate file to rename */

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

	/* Save initial session state */
	LOG_INF("Creating active.json...");
	ret = session_mgr_save();
	if (ret != 0) {
		LOG_ERR("Failed to save initial session: %d", ret);
		return ret;
	}

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
	if (!sd_logger_is_mounted()) {
		return -ENODEV;
	}

	/* Ensure UUID is initialized */
	if (session.uuid[0] == '\0') {
		LOG_ERR("Session UUID not initialized");
		return -EINVAL;
	}

	/* Get state names safely */
	const char *gps_state_str = gps_state_name(session.gps_state);
	const char *baro_state_str = baro_state_name(session.baro_state);
	
	if (gps_state_str == NULL) {
		gps_state_str = "unknown";
	}
	if (baro_state_str == NULL) {
		baro_state_str = "unknown";
	}
	
	/* Convert floats to integers to avoid floating-point printf issues */
	/* Use safe defaults - GPS values start at 0.0 and are only set when valid */
	int32_t lat_int = 0;
	int32_t lon_int = 0;
	int32_t alt_int = 0;
	
	/* Only convert if we have valid GPS data (non-zero coordinates) */
	/* Note: 0,0 is technically valid (Gulf of Guinea) but unlikely for flight tracking */
	if (session.last_lat != 0.0 && session.last_lon != 0.0) {
		/* Clamp to reasonable bounds to avoid overflow/underflow */
		if (session.last_lat >= -90.0 && session.last_lat <= 90.0 &&
		    session.last_lon >= -180.0 && session.last_lon <= 180.0) {
			lat_int = (int32_t)(session.last_lat * 1000000);
			lon_int = (int32_t)(session.last_lon * 1000000);
		}
	}
	
	/* Only convert altitude if non-zero and reasonable */
	if (session.last_altitude_ft != 0.0f &&
	    session.last_altitude_ft >= -1000.0f && session.last_altitude_ft <= 100000.0f) {
		alt_int = (int32_t)(session.last_altitude_ft * 10);
	}
	
	/* Convert 64-bit integers to 32-bit for printf (timestamps should fit) */
	uint32_t start_ms = (uint32_t)session.start_time_ms;
	uint32_t elapsed_ms = (uint32_t)session.elapsed_ms;
	
	/* Build JSON with pretty formatting, using 32-bit integers only */
	int len = snprintf(session_json_buf, sizeof(session_json_buf),
		"{\n"
		"  \"uuid\": \"%s\",\n"
		"  \"start_time_ms\": %u,\n"
		"  \"elapsed_ms\": %u,\n"
		"  \"gps_state\": \"%s\",\n"
		"  \"baro_state\": \"%s\",\n"
		"  \"last_lat\": %.6f,\n"
		"  \"last_lon\": %.6f,\n"
		"  \"last_altitude_ft\": %.1f,\n"
		"  \"samples_logged\": %u\n"
		"}\n",
		session.uuid,
		start_ms,
		elapsed_ms,
		gps_state_str,
		baro_state_str,
		(double)lat_int / 1000000.0,
		(double)lon_int / 1000000.0,
		(double)alt_int / 10.0,
		session.samples_logged);

	if (len <= 0 || len >= (int)sizeof(session_json_buf)) {
		LOG_ERR("JSON serialization failed: len=%d", len);
		return -ENOMEM;
	}

	session_json_buf[len] = '\0';

	LOG_INF("Writing active.json (%d bytes)...", len);
	int ret = sd_logger_write_file(ACTIVE_JSON_FILE, session_json_buf, len);
	if (ret != 0) {
		LOG_ERR("Failed to write session file: %d", ret);
		return ret;
	}
	LOG_INF("active.json written successfully");

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

