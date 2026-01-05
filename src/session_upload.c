/*
 * Session Upload Module
 * Handles periodic and state-change triggered uploads of session data over LTE
 */

#include "session_upload.h"
#include "session_mgr.h"
#include "flight_timer.h"
#include "sd_logger.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/http/client.h>
#include <zephyr/net/net_ip.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <hw_id.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(session_upload, CONFIG_LOG_DEFAULT_LEVEL);

/* Configuration */
#define UPLOAD_HOSTNAME "167.172.117.206"
#define UPLOAD_PATH "/ingest"
#define UPLOAD_PORT 80
#define UPLOAD_IOT_KEY "REPLACE_WITH_LONG_RANDOM_SECRET"
#define UPLOAD_TIMEOUT_MS (5000)  /* 5 seconds - balance between speed and reliability */
#define MAX_RETRIES 3
#define RETRY_BASE_DELAY_MS (1000)  /* Base delay: 1 second */
#define RETRY_MAX_DELAY_MS (8000)   /* Maximum delay: 8 seconds */

/* Periodic upload intervals */
#define GROUND_UPLOAD_INTERVAL_MS (60000)   /* 1 minute on ground */
#define AIRBORNE_UPLOAD_INTERVAL_MS (300000) /* 5 minutes in air */

/* State tracking */
static int64_t last_periodic_upload_ms = 0;
static flight_status_t last_upload_status = FLIGHT_STATUS_GROUND;
static bool upload_in_progress = false;

/* Forward declarations */
static int do_upload(bool is_state_change);
static bool is_lte_connected(void);
static int build_upload_json(char *json_buf, size_t json_buf_size, const char *hw_id);
static void http_response_cb(struct http_response *rsp, enum http_final_call final_data, void *user_data);

static bool is_lte_connected(void)
{
	if (!nrf_modem_is_initialized()) {
		return false;
	}

	enum lte_lc_nw_reg_status reg_status;
	int err = lte_lc_nw_reg_status_get(&reg_status);
	if (err != 0) {
		return false;
	}

	return (reg_status == LTE_LC_NW_REG_REGISTERED_HOME ||
		reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING);
}

static int build_upload_json(char *json_buf, size_t json_buf_size, const char *hw_id)
{
	const struct session_data *session = session_mgr_get_data();
	int pos = 0;

	/* Build JSON with hardware ID and session data */
	pos += snprintf(json_buf + pos, json_buf_size - pos,
		"{\"hw_id\":\"%s\"", hw_id);
	pos += snprintf(json_buf + pos, json_buf_size - pos,
		",\"uuid\":\"%s\"", session->uuid);
	pos += snprintf(json_buf + pos, json_buf_size - pos,
		",\"start_time\":%lld", (long long)session->start_time_ms);
	pos += snprintf(json_buf + pos, json_buf_size - pos,
		",\"stop_time\":%lld", (long long)session->stop_time_ms);
	pos += snprintf(json_buf + pos, json_buf_size - pos,
		",\"start\":{\"lat\":%.6f,\"lon\":%.6f}",
		session->start_lat, session->start_lon);
	pos += snprintf(json_buf + pos, json_buf_size - pos,
		",\"stop\":{\"lat\":%.6f,\"lon\":%.6f}",
		session->stop_lat, session->stop_lon);
	
	/* Add state changes array */
	pos += snprintf(json_buf + pos, json_buf_size - pos, ",\"changes\":[");
	for (uint16_t i = 0; i < session->state_change_count && pos < (int)(json_buf_size - 50); i++) {
		if (i > 0) {
			pos += snprintf(json_buf + pos, json_buf_size - pos, ",");
		}
		pos += snprintf(json_buf + pos, json_buf_size - pos, "{\"s\":%d,\"t\":%lld}",
			session->state_changes[i].status,
			(long long)session->state_changes[i].timestamp_ms);
	}
	pos += snprintf(json_buf + pos, json_buf_size - pos, "]}");

	if (pos >= (int)json_buf_size) {
		LOG_WRN("JSON buffer overflow, truncating");
		pos = json_buf_size - 1;
		json_buf[pos] = '\0';
	}

	return pos;
}

static int http_status_code = 0;

static void http_response_cb(struct http_response *rsp, enum http_final_call final_data, void *user_data)
{
	ARG_UNUSED(user_data);

	if (final_data == HTTP_DATA_MORE) {
		/* More data coming, just log status */
		LOG_DBG("HTTP response: status=%d, data_len=%d", rsp->http_status_code, rsp->data_len);
		http_status_code = rsp->http_status_code;
	} else if (final_data == HTTP_DATA_FINAL) {
		/* Final data chunk */
		http_status_code = rsp->http_status_code;
		LOG_INF("HTTP response complete: status=%d, total_len=%d", 
			rsp->http_status_code, rsp->data_len);
		if (rsp->http_status_code >= 200 && rsp->http_status_code < 300) {
			LOG_INF("Upload successful (HTTP %d)", rsp->http_status_code);
		} else {
			LOG_WRN("Upload rejected by server (HTTP %d)", rsp->http_status_code);
		}
	}
}

static int do_upload(bool is_state_change)
{
	int err;
	int fd = -1;
	int retry_count = 0;
	char hw_id[HW_ID_LEN];
	char json_buf[6144];  /* Same size as session_mgr uses */
	uint8_t recv_buf[256];
	struct http_request req;
	/* Build X-IOT-KEY header string */
	char iot_key_header[64];
	snprintf(iot_key_header, sizeof(iot_key_header), "X-IOT-KEY: %s\r\n", UPLOAD_IOT_KEY);
	const char *headers[] = {
		"Content-Type: application/json\r\n",
		"Connection: close\r\n",
		iot_key_header,
		NULL
	};

	/* Prevent concurrent uploads */
	if (upload_in_progress) {
		LOG_DBG("Upload already in progress, skipping");
		return -EINPROGRESS;
	}

	/* Check LTE connection */
	if (!is_lte_connected()) {
		LOG_WRN("LTE not connected, cannot upload");
		return -ENETUNREACH;
	}

	/* Get hardware ID */
	err = hw_id_get(hw_id, sizeof(hw_id));
	if (err != 0) {
		LOG_ERR("Failed to get hardware ID: %d", err);
		return err;
	}

	/* Build JSON payload */
	int json_len = build_upload_json(json_buf, sizeof(json_buf), hw_id);
	if (json_len <= 0) {
		LOG_ERR("Failed to build JSON payload");
		return -EINVAL;
	}

	LOG_INF("Uploading session data (state_change=%d, size=%d bytes) to %s%s",
		is_state_change, json_len, UPLOAD_HOSTNAME, UPLOAD_PATH);

	/* Retry loop with exponential backoff */
	upload_in_progress = true;
	for (retry_count = 0; retry_count <= MAX_RETRIES; retry_count++) {
		if (retry_count > 0) {
			/* Calculate exponential backoff delay: base * 2^(retry_count-1) */
			int32_t backoff_delay_ms = RETRY_BASE_DELAY_MS;
			for (int i = 0; i < retry_count - 1; i++) {
				backoff_delay_ms *= 2;
				/* Cap at maximum delay */
				if (backoff_delay_ms > RETRY_MAX_DELAY_MS) {
					backoff_delay_ms = RETRY_MAX_DELAY_MS;
					break;
				}
			}
			LOG_INF("Retry %d/%d (backoff: %d ms)", retry_count, MAX_RETRIES, backoff_delay_ms);
			k_sleep(K_MSEC(backoff_delay_ms));
		}

		/* Create socket */
		fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (fd < 0) {
			LOG_ERR("Failed to create socket: %d", errno);
			err = -errno;
			continue;
		}

		/* Resolve hostname */
		struct addrinfo hints = {
			.ai_family = AF_INET,
			.ai_socktype = SOCK_STREAM,
			.ai_protocol = IPPROTO_TCP,
		};
		struct addrinfo *result = NULL;
		err = getaddrinfo(UPLOAD_HOSTNAME, NULL, &hints, &result);
		if (err != 0 || result == NULL) {
			LOG_ERR("Failed to resolve hostname: %d", err);
			close(fd);
			fd = -1;
			err = -ENOENT;
			continue;
		}

		/* Connect to server */
		struct sockaddr_in *server_addr = (struct sockaddr_in *)result->ai_addr;
		server_addr->sin_port = htons(UPLOAD_PORT);
		err = connect(fd, (struct sockaddr *)server_addr, sizeof(*server_addr));
		freeaddrinfo(result);
		if (err != 0) {
			LOG_ERR("Failed to connect: %d", errno);
			close(fd);
			fd = -1;
			err = -errno;
			continue;
		}

		/* Setup HTTP request */
		memset(&req, 0, sizeof(req));
		req.method = HTTP_POST;
		req.url = UPLOAD_PATH;
		req.host = UPLOAD_HOSTNAME;
		req.protocol = "HTTP/1.1";
		req.payload = json_buf;
		req.payload_len = json_len;
		req.response = http_response_cb;
		req.recv_buf = recv_buf;
		req.recv_buf_len = sizeof(recv_buf);
		req.content_type_value = "application/json";
		req.header_fields = headers;

		/* Reset status code before request */
		http_status_code = 0;

		/* Send HTTP request */
		err = http_client_req(fd, &req, UPLOAD_TIMEOUT_MS, NULL);
		if (err >= 0) {
			/* http_client_req returns bytes received on success */
			/* Check if we got a successful HTTP status code */
			if (http_status_code >= 200 && http_status_code < 300) {
				LOG_INF("Upload successful (HTTP %d, %d bytes received)", 
					http_status_code, err);
				close(fd);
				upload_in_progress = false;
				return 0;
			} else if (http_status_code > 0) {
				LOG_WRN("Upload rejected by server (HTTP %d)", http_status_code);
				err = -EBADMSG;  /* Bad message - server rejected */
			} else {
				LOG_WRN("Upload failed: no HTTP status code received (ret=%d)", err);
			}
		} else {
			LOG_WRN("Upload failed: network error %d", err);
		}
		close(fd);
		fd = -1;
	}

	upload_in_progress = false;
	LOG_ERR("Upload failed after %d retries", MAX_RETRIES + 1);
	return err;
}

int session_upload_init(void)
{
	last_periodic_upload_ms = k_uptime_get();
	last_upload_status = FLIGHT_STATUS_GROUND;
	upload_in_progress = false;

	LOG_INF("Session upload module initialized");
	return 0;
}

int session_upload_trigger(bool is_state_change)
{
	return do_upload(is_state_change);
}

void session_upload_periodic_update(void)
{
	int64_t now = k_uptime_get();
	flight_status_t current_status = flight_timer_get_status();
	int64_t interval_ms;

	/* Determine upload interval based on current state */
	if (current_status == FLIGHT_STATUS_AIRBORNE) {
		interval_ms = AIRBORNE_UPLOAD_INTERVAL_MS;
	} else {
		interval_ms = GROUND_UPLOAD_INTERVAL_MS;
	}

	/* Check if it's time for a periodic upload */
	if (now - last_periodic_upload_ms >= interval_ms) {
		/* Only upload if state hasn't changed (state changes trigger immediate uploads) */
		if (current_status == last_upload_status) {
			LOG_DBG("Periodic upload triggered (status=%s, interval=%lld ms)",
				current_status == FLIGHT_STATUS_GROUND ? "ground" : "airborne",
				interval_ms);
			session_upload_trigger(false);
		}
		last_periodic_upload_ms = now;
		last_upload_status = current_status;
	} else if (current_status != last_upload_status) {
		/* State changed - reset periodic timer */
		last_periodic_upload_ms = now;
		last_upload_status = current_status;
	}
}

bool session_upload_is_lte_ready(void)
{
	return is_lte_connected();
}

