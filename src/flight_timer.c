/*
 * Flight Timer Module
 * Coordinates state machines and manages flight timing
 */

#include "flight_timer.h"
#include "gps_state.h"
#include "baro_state.h"
#include "session_mgr.h"
#include "rtc_sync.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

LOG_MODULE_REGISTER(flight_timer, CONFIG_LOG_DEFAULT_LEVEL);

#define SAVE_INTERVAL_MS     30000  /* Save every 30 seconds */
#define SAMPLE_INTERVAL_MS   100    /* Collect baro samples every 100ms (10 Hz) */
#define LOG_INTERVAL_MS      1000   /* Log CSV every 1 second */
#define STATUS_INTERVAL_MS   10000  /* Status update every 10 seconds */
#define MAX_BARO_SAMPLES     12     /* Max baro samples per second */

static struct flight_timer_data timer_data;
static int64_t last_save_time_ms = 0;
static int64_t last_sample_time_ms = 0;
static int64_t last_log_time_ms = 0;
static int64_t last_status_time_ms = 0;
static gps_state_t last_gps_state = GPS_STATE_UNKNOWN;
static baro_state_t last_baro_state = BARO_STATE_GROUND;

/* Buffer for baro pressure samples (collected at 10Hz, logged once per second) */
static float baro_samples[MAX_BARO_SAMPLES];
static uint8_t baro_sample_count = 0;

/* Sample rate tracking */
static uint32_t total_samples_collected = 0;  /* Total samples in current status period */
static int64_t status_period_start_ms = 0;     /* Start time of current status period */

static void update_flight_status(void)
{
	gps_state_t gps = gps_state_get();
	flight_status_t old_status = timer_data.status;
	flight_status_t new_status;
	int64_t now = k_uptime_get();

	/* Determine combined status */
	bool gps_airborne = (gps == GPS_STATE_FLIGHT);
	bool gps_taxiing = (gps == GPS_STATE_TAXI);
	bool baro_airborne = baro_state_is_in_flight();

	if (gps_airborne || baro_airborne) {
		new_status = FLIGHT_STATUS_AIRBORNE;
	} else if (gps_taxiing) {
		new_status = FLIGHT_STATUS_TAXIING;
	} else {
		new_status = FLIGHT_STATUS_GROUND;
	}

	/* Update airborne tracking */
	if (new_status == FLIGHT_STATUS_AIRBORNE && !timer_data.currently_airborne) {
		/* Just became airborne */
		timer_data.currently_airborne = true;
		timer_data.flight_start_ms = now;
		LOG_INF("Became airborne");
	} else if (new_status != FLIGHT_STATUS_AIRBORNE && timer_data.currently_airborne) {
		/* No longer airborne - accumulate flight time */
		timer_data.currently_airborne = false;
		timer_data.total_flight_time_ms += (now - timer_data.flight_start_ms);
		LOG_INF("No longer airborne, total flight time: %lld ms",
			timer_data.total_flight_time_ms);
	}

	if (new_status != old_status) {
		timer_data.status = new_status;
		LOG_INF("Flight status: %s",
			new_status == FLIGHT_STATUS_GROUND ? "ground" :
			new_status == FLIGHT_STATUS_TAXIING ? "taxiing" : "airborne");
	}
}

static void collect_baro_sample(float pressure_pa)
{
	if (baro_sample_count < MAX_BARO_SAMPLES) {
		baro_samples[baro_sample_count++] = pressure_pa;
		total_samples_collected++;  /* Track for rate calculation */
	}
}

static void log_csv_line(void)
{
	const struct gps_state_data *gps_data = gps_state_get_data();
	const struct baro_state_data *baro_data = baro_state_get_data();
	
	/* Format: baro_state,gps_state,lat,lon,speed,alt,temp,p1,p2,p3... */
	char line[256];
	int pos = 0;
	
	/* Header data: states, GPS position, speed, GPS altitude, temperature */
	pos += snprintf(line + pos, sizeof(line) - pos,
		"CSV,%c,%c,%.6f,%.6f,%.1f,%.1f,%.1f",
		baro_state_code(baro_data->state),
		gps_state_code(gps_data->state),
		gps_data->latitude,
		gps_data->longitude,
		(double)gps_data->speed_kts,
		(double)gps_data->altitude_m,
		(double)baro_data->temperature_c);
	
	/* Append all baro pressure samples from the last second */
	for (uint8_t i = 0; i < baro_sample_count && pos < (int)(sizeof(line) - 20); i++) {
		pos += snprintf(line + pos, sizeof(line) - pos, ",%.1f", (double)baro_samples[i]);
	}
	
	LOG_INF("%s", line);
	
	/* Reset sample buffer for next second */
	baro_sample_count = 0;
}

int flight_timer_init(void)
{
	memset(&timer_data, 0, sizeof(timer_data));
	timer_data.status = FLIGHT_STATUS_GROUND;
	timer_data.currently_airborne = false;
	
	last_save_time_ms = 0;
	last_sample_time_ms = 0;
	last_log_time_ms = 0;
	last_status_time_ms = 0;
	last_gps_state = GPS_STATE_UNKNOWN;
	last_baro_state = BARO_STATE_GROUND;
	baro_sample_count = 0;
	total_samples_collected = 0;
	status_period_start_ms = 0;

	LOG_INF("Flight timer initialized");
	return 0;
}

int flight_timer_start(void)
{
	timer_data.session_start_ms = k_uptime_get();
	timer_data.flight_start_ms = 0;
	timer_data.total_flight_time_ms = 0;
	timer_data.currently_airborne = false;
	timer_data.status = FLIGHT_STATUS_GROUND;
	
	last_save_time_ms = timer_data.session_start_ms;
	last_sample_time_ms = timer_data.session_start_ms;
	last_log_time_ms = timer_data.session_start_ms;
	last_status_time_ms = timer_data.session_start_ms;
	baro_sample_count = 0;
	total_samples_collected = 0;
	status_period_start_ms = timer_data.session_start_ms;

	LOG_INF("Flight timer started");
	return 0;
}

void flight_timer_process_gps(const struct nrf_modem_gnss_pvt_data_frame *pvt)
{
	if (pvt == NULL) {
		return;
	}

	/* Update GPS state machine */
	gps_state_update(pvt);
	gps_state_t new_state = gps_state_get();
	last_gps_state = new_state;

	/* Sync RTC from GPS time if we have a valid fix */
	if (pvt->flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
		rtc_sync_from_gps(&pvt->datetime, pvt->flags);
	}

	/* Update combined flight status */
	update_flight_status();
}

void flight_timer_process_baro(float pressure_pa, float temperature_c)
{
	/* Update barometer state machine */
	baro_state_update(pressure_pa, temperature_c);
	baro_state_t new_state = baro_state_get();
	last_baro_state = new_state;

	/* Update combined flight status */
	update_flight_status();

	int64_t now = k_uptime_get();
	
	/* Collect baro pressure sample at 10 Hz */
	if (now - last_sample_time_ms >= SAMPLE_INTERVAL_MS) {
		collect_baro_sample(pressure_pa);
		last_sample_time_ms = now;
	}
	
	/* Log CSV line once per second */
	if (now - last_log_time_ms >= LOG_INTERVAL_MS) {
		log_csv_line();
		last_log_time_ms = now;
	}
	
	/* Print status update every 10 seconds */
	if (now - last_status_time_ms >= STATUS_INTERVAL_MS) {
		const struct gps_state_data *gps_data = gps_state_get_data();
		const struct baro_state_data *baro_data = baro_state_get_data();
		const struct session_data *session = session_mgr_get_data();
		
		int64_t elapsed = session_mgr_get_elapsed_ms();
		
		/* Calculate samples per second over the status period */
		int64_t status_period_ms = now - status_period_start_ms;
		float samples_per_sec = 0.0f;
		if (status_period_ms > 0) {
			samples_per_sec = (float)total_samples_collected * 1000.0f / (float)status_period_ms;
		}
		
		LOG_INF("STATUS: uuid=%s, start=%lld, elapsed=%lld, gps_state=%s, baro_state=%s, "
			"lat=%.6f, lon=%.6f, alt=%.1f, pressure=%.1f Pa, temp=%.1f C, samples/sec=%.2f",
			session->uuid,
			session->start_time_ms,
			elapsed,
			gps_state_name(gps_data->state),
			baro_state_name(baro_data->state),
			gps_data->latitude,
			gps_data->longitude,
			(double)baro_data->altitude_ft,
			(double)baro_data->pressure_pa,
			(double)baro_data->temperature_c,
			(double)samples_per_sec);
		
		/* Reset counters for next status period */
		total_samples_collected = 0;
		status_period_start_ms = now;
		last_status_time_ms = now;
	}
}

flight_status_t flight_timer_get_status(void)
{
	return timer_data.status;
}

const struct flight_timer_data *flight_timer_get_data(void)
{
	return &timer_data;
}

int64_t flight_timer_get_flight_time_ms(void)
{
	int64_t total = timer_data.total_flight_time_ms;
	
	/* Add current flight time if airborne */
	if (timer_data.currently_airborne) {
		total += (k_uptime_get() - timer_data.flight_start_ms);
	}
	
	return total;
}

int64_t flight_timer_get_session_time_ms(void)
{
	return k_uptime_get() - timer_data.session_start_ms;
}

void flight_timer_periodic_update(void)
{
	int64_t now = k_uptime_get();
	
	/* Check for periodic save */
	if (now - last_save_time_ms >= SAVE_INTERVAL_MS) {
		flight_timer_save();
		last_save_time_ms = now;
	}
}

bool flight_timer_needs_save(void)
{
	int64_t now = k_uptime_get();
	return (now - last_save_time_ms >= SAVE_INTERVAL_MS);
}

int flight_timer_save(void)
{
	const struct gps_state_data *gps_data = gps_state_get_data();
	const struct baro_state_data *baro_data = baro_state_get_data();
	
	/* Update session manager with current state */
	session_mgr_update(
		gps_data->state,
		baro_data->state,
		gps_data->latitude,
		gps_data->longitude,
		baro_data->altitude_ft,
		0  /* Sample count no longer tracked separately */
	);
	
	/* Save session to SD card */
	int ret = session_mgr_save();
	if (ret != 0) {
		LOG_ERR("Failed to save session: %d", ret);
		return ret;
	}
	
	LOG_DBG("Session saved (flight time: %lld ms)",
		flight_timer_get_flight_time_ms());
	
	return 0;
}

