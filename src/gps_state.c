/*
 * GPS State Machine
 * Tracks flight state based on GPS speed with hysteresis
 */

#include "gps_state.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gps_state, CONFIG_LOG_DEFAULT_LEVEL);

/* Speed thresholds in knots */
#define SPEED_STOPPED_MAX_KTS      5.0f
#define SPEED_TAXI_MIN_KTS         5.0f
#define SPEED_TAXI_MAX_KTS         50.0f
#define SPEED_FLIGHT_MIN_KTS       50.0f

/* Hysteresis thresholds */
#define HYST_STOPPED_TO_TAXI_KTS   8.0f   /* Must exceed 8 kts to enter taxi */
#define HYST_TAXI_TO_STOPPED_KTS   4.0f   /* Must drop below 4.0 kts to stop */
#define HYST_TAXI_TO_FLIGHT_KTS    55.0f  /* Must exceed 55 kts to enter flight */
#define HYST_FLIGHT_TO_TAXI_KTS    45.0f  /* Must drop below 45 kts to exit flight */

/* Stale threshold in milliseconds */
#define STALE_THRESHOLD_MS         20000

/* Minimum number of valid fixes required before allowing state transitions */
/* Prevents false state changes from inaccurate initial GPS fixes */
#define MIN_FIXES_FOR_STATE_CHANGE 5

/* Conversion factor: m/s to knots */
#define MPS_TO_KTS                 1.94384f

static struct gps_state_data state_data;
static uint32_t valid_fix_count = 0;  /* Count of valid fixes received */

int gps_state_init(void)
{
	memset(&state_data, 0, sizeof(state_data));
	state_data.state = GPS_STATE_UNKNOWN;
	state_data.prev_state = GPS_STATE_UNKNOWN;
	state_data.last_fix_time_ms = 0;
	state_data.state_enter_time_ms = k_uptime_get();
	state_data.has_fix = false;
	valid_fix_count = 0;

	LOG_INF("GPS state machine initialized");
	return 0;
}

static gps_state_t calculate_new_state(float speed_kts, gps_state_t current)
{
	gps_state_t new_state = current;

	switch (current) {
	case GPS_STATE_UNKNOWN:
		/* Transition to stopped if we have valid low speed */
		if (speed_kts < SPEED_STOPPED_MAX_KTS) {
			new_state = GPS_STATE_STOPPED;
		} else if (speed_kts < SPEED_TAXI_MAX_KTS) {
			new_state = GPS_STATE_TAXI;
		} else {
			new_state = GPS_STATE_FLIGHT;
		}
		break;

	case GPS_STATE_STOPPED:
		/* Use hysteresis to enter taxi */
		if (speed_kts >= HYST_STOPPED_TO_TAXI_KTS) {
			new_state = GPS_STATE_TAXI;
		}
		break;

	case GPS_STATE_TAXI:
		/* Use hysteresis for transitions */
		if (speed_kts < HYST_TAXI_TO_STOPPED_KTS) {
			new_state = GPS_STATE_STOPPED;
		} else if (speed_kts >= HYST_TAXI_TO_FLIGHT_KTS) {
			new_state = GPS_STATE_FLIGHT;
		}
		break;

	case GPS_STATE_FLIGHT:
		/* Use hysteresis to exit flight */
		if (speed_kts < HYST_FLIGHT_TO_TAXI_KTS) {
			new_state = GPS_STATE_TAXI;
		}
		break;
	}

	return new_state;
}

bool gps_state_update(const struct nrf_modem_gnss_pvt_data_frame *pvt)
{
	if (pvt == NULL) {
		return false;
	}

	int64_t now = k_uptime_get();
	gps_state_t old_state = state_data.state;
	gps_state_t new_state;

	/* Check if we have a valid fix */
	bool valid_fix = (pvt->flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) != 0;

	if (valid_fix) {
		state_data.last_fix_time_ms = now;
		state_data.has_fix = true;
		state_data.latitude = pvt->latitude;
		state_data.longitude = pvt->longitude;
		state_data.altitude_m = pvt->altitude;
		state_data.speed_mps = pvt->speed;
		state_data.speed_kts = pvt->speed * MPS_TO_KTS;

		/* Increment valid fix counter */
		valid_fix_count++;

		/* Prevent state transitions from UNKNOWN until we have enough fixes */
		/* This prevents false state changes from inaccurate initial GPS readings */
		if (state_data.state == GPS_STATE_UNKNOWN) {
			if (valid_fix_count < MIN_FIXES_FOR_STATE_CHANGE) {
				/* Not enough fixes yet - stay in UNKNOWN */
				new_state = GPS_STATE_UNKNOWN;
			} else {
				/* Enough fixes collected - allow state transition */
				new_state = calculate_new_state(state_data.speed_kts, state_data.state);
			}
		} else {
			/* Already have a valid state - allow transitions */
			new_state = calculate_new_state(state_data.speed_kts, state_data.state);
		}
	} else {
		/* No valid fix - check if data is stale */
		if (state_data.state != GPS_STATE_UNKNOWN) {
			int64_t age = now - state_data.last_fix_time_ms;
			if (age > STALE_THRESHOLD_MS || state_data.last_fix_time_ms == 0) {
				new_state = GPS_STATE_UNKNOWN;
				state_data.has_fix = false;
				/* Reset fix counter when going stale */
				valid_fix_count = 0;
			} else {
				/* Keep current state, data not stale yet */
				new_state = state_data.state;
			}
		} else {
			new_state = GPS_STATE_UNKNOWN;
			/* Reset fix counter if we're in UNKNOWN and have no fix */
			valid_fix_count = 0;
		}
	}

	/* Update state if changed */
	if (new_state != old_state) {
		state_data.prev_state = old_state;
		state_data.state = new_state;
		state_data.state_enter_time_ms = now;

		LOG_INF("GPS state: %s -> %s (speed: %.1f kts)",
			gps_state_name(old_state),
			gps_state_name(new_state),
			(double)state_data.speed_kts);

		return true;
	}

	return false;
}

gps_state_t gps_state_get(void)
{
	/* Check for stale data */
	if (state_data.state != GPS_STATE_UNKNOWN) {
		int64_t age = k_uptime_get() - state_data.last_fix_time_ms;
		if (age > STALE_THRESHOLD_MS) {
			/* Transition to unknown if stale */
			if (state_data.state != GPS_STATE_UNKNOWN) {
				state_data.prev_state = state_data.state;
				state_data.state = GPS_STATE_UNKNOWN;
				state_data.state_enter_time_ms = k_uptime_get();
				state_data.has_fix = false;
				LOG_WRN("GPS data stale, entering UNKNOWN state");
			}
		}
	}
	return state_data.state;
}

const struct gps_state_data *gps_state_get_data(void)
{
	return &state_data;
}

const char *gps_state_name(gps_state_t state)
{
	switch (state) {
	case GPS_STATE_UNKNOWN: return "unknown";
	case GPS_STATE_STOPPED: return "stopped";
	case GPS_STATE_TAXI:    return "taxi";
	case GPS_STATE_FLIGHT:  return "flight";
	default:                return "invalid";
	}
}

char gps_state_code(gps_state_t state)
{
	switch (state) {
	case GPS_STATE_UNKNOWN: return 'U';
	case GPS_STATE_STOPPED: return 'S';
	case GPS_STATE_TAXI:    return 'T';
	case GPS_STATE_FLIGHT:  return 'F';
	default:                return '?';
	}
}

bool gps_state_is_stale(void)
{
	if (state_data.last_fix_time_ms == 0) {
		return true;
	}
	int64_t age = k_uptime_get() - state_data.last_fix_time_ms;
	return age > STALE_THRESHOLD_MS;
}

