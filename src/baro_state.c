/*
 * Barometer State Machine
 * Tracks flight state based on pressure altitude changes
 */

#include "baro_state.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(baro_state, CONFIG_LOG_DEFAULT_LEVEL);

/* Configuration */
#define SAMPLE_RATE_HZ          10      /* 10 Hz sampling */
#define BUFFER_SECONDS          20      /* 20 second window */
#define BUFFER_SIZE             (SAMPLE_RATE_HZ * BUFFER_SECONDS)  /* 200 samples */

/* Thresholds */
#define STDDEV_FLIGHT_THRESHOLD_FT  3.0f    /* StdDev > 3.0 ft = flight */
#define STDDEV_GROUND_THRESHOLD_FT  2.0f    /* StdDev < 2.0 ft = ground */
#define VERTICAL_RATE_CLIMB_FPM     200.0f  /* > 200 fpm = climbing */
#define VERTICAL_RATE_DESCEND_FPM   -200.0f /* < -200 fpm = descending */
#define CRUISE_GUARD_TIME_MS        (2 * 60 * 1000)  /* 2 minutes */

/* Standard atmosphere constants */
#define SEA_LEVEL_PRESSURE_PA   101325.0f
#define ALTITUDE_CONSTANT       145366.45f
#define PRESSURE_EXPONENT       0.190284f

/* Circular buffer for altitude samples */
struct altitude_sample {
	float altitude_ft;
	int64_t timestamp_ms;
};

static struct altitude_sample sample_buffer[BUFFER_SIZE];
static uint16_t buffer_head = 0;
static uint16_t buffer_count = 0;
static bool buffer_filled_logged = false;  /* Track if we've logged buffer fill */

static struct baro_state_data state_data;

float baro_pressure_to_altitude_ft(float pressure_pa)
{
	if (pressure_pa <= 0) {
		return 0.0f;
	}
	return ALTITUDE_CONSTANT * (1.0f - powf(pressure_pa / SEA_LEVEL_PRESSURE_PA, PRESSURE_EXPONENT));
}

static void add_sample(float altitude_ft, int64_t timestamp_ms)
{
	sample_buffer[buffer_head].altitude_ft = altitude_ft;
	sample_buffer[buffer_head].timestamp_ms = timestamp_ms;
	
	buffer_head = (buffer_head + 1) % BUFFER_SIZE;
	if (buffer_count < BUFFER_SIZE) {
		buffer_count++;
		/* Log when 20-second sliding window is first filled */
		if (buffer_count == BUFFER_SIZE && !buffer_filled_logged) {
			LOG_INF("Baro 20-second sliding window filled and ready (%u samples)", buffer_count);
			buffer_filled_logged = true;
		}
	}
}

static float calculate_mean(void)
{
	if (buffer_count == 0) {
		return 0.0f;
	}

	float sum = 0.0f;
	for (uint16_t i = 0; i < buffer_count; i++) {
		sum += sample_buffer[i].altitude_ft;
	}
	return sum / buffer_count;
}

static float calculate_std_dev(float mean)
{
	if (buffer_count < 2) {
		return 0.0f;
	}

	float sum_sq = 0.0f;
	for (uint16_t i = 0; i < buffer_count; i++) {
		float diff = sample_buffer[i].altitude_ft - mean;
		sum_sq += diff * diff;
	}
	return sqrtf(sum_sq / (buffer_count - 1));
}

static float calculate_vertical_rate(void)
{
	if (buffer_count < 2) {
		return 0.0f;
	}

	/* Use linear regression for smoothed vertical rate */
	/* Or simple delta between oldest and newest samples */
	
	/* Find oldest valid sample */
	uint16_t oldest_idx = (buffer_head + BUFFER_SIZE - buffer_count) % BUFFER_SIZE;
	uint16_t newest_idx = (buffer_head + BUFFER_SIZE - 1) % BUFFER_SIZE;
	
	float alt_diff = sample_buffer[newest_idx].altitude_ft - 
	                 sample_buffer[oldest_idx].altitude_ft;
	int64_t time_diff_ms = sample_buffer[newest_idx].timestamp_ms - 
	                       sample_buffer[oldest_idx].timestamp_ms;
	
	if (time_diff_ms <= 0) {
		return 0.0f;
	}

	/* Convert to feet per minute */
	float time_diff_min = (float)time_diff_ms / 60000.0f;
	return alt_diff / time_diff_min;
}

int baro_state_init(void)
{
	memset(&state_data, 0, sizeof(state_data));
	memset(sample_buffer, 0, sizeof(sample_buffer));
	
	buffer_head = 0;
	buffer_count = 0;
	buffer_filled_logged = false;
	
	state_data.state = BARO_STATE_GROUND;
	state_data.prev_state = BARO_STATE_GROUND;
	state_data.state_enter_time_ms = k_uptime_get();
	state_data.cruise_start_time_ms = 0;
	state_data.in_flight = false;

	LOG_INF("Barometer state machine initialized (buffer: %d samples)", BUFFER_SIZE);
	return 0;
}

bool baro_state_update(float pressure_pa, float temperature_c)
{
	int64_t now = k_uptime_get();
	baro_state_t old_state = state_data.state;
	baro_state_t new_state = old_state;

	/* Update current values */
	state_data.pressure_pa = pressure_pa;
	state_data.temperature_c = temperature_c;
	state_data.altitude_ft = baro_pressure_to_altitude_ft(pressure_pa);

	/* Add sample to buffer */
	add_sample(state_data.altitude_ft, now);

	/* Calculate statistics (only if we have enough samples) */
	if (buffer_count >= SAMPLE_RATE_HZ) {  /* At least 1 second of data */
		float mean = calculate_mean();
		state_data.altitude_std_dev_ft = calculate_std_dev(mean);
		state_data.vertical_rate_fpm = calculate_vertical_rate();
	}

	/* Determine if we're in flight based on altitude variability */
	bool altitude_indicates_flight = state_data.altitude_std_dev_ft > STDDEV_FLIGHT_THRESHOLD_FT;
	bool altitude_indicates_ground = state_data.altitude_std_dev_ft < STDDEV_GROUND_THRESHOLD_FT;

	/* Determine vertical state */
	baro_state_t vertical_state;
	if (state_data.vertical_rate_fpm > VERTICAL_RATE_CLIMB_FPM) {
		vertical_state = BARO_STATE_CLIMBING;
	} else if (state_data.vertical_rate_fpm < VERTICAL_RATE_DESCEND_FPM) {
		vertical_state = BARO_STATE_DESCENDING;
	} else {
		vertical_state = BARO_STATE_CRUISING;
	}

	/* State machine logic with guards */
	switch (old_state) {
	case BARO_STATE_GROUND:
		if (altitude_indicates_flight) {
			/* Entering flight - determine vertical state */
			new_state = vertical_state;
			state_data.in_flight = true;
		}
		break;

	case BARO_STATE_CLIMBING:
		if (altitude_indicates_ground) {
			/* Guard: Cannot transition directly from climb to ground */
			/* Must go through cruise or descent first */
			if (state_data.vertical_rate_fpm < VERTICAL_RATE_CLIMB_FPM) {
				new_state = BARO_STATE_CRUISING;
				state_data.cruise_start_time_ms = now;
			}
		} else {
			/* Update vertical state */
			if (state_data.vertical_rate_fpm < VERTICAL_RATE_DESCEND_FPM) {
				new_state = BARO_STATE_DESCENDING;
			} else if (state_data.vertical_rate_fpm < VERTICAL_RATE_CLIMB_FPM &&
			           state_data.vertical_rate_fpm > VERTICAL_RATE_DESCEND_FPM) {
				new_state = BARO_STATE_CRUISING;
				state_data.cruise_start_time_ms = now;
			}
		}
		break;

	case BARO_STATE_CRUISING:
		if (altitude_indicates_ground) {
			/* Guard: Cannot transition to ground if cruising for > 2 min */
			int64_t cruise_duration = now - state_data.cruise_start_time_ms;
			if (cruise_duration < CRUISE_GUARD_TIME_MS) {
				new_state = BARO_STATE_GROUND;
				state_data.in_flight = false;
			}
			/* If > 2 min cruise, stay in cruise until descent */
		} else {
			/* Update vertical state */
			if (state_data.vertical_rate_fpm > VERTICAL_RATE_CLIMB_FPM) {
				new_state = BARO_STATE_CLIMBING;
			} else if (state_data.vertical_rate_fpm < VERTICAL_RATE_DESCEND_FPM) {
				new_state = BARO_STATE_DESCENDING;
			}
		}
		break;

	case BARO_STATE_DESCENDING:
		if (altitude_indicates_ground) {
			/* Descending to ground is allowed */
			new_state = BARO_STATE_GROUND;
			state_data.in_flight = false;
		} else {
			/* Update vertical state */
			if (state_data.vertical_rate_fpm > VERTICAL_RATE_CLIMB_FPM) {
				new_state = BARO_STATE_CLIMBING;
			} else if (state_data.vertical_rate_fpm > VERTICAL_RATE_DESCEND_FPM &&
			           state_data.vertical_rate_fpm < VERTICAL_RATE_CLIMB_FPM) {
				new_state = BARO_STATE_CRUISING;
				state_data.cruise_start_time_ms = now;
			}
		}
		break;
	}

	/* Update state if changed */
	if (new_state != old_state) {
		state_data.prev_state = old_state;
		state_data.state = new_state;
		state_data.state_enter_time_ms = now;

		LOG_INF("Baro state: %s -> %s (stddev: %.2f ft, vrate: %.0f fpm)",
			baro_state_name(old_state),
			baro_state_name(new_state),
			(double)state_data.altitude_std_dev_ft,
			(double)state_data.vertical_rate_fpm);

		return true;
	}

	return false;
}

baro_state_t baro_state_get(void)
{
	return state_data.state;
}

const struct baro_state_data *baro_state_get_data(void)
{
	return &state_data;
}

const char *baro_state_name(baro_state_t state)
{
	switch (state) {
	case BARO_STATE_GROUND:     return "ground";
	case BARO_STATE_CLIMBING:   return "climbing";
	case BARO_STATE_CRUISING:   return "cruising";
	case BARO_STATE_DESCENDING: return "descending";
	default:                    return "invalid";
	}
}

char baro_state_code(baro_state_t state)
{
	switch (state) {
	case BARO_STATE_GROUND:     return 'G';
	case BARO_STATE_CLIMBING:   return 'C';
	case BARO_STATE_CRUISING:   return 'R';
	case BARO_STATE_DESCENDING: return 'D';
	default:                    return '?';
	}
}

bool baro_state_is_in_flight(void)
{
	return state_data.in_flight;
}

