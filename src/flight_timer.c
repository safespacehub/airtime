/*
 * Flight Timer Module
 * Coordinates state machines and manages flight timing
 */

#include "flight_timer.h"
#include "gps_state.h"
#include "baro_state.h"
#include "session_mgr.h"
#include "rtc_sync.h"
#include "session_upload.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_heap.h>
#include <zephyr/sys/mem_stats.h>
#include <stdio.h>
#include <modem/nrf_modem_lib.h>

/* External system heap for memory stats */
extern struct sys_heap _system_heap;

LOG_MODULE_REGISTER(flight_timer, CONFIG_LOG_DEFAULT_LEVEL);

#define SAVE_INTERVAL_MS     30000  /* Save every 30 seconds */
#define SAMPLE_INTERVAL_MS   100    /* Collect baro samples every 100ms (10 Hz) */
#define LOG_INTERVAL_MS      1000   /* Log CSV every 1 second */
#define STATUS_INTERVAL_MS   60000  /* Status update every 60 seconds */
#define MAX_BARO_SAMPLES     12     /* Max baro samples per second */
#define BARO_UNAVAILABLE_MS  5000   /* Barometer considered unavailable if no update for 5 seconds */
#define STATUS_CHANGE_BUFFER_SIZE 200  /* Buffer for flight status changes */
#define MIN_BARO_SAMPLES_FOR_STATE_CHANGE 200  /* Minimum samples needed before allowing state changes (20 seconds at 10 Hz) */

static struct flight_timer_data timer_data;
static int64_t last_save_time_ms = 0;
static int64_t last_sample_time_ms = 0;
static int64_t last_log_time_ms = 0;
static int64_t last_status_time_ms = 0;
static int64_t last_baro_update_time_ms = 0;  /* Track last barometer update for availability */
static gps_state_t last_gps_state = GPS_STATE_UNKNOWN;
static baro_state_t last_baro_state = BARO_STATE_GROUND;

/* Buffer for baro pressure samples (collected at 10Hz, logged once per second) */
static float baro_samples[MAX_BARO_SAMPLES];
static uint8_t baro_sample_count = 0;
static uint32_t total_baro_samples_collected = 0;  /* Total samples collected since start (never reset) */

/* Sample rate tracking */
static uint32_t total_samples_collected = 0;  /* Total samples in current status period */
static int64_t status_period_start_ms = 0;     /* Start time of current status period */

/* Flight status change buffer */
static struct state_change_entry status_change_buffer[STATUS_CHANGE_BUFFER_SIZE];
static uint16_t status_change_count = 0;  /* Number of entries in buffer */
static uint16_t status_change_index = 0;  /* Current write index (circular buffer) */

/* Forward declaration */
static void record_status_change(flight_status_t status, int64_t timestamp_ms);

static void update_flight_status(void)
{
	gps_state_t gps = gps_state_get();
	baro_state_t baro = baro_state_get();
	flight_status_t old_status = timer_data.status;
	flight_status_t new_status;
	int64_t now = k_uptime_get();

	/* Prevent state changes until baro buffer has collected enough samples */
	/* Always start in GROUND and stay there until sensors are ready */
	if (total_baro_samples_collected < MIN_BARO_SAMPLES_FOR_STATE_CHANGE) {
		/* Buffer still filling up - ensure we stay in GROUND state */
		if (old_status != FLIGHT_STATUS_GROUND) {
			timer_data.status = FLIGHT_STATUS_GROUND;
		}
		return;
	}

	/* Check if barometer is available (has recent updates) */
	bool baro_available = (last_baro_update_time_ms > 0) &&
	                      (now - last_baro_update_time_ms) < BARO_UNAVAILABLE_MS;

	/* Determine flight state using improved algorithm */
	bool is_flying = false;

	if (baro_available) {
		/* Barometer available: use as default, GPS as override */
		/* Default: barometer state (climbing/descending/cruising = flying) */
		bool baro_flying = (baro == BARO_STATE_CLIMBING ||
		                    baro == BARO_STATE_CRUISING ||
		                    baro == BARO_STATE_DESCENDING);
		
		/* Override: GPS flight forces flying, GPS stopped forces ground */
		if (gps == GPS_STATE_FLIGHT) {
			is_flying = true;  /* GPS override: flying */
		} else if (gps == GPS_STATE_STOPPED) {
			is_flying = false;  /* GPS override: ground */
		} else {
			/* Use barometer default */
			is_flying = baro_flying;
		}
	} else {
		/* Barometer unavailable: use GPS as fallback */
		is_flying = (gps == GPS_STATE_FLIGHT);
	}

	new_status = is_flying ? FLIGHT_STATUS_AIRBORNE : FLIGHT_STATUS_GROUND;

	/* Update airborne tracking */
	if (new_status == FLIGHT_STATUS_AIRBORNE && !timer_data.currently_airborne) {
		/* Just became airborne */
		timer_data.currently_airborne = true;
		timer_data.flight_start_ms = now;
		LOG_INF("Became airborne (baro_avail=%d, baro=%s, gps=%s)",
			baro_available,
			baro_state_name(baro),
			gps_state_name(gps));
	} else if (new_status != FLIGHT_STATUS_AIRBORNE && timer_data.currently_airborne) {
		/* No longer airborne - accumulate flight time */
		timer_data.currently_airborne = false;
		timer_data.total_flight_time_ms += (now - timer_data.flight_start_ms);
		LOG_INF("No longer airborne, total flight time: %lld ms (baro_avail=%d, baro=%s, gps=%s)",
			timer_data.total_flight_time_ms,
			baro_available,
			baro_state_name(baro),
			gps_state_name(gps));
	}

	if (new_status != old_status) {
		timer_data.status = new_status;
		LOG_INF("Flight status: %s (baro_avail=%d, baro=%s, gps=%s)",
			new_status == FLIGHT_STATUS_GROUND ? "ground" : "flying",
			baro_available,
			baro_state_name(baro),
			gps_state_name(gps));
		
		/* Record status change in buffer */
		record_status_change(new_status, now);
		
		/* Update session.json immediately when master state changes */
		flight_timer_save();
		
		/* Trigger upload on state change */
		session_upload_trigger(true);
	}
}

static void record_status_change(flight_status_t status, int64_t timestamp_ms)
{
	/* Add entry to circular buffer */
	status_change_buffer[status_change_index].status = status;
	status_change_buffer[status_change_index].timestamp_ms = timestamp_ms;
	
	/* Update circular buffer index */
	status_change_index = (status_change_index + 1) % STATUS_CHANGE_BUFFER_SIZE;
	
	/* Update count (stop incrementing when buffer is full) */
	if (status_change_count < STATUS_CHANGE_BUFFER_SIZE) {
		status_change_count++;
	}
}

uint16_t flight_timer_get_status_changes(struct state_change_entry *buffer, uint16_t max_count)
{
	if (buffer == NULL || max_count == 0) {
		return 0;
	}

	uint16_t copy_count = status_change_count;
	if (copy_count > max_count) {
		copy_count = max_count;
	}

	if (copy_count == 0) {
		return 0;
	}

	/* Handle circular buffer - copy in chronological order (oldest first) */
	if (status_change_count < STATUS_CHANGE_BUFFER_SIZE) {
		/* Buffer not yet wrapped - simple linear copy from start */
		memcpy(buffer, status_change_buffer, copy_count * sizeof(struct state_change_entry));
	} else {
		/* Buffer has wrapped - status_change_index points to oldest entry */
		/* Copy oldest entries from status_change_index to end of buffer */
		uint16_t first_part = STATUS_CHANGE_BUFFER_SIZE - status_change_index;
		if (copy_count <= first_part) {
			/* All requested entries are in the first part */
			memcpy(buffer, &status_change_buffer[status_change_index],
				copy_count * sizeof(struct state_change_entry));
		} else {
			/* Copy from status_change_index to end */
			memcpy(buffer, &status_change_buffer[status_change_index],
				first_part * sizeof(struct state_change_entry));
			/* Copy remaining from start of buffer */
			uint16_t remaining = copy_count - first_part;
			memcpy(&buffer[first_part], status_change_buffer,
				remaining * sizeof(struct state_change_entry));
		}
	}

	return copy_count;
}

static void collect_baro_sample(float pressure_pa)
{
	if (baro_sample_count < MAX_BARO_SAMPLES) {
		baro_samples[baro_sample_count++] = pressure_pa;
		total_samples_collected++;  /* Track for rate calculation */
		total_baro_samples_collected++;  /* Track total samples (never reset) */
	} else {
		LOG_WRN("Baro sample buffer full (%u/%u), dropping sample", 
			baro_sample_count, MAX_BARO_SAMPLES);
	}
}

static void log_csv_line(void)
{
	const struct gps_state_data *gps_data = gps_state_get_data();
	const struct baro_state_data *baro_data = baro_state_get_data();
	flight_status_t flight_status = flight_timer_get_status();
	
	/* Format: flight_status,baro_state,gps_state,lat,lon,speed,alt,temp,p1,p2,p3... */
	char line[256];
	int pos = 0;
	
	/* Header data: states, GPS position, speed, GPS altitude, temperature */
	pos += snprintf(line + pos, sizeof(line) - pos,
		"CSV,%c,%c,%c,%.6f,%.6f,%.1f,%.1f,%.1f",
		flight_status == FLIGHT_STATUS_GROUND ? 'G' : 'F',
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
	int64_t now = k_uptime_get();
	
	memset(&timer_data, 0, sizeof(timer_data));
	timer_data.status = FLIGHT_STATUS_GROUND;
	timer_data.currently_airborne = false;
	
	last_save_time_ms = 0;
	last_sample_time_ms = 0;
	last_log_time_ms = 0;
	last_status_time_ms = 0;
	last_baro_update_time_ms = 0;
	last_gps_state = GPS_STATE_UNKNOWN;
	last_baro_state = BARO_STATE_GROUND;
	baro_sample_count = 0;
	total_samples_collected = 0;
	total_baro_samples_collected = 0;
	status_period_start_ms = 0;
	
	/* Initialize status change buffer with default state */
	memset(status_change_buffer, 0, sizeof(status_change_buffer));
	status_change_count = 0;
	status_change_index = 0;
	record_status_change(FLIGHT_STATUS_GROUND, now);

	LOG_INF("Flight timer initialized");
	return 0;
}

int flight_timer_start(void)
{
	int64_t now = k_uptime_get();
	
	timer_data.session_start_ms = now;
	timer_data.flight_start_ms = 0;
	timer_data.total_flight_time_ms = 0;
	timer_data.currently_airborne = false;
	timer_data.status = FLIGHT_STATUS_GROUND;
	
	last_save_time_ms = timer_data.session_start_ms;
	last_sample_time_ms = timer_data.session_start_ms;
	last_log_time_ms = timer_data.session_start_ms;
	last_status_time_ms = timer_data.session_start_ms;
	last_baro_update_time_ms = timer_data.session_start_ms;
	baro_sample_count = 0;
	total_samples_collected = 0;
	total_baro_samples_collected = 0;
	status_period_start_ms = timer_data.session_start_ms;
	
	/* Reset status change buffer and initialize with default state */
	status_change_count = 0;
	status_change_index = 0;
	record_status_change(FLIGHT_STATUS_GROUND, now);

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
		
		/* Update session position tracking */
		const struct gps_state_data *gps_data = gps_state_get_data();
		if (gps_data->latitude != 0.0 && gps_data->longitude != 0.0) {
			/* Set start position on first valid fix */
			bool start_pos_set = session_mgr_set_start_position(gps_data->latitude, gps_data->longitude);
			/* Update stop position (most recent position) */
			session_mgr_update_position(gps_data->latitude, gps_data->longitude);
			
			/* Save session immediately when GPS first gets a fix */
			if (start_pos_set) {
				flight_timer_save();
			}
		}
	}

	/* Update combined flight status */
	update_flight_status();
}

void flight_timer_process_baro(float pressure_pa, float temperature_c)
{
	int64_t now = k_uptime_get();
	
	/* Update barometer state machine */
	baro_state_update(pressure_pa, temperature_c);
	baro_state_t new_state = baro_state_get();
	last_baro_state = new_state;
	
	/* Update barometer availability timestamp */
	last_baro_update_time_ms = now;

	/* Update combined flight status */
	update_flight_status();
	
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
		
		/* Collect device status (memory and CPU) */
		char mem_info[128] = "N/A";
		char cpu_info[32] = "N/A";
		
		#ifdef CONFIG_SYS_HEAP_RUNTIME_STATS
		struct sys_memory_stats sys_mem_stats;
		size_t sys_total = 0;
		size_t sys_used = 0;
		
		if (sys_heap_runtime_stats_get(&_system_heap, &sys_mem_stats) == 0) {
			sys_total = sys_mem_stats.free_bytes + sys_mem_stats.allocated_bytes;
			sys_used = sys_mem_stats.allocated_bytes;
		}
		
		/* Also get modem library heap stats if available */
		#if defined(CONFIG_NRF_MODEM_LIB) && defined(CONFIG_NRF_MODEM_LIB_MEM_DIAG)
		struct nrf_modem_lib_diag_stats modem_stats;
		size_t modem_total = 0;
		size_t modem_used = 0;
		
		if (nrf_modem_is_initialized() && 
		    nrf_modem_lib_diag_stats_get(&modem_stats) == 0) {
			modem_total = modem_stats.library.heap.free_bytes + 
			              modem_stats.library.heap.allocated_bytes;
			modem_used = modem_stats.library.heap.allocated_bytes;
		}
		
		/* Combine system and modem heap stats */
		size_t total_mem = sys_total + modem_total;
		size_t used_mem = sys_used + modem_used;
		float mem_used_pct = total_mem > 0 ? 
			(100.0f * (float)used_mem / (float)total_mem) : 0.0f;
		
		if (modem_total > 0) {
			/* Show combined stats: sys+modem */
			snprintf(mem_info, sizeof(mem_info), "heap=%lu/%lu (%.1f%%, sys:%lu+modem:%lu)",
				(unsigned long)used_mem, 
				(unsigned long)total_mem, 
				(double)mem_used_pct,
				(unsigned long)sys_used,
				(unsigned long)modem_used);
		} else {
			/* Only system heap available */
			snprintf(mem_info, sizeof(mem_info), "heap=%lu/%lu (%.1f%%)",
				(unsigned long)sys_used, 
				(unsigned long)sys_total, 
				(double)mem_used_pct);
		}
		#else
		/* Only system heap, no modem */
		float mem_used_pct = sys_total > 0 ? 
			(100.0f * (float)sys_used / (float)sys_total) : 0.0f;
		snprintf(mem_info, sizeof(mem_info), "heap=%lu/%lu (%.1f%%)",
			(unsigned long)sys_used, 
			(unsigned long)sys_total, 
			(double)mem_used_pct);
		#endif
		#endif
		
		#ifdef CONFIG_SCHED_THREAD_USAGE_ALL
		k_thread_runtime_stats_t cpu_stats;
		int cpu_ret = k_thread_runtime_stats_all_get(&cpu_stats);
		if (cpu_ret == 0) {
			float cpu_util = 0.0f;
			/* execution_cycles = total_cycles + idle_cycles for CPU stats */
			/* CPU utilization = (non-idle / total) * 100 */
			if (cpu_stats.execution_cycles > 0) {
				cpu_util = 100.0f * (float)cpu_stats.total_cycles / 
					(float)cpu_stats.execution_cycles;
				/* Clamp to valid range */
				if (cpu_util > 100.0f) {
					cpu_util = 100.0f;
				}
				if (cpu_util < 0.0f) {
					cpu_util = 0.0f;
				}
			}
			snprintf(cpu_info, sizeof(cpu_info), "%.1f%%", (double)cpu_util);
		} else {
			/* Stats not available yet or error */
			snprintf(cpu_info, sizeof(cpu_info), "err:%d", cpu_ret);
		}
		#endif
		
		LOG_INF("STATUS: uuid=%s, elapsed=%lld, gps_state=%s, baro_state=%s, "
			"lat=%.6f, lon=%.6f, alt=%.1f, pressure=%.1f Pa, temp=%.1f C, samples/sec=%.2f, "
			"mem=%s, cpu=%s",
			session->uuid,
			elapsed,
			gps_state_name(gps_data->state),
			baro_state_name(baro_data->state),
			gps_data->latitude,
			gps_data->longitude,
			(double)baro_data->altitude_ft,
			(double)baro_data->pressure_pa,
			(double)baro_data->temperature_c,
			(double)samples_per_sec,
			mem_info,
			cpu_info);
		
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
	/* Get status changes from buffer */
	struct state_change_entry state_changes[STATUS_CHANGE_BUFFER_SIZE];
	uint16_t count = flight_timer_get_status_changes(state_changes, STATUS_CHANGE_BUFFER_SIZE);
	
	/* Update stop position with current GPS position */
	const struct gps_state_data *gps_data = gps_state_get_data();
	if (gps_data->latitude != 0.0 && gps_data->longitude != 0.0) {
		session_mgr_update_position(gps_data->latitude, gps_data->longitude);
	}
	
	/* Save session to SD card */
	int ret = session_mgr_save(state_changes, count);
	if (ret != 0) {
		LOG_ERR("Failed to save session: %d", ret);
		return ret;
	}
	
	LOG_DBG("Session saved (flight time: %lld ms, %d state changes)",
		flight_timer_get_flight_time_ms(), count);
	
	return 0;
}

