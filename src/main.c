/*
 * Flight Timer Application for nRF9151 Feather
 *
 * Tracks flight time based on GPS speed and barometric altitude changes.
 * Logs data to SD card with session UUID management.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor/bmp581_user.h>

#include <modem/nrf_modem_lib.h>
#include <nrf_modem_gnss.h>
#include <modem/lte_lc.h>
#include <time.h>

#include "rtc_sync.h"
#include "supl_assist.h"
#include "gps_state.h"
#include "baro_state.h"
#include "sd_logger.h"
#include "session_mgr.h"
#include "flight_timer.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* Work queue for GNSS/SUPL operations */
#define GNSS_WORKQ_STACK_SIZE 4096
#define GNSS_WORKQ_PRIORITY   5

K_THREAD_STACK_DEFINE(gnss_workq_stack, GNSS_WORKQ_STACK_SIZE);
static struct k_work_q gnss_work_q;

/* Semaphores for event signaling */
static K_SEM_DEFINE(pvt_data_sem, 0, 1);
static K_SEM_DEFINE(lte_ready_sem, 0, 1);

/* GNSS data */
static struct nrf_modem_gnss_pvt_data_frame last_pvt;
static struct nrf_modem_gnss_agnss_data_frame last_agnss;
static struct k_work agnss_data_get_work;
static volatile bool requesting_assistance;

/* BMP581 device */
static const struct device *bmp581_dev;

/* Configuration */
#define BARO_SAMPLE_INTERVAL_MS  100  /* 10 Hz */
#define MAIN_LOOP_INTERVAL_MS    100

/* Forward declarations */
static void gnss_event_handler(int event);
static void lte_handler(const struct lte_lc_evt *const evt);
static void agnss_data_get_work_fn(struct k_work *item);

static const char *get_system_string(uint8_t system_id)
{
	switch (system_id) {
	case NRF_MODEM_GNSS_SYSTEM_INVALID:
		return "invalid";
	case NRF_MODEM_GNSS_SYSTEM_GPS:
		return "GPS";
	case NRF_MODEM_GNSS_SYSTEM_QZSS:
		return "QZSS";
	default:
		return "unknown";
	}
}

static int init_modem(void)
{
	int err;

	LOG_INF("Initializing modem...");

	err = nrf_modem_lib_init();
	if (err) {
		LOG_ERR("Modem library init failed: %d", err);
		return err;
	}

	/* Register LTE event handler */
	lte_lc_register_handler(lte_handler);

	/* Request PSM for power saving */
	err = lte_lc_psm_req(true);
	if (err) {
		LOG_WRN("Failed to request PSM: %d", err);
	}

	/* Connect to LTE network */
	LOG_INF("Connecting to LTE network...");
	err = lte_lc_connect();
	if (err) {
		LOG_ERR("LTE connection failed: %d", err);
		return err;
	}

	/* Wait for connection */
	k_sem_take(&lte_ready_sem, K_MINUTES(5));

	LOG_INF("LTE connected");
	return 0;
}

static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
		if (evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ||
		    evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING) {
			LOG_INF("LTE registered");
			k_sem_give(&lte_ready_sem);
		}
		break;
	default:
		break;
	}
}

static void agnss_data_get_work_fn(struct k_work *item)
{
	ARG_UNUSED(item);

	int err;

	/* GPS data need is always expected to be present and first in list. */
	if (last_agnss.system_count == 0) {
		LOG_WRN("GNSS system data need not found");
		return;
	}
	if (last_agnss.system[0].system_id != NRF_MODEM_GNSS_SYSTEM_GPS) {
		LOG_WRN("GPS data need not found");
		return;
	}

	/* SUPL doesn't usually provide NeQuick ionospheric corrections and satellite real time
	 * integrity information. If GNSS asks only for those, the request should be ignored.
	 */
	if (last_agnss.system[0].sv_mask_ephe == 0 &&
	    last_agnss.system[0].sv_mask_alm == 0 &&
	    (last_agnss.data_flags & ~(NRF_MODEM_GNSS_AGNSS_NEQUICK_REQUEST |
				       NRF_MODEM_GNSS_AGNSS_INTEGRITY_REQUEST)) == 0) {
		LOG_INF("Ignoring assistance request for only NeQuick and/or integrity");
		return;
	}

	if (last_agnss.data_flags == 0 &&
	    last_agnss.system[0].sv_mask_ephe == 0 &&
	    last_agnss.system[0].sv_mask_alm == 0) {
		LOG_INF("Ignoring assistance request because only QZSS data is requested");
		return;
	}

	requesting_assistance = true;

	LOG_INF("Assistance data needed: data_flags: 0x%02x", last_agnss.data_flags);
	for (int i = 0; i < last_agnss.system_count; i++) {
		LOG_INF("Assistance data needed: %s ephe: 0x%llx, alm: 0x%llx",
			get_system_string(last_agnss.system[i].system_id),
			last_agnss.system[i].sv_mask_ephe,
			last_agnss.system[i].sv_mask_alm);
	}

	/* Ensure LTE is registered before attempting SUPL */
	enum lte_lc_nw_reg_status reg_status;
	err = lte_lc_nw_reg_status_get(&reg_status);
	if (err != 0 || (reg_status != LTE_LC_NW_REG_REGISTERED_HOME &&
			 reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
		LOG_WRN("LTE not registered (status: %d), cannot perform SUPL", reg_status);
		requesting_assistance = false;
		return;
	}

	/* Wait a bit for IP address to be assigned (IPv4v6 PDN can take time) */
	k_sleep(K_SECONDS(1));

	err = supl_assist_request(&last_agnss);
	if (err) {
		LOG_ERR("Failed to request assistance data");
	}

	requesting_assistance = false;
}

static int init_gnss(void)
{
	int err;

	LOG_INF("Initializing GNSS...");

	/* Set event handler */
	err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
	if (err) {
		LOG_ERR("Failed to set GNSS event handler: %d", err);
		return err;
	}

	/* Use case: multiple hot starts, low accuracy allowed */
	uint8_t use_case = NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START |
			   NRF_MODEM_GNSS_USE_CASE_SCHED_DOWNLOAD_DISABLE;
	err = nrf_modem_gnss_use_case_set(use_case);
	if (err) {
		LOG_WRN("Failed to set use case: %d", err);
	}

	/* Continuous navigation mode */
	err = nrf_modem_gnss_fix_interval_set(1);
	if (err) {
		LOG_ERR("Failed to set fix interval: %d", err);
		return err;
	}

	/* Disable power saving for continuous tracking */
	err = nrf_modem_gnss_power_mode_set(NRF_MODEM_GNSS_PSM_DISABLED);
	if (err) {
		LOG_WRN("Failed to set power mode: %d", err);
	}

	/* Start GNSS */
	err = nrf_modem_gnss_start();
	if (err) {
		LOG_ERR("Failed to start GNSS: %d", err);
		return err;
	}

	LOG_INF("GNSS started (waiting for fix...)");
	return 0;
}

static void gnss_event_handler(int event)
{
	int err;

	switch (event) {
	case NRF_MODEM_GNSS_EVT_PVT:
		err = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt),
					  NRF_MODEM_GNSS_DATA_PVT);
		if (err == 0) {
			k_sem_give(&pvt_data_sem);
		}
		break;

	case NRF_MODEM_GNSS_EVT_AGNSS_REQ:
		err = nrf_modem_gnss_read(&last_agnss, sizeof(last_agnss),
					  NRF_MODEM_GNSS_DATA_AGNSS_REQ);
		if (err == 0) {
			k_work_submit_to_queue(&gnss_work_q, &agnss_data_get_work);
		}
		break;

	case NRF_MODEM_GNSS_EVT_FIX:
		LOG_DBG("GNSS fix obtained");
		break;

	case NRF_MODEM_GNSS_EVT_BLOCKED:
		LOG_WRN("GNSS blocked by LTE");
		break;

	case NRF_MODEM_GNSS_EVT_UNBLOCKED:
		LOG_INF("GNSS unblocked");
		break;
	
	case NRF_MODEM_GNSS_EVT_PERIODIC_WAKEUP:
		LOG_DBG("GNSS periodic wakeup");
		break;
	
	case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_TIMEOUT:
		LOG_WRN("GNSS sleep after timeout");
		break;
	
	case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_FIX:
		LOG_DBG("GNSS sleep after fix");
		break;
	
	case NRF_MODEM_GNSS_EVT_REF_ALT_EXPIRED:
		LOG_DBG("GNSS reference altitude expired");
		break;

	default:
		break;
	}
}

static int init_barometer(void)
{
	bmp581_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(bmp581));
	
	if (bmp581_dev == NULL) {
		LOG_ERR("BMP581 not found in devicetree");
		return -ENODEV;
	}

	if (!device_is_ready(bmp581_dev)) {
		LOG_ERR("BMP581 device not ready");
		return -ENODEV;
	}

	/* Configure BMP581:
	 * - IIR filter: bypass (0)
	 * - Oversampling: 128x
	 * - ODR: 10 Hz (continuous mode)
	 */
	struct sensor_value val;

	/* Set oversampling to 128x for both channels */
	/* val.val1 = oversampling rate, val.val2 = pressure enable (1=enabled) */
	/* Temperature is always enabled when oversampling is set for SENSOR_CHAN_ALL */
	val.val1 = 7;  /* BMP5_OVERSAMPLING_128X */
	val.val2 = 1;  /* Enable pressure (temperature is automatically enabled) */
	int err = sensor_attr_set(bmp581_dev, SENSOR_CHAN_ALL,
				  SENSOR_ATTR_OVERSAMPLING, &val);
	if (err) {
		LOG_WRN("Failed to set oversampling: %d", err);
	} else {
		LOG_INF("BMP581 oversampling set: 128x for pressure and temperature");
	}
	
	/* Explicitly configure temperature channel to ensure it's enabled */
	val.val1 = 7;  /* BMP5_OVERSAMPLING_128X for temperature */
	val.val2 = 0;  /* Not used for temperature channel */
	err = sensor_attr_set(bmp581_dev, SENSOR_CHAN_AMBIENT_TEMP,
			      SENSOR_ATTR_OVERSAMPLING, &val);
	if (err) {
		LOG_WRN("Failed to set temperature oversampling: %d", err);
	} else {
		LOG_INF("BMP581 temperature channel explicitly configured");
	}

	/* Set ODR to 10 Hz */
	val.val1 = 0x17;  /* BMP5_ODR_10_HZ */
	val.val2 = 0;
	err = sensor_attr_set(bmp581_dev, SENSOR_CHAN_ALL,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
	if (err) {
		LOG_WRN("Failed to set ODR: %d", err);
	}

	/* Explicitly set to continuous mode (required for continuous operation) */
	val.val1 = BMP5_POWERMODE_CONTINUOUS;
	val.val2 = 0;
	err = sensor_attr_set(bmp581_dev, SENSOR_CHAN_ALL,
			      BMP5_ATTR_POWER_MODE, &val);
	if (err) {
		LOG_WRN("Failed to set continuous mode: %d", err);
	} else {
		LOG_INF("BMP581 set to continuous mode");
	}

	/* Small delay to allow sensor to start continuous operation */
	k_sleep(K_MSEC(100));

	/* Trigger initial sample to verify it's working */
	err = sensor_sample_fetch(bmp581_dev);
	if (err) {
		LOG_WRN("Failed to fetch initial barometer sample: %d", err);
	}

	LOG_INF("BMP581 barometer initialized (10 Hz continuous mode)");
	return 0;
}

static int read_barometer(float *pressure_pa, float *temperature_c)
{
	struct sensor_value press, temp;
	int err;

	err = sensor_sample_fetch(bmp581_dev);
	if (err) {
		return err;
	}

	err = sensor_channel_get(bmp581_dev, SENSOR_CHAN_PRESS, &press);
	if (err) {
		LOG_WRN("Failed to read pressure channel: %d", err);
		return err;
	}

	err = sensor_channel_get(bmp581_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	if (err) {
		LOG_WRN("Failed to read temperature channel: %d, using 0.0", err);
		/* Continue with pressure reading even if temperature fails */
		temp.val1 = 0;
		temp.val2 = 0;
	}

	/* Convert to float - channels are correct, no swap needed */
	*pressure_pa = sensor_value_to_double(&press);
	*temperature_c = sensor_value_to_double(&temp);
	
	/* Validate readings are reasonable */
	if (*pressure_pa < 50000.0f || *pressure_pa > 120000.0f) {
		LOG_WRN("Pressure out of range: %.1f Pa", (double)*pressure_pa);
	}
	if (*temperature_c < -50.0f || *temperature_c > 100.0f) {
		LOG_WRN("Temperature out of range: %.1f C", (double)*temperature_c);
	}

	return 0;
}

static int init_storage(void)
{
	int err;

	LOG_INF("Initializing storage...");

	/* Initialize SD logger */
	err = sd_logger_init();
	if (err) {
		LOG_ERR("SD logger init failed: %d", err);
		return err;
	}

	/* Small delay to ensure SPI is ready */
	k_sleep(K_MSEC(100));

	/* Mount SD card */
	err = sd_logger_mount();
	if (err) {
		LOG_ERR("SD card mount failed: %d", err);
		return err;
	}

	/* Initialize session manager */
	err = session_mgr_init();
	if (err) {
		LOG_ERR("Session manager init failed: %d", err);
		return err;
	}

	return 0;
}

static int start_session(void)
{
	int err;

	LOG_INF("Starting new session...");

	/* Start session (rotates old files, generates new UUID) */
	err = session_mgr_start_session();
	if (err) {
		LOG_ERR("Failed to start session: %d", err);
		return err;
	}

	/* Start flight timer */
	err = flight_timer_start();
	if (err) {
		LOG_ERR("Failed to start flight timer: %d", err);
		return err;
	}

	LOG_INF("Session started: %s", session_mgr_get_uuid());
	return 0;
}

int main(void)
{
	int err;

	LOG_INF("=== Flight Timer for nRF9151 Feather ===");

	/* Initialize work queue for GNSS operations */
	struct k_work_queue_config wq_cfg = {
		.name = "gnss_workq",
		.no_yield = false,
	};
	k_work_queue_start(&gnss_work_q, gnss_workq_stack,
			   K_THREAD_STACK_SIZEOF(gnss_workq_stack),
			   GNSS_WORKQ_PRIORITY, &wq_cfg);

	/* Initialize RTC */
	err = rtc_sync_init();
	if (err) {
		LOG_WRN("RTC init failed: %d (continuing without RTC)", err);
	}

	/* Initialize storage first (needed for logging) */
	err = init_storage();
	if (err) {
		LOG_ERR("Storage init failed: %d", err);
		/* Continue without SD card - will retry later */
	}

	/* Initialize state machines */
	gps_state_init();
	baro_state_init();
	flight_timer_init();

	/* Initialize modem and LTE */
	err = init_modem();
	if (err) {
		LOG_ERR("Modem init failed: %d", err);
		return err;
	}

	/* Initialize A-GNSS work and SUPL assistance */
	k_work_init(&agnss_data_get_work, agnss_data_get_work_fn);

	err = supl_assist_init(&gnss_work_q);
	if (err) {
		LOG_WRN("SUPL init failed: %d (continuing without A-GNSS)", err);
	}

	/* Initialize GNSS */
	err = init_gnss();
	if (err) {
		LOG_ERR("GNSS init failed: %d", err);
		return err;
	}

	/* Initialize barometer */
	err = init_barometer();
	if (err) {
		LOG_ERR("Barometer init failed: %d", err);
		return err;
	}

	/* Start session (if storage is available) */
	if (sd_logger_is_mounted()) {
		err = start_session();
		if (err) {
			LOG_ERR("Failed to start session: %d", err);
	}
	}

	LOG_INF("Initialization complete, entering main loop");

	/* Main loop timing */
	int64_t last_baro_time = 0;
	int64_t last_status_time = 0;
	#define STATUS_UPDATE_INTERVAL_MS 10000  /* 10 seconds */
	
	/* Main loop */
	while (1) {
		int64_t now = k_uptime_get();

		/* Process GPS PVT data if available */
		if (k_sem_take(&pvt_data_sem, K_NO_WAIT) == 0) {
			flight_timer_process_gps(&last_pvt);
			
			/* Log fix status - only log when fix is lost or debug level */
			if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
				LOG_DBG("GPS: lat=%.6f, lon=%.6f, speed=%.1f kts, altitude=%.1f m",
					last_pvt.latitude, last_pvt.longitude,
					(double)(last_pvt.speed * 1.94384f),
					(double)last_pvt.altitude);
			} else {
				LOG_DBG("GPS: no fix (flags=0x%02x, exec_time=%u ms)",
					last_pvt.flags, last_pvt.execution_time);
			}
		}

		/* Sample barometer at 10 Hz */
		if (now - last_baro_time >= BARO_SAMPLE_INTERVAL_MS) {
			float pressure, temperature;
			err = read_barometer(&pressure, &temperature);
			if (err == 0) {
				/* Validate readings are reasonable before logging */
				if (pressure > 0 && pressure < 200000.0f) {  /* Reasonable pressure range */
					/* Temperature might be 0.0 if sensor channel fails, but log anyway */
				flight_timer_process_baro(pressure, temperature);
				} else {
					LOG_WRN("Baro pressure out of range: %.1f Pa", (double)pressure);
				}
			} else {
				LOG_WRN("Baro read failed: %d", err);
			}
			last_baro_time = now;
		}

		/* Status update every 10 seconds */
		if (now - last_status_time >= STATUS_UPDATE_INTERVAL_MS) {
			const struct gps_state_data *gps_data = gps_state_get_data();
			const struct baro_state_data *baro_data = baro_state_get_data();
			int64_t timestamp_ms = rtc_get_timestamp_ms();
			
			/* Format timestamp for display */
			time_t time_sec = timestamp_ms / 1000;
			struct tm *tm_info = gmtime(&time_sec);
			char time_str[32];
			if (tm_info != NULL) {
				strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", tm_info);
			} else {
				snprintf(time_str, sizeof(time_str), "unknown");
			}
			
			LOG_INF("STATUS: time=%s, gps_state=%s, baro_state=%s, "
				"lat=%.6f, lon=%.6f, speed=%.1f kts, "
				"pressure=%.1f Pa, temp=%.1f C",
				time_str,
				gps_state_name(gps_state_get()),
				baro_state_name(baro_state_get()),
				gps_data->latitude,
				gps_data->longitude,
				(double)gps_data->speed_kts,
				(double)baro_data->pressure_pa,
				(double)baro_data->temperature_c);
			
			last_status_time = now;
		}

		/* Periodic session save */
		flight_timer_periodic_update();

		/* Sleep until next iteration */
		k_msleep(MAIN_LOOP_INTERVAL_MS);
	}

	return 0;
}
