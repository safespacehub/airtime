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
#include <zephyr/sys/sys_heap.h>
#include <zephyr/sys/mem_stats.h>
#include <modem/nrf_modem_lib.h>

/* External system heap for memory stats */
extern struct sys_heap _system_heap;

#include <modem/nrf_modem_lib.h>
#include <nrf_modem_gnss.h>
#include <modem/lte_lc.h>
#include <nrf_modem_at.h>
#include <time.h>
#include <stdio.h>

#include "rtc_sync.h"
#include "supl_assist.h"
#include "gps_state.h"
#include "baro_state.h"
#include "sd_logger.h"
#include "session_mgr.h"
#include "flight_timer.h"
#include "session_upload.h"

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

/* LTE reception management based on flight status */
static struct k_work_delayable lte_disable_work;
static flight_status_t prev_flight_status_for_lte = FLIGHT_STATUS_GROUND;
static bool lte_disabled_for_flight = false;
#define LTE_DISABLE_DELAY_MS (60 * 1000)  /* 1 minute delay before disabling LTE */

/* BMP581 device */
static const struct device *bmp581_dev;

/* Configuration */
#define BARO_SAMPLE_INTERVAL_MS  100  /* 10 Hz */
#define MAIN_LOOP_INTERVAL_MS    100

/* Forward declarations */
static void gnss_event_handler(int event);
static void lte_handler(const struct lte_lc_evt *const evt);
static void agnss_data_get_work_fn(struct k_work *item);
static void get_modem_status(char *status_str, size_t status_str_size);
static int gnss_only_mode(void);
static int enable_lte(void);
static void lte_disable_work_fn(struct k_work *work);
static void check_flight_status_for_lte(void);

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

static const char *get_reg_status_string(enum lte_lc_nw_reg_status status)
{
	switch (status) {
	case LTE_LC_NW_REG_NOT_REGISTERED:
		return "NOT_REGISTERED";
	case LTE_LC_NW_REG_REGISTERED_HOME:
		return "REGISTERED_HOME";
	case LTE_LC_NW_REG_SEARCHING:
		return "SEARCHING";
	case LTE_LC_NW_REG_REGISTRATION_DENIED:
		return "REGISTRATION_DENIED";
	case LTE_LC_NW_REG_UNKNOWN:
		return "UNKNOWN";
	case LTE_LC_NW_REG_REGISTERED_ROAMING:
		return "REGISTERED_ROAMING";
	case LTE_LC_NW_REG_UICC_FAIL:
		return "UICC_FAIL";
	default:
		return "UNKNOWN";
	}
}

static const char *get_system_mode_string(enum lte_lc_system_mode mode)
{
	switch (mode) {
	case LTE_LC_SYSTEM_MODE_LTEM:
		return "LTE-M";
	case LTE_LC_SYSTEM_MODE_NBIOT:
		return "NB-IoT";
	case LTE_LC_SYSTEM_MODE_GPS:
		return "GPS";
	case LTE_LC_SYSTEM_MODE_LTEM_GPS:
		return "LTE-M+GPS";
	case LTE_LC_SYSTEM_MODE_NBIOT_GPS:
		return "NB-IoT+GPS";
	case LTE_LC_SYSTEM_MODE_LTEM_NBIOT:
		return "LTE-M+NB-IoT";
	case LTE_LC_SYSTEM_MODE_LTEM_NBIOT_GPS:
		return "LTE-M+NB-IoT+GPS";
	default:
		return "UNKNOWN";
	}
}

static const char *get_func_mode_string(enum lte_lc_func_mode mode)
{
	switch (mode) {
	case LTE_LC_FUNC_MODE_POWER_OFF:
		return "POWER_OFF";
	case LTE_LC_FUNC_MODE_NORMAL:
		return "NORMAL";
	case LTE_LC_FUNC_MODE_RX_ONLY:
		return "RX_ONLY";
	case LTE_LC_FUNC_MODE_OFFLINE:
		return "OFFLINE";
	case LTE_LC_FUNC_MODE_OFFLINE_UICC_ON:
		return "OFFLINE_UICC_ON";
	case LTE_LC_FUNC_MODE_DEACTIVATE_LTE:
		return "DEACTIVATE_LTE";
	case LTE_LC_FUNC_MODE_ACTIVATE_LTE:
		return "ACTIVATE_LTE";
	case LTE_LC_FUNC_MODE_DEACTIVATE_GNSS:
		return "DEACTIVATE_GNSS";
	case LTE_LC_FUNC_MODE_ACTIVATE_GNSS:
		return "ACTIVATE_GNSS";
	case LTE_LC_FUNC_MODE_DEACTIVATE_UICC:
		return "DEACTIVATE_UICC";
	case LTE_LC_FUNC_MODE_ACTIVATE_UICC:
		return "ACTIVATE_UICC";
	default:
		return "UNKNOWN";
	}
}

static void get_modem_status(char *status_str, size_t status_str_size)
{
	int err;
	enum lte_lc_nw_reg_status reg_status;
	enum lte_lc_lte_mode lte_mode = LTE_LC_LTE_MODE_NONE;
	enum lte_lc_system_mode system_mode = LTE_LC_SYSTEM_MODE_LTEM_NBIOT_GPS;
	enum lte_lc_system_mode_preference system_pref = LTE_LC_SYSTEM_MODE_PREFER_AUTO;
	enum lte_lc_func_mode func_mode = LTE_LC_FUNC_MODE_NORMAL;
	char rsrp_str[32] = "N/A";
	char band_str[64] = "N/A";
	char psm_str[64] = "N/A";
	char edrx_str[64] = "N/A";
	char sys_mode_str[64] = "N/A";
	char func_mode_str[32] = "N/A";
	
	/* Get network registration status */
	err = lte_lc_nw_reg_status_get(&reg_status);
	if (err != 0) {
		snprintf(status_str, status_str_size, "reg_status=ERROR(%d)", err);
		return;
	}
	
	/* Get LTE mode */
	err = lte_lc_lte_mode_get(&lte_mode);
	if (err != 0) {
		/* Not critical, continue */
	}
	
	/* Get system mode and preference */
	err = lte_lc_system_mode_get(&system_mode, &system_pref);
	if (err == 0) {
		snprintf(sys_mode_str, sizeof(sys_mode_str), "%s(pref:%d)",
			get_system_mode_string(system_mode), system_pref);
	}
	
	/* Get function mode */
	err = lte_lc_func_mode_get(&func_mode);
	if (err == 0) {
		snprintf(func_mode_str, sizeof(func_mode_str), "%s",
			get_func_mode_string(func_mode));
	}
	
	/* Get PSM status */
	#if defined(CONFIG_LTE_LC_PSM_MODULE)
	int psm_tau = -1, psm_active_time = -1;
	err = lte_lc_psm_get(&psm_tau, &psm_active_time);
	if (err == 0) {
		if (psm_active_time >= 0) {
			/* PSM is active */
			snprintf(psm_str, sizeof(psm_str), "PSM: tau=%ds, active=%ds",
				psm_tau, psm_active_time);
		} else {
			/* PSM is disabled */
			snprintf(psm_str, sizeof(psm_str), "PSM: disabled");
		}
	}
	#endif
	
	/* Get eDRX status */
	#if defined(CONFIG_LTE_LC_EDRX_MODULE)
	struct lte_lc_edrx_cfg edrx_cfg = {0};
	err = lte_lc_edrx_get(&edrx_cfg);
	if (err == 0) {
		if (edrx_cfg.mode != LTE_LC_LTE_MODE_NONE) {
			const char *edrx_mode_str = (edrx_cfg.mode == LTE_LC_LTE_MODE_LTEM) ? "LTE-M" : "NB-IoT";
			snprintf(edrx_str, sizeof(edrx_str), "eDRX: %s, interval=%.2fs, PTW=%.2fs",
				edrx_mode_str, (double)edrx_cfg.edrx, (double)edrx_cfg.ptw);
		} else {
			snprintf(edrx_str, sizeof(edrx_str), "eDRX: disabled");
		}
	}
	#endif
	
	/* Get RSRP (signal strength) via AT command */
	char response[128];
	err = nrf_modem_at_cmd(response, sizeof(response), "AT+CESQ");
	if (err == 0) {
		/* Parse response: +CESQ: <rxlev>,<ber>,<rscp>,<ecno>,<rsrq>,<rsrp> */
		/* Response may include \r\n and OK, so search for +CESQ: */
		const char *cesq_start = strstr(response, "+CESQ:");
		if (cesq_start != NULL) {
			int rxlev = -1, ber = -1, rscp = -1, ecno = -1, rsrq = -1, rsrp = -1;
			if (sscanf(cesq_start, "+CESQ: %d,%d,%d,%d,%d,%d", 
				   &rxlev, &ber, &rscp, &ecno, &rsrq, &rsrp) >= 6) {
				/* RSRP is in index format, convert to dBm */
				/* Index 0-97: RSRP_dBm = index - 140 (0 = -140 dBm, 97 = -44 dBm) */
				/* Index 255: unknown */
				if (rsrp >= 0 && rsrp <= 97) {
					int rsrp_dbm = rsrp - 140;
					snprintf(rsrp_str, sizeof(rsrp_str), "%d dBm", rsrp_dbm);
				} else if (rsrp == 255) {
					snprintf(rsrp_str, sizeof(rsrp_str), "unknown");
				} else {
					snprintf(rsrp_str, sizeof(rsrp_str), "invalid(%d)", rsrp);
				}
			}
		}
	}
	
	/* Get current band via AT command */
	err = nrf_modem_at_cmd(response, sizeof(response), "AT%%XCBAND");
	if (err == 0) {
		/* Parse response: %XCBAND: <band> */
		/* Response may include \r\n and OK, so search for %XCBAND: */
		const char *band_start = strstr(response, "%XCBAND:");
		if (band_start != NULL) {
			int band = -1;
			if (sscanf(band_start, "%%XCBAND: %d", &band) == 1) {
				snprintf(band_str, sizeof(band_str), "Band %d", band);
			} else {
				/* Try to extract band value from response string */
				band_start += 8; /* Skip "%XCBAND: " */
				/* Copy up to newline or end */
				size_t len = 0;
				while (band_start[len] != '\0' && band_start[len] != '\r' && 
				       band_start[len] != '\n' && len < sizeof(band_str) - 1) {
					band_str[len] = band_start[len];
					len++;
				}
				band_str[len] = '\0';
			}
		}
	}
	
	/* Format modem status string */
	const char *lte_mode_str = "N/A";
	switch (lte_mode) {
	case LTE_LC_LTE_MODE_NONE:
		lte_mode_str = "NONE";
		break;
	case LTE_LC_LTE_MODE_LTEM:
		lte_mode_str = "LTE-M";
		break;
	case LTE_LC_LTE_MODE_NBIOT:
		lte_mode_str = "NB-IoT";
		break;
	}
	
	snprintf(status_str, status_str_size,
		"reg=%s, lte_mode=%s, sys_mode=%s, func_mode=%s, rsrp=%s, band=%s, %s, %s",
		get_reg_status_string(reg_status),
		lte_mode_str,
		sys_mode_str,
		func_mode_str,
		rsrp_str,
		band_str,
		psm_str,
		edrx_str);
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

	/* Band lock will be applied after first GPS fix - see gnss_event_handler */

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

static void lte_disable_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	
	/* Verify flight status is still AIRBORNE before disabling LTE */
	flight_status_t current_status = flight_timer_get_status();
	
	if (current_status == FLIGHT_STATUS_AIRBORNE) {
		LOG_INF("Flight status still AIRBORNE after 1 minute - disabling LTE");
		
		/* Disconnect LTE first */
		int err = lte_lc_offline();
		if (err) {
			LOG_WRN("Failed to disconnect LTE: %d", err);
		} else {
			k_sleep(K_MSEC(500));
		}
		
		/* Switch to GNSS-only mode */
		err = gnss_only_mode();
		if (err) {
			LOG_ERR("Failed to switch to GNSS-only mode: %d", err);
		} else {
			lte_disabled_for_flight = true;
			LOG_INF("LTE disabled for flight, GNSS running");
		}
	} else {
		LOG_INF("Flight status returned to GROUND - LTE will remain enabled");
	}
}

static void check_flight_status_for_lte(void)
{
	flight_status_t current_status = flight_timer_get_status();
	
	/* Detect transition from GROUND to AIRBORNE (leaving ground) */
	if (prev_flight_status_for_lte == FLIGHT_STATUS_GROUND &&
	    current_status == FLIGHT_STATUS_AIRBORNE) {
		LOG_INF("Left ground (GROUND -> AIRBORNE) - scheduling LTE disable in 1 minute");
		
		/* Cancel any pending work and schedule new delayed work */
		k_work_cancel_delayable(&lte_disable_work);
		k_work_schedule_for_queue(&gnss_work_q, &lte_disable_work,
					  K_MSEC(LTE_DISABLE_DELAY_MS));
	}
	
	/* Detect transition from AIRBORNE to GROUND (entering ground) */
	if (prev_flight_status_for_lte == FLIGHT_STATUS_AIRBORNE &&
	    current_status == FLIGHT_STATUS_GROUND) {
		LOG_INF("Entered ground (AIRBORNE -> GROUND)");
		
		/* Cancel any pending LTE disable work */
		k_work_cancel_delayable(&lte_disable_work);
		
		/* Re-enable LTE if it was disabled for flight */
		if (lte_disabled_for_flight) {
			LOG_INF("Re-enabling LTE after landing");
			int err = enable_lte();
			if (err) {
				LOG_ERR("Failed to re-enable LTE: %d", err);
			} else {
				lte_disabled_for_flight = false;
				LOG_INF("LTE re-enabled");
			}
		}
	}
	
	/* Update previous status */
	prev_flight_status_for_lte = current_status;
}

static int gnss_only_mode(void)
{
	int err;

	/* After lte_lc_offline(), modem is in flight mode with BOTH LTE and GNSS disabled.
	 * We need to activate GNSS while keeping LTE off.
	 * Use ACTIVATE_GNSS (CFUN=31) to turn on GNSS without enabling LTE.
	 */
	LOG_INF("Activating GNSS while keeping LTE disabled...");
	err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS);
	if (err) {
		LOG_ERR("Failed to activate GNSS: %d", err);
		return err;
	}

	LOG_INF("GNSS activated (LTE remains disabled)");
	
	/* Small delay to ensure GNSS activation is processed */
	k_sleep(K_MSEC(500));
	
	/* Start GNSS to begin getting fixes */
	err = nrf_modem_gnss_start();
	if (err == -13) {
		/* Error -13 (-EACCES) likely means GPS is already running, which is OK */
		LOG_INF("GPS appears to already be running");
		return 0;
	} else if (err) {
		LOG_WRN("GPS start returned error %d (may already be running)", err);
		return err;
	} else {
		LOG_INF("GPS started successfully");
		return 0;
	}
}

static int enable_lte(void)
{
	int err;

	/* Activate LTE while keeping GNSS running.
	 * Use ACTIVATE_LTE (CFUN=21) to turn on LTE without changing GNSS state.
	 */
	LOG_INF("Activating LTE (GNSS will continue running)...");
	err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_LTE);
	if (err) {
		LOG_ERR("Failed to activate LTE: %d", err);
		return err;
	}

	LOG_INF("LTE activated (GNSS remains active)");
	
	/* Small delay to ensure LTE activation is processed */
	k_sleep(K_MSEC(500));
	
	/* Connect to LTE network */
	LOG_INF("Connecting to LTE network...");
	err = lte_lc_connect();
	if (err) {
		LOG_ERR("Failed to connect to LTE network: %d", err);
		return err;
	}

	LOG_INF("LTE connection initiated");
	return 0;
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

	/* Initialize session upload module */
	err = session_upload_init();
	if (err) {
		LOG_WRN("Session upload init failed: %d (continuing without uploads)", err);
	}

	/* Initialize modem and LTE */
	err = init_modem();
	if (err) {
		LOG_ERR("Modem init failed: %d", err);
		return err;
	}

	/* Initialize A-GNSS work and SUPL assistance */
	k_work_init(&agnss_data_get_work, agnss_data_get_work_fn);
	k_work_init_delayable(&lte_disable_work, lte_disable_work_fn);

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

	/* Trigger upload of all sessions at startup (after LTE is connected) */
	if (session_upload_is_lte_ready() && sd_logger_is_mounted()) {
		LOG_INF("Triggering startup upload of all sessions (active + prior)");
		session_upload_all();
	}

	LOG_INF("Initialization complete, entering main loop");
	
	/* Enable system runtime statistics for CPU monitoring */
	#ifdef CONFIG_SCHED_THREAD_USAGE_ALL
	k_sys_runtime_stats_enable();
	#endif

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
			
			/* Check flight status for LTE reception management */
			check_flight_status_for_lte();
			
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
			
			/* GPS fix status information - use last_pvt that's already updated by event handler */
			char gps_fix_status[1024] = "no_data";
			uint8_t tracked_sv = 0;
			uint8_t used_sv = 0;
			
			/* Use last_pvt data that's already being updated by the event handler */
			/* Count tracked and used satellites */
			for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++) {
				if (last_pvt.sv[i].sv == 0) {
					break;
				}
				tracked_sv++;
				if (last_pvt.sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX) {
					used_sv++;
				}
			}
			
			/* Build status string with flags */
			char flags_str[128] = "";
			size_t flags_len = 0;
			
			if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
				flags_len += snprintf(flags_str + flags_len, sizeof(flags_str) - flags_len, "FIX_VALID");
			} else {
				flags_len += snprintf(flags_str + flags_len, sizeof(flags_str) - flags_len, "NO_FIX");
			}
			
			/* Add additional flag indicators */
			if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_DEADLINE_MISSED) {
				flags_len += snprintf(flags_str + flags_len, sizeof(flags_str) - flags_len, "|DEADLINE_MISSED");
			}
			if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME) {
				flags_len += snprintf(flags_str + flags_len, sizeof(flags_str) - flags_len, "|INSUFFICIENT_TIME");
			}
			if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_SLEEP_BETWEEN_PVT) {
				flags_len += snprintf(flags_str + flags_len, sizeof(flags_str) - flags_len, "|SLEPT");
			}
			if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_VELOCITY_VALID) {
				flags_len += snprintf(flags_str + flags_len, sizeof(flags_str) - flags_len, "|VEL_VALID");
			}
			
			/* Format last fix time and age */
			char last_fix_time_str[64] = "never";
			if (gps_data->last_fix_time_ms > 0) {
				int64_t fix_age_ms = now - gps_data->last_fix_time_ms;
				int64_t fix_age_sec = fix_age_ms / 1000;
				
				/* Convert last fix time to absolute time if RTC is available */
				int64_t fix_timestamp_ms = rtc_get_timestamp_ms() - fix_age_ms;
				time_t fix_time_sec = fix_timestamp_ms / 1000;
				struct tm *fix_tm_info = gmtime(&fix_time_sec);
				
				if (fix_tm_info != NULL && fix_timestamp_ms > 0) {
					char fix_time_buf[32];
					strftime(fix_time_buf, sizeof(fix_time_buf), "%H:%M:%S", fix_tm_info);
					snprintf(last_fix_time_str, sizeof(last_fix_time_str), 
						"%s (%llds ago)", fix_time_buf, (long long)fix_age_sec);
				} else {
					/* Fallback to just age if RTC not available */
					snprintf(last_fix_time_str, sizeof(last_fix_time_str), 
						"%llds ago", (long long)fix_age_sec);
				}
			}
			
			snprintf(gps_fix_status, sizeof(gps_fix_status),
				"fix=%s, sv_tracked=%u, sv_used=%u, exec_time=%u ms, "
				"hdop=%.1f, accuracy=%.1f m, flags=0x%02x (%s), last_fix=%s",
				(last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) ? "YES" : "NO",
				tracked_sv, used_sv, last_pvt.execution_time,
				(double)last_pvt.hdop, (double)last_pvt.accuracy,
				last_pvt.flags, flags_str, last_fix_time_str);
			
			/* Show fix count when in UNKNOWN state to explain why state hasn't changed */
			gps_state_t current_gps_state = gps_state_get();
			const char *gps_state_str = gps_state_name(current_gps_state);
			char gps_state_with_count[64];
			
			if (current_gps_state == GPS_STATE_UNKNOWN && 
			    (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID)) {
				/* Show fix count when we have a valid fix but state is still UNKNOWN */
				uint32_t fix_count = gps_state_get_fix_count();
				snprintf(gps_state_with_count, sizeof(gps_state_with_count), 
					"%s (fixes=%u/%d)", gps_state_str, fix_count, GPS_STATE_MIN_FIXES_FOR_STATE_CHANGE);
				gps_state_str = gps_state_with_count;
			}
			
			/* Get modem status */
			char modem_status[512] = "N/A";
			if (nrf_modem_is_initialized()) {
				get_modem_status(modem_status, sizeof(modem_status));
			}
			
			LOG_INF("STATUS: time=%s, gps_state=%s, baro_state=%s, "
				"lat=%.6f, lon=%.6f, speed=%.1f kts, "
				"pressure=%.1f Pa, temp=%.1f C, mem=%s, cpu=%s",
				time_str,
				gps_state_str,
				baro_state_name(baro_state_get()),
				gps_data->latitude,
				gps_data->longitude,
				(double)gps_data->speed_kts,
				(double)baro_data->pressure_pa,
				(double)baro_data->temperature_c,
				mem_info,
				cpu_info);
			
			LOG_INF("GPS_FIX_STATUS: %s", gps_fix_status);
			LOG_INF("MODEM_STATUS: %s", modem_status);
			
			last_status_time = now;
		}

		/* Periodic session save */
		flight_timer_periodic_update();

		/* Periodic session upload */
		session_upload_periodic_update();

		/* Sleep until next iteration */
		k_msleep(MAIN_LOOP_INTERVAL_MS);
	}

	return 0;
}
