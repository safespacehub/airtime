/*
 * Power Management Module
 * Handles USB power detection, battery-powered shutdown with linger period
 */

#include "power_mgr.h"
#include "session_mgr.h"
#include "session_upload.h"
#include "sd_logger.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/mfd/npm1300.h>

/* NPM1300 LED register definitions (from led_npm1300.c) */
#define NPM_LED_BASE 0x0AU
#define NPM_LED_OFFSET_MODE 0x00U
#define NPM_LED_OFFSET_SET  0x03U
#define NPM_LED_OFFSET_CLR  0x04U
#define NPM_LED_HOST 2U

/* NPM1300 Ship mode register definitions */
#define SHIP_BASE 0x0BU
#define SHIP_OFFSET_CONFIG 0x04U
#define SHIP_OFFSET_LPCONFIG 0x06U
#define SHIP_OFFSET_CFGSTROBE 0x01U
#define SHIP_OFFSET_TASKENTERSHIPMODE 0x02U

/* NPM1300 BUCK register definitions */
#define NPM1300_BUCK_BASE 0x04U
#define NPM1300_BUCK_OFFSET_EN_CLR 0x01U
#define NPM1300_BUCK_BUCKCTRL0 0x15U
#define NPM1300_BUCK_STATUS 0x34U
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/drivers/sensor/bmp581_user.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <nrf_modem_gnss.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

LOG_MODULE_REGISTER(power_mgr, CONFIG_LOG_DEFAULT_LEVEL);

/* Configuration */
#define LINGER_DURATION_MS (30 * 1000)  /* 30 seconds */
#define LED_FLASH_INTERVAL_MS 500       /* 500ms on/off */
#define USB_CHECK_INTERVAL_MS 1000      /* Check USB status every 1 second */

/* State tracking */
static enum {
	POWER_STATE_IDLE,
	POWER_STATE_LINGERING,
	POWER_STATE_SHUTTING_DOWN
} power_state = POWER_STATE_IDLE;

static bool usb_connected = true;  /* Assume USB connected at startup */
static int64_t linger_start_time = 0;
static int64_t last_usb_check_time = 0;
static bool shutdown_triggered_by_command = false;  /* Track if shutdown was triggered by serial command */

/* Devices */
static const struct device *pmic = DEVICE_DT_GET(DT_NODELABEL(npm1300_pmic));
static const struct device *charger = DEVICE_DT_GET(DT_NODELABEL(npm1300_charger));
static const struct device *leds = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(npm1300_leds));
static const struct device *console_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/* Work items */
static struct k_work_delayable linger_work;
static struct k_work_delayable led_flash_work;
static struct k_work shutdown_work;

/* Dedicated work queue for shutdown operations (needs larger stack)
 * session_mgr_rotate_session() allocates 200 * 12 bytes = 2400 bytes for state_changes array
 * Plus JSON buffer and other operations, need at least 16KB
 */
#define SHUTDOWN_WORKQ_STACK_SIZE 16384
#define SHUTDOWN_WORKQ_PRIORITY   5
K_THREAD_STACK_DEFINE(shutdown_workq_stack, SHUTDOWN_WORKQ_STACK_SIZE);
static struct k_work_q shutdown_work_q;

/* Semaphore for USB detection events */
static K_SEM_DEFINE(usb_event_sem, 0, 1);

/* LED flash state */
static bool led_flash_state = false;
static bool led_flashing = false;

/* Serial command buffer */
#define SERIAL_CMD_BUFFER_SIZE 32
static char serial_cmd_buffer[SERIAL_CMD_BUFFER_SIZE];
static size_t serial_cmd_pos = 0;

/**
 * @brief USB power detection callback
 */
static void usb_event_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (pins & BIT(NPM1300_EVENT_VBUS_DETECTED)) {
		LOG_INF("USB power detected");
		usb_connected = true;
		k_sem_give(&usb_event_sem);
	}

	if (pins & BIT(NPM1300_EVENT_VBUS_REMOVED)) {
		LOG_INF("USB power removed");
		usb_connected = false;
		k_sem_give(&usb_event_sem);
	}
}

/**
 * @brief Check USB connection status via sensor API
 * @return true if USB is connected, false otherwise
 */
static bool check_usb_status_sensor(void)
{
	if (!device_is_ready(charger)) {
		return false;
	}

	struct sensor_value val;
	int ret = sensor_attr_get(charger, SENSOR_CHAN_CURRENT, SENSOR_ATTR_UPPER_THRESH, &val);

	if (ret < 0) {
		return false;
	}

	/* USB is connected if current threshold is non-zero */
	return (val.val1 != 0) || (val.val2 != 0);
}

/**
 * @brief LED flash work handler
 */
static void led_flash_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!leds || !device_is_ready(leds)) {
		return;
	}

	if (!led_flashing) {
		/* Stop flashing - turn off LED */
		led_off(leds, 2U);
		return;
	}

	/* Toggle LED */
	led_flash_state = !led_flash_state;
	if (led_flash_state) {
		led_on(leds, 2U);
	} else {
		led_off(leds, 2U);
	}

	/* Schedule next flash */
	k_work_reschedule(&led_flash_work, K_MSEC(LED_FLASH_INTERVAL_MS));
}

/**
 * @brief Start LED flashing
 */
static void start_led_flashing(void)
{
	if (!leds || !device_is_ready(leds)) {
		LOG_WRN("LED device not available");
		return;
	}

	/* Turn off solid running LED before starting flash */
	led_off(leds, 2U);
	
	led_flashing = true;
	led_flash_state = false;
	k_work_reschedule(&led_flash_work, K_MSEC(0));  /* Start immediately */
	LOG_INF("LED flashing started");
}

/**
 * @brief Stop LED flashing
 */
static void stop_led_flashing(void)
{
	led_flashing = false;
	k_work_cancel_delayable(&led_flash_work);
	
	if (leds && device_is_ready(leds)) {
		/* If not shutting down, turn running LED back on solid */
		if (power_state != POWER_STATE_SHUTTING_DOWN) {
			led_on(leds, 2U);
		} else {
			/* Shutting down - turn LED off */
			led_off(leds, 2U);
		}
	}
	
	LOG_INF("LED flashing stopped");
}

/* Forward declaration */
static void execute_shutdown_sequence(void);

/**
 * @brief Shutdown work handler (runs on dedicated work queue with larger stack)
 */
static void shutdown_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	execute_shutdown_sequence();
}

/**
 * @brief Execute shutdown sequence
 */
static void execute_shutdown_sequence(void)
{
	int err;

	LOG_INF("=== Beginning shutdown sequence ===");

	/* Stop LED flashing and switch to red LED (LED 0) for shutdown */
	stop_led_flashing();
	
	/* Turn off LED 2 (running indicator) */
	if (leds && device_is_ready(leds)) {
		led_off(leds, 2U);
	}
	
	/* Switch LED 0 to host mode and turn it on (red) */
	if (pmic && device_is_ready(pmic)) {
		/* Change LED 0 mode from "error" to "host" so we can control it */
		int err = mfd_npm1300_reg_write(pmic, NPM_LED_BASE, NPM_LED_OFFSET_MODE + 0U, NPM_LED_HOST);
		if (err == 0) {
			/* Turn on LED 0 (red) */
			err = mfd_npm1300_reg_write(pmic, NPM_LED_BASE, NPM_LED_OFFSET_SET + (0U * 2U), 1U);
			if (err == 0) {
				LOG_INF("Red LED (LED 0) turned on for shutdown");
			} else {
				LOG_WRN("Failed to turn on red LED: %d", err);
			}
		} else {
			LOG_WRN("Failed to switch LED 0 to host mode: %d", err);
		}
	}

	/* Step 0: Stop all running subsystems to prevent conflicts */
	LOG_INF("Stopping subsystems...");
	
	/* Stop GNSS/GPS receiver */
	if (nrf_modem_is_initialized()) {
		err = nrf_modem_gnss_stop();
		if (err != 0 && err != -1) {  /* -1 is NRF_EPERM (already stopped) */
			LOG_WRN("Failed to stop GNSS: %d (continuing)", err);
		} else {
			LOG_INF("GNSS stopped");
		}
	}
	
	/* Small delay to allow subsystems to stop cleanly */
	k_sleep(K_MSEC(200));

	/* Step 1: Mark current session as old (rotate it) */
	if (sd_logger_is_mounted()) {
		LOG_INF("Rotating current session...");
		err = session_mgr_rotate_session();
		if (err != 0) {
			LOG_WRN("Session rotation failed: %d (continuing with shutdown)", err);
		} else {
			LOG_INF("Session rotated successfully");
		}
	}

	/* Step 2: Upload the session */
	LOG_INF("Uploading session...");
	err = session_upload_trigger(false);
	if (err != 0) {
		LOG_WRN("Session upload failed: %d (continuing with shutdown)", err);
	} else {
		LOG_INF("Session uploaded successfully");
	}

	/* Step 3: Power off modem/LTE
	 * Note: Since we're going into SYSTEMOFF mode which completely powers down
	 * the system, we don't strictly need to shut down the modem library.
	 * However, we do it anyway to ensure clean state and allow logs to flush.
	 */
	LOG_INF("Powering off modem...");
	if (nrf_modem_is_initialized()) {
		/* Put modem into offline mode first */
		err = lte_lc_offline();
		if (err != 0) {
			LOG_WRN("Failed to put modem offline: %d (continuing)", err);
		} else {
			LOG_INF("Modem put into offline mode");
			k_sleep(K_MSEC(500));  /* Allow modem to fully enter offline mode */
		}
		
		/* Shutdown modem library completely
		 * Note: This may take some time, but SYSTEMOFF will power everything down anyway
		 */
		LOG_INF("Shutting down modem library...");
		err = nrf_modem_lib_shutdown();
		if (err != 0) {
			LOG_WRN("Failed to shutdown modem library: %d (SYSTEMOFF will power down anyway)", err);
		} else {
			LOG_INF("Modem library shut down");
			/* Give modem time to fully shut down before power-off */
			k_sleep(K_MSEC(500));
		}
	} else {
		LOG_INF("Modem not initialized, skipping shutdown");
	}

	/* Step 4: Put sensors into sleep mode */
	LOG_INF("Putting sensors into sleep mode...");
	const struct device *bmp581_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(bmp581));
	if (bmp581_dev != NULL && device_is_ready(bmp581_dev)) {
		struct sensor_value val;
		val.val1 = BMP5_POWERMODE_STANDBY;
		val.val2 = 0;
		err = sensor_attr_set(bmp581_dev, SENSOR_CHAN_ALL,
				      BMP5_ATTR_POWER_MODE, &val);
		if (err != 0) {
			LOG_WRN("Failed to put BMP581 into standby: %d (continuing)", err);
		} else {
			LOG_INF("BMP581 put into standby mode");
			/* Small delay to allow sensor to enter standby */
			k_sleep(K_MSEC(50));
		}
	}

	/* Step 5: Prepare PMIC for Ship mode
	 * Ship mode disconnects the battery from the system, preventing any
	 * peripherals (LEDs, sensors, etc.) from drawing power.
	 * The system will wake automatically when USB VBUS is connected.
	 */
	LOG_INF("Preparing PMIC for Ship mode...");
	if (pmic && device_is_ready(pmic)) {
		/* Enable pulldown for both Buck regulators to ensure clean shutdown */
		uint8_t reg = 0;
		err = mfd_npm1300_reg_read(pmic, NPM1300_BUCK_BASE, NPM1300_BUCK_BUCKCTRL0, &reg);
		if (err == 0) {
			if ((reg & 0x08) == 0) {
				/* Enable pulldown for both Bucks */
				err = mfd_npm1300_reg_write(pmic, NPM1300_BUCK_BASE, 
							    NPM1300_BUCK_BUCKCTRL0, 0x08 + 0x04);
				if (err != 0) {
					LOG_WRN("Failed to enable Buck pulldown: %d (continuing)", err);
				} else {
					LOG_INF("Buck pulldown enabled");
				}
			}
		} else {
			LOG_WRN("Failed to read Buck control register: %d (continuing)", err);
		}
		
		/* Configure Ship mode settings */
		err = mfd_npm1300_reg_write(pmic, SHIP_BASE, SHIP_OFFSET_CONFIG, 3U);
		if (err != 0) {
			LOG_WRN("Failed to configure Ship mode: %d (continuing)", err);
		}
		
		err = mfd_npm1300_reg_write(pmic, SHIP_BASE, SHIP_OFFSET_LPCONFIG, 0U);
		if (err != 0) {
			LOG_WRN("Failed to configure Ship LP config: %d (continuing)", err);
		}
		
		err = mfd_npm1300_reg_write(pmic, SHIP_BASE, SHIP_OFFSET_CFGSTROBE, 1U);
		if (err != 0) {
			LOG_WRN("Failed to write Ship config strobe: %d (continuing)", err);
		} else {
			LOG_INF("Ship mode configured - system will wake on USB reconnect");
		}
	} else {
		LOG_WRN("PMIC not ready - cannot enter Ship mode");
	}

	/* Step 6: Wait for session operations to complete before unmounting filesystem */
	LOG_INF("Waiting for session operations to complete...");
	
	/* Wait for upload to complete (if in progress) */
	int wait_count = 0;
	while (session_upload_is_in_progress() && wait_count < 30) {
		LOG_DBG("Upload in progress, waiting...");
		k_sleep(K_MSEC(100));
		wait_count++;
	}
	if (session_upload_is_in_progress()) {
		LOG_WRN("Upload still in progress after 3 seconds, proceeding anyway");
	}
	
	/* Wait for filesystem operations to complete */
	err = sd_logger_wait_for_operations_complete(3000);  /* 3 second timeout */
	if (err != 0) {
		LOG_WRN("Filesystem operations still in progress after timeout, proceeding anyway");
	} else {
		LOG_INF("All filesystem operations completed");
	}
	
	/* Small delay to ensure any pending writes are flushed */
	k_sleep(K_MSEC(500));

	/* Step 7: Safely shutdown log_backend_fs before unmounting */
	LOG_INF("Shutting down log backend...");
	const struct log_backend *fs_backend = log_backend_get_by_name("log_backend_fs");
	if (fs_backend != NULL) {
		/* Flush all logs first */
		LOG_PANIC();
		k_sleep(K_MSEC(200));  /* Give time for logs to flush */
		
		/* Deactivate the backend */
		log_backend_deactivate(fs_backend);
		LOG_INF("Log backend deactivated");
	} else {
		LOG_WRN("log_backend_fs not found, skipping deactivation");
	}

	/* Step 8: Unmount filesystem (do this last, after all operations that might need SD card) */
	if (sd_logger_is_mounted()) {
		LOG_INF("Unmounting filesystem...");
		err = sd_logger_unmount();
		if (err != 0) {
			LOG_ERR("Filesystem unmount failed: %d (continuing with shutdown)", err);
		} else {
			LOG_INF("Filesystem unmounted successfully");
		}
	}

	/* Step 9: Enter Ship mode (ultra-low power, battery disconnected from system) */
	LOG_INF("Entering Ship mode (ultra-low power, ~370 nA quiescent current)...");
	/* Note: LOG_PANIC() already called above when shutting down log backend */
	
	/* Turn off LED 0 (red shutdown indicator) before entering Ship mode */
	if (pmic && device_is_ready(pmic)) {
		err = mfd_npm1300_reg_write(pmic, NPM_LED_BASE, NPM_LED_OFFSET_CLR + (0U * 2U), 1U);
		if (err == 0) {
			LOG_INF("Red LED turned off before Ship mode");
		} else {
			LOG_WRN("Failed to turn off red LED: %d (continuing)", err);
		}
		/* Small delay to ensure LED register write completes */
		k_sleep(K_MSEC(50));
	}
	
	/* Small delay to ensure all operations complete */
	k_sleep(K_MSEC(100));
	
	/* Enter Ship mode - this disconnects the battery from the system,
	 * preventing LEDs and other peripherals from drawing power.
	 * Quiescent current is ~370 nA (vs ~800 nA for SYSTEMOFF).
	 * The system will wake automatically when USB VBUS is connected.
	 */
	if (pmic && device_is_ready(pmic)) {
		err = mfd_npm1300_reg_write(pmic, SHIP_BASE, SHIP_OFFSET_TASKENTERSHIPMODE, 1U);
		if (err != 0) {
			LOG_ERR("Failed to enter Ship mode: %d - falling back to sys_poweroff()", err);
			/* Fallback to SYSTEMOFF if Ship mode fails */
			sys_poweroff();
		} else {
			LOG_INF("Ship mode entered - system will wake on USB reconnect");
			/* Give time for Ship mode to take effect */
			k_sleep(K_MSEC(100));
			/* Ship mode should have powered down the system by now */
			/* If we reach here, something went wrong */
			LOG_ERR("Ship mode entered but system still running - entering SYSTEMOFF");
			sys_poweroff();
		}
	} else {
		LOG_WRN("PMIC not ready - falling back to sys_poweroff()");
		sys_poweroff();
	}
	
	/* Should not reach here */
	LOG_ERR("Power-off failed - system may not support power off");
}

/**
 * @brief Linger timer work handler
 */
static void linger_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	/* Check if USB was reconnected (unless shutdown was triggered by command) */
	if (usb_connected && !shutdown_triggered_by_command) {
		LOG_INF("USB reconnected during linger - cancelling shutdown");
		power_state = POWER_STATE_IDLE;
		stop_led_flashing();
		shutdown_triggered_by_command = false;  /* Reset flag */
		return;
	}

	/* USB still disconnected OR shutdown was triggered by command - proceed with shutdown */
	if (shutdown_triggered_by_command) {
		LOG_INF("Linger period expired - proceeding with command-triggered shutdown");
	} else {
		LOG_INF("Linger period expired - USB still disconnected");
	}
	power_state = POWER_STATE_SHUTTING_DOWN;
	/* Submit shutdown sequence to dedicated work queue with larger stack */
	k_work_submit_to_queue(&shutdown_work_q, &shutdown_work);
}

/**
 * @brief Handle USB disconnection
 */
static void handle_usb_disconnected(void)
{
	if (power_state == POWER_STATE_IDLE) {
		LOG_INF("USB disconnected - starting 30 second linger period");
		power_state = POWER_STATE_LINGERING;
		linger_start_time = k_uptime_get();
		/* Only reset flag if this is a real USB disconnect (not command-triggered) */
		/* If shutdown_triggered_by_command is already true, keep it true */
		if (!shutdown_triggered_by_command) {
			/* This is a real USB disconnect, flag stays false */
		}
		
		/* Start LED flashing */
		start_led_flashing();
		
		/* Schedule shutdown after linger period */
		k_work_schedule(&linger_work, K_MSEC(LINGER_DURATION_MS));
	}
}

/**
 * @brief Handle USB reconnection
 */
static void handle_usb_connected(void)
{
	if (power_state == POWER_STATE_LINGERING) {
		LOG_INF("handle_usb_connected() called, shutdown_triggered_by_command = %s", 
			shutdown_triggered_by_command ? "TRUE" : "FALSE");
		if (shutdown_triggered_by_command) {
			/* Shutdown was triggered by command - ignore USB reconnection */
			LOG_INF("USB reconnected during linger, but shutdown was triggered by command - continuing shutdown");
		} else {
			/* Shutdown was triggered by USB disconnect - cancel it */
			LOG_INF("USB reconnected during linger - cancelling shutdown");
			power_state = POWER_STATE_IDLE;
			k_work_cancel_delayable(&linger_work);
			stop_led_flashing();
			/* Turn running LED back on */
			if (leds && device_is_ready(leds)) {
				led_on(leds, 2U);  /* LED 2 = running indicator */
			}
			shutdown_triggered_by_command = false;  /* Reset flag */
		}
	}
	/* If already shutting down, ignore reconnection */
}

int power_mgr_init(void)
{
	int err;

	LOG_INF("Initializing power management module");

	/* Check if PMIC device is ready */
	if (!device_is_ready(pmic)) {
		LOG_ERR("PMIC device not ready");
		return -ENODEV;
	}

	/* Check if charger device is ready */
	if (!device_is_ready(charger)) {
		LOG_WRN("Charger device not ready - USB detection may be limited");
	}

	/* Check LED device */
	if (leds == NULL || !device_is_ready(leds)) {
		LOG_WRN("LED device not available - LED flashing will be disabled");
	} else {
		LOG_INF("LED device ready");
		/* Turn on running LED to indicate device is active
		 * Note: Only LED 2 is in "host" mode and can be controlled.
		 * LED 0 is in "error" mode (PMIC-controlled)
		 * LED 1 is in "charging" mode (PMIC-controlled)
		 */
		int led_err = led_on(leds, 2U);
		if (led_err == 0) {
			LOG_INF("Running LED (LED 2) turned on");
		} else {
			LOG_WRN("Failed to turn on running LED: %d", led_err);
		}
	}

	/* Setup USB detection callback */
	static struct gpio_callback event_cb;
	gpio_init_callback(&event_cb, usb_event_callback,
			   BIT(NPM1300_EVENT_VBUS_DETECTED) |
			   BIT(NPM1300_EVENT_VBUS_REMOVED));
	
	err = mfd_npm1300_add_callback(pmic, &event_cb);
	if (err != 0) {
		LOG_ERR("Failed to register PMIC callback: %d", err);
		return err;
	}

	/* Initialize work items */
	k_work_init_delayable(&linger_work, linger_work_fn);
	k_work_init_delayable(&led_flash_work, led_flash_work_fn);
	k_work_init(&shutdown_work, shutdown_work_fn);
	
	/* Initialize dedicated work queue for shutdown operations */
	struct k_work_queue_config shutdown_wq_cfg = {
		.name = "shutdown_workq",
		.no_yield = false,
	};
	k_work_queue_start(&shutdown_work_q, shutdown_workq_stack,
			   K_THREAD_STACK_SIZEOF(shutdown_workq_stack),
			   SHUTDOWN_WORKQ_PRIORITY, &shutdown_wq_cfg);

	/* Serial command input will be polled in power_mgr_check_serial_commands() */
	if (device_is_ready(console_uart)) {
		LOG_INF("Serial command input enabled (type any command and press Enter to trigger shutdown)");
	} else {
		LOG_WRN("Console UART not ready - serial commands disabled");
	}

	/* Check initial USB status */
	usb_connected = check_usb_status_sensor();
	last_usb_check_time = k_uptime_get();
	
	LOG_INF("Power management initialized (USB: %s)", 
		usb_connected ? "connected" : "disconnected");

	return 0;
}

void power_mgr_periodic_update(void)
{
	int64_t now = k_uptime_get();
	bool current_usb_status;

	/* Process USB event semaphore if available */
	if (k_sem_take(&usb_event_sem, K_NO_WAIT) == 0) {
		/* USB status already updated by callback */
		if (usb_connected) {
			handle_usb_connected();
		} else {
			handle_usb_disconnected();
		}
		return;
	}

	/* Periodic USB status check (fallback if callbacks don't fire) */
	if (now - last_usb_check_time >= USB_CHECK_INTERVAL_MS) {
		current_usb_status = check_usb_status_sensor();
		last_usb_check_time = now;

		/* Detect state changes */
		if (current_usb_status != usb_connected) {
			usb_connected = current_usb_status;
			if (usb_connected) {
				handle_usb_connected();
			} else {
				handle_usb_disconnected();
			}
		}
	}
}

bool power_mgr_is_usb_connected(void)
{
	return usb_connected;
}

bool power_mgr_is_shutting_down(void)
{
	return (power_state == POWER_STATE_LINGERING || 
		power_state == POWER_STATE_SHUTTING_DOWN);
}

int power_mgr_get_battery_state(struct power_mgr_battery_state *state)
{
	if (state == NULL) {
		return -EINVAL;
	}

	/* Initialize state */
	memset(state, 0, sizeof(*state));
	state->valid = false;

	if (!device_is_ready(charger)) {
		return -ENODEV;
	}

	/* Fetch sensor data */
	int err = sensor_sample_fetch(charger);
	if (err != 0) {
		LOG_DBG("Failed to fetch battery sensor data: %d", err);
		return err;
	}

	/* Read battery voltage */
	struct sensor_value voltage;
	err = sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &voltage);
	if (err == 0) {
		/* Voltage is in microvolts, convert to volts */
		state->voltage_v = (float)voltage.val1 + ((float)voltage.val2 / 1000000.0f);
		state->valid = true;
	} else {
		LOG_DBG("Failed to read battery voltage: %d", err);
	}

	/* Read battery current */
	struct sensor_value current;
	err = sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &current);
	if (err == 0) {
		/* Current is in microamps, convert to milliamps */
		state->current_ma = (float)current.val1 + ((float)current.val2 / 1000000.0f);
		state->current_ma /= 1000.0f;  /* Convert to milliamps */
	} else {
		LOG_DBG("Failed to read battery current: %d", err);
	}

	/* Read battery temperature */
	struct sensor_value temp;
	err = sensor_channel_get(charger, SENSOR_CHAN_GAUGE_TEMP, &temp);
	if (err == 0) {
		/* Temperature is in microdegrees, convert to Celsius */
		state->temperature_c = (float)temp.val1 + ((float)temp.val2 / 1000000.0f);
	} else {
		LOG_DBG("Failed to read battery temperature: %d", err);
	}

	/* Get USB connection status */
	state->usb_connected = usb_connected;

	return state->valid ? 0 : -EIO;
}

void power_mgr_trigger_shutdown(void)
{
	LOG_INF("Shutdown triggered manually (serial command)");
	/* Mark as command-triggered FIRST, before calling handle_usb_disconnected() */
	/* This prevents handle_usb_disconnected() from resetting it */
	shutdown_triggered_by_command = true;  /* Mark as command-triggered - will override USB reconnection */
	LOG_INF("shutdown_triggered_by_command set to TRUE");
	/* Simulate USB disconnect */
	usb_connected = false;
	handle_usb_disconnected();
	LOG_INF("After handle_usb_disconnected(), shutdown_triggered_by_command = %s", 
		shutdown_triggered_by_command ? "TRUE" : "FALSE");
}

void power_mgr_check_serial_commands(void)
{
	uint8_t c;
	int ret;
	
	if (!device_is_ready(console_uart)) {
		return;
	}
	
	/* Poll UART for characters (non-blocking) */
	/* Note: Console may consume characters, so we poll frequently in main loop */
	int char_count = 0;
	while (1) {
		ret = uart_poll_in(console_uart, &c);
		if (ret != 0) {
			/* No more characters available */
			if (char_count > 0) {
				LOG_DBG("Processed %d characters from UART", char_count);
			}
			break;
		}
		char_count++;
		
		/* Character received - process it */
		LOG_INF("Received char: 0x%02x ('%c')", c, isprint(c) ? c : '?');
		
		/* Handle backspace/delete */
		if (c == '\b' || c == 0x7F || c == 0x08) {
			if (serial_cmd_pos > 0) {
				serial_cmd_pos--;
				serial_cmd_buffer[serial_cmd_pos] = '\0';
				/* Echo backspace */
				uart_poll_out(console_uart, '\b');
				uart_poll_out(console_uart, ' ');
				uart_poll_out(console_uart, '\b');
			}
			continue;
		}
		
		/* Handle newline/carriage return - process command */
		if (c == '\n' || c == '\r') {
			if (serial_cmd_pos > 0) {
				serial_cmd_buffer[serial_cmd_pos] = '\0';
				
				/* Echo newline */
				uart_poll_out(console_uart, '\r');
				uart_poll_out(console_uart, '\n');
				
				/* Trim whitespace */
				while (serial_cmd_pos > 0 && isspace((unsigned char)serial_cmd_buffer[serial_cmd_pos - 1])) {
					serial_cmd_pos--;
					serial_cmd_buffer[serial_cmd_pos] = '\0';
				}
				
				LOG_INF("Processing command: '%s'", serial_cmd_buffer);
				
				/* Trigger shutdown on any non-empty command */
				if (strlen(serial_cmd_buffer) > 0) {
					LOG_INF("Received command via serial: '%s' - triggering shutdown", serial_cmd_buffer);
					power_mgr_trigger_shutdown();
				} else {
					LOG_INF("Empty command received, ignoring");
				}
				
				/* Reset buffer */
				serial_cmd_pos = 0;
				serial_cmd_buffer[0] = '\0';
			}
			continue;
		}
		
		/* Add character to buffer if there's room */
		if (serial_cmd_pos < (SERIAL_CMD_BUFFER_SIZE - 1) && isprint(c)) {
			serial_cmd_buffer[serial_cmd_pos++] = (char)c;
			serial_cmd_buffer[serial_cmd_pos] = '\0';
			/* Echo character */
			uart_poll_out(console_uart, c);
		}
	}
}

