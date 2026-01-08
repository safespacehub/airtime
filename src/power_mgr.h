/*
 * Power Management Module
 * Handles USB power detection, battery-powered shutdown with linger period
 */

#ifndef POWER_MGR_H
#define POWER_MGR_H

#include <stdbool.h>

/**
 * @brief Initialize power management module
 * Sets up USB detection callbacks and LED device
 * @return 0 on success, negative errno on failure
 */
int power_mgr_init(void);

/**
 * @brief Periodic update function (call from main loop)
 * Checks USB status and manages linger/shutdown state
 */
void power_mgr_periodic_update(void);

/**
 * @brief Check if USB power is currently connected
 * @return true if USB is connected, false otherwise
 */
bool power_mgr_is_usb_connected(void);

/**
 * @brief Check if system is currently in linger/shutdown state
 * @return true if lingering or shutting down, false otherwise
 */
bool power_mgr_is_shutting_down(void);

/**
 * @brief Battery state information
 */
struct power_mgr_battery_state {
	float voltage_v;        /* Battery voltage in Volts */
	float current_ma;        /* Battery current in milliamps (positive = charging, negative = discharging) */
	float temperature_c;     /* Battery temperature in Celsius */
	bool usb_connected;      /* USB power connected */
	bool valid;              /* True if battery state was successfully read */
};

/**
 * @brief Read current battery state
 * @param state Output parameter for battery state
 * @return 0 on success, negative errno on failure
 */
int power_mgr_get_battery_state(struct power_mgr_battery_state *state);

/**
 * @brief Trigger shutdown sequence manually (for testing via serial command)
 * Simulates USB disconnect to start linger period
 */
void power_mgr_trigger_shutdown(void);

/**
 * @brief Check for serial commands and process them
 * Should be called periodically from main loop
 */
void power_mgr_check_serial_commands(void);

#endif /* POWER_MGR_H */

