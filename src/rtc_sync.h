/*
 * RTC Synchronization Module
 * Manages PCF8523 RTC and syncs with GPS time
 */

#ifndef RTC_SYNC_H
#define RTC_SYNC_H

#include <stdint.h>
#include <stdbool.h>
#include <nrf_modem_gnss.h>

/**
 * @brief Initialize the RTC module
 * @return 0 on success, negative errno on failure
 */
int rtc_sync_init(void);

/**
 * @brief Sync RTC from GPS time
 * @param gps_time Pointer to GPS datetime structure
 * @param flags PVT flags (to check leap second validity)
 * @return 0 on success, negative errno on failure
 */
int rtc_sync_from_gps(const struct nrf_modem_gnss_datetime *gps_time, uint8_t flags);

/**
 * @brief Get current timestamp in milliseconds
 * Uses RTC if available, falls back to uptime
 * @return Unix timestamp in milliseconds
 */
int64_t rtc_get_timestamp_ms(void);

/**
 * @brief Check if RTC has been synced with GPS
 * @return true if synced, false otherwise
 */
bool rtc_is_synced(void);

/**
 * @brief Check if RTC needs initial sync (oscillator stopped or invalid time)
 * @return true if RTC needs sync, false if it has valid time
 */
bool rtc_needs_sync(void);

/**
 * @brief Get FAT filesystem timestamp
 * For use with get_fattime() hook
 * @return FAT timestamp format
 */
uint32_t rtc_get_fattime(void);

#endif /* RTC_SYNC_H */

