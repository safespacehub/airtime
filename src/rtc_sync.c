/*
 * RTC Synchronization Module
 * Manages PCF8523 RTC and syncs with GPS time
 */

#include "rtc_sync.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/logging/log.h>
#include <time.h>

LOG_MODULE_REGISTER(rtc_sync, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *rtc_dev;
static bool rtc_synced = false;
static int64_t last_sync_uptime_ms = 0;

int rtc_sync_init(void)
{
	rtc_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(pcf8523));
	
	if (rtc_dev == NULL) {
		LOG_ERR("PCF8523 RTC not found in devicetree");
		return -ENODEV;
	}

	if (!device_is_ready(rtc_dev)) {
		LOG_ERR("PCF8523 RTC device not ready");
		return -ENODEV;
	}

	LOG_INF("RTC initialized (PCF8523)");
	
	/* Check if RTC has valid time (battery backup) */
	struct rtc_time rtc_tm;
	int ret = rtc_get_time(rtc_dev, &rtc_tm);
	if (ret == 0 && rtc_tm.tm_year > 120) {  /* After 2020 */
		LOG_INF("RTC has valid time: %04d-%02d-%02d %02d:%02d:%02d",
			rtc_tm.tm_year + 1900, rtc_tm.tm_mon + 1, rtc_tm.tm_mday,
			rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec);
		
		/* Set system clock from RTC */
		struct tm tm = {
			.tm_sec = rtc_tm.tm_sec,
			.tm_min = rtc_tm.tm_min,
			.tm_hour = rtc_tm.tm_hour,
			.tm_mday = rtc_tm.tm_mday,
			.tm_mon = rtc_tm.tm_mon,
			.tm_year = rtc_tm.tm_year,
			.tm_wday = -1,
			.tm_yday = -1,
			.tm_isdst = -1,
		};
		
		time_t epoch = mktime(&tm);
		if (epoch != (time_t)-1) {
			struct timespec ts;
			ts.tv_sec = epoch;
			ts.tv_nsec = 0;
			
			ret = clock_settime(CLOCK_REALTIME, &ts);
			if (ret == 0) {
				LOG_INF("System clock set from RTC");
				rtc_synced = true;
			} else {
				LOG_WRN("Failed to set system clock from RTC: %d", ret);
			}
		} else {
			LOG_WRN("Failed to convert RTC time to epoch");
		}
	} else if (ret == -ENODATA) {
		/* Oscillator stop flag set - normal on first boot, will clear when GPS syncs */
		LOG_INF("RTC oscillator was stopped (normal on first boot), waiting for GPS sync");
	} else {
		LOG_WRN("RTC time invalid or not set, waiting for GPS sync");
	}

	return 0;
}

int rtc_sync_from_gps(const struct nrf_modem_gnss_datetime *gps_time, uint8_t flags)
{
	if (rtc_dev == NULL || !device_is_ready(rtc_dev)) {
		return -ENODEV;
	}

	/* Get current RTC time to check if oscillator is stopped */
	struct rtc_time current_tm;
	int ret = rtc_get_time(rtc_dev, &current_tm);
	bool oscillator_stopped = (ret == -ENODATA);
	
	/* If oscillator is stopped, sync immediately (first boot) */
	if (oscillator_stopped) {
		LOG_INF("RTC oscillator stopped, syncing from GPS (first boot)");
		/* Continue to set time below */
	} else if (!(flags & NRF_MODEM_GNSS_PVT_FLAG_LEAP_SECOND_VALID)) {
		/* If already synced, require leap second for accuracy */
		if (rtc_synced) {
			LOG_DBG("GPS leap second not valid, skipping RTC sync");
			return -EAGAIN;
		}
		/* If not synced yet, sync anyway (better than nothing) */
		LOG_INF("GPS leap second not valid, but syncing anyway (first sync)");
	}
	
	if (ret == 0 && rtc_synced && !oscillator_stopped) {
		/* Calculate difference - only sync if > 1 second off */
		struct tm gps_tm = {
			.tm_sec = gps_time->seconds,
			.tm_min = gps_time->minute,
			.tm_hour = gps_time->hour,
			.tm_mday = gps_time->day,
			.tm_mon = gps_time->month - 1,
			.tm_year = gps_time->year - 1900,
		};
		
		struct tm rtc_tm_struct = {
			.tm_sec = current_tm.tm_sec,
			.tm_min = current_tm.tm_min,
			.tm_hour = current_tm.tm_hour,
			.tm_mday = current_tm.tm_mday,
			.tm_mon = current_tm.tm_mon,
			.tm_year = current_tm.tm_year,
		};
		
		time_t gps_epoch = mktime(&gps_tm);
		time_t rtc_epoch = mktime(&rtc_tm_struct);
		int64_t delta = (int64_t)gps_epoch - (int64_t)rtc_epoch;
		
		if (delta >= -1 && delta <= 1) {
			LOG_DBG("RTC already synced (delta: %lld s)", delta);
			return 0;
		}
		
		LOG_INF("RTC drift detected: %lld seconds, resyncing", delta);
	}

	/* Set RTC from GPS time */
	struct rtc_time new_tm = {
		.tm_sec = gps_time->seconds,
		.tm_min = gps_time->minute,
		.tm_hour = gps_time->hour,
		.tm_mday = gps_time->day,
		.tm_mon = gps_time->month - 1,  /* RTC uses 0-11 */
		.tm_year = gps_time->year - 1900,
		.tm_wday = -1,  /* Let driver calculate */
		.tm_yday = -1,
	};

	ret = rtc_set_time(rtc_dev, &new_tm);
	if (ret != 0) {
		LOG_ERR("Failed to set RTC time: %d", ret);
		return ret;
	}

	/* Also set system clock (CLOCK_REALTIME) for logging timestamps */
	struct tm tm = {
		.tm_sec = gps_time->seconds,
		.tm_min = gps_time->minute,
		.tm_hour = gps_time->hour,
		.tm_mday = gps_time->day,
		.tm_mon = gps_time->month - 1,
		.tm_year = gps_time->year - 1900,
		.tm_wday = -1,
		.tm_yday = -1,
		.tm_isdst = -1,
	};

	time_t epoch = mktime(&tm);
	if (epoch != (time_t)-1) {
		struct timespec ts;
		ts.tv_sec = epoch;
		ts.tv_nsec = 0;
		
		ret = clock_settime(CLOCK_REALTIME, &ts);
		if (ret != 0) {
			LOG_WRN("Failed to set system clock: %d", ret);
		} else {
			LOG_DBG("System clock set to match RTC");
		}
	} else {
		LOG_WRN("Failed to convert GPS time to epoch");
	}

	rtc_synced = true;
	last_sync_uptime_ms = k_uptime_get();

	LOG_INF("RTC synced to GPS: %04d-%02d-%02d %02d:%02d:%02d",
		gps_time->year, gps_time->month, gps_time->day,
		gps_time->hour, gps_time->minute, gps_time->seconds);

	return 0;
}

int64_t rtc_get_timestamp_ms(void)
{
	if (rtc_dev == NULL || !device_is_ready(rtc_dev)) {
		return k_uptime_get();
	}

	struct rtc_time rtc_tm;
	int ret = rtc_get_time(rtc_dev, &rtc_tm);
	
	if (ret != 0) {
		LOG_WRN("Failed to read RTC, using uptime");
		return k_uptime_get();
	}

	/* Convert to Unix timestamp */
	struct tm tm = {
		.tm_sec = rtc_tm.tm_sec,
		.tm_min = rtc_tm.tm_min,
		.tm_hour = rtc_tm.tm_hour,
		.tm_mday = rtc_tm.tm_mday,
		.tm_mon = rtc_tm.tm_mon,
		.tm_year = rtc_tm.tm_year,
	};

	time_t epoch = mktime(&tm);
	if (epoch == (time_t)-1) {
		LOG_WRN("Failed to convert RTC time, using uptime");
		return k_uptime_get();
	}

	return (int64_t)epoch * 1000;
}

bool rtc_is_synced(void)
{
	return rtc_synced;
}

bool rtc_needs_sync(void)
{
	if (rtc_dev == NULL || !device_is_ready(rtc_dev)) {
		return true;  /* Need sync if RTC not available */
	}

	struct rtc_time rtc_tm;
	int ret = rtc_get_time(rtc_dev, &rtc_tm);
	
	/* If we can't read time or oscillator is stopped, need sync */
	if (ret != 0) {
		return true;
	}

	/* If time is before 2020, it's likely invalid */
	if (rtc_tm.tm_year < 120) {
		return true;
	}

	/* RTC has valid time, no sync needed */
	return false;
}

uint32_t rtc_get_fattime(void)
{
	if (rtc_dev == NULL || !device_is_ready(rtc_dev)) {
		/* Return a default timestamp: 2024-01-01 00:00:00 */
		return ((2024 - 1980) << 25) | (1 << 21) | (1 << 16);
	}

	struct rtc_time rtc_tm;
	int ret = rtc_get_time(rtc_dev, &rtc_tm);
	
	if (ret != 0) {
		return ((2024 - 1980) << 25) | (1 << 21) | (1 << 16);
	}

	/* FAT timestamp format:
	 * bits 31-25: Year from 1980 (0-127)
	 * bits 24-21: Month (1-12)
	 * bits 20-16: Day (1-31)
	 * bits 15-11: Hour (0-23)
	 * bits 10-5:  Minute (0-59)
	 * bits 4-0:   Second/2 (0-29)
	 */
	uint32_t year = (rtc_tm.tm_year + 1900 - 1980) & 0x7F;
	uint32_t month = (rtc_tm.tm_mon + 1) & 0x0F;
	uint32_t day = rtc_tm.tm_mday & 0x1F;
	uint32_t hour = rtc_tm.tm_hour & 0x1F;
	uint32_t minute = rtc_tm.tm_min & 0x3F;
	uint32_t second = (rtc_tm.tm_sec / 2) & 0x1F;

	return (year << 25) | (month << 21) | (day << 16) |
	       (hour << 11) | (minute << 5) | second;
}

/* Hook for FatFS timestamp */
__weak uint32_t get_fattime(void)
{
	return rtc_get_fattime();
}

