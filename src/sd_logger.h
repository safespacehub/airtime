/*
 * SD Card Logger Module
 * CSV data logging and serial log capture to SD card
 */

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "gps_state.h"
#include "baro_state.h"

/**
 * @brief Initialize SD card logger
 * @return 0 on success, negative errno on failure
 */
int sd_logger_init(void);

/**
 * @brief Mount the SD card filesystem
 * @return 0 on success, negative errno on failure
 */
int sd_logger_mount(void);

/**
 * @brief Unmount the SD card filesystem
 * @return 0 on success, negative errno on failure
 */
int sd_logger_unmount(void);

/**
 * @brief Check if SD card is mounted
 * @return true if mounted, false otherwise
 */
bool sd_logger_is_mounted(void);

/* CSV logging now handled via LOG_BACKEND_FS - use LOG_INF with CSV format */

/**
 * @brief Rename a file on SD card
 * @param old_name Current filename
 * @param new_name New filename
 * @return 0 on success, negative errno on failure
 */
int sd_logger_rename(const char *old_name, const char *new_name);

/**
 * @brief Check if a file exists
 * @param filename Filename to check
 * @return true if exists, false otherwise
 */
bool sd_logger_file_exists(const char *filename);

/**
 * @brief Write raw data to a file
 * @param filename Filename
 * @param data Data to write
 * @param len Length of data
 * @return 0 on success, negative errno on failure
 */
int sd_logger_write_file(const char *filename, const void *data, size_t len);

/**
 * @brief Read raw data from a file
 * @param filename Filename
 * @param data Buffer to read into
 * @param len Maximum length to read
 * @return Number of bytes read, or negative errno on failure
 */
int sd_logger_read_file(const char *filename, void *data, size_t len);

#endif /* SD_LOGGER_H */

