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

/**
 * @brief List files matching a pattern
 * @param pattern Pattern to match (supports * prefix/suffix, e.g., "old-*.json")
 * @param filenames Output array of matching filenames
 * @param max_files Maximum number of files to return
 * @param found_count Output parameter for number of files found
 * @return 0 on success, negative errno on failure
 */
int sd_logger_list_files(const char *pattern, char filenames[][64], size_t max_files, size_t *found_count);

/**
 * @brief Delete a file from SD card
 * @param filename Filename to delete
 * @return 0 on success, negative errno on failure
 */
int sd_logger_delete_file(const char *filename);

/**
 * @brief Wait for all filesystem operations to complete
 * Attempts to acquire the filesystem mutex to ensure no operations are in progress
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return 0 if filesystem is ready (no operations in progress), negative errno on timeout
 */
int sd_logger_wait_for_operations_complete(int32_t timeout_ms);

#endif /* SD_LOGGER_H */

