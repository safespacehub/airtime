/*
 * SD Card Logger Module
 * CSV data logging and serial log capture to SD card
 * 
 * Based on Zephyr fs_sample pattern for reliable SD card handling
 */

#include "sd_logger.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/hwinfo.h>
#include <ff.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

LOG_MODULE_REGISTER(sd_logger, CONFIG_LOG_DEFAULT_LEVEL);

#define SD_MOUNT_POINT   "/SD:"
#define DISK_NAME        "SD"
#define MAX_PATH_LEN     64
#define FS_RET_OK        FR_OK

/* FATFS mount structure - following fs_sample pattern */
static FATFS fat_fs;
static struct fs_mount_t fs_mnt = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = SD_MOUNT_POINT,
};

/* State */
static bool mounted = false;

/* Thread safety */
static struct k_mutex sd_logger_mutex;

int sd_logger_init(void)
{
	k_mutex_init(&sd_logger_mutex);
	
	LOG_INF("SD logger initialized");
	return 0;
}

int sd_logger_mount(void)
{
	int ret;
	uint32_t block_count;
	uint32_t block_size;
	
	if (mounted) {
		return 0;
	}

	LOG_INF("Initializing SD card: %s", DISK_NAME);
	
	/* Initialize disk - following fs_sample pattern */
	ret = disk_access_ioctl(DISK_NAME, DISK_IOCTL_CTRL_INIT, NULL);
	if (ret) {
		LOG_ERR("Storage init failed: %d", ret);
		return ret;
	}

	/* Get sector count */
	ret = disk_access_ioctl(DISK_NAME, DISK_IOCTL_GET_SECTOR_COUNT, &block_count);
	if (ret) {
		LOG_ERR("Unable to get sector count: %d", ret);
		disk_access_ioctl(DISK_NAME, DISK_IOCTL_CTRL_DEINIT, NULL);
		return ret;
	}
	LOG_INF("Block count: %u", block_count);

	/* Get sector size */
	ret = disk_access_ioctl(DISK_NAME, DISK_IOCTL_GET_SECTOR_SIZE, &block_size);
	if (ret) {
		LOG_ERR("Unable to get sector size: %d", ret);
		disk_access_ioctl(DISK_NAME, DISK_IOCTL_CTRL_DEINIT, NULL);
		return ret;
	}
	LOG_INF("Sector size: %u bytes", block_size);

	/* Mount filesystem - card must be pre-formatted with FAT */
	LOG_INF("Mounting FAT filesystem at %s", SD_MOUNT_POINT);
	ret = fs_mount(&fs_mnt);
	if (ret) {
		LOG_ERR("Error mounting disk: %d", ret);
		if (ret == -ENOENT || ret == -EIO) {
			LOG_ERR("SD card may not be formatted or has wrong filesystem");
			/* Try to unmount and remount - following fs_sample pattern */
			LOG_INF("Attempting to unmount and remount...");
			int unmount_ret = fs_unmount(&fs_mnt);
			if (unmount_ret == 0) {
				LOG_INF("Unmounted successfully, retrying mount...");
				ret = fs_mount(&fs_mnt);
				if (ret == 0) {
					LOG_INF("Remount successful");
				} else {
					LOG_ERR("Remount failed: %d", ret);
					disk_access_ioctl(DISK_NAME, DISK_IOCTL_CTRL_DEINIT, NULL);
					return ret;
				}
			} else {
				LOG_ERR("Unmount failed: %d", unmount_ret);
				disk_access_ioctl(DISK_NAME, DISK_IOCTL_CTRL_DEINIT, NULL);
				return ret;
			}
		} else {
			disk_access_ioctl(DISK_NAME, DISK_IOCTL_CTRL_DEINIT, NULL);
			return ret;
		}
	}

	mounted = true;
	LOG_INF("SD card mounted successfully");

	/* Print hardware ID */
	uint8_t device_id[8];
	ssize_t id_ret = hwinfo_get_device_id(device_id, sizeof(device_id));
	if (id_ret == sizeof(device_id)) {
		LOG_INF("Hardware ID: %02X%02X%02X%02X%02X%02X%02X%02X",
			device_id[0], device_id[1], device_id[2], device_id[3],
			device_id[4], device_id[5], device_id[6], device_id[7]);
	} else {
		LOG_WRN("Failed to get hardware ID: %zd", id_ret);
	}

	return 0;
}

int sd_logger_unmount(void)
{
	if (!mounted) {
		return 0;
	}

	/* No files to close - CSV logging now via LOG_BACKEND_FS */

	/* Unmount filesystem */
	int ret = fs_unmount(&fs_mnt);
	if (ret) {
		LOG_ERR("Failed to unmount: %d", ret);
		return ret;
	}

	/* Deinitialize disk */
	ret = disk_access_ioctl(DISK_NAME, DISK_IOCTL_CTRL_DEINIT, NULL);
	if (ret) {
		LOG_ERR("Storage deinit failed: %d", ret);
	}

	mounted = false;
	LOG_INF("SD card unmounted");
	return 0;
}

bool sd_logger_is_mounted(void)
{
	return mounted;
}

/* CSV logging now handled via LOG_BACKEND_FS - no separate file operations needed */

int sd_logger_rename(const char *old_name, const char *new_name)
{
	if (!mounted) {
		return -ENODEV;
	}

	/* Protect filesystem operations with mutex to avoid conflicts with LOG_BACKEND_FS */
	if (k_mutex_lock(&sd_logger_mutex, K_FOREVER) != 0) {
		return -EAGAIN;
	}

	char old_path[MAX_PATH_LEN];
	char new_path[MAX_PATH_LEN];
	
	snprintf(old_path, sizeof(old_path), "%s/%s", SD_MOUNT_POINT, old_name);
	snprintf(new_path, sizeof(new_path), "%s/%s", SD_MOUNT_POINT, new_name);

	int ret = fs_rename(old_path, new_path);
	if (ret == 0) {
		LOG_INF("Renamed %s -> %s", old_name, new_name);
	} else if (ret != -ENOENT) {
		LOG_ERR("Failed to rename %s to %s: %d", old_name, new_name, ret);
	}
	/* -ENOENT means file doesn't exist, which is fine */
	
	k_mutex_unlock(&sd_logger_mutex);
	return ret;
}

bool sd_logger_file_exists(const char *filename)
{
	if (!mounted) {
		return false;
	}

	/* Protect filesystem operations with mutex to avoid conflicts with LOG_BACKEND_FS */
	if (k_mutex_lock(&sd_logger_mutex, K_FOREVER) != 0) {
		return false;
	}

	char path[MAX_PATH_LEN];
	snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, filename);

	struct fs_dirent entry;
	bool exists = fs_stat(path, &entry) == 0;

	k_mutex_unlock(&sd_logger_mutex);
	return exists;
}

int sd_logger_write_file(const char *filename, const void *data, size_t len)
{
	if (!mounted || data == NULL) {
		return -EINVAL;
	}

	/* Protect filesystem operations with mutex to avoid conflicts with LOG_BACKEND_FS */
	if (k_mutex_lock(&sd_logger_mutex, K_FOREVER) != 0) {
		return -EAGAIN;
	}

	char path[MAX_PATH_LEN];
	snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, filename);

	struct fs_file_t file;
	fs_file_t_init(&file);

	int ret = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (ret) {
		LOG_ERR("Failed to open file for write: %d", ret);
		k_mutex_unlock(&sd_logger_mutex);
		return ret;
	}

	ssize_t written = fs_write(&file, data, len);
	if (written < 0) {
		LOG_ERR("Failed to write file: %zd", written);
		fs_close(&file);
		k_mutex_unlock(&sd_logger_mutex);
		return (int)written;
	}

	if (written != (ssize_t)len) {
		LOG_WRN("Partial write: %zd of %zu bytes", written, len);
	}

	/* Sync to ensure data is written */
	ret = fs_sync(&file);
	if (ret) {
		LOG_WRN("Failed to sync file: %d", ret);
	}

	fs_close(&file);
	k_mutex_unlock(&sd_logger_mutex);
	return 0;
}

int sd_logger_read_file(const char *filename, void *data, size_t len)
{
	if (!mounted || data == NULL) {
		return -EINVAL;
	}

	/* Protect filesystem operations with mutex to avoid conflicts with LOG_BACKEND_FS */
	if (k_mutex_lock(&sd_logger_mutex, K_FOREVER) != 0) {
		return -EAGAIN;
	}

	char path[MAX_PATH_LEN];
	snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, filename);

	struct fs_file_t file;
	fs_file_t_init(&file);

	int ret = fs_open(&file, path, FS_O_READ);
	if (ret) {
		k_mutex_unlock(&sd_logger_mutex);
		return ret;
	}

	ssize_t bytes_read = fs_read(&file, data, len);
	fs_close(&file);

	k_mutex_unlock(&sd_logger_mutex);
	return (int)bytes_read;
}
