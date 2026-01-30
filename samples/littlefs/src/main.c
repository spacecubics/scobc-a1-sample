/*
 * Copyright (c) 2026 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>

/*
 * LittleFS configuration
 * We use the "storage_partition" partition define in the Device Tree.
 */
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
static struct fs_mount_t lfs_storage_mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &storage,
	.storage_dev = (void *)FIXED_PARTITION_ID(storage_partition),
	.mnt_point = "/lfs",
};

#define TEST_FILE "/lfs/test.txt"

int main(void)
{
	struct fs_file_t file;
	int ret;
	uint32_t boot_count = 0;
	struct fs_dir_t dir;
	struct fs_dirent entry;

	printf("=== LittleFS Sample ===\n");

	/* 1. Mount */
	ret = fs_mount(&lfs_storage_mnt);
	if (ret < 0) {
		printf("Failed to mount %s: %d\n", lfs_storage_mnt.mnt_point, ret);
		goto end;
	}
	printf("LittleFS mounted at %s\n", lfs_storage_mnt.mnt_point);

	/* 2. Update boot count */
	fs_file_t_init(&file);

	ret = fs_open(&file, TEST_FILE, FS_O_CREATE | FS_O_RDWR);
	if (ret < 0) {
		printf("Failed to open %s: (%d)\n", TEST_FILE, ret);
		goto unmount;
	}

	/* Read current boot count value */
	ret = fs_read(&file, &boot_count, sizeof(boot_count));
	if (ret < 0) {
		printf("Failed to read the boot count in %s: (%d)\n", TEST_FILE, ret);
		goto close;
	}
	printf("%s read count:%u (bytes: %d)\n", TEST_FILE, boot_count, ret);

	ret = fs_seek(&file, 0, FS_SEEK_SET);
	if (ret < 0) {
		printf("Failed to seek in %s: (%d)\n", TEST_FILE, ret);
		goto close;
	}

	boot_count++;
	/* Write new boot count value to the test file */
	ret = fs_write(&file, &boot_count, sizeof(boot_count));
	if (ret < 0) {
		printf("Failed to write the boot count to %s: (%d)\n", TEST_FILE, ret);
		goto close;
	}

	printf("%s write new boot count %u: (bytes: %d)\n", TEST_FILE, boot_count, ret);

	/* 3. List Directory */
	fs_dir_t_init(&dir);
	ret = fs_opendir(&dir, lfs_storage_mnt.mnt_point);
	if (ret) {
		printf("Failed to open dir %s: [%d]\n", lfs_storage_mnt.mnt_point, ret);
		goto close;
	}

	while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != 0) {
		printf("LS: %s (%u bytes)\n", entry.name, entry.size);
	}
	fs_closedir(&dir);

close:
	ret = fs_close(&file);
	if (ret < 0) {
		printf("Failed to close %s: (%d)\n", TEST_FILE, ret);
	}

unmount:
	ret = fs_unmount(&lfs_storage_mnt);
	if (ret < 0) {
		printf("Failed to unmount %s: %d\n", lfs_storage_mnt.mnt_point, ret);
	} else {
		printf("Unmounted.\n");
	}

end:
	return ret;
}
