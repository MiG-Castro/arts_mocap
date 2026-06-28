/**
 * @file lsm6dsv16x_example.c
 * @brief Complete usage example for LSM6DSV16X driver in Zephyr
 * @date 2025
 * 
 * This example demonstrates:
 * 1. Sensor initialization
 * 2. Accelerometer and gyroscope configuration
 * 3. Filter configuration
 * 4. SFLP (Sensor Fusion) enable
 * 5. Reading direct register data
 * 6. Reading FIFO data with quaternions
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "lsm6dsv16x.h"

LOG_MODULE_REGISTER(example, LOG_LEVEL_INF);

/* Device tree node for I2C bus */
#define I2C_NODE DT_NODELABEL(i2c1)

/* Global configuration */
static lsm6dsv16x_config_t imu_config;

/**
 * @brief Initialize IMU sensor
 */
static int imu_init(void)
{
	int ret;
	
	/* Get I2C device */
	imu_config.i2c_dev = DEVICE_DT_GET(I2C_NODE);
	if (!device_is_ready(imu_config.i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}
	
	/* Set I2C address (0x6B when SA0 = VDD) */
	imu_config.i2c_addr = LSM6DSV16X_I2C_ADDR_HIGH;
	
	/* Initialize sensor */
	ret = lsm6dsv16x_init(&imu_config);
	if (ret < 0) {
		LOG_ERR("Failed to initialize sensor: %d", ret);
		return ret;
	}
	
	/* Configure accelerometer: 120 Hz, ±4g */
	ret = lsm6dsv16x_accel_config(&imu_config,
	                               LSM6DSV16X_ACCEL_ODR_120_HZ,
	                               LSM6DSV16X_ACCEL_FS_4G);
	if (ret < 0) {
		LOG_ERR("Failed to configure accelerometer: %d", ret);
		return ret;
	}
	
	/* Configure gyroscope: 120 Hz, ±2000 dps */
	ret = lsm6dsv16x_gyro_config(&imu_config,
	                              LSM6DSV16X_GYRO_ODR_120_HZ,
	                              LSM6DSV16X_GYRO_FS_2000_DPS);
	if (ret < 0) {
		LOG_ERR("Failed to configure gyroscope: %d", ret);
		return ret;
	}
	
	/* Configure accelerometer filter: LPF2 enabled, MEDIUM bandwidth */
	ret = lsm6dsv16x_accel_lpf_config(&imu_config, true,
	                                   LSM6DSV16X_ACCEL_LPF_MEDIUM);
	if (ret < 0) {
		LOG_ERR("Failed to configure accel filter: %d", ret);
		return ret;
	}
	
	/* Configure gyroscope filter: LPF1 enabled, MEDIUM bandwidth */
	ret = lsm6dsv16x_gyro_lpf_config(&imu_config, true,
	                                  LSM6DSV16X_GYRO_LPF_MEDIUM);
	if (ret < 0) {
		LOG_ERR("Failed to configure gyro filter: %d", ret);
		return ret;
	}
	
	LOG_INF("IMU initialized successfully");
	
	/* Store configuration in global config */
	imu_config.accel_fs = LSM6DSV16X_ACCEL_FS_4G;
	imu_config.gyro_fs = LSM6DSV16X_GYRO_FS_2000_DPS;
	imu_config.sflp_enabled = false;
	
	return 0;
}

/**
 * @brief Read and print direct register data
 */
static void read_direct_data(void)
{
	lsm6dsv16x_accel_raw_t accel_raw;
	lsm6dsv16x_gyro_raw_t gyro_raw;
	lsm6dsv16x_accel_mg_t accel_mg;
	lsm6dsv16x_gyro_mdps_t gyro_mdps;
	int ret;
	
	/* Read accelerometer */
	ret = lsm6dsv16x_accel_read_raw(&imu_config, &accel_raw);
	if (ret < 0) {
		LOG_ERR("Failed to read accel: %d", ret);
		return;
	}
	
	/* Read gyroscope */
	ret = lsm6dsv16x_gyro_read_raw(&imu_config, &gyro_raw);
	if (ret < 0) {
		LOG_ERR("Failed to read gyro: %d", ret);
		return;
	}
	
	/* Convert to physical units */
	lsm6dsv16x_accel_raw_to_mg(&accel_raw, imu_config.accel_fs, &accel_mg);
	lsm6dsv16x_gyro_raw_to_mdps(&gyro_raw, imu_config.gyro_fs, &gyro_mdps);
	
	/* Print data */
	LOG_INF("Accel: X=%6.2f Y=%6.2f Z=%6.2f mg",
	        (double)accel_mg.x, (double)accel_mg.y, (double)accel_mg.z);
	LOG_INF("Gyro:  X=%7.1f Y=%7.1f Z=%7.1f mdps",
	        (double)gyro_mdps.x, (double)gyro_mdps.y, (double)gyro_mdps.z);
}

/**
 * @brief Enable SFLP and configure for quaternion output
 */
static int enable_sflp(void)
{
	int ret;
	
	LOG_INF("Enabling SFLP...");
	
	/* Enable SFLP with 120 Hz ODR */
	ret = lsm6dsv16x_sflp_enable(&imu_config, LSM6DSV16X_SFLP_ODR_120_HZ);
	if (ret < 0) {
		LOG_ERR("Failed to enable SFLP: %d", ret);
		return ret;
	}
	
	imu_config.sflp_enabled = true;
	
	LOG_INF("SFLP enabled, quaternions available in FIFO");
	
	return 0;
}

/**
 * @brief Read and process FIFO data
 */
static void read_fifo_data(void)
{
	uint16_t fifo_count;
	lsm6dsv16x_fifo_sample_t sample;
	lsm6dsv16x_quaternion_t quat;
	lsm6dsv16x_accel_raw_t accel_raw;
	lsm6dsv16x_gyro_raw_t gyro_raw;
	lsm6dsv16x_accel_mg_t accel_mg;
	lsm6dsv16x_gyro_mdps_t gyro_mdps;
	int ret;
	
	/* Get FIFO level */
	ret = lsm6dsv16x_fifo_get_count(&imu_config, &fifo_count);
	if (ret < 0) {
		LOG_ERR("Failed to get FIFO count: %d", ret);
		return;
	}
	
	if (fifo_count == 0) {
		return;
	}
	
	LOG_INF("FIFO samples: %d", fifo_count);
	
	/* Read all FIFO samples */
	for (uint16_t i = 0; i < fifo_count; i++) {
		ret = lsm6dsv16x_fifo_read_sample(&imu_config, &sample);
		if (ret < 0) {
			LOG_ERR("Failed to read FIFO: %d", ret);
			break;
		}
		
		switch (sample.tag) {
		case LSM6DSV16X_FIFO_TAG_GYRO:
			/* Gyroscope data */
			gyro_raw.x = sys_get_le16(&sample.data[0]);
			gyro_raw.y = sys_get_le16(&sample.data[2]);
			gyro_raw.z = sys_get_le16(&sample.data[4]);
			lsm6dsv16x_gyro_raw_to_mdps(&gyro_raw, imu_config.gyro_fs, &gyro_mdps);
			LOG_DBG("FIFO Gyro: X=%7.1f Y=%7.1f Z=%7.1f mdps",
			        (double)gyro_mdps.x, (double)gyro_mdps.y, (double)gyro_mdps.z);
			break;
			
		case LSM6DSV16X_FIFO_TAG_ACCEL:
			/* Accelerometer data */
			accel_raw.x = sys_get_le16(&sample.data[0]);
			accel_raw.y = sys_get_le16(&sample.data[2]);
			accel_raw.z = sys_get_le16(&sample.data[4]);
			lsm6dsv16x_accel_raw_to_mg(&accel_raw, imu_config.accel_fs, &accel_mg);
			LOG_DBG("FIFO Accel: X=%6.2f Y=%6.2f Z=%6.2f mg",
			        (double)accel_mg.x, (double)accel_mg.y, (double)accel_mg.z);
			break;
			
		case LSM6DSV16X_FIFO_TAG_SFLP_QUAT:
			/* SFLP Quaternion */
			lsm6dsv16x_sflp_to_quaternion(sample.data, &quat);
			LOG_INF("Quaternion: W=%6.4f X=%6.4f Y=%6.4f Z=%6.4f",
			        (double)quat.w, (double)quat.x, 
			        (double)quat.y, (double)quat.z);
			break;
			
		default:
			LOG_DBG("Unknown FIFO tag: 0x%02X", sample.tag);
			break;
		}
	}
}

/**
 * @brief Main application thread
 */
int main(void)
{
	int ret;
	
	LOG_INF("LSM6DSV16X Example Starting...");
	
	/* Wait for system stabilization */
	k_sleep(K_MSEC(100));
	
	/* Initialize IMU */
	ret = imu_init();
	if (ret < 0) {
		LOG_ERR("IMU initialization failed");
		return ret;
	}
	
	/* Wait for sensor to stabilize */
	k_sleep(K_MSEC(100));
	
	/* Example 1: Read direct register data for 5 seconds */
	LOG_INF("=== Example 1: Direct Register Reading ===");
	for (int i = 0; i < 50; i++) {
		read_direct_data();
		k_sleep(K_MSEC(100));
	}
	
	/* Example 2: Enable SFLP and read FIFO */
	LOG_INF("=== Example 2: SFLP with FIFO ===");
	ret = enable_sflp();
	if (ret < 0) {
		LOG_ERR("Failed to enable SFLP");
		return ret;
	}
	
	/* Read FIFO periodically */
	while (1) {
		read_fifo_data();
		k_sleep(K_MSEC(100));
	}
	
	return 0;
}
