/**
 * @file lsm6dsv16x.h
 * @brief LSM6DSV16X IMU Driver for Zephyr RTOS
 * @author Based on STM32duino LSM6DSV16X library
 * @date 2025
 * 
 * 6-axis IMU with embedded sensor fusion (SFLP) for Zephyr RTOS
 * Features: Accelerometer, Gyroscope, Quaternion fusion, FIFO
 */

#ifndef LSM6DSV16X_H_
#define LSM6DSV16X_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== Type Definitions ======================== */

/**
 * @brief Accelerometer output data rate
 */
typedef enum {
	LSM6DSV16X_ACCEL_ODR_OFF = 0x00,
	LSM6DSV16X_ACCEL_ODR_1_875_HZ = 0x01,
	LSM6DSV16X_ACCEL_ODR_7_5_HZ = 0x02,
	LSM6DSV16X_ACCEL_ODR_15_HZ = 0x03,
	LSM6DSV16X_ACCEL_ODR_30_HZ = 0x04,
	LSM6DSV16X_ACCEL_ODR_60_HZ = 0x05,
	LSM6DSV16X_ACCEL_ODR_120_HZ = 0x06,
	LSM6DSV16X_ACCEL_ODR_240_HZ = 0x07,
	LSM6DSV16X_ACCEL_ODR_480_HZ = 0x08,
	LSM6DSV16X_ACCEL_ODR_960_HZ = 0x09,
	LSM6DSV16X_ACCEL_ODR_1920_HZ = 0x0A,
	LSM6DSV16X_ACCEL_ODR_3840_HZ = 0x0B,
	LSM6DSV16X_ACCEL_ODR_7680_HZ = 0x0C,
} lsm6dsv16x_accel_odr_t;

/**
 * @brief Gyroscope output data rate
 */
typedef enum {
	LSM6DSV16X_GYRO_ODR_OFF = 0x00,
	LSM6DSV16X_GYRO_ODR_7_5_HZ = 0x02,
	LSM6DSV16X_GYRO_ODR_15_HZ = 0x03,
	LSM6DSV16X_GYRO_ODR_30_HZ = 0x04,
	LSM6DSV16X_GYRO_ODR_60_HZ = 0x05,
	LSM6DSV16X_GYRO_ODR_120_HZ = 0x06,
	LSM6DSV16X_GYRO_ODR_240_HZ = 0x07,
	LSM6DSV16X_GYRO_ODR_480_HZ = 0x08,
	LSM6DSV16X_GYRO_ODR_960_HZ = 0x09,
	LSM6DSV16X_GYRO_ODR_1920_HZ = 0x0A,
	LSM6DSV16X_GYRO_ODR_3840_HZ = 0x0B,
	LSM6DSV16X_GYRO_ODR_7680_HZ = 0x0C,
} lsm6dsv16x_gyro_odr_t;

/**
 * @brief Accelerometer full scale range
 */
typedef enum {
	LSM6DSV16X_ACCEL_FS_2G = 0,   /* ±2g  -> 0.061 mg/LSB */
	LSM6DSV16X_ACCEL_FS_4G = 1,   /* ±4g  -> 0.122 mg/LSB */
	LSM6DSV16X_ACCEL_FS_8G = 2,   /* ±8g  -> 0.244 mg/LSB */
	LSM6DSV16X_ACCEL_FS_16G = 3,  /* ±16g -> 0.488 mg/LSB */
} lsm6dsv16x_accel_fs_t;

/**
 * @brief Gyroscope full scale range
 */
typedef enum {
	LSM6DSV16X_GYRO_FS_125_DPS = 0,   /* ±125 dps  -> 4.375 mdps/LSB */
	LSM6DSV16X_GYRO_FS_250_DPS = 1,   /* ±250 dps  -> 8.750 mdps/LSB */
	LSM6DSV16X_GYRO_FS_500_DPS = 2,   /* ±500 dps  -> 17.50 mdps/LSB */
	LSM6DSV16X_GYRO_FS_1000_DPS = 3,  /* ±1000 dps -> 35.0 mdps/LSB */
	LSM6DSV16X_GYRO_FS_2000_DPS = 4,  /* ±2000 dps -> 70.0 mdps/LSB */
	LSM6DSV16X_GYRO_FS_4000_DPS = 8,  /* ±4000 dps -> 140.0 mdps/LSB */
} lsm6dsv16x_gyro_fs_t;

/**
 * @brief SFLP (Sensor Fusion Low Power) output data rate
 */
typedef enum {
	LSM6DSV16X_SFLP_ODR_15_HZ = 0,
	LSM6DSV16X_SFLP_ODR_30_HZ = 1,
	LSM6DSV16X_SFLP_ODR_60_HZ = 2,
	LSM6DSV16X_SFLP_ODR_120_HZ = 3,
	LSM6DSV16X_SFLP_ODR_240_HZ = 4,
	LSM6DSV16X_SFLP_ODR_480_HZ = 5,
} lsm6dsv16x_sflp_odr_t;

/**
 * @brief Low-pass filter bandwidth for accelerometer
 */
typedef enum {
	LSM6DSV16X_ACCEL_LPF_ULTRA_LIGHT = 0,
	LSM6DSV16X_ACCEL_LPF_VERY_LIGHT = 1,
	LSM6DSV16X_ACCEL_LPF_LIGHT = 2,
	LSM6DSV16X_ACCEL_LPF_MEDIUM = 3,
	LSM6DSV16X_ACCEL_LPF_STRONG = 4,
	LSM6DSV16X_ACCEL_LPF_VERY_STRONG = 5,
	LSM6DSV16X_ACCEL_LPF_AGGRESSIVE = 6,
	LSM6DSV16X_ACCEL_LPF_XTREME = 7,
} lsm6dsv16x_accel_lpf_bw_t;

/**
 * @brief Low-pass filter bandwidth for gyroscope
 */
typedef enum {
	LSM6DSV16X_GYRO_LPF_ULTRA_LIGHT = 0,
	LSM6DSV16X_GYRO_LPF_VERY_LIGHT = 1,
	LSM6DSV16X_GYRO_LPF_LIGHT = 2,
	LSM6DSV16X_GYRO_LPF_MEDIUM = 3,
	LSM6DSV16X_GYRO_LPF_STRONG = 4,
	LSM6DSV16X_GYRO_LPF_VERY_STRONG = 5,
	LSM6DSV16X_GYRO_LPF_AGGRESSIVE = 6,
	LSM6DSV16X_GYRO_LPF_XTREME = 7,
} lsm6dsv16x_gyro_lpf_bw_t;

/**
 * @brief FIFO mode
 */
typedef enum {
	LSM6DSV16X_FIFO_BYPASS = 0,
	LSM6DSV16X_FIFO_MODE = 1,
	LSM6DSV16X_FIFO_CONTINUOUS = 6,
	LSM6DSV16X_FIFO_STREAM = 6,  /* Alias for continuous */
} lsm6dsv16x_fifo_mode_t;

/**
 * @brief Accelerometer raw data (3 axes)
 */
typedef struct {
	int16_t x;  /* X-axis raw value */
	int16_t y;  /* Y-axis raw value */
	int16_t z;  /* Z-axis raw value */
} lsm6dsv16x_accel_raw_t;

/**
 * @brief Gyroscope raw data (3 axes)
 */
typedef struct {
	int16_t x;  /* X-axis raw value */
	int16_t y;  /* Y-axis raw value */
	int16_t z;  /* Z-axis raw value */
} lsm6dsv16x_gyro_raw_t;

/**
 * @brief Accelerometer data in mg (milli-g)
 */
typedef struct {
	float x;  /* X-axis in mg */
	float y;  /* Y-axis in mg */
	float z;  /* Z-axis in mg */
} lsm6dsv16x_accel_mg_t;

/**
 * @brief Gyroscope data in mdps (milli-degrees per second)
 */
typedef struct {
	float x;  /* X-axis in mdps */
	float y;  /* Y-axis in mdps */
	float z;  /* Z-axis in mdps */
} lsm6dsv16x_gyro_mdps_t;

/**
 * @brief Quaternion data from SFLP
 * Normalized quaternion (|q| = 1)
 */
typedef struct {
	float w;  /* Scalar part */
	float x;  /* Vector i component */
	float y;  /* Vector j component */
	float z;  /* Vector k component */
} lsm6dsv16x_quaternion_t;

/**
 * @brief FIFO sample with tag
 */
typedef struct {
	uint8_t tag;      /* FIFO tag identifying data type */
	uint8_t data[6];  /* 6 bytes of data */
} lsm6dsv16x_fifo_sample_t;

/**
 * @brief Device configuration structure
 */
typedef struct {
	const struct device *i2c_dev;
	uint8_t i2c_addr;
	lsm6dsv16x_accel_fs_t accel_fs;
	lsm6dsv16x_gyro_fs_t gyro_fs;
	bool sflp_enabled;
} lsm6dsv16x_config_t;

/* ======================== API Functions ======================== */

/**
 * @brief Initialize the LSM6DSV16X sensor
 * 
 * Performs device identification, software reset, and basic configuration:
 * - Verifies WHO_AM_I register
 * - Enables auto-increment
 * - Enables block data update (BDU)
 * - Sets default ODR and FS values
 * 
 * @param config Configuration structure with I2C device and address
 * @return 0 on success, negative errno on failure
 */
int lsm6dsv16x_init(const lsm6dsv16x_config_t *config);

/**
 * @brief Perform software reset of the sensor
 * 
 * Resets all registers to default values and waits for reset completion.
 * 
 * @param config Configuration structure
 * @return 0 on success, negative errno on failure
 */
int lsm6dsv16x_reset(const lsm6dsv16x_config_t *config);

/**
 * @brief Set accelerometer output data rate and full scale
 * 
 * @param config Configuration structure
 * @param odr Output data rate
 * @param fs Full scale range
 * @return 0 on success, negative errno on failure
 */
int lsm6dsv16x_accel_config(const lsm6dsv16x_config_t *config,
                             lsm6dsv16x_accel_odr_t odr,
                             lsm6dsv16x_accel_fs_t fs);

/**
 * @brief Set gyroscope output data rate and full scale
 * 
 * @param config Configuration structure
 * @param odr Output data rate
 * @param fs Full scale range
 * @return 0 on success, negative errno on failure
 */
int lsm6dsv16x_gyro_config(const lsm6dsv16x_config_t *config,
                            lsm6dsv16x_gyro_odr_t odr,
                            lsm6dsv16x_gyro_fs_t fs);

/**
 * @brief Configure accelerometer low-pass filter
 * 
 * @param config Configuration structure
 * @param enable Enable (true) or disable (false) LPF2
 * @param bandwidth Filter bandwidth
 * @return 0 on success, negative errno on failure
 */
int lsm6dsv16x_accel_lpf_config(const lsm6dsv16x_config_t *config,
                                 bool enable,
                                 lsm6dsv16x_accel_lpf_bw_t bandwidth);

/**
 * @brief Configure gyroscope low-pass filter
 * 
 * @param config Configuration structure
 * @param enable Enable (true) or disable (false) LPF1
 * @param bandwidth Filter bandwidth
 * @return 0 on success, negative errno on failure
 */
int lsm6dsv16x_gyro_lpf_config(const lsm6dsv16x_config_t *config,
                                bool enable,
                                lsm6dsv16x_gyro_lpf_bw_t bandwidth);

/**
 * @brief Enable SFLP (Sensor Fusion Low Power) with Game Rotation
 * 
 * Configures the sensor for quaternion output:
 * - Sets SFLP ODR
 * - Enables game rotation vector
 * - Configures FIFO in STREAM mode
 * - Enables SFLP batching in FIFO
 * 
 * @param config Configuration structure
 * @param odr SFLP output data rate
 * @return 0 on success, negative errno on failure
 */
int lsm6dsv16x_sflp_enable(const lsm6dsv16x_config_t *config,
                            lsm6dsv16x_sflp_odr_t odr);

/**
 * @brief Disable SFLP
 * 
 * @param config Configuration structure
 * @return 0 on success, negative errno on failure
 */
int lsm6dsv16x_sflp_disable(const lsm6dsv16x_config_t *config);

/**
 * @brief Reset SFLP algorithm
 * 
 * Reinitializes the sensor fusion algorithm state.
 * 
 * @param config Configuration structure
 * @return 0 on success, negative errno on failure
 */
int lsm6dsv16x_sflp_reset(const lsm6dsv16x_config_t *config);

/**
 * @brief Read accelerometer raw data from registers
 * 
 * Reads the latest accelerometer data directly from output registers.
 * 
 * @param config Configuration structure
 * @param data Pointer to store raw data
 * @return 0 on success, negative errno on failure
 */
int lsm6dsv16x_accel_read_raw(const lsm6dsv16x_config_t *config,
                               lsm6dsv16x_accel_raw_t *data);

/**
 * @brief Read gyroscope raw data from registers
 * 
 * Reads the latest gyroscope data directly from output registers.
 * 
 * @param config Configuration structure
 * @param data Pointer to store raw data
 * @return 0 on success, negative errno on failure
 */
int lsm6dsv16x_gyro_read_raw(const lsm6dsv16x_config_t *config,
                              lsm6dsv16x_gyro_raw_t *data);

/**
 * @brief Get number of unread samples in FIFO
 * 
 * @param config Configuration structure
 * @param count Pointer to store sample count
 * @return 0 on success, negative errno on failure
 */
int lsm6dsv16x_fifo_get_count(const lsm6dsv16x_config_t *config,
                               uint16_t *count);

/**
 * @brief Read one sample from FIFO
 * 
 * Reads a single sample with tag from FIFO. The FIFO is automatically
 * advanced after reading (destructive read).
 * 
 * @param config Configuration structure
 * @param sample Pointer to store FIFO sample with tag
 * @return 0 on success, negative errno on failure
 */
int lsm6dsv16x_fifo_read_sample(const lsm6dsv16x_config_t *config,
                                 lsm6dsv16x_fifo_sample_t *sample);

/**
 * @brief Convert raw accelerometer data to mg
 * 
 * @param raw Raw accelerometer data
 * @param fs Full scale setting used during measurement
 * @param mg Pointer to store converted data in mg
 */
void lsm6dsv16x_accel_raw_to_mg(const lsm6dsv16x_accel_raw_t *raw,
                                 lsm6dsv16x_accel_fs_t fs,
                                 lsm6dsv16x_accel_mg_t *mg);

/**
 * @brief Convert raw gyroscope data to mdps
 * 
 * @param raw Raw gyroscope data
 * @param fs Full scale setting used during measurement
 * @param mdps Pointer to store converted data in mdps
 */
void lsm6dsv16x_gyro_raw_to_mdps(const lsm6dsv16x_gyro_raw_t *raw,
                                  lsm6dsv16x_gyro_fs_t fs,
                                  lsm6dsv16x_gyro_mdps_t *mdps);

/**
 * @brief Convert SFLP FIFO data to quaternion
 * 
 * Converts 3x int16_t half-precision float values from FIFO to
 * normalized quaternion. The 4th component (w) is calculated.
 * 
 * @param fifo_data 6 bytes from FIFO (3x int16_t)
 * @param quat Pointer to store quaternion
 */
void lsm6dsv16x_sflp_to_quaternion(const uint8_t fifo_data[6],
                                    lsm6dsv16x_quaternion_t *quat);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSV16X_H_ */
