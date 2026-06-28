/**
 * @file lsm6dsv16x.c
 * @brief LSM6DSV16X IMU Driver Implementation for Zephyr RTOS
 * @author Based on STM32duino LSM6DSV16X library
 * @date 2025
 */

#include "lsm6dsv16x.h"
#include "lsm6dsv16x_regs.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(lsm6dsv16x, CONFIG_LSM6DSV16X_LOG_LEVEL);

/* ======================== Private Helpers ======================== */

/**
 * @brief Write single register
 */
static int reg_write(const lsm6dsv16x_config_t *config, uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = {reg, val};
	
	return i2c_write(config->i2c_dev, buf, sizeof(buf), config->i2c_addr);
}

/**
 * @brief Read single register
 */
static int reg_read(const lsm6dsv16x_config_t *config, uint8_t reg, uint8_t *val)
{
	return i2c_write_read(config->i2c_dev, config->i2c_addr,
	                      &reg, 1, val, 1);
}

/**
 * @brief Read multiple registers (auto-increment must be enabled)
 */
static int reg_read_multi(const lsm6dsv16x_config_t *config, uint8_t reg,
                          uint8_t *buf, size_t len)
{
	return i2c_write_read(config->i2c_dev, config->i2c_addr,
	                      &reg, 1, buf, len);
}

/**
 * @brief Switch memory bank for embedded functions
 */
static int mem_bank_set(const lsm6dsv16x_config_t *config, uint8_t bank)
{
	uint8_t val;
	int ret;
	
	ret = reg_read(config, LSM6DSV16X_FUNC_CFG_ACCESS, &val);
	if (ret < 0) {
		return ret;
	}
	
	/* Clear bank bits */
	val &= ~(LSM6DSV16X_FUNC_CFG_EMB_FUNC_REG_ACCESS | 
	         LSM6DSV16X_FUNC_CFG_SHUB_REG_ACCESS);
	
	/* Set new bank */
	if (bank == LSM6DSV16X_MEM_BANK_EMBED) {
		val |= LSM6DSV16X_FUNC_CFG_EMB_FUNC_REG_ACCESS;
	} else if (bank == LSM6DSV16X_MEM_BANK_SENSOR_HUB) {
		val |= LSM6DSV16X_FUNC_CFG_SHUB_REG_ACCESS;
	}
	
	return reg_write(config, LSM6DSV16X_FUNC_CFG_ACCESS, val);
}

/**
 * @brief Read/modify/write register bits
 */
static int reg_update_bits(const lsm6dsv16x_config_t *config, uint8_t reg,
                           uint8_t mask, uint8_t val)
{
	uint8_t old_val;
	int ret;
	
	ret = reg_read(config, reg, &old_val);
	if (ret < 0) {
		return ret;
	}
	
	uint8_t new_val = (old_val & ~mask) | (val & mask);
	
	if (new_val == old_val) {
		return 0;  /* No change needed */
	}
	
	return reg_write(config, reg, new_val);
}

/* ======================== Sensitivity Tables ======================== */

/* Accelerometer sensitivity in mg/LSB */
static const float accel_sensitivity[] = {
	[LSM6DSV16X_ACCEL_FS_2G]  = 0.061f,
	[LSM6DSV16X_ACCEL_FS_4G]  = 0.122f,
	[LSM6DSV16X_ACCEL_FS_8G]  = 0.244f,
	[LSM6DSV16X_ACCEL_FS_16G] = 0.488f,
};

/* Gyroscope sensitivity in mdps/LSB */
static const float gyro_sensitivity[] = {
	[LSM6DSV16X_GYRO_FS_125_DPS]  = 4.375f,
	[LSM6DSV16X_GYRO_FS_250_DPS]  = 8.750f,
	[LSM6DSV16X_GYRO_FS_500_DPS]  = 17.50f,
	[LSM6DSV16X_GYRO_FS_1000_DPS] = 35.0f,
	[LSM6DSV16X_GYRO_FS_2000_DPS] = 70.0f,
	[LSM6DSV16X_GYRO_FS_4000_DPS] = 140.0f,
};

/* ======================== Public API Implementation ======================== */

int lsm6dsv16x_init(const lsm6dsv16x_config_t *config)
{
	uint8_t who_am_i;
	int ret;
	
	if (!config || !config->i2c_dev) {
		LOG_ERR("Invalid configuration");
		return -EINVAL;
	}
	
	/* Verify device identity */
	ret = reg_read(config, LSM6DSV16X_WHO_AM_I, &who_am_i);
	if (ret < 0) {
		LOG_ERR("Failed to read WHO_AM_I: %d", ret);
		return ret;
	}
	
	if (who_am_i != LSM6DSV16X_WHO_AM_I_VALUE) {
		LOG_ERR("Wrong WHO_AM_I: 0x%02X (expected 0x%02X)",
		        who_am_i, LSM6DSV16X_WHO_AM_I_VALUE);
		return -ENODEV;
	}
	
	LOG_INF("LSM6DSV16X detected (WHO_AM_I: 0x%02X)", who_am_i);
	
	/* Enable register auto-increment */
	ret = reg_update_bits(config, LSM6DSV16X_CTRL3,
	                      LSM6DSV16X_CTRL3_IF_INC,
	                      LSM6DSV16X_CTRL3_IF_INC);
	if (ret < 0) {
		LOG_ERR("Failed to enable auto-increment: %d", ret);
		return ret;
	}
	
	/* Enable Block Data Update (BDU) */
	ret = reg_update_bits(config, LSM6DSV16X_CTRL3,
	                      LSM6DSV16X_CTRL3_BDU,
	                      LSM6DSV16X_CTRL3_BDU);
	if (ret < 0) {
		LOG_ERR("Failed to enable BDU: %d", ret);
		return ret;
	}
	
	/* Set default accelerometer to power-down */
	ret = reg_write(config, LSM6DSV16X_CTRL1, 0x00);
	if (ret < 0) {
		LOG_ERR("Failed to configure accelerometer: %d", ret);
		return ret;
	}
	
	/* Set default gyroscope to power-down */
	ret = reg_write(config, LSM6DSV16X_CTRL2, 0x00);
	if (ret < 0) {
		LOG_ERR("Failed to configure gyroscope: %d", ret);
		return ret;
	}
	
	LOG_INF("LSM6DSV16X initialized successfully");
	
	return 0;
}

int lsm6dsv16x_reset(const lsm6dsv16x_config_t *config)
{
	int ret;
	uint8_t val;
	int timeout = 100;  /* 100ms timeout */
	
	if (!config) {
		return -EINVAL;
	}
	
	/* Trigger software reset */
	ret = reg_update_bits(config, LSM6DSV16X_CTRL3,
	                      LSM6DSV16X_CTRL3_SW_RESET,
	                      LSM6DSV16X_CTRL3_SW_RESET);
	if (ret < 0) {
		LOG_ERR("Failed to trigger reset: %d", ret);
		return ret;
	}
	
	/* Wait for reset to complete */
	do {
		k_msleep(1);
		ret = reg_read(config, LSM6DSV16X_CTRL3, &val);
		if (ret < 0) {
			LOG_ERR("Failed to read reset status: %d", ret);
			return ret;
		}
		
		if (!(val & LSM6DSV16X_CTRL3_SW_RESET)) {
			LOG_INF("Reset completed");
			return 0;
		}
		
		timeout--;
	} while (timeout > 0);
	
	LOG_ERR("Reset timeout");
	return -ETIMEDOUT;
}

int lsm6dsv16x_accel_config(const lsm6dsv16x_config_t *config,
                             lsm6dsv16x_accel_odr_t odr,
                             lsm6dsv16x_accel_fs_t fs)
{
	int ret;
	uint8_t val;
	
	if (!config) {
		return -EINVAL;
	}
	
	/* Configure ODR (bits 7:4) and FS (bits 3:2) */
	val = ((odr & 0x0F) << 4) | ((fs & 0x03) << 2);
	
	ret = reg_write(config, LSM6DSV16X_CTRL1, val);
	if (ret < 0) {
		LOG_ERR("Failed to configure accelerometer: %d", ret);
		return ret;
	}
	
	LOG_DBG("Accel configured: ODR=%d, FS=%d", odr, fs);
	
	return 0;
}

int lsm6dsv16x_gyro_config(const lsm6dsv16x_config_t *config,
                            lsm6dsv16x_gyro_odr_t odr,
                            lsm6dsv16x_gyro_fs_t fs)
{
	int ret;
	uint8_t val;
	
	if (!config) {
		return -EINVAL;
	}
	
	/* Configure ODR (bits 7:4) and FS (bits 3:0) */
	val = ((odr & 0x0F) << 4) | (fs & 0x0F);
	
	ret = reg_write(config, LSM6DSV16X_CTRL2, val);
	if (ret < 0) {
		LOG_ERR("Failed to configure gyroscope: %d", ret);
		return ret;
	}
	
	LOG_DBG("Gyro configured: ODR=%d, FS=%d", odr, fs);
	
	return 0;
}

int lsm6dsv16x_accel_lpf_config(const lsm6dsv16x_config_t *config,
                                 bool enable,
                                 lsm6dsv16x_accel_lpf_bw_t bandwidth)
{
	int ret;
	
	if (!config) {
		return -EINVAL;
	}
	
	/* Enable/disable LPF2 in CTRL9 */
	ret = reg_update_bits(config, LSM6DSV16X_CTRL9,
	                      LSM6DSV16X_CTRL9_LPF2_XL_EN,
	                      enable ? LSM6DSV16X_CTRL9_LPF2_XL_EN : 0);
	if (ret < 0) {
		LOG_ERR("Failed to configure LPF2 enable: %d", ret);
		return ret;
	}
	
	/* Set bandwidth in CTRL8 */
	ret = reg_update_bits(config, LSM6DSV16X_CTRL8,
	                      LSM6DSV16X_CTRL8_HP_LPF2_XL_BW_MASK,
	                      bandwidth & LSM6DSV16X_CTRL8_HP_LPF2_XL_BW_MASK);
	if (ret < 0) {
		LOG_ERR("Failed to configure LPF2 bandwidth: %d", ret);
		return ret;
	}
	
	LOG_DBG("Accel LPF configured: enable=%d, bw=%d", enable, bandwidth);
	
	return 0;
}

int lsm6dsv16x_gyro_lpf_config(const lsm6dsv16x_config_t *config,
                                bool enable,
                                lsm6dsv16x_gyro_lpf_bw_t bandwidth)
{
	int ret;
	
	if (!config) {
		return -EINVAL;
	}
	
	/* Enable/disable LPF1 in CTRL7 */
	ret = reg_update_bits(config, LSM6DSV16X_CTRL7,
	                      LSM6DSV16X_CTRL7_LPF1_G_EN,
	                      enable ? LSM6DSV16X_CTRL7_LPF1_G_EN : 0);
	if (ret < 0) {
		LOG_ERR("Failed to configure LPF1 enable: %d", ret);
		return ret;
	}
	
	/* Set bandwidth in CTRL6 */
	ret = reg_update_bits(config, LSM6DSV16X_CTRL6,
	                      LSM6DSV16X_CTRL6_LPF1_G_BW_MASK,
	                      bandwidth & LSM6DSV16X_CTRL6_LPF1_G_BW_MASK);
	if (ret < 0) {
		LOG_ERR("Failed to configure LPF1 bandwidth: %d", ret);
		return ret;
	}
	
	LOG_DBG("Gyro LPF configured: enable=%d, bw=%d", enable, bandwidth);
	
	return 0;
}

/* ======================== SFLP Functions ======================== */

int lsm6dsv16x_sflp_enable(const lsm6dsv16x_config_t *config,
                            lsm6dsv16x_sflp_odr_t odr)
{
	int ret;
	
	if (!config) {
		return -EINVAL;
	}
	
	LOG_INF("Enabling SFLP with ODR=%d", odr);
	
	/* Switch to embedded functions memory bank */
	ret = mem_bank_set(config, LSM6DSV16X_MEM_BANK_EMBED);
	if (ret < 0) {
		LOG_ERR("Failed to switch to embed bank: %d", ret);
		return ret;
	}
	
	/* Set SFLP ODR */
	ret = reg_update_bits(config, LSM6DSV16X_SFLP_ODR, 0x07, odr & 0x07);
	if (ret < 0) {
		LOG_ERR("Failed to set SFLP ODR: %d", ret);
		mem_bank_set(config, LSM6DSV16X_MEM_BANK_MAIN);
		return ret;
	}
	
	/* Enable SFLP game rotation in EMB_FUNC_EN_A */
	ret = reg_update_bits(config, LSM6DSV16X_EMB_FUNC_EN_A,
	                      LSM6DSV16X_EMB_FUNC_EN_A_SFLP_GAME_EN,
	                      LSM6DSV16X_EMB_FUNC_EN_A_SFLP_GAME_EN);
	if (ret < 0) {
		LOG_ERR("Failed to enable SFLP: %d", ret);
		mem_bank_set(config, LSM6DSV16X_MEM_BANK_MAIN);
		return ret;
	}
	
	/* Enable SFLP FIFO batching */
	ret = reg_update_bits(config, LSM6DSV16X_EMB_FUNC_FIFO_EN_A,
	                      LSM6DSV16X_SFLP_GAME_FIFO_EN,
	                      LSM6DSV16X_SFLP_GAME_FIFO_EN);
	if (ret < 0) {
		LOG_ERR("Failed to enable SFLP FIFO batching: %d", ret);
		mem_bank_set(config, LSM6DSV16X_MEM_BANK_MAIN);
		return ret;
	}
	
	/* Return to main memory bank */
	ret = mem_bank_set(config, LSM6DSV16X_MEM_BANK_MAIN);
	if (ret < 0) {
		LOG_ERR("Failed to return to main bank: %d", ret);
		return ret;
	}
	
	/* Configure FIFO in STREAM mode (mode = 6) */
	ret = reg_update_bits(config, LSM6DSV16X_FIFO_CTRL4, 0x07, 0x06);
	if (ret < 0) {
		LOG_ERR("Failed to set FIFO mode: %d", ret);
		return ret;
	}
	
	LOG_INF("SFLP enabled successfully");
	
	return 0;
}

int lsm6dsv16x_sflp_disable(const lsm6dsv16x_config_t *config)
{
	int ret;
	
	if (!config) {
		return -EINVAL;
	}
	
	/* Switch to embedded functions memory bank */
	ret = mem_bank_set(config, LSM6DSV16X_MEM_BANK_EMBED);
	if (ret < 0) {
		return ret;
	}
	
	/* Disable SFLP FIFO batching */
	ret = reg_update_bits(config, LSM6DSV16X_EMB_FUNC_FIFO_EN_A,
	                      LSM6DSV16X_SFLP_GAME_FIFO_EN, 0);
	if (ret < 0) {
		mem_bank_set(config, LSM6DSV16X_MEM_BANK_MAIN);
		return ret;
	}
	
	/* Disable SFLP game rotation */
	ret = reg_update_bits(config, LSM6DSV16X_EMB_FUNC_EN_A,
	                      LSM6DSV16X_EMB_FUNC_EN_A_SFLP_GAME_EN, 0);
	if (ret < 0) {
		mem_bank_set(config, LSM6DSV16X_MEM_BANK_MAIN);
		return ret;
	}
	
	/* Return to main memory bank */
	ret = mem_bank_set(config, LSM6DSV16X_MEM_BANK_MAIN);
	if (ret < 0) {
		return ret;
	}
	
	/* Set FIFO to BYPASS mode */
	ret = reg_update_bits(config, LSM6DSV16X_FIFO_CTRL4, 0x07, 0x00);
	if (ret < 0) {
		return ret;
	}
	
	LOG_INF("SFLP disabled");
	
	return 0;
}

int lsm6dsv16x_sflp_reset(const lsm6dsv16x_config_t *config)
{
	int ret;
	
	if (!config) {
		return -EINVAL;
	}
	
	/* Switch to embedded functions memory bank */
	ret = mem_bank_set(config, LSM6DSV16X_MEM_BANK_EMBED);
	if (ret < 0) {
		return ret;
	}
	
	/* Set SFLP_GAME_INIT bit */
	ret = reg_update_bits(config, LSM6DSV16X_EMB_FUNC_INIT_A,
	                      LSM6DSV16X_SFLP_GAME_INIT,
	                      LSM6DSV16X_SFLP_GAME_INIT);
	if (ret < 0) {
		mem_bank_set(config, LSM6DSV16X_MEM_BANK_MAIN);
		return ret;
	}
	
	/* Return to main memory bank */
	ret = mem_bank_set(config, LSM6DSV16X_MEM_BANK_MAIN);
	if (ret < 0) {
		return ret;
	}
	
	LOG_INF("SFLP reset");
	
	/* Wait for reset to take effect */
	k_msleep(10);
	
	return 0;
}

/* ======================== Data Reading Functions ======================== */

int lsm6dsv16x_accel_read_raw(const lsm6dsv16x_config_t *config,
                               lsm6dsv16x_accel_raw_t *data)
{
	uint8_t buf[6];
	int ret;
	
	if (!config || !data) {
		return -EINVAL;
	}
	
	/* Read 6 bytes starting from OUTX_L_A (auto-increment enabled) */
	ret = reg_read_multi(config, LSM6DSV16X_OUTX_L_A, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to read accelerometer data: %d", ret);
		return ret;
	}
	
	/* Convert to int16_t (little-endian) */
	data->x = sys_get_le16(&buf[0]);
	data->y = sys_get_le16(&buf[2]);
	data->z = sys_get_le16(&buf[4]);
	
	return 0;
}

int lsm6dsv16x_gyro_read_raw(const lsm6dsv16x_config_t *config,
                              lsm6dsv16x_gyro_raw_t *data)
{
	uint8_t buf[6];
	int ret;
	
	if (!config || !data) {
		return -EINVAL;
	}
	
	/* Read 6 bytes starting from OUTX_L_G (auto-increment enabled) */
	ret = reg_read_multi(config, LSM6DSV16X_OUTX_L_G, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to read gyroscope data: %d", ret);
		return ret;
	}
	
	/* Convert to int16_t (little-endian) */
	data->x = sys_get_le16(&buf[0]);
	data->y = sys_get_le16(&buf[2]);
	data->z = sys_get_le16(&buf[4]);
	
	return 0;
}

/* ======================== FIFO Functions ======================== */

int lsm6dsv16x_fifo_get_count(const lsm6dsv16x_config_t *config,
                               uint16_t *count)
{
	uint8_t buf[2];
	int ret;
	
	if (!config || !count) {
		return -EINVAL;
	}
	
	/* Read FIFO_STATUS1 and FIFO_STATUS2 */
	ret = reg_read_multi(config, LSM6DSV16X_FIFO_STATUS1, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to read FIFO status: %d", ret);
		return ret;
	}
	
	/* FIFO level is 9 bits: STATUS1[7:0] and STATUS2[0] */
	*count = buf[0] | ((buf[1] & 0x01) << 8);
	
	return 0;
}

int lsm6dsv16x_fifo_read_sample(const lsm6dsv16x_config_t *config,
                                 lsm6dsv16x_fifo_sample_t *sample)
{
	uint8_t buf[7];
	int ret;
	
	if (!config || !sample) {
		return -EINVAL;
	}
	
	/* Read 7 bytes: TAG + 6 data bytes */
	ret = reg_read_multi(config, LSM6DSV16X_FIFO_DATA_OUT_TAG, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to read FIFO sample: %d", ret);
		return ret;
	}
	
	/* Extract tag (bits 7:3 of first byte) */
	sample->tag = (buf[0] >> 3) & 0x1F;
	
	/* Copy data bytes */
	memcpy(sample->data, &buf[1], 6);
	
	LOG_DBG("FIFO sample: tag=0x%02X", sample->tag);
	
	return 0;
}

/* ======================== Conversion Functions ======================== */

void lsm6dsv16x_accel_raw_to_mg(const lsm6dsv16x_accel_raw_t *raw,
                                 lsm6dsv16x_accel_fs_t fs,
                                 lsm6dsv16x_accel_mg_t *mg)
{
	if (!raw || !mg || fs > LSM6DSV16X_ACCEL_FS_16G) {
		return;
	}
	
	float sensitivity = accel_sensitivity[fs];
	
	mg->x = (float)raw->x * sensitivity;
	mg->y = (float)raw->y * sensitivity;
	mg->z = (float)raw->z * sensitivity;
}

void lsm6dsv16x_gyro_raw_to_mdps(const lsm6dsv16x_gyro_raw_t *raw,
                                  lsm6dsv16x_gyro_fs_t fs,
                                  lsm6dsv16x_gyro_mdps_t *mdps)
{
	if (!raw || !mdps) {
		return;
	}
	
	float sensitivity;
	
	/* Handle non-contiguous FS values */
	switch (fs) {
	case LSM6DSV16X_GYRO_FS_125_DPS:
		sensitivity = gyro_sensitivity[0];
		break;
	case LSM6DSV16X_GYRO_FS_250_DPS:
		sensitivity = gyro_sensitivity[1];
		break;
	case LSM6DSV16X_GYRO_FS_500_DPS:
		sensitivity = gyro_sensitivity[2];
		break;
	case LSM6DSV16X_GYRO_FS_1000_DPS:
		sensitivity = gyro_sensitivity[3];
		break;
	case LSM6DSV16X_GYRO_FS_2000_DPS:
		sensitivity = gyro_sensitivity[4];
		break;
	case LSM6DSV16X_GYRO_FS_4000_DPS:
		sensitivity = gyro_sensitivity[5];
		break;
	default:
		sensitivity = gyro_sensitivity[4];  /* Default to 2000 dps */
		break;
	}
	
	mdps->x = (float)raw->x * sensitivity;
	mdps->y = (float)raw->y * sensitivity;
	mdps->z = (float)raw->z * sensitivity;
}

/* ======================== SFLP Conversion ======================== */

/**
 * @brief Convert half-precision float to single-precision float
 * 
 * Half-precision format: SEEEEEFFFFFFFFFF
 * S: 1 sign bit, E: 5 exponent bits, F: 10 fraction bits
 */
static float half_to_float(uint16_t h)
{
	uint32_t sign = (h & 0x8000) << 16;
	uint32_t exp = (h & 0x7C00) >> 10;
	uint32_t frac = (h & 0x03FF);
	
	uint32_t f;
	
	if (exp == 0) {
		if (frac == 0) {
			/* Zero */
			f = sign;
		} else {
			/* Subnormal */
			exp = 127 - 15;
			while ((frac & 0x0400) == 0) {
				frac <<= 1;
				exp--;
			}
			frac &= 0x03FF;
			f = sign | (exp << 23) | (frac << 13);
		}
	} else if (exp == 0x1F) {
		/* Infinity or NaN */
		f = sign | 0x7F800000 | (frac << 13);
	} else {
		/* Normalized */
		f = sign | ((exp + (127 - 15)) << 23) | (frac << 13);
	}
	
	return *((float *)&f);
}

void lsm6dsv16x_sflp_to_quaternion(const uint8_t fifo_data[6],
                                    lsm6dsv16x_quaternion_t *quat)
{
	if (!fifo_data || !quat) {
		return;
	}
	
	/* Extract 3x int16_t values (little-endian) */
	uint16_t sflp[3];
	sflp[0] = sys_get_le16(&fifo_data[0]);
	sflp[1] = sys_get_le16(&fifo_data[2]);
	sflp[2] = sys_get_le16(&fifo_data[4]);
	
	/* Convert half-precision to float */
	float qx = half_to_float(sflp[0]);
	float qy = half_to_float(sflp[1]);
	float qz = half_to_float(sflp[2]);
	
	/* Calculate w component (quaternion is normalized: |q| = 1) */
	float sum_sq = qx * qx + qy * qy + qz * qz;
	
	float qw;
	if (sum_sq < 1.0f) {
		qw = sqrtf(1.0f - sum_sq);
	} else {
		/* Handle numerical errors */
		qw = 0.0f;
		/* Renormalize */
		float norm = sqrtf(sum_sq);
		if (norm > 0.0f) {
			qx /= norm;
			qy /= norm;
			qz /= norm;
		}
	}
	
	/* Store quaternion (w, x, y, z) */
	quat->w = qw;
	quat->x = qx;
	quat->y = qy;
	quat->z = qz;
}
