/**
 * @file lsm6dsv16x_regs.h
 * @brief LSM6DSV16X Register definitions
 * @author Based on STM32duino LSM6DSV16X library
 * @date 2025
 */

#ifndef LSM6DSV16X_REGS_H_
#define LSM6DSV16X_REGS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== Register Addresses ======================== */

/* Device identification */
#define LSM6DSV16X_WHO_AM_I             0x0F
#define LSM6DSV16X_WHO_AM_I_VALUE       0x70

/* Control registers */
#define LSM6DSV16X_CTRL1                0x10  /* Accelerometer ODR */
#define LSM6DSV16X_CTRL2                0x11  /* Gyroscope ODR */
#define LSM6DSV16X_CTRL3                0x12  /* SW reset, BDU, IF_INC */
#define LSM6DSV16X_CTRL4                0x13  /* DRDY mask */
#define LSM6DSV16X_CTRL5                0x14  /* Rounding */
#define LSM6DSV16X_CTRL6                0x15  /* Gyro FS, LPF1 */
#define LSM6DSV16X_CTRL7                0x16  /* Gyro filters */
#define LSM6DSV16X_CTRL8                0x17  /* Accel filters */
#define LSM6DSV16X_CTRL9                0x18  /* Accel axis enable, LPF2 */
#define LSM6DSV16X_CTRL10               0x19  /* Timestamp, debug */

/* Status register */
#define LSM6DSV16X_STATUS_REG           0x1E

/* Temperature output */
#define LSM6DSV16X_OUT_TEMP_L           0x20
#define LSM6DSV16X_OUT_TEMP_H           0x21

/* Gyroscope output */
#define LSM6DSV16X_OUTX_L_G             0x22
#define LSM6DSV16X_OUTX_H_G             0x23
#define LSM6DSV16X_OUTY_L_G             0x24
#define LSM6DSV16X_OUTY_H_G             0x25
#define LSM6DSV16X_OUTZ_L_G             0x26
#define LSM6DSV16X_OUTZ_H_G             0x27

/* Accelerometer output */
#define LSM6DSV16X_OUTX_L_A             0x28
#define LSM6DSV16X_OUTX_H_A             0x29
#define LSM6DSV16X_OUTY_L_A             0x2A
#define LSM6DSV16X_OUTY_H_A             0x2B
#define LSM6DSV16X_OUTZ_L_A             0x2C
#define LSM6DSV16X_OUTZ_H_A             0x2D

/* FIFO control and status */
#define LSM6DSV16X_FIFO_CTRL1           0x07  /* Watermark */
#define LSM6DSV16X_FIFO_CTRL2           0x08  /* Stop on WTM, compression */
#define LSM6DSV16X_FIFO_CTRL3           0x09  /* Accel BDR */
#define LSM6DSV16X_FIFO_CTRL4           0x0A  /* Gyro BDR, temp BDR */
#define LSM6DSV16X_FIFO_STATUS1         0x1B  /* FIFO fill level LSB */
#define LSM6DSV16X_FIFO_STATUS2         0x1C  /* FIFO fill level MSB, flags */
#define LSM6DSV16X_FIFO_DATA_OUT_TAG    0x78  /* FIFO data output with tag */
#define LSM6DSV16X_FIFO_DATA_OUT_X_L    0x79  /* FIFO data output */

/* Embedded functions */
#define LSM6DSV16X_FUNC_CFG_ACCESS      0x01  /* Memory bank access */
#define LSM6DSV16X_EMB_FUNC_EN_A        0x04  /* Embedded functions enable A */
#define LSM6DSV16X_EMB_FUNC_EN_B        0x05  /* Embedded functions enable B */
#define LSM6DSV16X_EMB_FUNC_FIFO_EN_A   0x44  /* SFLP FIFO enable */

/* SFLP registers (accessed via memory bank) */
#define LSM6DSV16X_SFLP_ODR             0x5E  /* SFLP ODR */
#define LSM6DSV16X_EMB_FUNC_INIT_A      0x66  /* SFLP init */

/* ======================== Register Bit Definitions ======================== */

/* CTRL1 - Accelerometer ODR and FS */
#define LSM6DSV16X_CTRL1_ODR_XL_MASK    0xF0
#define LSM6DSV16X_CTRL1_FS_XL_MASK     0x0C
#define LSM6DSV16X_CTRL1_LPF2_XL_EN     0x02
#define LSM6DSV16X_CTRL1_OP_MODE_XL_MASK 0x01

/* CTRL2 - Gyroscope ODR and FS */
#define LSM6DSV16X_CTRL2_ODR_G_MASK     0xF0
#define LSM6DSV16X_CTRL2_FS_G_MASK      0x0F

/* CTRL3 - Software reset, BDU, Auto-increment */
#define LSM6DSV16X_CTRL3_BOOT           0x80
#define LSM6DSV16X_CTRL3_BDU            0x40
#define LSM6DSV16X_CTRL3_IF_INC         0x04
#define LSM6DSV16X_CTRL3_SW_RESET       0x01

/* CTRL6 - Gyroscope FS and LPF1 */
#define LSM6DSV16X_CTRL6_FS_G_MASK      0xF0
#define LSM6DSV16X_CTRL6_LPF1_G_BW_MASK 0x0F

/* CTRL7 - Gyroscope LPF1 enable */
#define LSM6DSV16X_CTRL7_LPF1_G_EN      0x40

/* CTRL8 - Accelerometer filters */
#define LSM6DSV16X_CTRL8_HP_LPF2_XL_BW_MASK 0x07

/* CTRL9 - Accelerometer LPF2 and axis enable */
#define LSM6DSV16X_CTRL9_LPF2_XL_EN     0x08
#define LSM6DSV16X_CTRL9_USR_OFF_ON_OUT 0x02
#define LSM6DSV16X_CTRL9_DEVICE_CONF    0x01

/* STATUS_REG bits */
#define LSM6DSV16X_STATUS_XLDA          0x01  /* Accel data available */
#define LSM6DSV16X_STATUS_GDA           0x02  /* Gyro data available */
#define LSM6DSV16X_STATUS_TDA           0x04  /* Temp data available */

/* FIFO_STATUS2 bits */
#define LSM6DSV16X_FIFO_STATUS2_WTM     0x80  /* Watermark flag */
#define LSM6DSV16X_FIFO_STATUS2_OVR     0x40  /* FIFO overrun */
#define LSM6DSV16X_FIFO_STATUS2_FULL    0x20  /* FIFO full */

/* FUNC_CFG_ACCESS - Memory bank selection */
#define LSM6DSV16X_FUNC_CFG_EMB_FUNC_REG_ACCESS 0x80
#define LSM6DSV16X_FUNC_CFG_SHUB_REG_ACCESS     0x40

/* EMB_FUNC_EN_A - SFLP enable */
#define LSM6DSV16X_EMB_FUNC_EN_A_SFLP_GAME_EN   0x02

/* EMB_FUNC_FIFO_EN_A - SFLP FIFO batching */
#define LSM6DSV16X_SFLP_GAME_FIFO_EN    0x02
#define LSM6DSV16X_SFLP_GRAVITY_FIFO_EN 0x04
#define LSM6DSV16X_SFLP_GBIAS_FIFO_EN   0x01

/* EMB_FUNC_INIT_A - SFLP reset */
#define LSM6DSV16X_SFLP_GAME_INIT       0x01

/* ======================== FIFO Tags ======================== */
#define LSM6DSV16X_FIFO_TAG_GYRO        0x01  /* Gyroscope NC */
#define LSM6DSV16X_FIFO_TAG_ACCEL       0x02  /* Accelerometer NC */
#define LSM6DSV16X_FIFO_TAG_TIMESTAMP   0x04  /* Timestamp */
#define LSM6DSV16X_FIFO_TAG_SFLP_QUAT   0x13  /* SFLP Game Rotation (Quaternion) */
#define LSM6DSV16X_FIFO_TAG_SFLP_GRAVITY 0x14 /* SFLP Gravity Vector */
#define LSM6DSV16X_FIFO_TAG_SFLP_GBIAS  0x15  /* SFLP Gyro Bias */

/* ======================== Memory Banks ======================== */
#define LSM6DSV16X_MEM_BANK_MAIN        0x00
#define LSM6DSV16X_MEM_BANK_EMBED       0x01
#define LSM6DSV16X_MEM_BANK_SENSOR_HUB  0x02

/* ======================== Constants ======================== */

/* I2C addresses */
#define LSM6DSV16X_I2C_ADDR_LOW         0x6A  /* SA0 = GND */
#define LSM6DSV16X_I2C_ADDR_HIGH        0x6B  /* SA0 = VDD */

/* FIFO size */
#define LSM6DSV16X_FIFO_SIZE            4608  /* 4.5 KB */
#define LSM6DSV16X_FIFO_SAMPLE_SIZE     7     /* 1 tag + 6 data bytes */

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSV16X_REGS_H_ */
