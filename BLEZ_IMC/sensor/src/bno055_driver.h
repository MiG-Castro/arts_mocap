/* 
Made by Miguel Angel Castro-Perez- 2025! 
BNO055 library for Zephyr RTOS!
*/

#ifndef BNO055_DRIVER_H_
#define BNO055_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdbool.h>

//==================================================================================================
// BNO055 REGISTERS - Public
//==================================================================================================
#define BNO_DEFAULT_ADDR    0x28    //0x28 or 0x29
#define BNO_REG_CHIP_ID     0x00
#define BNO_REG_PAGE_ID     0x07

// Page 0
#define BNO_PAGE_0			    0x00
#define BNO_REG_SYS_TRIGGER	    0x3F
#define BNO_SYS_TRIGGER_RST_SYS	0x20
#define BNO_SYS_TRIGGER_EXT_CRYSTAL	0x80
#define BNO_REG_PWR_MODE	    0x3E
#define BNO_PWR_MODE_NORMAL		0x00
#define BNO_REG_OPR_MODE	    0x3D
#define BNO_OPR_MODE_CONFIG		0x00
#define BNO_OPR_MODE_AMG		0x07
#define BNO_OPR_MODE_IMU		0x08
#define BNO_OPR_MODE_NDOF		0x0C
#define BNO_REG_CALIB_STAT	    0x35
#define BNO_REG_UNIT_SEL 	    0x3B

// Sensor Data Registers (Page 0)
#define BNO_REG_ACC   0X08
#define BNO_REG_GYR   0X14
#define BNO_REG_MAG   0X0E
#define BNO_REG_EUL   0X1A
#define BNO_REG_QUAT  0X20
#define BNO_REG_LIA   0X28
#define BNO_REG_GRV   0X2E
#define BNO_REG_TEMP  0X34

// Sensor Data Registers (Short Name)
#define acc     BNO_REG_ACC
#define gyr     BNO_REG_GYR
#define mag     BNO_REG_MAG
#define eul     BNO_REG_EUL
#define quat    BNO_REG_QUAT
#define lnAc    BNO_REG_LIA
#define grav    BNO_REG_GRV
#define temp    BNO_REG_TEMP

// Page 1
#define BNO_PAGE_1         0x01
#define BNO_REG_ACC_CONF	0x08	// Acc Configuration, Default 0x38 [00111000]=4G,62.5Hz,NM
#define BNO_REG_GYR_CONF_0	0x0A	// Gyr Configuration, Default 0x38 [00111000]
#define BNO_REG_GYR_CONF_1	0x0B	// Gyr Configuration, Default 0x00 [00000000]=2000dps,32Hz,NM
#define BNO_REG_MAG_CONF	0x09	// Mag Configuration, Default 0x0B [00001011]=10Hz,Regular,NM
#define BNO_REG_ACC_OFFSET_X_LSB	0x55

//==================================================================================================
// PUBLIC API FUNCTIONS
//==================================================================================================

/**
 * @brief Initializes the I2C bus, resets the BNO055 and configures it in an operation mode.
 * @param i2c_dev_node Device Tree node for the I2C bus (e.g. DT_NODELABEL(i2c1)).
 * @param initial_op_mode The initial operation mode (e.g. BNO_OPR_MODE_NDOF).
 * @return true if initialization was successful.
 */
bool bno055_init(const struct device *i2c_dev_node, uint8_t initial_op_mode);

/**
 * @brief Reads, saves and prints the basic configuration registers of the BNO055.
 * @param config_values Pointer to a 5-byte array where the configuration values will be stored. [Page ID, Power Mode, Operation mode, Unit Selection, System Trigger].
 * @param print_v True to print the values.
 * @return true if the read was successful, false on error.
 */
bool bno055_read_config(uint8_t *config_values, bool print_v);

/**
 * @brief Reads the calibration status of the BNO055.
 * @param cal_values Pointer to a 4-byte array where the calibration states will be stored [SYS, GYR, ACC, MAG].
 * @return true if the read was successful, false on error.
 */
bool bno055_calibration_status(uint8_t *cal_values);

/**
 * @brief Sets the BNO055 into configuration mode.
 * @return true if the operation was successful, false on error.
 */
bool bno055_set_config_mode(void);

/**
 * @brief Changes the operation mode of the BNO055.
 * @param new_op_mode The new operation mode (e.g. BNO_OPR_MODE_IMU).
 * @return true if the operation was successful, false on error.
 */
bool bno055_change_opm(uint8_t new_op_mode);

/**
 * @brief Reads or writes the calibration offsets of the BNO055.
 * @param write If true, writes the offsets to the BNO055. If false, reads them.
 * @param offsets Pointer to an array of 11 int16_t for the offset data. [Axyz, Mxyz, Gxyz, AccRad, MagRad].
 * @return true if the operation was successful, false on error.
 */
bool bno055_calibration_offsets(bool write, int16_t *offsets);

/**
 * @brief Changes the clock source of the BNO055 between internal and external crystal.
 * @param set_external_crystal If true, uses the external crystal. If false, uses the internal one.
 * @return true if the operation was successful, false on error.
 */
bool bno055_change_crystal(bool set_external_crystal);

/**
 * @brief Performs a system reset on the BNO055.
 * @return true if the operation was successful, false on error.
 */
bool bno055_reset(void);

/**
 * @brief Reads raw data from a starting register of the BNO055.
 * @param start_register Starting register to read from.
 * @param bytes_to_read Number of bytes to read.
 * @param buffer Pointer to the buffer where the data will be stored.
 * @param start_index Buffer index where storing should begin.
 * @return true if the read was successful, false on error.
 */
bool bno055_read_raw_sensor_data(uint8_t start_register, uint8_t bytes_to_read, uint8_t *buffer, uint8_t start_index);

/**
 * @brief Converts raw data to their corresponding units.
 * @param in_buffer Buffer containing the raw data.
 * @param in_buff_index Start index in the input buffer.
 * @param out_buffer Buffer for the converted data (float).
 * @param out_buff_index Start index in the output buffer.
 * @param sensor_source Sensor|Register source of the data. Acc|LnAcc|Grav = acc.
 * @param units Value of the Select_Unit register (0x3B).
 * @return true if the conversion was valid, false otherwise.
 */
bool bno055_convert_to_units(const uint8_t *in_buffer, uint8_t in_buff_index, float *out_buffer, uint8_t out_buff_index, uint8_t sensor_source, uint8_t units);


#ifdef __cplusplus
}
#endif

#endif /* BNO055_DRIVER_H_ */