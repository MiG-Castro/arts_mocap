/* Hecho por M.C. Miguel - 2025! 
   BNO055 example code for Zephyr RTOS!
   
   For an unknow reason (and contrary what the datasheet says)
   1. The BNO dont reset on power on, so you need to reset it manually.
   2. The BNO need a "big" delay after reset to star working correctly, I recommend 800ms.
   3. The BNO need a "big" delay after set the external crystal, even if the system clock status is ok ... I recommend 650ms.
*/

#include <stdio.h>
#include <string.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>    /* I2C */
#include <zephyr/drivers/gpio.h>   /* GPIO */
#include <zephyr/drivers/sensor.h> /* LSM6DSL sensor */


// BNO055 REGISTERS
#define BNO_DefaultAddr 0x28    //0x28 or 0x29
#define reg_BNO_ChipID  0x00
#define reg_page_id     0x07
#define page_0			0x00	// Default page****************************************************
#define reg_sis_trig	0x3F	// System Trigger, Default 0x00 [00000000]
#define reset_system	0x20	// Reset system configuration
#define ext_crystal		0x80	// External crystal
#define int_crystal		0x00	// Internal crystal
#define reg_pwr_mode	0x3E	// Power mode
#define normal_mode		0x00
#define reg_opr_mode	0x3D	// Operation mode
#define config_mode		0x00
#define amg_mode		0x07
#define imu_mode		0x08
#define ndof_mode		0x0C
#define reg_calib_sta	0x35	// Calibration status
#define reg_unit_sel 	0x3B	// Unit Selec - Default 0x80 [10000000]=m/s2,dps,degrees,°C,Android
#define reg_Acc_X_LSB   0X08	// Data output Acc
#define reg_Acc_X_MSB   0X09
#define reg_Acc_Y_LSB   0X0A
#define reg_Acc_Y_MSB   0X0B
#define reg_Acc_Z_LSB   0X0C
#define reg_Acc_Z_MSB   0X0D
#define reg_Mag_X_LSB   0X0E	// Data output Mag
#define reg_Mag_X_MSB   0X0F
#define reg_Mag_Y_LSB   0X10
#define reg_Mag_Y_MSB   0X11
#define reg_Mag_Z_LSB   0X12
#define reg_Mag_Z_MSB   0X13
#define reg_Gyr_X_LSB   0X14	// Data output Gyr
#define reg_Gyr_X_MSB   0X15
#define reg_Gyr_Y_LSB   0X16
#define reg_Gyr_Y_MSB   0X17
#define reg_Gyr_Z_LSB   0X18
#define reg_Gyr_Z_MSB   0X19
#define reg_Eul_H_LSB   0X1A	// Euler data -> reg 1A to 1F, T=6
#define reg_Quat_w_lsb  0X20	// Data output Quat
#define reg_Quat_w_msb  0X21
#define reg_Quat_x_lsb  0X22
#define reg_Quat_x_msb  0X23
#define reg_Quat_y_lsb  0X24
#define reg_Quat_y_msb  0X25
#define reg_Quat_z_lsb  0X26
#define reg_Quat_z_msb  0X27
#define reg_LnAc_X_LSB  0X28	// Linear acceleration data -> reg 28 to 2D, T=6
#define reg_Grav_X_LSB  0X2E	// Gravity data -> reg 2E to 33, T=6
#define reg_temperatur  0X34	// Temperature data -> reg 34, T=1
#define page_1 			    0x01	// SENSOR CONFIGURATION PAGE->1 *****************************
#define reg_acc_config		0x08	// Acc Configuration, Default 0x38 [00111000]=4G,62.5Hz,NM
#define reg_gyr_config_0	0x0A	// Gyr Configuration, Default 0x38 [00111000]
#define reg_gyr_config_1	0x0B	// Gyr Configuration, Default 0x00 [00000000]=2000dps,32Hz,NM
#define reg_mag_config		0x09	// Mag Configuration, Default 0x0B [00001011]=10Hz,Regular,NM
#define reg_AccX_LSB_OS		0x55	// Registers for offsets, begin with Acc X LSB (0x55), end with Mag Radius MSB (0x6A) = [22 registers]

// LED configuration
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec grn_led = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec blu_led = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

// Others variables
bool print_data = true, offsets_calibration = false;
uint8_t operation_mode = ndof_mode, units = 0x00, crystal=0x00;
#define sleep_config_ms 40
#define sleep_prints_ms 500

// to use in unit conversion function
#define accelerometer	reg_Acc_X_LSB
#define gyroscope		reg_Gyr_X_LSB
#define magnetometer	reg_Mag_X_LSB
#define euler			reg_Eul_H_LSB
#define quaternion		reg_Quat_w_lsb
#define linear_acc		reg_Acc_X_LSB
#define gravity			reg_Acc_X_LSB
#define temperature		reg_temperatur

// I2C configuration
#define I2C_NODE DT_NODELABEL(i2c1)
static const struct device *i2c1_dev = DEVICE_DT_GET(I2C_NODE);
int8_t calib_status_sys, calib_status_acc, calib_status_mag, calib_status_gyr;
int16_t calib_offsets[11] = { /* FOLLOW THE NEXT SECUENCE: Axyz, Mxyz, Gxyz, AccRad, MagRad */
	-11,  // OffSet_Ax
	15,   // OffSet_Ay
	-34,  // OffSet_Az
	341,  // OffSet_Mx
	-335, // OffSet_My
	-172, // OffSet_Mz
	0,    // OffSet_Gx
	0,    // OffSet_Gy
	-1,   // OffSet_Gz
	1000, // OffSet_AR (Acc Radius)
	752   // OffSet_MR (Mag Radius)
};

// FUNCTIONS DECLARATION ***************************************************************************
void read_config(void)
{	/* Read configuration registers and save some values in global variables*/
	uint8_t reg_read, out_1B;

	reg_read = reg_page_id;
	if (i2c_write_read(i2c1_dev, BNO_DefaultAddr, &reg_read, 1, &out_1B, 1) < 0) {
		if (print_data) printf("Error reading BNO055, page id");
	} else {
		if (print_data) printf("\nPage id: 0x%02X\n", out_1B);
	}

	reg_read = reg_pwr_mode;
	if (i2c_write_read(i2c1_dev, BNO_DefaultAddr, &reg_read, 1, &out_1B, 1) < 0) {
		if (print_data) printf("Error reading BNO055, pwm");
	} else {
		if (print_data) printf("Power Mode: 0x%02X\n", out_1B);
	}

	reg_read = reg_opr_mode;
	if (i2c_write_read(i2c1_dev, BNO_DefaultAddr, &reg_read, 1, &operation_mode, 1) < 0) {
		if (print_data) printf("Error reading BNO055, opm");
	} else {
		if (print_data) printf("Operation Mode: 0x%02X\n", operation_mode);
	}

	reg_read = reg_unit_sel;
	if (i2c_write_read(i2c1_dev, BNO_DefaultAddr, &reg_read, 1, &units, 1) < 0) {
		if (print_data) printf("Error reading BNO055, unit selection");
	} else {
		if (print_data) printf("Unit selection: 0x%02X\n", units);
	}

	reg_read = reg_sis_trig;
	if (i2c_write_read(i2c1_dev, BNO_DefaultAddr, &reg_read, 1, &crystal, 1) < 0) {
		if (print_data) printf("Error reading BNO055, system trigger");
	} else {
		if (print_data) printf("System trigger: 0x%02X\n\n", crystal);
	}
}

bool calibration_status(void)
{	/* Check the calibration status of the BNO055 */
	// CALIB_STAT Register (0x35): Contains the calibration status of the BNO055.
	// Values from 0 to 3 (2-bits). 0=00=Not Calibrated ... 11=3=Fully Calibrated.
	//  - bit0 and bit1 = magnetometer calibration status
	//  - bit2 and bit3 = gyroscope calibration status
	//  - bit4 and bit5 = accelerometer calibration status
	//  - bit6 and bit7 = system calibration status
	uint8_t reg_read = reg_calib_sta, out_1B = 0x00;
	bool ok = false;
	if (i2c_write_read(i2c1_dev, BNO_DefaultAddr, &reg_read, 1, &out_1B, 1) < 0) {
		if (print_data) printf("Error reading BNO055, calibration status");
		return ok;
	} else {
		calib_status_mag = out_1B & 0x03;
		calib_status_gyr = (out_1B >> 2) & 0x03;
		calib_status_acc = (out_1B >> 4) & 0x03;
		calib_status_sys = (out_1B >> 6) & 0x03;
		ok = calib_status_sys == 3 && calib_status_acc == 3 && calib_status_gyr == 3 && calib_status_mag == 3;
		if (print_data) printf("Calibration status: ok=%d, sys=%d, Acc=%d, Gyr=%d, Mag=%d\n", 
								ok, calib_status_sys, calib_status_acc, calib_status_gyr, calib_status_mag);
		return ok;
	}
}

void set_config_mode(void)
{	/* Set the operation modo to CONFIG MODE - This is the only way to change any other register */

	// Read operation mode
	uint8_t reg_2B[2] = {reg_opr_mode, 0xFF};
	i2c_write_read(i2c1_dev, BNO_DefaultAddr, &reg_2B[0], 1, &reg_2B[1], 1);
	if (reg_2B[1] == config_mode) return;	// Already in config mode
	
	reg_2B[1] = config_mode;
	if (i2c_write(i2c1_dev, reg_2B, 2, BNO_DefaultAddr) < 0) {
		gpio_pin_set_dt(&red_led, 1);
		if (print_data) printf("Error setting config mode\n");
		return;	
	}
	k_msleep(sleep_config_ms);
}

void change_opm(uint8_t new_opm)
{	/* Change the operation mode */

	// Read operation mode
	uint8_t reg_2B[2] = {reg_opr_mode, 0xFF};
	i2c_write_read(i2c1_dev, BNO_DefaultAddr, &reg_2B[0], 1, &reg_2B[1], 1);
	if (reg_2B[1] == new_opm) return;	// Already in the desired operation mode

	set_config_mode();					// Set config mode

	// Set new operation mode
	reg_2B[1] = new_opm;
	if (i2c_write(i2c1_dev, reg_2B, 2, BNO_DefaultAddr) < 0) {
		gpio_pin_set_dt(&red_led, 1);
		if (print_data) printf("Error changing operation mode: 0x%02Xn", new_opm);
		return;
	}
	k_msleep(sleep_config_ms);
}

bool calibration_offsets(bool set_offsets)
{	/* True=Set, False=Read -> Calibration offsets from the BNO055 (use the values from calib_offsets array)*/
	// The BNO has to be in CONFIG MODE to write or read the calibration offsets

	uint8_t reg_write_22B[23];			// Array for read/write registers 
	reg_write_22B[0] = reg_AccX_LSB_OS; // initial register for offsets

	uint8_t read_reg = reg_opr_mode;	// Read the current operation mode
	i2c_write_read(i2c1_dev, BNO_DefaultAddr, &read_reg, 1, &operation_mode, 1);
	set_config_mode();					// Set config mode

	if (set_offsets) {					// Set the calibration offsets
		// Take the calibration offsets values from calib_offsets array
		for (int i = 0; i < 21; i+=2) { // int16 array to uint8_t array
			reg_write_22B[i + 1] = (calib_offsets[i / 2]) & 0xFF;		// LSB
			reg_write_22B[i + 2] = (calib_offsets[i / 2] >> 8) & 0xFF;	// MSB
		}

		// Write the calibration offsets to the BNO055
		if (i2c_write(i2c1_dev, reg_write_22B, 23, BNO_DefaultAddr) < 0) {
			if (print_data) printf("Error writing BNO055, calibration offsets\n");
			if (operation_mode != config_mode) change_opm(operation_mode);
			return false;
		}

		k_msleep(sleep_config_ms);
		if (operation_mode != config_mode) change_opm(operation_mode);
		if (print_data) printf("Calibration offsets set successfully\n");
		return true;

	} else { // Read the calibration offsets and save them in calib_offsets array
		// Read the calibration status
		if (print_data && !calibration_status()) printf("Warning BNO not Fully Calibrated\n");

		uint8_t read_reg = reg_AccX_LSB_OS;
		if (i2c_write_read(i2c1_dev, BNO_DefaultAddr, &read_reg, 1, reg_write_22B, 22) < 0) {
			if (print_data) printf("Error reading BNO055, calibration offsets\n");
			if (operation_mode != config_mode) change_opm(operation_mode);
			return false;
		} else {
			// Convert the uint8_t array to int16_t array
			if (print_data) printf("Offsets Acc, Mag, Gyr, AccRad, MagRad: ");
			for (int i = 0; i < 11; i++) {
				calib_offsets[i] = (int16_t)((uint16_t)reg_write_22B[i * 2 + 1] << 8 | (uint16_t)reg_write_22B[i * 2]);
				if (print_data) printf("%d, ", calib_offsets[i]);
			}
			if (print_data) printf("\nCalibration offsets read&save successfully\n");
			if (operation_mode != config_mode) change_opm(operation_mode);
			return true;
		}
	}
}

void change_crystal(bool set_external_crystal)
{	/* Change the clock sourse of the BNO */
	// System trigger -> use external crystal
	// - Crystal must be configured AFTER loading calibration data into BNO055 (Acoording to an example of Adafruit BNO055 library)
	// - It takes minimum ~600ms to configure the external crystal and startup the BNO

	// Verify the crystal configuration - It takes to much time for the BNO to change the crystal, so its better to check it first
	uint8_t reg_read = reg_sis_trig;
	i2c_write_read(i2c1_dev, BNO_DefaultAddr, &reg_read, 1, &crystal, 1);
	bool is_external = (crystal & 0x80) ? true : false;

	if (set_external_crystal != is_external) {
		set_config_mode();
		if (print_data) printf("Set external crystal -> %d\n", set_external_crystal);

		// Check if the BNO is ready to change the crystal
		uint8_t read_2B[2] = {0x38, 0x00}; // 0x38 = system clock status register
		for (int i = 0; i < 10; i++) {
			i2c_write_read(i2c1_dev, BNO_DefaultAddr, &read_2B[0], 1, &read_2B[1], 1);
			if (print_data) printf("- BNO clock status (before): 0x%02X,\n", read_2B[1]);
			if (read_2B[1] & 0x01) k_msleep(100);	// bit0=1, BNO not ready to change the crystal
			else break;								// bit0=0, BNO ready to change the crystal
		}

		// Set crystal configuration
		uint8_t write_bno[2] = {reg_sis_trig, set_external_crystal ? ext_crystal : int_crystal};
		if (i2c_write(i2c1_dev, write_bno, 2, BNO_DefaultAddr) < 0) {
			gpio_pin_set_dt(&red_led, 1);
			if (print_data) printf("Error setting crystal configuration\n");
			if (operation_mode != config_mode) change_opm(operation_mode);
			return;
		}
		k_msleep(650); // "BIG" delay

		// Check if the BNO finished the crystal configuration
		read_2B[1] = 0xFF; // 0x38 = system clock status register
		for (int i = 0; i < 10; i++) {
			i2c_write_read(i2c1_dev, BNO_DefaultAddr, &read_2B[0], 1, &read_2B[1], 1);
			if (print_data) printf("- BNO clock status (after): 0x%02X,\n", read_2B[1]);
			if (read_2B[1] & 0x01) k_msleep(100);	// bit0=1, BNO not finished the crystal configuration
			else {									// bit0=0, BNO finished the crystal configuration
				// Check the crystal selecction, return to the last operation mode then finish
				i2c_write_read(i2c1_dev, BNO_DefaultAddr, &reg_read, 1, &crystal, 1);
				if (print_data) printf("-> Crystal 0x%02X\n", crystal);
				if (operation_mode != config_mode) change_opm(operation_mode);
				return;
			}
		}
	}
}

void reset_BNO(void)
{
	/* Config mode -> Page 0 -> Reset -> Config mode, return to default values = Page ID 0, PWM Normal */
	int total_error = 0;
	set_config_mode();

	// Set page 0
	uint8_t write_bno[2] = {reg_page_id, page_0};
	total_error += i2c_write(i2c1_dev, write_bno, 2, BNO_DefaultAddr);
	k_msleep(sleep_config_ms);

	// Reset system
	write_bno[0] = reg_sis_trig;
	write_bno[1] = reset_system;
	total_error += i2c_write(i2c1_dev, write_bno, 2, BNO_DefaultAddr);
	k_msleep(800);

	set_config_mode();

	// Return sys trigger to default value
	write_bno[0] = reg_sis_trig;
	write_bno[1] = 0x00;
	total_error += i2c_write(i2c1_dev, write_bno, 2, BNO_DefaultAddr);

	if (total_error < 0) {
		gpio_pin_set_dt(&red_led, 1);
		if (print_data) printf("Error resetting BNO055\n");
	}
}

void read_raw_sensor_data(uint8_t start_register, uint8_t bytes_to_read, uint8_t *buffer, uint8_t start_index)
{
	// Max buffer size considered in case to read all sensor data (Acc, Mag, Gyr, Eul, Quat, LinAcc, Grav and Temp)
	uint8_t out[45] = {0};
	if (i2c_write_read(i2c1_dev, BNO_DefaultAddr, &start_register, 1, out, bytes_to_read) < 0) {
		if (print_data) printf("Error reading BNO055, register 0x%02X\n", start_register);
		return;
	} else {
			for (int i = 0; i < bytes_to_read; i++) buffer[start_index + i] = out[i];
			if (print_data) printf("Read %d bytes, starting from 0x%02X register\n", bytes_to_read, start_register);
	}
}

void convert_to_units(const uint8_t *in_buffer, uint8_t in_buff_index, float32_t *out_buffer, uint8_t out_buff_index, uint8_t sensor_source, bool print_v, bool check_reg)
{	/* Convert the raw data of the sensor to the select unit */

	// in_buffer: Raw data from the sensor
	// in_buff_index: Index where the raw data begins in the in_buffer
	// out_buffer: Buffer to store the converted values
	// out_buff_index: Index where the converted values will be start stored in the out_buffer
	// sensor_source: type of sensor data to convert
	// print_v: If true, print the converted values
	// check_reg: If true, read unit_select register (0x3B) to determine the units
	// NOTE: The function cant diferentiate between acceleration, linear acceleration and gravity,
	//       so it will print all with the name "Acc"

	// The units depents of the values in the select_unit(0x3B) register:
	// - bit 0: Accelermeter -> 0=m/s2 (1m/s2 = 100 LSB), 1=mg  (1mg  = 100 LSB)
    // - bit 1: Gyroscope    -> 0=dps  (1dps  = 16  LSB), 1=rps (1rps = 900 LSB)
    // - bit 2: Euler Angles -> 0=deg  (1deg  = 16  LSB), 1=rad (1rad = 900 LSB)
    // - bit 3: Reserved
    // - bit 4: Temperature  -> 0=°C   (1°C   = 1   LSB), 1=F   (2F   = 2   LSB)
    // - bit 5: Reserved
    // - bit 6: Reserved
    // - bit 7: Orientation  -> 0=Android, 1=Windows
	// DEFAULT REGISTER VALUE 0x80=10000000= m/s2, dps, degrees, °C, Android
	// MAGNETOMETER UNITS: Always in uT (microteslas) - 1uT         = 16    LSB
	// QUATERNION UNITS: unit less                    - 1quaternion = 16384 LSB

	bool ok = false;						// Determines if the conversion is valid
	float32_t x[4] = {0.0f};				// Converted values
	float32_t div = 0.0f;					// Divisor factor
	int stop = 3;							// Maximum number of values to convert
	char info[20];							// Sensor information
	
	if (check_reg) {
		uint8_t reg_read = reg_unit_sel;	// Register to read the units
		i2c_write_read(i2c1_dev, BNO_DefaultAddr, &reg_read, 1, &units, 1);
	}

	switch (sensor_source)
	{
		case reg_Acc_X_LSB:
			// Accelerometer, linear acceleration and gravity
			ok = true;
			strcpy(info, "Acc(XYZ,m/s2): ");
			div = 100.0f;						// Default config bit0=0 -> m/s²
			if (units & 0x01) {					//                bit0=1 -> mg
				div = 1.0f;
				strcpy(info, "Acc(XYZ,mg): ");
			}
			break;
		case gyroscope:
			// Gyroscope
			ok = true;
			strcpy(info, "Gyr(XYZ,dps): ");
			div = 16.0f;						// Default config bit1=0 -> dps
			if (units & 0x02) {					//                bit1=1 -> rps
				div = 900.0f;
				strcpy(info, "Gyr(XYZ,rps): ");
			}
			break;
		case magnetometer:
			// Magnetometer in uT
			ok = true;
			strcpy(info, "Mag(XYZ,uT): ");
			div = 16.0f;						// Only in uT
			break;
		case euler:
			// Euler angles in degrees
			ok = true;
			strcpy(info, "Eul(HRP,deg): ");
			div = 16.0f;						// Default config bit2=0 -> deg
			if (units & 0x04) {					//                bit2=1 -> rad
				div = 900.0f;
				strcpy(info, "Eul(HRP,rad): ");
			}
			break;
		case quaternion:
			// Units less quaternion
			ok = true;
			strcpy(info, "Qua(WXYZ): ");
			stop = 4;							// Quaternion has 4 values
			div = 16384.0f;						// Default config bit2=0 -> deg
			break;
	}

	if (ok) {
		// Convert raw values to real values in their respective units and save!
		for (int i = 0; i < stop * 2; i += 2) {
			x[i / 2] = (float32_t)((int16_t)((uint16_t)in_buffer[in_buff_index + i + 1] << 8 | (uint16_t)in_buffer[in_buff_index + i])) / div;
			out_buffer[out_buff_index + (i / 2)] = x[i / 2];
		}
		
		if (print_data && print_v) {
			if (stop == 3) printf("%s%.4f, %.4f, %.4f - div=%0.1f\n", info, (double)x[0], (double)x[1], (double)x[2], (double)div);
			else printf("%s%.4f, %.4f, %.4f, %.4f - div=%0.1f\n", info, (double)x[0], (double)x[1], (double)x[2], (double)x[3], (double)div);
		}
	}
}

int main(void)
{
	// Initialize LED
	if (!gpio_is_ready_dt(&red_led) && !gpio_is_ready_dt(&grn_led) && !gpio_is_ready_dt(&blu_led)) {
		if (print_data) printf("Error led gpio\n");
		return -1;
	}
	gpio_pin_configure_dt(&red_led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&grn_led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&blu_led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set_dt(&red_led, 0);
	gpio_pin_set_dt(&grn_led, 0);
	gpio_pin_set_dt(&blu_led, 0);

	// Initial configuration
	k_msleep(800);
	if (!device_is_ready(i2c1_dev)) return -1;
	if (print_data) printf("\n\nI2C ready\n");

	reset_BNO();				// BNO reset
	calibration_offsets(true);	// Set calibration offsets
	calibration_offsets(false);	// read calibration offsets
	change_crystal(true);		// Set external crystal
	change_opm(ndof_mode);		// Set operation mode to NDOF
	read_config();				// Read configuration registers

	/* READ BNO055 OUTPUT DATA */
	uint8_t raw_output[26] = {0}; // Acc + Mag + Gyr + Quat = 26
	float32_t converted_output[13] = {0}; // Acc + Gyr + Mag + Quat = 13
	while(true) {
		k_msleep(2000); // Sleep to avoid flooding the output

		read_raw_sensor_data(reg_Acc_X_LSB, 18, raw_output, 0);		// Read Acc+Gyr+Mag data
		read_raw_sensor_data(reg_Quat_w_lsb, 8, raw_output, 18);	// Read quaternion data

		// Convert and print the sensors values
		convert_to_units(raw_output, 0, converted_output, 0, accelerometer, true, false);
		convert_to_units(raw_output, 6, converted_output, 3, magnetometer, true, false);
		convert_to_units(raw_output, 12, converted_output, 6, gyroscope, true, false);
		convert_to_units(raw_output, 18, converted_output, 9, quaternion, true, false);

		printf("************************************************************************\n");

		if (calibration_status()) gpio_pin_set_dt(&grn_led, 1);
		else gpio_pin_set_dt(&grn_led, 0);
	}
}