#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <stdio.h>

// Address of the sensor
#define LSM6DS3TRC 0x6A
#define ADDR_WHO_AM_I 0x0F // Default value = 01101010 = 0x6A

// Principal control registers
#define ADDR_CTRL1_XL 0x10  // Config Acc
#define ADDR_CTRL2_G  0x11  // Config Gyr
#define ADDR_CTRL3_C  0x12  // Config read
// Config values for CTRL3_C
#define BDU_EN 0x40
#define IF_INC_EN 0x04

// Output registers
#define OUT_TEMP_L 0x20
#define OUTX_L_G 0x22
#define OUTY_L_G 0x24
#define OUTZ_L_G 0x26
#define OUTX_L_XL 0x28
#define OUTY_L_XL 0x2A
#define OUTZ_L_XL 0x2C

// Sensor Output data rate
#define ODR_104 0x40 // 0100 0000
#define ODR_416 0x60 // 0110 0000

// Scale of the sensors
#define XL_2G  0x00 // 0000 0000
#define XL_16G 0x04 // 0000 0100
#define XL_4G  0x08 // 0000 1000
#define XL_8G  0x0C // 0000 1100
#define G_125_DPS  0x02 // 0000 0010
#define G_250_DPS  0x00 // 0000 0000
#define G_500_DPS  0x04 // 0000 0100
#define G_1000_DPS 0x08 // 0000 1000
#define G_2000_DPS 0x0C // 0000 1100

// Sensitivity for each scale
#define S_TEMP 256.0f
#define S_XL_2g  0.061f
#define S_XL_4g  0.122f
#define S_XL_8g  0.244f
#define S_XL_16g 0.488f
#define S_G_125  4.375f
#define S_G_250  8.75f
#define S_G_500  17.5f
#define S_G_1000 35.0f
#define S_G_2000 70.0f

// Constants
#define  gravity_ms2 9.80665f

static uint8_t config_gyr = 0, config_acc = 0, config_read = 0;
static const struct device *i2c_lsm = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lsm6ds3tr_c)));

/*
CTRL1_XL (10h) default 00000000
ODR_XL3 ODR_XL2 ODR_XL1 ODR_XL0          FS_XL1 FS_XL0        LPF1_BW_SEL      BW0_XL (ODR>=1.6kHz)
0       1       0       0       = 104Hz  0      0      = 2g   0 -> BW = ODR/2  0 -> BW @ 1.5kHz
0       1       1       0       = 416Hz  0      1      = 16g  1 -> BW = ODR/2  1 -> BW @ 400Hz
                                         1      0      = 4g
                                         1      1      = 8g


CTRL2_G (11h) default 00000000
ODR_G3  ODR_G2  ODR_G1  ODR_G0           FS_G1  FS_G0            FS_125                   0
0       1       0       0       = 104Hz  0      0      = 250dps  0 = Scale125dps disable  SET IN CERO
0       1       1       0       = 416Hz  0      1      = 500dps  1 = Scale125dps enable
                                         1      0      = 1000dps
                                         1      1      = 2000dps

CTRL3_C (12h) default 00000100
BOOT BDU H_LACTIVE PP_OD SIM IF_INC BLE SW_RESET
0    1   0         0     0   1      0   0       = 0x44
Config 0x44 to:
BDU = 1 : output registers not updated until MSB and LSB have been read
IF_INC = 1: Register address automatically incremented during a multiple byte access with a serial interface
*/

int main(void)
{
    if (!device_is_ready(i2c_lsm)) {
        printk("I2C not ready\n");
        return 0;
    }

    // Fast test
    // uint8_t who_am_i = 0;
    // i2c_reg_read_byte(i2c_lsm, LSM6DS3TRC, ADDR_WHO_AM_I, &who_am_i);
    // printk("WHO_AM_I 0x%02X\n", who_am_i);

    // Config registers
    config_gyr = ODR_104 | G_250_DPS;
    config_acc = ODR_104 | XL_2G;
    config_read = BDU_EN | IF_INC_EN;

    i2c_reg_write_byte(i2c_lsm, LSM6DS3TRC, ADDR_CTRL1_XL, config_acc);
    i2c_reg_write_byte(i2c_lsm, LSM6DS3TRC, ADDR_CTRL2_G, config_gyr);
    i2c_reg_write_byte(i2c_lsm, LSM6DS3TRC, ADDR_CTRL3_C, config_read);

    uint8_t raw_data[12];
    int16_t gyr[3], acc[3];
    float gyr_dps[3], acc_g[3];

    while (1) {
        i2c_burst_read(i2c_lsm, LSM6DS3TRC, OUTX_L_G, raw_data, 12);
        for (int i = 0; i < 3; i++) {
            gyr[i] = (int16_t)sys_get_le16(&raw_data[i * 2]);
            // gyr_dps[i] = (float)gyr[i] * S_G_250; // to mdps
            gyr_dps[i] = (float)gyr[i] * (S_G_250/1000); // to dps
        }

        for (int i = 0; i < 3; i++) {
            acc[i] = (int16_t)sys_get_le16(&raw_data[i * 2 + 6]);
            // acc_g[i] = (float)acc[i] * S_XL_2g; // to 'mg' units
            acc_g[i] = (float)acc[i] * (S_XL_2g/1000); // to 'g' units
            // acc_g[i] = (float)acc[i] * (S_XL_2g/1000) * gravity_ms2 ; // to 'm/s2' units
        }

        printk("%f %f %f\n", (double)acc_g[0], (double)acc_g[1], (double)acc_g[2]); // Acc
        printk("%f %f %f\n", (double)gyr_dps[0], (double)gyr_dps[1], (double)gyr_dps[2]); // Gyr
        k_sleep(K_MSEC(100));
    }
}