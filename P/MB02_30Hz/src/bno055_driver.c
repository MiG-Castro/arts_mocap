/* Hecho por M.C. Miguel - 2025!
   BNO055 library for Zephyr RTOS!
   
   For an unknow reason (and contrary what the datasheet says)
   1. The BNO dont reset on power on, so you need to reset it manually.
   2. The BNO need a "big" delay after reset to star working correctly, I recommend 800ms.
   3. The BNO need a "big" delay after set the external crystal, even if the system clock status is ok ... I recommend 650ms.
   To change configuration the BNO needs to be set to CONFIG MODE first
*/

#include "bno055_driver.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

//==================================================================================================
// CONFIGURACIÓN Y VARIABLES PRIVADAS
//==================================================================================================
LOG_MODULE_REGISTER(BNO, LOG_LEVEL_INF);
// Puntero estático al dispositivo I2C, se inicializa en bno055_init
static const struct device *i2c_dev;
#define BNO_CONFIG_DELAY_MS 25 // Pequeño delay después de cambiar de modo

//==================================================================================================
// IMPLEMENTACIÓN DE FUNCIONES PÚBLICAS
//==================================================================================================

bool bno055_reset(void)
{
    // CONFIG MODE -> PAGE 0 -> RESET -> CONFIG MODE -> SYS_TRIG = 0
    if (!bno055_set_config_mode()) return false;

    if (i2c_reg_write_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_PAGE_ID, BNO_PAGE_0) != 0) {
        LOG_ERR("Failed to set page ID to 0.");
        return false;
    }
    
    if (i2c_reg_write_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_SYS_TRIGGER, BNO_SYS_TRIGGER_RST_SYS) != 0) {
        LOG_ERR("Failed to write reset command.");
        return false;
    }
    k_msleep(800);

    if (!bno055_set_config_mode()) return false;

    if (i2c_reg_write_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_SYS_TRIGGER, 0x00) != 0) {
        LOG_ERR("Failed to write reset command.");
        return false;
    }

    LOG_INF("BNO055 reset successfully.");
    return true;
}

bool bno055_init(const struct device *i2c_dev_node, uint8_t initial_op_mode)
{
    i2c_dev = i2c_dev_node; // Asigna el puntero del dispositivo I2C
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready.");
        return false;
    }

    // RETASO PARA QUE INICIALICE EL BNO
    k_msleep(800);
    uint8_t chip_id = 0;
    if (i2c_reg_read_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_CHIP_ID, &chip_id) != 0) {
        LOG_ERR("Failed to read chip ID. BNO055 not found?");
        return false;
    }

    if (chip_id != 0xA0) {
        LOG_ERR("Invalid chip ID: 0x%02X (expected 0xA0)", chip_id);
        return false;
    }
    LOG_INF("BNO055 found with chip ID 0x%02X", chip_id);

    // RESET PARA ELIMINAR CUALQUIER CONFIGURACIÓN ANTERIOR
    if (!bno055_reset()) {
        LOG_ERR("Initial BNO055 reset failed.");
        return false;
    }

    // Configurar el modo de operación
    if (!bno055_change_opm(initial_op_mode)) {
        LOG_ERR("Failed to set initial operation mode.");
        return false;
    }
    
    LOG_INF("BNO055 initialized successfully in mode 0x%02X", initial_op_mode);
    return true;
}

bool bno055_read_config(uint8_t *config_values, bool print_v)
{
    uint8_t page_id, pwr_mode, opr_mode, unit_sel, sys_trigger;
    
    if (i2c_reg_read_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_PAGE_ID, &page_id) != 0 ||
        i2c_reg_read_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_PWR_MODE, &pwr_mode) != 0 ||
        i2c_reg_read_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_OPR_MODE, &opr_mode) != 0 ||
        i2c_reg_read_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_UNIT_SEL, &unit_sel) != 0 ||
        i2c_reg_read_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_SYS_TRIGGER, &sys_trigger) != 0) {
        LOG_ERR("Failed to read one or more configuration registers.");
        return false;
    }

    config_values[0] = page_id;
    config_values[1] = pwr_mode;
    config_values[2] = opr_mode;
    config_values[3] = unit_sel;
    config_values[4] = sys_trigger;
    
    if (print_v) printk("BNO Config: PageID=0x%02X, PwrMode=0x%02X, OprMode=0x%02X, Units=0x%02X, Trigger=0x%02X\n", page_id, pwr_mode, opr_mode, unit_sel, sys_trigger);
    return true;
}

bool bno055_calibration_status(uint8_t *cal_values)
{
    uint8_t cal_stat;
    if (i2c_reg_read_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_CALIB_STAT, &cal_stat) != 0) {
        LOG_ERR("Failed to read calibration status.");
        return false;
    }
    
    cal_values[0] = (cal_stat >> 6) & 0x03; // SYS
    cal_values[1] = (cal_stat >> 4) & 0x03; // ACC
    cal_values[2] = (cal_stat >> 2) & 0x03; // GYR
    cal_values[3] = cal_stat & 0x03;        // MAG
    // cal_values[4] = cal_values[1] + cal_values[2] + cal_values[3] + cal_values[4];
    
    LOG_INF("Calibration status: SYS=%d, ACC=%d, GYR=%d, MAG=%d",
            cal_values[0], cal_values[1], cal_values[2], cal_values[3]);
    return true;
}

bool bno055_set_config_mode(void)
{
    uint8_t current_op_mode;
    if (i2c_reg_read_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_OPR_MODE, &current_op_mode) != 0) {
        LOG_WRN("Could not read OPR_MODE to check if already in config mode.");
    }

    if (current_op_mode == BNO_OPR_MODE_CONFIG) {
        return true; // Ya está en modo config
    }

    if (i2c_reg_write_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_OPR_MODE, BNO_OPR_MODE_CONFIG) != 0) {
        LOG_ERR("Failed to set config mode.");
        return false;
    }
    k_msleep(BNO_CONFIG_DELAY_MS);
    return true;
}

bool bno055_change_opm(uint8_t new_op_mode)
{
    if (!bno055_set_config_mode()) return false;

    if (i2c_reg_write_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_OPR_MODE, new_op_mode) != 0) {
        LOG_ERR("Failed to set new operation mode 0x%02X", new_op_mode);
        return false;
    }
    k_msleep(BNO_CONFIG_DELAY_MS);
    return true;
}

bool bno055_calibration_offsets(bool write, int16_t *offsets)
{
    // The BNO has to be in CONFIG MODE to write or read the calibration offsets
    uint8_t current_mode;
    if (i2c_reg_read_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_OPR_MODE, &current_mode) != 0) {
        LOG_WRN("Failed to read original op mode.");
        return false;
    }

    if (!bno055_set_config_mode()) return false;

    bool success = false;
    uint8_t buffer[22];
    // WRITE OFFSETS ***************************************************************************
    if (write) {
        for (int i = 0; i < 11; i++) {
            buffer[i * 2] = offsets[i] & 0xFF;
            buffer[i * 2 + 1] = (offsets[i] >> 8) & 0xFF;
        }

        if (i2c_burst_write(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_ACC_OFFSET_X_LSB, buffer, 22) == 0) {
            LOG_INF("Calibration offsets written successfully.");
            success = true;
            k_msleep(40);
        } else LOG_ERR("Failed to write calibration offsets.");

    } else { 
    
        // READ OFFSETS **************************************************************************
        if (i2c_burst_read(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_ACC_OFFSET_X_LSB, buffer, 22) == 0) {
            for (int i = 0; i < 11; i++) {
                offsets[i] = (int16_t)(((uint16_t)buffer[i * 2 + 1] << 8) | (uint16_t)buffer[i * 2]);
            }
            LOG_INF("Calibration offsets read successfully.");
            success = true;
        } else LOG_ERR("Failed to read calibration offsets.");
    }

    // Restaurar el modo de operación original
    if (current_mode != BNO_OPR_MODE_CONFIG) bno055_change_opm(current_mode);
    return success;
}

bool bno055_change_crystal(bool set_external_crystal)
{
    uint8_t current_trigger, current_opm;
    if (i2c_reg_read_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_SYS_TRIGGER, &current_trigger) != 0) return false;

    bool is_external_now = (current_trigger & BNO_SYS_TRIGGER_EXT_CRYSTAL);
    if (set_external_crystal == is_external_now) {
        LOG_INF("Crystal already in desired state.");
        return true;
    }
    
    // Read the current opm -> set to config mode
    if (i2c_reg_read_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_OPR_MODE, &current_opm) != 0) return false;
    if (!bno055_set_config_mode()) return false;

    // Check if the BNO is ready to change the crystal
    uint8_t clk_ready;
    for (int i = 0; i < 10; i++) {
        i2c_reg_read_byte(i2c_dev, BNO_DEFAULT_ADDR, 0x38, &clk_ready); // 0x38 = system clock status register
        if (clk_ready & 0x01) k_msleep(100);	// bit0=1, BNO not ready to change the crystal
        else break;								// bit0=0, BNO ready to change the crystal
    }

    // CHANGE CRYSTAL SOURCE **************************************************
    uint8_t new_trigger_val = set_external_crystal ? BNO_SYS_TRIGGER_EXT_CRYSTAL : 0x00;
    if (i2c_reg_write_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_SYS_TRIGGER, new_trigger_val) != 0) {
        LOG_ERR("Failed to write to SYS_TRIGGER register.");
        return false;
    }
    k_msleep(650);  // Delay to allow the BNO to change the crystal

    // Check if the BNO finished the crystal configuration
    for (int i = 0; i < 10; i++) {
        i2c_reg_read_byte(i2c_dev, BNO_DEFAULT_ADDR, 0x38, &clk_ready);
        if (clk_ready & 0x01) k_msleep(100);	// bit0=1, BNO not ready
        else break;								// bit0=0, BNO ready
    }

    // Verificar el estado del cristal después de cambiarlo
    i2c_reg_read_byte(i2c_dev, BNO_DEFAULT_ADDR, BNO_REG_SYS_TRIGGER, &current_trigger);
    if (new_trigger_val != current_trigger) {
        LOG_ERR("Failed to change crystal");
        if (current_opm != BNO_OPR_MODE_CONFIG) bno055_change_opm(current_opm);
        return false;
    }
    
    LOG_INF("Crystal = 0x%02X (%s)", current_trigger, set_external_crystal ? "external" : "internal");
    // Restaurar el modo de operación original
    if (current_opm != BNO_OPR_MODE_CONFIG) bno055_change_opm(current_opm);
    return true;
}

bool bno055_read_raw_sensor_data(uint8_t start_register, uint8_t bytes_to_read, uint8_t *buffer, uint8_t start_index)
{
    if (i2c_burst_read(i2c_dev, BNO_DEFAULT_ADDR, start_register, &buffer[start_index], bytes_to_read) != 0) {
        LOG_WRN("Failed to read %d bytes from register 0x%02X", bytes_to_read, start_register);
        return false;
    }
    return true;
}

bool bno055_convert_to_units(const uint8_t *in_buffer, uint8_t in_buff_index, float *out_buffer, uint8_t out_buff_index, uint8_t sensor_source, uint8_t units)
{
    float div = 1.0f;
    int stop = 3; // 3 ejes (X,Y,Z) por defecto

    switch (sensor_source) {
        case BNO_REG_LIA:
        case BNO_REG_GRV:
        case BNO_REG_ACC:
            if (units & 0x01)   div = 1.0f;     // mg
            else                div = 100.0f;   // m/s^2
            break;
        case BNO_REG_GYR: 
            if (units & 0x02)   div = 900.0f;   // rps
            else                div = 16.0f;    // dps
            break;
        case BNO_REG_MAG: div = 16.0f; break;  // uT
        case BNO_REG_EUL:
            if (units & 0x04)   div = 900.0f;   // rad
            else                div = 16.0f;    // deg
            break;
        case BNO_REG_QUAT:
            div = 16384.0f;
            stop = 4; // W,X,Y,Z
            break;
        case BNO_REG_TEMP:
            div = 1.0f; // 1 LSB = 1 C
            stop = 1;
            break;
        default:
            LOG_WRN("Unknown sensor source for unit conversion: 0x%02X", sensor_source);
            return false;
    }
    
    for (int i = 0; i < stop; i++) {
        int16_t raw_val = (int16_t)sys_get_le16(&in_buffer[in_buff_index + i * 2]);
        out_buffer[out_buff_index + i] = (float)raw_val / div;
    }
    
    return true;
}