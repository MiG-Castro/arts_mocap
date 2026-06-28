/* Hecho por M.C. Miguel - 2025! 
   BNO055 library for Zephyr RTOS!
   
   For an unknow reason (and contrary what the datasheet says)
   1. The BNO dont reset on power on, so you need to reset it manually.
   2. The BNO need a "big" delay after reset to star working correctly, I recommend 800ms.
   3. The BNO need a "big" delay after set the external crystal, even if the system clock status is ok ... I recommend 650ms.
   To change configuration the BNO needs to be set to CONFIG MODE first
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
// BNO055 REGISTERS - Públicos para que la aplicación pueda usarlos
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
 * @brief Inicializa el bus I2C, resetea el BNO055 y lo configura en un modo de operación.
 * 
 * @param i2c_dev_node Nodo del Device Tree para el bus I2C (ej. DT_NODELABEL(i2c1)).
 * @param initial_op_mode El modo de operación inicial (ej. BNO_OPR_MODE_NDOF).
 * @return true si la inicialización fue exitosa, false en caso contrario.
 */
bool bno055_init(const struct device *i2c_dev_node, uint8_t initial_op_mode);

/**
 * @brief Lee, guarda e imprime los registros de configuración básicos del BNO055.
 * @param config_values Puntero a un array de 5 bytes donde se guardaran los valores de configuracion. 
 * [Page ID, Power Mode, Operation mode, Unit Selection, System Trigger].
 * @param print_v True para imprimir los valores.
 * @return true si la lectura fue exitosa, false en caso de error.
 */
bool bno055_read_config(uint8_t *config_values, bool print_v);

/**
 * @brief Lee el estado de calibración del BNO055.
 * 
 * @param cal_values Puntero a un array de 4 bytes donde se guardarán los estados
 *                   de calibración [SYS, GYR, ACC, MAG].
 * @return true si la lectura fue exitosa, false en caso de error.
 */
bool bno055_calibration_status(uint8_t *cal_values);

/**
 * @brief Pone el BNO055 en modo de configuración.
 * 
 * @return true si la operación fue exitosa, false en caso de error.
 */
bool bno055_set_config_mode(void);

/**
 * @brief Cambia el modo de operación del BNO055.
 * 
 * @param new_op_mode El nuevo modo de operación (ej. BNO_OPR_MODE_IMU).
 * @return true si la operación fue exitosa, false en caso de error.
 */
bool bno055_change_opm(uint8_t new_op_mode);

/**
 * @brief Lee o escribe los offsets de calibración del BNO055.
 * 
 * @param write Si es true, escribe los offsets en el BNO055. Si es false, los lee.
 * @param offsets Puntero a un array de 11 int16_t para los datos de offset. 
 * [Axyz, Mxyz, Gxyz, AccRad, MagRad].
 * @return true si la operación fue exitosa, false en caso de error.
 */
bool bno055_calibration_offsets(bool write, int16_t *offsets);

/**
 * @brief Cambia la fuente de reloj del BNO055 entre el cristal interno y externo.
 * 
 * @param set_external_crystal Si es true, usa el cristal externo. Si es false, usa el interno.
 * @return true si la operación fue exitosa, false en caso de error.
 */
bool bno055_change_crystal(bool set_external_crystal);

/**
 * @brief Realiza un reseteo de sistema en el BNO055.
 * 
 * @return true si la operación fue exitosa, false en caso de error.
 */
bool bno055_reset(void);

/**
 * @brief Lee datos crudos de un registro inicial del BNO055.
 * 
 * @param start_register Registro inicial desde donde leer.
 * @param bytes_to_read Cantidad de bytes a leer.
 * @param buffer Puntero al buffer donde guardar los datos.
 * @param start_index Índice del buffer donde empezar a guardar.
 * @return true si la lectura fue exitosa, false en caso de error.
 */
bool bno055_read_raw_sensor_data(uint8_t start_register, uint8_t bytes_to_read, uint8_t *buffer, uint8_t start_index);

/**
 * @brief Convierte datos crudos a unidades físicas.
 * 
 * @param in_buffer Buffer con los datos crudos.
 * @param in_buff_index Índice de inicio en el buffer de entrada.
 * @param out_buffer Buffer para los datos convertidos (float).
 * @param out_buff_index Índice de inicio en el buffer de salida.
 * @param sensor_source Sensor|Registro de procedencia de datos. Acc|LnAcc|Grav = acc.
 * @param units Valor del registro Select_Unit(0x3B).
 * 
 * @return true si la conversión fue válida, false en caso contrario.
 */
bool bno055_convert_to_units(const uint8_t *in_buffer, uint8_t in_buff_index, float *out_buffer, uint8_t out_buff_index, uint8_t sensor_source, uint8_t units);


#ifdef __cplusplus
}
#endif

#endif /* BNO055_DRIVER_H_ */