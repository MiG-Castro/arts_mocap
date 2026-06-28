/* Hecho por M.C. Miguel - 2025!
   Codigo de ejemplo para uso de la libreria personalizada BNO055!

   En caso de querer configurar otras cosas fuera de las funciones disponibles...
   El BNO debe de estar en CONFIG_MODE antes de realizar las configuraciones
*/
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include "bno055_driver.h"  // Libreria BNO!

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Nodo I2C del Device Tree
#define I2C_NODE DT_NODELABEL(i2c1)

/* NOTAS REGISTROS DE SALIDA DE LOS SENSORES|ALGORITMOS 
Los registros de los sensores principales estan en serie Acc(xyz)->Mag(xyz)->Gry(xyz)
Se pueden leer todos los valores (3 sensores x 3 ejes x 2 registros = 18) partiendo de ACC_X
ORDEN DE LOS REGISTROS DE SALIDA:
- acc  = BNO_REG_ACC_X_LSB   0x08 Acelerometro
- mag  = BNO_REG_MAG_X_LSB   0x0E Magnetometro
- gry  = BNO_REG_GYR_X_LSB   0x14 Gyroscopio
- eul  = BNO_REG_EUL_H_LSB   0X1A Angulos Euler (HRP)
- quat = BNO_REG_QUAT_W_LSB  0X20 Quaterniones (WXYZ)
- lnAc = BNO_REG_LIA_X_LSB   0X28 Aceleracion Lineal
- grav = BNO_REG_GRV_X_LSB   0X2E Gravedad
- temp = BNO_REG_TEMP        0X34 Temperatura
*/
/* NOTAS CONVERSION A UNIDADES 
The units depents of the values in the select_unit(0x3B) register:
- bit 0: Accelermeter -> 0=m/s2 (1m/s2 = 100 LSB), 1=mg  (1mg  = 100 LSB)
         LnAcc||Gravty
- bit 1: Gyroscope    -> 0=dps  (1dps  = 16  LSB), 1=rps (1rps = 900 LSB)
- bit 2: Euler Angles -> 0=deg  (1deg  = 16  LSB), 1=rad (1rad = 900 LSB)
- bit 3: Reserved
- bit 4: Temperature  -> 0=°C   (1°C   = 1   LSB), 1=F   (2F   = 2   LSB)
- bit 5: Reserved
- bit 6: Reserved
- bit 7: Orientation  -> 0=Android, 1=Windows
MAGNETOMETER UNITS: Always in uT (microteslas) - 1uT = 16 LSB
QUATERNION UNITS: unit less - 1 quaternion = 16384 LSB
DEFAULT REGISTER VALUE = 0x80 = 10000000 = m/s2, dps, degrees, °C, Android
*/
/* NOTAS OFFSETS DE CALIBRACION
La calibracion es un proceso automatico que SIEMPRE esta ejecutando el BNO
Los offsets se ajustan (CAMBIAN) segun el algoritmo de calibracion del BNO
Cargar los offsets ayuda a "acelerar" el proceso de estabilizacion del algoritmo de calibracion
El orden de los registros del los offset es: Axyz, Mxyz, Gxyz, AccRad, MagRad
*/
/* NOTAS MODOS DE OPERACION
Modos de operacion principales:
BNO_OPR_MODE_AMG  -> Todos los sensores disponibles (AGM) SIN FUSION DE SENSORES
BNO_OPR_MODE_NDOF -> Todos los sensores disponibles (AGM) + FUSION DE SENSORES
BNO_OPR_MODE_IMU  -> Solo Acc & Gyr + Fusion de sensores
Las salidas de los algoritmos de fusion son:
- Angulos de Euler
- Cuaterniones
- Aceleracion lineal
- Vector de Gravedad
*/

int main(void)
{
    gpio_pin_configure_dt(&red_led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set_dt(&red_led, 0);
    
    // Inicializar el BNO055
    const struct device *i2c_bus = DEVICE_DT_GET(I2C_NODE);
    if (!bno055_init(i2c_bus, BNO_OPR_MODE_NDOF)) {
        printk("BNO055 initialization failed.\n");
        gpio_pin_set_dt(&red_led, 1);
        while (1);
    }

    // Opcional: Cargar offsets de calibración si los tienes guardados. Ejemplo:
    // int16_t calib_offsets[11] = {
    //     -11,  // OffSet_Ax
    //     15,   // OffSet_Ay
    //     -34,  // OffSet_Az
    //     341,  // OffSet_Mx
    //     -335, // OffSet_My
    //     -172, // OffSet_Mz
    //     0,    // OffSet_Gx
    //     0,    // OffSet_Gy
    //     -1,   // OffSet_Gz
    //     1000, // OffSet_AR (Acc Radius)
    //     752   // OffSet_MR (Mag Radius)
    // };
    // Set the calibration offsets
    // bno055_calibration_offsets(true, my_offsets);

    // Opcional: Cambiar a cristal externo. Supuestamente es mejor...
    bno055_change_crystal(true);

    // Variables para almacenar los datos
    uint8_t raw_data[26];
    float converted_data[13];
    uint8_t cal_status[4];
    uint8_t config_values[5];
    
    // Leemos la configuracion
    bno055_read_config(config_values, false);
    uint8_t units = config_values[3];

    // Bucle principal para leer datos
    while(1) {
        k_msleep(20);
        bno055_calibration_status(cal_status);
        printk("Cal: SYS=%d, GYR=%d, ACC=%d, MAG=%d\n", cal_status[0], cal_status[1], cal_status[2], cal_status[3]);

        // Leemos los datos crudos
        if (bno055_read_raw_sensor_data(acc, 18, raw_data, 0) && 
            bno055_read_raw_sensor_data(quat, 8, raw_data, 18)) {
            
            // Convertir segun sus unidades
            bno055_convert_to_units(raw_data, 0, converted_data, 0, acc, units);
            bno055_convert_to_units(raw_data, 6, converted_data, 3, mag, units);
            bno055_convert_to_units(raw_data, 12, converted_data, 6, gyr, units);
            bno055_convert_to_units(raw_data, 18, converted_data, 9, quat, units);
            
            // Imprimir o usar los datos convertidos
            // printk("ACC: x=%.3f y=%.3f z=%.3f\n", (double)converted_data[0], 
            //       (double)converted_data[1], (double)converted_data[2]);

            // printk("MAG: x=%.3f y=%.3f z=%.3f\n", (double)converted_data[3], 
            //       (double)converted_data[4], (double)converted_data[5]);

            // printk("GYR: x=%.3f y=%.3f z=%.3f\n", (double)converted_data[6], 
            //       (double)converted_data[7], (double)converted_data[8]);

            // printk("QUAT: w=%.3f x=%.3f y=%.3f z=%.3f\n\n", (double)converted_data[9], 
            //       (double)converted_data[10], (double)converted_data[11], (double)converted_data[12]);

            printk("%.3f %.3f %.3f\n", (double)converted_data[6], (double)converted_data[7], (double)converted_data[8]);
        }
    }
    return 0;
}