# Guía de Integración Paso a Paso - LSM6DSV16X en Zephyr

## Paso 1: Estructura de Directorios

Crea la siguiente estructura en tu proyecto Zephyr:

```
mi_proyecto/
├── src/
│   ├── main.c
│   └── lsm6dsv16x/
│       ├── lsm6dsv16x.h
│       ├── lsm6dsv16x.c
│       ├── lsm6dsv16x_regs.h
│       └── Kconfig
├── boards/
│   └── mi_placa.overlay
├── prj.conf
└── CMakeLists.txt
```

## Paso 2: Copiar Archivos del Driver

Copia los siguientes archivos a `src/lsm6dsv16x/`:
- `lsm6dsv16x.h`
- `lsm6dsv16x.c`
- `lsm6dsv16x_regs.h`
- `Kconfig`

## Paso 3: Modificar CMakeLists.txt

Añade el driver a tu proyecto:

```cmake
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mi_proyecto)

# Añadir directorio del driver
target_sources(app PRIVATE 
    src/main.c
    src/lsm6dsv16x/lsm6dsv16x.c
)

# Añadir directorio de includes
target_include_directories(app PRIVATE
    src/lsm6dsv16x
)
```

## Paso 4: Configurar Device Tree

Edita `boards/mi_placa.overlay`:

```dts
/* LSM6DSV16X IMU Sensor Configuration */

&i2c1 {
    status = "okay";
    clock-frequency = <I2C_BITRATE_FAST>;  /* 400 kHz recomendado */
    
    lsm6dsv16x: lsm6dsv16x@6b {
        compatible = "st,lsm6dsv16x";
        reg = <0x6b>;  /* 0x6a si SA0 conectado a GND */
        label = "LSM6DSV16X_IMU";
    };
};
```

**Notas:**
- Usa `@6b` si el pin SA0 está conectado a VDD
- Usa `@6a` si el pin SA0 está conectado a GND
- Asegúrate de que el bus I2C esté habilitado en tu placa

## Paso 5: Configurar prj.conf

Añade las siguientes configuraciones a `prj.conf`:

```ini
# ===== I2C Configuration =====
CONFIG_I2C=y

# ===== Logging =====
CONFIG_LOG=y
CONFIG_LOG_MODE_IMMEDIATE=y

# ===== LSM6DSV16X Driver =====
CONFIG_LSM6DSV16X=y
CONFIG_LSM6DSV16X_LOG_LEVEL=3  # 0=OFF, 1=ERROR, 2=WARN, 3=INFO, 4=DEBUG

# ===== Optional: Default configurations =====
CONFIG_LSM6DSV16X_I2C_ADDR=0x6B
CONFIG_LSM6DSV16X_ACCEL_FS_DEFAULT=4
CONFIG_LSM6DSV16X_GYRO_FS_DEFAULT=2000
CONFIG_LSM6DSV16X_ACCEL_ODR_DEFAULT=120
CONFIG_LSM6DSV16X_GYRO_ODR_DEFAULT=120

# ===== Optional: Enable SFLP by default =====
# CONFIG_LSM6DSV16X_SFLP_ENABLE_DEFAULT=y
# CONFIG_LSM6DSV16X_SFLP_ODR_DEFAULT=120

# ===== Optional: Enable filters =====
# CONFIG_LSM6DSV16X_ACCEL_LPF_ENABLE=y
# CONFIG_LSM6DSV16X_GYRO_LPF_ENABLE=y

# ===== Math library (required for sqrtf) =====
CONFIG_NEWLIB_LIBC=y
CONFIG_NEWLIB_LIBC_FLOAT_PRINTF=y
```

## Paso 6: Código Mínimo de Ejemplo

Crea `src/main.c` con el siguiente contenido mínimo:

```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include "lsm6dsv16x/lsm6dsv16x.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define I2C_NODE DT_NODELABEL(i2c1)

int main(void)
{
    int ret;
    lsm6dsv16x_config_t imu_config = {
        .i2c_dev = DEVICE_DT_GET(I2C_NODE),
        .i2c_addr = LSM6DSV16X_I2C_ADDR_HIGH,
        .accel_fs = LSM6DSV16X_ACCEL_FS_4G,
        .gyro_fs = LSM6DSV16X_GYRO_FS_2000_DPS,
        .sflp_enabled = false
    };

    LOG_INF("LSM6DSV16X Test Starting...");

    /* Verificar que I2C esté listo */
    if (!device_is_ready(imu_config.i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -1;
    }

    /* Inicializar sensor */
    ret = lsm6dsv16x_init(&imu_config);
    if (ret < 0) {
        LOG_ERR("Failed to initialize sensor: %d", ret);
        return -1;
    }

    /* Configurar acelerómetro */
    ret = lsm6dsv16x_accel_config(&imu_config,
                                   LSM6DSV16X_ACCEL_ODR_120_HZ,
                                   LSM6DSV16X_ACCEL_FS_4G);
    if (ret < 0) {
        LOG_ERR("Failed to configure accelerometer");
        return -1;
    }

    /* Configurar giroscopio */
    ret = lsm6dsv16x_gyro_config(&imu_config,
                                  LSM6DSV16X_GYRO_ODR_120_HZ,
                                  LSM6DSV16X_GYRO_FS_2000_DPS);
    if (ret < 0) {
        LOG_ERR("Failed to configure gyroscope");
        return -1;
    }

    LOG_INF("Sensor initialized successfully!");

    /* Loop principal */
    lsm6dsv16x_accel_raw_t accel_raw;
    lsm6dsv16x_gyro_raw_t gyro_raw;
    lsm6dsv16x_accel_mg_t accel_mg;
    lsm6dsv16x_gyro_mdps_t gyro_mdps;

    while (1) {
        /* Leer acelerómetro */
        ret = lsm6dsv16x_accel_read_raw(&imu_config, &accel_raw);
        if (ret == 0) {
            lsm6dsv16x_accel_raw_to_mg(&accel_raw, imu_config.accel_fs, &accel_mg);
            LOG_INF("Accel: X=%6.2f Y=%6.2f Z=%6.2f mg",
                    (double)accel_mg.x, (double)accel_mg.y, (double)accel_mg.z);
        }

        /* Leer giroscopio */
        ret = lsm6dsv16x_gyro_read_raw(&imu_config, &gyro_raw);
        if (ret == 0) {
            lsm6dsv16x_gyro_raw_to_mdps(&gyro_raw, imu_config.gyro_fs, &gyro_mdps);
            LOG_INF("Gyro:  X=%7.1f Y=%7.1f Z=%7.1f mdps",
                    (double)gyro_mdps.x, (double)gyro_mdps.y, (double)gyro_mdps.z);
        }

        k_msleep(100);
    }

    return 0;
}
```

## Paso 7: Compilar y Flashear

```bash
# Compilar
west build -b mi_placa

# Flashear
west flash

# Ver logs (si tienes puerto serial)
west attach
```

## Paso 8: Verificación

Deberías ver en la consola:

```
*** Booting Zephyr OS ... ***
[00:00:00.100,000] <inf> main: LSM6DSV16X Test Starting...
[00:00:00.150,000] <inf> lsm6dsv16x: LSM6DSV16X detected (WHO_AM_I: 0x70)
[00:00:00.200,000] <inf> lsm6dsv16x: LSM6DSV16X initialized successfully
[00:00:00.250,000] <inf> main: Sensor initialized successfully!
[00:00:00.350,000] <inf> main: Accel: X=  12.20 Y=  -3.66 Z= 998.78 mg
[00:00:00.350,000] <inf> main: Gyro:  X=   -2.1 Y=    1.4 Z=   -0.7 mdps
```

## Troubleshooting

### Error: "I2C device not ready"
**Solución:**
- Verifica que el I2C esté habilitado en Device Tree
- Revisa los pines de I2C en tu placa
- Asegúrate de tener pull-ups en SDA/SCL

### Error: "Wrong WHO_AM_I"
**Solución:**
- Verifica la dirección I2C (0x6A vs 0x6B)
- Revisa el cableado del sensor
- Confirma voltaje de alimentación (1.71V-3.6V)

### Error: "Undefined reference to sqrtf"
**Solución:**
- Añade a `prj.conf`:
  ```ini
  CONFIG_NEWLIB_LIBC=y
  ```

### No se ven datos
**Solución:**
- Verifica que los sensores estén habilitados (ODR > 0)
- Revisa el nivel de log: `CONFIG_LSM6DSV16X_LOG_LEVEL=4`
- Intenta con un software reset: `lsm6dsv16x_reset(&config)`

## Ejemplo Avanzado con SFLP

Para habilitar SFLP y leer quaternions:

```c
/* Habilitar SFLP */
ret = lsm6dsv16x_sflp_enable(&imu_config, LSM6DSV16X_SFLP_ODR_120_HZ);
if (ret < 0) {
    LOG_ERR("Failed to enable SFLP");
    return -1;
}

/* Loop de lectura FIFO */
while (1) {
    uint16_t fifo_count;
    lsm6dsv16x_fifo_sample_t sample;
    lsm6dsv16x_quaternion_t quat;

    /* Obtener número de muestras */
    lsm6dsv16x_fifo_get_count(&imu_config, &fifo_count);

    /* Leer todas las muestras */
    for (int i = 0; i < fifo_count; i++) {
        lsm6dsv16x_fifo_read_sample(&imu_config, &sample);

        if (sample.tag == LSM6DSV16X_FIFO_TAG_SFLP_QUAT) {
            /* Convertir a quaternion */
            lsm6dsv16x_sflp_to_quaternion(sample.data, &quat);
            
            LOG_INF("Quat: W=%6.4f X=%6.4f Y=%6.4f Z=%6.4f",
                    (double)quat.w, (double)quat.x,
                    (double)quat.y, (double)quat.z);
        }
    }

    k_msleep(50);  /* Leer cada 50ms */
}
```

## Configuraciones Optimizadas

### Para bajo consumo
```c
/* Configuración de bajo consumo */
lsm6dsv16x_accel_config(&config, LSM6DSV16X_ACCEL_ODR_30_HZ,
                                  LSM6DSV16X_ACCEL_FS_2G);
lsm6dsv16x_gyro_config(&config, LSM6DSV16X_GYRO_ODR_30_HZ,
                                 LSM6DSV16X_GYRO_FS_500_DPS);
lsm6dsv16x_accel_lpf_config(&config, true, LSM6DSV16X_ACCEL_LPF_STRONG);
lsm6dsv16x_gyro_lpf_config(&config, true, LSM6DSV16X_GYRO_LPF_STRONG);
lsm6dsv16x_sflp_enable(&config, LSM6DSV16X_SFLP_ODR_30_HZ);
```

### Para alta velocidad
```c
/* Configuración de alta velocidad */
lsm6dsv16x_accel_config(&config, LSM6DSV16X_ACCEL_ODR_960_HZ,
                                  LSM6DSV16X_ACCEL_FS_8G);
lsm6dsv16x_gyro_config(&config, LSM6DSV16X_GYRO_ODR_960_HZ,
                                 LSM6DSV16X_GYRO_FS_2000_DPS);
lsm6dsv16x_accel_lpf_config(&config, true, LSM6DSV16X_ACCEL_LPF_LIGHT);
lsm6dsv16x_gyro_lpf_config(&config, true, LSM6DSV16X_GYRO_LPF_LIGHT);
lsm6dsv16x_sflp_enable(&config, LSM6DSV16X_SFLP_ODR_480_HZ);
```

## Recursos Adicionales

- **Documentación completa**: Ver `README.md`
- **Resumen técnico**: Ver `RESUMEN_TECNICO.md`
- **Ejemplo completo**: Ver `lsm6dsv16x_example.c`
- **Datasheet**: LSM6DSV16X datasheet (198 páginas)

## Soporte

Para problemas específicos:
1. Revisa el nivel de log (CONFIG_LSM6DSV16X_LOG_LEVEL=4)
2. Verifica el datasheet del LSM6DSV16X
3. Consulta los ejemplos de STM32duino
4. Revisa la documentación de Zephyr I2C

---

¡Listo! Con estos pasos deberías tener el sensor funcionando en tu proyecto Zephyr.
