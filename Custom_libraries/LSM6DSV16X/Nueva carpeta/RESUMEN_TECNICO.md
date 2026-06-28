# Resumen Técnico - Librería LSM6DSV16X para Zephyr RTOS

## Respuesta a tu pregunta: Tamaño de muestra SFLP

**Una muestra completa de SFLP pesa exactamente 7 BYTES:**
- 1 byte: TAG (0x13 para quaternions)
- 6 bytes: 3 valores half-precision float (16-bit cada uno)

Los 6 bytes representan: qx, qy, qz (el componente qw se calcula).

---

## Estructura de la Librería

### Archivos Generados

1. **lsm6dsv16x_regs.h** (6.1 KB)
   - Definiciones de todos los registros
   - Máscaras de bits
   - Constantes del hardware
   - Direcciones I2C y tags FIFO

2. **lsm6dsv16x.h** (12 KB)
   - API pública completa
   - Enumeraciones de configuración
   - Estructuras de datos
   - Documentación de funciones

3. **lsm6dsv16x.c** (18 KB)
   - Implementación completa del driver
   - Gestión de I2C
   - Conversiones de datos
   - Lógica de SFLP y FIFO

4. **Kconfig** (2.4 KB)
   - Configuración de Zephyr
   - Opciones configurables
   - Valores por defecto

5. **lsm6dsv16x_example.c** (6.7 KB)
   - Ejemplo completo de uso
   - Lectura directa de registros
   - Configuración de SFLP
   - Procesamiento de FIFO

6. **README.md** (9.0 KB)
   - Documentación completa
   - Guías de integración
   - Ejemplos de código
   - Troubleshooting

**Total: ~54 KB de código fuente**

---

## Funcionalidades Implementadas

### ✅ 1. Inicializar el sensor
```c
int lsm6dsv16x_init(const lsm6dsv16x_config_t *config);
```
- Verifica WHO_AM_I (0x70)
- Habilita auto-increment
- Habilita Block Data Update (BDU)
- Configura estado inicial

### ✅ 2. Habilitar el SFLP
```c
int lsm6dsv16x_sflp_enable(const lsm6dsv16x_config_t *config,
                            lsm6dsv16x_sflp_odr_t odr);
int lsm6dsv16x_sflp_disable(const lsm6dsv16x_config_t *config);
int lsm6dsv16x_sflp_reset(const lsm6dsv16x_config_t *config);
```
- Configura ODR del SFLP (15-480 Hz)
- Habilita game rotation vector
- Configura FIFO en modo STREAM
- Habilita batching en FIFO

### ✅ 3. Configurar ODR y rango del acc/gyr
```c
int lsm6dsv16x_accel_config(const lsm6dsv16x_config_t *config,
                             lsm6dsv16x_accel_odr_t odr,
                             lsm6dsv16x_accel_fs_t fs);

int lsm6dsv16x_gyro_config(const lsm6dsv16x_config_t *config,
                            lsm6dsv16x_gyro_odr_t odr,
                            lsm6dsv16x_gyro_fs_t fs);
```
**Acelerómetro:**
- ODR: 1.875 Hz a 7680 Hz
- FS: ±2g, ±4g, ±8g, ±16g

**Giroscopio:**
- ODR: 7.5 Hz a 7680 Hz
- FS: ±125, ±250, ±500, ±1000, ±2000, ±4000 dps

### ✅ 4. Leer FIFO y registros directos

**Registros directos:**
```c
int lsm6dsv16x_accel_read_raw(const lsm6dsv16x_config_t *config,
                               lsm6dsv16x_accel_raw_t *data);
int lsm6dsv16x_gyro_read_raw(const lsm6dsv16x_config_t *config,
                              lsm6dsv16x_gyro_raw_t *data);
```

**FIFO:**
```c
int lsm6dsv16x_fifo_get_count(const lsm6dsv16x_config_t *config,
                               uint16_t *count);
int lsm6dsv16x_fifo_read_sample(const lsm6dsv16x_config_t *config,
                                 lsm6dsv16x_fifo_sample_t *sample);
```

### ✅ 5. Convertir datos crudos a unidades físicas
```c
void lsm6dsv16x_accel_raw_to_mg(const lsm6dsv16x_accel_raw_t *raw,
                                 lsm6dsv16x_accel_fs_t fs,
                                 lsm6dsv16x_accel_mg_t *mg);

void lsm6dsv16x_gyro_raw_to_mdps(const lsm6dsv16x_gyro_raw_t *raw,
                                  lsm6dsv16x_gyro_fs_t fs,
                                  lsm6dsv16x_gyro_mdps_t *mdps);

void lsm6dsv16x_sflp_to_quaternion(const uint8_t fifo_data[6],
                                    lsm6dsv16x_quaternion_t *quat);
```

**Conversiones implementadas:**
- Accel raw → mg (miligravedades)
- Gyro raw → mdps (mili-grados por segundo)
- SFLP half-float → quaternion normalizado

### ✅ 6. Configurar filtros LPF1/LPF2
```c
int lsm6dsv16x_accel_lpf_config(const lsm6dsv16x_config_t *config,
                                 bool enable,
                                 lsm6dsv16x_accel_lpf_bw_t bandwidth);

int lsm6dsv16x_gyro_lpf_config(const lsm6dsv16x_config_t *config,
                                bool enable,
                                lsm6dsv16x_gyro_lpf_bw_t bandwidth);
```

**Opciones de bandwidth:**
- ULTRA_LIGHT, VERY_LIGHT, LIGHT, MEDIUM
- STRONG, VERY_STRONG, AGGRESSIVE, XTREME

### ✅ 7. Resetear el sensor
```c
int lsm6dsv16x_reset(const lsm6dsv16x_config_t *config);
```
- Software reset completo
- Espera confirmación de reset
- Timeout de 100ms

---

## Detalles Técnicos Importantes

### Conversión Half-Float a Float

El SFLP usa formato half-precision (16-bit):
```
SEEEEEFFFFFFFFFF
│└──┘└────────┘
│ │      └─ 10 bits fracción
│ └─ 5 bits exponente
└─ 1 bit signo
```

La función `half_to_float()` implementa la conversión completa incluyendo:
- Manejo de ceros
- Manejo de subnormales
- Manejo de infinitos y NaN
- Conversión de exponente (bias 15 → bias 127)

### Cálculo del Cuaternión W

```c
float sum_sq = qx * qx + qy * qy + qz * qz;
float qw = (sum_sq < 1.0f) ? sqrtf(1.0f - sum_sq) : 0.0f;
```

Con renormalización en caso de error numérico.

### Gestión de Memory Banks

El sensor tiene 3 bancos de memoria:
1. **MAIN** (0x00): Registros principales
2. **EMBED** (0x01): Funciones embebidas (SFLP)
3. **SENSOR_HUB** (0x02): Hub de sensores externos

La función `mem_bank_set()` gestiona el cambio automático.

### Tags FIFO

| Tag  | Valor | Tamaño Total | Contenido              |
|------|-------|--------------|------------------------|
| 0x01 | Gyro  | 7 bytes      | 1 tag + 3×int16 (6 B)  |
| 0x02 | Accel | 7 bytes      | 1 tag + 3×int16 (6 B)  |
| 0x13 | Quat  | 7 bytes      | 1 tag + 3×half16 (6 B) |

---

## Configuraciones Recomendadas

### Para aplicaciones generales
```c
Accel:  120 Hz, ±4g,      LPF2 MEDIUM
Gyro:   120 Hz, ±2000dps, LPF1 MEDIUM  
SFLP:   120 Hz
```

### Para captura de alta velocidad
```c
Accel:  960 Hz, ±8g,      LPF2 LIGHT
Gyro:   960 Hz, ±2000dps, LPF1 LIGHT
SFLP:   480 Hz (máximo)
```

### Para bajo consumo
```c
Accel:  30 Hz, ±2g,      LPF2 STRONG
Gyro:   30 Hz, ±500dps,  LPF1 STRONG
SFLP:   30 Hz
```

---

## Puntos Clave del Diseño

### Compatibilidad con Zephyr
- Usa `struct device` de Zephyr
- Compatible con Device Tree
- Integración con `LOG_MODULE`
- APIs I2C nativas de Zephyr
- `sys_get_le16()` para endianness

### Manejo de Errores
- Códigos de error estándar de Zephyr (-EINVAL, -ENODEV, etc.)
- Logging en múltiples niveles
- Validación de parámetros
- Timeouts configurables

### Optimizaciones
- Auto-increment habilitado por defecto
- Lecturas multi-byte para eficiencia
- BDU evita datos inconsistentes
- Conversiones con tablas de lookup

---

## Uso Básico

```c
#include "lsm6dsv16x.h"

/* 1. Configurar */
lsm6dsv16x_config_t config = {
    .i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1)),
    .i2c_addr = LSM6DSV16X_I2C_ADDR_HIGH,
    .accel_fs = LSM6DSV16X_ACCEL_FS_4G,
    .gyro_fs = LSM6DSV16X_GYRO_FS_2000_DPS,
};

/* 2. Inicializar */
lsm6dsv16x_init(&config);

/* 3. Configurar sensores */
lsm6dsv16x_accel_config(&config, LSM6DSV16X_ACCEL_ODR_120_HZ,
                                  LSM6DSV16X_ACCEL_FS_4G);
lsm6dsv16x_gyro_config(&config, LSM6DSV16X_GYRO_ODR_120_HZ,
                                 LSM6DSV16X_GYRO_FS_2000_DPS);

/* 4. Habilitar SFLP */
lsm6dsv16x_sflp_enable(&config, LSM6DSV16X_SFLP_ODR_120_HZ);

/* 5. Leer datos del FIFO */
uint16_t count;
lsm6dsv16x_fifo_sample_t sample;
lsm6dsv16x_quaternion_t quat;

lsm6dsv16x_fifo_get_count(&config, &count);

for (int i = 0; i < count; i++) {
    lsm6dsv16x_fifo_read_sample(&config, &sample);
    
    if (sample.tag == LSM6DSV16X_FIFO_TAG_SFLP_QUAT) {
        lsm6dsv16x_sflp_to_quaternion(sample.data, &quat);
        printk("Q: w=%.4f x=%.4f y=%.4f z=%.4f\n",
               quat.w, quat.x, quat.y, quat.z);
    }
}
```

---

## Referencias del Código

Basado en:
1. **Datasheet LSM6DSV16X** (lsm6dsv16x.pdf en proyecto)
2. **STM32duino/LSM6DSV16X** (archivos src/ en proyecto)
3. **Application Note AN5763**

Todo el código ha sido adaptado y optimizado específicamente para Zephyr RTOS.
