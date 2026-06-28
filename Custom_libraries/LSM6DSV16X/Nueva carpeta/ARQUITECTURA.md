# Arquitectura del Sistema LSM6DSV16X + Zephyr

## Diagrama de Arquitectura Completa

```
┌─────────────────────────────────────────────────────────────────────┐
│                        APLICACIÓN ZEPHYR                            │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                      main.c (Usuario)                        │   │
│  │  • Configuración del sensor                                  │   │
│  │  • Lectura de datos                                          │   │
│  │  • Procesamiento de quaternions                              │   │
│  └────────────────────┬─────────────────────────────────────────┘   │
│                       │ API calls                                   │
│                       ▼                                             │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │              LSM6DSV16X Driver (lsm6dsv16x.c/h)              │   │
│  │                                                              │   │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────────────┐      │   │
│  │  │ Init/Reset │  │   Config   │  │  Data Reading      │      │   │
│  │  │            │  │            │  │                    │      │   │
│  │  │ • WHO_AM_I │  │ • Accel FS │  │ • Read registers   │      │   │
│  │  │ • BDU      │  │ • Gyro FS  │  │ • FIFO operations  │      │   │
│  │  │ • AutoInc  │  │ • ODR      │  │ • Sample parsing   │      │   │
│  │  └────────────┘  └────────────┘  └────────────────────┘      │   │
│  │                                                              │   │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────────────┐      │   │
│  │  │    SFLP    │  │  Filters   │  │    Conversion      │      │   │
│  │  │            │  │            │  │                    │      │   │
│  │  │ • Enable   │  │ • LPF1 Gyr │  │ • Raw → mg         │      │   │
│  │  │ • Disable  │  │ • LPF2 Acc │  │ • Raw → mdps       │      │   │
│  │  │ • Reset    │  │ • Bandwidth│  │ • Half → Float     │      │   │
│  │  │ • FIFO cfg │  │            │  │ • Quaternion calc  │      │   │
│  │  └────────────┘  └────────────┘  └────────────────────┘      │   │
│  │                                                              │   │
│  └────────────────────┬─────────────────────────────────────────┘   │
│                       │ I2C HAL                                     │
│                       ▼                                             │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                    Zephyr I2C Driver                         │   │
│  │  • i2c_write()                                               │   │
│  │  • i2c_write_read()                                          │   │
│  │  • Device management                                         │   │
│  └────────────────────┬─────────────────────────────────────────┘   │
└───────────────────────┼─────────────────────────────────────────────┘
                        │ I2C Bus (SDA/SCL)
                        ▼
         ┌─────────────────────────────────┐
         │      Hardware LSM6DSV16X        │
         │                                 │
         │  ┌────────────┐  ┌────────────┐ │
         │  │   ACCEL    │  │    GYRO    │ │
         │  │            │  │            │ │
         │  │  ±2g-±16g  │  │±125-±4000  │ │
         │  │            │  │    dps     │ │
         │  │ Up to      │  │ Up to      │ │
         │  │ 7680 Hz    │  │ 7680 Hz    │ │
         │  └──────┬─────┘  └─────┬──────┘ │
         │         │              │        │
         │         └──────┬───────┘        │
         │                ▼                │
         │    ┌───────────────────────┐    │
         │    │  SFLP (Sensor Fusion) │    │
         │    │                       │    │
         │    │  • Game Rotation      │    │
         │    │  • Quaternion output  │    │
         │    │  • Up to 480 Hz       │    │
         │    └───────────┬───────────┘    │
         │                ▼                │
         │    ┌───────────────────────┐    │
         │    │    4.5 KB FIFO        │    │
         │    │                       │    │
         │    │  • Stream mode        │    │
         │    │  • Tagged samples     │    │
         │    │  • ~658 samples max   │    │
         │    └───────────────────────┘    │
         └─────────────────────────────────┘
```

## Flujo de Datos - Lectura Directa

```
┌──────────┐   Read Request    ┌──────────────┐
│   App    │ ───────────────>  │ lsm6dsv16x_  │
│          │                   │ accel_read_  │
│          │                   │     raw()    │
└──────────┘                   └──────┬───────┘
                                      │
                               I2C Read 6 bytes
                               (OUTX_L_A → OUTZ_H_A)
                                      │
                                      ▼
                          ┌───────────────────────┐
                          │  LSM6DSV16X Hardware  │
                          │  Latest measurement   │
                          │  Updated at ODR freq  │
                          └───────────┬───────────┘
                                      │
                                 6 bytes raw
                                  int16_t[3]
                                      │
┌──────────┐                          ▼
│   App    │ <─────────────  ┌────────────────┐
│ Convert  │   accel_raw_t   │  Driver        │
│ to mg    │                 │  sys_get_le16()│
└──────────┘                 └────────────────┘
```

## Flujo de Datos - SFLP con FIFO

```
┌─────────────────────────────────────────────────┐
│          LSM6DSV16X Hardware                    │
│                                                 │
│  Accel @ 120Hz ─┐                               │
│                 ├─> SFLP @ 120Hz ──> FIFO       │
│  Gyro @ 120Hz ──┘        │                      │
│                          │                      │
│                    Quaternions                  │
│                   (qx, qy, qz)                  │
│                   Half-precision                │
│                                                 │
└────────────────┬────────────────────────────────┘
                 │
        FIFO Read (7 bytes)
        ┌──────────────────┐
        │ TAG: 0x13        │ 1 byte
        │ DATA: 6 bytes    │ qx, qy, qz (half-float)
        └──────────────────┘
                 │
                 ▼
        ┌────────────────────┐
        │  Driver Processing │
        │                    │
        │  1. Extract tag    │
        │  2. Parse 3×int16  │
        │  3. half_to_float()│
        │  4. Calculate qw   │
        │     qw = √(1-sum)  │
        └────────┬───────────┘
                 │
                 ▼
        ┌────────────────────┐
        │  quaternion_t      │
        │  w, x, y, z        │
        │  Normalized: |q|=1 │
        └────────────────────┘
```

## Estructura de Memoria FIFO

```
┌─────────────────────────────────────────────────┐
│            FIFO (4608 bytes = 4.5 KB)           │
├─────────────────────────────────────────────────┤
│                                                 │
│  Sample 1: [TAG][DATA DATA DATA DATA DATA DATA] │ 7 bytes
│  Sample 2: [TAG][DATA DATA DATA DATA DATA DATA] │ 7 bytes
│  Sample 3: [TAG][DATA DATA DATA DATA DATA DATA] │ 7 bytes
│  ...                                            │
│  Sample N: [TAG][DATA DATA DATA DATA DATA DATA] │ 7 bytes
│                                                 │
│  Maximum: 658 samples (4608 / 7 = 658.28)       │
│                                                 │
│  Read pointer: →                                │
│  Write pointer: →                               │
│                                                 │
│  Mode: STREAM (circular buffer)                 │
│  Overflow: Oldest data overwritten              │
└─────────────────────────────────────────────────┘
```

## Tags FIFO y Contenido

```
┌──────────────────────────────────────────────────────┐
│  TAG    │  Value  │  Content                         │
├──────────────────────────────────────────────────────┤
│  GYRO   │  0x01   │  Gx, Gy, Gz (3 × int16)          │
│  ACCEL  │  0x02   │  Ax, Ay, Az (3 × int16)          │
│  TIME   │  0x04   │  Timestamp (32-bit)              │
│  QUAT   │  0x13   │  qx, qy, qz (3 × half-float)     │
│  GRAV   │  0x14   │  gx, gy, gz (3 × int16)          │
│  GBIAS  │  0x15   │  bx, by, bz (3 × int16)          │
└──────────────────────────────────────────────────────┘

Cada muestra FIFO:
┌────┬────┬────┬────┬────┬────┬────┐
│TAG │D0  │D1  │D2  │D3  │D4  │D5  │
└────┴────┴────┴────┴────┴────┴────┘
  1B   1B   1B   1B   1B   1B   1B   = 7 bytes total
```

## Conversión Half-Float a Float

```
Half-Precision (16 bits):
┌─┬─────┬──────────┐
│S│EEEEE│FFFFFFFFFF│
└─┴─────┴──────────┘
 1   5      10

S = Sign bit
E = Exponent (bias 15)
F = Fraction

Single-Precision (32 bits):
┌─┬────────┬───────────────────────┐
│S│EEEEEEEE│FFFFFFFFFFFFFFFFFFFFFFF│
└─┴────────┴───────────────────────┘
 1    8              23

S = Sign bit
E = Exponent (bias 127)
F = Fraction

Conversion:
1. Extract S, E, F from half
2. Adjust exponent: E_new = E_old + (127-15)
3. Shift fraction: F_new = F_old << 13
4. Handle special cases (zero, inf, NaN)
```

## Cálculo de Quaternion W

```
SFLP Output: qx, qy, qz (3 half-floats)

Constraint: |q| = 1 (normalized quaternion)
            qw² + qx² + qy² + qz² = 1

Calculate:  sum_sq = qx² + qy² + qz²
            
            if (sum_sq < 1.0):
                qw = √(1 - sum_sq)
            else:
                qw = 0
                renormalize(qx, qy, qz)

Output: Complete quaternion (qw, qx, qy, qz)
```

## Configuración de Memoria Banks

```
┌────────────────────────────────────────────┐
│         Register Memory Map                │
├────────────────────────────────────────────┤
│                                            │
│  MAIN BANK (0x00)                          │
│  ┌─────────────────────────────────────┐   │
│  │  WHO_AM_I      (0x0F)               │   │
│  │  CTRL1-10      (0x10-0x19)          │   │
│  │  STATUS        (0x1E)               │   │
│  │  OUT_TEMP      (0x20-0x21)          │   │
│  │  OUTX_L_G      (0x22-0x27)          │   │
│  │  OUTX_L_A      (0x28-0x2D)          │   │
│  │  FIFO_CTRL     (0x07-0x0A)          │   │
│  │  FIFO_STATUS   (0x1B-0x1C)          │   │
│  │  FIFO_DATA_OUT (0x78-0x79)          │   │
│  └─────────────────────────────────────┘   │
│                                            │
│  EMBEDDED FUNCTIONS BANK (0x01)            │
│  ┌─────────────────────────────────────┐   │
│  │  EMB_FUNC_EN_A      (0x04)          │   │
│  │  EMB_FUNC_EN_B      (0x05)          │   │
│  │  EMB_FUNC_FIFO_EN_A (0x44)          │   │
│  │  SFLP_ODR           (0x5E)          │   │
│  │  EMB_FUNC_INIT_A    (0x66)          │   │
│  └─────────────────────────────────────┘   │
│                                            │
│  SENSOR HUB BANK (0x02)                    │
│  ┌─────────────────────────────────────┐   │
│  │  External sensor configuration      │   │
│  │  (Not used in basic operation)      │   │
│  └─────────────────────────────────────┘   │
│                                            │
│  Switch: FUNC_CFG_ACCESS register (0x01)   │
│    bit 7: Embedded functions               │
│    bit 6: Sensor hub                       │
└────────────────────────────────────────────┘
```

## Resumen de Tamaños

```
┌──────────────────────────────────────────┐
│         Memory & Data Sizes              │
├──────────────────────────────────────────┤
│  FIFO Total:           4608 bytes        │
│  FIFO Sample:          7 bytes           │
│  Max FIFO Samples:     658 samples       │
│                                          │
│  Accel Raw:            6 bytes (3×int16) │
│  Gyro Raw:             6 bytes (3×int16) │
│  SFLP Raw:             6 bytes (3×half)  │
│  SFLP Quaternion:      16 bytes (4×float)│
│                                          │
│  Source Code:          ~54 KB total      │
│    lsm6dsv16x.c:       18 KB             │
│    lsm6dsv16x.h:       12 KB             │
│    lsm6dsv16x_regs.h:  6 KB              │
│    Example:            7 KB              │
│    Documentation:      28 KB             │
└──────────────────────────────────────────┘
```
