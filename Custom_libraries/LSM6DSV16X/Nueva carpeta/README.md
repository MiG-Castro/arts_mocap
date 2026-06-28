# LSM6DSV16X IMU Driver for Zephyr RTOS

Complete driver for the LSM6DSV16X 6-axis IMU with embedded sensor fusion (SFLP) for Zephyr RTOS.

## Features

- ✅ **6-axis IMU**: 3-axis accelerometer + 3-axis gyroscope
- ✅ **Sensor Fusion Low Power (SFLP)**: Hardware quaternion calculation
- ✅ **Configurable ranges**: 
  - Accelerometer: ±2g, ±4g, ±8g, ±16g
  - Gyroscope: ±125, ±250, ±500, ±1000, ±2000, ±4000 dps
- ✅ **High-speed sampling**: Up to 7680 Hz for accel/gyro
- ✅ **SFLP output**: Up to 480 Hz for quaternions
- ✅ **4.5 KB FIFO**: Batch processing with tags
- ✅ **Configurable filters**: LPF1 (gyro), LPF2 (accel)
- ✅ **Software reset**: Complete sensor reinitialization
- ✅ **I2C interface**: Compatible with Zephyr I2C API

## Hardware Requirements

- **LSM6DSV16X sensor** connected via I2C
- **I2C address**: 0x6A (SA0=GND) or 0x6B (SA0=VDD)
- **Power**: 1.71V - 3.6V

## File Structure

```
lsm6dsv16x/
├── lsm6dsv16x.h           # Public API
├── lsm6dsv16x.c           # Implementation
├── lsm6dsv16x_regs.h      # Register definitions
├── Kconfig                # Configuration options
└── lsm6dsv16x_example.c   # Complete usage example
```

## Integration with Zephyr

### 1. Device Tree Configuration

Add to your `.overlay` file:

```dts
&i2c1 {
    status = "okay";
    clock-frequency = <I2C_BITRATE_FAST>; /* 400 kHz */
    
    lsm6dsv16x@6b {
        compatible = "st,lsm6dsv16x";
        reg = <0x6b>;  /* 0x6a if SA0=GND */
        label = "LSM6DSV16X";
    };
};
```

### 2. Kconfig Configuration

Add to your `prj.conf`:

```
# I2C driver
CONFIG_I2C=y

# LSM6DSV16X driver
CONFIG_LSM6DSV16X=y
CONFIG_LSM6DSV16X_LOG_LEVEL=3

# Optional: Enable SFLP by default
CONFIG_LSM6DSV16X_SFLP_ENABLE_DEFAULT=y
CONFIG_LSM6DSV16X_SFLP_ODR_DEFAULT=120
```

## API Overview

### Initialization

```c
lsm6dsv16x_config_t config = {
    .i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1)),
    .i2c_addr = LSM6DSV16X_I2C_ADDR_HIGH,  // 0x6B
    .accel_fs = LSM6DSV16X_ACCEL_FS_4G,
    .gyro_fs = LSM6DSV16X_GYRO_FS_2000_DPS,
    .sflp_enabled = false
};

/* Initialize sensor */
int ret = lsm6dsv16x_init(&config);
```

### Sensor Configuration

```c
/* Configure accelerometer: 120 Hz, ±4g */
lsm6dsv16x_accel_config(&config, 
                        LSM6DSV16X_ACCEL_ODR_120_HZ,
                        LSM6DSV16X_ACCEL_FS_4G);

/* Configure gyroscope: 120 Hz, ±2000 dps */
lsm6dsv16x_gyro_config(&config,
                       LSM6DSV16X_GYRO_ODR_120_HZ,
                       LSM6DSV16X_GYRO_FS_2000_DPS);

/* Configure accelerometer filter */
lsm6dsv16x_accel_lpf_config(&config, true, 
                            LSM6DSV16X_ACCEL_LPF_MEDIUM);

/* Configure gyroscope filter */
lsm6dsv16x_gyro_lpf_config(&config, true,
                           LSM6DSV16X_GYRO_LPF_MEDIUM);
```

### Reading Direct Register Data

```c
lsm6dsv16x_accel_raw_t accel_raw;
lsm6dsv16x_gyro_raw_t gyro_raw;

/* Read raw data */
lsm6dsv16x_accel_read_raw(&config, &accel_raw);
lsm6dsv16x_gyro_read_raw(&config, &gyro_raw);

/* Convert to physical units */
lsm6dsv16x_accel_mg_t accel_mg;
lsm6dsv16x_gyro_mdps_t gyro_mdps;

lsm6dsv16x_accel_raw_to_mg(&accel_raw, config.accel_fs, &accel_mg);
lsm6dsv16x_gyro_raw_to_mdps(&gyro_raw, config.gyro_fs, &gyro_mdps);

printk("Accel: X=%.2f Y=%.2f Z=%.2f mg\n", 
       accel_mg.x, accel_mg.y, accel_mg.z);
printk("Gyro: X=%.1f Y=%.1f Z=%.1f mdps\n",
       gyro_mdps.x, gyro_mdps.y, gyro_mdps.z);
```

### SFLP (Sensor Fusion)

```c
/* Enable SFLP with 120 Hz output */
lsm6dsv16x_sflp_enable(&config, LSM6DSV16X_SFLP_ODR_120_HZ);
config.sflp_enabled = true;

/* SFLP data is available in FIFO */
```

### FIFO Reading

```c
uint16_t fifo_count;
lsm6dsv16x_fifo_sample_t sample;
lsm6dsv16x_quaternion_t quat;

/* Get number of samples in FIFO */
lsm6dsv16x_fifo_get_count(&config, &fifo_count);

/* Read all samples */
for (int i = 0; i < fifo_count; i++) {
    lsm6dsv16x_fifo_read_sample(&config, &sample);
    
    switch (sample.tag) {
    case LSM6DSV16X_FIFO_TAG_GYRO:
        /* Gyroscope data in sample.data */
        break;
        
    case LSM6DSV16X_FIFO_TAG_ACCEL:
        /* Accelerometer data in sample.data */
        break;
        
    case LSM6DSV16X_FIFO_TAG_SFLP_QUAT:
        /* Convert to quaternion */
        lsm6dsv16x_sflp_to_quaternion(sample.data, &quat);
        printk("Quat: W=%.4f X=%.4f Y=%.4f Z=%.4f\n",
               quat.w, quat.x, quat.y, quat.z);
        break;
    }
}
```

### Software Reset

```c
/* Reset sensor to default state */
lsm6dsv16x_reset(&config);

/* Reinitialize after reset */
lsm6dsv16x_init(&config);
```

## FIFO Details

### FIFO Sample Structure

Each FIFO sample consists of **7 bytes**:
- **1 byte**: TAG (identifies data type)
- **6 bytes**: Data payload

### FIFO Tags

| Tag   | Value | Description                    | Data Format      |
|-------|-------|--------------------------------|------------------|
| GYRO  | 0x01  | Gyroscope NC                   | 3x int16_t       |
| ACCEL | 0x02  | Accelerometer NC               | 3x int16_t       |
| QUAT  | 0x13  | SFLP Game Rotation (Quaternion)| 3x half-float    |
| GRAV  | 0x14  | SFLP Gravity Vector            | 3x int16_t       |
| GBIAS | 0x15  | SFLP Gyroscope Bias            | 3x int16_t       |

### SFLP Quaternion Format

The SFLP outputs quaternions in a compressed format:
- **6 bytes** in FIFO represent 3 half-precision floats (qx, qy, qz)
- The 4th component (qw) is calculated from normalization: qw = √(1 - qx² - qy² - qz²)
- Half-precision format: 1 sign bit + 5 exponent bits + 10 fraction bits
- Output is always normalized: |q| = 1

**Memory size per quaternion sample: 7 bytes (1 tag + 6 data)**

## Performance Characteristics

### Maximum Data Rates

| Sensor/Feature  | Maximum ODR | Notes                           |
|-----------------|-------------|---------------------------------|
| Accelerometer   | 7680 Hz     | Full range of motion            |
| Gyroscope       | 7680 Hz     | Full range of motion            |
| SFLP Quaternion | 480 Hz      | Hardware fusion limitation      |

### Recommended Configurations

#### General Purpose / Wearables
```c
Accelerometer: 120 Hz, ±4g, LPF2 MEDIUM
Gyroscope:     120 Hz, ±2000 dps, LPF1 MEDIUM
SFLP:          120 Hz
```

#### High-Speed Motion Capture
```c
Accelerometer: 960 Hz, ±8g, LPF2 LIGHT
Gyroscope:     960 Hz, ±2000 dps, LPF1 LIGHT
SFLP:          480 Hz (maximum)
```

#### Low-Power Mode
```c
Accelerometer: 30 Hz, ±2g, LPF2 STRONG
Gyroscope:     30 Hz, ±500 dps, LPF1 STRONG
SFLP:          30 Hz
```

## Filter Configuration

### Accelerometer Filter Chain

```
ADC → LPF1 (always on) → [LPF2 or HP] → Output
```

**LPF2 Configuration:**
- Bandwidth options: ULTRA_LIGHT, VERY_LIGHT, LIGHT, MEDIUM, STRONG, VERY_STRONG, AGGRESSIVE, XTREME
- Recommended for SFLP: LIGHT or MEDIUM

### Gyroscope Filter Chain

```
ADC → LPF1 (configurable) → Output
```

**LPF1 Configuration:**
- Bandwidth options: ULTRA_LIGHT, VERY_LIGHT, LIGHT, MEDIUM, STRONG, VERY_STRONG, AGGRESSIVE, XTREME
- Recommended for SFLP: LIGHT or MEDIUM

## Important Notes

### SFLP Requirements

1. **Both sensors must be enabled**: Accelerometer and gyroscope must be running
2. **FIFO mode required**: SFLP only outputs to FIFO in STREAM mode
3. **ODR matching recommended**: Set accel and gyro to same ODR as SFLP (or higher)
4. **Maximum SFLP ODR**: 480 Hz (sensors can run faster independently)

### SFLP Limitations

- **6-axis only**: No magnetometer (yaw will drift over time)
- **Quaternion format**: Only qx, qy, qz transmitted; qw calculated
- **No direct register output**: SFLP data only available via FIFO

### Memory Considerations

- **FIFO size**: 4608 bytes (4.5 KB)
- **Sample size**: 7 bytes per sample
- **Max samples**: ~658 samples in FIFO
- **At 120 Hz SFLP**: FIFO fills in ~5.5 seconds if not read

## Troubleshooting

### Sensor not detected
- Check I2C address (0x6A vs 0x6B)
- Verify I2C bus is working
- Check power supply (1.71V - 3.6V)

### No FIFO data
- Ensure FIFO mode is STREAM (not BYPASS)
- Verify sensors are enabled
- Check SFLP batching is enabled

### Incorrect quaternion values
- Verify SFLP ODR is set
- Ensure both accel and gyro are running
- Check filter configuration

### FIFO overflow
- Increase read frequency
- Reduce sensor ODR
- Process data faster

## Example Application

See `lsm6dsv16x_example.c` for a complete working example that demonstrates:
1. Sensor initialization
2. Configuration of accel/gyro/filters
3. Direct register reading
4. SFLP enable and FIFO reading
5. Data conversion and display

## References

- **Datasheet**: LSM6DSV16X datasheet (198 pages)
- **Application Note**: AN5763 - LSM6DSV16X sensor fusion
- **STM32duino Library**: https://github.com/stm32duino/LSM6DSV16X

## License

Based on STM32duino LSM6DSV16X library
Adapted for Zephyr RTOS - 2025

## Author

Based on ST Microelectronics reference implementation
Ported to Zephyr RTOS

---

**Note**: This driver is based on the official STM32duino LSM6DSV16X library and the sensor datasheet. All functionality has been tested and validated against the reference implementation.
