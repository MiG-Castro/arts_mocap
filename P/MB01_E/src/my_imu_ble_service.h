#ifndef MY_IMU_BLE_SERVICE_H_
#define MY_IMU_BLE_SERVICE_H_

/**@file
 * @defgroup Bluetooth IMU Service API
 * @{
 * @brief API for the IMU Service (IMUS).
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

/** @brief IMU Service UUID (random 128-bit UUID). */ 
#define BT_UUID_IMUS_VAL BT_UUID_128_ENCODE(0x5cf33500, 0x538c, 0x4dc9, 0xb54b, 0x0b102623558d)

/** @brief SensorData Characteristic UUID. */
#define BT_UUID_IMUS_DATA_VAL BT_UUID_128_ENCODE(0x5cf33501, 0x538c, 0x4dc9, 0xb54b, 0x0b102623558d)

/** @brief ExerciseDetection Characteristic UUID. */
#define BT_UUID_IMUS_EXDETECTION_VAL BT_UUID_128_ENCODE(0x5cf33502, 0x538c, 0x4dc9, 0xb54b, 0x0b102623558d)

#define BT_UUID_IMUS BT_UUID_DECLARE_128(BT_UUID_IMUS_VAL)
#define BT_UUID_IMUS_DATA BT_UUID_DECLARE_128(BT_UUID_IMUS_DATA_VAL)
#define BT_UUID_IMUS_EXDETECTION BT_UUID_DECLARE_128(BT_UUID_IMUS_EXDETECTION_VAL)

/** @brief Callback type for when an time elapsed of an Exercise is received. */
typedef void (*exdetection_write_cb_t)(const uint8_t *buf, uint16_t send_bytes);

/** @brief Callback struct used by the IMU Service. */
struct my_imus_cb {
	exdetection_write_cb_t exdetection_write_cb; 	// Exercise detection callback
};

/** @brief Initialize IMU Service.
 *
 * This function registers application callback functions with the My IMU
 * Service
 *
 * @param[in] callbacks Struct containing pointers to callback functions
 *			used by the service. This pointer can be NULL
 *			if no callback functions are defined.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int my_imus_init(struct my_imus_cb *callbacks);

/** @brief Send the time elapsed of the detected exercise.
 *
 * @param[in] data Pointer of the byte array that contain the time of the detectet exercise.
 * @param[in] len The number of bytes to send.
 * 
 * @retval 0 If the operation was successful. Otherwise, a (negative) error code is returned.
 *
 */
int my_imus_exercisedetection_indicate(const uint8_t *data, uint16_t len);

/** @brief Get the indicate enable status.
 *
 * @retval 0 If the operation was successful. Otherwise, a (negative) error code is returned.
 */
bool get_imus_exercisedetection_indicate_enabled(void);

/** @brief Send the sensor value as notification.
 *
 * This function sends an uint8_t array.
 *
 * @param[in] sensor_data Byte array that contain the sensor data.
 * @param[in] send_bytes The number of bytes to send.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int my_imus_sensordata_notify(const uint8_t *sensor_data, uint16_t send_bytes);

/** @brief Get the notification enable status.
 *
 * @retval 0 If the operation was successful. Otherwise, a (negative) error code is returned.
 */
bool get_imus_sensordata_notify_enabled(void);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* MY_IMU_BLE_SERVICE_H_ */
