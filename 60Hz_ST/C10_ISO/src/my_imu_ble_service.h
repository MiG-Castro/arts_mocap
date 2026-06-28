/**
 * @file my_imu_ble_service.h
 * @brief My IMU Service (IMUS) public API.
 *
 * This file contains the public API for the custom IMU BLE Service, which is
 * designed to handle sensor data streaming, command reception, and critical
 * event notifications between a Central and a Peripheral.
 */

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
#include <bluetooth/gatt_dm.h> // Necesario para struct bt_gatt_dm
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

/** @brief IMU Service UUID (random 128-bit UUID). */ 
#define BT_UUID_IMUS_VAL				BT_UUID_128_ENCODE(0x5cf33500, 0x538c, 0x4dc9, 0xb54b, 0x0b102623558d)

/** @brief SensorData Characteristic UUID. */
#define BT_UUID_IMUS_DATA_VAL			BT_UUID_128_ENCODE(0x5cf33501, 0x538c, 0x4dc9, 0xb54b, 0x0b102623558d)

/** @brief ExerciseDetection Characteristic UUID (for sending critical events to a Central).*/
#define BT_UUID_IMUS_EXDETECTION_VAL	BT_UUID_128_ENCODE(0x5cf33502, 0x538c, 0x4dc9, 0xb54b, 0x0b102623558d)

/** @brief Command Characteristic UUID (for receiving commands from a Central) */
#define BT_UUID_IMUS_COMMAND_VAL 		BT_UUID_128_ENCODE(0x5cf33503, 0x538c, 0x4dc9, 0xb54b, 0x0b102623558d)

#define BT_UUID_IMUS 				BT_UUID_DECLARE_128(BT_UUID_IMUS_VAL)
#define BT_UUID_IMUS_DATA 			BT_UUID_DECLARE_128(BT_UUID_IMUS_DATA_VAL)
#define BT_UUID_IMUS_EXDETECTION 	BT_UUID_DECLARE_128(BT_UUID_IMUS_EXDETECTION_VAL)
#define BT_UUID_IMUS_COMMAND 		BT_UUID_DECLARE_128(BT_UUID_IMUS_COMMAND_VAL)

// =============================================================================
// API for the PERIPHERAL ROLE
// =============================================================================
#ifdef CONFIG_BT_PERIPHERAL
/** @brief Callback function type for receiving data from the Command characteristic.
 *
 * @param[in] buf Pointer to the received data.
 * @param[in] len Length of the received data.
 */
typedef void (*command_write_cb_t)(const uint8_t *buf, uint16_t len);

/** @brief Structure for IMU Service application callbacks. */
struct my_imus_cb {
	/** @brief Callback function to be called when data is written to the Command characteristic by a client.*/
	command_write_cb_t command_write_cb;
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

/** @brief Send data to a CENTRAL. Requires ACK from client to ensuring delivery.
 *
 * @param[in] data Pointer of the byte array that contain the time of the detectet exercise.
 * @param[in] len The number of bytes to send.
 * 
 * @retval 0 If the operation was successful. Otherwise, a (negative) error code is returned.
 *
 */
int my_imus_send_exercisedetection(const uint8_t *data, uint16_t len);

/** @brief Get the indicate enable status.
 *
 * @retval 0 If the operation was successful. Otherwise, a (negative) error code is returned.
 */
bool is_exercisedetection_indicate_enabled(void);

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
int my_imus_send_sensordata(const uint8_t *sensor_data, uint16_t send_bytes);

/** @brief Get the notification enable status.
 *
 * @retval 0 If the operation was successful. Otherwise, a (negative) error code is returned.
 */
bool is_sensordata_notify_enabled(void);

#endif /* CONFIG_BT_PERIPHERAL */

// =============================================================================
// API for the CENTRAL ROLE
// =============================================================================
#ifdef CONFIG_BT_CENTRAL

/** @brief Structure to hold the handles discovered for the IMU Service. */
struct bt_imus_client {
	/** Handle of the SensorData characteristic value. */
	uint16_t sensordata_handle;
	/** Handle of the CCCD for the SensorData characteristic. */
	uint16_t sensordata_ccc_handle;
	/** GATT subscribe parameters for SensorData notifications. */
	struct bt_gatt_subscribe_params sub_params_sensordata;

	/** Handle of the ExerciseDetection characteristic value. */
	uint16_t exercisedetection_handle;
	/** Handle of the CCCD for the ExerciseDetection characteristic. */
	uint16_t exercisedetection_ccc_handle;
	/** GATT subscribe parameters for ExerciseDetection indications. */
	struct bt_gatt_subscribe_params sub_params_exercisedetection;
    
	/** Handle of the Command characteristic value. */
	uint16_t command_handle;
	/** GATT write parameters for the Command characteristic. */
	struct bt_gatt_write_params write_params;
};

/**
 * @brief Assign handles discovered by the GATT Discovery Manager.
 *
 * @param[in] dm Pointer to the GATT Discovery Manager instance.
 * @param[out] client Pointer to the IMU Service client instance to populate.
 *
 * @retval 0 If successful. Otherwise, a (negative) error code is returned.
 */
int my_imus_handles_assign(struct bt_gatt_dm *dm, struct bt_imus_client *client);

/**
 * @brief Subscribe to SensorData notifications.
 *
 * @param[in] conn Pointer to the active connection.
 * @param[in] client Pointer to the initialized IMU Service client instance.
 * @param[in] func The callback function to handle incoming notifications.
 *
 * @retval 0 If successful. Otherwise, a (negative) error code is returned.
 */
int my_imus_subscribe_sensordata(struct bt_conn *conn, struct bt_imus_client *client, bt_gatt_notify_func_t func);

/**
 * @brief Subscribe to ExerciseDetection indications.
 *
 * @param[in] conn Pointer to the active connection.
 * @param[in] client Pointer to the initialized IMU Service client instance.
 * @param[in] func The callback function to handle incoming indications.
 *
 * @retval 0 If successful. Otherwise, a (negative) error code is returned.
 */
int my_imus_subscribe_exercisedetection(struct bt_conn *conn, struct bt_imus_client *client, bt_gatt_notify_func_t func);

/**
 * @brief Write a command to the Command characteristic.
 *
 * @param[in] conn Pointer to the active connection.
 * @param[in] client Pointer to the initialized IMU Service client instance.
 * @param[in] cmd Pointer to the command data to be sent.
 * @param[in] len Length of the command data.
 * @param[in] cb  Callback function for write completion. Can be NULL.
 *
 * @retval 0 If successful. Otherwise, a (negative) error code is returned.
 */
int my_imus_write_command(struct bt_conn *conn, struct bt_imus_client *client, const uint8_t *cmd, uint16_t len, bt_gatt_write_func_t cb);

#endif // CONFIG_BT_CENTRAL

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* MY_IMU_BLE_SERVICE_H_ */
