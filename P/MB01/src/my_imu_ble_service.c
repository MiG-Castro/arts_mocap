/** @file
 *  @brief IMU Service (IMUS)
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include "my_imu_ble_service.h"

LOG_MODULE_DECLARE(IMU, LOG_LEVEL_INF);

// Static variables of the service
static bool notify_enabled = false;	// Flag notification of MySensor characteristic is enabled by a client
static bool indicate_enabled = false;

static struct my_imus_cb imus_cb;
static struct bt_gatt_indicate_params ind_params;

// Callback function of configuration change of CCCD (Client Characteristic Configuration Descriptor)
// Notify if the client is subscribed to the SensorData NOTIFY-characteristic
static void my_imus_ccc_sensordata_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{notify_enabled = (value == BT_GATT_CCC_NOTIFY);}

// Callback function of configuration change of CCCD (Client Characteristic Configuration Descriptor)
// Notify if the client is subscribed to the ExDetection INDICATE-characteristic
static void my_imus_ccc_excdetection_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{indicate_enabled = (value == BT_GATT_CCC_INDICATE);}

// Callbak when an AKC is received
static void indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err)
{
	// LOG_INF("Indication %s", err != 0U ? "fail" : "success");
}

// Callback function for writing to the ExDetection characteristic
static ssize_t write_exdetection(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, 
								uint16_t len, uint16_t offset, uint8_t flags)
{
    if (imus_cb.exdetection_write_cb) {			// Is a callback function defined?
        imus_cb.exdetection_write_cb(buf, len);	// Call the callback function with the received data
        return len;								// Return the number of bytes written
    }
    return BT_GATT_ERR(BT_ATT_ERR_WRITE_NOT_PERMITTED);	// If no callback is defined, return NOT WRITE PERMITTED
}

/* IMU Service Declaration */
BT_GATT_SERVICE_DEFINE(
	// Atributes of the IMU Service
	// attrs[0]: IMU Service declaration.
	// attrs[1]: SensorData Characteristic declaration.
	// attrs[2]: CCCD of SensorData.
	// attrs[3]: Exercise Detection declaration.
	// attrs[4]: Value of Exercise Detection (Write and Indicate operations).
	// attrs[5]: CCCD of Exercise Detection.

	my_imus_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_IMUS),
	/* 1. Send Data characteristic and its CCCD  */
	BT_GATT_CHARACTERISTIC(
		BT_UUID_IMUS_DATA,			// UUID of the characteristic
		BT_GATT_CHRC_NOTIFY,		// Properties of the characteristic
		BT_GATT_PERM_NONE,			// Permissions of the characteristic
		NULL,						// Read callback function (not used)
		NULL,						// Write callback function (not used)
		NULL						// User data (not used)
	),
	BT_GATT_CCC(my_imus_ccc_sensordata_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* 2. Exercise Detection characteristic and its CCCD  */
	BT_GATT_CHARACTERISTIC(
		BT_UUID_IMUS_EXDETECTION,
		BT_GATT_CHRC_INDICATE | BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE,
		NULL,
		write_exdetection,
		NULL
	),
	BT_GATT_CCC(my_imus_ccc_excdetection_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// PUBLIC API FUNCTIONS *******************************************************

// Initialize the IMU Service with the provided callbacks
int my_imus_init(struct my_imus_cb *callbacks)
{
    if (callbacks) {
        imus_cb.exdetection_write_cb = callbacks->exdetection_write_cb;
	}
    return 0;
}

// Function to send indications for the Excercise Detection characteristic
int my_imus_exercisedetection_indicate(const uint8_t *data, uint16_t len)
{
    if (!indicate_enabled) {
        return -EACCES;
    }

    // Config parameters for the indication
    ind_params.attr = &my_imus_svc.attrs[4];
    ind_params.func = indicate_cb;
    ind_params.destroy = NULL;
    ind_params.data = data;
    ind_params.len = len;

    return bt_gatt_indicate(NULL, &ind_params);
}

// Function to get the indicate enable status
bool get_imus_exercisedetection_indicate_enabled(void) {return indicate_enabled;}

// Function to send notifications for the Data characteristic
int my_imus_sensordata_notify(const uint8_t *sensor_data, uint16_t send_bytes)
{
	if (!notify_enabled) {return -EACCES;}
	return bt_gatt_notify(NULL, &my_imus_svc.attrs[1], sensor_data, send_bytes);
}

// Function to get the notification enable status
bool get_imus_sensordata_notify_enabled(void) {return notify_enabled;}
