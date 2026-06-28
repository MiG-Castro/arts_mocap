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
#include <bluetooth/gatt_dm.h> 
#include "my_imu_ble_service.h"

LOG_MODULE_REGISTER(my_imus, LOG_LEVEL_INF);

// =============================================================================
// PERIPHERAL ROLE IMPLEMENTATION
// =============================================================================
#ifdef CONFIG_BT_PERIPHERAL

// --- Internal state variables ---
static bool sensordata_notify_enabled = false;
static bool exercisedetection_indicate_enabled = false;
static struct my_imus_cb imus_cb;

// --- Internal GATT callbacks ---

static void ccc_sensordata_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	sensordata_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Suscripción a SensorData %s", sensordata_notify_enabled ? "habilitada" : "deshabilitada");
}

static void ccc_exercisedetection_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	exercisedetection_indicate_enabled = (value == BT_GATT_CCC_INDICATE);
	LOG_INF("Suscripción a ExerciseDetection %s", exercisedetection_indicate_enabled ? "habilitada" : "deshabilitada");
}

static ssize_t write_command_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
								const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (imus_cb.command_write_cb) {
		imus_cb.command_write_cb(buf, len);
		return len;
	}
	return BT_GATT_ERR(BT_ATT_ERR_WRITE_NOT_PERMITTED);
}

static void indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err)
{if (err != 0) LOG_WRN("Fallo en la indicación de ExerciseDetection (err %u)", err);}

/*
 * IMU Service Declaration (Order: Data, ExerciseDetection, Command)
 */
BT_GATT_SERVICE_DEFINE(
	// Atributes of the IMU Service
	// attrs[0]: IMU Service declaration.
	// attrs[1]: SensorData Characteristic declaration.
	// attrs[2]: CCCD of SensorData.
	// attrs[3]: Exercise Detection declaration.
	// attrs[4]: Value of Exercise Detection (Write and Indicate operations).
	// attrs[5]: CCCD of Exercise Detection.
	// attrs[6]: Command

	my_imus_svc,BT_GATT_PRIMARY_SERVICE(BT_UUID_IMUS),
	
	/* Characteristic 1: SensorData (Streaming) */
	BT_GATT_CHARACTERISTIC(BT_UUID_IMUS_DATA, BT_GATT_CHRC_NOTIFY,
						BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_sensordata_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* Characteristic 2: ExerciseDetection (Events) */
	BT_GATT_CHARACTERISTIC(BT_UUID_IMUS_EXDETECTION, BT_GATT_CHRC_INDICATE,
						BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_exercisedetection_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* Characteristic 3: Command (Control) */
	BT_GATT_CHARACTERISTIC(BT_UUID_IMUS_COMMAND, BT_GATT_CHRC_WRITE,
						BT_GATT_PERM_WRITE, NULL, write_command_cb, NULL)
);

// --- Public API for Peripheral ---
int my_imus_init(struct my_imus_cb *callbacks)
{
	if (callbacks) {
		imus_cb.command_write_cb = callbacks->command_write_cb;
	}
	return 0;
}

int my_imus_send_sensordata(const uint8_t *data, uint16_t len)
{
	if (!sensordata_notify_enabled) {
		return -EACCES;
	}
	// Atributo de SensorData: índice 1
	return bt_gatt_notify(NULL, &my_imus_svc.attrs[1], data, len);
}

bool is_sensordata_notify_enabled(void){return sensordata_notify_enabled;}

int my_imus_send_exercisedetection(const uint8_t *data, uint16_t len)
{
	if (!exercisedetection_indicate_enabled) {
		return -EACCES;
	}
	// Atributo de ExerciseDetection: índice 4
	static struct bt_gatt_indicate_params ind_params;
	ind_params.attr = &my_imus_svc.attrs[4];
	ind_params.func = indicate_cb;
	ind_params.data = data;
	ind_params.len = len;

	return bt_gatt_indicate(NULL, &ind_params);
}

bool is_exercisedetection_indicate_enabled(void){return exercisedetection_indicate_enabled;}

#endif // CONFIG_BT_PERIPHERAL


// =============================================================================
// CENTRAL ROLE IMPLEMENTATION
// =============================================================================
#ifdef CONFIG_BT_CENTRAL

int my_imus_handles_assign(struct bt_gatt_dm *dm, struct bt_imus_client *client)
{
	const struct bt_gatt_dm_attr *chrc_attr, *ccc_attr;

	// --- Característica 1: SensorData ---
	chrc_attr = bt_gatt_dm_char_by_uuid(dm, BT_UUID_IMUS_DATA);
	if (!chrc_attr) {
		LOG_ERR("No se encontró la característica SensorData.");
		return -ENOTSUP;
	}
	// El handle del VALOR es el handle de la declaración + 1.
	client->sensordata_handle = chrc_attr->handle + 1;

	ccc_attr = bt_gatt_dm_desc_by_uuid(dm, chrc_attr, BT_UUID_GATT_CCC);
	if (!ccc_attr) {
		LOG_ERR("No se encontró el CCCD para SensorData.");
		return -ENOTSUP;
	}
	client->sensordata_ccc_handle = ccc_attr->handle;

	// --- Característica 2: ExerciseDetection ---
	chrc_attr = bt_gatt_dm_char_by_uuid(dm, BT_UUID_IMUS_EXDETECTION);
	if (!chrc_attr) {
		LOG_ERR("No se encontró la característica ExerciseDetection.");
		return -ENOTSUP;
	}
	// El handle del VALOR es el handle de la declaración + 1.
	client->exercisedetection_handle = chrc_attr->handle + 1;

	ccc_attr = bt_gatt_dm_desc_by_uuid(dm, chrc_attr, BT_UUID_GATT_CCC);
	if (!ccc_attr) {
		LOG_ERR("No se encontró el CCCD para ExerciseDetection.");
		return -ENOTSUP;
	}
	client->exercisedetection_ccc_handle = ccc_attr->handle;

	// --- Característica 3: Command ---
	chrc_attr = bt_gatt_dm_char_by_uuid(dm, BT_UUID_IMUS_COMMAND);
	if (!chrc_attr) {
		LOG_ERR("No se encontró la característica Command.");
		return -ENOTSUP;
	}
	// El handle del VALOR es el handle de la declaración + 1.
	client->command_handle = chrc_attr->handle + 1;
	// Esta característica no tiene CCCD, por lo que no lo buscamos.

	LOG_INF("Handles del Servicio IMU asignados correctamente.");
	LOG_INF(" - SensorData (valor: 0x%04x, ccc: 0x%04x)", client->sensordata_handle, client->sensordata_ccc_handle);
	LOG_INF(" - ExDetection(valor: 0x%04x, ccc: 0x%04x)", client->exercisedetection_handle, client->exercisedetection_ccc_handle);
	LOG_INF(" - Command (valor: 0x%04x)", client->command_handle);

	return 0;
}

int my_imus_subscribe_sensordata(struct bt_conn *conn, struct bt_imus_client *client, bt_gatt_notify_func_t func)
{
	client->sub_params_sensordata.notify = func;
	client->sub_params_sensordata.value = BT_GATT_CCC_NOTIFY;
	client->sub_params_sensordata.value_handle = client->sensordata_handle;
	client->sub_params_sensordata.ccc_handle = client->sensordata_ccc_handle;
	// atomic_set_bit(client->sub_params_sensordata.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);
	return bt_gatt_subscribe(conn, &client->sub_params_sensordata);
}

int my_imus_subscribe_exercisedetection(struct bt_conn *conn, struct bt_imus_client *client, bt_gatt_notify_func_t func)
{
	client->sub_params_exercisedetection.notify = func;
	client->sub_params_exercisedetection.value = BT_GATT_CCC_INDICATE;
	client->sub_params_exercisedetection.value_handle = client->exercisedetection_handle;
	client->sub_params_exercisedetection.ccc_handle = client->exercisedetection_ccc_handle;
	// atomic_set_bit(client->sub_params_exercisedetection.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);
	return bt_gatt_subscribe(conn, &client->sub_params_exercisedetection);
}

int my_imus_write_command(struct bt_conn *conn, struct bt_imus_client *client, const uint8_t *cmd, uint16_t len, bt_gatt_write_func_t cb)
{
	// static struct bt_gatt_write_params write_params;
	client->write_params.func = cb;
	client->write_params.handle = client->command_handle;
	client->write_params.offset = 0;
	client->write_params.data = cmd;
	client->write_params.length = len;
	return bt_gatt_write(conn, &client->write_params);
}

#endif // CONFIG_BT_CENTRAL
