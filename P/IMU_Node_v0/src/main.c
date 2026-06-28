/*
Hecho por MC. Miguel! - 2025
Primera version de proyecto IMU NODE
Uso de librerias personalizadas de:
- Perfil IMU
  - Notificaciones de datos de sensores
  - Indicaciones de "deteccion de ejercicios"
- Sensor BN055
  - Configuracion basica (reset, cambio de cristal, offsets etc.)
  - Lectura de datos crudos
  - Conversion de datos a unidades

lectura de acc y envió de datos en el mismo hilo
Periferico solicita actualización de parámetros!
- Solicitudes concurrentes!!

MUCHO CUIDADO CON LA ASIGNACION DE STACKS DE MEMORIA
MENOS MENORIA DE LA QUE TIENE ACTUALMENTE HARA QUE EL PROYECTO NO FUNCIONE!!!
comprobado a la mala :'v
*/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/gpio.h>
// Include the custom IMU BLE service
#include "my_imu_ble_service.h"
#include "bno055_driver.h"

LOG_MODULE_REGISTER(IMU_Node, LOG_LEVEL_INF);

// LED configuration
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec grn_led = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec blu_led = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

/******************************************************************************************
					Structure and definitions for the Bluetooth LE
******************************************************************************************/
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME	// Device Bluetooth name defined in prj.conf
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
struct bt_conn *my_conn = NULL;

/***** Advertising parameters strcture *****/
// Small advertising interval for fast connection
static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONN |			// Connectable advertising
	BT_LE_ADV_OPT_USE_IDENTITY), 	// Use identity address (Static random address)
	BT_GAP_ADV_FAST_INT_MIN_1,		/* Min Advertising Interval = Value x 0.625ms = 30ms */
	BT_GAP_ADV_FAST_INT_MAX_1,		/* Max Advertising Interval = Value x 0.625ms = 60ms */
	NULL); /* Address of peer, Set to NULL for undirected advertising */

// Definition of advertising data
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL |	// General advertising
		BT_LE_AD_NO_BREDR)),							// No compatible with BR/EDR (Bluetooth Classic)
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN), // Use complete name
};

// Scan Response -> UUID of LBS Service
static const struct bt_data sd[] = {BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_IMUS_VAL),};

// Structure and definition for change parameters of the connection
static struct bt_gatt_exchange_params exchange_params;	// variable that holds callback for MTU negotiation

// Work item for asynchronous advertising
static struct k_work adv_work;

// Flags of the connection and update of connection parameters
static bool tryUp_ConnP = false, connect = false;
static int8_t try_connP = 0, triesUp = 3;
// NOTE: The retry of update PHY dont work.

/******************************************************************************************
								SENSOR & Send data
******************************************************************************************/
// I2C configuration
#define I2C_NODE DT_NODELABEL(i2c1)
static const struct device *i2c1_dev = DEVICE_DT_GET(I2C_NODE);
static uint8_t units;			// To store the unit configuration of BNO
#define TX_MS_PERIOD	1000	// Period to send data (ms) in send_data_thread()

// SENDDATA THREAD CONFIGURATION
#define SEND_DELAY_MS	4000	// Delay to start the thread
#define SEND_STACKSIZE	2048	// Stack size for the thread SendData
#define SEND_PRIORITY	7		// Thread priority for the thread SendData

/******************************************************************************************
					Functions
******************************************************************************************/

// Thread for sending data periodically
void send_data_thread(void)
{
	uint8_t send_exc = {0};
	uint8_t raw_data[6];
	// float val[3];
	
	while (1) {
		k_msleep(TX_MS_PERIOD);

		// If the client are subscribed to the notify
		if (get_imus_sensordata_notify_enabled()) {
			// Get the raw data of acc
			if (bno055_read_raw_sensor_data(acc, 6, raw_data, 0)) {
				// Send the raw data to the notify
				my_imus_sensordata_notify(raw_data, sizeof(raw_data));
				// Optional print the sensor data
				// LOG_HEXDUMP_INF(raw_data, array_size, "Arreglo"); // HEX FORMAT
				// bno055_convert_to_units(raw_data, 0, val, 0, acc, units);
				// LOG_INF("%.3f %.3f %.3f", (double)val[0], (double)val[1], (double)val[2]);
			}
		}

		// If the client are subscribed to the indicate
		if (get_imus_exercisedetection_indicate_enabled()) {
			my_imus_exercisedetection_indicate(&send_exc, 1);
			send_exc++;
		}
	}
}

// Start the advertisign in the work-queue
static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {LOG_ERR("Advertising failed to start (err %d)\n", err);return;}
	LOG_INF("Advertising successfully started");
}

// Function to start advertising sending the work item to the work-queue
static void advertising_start(void){k_work_submit(&adv_work);}

// Update the connection parameters
static void update_conn_params(struct bt_conn *conn)
{
    int err;
	struct bt_le_conn_param *conn_param = BT_LE_CONN_PARAM(
		CONFIG_DESIRED_MIN_CONN_INTERVAL,	// Min Connection Interval = Value x 1.25ms
		CONFIG_DESIRED_MAX_CONN_INTERVAL,	// Min Connection Interval = Value x 1.25ms
		CONFIG_BT_PERIPHERAL_PREF_LATENCY,	// Latency (Number of connection intervals the peripheral can skip)
		CONFIG_BT_PERIPHERAL_PREF_TIMEOUT	// Time that the central will wait before disconnecting
	);
    err = bt_conn_le_param_update(conn, conn_param);
    if (err) {LOG_ERR("Failed to request connection parameters update (err %d)", err);}
	tryUp_ConnP = true; // Set the flag to true to indicate the try to update the Connection Parameters
}

// Update the connection PHY
static void update_phy(struct bt_conn *conn)
{
	int err;
	const struct bt_conn_le_phy_param preferred_phy = {
		.options = BT_CONN_LE_PHY_OPT_NONE,
		.pref_rx_phy = BT_GAP_LE_PHY_2M,
		.pref_tx_phy = BT_GAP_LE_PHY_2M,
	};
	err = bt_conn_le_phy_update(conn, &preferred_phy);
	if (err) {LOG_ERR("bt_conn_le_phy_update() returned %d", err);}
}

// Update the connection data length
static void update_data_length(struct bt_conn *conn)
{
	int err;
	struct bt_conn_le_data_len_param my_data_len = {
		.tx_max_len = BT_GAP_DATA_LEN_MAX,
		.tx_max_time = BT_GAP_DATA_TIME_MAX,
	};
	err = bt_conn_le_data_len_update(my_conn, &my_data_len);
	if (err) {LOG_ERR("data_len_update failed (err %d)", err);}
}

// Inform the MTU exchange
static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params)
{
	LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
	if (!att_err) {
		uint16_t payload_mtu = bt_gatt_get_mtu(conn) - 3;   // 3 bytes used for Attribute headers.
		LOG_INF("New MTU: %d bytes", payload_mtu);
	}
}

// Beginning the update of the connection's MTU
static void update_mtu(struct bt_conn *conn)
{
	int err;
	exchange_params.func = exchange_func;
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
	}
}

/************************ Connection events callback functions ************************/
// Callback function for announcing the connection or error in the connection
static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {LOG_ERR("Connection failed (err %u)\n", err);return;}
	connect = true; // Set the connected flag to true
	LOG_INF("Connected!!!");
	LOG_INF("Reading intitial parameters...");

	my_conn = bt_conn_ref(conn);
	struct bt_conn_info info;
	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_ERR("bt_conn_get_info() returned %d", err);
		return;
	}

	// Connection parameters to the log
	double connection_interval = info.le.interval*1.25;	// in ms
	uint16_t supervision_timeout = info.le.timeout*10;	// in ms
	LOG_INF("- Connection parameters: interval %.2f ms, latency %d intervals, timeout %d ms", 
		connection_interval, info.le.latency, supervision_timeout);
	
	// Data Length parameters to the log
	LOG_INF("- Data length: TX %d bytes / %d us, RX %d bytes / %d us",
            info.le.data_len->tx_max_len, info.le.data_len->tx_max_time,
            info.le.data_len->rx_max_len, info.le.data_len->rx_max_time);

	// UPDATE the connection parameters
	LOG_INF("Beginning request for parameters update...");
	if (info.le.interval < CONFIG_DESIRED_MIN_CONN_INTERVAL ||
		info.le.interval > CONFIG_DESIRED_MAX_CONN_INTERVAL ||
		info.le.latency != CONFIG_BT_PERIPHERAL_PREF_LATENCY) {
			LOG_INF("Negociando conn params");
			update_conn_params(my_conn);
			k_sleep(K_MSEC(500));	// Small delay to not overload the central with requests.
		} else tryUp_ConnP = true;
	
	update_phy(my_conn);
	k_sleep(K_MSEC(500));	// Delay to avoid link layer collisions.
	update_data_length(my_conn);
	update_mtu(my_conn);
}

//Callback function for announcing the disconnection
static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)\n", reason);
	bt_conn_unref(my_conn);
	// Reset the update flags
	tryUp_ConnP = false;
	try_connP = 0;
	connect = false; // Reset the connected flag
}

// Callback function for when a connection object is recycled (restart advertising)
static void recycled_cb(void){LOG_INF("Disconnect complete. Restart advertising\n");advertising_start();}

// Callback to inform the connection parameter update
void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	double connection_interval = interval*1.25;         // in ms
	uint16_t supervision_timeout = timeout*10;          // in ms
	LOG_INF("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, latency, supervision_timeout);
	
	if (interval > CONFIG_DESIRED_MAX_CONN_INTERVAL && try_connP < triesUp) {
		try_connP++;
		update_conn_params(my_conn);
		LOG_INF("RETRYING connection parameters update (%d/%d)", try_connP, triesUp);
	}
}

// Callback to inform the update of PHY
void on_le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
	// PHY Updated
	if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_1M) {LOG_INF("PHY updated. New PHY: 1M");}
	else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_2M) {LOG_INF("PHY updated. New PHY: 2M");}
	else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_CODED_S8) {LOG_INF("PHY updated. New PHY: Long Range");}
}

// Callback to inform the update of data length
void on_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
	uint16_t tx_len  = info->tx_max_len; 
	uint16_t tx_time = info->tx_max_time;
	uint16_t rx_len  = info->rx_max_len;
	uint16_t rx_time = info->rx_max_time;
	LOG_INF("Data length updated. Length %d/%d bytes, time %d/%d us", tx_len, rx_len, tx_time, rx_time);
}

// Callback structure for connections events
struct bt_conn_cb connection_callbacks = {
	.connected				= on_connected,
	.disconnected			= on_disconnected,
	.recycled				= recycled_cb,
	.le_param_updated		= on_le_param_updated,
	.le_phy_updated			= on_le_phy_updated,
	.le_data_len_updated    = on_le_data_len_updated,
};

// Callback function for receiving data from the Exercise Detection characteristic
static void app_exercise_detec_cb(const uint8_t *buf, uint16_t len)
{
	LOG_HEXDUMP_INF(buf, len, "Datos recibidos en Exercise Detection Char:");
}

static struct my_imus_cb app_callbacks = {
	.exdetection_write_cb = app_exercise_detec_cb, 	// Exercise detection callback
};


// MAIN FUNCTION ********************************************************************
int main(void)
{
	LOG_INF("Starting IMU-Node");
	int err;

	//================================================================================
	//                                   LED'S
	//================================================================================
	// Initialize LED
	if (!gpio_is_ready_dt(&red_led) && !gpio_is_ready_dt(&grn_led) && !gpio_is_ready_dt(&blu_led)) {
		LOG_ERR("Error led gpio\n");
		return -1;
	}
	gpio_pin_configure_dt(&red_led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&grn_led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&blu_led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set_dt(&red_led, 0);
	gpio_pin_set_dt(&grn_led, 0);
	gpio_pin_set_dt(&blu_led, 0);

	//================================================================================
	//                                   SENSOR
	//================================================================================
	k_msleep(100);
	if (!device_is_ready(i2c1_dev)) {
		LOG_ERR("I2C-1 not ready");
		return -1;
	}

	if (!bno055_init(i2c1_dev, BNO_OPR_MODE_NDOF)) {
		LOG_ERR("BNO055 init failed.");
		gpio_pin_set_dt(&red_led, 1);
		return -1;
	} else {
		LOG_INF("BNO055 init OK.");
		bno055_change_crystal(true);				// Set external crystal
		uint8_t config_values[5];
		bno055_read_config(config_values, false);	// Read the configuration values
		units = config_values[3];					// Store the units configuration
	}

	//================================================================================
	//                                   BLE
	//================================================================================
	// Eneable the bluetooth stack
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return -1;
	}

	// Initialize the IMU service with the application callbacks
	err = my_imus_init(&app_callbacks);
	if (err) {
		LOG_ERR("Fail to init IMU Service (err %d)\n", err);
		return -1;
	}

	// Get the Bluetooth device address
	bt_addr_le_t addr;
	size_t count = 1;
	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_id_get(&addr, &count);
	bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
	LOG_INF("BLE Name=%s, Address=%s", CONFIG_BT_DEVICE_NAME, addr_str);

	// Register the connections events callbacks
	bt_conn_cb_register(&connection_callbacks);
	// Initialization of the work item for advertising
	k_work_init(&adv_work, adv_work_handler);
	// Start advertising using the the work-queue
	advertising_start();

	//================================================================================
	//                                LED Blink
	//================================================================================
	for (;;) {
		k_msleep(2000);
		if (connect) gpio_pin_toggle_dt(&blu_led);
		else gpio_pin_set_dt(&blu_led, 0);
	}
}

// Define and initialize a thread to send data periodically
K_THREAD_DEFINE(send_data_thread_id, SEND_STACKSIZE, send_data_thread, NULL, NULL, NULL, SEND_PRIORITY, 0, SEND_DELAY_MS);
