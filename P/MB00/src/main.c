/*
Hecho por MC. Miguel! - 2025
Implementacion de servidor BLE. Lee datos del BNO (I2C) -> Transmite por BLE.

1. Muestreo del Sensor 
   - Temporizador del kernel a 60 Hz. 
   - La lectura se delega a una cola de trabajo (work queue) dedicada.
2. Empaquetado de Datos
   - Agrupa dos muestras consecutivas del acelerómetro + número de secuencia en un único paquete de datos.
3. Transmisión BLE
   - Anuncia el dispositivo como un periférico BLE conectable.
   - Una vez que un dispositivo central se conecta, los paquetes de datos se envían mediante notificaciones GATT.
   - El inicio y stop del muestreo y envió se controlan de forma remota escribiendo un comando en la característica Exercise Detection.
     - 0xAA = Inicio.
     - 0xFF = Stop.
4. Sincronización y Concurrencia
   - Usa un semáforo para notificar al hilo de envío de BLE cuándo hay un nuevo paquete listo.
   - Mutex para proteger el búfer de datos compartido contra accesos concurrentes.
5. Gestión y Feedback
   - Gestiona los parámetros de la conexión BLE (PHY, longitud de datos, MTU).
   - LED's parpadeando en verde = Esperando conexión a central
   - LED's parpadeando en azul = Conectado a central
- Siempre se intenta transmitir la ultima muestra!!!
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
#include <zephyr/drivers/i2c.h>

// Include the custom IMU BLE service & BNO driver
#include "my_imu_ble_service.h"
#include "bno055_driver.h"

LOG_MODULE_REGISTER(IMU_server, LOG_LEVEL_INF);

//================================================================================
//                                LED configuration
//================================================================================
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec grn_led = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec blu_led = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

//================================================================================
//                                BLE configuration
//================================================================================
// Flags to know the status "update" of: PHY, Data Length and Connection Parameters
static bool connect = false;				// Dispositivo conectado?
static int8_t try_connP = 0, triesUp = 3;	// Intentos de negociacion ConnP, Maximo de intentos

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

// Work item for advertising & connection setup
static struct k_work adv_work;

//================================================================================
//                              SENSOR y MUESTREO
//================================================================================
// Hilo para enviar el paquete por BLE
#define SEND_STACKSIZE		4096 
#define SEND_PRIORITY		8
// Hilo para muestrear y formar el paquete
#define GETSAMPLE_STACKSIZE	4096
#define GETSAMPLE_PRIORITY	7

#define SAMPLING_RATE_HZ 60
#define SAMPLING_PERIOD_US (1000000 / SAMPLING_RATE_HZ)

#define I2C_NODE DT_NODELABEL(i2c1)
static const struct device *i2c1_dev = DEVICE_DT_GET(I2C_NODE);
uint8_t units; // Store the unit configuration of BNO

// Estructura para muestra -> paquete a transmitir
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} sensor_sample_t;

typedef struct {
	uint32_t packet_num; 
    sensor_sample_t sample1;
    sensor_sample_t sample2;
} ble_packet_t;

static uint32_t packet_counter = 0;
static bool first_sample_taken = false;
static bool start_send = false;

/*** Recursos del Kernel para el manejo del sensor ***/
// 1. Temporizador
static struct k_timer sensor_timer;

// 2. Cola de trabajo dedicada para leer el sensor fuera de la ISR
// static K_KERNEL_STACK_DEFINE(sensor_work_q_stack, GETSAMPLE_STACKSIZE);
static struct k_work_q sensor_work_q;
static struct k_work sensor_work;

// 3. Mailbox
static ble_packet_t latest_ble_packet;	// Unico buffer
static struct k_mutex packet_mutex;		// Mutex
static struct k_sem ble_data_sem;		// Semaforo

// DECLARACIONES PARA HILOS TOMA DE MUESTRAS Y ENVIO
K_KERNEL_STACK_MEMBER(sensor_work_q_stack, GETSAMPLE_STACKSIZE);
K_KERNEL_STACK_MEMBER(send_thread_stack, SEND_STACKSIZE);

static struct k_thread send_thread_data; // Estructura de datos del hilo
k_tid_t send_thread_tid;                 // ID del hilo

//================================================================================
//                         FUNCIONES MUESTREO Y ENVIO
//================================================================================
// Función leer sensor y empaquetar datos
static void sensor_work_handler(struct k_work *work)
{
	if (!start_send) {
		return;
	}

	static ble_packet_t packet_in_progress;
	uint8_t raw_accel_data[6];

	// Leer los datos crudos del acelerómetro (6 bytes)
	if (!bno055_read_raw_sensor_data(acc, 6, raw_accel_data, 0)) {
		LOG_ERR("Failed to read sensor data");
		return;
	}
	
	// El BNO055 devuelve los datos en formato Little Endian (LSB, MSB)
	int16_t ax = (int16_t)((raw_accel_data[1] << 8) | raw_accel_data[0]);
	int16_t ay = (int16_t)((raw_accel_data[3] << 8) | raw_accel_data[2]);
	int16_t az = (int16_t)((raw_accel_data[5] << 8) | raw_accel_data[4]);

	if (!first_sample_taken) {
		// Es la primera muestra -> la guardamos
		packet_in_progress.sample1.x = ax;
		packet_in_progress.sample1.y = ay;
		packet_in_progress.sample1.z = az;
		first_sample_taken = true;
	} else {
		// Es la segunda muestra -> completamos el paquete y lo enviamos a la cola
		packet_in_progress.sample2.x = ax;
		packet_in_progress.sample2.y = ay;
		packet_in_progress.sample2.z = az;
		packet_in_progress.packet_num = packet_counter;
		
		// LÓGICA DEL MAILBOX
		// Bloquear el mutex para acceder de forma segura al mailbox
		k_mutex_lock(&packet_mutex, K_FOREVER);
		// Sobrescribir el mailbox con el paquete más reciente
		latest_ble_packet = packet_in_progress;
		// Desbloquear el mutex
		k_mutex_unlock(&packet_mutex);
		// Señalizar al hilo BLE que hay nuevos datos listos
		k_sem_give(&ble_data_sem);

		// Incremento del numero de paquete
		packet_counter++;
		// Reseteamos para la siguiente pareja de muestras
		first_sample_taken = false;
	}
}

// ISR del temporizador: solo envía el trabajo a la cola
static void timer_expiry_function(struct k_timer *timer_id)
{k_work_submit_to_queue(&sensor_work_q, &sensor_work);}

// Thread for sending data periodically
void send_data_thread(void *p1, void *p2, void *p3)
{
	// ARG_UNUSED se usa para decirle al compilador que es intencional
	// que no usemos estos parámetros. Esto evita warnings de "unused variable".
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

    ble_packet_t packet_to_send;

    while (1) {
        // 1. Esperar a la señal del semaforo. El hilo dormirá aquí sin consumir CPU.
        k_sem_take(&ble_data_sem, K_FOREVER);

        // 2. Una vez despierto, copiar el paquete más reciente del mailbox a un búfer local.
        //    Mutex para garantizar la integridad de los datos.
        k_mutex_lock(&packet_mutex, K_FOREVER);
        packet_to_send = latest_ble_packet;
        k_mutex_unlock(&packet_mutex);

        // 3. Si estamos conectados, suscritos y recibimos el comando
		// if (connect && get_imus_sensordata_notify_enabled()) { //  && start_send
        if (get_imus_sensordata_notify_enabled() && start_send) {
            my_imus_sensordata_notify((uint8_t *)&packet_to_send, sizeof(packet_to_send));
        }
    }
}

//================================================================================
//                                  FUNCIONES BLE
//================================================================================
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
    if (err) LOG_ERR("Failed to request connection parameters update (err %d)", err);
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
	if (err) LOG_ERR("bt_conn_le_phy_update() returned %d", err);
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
	if (err) LOG_ERR("data_len_update failed (err %d)", err);
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
	if (err) LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
}

/************************ Connection events callback functions ************************/
// Callback function for announcing the connection or error in the connection
static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err %u)\n", err);
		return;
	}

	connect = true;
	LOG_INF("Connected. Reading intitial parameters...");

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
	}
	
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

	// Reset the connected flag
	connect = false; 
	// Reset the retry of update connection parameters
	try_connP = 0;

	// Detener muestreo en desconexión
	k_timer_stop(&sensor_timer);
	start_send = false;
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

	if (len == 1) {
		// Iniciar muestreo
		if (buf[0] == 0xAA) {
			start_send = true;
			packet_counter = 0;
			first_sample_taken = false;
			k_sem_take(&ble_data_sem, K_NO_WAIT); 
			k_timer_start(&sensor_timer, K_MSEC(1000), K_USEC(SAMPLING_PERIOD_US));
		}

		// Detener muestreo
		if (buf[0] == 0xFF) {
			k_timer_stop(&sensor_timer);
			start_send = false;
		}
	}
}

static struct my_imus_cb app_callbacks = {
	.exdetection_write_cb = app_exercise_detec_cb, 	// Exercise detection callback
};

// MAIN FUNCTION ********************************************************************
int main(void)
{
	int err;
	//================================================================================
	//                                  1. LED'S
	//================================================================================
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
	//                                 2. SENSOR
	//================================================================================
	if (!device_is_ready(i2c1_dev)) {
		LOG_ERR("I2C-1 not ready");
		return -1;
	}

	LOG_INF("Starting IMU-Node. Hardware OK.");

	if (!bno055_init(i2c1_dev, BNO_OPR_MODE_NDOF)) {
		LOG_ERR("BNO055 init failed.");
		gpio_pin_set_dt(&red_led, 1);
		return -1;
	} else {
		LOG_INF("BNO055 init OK.");
		bno055_change_crystal(true);
		uint8_t config_values[5];
		bno055_read_config(config_values, false);
		units = config_values[3];
	}

	//================================================================================
	//                            3. TOMA DE MUESTRAS
	//================================================================================
	// Inicializar Mutex y Semáforo PRIMERO
	k_mutex_init(&packet_mutex);
	k_sem_init(&ble_data_sem, 0, 1);

	// Cola de trabajo dedicada
	k_work_queue_start(&sensor_work_q, sensor_work_q_stack, K_KERNEL_STACK_SIZEOF(sensor_work_q_stack),
		K_PRIO_COOP(GETSAMPLE_PRIORITY), NULL);
	k_work_init(&sensor_work, sensor_work_handler);

	// Inicializar el temporizador
	k_timer_init(&sensor_timer, timer_expiry_function, NULL);
	// LOG_INF("Kernel objects for sampling are initialized.");

	// Hilo de envío BLE (Thread)
	send_thread_tid = k_thread_create(&send_thread_data, send_thread_stack, K_KERNEL_STACK_SIZEOF(send_thread_stack), 
		send_data_thread, NULL, NULL, NULL, SEND_PRIORITY, 0, K_NO_WAIT);

	if (!send_thread_tid) {
		LOG_ERR("Failed to create send_data_thread");
		return -1;
	}
	k_thread_name_set(send_thread_tid, "ble_send_thread");
	// LOG_INF("BLE Send Thread created.");

	//================================================================================
	//                               4. BLUETOOTH
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
	// Register the connections events callbacks
	bt_conn_cb_register(&connection_callbacks);
	// Initialization of the work item for advertising
	k_work_init(&adv_work, adv_work_handler);
	// Start advertising using the the work-queue
	advertising_start();
	// LOG_INF("Bluetooth stack started and advertising.");

	// Get the Bluetooth device address
	bt_addr_le_t addr;
	size_t count = 1;
	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_id_get(&addr, &count);
	bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
	LOG_INF("BLE Name=%s, Address=%s", CONFIG_BT_DEVICE_NAME, addr_str);
	
	//================================================================================
	//                                LED Blink
	//================================================================================
	for (;;) {
		k_msleep(2000);
		if (connect) {
			gpio_pin_toggle_dt(&blu_led);
			gpio_pin_set_dt(&grn_led, 0);
		} else {
			gpio_pin_toggle_dt(&grn_led);
			gpio_pin_set_dt(&blu_led, 0);
		}
	}
}
