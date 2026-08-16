/*
- Uso de timer de hw
*/
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
// BLE
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/gatt.h>
// ISO
#include <zephyr/bluetooth/iso.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
// Timer
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/counter.h>
#include <hal/nrf_clock.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_power.h>
// gpio & sensor
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
// Include the custom IMU BLE service & BNO driver
#include "my_imu_ble_service.h"
#include "bno055_driver.h"

LOG_MODULE_REGISTER(P, LOG_LEVEL_INF);

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
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME	// Device Bluetooth name defined in prj.conf
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
struct bt_conn *my_conn = NULL;

// Dirección MAC del Broadcaster/Central Específico (Little Endian)
// FB:5C:2A:77:77:0A (Random) -> Pasar en lista en Big Endian
static const bt_addr_le_t target_addr = {
    .type = BT_ADDR_LE_RANDOM, // O BT_ADDR_LE_PUBLIC si es pública
	// .a = {.val = {0x, 0x, 0x, 0x, 0x, 0x}}
	// .a = {.val = {0xC3, 0xFB, 0xF6, 0xBA, 0xC4, 0xCD}}
    // .a = {.val = {0x0A, 0x77, 0x77, 0x2A, 0x5C, 0xFB}}
	// .a = {.val = {0x2C, 0xEB, 0x6B, 0x4F, 0x8A, 0xF3}}
	// .a = {.val = {0xB3, 0x1C, 0x96, 0xB9, 0xCC, 0xE5}}
	// .a = {.val = {0xAF, 0xA4, 0xDE, 0x47, 0x9B, 0xC6}}
	.a = {.val = {0x86, 0xD4, 0x3C, 0x99, 0x28, 0xFA}}
	// .a = {.val = {0xDA, 0x37, 0x2B, 0x40, 0x58, 0xD3}}
};

// Advertising parameters strcture
static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	BT_LE_ADV_OPT_CONN |			// Connectable advertising
	BT_LE_ADV_OPT_USE_IDENTITY |	// Use identity address (Static random address)
	BT_LE_ADV_OPT_FILTER_CONN, 		// Eneable Whitelist
	BT_GAP_ADV_FAST_INT_MIN_1,		/* Min Advertising Interval = Value x 0.625ms = 30ms */
	BT_GAP_ADV_FAST_INT_MAX_1,		/* Max Advertising Interval = Value x 0.625ms = 60ms */
	NULL							/* Address of peer, Set to NULL for undirected advertising */
);

// Definition of advertising data
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL |	// General advertising
		BT_LE_AD_NO_BREDR)),							// No compatible with BR/EDR (Bluetooth Classic)
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN), // Use complete name
};

// Scan Response -> UUID of LBS Service
static const struct bt_data sd[] = {BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_IMUS_VAL),};

// Work item for advertising & connection setup
static struct k_work adv_work;

// BLE - ISO =====================================================================
#define BT_LE_SCAN_CUSTOM BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE, BT_LE_SCAN_OPT_NONE, BT_GAP_SCAN_FAST_INTERVAL, BT_GAP_SCAN_FAST_WINDOW)
#define NAME_LEN		30
#define PA_RETRY_COUNT	3
#define BIS_ISO_CHAN_COUNT 1U

static bool         per_adv_found, per_adv_lost;
static bt_addr_le_t per_addr;
static uint8_t      per_sid;
static uint32_t     per_interval_us;

static struct bt_le_per_adv_sync *sync;
static struct bt_iso_big *big;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_sync_lost, 0, 1);
static K_SEM_DEFINE(sem_sync_timer, 0, 1);
static K_SEM_DEFINE(sem_per_big_info, 0, 1);
static K_SEM_DEFINE(sem_big_sync, 0, BIS_ISO_CHAN_COUNT);

static uint32_t re_sync_k = 10;

//================================================================================
//                              SENSOR y MUESTREO
//================================================================================
// Hilo para enviar el paquete por BLE
#define SEND_STACKSIZE		4096
#define SEND_PRIORITY		8
// Hilo para muestrear y formar el paquete
#define GETSAMPLE_STACKSIZE	4096
#define GETSAMPLE_PRIORITY	4

// SENSOR
#define I2C_NODE DT_NODELABEL(i2c1)
static const struct device *i2c1_dev = DEVICE_DT_GET(I2C_NODE);

static uint64_t fs_hz = 60, ts_us, delta_us = 8000;
static uint32_t n_pkt = 0, stop_pkt, counter_tx;
static uint32_t now_ticks, ts_ticks, ts_first_tick, ts_resync;
static uint8_t units, samples_x_pkt = 3, samp_x_iso_Intrvl = 4;

static bool ble_tx = false, first_cycle = false, sync_ok = false;
static bool tx_notify = true, tx_indicate = false, bno_ok = false;

// Estructura para muestra -> paquete a transmitir
# define SAMPLE_SIZE 20 // 8 Quat + 6 Acc + 6 Gyr = 20
# define PACKET_SIZE 64 // 3 samples (3x20) + 4 (no. pkt) = 64
static uint8_t last_packet[PACKET_SIZE] = {0};

// Recursos del Kernel para el manejo del sensor ============================
// 1. HARDWARE TIMER ========================================================
const struct device *hw_timer_dev = DEVICE_DT_GET(DT_NODELABEL(timer2));
// 2. Cola de trabajo dedicada para leer el sensor fuera de la ISR
static struct k_work_q sensor_work_q;
static struct k_work sensor_work;

// 3. Mailbox
static struct k_mutex packet_mutex;		// Mutex
static struct k_sem ble_data_sem;		// Semaforo

// NUEVAS DECLARACIONES
K_KERNEL_STACK_MEMBER(sensor_work_q_stack, GETSAMPLE_STACKSIZE); // Stack para la work queue
K_KERNEL_STACK_MEMBER(send_thread_stack, SEND_STACKSIZE);       // Stack para el hilo de envío

static struct k_thread send_thread_data; // Estructura de datos del hilo
k_tid_t send_thread_tid;                 // ID del hilo

static struct counter_alarm_cfg alarm_cfg0;
//================================================================================
//                         Mensaje guardado
//================================================================================
static uint16_t idx = 0;
static int16_t motion_data[312] = {
	13924, -8583, -193, -914, 13959, -8528, -217, -906, 14001, -8459, -244, -895,
	14079, -8328, -323, -874, 14250, -8030, -411, -847, 14385, -7786, -435, -836,
	14546, -7477, -507, -825, 14811, -6923, -692, -817, 15059, -6358, -756, -809,
	15287, -5800, -746, -739, 15608, -4891, -711, -638, 15823, -4153, -697, -588,
	15992, -3451, -713, -538, 16176, -2443, -749, -500, 16263, -1753, -802, -491,
	16320, -1092, -812, -498, 16356, -227, -784, -486, 16352, 480, -799, -438,
	16311, 1246, -828, -386, 16207, 2216, -856, -357, 16096, 2918, -848, -364,
	15986, 3463, -853, -393, 15820, 4150, -867, -418, 15671, 4689, -831, -419,
	15387, 5564, -751, -411, 15094, 6318, -748, -345, 14776, 7033, -770, -222,
	14008, 8471, -674, -71, 14008, 8471, -674, -71, 13366, 9458, -579, 42,
	13025, 9919, -613, 129, 12678, 10364, -503, 173, 12229, 10898, -313, 160,
	11912, 11243, -308, 206, 11912, 11243, -308, 206, 11136, 12012, -172, 324,
	10756, 12352, -126, 406, 10446, 12612, -113, 483, 10316, 12718, -25, 522,
    10316, 12718, -25, 522, 10446, 12612, -113, 483, 10756, 12352, -126, 406,
    11136, 12012, -172, 324, 11912, 11243, -308, 206, 11912, 11243, -308, 206,
    12229, 10898, -313, 160, 12678, 10364, -503, 173, 13025, 9919, -613, 129,
    13366, 9458, -579, 42, 14008, 8471, -674, -71, 14008, 8471, -674, -71,
    14776, 7033, -770, -222, 15094, 6318, -748, -345, 15387, 5564, -751, -411,
    15671, 4689, -831, -419, 15820, 4150, -867, -418, 15986, 3463, -853, -393,
    16096, 2918, -848, -364, 16207, 2216, -856, -357, 16311, 1246, -828, -386,
    16352, 480, -799, -438, 16356, -227, -784, -486, 16320, -1092, -812, -498,
    16263, -1753, -802, -491, 16176, -2443, -749, -500, 15992, -3451, -713, -538,
    15823, -4153, -697, -588, 15608, -4891, -711, -638, 15287, -5800, -746, -739,
    15059, -6358, -756, -809, 14811, -6923, -692, -817, 14546, -7477, -507, -825,
    14385, -7786, -435, -836, 14250, -8030, -411, -847, 14079, -8328, -323, -874,
    14001, -8459, -244, -895, 13959, -8528, -217, -906, 13924, -8583, -193, -914
};
static uint8_t motion_pkt[624];

//================================================================================
//                         FUNCIONES MUESTREO Y ENVIO
//================================================================================
// Función leer sensor y empaquetar datos
static void sensor_work_handler(struct k_work *work) {
	static uint8_t sample_taken[SAMPLE_SIZE] = {0}, packet_in_progress[PACKET_SIZE] = {0};
	static uint8_t indx_pos, temporal_sample_counter;

	// 1. Tomar muestra (ESTO SE HACE PRIMERO!!!)
	// Guardamos datos quat + acc + gyr
	if (bno_ok) {
		bno055_read_raw_sensor_data(BNO_REG_QUAT, 8, sample_taken, 0);
		bno055_read_raw_sensor_data(BNO_REG_ACC, 6, sample_taken, 8);
		bno055_read_raw_sensor_data(BNO_REG_GYR, 6, sample_taken, 14);
	}

	// En caso de reinicio -> reset a packet_in_progress (para no enviar data mezclada)
	if (first_cycle) {
		first_cycle = false;
		memset(packet_in_progress, 0, sizeof(packet_in_progress));
		temporal_sample_counter = 0;
		idx = 0;
	}

	// Movimiento guardado
	if (!bno_ok) {
		memcpy(sample_taken, &motion_pkt[idx], 8);
		idx = idx + 8;
		if (idx >= 624) idx = 0;
	}

	// Acomodamos la muestra segun el contador temporal
	// Desplazamiento de 4B para agregar el No. de pkt al inicio
	indx_pos = temporal_sample_counter * SAMPLE_SIZE + 4;
	memcpy(&packet_in_progress[indx_pos], sample_taken, SAMPLE_SIZE);

	// Si ya es hora de enviar el paquete
	if (((temporal_sample_counter + 1) % samples_x_pkt) == 0) {
		// Agregamos el numero de pkt al inicio
		memcpy(packet_in_progress, &n_pkt, 4);

		// LÓGICA DEL MAILBOX
		// Bloquear el mutex para acceder de forma segura al mailbox
		k_mutex_lock(&packet_mutex, K_FOREVER);
		// Sobrescribir el mailbox con el paquete más reciente
		memcpy(last_packet, &packet_in_progress, sizeof(packet_in_progress));
		// Desbloquear el mutex
		k_mutex_unlock(&packet_mutex);
		// Señalizar al hilo BLE que hay nuevos datos listos
		// if((n_pkt % 15) == 0) k_sem_give(&ble_data_sem);
		k_sem_give(&ble_data_sem);

		n_pkt++;
	}

	// Ajuste a contadores
	temporal_sample_counter++;
	if (temporal_sample_counter == samples_x_pkt) temporal_sample_counter = 0;
}

void mi_alarma_hw_callback_0(const struct device *dev, uint8_t chan_id, uint32_t ticks, void *user_data)
{
	// gpio_pin_toggle_dt(&blu_led);

	// Re-armamos la alarma para el próximo periodo
	alarm_cfg0.ticks = ticks + ts_ticks;
	counter_set_channel_alarm(dev, chan_id, &alarm_cfg0);
	k_work_submit_to_queue(&sensor_work_q, &sensor_work);
}

// Thread for sending data periodically
void send_data_thread(void *p1, void *p2, void *p3)
{
	// ARG_UNUSED se usa para decirle al compilador que es intencional
	// que no usemos estos parámetros. Esto evita warnings de "unused variable".
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

    uint8_t packet_to_send[PACKET_SIZE] = {0};
	uint32_t num = 0;
	int err;

    while (1) {
        // 1. Esperar a la señal del semaforo. El hilo dormirá aquí sin consumir CPU.
        k_sem_take(&ble_data_sem, K_FOREVER);
        // 2. Una vez despierto, copiar el paquete más reciente del mailbox a un búfer local.
        // Mutex para garantizar la integridad de los datos.
        k_mutex_lock(&packet_mutex, K_FOREVER);
		memcpy(packet_to_send, &last_packet, sizeof(last_packet));
        k_mutex_unlock(&packet_mutex);

		// 3. Si estamos conectados, suscritos y recibimos el comando
		if (ble_tx) {
			memcpy(&num, packet_to_send, sizeof(num));
			if (tx_notify) {
				err = my_imus_send_sensordata(packet_to_send, sizeof(packet_to_send));
				LOG_INF("N[%u][%dB]", num, PACKET_SIZE);
				if (err) LOG_WRN("Fallo notificacion (err %d)", err);

				// bool scape = false;
				// for (int i = 0; i < PACKET_SIZE; i++) {
				// 	if (packet_to_send[i] == 0x7E || packet_to_send[i] == 0x7F || packet_to_send[i] == 0x7D) {
				// 		scape = true;
				// 		break;
				// 	}
				// }
				// if (scape) printk("scape byte -> true\n\n");

				// for (int i = 0; i < PACKET_SIZE; i++) {
				// 	printk("%02X", packet_to_send[i]);
				// 	if (packet_to_send[i] == 0x7E || packet_to_send[i] == 0x7F || packet_to_send[i] == 0x7D) {
				// 		printk("* ");
				// 	} else printk(" ");
				// }
				// printk("\n\n");
			}

			if (tx_indicate){
				err = my_imus_send_exercisedetection(packet_to_send, sizeof(packet_to_send));
				LOG_INF("I[%u][%dB]", num, PACKET_SIZE);
				if (err) LOG_WRN("Fallo indicacion (err %d)", err);
			}

			// Si esta activo stop pkt
			if (stop_pkt > 0) {
				counter_tx++;
				if (counter_tx == stop_pkt) ble_tx = false;
			}
		}
    }
}

//================================================================================
//                                  FUNCIONES BLE
//================================================================================
// Start the advertisign in the work-queue
static void adv_work_handler(struct k_work *work)
{
	int err;

	// Añadir filtro -> SOLO ACEPTA CONEXIONES CON UN DISPOSITIVO ESPECIFICO
	err = bt_le_filter_accept_list_add(&target_addr);
    if (err) LOG_WRN("No se pudo agregar a whitelist (err %d)", err);

	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {LOG_ERR("Advertising failed to start (err %d)\n", err);return;}
	LOG_INF("Advertising successfully started");
}

// Function to start advertising sending the work item to the work-queue
static void advertising_start(void){k_work_submit(&adv_work);}

/************************ Connection events callback functions ************************/
// Callback function for announcing the connection or error in the connection
static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err %u)\n", err);
		return;
	}
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
}

//Callback function for announcing the disconnection
static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)", reason);
	bt_conn_unref(my_conn);

	// Reset the connected flag
	sync_ok = false;
	counter_cancel_channel_alarm(hw_timer_dev, 0);
	k_sem_give(&sem_sync_lost);

	k_sem_take(&ble_data_sem, K_NO_WAIT); 	// Tomamos el turno
	ble_tx = false;							// Deshabilitamos la transmision
	// gpio_pin_set_dt(&blu_led, 0);
}

// Callback para las solicitudes de cambios en los parametros de conexion
static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{LOG_INF("Connection parameters update requested by central."); return true;}

// Callback to inform the connection parameter update
void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	double connection_interval = interval*1.25;         // in ms
	uint16_t supervision_timeout = timeout*10;          // in ms
	LOG_INF("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, latency, supervision_timeout);
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
	 // .recycled				= recycled_cb,
	.le_param_req			= le_param_req,
	.le_param_updated		= on_le_param_updated,		// Uso opcional solo informan
	.le_phy_updated			= on_le_phy_updated,		// Uso opcional solo informan
	.le_data_len_updated    = on_le_data_len_updated,	// Uso opcional solo informan
};

//******************************************************************************************************** */
// BLE ISO

// Scan and find Periodic Adv (PA)
static void scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf)
{
	// Si aun no se encuenta un PA y se acaba de detectar uno
	// Broadcas Adv -> Ext Adv. Si info->interval !=0 -> PA
	if (!per_adv_found && info->interval) {
		char le_addr[BT_ADDR_LE_STR_LEN];
		bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
		per_interval_us = BT_CONN_INTERVAL_TO_US(info->interval);
		char target[BT_ADDR_LE_STR_LEN];
		bt_addr_le_to_str(&target_addr, target, sizeof(target));
		LOG_INF("PA %s, Target=%s", le_addr, target);

		// Si el PA es del dispositivo esperado
		if(bt_addr_le_cmp(info->addr, &target_addr) == 0){
			per_adv_found = true;
			per_sid = info->sid;
			bt_addr_le_copy(&per_addr, info->addr);
			k_sem_give(&sem_per_adv);
		}
	}
}
static struct bt_le_scan_cb scan_callbacks = {.recv = scan_recv,};

// Sync to PA
static void sync_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info)
{
	LOG_INF("PAS_C[%u-%ums]", bt_le_per_adv_sync_get_index(sync), info->interval * 5 / 4);
	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_term_info *info)
{
	// char le_addr[BT_ADDR_LE_STR_LEN];
	// bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	// LOG_INF("PAS_C[%u] %s sync terminated", bt_le_per_adv_sync_get_index(sync), le_addr);
	per_adv_lost = true;
}

// 1st step for ISO - Here we can get some ISO parameters
static void biginfo_cb(struct bt_le_per_adv_sync *sync, const struct bt_iso_biginfo *biginfo)
{/*LOG_INFO("BIG_INF_CB");*/ k_sem_give(&sem_per_big_info);}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.biginfo = biginfo_cb,
};

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info, struct net_buf *buf)
{
	static uint32_t temp_k = 0, iso_seq_n = 0;

	counter_get_value(hw_timer_dev, &now_ticks);
	temp_k++;

	// Si es un paquete invalido
    if (!(info->flags & BT_ISO_FLAGS_VALID)) {
		// LOG_ERR("X");
		return; // NOS VAMOS!!!
	}
	
	// Obtenemos el numero de secuencia actual del BIG (uint_32)
	memcpy(&iso_seq_n, buf->data, 4);

	// Si no estamos sincronizados
	if (!sync_ok) {
		// Configuramos la alarma del timer con un desfase
		alarm_cfg0.ticks = now_ticks + ts_first_tick;
		counter_set_channel_alarm(hw_timer_dev, 0, &alarm_cfg0);

		// Configuracion de variables
		temp_k = 0;
		sync_ok = true;
		first_cycle = true;
		k_sem_give(&sem_sync_timer);
	// }
	} else {
		// Si ya estamos sincronizados
		// Y toca resyncronizar
		if ((iso_seq_n % re_sync_k) == 0 || temp_k >= re_sync_k) {
			// LOG_INF("%u-%u", n_pkt, iso_seq_n * samp_x_iso_Intrvl);
			n_pkt = iso_seq_n * samp_x_iso_Intrvl;
			counter_cancel_channel_alarm(hw_timer_dev, 0);
			alarm_cfg0.ticks = now_ticks + ts_resync;
			counter_set_channel_alarm(hw_timer_dev, 0, &alarm_cfg0);
			temp_k = 0;
		}
	}
	// LOG_INF("%u %u %u", iso_seq_n, info->seq_num, temp_k);
}

static void iso_connected(struct bt_iso_chan *chan)
{
    int err;
    struct bt_iso_info iso_info;

    // Obtener información detallada del canal ISO activo
    err = bt_iso_chan_get_info(chan, &iso_info);
    if (err) LOG_ERR("No se pudo obtener info del canal ISO (err %d)", err);
    else LOG_INF("ISO C[%p] connected, BIS Interval %u us", chan, iso_info.iso_interval * 1250);

	k_sem_give(&sem_big_sync);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	LOG_INF("ISO C[%p] disconnected 0x%02x", chan, reason);
	if (reason != BT_HCI_ERR_OP_CANCELLED_BY_HOST) {
		// Cancel the timer
		counter_cancel_channel_alarm(hw_timer_dev, 0);
		sync_ok = false;
		k_sem_give(&sem_sync_lost);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.recv			= iso_recv,
	.connected		= iso_connected,
	.disconnected	= iso_disconnected,
};

static struct bt_iso_chan_io_qos iso_rx_qos[BIS_ISO_CHAN_COUNT];
static struct bt_iso_chan_qos bis_iso_qos[] = {{ .rx = &iso_rx_qos[0], },};
static struct bt_iso_chan bis_iso_chan[] = {{ .ops = &iso_ops, .qos = &bis_iso_qos[0], },};
static struct bt_iso_chan *bis[] = {&bis_iso_chan[0],};

static struct bt_iso_big_sync_param big_sync_param = {
	.bis_channels = bis,
	.num_bis = 1,
	.bis_bitfield = BIT(0),
	.mse = BT_ISO_SYNC_MSE_ANY,
	.sync_timeout = 100,
};

static void reset_semaphores(void)
{
	k_sem_reset(&sem_per_adv);
	k_sem_reset(&sem_per_sync);
	k_sem_reset(&sem_per_big_info);
	k_sem_reset(&sem_big_sync);
	k_sem_reset(&sem_sync_timer);
	k_sem_reset(&sem_sync_lost);
}

//******************************************************************************************************** */
// Callback function for receiving data from the Write characteristic
static void app_command_cb(const uint8_t *buf, uint16_t len)
{
	if (len == 0) return;

	LOG_INF("Rx CMD[%02X:%uB]", buf[0], len);
	uint8_t cmd;
	cmd = buf[0];

	// Eneable-disable the tx characteristic
	if (cmd == 0x01 && len == 2) {
		if (buf[1] == 0x01) tx_notify = true;
		else tx_notify = false;
		LOG_INF("Notify %s", tx_notify ? "ON" : "OFF");
	}

	if (cmd == 0x02 && len == 2) {
		if (buf[1] == 0x01) tx_indicate = true;
		else tx_indicate = false;
		LOG_INF("Indicate %s", tx_indicate ? "ON" : "OFF");
	}

	// Star-Stop Tx
	if (cmd == 0x03 && len == 2) {
		if (buf[1] == 0x01) {
			ble_tx = true;
			first_cycle = true;
			counter_tx = 0;
		} else ble_tx = false;
		LOG_INF("%s", ble_tx ? "Start" : "Stop");
	}

	// Stop pkt
	if (cmd == 0x04 && len == 5) {
		stop_pkt = sys_get_le32(&buf[1]);
		LOG_INF("Stop-TxPkt = %u", stop_pkt);
	}
}

static struct my_imus_cb app_callbacks = {.command_write_cb = app_command_cb,};

// MAIN FUNCTION ********************************************************************
int main(void)
{
	// Pasamos muestro "archivo" de movimiento de int16 -> arreglo uint8
	uint16_t valor = 0;
	int index = 0;
	for (int i = 0; i < 312; i++) {
		valor = sys_cpu_to_le16(motion_data[i]);
		index = i * 2;
		memcpy(&motion_pkt[index], &valor, 2);
	}

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
	gpio_pin_set_dt(&red_led, 1);
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

	if (!bno055_init(i2c1_dev, BNO_OPR_MODE_IMU)) {
		LOG_ERR("BNO055 init failed");
		gpio_pin_set_dt(&red_led, 1);
		// return -1;
	} else {
		bno_ok = true;
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
	k_work_queue_start(&sensor_work_q, sensor_work_q_stack,
		K_KERNEL_STACK_SIZEOF(sensor_work_q_stack),
		K_PRIO_COOP(GETSAMPLE_PRIORITY), NULL);
	k_work_init(&sensor_work, sensor_work_handler);

	// Hilo de envío BLE (Thread)
	send_thread_tid = k_thread_create(&send_thread_data, send_thread_stack, K_KERNEL_STACK_SIZEOF(send_thread_stack),
		send_data_thread, NULL, NULL, NULL, SEND_PRIORITY, 0, K_NO_WAIT);

	if (!send_thread_tid) {
		LOG_ERR("Failed to create send_data_thread");
		return -1;
	}
	k_thread_name_set(send_thread_tid, "ble_send_thread");
	
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

	// Get the Bluetooth device address
	bt_addr_le_t addr;
	size_t count = 1;
	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_id_get(&addr, &count);
	bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
	LOG_INF("%s - %s [CEL=%dus]", CONFIG_BT_DEVICE_NAME, addr_str, CONFIG_BT_CTLR_SDC_MAX_CONN_EVENT_LEN_DEFAULT);

	//================================================================================
	//                                  TIMER
	//================================================================================
	const struct device *clock = DEVICE_DT_GET_ONE(nordic_nrf_clock);
    if (!device_is_ready(clock)) {
        LOG_ERR("clock not ready");
        return 0;
	}
    // Request HFXO (High-Frequency Crystal Oscillator)
	clock_control_on(clock, CLOCK_CONTROL_NRF_SUBSYS_HF);
	k_msleep(5);
    if (!nrf_clock_hf_is_running(NRF_CLOCK, NRF_CLOCK_HFCLK_HIGH_ACCURACY)) LOG_WRN("HFXO not set");
    // Force constant latency mode
    nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_CONSTLAT);

	if (!device_is_ready(hw_timer_dev)) {
        LOG_ERR("El dispositivo counter (TIMER) no está listo.\n");
        return 0;
    }
	counter_start(hw_timer_dev);

	ts_us = (1000000 / fs_hz);
	ts_ticks = counter_us_to_ticks(hw_timer_dev, ts_us);
	ts_first_tick = counter_us_to_ticks(hw_timer_dev, ts_us + delta_us);
	ts_resync = counter_us_to_ticks(hw_timer_dev, delta_us);

	alarm_cfg0 = (struct counter_alarm_cfg){
		.flags = COUNTER_ALARM_CFG_ABSOLUTE,
		.ticks = ts_first_tick,
		.callback = mi_alarma_hw_callback_0,
		.user_data = NULL
	};

	//================================================================================
	//                                BLE ISO
	//================================================================================
	struct bt_le_per_adv_sync_param sync_create_param;
	uint32_t sem_timeout_us;

	// Scan & Periodic Advertising - callbacks
	bt_le_scan_cb_register(&scan_callbacks);
	bt_le_per_adv_sync_cb_register(&sync_callbacks);

	while (true) {
		// Reset all
		gpio_pin_set_dt(&red_led, 1);
		k_msleep(500);
		gpio_pin_set_dt(&red_led, 0);
		reset_semaphores();
		per_adv_lost = false;
		per_adv_found = false;

		// Start scanning
		err = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
		if (err) {
			LOG_ERR("Star Scan err %d\n", err);
			return 0;
		}
		// Wait until finding a PA
		k_sem_take(&sem_per_adv, K_FOREVER);

		// Then Stop Scan & Create a Periodic Advertising Sync
		bt_le_scan_stop();
		bt_addr_le_copy(&sync_create_param.addr, &per_addr);
		sync_create_param.options = 0;
		sync_create_param.sid = per_sid;
		sync_create_param.skip = 0;
		sync_create_param.timeout = (per_interval_us * PA_RETRY_COUNT) / (10 * USEC_PER_MSEC);

		err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
		if (err) {
			LOG_ERR("PA-Sync err %d\n", err);
			return 0;
		}

		// Wait PA-Sync (sync_cb)
		sem_timeout_us = per_interval_us * PA_RETRY_COUNT;
		err = k_sem_take(&sem_per_sync, K_USEC(sem_timeout_us));
		if (err) {
			LOG_ERR("PA-Sync failed err %d\n", err);
			err = bt_le_per_adv_sync_delete(sync);
			if (err) return 0;
			continue;
		}

		// Wait BIG-inf (biginfo_cb)
		err = k_sem_take(&sem_per_big_info, K_USEC(sem_timeout_us));
		if (err) {
			LOG_ERR("BIG-inf err %d\n", err);
			if (per_adv_lost) continue;
			err = bt_le_per_adv_sync_delete(sync);
			if (err) return 0;
			continue;
		}

		// Create BIG Sync
		err = bt_iso_big_sync(sync, &big_sync_param, &big);
		if (err) {
			LOG_ERR("BIG-Create err %d\n", err);
			return 0;
		}

		// Waiting for BIG sync (ISO_Connected)
		err = k_sem_take(&sem_big_sync, K_SECONDS(10));
		if (err == 0) {
			LOG_INF("BIG-Connect Successful.\n");
			bt_le_per_adv_sync_delete(sync); // End PA Sync

			// Waiting sync the timer
			err = k_sem_take(&sem_sync_timer, K_SECONDS(10));
			if (err == 0 && sync_ok) {
				// Start the ACL connection process
				advertising_start();
				// And wait for disconnection to restart the process
				k_sem_take(&sem_sync_lost, K_FOREVER);
			} else LOG_ERR("Sync Timer/Sensor timeout");
		} else LOG_ERR("BIG-Connect timeout\n");

		sync_ok = false;
		// Terminte the BIG to restart the process
		bt_iso_big_terminate(big);
	}
}