/*
- Uso de timer de hw
*/
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/counter.h>
#include <hal/nrf_clock.h>
#include <hal/nrf_timer.h>

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

// SENSOR
#define I2C_NODE DT_NODELABEL(i2c1)
static const struct device *i2c1_dev = DEVICE_DT_GET(I2C_NODE);
static uint8_t units; // Store the unit configuration of BNO

// Variables to sync and control the content of the pkt
static int fs_hz, ts_us = 50000, start_delay_ms, sync_delay_us = 0;
static uint32_t n_sample = 0, k_stop = 10, counter_tx = 0;
static uint8_t temporal_sample_counter = 0, n_pkt = 0, samples_x_pkt = 3;

static bool ble_tx = false, k_stop_tx = true, k_stop_ti = false, first_cycle = false;
static bool tx_notify = true, tx_indicate = false, subscribe = false;

// static uint32_t ts = 1000000 / 30;
uint32_t delay_us = 0;

// Estructura para muestra -> paquete a transmitir
// Total bytes for 2 x (A + G + M + Q) + NoPkt = 56
// Total bytes for 2 x (A + G + Q) + NoPkt =     44
# define PACKET_SIZE 64
# define SAMPLE_SIZE 20
# define N_PKT_SIZE  1
static uint8_t last_packet[PACKET_SIZE];

/*** Recursos del Kernel para el manejo del sensor ***/
// 1. Temporizador
static struct k_timer sensor_timer;
static struct k_timer sensor_timer_off;

// 2. Cola de trabajo dedicada para leer el sensor fuera de la ISR
// static K_KERNEL_STACK_DEFINE(sensor_work_q_stack, GETSAMPLE_STACKSIZE);
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

// HARDWARE TIMER ================================================================
const struct device *hw_timer_dev = DEVICE_DT_GET(DT_NODELABEL(timer1));
static uint32_t period_ticks;

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
static void sensor_work_handler(struct k_work *work)
{
	static uint8_t packet_in_progress[PACKET_SIZE] = {0};
	static uint8_t sample_taken[SAMPLE_SIZE] = {0}, indx_pos;

	// Opcion 1: Tomar muestra (ESTO SE HACE PRIMERO!!!)
	// Guardamos datos quat + acc + gyr
	// bno055_read_raw_sensor_data(quat, 8, sample_taken, 0);
	// bno055_read_raw_sensor_data(acc, 12, sample_taken, 8);

	// En caso de reinicio -> reset a packet_in_progress (para no enviar data mezclada)
	if (first_cycle) {
		memset(packet_in_progress, 0, sizeof(packet_in_progress));
		idx = 0;
		first_cycle = false;
	}

	// Opcion 2: Movimiento guardado
	// memcpy(sample_taken, &motion_pkt[idx], 8);
	// idx = idx + 8;
	// if (idx >= 624) idx = 0;

	// Opcion 3: Comentar opcion 1 & 2 y enviar n_pkt + 0s

	// Acomodamos la muestra segun el contador temporal
	// Desplazamiento de 1B para agregar un contador de pkts (de 1B) al inicio del pkt
	indx_pos = temporal_sample_counter * SAMPLE_SIZE + N_PKT_SIZE;
	memcpy(&packet_in_progress[indx_pos], sample_taken, sizeof(sample_taken));

	// Si ya es hora de enviar el paquete
	if (!(n_sample % samples_x_pkt)) {
		// gpio_pin_set_dt(&blu_led, 1);
		// Agregamos el numero de pkt al inicio
		memcpy(packet_in_progress, &n_pkt, sizeof(n_pkt));

		// LÓGICA DEL MAILBOX
		// Bloquear el mutex para acceder de forma segura al mailbox
		k_mutex_lock(&packet_mutex, K_FOREVER);
		// Sobrescribir el mailbox con el paquete más reciente
		memcpy(last_packet, &packet_in_progress, sizeof(packet_in_progress));
		// Desbloquear el mutex
		k_mutex_unlock(&packet_mutex);
		// Señalizar al hilo BLE que hay nuevos datos listos
		k_sem_give(&ble_data_sem);

		n_pkt++;
		if (ble_tx && k_stop > 0) counter_tx++;
	}

	// Ajuste a contadores
	n_sample++;
	temporal_sample_counter++;
	if (temporal_sample_counter == samples_x_pkt) temporal_sample_counter = 0;
	// if (temporal_sample_counter == 1) gpio_pin_set_dt(&blu_led, 0);

	// STOP
	if (k_stop > 0 && k_stop == counter_tx + 1) {
		if (k_stop_tx) ble_tx = false;
		if (k_stop_ti) k_timer_stop(&sensor_timer);
	}
}

// ISR del temporizador: solo envía el trabajo a la cola
static void timer_expiry_function(struct k_timer *timer_id)
{	
	gpio_pin_set_dt(&blu_led, 1);
	// k_work_submit_to_queue(&sensor_work_q, &sensor_work);
}

static void timer_expiry_function_off(struct k_timer *timer_id)
{gpio_pin_set_dt(&blu_led, 0);}


void mi_alarma_hw_callback_0(const struct device *dev, uint8_t chan_id, 
                           uint32_t ticks, void *user_data)
{
	gpio_pin_set_dt(&blu_led, 1);
	// gpio_pin_toggle_dt(&blu_led);
    // Re-armamos la alarma para el próximo periodo */
    struct counter_alarm_cfg new_alarm_cfg = {
        .flags = COUNTER_ALARM_CFG_ABSOLUTE,	// Tiempo absoluto!!!
        .ticks = ticks + period_ticks,			// ISR en 'tick' actual + periodo (EVITAMOS DRIFT)
        .callback = mi_alarma_hw_callback_0,
        .user_data = user_data
    };
    // Re-configuramos la alarma.
    counter_set_channel_alarm(dev, chan_id, &new_alarm_cfg);
}

void mi_alarma_hw_callback_1(const struct device *dev, uint8_t chan_id, 
                           uint32_t ticks, void *user_data)
{
	gpio_pin_set_dt(&blu_led, 0);
    // Re-armamos la alarma para el próximo periodo */
    struct counter_alarm_cfg new_alarm_cfg = {
        .flags = COUNTER_ALARM_CFG_ABSOLUTE,	// Tiempo absoluto!!!
        .ticks = ticks + period_ticks,			// ISR en 'tick' actual + periodo (EVITAMOS DRIFT)
        .callback = mi_alarma_hw_callback_1,
        .user_data = user_data
    };
    // Re-configuramos la alarma.
    counter_set_channel_alarm(dev, chan_id, &new_alarm_cfg);
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
        if (ble_tx && tx_notify) {
			err = my_imus_send_sensordata(packet_to_send, sizeof(packet_to_send));
			LOG_INF("N-Pkt[%u]", packet_to_send[0]);
			if (err) LOG_WRN("Fallo notificacion (err %d)", err);
        }

		if (ble_tx && tx_indicate){
			err = my_imus_send_exercisedetection(packet_to_send, sizeof(packet_to_send));
			LOG_INF("I-Pkt[%u]", packet_to_send[0]);
			if (err) LOG_WRN("Fallo indicacion (err %d)", err);
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
}

//Callback function for announcing the disconnection
static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)\n", reason);
	bt_conn_unref(my_conn);

	// Reset the connected flag
	connect = false;
	subscribe = false;

	counter_cancel_channel_alarm(hw_timer_dev, 0);
	counter_stop(hw_timer_dev);
	gpio_pin_set_dt(&blu_led, 0);

	// k_timer_stop(&sensor_timer);			// Detenemos el timer
	// k_timer_stop(&sensor_timer_off);		// Detenemos el timer
	// k_sem_take(&ble_data_sem, K_NO_WAIT); 	// Tomamos el turno
	// ble_tx = false;							// Deshabilitamos la transmision
}

// Callback function for when a connection object is recycled (restart advertising)
static void recycled_cb(void)
{
	LOG_INF("Disconnect complete. Restart advertising\n");
	advertising_start();
}

// Callback para las solicitudes de cambios en los parametros de conexion
static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	LOG_INF("Connection parameters update requested by central.");
	// Simplemente acepta la solicitud.
	return true; 
}

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
	.recycled				= recycled_cb,
	.le_param_req			= le_param_req,
	.le_param_updated		= on_le_param_updated,		// Uso opcional solo informan
	.le_phy_updated			= on_le_phy_updated,		// Uso opcional solo informan
	.le_data_len_updated    = on_le_data_len_updated,	// Uso opcional solo informan
};

// Callback function for receiving data from the Write characteristic
static void app_command_cb(const uint8_t *buf, uint16_t len)
{

	// Sync Start
	if (buf[0] == 0x24) {
		delay_us = sys_get_le32(&buf[1]);

		sync_delay_us = counter_us_to_ticks(hw_timer_dev, delay_us);
		// Configurar la PRIMERA alarma
		struct counter_alarm_cfg alarm_cfg0 = {
			.flags = 0,
			.ticks = sync_delay_us,
			.callback = mi_alarma_hw_callback_0,
			.user_data = NULL
		};
		// Establecer la alarma en el canal 0
		int err = counter_set_channel_alarm(hw_timer_dev, 0, &alarm_cfg0);
		if (err != 0) printk("Error %d - Config alarm channel counter 0", err);

		delay_us += 10000;
		sync_delay_us = counter_us_to_ticks(hw_timer_dev, delay_us);
		// Configurar la SEGUNDA alarma
		struct counter_alarm_cfg alarm_cfg1 = {
			.flags = 0,
			.ticks = sync_delay_us,
			.callback = mi_alarma_hw_callback_1,
			.user_data = NULL
		};
		// Establecer la alarma en el canal 1
		err = counter_set_channel_alarm(hw_timer_dev, 1, &alarm_cfg1);
		if (err != 0) printk("Error %d - Config alarm channel counter 1", err);

		err = counter_start(hw_timer_dev);
		if (err != 0) printk("Error %d - Start counter", err);
		LOG_INF("%u", delay_us);

		// // Calculamos los ticks del periodo UNA SOLA VEZ
		// // period_us = 1000000;
		// period_ticks = counter_us_to_ticks(hw_timer_dev, 1000000);
		// // Configurar la PRIMERA alarma
		// struct counter_alarm_cfg alarm_cfg = {
		//     .flags = 0,
		//     .ticks = period_ticks,
		//     .callback = mi_alarma_hw_callback,
		//     .user_data = NULL
		// };
		// // Establecer la alarma en el canal 0
		// err = counter_set_channel_alarm(hw_timer_dev, 1, &alarm_cfg);
		// if (err != 0) {
		//     printk("Error: No se pudo configurar la alarma del counter %d\n", err);
		//     return 0;
		// }
		// //. Iniciar el contador de hardware */
		// counter_start(hw_timer_dev);
		// printk("Timer periódico iniciado. Ticks por periodo: %u\n", period_ticks);
	}

	// // Sync Start
	// if (buf[0] == 0x24 && len == 12) {

	// 	uint16_t hz_1 = sys_get_le16(&buf[1]);
	// 	uint32_t delay_us = sys_get_le32(&buf[3]);
	// 	static uint32_t led_on_us = 8000;
	// 	uint8_t sxp_1 = buf[7];
	// 	uint32_t np = 0;

	// 	//1 Fs & ts
	// 	if (hz_1 > 0) {
	// 		fs_hz = (int)hz_1;
	// 		ts_us = 1000000 / fs_hz;
	// 	}
	// 	// 2. Delay us & Start the timer
	// 	sync_delay_us = (int)delay_us;
	// 	k_timer_start(&sensor_timer, K_USEC(sync_delay_us), K_USEC(ts_us));
	// 	k_timer_start(&sensor_timer_off, K_USEC(sync_delay_us + led_on_us), K_USEC(ts_us));

	// 	// 3. Samples x pkt
	// 	if (sxp_1 > 0) samples_x_pkt = sxp_1;
	// 	// 4. Number of sample
	// 	n_sample = sys_get_le32(&buf[8]);
		
	// 	// Calculo de n_pkt & temp sample counter
	// 	if (n_sample == 1) {
	// 		// Somos el periferico de referencia P0!!!
	// 		n_pkt = 0;
	// 		n_sample = 1;
	// 		temporal_sample_counter = 0;
	// 	} else {
	// 		// Ha sincronizarnos con P0!!!
	// 		np = (n_sample - 1) / samples_x_pkt;	// Valor truncado
	// 		n_pkt = (uint8_t)np;					// Number of pkt (uint8)
	// 		// temporal sample counter -> index of the sample to take in the pkt
	// 		temporal_sample_counter = n_sample - np * samples_x_pkt - 1;
	// 	}

	// 	LOG_INF("Fs %dHz (%dus), Sync delay %dus, SXP %u, SS %u, n_pkt %u-%u, tsc %u", 
	// 		fs_hz, ts_us, sync_delay_us, samples_x_pkt, n_sample, np, n_pkt, temporal_sample_counter);
	// }

	// // Stop pkt
	// if (buf[0] == 0x26 && len == 8) {
	// 	if (buf[1] == 0x00) {
	// 		// Disable the stop pkt
	// 		k_stop = 0;
	// 		LOG_INF("k-Stop OFF");
	// 	} else {
	// 		k_stop = sys_get_le32(&buf[2]);

	// 		// Stop ble_tx when we reach k_stop?
	// 		if (buf[6] == 0x01) k_stop_tx = true;
	// 		else k_stop_tx = false;
	// 		// Stop timer when we reach k_stop?
	// 		if (buf[7] == 0x01) k_stop_ti = true;
	// 		else k_stop_ti = false;

	// 		// Stop now & disable 
	// 		if (k_stop == 0) {
	// 			if (k_stop_tx) ble_tx = false;
	// 			if (k_stop_ti) k_timer_stop(&sensor_timer);
	// 		} else counter_tx = 0;
			
	// 		LOG_INF("k-Stop=%u, Tx=%s, Timer=%s", k_stop, k_stop_tx?"ON":"OFF", k_stop_ti?"ON":"OFF");
	// 	}
	// }

	// // Star Tx
	// if (buf[0] == 0x27) {
	// 	ble_tx = true;
	// 	counter_tx = 0;
	// 	LOG_INF("Star");
	// }

	// First-Write
	if (buf[0] == 0x28) LOG_INF("First Write");

	// Stop
	// if (buf[0] == 0x29) {
	// 	k_timer_stop(&sensor_timer);			// Detenemos el timer
	// 	k_timer_stop(&sensor_timer_off);			// Detenemos el timer
	// 	k_sem_take(&ble_data_sem, K_NO_WAIT); 	// Tomamos el turno
	// 	ble_tx = false;							// Deshabilitamos la transmision
	// }

	// // Eneable-disable the tx characteristic
	// // Notify
	// if (buf[0] == 0x01) tx_notify = true;
	// if (buf[0] == 0x02) tx_notify = false;
	// if (buf[0] == 0x01 || buf[0] == 0x02) LOG_INF("Notify %s", tx_notify ? "ON" : "OFF");
	// // Indicate
	// if (buf[0] == 0x03) tx_indicate = true;
	// if (buf[0] == 0x04) tx_indicate = false;
	// if (buf[0] == 0x03 || buf[0] == 0x04) LOG_INF("Indicate %s", tx_indicate ? "ON" : "OFF");

	// LOG_INF("Rx CMD[%02X:%uB]", buf[0], len);
}

static struct my_imus_cb app_callbacks = {
	.command_write_cb = app_command_cb,	// Exercise detection callback
};

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
	// if (!device_is_ready(i2c1_dev)) {
	// 	LOG_ERR("I2C-1 not ready");
	// 	return -1;
	// }

	// LOG_INF("Starting IMU-Node. Hardware OK.");

	// if (!bno055_init(i2c1_dev, BNO_OPR_MODE_NDOF)) {
	// 	LOG_ERR("BNO055 init failed.");
	// 	gpio_pin_set_dt(&red_led, 1);
	// 	return -1;
	// } else {
	// 	LOG_INF("BNO055 init OK.");
	// 	bno055_change_crystal(true);
	// 	uint8_t config_values[5];
	// 	bno055_read_config(config_values, false);
	// 	units = config_values[3];
	// }

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
	// k_timer_init(&sensor_timer, timer_expiry_function, NULL);
	// k_timer_init(&sensor_timer_off, timer_expiry_function_off, NULL);
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
	LOG_INF("Periferico Name=%s, Address=%s, CEL=%dus", CONFIG_BT_DEVICE_NAME, addr_str, CONFIG_BT_CTLR_SDC_MAX_CONN_EVENT_LEN_DEFAULT);

	//================================================================================
	//                                  TIMER
	//================================================================================
	if (!device_is_ready(hw_timer_dev)) {
        printk("Error: El dispositivo counter (TIMER0) no está listo.\n");
        return 0;
    }

	period_ticks = counter_us_to_ticks(hw_timer_dev, ts_us);

	//================================================================================
	//                                LED Blink
	//================================================================================
	uint8_t k = 0;
	gpio_pin_set_dt(&red_led, 0);
	for (;;) {
		k_msleep(100);
		// if (!(k % 20)) {
		// 	if (connect) {
		// 		gpio_pin_toggle_dt(&blu_led);
		// 		gpio_pin_set_dt(&grn_led, 0);
		// 		k_msleep(50);
		// 		gpio_pin_toggle_dt(&blu_led);
		// 	} else {
		// 		gpio_pin_toggle_dt(&grn_led);
		// 		gpio_pin_set_dt(&blu_led, 0);
		// 		k_msleep(50);
		// 		gpio_pin_toggle_dt(&grn_led);
		// 	}
		// }
		// k++;

		// Check if the central is already subscribed
		if (!subscribe) {
			if (is_sensordata_notify_enabled() && is_exercisedetection_indicate_enabled()) {
				gpio_pin_set_dt(&blu_led, 1);
				k_msleep(500);
				subscribe = true;
				my_imus_send_exercisedetection(&(const uint8_t){0x00}, 1);
				LOG_INF("End warm-up time -> Start Sync");
				gpio_pin_set_dt(&blu_led, 0);
				// k_msleep(1000);
				// bt_conn_disconnect(my_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			}
		}
	}
}