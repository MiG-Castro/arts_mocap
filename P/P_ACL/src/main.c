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

// Address of the Central to connect
static const bt_addr_le_t target_addr = {
	// Address list in big endian
	// Example FB:5C:2A:77:77:0A in little endian = 0A:77:77:2A:5C:FB big endian
    .type = BT_ADDR_LE_RANDOM,
	.a = {.val = {0x0A, 0x77, 0x77, 0x2A, 0x5C, 0xFB}}
	// .a = {.val = {0xC3, 0xFB, 0xF6, 0xBA, 0xC4, 0xCD}}
	// .a = {.val = {0x2C, 0xEB, 0x6B, 0x4F, 0x8A, 0xF3}}
};
// Advertising parameters structure
static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	BT_LE_ADV_OPT_CONN |			// Connectable advertising
	BT_LE_ADV_OPT_USE_IDENTITY |	// Use identity address (Static random address)
	BT_LE_ADV_OPT_FILTER_CONN, 		// Eneable Whitelist
	BT_GAP_ADV_FAST_INT_MIN_1,		// Min Advertising Interval = Value x 0.625ms = 30ms
	BT_GAP_ADV_FAST_INT_MAX_1,		// Max Advertising Interval = Value x 0.625ms = 60ms
	NULL							// Address of peer, Set to NULL for undirected advertising
);
// Definition of advertising data
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL |	// General advertising
		BT_LE_AD_NO_BREDR)),							// No compatible with BR/EDR (Bluetooth Classic)
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN), // Use complete name
};
// Scan Response -> UUID of Service
static const struct bt_data sd[] = {BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_IMUS_VAL),};
// Work item for advertising & connection setup
static struct k_work adv_work;

//================================================================================
//                              Sensor & Sampling
//================================================================================
// BLE packet transmission thread configuration
#define SEND_STACKSIZE		4096
#define SEND_PRIORITY		8
// Sampling & packaging thread configuration
#define GETSAMPLE_STACKSIZE	4096
#define GETSAMPLE_PRIORITY	4

// SENSOR ========================================================================
#define I2C_NODE DT_NODELABEL(i2c1)
static const struct device *i2c1_dev = DEVICE_DT_GET(I2C_NODE);
static bool bno_ok = false;

// Sampling & packaging configuration
static uint64_t ts_us;
static uint32_t ts_ticks, fs_hz = 50;
static uint32_t n_pkt = 0, stop_pkt, counter_tx;
static uint8_t units, samples_x_pkt = 2, pkt_size;
// pkt to Tx
# define MAX_PACKET_SIZE 255
# define SAMPLE_SIZE 26   // Q=8B, Acc=6B, Mag=6B, G=6B
static uint8_t last_packet[MAX_PACKET_SIZE] = {0};
// control variables
static bool ble_tx = false, first_cycle = false;
static bool tx_notify = true, tx_indicate = false;

// 1. HARDWARE TIMER ========================================================
const struct device *hw_timer_dev = DEVICE_DT_GET(DT_NODELABEL(timer2));
static struct counter_alarm_cfg alarm_cfg0; // Alarm settings for timer
// 2. Dedicated workqueue for sensor reading outside ISR
static struct k_work_q sensor_work_q;
static struct k_work sensor_work;
K_KERNEL_STACK_MEMBER(sensor_work_q_stack, GETSAMPLE_STACKSIZE); // Work queue stack
// 3. Mailbox
static struct k_mutex packet_mutex;		// Mutex
static struct k_sem ble_data_sem;		// Semaphore

// BLE Send Thread
K_KERNEL_STACK_MEMBER(send_thread_stack, SEND_STACKSIZE); // TX thread stack
static struct k_thread send_thread_data; // Thread data structure
k_tid_t send_thread_tid;                 // Thread ID

// Stored movement data (quaternions) ============================================
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
//                     SAMPLING & TRANSMISSION FUNCTIONS
//================================================================================
// Read sensor and packetize data
static void sensor_work_handler(struct k_work *work)
{
	static uint8_t sample_taken[32] = {0}, packet_in_progress[MAX_PACKET_SIZE] = {0};
	static uint8_t indx_pos, temporal_sample_counter;

	/* BNO055 Register Map
	- acc  = BNO_REG_ACC_X_LSB   0x08 Acc
	- mag  = BNO_REG_MAG_X_LSB   0x0E Mag
	- gry  = BNO_REG_GYR_X_LSB   0x14 Gyr
	- eul  = BNO_REG_EUL_H_LSB   0X1A Euler angles (Heading/Roll/Pitch)
	- quat = BNO_REG_QUAT_W_LSB  0X20 Quaterniones (WXYZ)
	- lnAc = BNO_REG_LIA_X_LSB   0X28 lineal acc
	- grav = BNO_REG_GRV_X_LSB   0X2E gravity
	- temp = BNO_REG_TEMP        0X34 Temperature
	DEFAULT REGISTER VALUE = 0x80 = 10000000 = m/s2, dps, degrees, °C, Android
	*/

	// Read: Accelerometer + Magnetometer + Gyroscope + EA (not used) + Quat
	if (bno_ok) {
		bno055_read_raw_sensor_data(acc, 32, sample_taken, 0);
		// Reorder
		uint8_t temp_quat[8];
		memcpy(temp_quat, &sample_taken[24], 8);
		memmove(&sample_taken[8], &sample_taken[0], 24);
		memcpy(&sample_taken[0], temp_quat, 8);
	}

	// On startup
	if (first_cycle) {
		pkt_size = SAMPLE_SIZE * samples_x_pkt + 4;
		printk("%u, %u\n", pkt_size, samples_x_pkt);
		memset(packet_in_progress, 0, sizeof(packet_in_progress));
		n_pkt = 0;
		idx = 0;
		temporal_sample_counter = 0;
		first_cycle = false;
	}

	// Alternative -> Send stored data (quaternions)
	if (!bno_ok) {
		memcpy(sample_taken, &motion_pkt[idx], 8);
		idx = idx + 8;
		if (idx >= 624) idx = 0;
	}

	// Align sample according to the temporal counter
	// 4B offset to prepend Packet Number (NoPkts)
	indx_pos = temporal_sample_counter * SAMPLE_SIZE + 4;
	memcpy(&packet_in_progress[indx_pos], sample_taken, SAMPLE_SIZE);

	// Samples per packet limit reached
	if (((temporal_sample_counter + 1) % samples_x_pkt) == 0) {
		// add the NoPkt
		memcpy(packet_in_progress, &n_pkt, 4);

		// MAILBOX
		k_mutex_lock(&packet_mutex, K_FOREVER);
		memcpy(last_packet, &packet_in_progress, sizeof(packet_in_progress));
		k_mutex_unlock(&packet_mutex);
		// Pkt ready -> Semaphore give
		k_sem_give(&ble_data_sem);

		n_pkt++;
	}

	// Update counters
	temporal_sample_counter++;
	if (temporal_sample_counter == samples_x_pkt) temporal_sample_counter = 0;
}

// HW Timer ISR 
void mi_alarma_hw_callback_0(const struct device *dev, uint8_t chan_id, uint32_t ticks, void *user_data)
{
	// Rearm alarm for next period
	alarm_cfg0.ticks = ticks + ts_ticks;
	counter_set_channel_alarm(dev, chan_id, &alarm_cfg0);
	// Capture sample
	k_work_submit_to_queue(&sensor_work_q, &sensor_work);
}

// Thread for sending data periodically
void send_data_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

    uint8_t packet_to_send[MAX_PACKET_SIZE] = {0};
	uint32_t num = 0;
	int err;

    while (1) {
        // Wait for pkts
        k_sem_take(&ble_data_sem, K_FOREVER);
        // Copy latest packet from mailbox to local buffer
        k_mutex_lock(&packet_mutex, K_FOREVER);
		memcpy(packet_to_send, &last_packet, sizeof(last_packet));
        k_mutex_unlock(&packet_mutex);

		// If connected, subscribed, and TX command received
		if (ble_tx) {
			memcpy(&num, packet_to_send, sizeof(num));
			if (tx_notify) {
				err = my_imus_send_sensordata(packet_to_send, pkt_size);
				LOG_INF("N[%u][%uB]", num, pkt_size);
				if (err) LOG_WRN("Notification failed (err %d)", err);
			}
			if (tx_indicate){
				err = my_imus_send_exercisedetection(packet_to_send, pkt_size);
				LOG_INF("I[%u][%uB]", num, pkt_size);
				if (err) LOG_WRN("Indication failed (err %d)", err);
			}

			// If stop packet function is active
			if (stop_pkt > 0) {
				counter_tx++;
				if (counter_tx == stop_pkt) ble_tx = false;
			}
		}
    }
}

//================================================================================
//                            BLE FUNCTIONS
//================================================================================
// Start the advertisign in the work-queue
static void adv_work_handler(struct k_work *work)
{
	int err;
	// Address filter
	err = bt_le_filter_accept_list_add(&target_addr);
    if (err) LOG_WRN("whitelist err %d", err);
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
	LOG_INF("- Connection parameters: interval %.2f ms, latency %d (CI), timeout %d ms",
			connection_interval, info.le.latency, supervision_timeout);

	// Data Length parameters to the log
	LOG_INF("- Data length: TX %d bytes / %d us, RX %d bytes / %d us",
            info.le.data_len->tx_max_len, info.le.data_len->tx_max_time,
            info.le.data_len->rx_max_len, info.le.data_len->rx_max_time);
}

//Callback function for announcing the disconnection
static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)-----------\n", reason);
	bt_conn_unref(my_conn);

	// Stop time alarm
	counter_cancel_channel_alarm(hw_timer_dev, 0);
	k_sem_take(&ble_data_sem, K_NO_WAIT);
	ble_tx = false; // Tx OFF

	// For reconnection
	k_msleep(200);
	advertising_start();
}

// Callback for connection parameter update requests
static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{LOG_INF("Connection parameters update requested by central.");return true;}

// Callback to inform the connection parameter update
void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	double connection_interval = interval*1.25;         // in ms
	uint16_t supervision_timeout = timeout*10;          // in ms
	LOG_INF("Connection parameters updated: interval %.2f ms, latency %d (CI), timeout %d ms",
		connection_interval, latency, supervision_timeout);
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
	.le_param_req			= le_param_req,
	.le_param_updated		= on_le_param_updated,
	.le_phy_updated			= on_le_phy_updated,
	.le_data_len_updated    = on_le_data_len_updated,
};

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
		uint32_t now_ticks;
		counter_get_value(hw_timer_dev, &now_ticks);
		if (buf[1] == 0x01) {
			ble_tx = true;
			first_cycle = true;
			counter_tx = 0;
			alarm_cfg0.ticks = now_ticks + ts_ticks;
			counter_set_channel_alarm(hw_timer_dev, 0, &alarm_cfg0);
		} else {
			ble_tx = false;
			counter_cancel_channel_alarm(hw_timer_dev, 0);
		}
		LOG_INF("%s", ble_tx ? "Start" : "Stop");
	}

	// Stop pkt
	if (cmd == 0x04 && len == 5) {
		stop_pkt = sys_get_le32(&buf[1]);
		counter_tx = 0;
		LOG_INF("Stop-TxPkt = %u", stop_pkt);
	}

	// Sampling Frecuenci (Fs)
	if (cmd == 0x05 && len == 5) {
		uint32_t n;
		n = sys_get_le32(&buf[1]);
		if (n > 0) {
			fs_hz = n;
			ts_us = (1000000 / fs_hz);
			ts_ticks = counter_us_to_ticks(hw_timer_dev, ts_us);
		}
		LOG_INF("Fs = %u", fs_hz);
	}

	// Samples X pkt
	if (cmd == 0x06 && len == 2) {
		if (buf[1] > 0) {
			samples_x_pkt = buf[1];
			first_cycle = true;
		}
		LOG_INF("Samples X pkt (SXP) = %u", samples_x_pkt);
	}
}

static struct my_imus_cb app_callbacks = {.command_write_cb = app_command_cb,};

// MAIN FUNCTION ********************************************************************
int main(void)
{
	// Pass save motion data (raw quat) to uint8 array
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
	LOG_INF("Starting IMU-Node.");

	if (!bno055_init(i2c1_dev, BNO_OPR_MODE_IMU)) { // BNO_OPR_MODE_IMU - BNO_OPR_MODE_NDOF
		LOG_ERR("BNO055 init failed");
	} else {
		bno_ok = true;
		LOG_INF("BNO055 init OK.");
		bno055_change_crystal(true);
		// optional load the calibrations offsets of a previus calibration
		int16_t bno_cal_offsets[11] = {
			-3, 5, -34, 32325, -738, 10118, -1, -2, 3, 1000, 773}; // FA86 blanco
			// -30, 35, -35, 32469, -905, 9702, -1, -1, 0, 1000, 778}; // FA2F negro
		bno055_calibration_offsets(true, bno_cal_offsets);
		k_msleep(600);
		// Check tha config of BNO
		uint8_t config_values[5];
		bno055_read_config(config_values, false);
		units = config_values[3];
		// BNO Ok -> Turn off the red led
		gpio_pin_set_dt(&red_led, 0);
	}

	//================================================================================
	//                            3. DATA ACQUISITION
	//================================================================================
	// Mailbox: Mutex and Semaphore
	k_mutex_init(&packet_mutex);
	k_sem_init(&ble_data_sem, 0, 1);

	// Dedicated workqueue
	k_work_queue_start(&sensor_work_q, sensor_work_q_stack,
		K_KERNEL_STACK_SIZEOF(sensor_work_q_stack),
		K_PRIO_COOP(GETSAMPLE_PRIORITY), NULL);
	k_work_init(&sensor_work, sensor_work_handler);

	// BLE packet transmission thread
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
	LOG_INF("%s - %s [CEL=%dus]", CONFIG_BT_DEVICE_NAME, 
		addr_str, CONFIG_BT_CTLR_SDC_MAX_CONN_EVENT_LEN_DEFAULT);

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
        LOG_ERR("Timer not ready.\n");
        return 0;
    }
	counter_start(hw_timer_dev);

	ts_us = (1000000 / fs_hz);
	ts_ticks = counter_us_to_ticks(hw_timer_dev, ts_us);
	alarm_cfg0 = (struct counter_alarm_cfg){
		.flags = COUNTER_ALARM_CFG_ABSOLUTE,
		.ticks = ts_ticks,
		.callback = mi_alarma_hw_callback_0,
		.user_data = NULL
	};

	//================================================================================
	k_msleep(500);
	advertising_start();
}