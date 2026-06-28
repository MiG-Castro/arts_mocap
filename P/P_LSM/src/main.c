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
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
struct bt_conn *my_conn = NULL;

// Connection parameters
static uint16_t ci_n = 8, l_n = 3, t_n = 100;
static uint8_t try_update = 0, request_limit = 4;

// Advertising parameters structure
static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY),
	BT_GAP_ADV_FAST_INT_MIN_1, BT_GAP_ADV_FAST_INT_MAX_1, NULL);

// Definition of advertising data
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),};

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

// Address of the sensor
#define LSM6DS3TRC 0x6A
// Principal control registers
#define ADDR_CTRL1_XL 0x10  // Config Acc
#define ADDR_CTRL2_G  0x11  // Config Gyr
#define ADDR_CTRL3_C  0x12  // Config read
// Config values for CTRL3_C
#define BDU_EN 0x40
#define IF_INC_EN 0x04
// Output registers
#define OUT_TEMP_L 0x20
#define OUTX_L_G 0x22
#define OUTY_L_G 0x24
#define OUTZ_L_G 0x26
#define OUTX_L_XL 0x28
#define OUTY_L_XL 0x2A
#define OUTZ_L_XL 0x2C
// Sensor Output data rate
#define ODR_104 0x40 // 0100 0000
#define ODR_416 0x60 // 0110 0000
// Scale of the sensors
#define XL_2G  0x00     // 0000 0000
#define XL_16G 0x04     // 0000 0100
#define XL_4G  0x08     // 0000 1000
#define XL_8G  0x0C     // 0000 1100
#define G_125_DPS  0x02 // 0000 0010
#define G_250_DPS  0x00 // 0000 0000
#define G_500_DPS  0x04 // 0000 0100
#define G_1000_DPS 0x08 // 0000 1000
#define G_2000_DPS 0x0C // 0000 1100

static uint8_t config_gyr = 0, config_acc = 0, config_read = 0;
static const struct device *i2c_lsm = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lsm6ds3tr_c)));

// Sampling ========================================================================
static uint64_t ts_us;
static uint32_t ts_ticks, fs_hz = 30, n_pkt = 0;
// pkt to Tx
# define PACKET_SIZE 16
static uint8_t last_packet[PACKET_SIZE] = {0};
// control variables
static bool ble_tx = false;

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

//================================================================================
//                     SAMPLING & TRANSMISSION FUNCTIONS
//================================================================================
// Read sensor and packetize data
static void sensor_work_handler(struct k_work *work)
{
	static uint8_t packet_in_progress[PACKET_SIZE] = {0};

	/*take the sample*/
	i2c_burst_read(i2c_lsm, LSM6DS3TRC, OUTX_L_G, &packet_in_progress[4], 12);
	memcpy(packet_in_progress, &n_pkt, 4);

	// MAILBOX
	k_mutex_lock(&packet_mutex, K_FOREVER);
	memcpy(last_packet, &packet_in_progress, sizeof(packet_in_progress));
	k_mutex_unlock(&packet_mutex);
	// Pkt ready -> Semaphore give
	k_sem_give(&ble_data_sem);
	n_pkt++;
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

    uint8_t packet_to_send[PACKET_SIZE] = {0};
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
		if (is_sensordata_notify_enabled()) {
			memcpy(&num, packet_to_send, sizeof(num));
			err = my_imus_send_sensordata(packet_to_send, PACKET_SIZE);
			LOG_INF("N[%u][%uB]", num, PACKET_SIZE);
			if (err) LOG_WRN("Notification failed (err %d)", err);
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
	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {LOG_ERR("Advertising failed to start (err %d)\n", err);return;}
	LOG_INF("Advertising successfully started");
}

// Function to start advertising sending the work item to the work-queue
static void advertising_start(void){k_work_submit(&adv_work);}

static void update_conn_p(struct bt_conn *conn)
{
	LOG_INF("Request update of Connection Parameters");
	struct bt_le_conn_param *conn_p = BT_LE_CONN_PARAM(ci_n, ci_n, l_n, t_n);
	int err = bt_conn_le_param_update(conn, conn_p);
	if (err) LOG_ERR("Request CI err %d", err);
}

/************************ Connection events callback functions ************************/
// Callback function for announcing the connection or error in the connection
static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {LOG_ERR("Connection failed (err %u)\n", err);return;}
	LOG_INF("Connected.");

	my_conn = bt_conn_ref(conn);
	struct bt_conn_info info;
	err = bt_conn_get_info(conn, &info);
	if (err) {LOG_ERR("bt_conn_get_info() returned %d", err);return;}

	// Connection parameters to the log
	double ci = info.le.interval * 1.25;	// in ms
	uint16_t timeout = info.le.timeout * 10;	// in ms
	LOG_INF("- CP: interval %.2f ms, latency %d (CI), timeout %d ms", ci, info.le.latency, timeout);

	// Data Length parameters to the log
	LOG_INF("- DL: TX %d bytes / %d us, RX %d bytes / %d us",
            info.le.data_len->tx_max_len, info.le.data_len->tx_max_time,
            info.le.data_len->rx_max_len, info.le.data_len->rx_max_time);
	
	// If connection parameters are not the expected -> Request update
	if (info.le.interval != ci_n || info.le.latency != l_n || info.le.timeout != t_n) update_conn_p(my_conn);
}

//Callback function for announcing the disconnection
static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)\n", reason);
	bt_conn_unref(my_conn);
	// Stop the TX
	k_sem_take(&ble_data_sem, K_NO_WAIT);
	ble_tx = false;
	try_update = 0;
	// Start the adv
	k_msleep(200);
	advertising_start();
}

// Callback for connection parameter update requests
static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{LOG_INF("Connection parameters update requested by central.");return true;}

// Callback to inform the connection parameter update
void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	double ci = interval * 1.25;
	uint16_t s_timeout = timeout * 10;
	LOG_INF("Updated: interval %.2f ms, latency %d (CI), timeout %d ms", ci, latency, s_timeout);
	if ((interval > ci_n || latency > l_n || timeout > t_n) && try_update < request_limit) {
		update_conn_p(my_conn);
		try_update++;
	}
}

// Callback structure for connections events
struct bt_conn_cb connection_callbacks = {
	.connected				= on_connected,
	.disconnected			= on_disconnected,
	.le_param_req			= le_param_req,
	.le_param_updated		= on_le_param_updated,
};

//******************************************************************************************************** */
// Callback function for receiving data from the Write characteristic
static void app_command_cb(const uint8_t *buf, uint16_t len)
{
	if (len == 0) return;

	LOG_INF("Rx CMD[%02X:%uB]", buf[0], len);
	uint8_t cmd;
	cmd = buf[0];

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
}

static struct my_imus_cb app_callbacks = {.command_write_cb = app_command_cb,};

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
	gpio_pin_set_dt(&red_led, 1);
	gpio_pin_set_dt(&grn_led, 0);
	gpio_pin_set_dt(&blu_led, 0);

	//================================================================================
	//                                 2. SENSOR
	//================================================================================
	if (!device_is_ready(i2c_lsm)) {LOG_ERR("I2C-1 not ready");return -1;}
	LOG_INF("Starting IMU-Node.");
	// Config registers
    config_gyr = ODR_104 | G_250_DPS;
    config_acc = ODR_104 | XL_2G;
    config_read = BDU_EN | IF_INC_EN;
    i2c_reg_write_byte(i2c_lsm, LSM6DS3TRC, ADDR_CTRL1_XL, config_acc);
    i2c_reg_write_byte(i2c_lsm, LSM6DS3TRC, ADDR_CTRL2_G, config_gyr);
    i2c_reg_write_byte(i2c_lsm, LSM6DS3TRC, ADDR_CTRL3_C, config_read);

	//================================================================================
	//                            3. DATA ACQUISITION
	//================================================================================
	// Mailbox: Mutex and Semaphore
	k_mutex_init(&packet_mutex);
	k_sem_init(&ble_data_sem, 0, 1);

	// Dedicated workqueue
	k_work_queue_start(&sensor_work_q, sensor_work_q_stack,
		K_KERNEL_STACK_SIZEOF(sensor_work_q_stack), K_PRIO_COOP(GETSAMPLE_PRIORITY), NULL);
	k_work_init(&sensor_work, sensor_work_handler);

	// BLE packet transmission thread
	send_thread_tid = k_thread_create(&send_thread_data,
		send_thread_stack, K_KERNEL_STACK_SIZEOF(send_thread_stack),
		send_data_thread, NULL, NULL, NULL, SEND_PRIORITY, 0, K_NO_WAIT);

	if (!send_thread_tid) {LOG_ERR("Failed to create send_data_thread");return -1;}
	k_thread_name_set(send_thread_tid, "ble_send_thread");
	
	//================================================================================
	//                               4. BLUETOOTH
	//================================================================================
	// Eneable the bluetooth stack
	err = bt_enable(NULL);
	if (err) {LOG_ERR("Bluetooth init failed err %d\n", err);return -1;}

	// Initialize the IMU service with the application callbacks
	err = my_imus_init(&app_callbacks);
	if (err) {LOG_ERR("Fail to init IMU Service err %d\n", err);return -1;}

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
	LOG_INF("%s - %s [CEL=%dus]", CONFIG_BT_DEVICE_NAME, addr_str, 
		CONFIG_BT_CTLR_SDC_MAX_CONN_EVENT_LEN_DEFAULT);

	//================================================================================
	//                                  TIMER
	//================================================================================
	const struct device *clock = DEVICE_DT_GET_ONE(nordic_nrf_clock);
    if (!device_is_ready(clock)) {LOG_ERR("clock not ready");return 0;}
	
    // Request HFXO (High-Frequency Crystal Oscillator)
	clock_control_on(clock, CLOCK_CONTROL_NRF_SUBSYS_HF);
	k_msleep(5);
    if (!nrf_clock_hf_is_running(NRF_CLOCK, NRF_CLOCK_HFCLK_HIGH_ACCURACY)) LOG_WRN("HFXO not set");
    // Force constant latency mode
    nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_CONSTLAT);

	if (!device_is_ready(hw_timer_dev)) {LOG_ERR("Timer not ready.\n");return 0;}
	counter_start(hw_timer_dev);  // Start the hw timer

	ts_us = (1000000 / fs_hz);
	ts_ticks = counter_us_to_ticks(hw_timer_dev, ts_us);
	uint32_t now_ticks;
	counter_get_value(hw_timer_dev, &now_ticks);
	alarm_cfg0 = (struct counter_alarm_cfg){
		.flags = COUNTER_ALARM_CFG_ABSOLUTE,
		.ticks = now_ticks + ts_ticks,
		.callback = mi_alarma_hw_callback_0,
		.user_data = NULL
	};
	counter_set_channel_alarm(hw_timer_dev, 0, &alarm_cfg0);  // Set the alarm

	//================================================================================
	k_msleep(500);
	advertising_start();
	gpio_pin_set_dt(&red_led, 0);
}