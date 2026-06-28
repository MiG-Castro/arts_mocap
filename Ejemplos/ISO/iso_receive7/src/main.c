/*

 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>

#define BT_LE_SCAN_CUSTOM BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE, BT_LE_SCAN_OPT_NONE, BT_GAP_SCAN_FAST_INTERVAL, BT_GAP_SCAN_FAST_WINDOW)
#define NAME_LEN		30
#define PA_RETRY_COUNT	6
#define BIS_ISO_CHAN_COUNT 1U

static bool         per_adv_found, per_adv_lost;
static bt_addr_le_t per_addr;
static uint8_t      per_sid;
static uint32_t     per_interval_us;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);
static K_SEM_DEFINE(sem_per_big_info, 0, 1);
static K_SEM_DEFINE(sem_big_sync, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_sync_lost, 0, BIS_ISO_CHAN_COUNT);

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led_gpio = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

//Scan normal -> PA
static void scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf)
{
	// Si aun no se encuenta un PA y se acaba de detectar uno
	// Broadcas Adv -> Ext Adv
	if (!per_adv_found && info->interval) {
		per_adv_found = true;
		per_sid = info->sid;
		per_interval_us = BT_CONN_INTERVAL_TO_US(info->interval);
		bt_addr_le_copy(&per_addr, info->addr);

		char le_addr[BT_ADDR_LE_STR_LEN];
		bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
		printk("PA %s - %uus\n", le_addr, per_interval_us);

		k_sem_give(&sem_per_adv);
	}
}
static struct bt_le_scan_cb scan_callbacks = {.recv = scan_recv,};

// Sync to PA
static void sync_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("PAS_C[%u] %s, %ums\n", bt_le_per_adv_sync_get_index(sync), le_addr, info->interval * 5 / 4);
	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("PAS_C[%u] %s sync terminated\n", bt_le_per_adv_sync_get_index(sync), le_addr);

	per_adv_lost = true;
	k_sem_give(&sem_per_sync_lost);
}

// 1st step for ISO - Here we can get some ISO parameters
static void biginfo_cb(struct bt_le_per_adv_sync *sync, const struct bt_iso_biginfo *biginfo)
{k_sem_give(&sem_per_big_info);}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.biginfo = biginfo_cb,
};

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info, struct net_buf *buf)
{
	// First we get the cycles
    // uint32_t cycles = k_cycle_get_32();
	// uint32_t t_ack_us = k_cyc_to_us_floor32(cycles);
	// printk("%u - %u\n", info->ts, t_ack_us);
	// uint32_t ms = k_uptime_get_32();

	if ((info->seq_num % 10) == 0) {
		gpio_pin_set_dt(&led_gpio, 1);
		printk("%u - %u\n", info->seq_num, k_uptime_get_32());
	} else gpio_pin_set_dt(&led_gpio, 0);

    // if ((info->seq_num % 10) == 0) {
    //     gpio_pin_set_dt(&led_gpio, 1);
    //     printk("seq=%u ts=%u uptime_ms=%u uptime_us=%llu\n", 
    //            info->seq_num, 
    //            info->ts,
    //            k_uptime_get_32(),
    //            k_uptime_get() * 1000);
    // } else {
    //     gpio_pin_set_dt(&led_gpio, 0);
    // }
}

static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO C[%p] connected\n", chan);
	k_sem_give(&sem_big_sync);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO C[%p] disconnected 0x%02x\n", chan, reason);
	if (reason != BT_HCI_ERR_OP_CANCELLED_BY_HOST) {k_sem_give(&sem_big_sync_lost);}
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
	k_sem_reset(&sem_per_sync_lost);
	k_sem_reset(&sem_per_big_info);
	k_sem_reset(&sem_big_sync);
	k_sem_reset(&sem_big_sync_lost);
}

int main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	struct bt_iso_big *big;
	uint32_t sem_timeout_us;
	int err;

	// LED =========================================================
	if (!gpio_is_ready_dt(&led_gpio)) {
		printk("GPIO-LED err.\n");
		return 0;
	}
	gpio_pin_configure_dt(&led_gpio, GPIO_OUTPUT_ACTIVE);
	
	// Bluetooth ==================================================
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	// Scan & Periodic Advertising - callbacks
	bt_le_scan_cb_register(&scan_callbacks);
	bt_le_per_adv_sync_cb_register(&sync_callbacks);

	do {
		// Reset all
		k_msleep(500);
		gpio_pin_set_dt(&led_gpio, 0);
		reset_semaphores();
		per_adv_lost = false;
		per_adv_found = false;

		// Start scanning
		err = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
		if (err) {
			printk("Star Scan err %d\n", err);
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
			printk("PA-Sync err %d\n", err);
			return 0;
		}
		// Wait PA-Sync (sync_cb)
		sem_timeout_us = per_interval_us * PA_RETRY_COUNT;
		err = k_sem_take(&sem_per_sync, K_USEC(sem_timeout_us));
		if (err) {
			printk("PA-Sync failed err %d\n", err);
			err = bt_le_per_adv_sync_delete(sync);
			if (err) return 0;
			continue;
		}

		// Wait BIG-inf (biginfo_cb)
		err = k_sem_take(&sem_per_big_info, K_USEC(sem_timeout_us));
		if (err) {
			printk("BIG-inf err %d\n", err);
			if (per_adv_lost) continue;
			err = bt_le_per_adv_sync_delete(sync);
			if (err) return 0;
			continue;
		}

		// Create BIG Sync
		err = bt_iso_big_sync(sync, &big_sync_param, &big);
		if (err) {
			printk("BIG-Sync failed err %d\n", err);
			return 0;
		}

		// Waiting for BIG sync (ISO_Connected)
		err = k_sem_take(&sem_big_sync, K_SECONDS(10));
		if (err == 0) {
			printk("BIG sync Successful.\n");
			// Waiting for BIG sync lost chan
			err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
		}

		// In case of lost or faild sync -> restart the process
		printk("BIG sync failed/lost err %d\n", err);
		if (big) bt_iso_big_terminate(big);
		if (sync) bt_le_per_adv_sync_delete(sync);
		gpio_pin_set_dt(&led_gpio, 1);

	} while (true);
}