#include <stddef.h>
#include <stdint.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>

#define BIS_ISO_CHAN_COUNT 1
#define BIG_SDU_INTERVAL_US      (200000)
#define BUF_ALLOC_TIMEOUT_US     (BIG_SDU_INTERVAL_US * 2U) /* milliseconds */
#define BIG_TERMINATE_TIMEOUT_US (10 * USEC_PER_SEC) /* microseconds */

NET_BUF_POOL_FIXED_DEFINE(
	bis_tx_pool,
	BIS_ISO_CHAN_COUNT,
	BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
	CONFIG_BT_CONN_TX_USER_DATA_SIZE,
	NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_term, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_iso_data, CONFIG_BT_ISO_TX_BUF_COUNT, CONFIG_BT_ISO_TX_BUF_COUNT);

// Callbacks ISO Events ============================================================
static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO C[%p] connected\n", chan);
	k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO C[%p] disconnected with reason 0x%02x\n", chan, reason);
	k_sem_give(&sem_big_term);
}

static void iso_sent(struct bt_iso_chan *chan){k_sem_give(&sem_iso_data);}

static struct bt_iso_chan_ops iso_ops = {
	.connected		= iso_connected,
	.disconnected	= iso_disconnected,
	.sent           = iso_sent,
};

static struct bt_iso_chan_io_qos iso_tx_qos = {.sdu = 1, .rtn = 0, .phy = BT_GAP_LE_PHY_2M,};
static struct bt_iso_chan_qos bis_iso_qos = {.tx = &iso_tx_qos,};
static struct bt_iso_chan bis_iso_chan[] = {{.ops = &iso_ops, .qos = &bis_iso_qos, },};
static struct bt_iso_chan *bis[] = {&bis_iso_chan[0],};

static struct bt_iso_big_create_param big_create_param = {
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_channels = bis,
	.interval = BIG_SDU_INTERVAL_US, /* in microseconds */
	.latency = 5, /* in milliseconds */
	.packing = 0, /* 0 - sequential, 1 - interleaved */
	.framing = 0, /* 0 -   unframed, 1 - framed */
};

int main(void)
{
	// BIG_SDU_INTERVAL_US = 50000
	const uint32_t adv_interval_ms = BIG_SDU_INTERVAL_US / 1000U;
	const uint32_t ext_adv_interval_ms = adv_interval_ms - 10U;
	int err;

	// Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	// Adv & Big Structures
	struct bt_le_ext_adv *adv;
	struct bt_iso_big *big;

	// Create a non-connectable advertising set */
	const struct bt_le_adv_param *ext_adv_param = BT_LE_ADV_PARAM(
		BT_LE_ADV_OPT_EXT_ADV,
		BT_GAP_MS_TO_ADV_INTERVAL(ext_adv_interval_ms),
		BT_GAP_MS_TO_ADV_INTERVAL(ext_adv_interval_ms),
		NULL);
	err = bt_le_ext_adv_create(ext_adv_param, NULL, &adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return 0;
	}

	// Set advertising data to have complete local name set */
	const struct bt_data ad[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE,
		CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),};
	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set advertising data (err %d)\n", err);
		return 0;
	}

	// Set periodic advertising parameters
	const struct bt_le_per_adv_param *per_adv_param = BT_LE_PER_ADV_PARAM(
		BT_GAP_MS_TO_PER_ADV_INTERVAL(adv_interval_ms),
		BT_GAP_MS_TO_PER_ADV_INTERVAL(adv_interval_ms), 
		BT_LE_PER_ADV_OPT_NONE);
	err = bt_le_per_adv_set_param(adv, per_adv_param);
	if (err) {
		printk("Failed to set periodic advertising parameters (err %d)\n", err);
		return 0;
	}

	// Enable Periodic Advertising */
	err = bt_le_per_adv_start(adv);
	if (err) {
		printk("Failed to enable periodic advertising (err %d)\n", err);
		return 0;
	}

	// Start extended advertising */
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising (err %d)\n", err);
		return 0;
	}

	// Create BIG */
	err = bt_iso_big_create(adv, &big_create_param, &big);
	if (err) {
		printk("Failed to create BIG (err %d)\n", err);
		return 0;
	}

	// Waiting for BIG complete
	err = k_sem_take(&sem_big_cmplt, K_FOREVER);
	if (err) {
		printk("failed (err %d)\n", err);
		return 0;
	}

	uint8_t iso_data[1] = {0xAA};
	while (true) {
		// Send periodic msg ==============================================
		struct net_buf *buf;
		int ret;

		buf = net_buf_alloc(&bis_tx_pool, K_USEC(BUF_ALLOC_TIMEOUT_US));
		if (!buf) {
			printk("Data buffer allocate timeout on channel\n");
			return 0;
		}

		ret = k_sem_take(&sem_iso_data, K_USEC(BUF_ALLOC_TIMEOUT_US));
		if (ret) {
			printk("k_sem_take for ISO data sent failed\n");
			net_buf_unref(buf);
			return 0;
		}
		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
		net_buf_add_mem(buf, iso_data, sizeof(iso_data));
		ret = bt_iso_chan_send(&bis_iso_chan[0], buf, 0);
		if (ret < 0) {
			printk("Unable to broadcast data %d", ret);
			net_buf_unref(buf);
			return 0;
		}
	}

	// err = bt_iso_big_terminate(big);
	// err = k_sem_take(&sem_big_term, K_FOREVER);
	// err = bt_iso_big_create(adv, &big_create_param, &big);
	// err = k_sem_take(&sem_big_cmplt, K_FOREVER);
}