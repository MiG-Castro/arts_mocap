/*
*/
#include <stdlib.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/scan.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/conn_ctx.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/iso.h>
#include "my_imu_ble_service.h"

// LOG ===============================================================
#define LOG_MODULE_NAME C
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

// UART ==============================================================
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
#define UART_RX_BUF_SIZE 255
#define UART_TX_BUF_SIZE 100

// Reception thread
static void uart_rx_thread(void);
K_THREAD_DEFINE(uart_rx_tid, 1024, uart_rx_thread, NULL, NULL, NULL, 7, 0, 0);

// TxPkt structure
struct uart_packet {
    uint8_t msg_type;
    uint8_t p_idx;
    uint8_t payload[UART_TX_BUF_SIZE];
    uint16_t len;
};
// Cola de paquetes a enviar
K_MSGQ_DEFINE(uart_tx_queue, sizeof(struct uart_packet), 15, 4);

// Hilo de transmision
static void uart_tx_thread(void);
K_THREAD_DEFINE(uart_tx_tid, 1024, uart_tx_thread, NULL, NULL, NULL, 6, 0, 0);

// --- UART Protocol ---
#define PKT_START_BYTE  0x7E
#define PKT_END_BYTE    0x7F
#define PKT_ESC_BYTE    0x7D
#define PKT_XOR_BYTE    0x20
// COMMANDS
// To the peripherals
#define CMD_TYPE_SEND_CMD    0x00   // cambio de valor
#define CMD_CTRL_NOTIFY      0x01   // nuevo
#define CMD_CTRL_INDICATE    0x02   // nuevo
#define CMD_START_STOP_TX    0x03   // cambio de valor
#define CMD_STOP_PKT         0x04   // cambio de valor
#define CMD_FS               0x05   // nuevo
#define CMD_SAMP_X_PKT       0x06   // nuevo
// To the central
#define CMD_TYPE_TX_UART_OFF 0x16   // sin cambios
#define CMD_TYPE_TX_UART_ON  0x17   // sin cambios
#define CMD_TYPE_PRINT_OFF   0x18   // sin cambios
#define CMD_TYPE_PRINT_ON    0x19   // sin cambios
#define CMD_TYPE_START_SCAN  0x1A   // cambio de valor
#define CMD_TYPE_STOP_SCAN   0x1B   // cambio de valor
#define CMD_TYPE_DISCONNECT  0x1C   // cambio de valor
#define CMD_UPDATE_PARAMS    0x1D   // cambio de valor
// Pkt info
#define MSG_TYPE_SENSOR_DATA 0x60
#define MSG_TYPE_EVENT_DATA  0x70

// ===================================================================
// BLE configuration and variables (& default values)
// ===================================================================
#define MAX_CONNECTIONS CONFIG_BT_MAX_CONN
static bool scan_on = true;     // To activate/deactivate the scan process
static uint16_t scan_window_N = 20, scan_interval_N = 160; // u = 0.625ms
static bool phy_2M = false;     // True = Set PHY to 2M, False = Set PHY to 1M
// Structures to set the PHY
static struct bt_conn_le_phy_param *g_phy_param_2M = BT_CONN_LE_PHY_PARAM_2M;
static struct bt_conn_le_phy_param *g_phy_param_1M = BT_CONN_LE_PHY_PARAM_1M;

// BLE Connection parameters 
// CI: N x 1.25ms, latency = N x CI, timeout = N x 10ms
static uint16_t conn_interv_N = 10, latency_N = 3, timeout_N = 100;
// Values to set in the data length
static uint16_t data_length = 73, tx_time_us = 800; // DL = desired value + 7B of headers
// Structure for the MTU (the MTU is defined in the prj.conf)
static struct bt_gatt_exchange_params exchange_params;

// Context manager for connections
BT_CONN_CTX_DEF(conns, MAX_CONNECTIONS, sizeof(struct bt_imus_client));
static int idx; // Index of the connection

// BLE Address big endian pass to little endian
// FA:53:60:43:10:2F -> 2F:10:43:60:53:FA
/*
E9:B6:1E:C5:C0:74
C6:9B:47:DE:A4:AF
F3:8A:4F:6B:EB:2C
EF:59:5F:75:C1:FD
C6:8E:9B:51:3F:33
*/
static const bt_addr_le_t sensor_wl[] = {
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x2F, 0x10, 0x43, 0x60, 0x53, 0xFA}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x86, 0xD4, 0x3C, 0x99, 0x28, 0xFA}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x84, 0x20, 0x1F, 0x32, 0x67, 0xDC}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x1B, 0xF8, 0xB5, 0x7C, 0xB0, 0xC0}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x9B, 0xED, 0xDA, 0x2C, 0x1B, 0xEC}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0xD0, 0x8D, 0x38, 0xBE, 0xC0, 0xCC}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0xDA, 0x37, 0x2B, 0x40, 0x58, 0xD3}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x54, 0xB7, 0xDD, 0x0E, 0x29, 0xDD}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0xA6, 0x35, 0xFD, 0x4E, 0xC2, 0xCE}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0xFD, 0xC1, 0x75, 0x5F, 0x59, 0xEF}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x74, 0xC0, 0xC5, 0x1E, 0xB6, 0xE9}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0xAF, 0xA4, 0xDE, 0x47, 0x9B, 0xC6}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x2C, 0xEB, 0x6B, 0x4F, 0x8A, 0xF3}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x33, 0x3F, 0x51, 0x9B, 0x8E, 0xC6}}},
};

// Global variables for the system ==================================
// Print/log control and UART packet transmission
static bool print_log = true, send_uart = false;
static uint32_t fs_hz;
static uint8_t samples_x_pkt;

// Forward declarations =============================================
// BLE
static uint8_t on_gatt_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
void ble_cmd_write(uint8_t p_idx, const uint8_t *command_data, uint8_t len);
// UART
void send_uart_packet(uint8_t msg_type, uint8_t p_idx, const uint8_t *payload, uint16_t len);
static void handle_uart_command(const uint8_t *pkt, uint8_t lnn);

// ===================================================================
// Functions
// ===================================================================
// Get connection index
static int get_conn_index(struct bt_conn *conn)
{
    for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
        const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, i);
        if (ctx) {
            if (ctx->conn == conn) {
                bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
                return i;
            }
            bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
        }
    }
    return -1; // Connection context not registered
}

// Callbacks BL ======================================================
// ACK callback (for write operation)
void on_write_completed(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
    idx = get_conn_index(conn);
    if (print_log) {
        if (err) LOG_ERR("P[%d] AKC err %u", idx, err);
        else LOG_INF("ACK[P%d]\n", idx);
    }
}

// Callback for the received data via notify|indicate characteristic
static uint8_t on_gatt_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
    struct bt_imus_client *imus_client = bt_conn_ctx_get(&conns_ctx_lib, conn);
    if (!imus_client) return BT_GATT_ITER_STOP;

    if (!data) {
        bt_conn_ctx_release(&conns_ctx_lib, imus_client);
        return BT_GATT_ITER_STOP;
    }

    idx = get_conn_index(conn);
    const uint8_t *b = data;

    // Packet received
    if (length > 0) {
        uint32_t packet_num = sys_get_le32(b);

        if (params->value_handle == imus_client->sensordata_handle) {
            if (print_log) LOG_INF("RX_N[%uB]-P[%d:%u]", length, idx, packet_num);
        }
        if (params->value_handle == imus_client->exercisedetection_handle) {
            if (print_log) LOG_INF("RX_I[%uB]-P[%d:%u]", length, idx, packet_num);
        }

        if (send_uart) {
            struct uart_packet pkt = {
                .msg_type = MSG_TYPE_SENSOR_DATA,
                .p_idx = idx,
                .len = length
            };
            memcpy(pkt.payload, data, length);
            // Put packet into TX queue (non-blocking)
            k_msgq_put(&uart_tx_queue, &pkt, K_NO_WAIT);
        }
    }
    bt_conn_ctx_release(&conns_ctx_lib, imus_client);
    return BT_GATT_ITER_CONTINUE;
}

// Service discovery callback
static void discovery_complete(struct bt_gatt_dm *dm, void *context)
{
    struct bt_imus_client *imus_client = context;
    struct bt_conn *conn = (struct bt_conn *)bt_gatt_dm_conn_get(dm);
    int idx = get_conn_index(conn);

    int err = my_imus_handles_assign(dm, imus_client);
    if (err) {
        if (print_log) LOG_ERR("[P%d] Failed to assign handles.", idx);
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }

    err = my_imus_subscribe_sensordata(conn, imus_client, on_gatt_notify);
    if (err && print_log) { 
        LOG_ERR("[P%d] Failed to subscribe to SensorData (err %d).", idx, err);
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }

    err = my_imus_subscribe_exercisedetection(conn, imus_client, on_gatt_notify);
    if (err && print_log) {
        LOG_ERR("[P%d] Failed to subscribe to ExerciseDetection (err %d).", idx, err);
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }

    bt_gatt_dm_data_release(dm);
    if (print_log) printk("[P%d] subscription completed.\n", idx);
    if (scan_on) bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
}

static struct bt_gatt_dm_cb discovery_cb = {.completed = discovery_complete};

// ===================================================================
// Connection and Configuration Chain Callbacks
// ===================================================================
// MTU update callback
static void exchange_mtu_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (err) {
		if (print_log) LOG_ERR(" - MTU request failed (err %u)", err);
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}
	if (print_log) printk(" - MTU set to %d bytes.\n", bt_gatt_get_mtu(conn) - 3);

    // FINAL STEP: Start service discovery
    struct bt_imus_client *imus_client = bt_conn_ctx_get(&conns_ctx_lib, conn);
    if (!imus_client) return;

    int gatt_err = bt_gatt_dm_start(conn, BT_UUID_IMUS, &discovery_cb, imus_client);
    if (gatt_err && print_log) LOG_ERR("Failed to start discovery (err %d).", gatt_err);
    bt_conn_ctx_release(&conns_ctx_lib, imus_client);
}

// Data Length update callback
static void on_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
	if (print_log) printk(" - Data Length (Tx/Rx): %u/%u bytes, %u/%u us.\n", 
        info->tx_max_len, info->rx_max_len, info->tx_max_time, info->rx_max_time);

    // STEP 4: Exchange MTU
    exchange_params.func = exchange_mtu_cb;
    int err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err && print_log) LOG_ERR("MTU request failed (err %d).", err);
}

// Connection params update callback
static void on_conn_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    double ci = interval * 1.25;
	uint16_t sto = timeout * 10;
	if (print_log) printk(" - Connection parameters set to %.2f ms, %d, %d ms.\n", ci, latency, sto);

    // STEP 3: Update Data Length
    struct bt_conn_le_data_len_param g_data_len_param = {
		.tx_max_len = data_length,
		.tx_max_time = tx_time_us,
	};

    int err = bt_conn_le_data_len_update(conn, &g_data_len_param);
    if (err && print_log) LOG_ERR("DL request failed (err %d).", err);
}

// PHY update callback
static void on_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
    if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_1M) {if (print_log) printk(" - PHY set to 1M\n");}
    if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_2M) {if (print_log) printk(" - PHY set to 2M\n");}

    // STEP 2: Update the connection interval
    struct bt_le_conn_param *g_conn_param = BT_LE_CONN_PARAM(
		conn_interv_N,    // Min Connection Interval = Value x 1.25ms
		conn_interv_N,    // Min Connection Interval = Value x 1.25ms
		latency_N,        // Latency (Number of connection intervals that the peripheral can skip)
		timeout_N         // Time that the central will wait before disconnecting (ms) = Value x 10ms
	);
    int err = bt_conn_le_param_update(conn, g_conn_param);
    if (err && print_log) LOG_ERR("Connection parameters request failed (err %d).", err);
}

// Peripheral connection
static void connected(struct bt_conn *conn, uint8_t hci_err)
{
    // Stop the scanning to avoid interruptions during the configuration process
    bt_scan_stop();

    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (hci_err) {
        if (print_log) LOG_ERR("Failed connection to %s (err 0x%02x).", addr, hci_err);
        bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
        return;
    }
    if (print_log) printk("[C] Connected to: %s.\n", addr);

    // Store the connection context (IMPORTANT to manage multiple connections!!!)
    struct bt_imus_client *imus_client = bt_conn_ctx_alloc(&conns_ctx_lib, conn);
    if (!imus_client) {
        if (print_log) LOG_WRN("There is no memory for context.");
        bt_conn_disconnect(conn, BT_HCI_ERR_INSUFFICIENT_RESOURCES);
        return;
    }
    memset(imus_client, 0, sizeof(struct bt_imus_client));
    bt_conn_ctx_release(&conns_ctx_lib, imus_client);

    // STEP 1: Start the configuration chain by updating the PHY
    int err;
    if (phy_2M) err = bt_conn_le_phy_update(conn, g_phy_param_2M);
    else err = bt_conn_le_phy_update(conn, g_phy_param_1M);
    if (err && print_log) LOG_ERR("PHY request failed (err %d).\n", err);
}

// Peripheral disconnection
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    int idx = get_conn_index(conn);
    if (print_log) printk("[C] Peripheral %d - %s disconnected (0x%02x).*******************************\n",
        idx, addr, reason);
    bt_conn_ctx_free(&conns_ctx_lib, conn);

    if (scan_on) bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
}

// BLE Callbacks
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .le_phy_updated = on_phy_updated,
    .le_param_updated = on_conn_param_updated,
    .le_data_len_updated = on_data_len_updated
};

// Send data over BLE -> write characteristic
void ble_cmd_write(uint8_t p_idx, const uint8_t *command_data, uint8_t len)
{
    int err = 0;
    // "Broadcast" message (message for all peripherals- one by one)
    if (p_idx == 0xFF) {
        for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
            const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, i);
            if (ctx) {
                err = my_imus_write_command(ctx->conn, ctx->data, command_data, len, on_write_completed);
                bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
            }
        }
    } else {
        // Message to a specific peripheral
        const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, p_idx);
        if (ctx) {
            err = my_imus_write_command(ctx->conn, ctx->data, command_data, len, on_write_completed);
            bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
        }
    }
    if (print_log) printk("[C] BLE-TX CMD[%02X:%uB] to P[%02X]\n", command_data[0], len, p_idx);
    if(err != 0) LOG_ERR("Error %d in Write Operation", err);
}

// FUNCIONES UART ====================================================
// Byte masking
void send_uart_byte(uint8_t byte) {
    // If byte is special, escape it
    if (byte == PKT_START_BYTE || byte == PKT_END_BYTE || byte == PKT_ESC_BYTE) {
        uart_poll_out(uart_dev, PKT_ESC_BYTE);       // ESC BYTE
        uart_poll_out(uart_dev, byte ^ 0x20);        // MOD BYTE (XOR)
    } else {
        uart_poll_out(uart_dev, byte);               // No problem, just send
    }
}

// Data transmission - Structure: [START][LEN-PAYLOAD][TYPE][IDX][PAYLOAD][END]
void send_uart_packet(uint8_t msg_type, uint8_t p_idx, const uint8_t *payload, uint16_t len)
{
    static uint8_t tx_buf[UART_TX_BUF_SIZE];
    if (len + 3 > UART_TX_BUF_SIZE) {
        if (print_log) LOG_ERR("UART packet too big to send (%u bytes).", len);
    } else {
        uint8_t lnn = len + 3;
        tx_buf[0] = lnn;
        tx_buf[1] = msg_type;
        tx_buf[2] = p_idx;
        memcpy(&tx_buf[3], payload, len);

        // Send the packet
        uart_poll_out(uart_dev, PKT_START_BYTE);
        for (int i = 0; i < lnn; i++) send_uart_byte(tx_buf[i]);
        uart_poll_out(uart_dev, PKT_END_BYTE);
    }
}

static void uart_tx_thread(void) {
    struct uart_packet pkt;
    while (1) {
        // Wait until there is a packet in the queue
        k_msgq_get(&uart_tx_queue, &pkt, K_FOREVER);
        send_uart_packet(pkt.msg_type, pkt.p_idx, pkt.payload, pkt.len);
    }
}

// Command detection and execution
static void handle_uart_command(const uint8_t *pkt, uint8_t lnn)
{
    static uint8_t tx_w[UART_RX_BUF_SIZE] = {0};
    memcpy(tx_w, pkt, lnn);
    uint8_t len = tx_w[0], cmd_type = tx_w[1];
    uint8_t *payload = &tx_w[2];

    switch (cmd_type) {
        // Commands to the Central-Dongle ============================================
        case CMD_TYPE_TX_UART_ON:
            send_uart = true;
            if (print_log) printk("[C] UART Tx ON.\n");
            break;
        
        case CMD_TYPE_TX_UART_OFF:
            send_uart = false;
            if (print_log) printk("[C] UART Tx OFF.\n");
            break;

        case CMD_TYPE_PRINT_ON:
            print_log = true;
            printk("[C] Logs ON.\n");
            break;

        case CMD_TYPE_PRINT_OFF:
            print_log = false;
            printk("[C] Logs OFF.\n");
            break;

        case CMD_TYPE_START_SCAN:
            if (print_log) printk("[C] Start Scan.\n");
            bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
            scan_on = true;
            break;

        case CMD_TYPE_STOP_SCAN:
            if (print_log) printk("[C] Stop Scan.\n");
            bt_scan_stop();
            scan_on = false;
            break;

        // case CMD_TYPE_SAMP_CTRL_P:
        //     // change the Samples x packet and the Sampling frequency 
        //     // Fs:2B, SXP:1B, PXISO:1B
        //     if (len == 3) {
        //         uint8_t n;
        //         n = payload[0];
        //         if (n > 0) fs_hz = payload[0];
        //         n = payload[1];
        //         if (n > 0) samples_x_pkt = payload[1];
        //         n = payload[2];
        //          if (print_log) printk("[C] El lunes sin falta.\n");
        //     }
        //     break;

        case CMD_UPDATE_PARAMS:
            // Try to update only rigth values
            // PHY:1B, CI:2B, L:2B, STO:2B(ms), DL:2B, Tx/Rx:2B(us) = 11B
            if (len == 11) {
                uint16_t n;

                // 1. PHY
                if (payload[0] == 0x01) phy_2M = false;
                if (payload[0] == 0x02) phy_2M = true;
                
                // 2. Connection Interval
                n = sys_get_le16(&payload[1]);
                if (n >= 6 &&  n <= 3200) conn_interv_N = n;

                // 3. Latency
                n = sys_get_le16(&payload[3]);
                if (n >= 0) latency_N = n;
                
                // 4. Supervision time out (greater than latency)
                n = sys_get_le16(&payload[5]);
                if (n * 10 > latency_N * conn_interv_N * 1.25) timeout_N = n;
                else timeout_N = ((latency_N * conn_interv_N * 1.25) / 10) + 10;

                // Data Length
                n = sys_get_le16(&payload[7]);
                if (n >= 27 &&  n <= 251) data_length = n; 

                // Tx/Rx Radio usage time
                // Check the DL, PHY and Connection event length
                n = sys_get_le16(&payload[9]);
                if (n >= 328 &&  n <= 17040) tx_time_us = n;

                if (print_log) printk("[C] PHY=%s, CI=%.2fms (%u), L=%u, TO=%ums, DL=%u, Tx/Rx=%uus.\n",
                    phy_2M ? "2M" : "1M", conn_interv_N * 1.25, conn_interv_N, latency_N, timeout_N*10, 
                    data_length, tx_time_us);
            }
            break;

        case CMD_TYPE_DISCONNECT:
            if (payload[0] == 0xFF) {
                if (print_log) printk("[C] Disconnect ALL Peripherals.\n");
                for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
                    const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, i);
                    if (ctx) {
                        bt_conn_disconnect(ctx->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                        bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
                    }
                }
            } else {
                if (print_log) printk("[C] Disconnect [Peripheral %u].\n", payload[0]);
                const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, payload[0]);
                if (ctx) {
                    bt_conn_disconnect(ctx->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                    bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
                }
            }
            break;

        // Commands to the Peripherals ===============================================
        // ble_cmd_write(uint8_t p_idx, const uint8_t *command_data, uint8_t len)
        // tx_w = [LenPayload:0][CMD:1][PAYLOAD:2 to N -> {[idx:2][CMD:3][Data:4->N]}]
        
        case CMD_TYPE_SEND_CMD:
            cmd_type = tx_w[3];
            bool cmd_ok = false;
            switch (cmd_type) {
                case CMD_CTRL_NOTIFY:
                    // P_ID + CMD + Value(uint8) 0x01 = ON other OFF
                    if (len == 3) cmd_ok = true;
                    break;
                case CMD_CTRL_INDICATE:
                    // P_ID + CMD + Value(uint8) 0x01 = ON other OFF
                    if (len == 3) cmd_ok = true;
                    break;
                case CMD_START_STOP_TX:
                    // P_ID + CMD + Value(uint8) 0x01 = Start other Stop
                    if (len == 3) cmd_ok = true;
                    break;
                case CMD_STOP_PKT:
                    // P_ID + CMD + Value(uint32) 0x00 = Disable other activate
                    if (len == 6) cmd_ok = true;
                    break;
                case CMD_FS:
                    // P_ID + CMD + Value(uint32) > 0!
                    if (len == 6) {
                        uint32_t v;
                        v = sys_get_le32(&tx_w[4]);
                        if (v > 0) {
                            cmd_ok = true;
                            fs_hz = v;
                        } else {if (print_log) LOG_ERR("Value must be > 0");}
                    }
                    break;
                case CMD_SAMP_X_PKT:
                    // P_ID + CMD + Value(uint8) > 0!
                    if (len == 3) {
                        if (tx_w[4] > 0) {
                            cmd_ok = true;
                            samples_x_pkt = tx_w[4];
                        } else {if (print_log) LOG_ERR("Value must be > 0");}
                    }
                    break;

                default:
                    if (print_log) {
                        LOG_WRN("Unknown TX CMD");
                        for (int i = 0; i < lnn; i++) printk("%02X%s", tx_w[i], (i + 1 == lnn) ? "\n":" ");
                    }
                    break;
            }
            if (cmd_ok) ble_cmd_write(tx_w[2], &tx_w[3], len - 1);
            break;

        default:
            if (print_log) {
                LOG_WRN("Unknown UART CMD");
                for (int i = 0; i < lnn; i++) printk("%02X%s", tx_w[i], (i + 1 == lnn) ? "\n":" ");
            }
            break;
    }
}

static void uart_rx_thread(void)
{
    uint8_t rx_pkt[UART_RX_BUF_SIZE] = {0}, pos = 0, c;
    bool escape_next = false;

    while (true) {
        // Try to read 1B, else -> Sleep
        if (uart_poll_in(uart_dev, &c)) {
            k_msleep(1000);
            continue;
        }
        // Start of the pkt
        if (c == PKT_START_BYTE) {
            pos = 0;
            escape_next = false;
            rx_pkt[pos++] = c;
        } else {
            // we are in the pkt!!!

            // Is the end of the pkt?
            if (c == PKT_END_BYTE) {
                //PKT = [Start][LenPayload][CMD][PAYLOAD][End]
                rx_pkt[pos] = c;
                handle_uart_command(&rx_pkt[1], --pos);
                pos = 0;
                escape_next = false;
                continue;
            }

             // If a escape byte arrive
            if (c == PKT_ESC_BYTE) {
                // Flag ON!
                escape_next = true;
                // Skip -> dont save the byte
                continue;
            }

            if (escape_next) {
                // Restore the original value
                c = c ^ PKT_XOR_BYTE;
                escape_next = false;
            }

            // Payload byte
            if (pos < UART_RX_BUF_SIZE) rx_pkt[pos++] = c;
            else {
                LOG_ERR("RX buffer overflow, discarding packet");
                pos = 0;
                escape_next = false;
            }
        }
    }
}

int main(void)
{
    int err;
    // ===================================================================
    // UART
    // ===================================================================
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not found!");
        return 0;
    }
    // ===================================================================
    // BLE
    // ===================================================================
    // BLE - ACL *********************************************************
    k_msleep(1000);
    if (bt_enable(NULL)) return 0;
    bt_conn_cb_register(&conn_callbacks);

    // SCAN CONFIGURATION
    struct bt_le_conn_param *sc_conn_param = BT_LE_CONN_PARAM(
        conn_interv_N, conn_interv_N, latency_N, timeout_N);

    struct bt_le_scan_param scan_param = {
        .type = BT_SCAN_TYPE_SCAN_ACTIVE,
        .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = scan_interval_N,
        .window = scan_window_N,};
    struct bt_scan_init_param scan_init = {
        .connect_if_match = 1,
        .scan_param = &scan_param,
        .conn_param = sc_conn_param};
    bt_scan_init(&scan_init);
    
    // Filter by Service UUID
    bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_IMUS);

    // Whitelist
    for (int i = 0; i < ARRAY_SIZE(sensor_wl); i++) {
        err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_ADDR, &sensor_wl[i]);
        if (err) LOG_ERR("Whitelist %d, i=%d", err, i);
    }
    // Enable filters
    if(bt_scan_filter_enable(BT_SCAN_UUID_FILTER | BT_SCAN_ADDR_FILTER, true)){
        LOG_ERR("Error(%d) enabling filters", err);
    }

    // Get the Bluetooth device address
	bt_addr_le_t addr;
	size_t count = 1;
	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_id_get(&addr, &count);
	bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));

    printk("%s - %s [CEL=%dus]\n", 
        CONFIG_BT_DEVICE_NAME, addr_str,
        CONFIG_BT_CTLR_SDC_MAX_CONN_EVENT_LEN_DEFAULT);

    bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
}