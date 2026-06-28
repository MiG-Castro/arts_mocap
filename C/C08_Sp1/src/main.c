/*
 * Central para el servicio IMU - Arquitectura Asíncrona
 * Funcional para multiconexion - Elemento clave "conn ctx" !!!
 * Central hace todas las solicitudes de actulizacion de parametros
 * Comunicacion UART
 * - Hilo para Rx & Hilo para Tx - Sin checksum ni maquina de estados en hilo Rx!!
 * Reestructuración de uso de variables globales para ser congruente con las definiciones|estructuras de BLE
 * 
 * Adicion de rutina de sincronizacion
 * - El periferico inicia el proceso de sync con una "Indicacion"
 * - Rutina de syncronizacion en un callback aparte
 * - CAMBIO EN LOS COMANDOS!!!
*/
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/scan.h>
#include <bluetooth/gatt_dm.h>
#include <zephyr/sys/byteorder.h>
#include <bluetooth/conn_ctx.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/time_units.h>
#include "my_imu_ble_service.h"

// LOG ===============================================================
#define LOG_MODULE_NAME XCD
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

// UART ==============================================================
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

#define UART_RX_BUF_SIZE 255
#define UART_TX_BUF_SIZE 100

// Hilo de recepcion
// static void uart_rx_thread(void);
// K_THREAD_DEFINE(uart_rx_tid, 1024, uart_rx_thread, NULL, NULL, NULL, 7, 0, 0);

// struct uart_packet {
//     uint8_t msg_type;
//     uint8_t p_idx;
//     uint8_t payload[UART_TX_BUF_SIZE];
//     uint16_t len;
// };
// // Cola de paquetes a enviar
// K_MSGQ_DEFINE(uart_tx_queue, sizeof(struct uart_packet), 15, 4);

// // Hilo de transmision
// static void uart_tx_thread(void);
// K_THREAD_DEFINE(uart_tx_tid, 1024, uart_tx_thread, NULL, NULL, NULL, 6, 0, 0);

// --- Protocolo UART ---
#define PKT_START_BYTE 0x7E
#define PKT_END_BYTE   0x7F

// COMMANDS
// To the central
#define CMD_TYPE_SYNC_ON     0x13
#define CMD_TYPE_SYNC_OFF    0x14
#define CMD_TYPE_SAMP_CTRL_P 0x15
#define CMD_TYPE_TX_UART_OFF 0x16
#define CMD_TYPE_TX_UART_ON  0x17
#define CMD_TYPE_PRINT_OFF   0x18
#define CMD_TYPE_PRINT_ON    0x19
#define CMD_TYPE_START_SCAN  0x20
#define CMD_TYPE_STOP_SCAN   0x21
#define CMD_TYPE_DISCONNECT  0x22
#define CMD_UPDATE_PARAMS    0x23
// To the peripherals
#define CMD_SYNC_START       0x24
#define CMD_SIMP_START       0x25
#define CMD_STOP_PKT         0x26
#define CMD_START_TX         0x27
#define CMD_FIRST_WRT        0x28
#define CMD_TYPE_SEND_CMD    0x30
// others commands: 
// To the PC
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

// Connection parameters 
// CI: N x 1.25ms, latency = N x CI, timeout = N x 10ms
static uint16_t conn_interv_N = 40, latency_N = 0, timeout_N = 100;
// Values to set in the data length
static uint16_t data_length = 73, tx_time_us = 800; // DL = desired value + 7B of headers
// Structure for the MTU (the MTU is defined in the prj.conf)
static struct bt_gatt_exchange_params exchange_params;

// Context manager for connections
BT_CONN_CTX_DEF(conns, MAX_CONNECTIONS, sizeof(struct bt_imus_client));
static int idx = -1; // Index of the connection

// Global variables for the system ==================================
// Control the prints and the Tx in the UART of the received data 
static bool print_log = true, send_uart = false;

// Variables to sync the peripherals
static bool first_p = false, sync_pn[MAX_CONNECTIONS] = {0}, sync_protocol = true;
static uint32_t cycles, t_ref = 0, t_ack_us = 0, tpn[MAX_CONNECTIONS] = {0};

static uint8_t samples_x_pkt = 3;
static uint16_t fs_hz = 20; // start_delay_ms = 50;
static uint32_t ts_us, start_delay_us, n_sample, tol_us = 1000, delta_us = 5000;

static uint32_t ci_us, t_rx_us, t_rxapp;
static uint8_t cmd_tx[5]={0};

// variables to configure the stop pkt
// static uint8_t stop_pkt_on, stop_tx, stop_ti;
// uint32_t k_stop_pkt;

// Forward declarations =============================================
// BLE
static uint8_t on_gatt_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
void ble_cmd_write(uint8_t p_idx, const uint8_t *command_data, uint8_t len);
// UART
// void send_uart_packet(uint8_t msg_type, uint8_t p_idx, const uint8_t *payload, uint16_t len);
// static void handle_uart_command(const uint8_t *pkt, uint8_t lnn);

// ===================================================================
// Funciones ---------------------------------------------------------
// ===================================================================
// Obtener el índice de una conexión.
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
    return -1; // Connection not registered
}

// Callbacks BL ======================================================
// Recepcion de ACK (for write operation)
void on_write_completed(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
    idx = get_conn_index(conn);
    if (print_log) {
        if (err) LOG_ERR("P[%d] AKC err %u", idx, err);
        else LOG_INF("RX-ACK[P%d]\n", idx);
    }
}

// Process - Sync Protocol
void sync_pross(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
    // First we get the cycles
    cycles = k_cycle_get_32();
    t_ack_us = k_cyc_to_us_floor32(cycles);

    // Print info
    if (print_log) {if (err) LOG_ERR("P[%d] AKC err %u", idx, err);}

    // Response from?
    idx = get_conn_index(conn);

    // Second response
    if (t_ack_us - tpn[idx] < tol_us + 2 * ci_us) {
        // the ACK was received in t < 2 CI!!!
        if (!first_p) {
            first_p = true;
            t_ref = t_ack_us + ci_us;
        }
        // Peripheral N -> SYNC!!!
        sync_pn[idx] = true;
        if (print_log) LOG_INF("P[%d]- Tack0:%uus, Tack1:%uus (%uus) - Sync Success!!!\n", idx, tpn[idx], t_ack_us, t_ack_us - tpn[idx]);
    } else {
        // The process took more than 2 CI
        if (print_log) LOG_INF("P[%d]- Tack0:%uus, Tack1:%uus (%uus) - Failed Sync!!!\n", idx, tpn[idx], t_ack_us, t_ack_us - tpn[idx]);
    }
    // Re-start the scan
    if (scan_on) bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
}

// Callback for the received data via notify|indicate characteristics
static uint8_t on_gatt_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
    // First we get the cycles
    cycles = k_cycle_get_32();

    struct bt_imus_client *imus_client = bt_conn_ctx_get(&conns_ctx_lib, conn);
    if (!imus_client) return BT_GATT_ITER_STOP;

    if (!data) {
        // printk("[P%d] NULL handle 0x%04x\n", idx, params->value_handle);
        bt_conn_ctx_release(&conns_ctx_lib, imus_client);
        return BT_GATT_ITER_STOP;
    }

    idx = get_conn_index(conn);
    const uint8_t *b = data;

    // Protocolo de syncronizacion!!!
    if (sync_protocol && length == 1 && b[0] == 0x00){
        t_rx_us = k_cyc_to_us_floor32(cycles);
        tpn[idx] = t_rx_us;     // save the time

        if (!first_p) {
            // if is the first peripheral to connect
            n_sample = 1;
            start_delay_us = ts_us + ci_us;     // To star in sample 1
            t_ref = t_rx_us + (3 * ci_us);      // In case that P0 failed to sync
        } else {
            // Sync with first peripheral (P0)!!!
            // What is the next next sample (n + 1) that P0 will take?

            // 1. Calculate the time where the app layer of P[N] receive the CMD (from Central perspective)
            t_rxapp = t_rx_us + ci_us + ci_us;
            // 2. Calulate the sample "n + 1" for the time "t_rxapp"
            n_sample = ((t_rxapp - t_ref + ts_us - 1) / ts_us) + 1;
            // 3. Calculate the time to start in sample n + 1 (from Central perspective)
            start_delay_us = t_ref + (n_sample * ts_us);
            // 4. Calculate the time to start in sample n + 1 (from peripheral perspective)
            start_delay_us -= t_rxapp;
        }
        // Paquete - Comando Sincronizacion
        memcpy(&cmd_tx[1], &start_delay_us, sizeof(start_delay_us));
        // BLE-TX pkt
        my_imus_write_command(conn, imus_client, cmd_tx, sizeof(cmd_tx), sync_pross);
        LOG_INF("P[%d]: Trx %us, Ts %uus, CI %uus, Sd %uus", idx, t_rx_us, ts_us, ci_us, start_delay_us);
    }

    if (length > 0) {
        // uint32_t packet_num = sys_get_le32(b);
        uint8_t packet_num = b[0];
        if (params->value_handle == imus_client->sensordata_handle) {
            if (print_log) LOG_INF("RX-N[%uB]-P[%d:%u]", length, idx, packet_num);
        }

        if (params->value_handle == imus_client->exercisedetection_handle) {
            if (print_log) LOG_INF("RX-I[%uB]-P[%d:%u]", length, idx, packet_num);
        }

        // if (send_uart) {
        //     struct uart_packet pkt = {
        //         .msg_type = MSG_TYPE_SENSOR_DATA,
        //         .p_idx = idx,
        //         .len = length
        //     };
        //     memcpy(pkt.payload, data, length);
        //     // Encolar sin bloquear (máximo 20 paquetes)
        //     k_msgq_put(&uart_tx_queue, &pkt, K_NO_WAIT);
        // }
    }

    bt_conn_ctx_release(&conns_ctx_lib, imus_client);
    return BT_GATT_ITER_CONTINUE;
}

// Callbacks - Descubrimiento de servicios
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
    if (err && print_log) { LOG_ERR("[P%d] Failed to subscribe to SensorData (err %d).", idx, err); }
    
    err = my_imus_subscribe_exercisedetection(conn, imus_client, on_gatt_notify);
    if (err && print_log) { LOG_ERR("[P%d] Failed to subscribe to ExerciseDetection (err %d).", idx, err); }

    bt_gatt_dm_data_release(dm);
    if (print_log) printk("[P%d] subscription completed.\n", idx);
}

static struct bt_gatt_dm_cb discovery_cb = {.completed = discovery_complete};

// ===================================================================
// Callbacks de la Cadena de Configuración y Conexión
// ===================================================================

// MTU
static void exchange_mtu_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (err) {
		if (print_log) LOG_ERR(" - MTU request failed (err %u)", err);
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}
	if (print_log) printk(" - MTU set to %d bytes.\n", bt_gatt_get_mtu(conn) - 3);

    // PASO FINAL: Iniciar el descubrimiento de servicios
    struct bt_imus_client *imus_client = bt_conn_ctx_get(&conns_ctx_lib, conn);
    if (!imus_client) return;

    int gatt_err = bt_gatt_dm_start(conn, BT_UUID_IMUS, &discovery_cb, imus_client);
    if (gatt_err && print_log) LOG_ERR("Failed to start discovery (err %d).", gatt_err);
    bt_conn_ctx_release(&conns_ctx_lib, imus_client);
}

// Data Length
static void on_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
	if (print_log) printk(" - Data Length (Tx/Rx): %u/%u bytes, %u/%u us.\n", 
        info->tx_max_len, info->rx_max_len, info->tx_max_time, info->rx_max_time);

    // PASO 4: Intercambiar MTU
    exchange_params.func = exchange_mtu_cb;
    int err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err && print_log) LOG_ERR("MTU request failed (err %d).", err);
}

// Connection params
static void on_conn_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    double ci = interval * 1.25;
	uint16_t sto = timeout * 10;
	if (print_log) printk(" - Connection parameters set to %.2f ms, %d, %d ms.\n", ci, latency, sto);

    // PASO 3: Actualizar Data Length
    struct bt_conn_le_data_len_param g_data_len_param = {
		.tx_max_len = data_length,
		.tx_max_time = tx_time_us,
	};

    int err = bt_conn_le_data_len_update(conn, &g_data_len_param);
    if (err && print_log) LOG_ERR("DL request failed (err %d).", err);
}

// PHY
static void on_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
    if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_1M) {if (print_log) printk(" - PHY set to 1M\n");}
    if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_2M) {if (print_log) printk(" - PHY set to 2M\n");}

    // PASO 2: ACTUALIZAR EL CONNECTION INTERVAL
    struct bt_le_conn_param *g_conn_param = BT_LE_CONN_PARAM(
		conn_interv_N,    // Min Connection Interval = Value x 1.25ms
		conn_interv_N,    // Min Connection Interval = Value x 1.25ms
		latency_N,        // Latency (Number of connection intervals the peripheral can skip)
		timeout_N         // Time that the central will wait before disconnecting (ms) = Value x 10ms
	);
    int err = bt_conn_le_param_update(conn, g_conn_param);
    if (err && print_log) LOG_ERR("Connection parameters request failed (err %d).", err);
}

// Al realizar la conexion con un periferico
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

    // PASO 1: Iniciar la cadena de configuración actualizando el PHY
    int err;
    if (phy_2M) err = bt_conn_le_phy_update(conn, g_phy_param_2M);
    else err = bt_conn_le_phy_update(conn, g_phy_param_1M);
    if (err && print_log) LOG_ERR("PHY request failed (err %d).\n", err);
}

// Desconexion de un periferico
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    int idx = get_conn_index(conn);
    if (print_log) printk("[C] Peripheral %d - %s disconnected (0x%02x).*****************************************\n", idx, addr, reason);
    bt_conn_ctx_free(&conns_ctx_lib, conn);

    sync_pn[idx] = false;
    tpn[idx] = 0;

    if (scan_on) bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
}

// Callbacks eventos BLE
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .le_phy_updated = on_phy_updated,
    .le_param_updated = on_conn_param_updated,
    .le_data_len_updated = on_data_len_updated
};

// FUNCIONES UART ====================================================
// // Envio de datos - Estructura: [START][LEN][TIPO][IDX][PAYLOAD][END]
// void send_uart_packet(uint8_t msg_type, uint8_t p_idx, const uint8_t *payload, uint16_t len)
// {
//     static uint8_t tx_buf[UART_TX_BUF_SIZE];
//     if (len + 6 > UART_TX_BUF_SIZE) {
//         if (print_log) LOG_ERR("UART packet too big to send (%u bytes).", len);
//     } else {
//         tx_buf[0] = PKT_START_BYTE;
//         tx_buf[1] = (uint8_t)len;
//         tx_buf[2] = msg_type;
//         tx_buf[3] = p_idx;
//         memcpy(&tx_buf[4], payload, len);
//         tx_buf[4 + len] = PKT_END_BYTE;

//         // Send the packet
//         for (int i = 0; i < (6 + len); i++) uart_poll_out(uart_dev, tx_buf[i]);
//     }
// }

// Thread dedicado para TX UART (prioridad media)
// static void uart_tx_thread(void) {
//     struct uart_packet pkt;
//     while (1) {
//         // Espera hasta que haya paquete en la cola
//         k_msgq_get(&uart_tx_queue, &pkt, K_FOREVER);
//         // enviar sin prisa (no afecta BLE)
//         send_uart_packet(pkt.msg_type, pkt.p_idx, pkt.payload, pkt.len);
//     }
// }

// // Send data over BLE -> write characteristic
// void ble_cmd_write(uint8_t p_idx, const uint8_t *command_data, uint8_t len)
// {
//     int err = 1;
//     // "Broadcast" message (message for all peripherals- one by one)
//     if (p_idx == 0xFF) {
//         for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
//             const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, i);
//             if (ctx) {
//                 err = my_imus_write_command(ctx->conn, ctx->data, command_data, len, on_write_completed);
//                 bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
//             }
//         }
//     } else {
//         // Message to a specific peripheral
//         const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, p_idx);
//         if (ctx) {
//             err = my_imus_write_command(ctx->conn, ctx->data, command_data, len, on_write_completed);
//             bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
//         }
//     }
//     if (print_log) printk("[C] BLE-TX CMD[%02X:%uB] to P[%02X]\n", command_data[0], len, p_idx);
//     if(err != 0) LOG_ERR("Error %d in Write Operation", err);
// }

// // Ejecucion de comando
// static void handle_uart_command(const uint8_t *pkt, uint8_t lnn)
// {
//     static uint8_t tx_w[UART_RX_BUF_SIZE] = {0};
//     memcpy(tx_w, pkt, lnn); // [LenPayload][CMD][PAYLOAD]
//     uint8_t len = tx_w[0], cmd_type = tx_w[1];
//     uint8_t *payload = &tx_w[2];

//     // printk("%u - ", lnn);
//     // for (int i = 0; i < lnn; i++) printk("%02X%s", tx_w[i], (i + 1 == lnn) ? "\n":" ");

//     switch (cmd_type) {
//         // Commands to the Central-Dongle ============================================
//         case CMD_TYPE_SYNC_ON:
//             sync_protocol = true;
//             if (print_log) printk("[C] Sync protocol ON.\n");
//             break;

//         case CMD_TYPE_SYNC_OFF:
//             sync_protocol = false;
//             if (print_log) printk("[C] Sync protocol OFF.\n");
//             break;

//         case CMD_TYPE_TX_UART_ON:
//             send_uart = true;
//             if (print_log) printk("[C] UART Send Recieved Pkts ON.\n");
//             break;
        
//         case CMD_TYPE_TX_UART_OFF:
//             send_uart = false;
//             if (print_log) printk("[C] UART Send Recieved Pkts OFF.\n");
//             break;

//         case CMD_TYPE_PRINT_ON:
//             print_log = true;
//             printk("[C] Print info ON.\n");
//             break;

//         case CMD_TYPE_PRINT_OFF:
//             print_log = false;
//             printk("[C] Print info OFF.\n");
//             break;

//         case CMD_TYPE_START_SCAN:
//             if (print_log) printk("[C] Start Scan.\n");
//             bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
//             scan_on = true;
//             break;

//         case CMD_TYPE_STOP_SCAN:
//             if (print_log) printk("[C] Scan Stop.\n");
//             bt_scan_stop();
//             scan_on = false;
//             break;

//         case CMD_TYPE_SAMP_CTRL_P:
//             // In any case if the new value is wrong we keep the previus one
//             // Fs:2B, SXP:1B, delta_us:4B, start_delay_ms:2B = 9B
//             if (len == 9) {
//                 uint16_t n;     // Temporal variable to read 2B values

//                 // 1. Fs
//                 n = sys_get_le16(payload);
//                 if (n > 0) fs_hz = n;
//                 // 2. SXP
//                 if(payload[2] > 0) samples_x_pkt = payload[2];
//                 // 3. delta
//                 delta_us = sys_get_le32(&payload[3]);
//                 // 4. Start Delay ms
//                 start_delay_ms = sys_get_le16(&payload[7]);

//                 if (print_log) printk("[C] Update Fs %uHz, SXP %u, delta %uus, SD %ums\n",
//                     fs_hz, samples_x_pkt, delta_us, start_delay_ms);
//             }
//             break;

//         case CMD_UPDATE_PARAMS:
//             // In any case if the new value is wrong we keep the previus one
//             // PHY:1B, CI:2B, L:2B, STO:2B(ms), DL:2B, TR:2B(us) = 11B
//             if (len == 11) {
//                 uint16_t n;     // Temporal variable to read 2B values

//                 // 1. PHY
//                 if (payload[0] == 0x01) phy_2M = false;
//                 if (payload[0] == 0x02) phy_2M = true;
                
//                 // 2. Connection Interval
//                 n = sys_get_le16(&payload[1]);
//                 if (n >= 6 &&  n <= 3200) conn_interv_N = n;

//                 // 3. Latency
//                 n = sys_get_le16(&payload[3]);
//                 if (n >= 0) latency_N = n;
                
//                 // 4. Supervision time out (REVISAR DESPUES, DEBE DE CONSIDERA LATENCY)
//                 timeout_N = sys_get_le16(&payload[5]);

//                 // Data Length
//                 n = sys_get_le16(&payload[7]);
//                 if (n >= 27 &&  n <= 251) data_length = n; 

//                 // Tx/Rx time = TIEMPO QUE PUEDE TRANSMITIR O ESCUCHAR EL RADIO!!!
//                 // EL tiempo requerido depende de los datos a enviar y PHY.
//                 n = sys_get_le16(&payload[9]);
//                 if (n >= 328 &&  n <= 17040) tx_time_us = n;

//                 if (print_log) printk("[C] Update PHY=%s, CI=%.2fms (%u), L=%u, STO=%ums, DL=%u, T=%uus.\n",
//                     phy_2M ? "2M" : "1M", conn_interv_N * 1.25, conn_interv_N, latency_N, timeout_N*10, 
//                     data_length, tx_time_us);
//             }
//             break;

//         case CMD_TYPE_DISCONNECT:
//             if (payload[0] == 0xFF) {
//                 // if (print_log) printk("[C] Disconnect ALL Peripherals.\n");
//                 for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
//                     const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, i);
//                     if (ctx) {
//                         bt_conn_disconnect(ctx->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
//                         bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
//                     }
//                 }
//             } else {
//                 // if (print_log) printk("[C] Disconnect [Peripheral %u].\n", payload[0]);
//                 const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, payload[0]);
//                 if (ctx) {
//                     bt_conn_disconnect(ctx->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
//                     bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
//                 }
//             }
//             break;

//         // Commands to the Peripherals ===============================================
//         case CMD_TYPE_SEND_CMD:
//             // tx_w = [LenPayload:0][CMD:1][PAYLOAD:2->N], [PAYLOAD]=[idx:2][data:3->N]
//             // ble_cmd_write(uint8_t p_idx, const uint8_t *command_data, uint8_t len)
//             if (len >= 2) ble_cmd_write(tx_w[2], &tx_w[3], len - 1);
//             break;

//         case CMD_STOP_PKT:
//             printk("Stop-pkt\n");
//             break;

//         default:
//             if (print_log) LOG_WRN("Unknown UART CMD 0x%02X", cmd_type);
//             break;
//     }
// }

// // Hilo de recepcion de UART
// static void uart_rx_thread(void)
// {
//     uint8_t rx_pkt[UART_RX_BUF_SIZE] = {0}, pos = 0, c;
//     while (true) {
//         // 1. Intenta leer un byte. Si no hay, duerme y vuelve a intentarlo.
//         if (uart_poll_in(uart_dev, &c)) {
//             k_sleep(K_MSEC(1000));
//             continue;
//         }
//         if (c == PKT_START_BYTE) rx_pkt[pos++] = c;
//         else {
//             if (c != PKT_END_BYTE) rx_pkt[pos++] = c;
//             else {
//                 //PKT Completo [Start][LenPayload][CMD][PAYLOAD][End]
//                 rx_pkt[pos] = c;
//                 // for (int i = 0; i <= pos; i++) printk("%02X%s", rx_pkt[i], (i == pos) ? "\n":" ");
//                 // pos--;
//                 handle_uart_command(&rx_pkt[1], --pos);
//                 // Reset para el siguiente mensaje
//                 pos = 0;
//             }
//         }
//     }
// }

// ===================================================================
// Inicialización
// ===================================================================
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
    
    // SBLE - ACL ********************************************************
    // calculos para sincronizacion
    ts_us = 1000000 / fs_hz;        // Sampling period in us
    ci_us = conn_interv_N * 1250;   // Connection interval in us
    cmd_tx[0] = CMD_SYNC_START;

    err = bt_enable(NULL);
    if (err) return 0;
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
    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_IMUS);
    if (err) { return 0; }
    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err) { return 0; }
    k_msleep(3000);
    if (scan_on) bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);

    // BLE Broadcast ****************************************************
    printk("Central - Dongle. [CEL=%dus]\n\n", CONFIG_BT_CTLR_SDC_MAX_CONN_EVENT_LEN_DEFAULT);
    return 0;
}