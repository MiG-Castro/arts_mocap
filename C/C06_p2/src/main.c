/*
 * Central para el servicio IMU - Arquitectura Asíncrona
 * Funcional para multiconexion - Elemento clave "conn ctx" !!!
 * Central hace todas las solicitudes de actualizacion de parametros
 * Comunicacion UART
 * - Hilo para Rx & Hilo para Tx - NO CHECKSUM!!
 * Reestructuración de uso de variables globales para ser congruentes con las definiciones de las estructuras de BLE
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
#include <zephyr/shell/shell_uart.h>
#include <zephyr/sys/byteorder.h>
#include <bluetooth/conn_ctx.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/time_units.h>
#include "my_imu_ble_service.h"

// LOG ===============================================================
#define LOG_MODULE_NAME Central
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

// UART ==============================================================
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

#define UART_RX_BUF_SIZE 50
#define UART_TX_BUF_SIZE 100

// Hilo de recepcion
static void uart_rx_thread(void);
K_THREAD_DEFINE(uart_rx_tid, 1024, uart_rx_thread, NULL, NULL, NULL, 7, 0, 0);

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

// --- Protocolo UART ---
#define PKT_START_BYTE 0x7E
#define PKT_END_BYTE   0x7F

// COMMANDS
// To the central
#define CMD_TYPE_TX_UART_OFF 0x16
#define CMD_TYPE_TX_UART_ON  0x17
#define CMD_TYPE_PRINT_OFF   0x18
#define CMD_TYPE_PRINT_ON    0x19
#define CMD_TYPE_START_SCAN  0x20
#define CMD_TYPE_STOP_SCAN   0x21
#define CMD_TYPE_DISCONNECT  0x22
#define CMD_UPDATE_PARAMS    0x23
// To the peripherals
#define CMD_UPDATE_TIMER     0x24
#define CMD_STOP_PKT         0x25
#define CMD_TYPE_SEND_CMD    0x30
// others commands: 
// To the PC
#define MSG_TYPE_SENSOR_DATA 0x60
#define MSG_TYPE_EVENT_DATA  0x70

// Estructura para construir paquetes desde bytes recibidos
static struct {
    uint8_t buffer[UART_RX_BUF_SIZE];
    uint8_t pos;
    uint8_t len;
    uint8_t type;
    enum {
        STATE_WAIT_START,
        STATE_READ_LEN,
        STATE_READ_TYPE,
        STATE_READ_PAYLOAD,
        STATE_READ_END
    } state;
} pkt_builder = {.state = STATE_WAIT_START, .pos = 0};

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
static uint16_t conn_interv_N = 10, latency_N = 0, timeout_N = 1000;
// Values to set in the data length
static uint16_t data_length = 73, tx_time_us = 800; // DL = desired value + 7 
// Structure for the MTU (the MTU is defined in the prj.conf)
static struct bt_gatt_exchange_params exchange_params;

// Context manager for connections
BT_CONN_CTX_DEF(conns, MAX_CONNECTIONS, sizeof(struct bt_imus_client));
static int idx = -1; // Index of the connection

// Global variables for the system ==================================
// Control the prints and the Tx in the UART of the received data 
static bool print_log = true, send_uart = false;

// Test variables
static uint8_t counter_byte = 0, k_pktsP0 = 0, k_pktsP1 = 0, count_loopP0 = 0, count_loopP1 = 0;
static uint16_t time_ms16 = 0;
static int64_t t_us, uptime_ms;
static uint64_t cycles;

// Forward declarations =============================================
// BLE
static uint8_t on_gatt_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
void ble_cmd_write(uint8_t p_idx, const uint8_t *command_data, uint8_t len);
// UART
void send_uart_packet(uint8_t msg_type, uint8_t p_idx, const uint8_t *payload, uint16_t len);
static void handle_uart_command(uint8_t cmd_type, const uint8_t *payload, uint8_t len);

// ===================================================================
// Funciones ---------------------------------------------------------
// ===================================================================
// Obtener el índice de una conexión.
static int get_conn_index(struct bt_conn *conn)
{
    for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
        const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, i);
        if (ctx) {
            if (ctx->conn == conn) {return i;} // id in the context
        }
    }
    return -1; // Connection not registered
}

// Callbacks BL ======================================================
// Recepcion de ACK (for write operation)
void on_write_completed(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
    idx = get_conn_index(conn);
    if (err) LOG_ERR("P[%d] AKC err %u", idx, err);
    else LOG_INF("P[%d]ACK", idx);
}

// Callback for the received data via notify|indicate characteristics
static uint8_t on_gatt_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
    struct bt_imus_client *imus_client = bt_conn_ctx_get(&conns_ctx_lib, conn);
    if (!imus_client) return BT_GATT_ITER_STOP;

    if (!data) {
        // printk("[Peripheral %d] NULL handle 0x%04x\n", idx, params->value_handle);
        bt_conn_ctx_release(&conns_ctx_lib, imus_client);
        return BT_GATT_ITER_STOP;
    }

    // print the time of "arrive"
    // printk("%lld\n", k_cyc_to_us_floor64(k_cycle_get_64()));
    // uptime_ms = k_uptime_get();
    // cycles = k_cycle_get_64();
    // uptime_ms = k_cyc_to_us_floor64(cycles);
    idx = get_conn_index(conn);

    // uptime_ms = k_uptime_get();
    // counter_byte = *((uint8_t *)data);
    // printk("%lld-%d\n", uptime_ms, counter_byte);

    const uint8_t *buffer = data;
    counter_byte = buffer[0];
    k_pktsP0 = buffer[length - 1];
    time_ms16 = sys_get_le16(&buffer[1]);

    LOG_INF("Rx: P%d[%d-%u]%u, %u", idx, counter_byte, length, time_ms16, k_pktsP0);

    // if (idx == 0) k_pktsP0++;
    // if (idx == 1) k_pktsP1++;
    // static uint8_t ble_cmd = 0xAA;
    // if (k_pktsP0 == 10) {
    //     k_pktsP0 = 0;
    //     if (count_loopP0 < 10) {
    //         // uptime_ms = k_uptime_get();
    //         // Send the command star to the node
    //         my_imus_write_command(conn, imus_client, &ble_cmd, 1, on_write_completed);
    //         // my_imus_write_command(conn, imus_client, 0xAA, 1, on_write_completed);
    //         // printk("%lld, %uC\n", uptime_ms, count_loop);
    //         LOG_INF("TxP%d-0xAA, C%u", idx, count_loopP0);
    //         count_loopP0++;
    //     } else {
    //         // uptime_ms = k_uptime_get();
    //         // printk("%lld\n", uptime_ms);
    //         LOG_INF("P%d-FIN", idx);
    //     }
    // }

    // if (k_pktsP1 == 10) {
    //     k_pktsP1 = 0;
    //     if (count_loopP1 < 10) {
    //         // uptime_ms = k_uptime_get();
    //         // Send the command star to the node
    //         my_imus_write_command(conn, imus_client, &ble_cmd, 1, on_write_completed);
    //         // my_imus_write_command(conn, imus_client, 0xAA, 1, on_write_completed);
    //         // printk("%lld, %uC\n", uptime_ms, count_loop);
    //         LOG_INF("TxP%d-0xAA, C%u", idx, count_loopP1);
    //         count_loopP1++;
    //     } else {
    //         // uptime_ms = k_uptime_get();
    //         // printk("%lld\n", uptime_ms);
    //         LOG_INF("P%d-FIN", idx);
    //     }
    // }

    uint32_t packet_num = sys_get_le32(data);
    if (params->value_handle == imus_client->sensordata_handle) {
        if (print_log) printk("[Peripheral %d] Notify #%u\n", idx, packet_num);
        // if (send_uart) send_uart_packet(MSG_TYPE_SENSOR_DATA, (uint8_t)idx, data, length);
        if (send_uart) {
            struct uart_packet pkt = {
                .msg_type = MSG_TYPE_SENSOR_DATA,
                .p_idx = idx,
                .len = length
            };
            memcpy(pkt.payload, data, length);
            // Encolar sin bloquear (máximo 20 paquetes)
            k_msgq_put(&uart_tx_queue, &pkt, K_NO_WAIT);
        }
    } else if (params->value_handle == imus_client->exercisedetection_handle) {
        // CORRECCIÓN: Imprimir el índice en un log separado.
        if (print_log) printk("[Peripheral %d] Indicate #%u\n", idx, packet_num);
        if (send_uart) send_uart_packet(MSG_TYPE_EVENT_DATA, (uint8_t)idx, data, length);
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
        if (print_log) LOG_ERR("[Peripheral %d] Failed to assign handles.", idx);
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }
    
    err = my_imus_subscribe_sensordata(conn, imus_client, on_gatt_notify);
    if (err && print_log) { LOG_ERR("[Peripheral %d] Failed to subscribe to SensorData (err %d).", idx, err); }
    
    err = my_imus_subscribe_exercisedetection(conn, imus_client, on_gatt_notify);
    if (err && print_log) { LOG_ERR("[Peripheral %d] Failed to subscribe to ExerciseDetection (err %d).", idx, err); }

    bt_gatt_dm_data_release(dm);
    if (print_log) printk("[Peripheral %d] subscription completed.\n", idx);
    bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
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

    // Saltamos todo y pasamos directo a al descubrimiento de servicios
    // struct bt_imus_client *imus_client = bt_conn_ctx_get(&conns_ctx_lib, conn);
    // if (!imus_client) return;
    // int gatt_err = bt_gatt_dm_start(conn, BT_UUID_IMUS, &discovery_cb, imus_client);
    // if (gatt_err && print_log) LOG_ERR("Failed to start discovery (err %d).", gatt_err);
    // bt_conn_ctx_release(&conns_ctx_lib, imus_client);
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
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (hci_err) {
        if (print_log) LOG_ERR("Failed connection to %s (err 0x%02x).", addr, hci_err);
        return;
    }
    if (print_log) printk("[C] Connected to: %s.\n", addr);
    // Stop the scanning to avoid interruptions during the configuration process
    bt_scan_stop();

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
    if (print_log) printk("[C] Peripheral %d - %s disconnected (0x%02x).\n", idx, addr, reason);
    bt_conn_ctx_free(&conns_ctx_lib, conn);

    // EN PROCESO ********************************************************************************
    printk("Live connections: %d\n", bt_conn_ctx_count(&conns_ctx_lib));
    if (bt_conn_ctx_count(&conns_ctx_lib) < MAX_CONNECTIONS) printk("hay espacio");
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
// Envio de datos - Estructura: [START][LEN][TIPO][IDX][PAYLOAD][END]
void send_uart_packet(uint8_t msg_type, uint8_t p_idx, const uint8_t *payload, uint16_t len)
{
    static uint8_t tx_buf[UART_TX_BUF_SIZE];
    if (len + 6 > UART_TX_BUF_SIZE) {
        if (print_log) LOG_ERR("UART packet too big to send (%u bytes).", len);
    } else {
        tx_buf[0] = PKT_START_BYTE;
        tx_buf[1] = (uint8_t)len;
        tx_buf[2] = msg_type;
        tx_buf[3] = p_idx;
        memcpy(&tx_buf[4], payload, len);
        tx_buf[4 + len] = PKT_END_BYTE;

        // Send the packet
        for (int i = 0; i < (6 + len); i++) uart_poll_out(uart_dev, tx_buf[i]);
    }
}

// Thread dedicado para TX UART (prioridad media)
static void uart_tx_thread(void) {
    struct uart_packet pkt;
    while (1) {
        // Espera hasta que haya paquete en la cola
        k_msgq_get(&uart_tx_queue, &pkt, K_FOREVER);
        // enviar sin prisa (no afecta BLE)
        send_uart_packet(pkt.msg_type, pkt.p_idx, pkt.payload, pkt.len);
    }
}

// Send data over BLE -> write characteristic
void ble_cmd_write(uint8_t p_idx, const uint8_t *command_data, uint8_t len)
{
    int err = 1;
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
    if(err != 0) LOG_ERR("Error %d in Write Operation", err);
}

// Ejecucion de comando
static void handle_uart_command(uint8_t cmd_type, const uint8_t *payload, uint8_t len)
{
    uint8_t tx_w[UART_RX_BUF_SIZE] = {0};
    switch (cmd_type) {

        // Commands to the Central-Dongle ============================================
        case CMD_TYPE_TX_UART_ON:
            send_uart = true;
            if (print_log) printk("[C] UART Send Recieved Pkts ON.\n");
            break;
        
        case CMD_TYPE_TX_UART_OFF:
            send_uart = false;
            if (print_log) printk("[C] UART Send Recieved Pkts OFF.\n");
            break;

        case CMD_TYPE_PRINT_ON:
            print_log = true;
            printk("[C] Print info ON.\n");
            break;

        case CMD_TYPE_PRINT_OFF:
            print_log = false;
            printk("[C] Print info OFF.\n");
            break;

        case CMD_TYPE_START_SCAN:
            if (print_log) printk("[C] Start Scan.\n");
            bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
            scan_on = true;
            break;

        case CMD_TYPE_STOP_SCAN:
            if (print_log) printk("[C] Scan Stop.\n");
            bt_scan_stop();
            scan_on = false;
            break;

        case CMD_UPDATE_PARAMS:
            // In any case if the new value is wrong we keep the previus one
            // PHY:1B, CI:2B, L:2B, STO:2B(ms), DL:2B, TR:2B(us) = 11B
            if (len == 11) {
                uint16_t n;     // Temporal variable to read 2B values

                // 1. PHY
                if (payload[0] == 0x01) phy_2M = false;
                if (payload[0] == 0x02) phy_2M = true;
                
                // 2. Connection Interval
                n = sys_get_le16(&payload[1]);
                if (n >= 6 &&  n <= 3200) conn_interv_N = n;

                // 3. Latency
                n = sys_get_le16(&payload[3]);
                if (n >= 0) latency_N = n;
                
                // 4. Supervision time out
                n = sys_get_le16(&payload[5]);
                uint16_t cal_tout = DIV_ROUND_UP(((1 + latency_N) * conn_interv_N * 5), 4);
                if (n >= cal_tout) timeout_N = n;
                else timeout_N = cal_tout;

                // Data Length
                n = sys_get_le16(&payload[7]);
                if (n >= 27 &&  n <= 251) data_length = n; 

                // Tx/Rx time = TIEMPO QUE PUEDE TRANSMITIR O ESCUCHAR EL RADIO!!!
                // EL tiempo requerido depende de los datos a enviar y PHY.
                // Recomendacion: No mover y dejar en 2120us
                n = sys_get_le16(&payload[9]);
                if (n >= 328 &&  n <= 17040) tx_time_us = n;

                if (print_log) printk("[C] Update PHY=%s, CI=%.2fms (%u), L=%u, STO=%ums, DL=%u, T=%uus.\n",
                    phy_2M ? "2M" : "1M", conn_interv_N*1.25, conn_interv_N, latency_N, timeout_N*10, 
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
        case CMD_TYPE_SEND_CMD:
            if (len >= 2) {
                ble_cmd_write(payload[0], &payload[1], 1);
                if (print_log) printk("[C] Sending 0x%02X to %u.\n", payload[1], payload[0]);
            }
            break;

        case CMD_UPDATE_TIMER:
            if (len >= 5) {
                memcpy(tx_w, &cmd_type, 1);
                memcpy(&tx_w[1], &payload[1], 4);
                ble_cmd_write(payload[0], tx_w, 5);
;
                if (print_log) printk("[C] Sending 0x24 - Timer: %u Hz, %u ms to %u.\n",
                    sys_get_le16(&payload[1]), sys_get_le16(&payload[3]), payload[0]);
            } 
            break;

        case CMD_STOP_PKT:
            if (len >= 5) {
                memcpy(tx_w, &cmd_type, 1);
                memcpy(&tx_w[1], &payload[1], 4);
                ble_cmd_write(payload[0], tx_w, 5);
                
                if (print_log) printk("[C] Sending 0xBB - StopPkt: %u to %u.\n",
                    sys_get_le32(&payload[1]), payload[0]);
            }
            break;

        default:
            if (print_log) LOG_WRN("Unknown UART CMD 0x%02X", cmd_type);
            break;
    }
}

// Hilo de recepcion de datos
static void uart_rx_thread(void)
{
    uint8_t c;
    while (1) {
        // Leer byte por byte en modo polling
        int ret = uart_poll_in(uart_dev, &c);
        if (ret == 0) {
            // Byte recibido, procesar con máquina de estados
            switch (pkt_builder.state) {
                case STATE_WAIT_START:
                    if (c == PKT_START_BYTE) {
                        pkt_builder.state = STATE_READ_LEN;
                    }
                    break;

                case STATE_READ_LEN:
                    pkt_builder.len = c;
                    if (pkt_builder.len > sizeof(pkt_builder.buffer)) {
                        LOG_WRN("UART: Invalid len %u", pkt_builder.len);
                        pkt_builder.state = STATE_WAIT_START;
                    } else {
                        pkt_builder.state = STATE_READ_TYPE;
                    }
                    break;

                case STATE_READ_TYPE:
                    pkt_builder.type = c;
                    pkt_builder.pos = 0;
                    if (pkt_builder.len == 0) {
                        pkt_builder.state = STATE_READ_END;
                    } else {
                        pkt_builder.state = STATE_READ_PAYLOAD;
                    }
                    break;

                case STATE_READ_PAYLOAD:
                    pkt_builder.buffer[pkt_builder.pos++] = c;
                    if (pkt_builder.pos == pkt_builder.len) {
                        pkt_builder.state = STATE_READ_END;
                    }
                    break;

                case STATE_READ_END:
                    if (c == PKT_END_BYTE) {
                        handle_uart_command(pkt_builder.type, 
                                          pkt_builder.buffer, 
                                          pkt_builder.len);
                    } else {
                        LOG_WRN("UART: Wrong end byte 0x%02X", c);
                    }
                    pkt_builder.state = STATE_WAIT_START;
                    break;
            }
        } else {
            k_sleep(K_MSEC(1000));
        }
    }
}

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
    k_msleep(100);
    if (scan_on) bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);

    printk("Central - Dongle. [CEL=%dus]\n", CONFIG_BT_CTLR_SDC_MAX_CONN_EVENT_LEN_DEFAULT);
    return 0;
}