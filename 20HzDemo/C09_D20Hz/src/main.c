/*
 * Central para el servicio IMU - Arquitectura Asíncrona
 * Funcional para multiconexion - Elemento clave "conn ctx" !!!
 * Central hace todas las solicitudes de actulizacion de parametros
 * Comunicacion UART
 * - Hilo para Rx & Hilo para Tx - Sin checksum ni maquina de estados en hilo Rx!!
 * Reestructuración de uso de variables globales para ser congruente con las definiciones|estructuras de BLE
 * 
 * Adicion de rutina de sincronizacion por BIG (Broadcast ISO Grup)
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
#define LOG_MODULE_NAME XCD
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

// UART ==============================================================
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

#define UART_RX_BUF_SIZE 255
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
#define PKT_ESC_BYTE   0x7D

// COMMANDS
// To the peripherals
#define CMD_CTRL_NOTY_INDI   0x00
#define CMD_START_STOP_TX    0x01
#define CMD_STOP_PKT         0x02
#define CMD_TYPE_SEND_CMD    0x10
// To the central
#define CMD_TYPE_TX_UART_OFF 0x16
#define CMD_TYPE_TX_UART_ON  0x17
#define CMD_TYPE_PRINT_OFF   0x18
#define CMD_TYPE_PRINT_ON    0x19
#define CMD_TYPE_START_SCAN  0x20
#define CMD_TYPE_STOP_SCAN   0x21
#define CMD_TYPE_DISCONNECT  0x22
#define CMD_UPDATE_PARAMS    0x23
#define CMD_TYPE_SAMP_CTRL_P 0x24
// others commands: 
// To the PC
#define MSG_TYPE_SENSOR_DATA 0x60
#define MSG_TYPE_EVENT_DATA  0x70

// ===================================================================
// BLE configuration and variables (& default values)
// ===================================================================
#define MAX_CONNECTIONS CONFIG_BT_MAX_CONN
static bool scan_on = true;     // To activate/deactivate the scan process
static uint16_t scan_window_N = 16, scan_interval_N = 80; // u = 0.625ms 20-160=12.2-100
static bool phy_2M = false;     // True = Set PHY to 2M, False = Set PHY to 1M
// Structures to set the PHY
static struct bt_conn_le_phy_param *g_phy_param_2M = BT_CONN_LE_PHY_PARAM_2M;
static struct bt_conn_le_phy_param *g_phy_param_1M = BT_CONN_LE_PHY_PARAM_1M;

// Connection parameters 
// CI: N x 1.25ms, latency = N x CI, timeout = N x 10ms
static uint16_t conn_interv_N = 8, latency_N = 5, timeout_N = 100;
// Values to set in the data length
static uint16_t data_length = 30, tx_time_us = 500; // DL = desired value + 7B of headers
// Structure for the MTU (the MTU is defined in the prj.conf)
static struct bt_gatt_exchange_params exchange_params;

// Context manager for connections
BT_CONN_CTX_DEF(conns, MAX_CONNECTIONS, sizeof(struct bt_imus_client));
static int idx = -1; // Index of the connection

/* Direcciones "originales" (Estan en big endian)
FA:53:60:43:10:2F (case blanco)
FA:28:99:3C:D4:86 (case blanco)
DC:67:32:1F:20:84 (proto - lsm)
C0:B0:7C:B5:F8:1B (caja pruebas)
EC:1B:2C:DA:ED:9B (caja pruebas)
CC:C0:BE:38:8D:D0 (caja pruebas)
D3:58:40:2B:37:DA (caja pruebas)
DD:29:0E:DD:B7:54 Zeydel, nuevo
CD:C4:BA:F6:FB:C3 nuevo
*/
// Array con las direcciones de TODOS los dispositivos permitidos
// DEBEN DE ESTAR EN LITTLE ENDIAN!!!
static const bt_addr_le_t sensor_wl[] = {
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x2F, 0x10, 0x43, 0x60, 0x53, 0xFA}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x86, 0xD4, 0x3C, 0x99, 0x28, 0xFA}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x84, 0x20, 0x1F, 0x32, 0x67, 0xDC}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x1B, 0xF8, 0xB5, 0x7C, 0xB0, 0xC0}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x9B, 0xED, 0xDA, 0x2C, 0x1B, 0xEC}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0xD0, 0x8D, 0x38, 0xBE, 0xC0, 0xCC}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0xDA, 0x37, 0x2B, 0x40, 0x58, 0xD3}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0x54, 0xB7, 0xDD, 0x0E, 0x29, 0xDD}}},
    {.type = BT_ADDR_LE_RANDOM, .a = { .val = {0xC3, 0xFB, 0xF6, 0xBA, 0xC4, 0xCD}}},
};

// Global variables for the system ==================================
// Control the prints and the Tx in the UART of the received data 
static bool print_log = true, send_uart = false;

// BLE ISO BROADCAST ==============================================================
#define BIS_ISO_CHAN_COUNT 1
#define BIG_SDU_INTERVAL_US      (200000)
#define BUF_ALLOC_TIMEOUT_US     (250000)
#define BIG_TERMINATE_TIMEOUT_US (10 * USEC_PER_SEC)
#define SDU_SIZE CONFIG_BT_ISO_TX_MTU

uint32_t iso_seq_n = 0;
uint8_t iso_data[SDU_SIZE] = {0};

NET_BUF_POOL_FIXED_DEFINE(
	bis_tx_pool,
	BIS_ISO_CHAN_COUNT,
	BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
	CONFIG_BT_CONN_TX_USER_DATA_SIZE,
	NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_term, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_iso_data, CONFIG_BT_ISO_TX_BUF_COUNT, CONFIG_BT_ISO_TX_BUF_COUNT);

// Forward declarations =============================================
// BLE
static uint8_t on_gatt_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
void ble_cmd_write(uint8_t p_idx, const uint8_t *command_data, uint8_t len);
// UART
void send_uart_packet(uint8_t msg_type, uint8_t p_idx, const uint8_t *payload, uint16_t len);
static void handle_uart_command(const uint8_t *pkt, uint8_t lnn);

// ===================================================================
// Funciones
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
    return -1; // Connection context not registered
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

// Callback for the received data via notify|indicate characteristics
static uint8_t on_gatt_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
    struct bt_imus_client *imus_client = bt_conn_ctx_get(&conns_ctx_lib, conn);
    if (!imus_client) return BT_GATT_ITER_STOP;

    if (!data) {
        // printk("[P%d] NULL handle 0x%04x\n", idx, params->value_handle);
        bt_conn_ctx_release(&conns_ctx_lib, imus_client);
        return BT_GATT_ITER_STOP;
    }

    idx = get_conn_index(conn);
    const uint8_t *b = data;

    if (length > 0) {
        uint32_t packet_num = sys_get_le32(b);
        // uint8_t packet_num = b[0];

        if (params->value_handle == imus_client->sensordata_handle) {
            if (print_log) LOG_INF("RX-N[%uB]-P[%d:%u]", length, idx, packet_num);
        }

        if (params->value_handle == imus_client->exercisedetection_handle) {
            if (print_log) LOG_INF("RX-I[%uB]-P[%d:%u]", length, idx, packet_num);
        }

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
    if (scan_on) bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
}

static struct bt_gatt_dm_cb discovery_cb = {.completed = discovery_complete};

// ===================================================================
// Callbacks de la Cadena de Configuración y Conexión
// ===================================================================
// Connection params
static void on_conn_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    double ci = interval * 1.25;
	uint16_t sto = timeout * 10;
	if (print_log) printk(" - Connection parameters set to %.2f ms, %d, %d ms.\n", ci, latency, sto);

    // PASO FINAL: Iniciar el descubrimiento de servicios
    struct bt_imus_client *imus_client = bt_conn_ctx_get(&conns_ctx_lib, conn);
    if (!imus_client) return;

    int gatt_err = bt_gatt_dm_start(conn, BT_UUID_IMUS, &discovery_cb, imus_client);
    if (gatt_err && print_log) LOG_ERR("Failed to start discovery (err %d).", gatt_err);
    bt_conn_ctx_release(&conns_ctx_lib, imus_client);
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
    if (print_log) printk("[C] Peripheral %d - %s disconnected (0x%02x).*******************************\n",idx, addr, reason);
    bt_conn_ctx_free(&conns_ctx_lib, conn);

    if (scan_on) bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
}

// Callbacks eventos BLE
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .le_phy_updated = on_phy_updated,
    .le_param_updated = on_conn_param_updated,
};

// Callbacks ISO Events ============================================================
static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO C[%p] connected\n", chan);
    if (scan_on) bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
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

static struct bt_iso_chan_io_qos iso_tx_qos = {.sdu = SDU_SIZE, .rtn = 0, .phy = BT_GAP_LE_PHY_2M,};
static struct bt_iso_chan_qos bis_iso_qos = {.tx = &iso_tx_qos,};
static struct bt_iso_chan bis_iso_chan[] = {{.ops = &iso_ops, .qos = &bis_iso_qos,},};
static struct bt_iso_chan *bis[] = {&bis_iso_chan[0],};

static struct bt_iso_big_create_param big_create_param = {
	.num_bis = BIS_ISO_CHAN_COUNT,      
	.bis_channels = bis,                
	.interval = BIG_SDU_INTERVAL_US,    /* in microseconds */
	.latency = 205,                     /* in milliseconds */
	.packing = 0,                       /* 0 - sequential, 1 - interleaved */
	.framing = 0,                       /* 0 -   unframed, 1 - framed */
    .encryption = false,
};

// FUNCIONES UART ====================================================
void send_uart_byte(uint8_t byte) {
    // Si el byte es especial, lo escapamos
    if (byte == PKT_START_BYTE || byte == PKT_END_BYTE || byte == PKT_ESC_BYTE) {
        uart_poll_out(uart_dev, PKT_ESC_BYTE);       // Enviar ESC
        uart_poll_out(uart_dev, byte ^ 0x20);        // Enviar byte modificado
    } else {
        uart_poll_out(uart_dev, byte);               // Enviar normal
    }
}

// Envio de datos - Estructura: [START][LEN-PAYLOAD][TIPO][IDX][PAYLOAD][END]
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
    if (print_log) printk("[C] BLE-TX CMD[%02X:%uB] to P[%02X]\n", command_data[0], len, p_idx);
    if(err != 0) LOG_ERR("Error %d in Write Operation", err);
}

// Ejecucion de comando
static void handle_uart_command(const uint8_t *pkt, uint8_t lnn)
{
    static uint8_t tx_w[UART_RX_BUF_SIZE] = {0};
    memcpy(tx_w, pkt, lnn); // [LenPayload][CMD][PAYLOAD]
    uint8_t len = tx_w[0], cmd_type = tx_w[1];
    uint8_t *payload = &tx_w[2];

    // printk("%u - ", lnn);
    // for (int i = 0; i < lnn; i++) printk("%02X%s", tx_w[i], (i + 1 == lnn) ? "\n":" ");

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

        // case CMD_TYPE_SAMP_CTRL_P:
        //     // change the Samples x packet and the Sampling frequency 
        //     // Fs:1B, SXP:1B, PXISO:1B
        //     if (len == 3) {
        //         uint8_t n;
        //         n = payload[0];
        //         if (n > 0) fs_hz = payload[0];
        //         n = payload[1];
        //         if (n > 0) samples_x_pkt = payload[1];
        //         n = payload[2];
        //         if (n > 0) pkts_x_iso = payload[2];

        //         // update sdu-pkt
        //         iso_data[0] = fs_hz;
        //         iso_data[1] = samples_x_pkt;
        //         iso_data[2] = pkts_x_iso;

        //         if (print_log) printk("[C] Update Fs %uHz, SXP %u, PXI %u", fs_hz, samples_x_pkt, pkts_x_iso);
        //     }
        //     break;

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
                
                // 4. Supervision time out (REVISAR DESPUES, DEBE DE CONSIDERA LATENCY)
                timeout_N = sys_get_le16(&payload[5]);

                // Data Length
                n = sys_get_le16(&payload[7]);
                if (n >= 27 &&  n <= 251) data_length = n; 

                // Tx/Rx time = TIEMPO QUE PUEDE TRANSMITIR O ESCUCHAR EL RADIO!!!
                // EL tiempo requerido depende de los datos a enviar y PHY.
                n = sys_get_le16(&payload[9]);
                if (n >= 328 &&  n <= 17040) tx_time_us = n;

                if (print_log) printk("[C] Update PHY=%s, CI=%.2fms (%u), L=%u, STO=%ums, DL=%u, T=%uus.\n",
                    phy_2M ? "2M" : "1M", conn_interv_N * 1.25, conn_interv_N, latency_N, timeout_N*10, 
                    data_length, tx_time_us);
            }
            break;

        case CMD_TYPE_DISCONNECT:
            if (payload[0] == 0xFF) {
                // if (print_log) printk("[C] Disconnect ALL Peripherals.\n");
                for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
                    const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, i);
                    if (ctx) {
                        bt_conn_disconnect(ctx->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                        bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
                    }
                }
            } else {
                // if (print_log) printk("[C] Disconnect [Peripheral %u].\n", payload[0]);
                const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, payload[0]);
                if (ctx) {
                    bt_conn_disconnect(ctx->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                    bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
                }
            }
            break;

        // Commands to the Peripherals ===============================================
        // ble_cmd_write(uint8_t p_idx, const uint8_t *command_data, uint8_t len)
        // tx_w = [LenPayload:0][CMD:1][PAYLOAD:2->N]{[idx:2][CMD:3][Data:4->N]}
        
        case CMD_TYPE_SEND_CMD:
            bool cmd_ok = false;
            if (tx_w[3] == CMD_CTRL_NOTY_INDI && len == 3) cmd_ok = true;
            if (tx_w[3] == CMD_START_STOP_TX && len == 3) cmd_ok = true;
            if (tx_w[3] == CMD_STOP_PKT && len == 6) cmd_ok = true;

            if (cmd_ok) ble_cmd_write(tx_w[2], &tx_w[3], len - 1);
            break;

        default:
            if (print_log) LOG_WRN("Unknown UART CMD 0x%02X", cmd_type);
            break;
    }
}

// Hilo de recepcion de UART
static void uart_rx_thread(void)
{
    uint8_t rx_pkt[UART_RX_BUF_SIZE] = {0}, pos = 0, c;
    while (true) {
        // 1. Intenta leer un byte. Si no hay, duerme y vuelve a intentarlo.
        if (uart_poll_in(uart_dev, &c)) {
            k_sleep(K_MSEC(1000));
            continue;
        }
        if (c == PKT_START_BYTE) rx_pkt[pos++] = c;
        else {
            if (c != PKT_END_BYTE) rx_pkt[pos++] = c;
            else {
                //PKT Completo [Start][LenPayload][CMD][PAYLOAD][End]
                rx_pkt[pos] = c;
                // for (int i = 0; i <= pos; i++) printk("%02X%s", rx_pkt[i], (i == pos) ? "\n":" ");
                // pos--;
                handle_uart_command(&rx_pkt[1], --pos);
                // Reset para el siguiente mensaje
                pos = 0;
            }
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

    // Añadir whitelist
    for (int i = 0; i < ARRAY_SIZE(sensor_wl); i++) {
        err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_ADDR, &sensor_wl[i]);
        if (err) LOG_ERR("Whitelist %d, i=%d", err, i);
    }
    // Habilitar los filtros conjuntos!
    if(bt_scan_filter_enable(BT_SCAN_UUID_FILTER | BT_SCAN_ADDR_FILTER, true)){
        LOG_ERR("Error habilitando filtros (err %d)", err);
    }

    // Get the Bluetooth device address
	bt_addr_le_t addr;
	size_t count = 1;
	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_id_get(&addr, &count);
	bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));

    printk("Central - %s %s.\nCEL=%dus, ISO-I=%dms\n", 
        CONFIG_BT_DEVICE_NAME, addr_str,
        CONFIG_BT_CTLR_SDC_MAX_CONN_EVENT_LEN_DEFAULT,
        BIG_SDU_INTERVAL_US / 1000);
    
    k_msleep(2000);
    // BLE Broadcast ****************************************************
    const uint32_t adv_interval_ms = BIG_SDU_INTERVAL_US / 1000U;
	const uint32_t ext_adv_interval_ms = adv_interval_ms - 10U;

	// Adv & Big Structures
	struct bt_le_ext_adv *adv;
	struct bt_iso_big *big;

	// Create a non-connectable advertising set */
	const struct bt_le_adv_param *ext_adv_param = BT_LE_ADV_PARAM(
		BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY,
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
		printk("BIG failed (err %d)\n", err);
		return 0;
	}

    // Send BIG msg =====================================================
    while (true) {
        struct net_buf *buf;
        int ret;

        buf = net_buf_alloc(&bis_tx_pool, K_USEC(BUF_ALLOC_TIMEOUT_US));
        if (!buf) {
            printk("Data buffer allocate timeout on channel\n");
            continue;
        }

        ret = k_sem_take(&sem_iso_data, K_USEC(BUF_ALLOC_TIMEOUT_US));
        if (ret) {
            printk("k_sem_take for ISO data sent failed\n");
            net_buf_unref(buf);
            continue;
        }

        // Increment the pkt_num
        iso_seq_n++;
        sys_put_le32(iso_seq_n, iso_data);

        net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
        net_buf_add_mem(buf, iso_data, sizeof(iso_data));
        ret = bt_iso_chan_send(&bis_iso_chan[0], buf, 0);

        if (ret < 0) {
            printk("Unable to broadcast data %d", ret);
            net_buf_unref(buf);
            continue;
        }
    }
}