/*
 * Central para el servicio IMU - Arquitectura Asíncrona
 * Funcional para multiconexion - Elemento clave "conn ctx" !!!
 * Central hace todas las solicitudes de actulizacion de parametros
 * Comunicacion UART 
 * - Hilo de recepcion con maquina de stados & checksum
 * - Funcion de uart_tx
 * Pruebas de tiempo de Rx-pkts con LOGs 
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
#include "my_imu_ble_service.h"

#define LOG_MODULE_NAME central_main
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

#define MAX_CONNECTIONS 3

// ===================================================================
// COMUNICACIÓN UART
// ===================================================================
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
#define UART_RX_BUF_SIZE 128
#define UART_TX_BUF_SIZE 255

// --- Protocolo UART ---
#define PKT_START_BYTE 0x7E
#define PKT_END_BYTE 0x7F

// Comandos: PC -> Dongle
#define CMD_TYPE_TX_UART 0x18
#define CMD_TYPE_PRINT 0x19
#define CMD_TYPE_START_SCAN 0x20
#define CMD_TYPE_STOP_SCAN 0x21
#define CMD_TYPE_DISCONNECT 0x22
#define CMD_UPDATE_PARAMS 0x23
#define CMD_UPDATE_TIMER 0x24
#define CMD_STOP_PKT 0x25
#define CMD_TYPE_SEND_CMD 0x30

// Mensajes: Dongle -> PC
#define MSG_TYPE_SENSOR_DATA 0x60
#define MSG_TYPE_EVENT_DATA 0x70

static bool scan_on = false, print_log = true, send_uart = true;
static bool phy_2M = false;
static uint16_t conn_interv = 6, latency = 0, timeout_ms = 4000;
static uint16_t data_length = 75, tx_time = 400;

// ===================================================================
// Parámetros de Conexión por Defecto
// ===================================================================
static struct bt_conn_le_phy_param *g_phy_param_2M = BT_CONN_LE_PHY_PARAM_2M;
static struct bt_conn_le_phy_param *g_phy_param_1M = BT_CONN_LE_PHY_PARAM_1M;
static struct bt_gatt_exchange_params exchange_params;

// --- Gestor de Contexto de Conexión ---
BT_CONN_CTX_DEF(conns, MAX_CONNECTIONS, sizeof(struct bt_imus_client));

// DECLARACIONES ADELANTADAS
static uint8_t on_gatt_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
void send_uart_packet(uint8_t msg_type, uint8_t p_idx, const uint8_t *payload, uint16_t len);

static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

// --- Hilo de Recepción UART ---
static void uart_rx_thread(void);
K_THREAD_DEFINE(uart_rx_tid, 1024, uart_rx_thread, NULL, NULL, NULL, 7, 0, 0);

// ===================================================================
// Funciones de Ayuda
// ===================================================================
// Función para obtener el índice de una conexión.
static int get_conn_index(struct bt_conn *conn)
{
    for (size_t i = 0; i < MAX_CONNECTIONS; i++)
    {
        const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, i);
        if (ctx)
        {
            if (ctx->conn == conn)
            {
                bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
                return i;
            }
            bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
        }
    }
    return -1;
}

// ===================================================================
// Callbacks de la Aplicación
// ===================================================================
void on_write_completed(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
    if (print_log)
        printk("[] ACK\n");
}

static uint8_t on_gatt_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
    LOG_INF(" ");

    struct bt_imus_client *imus_client = bt_conn_ctx_get(&conns_ctx_lib, conn);
    if (!imus_client)
        return BT_GATT_ITER_STOP;

    // int idx = get_conn_index(conn);
    // LOG_INF("ok %d", idx);

    if (!data)
    {
        LOG_INF("N");
        // printk("[Peripheral %d] NULL handle 0x%04x\n", idx, params->value_handle);
        bt_conn_ctx_release(&conns_ctx_lib, imus_client);
        return BT_GATT_ITER_STOP;
    }

    // uint32_t packet_num = sys_get_le32(data);
    // if (params->value_handle == imus_client->sensordata_handle) {
    //     if (print_log) printk("[Peripheral %d] Notify #%u\n", idx, packet_num);
    //     if (send_uart) send_uart_packet(MSG_TYPE_SENSOR_DATA, (uint8_t)idx, data, length);
    // } else if (params->value_handle == imus_client->exercisedetection_handle) {
    //     // CORRECCIÓN: Imprimir el índice en un log separado.
    //     if (print_log) printk("[Peripheral %d] Indicate #%u\n", idx, packet_num);
    //     if (send_uart) send_uart_packet(MSG_TYPE_EVENT_DATA, (uint8_t)idx, data, length);
    // }
    bt_conn_ctx_release(&conns_ctx_lib, imus_client);
    return BT_GATT_ITER_CONTINUE;
}

// ===================================================================
// Callbacks de Conexión y Descubrimiento
// ===================================================================
static void discovery_complete(struct bt_gatt_dm *dm, void *context)
{
    struct bt_imus_client *imus_client = context;
    struct bt_conn *conn = (struct bt_conn *)bt_gatt_dm_conn_get(dm);
    int idx = get_conn_index(conn);

    int err = my_imus_handles_assign(dm, imus_client);
    if (err)
    {
        if (print_log)
            LOG_ERR("[Peripheral %d] Failed to assign handles.", idx);
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }

    err = my_imus_subscribe_sensordata(conn, imus_client, on_gatt_notify);
    if (err && print_log)
    {
        LOG_ERR("[Peripheral %d] Failed to subscribe to SensorData (err %d).", idx, err);
    }

    err = my_imus_subscribe_exercisedetection(conn, imus_client, on_gatt_notify);
    if (err && print_log)
    {
        LOG_ERR("[Peripheral %d] Failed to subscribe to ExerciseDetection (err %d).", idx, err);
    }

    bt_gatt_dm_data_release(dm);
    if (print_log)
        printk("[Peripheral %d] subscription completed.\n", idx);
    bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
}

static struct bt_gatt_dm_cb discovery_cb = {.completed = discovery_complete};

// ===================================================================
// Callbacks de la Cadena de Configuración y Conexión
// ===================================================================

// --- Callbacks de la Máquina de Estados de Configuración ---
static void exchange_mtu_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
    if (err)
    {
        if (print_log)
            LOG_ERR("MTU request failed (err %u)", err);
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }
    if (print_log)
        printk("[]  - MTU set to %d bytes.\n", bt_gatt_get_mtu(conn) - 3);

    // PASO FINAL: Iniciar el descubrimiento de servicios
    struct bt_imus_client *imus_client = bt_conn_ctx_get(&conns_ctx_lib, conn);
    if (!imus_client)
        return;

    int gatt_err = bt_gatt_dm_start(conn, BT_UUID_IMUS, &discovery_cb, imus_client);
    if (gatt_err && print_log)
        LOG_ERR("Failed to start discovery (err %d).", gatt_err);
    bt_conn_ctx_release(&conns_ctx_lib, imus_client);
}

static void on_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
    if (print_log)
        printk("[]  - Data Length (Tx/Rx): %u/%u bytes, %u/%u us.\n",
               info->tx_max_len, info->rx_max_len, info->tx_max_time, info->rx_max_time);

    // PASO 4: Intercambiar MTU
    exchange_params.func = exchange_mtu_cb;
    int err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err && print_log)
        LOG_ERR("MTU request failed (err %d).", err);
}

static void on_conn_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    double ci = interval * 1.25;
    uint16_t sto = timeout * 10;
    if (print_log)
        printk("[]  - Connection parameters set to %.2f ms, %d, %d ms.\n", ci, latency, sto);

    struct bt_conn_le_data_len_param g_data_len_param = {
        .tx_max_len = data_length,
        .tx_max_time = tx_time,
    };

    // Paso 3: Actualizar Data length
    int err = bt_conn_le_data_len_update(conn, &g_data_len_param);
    if (err && print_log)
        LOG_ERR("DL request failed (err %d).", err);
}

static void on_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
    if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_1M)
    {
        if (print_log)
            printk("[]  - PHY set to 1M\n");
    }
    if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_2M)
    {
        if (print_log)
            printk("[]  - PHY set to 2M\n");
    }

    uint16_t timeout = timeout_ms / 10;
    struct bt_le_conn_param *g_conn_param = BT_LE_CONN_PARAM(
        conn_interv, // Min Connection Interval = Value x 1.25ms
        conn_interv, // Min Connection Interval = Value x 1.25ms
        latency,     // Latency (Number of connection intervals the peripheral can skip)
        timeout      // Time that the central will wait before disconnecting
    );

    // Paso 2: Actualizar parametros de conexion
    int err = bt_conn_le_param_update(conn, g_conn_param);
    if (err && print_log)
        LOG_ERR("Connection parameters request failed (err %d).", err);
}

static void connected(struct bt_conn *conn, uint8_t hci_err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (hci_err)
    {
        if (print_log)
            LOG_ERR("Failed connection to %s (err 0x%02x).", addr, hci_err);
        return;
    }
    if (print_log)
        printk("[] Connected to: %s.\n", addr);

    bt_scan_stop();

    struct bt_imus_client *imus_client = bt_conn_ctx_alloc(&conns_ctx_lib, conn);
    if (!imus_client)
    {
        if (print_log)
            LOG_WRN("There is no memory for context.");
        bt_conn_disconnect(conn, BT_HCI_ERR_INSUFFICIENT_RESOURCES);
        return;
    }
    memset(imus_client, 0, sizeof(struct bt_imus_client));
    bt_conn_ctx_release(&conns_ctx_lib, imus_client);

    // PASO 1: Iniciar la cadena de configuración actualizando el PHY
    int err;
    if (phy_2M)
        err = bt_conn_le_phy_update(conn, g_phy_param_2M);
    else
        err = bt_conn_le_phy_update(conn, g_phy_param_1M);
    if (err && print_log)
        LOG_ERR("PHY request failed (err %d).\n", err);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    int idx = get_conn_index(conn);
    if (print_log)
        printk("[] Peripheral %d - %s disconnected (0x%02x).\n", idx, addr, reason);
    bt_conn_ctx_free(&conns_ctx_lib, conn);

    if (scan_on)
        bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
}

// Callbacks eventos BLE
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .le_phy_updated = on_phy_updated,
    .le_param_updated = on_conn_param_updated,
    .le_data_len_updated = on_data_len_updated};

// ===================================================================
// FUNCIONES UART
// ===================================================================

// Revisar la integridad del paquete recibido
static uint8_t calculate_checksum(const uint8_t *data, uint8_t len)
{
    uint8_t checksum = 0;
    for (int i = 0; i < len; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

// Estructura del paquete: [START][LEN][TIPO][IDX][PAYLOAD][CS][END]
void send_uart_packet(uint8_t msg_type, uint8_t p_idx, const uint8_t *payload, uint16_t len)
{
    static uint8_t tx_buf[UART_TX_BUF_SIZE];
    if (len + 6 > UART_TX_BUF_SIZE)
    {
        if (print_log)
            LOG_ERR("UART packet too big to send (%u bytes).", len);
    }
    else
    {
        tx_buf[0] = PKT_START_BYTE;
        tx_buf[1] = (uint8_t)len;
        tx_buf[2] = msg_type;
        tx_buf[3] = p_idx;
        memcpy(&tx_buf[4], payload, len);

        // Checksum incluye TIPO, ÍNDICE, LONGITUD y PAYLOAD
        uint8_t checksum = calculate_checksum(&tx_buf[1], 3 + len);
        tx_buf[4 + len] = checksum;
        tx_buf[5 + len] = PKT_END_BYTE;
        // Send the packet
        for (int i = 0; i < (6 + len); i++)
        {
            uart_poll_out(uart_dev, tx_buf[i]);
        }
    }
}

static void handle_uart_command(uint8_t cmd_type, const uint8_t *payload, uint8_t len)
{
    uint16_t n = 0, p_idx = 0;
    switch (cmd_type)
    {
    case CMD_TYPE_TX_UART:
        send_uart = !send_uart;
        if (print_log)
            printk("[] UART Send Recieved Pkts -> %s.\n", send_uart ? "ON" : "OFF");
        break;

    case CMD_TYPE_PRINT:
        print_log = !print_log;
        printk("[] Print Rx Data -> %s.\n", print_log ? "ON" : "OFF");
        break;

    case CMD_TYPE_START_SCAN:
        if (print_log)
            printk("[] Start Scan.\n");
        bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
        scan_on = true;
        break;

    case CMD_TYPE_STOP_SCAN:
        if (print_log)
            printk("[] Scan Stop.\n");
        bt_scan_stop();
        scan_on = false;
        break;

    case CMD_TYPE_DISCONNECT:
        p_idx = payload[0];
        if (p_idx == 0xFF)
        {
            if (print_log)
                printk("[] Disconnect ALL Peripherals.\n");
            for (size_t i = 0; i < MAX_CONNECTIONS; i++)
            {
                const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, i);
                if (ctx)
                {
                    bt_conn_disconnect(ctx->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                    bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
                }
            }
        }
        else
        {
            if (print_log)
                printk("[] Disconnect [Peripheral %d].\n", p_idx);
            const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, p_idx);
            if (ctx)
            {
                bt_conn_disconnect(ctx->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
            }
        }
        break;

    case CMD_UPDATE_PARAMS:
        if (len == 11)
        {

            // PHY
            if (payload[0] == 0x01)
                phy_2M = false;
            if (payload[0] == 0x02)
                phy_2M = true;

            // Connection Interval
            n = sys_get_le16(&payload[1]);
            if (n >= 6 && n <= 3200)
                conn_interv = n;

            // Latency
            n = sys_get_le16(&payload[3]);
            if (n >= 0)
                latency = n;

            // Supervision time out
            n = sys_get_le16(&payload[5]);
            uint16_t cal_tout = DIV_ROUND_UP(((1 + latency) * conn_interv * 5), 4);
            if (n >= cal_tout)
                timeout_ms = n;
            else
                timeout_ms = cal_tout;

            // Data Length
            n = sys_get_le16(&payload[7]);
            if (n >= 27 && n <= 251)
                data_length = n;

            // Tx/Rx time = TIEMPO QUE PUEDE TRANSMITIR O ESCUCHAR EL RADIO!!!
            // EL tiempo requerido depende de los datos a enviar y PHY.
            // Recomendacion: No mover y dejar en 2120us
            n = sys_get_le16(&payload[9]);
            if (n >= 328 && n <= 17040)
                tx_time = n;

            if (print_log)
                printk("[] Update PHY=%s, CI=%.2fms (%u), L=%u, STO=%ums, DL=%u, T=%uus.\n",
                       phy_2M ? "2M" : "1M", conn_interv * 1.25, conn_interv, latency, timeout_ms,
                       data_length, tx_time);
        }
        break;

    case CMD_TYPE_SEND_CMD:
        if (len >= 2)
        {
            p_idx = payload[0];
            uint8_t ble_cmd = payload[1];
            if (print_log)
                printk("[] Sending CMD 0x%02X to %u.\n", ble_cmd, p_idx);

            // Broadcast
            if (p_idx == 0xFF)
            {
                for (size_t i = 0; i < MAX_CONNECTIONS; i++)
                {
                    const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, i);
                    if (ctx)
                    {
                        my_imus_write_command(ctx->conn, ctx->data, &ble_cmd, 1, on_write_completed);
                        bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
                    }
                }
            }
            else
            {
                const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, p_idx);
                if (ctx)
                {
                    my_imus_write_command(ctx->conn, ctx->data, &ble_cmd, 1, on_write_completed);
                    bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
                }
            }
        }
        break;

    case CMD_UPDATE_TIMER:
        if (len >= 5)
        {
            p_idx = payload[0];
            uint8_t timer[5] = {0};

            timer[0] = 0x00;
            n = sys_get_le16(&payload[1]); // Frecuencia de muestreo
            memcpy(&timer[1], &n, sizeof(n));
            uint16_t k = sys_get_le16(&payload[3]); // Start delay
            memcpy(&timer[3], &k, sizeof(k));

            if (print_log)
                printk("[] Sending CMD 0x%02X -> %d Hz, %d ms to %u.\n", timer[0], n, k, p_idx);

            // Broadcast
            if (p_idx == 0xFF)
            {
                for (size_t i = 0; i < MAX_CONNECTIONS; i++)
                {
                    const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, i);
                    if (ctx)
                    {
                        my_imus_write_command(ctx->conn, ctx->data, timer, 5, on_write_completed);
                        bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
                    }
                }
            }
            else
            {
                const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, p_idx);
                if (ctx)
                {
                    my_imus_write_command(ctx->conn, ctx->data, timer, 5, on_write_completed);
                    bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
                }
            }
        }
        break;

    case CMD_STOP_PKT:
        if (len >= 5)
        {
            uint8_t x[5] = {0};
            uint32_t pkt = sys_get_le32(&payload[1]);

            p_idx = payload[0];
            x[0] = 0xBB;
            memcpy(&x[1], &pkt, sizeof(pkt));

            if (print_log)
                printk("[] Sending CMD 0xBB -> StopPkt %d to %u.\n", pkt, p_idx);

            // Broadcast
            if (p_idx == 0xFF)
            {
                for (size_t i = 0; i < MAX_CONNECTIONS; i++)
                {
                    const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, i);
                    if (ctx)
                    {
                        my_imus_write_command(ctx->conn, ctx->data, x, 5, on_write_completed);
                        bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
                    }
                }
            }
            else
            {
                const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, p_idx);
                if (ctx)
                {
                    my_imus_write_command(ctx->conn, ctx->data, x, 5, on_write_completed);
                    bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
                }
            }
        }
        break;

    default:
        if (print_log)
            LOG_WRN("Unknown UART CMD 0x%02X", cmd_type);
        break;
    }
}

// Hilo de recepcion de UART
static void uart_rx_thread(void)
{
    uint8_t c;
    uint8_t buffer[UART_RX_BUF_SIZE];
    uint8_t len = 0, type = 0;
    int pos = 0;

    // Definimos los estados de nuestra máquina
    enum
    {
        STATE_WAIT_START,
        STATE_READ_LEN,
        STATE_READ_TYPE,
        STATE_READ_PAYLOAD,
        STATE_READ_CHECKSUM,
        STATE_READ_END
    } state = STATE_WAIT_START;

    while (true)
    {
        // 1. Intenta leer un byte. Si no hay, duerme y vuelve a intentarlo.
        if (uart_poll_in(uart_dev, &c))
        {
            k_sleep(K_MSEC(10));
            continue;
        }

        // 2. Si hay un byte, lo procesamos según nuestro estado actual.
        switch (state)
        {
        case STATE_WAIT_START:
            if (c == PKT_START_BYTE)
            {
                state = STATE_READ_LEN;
                pos = 0;
            }
            break;

        case STATE_READ_LEN:
            len = c;
            // ¿El paquete cabe en nuestro buffer?
            // len + 3 = len del payload + tipo + checksum
            if (len + 3 > UART_RX_BUF_SIZE)
            {
                LOG_WRN("UART: Wrong len payload (%u). Discarding message.", len);
                state = STATE_WAIT_START; // Volvemos al inicio
            }
            else
            {
                state = STATE_READ_TYPE;
            }
            break;

        case STATE_READ_TYPE:
            type = c;
            pos = 0;
            // Post incremento, se ejecuta el valor actual y despues del acceso al array se incrementa pos
            buffer[pos++] = type;
            // Si no hay payload, saltamos directo al checksum
            if (len == 0)
                state = STATE_READ_CHECKSUM;
            else
                state = STATE_READ_PAYLOAD;
            break;

        case STATE_READ_PAYLOAD:
            // Post incremento, se ejecuta el valor actual y despues del acceso al array se incrementa pos
            buffer[pos++] = c;
            if (pos > len)
                state = STATE_READ_CHECKSUM;
            break;

        case STATE_READ_CHECKSUM:
        {
            uint8_t received_checksum = c;
            // El checksum se calcula sobre tipo + payload
            uint8_t calculated_checksum = calculate_checksum(buffer, len + 1);

            if (received_checksum == calculated_checksum)
            {
                state = STATE_READ_END;
            }
            else
            {
                LOG_WRN("UART: Wrong checksum.");
                // state = STATE_WAIT_START;
            }
        }
        break;

        case STATE_READ_END:
            if (c == PKT_END_BYTE)
            {
                // El payload empieza en buffer[1]
                handle_uart_command(type, &buffer[1], len);
            }
            else
            {
                LOG_WRN("UART: Wrong end-byte.");
                // state = STATE_WAIT_START;
            }
            // En cualquier caso, hemos terminado con este paquete.
            state = STATE_WAIT_START; // Volvemos al inicio para buscar el siguiente.
            break;
        }
    }
}

// ===================================================================
// Inicialización
// ===================================================================
int main(void)
{
    int err;
    printk("[] Central - Dongle. ");

    if (!device_is_ready(uart_dev))
    {
        LOG_ERR("UART device not found!");
        return 0;
    }

    err = bt_enable(NULL);
    if (err)
        return 0;
    bt_conn_cb_register(&conn_callbacks);

    // SCAN CONFIGURATION
    struct bt_le_conn_param *sc_conn_param = BT_LE_CONN_PARAM(6, 6, 0, 400);
    struct bt_le_scan_param scan_param = {
        .type = BT_SCAN_TYPE_SCAN_ACTIVE,
        .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = 0x0010,
        .window = 0x0010,
    };
    struct bt_scan_init_param scan_init = {
        .connect_if_match = 1,
        .scan_param = &scan_param,
        .conn_param = sc_conn_param};
    bt_scan_init(&scan_init);

    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_IMUS);
    if (err)
    {
        return 0;
    }
    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err)
    {
        return 0;
    }
    k_msleep(100);

    bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);

    printk("Inicializacion completa.\n");
    return 0;
}