/*
 * Central para el servicio IMU - Arquitectura Asíncrona
 * Funcional para multiconexion - Elemento clave "conn ctx" !!!
 * Central hace todas las solicitudes de actulizacion de parametros
 * Hace uso de shell
 * Codigo fijo - Sin comunicacion UART
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
#include "my_imu_ble_service.h"

#define LOG_MODULE_NAME central_main
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

#define MAX_CONNECTIONS 3

// ===================================================================
// Parámetros de Conexión por Defecto
// ===================================================================
static struct bt_le_conn_param *g_conn_param = BT_LE_CONN_PARAM(6, 6, 0, 400); // 7.5ms
static struct bt_conn_le_phy_param *g_phy_param = BT_CONN_LE_PHY_PARAM_2M;
static struct bt_conn_le_data_len_param *g_data_len_param = BT_LE_DATA_LEN_PARAM_MAX;
static struct bt_gatt_exchange_params exchange_params;

// --- Gestor de Contexto de Conexión ---
BT_CONN_CTX_DEF(conns, MAX_CONNECTIONS, sizeof(struct bt_imus_client));

// --- Declaraciones ---
static uint8_t on_gatt_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);

// ===================================================================
// Funciones de Ayuda
// ===================================================================
// Función para obtener el índice de una conexión.
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
    return -1;
}

// ===================================================================
// Callbacks de la Aplicación
// ===================================================================
void on_write_completed(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
    const struct shell *shell = shell_backend_uart_get_ptr();
    if (err) { shell_error(shell, "Escritura GATT fallida (err %u)", err); } 
    else { shell_print(shell, "Comando enviado con éxito."); }
}

static uint8_t on_gatt_notify(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
    struct bt_imus_client *imus_client = bt_conn_ctx_get(&conns_ctx_lib, conn);
    if (!imus_client) { return BT_GATT_ITER_STOP; }

    int idx = get_conn_index(conn);

    if (!data) {
        LOG_INF("[Periférico %d] Suscripción habilitada para el handle 0x%04x", idx, params->value_handle);
        bt_conn_ctx_release(&conns_ctx_lib, imus_client);
        return BT_GATT_ITER_STOP;
    }
    
    if (params->value_handle == imus_client->sensordata_handle) {
        uint32_t packet_num = sys_get_le32(data);
        LOG_INF("[Periférico %d] >> NOTIFY: Paquete IMU #%u", idx, packet_num);
    } else if (params->value_handle == imus_client->exercisedetection_handle) {
        // CORRECCIÓN: Imprimir el índice en un log separado.
        LOG_INF("[Periférico %d] >> INDICATE: Evento Recibido:", idx);
        LOG_HEXDUMP_INF(data, length, "Datos del Evento:");
    }

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

    LOG_INF("[Periférico %d] Descubrimiento de servicios completo.", idx);

    int err = my_imus_handles_assign(dm, imus_client);
    if (err) {
        LOG_ERR("[Periférico %d] Fallo al asignar handles.", idx);
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }
    
    err = my_imus_subscribe_sensordata(conn, imus_client, on_gatt_notify);
    if (err) { LOG_ERR("[Periférico %d] Fallo al suscribirse a SensorData (err %d)", idx, err); }
    
    err = my_imus_subscribe_exercisedetection(conn, imus_client, on_gatt_notify);
    if (err) { LOG_ERR("[Periférico %d] Fallo al suscribirse a ExerciseDetection (err %d)", idx, err); }

    bt_gatt_dm_data_release(dm);
    LOG_INF(">>>>> [Periférico %d] Dispositivo LISTO. <<<<<", idx);
    bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
}

static struct bt_gatt_dm_cb discovery_cb = {.completed = discovery_complete};

// ===================================================================
// Callbacks de la Cadena de Configuración y Conexión
// ===================================================================

// --- Callbacks de la Máquina de Estados de Configuración ---
static void exchange_mtu_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (err) {
		LOG_ERR("Fallo en el intercambio de MTU (err %u)", err);
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}
	LOG_INF("MTU actualizado a %u bytes", bt_gatt_get_mtu(conn));

    // PASO FINAL: Iniciar el descubrimiento de servicios
    struct bt_imus_client *imus_client = bt_conn_ctx_get(&conns_ctx_lib, conn);
    if (!imus_client) return;

    int gatt_err = bt_gatt_dm_start(conn, BT_UUID_IMUS, &discovery_cb, imus_client);
    if (gatt_err) {
        LOG_ERR("Fallo al iniciar el descubrimiento (err %d)", gatt_err);
    }
    bt_conn_ctx_release(&conns_ctx_lib, imus_client);
}

static void on_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
	LOG_INF("Data Length actualizado: TX %u bytes, RX %u bytes", info->tx_max_len, info->rx_max_len);

    // PASO 4: Intercambiar MTU
    exchange_params.func = exchange_mtu_cb;
    int err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err) {
        LOG_ERR("Fallo en la petición de intercambio de MTU (err %d)", err);
    }
}

static void on_conn_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	LOG_INF("Parámetros de conexión actualizados: Intervalo %.2fms", (double)interval * 1.25);
    // PASO 3: Actualizar Data Length
    int err = bt_conn_le_data_len_update(conn, g_data_len_param);
    if (err) {
        LOG_ERR("Fallo en la petición de DLE (err %d)", err);
    }
}

static void on_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
	LOG_INF("PHY actualizado a 2M");
    // PASO 2: Actualizar Parámetros de Conexión
    int err = bt_conn_le_param_update(conn, g_conn_param);
    if (err) {
        LOG_ERR("Fallo en la petición de parámetros de conexión (err %d)", err);
    }
}

static void connected(struct bt_conn *conn, uint8_t hci_err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (hci_err) {
        LOG_ERR("Conexión fallida con %s (err 0x%02x)", addr, hci_err);
        return;
    }
    LOG_INF("Conectado con: %s. Iniciando configuración...", addr);

    bt_scan_stop();

    struct bt_imus_client *imus_client = bt_conn_ctx_alloc(&conns_ctx_lib, conn);
    if (!imus_client) {
        LOG_WRN("No hay memoria para el contexto.");
        bt_conn_disconnect(conn, BT_HCI_ERR_INSUFFICIENT_RESOURCES);
        return;
    }
    memset(imus_client, 0, sizeof(struct bt_imus_client));
    bt_conn_ctx_release(&conns_ctx_lib, imus_client);

    // PASO 1: Iniciar la cadena de configuración actualizando el PHY
    int err = bt_conn_le_phy_update(conn, g_phy_param);
    if (err) {
        LOG_ERR("Fallo en la petición de PHY (err %d)", err);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Desconectado de: %s (razón 0x%02x)", addr, reason);
    
    bt_conn_ctx_free(&conns_ctx_lib, conn);
    bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
}

// CORRECCIÓN: La definición va ANTES de que se use en main()
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .le_phy_updated = on_phy_updated,
    .le_param_updated = on_conn_param_updated,
    .le_data_len_updated = on_data_len_updated
};

// ===================================================================
// Comandos de la Shell
// ===================================================================
static int cmd_scan_start(const struct shell *shell, size_t argc, char **argv) {
    int err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err) { shell_error(shell, "Fallo al iniciar escaneo (err %d)", err); } 
    else { shell_print(shell, "Escaneo iniciado..."); }
    return err;
}

static int cmd_send_start(const struct shell *shell, size_t argc, char **argv) {
    if (argc < 2) {
        shell_error(shell, "Uso: send_start <index|all>");
        return -EINVAL;
    }
    static const uint8_t cmd_start[] = {0xAA};
    if (strcmp(argv[1], "all") == 0) {
        shell_print(shell, "Enviando comando a todos los periféricos...");
        // CORRECCIÓN: Usar un bucle for normal
        for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
            const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, i);
            if (ctx) {
                my_imus_write_command(ctx->conn, ctx->data, cmd_start, sizeof(cmd_start), on_write_completed);
                bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
            }
        }
    } else {
        int idx = atoi(argv[1]);
        const struct bt_conn_ctx *ctx = bt_conn_ctx_get_by_id(&conns_ctx_lib, idx);
        if (!ctx) {
            shell_error(shell, "Índice de periférico inválido o no conectado.");
            return -EINVAL;
        }
        shell_print(shell, "Enviando comando al periférico %d...", idx);
        my_imus_write_command(ctx->conn, ctx->data, cmd_start, sizeof(cmd_start), on_write_completed);
        bt_conn_ctx_release(&conns_ctx_lib, ctx->data);
    }
    return 0;
}

SHELL_CMD_REGISTER(scan_start, NULL, "1. Iniciar escaneo", cmd_scan_start);
SHELL_CMD_REGISTER(send_start, NULL, "2. Enviar comando de inicio <index|all>", cmd_send_start);

// ===================================================================
// Inicialización
// ===================================================================
int main(void)
{
    int err;
    LOG_INF("Iniciando Central (Arquitectura Asíncrona Corregida)...");

    err = bt_enable(NULL);
    if (err) { return 0; }
    LOG_INF("Bluetooth inicializado.");

    bt_conn_cb_register(&conn_callbacks);

    bt_scan_init(&(struct bt_scan_init_param){ .connect_if_match = 1 });
    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_IMUS);
    if (err) { return 0; }
    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err) { return 0; }
    
    LOG_INF("Configuración completa. Escriba 'scan_start' para comenzar.");
    return 0;
}
