/*
 * Central para el servicio IMU, versión de depuración con aislamiento granular.
 - Intento de multiconexion basado en semaforizacion
 Basado en el ejemplo de througput
 Incapaz de soportar multiconexion
 hace uso de shell
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/scan.h>
#include <bluetooth/gatt_dm.h>
#include <zephyr/shell/shell_uart.h>
#include <zephyr/sys/byteorder.h>

#include "my_imu_ble_service.h"

#define LOG_MODULE_NAME central_main
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

// --- Sincronización y Estado ---
static K_SEM_DEFINE(op_sem, 0, 1);
static uint8_t g_att_err;
static struct bt_conn *default_conn;
static struct bt_imus_client imus_client;

// --- Parámetros de Conexión ---
static struct bt_le_conn_param *conn_param_fast = BT_LE_CONN_PARAM(6, 6, 0, 400);
static struct bt_conn_le_phy_param *phy_param_2m = BT_CONN_LE_PHY_PARAM_2M;
static struct bt_conn_le_data_len_param *data_len_max = BT_LE_DATA_LEN_PARAM_MAX;
static struct bt_gatt_exchange_params exchange_params;

// ===================================================================
// Callbacks de la Aplicación (Recepción de Datos)
// ===================================================================
void on_write_completed(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
    if (err) { LOG_ERR("Escritura GATT fallida (err %u)", err); } 
    else { LOG_INF("Comando enviado con éxito."); }
}

static uint8_t sensordata_received(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
    if (!data) { return BT_GATT_ITER_STOP; }
    uint32_t packet_num = sys_get_le32(data);
    LOG_INF(">> NOTIFY: Paquete IMU recibido: #%u", packet_num);
    return BT_GATT_ITER_CONTINUE;
}

static uint8_t exercisedetection_received(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
    if (!data) { return BT_GATT_ITER_STOP; }
    LOG_HEXDUMP_INF(data, length, ">> INDICATE: Evento de Ejercicio Recibido:");
    return BT_GATT_ITER_CONTINUE;
}

// ===================================================================
// Callbacks de Sincronización
// ===================================================================
static void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param) { k_sem_give(&op_sem); }
static void le_param_updated(struct bt_conn *conn, uint16_t i, uint16_t l, uint16_t t) { k_sem_give(&op_sem); }
static void le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info) { k_sem_give(&op_sem); }
static void exchange_mtu_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *p) { g_att_err = err; k_sem_give(&op_sem); }

static void discovery_complete(struct bt_gatt_dm *dm, void *context) {
    g_att_err = my_imus_handles_assign(dm, (struct bt_imus_client *)context);
    bt_gatt_dm_data_release(dm);
    k_sem_give(&op_sem);
}
static struct bt_gatt_dm_cb discovery_cb = { .completed = discovery_complete };

// --- Callbacks de CONFIRMACIÓN de Suscripción (SEPARADOS) ---
static uint8_t data_sub_confirmed_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
    if (data) { return sensordata_received(conn, params, data, length); }
    LOG_INF(">>>>> Confirmación de SUSCRIPCIÓN a SensorData recibida.");
    params->notify = sensordata_received;
    g_att_err = 0;
    k_sem_give(&op_sem);
    return BT_GATT_ITER_STOP;
}

static uint8_t event_sub_confirmed_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length)
{
    if (data) { return exercisedetection_received(conn, params, data, length); }
    LOG_INF(">>>>> Confirmación de SUSCRIPCIÓN a ExerciseDetection recibida.");
    params->notify = exercisedetection_received;
    g_att_err = 0;
    k_sem_give(&op_sem);
    return BT_GATT_ITER_STOP;
}

// ===================================================================
// Callbacks de Conexión y Escaneo
// ===================================================================
static void connected(struct bt_conn *conn, uint8_t hci_err) {
    if (hci_err) { LOG_ERR("Conexión fallida (err 0x%02x)", hci_err); return; }
    LOG_INF("Conectado. Escriba 'run_config' para descubrir servicios.");
    default_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("Desconectado (razón 0x%02x)", reason);
    if (default_conn) { bt_conn_unref(default_conn); default_conn = NULL; }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected, .disconnected = disconnected,
    .le_phy_updated = le_phy_updated, .le_param_updated = le_param_updated,
    .le_data_len_updated = le_data_len_updated
};

// ===================================================================
// Funciones de la Shell y Orquestador
// ===================================================================
#define CHECK(func, name) \
    shell_print(shell, "Paso: %s...", name); \
    k_sem_reset(&op_sem); \
    g_att_err = 0xFF; \
    err = func; \
    if (err) { shell_error(shell, "La petición '%s' falló en la llamada: %d", name, err); return err; } \
    if (k_sem_take(&op_sem, K_SECONDS(10)) != 0) { shell_error(shell, "Timeout en '%s'!", name); return -ETIMEDOUT; } \
    if (g_att_err != 0 && g_att_err != 0xFF) { shell_error(shell, "'%s' falló con error de callback: %d", name, g_att_err); return g_att_err; } \
    shell_print(shell, "Paso '%s' exitoso.", name)

static int cmd_run_config(const struct shell *shell, size_t argc, char **argv) {
    int err;
    if (!default_conn) { shell_error(shell, "No conectado."); return -ENOENT; }
    shell_print(shell, "----- Iniciando Configuración de Conexión y Descubrimiento -----");
    CHECK(bt_conn_le_phy_update(default_conn, phy_param_2m), "Actualizar PHY");
    CHECK(bt_conn_le_param_update(default_conn, conn_param_fast), "Actualizar Parámetros");
    CHECK(bt_conn_le_data_len_update(default_conn, data_len_max), "Actualizar DLE");
    exchange_params.func = exchange_mtu_func;
    CHECK(bt_gatt_exchange_mtu(default_conn, &exchange_params), "Intercambiar MTU");
    CHECK(bt_gatt_dm_start(default_conn, BT_UUID_IMUS, &discovery_cb, &imus_client), "Descubrir Servicios GATT");
    shell_print(shell, "----- Configuración y Descubrimiento Completos. Listo para suscribirse. -----");
    shell_print(shell, "Use 'subscribe_data', 'subscribe_event', y 'send_start'.");
    return 0;
}

static int cmd_subscribe_data(const struct shell *shell, size_t argc, char **argv) {
    int err;
    if (!default_conn || imus_client.sensordata_handle == 0) { shell_error(shell, "No conectado o handles no descubiertos."); return -EFAULT; }
    CHECK(my_imus_subscribe_sensordata(default_conn, &imus_client, data_sub_confirmed_cb), "Suscribirse a SensorData");
    return 0;
}

static int cmd_subscribe_event(const struct shell *shell, size_t argc, char **argv) {
    int err;
    if (!default_conn || imus_client.exercisedetection_handle == 0) { shell_error(shell, "No conectado o handles no descubiertos."); return -EFAULT; }
    CHECK(my_imus_subscribe_exercisedetection(default_conn, &imus_client, event_sub_confirmed_cb), "Suscribirse a ExerciseDetection");
    return 0;
}

static int cmd_send_start(const struct shell *shell, size_t argc, char **argv) {
    if (!default_conn || imus_client.command_handle == 0) { shell_error(shell, "No conectado o handles no descubiertos."); return -EFAULT; }
    shell_print(shell, "Enviando comando de inicio (0xAA)...");
    static const uint8_t cmd_start[] = {0xAA};
    return my_imus_write_command(default_conn, &imus_client, cmd_start, sizeof(cmd_start), on_write_completed);
}

static int cmd_scan_start(const struct shell *shell, size_t argc, char **argv) {
    int err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err) { shell_error(shell, "Fallo al iniciar escaneo (err %d)", err); return err; }
    shell_print(shell, "Escaneo iniciado...");
    return 0;
}

SHELL_CMD_REGISTER(scan_start, NULL, "Iniciar escaneo", cmd_scan_start);
SHELL_CMD_REGISTER(run_config, NULL, "1. Configurar conexión y descubrir servicios", cmd_run_config);
SHELL_CMD_REGISTER(subscribe_data, NULL, "2. Suscribirse a SensorData", cmd_subscribe_data);
SHELL_CMD_REGISTER(subscribe_event, NULL, "3. Suscribirse a ExerciseDetection", cmd_subscribe_event);
SHELL_CMD_REGISTER(send_start, NULL, "4. Enviar comando de inicio (0xAA)", cmd_send_start);

// ===================================================================
// Inicialización
// ===================================================================
int main(void) {
    int err;
    LOG_INF("Iniciando Aplicación Central IMU (Versión Aislamiento)...");
    err = bt_enable(NULL);
    if (err) { LOG_ERR("Fallo al inicializar Bluetooth (err %d)", err); return 0; }
    LOG_INF("Bluetooth inicializado.");
    bt_scan_init(&(struct bt_scan_init_param){ .connect_if_match = 1, .conn_param = conn_param_fast });
    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_IMUS);
    if (err) { LOG_ERR("Fallo al añadir filtro de escaneo (err %d)", err); return 0; }
    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err) { LOG_ERR("Fallo al habilitar filtro de escaneo (err %d)", err); return 0; }
    LOG_INF("Configuración completa. Escriba 'scan_start' para comenzar.");
    return 0;
}