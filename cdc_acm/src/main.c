#include <sample_usbd.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

LOG_MODULE_REGISTER(cdc_acm_sim, LOG_LEVEL_INF);
const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
#define SOF 0x7E
#define EOF 0x7F
#define ESCAPE_BYTE 0x7D
#define ESC_MASK 0x20
#define PAYLOAD_SIZE 64
#define ENCODED_MAX_SIZE (PAYLOAD_SIZE * 2 + 2)
#define SEND_PERIOD_MS 1000
#define NUM_SENSORS 12
#define RING_BUF_SIZE 4096
static uint8_t ring_buffer[RING_BUF_SIZE];
static struct ring_buf ringbuf;

size_t uart_frame_encode(uint8_t *dst, size_t dst_size, const uint8_t *src, size_t src_len){
    size_t out_len = 0;
    if (dst_size < 2) return 0;
    dst[out_len++] = SOF;
    for (size_t i = 0; i < src_len; i++) {
        uint8_t byte = src[i];
        if (byte == SOF || byte == EOF || byte == ESCAPE_BYTE) {
            if (out_len + 2 > dst_size) return 0;
            dst[out_len++] = ESCAPE_BYTE;
            dst[out_len++] = byte ^ ESC_MASK;
        } else {
            if (out_len + 1 > dst_size) return 0;
            dst[out_len++] = byte;
        }
    }
    if (out_len + 1 > dst_size) return 0;
    dst[out_len++] = EOF;
    return out_len;
}

static void interrupt_handler(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_tx_ready(dev)) {
            uint8_t buffer[64];
            int rb_len;
            rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
            if (!rb_len) {
                uart_irq_tx_disable(dev);
                continue;
            }
            int sent = uart_fifo_fill(dev, buffer, rb_len);
            if (sent < rb_len) {ring_buf_put(&ringbuf, buffer + sent, rb_len - sent);}
        }
    }
}

static uint32_t global_seq = 0;
static uint8_t sensor_id = 0;

void sim_timer_handler(struct k_timer *timer_id)
{
    uint8_t payload[PAYLOAD_SIZE];
    uint8_t encoded[ENCODED_MAX_SIZE];
    payload[0] = sensor_id;
    payload[1] = (global_seq >> 16) & 0xFF;
    payload[2] = (global_seq >> 8) & 0xFF;
    payload[3] = (global_seq) & 0xFF;
    for (int i = 4; i < PAYLOAD_SIZE; i++) {payload[i] = (uint8_t)(sensor_id + i + global_seq);}
    global_seq++;
    sensor_id++;
    if (sensor_id >= NUM_SENSORS) sensor_id = 0;
    size_t enc_len = uart_frame_encode(encoded, sizeof(encoded), payload, PAYLOAD_SIZE);
    if (enc_len == 0) return;
    uint32_t written = ring_buf_put(&ringbuf, encoded, enc_len);
    if (written < enc_len) {LOG_WRN("Ring buffer full, dropped %d bytes", enc_len - written);}
    uart_irq_tx_enable(uart_dev);
}

K_TIMER_DEFINE(sim_timer, sim_timer_handler, NULL);

int main(void)
{
    int ret;
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("CDC ACM device not ready");
        return 0;
    }
    ret = usb_enable(NULL);
    if (ret != 0) {
        LOG_ERR("Failed to enable USB");
        return 0;
    }
    LOG_INF("Wait for DTR");
    while (true) {
        uint32_t dtr = 0U;
        uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
        if (dtr) break;
        k_sleep(K_MSEC(100));
    }
    LOG_INF("DTR set - starting simulation");
    k_msleep(100);
    ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);
    uart_irq_callback_set(uart_dev, interrupt_handler);
    k_timer_start(&sim_timer, K_MSEC(SEND_PERIOD_MS), K_MSEC(SEND_PERIOD_MS));
    return 0;
}