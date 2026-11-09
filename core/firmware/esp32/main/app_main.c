/*
 * core on an ESP32, as the companion computer (issue #22).
 *
 * The board's USB serial port (UART0 behind the dev board's USB-UART bridge)
 * is wired, through the laptop, to SITL's or a flight controller's telemetry
 * port. core reads the vehicle's real telemetry over it, runs GNSS spoofing
 * and lost-link detection, and its autonomy sends the vehicle's response
 * (EKF source failover, hold, return, land) back the same way - exactly what
 * core_main does over UDP, running the same loop (core/src/app/core_app.c).
 * The detector sees whatever the vehicle reports: here that is SITL's
 * simulated sensors, so a demo is "real detector on real hardware, simulated
 * vehicle", not a real flight.
 *
 * UART0 carries only MAVLink: the console is off (sdkconfig.defaults) and
 * core's diagnostics are not printed. The status LED shows the state instead.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "soc/uart_pins.h"

#include "core/app/core_app.h"
#include "core/common/time.h"

#define CORE_UART UART_NUM_0
#define CORE_UART_RX_BUF 8192
#define CORE_UART_TX_BUF 4096
#define TICK_MS 20

static int uart_stream_read(void *ctx, uint8_t *buf, size_t len) {
    (void)ctx;
    int n = uart_read_bytes(CORE_UART, buf, (uint32_t)len, 0);
    return n < 0 ? 0 : n; /* a UART doesn't "fail"; keep listening */
}

/* Whole frames only: a frame that doesn't fit in the TX ring is dropped
 * rather than blocking the loop or being sent in part. */
static bool uart_stream_write(void *ctx, const uint8_t *buf, size_t len) {
    (void)ctx;
    size_t free_bytes = 0;
    if (uart_get_tx_buffer_free_size(CORE_UART, &free_bytes) != ESP_OK || free_bytes < len) {
        return false;
    }
    return uart_write_bytes(CORE_UART, buf, len) == (int)len;
}

static void uart_init(void) {
    const uart_config_t cfg = {
        .baud_rate = CONFIG_CORE_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CORE_UART, CORE_UART_RX_BUF, CORE_UART_TX_BUF, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CORE_UART, &cfg));
    /* With the console off nothing else routes UART0 to its pins, so name
     * them: the dev board's USB-UART bridge is wired to U0TXD/U0RXD. */
    ESP_ERROR_CHECK(uart_set_pin(CORE_UART, U0TXD_GPIO_NUM, U0RXD_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

#if CONFIG_CORE_STATUS_LED_GPIO >= 0
static void led_init(void) {
    gpio_reset_pin(CONFIG_CORE_STATUS_LED_GPIO);
    gpio_set_direction(CONFIG_CORE_STATUS_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_CORE_STATUS_LED_GPIO, 0);
}

/* Off: no vehicle. 1 Hz: connected, nominal. 5 Hz: autonomy acting. */
static void led_update(const core_app_t *app, uint64_t now) {
    int level = 0;
    if (core_mavlink_conn_get_state(app->conn) == CORE_MAVLINK_CONN_CONNECTED) {
        uint64_t half_period = app->autonomy.behavior == CORE_AUTONOMY_NOMINAL ? 500 : 100;
        level = (int)((now / half_period) & 1u);
    }
    gpio_set_level(CONFIG_CORE_STATUS_LED_GPIO, level);
}
#else
static void led_init(void) {}
static void led_update(const core_app_t *app, uint64_t now) {
    (void)app;
    (void)now;
}
#endif

static core_app_t s_app;

void app_main(void) {
    uart_init();
    led_init();

    core_mavlink_conn_config_t conn_cfg = {
        .type = CORE_MAVLINK_TRANSPORT_STREAM,
        .transport.stream = {.read = uart_stream_read, .write = uart_stream_write, .ctx = NULL},
        .system_id = 1,
        .component_id = 191, /* MAV_COMP_ID_ONBOARD_COMPUTER, as core_main */
        .reconnect_interval_ms = 1000,
        .heartbeat_timeout_ms = 5000,
        .allow_recovery_control = CONFIG_CORE_ALLOW_RECOVERY_CONTROL,
    };
    core_mavlink_conn_t *conn = core_mavlink_conn_open(&conn_cfg);
    if (!conn) {
        abort(); /* only on misconfiguration; reboots */
    }

    const core_app_config_t app_cfg = {
        .allow_recovery_control = CONFIG_CORE_ALLOW_RECOVERY_CONTROL,
        .gps_free_backup = CONFIG_CORE_GPS_FREE_BACKUP,
        .autonomy_log = NULL, /* no filesystem; transitions show as STATUSTEXT */
        .print = NULL,        /* UART0 carries only MAVLink */
    };
    core_app_init(&s_app, conn, &app_cfg);

    bool announced = false;
    uint64_t last_announce_ms = 0;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        core_app_tick(&s_app);
        uint64_t now = core_now_ms();

        /* Says which core is on the link, so SITL's log and the dashboard's
         * messages show the detector is running on the board. Repeated every
         * minute in case nothing was listening the first time. */
        if (core_mavlink_conn_get_state(conn) == CORE_MAVLINK_CONN_CONNECTED &&
            (!announced || now - last_announce_ms >= 60000)) {
            core_mavlink_send_statustext(conn, CORE_MAV_SEVERITY_INFO, "core running on ESP32 (companion)");
            announced = true;
            last_announce_ms = now;
        }

        led_update(&s_app, now);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(TICK_MS));
    }
}
