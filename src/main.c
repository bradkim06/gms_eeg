/*
 * Nordic NUS + UART + HHS parser
 * - UART 스트림에서 HHS 17바이트 프레임 조립
 * - BLE NUS로 MTU-3 크기 청크로 전송
 */

#include <bluetooth/services/nus.h>
#include <dk_buttons_and_leds.h>
#include <soc.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <uart_async_adapter.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>
#include <zephyr/usb/usb_device.h>

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000
#define CON_STATUS_LED DK_LED2

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

/* --- HHS protocol --- */
#define HHS_PKT_SIZE 17

static struct {
    uint8_t buf[HHS_PKT_SIZE];
    uint8_t pos;
    enum { RS_SYNC0, RS_SYNC1, RS_VER, RS_PAYLOAD } st;
} hhs_rx = {.st = RS_SYNC0};

typedef void (*hhs_on_frame_t)(const uint8_t *p);
static inline void hhs_feed(uint8_t c, hhs_on_frame_t on_frame) {
    switch (hhs_rx.st) {
    case RS_SYNC0:
        if (c == 0xA5) {
            hhs_rx.buf[0] = c;
            hhs_rx.st = RS_SYNC1;
        }
        break;
    case RS_SYNC1:
        if (c == 0x5A) {
            hhs_rx.buf[1] = c;
            hhs_rx.st = RS_VER;
        } else
            hhs_rx.st = RS_SYNC0;
        break;
    case RS_VER:
        if (c == 0x02) {
            hhs_rx.buf[2] = c;
            hhs_rx.pos = 3;
            hhs_rx.st = RS_PAYLOAD;
        } else
            hhs_rx.st = RS_SYNC0;
        break;
    case RS_PAYLOAD:
        hhs_rx.buf[hhs_rx.pos++] = c;
        if (hhs_rx.pos >= HHS_PKT_SIZE) {
            if (on_frame)
                on_frame(hhs_rx.buf);
            hhs_rx.st = RS_SYNC0;
        }
        break;
    }
}

/* --- BLE / UART glue --- */

static K_SEM_DEFINE(ble_init_ok, 0, 1);
static struct bt_conn *current_conn;

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;

struct uart_data_t {
    void *fifo_reserved;
    uint8_t data[UART_BUF_SIZE];
    uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL)};

#ifdef CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
#define async_adapter NULL
#endif

/* 누적 버퍼(ATT MTU-3 기준으로 채워서 전송) */
static uint8_t g_acc[UART_BUF_SIZE];
static size_t g_acc_len;

static size_t nus_max_payload(void) {
    uint16_t mtu = current_conn ? bt_gatt_get_mtu(current_conn) : 23;
    if (mtu < 23)
        mtu = 23;
    size_t max_payload = mtu - 3; /* ATT Notification header */
    if (max_payload > sizeof(g_acc))
        max_payload = sizeof(g_acc);
    return max_payload;
}

/* 1) MTU-3로 쪼개 전송하도록 nus_flush 교체 */
static void nus_flush(uint8_t *acc, size_t *len) {
    if (!*len)
        return;
    size_t off = 0, maxp = nus_max_payload();

    while (off < *len) {
        size_t chunk = MIN(maxp, *len - off);
        int ret;
        do {
            ret = bt_nus_send(NULL, &acc[off], chunk);
            if (ret == -ENOMEM)
                k_sleep(K_MSEC(5));
        } while (ret == -ENOMEM);

        if (ret) { /* 다른 에러면 남은 건 버림 */
            LOG_WRN("bt_nus_send err=%d", ret);
            break;
        }
        off += chunk;
    }
    *len = 0;
}

/* 2) 프레임 그대로 누적하도록 on_hhs_frame_cb 교체 */
static void on_hhs_frame_cb(const uint8_t *p) {
    size_t maxp = nus_max_payload();

    /* 누적 버퍼 부족 시 먼저 플러시 */
    if (g_acc_len + HHS_PKT_SIZE > sizeof(g_acc)) {
        nus_flush(g_acc, &g_acc_len);
    }

    /* 17바이트 프레임 그대로 붙임 */
    memcpy(&g_acc[g_acc_len], p, HHS_PKT_SIZE);
    g_acc_len += HHS_PKT_SIZE;

    /* 가득 찼으면 즉시 송신 */
    if (g_acc_len >= maxp) {
        nus_flush(g_acc, &g_acc_len);
    }
}

/* UART 콜백: 라인 종료 문자를 기준으로 끊지 않음. 버퍼 교체 이벤트로만 큐잉 */
static void uart_cb(const struct device *dev, struct uart_event *evt,
                    void *user_data) {
    ARG_UNUSED(dev);
    static size_t aborted_len;
    struct uart_data_t *buf;
    static uint8_t *aborted_buf;

    switch (evt->type) {
    case UART_TX_DONE:
        if ((evt->data.tx.len == 0) || (!evt->data.tx.buf))
            return;
        if (aborted_buf) {
            buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data[0]);
            aborted_buf = NULL;
            aborted_len = 0;
        } else {
            buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t, data[0]);
        }
        k_free(buf);
        break;

    case UART_RX_RDY:
        buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
        buf->len += evt->data.rx.len;
        break;

    case UART_RX_DISABLED: {
        buf = k_malloc(sizeof(*buf));
        if (!buf) {
            LOG_WRN("RX_DISABLED: alloc fail");
            k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
            return;
        }
        buf->len = 0;
        uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
        break;
    }

    case UART_RX_BUF_REQUEST:
        buf = k_malloc(sizeof(*buf));
        if (buf) {
            buf->len = 0;
            uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
        } else {
            LOG_WRN("RX_BUF_REQUEST: alloc fail");
        }
        break;

    case UART_RX_BUF_RELEASED:
        buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t, data[0]);
        if (buf->len > 0) {
            k_fifo_put(&fifo_uart_rx_data, buf);
        } else {
            k_free(buf);
        }
        break;

    case UART_TX_ABORTED:
        if (!aborted_buf)
            aborted_buf = (uint8_t *)evt->data.tx.buf;
        aborted_len += evt->data.tx.len;
        buf = CONTAINER_OF((void *)aborted_buf, struct uart_data_t, data);
        uart_tx(uart, &buf->data[aborted_len], buf->len - aborted_len,
                SYS_FOREVER_MS);
        break;

    default:
        break;
    }
}

static void uart_work_handler(struct k_work *item) {
    struct uart_data_t *buf = k_malloc(sizeof(*buf));
    if (!buf) {
        LOG_WRN("uart_work: alloc fail");
        k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
        return;
    }
    buf->len = 0;
    uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev) {
    const struct uart_driver_api *api =
        (const struct uart_driver_api *)dev->api;
    return (api->callback_set != NULL);
}

static int uart_init(void) {
    int err;
    int pos;
    struct uart_data_t *rx;
    struct uart_data_t *tx;

    if (!device_is_ready(uart))
        return -ENODEV;

    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
        err = usb_enable(NULL);
        if (err && (err != -EALREADY))
            return err;
    }

    rx = k_malloc(sizeof(*rx));
    if (!rx)
        return -ENOMEM;
    rx->len = 0;

    k_work_init_delayable(&uart_work, uart_work_handler);

    if (IS_ENABLED(CONFIG_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
        uart_async_adapter_init(async_adapter, uart);
        uart = async_adapter;
    }

    err = uart_callback_set(uart, uart_cb, NULL);
    if (err) {
        k_free(rx);
        return err;
    }

    if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
        while (true) {
            uint32_t dtr = 0;
            uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
            if (dtr)
                break;
            k_sleep(K_MSEC(100));
        }
        (void)uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
        (void)uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
    }

    tx = k_malloc(sizeof(*tx));
    if (!tx) {
        k_free(rx);
        return -ENOMEM;
    }

    pos = snprintf(tx->data, sizeof(tx->data), "Starting NUS + HHS parser\r\n");
    if ((pos < 0) || (pos >= sizeof(tx->data))) {
        k_free(rx);
        k_free(tx);
        return -ENOMEM;
    }
    tx->len = pos;

    err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
    if (err) {
        k_free(rx);
        k_free(tx);
        return err;
    }

    err = uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX);
    if (err) {
        k_free(rx);
    }
    return err;
}

/* --- BLE conn callbacks --- */

static void connected(struct bt_conn *conn, uint8_t err) {
    char addr[BT_ADDR_LE_STR_LEN];
    if (err) {
        LOG_ERR("Connection failed, err 0x%02x %s", err,
                bt_hci_err_to_str(err));
        return;
    }
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Connected %s", addr);
    current_conn = bt_conn_ref(conn);
    dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason,
            bt_hci_err_to_str(reason));

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
        dk_set_led_off(CON_STATUS_LED);
    }
    /* 끊길 때 남은 누적 버퍼 폐기 */
    g_acc_len = 0;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* NUS 콜백: 현재는 수신 기능 불필요 */
static struct bt_nus_cb nus_cb;

void error(void) {
    dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);
    while (true) {
        k_sleep(K_MSEC(1000));
    }
}

static void configure_gpio(void) {
    if (dk_leds_init())
        LOG_ERR("LEDs init fail");
}

int main(void) {
    int blink_status = 0;
    int err = 0;

    configure_gpio();

    err = uart_init();
    if (err)
        error();

    err = bt_enable(NULL);
    if (err)
        error();
    LOG_INF("Bluetooth initialized");
    k_sem_give(&ble_init_ok);

    if (IS_ENABLED(CONFIG_SETTINGS))
        settings_load();

    err = bt_nus_init(&nus_cb);
    if (err) {
        LOG_ERR("bt_nus_init err %d", err);
        return 0;
    }

    err =
        bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising start err %d", err);
        return 0;
    }

    for (;;) {
        dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
        k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
    }
}

/* BLE 송신 전용 쓰레드: UART FIFO에서 읽어 HHS 파서 거쳐 누적 후 NUS 전송 */
void ble_write_thread(void) {
    k_sem_take(&ble_init_ok, K_FOREVER);

    for (;;) {
        struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);

        /* 바이트 스트림을 HHS 파서에 투입 */
        for (uint16_t i = 0; i < buf->len; i++) {
            hhs_feed(buf->data[i], on_hhs_frame_cb);
        }

        k_free(buf);

        /* 연결이 여유 있을 때 잔여 버퍼 플러시 */
        if (g_acc_len && current_conn) {
            nus_flush(g_acc, &g_acc_len);
        }
    }
}
K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
                NULL, PRIORITY, 0, 0);
