/*
 * HHS BLE bridge (custom GATT service) + UART + HHS parser
 * - UART 스트림에서 HHS 17바이트 프레임 조립
 *   (헤더 0xA5 0x5A 0x02, 끝 byte Switches 0x04|0x05)
 * - 프레임 완성 즉시 HHS Notify 특성(FFF1)으로 1프레임 = 1 notification 전송
 */

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

/* 연결 즉시 짧은 연결 간격(7.5~15ms) 요청.
 * BT_GAP_AUTO_UPDATE_CONN_PARAMS의 5초 대기 없이 바로 협상 시도하고,
 * 거절되면 5초 뒤 stack의 auto-update가 같은 선호값으로 재시도한다. */
static void update_conn_params(struct bt_conn *conn) {
    int err = bt_conn_le_param_update(
        conn, BT_LE_CONN_PARAM(CONFIG_BT_PERIPHERAL_PREF_MIN_INT,
                               CONFIG_BT_PERIPHERAL_PREF_MAX_INT,
                               CONFIG_BT_PERIPHERAL_PREF_LATENCY,
                               CONFIG_BT_PERIPHERAL_PREF_TIMEOUT));
    if (err)
        LOG_WRN("conn param update req failed (err %d)", err);
}

/*
 * Function: update_data_length
 * Description: This function updates the data length of the given connection.
 * Parameters:
 * 		conn - pointer to the connection to be updated
 * Return: None
 */
static void update_data_length(struct bt_conn *conn) {
    int err;
    /* Define the data length parameters */
    struct bt_conn_le_data_len_param my_data_len = {
        .tx_max_len = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };

    /* Update the data length of the connection */
    err = bt_conn_le_data_len_update(conn, &my_data_len);

    /* Check for errors */
    if (err) {
        LOG_ERR("data_len_update failed (err %d)", err);
    }
}

/* Implement callback function for MTU exchange */
static void exchange_func(struct bt_conn *conn, uint8_t att_err,
                          struct bt_gatt_exchange_params *params) {
    // Log the result of the MTU exchange
    LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");

    // If the exchange was successful, update the MTU size
    if (!att_err) {
        uint16_t payload_mtu =
            bt_gatt_get_mtu(conn) - 3; // 3 bytes used for Attribute headers.
        LOG_INF("New MTU: %d bytes", payload_mtu);
    }
}

/**
 * @brief Update the Maximum Transmission Unit (MTU) for the Bluetooth
 * connection.
 *
 * This function initiates an MTU exchange with a Bluetooth connection. It sets
 * up the callback function for handling MTU negotiation. If the exchange fails,
 * it logs an error message.
 *
 * @param conn The Bluetooth connection to update the MTU for.
 */
static void update_mtu(struct bt_conn *conn) {
    /* Create variable that holds callback for MTU negotiation */
    static struct bt_gatt_exchange_params exchange_params;

    /* Set the callback function for handling MTU negotiation */
    exchange_params.func = exchange_func;

    /* Initiate MTU exchange with the given Bluetooth connection */
    int err = bt_gatt_exchange_mtu(conn, &exchange_params);

    /* Check if the exchange was successful */
    if (err) {
        /* Log an error message if the exchange failed */
        LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
    }
}

typedef void (*hhs_on_frame_t)(const uint8_t *p);
static inline void hhs_feed(uint8_t c, hhs_on_frame_t on_frame) {
    switch (hhs_rx.st) {
    case RS_SYNC0:
        if (c == 0xA5)
            hhs_rx.st = RS_SYNC1;
        break;
    case RS_SYNC1:
        if (c == 0x5A)
            hhs_rx.st = RS_VER;
        else if (c != 0xA5) /* 0xA5면 새 헤더 시작 후보로 SYNC1 유지 */
            hhs_rx.st = RS_SYNC0;
        break;
    case RS_VER:
        if (c == 0x02) {
            hhs_rx.buf[0] = 0xA5;
            hhs_rx.buf[1] = 0x5A;
            hhs_rx.buf[2] = c;
            hhs_rx.pos = 3;
            hhs_rx.st = RS_PAYLOAD;
        } else {
            hhs_rx.st = (c == 0xA5) ? RS_SYNC1 : RS_SYNC0;
        }
        break;
    case RS_PAYLOAD:
        hhs_rx.buf[hhs_rx.pos++] = c;
        if (hhs_rx.pos >= HHS_PKT_SIZE) {
            /* Switches byte: 0x04(No Wear) / 0x05(Wear)만 유효한 프레임 */
            uint8_t sw = hhs_rx.buf[HHS_PKT_SIZE - 1];
            if (sw == 0x04 || sw == 0x05) {
                if (on_frame)
                    on_frame(hhs_rx.buf);
            } else {
                LOG_WRN("frame dropped: bad switches 0x%02x", sw);
            }
            hhs_rx.st = RS_SYNC0;
        }
        break;
    }
}

/* --- HHS GATT service --- */

/** @brief HHS Service UUID. */
#define BT_UUID_HHS_VAL                                                        \
    BT_UUID_128_ENCODE(0x0000FFF0, 0x0000, 0x1000, 0x8000, 0x00805F9B34FB)
/** @brief Notify Characteristic UUID. */
#define BT_UUID_HHS_NOTI_VAL                                                   \
    BT_UUID_128_ENCODE(0x0000FFF1, 0x0000, 0x1000, 0x8000, 0x00805F9B34FB)
/** @brief Write Characteristic UUID. */
#define BT_UUID_HHS_WRITE_VAL                                                  \
    BT_UUID_128_ENCODE(0x0000FFF2, 0x0000, 0x1000, 0x8000, 0x00805F9B34FB)

#define BT_UUID_HHS BT_UUID_DECLARE_128(BT_UUID_HHS_VAL)
#define BT_UUID_HHS_NOTI BT_UUID_DECLARE_128(BT_UUID_HHS_NOTI_VAL)
#define BT_UUID_HHS_WRITE BT_UUID_DECLARE_128(BT_UUID_HHS_WRITE_VAL)

static void hhs_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    LOG_INF("HHS notifications %s",
            value == BT_GATT_CCC_NOTIFY ? "enabled" : "disabled");
}

/* Write 특성: 현재 수신 데이터는 사용하지 않음 */
static ssize_t hhs_write_cb(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr, const void *buf,
                            uint16_t len, uint16_t offset, uint8_t flags) {
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    LOG_HEXDUMP_DBG(buf, len, "HHS write");
    return len;
}

BT_GATT_SERVICE_DEFINE(
    hhs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_HHS),
    BT_GATT_CHARACTERISTIC(BT_UUID_HHS_NOTI, BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(hhs_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_HHS_WRITE,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, hhs_write_cb, NULL));

/* attrs[2] = Notify 특성 값(FFF1). 구독한 모든 연결로 notify */
static int hhs_send(const uint8_t *data, uint16_t len) {
    return bt_gatt_notify(NULL, &hhs_svc.attrs[2], data, len);
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
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_HHS_VAL)};

#ifdef CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
#define async_adapter NULL
#endif

/* 프레임 완성 즉시 notify. 17B는 최소 ATT MTU(23)-3 안에 항상 들어가므로
 * 1프레임 = 1 notification이 보장된다. */
static void on_hhs_frame_cb(const uint8_t *p) {
    int ret;

    do {
        ret = hhs_send(p, HHS_PKT_SIZE);
        if (ret == -ENOMEM) /* TX 버퍼 고갈: 잠시 대기 후 재시도 */
            k_sleep(K_MSEC(5));
    } while (ret == -ENOMEM);

    if (ret && ret != -ENOTCONN) /* 미연결/미구독 시엔 조용히 폐기 */
        LOG_WRN("hhs_send err=%d", ret);
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

    case UART_RX_RDY: {
        /* 수신분을 즉시 복사해 큐잉: 드라이버 버퍼가 가득 차길 기다리지
         * 않으므로(타임아웃 단위로 이벤트 발생) 패킷 지연이 없다 */
        struct uart_data_t *chunk = k_malloc(sizeof(*chunk));
        if (!chunk) {
            LOG_WRN("RX_RDY: alloc fail, %d bytes dropped", evt->data.rx.len);
            break;
        }
        chunk->len = MIN(evt->data.rx.len, sizeof(chunk->data));
        memcpy(chunk->data, &evt->data.rx.buf[evt->data.rx.offset], chunk->len);
        k_fifo_put(&fifo_uart_rx_data, chunk);
        break;
    }

    case UART_RX_DISABLED: {
        buf = k_malloc(sizeof(*buf));
        if (!buf) {
            LOG_WRN("RX_DISABLED: alloc fail");
            k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
            return;
        }
        if (uart_rx_enable(uart, buf->data, sizeof(buf->data),
                           UART_WAIT_FOR_RX)) {
            LOG_WRN("RX_DISABLED: rx_enable fail");
            k_free(buf);
            k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
        }
        break;
    }

    case UART_RX_BUF_REQUEST:
        buf = k_malloc(sizeof(*buf));
        if (buf) {
            uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
        } else {
            LOG_WRN("RX_BUF_REQUEST: alloc fail");
        }
        break;

    case UART_RX_BUF_RELEASED:
        /* 데이터는 RX_RDY에서 이미 큐잉됨. 컨테이너만 반납 */
        buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t, data[0]);
        k_free(buf);
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
    if (uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX)) {
        LOG_WRN("uart_work: rx_enable fail");
        k_free(buf);
        k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
    }
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

    pos = snprintf(tx->data, sizeof(tx->data), "Starting HHS parser\r\n");
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

    // Update the connection parameters, data length and MTU
    update_conn_params(conn);
    update_data_length(conn);
    update_mtu(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason,
            bt_hci_err_to_str(reason));

    if (current_conn != conn) {
        return;
    }

    bt_conn_unref(current_conn);
    current_conn = NULL;
    dk_set_led_off(CON_STATUS_LED);
}

/* 협상된 연결 파라미터 확인용(interval 단위: 1.25ms, timeout 단위: 10ms) */
static void le_param_updated(struct bt_conn *conn, uint16_t interval,
                             uint16_t latency, uint16_t timeout) {
    ARG_UNUSED(conn);
    LOG_INF("Conn params: interval %u us, latency %u, timeout %u ms",
            interval * 1250, latency, timeout * 10);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .le_param_updated = le_param_updated,
};

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

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd,
                          ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising start err %d", err);
        return 0;
    }

    for (;;) {
        dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
        k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
    }
}

/* BLE 송신 전용 쓰레드: UART FIFO에서 읽어 HHS 파서 투입, 프레임 즉시 전송 */
void ble_write_thread(void) {
    k_sem_take(&ble_init_ok, K_FOREVER);

    for (;;) {
        struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);

        for (uint16_t i = 0; i < buf->len; i++) {
            hhs_feed(buf->data[i], on_hhs_frame_cb);
        }

        k_free(buf);
    }
}
K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
                NULL, PRIORITY, 0, 0);
