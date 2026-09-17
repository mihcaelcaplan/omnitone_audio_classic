/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "bt_app_core.h"

#include "esp_app_desc.h"
#include "bridge.h"
#include "ota_ctl.h"
#include "flog.h"




/*******************************
 * STATIC FUNCTION DECLARATIONS
 ******************************/

/* handler for application task */
static void bt_app_task_handler(void *arg);
/* message sender */
static bool bt_app_send_msg(bt_app_msg_t *msg);
/* handle dispatched messages */
static void bt_app_work_dispatched(bt_app_msg_t *msg);

// my spi interface to the nrf5430
static void bt_spi_task_handler(void* arg);

/*******************************
 * STATIC VARIABLE DEFINITIONS
 ******************************/

static QueueHandle_t s_bt_app_task_queue = NULL;  /* handle of work queue */
static TaskHandle_t s_bt_app_task_handle = NULL;  /* handle of application task  */
static TaskHandle_t s_bt_spi_task_handle = NULL;  /* handle of application task  */


// my queue for spi out
static QueueHandle_t spi_send_queue = NULL;  /* handle of work queue */

#define MAX_AD2DP_DATA_LEN 5120

typedef struct {
    uint8_t* data;
    uint32_t len;
} bridge_data_t;


/*******************************
 * STATIC FUNCTION DEFINITIONS
 ******************************/

static bool bt_app_send_msg(bt_app_msg_t *msg)
{
    if (msg == NULL) {
        return false;
    }

    /* send the message to work queue */
    if (xQueueSend(s_bt_app_task_queue, msg, 10 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(BT_APP_CORE_TAG, "%s xQueue send failed", __func__);
        return false;
    }
    return true;
}

static void bt_app_work_dispatched(bt_app_msg_t *msg)
{
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}

static void bt_app_task_handler(void *arg)
{
    bt_app_msg_t msg;

    for (;;) {
        /* receive message from work queue and handle it */
        if (pdTRUE == xQueueReceive(s_bt_app_task_queue, &msg, (TickType_t)portMAX_DELAY)) {
            ESP_LOGD(BT_APP_CORE_TAG, "%s, signal: 0x%x, event: 0x%x", __func__, msg.sig, msg.event);

            switch (msg.sig) {
            case BT_APP_SIG_WORK_DISPATCH:
                bt_app_work_dispatched(&msg);
                break;
            default:
                ESP_LOGW(BT_APP_CORE_TAG, "%s, unhandled signal: %d", __func__, msg.sig);
                break;
            } /* switch (msg.sig) */

            if (msg.param) {
                free(msg.param);
            }
        }
    }
}

/* Little-endian field helpers for frames from the master */
static inline uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

/* Stage a reply for the next transaction: [cmd | tag][payload...] */
static void bridge_reply(uint8_t *tx, uint8_t cmd, const void *payload, size_t len)
{
    tx[0] = cmd | BRIDGE_REPLY_TAG;
    if (len) {
        memcpy(&tx[1], payload, len);
    }
}

/* Is this the exact frame the master sends for cmd? Anything else is the
 * tail of a frame we armed for too late (see wait_cs_idle) and must not be
 * acted on: an image byte in rx[0] looks just like a command. */
static bool frame_shape_ok(const uint8_t *rx, size_t rx_len)
{
    switch (rx[0]) {
    case BRIDGE_CMD_NONE:
        return true;
    case BRIDGE_CMD_OTA_BEGIN:
        return rx_len == BRIDGE_OTA_BEGIN_LEN;
    case BRIDGE_CMD_OTA_DATA: {
        if (rx_len < BRIDGE_OTA_DATA_HDR_LEN) {
            return false;
        }
        uint16_t len = rd16(&rx[3]);
        return len <= MAX_SPI_TRANSFER_CHUNK && rx_len == BRIDGE_OTA_DATA_FRAME_LEN(len);
    }
    case BRIDGE_CMD_LOG_READ:
        return rx_len == BRIDGE_LOG_READ_LEN;
    default:
        // STATUS, VERSION, the payload-less OTA_* commands, and anything we
        // don't know (which still gets an UNKNOWN_CMD reply if well formed)
        return rx_len == BRIDGE_CMD_ONLY_LEN;
    }
}

/* Don't arm the slave while the master is mid-frame. A transaction queued
 * with CS already low captures the tail of that frame, and whatever image
 * byte lands in rx[0] is decoded as a command - a payload 0x26 has aborted a
 * live update this way. The in-flight frame is lost either way; the master
 * notices the missing ack and resyncs on OTA_STATE. Bounded so a stuck CS
 * can't hang the bridge. */
#define CS_IDLE_WAIT_US 5000

static void wait_cs_idle(void)
{
    int64_t deadline = esp_timer_get_time() + CS_IDLE_WAIT_US;
    while (gpio_get_level(BRIDGE_PIN_CS) == 0 && esp_timer_get_time() < deadline) {
        esp_rom_delay_us(20);
    }
}

static void bt_spi_task_handler(void* arg){

    esp_err_t err = 0;

    // Both buffers are the full frame size: rx because OTA_DATA is that big,
    // tx because the DMA clocks .length bits out of it regardless of how short
    // the reply is. ESP32 slave DMA also works in whole words, so the master
    // pads every frame to a multiple of 4 bytes.
    uint8_t *rx_buf = heap_caps_calloc(1, BRIDGE_FRAME_MAX, MALLOC_CAP_DMA);
    uint8_t *tx_buf = heap_caps_calloc(1, BRIDGE_FRAME_MAX, MALLOC_CAP_DMA);
    assert(rx_buf != NULL && tx_buf != NULL);

    // Every command is answered one transaction late: the master leaves only
    // ~12us between back-to-back transceive() calls, which isn't enough time
    // for this task to wake, decode, and requeue a live response (measured on
    // a capture - response always came back 0x00 0x00). Preparing tx_buf here
    // and letting the *next* transaction clock it out removes the race.
    //
    // The flip side is that a frame the master sends while we are still busy
    // with the previous one (or while a flash erase has the cache off) is
    // simply lost - no transaction is queued to receive it. The OTA protocol
    // is stop-and-wait with sequence numbers for exactly that reason: the
    // master polls OTA_STATE until next_seq moves, and resends if it doesn't.
    spi_slave_transaction_t *done_trans = NULL;
    spi_slave_transaction_t txn = {
        .length = BRIDGE_FRAME_MAX * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    for (;;){ /* loop here forever because task*/

        wait_cs_idle();
        err = spi_slave_queue_trans(BRIDGE_DEV, &txn, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE("OMNI", "spi_slave_queue_trans failed: %d", err);
            continue;
        }

        err = spi_slave_get_trans_result(BRIDGE_DEV, &done_trans, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE("OMNI", "spi_slave_get_trans_result failed: %d", err);
            continue;
        }

        size_t rx_bits = done_trans->trans_len; // what the master actually clocked
        if (rx_bits == 0) {
            continue;
        }
        size_t rx_len = rx_bits / 8;
        if ((rx_bits % 8) != 0 || !frame_shape_ok(rx_buf, rx_len)) {
            // caught the tail of a frame: no reply staged, the master resyncs
            ESP_LOGW("OMNI", "dropping misaligned frame: %u bits, first byte %02x",
                     (unsigned)rx_bits, rx_buf[0]);
            continue;
        }

        uint8_t incoming_command = rx_buf[0];

        switch (incoming_command)
        {
            case BRIDGE_CMD_NONE:
                // master is only clocking out our last-prepared reply, not
                // submitting new work - leave tx_buf as-is
                break;

            case BRIDGE_CMD_STATUS: {
                bridge_reply(tx_buf, incoming_command, &status_flags, 1);
                break;
            }

            case BRIDGE_CMD_VERSION: {
                // 32 bytes, NUL padded - whatever the build stamped into the
                // app descriptor (git describe unless version.txt / PROJECT_VER
                // says otherwise). The nRF compares this against the image it
                // just sent before it confirms.
                const esp_app_desc_t *desc = esp_app_get_description();
                bridge_reply(tx_buf, incoming_command, desc->version, sizeof(desc->version));
                break;
            }

            case BRIDGE_CMD_OTA_BEGIN: {
                // [cmd][size u32]
                uint8_t res = ota_ctl_begin(rd32(&rx_buf[1]));
                bridge_reply(tx_buf, incoming_command, &res, 1);
                break;
            }

            case BRIDGE_CMD_OTA_DATA: {
                // [cmd][seq u16][len u16][payload][crc32]; lengths already
                // checked by frame_shape_ok
                uint16_t seq = rd16(&rx_buf[1]);
                uint16_t len = rd16(&rx_buf[3]);
                const uint8_t *payload = &rx_buf[BRIDGE_OTA_DATA_HDR_LEN];
                uint8_t res = ota_ctl_write_block(seq, payload, len, rd32(payload + len));
                ota_state_report_t st;
                ota_ctl_get_state(&st);
                uint8_t reply[3] = { res, (uint8_t)st.next_seq, (uint8_t)(st.next_seq >> 8) };
                bridge_reply(tx_buf, incoming_command, reply, sizeof(reply));
                break;
            }

            case BRIDGE_CMD_OTA_END: {
                uint8_t res = ota_ctl_end();
                bridge_reply(tx_buf, incoming_command, &res, 1);
                break;
            }

            case BRIDGE_CMD_OTA_STATE: {
                ota_state_report_t st;
                ota_ctl_get_state(&st);
                uint8_t reply[6] = {
                    (uint8_t)st.state,
                    (uint8_t)st.err,      (uint8_t)(st.err >> 8),
                    (uint8_t)st.next_seq, (uint8_t)(st.next_seq >> 8),
                    st.percent,
                };
                bridge_reply(tx_buf, incoming_command, reply, sizeof(reply));
                break;
            }

            case BRIDGE_CMD_OTA_REBOOT: {
                uint8_t res = ota_ctl_reboot();
                bridge_reply(tx_buf, incoming_command, &res, 1);
                break;
            }

            case BRIDGE_CMD_OTA_CONFIRM: {
                uint8_t res = ota_ctl_confirm();
                bridge_reply(tx_buf, incoming_command, &res, 1);
                break;
            }

            case BRIDGE_CMD_OTA_ABORT: {
                uint8_t res = ota_ctl_abort();
                bridge_reply(tx_buf, incoming_command, &res, 1);
                break;
            }

            case BRIDGE_CMD_LOG_INFO: {
                flog_info_t info = { 0 };
                uint8_t res = flog_info(&info);
                uint8_t reply[11] = {
                    res,
                    (uint8_t)info.oldest, (uint8_t)(info.oldest >> 8), (uint8_t)(info.oldest >> 16), (uint8_t)(info.oldest >> 24),
                    (uint8_t)info.end,    (uint8_t)(info.end >> 8),    (uint8_t)(info.end >> 16),    (uint8_t)(info.end >> 24),
                    (uint8_t)info.pending, (uint8_t)(info.pending >> 8),
                };
                bridge_reply(tx_buf, incoming_command, reply, sizeof(reply));
                break;
            }

            case BRIDGE_CMD_LOG_READ: {
                // [cmd][offset u32][len u16] -> [tag][result][len u16][data].
                // The data goes straight into tx_buf; the SPI DMA reads it
                // from there on the master's next NONE frame.
                uint32_t offset = rd32(&rx_buf[1]);
                uint16_t want = rd16(&rx_buf[5]);
                uint16_t got = 0;
                uint8_t res = flog_read(offset, &tx_buf[4], want, &got);
                uint8_t hdr[3] = { res, (uint8_t)got, (uint8_t)(got >> 8) };
                bridge_reply(tx_buf, incoming_command, hdr, sizeof(hdr));
                break;
            }

            case BRIDGE_CMD_LOG_CLEAR: {
                uint8_t res = flog_clear();
                bridge_reply(tx_buf, incoming_command, &res, 1);
                break;
            }

            default: {
                ESP_LOGW("OMNI", "Unknown command: %02x", incoming_command);
                uint8_t res = BRIDGE_RESULT_UNKNOWN_CMD;
                bridge_reply(tx_buf, incoming_command, &res, 1);
                break;
            }
        }




        // // get data
        // ret = xQueueReceive(spi_send_queue, &rcv_data, portMAX_DELAY); //block until ready (won't waste cycles in RTOS :)
        

        // // push it over spi with no framing or control :)
        // if (ret == pdTRUE){
            
        //     // unpack the bridge_data_t and push to spi transmit func
        //     transmit_SPI(rcv_data.data, rcv_data.len);
        //     ESP_LOGI("BT_SPI", "transmitting len %d", rcv_data.len);

        //     heap_caps_free(rcv_data.data); // free the pointer to the copied data
        // }
    }
}

/********************************
 * EXTERNAL FUNCTION DEFINITIONS
 *******************************/

bool bt_app_work_dispatch(bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len, bt_app_copy_cb_t p_copy_cback)
{
    ESP_LOGD(BT_APP_CORE_TAG, "%s event: 0x%x, param len: %d", __func__, event, param_len);

    bt_app_msg_t msg;
    memset(&msg, 0, sizeof(bt_app_msg_t));

    msg.sig = BT_APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    if (param_len == 0) {
        return bt_app_send_msg(&msg);
    } else if (p_params && param_len > 0) {
        if ((msg.param = malloc(param_len)) != NULL) {
            memcpy(msg.param, p_params, param_len);
            /* check if caller has provided a copy callback to do the deep copy */
            if (p_copy_cback) {
                p_copy_cback(msg.param, p_params, param_len);
            }
            return bt_app_send_msg(&msg);
        }
    }

    return false;
}

void bt_app_task_start_up(void)
{
    s_bt_app_task_queue = xQueueCreate(10, sizeof(bt_app_msg_t));
    spi_send_queue = xQueueCreate(10, sizeof(bridge_data_t)); // 1 byte = 8 bit data from a2dp callback

    xTaskCreate(bt_app_task_handler, "BtAppTask", 3072, NULL, 10, &s_bt_app_task_handle);
    

}

void bt_app_task_shut_down(void)
{
    if (s_bt_app_task_handle) {
        vTaskDelete(s_bt_app_task_handle);
        s_bt_app_task_handle = NULL;
    }
    if (s_bt_app_task_queue) {
        vQueueDelete(s_bt_app_task_queue);
        s_bt_app_task_queue = NULL;
    }
}

void bt_spi_task_start_up(void){

    xTaskCreate(bt_spi_task_handler, "BtSPITask", 8192, NULL, configMAX_PRIORITIES-3, &s_bt_spi_task_handle);
}

void bt_spi_task_shut_down(void){
    if (s_bt_spi_task_handle) {
        vTaskDelete(s_bt_spi_task_handle);
        s_bt_spi_task_handle = NULL;
    }
}


void write_spi_queue(uint8_t* data, uint32_t len){
        bridge_data_t spi_bridge;
        memset(&spi_bridge, 0, sizeof(spi_bridge));

        spi_bridge.data = heap_caps_malloc(len, MALLOC_CAP_DMA); 
        assert(spi_bridge.data != NULL);
        memcpy(spi_bridge.data, data, len);
        
        spi_bridge.len = len;

        BaseType_t ok = xQueueSend(spi_send_queue, &spi_bridge, portMAX_DELAY); //wait to push if q empty
        if (!ok){
            ESP_LOGW("BT_SPI", "spi send queue full");
            heap_caps_free(spi_bridge.data);
        }
}

