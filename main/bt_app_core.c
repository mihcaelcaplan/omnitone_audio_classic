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
#include "bt_app_core.h"
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
#include "driver/dac_continuous.h"
#else
#include "driver/i2s_std.h"
#endif
#include "freertos/ringbuf.h"


#include "bridge.h"



#define RINGBUF_HIGHEST_WATER_LEVEL    (32 * 1024)
#define RINGBUF_PREFETCH_WATER_LEVEL   (20 * 1024)

enum {
    RINGBUFFER_MODE_PROCESSING,    /* ringbuffer is buffering incoming audio data, I2S is working */
    RINGBUFFER_MODE_PREFETCHING,   /* ringbuffer is buffering incoming audio data, I2S is waiting */
    RINGBUFFER_MODE_DROPPING       /* ringbuffer is not buffering (dropping) incoming audio data, I2S is working */
};

/*******************************
 * STATIC FUNCTION DECLARATIONS
 ******************************/

/* handler for application task */
static void bt_app_task_handler(void *arg);
/* handler for I2S task */
// static void bt_i2s_task_handler(void *arg);
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

static TaskHandle_t s_bt_i2s_task_handle = NULL;  /* handle of I2S task */
static RingbufHandle_t s_ringbuf_i2s = NULL;     /* handle of ringbuffer for I2S */
static SemaphoreHandle_t s_i2s_write_semaphore = NULL;
static uint16_t ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;

// my queue for spi out
static QueueHandle_t spi_send_queue = NULL;  /* handle of work queue */

#define MAX_AD2DP_DATA_LEN 5120

typedef struct {
    uint8_t* data;
    uint32_t len;
} bridge_data_t;


/*********************************
 * EXTERNAL FUNCTION DECLARATIONS
 ********************************/
#ifndef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
extern i2s_chan_handle_t tx_chan;
#else
extern dac_continuous_handle_t tx_chan;
#endif

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

static void bt_spi_task_handler(void* arg){

    esp_err_t err = 0;

    // ESP32 DMA writes in full words regardless of the declared bit length,
    // so a <4 byte allocation would let the DMA HW overwrite adjacent heap memory
    uint8_t *rx_buf = heap_caps_malloc(4, MALLOC_CAP_DMA);
    uint8_t *tx_buf = heap_caps_malloc(4, MALLOC_CAP_DMA);
    tx_buf[0] = 0x00;
    tx_buf[1] = 0x00;

    // Single 2-byte full-duplex exchange per poll instead of two separate
    // command/response transactions: the master leaves only ~12us between
    // back-to-back transceive() calls, which isn't enough time for this task
    // to wake, decode, and requeue a live response (measured on a capture -
    // response always came back 0x00 0x00). Responding one poll late removes
    // the race entirely: tx_buf is prepared here with ~100ms of slack before
    // the *next* transaction clocks it out, instead of ~12us.
    spi_slave_transaction_t *done_trans = NULL;
    spi_slave_transaction_t txn = {
        .length = 16,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    for (;;){ /* loop here forever because task*/

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

        uint8_t incoming_command = rx_buf[0];

        // 0x00 = "empty": master is only clocking out our last-prepared
        // response, not submitting new work - leave tx_buf as-is
        if (incoming_command != 0x00) {
            switch (incoming_command)
            {
                case 0x0A: {
                    // status |= EXT_MCU_ON_FLAG; // report on

                    tx_buf[0] = incoming_command | 0x80; // tag so the master knows which command this answers
                    tx_buf[1] = status_flags;
                    // ESP_LOGI("OMNI", "queued response %02x %02x for command %02x", tx_buf[0], tx_buf[1], incoming_command);
                    break;
                }

                default:
                    ESP_LOGW("OMNI", "Unknown command: %02x", incoming_command);
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

static void bt_i2s_task_handler(void *arg)
{
    uint8_t *data = NULL;
    size_t item_size = 0;
    /**
     * The total length of DMA buffer of I2S is:
     * `dma_frame_num * dma_desc_num * i2s_channel_num * i2s_data_bit_width / 8`.
     * Transmit `dma_frame_num * dma_desc_num` bytes to DMA is trade-off.
     */
    const size_t item_size_upto = 240 * 6;
    size_t bytes_written = 0;

    for (;;) {
        if (pdTRUE == xSemaphoreTake(s_i2s_write_semaphore, portMAX_DELAY)) {
            for (;;) {
                item_size = 0;
                /* receive data from ringbuffer and write it to I2S DMA transmit buffer */
                data = (uint8_t *)xRingbufferReceiveUpTo(s_ringbuf_i2s, &item_size, (TickType_t)pdMS_TO_TICKS(20), item_size_upto);
                if (item_size == 0) {
                    ESP_LOGI(BT_APP_CORE_TAG, "ringbuffer underflowed! mode changed: RINGBUFFER_MODE_PREFETCHING");
                    ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
                    break;
                }

            #ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
                dac_continuous_write(tx_chan, data, item_size, &bytes_written, -1);
            #else
                i2s_channel_write(tx_chan, data, item_size, &bytes_written, portMAX_DELAY);
            #endif
                vRingbufferReturnItem(s_ringbuf_i2s, (void *)data);
            }
        }
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

void bt_i2s_task_start_up(void)
{
    ESP_LOGI(BT_APP_CORE_TAG, "ringbuffer data empty! mode changed: RINGBUFFER_MODE_PREFETCHING");
    ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
    if ((s_i2s_write_semaphore = xSemaphoreCreateBinary()) == NULL) {
        ESP_LOGE(BT_APP_CORE_TAG, "%s, Semaphore create failed", __func__);
        return;
    }
    if ((s_ringbuf_i2s = xRingbufferCreate(RINGBUF_HIGHEST_WATER_LEVEL, RINGBUF_TYPE_BYTEBUF)) == NULL) {
        ESP_LOGE(BT_APP_CORE_TAG, "%s, ringbuffer create failed", __func__);
        return;
    }
    xTaskCreate(bt_i2s_task_handler, "BtI2STask", 2048, NULL, configMAX_PRIORITIES - 3, &s_bt_i2s_task_handle);
}

void bt_i2s_task_shut_down(void)
{
    if (s_bt_i2s_task_handle) {
        vTaskDelete(s_bt_i2s_task_handle);
        s_bt_i2s_task_handle = NULL;
    }
    if (s_ringbuf_i2s) {
        vRingbufferDelete(s_ringbuf_i2s);
        s_ringbuf_i2s = NULL;
    }
    if (s_i2s_write_semaphore) {
        vSemaphoreDelete(s_i2s_write_semaphore);
        s_i2s_write_semaphore = NULL;
    }
}

size_t write_ringbuf(const uint8_t *data, size_t size)
{
    size_t item_size = 0;
    BaseType_t done = pdFALSE;

    if (ringbuffer_mode == RINGBUFFER_MODE_DROPPING) {
        ESP_LOGW(BT_APP_CORE_TAG, "ringbuffer is full, drop this packet!");
        vRingbufferGetInfo(s_ringbuf_i2s, NULL, NULL, NULL, NULL, &item_size);
        if (item_size <= RINGBUF_PREFETCH_WATER_LEVEL) {
            ESP_LOGI(BT_APP_CORE_TAG, "ringbuffer data decreased! mode changed: RINGBUFFER_MODE_PROCESSING");
            ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
        }
        return 0;
    }

    done = xRingbufferSend(s_ringbuf_i2s, (void *)data, size, (TickType_t)0);

    if (!done) {
        ESP_LOGW(BT_APP_CORE_TAG, "ringbuffer overflowed, ready to decrease data! mode changed: RINGBUFFER_MODE_DROPPING");
        ringbuffer_mode = RINGBUFFER_MODE_DROPPING;
    }

    if (ringbuffer_mode == RINGBUFFER_MODE_PREFETCHING) {
        vRingbufferGetInfo(s_ringbuf_i2s, NULL, NULL, NULL, NULL, &item_size);
        if (item_size >= RINGBUF_PREFETCH_WATER_LEVEL) {
            ESP_LOGI(BT_APP_CORE_TAG, "ringbuffer data increased! mode changed: RINGBUFFER_MODE_PROCESSING");
            ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
            if (pdFALSE == xSemaphoreGive(s_i2s_write_semaphore)) {
                ESP_LOGE(BT_APP_CORE_TAG, "semphore give failed");
            }
        }
    }

    return done ? size : 0;
}
