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

#include "bridge.h"




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

