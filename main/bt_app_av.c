/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"

#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
#include "driver/dac_continuous.h"
#else
#include "driver/i2s_std.h"
#endif

#include "sys/lock.h"

// my drivers
#include "bridge.h"

/* AVRCP used transaction labels */
#define APP_RC_CT_TL_GET_CAPS            (0)
#define APP_RC_CT_TL_GET_META_DATA       (1)
#define APP_RC_CT_TL_RN_TRACK_CHANGE     (2)
#define APP_RC_CT_TL_RN_PLAYBACK_CHANGE  (3)
#define APP_RC_CT_TL_RN_PLAY_POS_CHANGE  (4)

/* Application layer causes delay value */
#define APP_DELAY_VALUE                  50  // 5ms

/*******************************
 * STATIC FUNCTION DECLARATIONS
 ******************************/

/* allocate new meta buffer */
static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param);
/* handler for new track is loaded */
static void bt_av_new_track(void);
/* handler for track status change */
static void bt_av_playback_changed(void);
/* handler for track playing position change */
static void bt_av_play_pos_changed(void);
/* notification event handler */
static void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter);
/* installation for i2s */
static void bt_i2s_driver_install(void);
/* uninstallation for i2s */
static void bt_i2s_driver_uninstall(void);
/* set volume by remote controller */
static void volume_set_by_controller(uint8_t volume);
/* set volume by local host */
static void volume_set_by_local_host(uint8_t volume);
/* simulation volume change */
static void volume_change_simulation(void *arg);
/* a2dp event handler */
static void bt_av_hdl_a2d_evt(uint16_t event, void *p_param);
/* avrc controller event handler */
static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param);
/* avrc target event handler */
static void bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param);

/*******************************
 * STATIC VARIABLE DEFINITIONS
 ******************************/

static uint32_t s_pkt_cnt = 0;               /* count for audio packet */
static esp_a2d_audio_state_t s_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;
                                             /* audio stream datapath state */
static const char *s_a2d_conn_state_str[] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};
                                             /* connection state in string */
static const char *s_a2d_audio_state_str[] = {"Suspended", "Started"};
                                             /* audio stream datapath state in string */
static esp_avrc_rn_evt_cap_mask_t s_avrc_peer_rn_cap;
                                             /* AVRC target notification capability bit mask */
static _lock_t s_volume_lock;
static TaskHandle_t s_vcs_task_hdl = NULL;    /* handle for volume change simulation task */
static uint8_t s_volume = 0x40;                 /* local volume value */
static bool s_volume_notify;                 /* notify volume change or not */
#ifndef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
i2s_chan_handle_t tx_chan = NULL;
#else
dac_continuous_handle_t tx_chan;
#endif

#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
static bool cover_art_connected = false;
static bool cover_art_getting = false;
static uint32_t cover_art_image_size = 0;
static uint8_t image_handle_old[7];
#endif

/********************************
 * STATIC FUNCTION DEFINITIONS
 *******************************/

static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param)
{
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);
    uint8_t *attr_text = (uint8_t *) malloc (rc->meta_rsp.attr_length + 1);

    memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
    attr_text[rc->meta_rsp.attr_length] = 0;
    rc->meta_rsp.attr_text = attr_text;
}

#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
static bool image_handle_check(uint8_t *image_handle, int len)
{
    /* Image handle length must be 7 */
    if (len == 7 && memcmp(image_handle_old, image_handle, 7) != 0) {
        memcpy(image_handle_old, image_handle, 7);
        return true;
    }
    return false;
}
#endif

static void bt_av_new_track(void)
{
    /* request metadata */
    uint8_t attr_mask = ESP_AVRC_MD_ATTR_TITLE |
                        ESP_AVRC_MD_ATTR_ARTIST |
                        ESP_AVRC_MD_ATTR_ALBUM |
                        ESP_AVRC_MD_ATTR_GENRE;
#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
    if (cover_art_connected) {
        attr_mask |= ESP_AVRC_MD_ATTR_COVER_ART;
    }
#endif
    esp_avrc_ct_send_metadata_cmd(APP_RC_CT_TL_GET_META_DATA, attr_mask);

    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_TRACK_CHANGE)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_TRACK_CHANGE,
                                                   ESP_AVRC_RN_TRACK_CHANGE, 0);
    }
}

static void bt_av_playback_changed(void)
{
    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_PLAY_STATUS_CHANGE)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_PLAYBACK_CHANGE,
                                                   ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);
    }
}

static void bt_av_play_pos_changed(void)
{
    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_PLAY_POS_CHANGED)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_PLAY_POS_CHANGE,
                                                   ESP_AVRC_RN_PLAY_POS_CHANGED, 10);
    }
}

static void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter)
{
    switch (event_id) {
    /* when new track is loaded, this event comes */
    case ESP_AVRC_RN_TRACK_CHANGE:
        bt_av_new_track();
        break;
    /* when track status changed, this event comes */
    case ESP_AVRC_RN_PLAY_STATUS_CHANGE:
        ESP_LOGI(BT_AV_TAG, "Playback status changed: 0x%x", event_parameter->playback);
        bt_av_playback_changed();
        break;
    /* when track playing position changed, this event comes */
    case ESP_AVRC_RN_PLAY_POS_CHANGED:
        ESP_LOGI(BT_AV_TAG, "Play position changed: %"PRIu32"-ms", event_parameter->play_pos);
        bt_av_play_pos_changed();
        break;
    /* others */
    default:
        ESP_LOGI(BT_AV_TAG, "unhandled event: %d", event_id);
        break;
    }
}

void bt_i2s_driver_install(void)
{
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
    dac_continuous_config_t cont_cfg = {
        .chan_mask = DAC_CHANNEL_MASK_ALL,
        .desc_num = 8,
        .buf_size = 2048,
        .freq_hz = 44100,
        .offset = 127,
        .clk_src = DAC_DIGI_CLK_SRC_DEFAULT,   // Using APLL as clock source to get a wider frequency range
        .chan_mode = DAC_CHANNEL_MODE_ALTER,
    };
    /* Allocate continuous channels */
    ESP_ERROR_CHECK(dac_continuous_new_channels(&cont_cfg, &tx_chan));
    /* Enable the continuous channels */
    ESP_ERROR_CHECK(dac_continuous_enable(tx_chan));
#else
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100), //TODO: change to 48k or see how ASRC handles
        // .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO), //TODO: confirm this is wrong
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = CONFIG_EXAMPLE_I2S_BCK_PIN,
            .ws = CONFIG_EXAMPLE_I2S_LRCK_PIN,
            .dout = CONFIG_EXAMPLE_I2S_DATA_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    /* enable I2S */
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, NULL));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    ESP_LOGI(BT_AV_TAG, "I2S installed and enabled: bclk %d, ws %d, dout %d",
             CONFIG_EXAMPLE_I2S_BCK_PIN, CONFIG_EXAMPLE_I2S_LRCK_PIN, CONFIG_EXAMPLE_I2S_DATA_PIN);
#endif
}

void bt_i2s_driver_uninstall(void)
{
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
    ESP_ERROR_CHECK(dac_continuous_disable(tx_chan));
    ESP_ERROR_CHECK(dac_continuous_del_channels(tx_chan));
#else
    ESP_ERROR_CHECK(i2s_channel_disable(tx_chan));
    ESP_ERROR_CHECK(i2s_del_channel(tx_chan));
    tx_chan = NULL;
#endif
}

static void volume_set_by_controller(uint8_t volume)
{
    ESP_LOGI(BT_RC_TG_TAG, "Volume is set by remote controller to: %"PRIu32"%%", (uint32_t)volume * 100 / 0x7f);
    /* set the volume in protection of lock */
    _lock_acquire(&s_volume_lock);
    s_volume = volume;
    _lock_release(&s_volume_lock);
}

static void volume_set_by_local_host(uint8_t volume)
{
    ESP_LOGI(BT_RC_TG_TAG, "Volume is set locally to: %"PRIu32"%%", (uint32_t)volume * 100 / 0x7f);
    /* set the volume in protection of lock */
    _lock_acquire(&s_volume_lock);
    s_volume = volume;
    _lock_release(&s_volume_lock);

    /* send notification response to remote AVRCP controller */
    if (s_volume_notify) {
        esp_avrc_rn_param_t rn_param;
        rn_param.volume = s_volume;
        esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_CHANGED, &rn_param);
        s_volume_notify = false;
    }
}

static void volume_change_simulation(void *arg)
{
    ESP_LOGI(BT_RC_TG_TAG, "start volume change simulation");

    for (;;) {
        /* volume up locally every 10 seconds */
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        uint8_t volume = (s_volume + 5) & 0x7f;
        volume_set_by_local_host(volume);
    }
}

static void bt_av_hdl_a2d_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s event: %d", __func__, event);

    esp_a2d_cb_param_t *a2d = NULL;

    switch (event) {
    /* when connection state changed, this event comes */
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        uint8_t *bda = a2d->conn_stat.remote_bda;
        ESP_LOGI(BT_AV_TAG, "A2DP connection state: %s, [%02x:%02x:%02x:%02x:%02x:%02x]",
            s_a2d_conn_state_str[a2d->conn_stat.state], bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        
            if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            /* stop the writer before destroying what it writes to: the I2S
             * task runs unpinned at high priority, so on the other core it can
             * be entering i2s_channel_write() with a handle this task is about
             * to i2s_del_channel() -> "this channel is not tx channel" */
            bt_i2s_task_shut_down();
            bt_i2s_driver_uninstall();
            
            status_flags = (status_flags & (~EXT_MCU_BT_FLAG)); // turn off bt flag globally

            // bt_spi_task_shut_down(); //shut down task


        
        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED){
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

            status_flags |= EXT_MCU_BT_FLAG; //add bt flag globally
            
            // TODO: change i2s to spi and hand over better
            bt_i2s_task_start_up();
            // bt_spi_task_start_up(); // create task
        
        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
            
            bt_i2s_driver_install();
        }
        break;
    }
    /* when audio stream transmission state changed, this event comes */
    case ESP_A2D_AUDIO_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "A2DP audio state: %s", s_a2d_audio_state_str[a2d->audio_stat.state]);
        s_audio_state = a2d->audio_stat.state;
        if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
            s_pkt_cnt = 0;
        }
        break;
    }
    /* when audio codec is configured, this event comes */
    case ESP_A2D_AUDIO_CFG_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        esp_a2d_mcc_t *p_mcc = &a2d->audio_cfg.mcc;
        ESP_LOGI(BT_AV_TAG, "A2DP audio stream configuration, codec type: %d", p_mcc->type);
        /* for now only SBC stream is supported */
        if (p_mcc->type == ESP_A2D_MCT_SBC) {
            int sample_rate = 16000;
            int ch_count = 2;
            if (p_mcc->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_32K) {
                sample_rate = 32000;
            } else if (p_mcc->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_44K) {
                sample_rate = 44100;
            } else if (p_mcc->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_48K) {
                sample_rate = 48000;
            }

            if (p_mcc->cie.sbc_info.ch_mode & ESP_A2D_SBC_CIE_CH_MODE_MONO) {
                ch_count = 1;
            }
        #ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
            dac_continuous_disable(tx_chan);
            dac_continuous_del_channels(tx_chan);
            dac_continuous_config_t cont_cfg = {
                .chan_mask = DAC_CHANNEL_MASK_ALL,
                .desc_num = 8,
                .buf_size = 2048,
                .freq_hz = sample_rate,
                .offset = 127,
                .clk_src = DAC_DIGI_CLK_SRC_DEFAULT,   // Using APLL as clock source to get a wider frequency range
                .chan_mode = (ch_count == 1) ? DAC_CHANNEL_MODE_SIMUL : DAC_CHANNEL_MODE_ALTER,
            };
            /* Allocate continuous channels */
            dac_continuous_new_channels(&cont_cfg, &tx_chan);
            /* Enable the continuous channels */
            dac_continuous_enable(tx_chan);
        #else
            /* the channel is taken down here and only comes back at the enable
             * below, so a failure anywhere in between leaves the interface dead
             * with no clocks on the pins -- these returns must not be dropped */
            if (tx_chan == NULL) {
                ESP_LOGE(BT_AV_TAG, "audio cfg arrived with no I2S channel installed");
            } else {
                esp_err_t cfg_err;
                i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
                i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, ch_count);
                if ((cfg_err = i2s_channel_disable(tx_chan)) != ESP_OK) {
                    ESP_LOGE(BT_AV_TAG, "i2s_channel_disable: %s", esp_err_to_name(cfg_err));
                }
                if ((cfg_err = i2s_channel_reconfig_std_clock(tx_chan, &clk_cfg)) != ESP_OK) {
                    ESP_LOGE(BT_AV_TAG, "i2s_channel_reconfig_std_clock: %s", esp_err_to_name(cfg_err));
                }
                if ((cfg_err = i2s_channel_reconfig_std_slot(tx_chan, &slot_cfg)) != ESP_OK) {
                    ESP_LOGE(BT_AV_TAG, "i2s_channel_reconfig_std_slot: %s", esp_err_to_name(cfg_err));
                }
                if ((cfg_err = i2s_channel_enable(tx_chan)) != ESP_OK) {
                    ESP_LOGE(BT_AV_TAG, "i2s_channel_enable: %s", esp_err_to_name(cfg_err));
                } else {
                    ESP_LOGI(BT_AV_TAG, "I2S re-enabled: %d Hz, %d ch", sample_rate, ch_count);
                }
            }
        #endif
            ESP_LOGI(BT_AV_TAG, "Configure audio player: 0x%x-0x%x-0x%x-0x%x-0x%x-%d-%d",
                     p_mcc->cie.sbc_info.samp_freq,
                     p_mcc->cie.sbc_info.ch_mode,
                     p_mcc->cie.sbc_info.block_len,
                     p_mcc->cie.sbc_info.num_subbands,
                     p_mcc->cie.sbc_info.alloc_mthd,
                     p_mcc->cie.sbc_info.min_bitpool,
                     p_mcc->cie.sbc_info.max_bitpool);
            ESP_LOGI(BT_AV_TAG, "Audio player configured, sample rate: %d", sample_rate);
        }
        break;
    }
    /* when a2dp init or deinit completed, this event comes */
    case ESP_A2D_PROF_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        if (ESP_A2D_INIT_SUCCESS == a2d->a2d_prof_stat.init_state) {
            ESP_LOGI(BT_AV_TAG, "A2DP PROF STATE: Init Complete");
        } else {
            ESP_LOGI(BT_AV_TAG, "A2DP PROF STATE: Deinit Complete");
        }
        break;
    }
    /* when using external codec, after sep registration done, this event comes */
    case ESP_A2D_SEP_REG_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        if (a2d->a2d_sep_reg_stat.reg_state == ESP_A2D_SEP_REG_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "A2DP register SEP success, seid: %d", a2d->a2d_sep_reg_stat.seid);
        }
        else {
            ESP_LOGI(BT_AV_TAG, "A2DP register SEP fail, seid: %d, state: %d", a2d->a2d_sep_reg_stat.seid, a2d->a2d_sep_reg_stat.reg_state);
        }
        break;
    }
    /* When protocol service capabilities configured, this event comes */
    case ESP_A2D_SNK_PSC_CFG_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "protocol service capabilities configured: 0x%x ", a2d->a2d_psc_cfg_stat.psc_mask);
        if (a2d->a2d_psc_cfg_stat.psc_mask & ESP_A2D_PSC_DELAY_RPT) {
            ESP_LOGI(BT_AV_TAG, "Peer device support delay reporting");
        } else {
            ESP_LOGI(BT_AV_TAG, "Peer device unsupported delay reporting");
        }
        break;
    }
    /* when set delay value completed, this event comes */
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        if (ESP_A2D_SET_INVALID_PARAMS == a2d->a2d_set_delay_value_stat.set_state) {
            ESP_LOGI(BT_AV_TAG, "Set delay report value: fail");
        } else {
            ESP_LOGI(BT_AV_TAG, "Set delay report value: success, delay_value: %u * 1/10 ms", a2d->a2d_set_delay_value_stat.delay_value);
        }
        break;
    }
    /* when get delay value completed, this event comes */
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "Get delay report value: delay_value: %u * 1/10 ms", a2d->a2d_get_delay_value_stat.delay_value);
        /* Default delay value plus delay caused by application layer */
        esp_a2d_sink_set_delay_value(a2d->a2d_get_delay_value_stat.delay_value + APP_DELAY_VALUE);
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_RC_CT_TAG, "%s event: %d", __func__, event);

    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(p_param);

    switch (event) {
    /* when connection state changed, this event comes */
    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
        uint8_t *bda = rc->conn_stat.remote_bda;
        ESP_LOGI(BT_RC_CT_TAG, "AVRC conn_state event: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

        if (rc->conn_stat.connected) {
            /* get remote supported event_ids of peer AVRCP Target */
            esp_avrc_ct_send_get_rn_capabilities_cmd(APP_RC_CT_TL_GET_CAPS);
        } else {
            /* clear peer notification capability record */
            s_avrc_peer_rn_cap.bits = 0;
        }
        break;
    }
    /* when passthrough response, this event comes */
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC passthrough rsp: key_code 0x%x, key_state %d, rsp_code %d", rc->psth_rsp.key_code,
                    rc->psth_rsp.key_state, rc->psth_rsp.rsp_code);
        break;
    }
    /* when metadata response, this event comes */
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC metadata rsp: attribute id 0x%x, %s", rc->meta_rsp.attr_id, rc->meta_rsp.attr_text);
#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
        if(rc->meta_rsp.attr_id == 0x80 && cover_art_connected && cover_art_getting == false) {
            /* check image handle is valid and different with last one, wo dont want to get an image repeatedly */
            if(image_handle_check(rc->meta_rsp.attr_text, rc->meta_rsp.attr_length)) {
                esp_avrc_ct_cover_art_get_linked_thumbnail(rc->meta_rsp.attr_text);
                cover_art_getting = true;
            }
        }
#endif
        free(rc->meta_rsp.attr_text);
        break;
    }
    /* when notified, this event comes */
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC event notification: %d", rc->change_ntf.event_id);
        bt_av_notify_evt_handler(rc->change_ntf.event_id, &rc->change_ntf.event_parameter);
        break;
    }
    /* when feature of remote device indicated, this event comes */
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC remote features %"PRIx32", TG features %x", rc->rmt_feats.feat_mask, rc->rmt_feats.tg_feat_flag);
#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
        if ((rc->rmt_feats.tg_feat_flag & ESP_AVRC_FEAT_FLAG_TG_COVER_ART) && !cover_art_connected) {
            ESP_LOGW(BT_RC_CT_TAG, "Peer support Cover Art feature, start connection...");
            /* set mtu to zero to use a default value */
            esp_avrc_ct_cover_art_connect(0);
        }
#endif
        break;
    }
    /* when notification capability of peer device got, this event comes */
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "remote rn_cap: count %d, bitmask 0x%x", rc->get_rn_caps_rsp.cap_count,
                 rc->get_rn_caps_rsp.evt_set.bits);
        s_avrc_peer_rn_cap.bits = rc->get_rn_caps_rsp.evt_set.bits;
        bt_av_new_track();
        bt_av_playback_changed();
        bt_av_play_pos_changed();
        break;
    }
    case ESP_AVRC_CT_COVER_ART_STATE_EVT: {
#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
        if (rc->cover_art_state.state == ESP_AVRC_COVER_ART_CONNECTED) {
            cover_art_connected = true;
            ESP_LOGW(BT_RC_CT_TAG, "Cover Art Client connected");
        }
        else {
            cover_art_connected = false;
            ESP_LOGW(BT_RC_CT_TAG, "Cover Art Client disconnected, reason:%d", rc->cover_art_state.reason);
        }
#endif
        break;
    }
    case ESP_AVRC_CT_COVER_ART_DATA_EVT: {
#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
        /* when rc->cover_art_data.final is true, it means we have received the entire image or get operation failed */
        if (rc->cover_art_data.final) {
            if(rc->cover_art_data.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(BT_RC_CT_TAG, "Cover Art Client final data event, image size: %lu bytes", cover_art_image_size);
            }
            else {
                ESP_LOGE(BT_RC_CT_TAG, "Cover Art Client get operation failed");
            }
            cover_art_image_size = 0;
            /* set the getting state to false, we can get next image now */
            cover_art_getting = false;
        }
#endif
        break;
    }
    /* when avrcp controller init or deinit completed, this event comes */
    case ESP_AVRC_CT_PROF_STATE_EVT: {
        if (ESP_AVRC_INIT_SUCCESS == rc->avrc_ct_init_stat.state) {
            ESP_LOGI(BT_RC_CT_TAG, "AVRCP CT STATE: Init Complete");
        } else if (ESP_AVRC_DEINIT_SUCCESS == rc->avrc_ct_init_stat.state) {
            ESP_LOGI(BT_RC_CT_TAG, "AVRCP CT STATE: Deinit Complete");
        } else {
            ESP_LOGE(BT_RC_CT_TAG, "AVRCP CT STATE error: %d", rc->avrc_ct_init_stat.state);
        }
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_RC_CT_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

static void bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_RC_TG_TAG, "%s event: %d", __func__, event);

    esp_avrc_tg_cb_param_t *rc = (esp_avrc_tg_cb_param_t *)(p_param);

    switch (event) {
    /* when connection state changed, this event comes */
    case ESP_AVRC_TG_CONNECTION_STATE_EVT: {
        uint8_t *bda = rc->conn_stat.remote_bda;
        ESP_LOGI(BT_RC_TG_TAG, "AVRC conn_state evt: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        if (rc->conn_stat.connected) {
            /* create task to simulate volume change */ //TODO: add a task here to handle volume control by remote
            // xTaskCreate(volume_change_simulation, "vcsTask", 2048, NULL, 5, &s_vcs_task_hdl);
        } else if (s_vcs_task_hdl) {
            /* the create above is commented out, so this handle is NULL --
             * and vTaskDelete(NULL) deletes the CALLING task, which here is
             * BtAppTask, silently killing the whole app event pipeline */
            vTaskDelete(s_vcs_task_hdl);
            s_vcs_task_hdl = NULL;
            ESP_LOGI(BT_RC_TG_TAG, "Stop volume change simulation");
        }
        break;
    }
    /* when passthrough commanded, this event comes */
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC passthrough cmd: key_code 0x%x, key_state %d", rc->psth_cmd.key_code, rc->psth_cmd.key_state);
        break;
    }
    /* when absolute volume command from remote device set, this event comes */
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC set absolute volume: %d%%", (int)rc->set_abs_vol.volume * 100 / 0x7f);
        volume_set_by_controller(rc->set_abs_vol.volume);
        break;
    }
    /* when notification registered, this event comes */
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC register event notification: %d, param: 0x%"PRIx32, rc->reg_ntf.event_id, rc->reg_ntf.event_parameter);
        if (rc->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
            s_volume_notify = true;
            esp_avrc_rn_param_t rn_param;
            rn_param.volume = s_volume;
            esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn_param);
        }
        break;
    }
    /* when feature of remote device indicated, this event comes */
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC remote features: %"PRIx32", CT features: %x", rc->rmt_feats.feat_mask, rc->rmt_feats.ct_feat_flag);
        break;
    }
    /* when avrcp target init or deinit completed, this event comes */
    case ESP_AVRC_TG_PROF_STATE_EVT: {
        if (ESP_AVRC_INIT_SUCCESS == rc->avrc_tg_init_stat.state) {
            ESP_LOGI(BT_RC_CT_TAG, "AVRCP TG STATE: Init Complete");
        } else if (ESP_AVRC_DEINIT_SUCCESS == rc->avrc_tg_init_stat.state) {
            ESP_LOGI(BT_RC_CT_TAG, "AVRCP TG STATE: Deinit Complete");
        } else {
            ESP_LOGE(BT_RC_CT_TAG, "AVRCP TG STATE error: %d", rc->avrc_tg_init_stat.state);
        }
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_RC_TG_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

/********************************
 * EXTERNAL FUNCTION DEFINITIONS
 *******************************/

void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_PROF_STATE_EVT:
    case ESP_A2D_SEP_REG_STATE_EVT:
    case ESP_A2D_SNK_PSC_CFG_EVT:
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT:
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {
        bt_app_work_dispatch(bt_av_hdl_a2d_evt, event, param, sizeof(esp_a2d_cb_param_t), NULL);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "Invalid A2DP event: %d", event);
        break;
    }
}

#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE

/*
 * Decoded PCM arrives here on the BTC task. This is the single fan-out point
 * for the stream: today it goes straight out I2S to the DSP. Later this
 * selects between the I2S sink and write_spi_queue() (raw PCM to the nRF for
 * rebroadcast) -- the SPI bridge task runs alongside as a status machine and
 * must never contend for this data. Not built yet.
 *
 * Note this writes I2S inline, so a DMA stall blocks the Bluetooth stack.
 * bt_i2s_task_handler() + write_ringbuf() in bt_app_core.c are the decoupled
 * path if that becomes a problem; they are left in place but unused.
 */
void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    _lock_acquire(&s_volume_lock);
    uint8_t vol = s_volume;
    _lock_release(&s_volume_lock);

    if (vol != 0x7f) {
        int16_t *samples = (int16_t *)data;
        uint32_t n = len / sizeof(int16_t);
        for (uint32_t i = 0; i < n; i++) {
            samples[i] = (int16_t)((int32_t)samples[i] * vol / 0x7f);
        }
    }

    size_t bytes_written = 0;
    /* uninstall runs on the app task and nulls this on disconnect */
    if (tx_chan != NULL) {
        esp_err_t err = i2s_channel_write(tx_chan, data, len, &bytes_written, portMAX_DELAY);
        if (err != ESP_OK || bytes_written != len) {
            ESP_LOGW(BT_AV_TAG, "i2s write: %s, %u/%"PRIu32" bytes",
                     esp_err_to_name(err), (unsigned)bytes_written, len);
        }
    }

    if (++s_pkt_cnt % 100 == 0) {
        ESP_LOGI(BT_AV_TAG, "Audio packet count %"PRIu32", len %"PRIu32", vol 0x%02x, wrote %u",
                 s_pkt_cnt, len, vol, (unsigned)bytes_written);
    }
}

#else

void bt_app_a2d_audio_data_cb(esp_a2d_conn_hdl_t conn_hdl, esp_a2d_audio_buff_t *audio_buf)
{
    ESP_LOGI(BT_AV_TAG, "data_len: %d, number_frame: %d, ts: %lu", audio_buf->data_len, audio_buf->number_frame, audio_buf->timestamp);

    /*
     * Normally, user should send the audio_buf to other task, decode and free audio buff,
     * But the codec component is not merge into IDF now, so we just free audio data here
     */
    esp_a2d_audio_buff_free(audio_buf);
}

#endif

void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
#if CONFIG_EXAMPLE_AVRCP_CT_COVER_ART_ENABLE
    /* we must handle ESP_AVRC_CT_COVER_ART_DATA_EVT in this callback, copy image data to other buff before return if need */
    if (event == ESP_AVRC_CT_COVER_ART_DATA_EVT && param->cover_art_data.status == ESP_BT_STATUS_SUCCESS) {
        cover_art_image_size += param->cover_art_data.data_len;
        /* copy image data to other place */
        /* memcpy(p_buf, param->cover_art_data.p_data, param->cover_art_data.data_len); */
    }
#endif
    switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
        bt_app_alloc_meta_buffer(param);
        /* fall through */
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
    case ESP_AVRC_CT_COVER_ART_STATE_EVT:
    case ESP_AVRC_CT_COVER_ART_DATA_EVT:
    case ESP_AVRC_CT_PROF_STATE_EVT: {
        bt_app_work_dispatch(bt_av_hdl_avrc_ct_evt, event, param, sizeof(esp_avrc_ct_cb_param_t), NULL);
        break;
    }
    default:
        ESP_LOGE(BT_RC_CT_TAG, "Invalid AVRC event: %d", event);
        break;
    }
}

void bt_app_rc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT:
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
    case ESP_AVRC_TG_SET_PLAYER_APP_VALUE_EVT:
    case ESP_AVRC_TG_PROF_STATE_EVT:
        bt_app_work_dispatch(bt_av_hdl_avrc_tg_evt, event, param, sizeof(esp_avrc_tg_cb_param_t), NULL);
        break;
    default:
        ESP_LOGE(BT_RC_TG_TAG, "Invalid AVRC event: %d", event);
        break;
    }
}
