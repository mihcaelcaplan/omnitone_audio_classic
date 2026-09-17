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
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "sys/lock.h"

// my drivers
#include "audio_out.h"
#include "bridge.h"
#include "sfx.h"

/* AVRCP used transaction labels */
#define APP_RC_CT_TL_GET_CAPS            (0)
#define APP_RC_CT_TL_GET_META_DATA       (1)
#define APP_RC_CT_TL_RN_TRACK_CHANGE     (2)
#define APP_RC_CT_TL_RN_PLAYBACK_CHANGE  (3)
#define APP_RC_CT_TL_RN_PLAY_POS_CHANGE  (4)
#define APP_RC_CT_TL_RESUME_PLAY         (5)

/* Application layer causes delay value */
#define APP_DELAY_VALUE                  50  // 5ms

/* Reconnect campaign: how many of the most recently bonded devices we are
 * willing to page, and how many pages to send before giving up. Every page that
 * goes unanswered costs one BT page timeout (~5s by default), so 4 attempts is
 * roughly 20s of trying -- which is also what paces the retries, see
 * bt_av_reconnect_page_next(). */
#define RECONNECT_MAX_PEERS              (2)
#define RECONNECT_MAX_ATTEMPTS           (1)

/* How close to the disconnect a stream stopping still counts as "the link took
 * the music with it" rather than "somebody pressed pause". Measured gap is about
 * 10ms, so this is enormously generous and still nowhere near human timing. */
#define AUDIO_STOP_GRACE_MS              (2000)

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
/* start a reconnect campaign, optionally resuming playback once it lands */
static void bt_av_reconnect_begin(bool resume_playback);
/* page the next device in a reconnect campaign */
static void bt_av_reconnect_page_next(void);
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
static esp_bd_addr_t s_reconnect_peers[RECONNECT_MAX_PEERS];
                                             /* devices to page, most recently connected first */
static int s_reconnect_peer_count = 0;       /* how many entries of s_reconnect_peers are valid */
static int s_reconnect_next_peer = 0;        /* which of those the next page targets */
static int s_reconnect_attempts_left = 0;    /* nonzero while a campaign is in progress */
static bool s_reconnect_resume_play = false; /* ask the peer to play once we are back */
static TickType_t s_audio_stopped_at = 0;    /* when the stream last went quiet */
static bool s_ota_hold = false;              /* firmware update in progress: the stack is going down, do not re-page */

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

/* debug: what a scanner should see for us -- our address and the class of
 * device the host stack believes it has written to the controller */
static void bt_av_log_identity(void)
{
    const uint8_t *a = esp_bt_dev_get_address();
    esp_bt_cod_t cod = {0};
    esp_bt_gap_get_cod(&cod);
    uint32_t raw = (cod.service << 13) | (cod.major << 8) | (cod.minor << 2) | cod.reserved_2;
    ESP_LOGI(BT_AV_TAG, "identity: bd_addr [%02x:%02x:%02x:%02x:%02x:%02x] cod 0x%06" PRIx32
             " (major 0x%02x minor 0x%02x service 0x%03x)",
             a[0], a[1], a[2], a[3], a[4], a[5], raw, cod.major, cod.minor, cod.service);
}

static void bt_av_reconnect_begin(bool resume_playback)
{
    /* Bluedroid keeps the bond list in NVS ordered by most recent ACL, so the
     * head of this list is the device we were last talking to. Re-reading it at
     * the top of every campaign means we always page in current recency order. */
    s_reconnect_peer_count = RECONNECT_MAX_PEERS;
    esp_err_t err = esp_bt_gap_get_bond_device_list(&s_reconnect_peer_count, s_reconnect_peers);
    if (err != ESP_OK || s_reconnect_peer_count == 0) {
        s_reconnect_peer_count = 0;
        s_reconnect_attempts_left = 0;
        s_reconnect_resume_play = false;
        ESP_LOGI(BT_AV_TAG, "reconnect: nothing bonded yet, waiting to be connected");
        bt_av_log_identity();
        return;
    }

    ESP_LOGI(BT_AV_TAG, "reconnect: trying the %d most recent device(s)", s_reconnect_peer_count);
    s_reconnect_next_peer = 0;
    s_reconnect_attempts_left = RECONNECT_MAX_ATTEMPTS;
    s_reconnect_resume_play = resume_playback;
    bt_av_reconnect_page_next();
}

/* Send one page to the next device in the list. We never wait on this: a page
 * that goes unanswered comes back as ESP_A2D_CONNECTION_STATE_DISCONNECTED once
 * the controller's page timeout expires, and that event calls us again. So the
 * radio itself paces the retries and there is no timer to run. */
static void bt_av_reconnect_page_next(void)
{
    uint8_t *peer = s_reconnect_peers[s_reconnect_next_peer];
    ESP_LOGI(BT_AV_TAG, "reconnect: paging [%02x:%02x:%02x:%02x:%02x:%02x], %d attempt(s) left",
             peer[0], peer[1], peer[2], peer[3], peer[4], peer[5], s_reconnect_attempts_left);
    esp_a2d_sink_connect(peer);

    /* line up the next one, wrapping back to the front of the list */
    s_reconnect_next_peer++;
    if (s_reconnect_next_peer >= s_reconnect_peer_count) {
        s_reconnect_next_peer = 0;
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
            /* The audio task owns the channel and releases it on its own once
             * both producers go quiet, so there is nothing to tear down here -
             * just stop being a producer. */
            audio_out_stream_stop();
            
            status_flags = (status_flags & (~EXT_MCU_BT_FLAG)); // turn off bt flag globally

            // bt_spi_task_shut_down(); //shut down task

            /* This disconnect is bt_av_shutdown() pulling the stack out from
             * under us for a firmware update. Paging again would just fail
             * against a controller that is on its way down. */
            if (s_ota_hold) {
                ESP_LOGI(BT_AV_TAG, "reconnect: not while shutting down for a firmware update");
                break;
            }
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

            /* Three different things all arrive here as DISCONNECTED, and only
             * one of them is somebody's decision:
             *   - a page we sent went unanswered: keep working through the list.
             *     disc_rsn is no help here, the failed-open path reports NORMAL
             *     just like a clean hang-up does, so the campaign counter is
             *     what tells us there was never a link in the first place
             *   - a link that was up dropped out: nobody chose that, go get it back
             *   - somebody hung up on purpose: leave them alone, otherwise the
             *     speaker would be impossible to walk away from */
            if (s_reconnect_attempts_left > 0) {
                /* the page we sent went unanswered; spend the attempt here, where
                 * the failure actually shows up, so the count stays truthful while
                 * the last page is still in flight */
                s_reconnect_attempts_left--;
                if (s_reconnect_attempts_left > 0) {
                    bt_av_reconnect_page_next();
                } else {
                    ESP_LOGI(BT_AV_TAG, "reconnect: out of attempts, staying connectable");
                    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                    bt_av_log_identity();

                    /* the offer to resume expires with the campaign: if they wander
                     * back an hour later and the phone reconnects on its own, music
                     * starting by itself would be a surprise, not a convenience */
                    s_reconnect_resume_play = false;
                }
            } else if (a2d->conn_stat.disc_rsn == ESP_A2D_DISC_RSN_ABNORMAL) {
                /* A dying link stops the stream on its way out -- bta_av_str_stopped
                 * reports SUSPEND about 10ms ahead of the disconnect -- so "is audio
                 * playing right now" is false here no matter what, and cannot be the
                 * test. What separates the cases is *when* the music stopped: in the
                 * same breath as the link, or minutes ago because somebody paused. */
                bool was_playing = (s_audio_state == ESP_A2D_AUDIO_STATE_STARTED) ||
                                   (xTaskGetTickCount() - s_audio_stopped_at) <
                                       pdMS_TO_TICKS(AUDIO_STOP_GRACE_MS);
                ESP_LOGI(BT_AV_TAG, "reconnect: link lost while %s, trying to get it back",
                         was_playing ? "playing" : "idle");
                bt_av_reconnect_begin(was_playing);
            }

        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED){
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

            /* whoever got here, we are done paging; a later dropout starts fresh */
            s_reconnect_attempts_left = 0;

            status_flags |= EXT_MCU_BT_FLAG; //add bt flag globally

            /* Declare the stream open rather than bringing up hardware: the
             * audio task opens the channel when a producer actually has
             * something for it. A reconnect campaign that leaves pages
             * unanswered therefore costs nothing. */
            // TODO: change i2s to spi and hand over better
            audio_out_stream_start();

            /* Connect tag. Fire and forget: it queues on the audio task, which
             * mixes it over the stream once one starts, so it does not matter
             * that this lands before the codec is configured or that the phone
             * may begin playing immediately. */
            sfx_play(SFX_CONNECTED);
            // bt_spi_task_start_up(); // create task
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
        } else {
            /* note when the music stopped: if a disconnect follows within a breath
             * of this, the link is what stopped it rather than a person */
            s_audio_stopped_at = xTaskGetTickCount();
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
            /* Hand the format to the audio task rather than touching the
             * peripheral here. It applies the change between mix chunks, so it
             * can never land mid-write, and a chime that happens to be playing
             * is retuned to the new rate instead of being cut off. */
            audio_out_set_format((uint32_t)sample_rate, (uint8_t)ch_count);

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

        /* A passthrough key is a press and a release, and our press just came
         * back answered -- so let go of it now rather than stuffing both halves
         * onto the wire back to back and hoping the peer keeps up. */
        if (rc->psth_rsp.tl == APP_RC_CT_TL_RESUME_PLAY &&
            rc->psth_rsp.key_code == ESP_AVRC_PT_CMD_PLAY &&
            rc->psth_rsp.key_state == ESP_AVRC_PT_CMD_STATE_PRESSED) {
            esp_avrc_ct_send_passthrough_cmd(APP_RC_CT_TL_RESUME_PLAY, ESP_AVRC_PT_CMD_PLAY,
                                             ESP_AVRC_PT_CMD_STATE_RELEASED);
        }
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

        /* We are here because the peer answered our first AVRCP command, which
         * makes this a better moment to ask for playback than the bare connect
         * event: the control channel is not just open, it is demonstrably
         * serving. Same idea as letting the page timeout pace the reconnect --
         * the protocol tells us when it is ready instead of a timer guessing. */
        if (s_reconnect_resume_play) {
            ESP_LOGI(BT_RC_CT_TAG, "reconnect: asking the peer to resume playback");
            esp_avrc_ct_send_passthrough_cmd(APP_RC_CT_TL_RESUME_PLAY, ESP_AVRC_PT_CMD_PLAY,
                                             ESP_AVRC_PT_CMD_STATE_PRESSED);
            s_reconnect_resume_play = false;   /* asked once; do not nag on the next response */
        }
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

void bt_av_reconnect_start(void)
{
    /* powering on is not a reason to make noise: come back to whoever we were
     * with, but wait to be asked before playing anything */
    bt_av_reconnect_begin(false);
}

void bt_av_shutdown(void)
{
    /* Called from the OTA erase task. One way: nothing here comes back short
     * of a reset, and ota_ctl resets on every exit from an update, successful
     * or not. Both disables block until the stack has actually stopped, so on
     * return there is no controller left to miss a slot while the flash is
     * being erased under it. */
    ESP_LOGI(BT_AV_TAG, "firmware update: shutting Bluetooth down");
    s_ota_hold = true;
    s_reconnect_attempts_left = 0;
    s_reconnect_resume_play = false;
    esp_bluedroid_disable();
    esp_bt_controller_disable();
}

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
 * for the stream: today it goes to the audio task, which mixes it with any
 * sound effect and owns the write to the DSP. Later this also selects
 * write_spi_queue() (raw PCM to the nRF for rebroadcast) -- the SPI bridge task
 * runs alongside as a status machine and must never contend for this data.
 *
 * This used to call i2s_channel_write() inline, which meant a DMA stall blocked
 * the Bluetooth stack. Handing off to a ringbuffer decouples the two, at the
 * cost of the prefetch latency audio_out.c documents.
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

    size_t bytes_written = audio_out_stream_write(data, len);

    if (++s_pkt_cnt % 100 == 0) {
        ESP_LOGI(BT_AV_TAG, "Audio packet count %"PRIu32", len %"PRIu32", vol 0x%02x, queued %u",
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
