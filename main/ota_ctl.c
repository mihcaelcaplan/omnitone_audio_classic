#include "ota_ctl.h"
#include "bridge.h"
#include "bt_app_av.h"

#include <assert.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_rom_crc.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "OMNI_OTA";

// how long after acking OTA_REBOOT / a pending-state OTA_ABORT we actually go
// down: long enough for the ack to be clocked out on the master's next poll
#define OTA_DEFERRED_ACTION_MS 500

// how long a failed or aborted update lingers before the restart that brings
// Bluetooth back: long enough for the master to ack-and-read the error out of
// OTA_STATE (it polls every 100 ms) before the reset wipes it
#define OTA_FAIL_RESTART_MS 2000

// stack for the one-shot erase / verify workers. esp_ota_end() walks the whole
// image through the SHA engine and needs more than the SPI task can spare.
#define OTA_WORKER_STACK 8192
#define OTA_WORKER_PRIO  5

// how long the erase task will wait for Bluetooth bring-up to finish before
// giving up on the update. Bring-up is a couple of seconds; the nRF's erase
// timeout is 60 s and this has to fit inside it.
#define BT_READY_BIT        (1u << 0)
#define OTA_BT_READY_WAIT_MS 20000

typedef enum {
    DEFERRED_NONE = 0,
    DEFERRED_RESTART,   // after OTA_REBOOT
    DEFERRED_ROLLBACK,  // OTA_ABORT while PENDING_CONFIRM
} deferred_action_t;

static struct {
    SemaphoreHandle_t lock;
    ota_state_t state;
    esp_err_t err;
    uint16_t next_seq;
    uint32_t image_size;
    uint32_t written;
    uint8_t last_logged_pct;
    uint32_t write_us_max;  // slowest esp_ota_write this session; sets the master's block gap
    const esp_partition_t *target;
    esp_ota_handle_t handle;
    bool handle_open;
    EventGroupHandle_t bt_ready;  // BT_READY_BIT once Bluetooth bring-up is over; the erase waits for it
    bool bt_down;           // Bluetooth was shut down for this update; only a reset brings it back
    bool abort_requested;   // OTA_ABORT arrived while a worker was mid-erase / mid-verify
    deferred_action_t deferred_action;
    esp_timer_handle_t deferred_tmr;  // one-shot: restart / rollback
    esp_timer_handle_t confirm_tmr;   // PENDING_CONFIRM -> rollback if the nRF never confirms
    esp_timer_handle_t idle_tmr;      // RECEIVING / READY_TO_REBOOT -> ERROR if the nRF goes quiet
} s;

static const char *state_name(ota_state_t st)
{
    static const char *names[] = {
        "IDLE", "ERASING", "RECEIVING", "VERIFYING", "READY_TO_REBOOT", "PENDING_CONFIRM", "ERROR",
    };
    return (st < sizeof(names) / sizeof(names[0])) ? names[st] : "?";
}

#define LOCK()   xSemaphoreTake(s.lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s.lock)

/* Lock held. Moves the state and keeps the three status_flags bits in step. */
static void set_state(ota_state_t st, esp_err_t err)
{
    ESP_LOGI(TAG, "%s -> %s", state_name(s.state), state_name(st));
    s.state = st;
    s.err = err;

    uint8_t f = status_flags & ~(EXT_MCU_OTA_BUSY_FLAG | EXT_MCU_OTA_PENDING_FLAG | EXT_MCU_OTA_ERROR_FLAG);
    switch (st) {
    case OTA_STATE_ERASING:
    case OTA_STATE_RECEIVING:
    case OTA_STATE_VERIFYING:
    case OTA_STATE_READY_TO_REBOOT:
        f |= EXT_MCU_OTA_BUSY_FLAG;
        break;
    case OTA_STATE_PENDING_CONFIRM:
        f |= EXT_MCU_OTA_PENDING_FLAG;
        break;
    case OTA_STATE_ERROR:
        f |= EXT_MCU_OTA_ERROR_FLAG;
        break;
    default:
        break;
    }
    status_flags = f;
}

/* Lock held. Re-arm the "nRF went quiet" timer. */
static void touch_idle(void)
{
    esp_timer_stop(s.idle_tmr);
    esp_timer_start_once(s.idle_tmr, (uint64_t)CONFIG_OMNI_OTA_IDLE_TIMEOUT_S * 1000000ULL);
}

/* Lock held. Bluetooth went down for this update and nothing in our code
 * brings it back up: every way out of the session other than booting the new
 * image is a restart into the old one, which redoes the whole bring-up for
 * free. Deferred so the reply to whatever command got us here, or the master's
 * next OTA_STATE poll after a worker failure, still goes out first. */
static void restart_if_bt_down(void)
{
    if (!s.bt_down || s.deferred_action != DEFERRED_NONE) {
        return;
    }
    ESP_LOGW(TAG, "Bluetooth is down for this update: restarting in %d ms", OTA_FAIL_RESTART_MS);
    s.deferred_action = DEFERRED_RESTART;
    esp_timer_start_once(s.deferred_tmr, OTA_FAIL_RESTART_MS * 1000ULL);
}

/* Lock held. Drop everything and go back to IDLE (by way of a reset, if Bluetooth is down). */
static void to_idle(const char *why)
{
    ESP_LOGI(TAG, "back to idle: %s", why);
    esp_timer_stop(s.idle_tmr);
    if (s.handle_open) {
        esp_ota_abort(s.handle);
        s.handle_open = false;
    }
    s.abort_requested = false;
    s.written = 0;
    s.next_seq = 0;
    set_state(OTA_STATE_IDLE, ESP_OK);
    restart_if_bt_down();
}

/* Lock held. Fail the update; the nRF clears this with OTA_ABORT. */
static void enter_error(esp_err_t err, const char *what)
{
    ESP_LOGE(TAG, "%s: %s (0x%x)", what, esp_err_to_name(err), err);
    esp_timer_stop(s.idle_tmr);
    if (s.handle_open) {
        esp_ota_abort(s.handle);
        s.handle_open = false;
    }
    s.abort_requested = false;
    set_state(OTA_STATE_ERROR, err);
    restart_if_bt_down();
}

/* Lock held. Undo the esp_ota_set_boot_partition(target) that READY_TO_REBOOT
 * did. Pointing otadata back at the running slot is not enough on its own:
 * with app rollback on, esp_ota_set_boot_partition() writes a fresh entry in
 * state NEW, the bootloader promotes that to PENDING_VERIFY on the restart
 * that follows, and this image - the one that has been running fine all along
 * - comes back up PENDING_CONFIRM. The nRF sees the old version string, does
 * not confirm, and the confirm timeout then rolls "back" into the very image
 * that was just aborted. Marking the entry VALID right away keeps the running
 * image a plain known-good boot. */
static void restore_running_partition(void)
{
    esp_err_t err = esp_ota_set_boot_partition(esp_ota_get_running_partition());
    if (err == ESP_OK) {
        err = esp_ota_mark_app_valid_cancel_rollback();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "restoring the running partition as boot: %s", esp_err_to_name(err));
    }
}

/*******************************
 * WORKERS AND TIMERS
 ******************************/

static void erase_task(void *arg)
{
    esp_ota_handle_t h = 0;

    // The SPI task is up before the controller is, and the nRF starts a queued
    // update as soon as STATUS replies arrive. An OTA_BEGIN that lands in that
    // window has nothing to shut down yet, and app_main would then enable the
    // controller underneath the erase. So wait for bring-up to finish first;
    // we are already reporting ERASING, which the nRF gives 60 s.
    if (!(xEventGroupWaitBits(s.bt_ready, BT_READY_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(OTA_BT_READY_WAIT_MS)) & BT_READY_BIT)) {
        LOCK();
        if (s.abort_requested) {
            to_idle("aborted while waiting for Bluetooth bring-up");
        } else {
            enter_error(ESP_ERR_TIMEOUT, "Bluetooth bring-up never finished");
        }
        UNLOCK();
        vTaskDelete(NULL);
        return;
    }

    // Bluetooth first, and all the way down. The controller runs to hard
    // real-time deadlines and a sector erase stalls both cores' caches; with a
    // link or page in flight that ends in a controller assert (ld_acl.c). We
    // don't get it back until the reset that ends this update either way.
    bt_av_shutdown();

    int64_t t0 = esp_timer_get_time();
    // erases just enough of the slot for image_size, sector by sector, yielding
    // between sectors so the SPI task keeps answering OTA_STATE meanwhile
    esp_err_t err = esp_ota_begin(s.target, s.image_size, &h);

    LOCK();
    if (s.abort_requested) {
        if (err == ESP_OK) {
            esp_ota_abort(h);
        }
        to_idle("aborted during erase");
    } else if (err != ESP_OK) {
        enter_error(err, "esp_ota_begin");
    } else {
        s.handle = h;
        s.handle_open = true;
        ESP_LOGI(TAG, "slot erased in %lld ms, accepting blocks", (esp_timer_get_time() - t0) / 1000);
        set_state(OTA_STATE_RECEIVING, ESP_OK);
        touch_idle();
    }
    UNLOCK();
    vTaskDelete(NULL);
}

static void verify_task(void *arg)
{
    int64_t t0 = esp_timer_get_time();
    // checks the image header, the appended SHA-256, and the signature if
    // CONFIG_SECURE_SIGNED_APPS_NO_SECURE_BOOT is on. Consumes the handle.
    esp_err_t err = esp_ota_end(s.handle);

    LOCK();
    s.handle_open = false;
    if (s.abort_requested) {
        to_idle("aborted during verify");
    } else if (err != ESP_OK) {
        enter_error(err, "esp_ota_end (image verify)");
    } else {
        ESP_LOGI(TAG, "image verified in %lld ms", (esp_timer_get_time() - t0) / 1000);
        err = esp_ota_set_boot_partition(s.target);
        if (err != ESP_OK) {
            enter_error(err, "esp_ota_set_boot_partition");
        } else {
            ESP_LOGI(TAG, "%s is now the boot partition, waiting for OTA_REBOOT", s.target->label);
            set_state(OTA_STATE_READY_TO_REBOOT, ESP_OK);
            touch_idle();
        }
    }
    UNLOCK();
    vTaskDelete(NULL);
}

static void deferred_cb(void *arg)
{
    LOCK();
    deferred_action_t a = s.deferred_action;
    UNLOCK();

    switch (a) {
    case DEFERRED_RESTART:
        ESP_LOGI(TAG, "restarting");
        esp_restart();
        break;
    case DEFERRED_ROLLBACK:
        ESP_LOGW(TAG, "rolling back on request");
        esp_ota_mark_app_invalid_rollback_and_reboot();
        break;
    default:
        break;
    }
}

static void confirm_timeout_cb(void *arg)
{
    LOCK();
    bool pending = (s.state == OTA_STATE_PENDING_CONFIRM);
    UNLOCK();
    if (pending) {
        ESP_LOGE(TAG, "no OTA_CONFIRM within %d s, rolling back", CONFIG_OMNI_OTA_CONFIRM_TIMEOUT_S);
        esp_ota_mark_app_invalid_rollback_and_reboot();
    }
}

static void idle_timeout_cb(void *arg)
{
    LOCK();
    if (s.state == OTA_STATE_RECEIVING || s.state == OTA_STATE_READY_TO_REBOOT) {
        if (s.state == OTA_STATE_READY_TO_REBOOT) {
            // the new slot is already selected; put the running one back
            restore_running_partition();
        }
        enter_error(ESP_ERR_TIMEOUT, "nRF went quiet");
    }
    UNLOCK();
}

/*******************************
 * COMMANDS (SPI task)
 ******************************/

ota_result_t ota_ctl_begin(uint32_t image_size)
{
    ota_result_t res = OTA_RES_OK;

    LOCK();
    if (s.deferred_action != DEFERRED_NONE) {
        // a restart is on its way; there is no point erasing under it
        ESP_LOGW(TAG, "OTA_BEGIN refused: restart pending");
        res = OTA_RES_BAD_STATE;
        goto out;
    }
    if (s.state != OTA_STATE_IDLE) {
        ESP_LOGW(TAG, "OTA_BEGIN refused in %s", state_name(s.state));
        res = OTA_RES_BAD_STATE;
        goto out;
    }

    s.target = esp_ota_get_next_update_partition(NULL);
    if (s.target == NULL) {
        enter_error(ESP_ERR_NOT_FOUND, "no update slot");
        res = OTA_RES_FLASH_FAILED;
        goto out;
    }
    if (image_size == 0 || image_size > s.target->size) {
        ESP_LOGW(TAG, "OTA_BEGIN refused: %lu bytes does not fit %s (%lu)",
                 (unsigned long)image_size, s.target->label, (unsigned long)s.target->size);
        res = OTA_RES_BAD_LEN;
        goto out;
    }

    s.image_size = image_size;
    s.written = 0;
    s.next_seq = 0;
    s.last_logged_pct = 0;
    s.write_us_max = 0;
    s.abort_requested = false;

    ESP_LOGI(TAG, "OTA_BEGIN: %lu bytes into %s @ 0x%lx, Bluetooth going down",
             (unsigned long)image_size, s.target->label, (unsigned long)s.target->address);
    s.bt_down = true;   // the erase task does the actual shutdown; from here on the way out is a reset
    set_state(OTA_STATE_ERASING, ESP_OK);

    if (xTaskCreate(erase_task, "ota_erase", OTA_WORKER_STACK, NULL, OTA_WORKER_PRIO, NULL) != pdPASS) {
        enter_error(ESP_ERR_NO_MEM, "erase task");
        res = OTA_RES_FLASH_FAILED;
    }
out:
    UNLOCK();
    return res;
}

ota_result_t ota_ctl_write_block(uint16_t seq, const uint8_t *data, uint16_t len, uint32_t crc)
{
    ota_result_t res = OTA_RES_OK;

    LOCK();
    if (s.state != OTA_STATE_RECEIVING) {
        res = OTA_RES_BAD_STATE;
        goto out;
    }
    touch_idle();
    if (seq != s.next_seq) {
        ESP_LOGW(TAG, "block %u: expected %u", seq, s.next_seq);
        res = OTA_RES_BAD_SEQ;
        goto out;
    }
    if (s.written + len > s.image_size) {
        ESP_LOGW(TAG, "block %u: %u bytes overruns the %lu byte image", seq, len, (unsigned long)s.image_size);
        res = OTA_RES_BAD_LEN;
        goto out;
    }
    // same polynomial / init / final-xor as zlib crc32() and Zephyr crc32_ieee()
    if (esp_rom_crc32_le(0, data, len) != crc) {
        ESP_LOGW(TAG, "block %u: crc mismatch", seq);
        res = OTA_RES_BAD_CRC;
        goto out;
    }

    int64_t t0 = esp_timer_get_time();
    esp_err_t err = esp_ota_write(s.handle, data, len);
    uint32_t write_us = (uint32_t)(esp_timer_get_time() - t0);
    if (err != ESP_OK) {
        enter_error(err, "esp_ota_write");
        res = OTA_RES_FLASH_FAILED;
        goto out;
    }
    if (write_us > s.write_us_max) {
        s.write_us_max = write_us;
    }

    s.written += len;
    s.next_seq++;

    uint8_t pct = (uint8_t)((uint64_t)s.written * 100 / s.image_size);
    if (pct >= s.last_logged_pct + 10 || s.written == s.image_size) {
        s.last_logged_pct = pct - (pct % 10);
        ESP_LOGI(TAG, "%3u%%  %lu / %lu bytes, %u blocks, slowest write %lu us", pct,
                 (unsigned long)s.written, (unsigned long)s.image_size, s.next_seq,
                 (unsigned long)s.write_us_max);
    }
    if (s.written == s.image_size) {
        ESP_LOGI(TAG, "image complete, waiting for OTA_END");
    }
out:
    UNLOCK();
    return res;
}

ota_result_t ota_ctl_end(void)
{
    ota_result_t res = OTA_RES_OK;

    LOCK();
    if (s.state != OTA_STATE_RECEIVING) {
        res = OTA_RES_BAD_STATE;
        goto out;
    }
    if (s.written != s.image_size) {
        ESP_LOGW(TAG, "OTA_END with %lu of %lu bytes", (unsigned long)s.written, (unsigned long)s.image_size);
        res = OTA_RES_BAD_LEN;
        goto out;
    }

    ESP_LOGI(TAG, "OTA_END: verifying image");
    esp_timer_stop(s.idle_tmr);
    set_state(OTA_STATE_VERIFYING, ESP_OK);
    if (xTaskCreate(verify_task, "ota_verify", OTA_WORKER_STACK, NULL, OTA_WORKER_PRIO, NULL) != pdPASS) {
        enter_error(ESP_ERR_NO_MEM, "verify task");
        res = OTA_RES_FLASH_FAILED;
    }
out:
    UNLOCK();
    return res;
}

ota_result_t ota_ctl_reboot(void)
{
    ota_result_t res = OTA_RES_OK;

    LOCK();
    if (s.state != OTA_STATE_READY_TO_REBOOT || s.deferred_action != DEFERRED_NONE) {
        res = OTA_RES_BAD_STATE;
        goto out;
    }
    ESP_LOGI(TAG, "OTA_REBOOT: restarting in %d ms", OTA_DEFERRED_ACTION_MS);
    esp_timer_stop(s.idle_tmr);
    s.deferred_action = DEFERRED_RESTART;
    esp_timer_start_once(s.deferred_tmr, OTA_DEFERRED_ACTION_MS * 1000ULL);
out:
    UNLOCK();
    return res;
}

ota_result_t ota_ctl_confirm(void)
{
    ota_result_t res = OTA_RES_OK;

    LOCK();
    if (s.state != OTA_STATE_PENDING_CONFIRM) {
        res = OTA_RES_BAD_STATE;
        goto out;
    }
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        // stay pending: the confirm timer still decides the outcome
        ESP_LOGE(TAG, "OTA_CONFIRM: mark valid failed: %s", esp_err_to_name(err));
        s.err = err;
        res = OTA_RES_FLASH_FAILED;
        goto out;
    }
    esp_timer_stop(s.confirm_tmr);
    ESP_LOGI(TAG, "OTA_CONFIRM: image marked valid, update complete");
    set_state(OTA_STATE_IDLE, ESP_OK);
out:
    UNLOCK();
    return res;
}

ota_result_t ota_ctl_abort(void)
{
    ota_result_t res = OTA_RES_OK;

    LOCK();
    switch (s.state) {
    case OTA_STATE_IDLE:
        break;

    case OTA_STATE_ERASING:
    case OTA_STATE_VERIFYING:
        // the worker owns the handle right now; it finishes what it is doing
        // and then tears down
        ESP_LOGI(TAG, "OTA_ABORT during %s, will drop when the worker returns", state_name(s.state));
        s.abort_requested = true;
        break;

    case OTA_STATE_RECEIVING:
        to_idle("OTA_ABORT");
        break;

    case OTA_STATE_READY_TO_REBOOT:
        if (s.deferred_action != DEFERRED_NONE) {
            res = OTA_RES_BAD_STATE; // restart already in flight
            break;
        }
        restore_running_partition();
        to_idle("OTA_ABORT, boot partition restored");
        break;

    case OTA_STATE_PENDING_CONFIRM:
        if (s.deferred_action != DEFERRED_NONE) {
            res = OTA_RES_BAD_STATE;
            break;
        }
        ESP_LOGW(TAG, "OTA_ABORT while pending confirm: rolling back in %d ms", OTA_DEFERRED_ACTION_MS);
        esp_timer_stop(s.confirm_tmr);
        s.deferred_action = DEFERRED_ROLLBACK;
        esp_timer_start_once(s.deferred_tmr, OTA_DEFERRED_ACTION_MS * 1000ULL);
        break;

    case OTA_STATE_ERROR:
        to_idle("OTA_ABORT cleared error");
        break;
    }
    UNLOCK();
    return res;
}

void ota_ctl_get_state(ota_state_report_t *out)
{
    LOCK();
    out->state = s.state;
    out->err = (uint16_t)s.err;
    out->next_seq = s.next_seq;
    out->percent = s.image_size ? (uint8_t)((uint64_t)s.written * 100 / s.image_size) : 0;
    UNLOCK();
}

void ota_ctl_bt_bringup_done(void)
{
    xEventGroupSetBits(s.bt_ready, BT_READY_BIT);
}

/*******************************
 * BOOT
 ******************************/

void ota_ctl_init(void)
{
    memset(&s, 0, sizeof(s));
    s.lock = xSemaphoreCreateMutex();
    assert(s.lock != NULL);
    s.bt_ready = xEventGroupCreate();
    assert(s.bt_ready != NULL);

    const esp_timer_create_args_t deferred_args = { .callback = deferred_cb, .name = "ota_deferred" };
    const esp_timer_create_args_t confirm_args  = { .callback = confirm_timeout_cb, .name = "ota_confirm" };
    const esp_timer_create_args_t idle_args     = { .callback = idle_timeout_cb, .name = "ota_idle" };
    ESP_ERROR_CHECK(esp_timer_create(&deferred_args, &s.deferred_tmr));
    ESP_ERROR_CHECK(esp_timer_create(&confirm_args, &s.confirm_tmr));
    ESP_ERROR_CHECK(esp_timer_create(&idle_args, &s.idle_tmr));

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t img_state;

    if (esp_ota_get_state_partition(running, &img_state) == ESP_OK && img_state == ESP_OTA_IMG_PENDING_VERIFY) {
        // first boot of a freshly written image. The bootloader will fall back
        // to the other slot on any reset until this one is marked valid.
        ESP_LOGW(TAG, "first boot of this image: waiting up to %d s for OTA_CONFIRM, else rolling back",
                 CONFIG_OMNI_OTA_CONFIRM_TIMEOUT_S);
        set_state(OTA_STATE_PENDING_CONFIRM, ESP_OK);
        esp_timer_start_once(s.confirm_tmr, (uint64_t)CONFIG_OMNI_OTA_CONFIRM_TIMEOUT_S * 1000000ULL);
        return;
    }

    const esp_partition_t *other = esp_ota_get_next_update_partition(NULL);
    if (other != NULL && esp_ota_get_state_partition(other, &img_state) == ESP_OK &&
        (img_state == ESP_OTA_IMG_ABORTED || img_state == ESP_OTA_IMG_INVALID)) {
        // we are here because the image in the other slot booted and was
        // thrown out - by the bootloader (crash before confirm) or by us
        // (confirm timeout / OTA_ABORT). Surface that; the nRF's next update
        // attempt clears it with OTA_ABORT. otadata keeps saying so until that
        // slot is rewritten, so this shows up on every boot until then.
        ESP_LOGW(TAG, "%s holds an image that was rolled back (%s); reporting OTA_ERROR until OTA_ABORT",
                 other->label, img_state == ESP_OTA_IMG_ABORTED ? "bootloader" : "app");
        set_state(OTA_STATE_ERROR, OTA_ERR_ROLLED_BACK);
    }
}
