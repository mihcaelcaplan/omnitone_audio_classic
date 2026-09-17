#ifndef OTA_CTL_H
#define OTA_CTL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/*
 * Firmware update over the SPI bridge.
 *
 * The nRF5340 holds the image (it got it over BLE; we never see that side) and
 * pushes it to us block by block. We write it into the inactive app slot with
 * esp_ota_*, the nRF tells us when to reboot, and after the reboot the new
 * image runs as PENDING_VERIFY until the nRF sends OTA_CONFIRM - or it gets
 * rolled back by the bootloader on the next reset.
 *
 * Wire-level command formats live in bridge.h; this is the state machine those
 * commands drive. The nRF side is ot_esp_ota.c in the omnitone_audio tree.
 */

// Reported in OTA_STATE.
typedef enum {
    OTA_STATE_IDLE = 0,
    OTA_STATE_ERASING,          // OTA_BEGIN accepted, slot erase in progress - not accepting blocks yet
    OTA_STATE_RECEIVING,        // accepting OTA_DATA
    OTA_STATE_VERIFYING,        // OTA_END accepted, image check in progress
    OTA_STATE_READY_TO_REBOOT,  // image verified and set as boot partition, waiting for OTA_REBOOT
    OTA_STATE_PENDING_CONFIRM,  // running a freshly flashed image, waiting for OTA_CONFIRM
    OTA_STATE_ERROR,            // see err in the state report; OTA_ABORT returns to IDLE. Raised mid-update
                                // (Bluetooth down), the ESP resets itself ~2 s later regardless
} ota_state_t;

// Result byte for every OTA_* command.
typedef enum {
    OTA_RES_OK = 0,
    OTA_RES_BAD_STATE,       // command not valid in the current state
    OTA_RES_BAD_LEN,         // frame shorter than its header claims, payload > MAX_SPI_TRANSFER_CHUNK, or image size wrong
    OTA_RES_BAD_SEQ,         // seq != next_seq; block not written, resend the one we asked for
    OTA_RES_BAD_CRC,         // crc32 mismatch; block not written, resend
    OTA_RES_FLASH_FAILED,    // esp_ota_* returned an error; see err in OTA_STATE
    OTA_RES_NOT_IMPLEMENTED,
} ota_result_t;

typedef struct {
    ota_state_t state;
    uint16_t err;       // low 16 bits of the esp_err_t behind OTA_STATE_ERROR, else 0
    uint16_t next_seq;  // the OTA_DATA seq we will accept next
    uint8_t percent;    // bytes written / image size, 0..100
} ota_state_report_t;

// err values of our own that can show up in OTA_STATE alongside the esp_err_t
// codes (ESP_ERR_OTA_* are 0x1501..0x1507, ESP_ERR_TIMEOUT is 0x0107)
#define OTA_ERR_ROLLED_BACK (ESP_ERR_OTA_BASE + 0x80) // 0x1580: the other slot holds an image that booted and was rolled back

ota_result_t ota_ctl_begin(uint32_t image_size);
ota_result_t ota_ctl_write_block(uint16_t seq, const uint8_t *data, uint16_t len, uint32_t crc);
ota_result_t ota_ctl_end(void);
ota_result_t ota_ctl_reboot(void);
ota_result_t ota_ctl_confirm(void);
ota_result_t ota_ctl_abort(void);
void ota_ctl_get_state(ota_state_report_t *out);

// Call once from app_main before the SPI task starts. Creates the lock and
// timers, and looks at otadata: if this boot is the first run of a new image it
// raises EXT_MCU_OTA_PENDING_FLAG and starts the confirm timeout; if the other
// slot holds an image that was rolled back it raises EXT_MCU_OTA_ERROR_FLAG.
void ota_ctl_init(void);

#endif
