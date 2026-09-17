#ifndef FLOG_H
#define FLOG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/*
 * Runtime log on flash, readable over the SPI bridge.
 *
 * Every ESP_LOG line (the same text that goes to the UART) is copied into a
 * RAM ring and drained by a low-priority task into the 'log' partition. The
 * partition is a ring of 4K sectors, no filesystem: each sector carries a
 * small header with a monotonic sequence number, the rest is raw log text.
 * The nRF side sees one linear byte stream addressed by a 32-bit offset that
 * only ever grows - byte 0 was the first byte ever logged - and pulls it in
 * chunks with LOG_READ, remembering where it got to. What the ring has
 * overwritten is reported as 'oldest' by LOG_INFO.
 *
 * The two rules that shape this module (both from audio_out.c / ota_ctl.c):
 *
 *   - A sector erase stops both cores for longer than the I2S DMA holds, so
 *     flash is only ever erased while the output is idle. Sectors are erased
 *     ahead of need; when playback outruns them, lines wait in RAM and the
 *     oldest are dropped if that fills too. Page writes are sub-millisecond
 *     and go ahead regardless.
 *   - Flash is not touched at all until Bluetooth bring-up is over
 *     (flog_flash_ready), so the boot log sits in RAM until then.
 *
 * Wire-level command formats live in bridge.h; the nRF mirror is
 * ot_interprocessor.h in the omnitone_audio tree.
 */

// Result byte for every LOG_* command.
typedef enum {
    FLOG_RES_OK = 0,
    FLOG_RES_BAD_OFFSET,     // LOG_READ offset outside [oldest, end]; re-issue LOG_INFO
    FLOG_RES_BAD_LEN,        // LOG_READ len > MAX_LOG_READ_CHUNK
    FLOG_RES_NOT_READY,      // no 'log' partition, or flash not yet writable (LOG_CLEAR)
    FLOG_RES_BUSY,           // LOG_CLEAR needs an erased sector and audio is holding the flash; retry
    FLOG_RES_FLASH_FAILED,   // esp_partition_* returned an error
} flog_result_t;

// Reported in LOG_INFO.
typedef struct {
    uint32_t oldest;    // lowest stream offset still readable
    uint32_t end;       // one past the newest byte in flash; LOG_READ here returns len 0
    uint16_t pending;   // bytes waiting in RAM (audio is holding the flash, or the flush timer has not fired)
} flog_info_t;

// Call first thing in app_main, before the first ESP_LOG: installs the log
// hook and scans the partition to find where the stream left off. Only reads
// flash. Logs a BOOT banner with the reset reason and firmware version.
void flog_init(void);

// Call once Bluetooth bring-up is over (same moment as ota_ctl_bt_bringup_done).
// Nothing is written to the partition before this.
void flog_flash_ready(void);

// Writes whatever is pending to flash first (page writes only), so 'end' is
// current as of the call.
flog_result_t flog_info(flog_info_t *out);

// Copy up to len bytes of the stream starting at offset into buf. *out_len is
// what was actually copied: shorter than len at a sector boundary or at the
// end of the stream, 0 when offset == end. Never crosses a sector, so a
// reader loops until it gets 0.
flog_result_t flog_read(uint32_t offset, uint8_t *buf, uint16_t len, uint16_t *out_len);

// Forget everything logged so far: 'oldest' becomes 'end'. Sectors are
// reclaimed as the ring reuses them, so this is cheap.
flog_result_t flog_clear(void);

#endif
