#include "flog.h"
#include "audio_out.h"
#include "bridge.h"

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "FLOG";

// partitions.csv: user type 0x40, label "log"
#define LOG_PARTITION_TYPE  ((esp_partition_type_t)0x40)
#define LOG_PARTITION_LABEL "log"

/*
 * On-flash layout. The partition is a ring of SECTOR-sized slots; slot i holds
 * whichever sector has seq % nsect == i. Each begins with hdr_t, the rest is
 * log text written front to back. NOR flash only ever clears bits, so an
 * unwritten byte reads 0xFF; the writer replaces any 0xFF in the text with
 * '?' so that "first 0xFF" is unambiguously where the text ends.
 *
 * Stream offset = seq * PAYLOAD + bytes into the payload. It grows forever
 * (4G before it wraps, ~4000 laps of a 1M partition) and is what LOG_READ
 * addresses, so the master never needs to know about sectors.
 */
#define SECTOR         4096
#define HDR_LEN        16
#define PAYLOAD        (SECTOR - HDR_LEN)
#define HDR_MAGIC      0x474F4C46u          // "FLOG" little-endian
#define HDR_FLAG_TRUNC (1u << 0)            // LOG_CLEAR: everything before this sector is gone

typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint32_t seq_inv;   // ~seq: a header that lost power mid-write does not validate
    uint32_t flags;
} hdr_t;
_Static_assert(sizeof(hdr_t) == HDR_LEN, "hdr_t is the on-flash header");

// One ESP_LOG line is one vprintf call; longer lines are cut, not split.
#define LOG_LINE_MAX       256
// Flush as soon as this much is pending, whatever the timer says.
#define FLUSH_MIN      256
// Bytes moved from the RAM ring to flash per write.
#define STAGE_LEN      512
// How often to look again while audio is holding the flash.
#define BLOCKED_POLL_MS 250

#define TASK_STACK     4096
#define TASK_PRIO      3

static struct {
    const esp_partition_t *part;
    uint32_t nsect;
    TaskHandle_t task;
    vprintf_like_t orig_vprintf;

    // RAM side: any task, guarded by s_mux. A byte FIFO; when it is full the
    // oldest whole line goes and 'dropped' remembers how much.
    uint8_t *ram;
    uint32_t ram_len;
    uint32_t ram_head;      // next byte in
    uint32_t ram_tail;      // next byte out
    uint32_t ram_used;
    uint32_t dropped;
    int64_t first_pending_us;   // when the oldest pending byte arrived; 0 when nothing is pending

    // Flash side: guarded by 'lock'. Touched by the flush task and the SPI
    // task (LOG_INFO / LOG_READ / LOG_CLEAR).
    SemaphoreHandle_t lock;
    volatile bool flash_ready;
    bool open;              // a sector is open for writing (header on flash)
    uint32_t head_seq;      // the open sector
    uint32_t intra;         // payload bytes written into it
    uint32_t next_seq;      // what the next sector opened will be numbered
    uint32_t erased;        // consecutive erased slots starting at slot(next_seq)
    bool have_data;
    uint32_t oldest_seq;    // valid when have_data
    bool clear_pending;     // next sector opened carries HDR_FLAG_TRUNC
    bool blocked_logged;    // rate limit: "waiting for audio" warned once per stretch
    bool write_err_logged;  // rate limit: flash errors warned once per streak

    uint8_t stage[STAGE_LEN];
} s;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

#define LOCK()   xSemaphoreTake(s.lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s.lock)

static inline uint32_t slot_addr(uint32_t seq)
{
    return (seq % s.nsect) * SECTOR;
}

static inline uint32_t end_offset(void)
{
    return s.open ? s.head_seq * PAYLOAD + s.intra : s.next_seq * PAYLOAD;
}

static inline uint32_t oldest_offset(void)
{
    return s.have_data ? s.oldest_seq * PAYLOAD : end_offset();
}

/*******************************
 * RAM RING
 ******************************/

/* s_mux held. Throw away the oldest line (through its '\n'). */
static void ram_drop_line(void)
{
    while (s.ram_used) {
        uint8_t c = s.ram[s.ram_tail];
        s.ram_tail = (s.ram_tail + 1) % s.ram_len;
        s.ram_used--;
        s.dropped++;
        if (c == '\n') {
            break;
        }
    }
}

static void ram_push(const uint8_t *p, uint32_t n)
{
    if (n > s.ram_len) {
        return;
    }
    portENTER_CRITICAL(&s_mux);
    while (s.ram_len - s.ram_used < n) {
        ram_drop_line();
    }
    uint32_t first = s.ram_len - s.ram_head;
    if (first > n) {
        first = n;
    }
    memcpy(&s.ram[s.ram_head], p, first);
    memcpy(&s.ram[0], p + first, n - first);
    s.ram_head = (s.ram_head + n) % s.ram_len;
    s.ram_used += n;
    if (s.first_pending_us == 0) {
        s.first_pending_us = esp_timer_get_time();
    }
    portEXIT_CRITICAL(&s_mux);

    if (s.task) {
        if (xPortInIsrContext()) {
            BaseType_t woken = pdFALSE;
            vTaskNotifyGiveFromISR(s.task, &woken);
            portYIELD_FROM_ISR(woken);
        } else {
            xTaskNotifyGive(s.task);
        }
    }
}

/* Copy up to n pending bytes into dst without consuming them. */
static uint32_t ram_peek(uint8_t *dst, uint32_t n)
{
    portENTER_CRITICAL(&s_mux);
    if (n > s.ram_used) {
        n = s.ram_used;
    }
    uint32_t first = s.ram_len - s.ram_tail;
    if (first > n) {
        first = n;
    }
    memcpy(dst, &s.ram[s.ram_tail], first);
    memcpy(dst + first, &s.ram[0], n - first);
    portEXIT_CRITICAL(&s_mux);
    return n;
}

static void ram_consume(uint32_t n)
{
    portENTER_CRITICAL(&s_mux);
    s.ram_tail = (s.ram_tail + n) % s.ram_len;
    s.ram_used -= n;
    if (s.ram_used == 0) {
        s.first_pending_us = 0;
    }
    portEXIT_CRITICAL(&s_mux);
}

static uint32_t ram_pending(void)
{
    portENTER_CRITICAL(&s_mux);
    uint32_t n = s.ram_used;
    portEXIT_CRITICAL(&s_mux);
    return n;
}

/* The esp_log hook. Runs on whichever task called ESP_LOGx, so it does the
 * minimum: hand the line to the UART as before, format it once more into a
 * stack buffer, and queue it. No flash, no blocking. */
static int flog_vprintf(const char *fmt, va_list ap)
{
    va_list ap2;
    va_copy(ap2, ap);
    int n = s.orig_vprintf ? s.orig_vprintf(fmt, ap) : vprintf(fmt, ap);

    char line[LOG_LINE_MAX];
    int len = vsnprintf(line, sizeof(line), fmt, ap2);
    va_end(ap2);

    if (len > 0) {
        if (len >= (int)sizeof(line)) {
            len = sizeof(line) - 1;
            line[len - 1] = '\n';   // cut, but still a whole line
        }
        for (int i = 0; i < len; i++) {
            if ((uint8_t)line[i] == 0xFF) {
                line[i] = '?';      // 0xFF means "unwritten" on flash
            }
        }
        ram_push((const uint8_t *)line, (uint32_t)len);
    }
    return n;
}

/*******************************
 * FLASH SIDE (lock held throughout)
 ******************************/

static bool hdr_valid(const hdr_t *h, uint32_t slot)
{
    return h->magic == HDR_MAGIC && h->seq_inv == ~h->seq && (h->seq % s.nsect) == slot;
}

static bool read_hdr(uint32_t seq, hdr_t *h)
{
    if (esp_partition_read(s.part, slot_addr(seq), h, sizeof(*h)) != ESP_OK) {
        return false;
    }
    return hdr_valid(h, seq % s.nsect);
}

/* Is the slot for seq blank end to end? */
static bool slot_erased(uint32_t seq)
{
    uint32_t base = slot_addr(seq);
    for (uint32_t off = 0; off < SECTOR; off += STAGE_LEN) {
        if (esp_partition_read(s.part, base + off, s.stage, STAGE_LEN) != ESP_OK) {
            return false;
        }
        for (uint32_t i = 0; i < STAGE_LEN; i++) {
            if (s.stage[i] != 0xFF) {
                return false;
            }
        }
    }
    return true;
}

/* Where the text in the open sector ends: one past the last non-0xFF byte. */
static uint32_t find_intra(uint32_t seq)
{
    uint32_t base = slot_addr(seq) + HDR_LEN;
    uint32_t end = 0;
    for (uint32_t off = 0; off < PAYLOAD; off += STAGE_LEN) {
        uint32_t n = (PAYLOAD - off < STAGE_LEN) ? PAYLOAD - off : STAGE_LEN;
        if (esp_partition_read(s.part, base + off, s.stage, n) != ESP_OK) {
            break;
        }
        for (uint32_t i = 0; i < n; i++) {
            if (s.stage[i] != 0xFF) {
                end = off + i + 1;
            }
        }
    }
    return end;
}

/* Erase ahead while the output is idle. Reclaiming the oldest live sector
 * is what moves 'oldest' forward. */
static void maintain_erased(void)
{
    while (s.erased < CONFIG_OMNI_FLOG_ERASE_AHEAD && s.erased + 2 < s.nsect) {
        if (audio_out_is_active()) {
            return;
        }
        uint32_t seq = s.next_seq + s.erased;
        esp_err_t err = esp_partition_erase_range(s.part, slot_addr(seq), SECTOR);
        if (err != ESP_OK) {
            if (!s.write_err_logged) {
                ESP_LOGE(TAG, "erase failed: %s", esp_err_to_name(err));
                s.write_err_logged = true;
            }
            return;
        }
        if (s.have_data && seq >= s.nsect && seq - s.nsect == s.oldest_seq) {
            s.oldest_seq++;
        }
        s.erased++;
    }
}

/* Start writing into the next sector. False if nothing is erased and either
 * audio is holding the flash or the caller may not erase; the bytes stay in
 * RAM and the flush task tries again later. may_erase is false on the SPI
 * task: a sector erase there outlasts the master's reply retries, so only
 * page writes happen on that side (same rule as OTA). */
static bool open_next_sector(bool may_erase)
{
    if (s.erased == 0) {
        if (may_erase) {
            maintain_erased();
        }
        if (s.erased == 0) {
            if (!s.blocked_logged) {
                ESP_LOGW(TAG, "no erased sector and the output is busy; lines wait in RAM");
                s.blocked_logged = true;
            }
            return false;
        }
    }

    hdr_t h = {
        .magic = HDR_MAGIC,
        .seq = s.next_seq,
        .seq_inv = ~s.next_seq,
        .flags = s.clear_pending ? HDR_FLAG_TRUNC : 0,
    };
    esp_err_t err = esp_partition_write(s.part, slot_addr(s.next_seq), &h, sizeof(h));
    if (err != ESP_OK) {
        if (!s.write_err_logged) {
            ESP_LOGE(TAG, "header write failed: %s", esp_err_to_name(err));
            s.write_err_logged = true;
        }
        return false;
    }

    s.head_seq = s.next_seq;
    s.next_seq++;
    s.intra = 0;
    s.open = true;
    s.erased--;
    if (!s.have_data || s.clear_pending) {
        s.have_data = true;
        s.oldest_seq = s.head_seq;
    }
    s.clear_pending = false;
    if (s.blocked_logged) {
        ESP_LOGI(TAG, "flash free again, draining RAM");
        s.blocked_logged = false;
    }
    return true;
}

/* Append n bytes to the stream, spanning sectors as needed. Returns how many
 * landed: short when the next sector could not be opened. */
static uint32_t write_bytes(const uint8_t *p, uint32_t n, bool may_erase)
{
    uint32_t done = 0;
    while (done < n) {
        if (!s.open || s.intra >= PAYLOAD) {
            if (!open_next_sector(may_erase)) {
                break;
            }
        }
        uint32_t room = PAYLOAD - s.intra;
        uint32_t m = (n - done < room) ? n - done : room;
        esp_err_t err = esp_partition_write(s.part, slot_addr(s.head_seq) + HDR_LEN + s.intra, p + done, m);
        if (err != ESP_OK) {
            if (!s.write_err_logged) {
                ESP_LOGE(TAG, "write failed: %s", esp_err_to_name(err));
                s.write_err_logged = true;
            }
            break;
        }
        s.write_err_logged = false;
        s.intra += m;
        done += m;
    }
    return done;
}

typedef enum { DRAIN_DONE, DRAIN_BLOCKED } drain_result_t;

/* Move everything pending from RAM to flash. Page writes only - if the open
 * sector fills and no erased one is waiting, the rest stays in RAM. */
static drain_result_t drain(bool may_erase)
{
    if (s.part == NULL || !s.flash_ready) {
        return DRAIN_BLOCKED;
    }

    for (;;) {
        portENTER_CRITICAL(&s_mux);
        uint32_t dropped = s.dropped;
        s.dropped = 0;
        portEXIT_CRITICAL(&s_mux);
        if (dropped) {
            char marker[48];
            int len = snprintf(marker, sizeof(marker), "[flog: dropped %lu bytes]\n", (unsigned long)dropped);
            if (write_bytes((const uint8_t *)marker, (uint32_t)len, may_erase) < (uint32_t)len) {
                portENTER_CRITICAL(&s_mux);
                s.dropped += dropped;
                portEXIT_CRITICAL(&s_mux);
                return DRAIN_BLOCKED;
            }
        }

        uint32_t n = ram_peek(s.stage, STAGE_LEN);
        if (n == 0) {
            return DRAIN_DONE;
        }
        uint32_t w = write_bytes(s.stage, n, may_erase);
        ram_consume(w);
        if (w < n) {
            return DRAIN_BLOCKED;
        }
    }
}

/* Find where the stream left off. Reads only. */
static void scan(void)
{
    hdr_t h;
    bool found = false;
    uint32_t head = 0;

    for (uint32_t slot = 0; slot < s.nsect; slot++) {
        if (read_hdr(slot, &h) && (!found || h.seq > head)) {
            head = h.seq;
            found = true;
        }
    }

    if (!found) {
        // fresh partition (or junk from before it was allocated)
        s.open = false;
        s.next_seq = 0;
        s.have_data = false;
    } else {
        s.open = true;
        s.head_seq = head;
        s.next_seq = head + 1;
        s.intra = find_intra(head);
        s.have_data = true;

        // walk back over contiguous predecessors until a gap or a TRUNC mark
        uint32_t oldest = head;
        for (;;) {
            if (!read_hdr(oldest, &h) || (h.flags & HDR_FLAG_TRUNC) || oldest == 0) {
                break;
            }
            if (!read_hdr(oldest - 1, &h) || h.seq != oldest - 1) {
                break;
            }
            oldest--;
            if (head - oldest >= s.nsect - 1) {
                break;
            }
        }
        s.oldest_seq = oldest;
    }

    s.erased = 0;
    while (s.erased < CONFIG_OMNI_FLOG_ERASE_AHEAD && slot_erased(s.next_seq + s.erased)) {
        s.erased++;
    }
}

/*******************************
 * FLUSH TASK
 ******************************/

static void flog_task(void *arg)
{
    (void)arg;
    TickType_t wait = portMAX_DELAY;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, wait);

        if (!s.flash_ready) {
            wait = portMAX_DELAY;   // flog_flash_ready() wakes us
            continue;
        }

        uint32_t pending = ram_pending();
        if (pending == 0) {
            LOCK();
            maintain_erased();
            UNLOCK();
            wait = portMAX_DELAY;
            continue;
        }

        // Coalesce: write when a page is pending or the oldest byte is old enough.
        portENTER_CRITICAL(&s_mux);
        int64_t first = s.first_pending_us;
        portEXIT_CRITICAL(&s_mux);
        int64_t age_ms = (esp_timer_get_time() - first) / 1000;
        if (pending < FLUSH_MIN && age_ms < CONFIG_OMNI_FLOG_FLUSH_MS) {
            wait = pdMS_TO_TICKS(CONFIG_OMNI_FLOG_FLUSH_MS - age_ms) + 1;
            continue;
        }

        LOCK();
        drain_result_t r = drain(true);
        maintain_erased();
        UNLOCK();

        wait = (r == DRAIN_BLOCKED) ? pdMS_TO_TICKS(BLOCKED_POLL_MS) : portMAX_DELAY;
    }
}

/*******************************
 * PUBLIC
 ******************************/

static const char *reset_name(esp_reset_reason_t r, bool *abnormal)
{
    *abnormal = false;
    switch (r) {
    case ESP_RST_POWERON:   return "poweron";
    case ESP_RST_SW:        return "software";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_EXT:       return "external";
    case ESP_RST_PANIC:     *abnormal = true; return "PANIC";
    case ESP_RST_INT_WDT:   *abnormal = true; return "INT_WDT";
    case ESP_RST_TASK_WDT:  *abnormal = true; return "TASK_WDT";
    case ESP_RST_WDT:       *abnormal = true; return "WDT";
    case ESP_RST_BROWNOUT:  *abnormal = true; return "BROWNOUT";
    case ESP_RST_SDIO:      return "sdio";
    default:                *abnormal = true; return "unknown";
    }
}

void flog_init(void)
{
    memset(&s, 0, sizeof(s));
    s.lock = xSemaphoreCreateMutex();
    assert(s.lock != NULL);

    s.ram_len = CONFIG_OMNI_FLOG_RAM_KB * 1024;
    s.ram = heap_caps_malloc(s.ram_len, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(s.ram != NULL);

    // From here on every ESP_LOG line is captured
    s.orig_vprintf = esp_log_set_vprintf(flog_vprintf);

    s.part = esp_partition_find_first(LOG_PARTITION_TYPE, ESP_PARTITION_SUBTYPE_ANY, LOG_PARTITION_LABEL);
    if (s.part == NULL) {
        ESP_LOGE(TAG, "no 'log' partition - old partition table flashed? log stays in RAM");
    } else {
        s.nsect = s.part->size / SECTOR;
        scan();
    }

    BaseType_t ok = xTaskCreate(flog_task, "flog", TASK_STACK, NULL, TASK_PRIO, &s.task);
    assert(ok == pdPASS);

    bool abnormal;
    const char *why = reset_name(esp_reset_reason(), &abnormal);
    if (abnormal) {
        status_flags |= EXT_MCU_LOG_CRASH_FLAG;
    }
    ESP_LOGI(TAG, "=== BOOT reset=%s fw=%s stream %lu..%lu %s%lu erased ahead ===",
             why, esp_app_get_description()->version,
             (unsigned long)oldest_offset(), (unsigned long)end_offset(),
             s.open ? "" : "(no sector open) ", (unsigned long)s.erased);
}

void flog_flash_ready(void)
{
    s.flash_ready = true;
    if (s.task) {
        xTaskNotifyGive(s.task);
    }
}

flog_result_t flog_info(flog_info_t *out)
{
    if (s.part == NULL) {
        return FLOG_RES_NOT_READY;
    }
    LOCK();
    drain(false);
    out->oldest = oldest_offset();
    out->end = end_offset();
    uint32_t pending = ram_pending();
    out->pending = pending > UINT16_MAX ? UINT16_MAX : (uint16_t)pending;
    UNLOCK();
    return FLOG_RES_OK;
}

flog_result_t flog_read(uint32_t offset, uint8_t *buf, uint16_t len, uint16_t *out_len)
{
    *out_len = 0;
    if (s.part == NULL) {
        return FLOG_RES_NOT_READY;
    }
    if (len > MAX_LOG_READ_CHUNK) {
        return FLOG_RES_BAD_LEN;
    }

    flog_result_t res = FLOG_RES_OK;
    LOCK();
    uint32_t end = end_offset();
    if (offset < oldest_offset() || offset > end) {
        res = FLOG_RES_BAD_OFFSET;
    } else if (offset < end) {
        uint32_t seq = offset / PAYLOAD;
        uint32_t intra = offset % PAYLOAD;
        uint32_t avail = (seq == s.head_seq) ? s.intra - intra : PAYLOAD - intra;
        uint32_t n = (len < avail) ? len : avail;
        esp_err_t err = esp_partition_read(s.part, slot_addr(seq) + HDR_LEN + intra, buf, n);
        if (err != ESP_OK) {
            res = FLOG_RES_FLASH_FAILED;
        } else {
            *out_len = (uint16_t)n;
        }
    }
    if (res == FLOG_RES_OK && offset + *out_len == end) {
        status_flags &= ~EXT_MCU_LOG_CRASH_FLAG;   // the master has now seen everything on flash
    }
    UNLOCK();
    return res;
}

flog_result_t flog_clear(void)
{
    if (s.part == NULL || !s.flash_ready) {
        return FLOG_RES_NOT_READY;
    }

    // On the SPI task, so no erasing here: if playback has used up the
    // erased-ahead sectors the master gets BUSY and asks again once the flush
    // task has had an idle moment to erase.
    flog_result_t res = FLOG_RES_OK;
    LOCK();
    drain(false);
    if (s.erased == 0) {
        res = FLOG_RES_BUSY;
    } else {
        s.clear_pending = true;
        if (!open_next_sector(false)) {
            s.clear_pending = false;
            res = FLOG_RES_FLASH_FAILED;
        } else {
            status_flags &= ~EXT_MCU_LOG_CRASH_FLAG;
            ESP_LOGI(TAG, "log cleared at offset %lu", (unsigned long)oldest_offset());
        }
    }
    UNLOCK();
    return res;
}
