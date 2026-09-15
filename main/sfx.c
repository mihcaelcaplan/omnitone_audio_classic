/*
 * Sound effect bank, read out of the 'audio' flash partition.
 *
 * Clips are 16-bit mono PCM at SFX_BANK_SAMPLE_RATE. This file owns the bank
 * and hands audio_out.c pointers into it; it does no playback and touches no
 * peripheral.
 */

#include <inttypes.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"

#include "audio_out.h"
#include "sfx.h"

static const char *SFX_TAG = "SFX";

/* Mirrors the on-flash layout documented in sounds/README.md. ESP32 is
 * little-endian and these are packed, so the struct maps straight onto what the
 * packer wrote - the static asserts below tie the two together via the
 * constants in the generated sfx_ids.h, so a format change that misses one side
 * fails the build instead of producing noise. */
typedef struct __attribute__((packed)) {
    char     magic[4];
    uint16_t version;
    uint16_t count;
    uint32_t sample_rate;
    uint32_t reserved;
} sfx_bank_header_t;

typedef struct __attribute__((packed)) {
    uint32_t offset;   /* from the start of the partition */
    uint32_t frames;   /* mono frames, so byte length is 2x this */
} sfx_entry_t;

_Static_assert(sizeof(sfx_bank_header_t) == SFX_BANK_HEADER_SIZE,
               "bank header size disagrees with pack_sounds.py");
_Static_assert(sizeof(sfx_entry_t) == SFX_BANK_ENTRY_SIZE,
               "bank entry size disagrees with pack_sounds.py");

static const esp_partition_t   *s_part;
static sfx_entry_t              s_entries[SFX_COUNT];
static const uint8_t           *s_bank;      /* mmap'd base of the partition */
static esp_partition_mmap_handle_t s_map;
static bool                     s_ready;

esp_err_t sfx_init(void)
{
    s_ready = false;

    /* 'spiffs' is just a subtype the partition CSV parser accepts for a data
     * partition - nothing is mounted, this is raw flash. See partitions.csv. */
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "audio");
    if (s_part == NULL) {
        ESP_LOGE(SFX_TAG, "no 'audio' partition - old partition table flashed?");
        return ESP_ERR_NOT_FOUND;
    }

    sfx_bank_header_t hdr;
    esp_err_t err = esp_partition_read(s_part, 0, &hdr, sizeof(hdr));
    if (err != ESP_OK) {
        ESP_LOGE(SFX_TAG, "reading bank header: %s", esp_err_to_name(err));
        return err;
    }

    /* Erased flash reads as 0xFF, so this is also the "no bank was ever
     * flashed" case, which is worth distinguishing from a corrupt one. */
    if (memcmp(hdr.magic, SFX_BANK_MAGIC, sizeof(hdr.magic)) != 0) {
        ESP_LOGE(SFX_TAG, "no sound bank in the audio partition (magic %02x%02x%02x%02x)",
                 hdr.magic[0], hdr.magic[1], hdr.magic[2], hdr.magic[3]);
        return ESP_ERR_INVALID_STATE;
    }
    if (hdr.version != SFX_BANK_VERSION) {
        ESP_LOGE(SFX_TAG, "bank is version %u, firmware speaks %u",
                 hdr.version, SFX_BANK_VERSION);
        return ESP_ERR_INVALID_STATE;
    }
    /* The packer runs by hand, so a bank can lag the firmware that indexes it.
     * A count mismatch is the cheap, reliable half of catching that - it means
     * sounds were added or removed without re-running pack_sounds.py, and every
     * sfx_id_t past this point would index into the wrong clip. */
    if (hdr.count != SFX_COUNT) {
        ESP_LOGE(SFX_TAG, "bank holds %u clips, firmware expects %d - "
                 "re-run sounds/pack_sounds.py and reflash", hdr.count, SFX_COUNT);
        return ESP_ERR_INVALID_STATE;
    }
    if (hdr.sample_rate != SFX_BANK_SAMPLE_RATE) {
        ESP_LOGE(SFX_TAG, "bank is %"PRIu32"Hz, firmware expects %dHz",
                 hdr.sample_rate, SFX_BANK_SAMPLE_RATE);
        return ESP_ERR_INVALID_STATE;
    }

    err = esp_partition_read(s_part, SFX_BANK_HEADER_SIZE, s_entries, sizeof(s_entries));
    if (err != ESP_OK) {
        ESP_LOGE(SFX_TAG, "reading bank index: %s", esp_err_to_name(err));
        return err;
    }

    /* Bounds-check the index once here rather than trusting it on every play:
     * these offsets become raw pointers into mapped flash, and a truncated or
     * half-written bank would otherwise walk off the end of it. */
    const uint32_t pcm_start = SFX_BANK_HEADER_SIZE + SFX_BANK_ENTRY_SIZE * SFX_COUNT;
    uint32_t used = pcm_start;
    for (int i = 0; i < SFX_COUNT; i++) {
        uint32_t off = s_entries[i].offset;
        uint32_t bytes = s_entries[i].frames * sizeof(int16_t);
        if ((off & 1) || off < pcm_start || bytes > s_part->size || off > s_part->size - bytes) {
            ESP_LOGE(SFX_TAG, "clip %d has a bad extent (offset %"PRIu32", %"PRIu32" bytes)",
                     i, off, bytes);
            return ESP_ERR_INVALID_STATE;
        }
        if (off + bytes > used) {
            used = off + bytes;
        }
    }

    /* Map only the part of the partition the bank actually occupies.
     *
     * This is what lets the mixer read clip samples straight out of flash from
     * inside the audio loop. The alternative, esp_partition_read(), goes through
     * spi_flash_disable_interrupts_caches_and_other_cpu() on the ESP32 - it has
     * no flash suspend - which stalls *both* cores for the length of the read.
     * Fine here at init; not something to do every few milliseconds with the
     * Bluetooth stack live on the other core. Mapped pages fault through the
     * cache instead: no lock, no stall. */
    err = esp_partition_mmap(s_part, 0, used, ESP_PARTITION_MMAP_DATA,
                             (const void **)&s_bank, &s_map);
    if (err != ESP_OK) {
        ESP_LOGE(SFX_TAG, "mapping the bank: %s", esp_err_to_name(err));
        return err;
    }

    s_ready = true;
    ESP_LOGI(SFX_TAG, "bank ready: %d clip(s), %"PRIu32" bytes mapped at %"PRIu32"Hz",
             SFX_COUNT, used, hdr.sample_rate);
    return ESP_OK;
}

bool sfx_lookup(int clip_id, const int16_t **pcm, uint32_t *frames)
{
    if (!s_ready || (unsigned)clip_id >= SFX_COUNT) {
        return false;
    }
    *pcm = (const int16_t *)(s_bank + s_entries[clip_id].offset);
    *frames = s_entries[clip_id].frames;
    return true;
}

esp_err_t sfx_play(sfx_id_t id)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    return audio_out_play_sfx((int)id);
}
