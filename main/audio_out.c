/*
 * Audio output task - the single I2S writer. See audio_out.h for the shape.
 */

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"

#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "audio_out.h"
#include "sfx.h"

#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
#error "audio_out drives external I2S only; the internal-DAC path was removed"
#endif

static const char *TAG = "AUDIO";

/* Stream buffering, unchanged from what the A2DP path used before. At 44.1kHz
 * stereo the prefetch level is ~116ms of jitter absorption. */
#define RINGBUF_SIZE     (32 * 1024)
#define PREFETCH_LEVEL   (20 * 1024)

/* 360 stereo frames = 1440 bytes per pass through the mixer. */
#define CHUNK_FRAMES     360
#define CHUNK_BYTES      (CHUNK_FRAMES * 2 * (int)sizeof(int16_t))

/* DMA sizing. The depth (DESC_NUM * FRAME_NUM frames, 87ms at 44.1kHz) is the
 * only thing holding audio up when this task cannot run at all - and on the
 * ESP32 that happens for reasons no amount of task priority fixes. Any flash
 * erase or write in the system takes spi_flash_disable_interrupts_caches_and_
 * other_cpu(), which stops BOTH cores: with the cache off this task can't read
 * the mmap'd bank or even fetch its own code from flash. NVS writes during the
 * Bluetooth bringup are the usual offender, and a sector erase outlasts the
 * driver's default 33ms of DMA easily.
 *
 * The cost is latency - a full pipeline holds this much audio - so it buys
 * roughly one erase of headroom and stops there. The driver caps a descriptor
 * at 4092 bytes, so FRAME_NUM must stay under 1023 for 16-bit stereo. */
#define DMA_DESC_NUM     8
#define DMA_FRAME_NUM    480

/* How long the channel stays up after everything goes quiet, so chimes fired
 * back to back don't each cost an open/close (and a click). */
#define LINGER_MS        500

/* Levels, Q12 (4096 = unity). Both are tuning knobs - change them here.
 *
 * Chimes are mastered hot relative to streamed music, so they play at half
 * amplitude. The duck is deliberately gentle: it exists to keep the chime
 * intelligible over a loud track, not to punch a hole in the music.
 *
 * The duck is ramped rather than stepped - a gain step is a discontinuity, and
 * a discontinuity is a click. */
#define GAIN_UNITY_Q12   4096
#define SFX_GAIN_Q12     2048   /* -6dB: chime level */
#define DUCK_TARGET_Q12  2048   /* -6dB: music level while a chime plays */
#define DUCK_RAMP_MS     10

#define CTL_QUEUE_DEPTH  6

enum { RING_PROCESSING, RING_PREFETCHING, RING_DROPPING };

typedef enum {
    CTL_STREAM_START,
    CTL_STREAM_STOP,
    CTL_STREAM_READY,   /* producer crossed the prefetch level; a wakeup */
    CTL_SET_FORMAT,
    CTL_PLAY_SFX,
} ctl_op_t;

typedef struct {
    ctl_op_t op;
    uint32_t rate;
    uint8_t  ch;
    int      clip;
} ctl_msg_t;

/* Playback position, kept as a whole frame index plus a Q16 fraction so a bank
 * recorded at one rate plays correctly at whatever rate the codec negotiated.
 * At 44.1kHz out the step is exactly 65536 and the fraction stays zero.
 *
 * The index is deliberately separate from the fraction rather than one 16.16
 * value: packing both into a uint32_t leaves only 16 bits of frame index, which
 * wraps at 65536 frames - 1.5 seconds at 44.1kHz - and silently loops the head
 * of any clip longer than that instead of ever ending. */
typedef struct {
    bool           active;
    int            clip;
    const int16_t *pcm;
    uint32_t       frames;
    uint32_t       pos;        /* whole frames into the clip */
    uint32_t       frac_q16;   /* fraction within frame `pos` */
    uint32_t       step_q16;
} cursor_t;

static QueueHandle_t     s_ctl_q;
static RingbufHandle_t   s_ring;
static i2s_chan_handle_t s_chan;

/* Audio task only, except s_ring_mode. */
static bool       s_stream_on;
static uint32_t   s_out_rate = SFX_BANK_SAMPLE_RATE;
static uint8_t    s_out_ch = 2;
static TickType_t s_last_active;
static int32_t    s_gain_q12 = GAIN_UNITY_Q12;
static int32_t    s_duck_step = 1;   /* per-frame ramp, depends on s_out_rate */
static cursor_t   s_cur;
static int        s_pending = -1;
static int64_t    s_dma_depth_us;
static int64_t    s_last_write_us;

/* Advisory, written by the producer too, as the original ringbuffer code did.
 * A race here costs one chunk of latency, never correctness. */
static volatile uint16_t s_ring_mode = RING_PREFETCHING;

/* File scope, not stack: one task touches these and 2.9KB would otherwise come
 * out of its stack. s_stage is aligned because it is filled as bytes and read
 * as int16_t. */
static __attribute__((aligned(4))) uint8_t s_stage[CHUNK_BYTES + 8];
static int16_t s_mix[CHUNK_FRAMES * 2];
static uint8_t s_carry[8];      /* partial frame held over between chunks */
static size_t  s_carry_len;

/*******************************
 * CHANNEL
 ******************************/

static esp_err_t channel_open(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    chan_cfg.dma_desc_num = DMA_DESC_NUM;
    chan_cfg.dma_frame_num = DMA_FRAME_NUM;
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(s_out_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            (s_out_ch == 1) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = CONFIG_EXAMPLE_I2S_BCK_PIN,
            .ws   = CONFIG_EXAMPLE_I2S_LRCK_PIN,
            .dout = CONFIG_EXAMPLE_I2S_DATA_PIN,
            .din  = I2S_GPIO_UNUSED,
        },
    };

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_chan, NULL);
    if (err != ESP_OK) {
        return err;
    }
    if ((err = i2s_channel_init_std_mode(s_chan, &std_cfg)) != ESP_OK ||
        (err = i2s_channel_enable(s_chan)) != ESP_OK) {
        i2s_del_channel(s_chan);
        s_chan = NULL;
        return err;
    }
    /* Nothing has been written yet, so there is no previous write to measure a
     * gap against. */
    s_dma_depth_us = ((uint64_t)DMA_DESC_NUM * DMA_FRAME_NUM * 1000000ULL) / s_out_rate;
    s_last_write_us = 0;

    ESP_LOGI(TAG, "I2S up: %"PRIu32" Hz, %u ch, %"PRId64" ms of DMA",
             s_out_rate, s_out_ch, s_dma_depth_us / 1000);
    return ESP_OK;
}

static void channel_close(void)
{
    if (s_chan == NULL) {
        return;
    }
    /* No writer to race: this runs on the only task that touches the channel. */
    i2s_channel_disable(s_chan);
    i2s_del_channel(s_chan);
    s_chan = NULL;
}

/*******************************
 * MIXING
 ******************************/

static inline int16_t sat16(int32_t v)
{
    return (v > 32767) ? 32767 : (v < -32768) ? -32768 : (int16_t)v;
}

/* Everything that depends on the output rate. */
static void retune(void)
{
    s_cur.step_q16 = (uint32_t)(((uint64_t)SFX_BANK_SAMPLE_RATE << 16) / s_out_rate);
    uint32_t ramp = (s_out_rate * DUCK_RAMP_MS) / 1000;
    s_duck_step = ramp ? (GAIN_UNITY_Q12 - DUCK_TARGET_Q12) / (int32_t)ramp : GAIN_UNITY_Q12;
    if (s_duck_step < 1) {
        s_duck_step = 1;
    }
}

static void cursor_start(int clip)
{
    if (!sfx_lookup(clip, &s_cur.pcm, &s_cur.frames)) {
        ESP_LOGW(TAG, "no clip %d in the bank", clip);
        return;
    }
    s_cur.clip = clip;
    s_cur.pos = 0;
    s_cur.frac_q16 = 0;
    s_cur.active = true;
    retune();
}

/* One mono sample out of the mmap'd bank. Reading through the cache costs the
 * odd miss; it never takes the flash lock, which is why the bank is mapped
 * rather than read - esp_partition_read() stalls both cores on the ESP32. */
static inline int32_t cursor_next(void)
{
    if (s_cur.pos >= s_cur.frames) {
        s_cur.active = false;
        return 0;
    }
    int32_t a = s_cur.pcm[s_cur.pos];
    int32_t b = (s_cur.pos + 1 < s_cur.frames) ? s_cur.pcm[s_cur.pos + 1] : a;
    int32_t out = a + (((b - a) * (int32_t)s_cur.frac_q16) >> 16);

    /* Carry the fraction into the frame index. step_q16 exceeds 1.0 whenever we
     * are downsampling, so this can advance by more than one frame. */
    s_cur.frac_q16 += s_cur.step_q16;
    s_cur.pos += s_cur.frac_q16 >> 16;
    s_cur.frac_q16 &= 0xffff;
    return (out * SFX_GAIN_Q12) >> 12;
}

/* Pull one chunk of stream PCM into s_stage, spliced onto whatever partial
 * frame was left over last time. Returns the number of whole frames staged.
 *
 * The splice is the point: the ringbuffer is byte-oriented and can hand back a
 * chunk that ends mid-frame, and simply dropping that remainder would shift L/R
 * phasing for the rest of the session. */
static uint32_t stage_stream(size_t framesz)
{
    size_t   item = 0;
    uint8_t *chunk = NULL;

    /* Drain whenever there is anything to drain - RING_DROPPING included. The
     * mode gates the producer, never this side: refusing to read while dropping
     * is a deadlock, since the producer only leaves RING_DROPPING once the
     * level it is waiting on falls, and nothing else can lower it. */
    if (s_stream_on && s_ring_mode != RING_PREFETCHING) {
        /* Never wait on the stream while a chime runs: it has to keep feeding
         * DMA even when the music has nothing for us this instant. */
        TickType_t rx = s_cur.active ? 0 : pdMS_TO_TICKS(20);
        chunk = (uint8_t *)xRingbufferReceiveUpTo(s_ring, &item, rx, CHUNK_FRAMES * framesz);
        if (item == 0) {
            s_ring_mode = RING_PREFETCHING;
        }
    }

    size_t staged = s_carry_len;
    memcpy(s_stage, s_carry, s_carry_len);
    if (chunk != NULL) {
        memcpy(s_stage + staged, chunk, item);
        staged += item;
        vRingbufferReturnItem(s_ring, chunk);
    }

    uint32_t frames = (uint32_t)(staged / framesz);
    s_carry_len = staged - (size_t)frames * framesz;
    memcpy(s_carry, s_stage + (size_t)frames * framesz, s_carry_len);
    return frames;
}

/* Move the music gain one frame closer to its target, without overshooting. */
static inline int32_t next_gain(int32_t target)
{
    if (s_gain_q12 > target) {
        s_gain_q12 -= s_duck_step;
        if (s_gain_q12 < target) {
            s_gain_q12 = target;
        }
    } else if (s_gain_q12 < target) {
        s_gain_q12 += s_duck_step;
        if (s_gain_q12 > target) {
            s_gain_q12 = target;
        }
    }
    return s_gain_q12;
}

static void mix_and_write(void)
{
    size_t   framesz = s_out_ch * sizeof(int16_t);
    uint32_t frames = stage_stream(framesz);

    /* NULL means no music this chunk - either the stream is idle or it
     * underran. A chime still has to be written, over silence. */
    const int16_t *music = (frames > 0) ? (const int16_t *)s_stage : NULL;
    if (frames == 0) {
        if (!s_cur.active) {
            return;                 /* neither producer has anything */
        }
        frames = CHUNK_FRAMES;
    }

    int32_t target = s_cur.active ? DUCK_TARGET_Q12 : GAIN_UNITY_Q12;

    for (uint32_t f = 0; f < frames; f++) {
        int32_t gain = next_gain(target);
        int32_t chime = s_cur.active ? cursor_next() : 0;

        for (uint8_t c = 0; c < s_out_ch; c++) {
            size_t i = (size_t)f * s_out_ch + c;
            int32_t ducked = music ? ((int32_t)music[i] * gain) >> 12 : 0;
            s_mix[i] = sat16(ducked + chime);
        }
    }

    /* A clip that ended inside this chunk hands off to whatever was waiting. */
    if (!s_cur.active && s_pending >= 0) {
        cursor_start(s_pending);
        s_pending = -1;
    }

    size_t written = 0;
    esp_err_t err = i2s_channel_write(s_chan, s_mix, (size_t)frames * framesz,
                                      &written, portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2s write: %s", esp_err_to_name(err));
    }

    /* A full pipeline blocks in the write, so back-to-back writes land one chunk
     * apart. A gap wider than the whole DMA depth means it drained before we got
     * back - the audio was already a hole by the time we noticed. Worth a log:
     * it is otherwise indistinguishable from a bad clip or a flaky amp. */
    int64_t now = esp_timer_get_time();
    if (s_last_write_us != 0 && now - s_last_write_us > s_dma_depth_us) {
        ESP_LOGW(TAG, "underrun: %"PRId64" ms between writes, DMA holds %"PRId64" ms",
                 (now - s_last_write_us) / 1000, s_dma_depth_us / 1000);
    }
    s_last_write_us = now;
}

/*******************************
 * TASK
 ******************************/

static void handle_ctl(const ctl_msg_t *m)
{
    switch (m->op) {
    case CTL_STREAM_START:
        s_stream_on = true;
        s_ring_mode = RING_PREFETCHING;
        break;

    case CTL_STREAM_STOP: {
        /* Drop what is left so the next session doesn't open with the tail of
         * the last one. */
        size_t n;
        void *p;
        s_stream_on = false;
        while ((p = xRingbufferReceiveUpTo(s_ring, &n, 0, RINGBUF_SIZE)) != NULL) {
            vRingbufferReturnItem(s_ring, p);
        }
        s_carry_len = 0;
        /* Packets keep landing for a moment after the link goes away, and with
         * the stream stopped nothing drains them. Start the next session from
         * prefetch rather than from a mode the last one left behind. */
        s_ring_mode = RING_PREFETCHING;
        break;
    }

    case CTL_STREAM_READY:
        break;   /* the wakeup was the point */

    case CTL_SET_FORMAT:
        if (m->rate != s_out_rate || m->ch != s_out_ch) {
            ESP_LOGI(TAG, "format -> %"PRIu32" Hz, %u ch", m->rate, m->ch);
            s_out_rate = m->rate;
            s_out_ch = m->ch;
            s_carry_len = 0;      /* leftovers are in the old frame size */
            if (s_chan != NULL) {
                channel_close();
                channel_open();   /* reopening beats disable/reconfig/enable,
                                   * where a mid-sequence failure leaves the
                                   * pins dead with no clocks */
            }
            retune();             /* a chime mid-flight follows the rate */
        }
        break;

    case CTL_PLAY_SFX:
        /* One voice, one waiting slot. A repeat of what is already playing is
         * dropped, so a state machine re-entering a state can't stutter it. */
        if (!s_cur.active) {
            cursor_start(m->clip);
        } else if (s_cur.clip != m->clip) {
            s_pending = m->clip;
        }
        break;
    }
}

static TickType_t linger_left(void)
{
    TickType_t total = pdMS_TO_TICKS(LINGER_MS);
    TickType_t gone = xTaskGetTickCount() - s_last_active;
    return (gone >= total) ? 0 : total - gone;
}

static void audio_out_task(void *arg)
{
    (void)arg;

    for (;;) {
        /* Two questions, and conflating them costs a click: 'writing' is
         * whether there is a chunk to mix now, 'needed' is whether the channel
         * should stay open. A paused stream is needed but not writing. */
        bool writing = (s_stream_on && s_ring_mode != RING_PREFETCHING) || s_cur.active;
        bool needed = s_stream_on || s_cur.active;

        if (needed) {
            s_last_active = xTaskGetTickCount();
        }

        ctl_msg_t msg;
        TickType_t wait = writing ? 0 : (s_chan ? linger_left() : portMAX_DELAY);
        if (xQueueReceive(s_ctl_q, &msg, wait) == pdTRUE) {
            handle_ctl(&msg);
            continue;
        }

        if (!writing) {
            /* An idle pass is a legitimate gap in the writes - a paused stream
             * inside the linger, say - so don't let the next write measure
             * against a stale timestamp and report it as an underrun. */
            s_last_write_us = 0;
            if (s_chan != NULL && !needed && linger_left() == 0) {
                channel_close();
                ESP_LOGI(TAG, "output idle, I2S released");
            }
            continue;
        }

        if (s_chan == NULL && channel_open() != ESP_OK) {
            ESP_LOGE(TAG, "could not open the output channel");
            vTaskDelay(pdMS_TO_TICKS(100));   /* don't spin on a dead peripheral */
            continue;
        }

        mix_and_write();
    }
}

/*******************************
 * PUBLIC
 ******************************/

static void ctl_post(ctl_op_t op, uint32_t rate, uint8_t ch, int clip)
{
    if (s_ctl_q == NULL) {
        return;
    }
    ctl_msg_t m = { .op = op, .rate = rate, .ch = ch, .clip = clip };
    if (xQueueSend(s_ctl_q, &m, 0) != pdTRUE) {
        ESP_LOGW(TAG, "control queue full, dropped op %d", (int)op);
    }
}

esp_err_t audio_out_init(void)
{
    /* Allocated once and never freed, unlike the old per-session buffer: same
     * peak cost, and it closes the case where the decode callback is mid-write
     * on a buffer the app task is deleting. */
    s_ring = xRingbufferCreate(RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    s_ctl_q = xQueueCreate(CTL_QUEUE_DEPTH, sizeof(ctl_msg_t));
    if (s_ring == NULL || s_ctl_q == NULL) {
        ESP_LOGE(TAG, "out of memory setting up audio output");
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(audio_out_task, "AudioOut", 3072, NULL,
                    configMAX_PRIORITIES - 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "could not create the audio task");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "audio output ready (channel opens on demand)");
    return ESP_OK;
}

void audio_out_stream_start(void)
{
    ctl_post(CTL_STREAM_START, 0, 0, 0);
}

void audio_out_stream_stop(void)
{
    ctl_post(CTL_STREAM_STOP, 0, 0, 0);
}

void audio_out_set_format(uint32_t sample_rate, uint8_t channels)
{
    ctl_post(CTL_SET_FORMAT, sample_rate, channels, 0);
}

esp_err_t audio_out_play_sfx(int clip_id)
{
    if (s_ctl_q == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    ctl_msg_t m = { .op = CTL_PLAY_SFX, .clip = clip_id };
    return (xQueueSend(s_ctl_q, &m, 0) == pdTRUE) ? ESP_OK : ESP_ERR_NO_MEM;
}

size_t audio_out_stream_write(const uint8_t *data, size_t size)
{
    size_t filled = 0;

    if (s_ring == NULL) {
        return 0;
    }

    if (s_ring_mode == RING_DROPPING) {
        vRingbufferGetInfo(s_ring, NULL, NULL, NULL, NULL, &filled);
        if (filled <= PREFETCH_LEVEL) {
            s_ring_mode = RING_PROCESSING;
        }
        return 0;
    }

    if (xRingbufferSend(s_ring, (void *)data, size, 0) != pdTRUE) {
        ESP_LOGW(TAG, "stream ringbuffer overflowed, dropping");
        s_ring_mode = RING_DROPPING;
        return 0;
    }

    if (s_ring_mode == RING_PREFETCHING) {
        vRingbufferGetInfo(s_ring, NULL, NULL, NULL, NULL, &filled);
        if (filled >= PREFETCH_LEVEL) {
            s_ring_mode = RING_PROCESSING;
            ctl_post(CTL_STREAM_READY, 0, 0, 0);
        }
    }

    return size;
}
