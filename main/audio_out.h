/*
 * Audio output: the single owner of the I2S channel.
 *
 * One task does every write to the peripheral. Two producers feed it:
 *
 *   - the SBC decode callback, pushing PCM into a ringbuffer
 *     (audio_out_stream_write)
 *   - anyone at all, asking for a sound effect (sfx_play -> audio_out_play_sfx)
 *
 * The task mixes the second on top of the first, so a chime is heard whether or
 * not music is playing, and never interrupts it.
 *
 * Nothing outside audio_out.c ever touches the channel handle. Callers do not
 * install, release, or reason about it: the task brings I2S up when a producer
 * needs it and drops it once both go quiet. That is deliberate - chimes get
 * fired from state machines all over the stack (Bluetooth events, the nRF SPI
 * bridge, battery monitoring) that run at completely different timescales, and
 * none of them should have to know what the others are doing.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* Create the control queue, the stream ringbuffer, and the audio task. The task
 * runs for the life of the device; the I2S channel does not. Call once, early -
 * before anything can call the functions below. */
esp_err_t audio_out_init(void);

/* A2DP session boundaries. The stream is a producer for as long as it is
 * started, which is what keeps I2S alive across a pause. */
void audio_out_stream_start(void);
void audio_out_stream_stop(void);

/* Output format, from the SBC codec configuration. Asynchronous like the rest:
 * it lands between mix chunks, so it can never tear a buffer that is mid-write,
 * and a chime that is playing is retuned to the new rate rather than cut. */
void audio_out_set_format(uint32_t sample_rate, uint8_t channels);

/* PCM from the decode callback. Safe to call from the BT task at stream rate;
 * this only touches the ringbuffer. Returns bytes accepted, 0 if dropped. */
size_t audio_out_stream_write(const uint8_t *pcm, size_t len);

/* Queue a clip. Non-blocking and bounded - a queue send, nothing more - so it
 * is safe from any task and any callback context. Prefer sfx_play() in sfx.h,
 * which is this plus the type-checked id. */
esp_err_t audio_out_play_sfx(int clip_id);
