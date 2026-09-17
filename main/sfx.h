/*
 * Sound effect bank: what clips exist and where their samples live.
 *
 * The bank is built by sounds/pack_sounds.py and flashed alongside the app; see
 * sounds/README.md for the format and the workflow. Playback itself belongs to
 * audio_out.c, which is the only thing that touches the I2S peripheral.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "sounds/sfx_ids.h"

/* Locate, validate and memory-map the bank. Call once, before sfx_play().
 *
 * Returns ESP_ERR_NOT_FOUND if the partition is missing, ESP_ERR_INVALID_STATE
 * if the bank is absent, stale or malformed. Either way playback degrades to
 * silence rather than an error callers have to handle - a missing chime must
 * never stop a speaker from working. */
esp_err_t sfx_init(void);

/* Ask for a clip. Fire and forget, from any task, in any state, at any time:
 * this is a queue send and returns in microseconds.
 *
 * The clip will be mixed on top of whatever the A2DP stream is doing, and the
 * audio task brings the output up and down around it as needed. Callers never
 * deal with the I2S lifecycle. */
esp_err_t sfx_play(sfx_id_t id);

/* Resolve a clip to its samples. For audio_out's mixer; the pointer is into
 * mmap'd flash and stays valid for the life of the program. Returns false for
 * an unknown id or an unusable bank. */
bool sfx_lookup(int clip_id, const int16_t **pcm, uint32_t *frames);
