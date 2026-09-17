# Sound effects

Event chimes (power on, connect, ...) that the firmware plays out of the `audio`
flash partition.

```
sounds.yaml     the sound list: name -> wav + level. The other thing you edit
wav/            source .wav files - the thing you edit
pack_sounds.py  manual: sounds.yaml + wav/ -> sfx_bank.bin + sfx_ids.h
sfx_bank.bin    generated, committed, flashed to the 'audio' partition
sfx_ids.h       generated, committed, gives C the sfx_id_t enum
```

## Adding or changing a sound

1. Drop a `.wav` into `wav/`.
2. Add it to `sounds.yaml`. The key becomes the enum name: `low_battery` →
   `SFX_LOW_BATTERY`.
   ```yaml
   sounds:
     low_battery:
       file: low_battery.wav
       gain_db: -3
   ```
3. Run the packer:
   ```
   ./pack_sounds.py
   ```
4. Commit **all three** of `sounds.yaml`, `sfx_bank.bin` and `sfx_ids.h` along
   with the `.wav`.
5. Build and flash normally.

## sounds.yaml

The sound list, and the only place clip level is set.

```yaml
defaults:
  peak_db: -6.0        # applied to every clip that doesn't override it

sounds:
  "on": on.wav         # shorthand: just a filename, everything else defaults
  connect:
    file: trapaholics.wav
    peak_db: -6.0      # normalise this clip's peak to -6dBFS
    gain_db: -3.0      # ...then trim it 3dB under the rest
```

| key | default | meaning |
| --- | --- | --- |
| `file` | `<name>.wav` | which wav in `wav/`. Only needed when the clip name and the filename differ - which is how you name a clip for what it is *for* rather than what the artist called it. |
| `peak_db` | unset | normalise the clip's loudest sample to this dBFS. The knob for "make everything sit at the same level". |
| `gain_db` | `0` | flat trim, applied *after* `peak_db`. Use alone to nudge a clip you are already happy with. |

Clips are ordered by name, so the bank is reproducible and rebuilds diff
cleanly. Adding a clip renumbers the ones after it - which is fine, because
`sfx_ids.h` and `sfx_bank.bin` are always regenerated and flashed together, and
`sfx_init()` refuses to run a bank whose clip count disagrees with the enum.

Quote `"on"`, `"off"`, `"no"` and friends: YAML 1.1 reads them as booleans. The
packer's own reader treats every key as a string, but other YAML tools won't.

**Level is baked into the samples at pack time**, not applied at playback. The
bank format is unchanged by any of this and the firmware never sees a gain
field - the mixer stays an add. The cost is that retuning a chime needs a
reflash, which is the right price for something that changes about as often as
the artwork does. Note that `audio_out.c` then applies a further -6dB
(`SFX_GAIN_Q12`) to every clip, so these numbers set clips relative to each
other; that constant sets them relative to the music.

If `sounds.yaml` is missing entirely the packer falls back to globbing
`wav/*.wav` at unity gain, which is what it did before this file existed. A wav
that sits in `wav/` without a `sounds.yaml` entry is *not* packed, and the
packer says so.

### Why a hand-rolled YAML reader

`pack_sounds.py` parses a small subset itself - nested mappings of scalars,
`#` comments, no lists or flow style or anchors - rather than importing PyYAML.
The whole point of keeping the packer manual and stdlib-only is that a sound
change needs nothing but whatever Python is already on the machine; a
`pip install` in that path would give back the friction the design was avoiding.
Anything outside the subset is a hard error with a line number, so the failure
mode is a message rather than a surprise.

**The build does not run the packer.** If you change a `.wav` and forget step 3,
the build succeeds and flashes the *old* sounds — nothing will warn you. That is
a deliberate trade: keeping the conversion manual means a fresh clone builds and
flashes with only the IDF toolchain, no Python version pinning, no ffmpeg, no
venv. Given sounds change roughly never and the build runs constantly, the cost
lands in the right place. If you find yourself getting bitten, `add_custom_command`
on `sounds.yaml` and the `wav/` glob in `main/CMakeLists.txt` is the fix.

## Source file requirements

Any sample rate, any channel count, 8/16/24/32-bit **PCM** wav. The packer
downmixes to mono, resamples to 44.1kHz, and converts to signed 16-bit.

Python's `wave` module only reads PCM, so float32 or compressed wavs are
rejected with a message telling you this. Convert first:

```
ffmpeg -i whatever.mp3 -acodec pcm_s16le -ar 44100 wav/on.wav
```

Aim for a peak around −6dBFS. Chimes get mixed against live music and may be
ducked, so leaving headroom is worth more than being loud. You do not have to
hit that in the source file - set `peak_db: -6.0` in `sounds.yaml` and the
packer normalises for you. It warns either way if a clip ends up clipping or
nearly silent.

`wav/on.wav` is currently a **synthesised placeholder** — a 220ms sine glide from
500Hz to 1kHz. Replace it with real audio.

## Why mono 44.1kHz

Everything is stored at one rate and format so a clip can be mixed straight into
the A2DP output path with no runtime resampling. 44.1kHz because that is what
nearly every phone negotiates for SBC — a 48kHz stream is the one case the
firmware has to interpolate, and it interpolates the short chime rather than the
music.

Mono halves the flash cost (88KB/sec vs 176) and costs one duplicated store per
frame at playback. At 2MB the `audio` partition holds ~23 seconds; the packer
refuses to write a bank that would overflow it, reading the real size out of
`partitions.csv` rather than hardcoding it.

## Bank format (v1)

Little-endian. Offsets are from the start of the partition, so the firmware
`esp_partition_read()`s them directly with no rebasing.

```
0x00  char magic[4]      'OSFX'
0x04  u16  version       = 1
0x06  u16  count
0x08  u32  sample_rate   applies to every clip
0x0c  u32  reserved      = 0
0x10  entry[count] { u32 offset; u32 frames; }
      ... s16le mono PCM, each clip padded to a 4-byte boundary
```

Clips are ordered by name so the bank is reproducible and rebuilds diff
cleanly. `sfx_ids.h` re-exports the magic, version, header/entry sizes and rate
as `#define`s, so the C reader takes its constants from the same place the packer
wrote them and the two cannot drift.

### The 50ms of silence on the end of every clip

`i2s_channel_write()` returns once data is in the DMA descriptors, not once it
has been clocked out. The default ESP32 channel config buffers
`dma_desc_num(6) * dma_frame_num(240)` = 1440 frames ≈ 33ms at 44.1kHz, so
tearing the channel down right after the last write truncates the tail of the
sound. Padding here costs ~4KB per clip and means no playback path has to
remember to wait before calling `i2s_channel_disable()`.

## Flashing

`esptool_py_flash_to_partition()` in `../CMakeLists.txt` writes `sfx_bank.bin`
into the `audio` partition on every app flash, so sounds and firmware stay in
step on a device.

Note this needs a full flash rather than an app-only one. It is a separate image
at its own offset, so an app-only shortcut leaves whatever bank was already
there.
