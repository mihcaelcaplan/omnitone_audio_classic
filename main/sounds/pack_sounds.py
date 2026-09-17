#!/usr/bin/env python3
"""Pack wav/*.wav into the sound bank the firmware flashes to the 'audio' partition.

Run by hand after adding or changing a sound; the build does NOT invoke this.
See README.md in this directory for the workflow and the reasoning.

    ./pack_sounds.py

Inputs:
    sounds.yaml   - which clips exist, which wav each one is, and how loud to
                    pack it. Optional: with no sounds.yaml every wav/*.wav is
                    packed at unity gain, which is what this script used to do.
    wav/*.wav     - the source audio

Outputs (both committed, both consumed by the build):
    sfx_bank.bin  - flashed to the 'audio' partition by main/CMakeLists.txt
    sfx_ids.h     - the sfx_id_t enum, so C can't drift from the sound list

Stdlib only, on purpose: this is a tool people run on whatever Python they
happen to have, and the one thing that would have made it easier - audioop -
was removed in Python 3.13. So the resampler below is hand-rolled, and so is
the (deliberately tiny) YAML reader.
"""

import math
import os
import struct
import sys
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
WAV_DIR = os.path.join(HERE, "wav")
CONFIG_IN = os.path.join(HERE, "sounds.yaml")
BANK_OUT = os.path.join(HERE, "sfx_bank.bin")
HEADER_OUT = os.path.join(HERE, "sfx_ids.h")
PARTITIONS_CSV = os.path.join(HERE, "..", "..", "partitions.csv")

# ---------------------------------------------------------------------------
# Bank format v1. Offsets are from the start of the partition so the firmware
# can esp_partition_read() them directly with no rebasing.
#
#   0x00  char magic[4]      'OSFX'
#   0x04  u16  version       = 1
#   0x06  u16  count
#   0x08  u32  sample_rate   (every clip, no per-clip rates by design)
#   0x0c  u32  reserved      = 0
#   0x10  entry[count] { u32 offset; u32 frames; }
#         ... s16le mono PCM, each clip padded to a 4-byte boundary
#
# Note that per-clip gain does NOT appear here. Level is baked into the samples
# at pack time, so the firmware reads exactly the same bank it always did and
# the mixer stays a memcpy-and-add. Retuning a chime costs a reflash, which is
# the right price for something that changes about as often as the artwork.
# ---------------------------------------------------------------------------
MAGIC = b"OSFX"
VERSION = 1
HEADER_SIZE = 0x10
ENTRY_SIZE = 8

# Everything lands at one rate/format so a clip can be mixed straight into the
# A2DP path without resampling at playback time. 44100 because that is what the
# overwhelming majority of phones negotiate for SBC; a 48k stream is the one
# case the firmware has to interpolate.
SAMPLE_RATE = 44100

# i2s_channel_write() returns once data is in the DMA descriptors, not once it
# has been clocked out. The default ESP32 channel config buffers
# dma_desc_num(6) * dma_frame_num(240) = 1440 frames, ~33ms at 44.1k, so a
# teardown right after the last write truncates the tail. Padding every clip
# here costs ~4KB of flash and means no playback path has to remember to wait.
TAIL_SILENCE_MS = 50

# Per-clip knobs, and what you get if sounds.yaml doesn't set them. peak_db of
# None means "leave the level alone"; set it to normalise a clip's peak to a
# target instead of guessing at a trim.
CLIP_DEFAULTS = {
    "file": None,
    "gain_db": 0.0,
    "peak_db": None,
}


def die(msg):
    sys.exit(f"pack_sounds: {msg}")


# ---------------------------------------------------------------------------
# A very small YAML subset: nested mappings of scalars, '#' comments, and the
# `name: file.wav` shorthand. No lists, no anchors, no multi-line strings, no
# flow style. That covers sounds.yaml completely and keeps this script runnable
# on a bare interpreter - PyYAML would be one `pip install` between someone and
# a sound change, which is exactly the friction the manual-packer trade was
# meant to avoid.
# ---------------------------------------------------------------------------

def _strip_comment(line):
    """Drop a trailing '# ...'. A '#' only starts a comment at the start of the
    line or after whitespace, and not inside quotes - same rule as real YAML."""
    quote = None
    for i, ch in enumerate(line):
        if quote:
            if ch == quote:
                quote = None
        elif ch in "\"'":
            quote = ch
        elif ch == "#" and (i == 0 or line[i - 1] in " \t"):
            return line[:i]
    return line


def _scalar(text):
    text = text.strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in "\"'":
        return text[1:-1]
    low = text.lower()
    if low in ("true", "yes"):
        return True
    if low in ("false", "no"):
        return False
    if low in ("null", "~", ""):
        return None
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        pass
    return text


def parse_yaml_subset(text, where):
    """text -> nested dicts. Raises SystemExit with a line number on anything
    outside the subset, rather than silently doing something surprising."""
    root = {}
    stack = [(-1, root)]

    for lineno, raw in enumerate(text.splitlines(), 1):
        if "\t" in raw[: len(raw) - len(raw.lstrip())]:
            die(f"{where}:{lineno}: tab in the indentation - YAML wants spaces")
        line = _strip_comment(raw).rstrip()
        if not line.strip():
            continue

        indent = len(line) - len(line.lstrip())
        line = line.strip()
        if line.startswith("- "):
            die(f"{where}:{lineno}: lists aren't supported here, use `name: value` mappings")

        # Split on the first ':' that isn't inside quotes.
        quote, sep = None, -1
        for i, ch in enumerate(line):
            if quote:
                if ch == quote:
                    quote = None
            elif ch in "\"'":
                quote = ch
            elif ch == ":":
                sep = i
                break
        if sep < 0:
            die(f"{where}:{lineno}: expected `key: value`, got {line!r}")

        key, value = line[:sep].strip(), line[sep + 1:].strip()
        if len(key) >= 2 and key[0] == key[-1] and key[0] in "\"'":
            key = key[1:-1]
        if not key:
            die(f"{where}:{lineno}: empty key")

        while indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]
        if not isinstance(parent, dict):
            die(f"{where}:{lineno}: `{key}` is nested under a value, not a mapping")
        if key in parent:
            die(f"{where}:{lineno}: duplicate key `{key}`")

        if value[:1] in ("{", "["):
            die(f"{where}:{lineno}: flow style ({value[:1]}...) isn't supported - "
                f"indent `{key}`'s settings on the following lines")
        if value == "":
            child = {}
            parent[key] = child
            stack.append((indent, child))
        else:
            parent[key] = _scalar(value)

    return root


# ---------------------------------------------------------------------------

def read_wav_mono(path):
    """Return (list[float] in [-1,1], sample_rate). Downmixes, any PCM width."""
    try:
        with wave.open(path, "rb") as w:
            channels, width, rate = w.getnchannels(), w.getsampwidth(), w.getframerate()
            raw = w.readframes(w.getnframes())
    except wave.Error as e:
        # wave only speaks WAVE_FORMAT_PCM, so float32 or compressed wavs land
        # here. Not worth a decoder - just say what to do about it.
        die(f"{os.path.basename(path)}: {e}\n"
            f"  Re-export as PCM, or: ffmpeg -i in.wav -acodec pcm_s16le -ar {SAMPLE_RATE} out.wav")

    count = len(raw) // width
    if width == 1:
        # 8-bit wav is unsigned, centred on 128
        vals = [(b - 128) / 128.0 for b in raw]
    elif width == 2:
        vals = [v / 32768.0 for v in struct.unpack(f"<{count}h", raw)]
    elif width == 3:
        vals = [int.from_bytes(raw[i:i + 3], "little", signed=True) / 8388608.0
                for i in range(0, len(raw), 3)]
    elif width == 4:
        vals = [v / 2147483648.0 for v in struct.unpack(f"<{count}i", raw)]
    else:
        die(f"{os.path.basename(path)}: unsupported sample width {width * 8}-bit")

    if channels > 1:
        vals = [sum(vals[i:i + channels]) / channels
                for i in range(0, len(vals) - channels + 1, channels)]
    return vals, rate


def resample_linear(samples, src_rate, dst_rate):
    """Linear interpolation. Fine for short chimes; do not use this on music."""
    if src_rate == dst_rate or not samples:
        return samples
    n_out = int(len(samples) * dst_rate / src_rate)
    ratio = src_rate / dst_rate
    out = []
    for i in range(n_out):
        pos = i * ratio
        j = int(pos)
        frac = pos - j
        a = samples[j]
        b = samples[j + 1] if j + 1 < len(samples) else a
        out.append(a + (b - a) * frac)
    return out


def audio_partition_size():
    """Size of the 'audio' partition, so an oversized bank fails here and not at
    flash time. Parsed rather than hardcoded to keep this honest if the table
    is ever resized."""
    try:
        with open(PARTITIONS_CSV) as f:
            for line in f:
                fields = [x.strip() for x in line.split("#")[0].split(",")]
                if len(fields) >= 5 and fields[0] == "audio":
                    size = fields[4]
                    if size.startswith("0x"):
                        return int(size, 16)
                    if size[-1] in "KM":
                        return int(size[:-1]) * 1024 * (1024 if size[-1] == "M" else 1)
                    return int(size)
    except OSError:
        pass
    print("  warning: could not read audio partition size from partitions.csv")
    return None


def c_ident(name):
    return "SFX_" + "".join(c if c.isalnum() else "_" for c in name).upper()


def db_str(scale):
    """The net level change, for the per-clip log line."""
    if scale <= 0.0:
        return "muted"
    return f"{20.0 * math.log10(scale):+.1f}dB"


def load_clips():
    """The sound list: [(name, {file, gain_db, peak_db}), ...] in enum order.

    From sounds.yaml when it exists, otherwise from a wav/*.wav glob so the
    directory still works with no config at all. Either way the list is sorted
    by name, which is what makes the bank reproducible and keeps a rebuild to a
    minimal diff - and note that it means adding a clip renumbers the ones after
    it, so the enum and the bank have to be regenerated and flashed together.
    """
    if not os.path.isdir(WAV_DIR):
        die(f"no wav directory at {WAV_DIR}")

    on_disk = sorted(f for f in os.listdir(WAV_DIR) if f.lower().endswith(".wav"))

    if not os.path.exists(CONFIG_IN):
        if not on_disk:
            die(f"no .wav files in {WAV_DIR} and no {os.path.basename(CONFIG_IN)}")
        print(f"no {os.path.basename(CONFIG_IN)}, packing every wav at unity gain")
        return [(os.path.splitext(n)[0], dict(CLIP_DEFAULTS, file=n)) for n in on_disk]

    with open(CONFIG_IN) as f:
        doc = parse_yaml_subset(f.read(), os.path.basename(CONFIG_IN))

    unknown = set(doc) - {"defaults", "sounds"}
    if unknown:
        die(f"{os.path.basename(CONFIG_IN)}: unknown top-level key(s): "
            f"{', '.join(sorted(unknown))} (expected `defaults`, `sounds`)")

    defaults = dict(CLIP_DEFAULTS)
    for key, value in (doc.get("defaults") or {}).items():
        if key not in CLIP_DEFAULTS or key == "file":
            die(f"{os.path.basename(CONFIG_IN)}: `defaults.{key}` isn't a clip setting "
                f"(expected gain_db, peak_db)")
        defaults[key] = value

    sounds = doc.get("sounds")
    if not isinstance(sounds, dict) or not sounds:
        die(f"{os.path.basename(CONFIG_IN)}: needs a non-empty `sounds:` mapping")

    clips, idents, used_files = [], {}, set()
    for name, spec in sounds.items():
        # `name: file.wav` is the shorthand for a clip with no settings.
        if not isinstance(spec, dict):
            spec = {"file": spec}
        clip = dict(defaults)
        for key, value in spec.items():
            if key not in CLIP_DEFAULTS:
                die(f"{os.path.basename(CONFIG_IN)}: `sounds.{name}.{key}` is not a setting "
                    f"(expected {', '.join(sorted(CLIP_DEFAULTS))})")
            clip[key] = value

        # Default the filename from the clip name, so the common case where the
        # two match needs no `file:` line at all.
        if clip["file"] is None:
            clip["file"] = f"{name}.wav"
        if not isinstance(clip["file"], str):
            die(f"{os.path.basename(CONFIG_IN)}: `sounds.{name}.file` must be a filename")

        for key in ("gain_db", "peak_db"):
            if clip[key] is not None and not isinstance(clip[key], (int, float)):
                die(f"{os.path.basename(CONFIG_IN)}: `sounds.{name}.{key}` must be a number in dB")
        if clip["gain_db"] is None:
            clip["gain_db"] = 0.0

        # The name becomes a C enumerator, so it has to survive c_ident() as one.
        ident = c_ident(str(name))
        if not str(name)[:1].isalpha():
            die(f"{os.path.basename(CONFIG_IN)}: sound name `{name}` must start with a letter")
        if ident in idents:
            die(f"{os.path.basename(CONFIG_IN)}: `{name}` and `{idents[ident]}` both become {ident}")
        idents[ident] = name

        path = os.path.join(WAV_DIR, clip["file"])
        if not os.path.isfile(path):
            die(f"{os.path.basename(CONFIG_IN)}: `sounds.{name}` points at wav/{clip['file']}, "
                f"which doesn't exist")
        used_files.add(clip["file"])
        clips.append((str(name), clip))

    for orphan in on_disk:
        if orphan not in used_files:
            print(f"  warning: wav/{orphan} isn't listed in {os.path.basename(CONFIG_IN)}, "
                  f"so it is not being packed")

    clips.sort(key=lambda c: c[0])
    return clips


def main():
    clips = load_clips()
    tail = [0.0] * int(SAMPLE_RATE * TAIL_SILENCE_MS / 1000)

    entries, blobs, offset = [], [], HEADER_SIZE + ENTRY_SIZE * len(clips)
    print(f"packing {len(clips)} sound(s) at {SAMPLE_RATE}Hz mono s16le")

    for name, clip in clips:
        samples, rate = read_wav_mono(os.path.join(WAV_DIR, clip["file"]))
        src_frames = len(samples)

        if rate != SAMPLE_RATE:
            samples = resample_linear(samples, rate, SAMPLE_RATE)

        # Level, baked in here rather than at playback: peak_db normalises the
        # clip's loudest sample to a target, gain_db then trims it. Applying
        # them in that order means `defaults: {peak_db: -6}` plus a per-clip
        # `gain_db` reads as "everything to the same level, except this one,
        # which sits 3dB under" - which is how you actually think about chimes.
        peak = max((abs(s) for s in samples), default=0.0)
        scale = 1.0
        if clip["peak_db"] is not None:
            if peak <= 0.0:
                print(f"  warning: {clip['file']} is silent, peak_db has nothing to normalise")
            else:
                scale = (10.0 ** (clip["peak_db"] / 20.0)) / peak
        scale *= 10.0 ** (clip["gain_db"] / 20.0)
        if scale != 1.0:
            samples = [s * scale for s in samples]
            peak *= scale

        if peak > 1.0:
            print(f"  warning: {name} clips (peak {peak:.2f}), it will be hard-limited"
                  + ("  - lower gain_db" if clip["gain_db"] > 0 else ""))
        elif peak < 0.05:
            print(f"  warning: {name} is very quiet (peak {peak:.3f})")

        samples = samples + tail
        pcm = b"".join(
            struct.pack("<h", max(-32768, min(32767, int(s * 32767)))) for s in samples
        )
        pcm += b"\x00" * (-len(pcm) % 4)  # keep the next clip 4-byte aligned

        entries.append((offset, len(samples)))
        blobs.append(pcm)

        # Report the level actually applied, so the log is enough to tune from.
        print(f"  {c_ident(name):<20} {src_frames / rate * 1000:6.0f}ms  "
              f"{len(pcm) / 1024:6.1f}K  {db_str(scale):>8}  peak {peak:4.2f}  @{offset:#08x}"
              + (f"  (resampled from {rate}Hz)" if rate != SAMPLE_RATE else ""))
        offset += len(pcm)

    bank = bytearray()
    bank += struct.pack("<4sHHII", MAGIC, VERSION, len(clips), SAMPLE_RATE, 0)
    for off, frames in entries:
        bank += struct.pack("<II", off, frames)
    for blob in blobs:
        bank += blob

    limit = audio_partition_size()
    if limit and len(bank) > limit:
        die(f"bank is {len(bank) / 1024:.1f}K but the audio partition is only "
            f"{limit / 1024:.0f}K - shorten a clip or grow the partition")

    with open(BANK_OUT, "wb") as f:
        f.write(bank)

    guard_names = "\n".join(
        f"    {c_ident(name):<24} = {i},   /* wav/{clip['file']} */"
        for i, (name, clip) in enumerate(clips)
    )
    with open(HEADER_OUT, "w") as f:
        f.write(f"""/* Generated by pack_sounds.py - do not edit.
 *
 * Regenerate with ./pack_sounds.py after changing sounds.yaml or anything in
 * wav/. Committed alongside sfx_bank.bin so the build never depends on Python.
 */
#pragma once

#include <stdint.h>

#define SFX_BANK_MAGIC        "{MAGIC.decode()}"
#define SFX_BANK_VERSION      {VERSION}
#define SFX_BANK_HEADER_SIZE  {HEADER_SIZE}
#define SFX_BANK_ENTRY_SIZE   {ENTRY_SIZE}
#define SFX_BANK_SAMPLE_RATE  {SAMPLE_RATE}

typedef enum {{
{guard_names}
    SFX_COUNT = {len(clips)}
}} sfx_id_t;
""")

    used = f" ({100 * len(bank) / limit:.1f}% of partition)" if limit else ""
    print(f"\n{os.path.relpath(BANK_OUT, HERE)}: {len(bank) / 1024:.1f}K{used}")
    print(f"{os.path.relpath(HEADER_OUT, HERE)}: {len(clips)} id(s)")
    print("\nremember to commit both outputs, then flash.")


if __name__ == "__main__":
    main()
