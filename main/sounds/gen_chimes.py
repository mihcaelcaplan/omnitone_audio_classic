#!/usr/bin/env python3
"""Generate wavetable-synth chord chimes into wav/, then optionally wire them
into sounds.yaml.

    ./gen_chimes.py

Writes each clip in CLIPS below to wav/<name>.wav (asking before clobbering
one that already exists), then asks once whether to add them to sounds.yaml.
If you say yes, it asks only for a volume (gain_db) per clip - the `file:`
line is never needed since the clip name already matches the wav filename.
Clips already present in sounds.yaml are left alone.

Stdlib only, same rule as pack_sounds.py: no numpy, hand-rolled synthesis.
This script does NOT run pack_sounds.py - do that yourself once sounds.yaml
looks right.
"""

import math
import os
import struct
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
WAV_DIR = os.path.join(HERE, "wav")
CONFIG_IN = os.path.join(HERE, "sounds.yaml")
SR = 44100

NOTE = {
    "C4": 261.63, "E4": 329.63, "F4": 349.23, "G4": 392.00, "A4": 440.00,
    "C5": 523.25, "E5": 659.25, "G5": 783.99,
}


def amp_env(t, dur, a, d, s, r):
    if t < a:
        return t / a
    if t < a + d:
        return 1.0 + (s - 1.0) * (t - a) / d
    if t < dur - r:
        return s
    if t >= dur:
        return 0.0
    return s * (1.0 - (t - (dur - r)) / r)


# ---------------------------------------------------------------------------
# Wavetables: single-cycle waveforms built once by additive synthesis, from
# harmonic-rich/buzzy down to a near-sine. A note reads through one of these
# (linear-interpolated, looping) rather than a live sine call per sample, and
# crossfades along the bank over its lifetime for the same "bright attack
# settling into a warm sustain" shape the old FM patch had.
# ---------------------------------------------------------------------------
WAVE_LEN = 2048


def build_table(harmonics):
    """harmonics: [(n, amplitude), ...]. Returns a peak-normalized table."""
    table = [0.0] * WAVE_LEN
    for n, amp in harmonics:
        for i in range(WAVE_LEN):
            table[i] += amp * math.sin(2 * math.pi * n * i / WAVE_LEN)
    peak = max(abs(x) for x in table)
    return [x / peak for x in table]


WAVETABLES = [
    build_table([(n, 1.0 / n) for n in range(1, 17)]),        # bright, saw-like
    build_table([(n, 1.0 / n ** 1.3) for n in range(1, 9)]),   # softer
    build_table([(n, 1.0 / n ** 1.7) for n in range(1, 5)]),   # softer still
    build_table([(1, 1.0), (2, 0.15), (3, 0.05)]),             # near-sine, mellow
]


def read_table(table, pos):
    i0 = int(pos) % WAVE_LEN
    i1 = (i0 + 1) % WAVE_LEN
    frac = pos - int(pos)
    return table[i0] * (1.0 - frac) + table[i1] * frac


def wavetable_note(freq, start, dur, sr=SR, morph_start=0.0, morph_end=3.0,
                    morph_decay=6.0, a=0.004, d=0.25, s=0.35, r=0.35, gain=1.0):
    """One oscillator reading through WAVETABLES, morphing from morph_start to
    morph_end (table index, can be fractional) as the note decays."""
    n = int(dur * sr)
    out = [0.0] * n
    n_tables = len(WAVETABLES)
    inc = freq * WAVE_LEN / sr
    phase = 0.0
    for i in range(n):
        t = i / sr
        pos_idx = morph_end + (morph_start - morph_end) * math.exp(-t * morph_decay)
        pos_idx = max(0.0, min(n_tables - 1, pos_idx))
        lo = int(pos_idx)
        hi = min(lo + 1, n_tables - 1)
        frac = pos_idx - lo
        sample = (read_table(WAVETABLES[lo], phase) * (1.0 - frac)
                  + read_table(WAVETABLES[hi], phase) * frac)
        out[i] = gain * amp_env(t, dur, a, d, s, r) * sample
        phase = (phase + inc) % WAVE_LEN
    return int(start * sr), out


def render(events, total_dur, sr=SR):
    n_total = int(total_dur * sr)
    buf = [0.0] * n_total
    for start_sample, samples in events:
        for i, v in enumerate(samples):
            idx = start_sample + i
            if idx < n_total:
                buf[idx] += v
    peak = max(1e-9, max(abs(x) for x in buf))
    scale = 0.9 / peak
    return [int(max(-32768, min(32767, x * scale * 32767))) for x in buf]


def write_wav(path, samples, sr=SR):
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(struct.pack("<%dh" % len(samples), *samples))


def make_startup():
    """Ascending C major arpeggio blooming into a sustained chord."""
    notes = [("C4", 0.00, 1.50), ("E4", 0.09, 1.41), ("G4", 0.18, 1.32), ("C5", 0.27, 1.23)]
    events = [wavetable_note(NOTE[n], start, dur, morph_start=0.0, morph_end=3.0,
                              morph_decay=5.0, a=0.006, d=0.35, s=0.30, r=0.55, gain=0.9)
              for n, start, dur in notes]
    return render(events, total_dur=1.6)


def make_connected():
    """Quick two-chord ding-dong: soft IV stab landing on a brighter I stab."""
    events = []
    for n in ("F4", "A4", "C5"):
        events.append(wavetable_note(NOTE[n], 0.00, 0.28, morph_start=0.5, morph_end=3.0,
                                      morph_decay=9.0, a=0.003, d=0.10, s=0.15, r=0.15, gain=0.55))
    for n in ("C5", "E5", "G5"):
        events.append(wavetable_note(NOTE[n], 0.14, 0.55, morph_start=0.0, morph_end=2.5,
                                      morph_decay=6.0, a=0.003, d=0.15, s=0.30, r=0.35, gain=0.85))
    return render(events, total_dur=0.75)


# name -> (builder, wav filename)
CLIPS = {
    "startup": make_startup,
    "connected": make_connected,
}


def ask(prompt, default=""):
    reply = input(prompt).strip()
    return reply if reply else default


def generate_wavs():
    written = []
    for name, builder in CLIPS.items():
        path = os.path.join(WAV_DIR, f"{name}.wav")
        if os.path.exists(path):
            if ask(f"wav/{name}.wav already exists, overwrite? [y/N]: ").lower() not in ("y", "yes"):
                print(f"  skipped {name}.wav")
                continue
        write_wav(path, builder())
        print(f"  wrote wav/{name}.wav")
        written.append(name)
    return written


def existing_sound_keys(text):
    """Top-level keys already under `sounds:` in sounds.yaml, so we don't
    clobber or duplicate an entry someone already wrote."""
    lines = text.splitlines()
    keys, in_sounds = set(), False
    for line in lines:
        stripped = line.split("#", 1)[0].rstrip()
        if not stripped.strip():
            continue
        indent = len(stripped) - len(stripped.lstrip())
        if indent == 0:
            in_sounds = stripped.strip() == "sounds:"
            continue
        if in_sounds and indent == 2:
            key = stripped.strip().split(":")[0].strip().strip("\"'")
            keys.add(key)
    return keys


def append_sound_entries(entries):
    """entries: [(name, gain_db)]. Appends to the end of the `sounds:` block,
    skipping any name already present. Leaves everything else in the file -
    including its comments - untouched."""
    with open(CONFIG_IN) as f:
        text = f.read()
    lines = text.splitlines()

    sounds_idx = next((i for i, l in enumerate(lines) if l.rstrip() == "sounds:"), None)
    if sounds_idx is None:
        print(f"  no top-level `sounds:` key in {os.path.basename(CONFIG_IN)}, skipping")
        return

    existing = existing_sound_keys(text)
    end_idx = len(lines)
    for i in range(sounds_idx + 1, len(lines)):
        if lines[i].strip() and len(lines[i]) - len(lines[i].lstrip()) == 0:
            end_idx = i
            break

    new_lines = []
    for name, gain_db in entries:
        if name in existing:
            print(f"  sounds.yaml already has `{name}`, leaving it alone")
            continue
        if gain_db:
            new_lines += [f"  {name}:", f"    gain_db: {gain_db:.1f}"]
        else:
            new_lines += [f"  {name}: {name}.wav"]
    if not new_lines:
        return

    if end_idx > 0 and lines[end_idx - 1].strip() != "":
        new_lines = [""] + new_lines
    lines[end_idx:end_idx] = new_lines
    with open(CONFIG_IN, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  updated {os.path.basename(CONFIG_IN)}")


def configure(written):
    if not written:
        return
    if ask(f"\nConfigure sounds.yaml now for {', '.join(written)}? [y/N]: ").lower() not in ("y", "yes"):
        print("Skipped sounds.yaml - add entries by hand when ready.")
        return
    entries = []
    for name in written:
        raw = ask(f"  {name} volume (gain_db, default 0.0): ")
        entries.append((name, float(raw) if raw else 0.0))
    append_sound_entries(entries)
    print("Run ./pack_sounds.py to rebuild the bank once sounds.yaml looks right.")


if __name__ == "__main__":
    written = generate_wavs()
    configure(written)
