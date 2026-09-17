#!/usr/bin/env python3
"""Push an ESP app image to the nRF over BLE SMP so it updates the ESP at its next boot.

Uploads build/a2dp_sink.bin to /lfs/pkg/esp.bin (CONFIG_OT_ESP_OTA_IMAGE_PATH on
the nRF), verifies the CRC32 on the device, and resets it. On boot the nRF
compares the version embedded in the image against /lfs/sync/applied and, if
it differs, streams it over the SPI bridge (ot_esp_ota.c). Watch the ESP's
UART: it logs every OTA_* step under OMNI_OTA and prints the version in its
banner once it comes back.

Same shape as the nRF tree's dsp/upload.py. The device advertises SMP for
CONFIG_AUDIO_BT_MGMT_DFU_BOOT_WINDOW_MS (5 s) after boot, so if it is not
found, power-cycle it while the script is scanning.

    tools/upload.py                        # build/a2dp_sink.bin -> /lfs/pkg/esp.bin, then reset
    tools/upload.py other.bin --no-reset   # stage only; applied on the nRF's next boot
    tools/upload.py --name MY_DEVICE_DFU

Needs the smpclient library (ships with smpmgr, https://github.com/intercreate/smpmgr).
First run creates tools/.venv and installs it there; nothing else to set up.
"""

import argparse
import asyncio
import os
import struct
import subprocess
import sys
import time
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
VENV = HERE / ".venv"

DEFAULT_IMAGE = HERE.parent / "build" / "a2dp_sink.bin"
DEST = "/lfs/pkg/esp.bin"                     # CONFIG_OT_ESP_OTA_IMAGE_PATH
DEFAULT_NAME = "NRF5340_BIS_HEADSET_DFU"      # CONFIG_BT_DEVICE_NAME + "_DFU" (bt_mgmt_dfu.c)

# ESP app image layout (esp_app_format.h), the same fields ot_esp_ota_image_version() reads:
# [esp_image_header_t 24 B][esp_image_segment_header_t 8 B][esp_app_desc_t: magic u32, secure_version u32,
#  reserv1[2] u32, version[32], project_name[32], time[16], date[16], idf_ver[32], ...]
ESP_IMAGE_MAGIC = 0xE9
ESP_APP_DESC_OFFSET = 32
ESP_APP_DESC_MAGIC = 0xABCD5432


def esp_image_info(data: bytes) -> dict:
    """Return the app descriptor fields of an ESP app image, or raise ValueError."""
    if len(data) < ESP_APP_DESC_OFFSET + 16 + 32 + 32 + 16 + 16 + 32 or data[0] != ESP_IMAGE_MAGIC:
        raise ValueError("not an ESP app image (bad header magic)")
    (magic,) = struct.unpack_from("<I", data, ESP_APP_DESC_OFFSET)
    if magic != ESP_APP_DESC_MAGIC:
        raise ValueError("not an ESP app image (no esp_app_desc_t)")
    off = ESP_APP_DESC_OFFSET + 16
    fields = {}
    for name, size in (("version", 32), ("project", 32), ("time", 16), ("date", 16), ("idf", 32)):
        fields[name] = data[off:off + size].split(b"\0", 1)[0].decode(errors="replace")
        off += size
    return fields


def ensure_smpclient():
    """Import smpclient, bootstrapping a local venv on first use."""
    try:
        import smpclient  # noqa: F401
        return
    except ImportError:
        pass

    venv_python = VENV / "bin" / "python"
    if sys.executable != str(venv_python):
        if not venv_python.exists():
            print(f"setting up {VENV} (once) ...", file=sys.stderr)
            subprocess.check_call([sys.executable, "-m", "venv", str(VENV)])
            subprocess.check_call([str(VENV / "bin" / "pip"), "install", "-q", "smpmgr"])
        os.execv(str(venv_python), [str(venv_python), *sys.argv])
    sys.exit(f"smpclient still not importable inside {VENV}; delete it and retry")


async def run(image: Path, name: str, scan_timeout: float, reset: bool) -> int:
    from smpclient import SMPClient
    from smpclient.generics import error, success
    from smpclient.requests.file_management import FileHashChecksum
    from smpclient.requests.os_management import ResetWrite
    from smpclient.transport.ble import SMPBLETransport

    data = image.read_bytes()
    local_crc = zlib.crc32(data) & 0xFFFFFFFF
    print(f"{image.name}: {len(data)} bytes, crc32 {local_crc:08x} -> {DEST}")
    print(f"scanning for '{name}' (up to {scan_timeout:.0f} s; power-cycle the device if it is not advertising) ...")

    client = SMPClient(SMPBLETransport(), name, timeout_s=5.0)
    await client.connect(connect_timeout_s=scan_timeout)
    print("connected")

    # smpclient raises TimeoutError / RuntimeError("Lock is not acquired") /
    # OSError when the device drops the link mid-request
    LINK_ERRORS = (TimeoutError, RuntimeError, OSError)
    step = "upload"
    try:
        t0 = time.monotonic()
        last = 0
        async for offset in client.upload_file(data, DEST):
            if offset - last >= 16384 or offset == len(data):
                dt = time.monotonic() - t0
                rate = offset / dt / 1024 if dt > 0 else 0
                print(f"  {offset}/{len(data)}  {offset * 100 // len(data):3d}%  {rate:5.1f} KB/s", end="\r", flush=True)
                last = offset
        print(f"\nuploaded {len(data)} bytes in {time.monotonic() - t0:.0f} s")

        step = "crc32 check"
        r = await client.request(FileHashChecksum(name=DEST, type="crc32"))
        if error(r):
            print(f"hash request failed: {r}")
            return 1
        remote_crc = int(r.output) & 0xFFFFFFFF
        if remote_crc != local_crc:
            print(f"CRC MISMATCH: device {remote_crc:08x}, local {local_crc:08x}")
            return 1
        print(f"crc32 verified ({remote_crc:08x})")

        if reset:
            step = "reset"
            r = await client.request(ResetWrite())
            print("reset requested; the nRF applies the image at boot - watch the ESP UART for OMNI_OTA"
                  if success(r) else f"reset failed: {r}")
        else:
            print("not resetting; the image is applied on the nRF's next boot")
    except LINK_ERRORS as e:
        print(f"\nlink lost during {step} ({e.__class__.__name__}): the device dropped the connection,"
              " which usually means it faulted - check its log (stack overflow / assert)")
        return 1
    finally:
        try:
            await client.disconnect()
        except Exception:
            pass
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("image", nargs="?", type=Path, default=DEFAULT_IMAGE,
                   help=f"ESP app .bin to send (default {DEFAULT_IMAGE.relative_to(HERE.parent)})")
    p.add_argument("--name", default=DEFAULT_NAME, help=f"advertised BLE name or address (default {DEFAULT_NAME})")
    p.add_argument("--scan-timeout", type=float, default=60.0, help="seconds to wait for the device (default 60)")
    p.add_argument("--no-reset", action="store_true", help="leave the device running after upload")
    args = p.parse_args()

    if not args.image.exists():
        sys.exit(f"{args.image}: not found (build first)")
    try:
        info = esp_image_info(args.image.read_bytes())
    except ValueError as e:
        sys.exit(f"{args.image}: {e}")
    print(f"{info['project']} {info['version']}  built {info['date']} {info['time']}  idf {info['idf']}")
    if info["version"].endswith("-dirty") or not info["version"]:
        print("note: version comes from git describe; put a string in version.txt to give each test image its own",
              file=sys.stderr)

    ensure_smpclient()
    sys.exit(asyncio.run(run(args.image, args.name, args.scan_timeout, not args.no_reset)))


if __name__ == "__main__":
    main()
