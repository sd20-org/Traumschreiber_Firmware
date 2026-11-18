#!/usr/bin/env python3
"""Install a firmware image on a Traumschreiber over Bluetooth (MCUmgr / SMP).

Works on Linux, macOS and Windows with Python 3.9+:

    pip install bleak cbor2
    python ota_update.py --info                              # show what is installed
    python ota_update.py --latest                            # our newest official firmware
    python ota_update.py build/Firmware/zephyr/zephyr.signed.bin   # your own build

--latest downloads the newest official image (signed by us) from the Releases of
github.com/sd20-org/Traumschreiber_Firmware. A maker edition accepts these and images signed
with its own device key (Firmware/TS2607-<code>_signing_key.pem on its microSD card). An image
signed with any other key is uploaded, but the bootloader refuses to start it and the old
firmware keeps running.

Sequence: upload into the second slot, mark it for a test boot, reset. The firmware
confirms itself after a successful start; if it does not start, MCUboot falls back to
the previous image on the next reset.

Switch the recording off in the app first (or close the app): the device accepts only
one Bluetooth connection.
"""
from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import struct
import sys
import time
import urllib.request

try:
    import cbor2
    from bleak import BleakClient, BleakScanner
except ImportError:
    sys.exit("Missing packages. Install them with:  pip install bleak cbor2")

SMP_SERVICE = "8d53dc1d-1db7-4cd3-868b-8a527460aa84"
SMP_CHAR = "da2e7828-fbce-4e01-ae9e-261174997c48"
DEVICE_NAME = "Traumschreiber"
RELEASES = "https://api.github.com/repos/sd20-org/Traumschreiber_Firmware/releases/latest"

# The firmware's SMP buffer holds 384 bytes and does not reassemble frames that span
# several Bluetooth writes, so every request must fit into ONE write of at most 384 bytes.
FRAME_MAX = 350

OP_READ, OP_WRITE = 0, 2
GROUP_OS, GROUP_IMAGE = 0, 1
ID_IMAGE_STATE, ID_IMAGE_UPLOAD, ID_OS_RESET = 0, 1, 5


class Smp:
    def __init__(self, client: BleakClient):
        self.c = client
        self.seq = 0
        self.buf = bytearray()
        self.waiting: dict[int, asyncio.Future] = {}

    async def start(self):
        await self.c.start_notify(SMP_CHAR, self._on_notify)

    def _on_notify(self, _sender, data: bytearray):
        self.buf.extend(data)
        while len(self.buf) >= 8:
            length = struct.unpack(">H", self.buf[2:4])[0]
            if len(self.buf) < 8 + length:
                return
            frame, self.buf = bytes(self.buf[:8 + length]), self.buf[8 + length:]
            fut = self.waiting.pop(frame[6], None)
            if fut and not fut.done():
                fut.set_result(cbor2.loads(frame[8:]) if length else {})

    def frame(self, op, group, cmd, payload: dict) -> bytes:
        body = cbor2.dumps(payload)
        return struct.pack(">BBHHBB", op, 0, len(body), group, self.seq & 0xFF, cmd) + body

    async def request(self, op, group, cmd, payload: dict, timeout=10.0) -> dict:
        data = self.frame(op, group, cmd, payload)
        if len(data) > FRAME_MAX:
            raise ValueError(f"SMP frame too large ({len(data)} bytes)")
        seq = self.seq & 0xFF
        self.seq += 1
        fut = asyncio.get_running_loop().create_future()
        self.waiting[seq] = fut
        await self.c.write_gatt_char(SMP_CHAR, data, response=True)
        rsp = await asyncio.wait_for(fut, timeout)
        if rsp.get("rc", 0) != 0:
            raise RuntimeError(f"device answered rc={rsp['rc']} ({rsp})")
        return rsp

    async def image_state(self) -> list:
        return (await self.request(OP_READ, GROUP_IMAGE, ID_IMAGE_STATE, {})).get("images", [])


def show_state(images: list):
    for img in images:
        flags = [k for k in ("active", "confirmed", "pending", "permanent") if img.get(k)]
        print(f"  slot {img.get('slot')}: version {img.get('version')}  {' '.join(flags)}")


async def find_device(address: str | None):
    if address:
        dev = await BleakScanner.find_device_by_address(address, timeout=20)
        if not dev:
            sys.exit(f"No device with address {address} found. Is it switched on and not connected to a phone?")
        return dev
    print("Searching for Traumschreiber devices (10 s) …")
    found = [d for d in await BleakScanner.discover(timeout=10) if (d.name or "").startswith(DEVICE_NAME)]
    if not found:
        sys.exit("No Traumschreiber found. Switch it on and close the app on your phone.")
    if len(found) > 1:
        print("Several devices found, choose one with --address:")
        for d in found:
            print(f"  {d.address}  {d.name}")
        sys.exit(1)
    print(f"Found {found[0].address}")
    return found[0]


async def upload(smp: Smp, image: bytes):
    total, off, t0 = len(image), 0, time.monotonic()
    sha = hashlib.sha256(image).digest()
    while off < total:
        payload = {"off": off, "data": b""}
        if off == 0:
            payload.update({"image": 0, "len": total, "sha": sha})
        room = FRAME_MAX - len(smp.frame(OP_WRITE, GROUP_IMAGE, ID_IMAGE_UPLOAD, payload)) - 3
        payload["data"] = image[off:off + room]
        rsp = await smp.request(OP_WRITE, GROUP_IMAGE, ID_IMAGE_UPLOAD, payload, timeout=30)
        off = rsp.get("off", off + len(payload["data"]))
        rate = off / max(time.monotonic() - t0, 0.1) / 1024
        print(f"\r  {off * 100 // total:3d} %  {off // 1024} / {total // 1024} kB  {rate:.1f} kB/s", end="", flush=True)
    print()


def latest_release() -> tuple[str, bytes]:
    """Newest official release: (tag, signed image bytes)."""
    req = urllib.request.Request(RELEASES, headers={"Accept": "application/vnd.github+json",
                                                    "User-Agent": "traumschreiber-ota"})
    with urllib.request.urlopen(req, timeout=30) as r:
        rel = json.load(r)
    assets = [x for x in rel.get("assets", []) if x["name"].endswith(".signed.bin")]
    if not assets:
        sys.exit(f"Release {rel.get('tag_name')} has no .signed.bin file.")
    print(f"Downloading {assets[0]['name']} ({rel.get('tag_name')}) …")
    with urllib.request.urlopen(assets[0]["browser_download_url"], timeout=120) as r:
        return rel.get("tag_name", "?"), r.read()


async def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image", nargs="?", help="signed image, e.g. build/Firmware/zephyr/zephyr.signed.bin")
    ap.add_argument("--address", help="Bluetooth address (Windows/Linux) or UUID (macOS) of the device")
    ap.add_argument("--info", action="store_true", help="only show the installed images")
    ap.add_argument("--latest", action="store_true", help="install our newest official firmware from GitHub")
    a = ap.parse_args()
    if sum(map(bool, (a.info, a.latest, a.image))) != 1:
        ap.error("give exactly one of: an image file, --latest, --info")
    if a.latest:
        a.image, image = latest_release()
    else:
        image = open(a.image, "rb").read() if a.image else b""
    if image and image[:4] != b"\x3d\xb8\xf3\x96":
        sys.exit("This is not a signed MCUboot image. Use zephyr.signed.bin, not zephyr.bin.")

    dev = await find_device(a.address)
    async with BleakClient(dev, timeout=30) as client:
        smp = Smp(client)
        await smp.start()
        print("Installed:")
        show_state(await smp.image_state())
        if a.info:
            return
        print(f"Uploading {a.image} ({len(image) // 1024} kB). This takes a few minutes; keep the device close.")
        await upload(smp, image)
        state = await smp.image_state()
        slot1 = [i for i in state if i.get("slot") == 1]
        active = [i for i in state if i.get("active")]
        if not slot1:
            sys.exit("Upload finished, but the device shows no image in slot 1.")
        if active and active[0].get("hash") == slot1[0].get("hash"):
            print("This image is already running (same content). Nothing to do.")
            return
        print(f"Marking version {slot1[0].get('version')} for a test boot and restarting …")
        try:
            await smp.request(OP_WRITE, GROUP_IMAGE, ID_IMAGE_STATE, {"hash": slot1[0]["hash"], "confirm": False})
        except RuntimeError as e:
            sys.exit(f"The device refused to test this image ({e}). Nothing was changed.")
        try:
            await smp.request(OP_WRITE, GROUP_OS, ID_OS_RESET, {}, timeout=5)
        except Exception:               # the device may drop the connection before it answers
            pass

    print("Waiting for the device to restart (swapping images takes up to a minute) …")
    await asyncio.sleep(20)
    for _ in range(12):
        dev2 = await BleakScanner.find_device_by_address(dev.address, timeout=10)
        if dev2:
            async with BleakClient(dev2, timeout=30) as client:
                smp = Smp(client)
                await smp.start()
                images = await smp.image_state()
            print("Installed now:")
            show_state(images)
            active = [i for i in images if i.get("active")]
            if active and active[0].get("hash") == slot1[0].get("hash"):
                print("Done: the new firmware is running.")
            else:
                print("The old firmware is still running. The bootloader probably rejected the image: "
                      "is it signed with this device's key?")
            return
    print("The device did not come back within 3 minutes. Switch it off and on and run  --info.")


if __name__ == "__main__":
    asyncio.run(main())
