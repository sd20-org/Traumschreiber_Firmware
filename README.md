# Traumschreiber firmware

Firmware of the [Traumschreiber](https://github.com/sd20-org/traumschreiber) TS2607, an open, non-profit
forehead EEG for sleep and dream research. It runs on the nRF52840 (Zephyr RTOS / nRF Connect SDK v3.2.2)
with the MCUboot bootloader and

- reads 8 ExG channels from the ADS131M08 at 250 samples/s (24 bit),
- writes them to the microSD card as BDF files (folder `DATA/`),
- streams them over Bluetooth Low Energy to the [sd20 App](https://play.google.com/store/apps/details?id=org.sd20.sd20app)
  or any other client (see [Traumschreiber_Python_BLE-Live-Plot](https://github.com/sd20-org/Traumschreiber_Python_BLE-Live-Plot)),
- plays light and sound cues and can be updated over the air (MCUmgr/SMP).

This repository holds the source of every released version (see Releases / tags). It contains no keys.

## Build

Install the nRF Connect SDK **v3.2.2** (easiest: "nRF Connect for VS Code"), put a signing key into
`keys/signing_key.pem` (see [keys/README.md](keys/README.md)), then in an nRF Connect terminal:

```bash
cd Firmware
west build --pristine -b Traumschreiber2603/nrf52840 -- \
  -DBOARD_ROOT="$PWD/.." -DDTC_OVERLAY_FILE=boards/Traumschreiber2603.overlay
# result: build/Firmware/zephyr/zephyr.signed.bin  (over the air)
#         build/merged.hex                         (bootloader + app, by cable)
```

## Install over Bluetooth

```bash
pip install bleak cbor2
python tools/ota_update.py --info
python tools/ota_update.py --latest                                   # our newest official firmware
python tools/ota_update.py Firmware/build/Firmware/zephyr/zephyr.signed.bin   # your own build
```

Close the app on your phone first; the device keeps only one Bluetooth connection. The script uploads the
image, marks it for a test boot and restarts the device. The firmware confirms itself after starting; if
it does not start, the bootloader returns to the previous version at the next restart.

## Which devices accept your build

MCUboot only starts firmware signed with the key its bootloader was built with.

- **Maker edition** ("Bastelversion"): the bootloader accepts our signed images **and** images signed with
  a key of its own; you get that private key on the microSD card. Build with it and install over
  Bluetooth as above. `ota_update.py --latest` installs our newest release and takes you back to our
  firmware at any time.
- **Standard devices**: the bootloader only accepts images signed with our production key, which is not
  part of this repository and never will be. Your build is uploaded but not started. To run your own
  firmware on such a device, flash bootloader and app by cable (SWD, e.g. J-Link) with a key of your own;
  after that, our official updates no longer install until our bootloader is flashed back.

This protects every user's device from firmware nobody checked: the device has no Bluetooth pairing,
so a key is the only thing that decides what it runs. Keep your key private.

With your own firmware, the CE marking and declaration of conformity no longer apply to the device; you
are responsible for its behaviour. Keep Bluetooth LE at 2.4 GHz with at most +4 dBm (as in our firmware),
and keep the charging lock (the device switches off while USB is plugged in).

## Contributing

We are happy about pull requests, bug reports and ideas: [ts-repo@sd20.org](mailto:ts-repo@sd20.org)

## License

The firmware source is licensed under the MIT License, see [LICENSE](LICENSE). Third-party parts (Zephyr
RTOS, nRF Connect SDK, MCUboot) keep their own licenses. The hardware design is licensed separately in the
[traumschreiber](https://github.com/sd20-org/traumschreiber) repository.
