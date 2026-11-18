# keys/

Put your signing key here as `signing_key.pem`. The build (`Firmware/sysbuild.conf`) reads it from this
place. Everything in this folder except this README is ignored by git.

- **Maker edition:** use the key from your microSD card, `Firmware/TS2607-<code>_signing_key.pem`.
- **Your own hardware or a cable-flashed device:** create a key with
  `python3 <ncs>/bootloader/mcuboot/scripts/imgtool.py keygen -k keys/signing_key.pem -t ecdsa-p256`
  and flash bootloader and app together by cable (`west flash`).
