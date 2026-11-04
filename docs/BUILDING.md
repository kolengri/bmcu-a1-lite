# Build and verification

## Firmware

Use PlatformIO with the community CH32V platform. The release was built with
platform commit `7c674802c9041d215c8cb285a9e787a4aa0b487b`,
`framework-wch-noneos-sdk 2.30000.0+sha.4fcf972` and
`toolchain-riscv 1.120200.220829+sha.e6360e0`.
The platform is [Community-PIO-CH32V/platform-ch32v](https://github.com/Community-PIO-CH32V/platform-ch32v/tree/7c674802c9041d215c8cb285a9e787a4aa0b487b).
Install that platform revision into your PlatformIO environment before building:

```sh
python3 -m platformio platform install https://github.com/Community-PIO-CH32V/platform-ch32v.git#7c674802c9041d215c8cb285a9e787a4aa0b487b
python3 -m platformio run -e USB_ISP_EXPORT
shasum -a 256 .pio/build/USB_ISP_EXPORT/firmware.bin
```

The public default selects `USB_ISP_EXPORT`. Historical environments remain for
source comparison; they are not interchangeable release images. Do not select an
ordinary or earlier diagnostic profile when intending to build260b0403.
`isp-export.ld` reserves intent, journal and original control NVM.

Compare a public source rebuild against the delivered BIN hash. Fresh dependency resolution is not a complete
lock of every upstream SDK/tool dependency. Compare the resulting hash with
`firmware/manifest.json`; a different toolchain is not covered by that identity.
No compiler, SDK, private package cache or ELF with local debug paths is bundled.

## Firmware host tests

Python 3.9+ and C/C++ compilers with AddressSanitizer and UndefinedBehaviorSanitizer
are needed. From the repository root:

```sh
python3 -B validation/firmware/tests/run_final.py
```

This runs105 cases against production source bodies with modeled hardware:7 intent,17
working boot, 10 journal, 5 retention, 46 requested-slot reply and 20 feature-OFF
encoder regressions. It includes463 modeled interruption prefixes. Outputs are
under ignored `validation/firmware/logs/`; receipts hash source and fixtures.
The six historical source files needed for serializer comparisons are included
as four small local archives, with a manifest. No private checkout is required.

The working-mode cases replace obsolete one-use boot/rearm assumptions. Other
historical fixture files remain because the harness records their identities;
their presence does not mean every historical scenario applies to this profile.

## Collector and wrapper

The bundled collector is a universal macOS 11+ binary for arm64 and x86_64.
It accepts regular input files only and rejects serial-device and live-operation
arguments. A macOS SDK is used by the bundled universal build command:

```sh
python3 -B validation/firmware/tests/build_collector.py
python3 -B validation/firmware/tests/check_offline_collector.py
python3 -B validation/firmware/tests/check_retention_collector.py
python3 -m unittest discover -s tools/bmcu-usb-logs/tests -v
cd tools/bmcu-flasher-mcp
.venv/bin/python -m unittest discover -s tests -v
```

Decoder checks use synthetic frames and never open a physical serial port.
Host transport tests use the actual pinned packet codec with a device model,
exercise full-image verification before marker writes, failure persistence and
one-opening capture. They cannot qualify the ROM bootloader, electrical wiring
or printer bus timing. See receipt counts for the exact tested source state.

Rebuilding the collector with a different compiler can change its hash. The
wrapper deliberately refuses an executable that differs from its pinned
`profiles.json` identity; review and update that identity for a locally modified
build instead of disabling verification.
