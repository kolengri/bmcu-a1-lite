# BMCU A1 Lite

BMCU firmware for **Bambu Lab A1 on firmware 01.08.01.00**, using unmodified
Bambu Studio. Development hardware: **BMCU 370C DM Pro**, four channels,
CH32V203C8T6. The exact PCB revision is unknown.

**Build 260b0404: the owner successfully printed a two-color part on 2026-11-04.**
This is one reported physical print on the development setup, not a qualification
of every board, slot combination or printer version. The same installed binary
is provided below. [Evidence and limits](docs/VALIDATION.json).

## Download and install

- [Release and downloads](https://github.com/kolengri/bmcu-a1-lite/releases/tag/260b0404)
- [Firmware BIN](firmware/bmcu-a1-lite-260b0404.bin)
- [Exact hashes and source manifest](firmware/manifest.json)
- [Installation and calibration](docs/OPERATING.md)
- [Build and host tests](docs/BUILDING.md)

BIN SHA256:

```text
487c6c9f07f93a5682f81b9a1f838ee1f6106c1b19d8ab09875d07543b7efdb5
```

Flash with all four paths empty, the complete printer cable disconnected, and
USB as the only connection. Flashing erases calibration and saved logs. Disconnect
USB before reconnecting BMCU to the powered-off printer and completing setup.

**Known limitation:** automatic USB log retrieval remains unavailable. The older
bundled export tools are research tools pinned to260b0403, not a supported log
collector for260b0404. Simultaneous printer and USB operation is not supported.

## What changed

- Implements an AMS Lite protocol path; this is more than changing the advertised
  AMS identity. Addressed `07/99` replies identify the requested slot and its real
  encoder coordinate, fixing the observed selected-slot response discrepancy.
- Supports four channels in the controller and repeated Load/Unload. The current
  physical evidence covers a two-color print; all-four-slot endurance is untested.
- Keeps path ownership, heartbeat lease, sensor validity, calibration and jam limits.
- Starts normally on printer traffic without a diagnostic re-arm for every session.
- Recovers from early filament insertion during setup: motion pauses, and removal
  followed by stable empty inputs lets setup continue. Calibration-only completion
  is yellow because direction detection and storage preparation still follow.
- Prevents diagnostic Flash writes from cancelling local filament intake.
- Keeps a bounded continuous diagnostic journal without the old120-second stop.

Read [the research findings](docs/RESEARCH.md), [runtime and storage design](docs/ARCHITECTURE.md)
and [the change-size assessment](docs/UPSTREAM-DELTA.md). Motor speed is unchanged;
there is no measured speed improvement.

Based on [BMCU-C-PJARCZAK](https://github.com/jarczakpawel/BMCU-C-PJARCZAK),
commit `5b8e235576f9d6c30403e737451ef60f97857dac`. This is an independent project,
not an official Bambu Lab firmware. No printer downgrade or authentication bypass
is included. See [attribution and upstream licensing status](NOTICE.md).
