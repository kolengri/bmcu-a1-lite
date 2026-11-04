# BMCU A1 Lite

Experimental BMCU firmware for a **Bambu Lab A1 on firmware 01.08.01.00**, using
unmodified Bambu Studio. Development hardware is a **BMCU 370C DM Pro** with four
channels and the CH32V203C8T6 target. The seller and exact PCB revision are unknown;
the product name alone does not establish compatibility with other boards.

**Status, 2026-11-04:** the owner completed a physical Unload successfully on
build **260b0401**. Build **260b0403** retains that control/protocol code and adds
normal startup across power cycles plus an experimental ISP-selected USB log path. Build0402
programmed successfully but its first USB export returned zero bytes; it is superseded.
Build0403's installed image was independently verified in full. Its printer
operation has not yet been physically qualified. **Automatic USB log retrieval
is currently unavailable:** ISP acknowledges the marker write, but comparison
returns F500; the corrected tool stops and reports failure. No print,
multicolor endurance, speed increase or general A1 firmware compatibility is claimed.

## Download and use

- [260b0403 firmware](firmware/bmcu-a1-lite-260b0403.bin)
- [SHA256 and source manifest](firmware/manifest.json)
- [Installation, calibration and log retrieval](docs/OPERATING.md)
- [What changed and why Unload failed](docs/RESEARCH.md)
- [Architecture and evidence limits](docs/ARCHITECTURE.md)
- [Build and test instructions](docs/BUILDING.md)
- [Attribution and upstream licensing status](NOTICE.md)

The source is based on [BMCU-C-PJARCZAK](https://github.com/jarczakpawel/BMCU-C-PJARCZAK),
commit `5b8e235576f9d6c30403e737451ef60f97857dac`. This is an independent research
project, not an official Bambu Lab firmware. No printer firmware downgrade or
authentication bypass is included.

## What this firmware does

- Implements the project's AMS Lite protocol path, including requested-slot
  `07/99` replies with that slot's actual encoder coordinate.
- Supports four channels and repeated Load/Unload through the existing controller.
- Preserves durable shared-path ownership, heartbeat lease, sensor checks,
  calibration, jam and movement limits.
- Keeps a bounded, CRC-protected journal without stopping operation after 120 seconds.
- Starts normally after validated printer traffic; USB retrieval does not consume
  or require a one-use boot permit.

This goes beyond changing the advertised AMS type. See the research notes for
protocol, control and diagnostic changes, rejected hypotheses and remaining gaps.
USB collection requires disconnecting the entire printer cable from BMCU.
Simultaneous printer and USB operation is not supported by this release.

