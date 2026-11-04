# Installation and operation

## Scope

260b0404 completed an owner-reported two-color print on the development setup.
Other board revisions, printer versions, all-four-slot endurance and throughput
are not qualified. Keep a known firmware image for recovery.

## Flash and first setup

1. Remove filament so all four paths are empty. Turn the printer off and fully
   disconnect its cable from BMCU, including power. Connect BMCU only to USB.
2. Verify `firmware/bmcu-a1-lite-260b0404.bin` against `firmware/manifest.json`.
   Install with the board's established WCH-compatible [BMCU Flasher](https://github.com/jarczakpawel/BMCU-Flasher). The tested
   flashing workflow erases all 64 KiB, including calibration, ownership and logs;
   this release does not provide an in-place NVM migration.
3. Close the flasher and disconnect USB. With the printer off, reconnect its
   normal BMCU cable, then power on the printer. No USB re-arm command is needed.
4. Keep all four paths empty during initial setup. In the blue setup window,
   press one buffer for the one-second acceptance gesture and fully release it
   when accepted. Follow the existing calibration prompts: blue inward, red
   outward, then release/center the buffers. Direction testing may rotate the
   empty motors. The existing setup checks must finish before operation is admitted.
5. Once setup finishes, insert filament and use the printer's normal Load/Unload
   controls. If filament is inserted during setup, remove it and leave the buffers
   neutral: the firmware waits for1.5 seconds of stable empty inputs and continues.
   The calibration-only cue is yellow; direction testing and storage preparation
   may still follow. Invalid sensor/storage faults remain faults.

Valid retained calibration and ownership skip empty recalibration on later boots.
Retained ownership never synthesizes a movement command. Fresh printer commands
are still required. An incomplete or invalid control-storage record can inhibit
operation; do not force it clear while filament remains in the path.

## USB logs: known limitation

Automatic USB log collection is unavailable. The firmware retains a bounded
journal, but the development board rejects verification of the non-erasing ISP
export marker. Erasing/reflashing would destroy the evidence and is not a remedy.
The bundled export helper and wrapper are historical260b0403 tools; they are not
bound to260b0404. Do not use them against this release or bypass image checks.

The successful two-color print and strict full-image installation verification
do not establish successful marker programming or export. The detailed research
notes distinguish these outcomes. Stock USB and the printer bus share a UART;
this release does not support simultaneous USB streaming during printing.

## Flashing transport note

The development installation verified all1,079 padded blocks with actual ISP
status payload0000. Its pinned host adapter needed a narrow correction to the
post-erase tail comparison: this board compared as repeated E339, not FF. The
correction did not relax ordinary firmware comparisons. A flasher that stops
at this check after erasing may leave no usable application; retain its error
and use a compatible verified programmer rather than ignoring verification.
The old bundled export-only helper is not the full flashing workflow.

## Physical acceptance and speed measurements

First verify one complete Load/Unload, another printer power cycle, and another
complete cycle. Then exercise each intended slot before a small multicolor print.
Keep spool, material, PTFE routing and slicer settings constant when comparing time.
Record Load, Unload and total color-change durations separately. Printer-side
heating, cutting and purging are part of the last measurement, not BMCU motor speed.
The release does not increase motor speed. Host virtual-time tests are not physical
throughput measurements; the rolling journal may not retain both endpoints.

