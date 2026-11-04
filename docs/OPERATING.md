# Installation and operation

## Scope

260b0403 is an experimental build for the development hardware in the README.
It has not yet had a physical working-mode acceptance run. Keep the previous
known image and collect logs before replacing a firmware that just failed.

## Flash and first setup

1. Remove filament so all four paths are empty. Turn the printer off and fully
   disconnect its cable from BMCU, including power. Connect BMCU only to USB.
2. Verify `firmware/bmcu-a1-lite-260b0403.bin` against `firmware/manifest.json`.
   Install with the board's established WCH-compatible BMCU flasher. The tested
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
   controls. Do not insert filament during calibration or direction testing.

Valid retained calibration and ownership skip empty recalibration on later boots.
Retained ownership never synthesizes a movement command. Fresh printer commands
are still required. An incomplete or invalid control-storage record can inhibit
operation; do not force it clear while filament remains in the path.

## Experimental USB log path on macOS

**Known blocker on the development board:** the ISP marker write acknowledges
success but does not pass the actual A6 comparison, even for the16-byte intent
prefix. The corrected tool aborts before starting the application. The following
command is retained for research and is not a working routine log-retrieval
workflow. Do not erase/reflash a failed printer session to work around this.

After a failure or completed test, turn off the printer, disconnect its complete
cable from BMCU, and connect only USB. Do not reflash before collection. Close
Flasher. Prepare the companion environment once, then collect from the repository:

```sh
python3 -m venv tools/bmcu-flasher-mcp/.venv
tools/bmcu-flasher-mcp/.venv/bin/python -m pip install -r tools/bmcu-flasher-mcp/requirements.txt
tools/bmcu-usb-logs/bmcu-usb collect --build 260b0403 --isolated-usb-only \
  --note 'Printer cable disconnected; USB only; Flasher closed'
```

The wrapper checks the USB descriptor and port ownership without opening the
serial device during preflight. Its flasher companion opens the port once,
enters ISP using the development board's known DTR/RTS mapping, verifies every
padded firmware block, and writes only the dedicated export-intent row at ED00.
It never bulk-erases Flash or changes option bytes. The ROM's A2 command with
reset parameter 1 requests
inhibited export on the same 115200/8N1 descriptor, with no subsequent reset or
modem-line change. The ISP response and application data are saved together. The C decoder
then processes the saved raw file offline; its metadata does not claim live timing.
The bundled decoder is universal macOS arm64/x86_64; Python3.9+, pyserial3.5 and
macOS tools are required. This fixed AutoDI mapping is not qualified for other boards.
There is no automatic retry, reconnect or re-arm. Save the entire output directory
on failure. **Keep USB-only isolation until valid0403 export confirms marker
consumption:** an interrupted operation can leave export intent pending.

The output includes raw bytes, decoded events, exact marker, ISP result and integrity
bindings. A blank journal before the first printer session is a valid transport
with no event evidence. The journal is a rolling tail, not a complete test trace.
Power-off is not a final flush; there is no terminal green-save wait in this mode.

Offline validation of a saved directory never opens a port:

```sh
tools/bmcu-usb-logs/bmcu-usb verify artifacts/YOUR_CAPTURE --build 260b0403
```

After collection, disconnect USB before reconnecting the printer cable. The next
normal printer boot needs no host ACK. Stock USB and the printer bus share a UART;
this firmware does not offer simultaneous USB streaming during printing.

## Physical acceptance and speed measurements

First verify one complete Load/Unload, another printer power cycle, and another
complete cycle. Then exercise each intended slot before a small multicolor print.
Keep spool, material, PTFE routing and slicer settings constant when comparing time.
Record Load, Unload and total color-change durations separately. Printer-side
heating, cutting and purging are part of the last measurement, not BMCU motor speed.
The release does not increase motor speed. Host virtual-time tests are not physical
throughput measurements; the rolling journal may not retain both endpoints.

