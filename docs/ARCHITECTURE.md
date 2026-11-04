# Runtime and diagnostic boundaries

At reset, motor outputs and UART transmission are inhibited. A receive-only
bootstrap listens at the printer bus format, 1250000 baud with even parity.
It reuses the production framing and CRC classifier. Silence, unknown commands,
wrong addresses/builds, AHUB traffic and malformed frames do not start setup.

Recognized addressed printer traffic or a valid heartbeat selects normal startup.
The selecting packet is discarded; it is not dispatched as a motor command.
Normal setup validates persistent calibration, direction and shared-path ownership.
Fresh eligible commands then pass the existing lease, sensor and PWM checks.

Before listening, firmware checks a build-bound, CRC-protected export marker in
the dedicated ED00 page. The intended isolated ISP flow creates it after verifying the installed image;
physical marker programming is currently blocked, and the bundled host profile
is pinned to0403. This firmware branch alone does not provide working USB export. Firmware explicitly quiesces inherited UART/DMA
state, consumes and verifies that page, then exports both diagnostic pages at
115200/8N1. Consumption failure keeps motors and all UART transmission inhibited.
Absent, malformed, torn or wrong-build intent retains passive printer startup.
The previous BFX2 in-band selector is compiled out. Export never starts setup or
motion. This is a mode selector, not authentication or power-source detection.

The native protocol path retains the260b0401 requested-slot repair. Build0404
adds recoverable setup pauses and defers diagnostics during local intake. Durable ownership prevents
another slot taking an occupied shared path. Heartbeat expiry inhibits movement;
sensor faults, jam limits, send length and interrupted pull recovery remain active.
Recoverable encoder sampling errors can resume only after the existing valid-input
checks; the firmware does not fabricate encoder progress.

## Storage

The CH32V203 target has 64 KiB Flash and 20 KiB RAM. Firmware is bounded below
`0xED00`; the intent page occupies `0xED00..0xEDFF`, and two256-byte diagnostic
pages occupy `0xEE00..0xEFFF`. The original control NVM region remains separate.
The260b0404 BIN is60,372 bytes;56-byte flashing blocks extend to60,424, leaving
248 bytes before intent storage. The boot-only intent eraser grants no runtime
journal or setup writer permission to address this page.
Static RAM plus the reserved 2,048-byte stack is 17,888 bytes. Stack high-water
and dynamic timing have not been physically measured.

The journal stores up to 28 committed records plus a bounded queue of eight.
CRC-last commits reject incomplete records. Normal boot prepares the opposite
page while preserving the newest committed page. Runtime rotation overwrites
older events. Semantic filtering suppresses unchanged polling traffic; paired
response/meter records require matching session, generation, timestamp and channel.
Loss counters expose dropped queued events. These properties do not make it a
full bus trace, a guaranteed quiet Flash window or a printer-error timestamp.

Both header and record checks bind the exact build. There is no cross-build log
migration. Invalid diagnostic storage disables logging without granting control
authority. Invalid control storage remains subject to the original strict readers.


## Setup recovery and local intake

Filament-key presence during setup stops the bridges and pauses setup. Removal
must be followed by1.5 seconds of fresh neutral stable input. Interrupted gesture,
calibration sampling and direction sampling restart their local attempt; direction
samples from interrupted movement cannot validate a direction result. Validated
Flash records finish through the existing bounded writer. The total five-minute
setup bound and sensor/storage/clock faults still apply.

The key threshold is the recovery trigger. Buffer movement before that threshold
can still meet an existing centering or stability fault; this is not a blanket
suppression of sensor faults. No physical early-insertion trial is claimed by the
successful two-color print.

Local intake previously generated a false pull-PWM journal event. Reserving Flash
then made the next local control tick ineligible and cancelled intake. Build0404
restricts pull-PWM records to pull-back and defers new diagnostic Flash work while
local intake/PWM is active with no selected or owned shared path. Legitimate events
remain queued for an idle window. Native Load/Unload/on-use logging and genuine
control-storage busy checks retain their behavior.
