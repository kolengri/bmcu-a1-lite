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
the dedicated ED00 page. Only the isolated host ISP companion creates it after
verifying the installed image. Firmware explicitly quiesces inherited UART/DMA
state, consumes and verifies that page, then exports both diagnostic pages at
115200/8N1. Consumption failure keeps motors and all UART transmission inhibited.
Absent, malformed, torn or wrong-build intent retains passive printer startup.
The previous BFX2 in-band selector is compiled out. Export never starts setup or
motion. This is a mode selector, not authentication or power-source detection.

The control path preserves the 260b0401 implementation. Durable ownership prevents
another slot taking an occupied shared path. Heartbeat expiry inhibits movement;
sensor faults, jam limits, send length and interrupted pull recovery remain active.
Recoverable encoder sampling errors can resume only after the existing valid-input
checks; the firmware does not fabricate encoder progress.

## Storage

The CH32V203 target has 64 KiB Flash and 20 KiB RAM. Firmware is bounded below
`0xED00`; the intent page occupies `0xED00..0xEDFF`, and two256-byte diagnostic
pages occupy `0xEE00..0xEFFF`. The original control NVM region remains separate.
The260b0403 BIN is59,492 bytes;56-byte flashing blocks extend to59,528, leaving
1,144 bytes before intent storage. The boot-only intent eraser grants no runtime
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

