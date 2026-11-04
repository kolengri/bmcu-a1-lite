# A1 compatibility and Unload investigation

## Current physical result

On2026-11-04 the owner reported a completed two-color print on260b0404.
This is the first reported completed multicolor print in this investigation.
It supports operation of the integrated firmware on the development setup;
slot numbers, change count, duration and material were not supplied.
[The exact image and evidence limits](VALIDATION.json) are retained separately.

## Starting point

The project began with BMCU-C-PJARCZAK V11 and an A1 running 01.08.01.00.
The owner retained native Bambu Studio and excluded printer firmware downgrade.
An early registration/heartbeat repair did not resolve printer rejection
0500-409D. The later work implemented and exercised an AMS Lite protocol path;
changing the advertised AMS identity alone was insufficient.

The implementation accumulated protocol compatibility, durable path ownership,
empty setup/calibration, native feed authority, heartbeat recovery, four-slot
handling, sensor diagnostics and USB evidence collection. This is why the result
is a standalone project rather than a small upstream patch. At260b0404, the normalized source/config delta against pinned upstream is
29 files,5,518 insertions and553 deletions, excluding tests, logs, research copies
and additional linker scripts. See [the assessment](UPSTREAM-DELTA.md).

## What the evidence showed

1. Load eventually completed and plastic came out of the nozzle, but Unload
   failed with 1200-8014, sometimes before the printer operated its cutter.
   Bambu's [official error table](https://wiki.bambulab.com/en/hms/error-code)
   describes this code as a failure to find filament location in the toolhead.
   That message alone does not identify a particular wire field or failed sensor.
2. Earlier saved snapshots showed no recognized eligible pull at the instrumented
   parser/controller boundary. Decoder offsets agreed and host tests admitted
   valid addressed pulls. This did not prove that no electrical request arrived:
   CRC rejection, receive loss and incomplete capture remained possible.
3. A separate Load regression exposed encoder sampling/recovery behavior. Its
   repair was checked separately. A successful subsequent Load did not prove
   the Unload problem fixed. Changing PWM or timeout values without an admitted
   pull would not have addressed the demonstrated protocol boundary.
4. Continuous logs removed the old 120-second terminal cutoff. A later tail
   retained addressed `07/99` queries for slots 1, 2 and 3, while every reply
   identified selected slot 2 and its meter. This was a concrete discrepancy.
5. The requested-slot experiment changed addressed `07/99` responses to identify
   the requested slot and that slot's real odometer. The global use flag remains
   global; ownership and motion authority are unchanged. Build 260b0401 combined
   this change with journal-retention repairs. The owner then reported successful
   physical Unload.

The last result supports the integrated 260b0401 change. It is not an isolated
A/B proof that one response field alone caused every earlier failure, and it does
not qualify every slot, printer version or print mode.

## Alternatives investigated

- **Pull admission and parser offsets:** no defect demonstrated at the validated,
  consumed boundary. Earlier receive rejection/loss was not ruled out.
- **Addressed `07/66`:** Tiny transitions to a waiting pre-pull state; older V6
  ignores it. Copying that assignment into a controller whose state also drives
  buffer-release motion would change semantics. No arbitrary reverse motion was added.
- **Meter origin/sign/scale:** encoder changes are wheel coordinates, not measured
  travel at the toolhead. Invented progress, sign flips and per-load rebasing were
  not justified by a stock A1 trace and were not used as this Unload fix.
- **Global feedback:** an earlier physical attempt already weakened the claim
  that global-versus-selected feedback alone explained the failure.
- **Diagnostic blind spots:** frequent polls evicted meaningful events; TX/meter
  pairing and refusal visibility also needed correction. Those are evidence-quality
  fixes and must not be confused with independent proof of printer behavior.

Protocol comparisons used pinned [Tiny](https://github.com/zfkun/ams-lite-tiny/tree/06573b58b8123a219478e14cbe7807e720d82cd8)
and [BMCU V6](https://github.com/jarczakpawel/BMCU-C-PJARCZAK/tree/79d87943e8f681de694205a4ecf7a8570e47a8f2)
sources. Neither provides a stock A1 01.08 wire trace. Public
[Bambu-Bus issue 9](https://github.com/Bambu-Research-Group/Bambu-Bus/issues/9)
also did not supply a complete Unload oracle at the investigation checkpoint.

## Normal operation and the0402 export failure

260b0401 already allowed all four slots and repeated cycles within one session,
but its diagnostic boot permit required USB re-arm for later sessions.
260b0402 removes that usability blocker through passive printer/export selection.
The main loop, setup, controller, sensors, Flash scheduler and wire serializer
remain byte-identical to 260b0401. Motor speed is unchanged.

The0402 host checks passed, but the first real USB export after successful ISP
programming returned zero bytes. Software packet tests had not modeled an
electrical assumption: family schematics put CH340TX and the RS485 receiver
output on the same MCU input. Enabling printer reception may prevent USB command
reception. The exact owner's PCB has not been continuity-tested, so contention
remains a hypothesis. Reset-line behavior and actual adapter baud also remain
possible contributors. Raising the common DE/RE line also enables RS485 output,
so alternating receive windows cannot be described as electrically passive.

Build0403 replaces that uncertain in-band command with ISP-prepared export intent.
It uses the known programming transport at115200, verifies the full padded image,
programs only a dedicated intent row without bulk erase or option-byte changes,
and requests application start with A2 without reopening the serial descriptor. Firmware consumes
intent before enabling export, preserving diagnostic and control storage.
Failure leaves an explicit isolation requirement; reset flags never infer power source.

The non-erasing host sequence reuses the pinned Flasher1.4 packet codec and final
empty A5 flush. The [CH32V003 bootloader investigation](https://github.com/basilhussain/ch32v003-bootloader-docs)
also documents buffering and finalization, but is not proof of every CH32V203 ROM
detail. Models prove host command/address restrictions and firmware erase boundaries;
they cannot prove physical ROM behavior or electrical contention.

No multicolor printing, endurance, stack high-water or throughput result is inferred
from the host tests. The0403 hardware acceptance result is recorded separately.

The first0403 host sequence verified the entire installed image and marker but
received zero bytes after a hardware-reset-only handoff. That sequence had omitted
the pinned flasher's A2 command. An A2-only handoff with BOOT still selected then
returned just the9-byte ISP response. The corrected handoff releases BOOT, checks
that the ROM still responds to A1, then sends A2 and captures its response and application
bytes without another reset or receive-buffer flush. An explicit `--resume-pending`
companion mode verifies an already-present exact marker and sends A2 without any
Flash program command. This separates handoff diagnosis from marker rewriting.

That BOOT-release attempt also captured only an ISP ACK. The new helper had
copied END(0) from the stock flasher without its following hardware reset.
END(0) does not request reset; the corrected command is END(1), wire bytes
`57aba2010001a4`. The [ch32rv-isp source](https://docs.rs/ch32rv-isp/latest/src/ch32rv_isp/lib.rs.html)
encodes the reset request explicitly. The pinned backend also uses END(1) for
option reload, while ordinary flashing pairs END(0) with a hardware pulse.
Tests now assert the independent literal packet, and receipt verification rejects
the former END(0). Neither the firmware image nor the motion behavior changed.

The END(1) attempt also returned only an ISP ACK. A physical comparison then
exposed a host verification defect: the pinned decoder's header `code` was
treated as operation success, ignoring the actual two-byte status payload.
The board returned header0/payloadF500 for a mismatch and FE00 for latched
subsequent failures. The wrapper now requires explicit payload validation;
older success receipts are insufficient. A separate corrected read-only probe
did verify all59,528 padded image bytes with payload0000 and found the marker
prefix erased (E339E339), so installed image integrity was independently restored
as evidence. The timing of any earlier marker consumption remains unknown.

With status checking fixed, another preparation accepted A5 but rejected A6
with F500. A separate fresh comparison also rejected just the16-byte intent
prefix, ruling out padding alone. No destructive workaround was attempted.

This does not establish that all non-erasing ISP writes are impossible:
[wchisp's CLI](https://github.com/ch32-rs/wchisp/blob/main/src/main.rs) exposes
`--no-erase`, but its [writer](https://github.com/ch32-rs/wchisp/blob/main/src/flashing.rs)
starts at offset0. The unresolved boundary is V203 programming at an initial
nonzero offset ED00 after full-image verification. A4/erase is not an acceptable
log-retrieval workaround because it would destroy the journal.

The author's newer CHISP-Flasher at commit
`7e06f97f557819add9987b1e8fffc5d147dc5475` also separates
[header code](https://github.com/jarczakpawel/CHISP-Flasher/blob/7e06f97f557819add9987b1e8fffc5d147dc5475/src/chisp_flasher/protocol/framing.py)
from payload while its
[generic backend](https://github.com/jarczakpawel/CHISP-Flasher/blob/7e06f97f557819add9987b1e8fffc5d147dc5475/src/chisp_flasher/backends/uart_framed_generic.py)
uses the header for flash-command checks. It is not an independent oracle for
the failed marker operation. No accessible exact factory-ROM dump was obtained;
V003 behavior is not substituted for V203 evidence.

## 260b0404: setup recovery and diagnostics affecting intake

The next owner report was red indicators and no intake after visible calibration.
Source inspection showed that the green calibration cue preceded empty settling,
direction detection and the final ownership commit. Filament presence during
those stages set a sticky setup fault. Build0404 changes the intermediate cue to
yellow and implements a stopped, recoverable insertion pause. Sensor validity,
clock/storage faults and the overall setup bound remain enforced.

An actual-body regression then exposed a separate mechanism: local intake PWM
was classified as a pull event, and diagnostic Flash reservation made the next
local-intake tick cancel itself. Restricting that event alone was insufficient:
legitimate RX-loss events reproduced the same scheduling conflict. Build0404
also defers diagnostic submissions during local intake while retaining genuine
control-storage and native motion protections.

The final binary passed133 production-body host scenarios and13 profile guards;
source-archive and patch replays produced the same binary. Independent firmware
and host-adapter reviews accepted the change before installation. All1,079
padded image blocks were physically compared successfully. The later owner report
of a successful two-color print is physical acceptance evidence for this integrated
image, not proof that every earlier failure had one cause or an isolated trial of
the early-insertion boundary.

The installation also exposed a separate host-flasher assumption: after a full
erase, the board accepted a tail comparison against repeated E339 and rejected FF.
The corrected adapter changes only that post-erase comparison and still requires
header0 plus payload0000 for program/erase/verify operations. This successful
full-erase installation does not solve the non-erasing USB export marker failure.
