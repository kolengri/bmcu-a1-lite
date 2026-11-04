"""Build-bound, non-erasing ISP selection of isolated USB journal export.

This is a companion to the pinned flasher, not its full-flash algorithm. It never
issues erase/option-write commands. One exclusive descriptor spans verification,
marker programming, normal reset, and raw capture. No automatic retries.
"""
from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import signal
import time

from .core import FlasherError, FlasherService, atomic_json, devices, image_bytes, preflight
from .worker import load_backend, validate_status_payload

BUILD = "260b0403"
PINNED_IMAGE_SHA256 = "c0fae2a3df3ab9447292f849336d5335bb276852c5407c78be9ddad258491a9a"
MARKER_ADDRESS = 0xED00
MARKER = bytes.fromhex("4246493303040b260100010062b34213")
MARKER_ROW = MARKER + b"\xff" * 40
CHUNK = 56
# Owner-board mapping established by prior successful pinned-flasher jobs.
BOOT_IS_DTR, BOOT_ASSERT, RESET_ASSERT = True, False, False


def require(condition, message):
    if not condition:
        raise FlasherError("EXPORT_FAILED", message)


def save_bytes(path, data):
    with path.open("xb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())


def sync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def save_receipt(output, receipt):
    atomic_json(output / "isp-export.json", receipt)
    sync_directory(output)


def command(isp, packet, cmd, timeout=2.0):
    """Check short writes; reject every command outside this operation's scope."""
    require(cmd in (0xA1, 0xA3, 0xA5, 0xA6, 0xA7), "Forbidden ISP command")
    require(packet[2] == cmd, "ISP packet/command mismatch")
    if cmd == 0xA5:
        address = int.from_bytes(packet[5:9], "little")
        length = len(packet) - 11
        require((address, length) in ((MARKER_ADDRESS, CHUNK), (MARKER_ADDRESS + CHUNK, 0)),
                "ISP programming outside marker row/flush is forbidden")
    require(isp.ser.write(packet) == len(packet), "Short ISP write; no retry")
    code, data = isp.recv(cmd, timeout)
    validate_status_payload(cmd, code, data)
    require(code == 0, "ISP %02x returned %02x; no retry" % (cmd, code))
    return data


def prepare_and_capture(module, isp, firmware, output, seconds, max_bytes, receipt, resume_pending=False):
    """Caller owns the exclusive open descriptor and cleanup, including on failure."""
    def checkpoint(stage):
        receipt["stage"] = stage
        save_receipt(output, receipt)

    padded = firmware + b"\xff" * (-len(firmware) % CHUNK)
    require(0 < len(padded) <= MARKER_ADDRESS, "Image overlaps export-intent page")
    checkpoint("entering_isp")
    module.set_lines(isp, BOOT_IS_DTR, BOOT_ASSERT, RESET_ASSERT)
    time.sleep(0.02)
    module.pulse_reset(isp, BOOT_IS_DTR, RESET_ASSERT)
    time.sleep(0.06)
    isp.flush()
    identity = command(isp, module.build_identify(0x31, 0x19), 0xA1)
    require(identity[:2] == b"\x31\x19", "ISP identity is not CH32V203C8T6")
    cfg = command(isp, module.build_read_cfg(module.BMCU_CFG_MASK), 0xA7)
    require(len(cfg) >= 22, "Incomplete ISP configuration/UID")
    require(cfg[10:14] == b"\xff" * 4, "Write protection active; no option changes allowed")
    seed = b"\0" * 30
    response = command(isp, module.build_isp_key(seed), 0xA3)
    require(len(response) >= 1, "Missing ISP key checksum")
    candidates = (module.calc_xor_key_uid(cfg[-8:], 0x31),
                  module.calc_xor_key_seed(seed, cfg[2], 0x31))
    key = next((key for key in candidates if sum(key) & 255 == response[0]), None)
    require(key is not None, "ISP key checksum mismatch")
    checkpoint("verifying_installed_image")
    for address in range(0, len(padded), CHUNK):
        command(isp, module.build_verify(address, 0, module.xor_crypt(padded[address:address + CHUNK], key)), 0xA6)
    receipt["verified_image_bytes"] = len(padded)
    refreshed = command(isp, module.build_isp_key(seed), 0xA3)
    require(refreshed[:1] == response[:1], "ISP key changed before marker programming")
    if not resume_pending:
        # Persist intent BEFORE the only two allowed program commands. Failure
        # can leave a valid marker pending; isolation must remain in force.
        checkpoint("marker_write_may_be_pending")
        command(isp, module.build_program(MARKER_ADDRESS, 0, module.xor_crypt(MARKER_ROW, key)), 0xA5, 5)
        command(isp, module.build_program(MARKER_ADDRESS + CHUNK, 0, b""), 0xA5, 5)
        refreshed = command(isp, module.build_isp_key(seed), 0xA3)
        require(refreshed[:1] == response[:1], "ISP key changed after marker programming")
    else:
        checkpoint("verifying_existing_marker")
    command(isp, module.build_verify(MARKER_ADDRESS, 0, module.xor_crypt(MARKER_ROW, key)), 0xA6)
    receipt["marker_verified"] = True
    checkpoint("starting_export")
    # A2 can reset through the boot selection pins. Release BOOT first, then
    # prove the ROM still responds (a wrong modem mapping could assert reset).
    module.set_lines(isp, BOOT_IS_DTR, not BOOT_ASSERT, RESET_ASSERT)
    identity = command(isp, module.build_identify(0x31, 0x19), 0xA1)
    require(identity[:2] == b"\x31\x19", "ISP identity lost after releasing BOOT")
    receipt["boot_released_before_start"] = True
    # Save the A2 response and application bytes together;
    # waiting for only an ISP ACK could consume/drop the first application frame.
    # No additional reset, line change, or input flush occurs after this write.
    isp.flush()
    # END(0) only acknowledges completion. The stock full flasher follows it
    # with a hardware reset; this same-descriptor path needs END(1) to reboot.
    start = module.build_isp_end(1)
    receipt["start_command_hex"] = start.hex()
    checkpoint("application_start_may_be_pending")
    require(isp.ser.write(start) == len(start), "Short A2 application-start write; no retry")
    checkpoint("capturing")
    started = time.monotonic()
    total = 0
    with (output / "raw.bin").open("xb") as stream:
        try:
            while time.monotonic() - started < seconds and total < max_bytes:
                block = isp.ser.read(min(4096, max_bytes - total))
                if block:
                    stream.write(block)
                    total += len(block)
                else:
                    time.sleep(0.002)
        finally:
            stream.flush()
            os.fsync(stream.fileno())
            receipt["raw_bytes"] = total
    receipt["raw_sha256"] = hashlib.sha256((output / "raw.bin").read_bytes()).hexdigest()
    receipt["capture_finished"] = True
    checkpoint("captured_unvalidated")
    require(total > 0, "No application bytes received; retain USB isolation; no retry")


def run(args):
    require(args.isolated_usb_only, "Explicit USB-only isolation is required")
    require(1 <= args.seconds <= 60 and 536 <= args.max_bytes <= 1048576, "Invalid capture bounds")
    firmware = image_bytes(Path(args.firmware))
    require(args.sha256 == PINNED_IMAGE_SHA256, "Only the reviewed 0403 image is eligible")
    require(hashlib.sha256(firmware).hexdigest() == args.sha256, "Firmware checksum mismatch")
    require(0 < ((len(firmware) + 55) // 56) * 56 <= MARKER_ADDRESS, "Image exceeds 0403 boundary")
    output = Path(args.output).absolute()
    require(not any(p.is_symlink() for p in (output,) + tuple(output.parents)), "Unsafe output path")
    output.mkdir(mode=0o700)
    sync_directory(output.parent)
    receipt = dict(format=2, build=BUILD, isolated_usb_only=True, firmware_sha256=args.sha256,
                   status_payload_checked=True,
                   marker_address=MARKER_ADDRESS, marker_hex=MARKER_ROW.hex(),
                   verified_image_bytes=0, marker_verified=False, capture_finished=False,
                   raw_bytes=0, port_closed=False, stage="prepared", success=False,
                   baud=115200, parity="none", serial_openings=0,
                   operation="resume_pending" if args.resume_pending else "prepare")
    save_bytes(output / "marker.bin", MARKER_ROW)
    save_receipt(output, receipt)
    isp = None
    lock = None
    try:
        module = load_backend()
        service = FlasherService(Path.cwd())
        lock = service._lock()
        require(not preflight(args.port), "Flasher GUI is running")
        require(any(d["port"] == args.port for d in devices()), "Port is not a WCH 1a86:7523 device")
        # Attach an exclusively opened pyserial object to the unmodified parser.
        isp = module.WchIsp(args.port, 115200, module.serial.PARITY_NONE, False)
        isp.ser = module.serial.Serial(args.port, baudrate=115200, timeout=0, write_timeout=1,
                                       exclusive=True, rtscts=False, dsrdtr=False)
        receipt["serial_openings"] = 1
        import fcntl
        import termios
        fcntl.ioctl(isp.ser.fileno(), termios.TIOCEXCL)
        prepare_and_capture(module, isp, firmware, output, args.seconds, args.max_bytes, receipt, args.resume_pending)
        receipt["success"] = True  # Raw capture only; offline decoder owns acceptance.
    except BaseException as exc:
        receipt["error"] = str(exc) or type(exc).__name__
    finally:
        try:
            if isp is not None and isp.ser is not None:
                # Pinned WchIsp.close() rewrites DTR/RTS. Preserve the established
                # lines; ordinary OS close behavior remains adapter-dependent.
                isp.ser.close()
            receipt["port_closed"] = True
        except BaseException as exc:
            receipt["success"] = False
            receipt["close_error"] = str(exc)
        finally:
            if lock is not None:
                os.close(lock)
            save_receipt(output, receipt)
    return 0 if receipt["success"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware", required=True)
    parser.add_argument("--sha256", required=True)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--isolated-usb-only", action="store_true")
    parser.add_argument("--resume-pending", action="store_true",
                        help="Verify an existing exact marker and request export without programming any Flash")
    parser.add_argument("--seconds", type=int, default=10)
    parser.add_argument("--max-bytes", type=int, default=1048576)
    args = parser.parse_args()
    def interrupt(signum, frame):
        raise KeyboardInterrupt("Interrupted; no automatic retry")
    signal.signal(signal.SIGTERM, interrupt)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
