"""One-opening USB capture orchestration and read-only saved-byte verification.

Legacy frozen C collectors own serial access and ACK. Build0403 uses the pinned
flasher companion for ISP entry/raw capture and the C decoder offline. This
orchestrator never opens a serial device. Python 3.9 standard library only.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import plistlib
import re
import signal
import stat
import struct
import subprocess
import sys
import tempfile
import time
import uuid
import zlib

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
JOURNAL_BUILDS = frozenset(("260b0102", "260b0401", "260b0402", "260b0403"))
# Exact frozen boot-export request: short-frame CRC8, BFX2, build, bus CRC16.
# Working firmware uses this command only to select inhibited log export.
BOOT_EXPORT_REQUESTS = {"260b0402": bytes.fromhex("3dc50f83fe4246583202040b26a283")}
ISP_EXPORT_BUILDS = frozenset(("260b0403",))
WORKING_BUILDS = frozenset(BOOT_EXPORT_REQUESTS) | ISP_EXPORT_BUILDS
ISP_MARKER = bytes.fromhex("4246493303040b260100010062b34213") + b"\xff" * 40
IOREG = ["/usr/sbin/ioreg", "-p", "IOService", "-r", "-c", "IOUSBHostDevice", "-a", "-l"]
BOUNDARY = "After a confirmed ACK, the next power cycle must be the intended A1-only test, not another USB boot."
WORKING_BOUNDARY = "Working firmware needs no re-arm. Disconnect USB before reconnecting the printer for normal operation."
ISP_BOUNDARY = ("Retain USB-only isolation until a valid0403 export confirms marker consumption. "
                "An interrupted preparation can leave export intent pending. After success, "
                "disconnect USB before reconnecting the printer; no re-arm is required.")
DISCLOSURES = [
    "USB/A1/24V isolation is an owner assertion; inventory cannot establish it.",
    "Opening/closing CH340 may cause unqualified driver AutoDI transitions.",
    "A build header is not full installed-BIN readback.",
    "Raw ACK response bytes are not retained by the existing collector; confirmation is its saved claim.",
    "Killed-child RAM buffers may not have reached the saved files.",
    "No automatic retry or printer traffic. Build0403 explicitly enters ISP and requests application start with A2; only its intent page is programmed.",
]


class WorkflowError(Exception):
    pass


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def crc(data):
    return zlib.crc32(data) & 0xffffffff


def u32(data, offset):
    return struct.unpack_from("<I", data, offset)[0]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_bytes(path, data):
    with path.open("xb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())


def write_json(path, value):
    write_bytes(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode())


def read_json(path):
    def object_pairs(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("Duplicate JSON key: " + key)
            result[key] = value
        return result
    try:
        result = json.loads(path.read_text(), object_pairs_hook=object_pairs)
        require(isinstance(result, dict), "JSON root must be an object: %s" % path)
        return result
    except (OSError, ValueError) as exc:
        raise WorkflowError("Cannot read JSON %s: %s" % (path, exc)) from exc


def require(condition, message):
    if not condition:
        raise WorkflowError(message)


def same_json_value(left, right):
    """Compare parsed values without Python's boolean/number coercion."""
    if type(left) is not type(right):
        return False
    if isinstance(left, dict):
        return left.keys() == right.keys() and all(same_json_value(left[key], right[key]) for key in left)
    if isinstance(left, list):
        return len(left) == len(right) and all(same_json_value(a, b) for a, b in zip(left, right))
    return left == right


def profile(build=None):
    registry = read_json(HERE / "profiles.json")
    build = build or registry["default"]
    require(build in registry["profiles"], "Unsupported build profile: %s" % build)
    result = dict(registry["profiles"][build], build=build)
    report_bytes = {"260a3101": 400, "260a3102": 472, "260a3103": 472, "260b0101": 472}.get(build, 388)
    expected = (2, 536, 512, 0) if build in JOURNAL_BUILDS else (1, 536, 512, report_bytes)
    require((result["format"], result["frame_bytes"], result["flash_bytes"], result["report_bytes"]) ==
            expected, "Unsupported profile format")
    require(result.get("boot_export", False) is (build in BOOT_EXPORT_REQUESTS),
            "Profile boot-export capability does not match the pinned build")
    require(result.get("isp_export", False) is (build in ISP_EXPORT_BUILDS),
            "Profile ISP-export capability does not match the pinned build")
    result["collector"] = str(ROOT / result["collector"])
    return result


def checked_collector(selected):
    binary = Path(selected["collector"])
    require(binary.is_file() and not binary.is_symlink(), "Collector is missing or is a symlink")
    require(digest(binary) == selected["sha256"], "Collector SHA-256 mismatch; no capture started")
    return str(binary)


def allocate(output=None):
    if output is not None:
        target = Path(os.path.abspath(output))
        candidates = [target]
    else:
        base = ROOT / "artifacts" / "bmcu-a1"
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        candidates = [base / ("usb-capture-%s-%s" % (stamp, uuid.uuid4().hex[:12])) for _ in range(8)]
    for target in candidates:
        require(target.parent.is_dir(), "Output parent must already exist: %s" % target.parent)
        require(not any(p.is_symlink() for p in (target,) + tuple(target.parents)),
                "Symlink output paths are not allowed")
        try:
            target.mkdir(mode=0o700)
            # Persist the capture path before a child can program export intent.
            parent_fd = os.open(target.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(parent_fd)
            finally:
                os.close(parent_fd)
            return target
        except FileExistsError as exc:
            if output is not None:
                raise WorkflowError("Output must be a new directory: %s" % target) from exc
    raise WorkflowError("Could not allocate a unique output directory")


def stop_child(child):
    """Bounded cancellation. Return true only after wait confirms process exit."""
    if child.poll() is not None:
        return True
    try:
        child.terminate()
    except ProcessLookupError:
        pass
    try:
        child.wait(timeout=2)
    except subprocess.TimeoutExpired:
        try:
            child.kill()
        except ProcessLookupError:
            pass
        try:
            child.wait(timeout=2)
        except subprocess.TimeoutExpired:
            return False
    return True


def run_logged(argv, output, stem, timeout):
    started = time.monotonic()
    record = {"argv": argv, "started_utc": utc_now(), "exit": None,
              "timed_out": False, "interrupted": False, "process_exited": False}
    child = None
    with (output / (stem + ".stdout.log")).open("xb") as stdout, \
            (output / (stem + ".stderr.log")).open("xb") as stderr:
        try:
            child = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr)
            record["exit"] = child.wait(timeout=timeout)
            record["process_exited"] = True
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
            record["timed_out"] = isinstance(exc, subprocess.TimeoutExpired)
            record["interrupted"] = isinstance(exc, KeyboardInterrupt)
            if child is not None:
                try:
                    record["process_exited"] = stop_child(child)
                except OSError as cleanup_error:
                    record["cleanup_error"] = str(cleanup_error)
                    record["process_exited"] = child.poll() is not None
                record["exit"] = child.returncode
        except OSError as exc:
            record["launch_error"] = str(exc)
        finally:
            stdout.flush()
            stderr.flush()
            os.fsync(stdout.fileno())
            os.fsync(stderr.fileno())
    record["elapsed_seconds"] = time.monotonic() - started
    record["completed_utc"] = utc_now()
    write_json(output / (stem + ".result.json"), record)
    return record


def command_output(argv, output, stem):
    record = run_logged(argv, output, stem, 10)
    return record, (output / (stem + ".stdout.log")).read_bytes(), (output / (stem + ".stderr.log")).read_bytes()


def completed(record, exit_code):
    return (record["exit"] == exit_code and record["process_exited"] and
            not record["timed_out"] and not record["interrupted"])


def inventory_devices(raw):
    try:
        roots = plistlib.loads(raw)
    except Exception as exc:
        raise WorkflowError("Invalid ioreg plist: %s" % exc) from exc
    require(isinstance(roots, list) and all(isinstance(x, dict) for x in roots), "ioreg plist root must be an array of dictionaries")
    devices = []
    seen = {}

    def ports_in(node):
        found = []
        if "IOCalloutDevice" in node:
            require(isinstance(node["IOCalloutDevice"], str), "Invalid IOCalloutDevice value")
            found.append(node["IOCalloutDevice"])
        for child in children(node):
            if child.get("IOObjectClass") != "IOUSBHostDevice":
                found.extend(ports_in(child))
        return found

    def children(node):
        result = node.get("IORegistryEntryChildren", [])
        require(isinstance(result, list) and all(isinstance(x, dict) for x in result), "Invalid ioreg children")
        return result

    def visit(node):
        if node.get("IOObjectClass") == "IOUSBHostDevice":
            entry = node.get("IORegistryEntryID")
            require(entry is None or type(entry) is int, "Invalid registry entry ID")
            vendor, product = node.get("idVendor"), node.get("idProduct")
            require(type(vendor) is int and type(product) is int, "USB device has missing/invalid VID or PID")
            device = {"name": node.get("IORegistryEntryName"), "vendor": vendor,
                      "product": product, "location": node.get("locationID"),
                      "registry_id": entry, "ports": sorted(set(ports_in(node)))}
            if entry is not None and entry in seen:
                require(seen[entry] == device, "Conflicting duplicate USB registry entry")
                return
            if entry is not None:
                seen[entry] = device
            devices.append(device)
        for child in children(node):
            visit(child)
    for root in roots:
        visit(root)
    return devices


def preflight(output, explicit_port=None):
    evidence = {"format": 1, "started_utc": utc_now(), "passed": False, "errors": []}
    try:
        record, raw, stderr = command_output(IOREG, output, "ioreg")
        evidence["inventory_process"] = record
        require(completed(record, 0) and not stderr, "ioreg failed or emitted errors; see retained logs")
        devices = inventory_devices(raw)
        evidence["devices"] = devices
        matching = [d for d in devices if (d["vendor"], d["product"]) == (0x1a86, 0x7523)]
        require(len(matching) == 1, "Expected exactly one WCH 1a86:7523 device; found %d" % len(matching))
        device = matching[0]
        require(len(device["ports"]) == 1, "Matched WCH must have exactly one callout port")
        port = device["ports"][0]
        require(re.fullmatch(r"/dev/cu\.usbserial-[A-Za-z0-9._-]+", port) is not None,
                "Matched WCH callout is not /dev/cu.usbserial-*")
        require(explicit_port is None or explicit_port == port, "Explicit port does not match the actual WCH subtree")
        info = os.lstat(port)
        require(stat.S_ISCHR(info.st_mode), "Matched port is not a character device")
        evidence["port"] = port
        record, raw, stderr = command_output(["/bin/ps", "-axo", "pid=,comm="], output, "processes")
        evidence["process_check"] = record
        require(completed(record, 0) and not stderr, "Process inspection failed; see retained logs")
        flasher = [line for line in raw.decode("utf-8", "replace").splitlines()
                   if re.search(r"flasher", line, re.IGNORECASE)]
        evidence["flasher_processes"] = flasher
        require(not flasher, "Flasher process is present; close it explicitly before a new attempt")
        tty_port = port.replace("/dev/cu.", "/dev/tty.", 1)
        record, raw, stderr = command_output(["/usr/sbin/lsof", "-nP", "-Fpcn", "--", port, tty_port], output, "port-holders")
        evidence["port_holders_check"] = record
        require(completed(record, 1) and not raw and not stderr,
                "Port is held or holder inspection failed; see retained lsof logs")
        evidence["passed"] = True
    except (WorkflowError, OSError) as exc:
        evidence["errors"].append(str(exc))
    evidence["completed_utc"] = utc_now()
    write_json(output / "preflight.json", evidence)
    return evidence


def journal_frame_facts(frame, build):
    """Validate BFL2 pages independently; a rolling journal is never a complete trial."""
    require(len(frame) == 536 and frame[:4] == b"BFL2", "Invalid journal frame magic/size")
    require(struct.unpack_from("<HH", frame, 4) == (2, 512), "Invalid journal format/Flash size")
    require(u32(frame, 8) == int(build, 16), "Outer build does not match pinned profile")
    require(u32(frame, 532) == crc(frame[:532]), "Outer frame CRC mismatch")
    pages = [frame[20:276], frame[276:532]]
    valid = [u32(p, 0) == 0x324a4642 and u32(p, 12) == int(build, 16) and
             0 < u32(p, 4) < 0xffffffff and 0 < u32(p, 8) < 0xffffffff and
             u32(p, 16) == crc(p[:16]) for p in pages]
    newest, ambiguous = None, False
    if all(valid):
        delta = (u32(pages[0], 4) - u32(pages[1], 4)) & 0xffffffff
        ambiguous = delta in (0, 0x80000000)
        if not ambiguous:
            newest = 0 if delta < 0x80000000 else 1
    elif any(valid):
        newest = 0 if valid[0] else 1
    session = u32(pages[newest], 8) if newest is not None else 0
    overwritten = newest is not None and u32(pages[newest], 4) > 2
    flags = u32(frame, 16)
    expected = 1 | (int(valid[0]) << 1) | (int(valid[1]) << 2) | (int(overwritten) << 4) | (int(ambiguous) << 5)
    require((flags & ~8) == expected, "Outer journal flags disagree with page headers")
    require(u32(frame, 12) == session, "Outer and journal sessions disagree")
    records = 0
    for p, usable in zip(pages, valid):
        if not usable:
            continue
        for offset in range(32, 256, 16):
            record = p[offset:offset + 16]
            # Retained records may span sessions; each decoder event carries its session.
            # CRC integrity does not establish chronological order for ambiguous headers.
            seeded = zlib.crc32(record[:12], u32(p, 4) ^ 0xffffffff) & 0xffffffff
            if seeded == u32(record, 12):
                records += 1
    blank = all(u32(p, offset) in (0xffffffff, 0xe339e339)
                for p in pages for offset in range(0, 256, 4))
    return {"build": build, "session": session, "frame_crc32": "%08x" % u32(frame, 532),
            "header_valid": any(valid), "complete": False, "firmware_storage_error": bool(flags & 8),
            "journal_blank": blank,
            "wire_format": 2, "journal_pages_valid": valid, "journal_ambiguous": ambiguous,
            "journal_generation": u32(pages[newest], 4) if newest is not None else None,
            "journal_committed_records": records, "journal_older_generations_discarded": overwritten}


def frame_facts(frame, build, report_bytes=388):
    if build in JOURNAL_BUILDS:
        return journal_frame_facts(frame, build)
    require(len(frame) == 536 and frame[:4] == b"BFL1", "Invalid outer frame magic/size")
    require(struct.unpack_from("<HH", frame, 4) == (1, 512), "Invalid outer frame format/Flash size")
    require(u32(frame, 8) == int(build, 16), "Outer build does not match pinned profile")
    require(u32(frame, 532) == crc(frame[:532]), "Outer frame CRC mismatch")
    flash = frame[20:532]
    h = (u32(flash, 0) == 0x314c4642 and u32(flash, 4) == int(build, 16) and
         u32(flash, 8) not in (0, 0xffffffff) and struct.unpack_from("<HH", flash, 12) == (1, report_bytes) and
         u32(flash, 16) == crc(flash[:16]))
    complete = (h and u32(flash, 24) == 0x51c0da7a and
                u32(flash, 28) == crc(flash[32:32 + report_bytes]) and u32(flash, 508) == 0x6c726731)
    flags = u32(frame, 16)
    require(flags <= 7 and bool(flags & 1) == h and bool(flags & 2) == complete,
            "Outer flags disagree with header/report CRC/commit markers")
    require(not h or u32(frame, 12) == u32(flash, 8), "Outer and Flash sessions disagree")
    return {"build": build, "session": u32(frame, 12), "frame_crc32": "%08x" % u32(frame, 532),
            "header_valid": h, "complete": complete, "firmware_storage_error": bool(flags & 4),
            "report_crc_valid": u32(flash, 28) == crc(flash[32:32 + report_bytes]),
            "report_commit_present": u32(flash, 24) == 0x51c0da7a,
            "record_commit_present": u32(flash, 508) == 0x6c726731}


def scan_frames(raw, build, report_bytes=388):
    """Match the collector's byte-wise resynchronization, using zlib CRC."""
    pending = bytearray()
    frames, offsets = [], []
    corrupt = noise = 0
    magic = b"BFL2" if build in JOURNAL_BUILDS else b"BFL1"
    for offset, value in enumerate(raw):
        pending.append(value)
        while len(pending) >= 4 and pending[:4] != magic:
            del pending[0]
            noise += 1
        if len(pending) == 536:
            try:
                frame_facts(pending, build, report_bytes)
            except WorkflowError:
                corrupt += 1
                del pending[0]
            else:
                frames.append(bytes(pending))
                offsets.append(offset - 535)
                pending.clear()
    return frames, {"frames": len(frames), "corrupt_frames": corrupt, "noise_bytes": noise,
                    "partial_tail_bytes": len(pending), "first_valid_offset": offsets[0] if offsets else None,
                    "last_valid_offset": offsets[-1] if offsets else None}


def snapshot_hashes(directory):
    result = {}
    for path in sorted(directory.rglob("*")):
        require(not path.is_symlink(), "Symlinks are not allowed in saved captures")
        if path.is_file() and path != directory / "hashes.json":
            result[str(path.relative_to(directory))] = digest(path)
    return result


def verify_hashes(directory):
    manifest = directory / "hashes.json"
    if manifest.exists():
        require(read_json(manifest) == snapshot_hashes(directory), "Saved hash manifest does not match files")
        return True
    return False


def verify_boot_export(capture, selected, metadata):
    """Bind working-mode selection evidence without performing a device operation."""
    request = capture / "export-request.bin"
    result = capture / "export-result.json"
    fields = ("export_requested", "export_request_sent", "export_mode")
    working = selected["build"] in WORKING_BUILDS
    if not working:
        require(not any(key in metadata for key in fields) and
                not request.exists() and not result.exists() and
                not request.is_symlink() and not result.is_symlink(),
                "Boot-export evidence is not allowed for this build")
        return "not_requested"
    require(metadata["ack_requested"] is False, "Working mode does not support re-arm")
    live = metadata["mode"] == "capture"
    require(selected["build"] not in ISP_EXPORT_BUILDS or not live,
            "ISP build requires truthful offline decoder metadata")
    expected = (live, 15 if live else 0, "boot-request" if live else "offline")
    require(all(same_json_value(metadata.get(key), value) for key, value in zip(fields, expected)),
            "Working-mode export metadata disagrees with capture mode")
    if not live:
        require(not request.exists() and not result.exists() and
                not request.is_symlink() and not result.is_symlink(),
                "Offline decoding must not contain boot-export request artifacts")
        return "not_requested"
    require(request.is_file() and result.is_file() and not request.is_symlink() and not result.is_symlink(),
            "Missing/unsafe boot-export artifact pair")
    require(request.read_bytes() == BOOT_EXPORT_REQUESTS[selected["build"]],
            "Boot-export request differs from the exact build-bound command")
    expected_result = dict(format=1, build=selected["build"], mode="boot-request",
                           request_bytes=15, sent=15, initial_baud=1250000,
                           initial_parity="even", export_baud=115200, export_parity="none",
                           mode_switched=True, confirmed=True, reason="confirmed")
    require(same_json_value(read_json(result), expected_result),
            "Boot-export result is incomplete, inconsistent or unconfirmed")
    return "confirmed_by_collector"


def verify_isp_export(directory, selected, raw):
    transport = directory / "isp"
    if not transport.exists():
        require(not transport.is_symlink(), "Unsafe ISP evidence path")
        return "not_requested"
    require(selected["build"] in ISP_EXPORT_BUILDS, "ISP evidence forbidden for this build")
    require(transport.is_dir() and not transport.is_symlink(), "Unsafe ISP evidence path")
    for name in ("marker.bin", "isp-export.json", "raw.bin"):
        require((transport / name).is_file() and not (transport / name).is_symlink(),
                "Missing/unsafe ISP evidence: " + name)
    require((transport / "marker.bin").read_bytes() == ISP_MARKER, "ISP marker mismatch")
    require((transport / "raw.bin").read_bytes() == raw, "ISP raw bytes differ from decoder input")
    actual = read_json(transport / "isp-export.json")
    require(actual.get("operation") in ("prepare", "resume_pending"), "Unknown ISP operation")
    expected = dict(format=2, status_payload_checked=True, build=selected["build"], isolated_usb_only=True,
                    firmware_sha256=selected["firmware_sha256"], marker_address=0xED00,
                    marker_hex=ISP_MARKER.hex(), verified_image_bytes=selected["firmware_padded_bytes"],
                    marker_verified=True, capture_finished=True, raw_bytes=len(raw),
                    raw_sha256=hashlib.sha256(raw).hexdigest(), port_closed=True,
                    stage="captured_unvalidated", success=True, baud=115200, parity="none", serial_openings=1,
                    operation=actual["operation"], start_command_hex="57aba2010001a4",
                    boot_released_before_start=True)
    require(same_json_value(actual, expected),
            "ISP transport evidence is incomplete or inconsistent; retain isolation")
    return "marker_consumption_confirmed_by_valid_frame"


def isp_capture(selected, args, output, port, decoder):
    firmware = ROOT / selected["firmware"]
    require(firmware.is_file() and not firmware.is_symlink() and
            digest(firmware) == selected["firmware_sha256"], "Pinned ISP firmware is missing/changed")
    host = ROOT / "tools/bmcu-flasher-mcp"
    python = host / ".venv/bin/python"
    require(python.is_file(), "ISP export requires the existing flasher Python environment")
    process = run_logged([str(python), str(host / "export.py"), "--firmware", str(firmware),
                          "--sha256", selected["firmware_sha256"], "--isolated-usb-only",
                          "--port", port, "--output", str(output / "isp"),
                          "--seconds", str(args.seconds), "--max-bytes", str(args.max_bytes)],
                         output, "isp-transport", args.seconds + 120)
    if process["process_exited"] and (output / "isp/raw.bin").is_file():
        decoded = run_logged([decoder, "--input", str(output / "isp/raw.bin"),
                              "--max-bytes", str(args.max_bytes), "--output", str(output / "capture")],
                             output, "decoder", 10)
        if decoded["exit"] != 0:
            process = dict(process, exit=1)
    return process


def verify_capture(directory, selected):
    """Return a receipt, never modify the supplied directory, never access hardware."""
    directory = Path(directory)
    require(directory.is_dir() and not directory.is_symlink(), "Capture must be a directory")
    hashes_verified = verify_hashes(directory)
    capture = directory / "capture" if (directory / "capture").is_dir() else directory
    require(not capture.is_symlink(), "Capture symlinks are not allowed")
    for name in ("raw.bin", "report.json", "metadata.json"):
        require((capture / name).is_file() and not (capture / name).is_symlink(), "Missing/unsafe capture file: " + name)
    raw = (capture / "raw.bin").read_bytes()
    require(len(raw) <= 1048576, "Capture exceeds the supported collector byte limit")
    report, metadata = read_json(capture / "report.json"), read_json(capture / "metadata.json")
    frames, counts = scan_frames(raw, selected["build"], selected["report_bytes"])
    require(frames, "No valid frame for the pinned build")
    require(all(frame == frames[0] for frame in frames), "Conflicting valid repeated frames")
    facts = frame_facts(frames[0], selected["build"], selected["report_bytes"])
    require(facts["header_valid"] or
            (selected["build"] in WORKING_BUILDS and facts.get("journal_blank") is True),
            "No valid Flash header; payload and re-arm binding unavailable")
    require(report.get("raw_flash") == frames[0][20:532].hex(), "Decoded raw Flash differs from captured frame")
    for key in ("build", "session", "frame_crc32", "header_valid", "complete", "firmware_storage_error"):
        require(same_json_value(report.get(key), facts[key]), "Decoded report mismatch: " + key)
    if selected["format"] == 2:
        journal = report.get("journal")
        require(isinstance(journal, dict), "Missing decoded journal")
        for field, fact in (("committed_events", "journal_committed_records"),
                            ("newest_generation", "journal_generation"),
                            ("ambiguous_headers", "journal_ambiguous"),
                            ("prior_generations_discarded", "journal_older_generations_discarded")):
            require(same_json_value(journal.get(field), facts[fact]), "Decoded journal mismatch: " + field)
    require(report.get("transport_valid") is True and same_json_value(report.get("format"), selected["format"]), "Invalid decoded transport/format")
    require(same_json_value(metadata.get("format"), 1) and metadata.get("mode") in ("capture", "offline"), "Invalid metadata format/mode")
    for key in ("frames", "corrupt_frames", "noise_bytes", "partial_tail_bytes"):
        require(type(metadata.get(key)) is int and metadata[key] == counts[key], "Metadata mismatch: " + key)
    require(metadata.get("bytes") == len(raw) and metadata.get("raw_crc32") == "%08x" % crc(raw), "Metadata raw byte/CRC mismatch")
    for key in ("capture_failed", "ack_requested", "observe_stream", "frame_timestamps_available"):
        require(type(metadata.get(key)) is bool, "Invalid metadata boolean: " + key)
    require(metadata["frame_timestamps_available"] == (metadata["mode"] == "capture"), "Metadata timestamp availability mismatch")
    times = [metadata.get("first_frame_seconds"), metadata.get("last_frame_seconds")]
    require(all(type(t) in (int, float) and math.isfinite(t) for t in times) and 0 <= times[0] <= times[1] <= 61,
            "Invalid frame receive timestamps")
    require(metadata["mode"] != "offline" or (times == [0, 0] and not metadata["ack_requested"] and not metadata["observe_stream"]),
            "Offline metadata claims live operations")
    require(metadata.get("reason") in ("eof", "timeout", "duration", "report", "size_limit", "clock_error", "poll_error", "disconnected", "read_error", "disk_error", "conflicting_reports"), "Invalid metadata stop reason")
    require(not metadata["capture_failed"], "Collector marked capture_failed; bytes retained but capture is not accepted")
    allowed_reasons = {"eof", "size_limit"} if metadata["mode"] == "offline" else {"timeout", "duration", "report", "size_limit"}
    require(metadata["reason"] in allowed_reasons, "Metadata stop reason disagrees with mode/success")
    require(metadata["reason"] != "duration" or metadata["observe_stream"], "Duration stop requires stream observation")
    require(metadata["reason"] != "report" or not metadata["observe_stream"], "First-report stop disagrees with stream observation")
    require(not facts["firmware_storage_error"], "Firmware storage error flag is set")
    export_status = verify_boot_export(capture, selected, metadata)
    # Reuse the exact pinned decoder; compare every decoded field, not just selected facts.
    binary = checked_collector(selected)
    with tempfile.TemporaryDirectory(prefix="bmcu-usb-verify-") as scratch:
        decoded = Path(scratch) / "decoded"
        try:
            result = subprocess.run([binary, "--input", str((capture / "raw.bin").resolve()),
                                     "--max-bytes", str(len(raw)), "--output", str(decoded)],
                                    stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise WorkflowError("Offline pinned decoder failed: %s" % exc) from exc
        require(result.returncode == 0, "Offline pinned decoder rejected capture: " + result.stderr.decode("utf-8", "replace"))
        require(same_json_value(read_json(decoded / "report.json"), report), "Decoded report differs from pinned offline decoder")
    ack, ack_result = capture / "ack.bin", capture / "ack-result.json"
    ack_status = "not_requested"
    if ack.exists() or ack_result.exists():
        require(metadata["ack_requested"], "ACK artifacts exist for a receive-only capture")
        require(not facts.get("journal_ambiguous", False), "Ambiguous journal cannot bind a re-arm")
        require(ack.is_file() and ack_result.is_file() and not ack.is_symlink() and not ack_result.is_symlink(), "Incomplete/unsafe ACK artifact pair")
        command = ack.read_bytes()
        magic = b"BFA2" if selected["format"] == 2 else b"BFA1"
        expected = magic + struct.pack("<III", int(selected["build"], 16), facts["session"], int(facts["frame_crc32"], 16))
        require(command == expected + struct.pack("<I", crc(expected)), "ACK command magic/build/session/frameCRC/CRC mismatch")
        outcome = read_json(ack_result)
        require(set(outcome) == {"sent", "confirmed", "uncertain", "immediate_motion"}, "Invalid ACK result fields")
        sent, confirmed, uncertain = outcome["sent"], outcome["confirmed"], outcome["uncertain"]
        require(type(sent) is int and 0 <= sent <= 20 and type(confirmed) is bool and type(uncertain) is bool and
                outcome["immediate_motion"] is False, "Invalid ACK result types/values")
        require((not confirmed or sent == 20) and uncertain == bool(sent and not confirmed), "Inconsistent ACK result")
        ack_status = "confirmed_by_collector" if confirmed else "uncertain" if uncertain else "not_sent"
    elif metadata["ack_requested"]:
        raise WorkflowError("Requested ACK has no saved result; outcome unknown, no retry")
    if (directory / "context.json").exists():
        context = read_json(directory / "context.json")
        require(context.get("build") == selected["build"] and same_json_value(context.get("rearm"), metadata["ack_requested"]), "Context build/re-arm disagrees with capture")
        require(context.get("owner_isolated_usb_only_assertion") is True, "Capture context lacks owner isolation assertion")
        if selected["build"] in BOOT_EXPORT_REQUESTS:
            require(context.get("boot_export") is True, "Capture context lacks working-mode export selection")
        if metadata["ack_requested"]:
            next_test(context.get("next_test") or "")
            require(context.get("next_power_cycle_boundary") == BOUNDARY, "Re-arm context lacks next A1-only boundary")
    isp_status = verify_isp_export(directory, selected, raw)
    if selected["build"] in ISP_EXPORT_BUILDS and (directory / "context.json").exists():
        require(isp_status != "not_requested", "Live context lacks ISP capture evidence")
    if isp_status != "not_requested":
        export_status = isp_status
    return dict(facts, format=1, scope="Offline saved-byte binding; no serial access", raw_bytes=len(raw),
                raw_crc32="%08x" % crc(raw), counts=counts, all_valid_frames_identical=True,
                raw_flash_matches_frame=True, all_decoded_fields_match_pinned_decoder=True,
                ack_status=ack_status, export_status=export_status,
                ack_response_raw_available=False, saved_manifest_verified=hashes_verified,
                hashes=snapshot_hashes(directory), installed_entire_BIN_readback=False,
                payload_facts_available=(facts.get("journal_committed_records", 0) > 0
                                         if selected["format"] == 2 else facts["complete"]))


def next_test(value):
    value = value.strip()
    require(3 <= len(value) <= 200 and re.search(r"[A-Za-z0-9]", value) and
            value.lower() not in {"test", "next", "next-test", "default", "generic", "unknown", "tbd", "none", "n/a"},
            "--rearm requires a concrete nonempty --next-test, such as empty-setup or load-unload")
    return value


def result_markdown(result):
    lines = ["# USB workflow result", "", "Status: " + ("PASS" if result["success"] else "FAIL / INCOMPLETE"),
             "Saved: `" + result["output"] + "`", "", "Descriptor: " + result.get("descriptor", "not verified"),
             "ACK: " + result.get("ack_status", "unknown / not verified"),
             "Boot export: " + result.get("export_status", "not requested / not verified"),
             "Collector process confirmed exited: " + str(result.get("collector_process_exited", False)),
             "Port closed after collector exit: " + str(result.get("port_closed_after_process_exit", False)), ""]
    if result.get("capture_scope"):
        lines += [result["capture_scope"], ""]
    if result.get("errors"):
        lines += ["Errors:"] + ["- " + error for error in result["errors"]] + [""]
    boundary = ISP_BOUNDARY if result.get("isp_export") else WORKING_BOUNDARY if result.get("boot_export") else BOUNDARY
    return "\n".join(lines + [boundary, ""] + ["- " + note for note in DISCLOSURES]) + "\n"


def collect(args):
    if args.command == "collect":
        require(args.isolated_usb_only, "collect requires the explicit --isolated-usb-only owner assertion")
        require(not args.next_test or args.rearm, "--next-test requires --rearm")
        if args.rearm:
            args.next_test = next_test(args.next_test or "")
    selected = profile(args.build)
    working = selected["build"] in WORKING_BUILDS
    require(not working or not getattr(args, "rearm", False),
            "Working firmware starts normally without re-arm; --rearm is not supported")
    require(selected["build"] not in ISP_EXPORT_BUILDS or not getattr(args, "observe_stream", False),
            "ISP export uses a bounded raw capture; --observe-stream is unsupported")
    output = allocate(args.output)
    result = {"format": 1, "output": str(output), "success": False, "errors": [], "started_utc": utc_now(),
              "boot_export": working and args.command == "collect",
              "isp_export": selected["build"] in ISP_EXPORT_BUILDS and args.command == "collect",
              "collector_process_exited": False, "port_closed_after_process_exit": False,
              "ack_status": "not_requested" if not getattr(args, "rearm", False) else "unknown / not verified"}
    context = {"format": 1, "build": selected["build"], "profile": selected, "invocation_argv": sys.argv,
               "owner_isolated_usb_only_assertion": getattr(args, "isolated_usb_only", False),
               "owner_note": args.note, "rearm": getattr(args, "rearm", False),
               "boot_export": working and args.command == "collect",
               "next_test": getattr(args, "next_test", None),
               "next_power_cycle_boundary": (ISP_BOUNDARY if selected["build"] in ISP_EXPORT_BUILDS
                                             else WORKING_BOUNDARY if working else BOUNDARY),
               "disclosures": DISCLOSURES}
    write_json(output / "context.json", context)
    try:
        binary = checked_collector(selected)
        checks = preflight(output, args.port)
        require(checks["passed"], "; ".join(checks["errors"]))
        result["descriptor"] = "WCH 1a86:7523 -> " + checks["port"]
        if args.command == "collect":
            argv = [binary, "--port", checks["port"], "--isolated-usb-only", "--seconds", str(args.seconds),
                    "--max-bytes", str(args.max_bytes), "--output", str(output / "capture")]
            if args.observe_stream:
                argv.append("--observe-stream")
            if selected["build"] in BOOT_EXPORT_REQUESTS:
                argv.append("--boot-export")
            if args.rearm:
                argv.append("--rearm")
            checked_collector(selected)
            process = (isp_capture(selected, args, output, checks["port"], binary)
                       if selected["build"] in ISP_EXPORT_BUILDS else
                       run_logged(argv, output, "collector", args.seconds + 8))
            result["collector_process_exited"] = process["process_exited"]
            result["port_closed_after_process_exit"] = process["process_exited"]
            # Verify even a nonzero exit: preserve any valid byte binding and ACK evidence.
            try:
                receipt = verify_capture(output, selected)
                write_json(output / "byte-bindings.json", receipt)
                result["ack_status"] = receipt["ack_status"]
                result["export_status"] = receipt["export_status"]
                result["capture"] = {key: receipt[key] for key in ("build", "session", "complete", "frame_crc32")}
                if selected["format"] == 2:
                    result["capture"].update({key: receipt[key] for key in
                                              ("journal_generation", "journal_committed_records", "journal_ambiguous")})
                    result["capture_scope"] = ("Validated rolling journal bytes, not a complete trial or a printer success. "
                                               "Old events can be overwritten; pending events can be lost at power-off.")
            except WorkflowError as exc:
                write_json(output / "byte-bindings.json", {"format": 1, "passed": False,
                           "error": str(exc), "available_file_hashes": snapshot_hashes(output),
                           "ack_status": "unknown / not verified", "hardware_access": False})
                result["errors"].append("Offline verification: " + str(exc))
            require(process["process_exited"] and process["exit"] == 0 and not process["timed_out"] and not process["interrupted"],
                    "Collector did not complete successfully; no retry; child RAM buffers may be unsaved")
            require(not result["errors"], "Capture verification did not pass")
            require(not args.rearm or result["ack_status"] == "confirmed_by_collector", "Re-arm is not confirmed; retain isolation; no retry")
        result["success"] = True
    except (WorkflowError, OSError, KeyboardInterrupt) as exc:
        result["errors"].append(str(exc) or "Interrupted")
    result["completed_utc"] = utc_now()
    write_json(output / "result.json", result)
    write_bytes(output / "RESULT.md", result_markdown(result).encode())
    write_json(output / "hashes.json", snapshot_hashes(output))
    print(("PASS" if result["success"] else "FAIL / INCOMPLETE") + ": " + str(output))
    print("Descriptor: " + result.get("descriptor", "not verified") + "; ACK: " + result["ack_status"])
    if args.command == "collect":
        print("Port closed after confirmed collector exit: " + str(result["port_closed_after_process_exit"]))
    for error in result["errors"]:
        print(error, file=sys.stderr)
    return 0 if result["success"] else 1


def bounded_int(low, high):
    def parse(value):
        number = int(value)
        if not low <= number <= high:
            raise argparse.ArgumentTypeError("must be in %d..%d" % (low, high))
        return number
    return parse


def parser():
    root = argparse.ArgumentParser(description=__doc__)
    commands = root.add_subparsers(dest="command", required=True)
    for name in ("preflight", "collect"):
        command = commands.add_parser(name)
        command.add_argument("--build", default=None, help="pinned profile (default: profiles.json current build)")
        command.add_argument("--port", help="optional explicit port; must match the unique WCH subtree")
        command.add_argument("--output", help="new directory; parent must exist")
        command.add_argument("--note", default="", help="owner-supplied context; not inferred readiness")
        if name == "collect":
            command.add_argument("--isolated-usb-only", action="store_true")
            command.add_argument("--rearm", action="store_true")
            command.add_argument("--next-test", help="concrete intended next A1-only test; required with --rearm")
            command.add_argument("--seconds", type=bounded_int(1, 60), default=10)
            command.add_argument("--max-bytes", type=bounded_int(536, 1048576), default=65536)
            command.add_argument("--observe-stream", action="store_true")
    command = commands.add_parser("verify", help="read-only offline verification; prints JSON receipt")
    command.add_argument("capture", type=Path)
    command.add_argument("--build", default=None)
    return root


def main(argv=None):
    args = parser().parse_args(argv)
    def interrupted(signum, frame):
        raise KeyboardInterrupt("Terminated")
    previous = signal.signal(signal.SIGTERM, interrupted)
    try:
        if args.command == "verify":
            receipt = verify_capture(args.capture, profile(args.build))
            print(json.dumps(receipt, indent=2, sort_keys=True))
            return 0 if receipt["ack_status"] in ("not_requested", "confirmed_by_collector") else 1
        return collect(args)
    except (WorkflowError, OSError) as exc:
        print("ERROR: " + str(exc), file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGTERM, previous)
