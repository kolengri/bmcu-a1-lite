"""Detached worker: pinned stock USB flash, with identity and cleanup guards."""
from __future__ import annotations

import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

from .core import BACKEND, FlasherError, atomic_json, check_backend, devices, image_bytes, preflight


def validate_status_payload(cmd: int, code: int, payload: bytes) -> None:
    """The pinned parser returns a reserved header byte separately from status."""
    if cmd in (0xA4, 0xA5, 0xA6, 0xA8) and (code != 0 or payload != b"\0\0"):
        raise FlasherError("ISP_STATUS", "ISP 0x%02X rejected header=0x%02X payload=%s; no retry" %
                           (cmd, code, payload.hex() or "<empty>"))


def load_backend():
    check_backend()
    spec = importlib.util.spec_from_file_location("bmcu_flasher_pinned_v14", BACKEND)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def flash(module, firmware: Path, port: str) -> None:
    """Use the original algorithm; guard every identification and close all transports."""
    original = module.WchIsp
    opened = []

    class GuardedIsp(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            opened.append(self)

        def open(self):
            # Request POSIX advisory exclusivity before the first serial write.
            serial_factory = module.serial.Serial
            def exclusive_serial(*args, **kwargs):
                kwargs["exclusive"] = True
                serial = serial_factory(*args, **kwargs)
                try:
                    import termios
                    fcntl.ioctl(serial.fileno(), termios.TIOCEXCL)
                except BaseException:
                    serial.close()
                    raise
                return serial
            module.serial.Serial = exclusive_serial
            try:
                super().open()
            finally:
                module.serial.Serial = serial_factory

        def recv(self, expect_cmd, timeout_s):
            code, data = super().recv(expect_cmd, timeout_s)
            validate_status_payload(expect_cmd, code, data)
            if expect_cmd == 0xA1 and code == 0 and data[:2] != b"\x31\x19":
                raise FlasherError("TARGET_MISMATCH", "ISP identity is not CH32V203C8T6")
            return code, data

    module.WchIsp = GuardedIsp
    try:
        module.flash_firmware(str(firmware), mode="usb", port=port, flash_kb=64,
                              no_fast=True, verify=True, verify_every=1, verify_last=True,
                              trace=False, log_cb=lambda level, message: print(f"{level}: {message}", flush=True))
    finally:
        for transport in opened:
            transport.close()
        module.WchIsp = original


def run_job(directory: Path, lock_fd: int) -> int:
    # The inherited descriptor keeps the same flock alive across parent shutdown.
    fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    atomic_json(directory / "started.json", {"pid": os.getpid(), "started_at": time.time()})
    result = {"state": "failed", "physical_state": "unknown", "completed_at": None,
              "status_payload_checked": True}
    try:
        manifest = json.loads((directory / "job.json").read_text())
        if manifest.get("isolated_usb_only") is not True or manifest.get("paths_empty") is not True:
            raise FlasherError("CONFIRMATION_REQUIRED", "Missing owner facts")
        firmware = directory / "firmware.bin"
        if hashlib.sha256(image_bytes(firmware)).hexdigest() != manifest["sha256"]:
            raise FlasherError("HASH_MISMATCH", "Job snapshot changed")
        device = manifest["device"]
        if device not in devices():
            raise FlasherError("DEVICE_CHANGED", "USB device metadata changed before opening")
        if preflight(device["port"]):
            raise FlasherError("FLASHER_RUNNING", "Flasher GUI is running")
        flash(load_backend(), firmware, device["port"])
        result.update(state="succeeded", physical_state="firmware_programmed",
                      verification="ISP A6 accepted every 56-byte padded block including the last")
    except BaseException as exc:
        result.update(error_code=getattr(exc, "code", type(exc).__name__), error=str(exc))
        print(f"FAILED: {exc}", flush=True)
    finally:
        result["completed_at"] = time.time()
        atomic_json(directory / "result.json", result)
        os.close(lock_fd)
    return 0 if result["state"] == "succeeded" else 1


if __name__ == "__main__":
    sys.exit(run_job(Path(sys.argv[1]), int(sys.argv[2])))
