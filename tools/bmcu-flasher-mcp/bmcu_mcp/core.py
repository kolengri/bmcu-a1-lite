"""Local image validation, exclusive flash jobs, and read-only observation."""
from __future__ import annotations

import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import time
import uuid

BACKEND_SHA256 = "217e86d191da18a28a2b1f4c89141ce5e4ece4adead45f709b97a0f0a8822d51"
BACKEND = Path(__file__).resolve().parents[1] / "vendor/bmcu_flasher.py"
MAX_IMAGE = 65520


class FlasherError(Exception):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def image_bytes(path: Path) -> bytes:
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(fd, "rb") as stream:
            if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
                raise FlasherError("INVALID_IMAGE", "Firmware must be a regular file")
            data = stream.read(MAX_IMAGE + 1)
    except OSError as exc:
        raise FlasherError("INVALID_IMAGE", str(exc)) from exc
    if path.suffix.lower() != ".bin" or not 0 < len(data) <= MAX_IMAGE:
        raise FlasherError("INVALID_IMAGE", f"Expected a nonempty .bin file of at most {MAX_IMAGE} bytes")
    return data


def atomic_json(path: Path, value: dict) -> None:
    temp = path.with_suffix(".tmp")
    fd = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as stream:
        json.dump(value, stream, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, path)


def devices() -> list[dict]:
    from serial.tools.list_ports import comports
    return [dict(port=p.device, vid=p.vid, pid=p.pid, serial_number=p.serial_number,
                 location=p.location) for p in comports() if (p.vid, p.pid) == (0x1A86, 0x7523)]


def preflight(port: str | None = None) -> list[str]:
    """Inspect processes and open-file metadata; never open a device."""
    result = subprocess.run(["/bin/ps", "-axo", "pid=,comm="], capture_output=True, text=True)
    if result.returncode:
        raise FlasherError("PREFLIGHT_FAILED", "Cannot inspect running processes")
    conflicts = [line.strip() for line in result.stdout.splitlines()
                 if "BMCU Flasher.app/Contents/MacOS/" in line]
    if port:
        aliases = [port]
        if port.startswith("/dev/cu."):
            aliases.append(port.replace("/dev/cu.", "/dev/tty.", 1))
        result = subprocess.run(["/usr/sbin/lsof", "-nP", "-Fpcn", "--", *aliases],
                                capture_output=True, text=True)
        if result.returncode not in (0, 1) or (result.returncode == 1 and result.stderr.strip()):
            raise FlasherError("PREFLIGHT_FAILED", "Cannot inspect serial-port ownership")
        if result.stdout.strip():
            raise FlasherError("PORT_BUSY", result.stdout.strip())
    return conflicts


def check_backend() -> None:
    if hashlib.sha256(BACKEND.read_bytes()).hexdigest() != BACKEND_SHA256:
        raise FlasherError("BACKEND_CHANGED", "Pinned flasher source checksum mismatch")


class FlasherService:
    def __init__(self, workspace: Path, state_dir: Path | None = None):
        self.workspace = Path(workspace).absolute()
        self.jobs = self.workspace / "artifacts/bmcu-flasher-mcp/jobs"
        self.state_dir = state_dir or Path.home() / "Library/Caches/bmcu-flasher-mcp"
        self.children: dict[str, subprocess.Popen] = {}

    def _lock(self, create: bool = True) -> int | None:
        if not create and not (self.state_dir / "flash.lock").exists():
            return None
        self.state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        fd = os.open(self.state_dir / "flash.lock", os.O_RDWR | os.O_NOFOLLOW |
                     (os.O_CREAT if create else 0), 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            os.close(fd)
            raise FlasherError("BUSY", "Another flash worker holds the device lock") from exc
        return fd

    def status(self) -> dict:
        check_backend()
        busy = False
        try:
            fd = self._lock(False)
            if fd is not None:
                os.close(fd)
        except FlasherError:
            busy = True
        return {"platform": sys.platform, "backend": "BMCU-Flasher v1.4", "backend_sha256": BACKEND_SHA256,
                "devices": devices(), "gui_conflicts": preflight(), "busy": busy,
                "recent_job_ids": sorted((p.name for p in self.jobs.iterdir() if re.fullmatch(r"[0-9a-f]{32}", p.name)), key=lambda name: (self.jobs / name).stat().st_mtime, reverse=True)[:10] if self.jobs.exists() else [],
                "serial_opened": False, "erase_policy": "full 64 KiB; calibration and logs are erased"}

    def _image(self, firmware_path: str) -> tuple[Path, bytes]:
        path = Path(firmware_path).expanduser()
        if not path.is_absolute():
            path = self.workspace / path
        return path, image_bytes(path)

    def inspect_firmware(self, firmware_path: str) -> dict:
        path, data = self._image(firmware_path)
        return {"path": str(path), "sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data),
                "padded_bytes": ((len(data) + 55) // 56) * 56,
                "target": "CH32V203C8T6 (device 0x31, type 0x19)", "erase_bytes": 65536,
                "calibration_and_logs_erased": True, "serial_opened": False}

    def start_flash(self, firmware_path: str, expected_sha256: str,
                    isolated_usb_only: bool, paths_empty: bool, port: str | None = None) -> dict:
        if isolated_usb_only is not True or paths_empty is not True:
            raise FlasherError("CONFIRMATION_REQUIRED", "Owner must confirm isolated USB-only power and empty filament paths")
        path, data = self._image(firmware_path)
        digest = hashlib.sha256(data).hexdigest()
        if not isinstance(expected_sha256, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", expected_sha256) or digest != expected_sha256.lower():
            raise FlasherError("HASH_MISMATCH", "Firmware differs from the inspected SHA-256")
        check_backend()
        if sys.platform != "darwin":
            raise FlasherError("UNSUPPORTED_PLATFORM", "This adapter is qualified for macOS CH340 USB only")
        fd = self._lock()
        try:
            matches = [d for d in devices() if port is None or d["port"] == port]
            if not matches:
                raise FlasherError("NO_DEVICE", "No matching CH340 USB device")
            if len(matches) != 1:
                raise FlasherError("AMBIGUOUS_DEVICE", "Select an explicit matching serial port")
            device = matches[0]
            if preflight(device["port"]):
                raise FlasherError("FLASHER_RUNNING", "Close the BMCU Flasher GUI before flashing")
            job_id = uuid.uuid4().hex
            job_dir = self.jobs / job_id
            job_dir.mkdir(parents=True, mode=0o700)
            snapshot = job_dir / "firmware.bin"
            with open(snapshot, "xb") as stream:
                os.chmod(snapshot, 0o600)
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            manifest = {"job_id": job_id, "created_at": time.time(), "source": str(path),
                        "sha256": digest, "device": device, "isolated_usb_only": True,
                        "paths_empty": True, "erase_bytes": 65536}
            atomic_json(job_dir / "job.json", manifest)
            with open(job_dir / "worker.log", "ab", buffering=0) as output:
                child = subprocess.Popen([sys.executable, "-m", "bmcu_mcp.worker", str(job_dir), str(fd)],
                                         cwd=Path(__file__).resolve().parents[1], pass_fds=(fd,),
                                         stdin=subprocess.DEVNULL, stdout=output, stderr=output,
                                         start_new_session=True)
            self.children[job_id] = child
            manifest["pid"] = child.pid
            atomic_json(job_dir / "job.json", manifest)
            return {"job_id": job_id, "state": "queued", "port": device["port"],
                    "output_dir": str(job_dir), "erase_bytes": 65536}
        finally:
            os.close(fd)

    def job_status(self, job_id: str) -> dict:
        if not re.fullmatch(r"[0-9a-f]{32}", job_id):
            raise FlasherError("UNKNOWN_JOB", "Invalid job ID")
        directory = self.jobs / job_id
        try:
            manifest = json.loads((directory / "job.json").read_text())
        except (OSError, ValueError) as exc:
            raise FlasherError("UNKNOWN_JOB", "No readable job manifest") from exc
        child = self.children.get(job_id)
        if child is not None:
            alive = child.poll() is None
        else:
            try:
                pid = json.loads((directory / "started.json").read_text())["pid"]
            except (OSError, ValueError, KeyError):
                pid = manifest.get("pid")
            if not isinstance(pid, int):
                return {"job_id": job_id, "state": "unknown", "port_closed_after_process_exit": False}
            process = subprocess.run(["/bin/ps", "-p", str(pid), "-o", "args="], capture_output=True, text=True)
            if process.returncode not in (0, 1):
                raise FlasherError("STATUS_FAILED", "Cannot establish worker process state")
            alive = str(directory) in process.stdout and "bmcu_mcp.worker" in process.stdout
        result = {}
        try:
            result = json.loads((directory / "result.json").read_text())
        except (OSError, ValueError):
            pass
        log = directory / "worker.log"
        tail = ""
        if log.exists():
            with log.open("rb") as stream:
                stream.seek(max(0, log.stat().st_size - 8192))
                tail = stream.read(8192).decode("utf-8", errors="replace")
        state = "running" if alive else result.get("state", "interrupted")
        if child is not None and child.returncode not in (None, 0) and state == "succeeded":
            state = "failed"
        return {"job_id": job_id, "state": state, "port_closed_after_process_exit": not alive,
                "result": result, "log_tail": tail, "output_dir": str(directory)}
