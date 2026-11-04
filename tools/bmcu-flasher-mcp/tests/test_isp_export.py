"""Transport fault boundaries using the actual pinned packet codec; no hardware."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from contextlib import ExitStack
from unittest.mock import patch

from bmcu_mcp import isp_export as export
from bmcu_mcp.worker import load_backend


class ResponseSerial:
    """Feed literal ISP response bytes to the unmodified pinned recv parser."""
    def __init__(self, response):
        self.response = bytearray(response)
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.response)

    def read(self, count):
        result = bytes(self.response[:count])
        del self.response[:count]
        return result

    def write(self, packet):
        self.writes.append(packet)
        return len(packet)


class CapturedStatusTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_backend()

    def transport(self, response):
        isp = self.module.WchIsp("FAKE_ONLY", 115200, self.module.serial.PARITY_NONE, False)
        isp.ser = ResponseSerial(bytes.fromhex(response))
        return isp

    def test_actual_recv_exposes_reserved_zero_separately_from_failure_payload(self):
        # Literal header0/f500 and header0/fe00 frames matching physical probes.
        for wire, payload in (("55aaa6000200f5009d", b"\xf5\0"),
                              ("55aaa6000200fe00a6", b"\xfe\0"),
                              ("55aaa60002000000a8", b"\0\0")):
            with self.subTest(payload=payload.hex()):
                self.assertEqual(self.transport(wire).recv(0xA6, 0.1), (0, payload))

    def test_companion_rejects_failed_and_malformed_actual_recv_status(self):
        for wire, payload in (("55aaa6000200f5009d", "f500"),
                              ("55aaa6000200fe00a6", "fe00"),
                              ("55aaa6000000a6", "<empty>"),
                              ("55aaa600010000a7", "00"),
                              ("55aaa6000300000000a9", "000000"),
                              ("55aaa60102000000a9", "0000")):
            with self.subTest(wire=wire):
                isp = self.transport(wire)
                with self.assertRaises(export.FlasherError) as error:
                    export.command(isp, self.module.build_verify(0, 0, b"x" * 56), 0xA6)
                self.assertIn("A6", str(error.exception))
                self.assertIn(payload, str(error.exception))
                self.assertEqual(len(isp.ser.writes), 1)

    def test_companion_accepts_only_actual_zero_status(self):
        isp = self.transport("55aaa60002000000a8")
        self.assertEqual(export.command(isp, self.module.build_verify(0, 0, b"x" * 56), 0xA6), b"\0\0")


class Transport:
    def __init__(self, module, firmware):
        self.module = module
        self.ser = self
        self.dtr = self.rts = False
        self.commands = []
        self.memory = bytearray(b"\xff" * 65536)
        self.memory[:len(firmware)] = firmware
        self.memory[0xEE00:] = bytes(range(256)) * 18
        self.protected_before = bytes(self.memory[0xEE00:])
        self.uid = bytes(range(8))
        self.key = module.calc_xor_key_uid(self.uid, 0x31)
        self.reads = [b"BFL2", b"example"]
        self.fail_verify = False
        self.status_responses = {}
        self.fail_command = None
        self.short_command = None
        self.fail_read = False
        self.short_program = False
        self.bad_identity = False
        self.write_protected = False
        self.pending = None
        self.flushes = 0

    def flush(self):
        self.flushes += 1

    def write(self, packet):
        self.pending = packet
        self.commands.append(packet[2])
        return len(packet) - int(packet[2] == self.short_command)

    def recv(self, cmd, timeout):
        if cmd == self.fail_command:
            raise TimeoutError("Injected lost response")
        packet = self.pending
        if cmd == 0xA1:
            return 0, b"\x00\x00" if self.bad_identity else b"\x31\x19"
        if cmd == 0xA7:
            cfg = bytearray(b"\0" * 22)
            cfg[10:14] = b"\0" * 4 if self.write_protected else b"\xff" * 4
            cfg[-8:] = self.uid
            return 0, bytes(cfg)
        if cmd == 0xA3:
            return 0, bytes([sum(self.key) & 255])
        if cmd in (0xA5, 0xA6):
            address = int.from_bytes(packet[5:9], "little")
            plain = self.module.xor_crypt(packet[10:-1], self.key)
            override = self.status_responses.get((cmd, self.commands.count(cmd)))
            if cmd == 0xA6:
                mismatch = self.fail_verify or bytes(self.memory[address:address + len(plain)]) != plain
                return override or (0, b"\xf5\0" if mismatch else b"\0\0")
            if override and override != (0, b"\0\0"):
                return override
            for i, value in enumerate(plain):
                self.memory[address + i] &= value
            return 0, b"\0\0"
        raise AssertionError("Forbidden command")

    def read(self, count):
        if self.reads:
            return self.reads.pop(0)[:count]
        if self.fail_read:
            raise OSError("Injected capture disconnect")
        return b""


class ExportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_backend()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name).resolve()
        self.firmware = bytes(range(112))
        self.isp = Transport(self.module, self.firmware)
        self.receipt = {}

    def perform(self):
        # Clock advances only during capture; handshake sleeps are eliminated.
        ticks = iter((0, 0, 0, 0, 2))
        with patch.object(export.time, "sleep"), patch.object(export.time, "monotonic", side_effect=lambda: next(ticks)):
            export.prepare_and_capture(self.module, self.isp, self.firmware,
                                       self.output, 1, 536, self.receipt)

    def test_actual_codec_programs_only_marker_and_preserves_journal_and_control(self):
        self.perform()
        self.assertEqual(self.isp.memory[0xED00:0xED38], export.MARKER_ROW)
        self.assertEqual(self.isp.memory[:112], self.firmware)
        self.assertEqual(self.isp.memory[0xEE00:], self.isp.protected_before)
        first_program = self.isp.commands.index(0xA5)
        self.assertEqual(self.isp.commands[:first_program].count(0xA6), 2)
        self.assertEqual(self.isp.commands.count(0xA5), 2)
        self.assertNotIn(0xA4, self.isp.commands)
        self.assertNotIn(0xA8, self.isp.commands)
        self.assertEqual(self.isp.commands[-1], 0xA2)
        # Independent wire expectation: payload 01 requests reset; 00 merely
        # acknowledges END and was observed to leave only a nine-byte ISP ACK.
        self.assertEqual(self.isp.pending, bytes.fromhex("57aba2010001a4"))
        self.assertEqual((self.output / "raw.bin").read_bytes(), b"BFL2example")
        self.assertTrue(self.receipt["marker_verified"])
        self.assertEqual(self.isp.flushes, 2)  # Both before A2; never during capture.

    def test_wrong_installed_image_never_programs(self):
        self.isp.memory[90] ^= 1
        with self.assertRaises(export.FlasherError):
            self.perform()
        self.assertNotIn(0xA5, self.isp.commands)

    def test_failed_status_stops_before_later_program_or_application_start(self):
        for cmd, occurrence, payload, writes in ((0xA6, 1, b"\xf5\0", 0),
                                                 (0xA5, 1, b"\xfe\0", 1),
                                                 (0xA5, 2, b"", 2),
                                                 (0xA6, 3, b"\0", 2)):
            with self.subTest(cmd=cmd, occurrence=occurrence):
                self.isp = Transport(self.module, self.firmware)
                self.receipt = {}
                self.isp.status_responses[(cmd, occurrence)] = (0, payload)
                with self.assertRaises(export.FlasherError) as error:
                    self.perform()
                self.assertEqual(error.exception.code, "ISP_STATUS")
                self.assertEqual(self.isp.commands.count(0xA5), writes)
                self.assertEqual(self.isp.commands[-1], cmd)
                self.assertNotIn(0xA2, self.isp.commands)
                self.assertFalse(self.receipt.get("marker_verified", False))
                self.assertFalse((self.output / "raw.bin").exists())

    def test_a2_ack_and_application_bytes_remain_in_raw_capture(self):
        ack = bytes.fromhex("55aaa20002000000a4")
        self.isp.reads = [ack, b"BFL2example"]
        self.perform()
        self.assertEqual((self.output / "raw.bin").read_bytes(), ack + b"BFL2example")

    def test_short_writes_fail_without_retransmission(self):
        for cmd in (0xA1, 0xA3, 0xA5, 0xA6, 0xA7):
            with self.subTest(cmd=cmd):
                self.isp = Transport(self.module, self.firmware)
                self.isp.short_command = cmd
                with self.assertRaises(export.FlasherError):
                    self.perform()
                self.assertEqual(self.isp.commands.count(cmd), 1)
                if cmd != 0xA5:
                    self.assertNotIn(0xA5, self.isp.commands)

    def test_lost_program_response_leaves_durable_pending_intent(self):
        self.isp.fail_command = 0xA5
        with self.assertRaises(TimeoutError):
            self.perform()
        self.assertEqual(self.isp.commands.count(0xA5), 1)
        saved = json.loads((self.output / "isp-export.json").read_text())
        self.assertEqual(saved["stage"], "marker_write_may_be_pending")
        self.assertEqual(saved["verified_image_bytes"], 112)
        self.assertEqual(self.isp.flushes, 1)  # Never start app after uncertain program.

    def test_directory_sync_precedes_program_and_its_failure_suppresses_program(self):
        original_sync = export.sync_directory
        synced = []
        def sync(path):
            original_sync(path)
            synced.append(json.loads((path / "isp-export.json").read_text())["stage"])
        original_write = self.isp.write
        def write(packet):
            if packet[2] == 0xA5:
                self.assertIn("marker_write_may_be_pending", synced)
            return original_write(packet)
        with patch.object(export, "sync_directory", side_effect=sync), patch.object(self.isp, "write", side_effect=write):
            self.perform()
        (self.output / "raw.bin").unlink()
        self.isp = Transport(self.module, self.firmware)
        def fail_pending(path):
            stage = json.loads((path / "isp-export.json").read_text())["stage"]
            if stage == "marker_write_may_be_pending":
                raise OSError("Injected directory fsync failure")
            original_sync(path)
        with patch.object(export, "sync_directory", side_effect=fail_pending), self.assertRaises(OSError):
            self.perform()
        self.assertNotIn(0xA5, self.isp.commands)

    def test_marker_corruption_rejected_before_app_reset(self):
        self.isp.memory[0xED00] = 0
        with self.assertRaises(export.FlasherError):
            self.perform()
        self.assertNotEqual(self.receipt.get("marker_verified"), True)
        self.assertEqual(self.isp.flushes, 1)

    def test_partial_capture_survives_disconnect(self):
        self.isp.fail_read = True
        with self.assertRaises(OSError):
            self.perform()
        self.assertEqual((self.output / "raw.bin").read_bytes(), b"BFL2example")
        self.assertEqual(self.receipt["raw_bytes"], 11)
        self.assertNotEqual(self.receipt.get("capture_finished"), True)

    def test_wrong_target_and_write_protection_fail_before_program(self):
        for field in ("bad_identity", "write_protected"):
            self.isp = Transport(self.module, self.firmware)
            setattr(self.isp, field, True)
            with self.assertRaises(export.FlasherError):
                self.perform()
            self.assertNotIn(0xA5, self.isp.commands)

    def test_command_boundary_rejects_erase_options_and_nonmarker_addresses(self):
        for packet, cmd in ((self.module.build_erase(64), 0xA4),
                            (self.module.build_write_cfg(7, b"\xff" * 12), 0xA8),
                            (self.module.build_program(0xEE00, 0, b"\xff" * 56), 0xA5),
                            (self.module.build_program(0xED00, 0, b"\xff" * 57), 0xA5)):
            with self.assertRaises(export.FlasherError):
                export.command(self.isp, packet, cmd)
        self.assertEqual(self.isp.commands, [])

    def test_oversize_image_rejected_before_isp(self):
        self.firmware = b"\xff" * (0xED00 + 1)
        with self.assertRaises(export.FlasherError):
            self.perform()
        self.assertEqual(self.isp.commands, [])

    def test_run_uses_one_serial_open_and_closes_without_line_rewrite(self):
        firmware = self.output / "firmware.bin"
        firmware.write_bytes(self.firmware)
        sha = hashlib.sha256(self.firmware).hexdigest()
        args = SimpleNamespace(firmware=str(firmware), sha256=sha, port="/dev/mock",
                               output=str(self.output / "captured"), isolated_usb_only=True,
                               seconds=1, max_bytes=536, resume_pending=False)
        self.isp.close = unittest.mock.Mock()
        self.isp.fileno = lambda: 123
        ticks = iter((0, 0, 0, 0, 2))
        with ExitStack() as stack:
            stack.enter_context(patch.object(export, "PINNED_IMAGE_SHA256", sha))
            stack.enter_context(patch.object(export, "load_backend", return_value=self.module))
            stack.enter_context(patch.object(export, "preflight", return_value=[]))
            stack.enter_context(patch.object(export, "devices", return_value=[dict(port=args.port)]))
            stack.enter_context(patch.object(export.FlasherService, "_lock", return_value=None))
            stack.enter_context(patch.object(self.module, "WchIsp", return_value=self.isp))
            opened = stack.enter_context(patch.object(self.module.serial, "Serial", return_value=self.isp))
            stack.enter_context(patch("fcntl.ioctl"))
            stack.enter_context(patch.object(export.time, "sleep"))
            stack.enter_context(patch.object(export.time, "monotonic", side_effect=lambda: next(ticks)))
            self.assertEqual(export.run(args), 0)
        opened.assert_called_once()
        self.isp.close.assert_called_once()
        self.assertEqual((self.isp.dtr, self.isp.rts), (True, False))
        receipt = json.loads((Path(args.output) / "isp-export.json").read_text())
        self.assertEqual(receipt["format"], 2)
        self.assertIs(receipt["status_payload_checked"], True)
        self.assertTrue(receipt["port_closed"])
        self.assertEqual(receipt["serial_openings"], 1)

    def test_owner_isolation_and_pinned_image_are_required_before_any_open(self):
        firmware = self.output / "firmware.bin"
        firmware.write_bytes(self.firmware)
        args = SimpleNamespace(firmware=str(firmware), sha256=hashlib.sha256(self.firmware).hexdigest(),
                               port="/dev/mock", output=str(self.output / "forbidden"),
                               isolated_usb_only=False, seconds=1, max_bytes=536, resume_pending=False)
        with patch.object(export, "load_backend") as backend:
            with self.assertRaises(export.FlasherError):
                export.run(args)
            args.isolated_usb_only = True
            with self.assertRaises(export.FlasherError):
                export.run(args)
            backend.assert_not_called()
        self.assertFalse(Path(args.output).exists())

    def test_existing_marker_resume_has_no_program_and_absence_prevents_a2(self):
        self.isp.memory[0xED00:0xED38] = export.MARKER_ROW
        ticks = iter((0, 0, 0, 0, 2))
        with patch.object(export.time, "sleep"), patch.object(export.time, "monotonic", side_effect=lambda: next(ticks)):
            export.prepare_and_capture(self.module, self.isp, self.firmware,
                                       self.output, 1, 536, self.receipt, resume_pending=True)
        self.assertNotIn(0xA5, self.isp.commands)
        self.assertEqual(self.isp.commands[-1], 0xA2)
        self.assertEqual(self.isp.pending, bytes.fromhex("57aba2010001a4"))
        self.assertEqual(self.isp.memory[0xEE00:], self.isp.protected_before)
        self.isp = Transport(self.module, self.firmware)
        with patch.object(export.time, "sleep"), self.assertRaises(export.FlasherError):
            export.prepare_and_capture(self.module, self.isp, self.firmware,
                                       self.output, 1, 536, {}, resume_pending=True)
        self.assertNotIn(0xA5, self.isp.commands)
        self.assertNotIn(0xA2, self.isp.commands)

    def test_short_application_start_write_is_not_retried(self):
        self.isp.short_command = 0xA2
        with self.assertRaises(export.FlasherError):
            self.perform()
        self.assertEqual(self.isp.commands.count(0xA2), 1)
        self.assertFalse((self.output / "raw.bin").exists())

    def test_boot_release_must_leave_rom_responsive_before_application_start(self):
        original_recv = self.isp.recv
        def recv(cmd, timeout):
            if cmd == 0xA1 and self.isp.dtr is True:
                raise TimeoutError("ROM not responsive after BOOT release")
            return original_recv(cmd, timeout)
        with patch.object(self.isp, "recv", side_effect=recv), self.assertRaises(TimeoutError):
            self.perform()
        self.assertNotIn(0xA2, self.isp.commands)
        self.assertTrue(self.receipt["marker_verified"])


if __name__ == "__main__":
    unittest.main()
