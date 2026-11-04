"""0403 raw transport evidence and real offline C decoding; never serial I/O."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import workflow as w


class IspWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name)
        self.selected = w.profile("260b0403")
        self.binary = w.checked_collector(self.selected)
        self.frame = w.ROOT / "validation/firmware/examples/marker-admission.frame"
        self.raw = self.frame.read_bytes()
        result = subprocess.run([self.binary, "--input", str(self.frame),
                                 "--output", str(self.output / "capture")], capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def transport_evidence(self):
        transport = self.output / "isp"
        transport.mkdir()
        (transport / "raw.bin").write_bytes(self.raw)
        (transport / "marker.bin").write_bytes(w.ISP_MARKER)
        self.receipt = dict(format=2, build="260b0403", isolated_usb_only=True,
                            firmware_sha256=self.selected["firmware_sha256"], marker_address=0xED00,
                            marker_hex=w.ISP_MARKER.hex(),
                            verified_image_bytes=self.selected["firmware_padded_bytes"],
                            marker_verified=True, capture_finished=True, raw_bytes=len(self.raw),
                            raw_sha256=hashlib.sha256(self.raw).hexdigest(), port_closed=True,
                            stage="captured_unvalidated", success=True, baud=115200, parity="none", serial_openings=1,
                            operation="prepare", start_command_hex="57aba2010001a4", boot_released_before_start=True,
                            status_payload_checked=True)
        self.receipt_path = transport / "isp-export.json"
        self.receipt_path.write_text(json.dumps(self.receipt))

    def test_actual_firmware_virgin_frame_decodes_offline_without_claiming_live_access(self):
        receipt = w.verify_capture(self.output, self.selected)
        self.assertTrue(receipt["journal_blank"])
        self.assertEqual(receipt["export_status"], "not_requested")
        self.assertFalse(receipt["payload_facts_available"])
        metadata = json.loads((self.output / "capture/metadata.json").read_text())
        self.assertEqual(metadata["mode"], "offline")
        self.assertFalse(metadata["frame_timestamps_available"])

    def test_pinned_transport_evidence_and_valid_frame_confirm_marker_consumption(self):
        self.transport_evidence()
        receipt = w.verify_capture(self.output, self.selected)
        self.assertEqual(receipt["export_status"], "marker_consumption_confirmed_by_valid_frame")
        for change in ({"serial_openings": 2}, {"marker_verified": False}, {"port_closed": False},
                       {"firmware_sha256": "wrong"}, {"marker_address": 0xEE00},
                       {"raw_sha256": "wrong"}, {"success": 1}, {"stage": "starting_export"},
                       {"operation": "unverified_resume"}, {"start_command_hex": "wrong"},
                       {"boot_released_before_start": False}, {"format": 1},
                       {"status_payload_checked": False}):
            self.receipt_path.write_text(json.dumps(dict(self.receipt, **change)))
            with self.subTest(change=change), self.assertRaises(w.WorkflowError):
                w.verify_capture(self.output, self.selected)

    def test_raw_or_marker_substitution_is_rejected(self):
        self.transport_evidence()
        for name in ("raw.bin", "marker.bin"):
            path = self.output / "isp" / name
            original = path.read_bytes()
            path.write_bytes(original + b"\0")
            with self.assertRaises(w.WorkflowError):
                w.verify_capture(self.output, self.selected)
            path.write_bytes(original)

    def test_live_context_cannot_be_accepted_as_offline_without_transport(self):
        (self.output / "context.json").write_text(json.dumps(dict(
            build="260b0403", rearm=False, owner_isolated_usb_only_assertion=True)))
        with self.assertRaisesRegex(w.WorkflowError, "lacks ISP"):
            w.verify_capture(self.output, self.selected)

    def test_decoder_live_flags_and_devices_reject_before_open(self):
        for args in (("--port", "/dev/does-not-exist"), ("--input", "/dev/null"),
                     ("--rearm",), ("--boot-export",)):
            target = self.output / "rejected"
            result = subprocess.run([self.binary, *args, "--output", str(target)], capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(target.exists())


if __name__ == "__main__":
    unittest.main()
