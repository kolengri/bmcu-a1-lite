"""The documented default command must allocate output in a fresh checkout."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import workflow


class PublicOutputTests(unittest.TestCase):
    def test_shipped_parent_allows_default_capture_without_hardware(self):
        parent = workflow.ROOT / "artifacts/bmcu-a1"
        self.assertTrue((parent / "README.md").is_file())
        capture = workflow.allocate()
        try:
            self.assertEqual(capture.parent, parent)
            self.assertTrue(capture.is_dir())
            self.assertEqual(list(capture.iterdir()), [])
        finally:
            capture.rmdir()
