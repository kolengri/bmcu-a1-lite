#!/usr/bin/env python3
"""Run actual-firmware RX queue cases through the shared source-bound runner."""
import argparse
from pathlib import Path
import subprocess
import sys

PACKET = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source', type=Path, default=PACKET.parents[1])
parser.add_argument('--logs', type=Path, default=PACKET/'logs/queue')
parser.add_argument('cases', nargs='*')
args = parser.parse_args()
flags = '--full --features --authority --observation --idle04 --flight --gesture-1s --feed-latch --phase-compat --input-detail --meter-origin --global-phase --release-cue --unload-continue --phase99 --phase99-03 --phase99-all-slots --continuous --onuse-idle --unload-diagnostic --lease-feedback --automatic-capture --rx-backpressure --precut-phase --encoder-recovery'.split()
cases = args.cases or [
    *['full-rx-burst-owner%d%s' % (owner, '-sender04' if command == 4 else '')
      for owner in (0, 2, 3) for command in (3, 4)],
    'full-rx-fragment', 'full-rx-fragment-owner3-sender04',
    'full-rx-pull-first', 'full-rx-pull-first-owner0-sender04',
    'full-rx-invalid-snapshot', 'full-rx-overflow',
    'full-rx-busy-wrap-owner3-sender04',
]
command = [
    sys.executable, '-B', str(PACKET/'tests/check_setup.py'),
    str(args.source),
    '--fixtures', str(PACKET/'tests'), '--logs', str(args.logs),
    *flags, '--keep-going', '--cases', *cases,
]
raise SystemExit(subprocess.run(command, check=False).returncode)
