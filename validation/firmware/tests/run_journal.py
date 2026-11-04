#!/usr/bin/env python3
"""Source-bound deterministic firmware tests; no hardware access."""
from pathlib import Path
import subprocess,sys
P=Path(__file__).resolve().parents[1]
for directory in ('logs','work','evidence','builds'): (P/directory).mkdir(parents=True,exist_ok=True)
flags='--full --features --authority --observation --idle04 --flight --gesture-1s --feed-latch --phase-compat --input-detail --meter-origin --global-phase --release-cue --unload-continue --phase99 --phase99-03 --phase99-all-slots --continuous --onuse-idle --unload-diagnostic --lease-feedback --automatic-capture --rx-backpressure --precut-phase --encoder-recovery --continuous-journal --retention --phase99-request-feedback --working-mode --isp-export --setup-insertion-recovery'.split()
cases=sys.argv[1:] or ['full-journal-'+s for s in ['run','fault','setup-fault','rollover','powercuts','loss','wrap','busy','diagnostic-error','contention']]
n=1
while (out:=P/'logs'/('host-journal-'+str(n))).exists():n+=1
out.mkdir()
cmd=[sys.executable,'-B',str(P/'tests/check_setup.py'),str(P.parents[1]),'--fixtures',str(P/'tests'),'--logs',str(out),*flags,'--keep-going','--cases',*cases]
with (out/'console.log').open('w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
print(out);print('\n'.join((out/'console.log').read_text().splitlines()[-50:]));raise SystemExit(r.returncode)
