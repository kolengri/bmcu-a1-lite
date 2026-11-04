#!/usr/bin/env python3
"""Actual-source retention regressions; only modeled hardware, never serial."""
from pathlib import Path
import argparse,subprocess,sys
P=Path(__file__).resolve().parents[1]
for directory in ('logs','work','evidence','builds'): (P/directory).mkdir(parents=True,exist_ok=True)
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=P.parents[1]);p.add_argument('--parent',action='store_true');p.add_argument('cases',nargs='*');a=p.parse_args()
flags='--full --features --authority --observation --idle04 --flight --gesture-1s --feed-latch --phase-compat --input-detail --meter-origin --global-phase --release-cue --unload-continue --phase99 --phase99-03 --phase99-all-slots --continuous --onuse-idle --unload-diagnostic --lease-feedback --automatic-capture --rx-backpressure --precut-phase --encoder-recovery --continuous-journal'.split()
if not a.parent:flags+=['--retention','--phase99-request-feedback','--working-mode','--isp-export']
cases=a.cases or ['full-retention-scan','full-retention-pair-boundary','full-retention-pair-wrap','full-retention-pair-crosspage','full-retention-consumed-pull']
n=1
while (out:=P/'logs'/('retention-'+('parent' if a.parent else 'candidate')+'-'+str(n))).exists():n+=1
out.mkdir()
cmd=[sys.executable,'-B',str(P/'tests/check_setup.py'),str(a.source),'--fixtures',str(P/'tests'),'--logs',str(out),*flags,'--keep-going','--cases',*cases]
with (out/'console.log').open('w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
print(out);print('\n'.join((out/'console.log').read_text().splitlines()[-40:]));raise SystemExit(r.returncode)
