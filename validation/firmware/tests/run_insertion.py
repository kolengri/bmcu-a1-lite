#!/usr/bin/env python3
"""Actual setup/Flash/controller insertion regressions; no hardware or network."""
from pathlib import Path
import argparse,json,subprocess,sys
P=Path(__file__).resolve().parents[1]
for directory in ('logs','work','evidence','builds'): (P/directory).mkdir(parents=True,exist_ok=True)
p=argparse.ArgumentParser();p.add_argument('--parent',action='store_true');p.add_argument('cases',nargs='*');a=p.parse_args()
flags='--full --features --authority --observation --idle04 --flight --gesture-1s --feed-latch --phase-compat --input-detail --meter-origin --global-phase --release-cue --unload-continue --phase99 --phase99-03 --phase99-all-slots --continuous --onuse-idle --unload-diagnostic --lease-feedback --automatic-capture --rx-backpressure --precut-phase --encoder-recovery --continuous-journal --retention --phase99-request-feedback --working-mode --isp-export'.split()
if not a.parent:flags+=['--setup-insertion-recovery']
cases=a.cases or ['continuous-fresh-insertion-'+stage+'-'+str(ch) for stage in ['postcue','postcal','direction','journalsettle'] for ch in range(4)]+['continuous-fresh-insertion-'+stage+'-0' for stage in ['inspect','gesture','cal-sample','calwrite','motionwrite','journalcommit']]+['continuous-fresh-insertion-postcue-'+suffix+'-0' for suffix in ['deflected','fault-adc','fault-encoder','fault-clock','fault-storage','localfault']]
source=P.parents[1] if not a.parent else P/'work/base0403'
n=1
while (out:=P/'logs'/('insertion-%s-%d'%('parent' if a.parent else 'candidate',n))).exists():n+=1
out.mkdir(parents=True)
cmd=[sys.executable,'-B',str(P/'tests/check_setup.py'),str(source),'--fixtures',str(P/'tests'),'--logs',str(out),*flags,'--keep-going','--cases',*cases]
with (out/'console.log').open('w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
print(out);print('\n'.join((out/'console.log').read_text().splitlines()[-75:]));raise SystemExit(r.returncode)
