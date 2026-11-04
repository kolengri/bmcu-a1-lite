#!/usr/bin/env python3
"""Execute the bounded99 wire hypothesis through actual CRC/main/DMA bodies."""
from pathlib import Path
import argparse,subprocess,sys
P=Path(__file__).resolve().parents[1]
for directory in ('logs','work','evidence','builds'): (P/directory).mkdir(parents=True,exist_ok=True)
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=P.parents[1]);p.add_argument('--parent',action='store_true');p.add_argument('cases',nargs='*');a=p.parse_args()
flags='--full --features --authority --observation --idle04 --flight --gesture-1s --feed-latch --phase-compat --input-detail --meter-origin --global-phase --release-cue --unload-continue --phase99 --phase99-03 --phase99-all-slots --continuous --onuse-idle --unload-diagnostic --lease-feedback --automatic-capture --rx-backpressure --precut-phase --encoder-recovery --continuous-journal --retention'.split()
if not a.parent:flags+=['--phase99-request-feedback','--working-mode','--isp-export','--setup-insertion-recovery']
cases=a.cases or ['full-request99-owner%d-slot%d-sender%02d'%(owner,slot,cmd) for owner in range(4) for slot in range(4) for cmd in [3,4]]+['full-request99-%s-sender%02d'%(boundary,cmd) for boundary in ['zero','empty','lease','fault','ownership','global','busy'] for cmd in [3,4]]
n=1
while True:
    logs=P/'logs'/('request99-%s-%d'%('parent' if a.parent else 'candidate',n))
    try:logs.mkdir();break
    except FileExistsError:n+=1
command=[sys.executable,'-B',str(P/'tests/check_setup.py'),str(a.source),'--fixtures',str(P/'tests'),'--logs',str(logs),*flags,'--keep-going','--cases',*cases]
raise SystemExit(subprocess.run(command).returncode)
