#!/usr/bin/env python3
"""Run actual driver/ACK/raw-range/controller bodies with modeled SDA samples."""
from pathlib import Path
import argparse,subprocess,sys
P=Path(__file__).resolve().parents[1]
for directory in ('logs','work','evidence','builds'): (P/directory).mkdir(parents=True,exist_ok=True)
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=P.parents[1]);p.add_argument('--parent',action='store_true');p.add_argument('cases',nargs='*');a=p.parse_args()
flags='--full --features --authority --observation --idle04 --flight --gesture-1s --feed-latch --phase-compat --input-detail --meter-origin --global-phase --release-cue --unload-continue --phase99 --phase99-03 --phase99-all-slots --continuous --onuse-idle --unload-diagnostic --lease-feedback --automatic-capture --rx-backpressure --precut-phase --encoder-driver'.split()
if not a.parent:flags.append('--encoder-recovery')
cases=a.cases or ['full-encoder-transient-owner%d-sender%02d%s'%(owner,cmd,suffix) for owner in range(4) for cmd in [3,4] for suffix in ['', '-other']]+['full-encoder-persistent','full-encoder-persistent-other','full-encoder-transient-raw','full-encoder-persistent-raw','full-encoder-magnet','full-encoder-transient-lease','full-encoder-transient-deadline','full-encoder-transient-clock','full-encoder-transient-flash','full-encoder-transient-wrap','full-encoder-transient-odometry-complete','full-encoder-driver-equivalence']+['full-encoder-priority-s%u'%source for source in range(1,10)]+['full-encoder-persistent-alternating-owner%d-sender%02d%s'%(owner,cmd,suffix) for owner in range(4) for cmd in [3,4] for suffix in ['', '-other']]
n=1
while (logs:=P/'logs'/('encoder-%s-%d'%('parent' if a.parent else 'candidate',n))).exists():n+=1
cmd=[sys.executable,'-B',str(P/'tests/check_setup.py'),str(a.source),'--fixtures',str(P/'tests'),'--logs',str(logs),*flags,'--keep-going','--cases',*cases]
raise SystemExit(subprocess.run(cmd).returncode)
