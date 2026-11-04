#!/usr/bin/env python3
from pathlib import Path
import subprocess,hashlib,json,platform,shutil
P=Path(__file__).resolve().parents[1];R=P.parents[2];S=P/'collector/collector.c'
for directory in ('logs','work','evidence','builds'): (P/directory).mkdir(parents=True,exist_ok=True)
cmd=['cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-mmacosx-version-min=11.0','-arch','arm64','-arch','x86_64',str(S),'-o',str(P/'builds/bmcu-usb-logs')]
r=subprocess.run(cmd,capture_output=True,text=True);n=1
while (P/'logs'/f'collector-build-{n}.json').exists():n+=1
(P/'logs'/f'collector-build-{n}.log').write_text(r.stdout+r.stderr)
(P/'logs'/f'collector-build-{n}.json').write_text(json.dumps({'argv':cmd,'returncode':r.returncode,'source_sha256':hashlib.sha256(S.read_bytes()).hexdigest(),'decoder_sha256':hashlib.sha256((P/'collector/journal_decode.inc').read_bytes()).hexdigest(),'source_inputs':{str(f.relative_to(P)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted((P/'collector').iterdir()) if f.is_file()},'cc_path':shutil.which('cc'),'cc_sha256':hashlib.sha256(Path(shutil.which('cc')).resolve().read_bytes()).hexdigest(),'host':platform.platform()},indent=2)+'\n')
print(r.stdout+r.stderr);r.check_returncode()
print(subprocess.check_output(['file',str(P/'builds/bmcu-usb-logs')],text=True))
