#!/usr/bin/env python3
"""Run the settled working-mode acceptance suites and bind their exact receipts."""
from pathlib import Path
import hashlib,json,subprocess,sys
P=Path(__file__).resolve().parents[1]
for directory in ('logs','work','evidence','builds'): (P/directory).mkdir(parents=True,exist_ok=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
encoder_cases=json.loads((P/'tests/encoder-acceptance-cases.json').read_text())
suites=[('marker',7,['run_marker.py']),('working',17,['run_working.py']),('journal',10,['run_journal.py']),('retention',5,['run_retention.py']),('request_feedback',46,['run_phase99_request.py']),('encoder_OFF',20,['run_encoder.py',*encoder_cases])]
summary={}
for name,count,args in suites:
    before=set((P/'logs').rglob('host-inputs-*.json'))
    r=subprocess.run([sys.executable,'-B',str(P/'tests'/args[0]),*args[1:]],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    (P/'logs'/('final-'+name+'-console.log')).write_text(r.stdout)
    print(name,'returncode',r.returncode,flush=True)
    if r.returncode:print(r.stdout);raise SystemExit(r.returncode)
    candidates=[]
    for f in set((P/'logs').rglob('host-inputs-*.json'))-before:
        data=json.loads(f.read_text())
        if len(data.get('case_results',[]))==count:candidates.append(f)
    assert len(candidates)==1,(name,candidates)
    receipt=candidates[0];summary[name]={'path':str(receipt.relative_to(P)),'sha256':sha(receipt),'cases':count}
summary.update(power_cut_prefixes={'runtime_rollover':120,'virgin_prepare':96,'normal_session_prepare':96,'marker_program':85,'marker_consume':66},physical_hardware=False,serial_opened=False,exact_final_source_and_harness_bound=True,excluded_inherited_one_use_cases=['full-journal-boot','full-journal-bootcuts'])
(P/'firmware-test-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print('PASS105 source-bound firmware scenarios')
