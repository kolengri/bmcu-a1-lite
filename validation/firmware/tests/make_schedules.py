"""Declare deterministic integer raw-DMA schedules, independent of rendered colors."""
import json
from pathlib import Path
cases={}
def add(name,segments=(),parent=0,detail=0,ack=True,**kw):
 cases['cue-'+name]=dict(baseline_raw=[2048]*4,segments_ms_channel_raw=list(segments),parent=parent,detail=detail,ack=ack,fault='',fault_stage='',**kw)
add('zero',parent=9,detail=1,ack=False)
add('park-reentry',ack=False)
add('subthreshold',[(300,30000,2,2172)],parent=9,detail=1,ack=False)
add('short',[(300,600,2,1799)],parent=9,detail=2,ack=False)
add('held',[(300,30000,2,1799)],parent=9,detail=3)
for ch in range(4):add('valid-'+str(ch),[(300,2800,ch,1799 if ch%2==0 else 2296)])
add('quick-release',[(300,2350,2,1799)])
add('late-release',[(9500,13500,2,1799)])
add('delayed-admission',[(300,2800,2,1799)])
add('offset-baselines',[(300,2800,2,1801)]);cases['cue-offset-baselines']['baseline_raw']=[1990,2020,2050,2080]
add('candidate-switch',[(300,600,2,1799),(1500,30000,1,1799)],parent=6,ack=False)
add('competition',[(300,30000,2,1799),(300,30000,1,1799)],parent=6,ack=False)
add('queued-fault',[(300,30000,2,1799)],parent=5);cases['cue-queued-fault'].update(fault='key',fault_stage='ack')
for stage in ['baseline','window','ack']:
 for why,parent in [('adc',6),('key',5),('storage',1),('clock',8)]:
  add(stage+'-'+why,[(300,30000,2,1799)],parent=parent,ack=stage=='ack')
  cases['cue-'+stage+'-'+why].update(fault=why,fault_stage=stage)
Path(__file__).with_name('cue-schedules.json').write_text(json.dumps(cases,indent=2,sort_keys=True)+'\n')
