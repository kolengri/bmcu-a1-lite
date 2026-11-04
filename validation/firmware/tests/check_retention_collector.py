#!/usr/bin/env python3
"""Offline pair integrity against the built C collector; no serial access."""
from pathlib import Path
import hashlib,json,struct,subprocess,tempfile,zlib
P=Path(__file__).resolve().parents[1];BUILD=0x260b0403
for directory in ('logs','work','evidence','builds'): (P/directory).mkdir(parents=True,exist_ok=True)
get=lambda b,o=0:struct.unpack_from('<I',b,o)[0]
def put(b,o,v):struct.pack_into('<I',b,o,v)
def record_crc(b,g):return zlib.crc32(b, g^0xffffffff)&0xffffffff
def header(g,s):
    b=bytearray(b'\xff'*256);struct.pack_into('<4I',b,0,0x324a4642,g,s,BUILD);put(b,16,zlib.crc32(b[:16]));return b
def outer(raw):
    a,b=raw[:256],raw[256:];ga,gb=get(a,4),get(b,4);latest=b if gb>ga else a
    flags=7|(16 if get(latest,4)>2 else 0)
    f=bytearray(b'BFL2'+struct.pack('<4I',(512<<16)|2,BUILD,get(latest,8),flags)+raw)
    f+=struct.pack('<I',zlib.crc32(f));return f
results=[]
with tempfile.TemporaryDirectory(prefix='collector-checks-',dir=P/'work') as tmp:
    root=Path(tmp)
    def decode(name,data):
        source=root/(name+'.frame');source.write_bytes(data);out=root/name
        r=subprocess.run([str(P/'builds/bmcu-usb-logs'),'--input',str(source),'--output',str(out)],capture_output=True,text=True)
        assert r.returncode==0,(name,r.stdout,r.stderr)
        return json.loads((out/'report.json').read_text())
    examples=P/'examples'
    for name in ['full-retention-pair-boundary','full-retention-pair-wrap','full-retention-pair-crosspage']:
        r=decode(name,(examples/(name+'.frame')).read_bytes())
        tx=[e for e in r['journal']['events'] if e['kind'] in [16,17]]
        assert tx and all(e['submitted_response']['meter_record_committed'] for e in tx),name
        results.append(name)
    sample=(examples/'full-retention-pair-boundary.frame').read_bytes();tx=bytearray(sample[52:68]);meter=bytearray(sample[68:84])
    assert tx[8] in [16,17] and meter[8]==18
    def cross(name,g=2,s=1,timestamp=None,channel=None,dropped=0,prefix=16,wrong_seed=False):
        a,b=header(1,1),header(g,s);t=bytearray(tx);m=bytearray(meter)
        if timestamp is not None:put(m,0,timestamp)
        if channel is not None:m[9]=channel
        struct.pack_into('<H',m,10,dropped)
        put(t,12,record_crc(t[:12],1));put(m,12,record_crc(m[:12],1 if wrong_seed else g))
        a[240:256]=t;b[32:32+prefix]=m[:prefix]
        report=decode(name,outer(a+b));event=next(e for e in report['journal']['events'] if e['kind'] in [16,17])
        return event['submitted_response']['meter_record_committed']
    assert cross('consecutive-valid');results.append('consecutive-valid')
    for name,kw in [('generation-gap',dict(g=4)),('session-mix',dict(s=2)),('timestamp-mismatch',dict(timestamp=get(tx)+1)),('channel-mismatch',dict(channel=3)),('dropped-meter',dict(dropped=1)),('generation-seed-mismatch',dict(wrong_seed=True))]:
        assert not cross(name,**kw),name;results.append(name)
    for prefix in range(16):
        name='torn-meter-prefix-'+str(prefix);assert not cross(name,prefix=prefix),name;results.append(name)
    offline=subprocess.run([str(P/'builds/bmcu-usb-logs'),'--input',str(examples/'full-retention-scan.frame'),'--output',str(root/'forbidden-rearm'),'--rearm'],capture_output=True,text=True)
    assert offline.returncode==2;results.append('offline-rearm-rejected')
(P/'evidence/collector-checks.json').write_text(json.dumps({'passed':results,'count':len(results),'collector_sha256':hashlib.sha256((P/'builds/bmcu-usb-logs').read_bytes()).hexdigest(),'serial_opened':False},indent=2)+'\n')
print('PASS',len(results),'offline C collector pair/binding checks')
