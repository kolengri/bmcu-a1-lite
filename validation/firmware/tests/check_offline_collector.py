#!/usr/bin/env python3
"""Actual0403 C offline contract, input types and forbidden live options."""
from pathlib import Path
import hashlib,json,os,struct,subprocess,tempfile,zlib
P=Path(__file__).resolve().parents[1];collector=P/'builds/bmcu-usb-logs';fixture=P/'examples/working-export-saved.frame'
for directory in ('logs','work','evidence','builds'): (P/directory).mkdir(parents=True,exist_ok=True)
checks=[]
with tempfile.TemporaryDirectory(prefix='offline-check-',dir=P/'work') as tmp:
    tmp=Path(tmp)
    def run(name,source=fixture,extra=()):
        out=tmp/("out-"+name);r=subprocess.run([str(collector),'--input',str(source),'--output',str(out),*extra],capture_output=True,text=True)
        return r,out
    for name,flags in [('port',['--port','/dev/cu.MOCK']),('rearm',['--rearm']),('boot-export',['--boot-export']),('observe',['--observe-stream']),('seconds',['--seconds','1']),('small-limit',['--max-bytes','535'])]:
        r,out=run(name,extra=flags);assert r.returncode==2 and not out.exists(),(name,r.stdout,r.stderr);checks.append('reject-'+name)
    fifo=tmp/'fifo';os.mkfifo(fifo);link=tmp/'symlink';link.symlink_to(fixture.resolve())
    for name,source in [('device',Path('/dev/null')),('fifo',fifo),('symlink',link),('directory',tmp)]:
        r,out=run(name,source);assert r.returncode!=0 and not out.exists();checks.append('reject-'+name)
    for name in ['working-export-virgin','working-export-saved','marker-admission','marker-quiesce']:
        source=P/'examples'/(name+'.frame');r,out=run(name,source);assert r.returncode==0,(name,r.stdout,r.stderr)
        report=json.loads((out/'report.json').read_text());meta=json.loads((out/'metadata.json').read_text())
        assert report['build']=='260b0403' and not report['complete'] and report['transport_valid']
        assert meta['mode']=='offline' and not meta['export_requested'] and meta['export_request_sent']==0 and meta['export_mode']=='offline' and not meta['capture_failed']
        assert not (out/'export-request.bin').exists() and not (out/'export-result.json').exists() and (out/'raw.bin').read_bytes()==source.read_bytes();checks.append('decode-'+name)
    sample=bytearray(fixture.read_bytes());struct.pack_into('<I',sample,8,0x260b0402);struct.pack_into('<I',sample,532,zlib.crc32(sample[:532]));old=tmp/'old.frame';old.write_bytes(sample)
    r,out=run('old-build',old);assert r.returncode!=0;checks.append('reject-old-build')
    imports=subprocess.check_output(['nm','-u',str(collector)],text=True)
    assert all(symbol not in imports for symbol in ['_tcsetattr','_tcdrain','_ioctl','_poll','_TIOCM']);checks.append('no-live-serial-imports')
    expected=bytes.fromhex('4246493303040b260100010062b34213');assert (P/'examples/isp-marker.bin').read_bytes()==expected and (P/'examples/isp-marker-row.bin').read_bytes()==expected+b'\xff'*40;checks.append('actual-firmware-marker-contract')
assert len(checks)==17
(P/'evidence/collector-offline-checks.json').write_text(json.dumps({'checks':checks,'count':len(checks),'collector_sha256':hashlib.sha256(collector.read_bytes()).hexdigest(),'test_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'source_inputs':{str(f.relative_to(P)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted((P/'collector').iterdir()) if f.is_file()},'serial_opened':False},indent=2)+'\n')
print('PASS17 actualC offline-only/build/input/marker checks; no serial')
