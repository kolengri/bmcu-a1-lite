#!/usr/bin/env python3
"""Validate opt-in boundaries using actual profile headers and accepted defines."""
from pathlib import Path
import argparse,hashlib,json,subprocess
P=Path(__file__).resolve().parents[1];src=P.parents[1]/'src'
for directory in ('logs','work','evidence','builds'): (P/directory).mkdir(parents=True,exist_ok=True)
p=argparse.ArgumentParser();p.add_argument('--receipt',type=Path,required=True);args=p.parse_args()
input_receipt=args.receipt if args.receipt.is_absolute() else P/args.receipt
receipt=json.loads(input_receipt.read_text())
flags=receipt['defines']
def check(defines):
    return subprocess.run(['c++','-fsyntax-only','-x','c++','-I',str(src),*['-D'+f for f in defines],'-'],input='#include "lite_profile.h"\n',text=True,capture_output=True)
assert check(flags).returncode==0
default=subprocess.run(['c++','-E','-dM','-x','c++','-I',str(src),'-'],input='#include "lite_profile.h"\n',text=True,capture_output=True,check=True).stdout
assert '#define BMCU_USB_CONTINUOUS_JOURNAL 0' in default
assert '#define BMCU_USB_WORKING_MODE 0' in default
assert '#define BMCU_USB_ISP_EXPORT 0' in default
assert '#define BMCU_SETUP_INSERTION_RECOVERY 0' in default
for option in ['BMCU_SETUP_INSERTION_RECOVERY=2','BMCU_USB_ISP_EXPORT=0','BMCU_USB_ISP_EXPORT=2','BMCU_USB_WORKING_MODE=0','BMCU_USB_WORKING_MODE=2','BMCU_USB_CONTINUOUS_JOURNAL=2','BMCU_LITE_ENCODER_RECOVERY=0','BMCU_USB_FLIGHT_RECORDER=0','BMCU_LITE_FULL_LOAD=0','BMCU_LITE_PHASE99_REQUEST_FEEDBACK=2','BMCU_LITE_PHASE99_REQUEST_FEEDBACK=0','BMCU_USB_CONTINUOUS_JOURNAL=0','BMCU_LITE_PHASE99_ALL_SLOTS=0']:
    key=option.split('=')[0];invalid=[f for f in flags if not f.startswith(key+'=')]+[option]
    assert check(invalid).returncode!=0,option
assert 'default_envs = USB_SETUP_INSERTION_RECOVERY' in (P.parents[1]/'platformio.ini').read_text()
(P/'flag-receipt.json').write_text(json.dumps(dict(default_off=True,default_profile='USB_SETUP_INSERTION_RECOVERY',valid_profile=True,invalid_profiles_rejected=13,input_receipt=str(input_receipt.relative_to(P)),input_receipt_sha256=hashlib.sha256(input_receipt.read_bytes()).hexdigest(),header_sha256=hashlib.sha256((src/'lite_profile.h').read_bytes()).hexdigest()),indent=2)+'\n')
print('PASS strict defaultOFF and thirteen incompatible profile selections')
