"""Apply observer-only extensions to the copied accepted0009 harness once."""
from pathlib import Path
p=Path(__file__).with_name('setup_test.cpp');s=p.read_text()
s=s.replace('#include "setup_trial.cpp"','static void host_center_observe(unsigned,float);\nstatic void host_window_observe(uint64_t);\nstatic void host_gesture_sample();\nstatic void host_release_observe();\nstatic void host_fault_observe();\n#include "setup_trial.cpp"')
s=s.replace('static void input(){','#include "cue_test.inc"\nstatic void input(){')
s=s.replace('}else if(!is("gesture-timeout")){','}else if(!cue_case()&&!is("gesture-timeout")){')
s=s.replace('    if(mode==SetupPhase::calibration && !is("cal-timeout")){','    if(cue_case())cue_input(mode,ms);\n    if(mode==SetupPhase::calibration && !is("cal-timeout")){')
s=s.replace('void AS5600_soft_IIC_many::init(GPIO_TypeDef*const*,const uint16_t*,GPIO_TypeDef*const*,const uint16_t*,int n){numbers=n;}','void AS5600_soft_IIC_many::init(GPIO_TypeDef*const*,const uint16_t*,GPIO_TypeDef*const*,const uint16_t*,int n){numbers=n;if(is("cue-delayed-admission"))delay(600);}')
s=s.replace('    CHECK(i<2);leds[led][i]=rgb;','    CHECK(i<2);leds[led][i]=rgb;\n    if(cue_case())cue_queue_observe(led,rgb);')
s=s.replace('    ++host_clock;raw_dma_boundary();','    ++host_clock;raw_dma_boundary();\n    if(cue_case()&&setup_trial_phase()==SetupPhase::fault){if(!fault_tick)fault_tick=host_clock;if(is("cue-queued-fault"))time_hw_slow=0;}')
s=s.replace('    if(schedule){\n        CHECK(!host_programs', '    if(cue_case())return cue_verify();\n    if(schedule){\n        CHECK(!host_programs')
p.write_text(s)
p=Path(__file__).with_name('check_setup.py');s=p.read_text()
s=s.replace("'--before-fix',action='store_true'","'--before-fix',action='store_true'")
s=s.replace("tests=Path(__file__).resolve().parent","tests=Path(__file__).resolve().parent\n(tests.parent/'logs').mkdir(exist_ok=True)")
anchor="    gestures=json.loads((tests/'gesture-schedules.json').read_text())"
insert='''    # Observation-only seams: no cue renderer, value, filter or decision substitution.
    f=src/'setup_trial.cpp';s=f.read_text()
    old='for (uint8_t ch=0;ch<4;++ch) center[ch]=ADC_DMA_get_value()[6-2*ch];'
    assert s.count(old)==1
    s=s.replace(old,'for (uint8_t ch=0;ch<4;++ch) { center[ch]=ADC_DMA_get_value()[6-2*ch]; host_center_observe(ch,center[ch]); }')
    old='const uint64_t start=time_ticks64(); uint64_t held=0,released=0;'
    assert s.count(old)==1;s=s.replace(old,old+'\\n    host_window_observe(start);')
    old='        if (!setup_trial_poll()) return false;'
    assert s.count(old)==1;s=s.replace(old,old+'\\n        host_gesture_sample();')
    old='if (now-released>=uint64_t(scale)*1200u) {'
    assert s.count(old)==1;s=s.replace(old,old+'\\n                    host_release_observe();')
    old='    phase = SetupPhase::fault;'
    assert s.count(old)==1;s=s.replace(old,old+'\\n    host_fault_observe();')
    f.write_text(s)
'''
s=s.replace(anchor,insert+anchor)
s=s.replace("'startup-schedules.json','gesture-schedules.json']","'startup-schedules.json','gesture-schedules.json','cue_test.inc','cue-schedules.json','extend_harness.py']")
s=s.replace("str(tests/'setup_test.cpp'),str(src/'ams.cpp')","'-I',str(tests),str(tests/'setup_test.cpp'),str(src/'ams.cpp')")
s=s.replace("('before-fix-current-inputs.json' if a.before_fix else 'host-inputs.json')","('before-fix-current-inputs.json' if a.before_fix else 'host-inputs.json')")
p.write_text(s)
