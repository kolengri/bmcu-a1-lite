#!/usr/bin/env python3
"""Run production setup/main/calibration/direction/PWM/journal against hardware boundaries."""
import argparse,shutil,subprocess,tempfile,re,hashlib,json,os,tarfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument("--fixtures",type=Path,default=Path(__file__).resolve().parent);p.add_argument("--logs",type=Path,default=Path(__file__).resolve().parents[1]/"logs");p.add_argument('source',type=Path);p.add_argument('--cases',nargs='+');p.add_argument('--keep-going',action='store_true');p.add_argument('--before-fix',action='store_true');p.add_argument('--outcome-only',action='store_true');p.add_argument('--full',action='store_true');p.add_argument('--features',action='store_true');p.add_argument('--observation',action='store_true');p.add_argument('--idle04',action='store_true');p.add_argument('--authority',action='store_true');p.add_argument('--flight',action='store_true');p.add_argument('--ordinary-kernel',action='store_true');p.add_argument('--gesture-1s',action='store_true');p.add_argument('--feed-latch',action='store_true');p.add_argument('--phase-compat',action='store_true');p.add_argument('--input-detail',action='store_true');p.add_argument('--meter-origin',action='store_true');p.add_argument('--global-phase',action='store_true');p.add_argument('--release-cue',action='store_true');p.add_argument('--expect-release-cue',action='store_true');p.add_argument('--unload-continue',action='store_true');p.add_argument('--phase99',action='store_true');p.add_argument('--phase99-03',action='store_true');p.add_argument('--phase99-all-slots',action='store_true');p.add_argument('--continuous',action='store_true');p.add_argument('--onuse-idle',action='store_true');p.add_argument('--unload-diagnostic',action='store_true');p.add_argument('--lease-feedback',action='store_true');p.add_argument('--automatic-capture',action='store_true');p.add_argument('--rx-backpressure',action='store_true');p.add_argument('--precut-phase',action='store_true');p.add_argument('--encoder-recovery',action='store_true');p.add_argument('--encoder-driver',action='store_true');p.add_argument('--continuous-journal',action='store_true');p.add_argument('--retention',action='store_true');p.add_argument('--working-mode',action='store_true');p.add_argument('--isp-export',action='store_true');p.add_argument('--phase99-request-feedback',action='store_true');p.add_argument('--setup-insertion-recovery',action='store_true');a=p.parse_args()
tests=a.fixtures.resolve()
logs=a.logs.resolve()
# Atomic reservation: parallel read-only builds must not overwrite each other's receipts.
logs.mkdir(parents=True,exist_ok=True)
receipt_number=1
while True:
    receipt_path=logs/('host-inputs-%d.json'%receipt_number)
    try:
        with receipt_path.open('x') as reserved:reserved.write('{"status":"reserved"}\n')
        break
    except FileExistsError:receipt_number+=1
with tempfile.TemporaryDirectory(prefix='bmcu-setup-') as tmp:
    root=Path(tmp);src=root/'src';shutil.copytree(a.source/'src',src)
    for name in ['host_hal.h','host_flash.h']:shutil.copy(tests/name,src/name)
    with tarfile.open(tests/'oracles/2704.tar') as base_tar:
        baseline_bus=base_tar.extractfile('src/bambu_bus_ams.cpp').read().decode()
    with tarfile.open(tests/'oracles/2704.tar') as bt:
        adc_base=bt.extractfile('src/ADC_DMA.cpp').read().decode()
    adc_lines=[line for line in adc_base.splitlines() if re.match(r'        out\[\d\] = ',line)]
    assert len(adc_lines)==8
    (src/'baseline_adc_assign.h').write_text('static void baseline_adc_assign(float* out,const uint32_t* g_acc_sum,float scale){\n'+'\n'.join(adc_lines)+'\n}\n')
    oracle=[]
    for name in ['lite_feedback','get_package_motion','get_package_stu_motion']:
        start=baseline_bus.index('void '+name+'(');brace=baseline_bus.index('{',start);depth=0
        for end in range(brace,len(baseline_bus)):
            if baseline_bus[end]=='{':depth+=1
            if baseline_bus[end]=='}':
                depth-=1
                if depth==0:break
        oracle.append(baseline_bus[start:end+1].replace('void '+name+'(','static void baseline_'+name+'(',1).replace('    lite_feedback(','    baseline_lite_feedback('))
    with tarfile.open(tests/'oracles/2704.tar') as bt:
        motion_base=bt.extractfile('src/Motion_control.cpp').read().decode()
    start=motion_base.index('float Motion_control_filament_meters(');end=motion_base.index('\n}',start)+2
    oracle.append(motion_base[start:end].replace('float Motion_control_filament_meters(','static float baseline_meter_value(',1))
    (src/'baseline_serializers.h').write_text('\n'.join(oracle)+'\n')
    # Immutable build2705 helper and both serializers: exact bytes, no meter rewrite.
    with tarfile.open(tests/'oracles/2705.tar') as bt:
        exact_bus=bt.extractfile('src/bambu_bus_ams.cpp').read().decode()
    exact=[]
    for name in ['lite_feedback','get_package_motion','get_package_stu_motion']:
        start=exact_bus.index('void '+name+'(');brace=exact_bus.index('{',start);depth=0
        for end in range(brace,len(exact_bus)):
            if exact_bus[end]=='{':depth+=1
            if exact_bus[end]=='}':
                depth-=1
                if depth==0:break
        exact.append(exact_bus[start:end+1].replace('void '+name+'(','static void exact5_'+name+'(',1).replace('    lite_feedback(','    exact5_lite_feedback('))
    (src/'exact5_serializers.h').write_text('\n'.join(exact)+'\n')
    # Full accepted04 serializer oracle: remove classifier invocation only.
    # Request validation, templates, field writes, CRC and sequence remain actual bodies.
    with tarfile.open(tests/'oracles/2804.tar') as bt:
        accepted_bus=bt.extractfile('src/bambu_bus_ams.cpp').read().decode()
    accepted=[]
    for name in ['lite_feedback','get_package_motion','get_package_stu_motion']:
        start=accepted_bus.index(('static __attribute__((noinline)) void ' if name=='lite_feedback' else 'void ')+name+'(')
        brace=accepted_bus.index('{',start);depth=0
        for end in range(brace,len(accepted_bus)):
            if accepted_bus[end]=='{':depth+=1
            if accepted_bus[end]=='}':
                depth-=1
                if not depth:break
        body=accepted_bus[start:end+1]
        if name!='lite_feedback':
            lo=body.index('    if (!set_motion(');hi=body.index('    auto *package_send',lo);body=body[:lo]+body[hi:]
        for symbol in ['lite_feedback','get_package_motion','get_package_stu_motion']:
            body=body.replace(symbol+'(', 'accepted04_'+symbol+'(')
        accepted.append(body)
    (src/'accepted04_serializers.h').write_text('\n'.join(accepted)+'\n')



    # Sealed3103 serializers: classification/telemetry hooks are excluded from
    # the oracle, while every wire store/template/CRC/sequence remains actual.
    with tarfile.open(tests/'oracles/3103.tar') as bt:
        selected3103=bt.extractfile('src/bambu_bus_ams.cpp').read().decode()
    def source_function(text,name):
        start=text.index(name+'(');start=text.rfind('\n',0,start)+1
        brace=text.index('{',start);depth=0
        for end in range(brace,len(text)):
            if text[end]=='{':depth+=1
            if text[end]=='}':
                depth-=1
                if not depth:return text[start:end+1]
        raise AssertionError(name)
    selected_oracle=[source_function(selected3103,'lite_reply_init')]
    lo=selected3103.index('#if BMCU_LITE_NATIVE_PHASE_COMPAT\n// The two Lite serializers')
    hi=selected3103.index('void get_package_motion(',lo)
    selected_oracle.append(selected3103[lo:hi])
    for name in ['get_package_motion','get_package_stu_motion']:
        body=source_function(selected3103,name)
        lo=body.index('#if BMCU_LITE_ONUSE_IDLE_COMPAT\n    const auto motion_reply=')
        hi=body.index('    auto *package_send',lo)
        body=body[:lo]+'    const MotionReply motion_reply=1u;\n'+body[hi:]
        body=body.replace('    flight::response(in.filamnet_channel,in.statu_flag,in.motion_flag);','')
        selected_oracle.append(body)
    selected_oracle='\n'.join(selected_oracle)
    for name in ['lite_reply_init','lite_feedback','get_package_motion','get_package_stu_motion']:
        selected_oracle=selected_oracle.replace(name+'(', 'parent3103_'+name+'(')
    (src/'parent3103_serializers.h').write_text(selected_oracle+'\n')




    for name in ['ch32v20x_adc.h','hal/irq_wch.h','hal/time_hw.h','ch32v20x.h','ch32v20x_gpio.h','ch32v20x_rcc.h','ch32v20x_usart.h','ch32v20x_dma.h','ch32v20x_misc.h','core_riscv.h','ch32v20x_flash.h','ch32v20x_crc.h']:
        (src/name).write_text('#pragma once\n#include "'+('host_flash.h' if name in ['ch32v20x_flash.h','ch32v20x_crc.h'] else 'host_hal.h')+'"\n')
    if a.flight:
        f=src/'main.cpp';s=f.read_text();needle='for (;;) time_ticks32(); // Terminal capture'
        assert s.count(needle)==2
        s=s.replace('    for(;;)time_ticks32();','    for(;;){host_terminal_seen=true;throw TerminalCapture{};}')
        f.write_text(s.replace(needle,'for (;;) {host_terminal_seen=true;throw TerminalCapture{};} // Terminal capture'))
    f=src/'_bus_hardware.cpp';s=f.read_text().replace('__attribute__((interrupt("WCH-Interrupt-fast")))','').replace('__asm__ volatile("fence iorw, iorw" ::: "memory");','__asm__ volatile("" ::: "memory");')
    for ptr in ['rx_dma','&USART1->DATAR','data']:s=s.replace('(uint32_t)'+ptr,'(uintptr_t)'+ptr)
    f.write_text(s)
    f=src/'ADC_DMA.cpp';s=f.read_text().replace('__asm__ volatile("fence iorw, iorw" ::: "memory");','__asm__ volatile("" ::: "memory");').replace('(uint32_t)g_dma_buf','(uintptr_t)g_dma_buf');f.write_text(s)
    f=src/'ADC_DMA.cpp';s=f.read_text()
    for n,k in [('ADC_DMA_get_value',1),('ADC_DMA_generation',2),('ADC_DMA_ready',3)]:
        needle=n+'()\n{';assert s.count(needle)==1;s=s.replace(needle,needle+'\n    host_input_detail_read('+str(k)+');')
    begin=s.index('const float *ADC_DMA_get_value()');end=s.index('uint32_t ADC_DMA_generation()',begin)
    part=s[begin:end];assert part.count('adc_dma_compiler_barrier();')==1
    s=s[:begin]+part.replace('adc_dma_compiler_barrier();','host_input_detail_barrier(out,wr);\n        adc_dma_compiler_barrier();')+s[end:]
    f.write_text(s)
    f=src/'ADC_DMA.cpp';s=f.read_text();assert s.count('void ADC_DMA_poll()\n{')==1;s=s.replace('void ADC_DMA_poll()\n{','void ADC_DMA_poll()\n{\n    host_adc_poll_observe(); host_input_detail_read(8);');f.write_text(s)
    if a.encoder_driver:
        f=src/'many_soft_AS5600.cpp';s=f.read_text()
        assert s.count('port_SDA[j]->INDR & pin_SDA[j]')==1
        assert s.count('port_SDA[i]->INDR & pin_SDA[i]')==1
        s=s.replace('port_SDA[j]->INDR & pin_SDA[j]','host_i2c_sample(j,false)')
        s=s.replace('port_SDA[i]->INDR & pin_SDA[i]','host_i2c_sample(i,true)')
        for width in [8,16]:
            needle='void AS5600_soft_IIC_many::read_reg%d(uint8_t reg)\n{'%width
            # The ON driver shares the transaction body; the OFF body remains sealed-parent exact.
            if s.count(needle)==1:
                s=s.replace(needle,needle+'\n    host_i2c_begin(reg,%du);'%width)
        if 'void AS5600_soft_IIC_many::read_reg(uint8_t reg,bool word)' in s:
            needle='void AS5600_soft_IIC_many::read_reg(uint8_t reg,bool word)\n{'
            assert s.count(needle)==1
            s=s.replace(needle,needle+'\n    host_i2c_begin(reg,word?16u:8u);')
        for needle,event,arg in [('void AS5600_soft_IIC_many::clear_datas()\n{',1,'0'),('void AS5600_soft_IIC_many::start_iic(unsigned char ADR)\n{',2,'ADR'),('void AS5600_soft_IIC_many::write_iic(uint8_t byte)\n{',3,'byte'),('void AS5600_soft_IIC_many::read_iic(bool ack)\n{',4,'ack'),('void AS5600_soft_IIC_many::stop_iic()\n{',5,'0')]:
            assert s.count(needle)==1
            s=s.replace(needle,needle+'\n    host_i2c_event(%du,%s);'%(event,arg))
        f.write_text(s)
    f=src/'Motion_control.cpp';s=f.read_text()
    s=s.replace('float Motion_control_filament_meters(uint8_t ch)\n{','float Motion_control_filament_meters(uint8_t ch)\n{\n    if(host_meter_trace)++host_meter_reads;')
    for needle,kind in [('static inline void MC_PULL_ONLINE_read(uint32_t now_ticks)\n{',4),('void AS5600_distance_updata(uint32_t now_ticks)\n{',5)]:
        assert s.count(needle)==1;s=s.replace(needle,needle+'\n    host_input_detail_read('+str(kind)+');')
    needle='    if (!full_load_check_inputs()) return;\n    const float *raw=ADC_DMA_get_value();';assert s.count(needle)==1
    s=s.replace(needle,'    host_input_detail_hook(1);\n'+needle)
    needle='    const uint8_t valid=MC_AS5600.updata_angle(); MC_AS5600.updata_stu();';assert s.count(needle)==2
    s=s.replace(needle,needle+'\n    host_input_detail_hook(2);')
    if a.ordinary_kernel:
        base=tests.parent/'snapshots/base/src/Motion_control.cpp'
        def kernel(text):
            lo=text.index('    void run(float time_E, uint64_t now_ms)');hi=text.index('\n};\n\n_MOTOR_CONTROL',lo)
            return text[lo:hi],lo,hi
        common=['-DBMCU_DM_TWO_MICROSWITCH=1','-DBMCU_SOFT_LOAD=0','-DBMCU_P1S=0']
        def pp(text,flags):return subprocess.check_output(['c++','-E','-P','-x','c++',*common,*flags,'-'],input=text,text=True)
        ordinary=pp(kernel(base.read_text())[0],['-DBMCU_LITE_FULL_LOAD=0','-DBMCU_LITE_V11_FEATURES=0'])
        restored=pp(kernel(s)[0],['-DBMCU_LITE_FULL_LOAD=1','-DBMCU_LITE_V11_FEATURES=1'])
        assert ordinary==restored,'Restored controller kernel differs from actual ordinary V11 body'
        _,lo,hi=kernel(s);s=s[:lo]+ordinary+s[hi:]
        (logs/(receipt_path.stem+'-ordinary-kernel-identity.json')).write_text(json.dumps({'accepted_motion_sha256':hashlib.sha256(base.read_bytes()).hexdigest(),'ordinary_preprocessed_kernel_sha256':hashlib.sha256(ordinary.encode()).hexdigest(),'restored_body_equal':True,'scope':'Actual ordinary _MOTOR_CONTROL::run only. New real main, parser, durable PWM barrier, storage, ADC and local authority remain compiled and asserted; this is not an ordinary-main acceptance claim.'},indent=2)+'\n')
    start=s.index('bool Motion_control_setup_init(bool empty) {');end=s.index('bool Motion_control_setup_inputs(',start)
    body=s[start:end].replace('    if (!ADC_DMA_ready())','    host_admission_observe(0);\n    if (!ADC_DMA_ready())').replace('    MC_PULL_detect_channels_inserted();','    MC_PULL_detect_channels_inserted();\n    host_admission_observe(1);')
    s=s[:start]+body+s[end:]
    assert s.count('    uint16_t set1 = 0, set2 = 0;')==1
    s=s.replace('    uint16_t set1 = 0, set2 = 0;','    host_pwm_observe(CHx,PWM);\n    uint16_t set1 = 0, set2 = 0;')
    f.write_text(s)
    import json
    schedules=json.loads((tests/'startup-schedules.json').read_text())
    header=['struct StartupSchedule {const char* name;unsigned activity,in_range;int bad;unsigned stream;bool positive;unsigned detail,parent;float key_v;};','static const StartupSchedule schedules[] = {']
    for name,v in schedules.items():header.append('{"%s",%d,%d,%d,%d,%s,%d,%d,%.6ff},'%(name,v['activity_ms'],v['in_range_ms'],v['bad_channel'],{'samples':0,'te':1,'flags':2}[v['stream']],'true' if v['expect']=='gesture' else 'false',v.get('detail',0),{'gesture':0,'adc':6,'module':4,'filament':5}[v['expect']],v.get('key_v',.20)))
    header.append('};');(src/'startup_schedules.h').write_text('\n'.join(header)+'\n')
    f=src/'bambu_bus_ams.cpp';s=f.read_text();assert s.count('(volatile const uint8_t *)0x1FFFF7E8')==1;f.write_text(s.replace('(volatile const uint8_t *)0x1FFFF7E8','host_uid'))
    f=src/'Flash_saves.cpp';s=f.read_text()
    # Physical load/store translation only. No validator, scan, cache or job logic is replaced.
    s=s.replace('*(volatile uint16_t*)addr = (uint16_t)data;','host_store16(addr,(uint16_t)data);')
    s=s.replace('*(volatile uint16_t*)(addr + 2u) = (uint16_t)(data >> 16);','host_store16(addr+2u,(uint16_t)(data>>16));')
    s=s.replace('*(volatile uint16_t*)(job.addr + (uint32_t)index * 2u) =\n            (uint16_t)(job.words[index >> 1] >> ((index & 1u) * 16u));','host_store16(job.addr+(uint32_t)index*2u,(uint16_t)(job.words[index>>1]>>((index&1u)*16u)));')
    for operand,replacement in {
        '(const void*)boot_export_page':'(const void*)host_flash_ptr(boot_export_page)',
        '(const uint32_t*)base_addr':'(const uint32_t*)host_flash_ptr(base_addr)',
        '(const uint8_t*)page_addr':'host_flash_ptr(page_addr)',
        '(const uint32_t*)(base + s * FIL_SLOT_BYTES)':'(const uint32_t*)host_flash_ptr(base + s * FIL_SLOT_BYTES)',
        '(const volatile uint32_t*)(a + 0u)':'(const volatile uint32_t*)host_flash_ptr(a + 0u)',
        '(const volatile uint32_t*)(a + 4u)':'(const volatile uint32_t*)host_flash_ptr(a + 4u)',
        '(const volatile uint32_t*)addr':'(const volatile uint32_t*)host_flash_ptr(addr)',
        '(const uint32_t*)FIL_SCRATCH_ADDR':'(const uint32_t*)host_flash_ptr(FIL_SCRATCH_ADDR)',
        '(const volatile uint16_t*)(job.addr + (uint32_t)index * 2u)':'(const volatile uint16_t*)host_flash_ptr(job.addr + (uint32_t)index * 2u)',
    }.items():
        assert operand in s,operand;s=s.replace(operand,replacement)
    if a.flight:
        s=s.replace('(const void*)0x0800ee00u','(const void*)host_flash_ptr(0x0800ee00u)')
        for timer,index in [('TIM2',0),('TIM3',1),('TIM4',2)]:
            for channel in range(4):s=s.replace(timer+'->CH'+str(channel+1)+'CVR','compare_values['+str(index)+']['+str(channel)+']')
    f.write_text(s)
    if a.flight:
        f=src/'flight_recorder.cpp';s=f.read_text()
        if a.isp_export:s=s.replace('GPIOA->BSHR=GPIO_Pin_12;', 'host_export_de_high();')
        if a.phase99_request_feedback:
            begin='void tx_start(const uint8_t* data,uint16_t length){'
            assert s.count(begin)==1
            s=s.replace(begin,begin+'\n    host_active_request_phase=(jrequest==0x9907u&&jrequest_channel<4u)?uint8_t(0xe0u|jrequest_channel):0u; // Host-only actual DMA request observer.')
        if a.lease_feedback:
            assert s.count('Motion_control_filament_meters(ch)')==2
            s=s.replace('Motion_control_filament_meters(ch)','(host_onuse_meter_reads++,Motion_control_filament_meters(ch))')
        for timer,index in [('TIM2',0),('TIM3',1),('TIM4',2)]:
            for channel in range(4):s=s.replace(timer+'->CH'+str(channel+1)+'CVR','compare_values['+str(index)+']['+str(channel)+']')
        f.write_text(s)
    # Observe queue writes and completed physical transmissions; preserve actual LED bodies/guards.
    f=src/'ws2812.cpp';s=f.read_text()
    needle='    dirty = false;\n    irq_restore_wch(irq);'
    assert s.count(needle)==1;s=s.replace(needle,'    host_ws_delivered(this,last_grb,num);\n'+needle)
    begin=s.index('void WS2812_class::set_RGB(');end=s.index('void WS2812_class::set_RGB_online',begin)
    body=s[begin:end];assert body.count('    dirty = true;')==1
    s=s[:begin]+body.replace('    dirty = true;','    dirty = true;\n    host_ws_queued(this,(uint32_t(R)<<16)|(uint32_t(G)<<8)|B,index);')+s[end:];f.write_text(s)
    # Observation-only seams: no cue renderer, value, filter or decision substitution.
    f=src/'setup_trial.cpp';s=f.read_text()
    old='for (uint8_t ch=0;ch<4;++ch) center[ch]=ADC_DMA_get_value()[6-2*ch];'
    assert s.count(old)==1
    s=s.replace(old,'for (uint8_t ch=0;ch<4;++ch) { center[ch]=ADC_DMA_get_value()[6-2*ch]; host_center_observe(ch,center[ch]); }')
    old='const uint64_t start=time_ticks64(); uint64_t held=0,released=0;'
    assert s.count(old)==1;s=s.replace(old,old+'\n    host_window_observe(start);')
    old='        if (!setup_trial_poll()) return false;'
    assert s.count(old)==1;s=s.replace(old,old+'\n        host_gesture_sample();')
    old='if (now-released>=uint64_t(scale)*1200u) {'
    assert s.count(old)==1;s=s.replace(old,old+'\n                    host_release_observe();')
    old='    phase = SetupPhase::fault;'
    assert s.count(old)==1;s=s.replace(old,old+'\n    host_fault_observe();')
    old='    MC_PULL_calibration_boot();'
    assert s.count(old) in [1,2];s=s.replace(old,'    host_cal_observe();\n'+old)
    for stage,label in [(1,'gesture'),(2,'calibration'),(3,'direction')]:
        old='phase=SetupPhase::'+label+'; reached='+str(stage)+';'
        assert old in s
        s=s.replace(old,old+' host_insertion_stage('+str(stage)+');')
    old='bool setup_trial_flash_address(uint32_t address) {'
    assert old in s;s=s.replace(old,old+'\n    host_insertion_write(address);')
    old='bool setup_trial_write_allowed(SetupWrite kind) {'
    assert old in s;s=s.replace(old,old+'\n    if(kind==SetupWrite::journal)host_insertion_write(0x0800f600u);')
    f.write_text(s)
    gestures=json.loads((tests/'gesture-schedules.json').read_text())
    header=['struct GestureSegment {unsigned begin,end;float delta;};', 'struct GestureSchedule {const char* name;unsigned channel,parent,detail;bool ack;const char* fault;unsigned fault_ms;GestureSegment segments[8];unsigned count;};', 'static const GestureSchedule gesture_schedules[] = {']
    for name,v in gestures.items():
        segments=','.join('{%d,%d,%.6ff}'%tuple(row) for row in v['segments_ms_deltaV'])
        header.append('{"%s",%d,%d,%d,%s,"%s",%d,{%s},%d},'%(name,v['channel'],v['parent'],v['detail'],'true' if v['ack'] else 'false',v['fault'],v['fault_ms'],segments,len(v['segments_ms_deltaV'])))
    header.append('};');(src/'gesture_schedules.h').write_text('\n'.join(header)+'\n')
    cues=json.loads((tests/'cue-schedules.json').read_text())
    header=['struct CueSegment {unsigned begin,end,channel,raw;};','struct CueSchedule {const char* name;unsigned baseline[4],parent,detail;bool ack;const char* fault;const char* stage;CueSegment segments[8];unsigned count;};','static const CueSchedule cue_schedules[] = {']
    for name,v in cues.items():
        segments=','.join('{%d,%d,%d,%d}'%tuple(row) for row in v['segments_ms_channel_raw'])
        baseline=','.join(map(str,v['baseline_raw']))
        header.append('{"%s",{%s},%d,%d,%s,"%s","%s",{%s},%d},'%(name,baseline,v['parent'],v['detail'],'true' if v['ack'] else 'false',v['fault'],v['fault_stage'],segments,len(v['segments_ms_channel_raw'])))
    header.append('};');(src/'cue_schedules.h').write_text('\n'.join(header)+'\n')
    def digest(f):return hashlib.sha256(f.read_bytes()).hexdigest()
    provenance={'runner':{'path':str(Path(__file__).resolve()),'sha256':digest(Path(__file__))},'source':{str(f.relative_to(a.source)):digest(f) for f in sorted((a.source/'src').rglob('*')) if f.is_file()},'compiled_source':{str(f.relative_to(src)):digest(f) for f in sorted(src.rglob('*')) if f.is_file()},'tests':{n:digest(tests/n) for n in ['check_setup.py','setup_test.cpp','host_hal.h','host_flash.h','startup-schedules.json','gesture-schedules.json','cue_test.inc','cue-schedules.json','extend_harness.py','make_schedules.py','full_test.inc','selected-cases.json','round1-cases.json','v11_test.inc','repair_idle_test.inc','authority_test.inc','observation_test.inc','idle04_test.inc','flight_test.inc','flight_hal.h','phase_test.inc','meter_test.inc','global_phase_test.inc','release_cue_test.inc','unload_continue_test.inc','phase99_test.inc','phase99_03_test.inc','phase99_all_slots_test.inc','ff07_test.inc','onuse_idle_test.inc','unload_diagnostic_test.inc','lease_test.inc','automatic_test.inc','rx_queue_test.inc','precut_phase_test.inc','run_queue.py','encoder_driver_model.inc','encoder_recovery_test.inc','run_encoder.py','continuous_test.inc','continuous_boot_test.inc','meter-cases.json','input_detail_test.inc','phase-cases.json','latch_test.inc','latch-cases.json','native-affected-cases.json','restored-cases.json','journal_test.inc','run_journal.py','retention_test.inc','run_retention.py','phase99_request_test.inc','run_phase99_request.py','working_test.inc','run_working.py','marker_test.inc','run_marker.py','insertion_recovery_test.inc','run_insertion.py']},'tools':{n:{'path':shutil.which(n),'executable_sha256':digest(Path(shutil.which(n)).resolve()),'version':subprocess.check_output([n,'--version'],text=True)} for n in ['cc','c++']},'clock':'fixed virtual integer ticks; raw DMA half/flag cadence1ms; no wall-clock sleeps or random input','argv':__import__('sys').argv,'before_fix':a.before_fix}
    receipt_path.write_text(json.dumps(provenance,indent=2,sort_keys=True)+'\n')
    san=['-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-g']
    subprocess.run(['cc',*san,*(['-DBMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE=1'] if a.unload_diagnostic else []),'-I',str(src),'-c',str(src/'crc_bus.c'),'-o',str(root/'crc.o')],check=True)
    flags=['BMCU_P1S=0','BMCU_SOFT_LOAD=0','BMCU_EXPERIMENTAL_AMS_LITE=1','BMCU_LITE_MOTORLOCK=0','BMCU_LITE_SINGLE_CHANNEL_TRIAL=1','BMCU_LITE_OWNERSHIP_DETAIL=1','BMCU_LITE_SETUP_TRIAL=1','BAMBU_BUS_AMS_NUM=0','AMS_RETRACT_LEN=0.095f','BMCU_DM_TWO_MICROSWITCH=1','BMCU_ONLINE_LED_FILAMENT_RGB=1']
    if a.full:flags=[x for x in flags if not x.startswith(('BMCU_LITE_SINGLE_CHANNEL_TRIAL=','BMCU_LITE_OWNERSHIP_DETAIL='))]+['BMCU_LITE_SINGLE_CHANNEL_TRIAL=0','BMCU_LITE_OWNERSHIP_DETAIL=0','BMCU_LITE_FULL_LOAD=1']
    if a.flight:flags.append('BMCU_USB_FLIGHT_RECORDER=1')
    if a.continuous_journal:flags.append('BMCU_USB_CONTINUOUS_JOURNAL=1')
    if a.retention:flags.append('HOST_JOURNAL_RETENTION=1')
    if a.working_mode:flags.append('BMCU_USB_WORKING_MODE=1')
    if a.isp_export:flags.append('BMCU_USB_ISP_EXPORT=1')
    if a.setup_insertion_recovery:flags.append('BMCU_SETUP_INSERTION_RECOVERY=1')
    if a.phase99_request_feedback:flags.append('BMCU_LITE_PHASE99_REQUEST_FEEDBACK=1')
    if a.gesture_1s:flags.append('BMCU_LITE_SETUP_GESTURE_1S=1')
    if a.feed_latch:flags.append('BMCU_LITE_V11_FEED_LATCH=1')
    if a.phase_compat:flags.append('BMCU_LITE_NATIVE_PHASE_COMPAT=1')
    if a.input_detail:flags.append('BMCU_LITE_INPUT_FAULT_DETAIL=1')
    if a.meter_origin:flags.append('BMCU_LITE_METER_ORIGIN_ZERO=1')
    if a.global_phase:flags.append('BMCU_LITE_GLOBAL_PHASE_COMPAT=1')
    if a.release_cue:flags.append('BMCU_LITE_SETUP_RELEASE_CUE=1')
    if a.unload_continue:flags.append('BMCU_LITE_NATIVE_UNLOAD_CONTINUE=1')
    if a.phase99:flags.append('BMCU_LITE_PHASE99_COMPAT=1')
    if a.phase99_03:flags.append('BMCU_LITE_PHASE99_03_COMPAT=1')
    if a.phase99_all_slots:flags.append('BMCU_LITE_PHASE99_ALL_SLOTS=1')
    if a.continuous:flags.append('BMCU_LITE_CONTINUOUS_NATIVE=1')
    if a.onuse_idle:flags.append('BMCU_LITE_ONUSE_IDLE_COMPAT=1')
    if a.unload_diagnostic:flags.append('BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE=1')
    if a.lease_feedback:flags.append('BMCU_LITE_LEASE_FEEDBACK=1')
    if a.automatic_capture:flags.append('BMCU_LITE_AUTOMATIC_CAPTURE=1')
    if a.rx_backpressure:flags.append('BMCU_LITE_RX_BACKPRESSURE=1')
    if a.precut_phase:flags.append('BMCU_LITE_PRECUT_PHASE_CAPTURE=1')
    if a.encoder_recovery:flags.append('BMCU_LITE_ENCODER_RECOVERY=1')
    if a.encoder_driver:flags.append('HOST_ENCODER_DRIVER=1')
    if a.expect_release_cue:flags.append('HOST_EXPECT_RELEASE_CUE=1')
    if a.idle04:flags.append('BMCU_LITE_NATIVE_IDLE_04=1')
    if a.observation:flags.append('BMCU_LITE_AUTHORITY_OBSERVATION=1')
    if a.authority:flags.append('BMCU_LITE_NATIVE_FEED_AUTHORITY=1')
    if a.features:flags.append('BMCU_LITE_V11_FEATURES=1')
    if a.ordinary_kernel:flags.append('HOST_ORDINARY_KERNEL=1')
    if a.before_fix:flags.append('HOST_BEFORE_FIX=1')
    if a.outcome_only:flags.append('HOST_OUTCOME_ONLY=1')
    provenance['defines']=flags;provenance['compiler_options']=['-std=c++17',*san];receipt_path.write_text(json.dumps(provenance,indent=2,sort_keys=True)+'\n')
    binary=root/'setup-test'
    subprocess.run(['c++','-std=c++17',*san,*['-D'+x for x in flags],'-I',str(src),'-I',str(tests),str(tests/'setup_test.cpp'),str(src/'ams.cpp'),str(root/'crc.o'),'-o',str(binary)],check=True)
    cases=a.cases or [*gestures,'cold-trial','startup-channel2','startup-default-flags','startup-no-dma']
    state=root/'synthetic-nvm.bin'
    # Success precedes cold-trial to exercise a genuine production-written fixture across processes.
    if 'cold-trial' in cases and not any(c in cases for c in ['any-3','success','detail-normal']):cases=['any-3',*cases]
    failures=[];provenance['case_results']=[]
    trace_dir=logs/'traces'/receipt_path.stem;trace_dir.mkdir(parents=True,exist_ok=False)
    run_env=dict(os.environ,CUE_TRACE_DIR=str(trace_dir),FLIGHT_TRACE_DIR=str(trace_dir))
    for case in cases:
        print('CASE '+case,flush=True);result=subprocess.run([str(binary),case,str(state)],check=not a.keep_going,env=run_env,timeout=30)
        provenance['case_results'].append({'case':case,'returncode':result.returncode});receipt_path.write_text(json.dumps(provenance,indent=2,sort_keys=True)+'\n')
        if result.returncode:failures.append((case,result.returncode))
        if case.startswith('cut-'):
            print('CASE after-cut '+case,flush=True);subprocess.run([str(binary),'after-cut',str(state)],check=True)
    assert not failures,failures
    print('Production setup scenarios passed with ASan/UBSan.')

    guard=root/'guard.cpp';guard.write_text('#include "lite_profile.h"\n')
    for invalid in [[],['-DBMCU_EXPERIMENTAL_AMS_LITE=1'],['-DBMCU_EXPERIMENTAL_AMS_LITE=1','-DBMCU_LITE_SINGLE_CHANNEL_TRIAL=1'],['-DBMCU_EXPERIMENTAL_AMS_LITE=1','-DBMCU_LITE_SINGLE_CHANNEL_TRIAL=1','-DBMCU_DM_TWO_MICROSWITCH=1','-DBMCU_LITE_MOTORLOCK=1']]:
        result=subprocess.run(['c++','-fsyntax-only','-I',str(src),'-DBMCU_LITE_SETUP_TRIAL=1',*invalid,str(guard)],capture_output=True,text=True)
        assert result.returncode and ('Setup/trial requires' in result.stderr or 'Single-channel trial requires' in result.stderr),result.stderr
    result=subprocess.run(['c++','-fsyntax-only','-I',str(src),'-DBMCU_LITE_SETUP_TRIAL=2',str(guard)],capture_output=True,text=True)
    assert result.returncode and 'flag must be 0 or 1' in result.stderr
    if a.full:
        valid=['-DBMCU_LITE_FULL_LOAD=1','-DBMCU_EXPERIMENTAL_AMS_LITE=1','-DBMCU_LITE_SETUP_TRIAL=1','-DBMCU_LITE_SINGLE_CHANNEL_TRIAL=0','-DBMCU_LITE_MOTORLOCK=0','-DBMCU_LITE_OWNERSHIP_DETAIL=0','-DBMCU_DM_TWO_MICROSWITCH=1','-DBAMBU_BUS_AMS_NUM=0']
        result=subprocess.run(['c++','-fsyntax-only','-I',str(src),*valid,str(guard)],capture_output=True,text=True)
        assert result.returncode==0,result.stderr
        for option in ['BMCU_LITE_FULL_LOAD=2','BMCU_EXPERIMENTAL_AMS_LITE=0','BMCU_LITE_SETUP_TRIAL=0','BMCU_LITE_SINGLE_CHANNEL_TRIAL=1','BMCU_LITE_MOTORLOCK=1','BMCU_LITE_OWNERSHIP_DETAIL=1','BMCU_DM_TWO_MICROSWITCH=0','BAMBU_BUS_AMS_NUM=1']:
            key=option.split('=')[0];invalid=[v for v in valid if not v.startswith('-D'+key+'=')]+['-D'+option]
            result=subprocess.run(['c++','-fsyntax-only','-I',str(src),*invalid,str(guard)],capture_output=True,text=True)
            assert result.returncode!=0,option
        print('Full-load valid profile and eight contradictory/invalid selections checked against real headers.')
    if a.features:
        valid+=['-DBMCU_LITE_V11_FEATURES=1','-DBMCU_SOFT_LOAD=0','-DBMCU_P1S=0']
        for option in ['BMCU_LITE_V11_FEATURES=2','BMCU_LITE_FULL_LOAD=0','BMCU_SOFT_LOAD=1','BMCU_P1S=1']:
            key=option.split('=')[0];invalid=[v for v in valid if not v.startswith('-D'+key+'=')]+['-D'+option]
            result=subprocess.run(['c++','-fsyntax-only','-I',str(src),*invalid,str(guard)],capture_output=True,text=True)
            assert result.returncode!=0,option
        print('Restored V11 selection: four invalid flag/mechanical/parent choices rejected.')
    if a.authority:
        valid+=['-DBMCU_LITE_NATIVE_FEED_AUTHORITY=1']
        result=subprocess.run(['c++','-fsyntax-only','-I',str(src),*valid,str(guard)],capture_output=True,text=True)
        assert result.returncode==0,result.stderr
        for option in ['BMCU_LITE_NATIVE_FEED_AUTHORITY=2','BMCU_LITE_V11_FEATURES=0']:
            key=option.split('=')[0];invalid=[v for v in valid if not v.startswith('-D'+key+'=')]+['-D'+option]
            result=subprocess.run(['c++','-fsyntax-only','-I',str(src),*invalid,str(guard)],capture_output=True,text=True)
            assert result.returncode!=0,option
        print('Native feed authority: valid opt-in and two invalid selections verified.')
    print('Combined flag default OFF and incompatible profile compile guards verified.')

    if a.flight:
        valid+=['-DBMCU_LITE_NATIVE_IDLE_04=1','-DBMCU_LITE_AUTHORITY_OBSERVATION=1','-DBMCU_USB_FLIGHT_RECORDER=1']
        result=subprocess.run(['c++','-fsyntax-only','-I',str(src),*valid,str(guard)],capture_output=True,text=True)
        assert result.returncode==0,result.stderr
        for option in ['BMCU_USB_FLIGHT_RECORDER=2','BMCU_LITE_NATIVE_IDLE_04=0','BMCU_LITE_AUTHORITY_OBSERVATION=0']:
            key=option.split('=')[0];invalid=[v for v in valid if not v.startswith('-D'+key+'=')]+['-D'+option]
            result=subprocess.run(['c++','-fsyntax-only','-I',str(src),*invalid,str(guard)],capture_output=True,text=True)
            assert result.returncode!=0,option
        print('Flight recorder: actual parent/default/invalid compile guards passed.')
