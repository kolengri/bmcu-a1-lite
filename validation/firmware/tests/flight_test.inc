// Actual firmware/Flash_saves/recorder bodies; only MMIO and sensor/time boundaries modelled.
static void flight_reset(){
#if BMCU_USB_CONTINUOUS_JOURNAL
    flight::jqhead=flight::jqcount=flight::jpage=flight::jslot=flight::jword=flight::jstate=0;
    flight::jlost=0;flight::jgeneration=flight::jsession=flight::jlast_heartbeat=0;
#ifndef HOST_JOURNAL_RETENTION
    flight::jlast_meter=0;
#endif
    flight::jrequest=flight::jlast_rx=0;flight::jrequest_channel=255;flight::jrequest_command=0;
    flight::jtuple_valid=0;flight::jwaiting=flight::jfailed=false;diagnostic_result=0;
#ifdef HOST_JOURNAL_RETENTION
    memset(flight::jlast_tuple,0,sizeof(flight::jlast_tuple));
    flight::jrefusal_pending=false;
#else
    for(unsigned i=0;i<2;++i){flight::jlast_tuple[i]=flight::jmeter_bits[i]=flight::jmeter_at[i]=0;flight::jlast_channel[i]=255;}
#endif
#endif
#if BMCU_USB_WORKING_MODE
    bus_passive=false;
#if BMCU_USB_ISP_EXPORT
    boot_quiesced=false;boot_export_checked=boot_export_erasing=false;phase=SetupPhase::inspect;
    marker_erase_fail=marker_verify_fail=false;
#endif
#endif
    host_rcc.RSTSCKR.value=0x0c000000u;
#if BMCU_LITE_AUTOMATIC_CAPTURE
    flight::automatic_deadline=0;
#if BMCU_LITE_PRECUT_PHASE_CAPTURE
    flight::wire_phase=flight::staged_phase=flight::phase_count=0;flight::phase_origin=0;
#endif
#endif
    flight::report={};flight::session=flight::frame_crc=flight::window_scale=0;
    flight::window_start=flight::last_sample=0;
    std::memset(flight::comparator_delta,0,sizeof(flight::comparator_delta));flight::tx=flight::rx=0;flight::tx_at=flight::rx_at=0;
    flight::capturing=flight::finished=flight::acknowledged=flight::storage_error=false;
#if BMCU_LITE_V11_FEED_LATCH
    flight::staged_kind=flight::active_kind=0;
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
    flight::verified_saved=flight::pull_first=flight::operator_stop=false;
    flight::pull_command=flight::pull_channel=0;
#endif
#endif
    std::memset(flight::frame,0,sizeof(flight::frame));std::memset(flight::ack,0,20);std::memset(flight::reply,0,20);
    host_flash.STATR=0;host_flash.CTLR.value=0;g_flash_fault=false;g_flash_job={};log_rx.clear();log_tx.clear();log_cut=-1;
    host_hang=host_fail_program=false;host_time_hook=nullptr;host_compare_hook=nullptr;host_irq_restore_hook=nullptr;host_irq_enabled=true;
    time_hw_slow=0;time_hw_tpms=1000;time_hw_tpus=1;
    for(auto&row:compare_values)for(auto&v:row)v=0;
}
static void flight_save(const char*name,const void*data,size_t bytes){
    const char*dir=std::getenv("FLIGHT_TRACE_DIR");CHECK(dir);
    FILE*f=std::fopen((std::string(dir)+"/"+name).c_str(),"wb");CHECK(f);CHECK(std::fwrite(data,1,bytes,f)==bytes);CHECK(!std::fclose(f));
}
static void flight_setup_verify(){
    if(!(phase==SetupPhase::fault||phase==SetupPhase::success))return;
    const bool complete=flight::complete(host_log);
    // Storage/clock/input failures may deny the write; never label partial as complete.
    CHECK(flight::header_valid(host_log));
    if(!complete){std::puts("FLIGHT incomplete: terminal write denied/failed; durable session marker retained");return;}
    flight::Report r;std::memcpy(&r,host_log+32,sizeof(r));
    CHECK(r.phase==uint8_t(phase)&&r.fault==uint8_t(fault)&&r.detail==boot_admission_detail&&r.reached==reached);
    if(cue_window||gesture_start){
        if(r.flags&1){CHECK(r.samples&&r.gen_last!=r.gen_first);for(unsigned ch=0;ch<4;++ch){CHECK(r.minimum[ch]<=r.baseline[ch]);CHECK(r.maximum[ch]>=r.baseline[ch]);CHECK(std::isfinite(r.peak_delta[ch]));}}
    }
    if(r.flags&1){CHECK(r.core_hz==144000000u&&r.ticks_per_ms==1000u);CHECK(r.rcc_cfgr0==0x003c9408u&&r.exten_ctlr==0x10u&&r.systick_ctlr==9u);
        for(unsigned i=0;i<r.event_count;++i){const auto&e=r.events[i];const float delta=r.event_delta[i];CHECK(std::isfinite(delta)&&delta>=0.f);if((e.kind&1)&&!(e.kind&2))CHECK(delta>=.15f);if(!(e.kind&1)&&e.channel<4)CHECK(delta<.15f);if(e.kind&4)CHECK(delta<=.04f);}}
    if(any_success()){CHECK(r.flags&1);CHECK(r.candidate==gesture_ch());CHECK(r.hold_ms>=(BMCU_LITE_SETUP_GESTURE_1S?1000u:2000u)&&r.hold_ms<(BMCU_LITE_SETUP_GESTURE_1S?1010u:2010u));CHECK(r.release_ms>=1200);CHECK(r.progress==7);}
    if(is("any-none")){CHECK(r.flags&1);CHECK(r.candidate==255&&!r.confirmed_ms&&!r.hold_ms);}
    if(schedule&&!schedule->positive){CHECK(!(r.flags&1));CHECK(!r.samples&&!r.gen_first);}
    std::printf("FLIGHT complete phase=%u reached=%u fault=%u detail=%u window=%u samples=%u candidate=%u hold=%u release=%u gap=%u age=%u\n",r.phase,r.reached,r.fault,r.detail,unsigned(r.flags&1),r.samples,r.candidate,r.hold_ms,r.release_ms,r.max_gap_ms,r.max_age_ms);
    flight::export_frame();flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));
}
static void flight_ack(uint32_t change=0,unsigned bytes=20){
    uint8_t ack[20];std::memcpy(ack,"BFA1",4);flight::put32(ack+4,flight::build);flight::put32(ack+8,flight::session+change);
    flight::put32(ack+12,flight::frame_crc);flight::put32(ack+16,flight::crc(ack,16));
    for(unsigned i=0;i<bytes;++i)log_rx.push_back(ack[i]);
    for(unsigned i=0;i<4;++i)flight::export_step();
}
#include "continuous_boot_test.inc"
static int flight_case(const char*){
    flight_reset();std::memset(host_log,255,512);
    for(unsigned i=0;i<4096;++i)host_nvm[i]=uint8_t(i*17+3);
    uint8_t independent[4096];std::memcpy(independent,host_nvm,4096);
    if(scenario.rfind("flight-continuous-",0)==0)return continuous_boot_case();
    if(is("flight-resets")){
        CHECK(!flight::boot()&&!flight::capturing);CHECK(!(uint32_t(RCC->RSTSCKR)&0xfc000000u));
        // The POR flag from this initial boot must be cleared before export/ACK.
        flight_reset();CHECK(!flight::boot());CHECK(!(uint32_t(RCC->RSTSCKR)&0xfc000000u));
        flight_ack();CHECK(flight::acknowledged);
        std::array<uint8_t,512> armed;std::memcpy(armed.data(),host_log,512);
        for(uint32_t cause:{0u,0x04000000u,0x10000000u,0x20000000u,0x40000000u,0x80000000u,0x18000000u}){
            flight_reset();std::memcpy(host_log,armed.data(),512);RCC->RSTSCKR.value=cause|1u;
            CHECK(!flight::boot()&&!flight::capturing);CHECK(uint32_t(RCC->RSTSCKR)==1u);
            CHECK(!std::memcmp(host_log,armed.data(),512));CHECK(!std::memcmp(host_nvm,independent,4096));
        }
        // Actual main must remain inhibited after ACK followed by pin/software reset.
        for(uint32_t cause:{0x04000000u,0x10000000u}){
            flight_reset();std::memcpy(host_log,armed.data(),512);RCC->RSTSCKR.value=cause;
            bus_started=false;g_adc_dma_inited=false;host_dma1.CFGR=0;
            host_time_hook=[](){++host_clock;if(log_tx.size()>=536)throw LoopComplete{};};
            try{firmware_main();CHECK(false);}catch(const LoopComplete&){}
            CHECK(!bus_started&&!g_adc_dma_inited&&!host_dma1.CFGR&&!flight::capturing);
            CHECK(!std::memcmp(host_log,armed.data(),512));
            for(auto&row:compare_values)for(auto v:row)CHECK(v==0);
        }
        for(uint32_t cause:{0x08000000u,0x0c000000u}){
            flight_reset();std::memcpy(host_log,armed.data(),512);RCC->RSTSCKR.value=cause|1u;
            CHECK(flight::boot()&&flight::capturing&&flight::session==2);CHECK(uint32_t(RCC->RSTSCKR)==1u);
            CHECK(!std::memcmp(host_nvm,independent,4096));
        }
        std::puts("FLIGHT resets: sticky flags consumed; pin/software/watchdog/unknown inhibited; POR/PDR admits armed test");return 0;
    }
    if(is("flight-persistence")){
        for(int cut=0;cut<=30;++cut){
            flight_reset();std::memset(host_log,255,512);log_cut=cut;
            try{CHECK(!flight::boot());}catch(const PowerCut&){}
            log_cut=-1;bool blank=true;for(unsigned i=0;i<512;++i)blank=blank&&host_log[i]==255;
            flight_reset();CHECK(!flight::boot()&&!flight::capturing);CHECK(!std::memcmp(host_nvm,independent,4096));
        }
        std::puts("FLIGHT initial marker: 31 cut boundaries; virgin preparation always exports, never starts a test");
        flight_reset();std::memset(host_log,255,512);
        CHECK(!flight::boot());CHECK(flight::header_valid(host_log)&&!flight::complete(host_log));
        std::array<uint8_t,512> started;std::memcpy(started.data(),host_log,512);
        // Every byte halfword step, including commit and CRC, is interrupted once.
        unsigned cuts=0;
        for(int cut=0;cut<1000;++cut){
            flight_reset();std::memcpy(host_log,started.data(),512);flight::session=1;flight::capturing=true;log_cut=cut;
            bool interrupted=false;
            try{flight::finish_setup(7,1,9,2);}catch(const PowerCut&){interrupted=true;}
            log_cut=-1;CHECK(!std::memcmp(host_nvm,independent,4096));
            if(!interrupted){CHECK(flight::complete(host_log));cuts=cut;break;}
            if(flight::complete(host_log)){CHECK(host_log[32+112]==9);}
            flight_reset();CHECK(!flight::boot());CHECK(!flight::capturing);
        }
        CHECK(cuts>200);std::printf("FLIGHT persistence: %u cut boundaries, independent NVM unchanged\n",cuts);
        // Exact report-bound ACK; stale/wrong/partial commands cannot arm.
        flight_reset();std::memcpy(host_log,started.data(),512);CHECK(!flight::boot());
        flight_ack(1);CHECK(!flight::acknowledged);flight_ack(0,7);host_clock+=300000;flight::export_step();CHECK(!flight::acknowledged);
        flight_ack();CHECK(flight::acknowledged);std::array<uint8_t,512> armed;std::memcpy(armed.data(),host_log,512);
        CHECK(!std::memcmp(host_nvm,independent,4096));
        unsigned armcuts=0;
        // ACK itself is a word (two halfwords, split into bytes) with a torn-state check.
        for(int cut=0;cut<8;++cut){flight_reset();std::memcpy(host_log,started.data(),512);CHECK(!flight::boot());log_cut=cut;try{flight_ack();}catch(const PowerCut&){}log_cut=-1;uint16_t a;std::memcpy(&a,host_log+22,2);CHECK(a==65535||a==0xff51||a==flight::armed);}
        // Guard B before erase A; durable new A before erase B. No torn state runs.
        for(int cut=0;cut<1000;++cut){
            flight_reset();std::memcpy(host_log,armed.data(),512);log_cut=cut;bool interrupted=false;
            try{bool ran=flight::boot();CHECK(ran);}catch(const PowerCut&){interrupted=true;}
            log_cut=-1;CHECK(!std::memcmp(host_nvm,independent,4096));
            if(!interrupted){armcuts=cut;CHECK(flight::session==2);break;}
            std::array<uint8_t,512> torn;std::memcpy(torn.data(),host_log,512);flight_reset();
            const bool next=flight::boot();
            // An unchanged, exact explicitly armed record can repeat preparation;
            // once either erase/header changed persistent bytes, it must export.
            if(next)CHECK(!std::memcmp(torn.data(),armed.data(),512)||flight::header_valid(torn.data()));
            if(flight::header_valid(torn.data())){uint16_t a;std::memcpy(&a,torn.data()+22,2);if(a!=flight::armed)CHECK(!next);}
            else CHECK(!next);
        }
        CHECK(armcuts>150);std::printf("FLIGHT rearm: %u cut boundaries, no implicit motion or NVM alias\n",armcuts);

        // Corruption of every semantically protected byte fails completeness.
        flight_reset();std::memcpy(host_log,started.data(),512);flight::capturing=true;flight::finish_setup(6,1,9,2);
        CHECK(flight::complete(host_log));std::array<uint8_t,512> committed;std::memcpy(committed.data(),host_log,512);
        unsigned corruptions=0;
        for(unsigned byte=0;byte<512;++byte){
            if(byte>=20&&byte<24)continue;
            if(byte>=32+sizeof(flight::Report)&&byte<508)continue;
            std::memcpy(host_log,committed.data(),512);host_log[byte]^=1;
            CHECK(!flight::complete(host_log));flight_reset();CHECK(!flight::boot());++corruptions;
        }
        std::printf("FLIGHT corruption: %u protected byte flips rejected\n",corruptions);
        // Wrong build, exact-frame CRC, command CRC, session and interrupted RX.
        for(unsigned field:{4u,8u,12u,16u}){
            flight_reset();std::memcpy(host_log,started.data(),512);CHECK(!flight::boot());
            uint8_t a[20]={};std::memcpy(a,"BFA1",4);flight::put32(a+4,flight::build);flight::put32(a+8,flight::session);flight::put32(a+12,flight::frame_crc);flight::put32(a+16,flight::crc(a,16));a[field]^=1;
            if(field!=16)flight::put32(a+16,flight::crc(a,16));
            for(auto v:a)log_rx.push_back(v);for(unsigned n=0;n<4;++n)flight::export_step();CHECK(!flight::acknowledged);
        }
        flight_reset();std::memcpy(host_log,started.data(),512);CHECK(!flight::boot());flight_ack();CHECK(flight::acknowledged);
        const auto programs=log_programs;flight_ack();CHECK(flight::acknowledged&&programs==log_programs);CHECK(!std::memcmp(host_nvm,independent,4096));
        // A pending original NVM job denies report writes; no Boolean persistence mock.
        flight_reset();std::memcpy(host_log,started.data(),512);flight::capturing=true;g_flash_job.kind=2;
        flight::finish_setup(6,1,1,0);CHECK(!flight::complete(host_log));CHECK(!std::memcmp(host_nvm,independent,4096));
        // Exact address and live PWM gates use actual physical compare values.
        flight_reset();CHECK(!Flash_log_write(0x0800f000,0,false));CHECK(!Flash_log_write(0x0800edfc,0,false));CHECK(!Flash_log_write(0x0800ee02,0,false));CHECK(!Flash_log_write(0x0800ee04,0,true));
        compare_values[2][3]=500;CHECK(!Flash_log_write(flight::base,0,false));compare_values[2][3]=0;
        host_hang=true;host_time_hook=[](){++host_clock;};CHECK(!Flash_log_write(flight::base,0,false));CHECK(g_flash_fault);CHECK(!std::memcmp(host_nvm,independent,4096));
        flight_reset();std::memset(host_log,255,512);host_hang=true;const auto clock_before=host_clock;
        CHECK(!Flash_log_write(flight::base,0,false));CHECK(g_flash_fault&&host_clock==clock_before);
        CHECK(!std::memcmp(host_nvm,independent,4096));std::puts("FLIGHT stalled timer: actual finite wait fuse terminates, NVM retained");
        return 0;
    }
    if(is("flight-export")){
        CHECK(!flight::boot());std::array<uint8_t,512> started;std::memcpy(started.data(),host_log,512);
        for(unsigned kind=0;kind<9;++kind){
            flight_reset();std::memcpy(host_log,started.data(),512);
            if(kind==1){flight::capturing=true;flight::finish_setup(7,1,9,2);}
            if(kind==2)host_log[4]^=1; // wrong build
            if(kind==3)host_log[16]^=1;
            if(kind==4)host_log[24]=0x7a; // torn commit
            if(kind==5)host_log[0]=0xff;
            if(kind==6)host_log[8]=0; // invalid session
            if(kind==7)std::memset(host_log,0,512);
            if(kind==8)std::memset(host_log,255,512); // The flasher's virgin USB reset.
            flight_reset();host_time_hook=[](){++host_clock;if(log_tx.size()>=536)throw LoopComplete{};};
            bus_started=false;g_adc_dma_inited=false;host_dma1.CFGR=0;
            auto before=ams[0];const auto writes=host_programs,erases=host_erases;
            try{firmware_main();CHECK(false);}catch(const LoopComplete&){}
            CHECK(!bus_started&&!g_adc_dma_inited&&!host_dma1.CFGR);CHECK(host_programs==writes&&host_erases==erases);
            CHECK(!std::memcmp(&before,&ams[0],sizeof(before)));CHECK(!std::memcmp(host_nvm,independent,4096));
            for(auto&row:compare_values)for(auto v:row)CHECK(v==0);
            CHECK(log_tx.size()==536);CHECK(flight::crc(log_tx.data(),532)==flight::get32(log_tx.data()+532));
            if(kind==0)flight_save("incomplete.frame",log_tx.data(),536);
        }
        std::puts("FLIGHT export: nine actual-main routes including virgin preparation, literal eight-zero, no bus/ADC/CAL/owner mutation");return 0;
    }
    CHECK(false);return 1;
}
