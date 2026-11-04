#if BMCU_LITE_V11_FEED_LATCH && BMCU_LITE_AUTHORITY_OBSERVATION && BMCU_USB_FLIGHT_RECORDER
#include "baseline_serializers.h"
// Uses the production parser/main/controller/Flash/DMA harness. Only named raw
// input/MMIO/time stimuli and explicit RAM boundary injections below are synthetic.
static bool phase_has(const char* p){return scenario.find(p)!=std::string::npos;}
static void phase_extension(uint8_t* out){
    authority_observation::snapshot(out);CHECK(package_check_crc16(out,80));
#if BMCU_LITE_NATIVE_PHASE_COMPAT
    CHECK(out[61]==0xa1&&out[60]<=2);for(unsigned i=70;i<78;++i)CHECK(!out[i]);
#else
    CHECK(out[60]==authority_observation::post_count&&out[61]==authority_observation::post_flags);
#endif
}
static void phase_no_effect(unsigned ch,unsigned motion,bool parser_only=false){
    const auto stamp=forward_received;const auto grants=authority_observation::count;
    const auto own=owner,durable=durable_owner,selected=ams[0].now_filament_num,flag=ams[0].filament_use_flag;
    const auto journal=intent;const bool waiting=waiting_send,grant=forward_grant;
    const auto current=ams[0].filament[au_ch].motion;
    const auto native=flight::report.flags&128u;
    if(parser_only)full_rx(full_motion(ch,7,motion,au_cmd));else au_request(ch,7,motion,au_cmd);
    CHECK(full_load_fault()==FullLoadFault::none&&forward_received==stamp&&authority_observation::count==grants);
    CHECK(owner==own&&durable_owner==durable&&intent==journal&&waiting_send==waiting&&forward_grant==grant);
    CHECK(ams[0].now_filament_num==selected&&ams[0].filament_use_flag==flag&&ams[0].filament[au_ch].motion==current);
    CHECK((flight::report.flags&128u)==native);
}
static void phase_persist_export(){
    CHECK(full_load_fault()!=FullLoadFault::none&&full_stopped());
    repair_step();CHECK(host_terminal_seen&&flight::finished&&flight::complete(host_log));
    uint8_t current[80];phase_extension(current);CHECK(!std::memcmp(host_log+32+164,current,80));
    flight::export_frame();flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));
    std::array<uint8_t,512> saved{};std::memcpy(saved.data(),host_log,512);
    std::array<uint8_t,4096> nvm{};std::memcpy(nvm.data(),host_nvm,4096);
    flight_reset(); // RAM loss includes the new history's existing observation buffers.
    std::memset(authority_observation::history,0,8);authority_observation::history_count=0;
    std::memset(authority_observation::records,0,52);authority_observation::mask=0;authority_observation::terminal_fault=0;
    bus_started=false;g_adc_dma_inited=false;host_dma1.CFGR=0;
    host_time_hook=[](){++host_clock;if(log_tx.size()>=536)throw LoopComplete{};};
    try{firmware_main();CHECK(false);}catch(const LoopComplete&){}
    CHECK(!bus_started&&!g_adc_dma_inited&&!host_dma1.CFGR);
    CHECK(!std::memcmp(saved.data(),host_log,512)&&!std::memcmp(nvm.data(),host_nvm,4096));
    CHECK(log_tx.size()==536&&!std::memcmp(log_tx.data()+20,saved.data(),512));
    for(auto& row:compare_values)for(auto v:row)CHECK(!v);
}
static int phase_verify(){
    au_ch=phase_has("ch0")?0u:phase_has("ch1")?1u:phase_has("ch3")?3u:2u;au_cmd=phase_has("sender04")?4u:3u;
    const auto path=std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv";
    au_trace=std::fopen(path.c_str(),"wx");CHECK(au_trace);
    host_clock=((host_clock+999)/1000)*1000;
    host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto& p:full_pulls)p=float(2048)*(3.3f/4095.f);
    repair_request(short_packet(0x20));au_tick(20);CHECK(v11_local_eligible());
    if(phase_has("no-wire")){
        full_load_fail(FullLoadFault::controller);uint8_t out[80];phase_extension(out);
        CHECK(!out[60]&&out[6]==uint8_t(FullLoadFault::controller));phase_persist_export();return au_done();
    }
    if(phase_has("unowned")||phase_has("local")){
        full_keys[au_ch]=phase_has("local")?1.55f:1.8f;au_tick(phase_has("local")?180:10);
        const auto helper=dm_auto_state[au_ch];if(phase_has("local"))CHECK(helper==DM_AUTO_S1_PUSH&&v11_pwm(au_ch));
        const auto writes=host_programs;phase_no_effect(au_ch,0x26);phase_no_effect(au_ch,0x66);
        CHECK(!forward_grant&&full_load_empty()&&host_programs==writes&&dm_auto_state[au_ch]==helper);
        if(phase_has("local"))CHECK(v11_pwm(au_ch));return au_done();
    }
    if(phase_has("pending")){
        full_keys[au_ch]=1.8f;au_tick(10);au_bf(au_cmd);
        CHECK(waiting_send&&full_load_journal_pending()&&!full_load_durable(au_ch));
        // Parser-only dispatch keeps the ongoing production journal at the same
        // boundary; phase traffic must not cancel/renew it or create selection.
        const auto stamp=forward_received;const auto grants=authority_observation::count;
        const auto writes=host_programs;const auto report_flags=flight::report.flags;
        for(unsigned m:{0x26u,0x66u}){full_rx(full_motion(au_ch,7,m,au_cmd));CHECK(waiting_send&&intent==JournalIntent::reserve&&ams[0].now_filament_num==255);}
        CHECK(forward_received==stamp&&authority_observation::count==grants&&host_programs==writes&&flight::report.flags==report_flags);
        CHECK(!ams[0].filament_use_flag&&full_stopped());return au_done();
    }
    au_start();latch_live();
    const auto stamp=forward_received;const auto grants=authority_observation::count;const auto writes=host_programs;
#if BMCU_LITE_NATIVE_PHASE_COMPAT
    if(phase_has("serializer")){
        unsigned compared=0;
        for(unsigned selected:{0u,1u,2u,3u,255u})for(unsigned flag:{0u,2u,4u}){
            ams[0].now_filament_num=selected;ams[0].filament_use_flag=flag;
            const auto before=ams[0];const auto packet=full_motion(au_ch,7,0x66,au_cmd);
            package_num=3;bus_port_to_host.send_data_len=0;
            if(au_cmd==3)baseline_get_package_motion((bambubus_printer_motion_package_struct*)packet.data(),packet.size());
            else baseline_get_package_stu_motion((bambubus_printer_stu_motion_package_struct*)packet.data(),packet.size());
            const unsigned n=bus_port_to_host.send_data_len;CHECK(n==(au_cmd==3?44:60));
            const auto* out=bus_port_to_host.tx_build_buf();const Packet expected(out,out+n);const auto old_meters=ams[0];
            ams[0]=before;package_num=3;bus_port_to_host.send_data_len=0;
            if(au_cmd==3)get_package_motion((bambubus_printer_motion_package_struct*)packet.data(),packet.size());
            else get_package_stu_motion((bambubus_printer_stu_motion_package_struct*)packet.data(),packet.size());
            CHECK(bus_port_to_host.send_data_len==n&&!std::memcmp(expected.data(),bus_port_to_host.tx_build_buf(),n));
            CHECK(package_check_crc16(bus_port_to_host.tx_build_buf(),n));
            for(unsigned ch=0;ch<4;++ch)CHECK(ams[0].filament[ch].meters==old_meters.filament[ch].meters);
            ++compared;
        }
        bus_port_to_host.send_data_len=0;CHECK(compared==15);std::printf("SERIALIZER exact baseline body comparisons %u\n",compared);return au_done();
    }
#endif

#if !BMCU_LITE_NATIVE_PHASE_COMPAT
    CHECK(phase_has("old26")||phase_has("old66"));
    au_request(au_ch,7,phase_has("old26")?0x26:0x66,au_cmd);
    CHECK(full_load_fault()==FullLoadFault::unsupported&&!forward_grant&&full_stopped());return au_done();
#else
    if(phase_has("foreign")){
        full_keys[(au_ch+1)%4]=1.8f;au_tick(10);
        phase_no_effect((au_ch+1)%4,0x26);phase_no_effect((au_ch+1)%4,0x66);
        CHECK(host_programs==writes);latch_live();return au_done();
    }
    if(phase_has("gate-")){
        if(phase_has("storage")||phase_has("metadata")){
            au_request(au_ch,7,0,au_cmd);CHECK(!forward_grant);
            ams[0].filament_use_flag=2; // Distinguish an attempted flag-only response change.
            if(phase_has("storage"))g_flash_fault=true;else metadata_recovery=true;
        }
        // Explicit RAM/input boundary injections isolate guards; each case remains
        // distinct from a claim that a printer can manufacture this state.
        if(phase_has("selection"))ams[0].now_filament_num=255;
        if(phase_has("durable"))durable_owner=255;
        if(phase_has("calibration"))g_valid_mask&=uint8_t(~(1u<<au_ch));
        if(phase_has("absent"))filament_channel_inserted[au_ch]=false;
        if(phase_has("idle")){full_load_cancel();ams[0].filament[au_ch].motion=_filament_motion::idle;}
        if(phase_has("pull")){au_request(au_ch,3,0x3f,au_cmd);}
        const auto flag=ams[0].filament_use_flag;
        phase_no_effect(au_ch,0x26,true);phase_no_effect(au_ch,0x66,true);CHECK(ams[0].filament_use_flag==flag);
        CHECK(host_programs==writes);return au_done();
    }
    if(phase_has("lease")){
        const uint64_t deadline=bambubus_heartbeat_deadline;
        while(host_clock+1000<=deadline){delay(1);repair_step();}
        host_clock=deadline;raw_dma_boundary();au_request(au_ch,7,0x26,au_cmd);latch_live();CHECK(lite_motion_ready());
        ++host_clock;raw_dma_boundary();au_request(au_ch,7,0x66,au_cmd);
        CHECK(!forward_grant&&full_stopped()&&ams[0].now_filament_num==255&&full_load_fault()==FullLoadFault::none);
        repair_request(short_packet(0x20));phase_no_effect(au_ch,0x26);phase_no_effect(au_ch,0x66);
        CHECK(!forward_grant&&forward_received==stamp&&full_stopped());return au_done();
    }
    if(phase_has("fault")){
        full_load_fail(FullLoadFault::controller);const auto first=full_load_fault();
        // Actual classifier entry after a preexisting fault cannot revive it.
        set_motion(au_ch,7,0x26,0,au_cmd);set_motion(au_ch,7,0x66,0,au_cmd);
        CHECK(full_load_fault()==first&&!forward_grant&&host_programs==writes&&full_stopped());
        phase_persist_export();return au_done();
    }
    if(phase_has("history")||phase_has("unrelated")){
        full_pulls[au_ch]=float(2310)*(3.3f/4095.f);au_tick(40);CHECK(MOTOR_CONTROL[au_ch].send_stop_latch);
        full_pulls[au_ch]=float(2345)*(3.3f/4095.f);au_tick(40);CHECK(full_stopped());
        CHECK(authority_observation::mask&2u);CHECK(authority_observation::records[1][1]==authority_observation::hold_zero);
        CHECK(!authority_observation::terminal_fault);
        std::array<uint8_t,52> old{};std::memcpy(old.data(),authority_observation::records,52);
        const auto frozen_count=authority_observation::count;
        au_request(au_ch,7,0x26,au_cmd);CHECK(ams[0].filament_use_flag==4);
        au_request(au_ch,7,0x66,au_cmd);CHECK(forward_grant);
        if(phase_has("unrelated"))full_load_fail(FullLoadFault::controller);
        else au_request(au_ch,9,0x7f,au_cmd); // Standard AMS is not Lite evidence.
        CHECK(full_load_fault()==(phase_has("unrelated")?FullLoadFault::controller:FullLoadFault::unsupported));
        uint8_t out[80];phase_extension(out);CHECK(out[60]==2&&out[7]==2);
        CHECK(out[62]==au_cmd&&out[63]==au_ch);
        CHECK(out[64]==(phase_has("unrelated")?7:9)&&out[65]==(phase_has("unrelated")?0x66:0x7f));
        CHECK(out[66]==au_cmd&&out[67]==au_ch&&out[68]==7&&out[69]==(phase_has("unrelated")?0x26:0x66));
        CHECK(!std::memcmp(old.data(),authority_observation::records,52)&&authority_observation::count==frozen_count);
        authority_observation::wire(4,(au_ch+1)%4,3,0xbf);full_load_fail(FullLoadFault::capability);
        uint8_t later[80];phase_extension(later);CHECK(!std::memcmp(out,later,80));
        phase_persist_export();return au_done();
    }
    if(phase_has("unknown")){
        const unsigned state=phase_has("09")?9u:7u;
        const unsigned motion=phase_has("a5")?0xa5u:phase_has("3f")?0x3fu:0x7fu;
        au_request(au_ch,state,motion,au_cmd);CHECK(full_load_fault()==FullLoadFault::unsupported&&!forward_grant&&full_stopped());
        uint8_t out[80];phase_extension(out);CHECK(out[60]==2&&out[64]==state&&out[65]==motion);
        phase_persist_export();return au_done();
    }
    if(phase_has("invalid-channel")){
        au_request(4,7,0x26,au_cmd);CHECK(full_load_fault()==FullLoadFault::unsupported&&full_stopped());return au_done();
    }
    // Both historical phases preserve send state and grant age; only 26 changes
    // the flag. Original V11 07/00 remains the only on_use transition here.
    phase_no_effect(au_ch,0x66);
    au_request(au_ch,7,0x26,au_cmd);latch_live();CHECK(ams[0].filament_use_flag==4);
    CHECK(forward_received==stamp&&authority_observation::count==grants&&host_programs==writes);
    au_tick(1100);latch_live();CHECK(forward_received==stamp);
    au_request(au_ch,7,0,au_cmd);CHECK(!forward_grant&&ams[0].filament[au_ch].motion==_filament_motion::on_use);
    phase_no_effect(au_ch,0x66);phase_no_effect(au_ch,0x26);
    CHECK(host_programs==writes&&full_load_durable(au_ch));return au_done();
#endif
}
#endif
