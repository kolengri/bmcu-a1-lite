// Bounded actual-body footprint experiment evidence, not a release acceptance suite.
#if BMCU_USB_FLIGHT_RECORDER && BMCU_LITE_CONTINUOUS_NATIVE
static void ud_global(unsigned cmd,unsigned motion,bool expected_global){
    const auto before=ams[0];const auto flags=flight::report.flags,np=host_programs,lp=log_programs;
    const auto count=authority_observation::count;const auto stamp=forward_received;
    if(expected_global&&(flight::report.flags&oi_seen))oi_packet(cmd,255,7,motion,true);
    else au_request(255,7,motion,cmd);
    const unsigned o=cmd==3?7:19;
    CHECK(full_reply.size()==(cmd==3?44:60)&&package_check_crc16(full_reply.data(),full_reply.size()));
    CHECK(full_reply[o]==4&&full_reply[o+1]==(expected_global?255u:au_ch));
    if(expected_global)CHECK(!std::memcmp(full_reply.data()+o+2,"\0\0\0\0",4)&&full_reply[o+6]==255&&full_reply[o+7]==255);
    CHECK(ams[0].now_filament_num==au_ch&&ams[0].filament[au_ch].motion==_filament_motion::on_use&&ams[0].filament_use_flag==4);
    CHECK(full_load_durable(au_ch)&&!forward_grant&&authority_observation::count==count&&forward_received==stamp);
    CHECK(host_programs==np&&log_programs==lp&&flight::report.flags==flags&&!flight::finished&&!flight::native_checkpoint_pending());
    if(expected_global)CHECK(!std::memcmp(&before,&ams[0],sizeof(before)));
}
#include "lease_test.inc"
static int unload_diagnostic_verify(){
    au_ch=oi_has("owner0")?0:oi_has("owner1")?1:oi_has("owner3")?3:2;au_cmd=oi_has("sender04")?4:3;
    au_trace=std::fopen((std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv").c_str(),"wx");CHECK(au_trace);
    host_clock=((host_clock+999)/1000)*1000;host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto& p:full_pulls)p=float(2048)*(3.3f/4095.f);repair_request(short_packet(0x20));au_tick(20);
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
    if(oi_has("cold-no-provenance")){
        CHECK(full_load_durable(2)&&!(flight::report.flags&128));full_pulls[1]=1.4f;au_tick(5200);
        CHECK(!flight::operator_pending()&&!flight::finished&&full_load_durable(2)&&!host_programs);return oi_export();
    }
    if(oi_has("gesture-held"))full_pulls[(au_ch+1)%4]=1.4f;
#endif
    if(oi_has("lease-"))return lease_verify();
    oi_start();
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
    if(oi_has("operator-recovery-order")){
        au_request(au_ch,3,0x3f,au_cmd);au_tick(3);CHECK(lite_pull_state[au_ch]==LitePullState::active);
        // Keep a valid long-frame header partially receiving with real RX
        // progress, then let its existing quiet timeout expire inside stop.
        Packet prefix{0x3d,0,0,0,0,5,0};prefix[6]=bus_crc8(prefix.data(),6);append(prefix);
        bambubus_heartbeat_deadline=time_ticks32()-1;
        auto step=[](){try{full_load_step();}catch(const TerminalCapture&){}};
        step();CHECK(lite_recovery_fault()&&ams[0].now_filament_num==255&&!flight::finished);
        for(unsigned n=0;n<10;++n){delay(1);if(n%5==0)append(Packet{0});step();}
        CHECK(!flight::finished&&!bus_background_ready());
        const auto lp=log_programs,np=host_programs;std::array<uint8_t,4096> retained{};std::memcpy(retained.data(),host_nvm,4096);
        static bool completed=false;completed=false;
        host_compare_hook=[](int,int,uint16_t){if(flight::operator_pending()&&!completed){completed=true;host_clock+=us_to_ticks32(wire_time_us(1282u))+1u;bus_rx_poll();CHECK(!bus_port_to_host.receiving()&&bus_port_to_host.idle);}};
        full_pulls[(au_ch+1)%4]=1.4f;
        for(unsigned n=0;n<5300&&!flight::operator_pending()&&!flight::finished;++n){delay(1);if(n%5==0)append(Packet{0});step();}
        host_compare_hook=nullptr;
        std::printf("ORDER witness completed=%d pending=%d eligible=%d finished=%d fault=%u recovery=%d selected=%u flags=%u writes=%u/%u\n",completed,flight::operator_pending(),flight::operator_eligible(),flight::finished,unsigned(full_load_fault()),lite_recovery_fault(),ams[0].now_filament_num,flight::report.flags,log_programs,lp);
        for(unsigned c=0;c<4;++c)std::printf("ORDER ch%u key=%d raw=%f pct=%u dm=%u arm=%u active=%u gst=%u/%u\n",c,Motion_control_filament_present(c),MC_PULL_stu_raw[c],unsigned(MC_PULL_pct[c]),unsigned(dm_auto_state[c]),auto_unload_arm[c],auto_unload_active[c],gst_active[c],gst_step[c]);std::fflush(stdout);
        CHECK(completed&&flight::operator_pending());
        CHECK(!flight::finished&&!flight::saved()&&log_programs==lp&&bus_started&&!diagnostic_green_deliveries&&!diagnostic_amber_deliveries);
        step();CHECK(flight::saved()&&flight::complete(host_log)&&!bus_started&&full_stopped()&&diagnostic_amber_deliveries&&diagnostic_green_deliveries);
        CHECK(full_load_owner()==au_ch&&host_programs==np&&!std::memcmp(retained.data(),host_nvm,4096));
        CHECK(!full_load_ready()&&!lite_motion_ready());return oi_export();
    }
    if(oi_has("refusal-")){
        using R=flight::PullRefusal;R expected=R::loaded_mismatch;unsigned request_ch=au_ch;
        if(oi_has("offline")){ams[0].online=false;expected=R::offline;}
        if(oi_has("not-ready")){bambubus_heartbeat_reset_armed=false;expected=R::not_ready;}
        if(oi_has("metadata")){metadata_recovery=true;expected=R::metadata;}
        if(oi_has("journal")){intent=JournalIntent::reserve;expected=R::journal;}
        if(oi_has("loaded")){request_ch=(au_ch+1)%4;expected=R::loaded_mismatch;}
        if(oi_has("selection")){ams[0].now_filament_num=(au_ch+1)%4;expected=R::selection_mismatch;}
        if(oi_has("pull-state")){lite_pull_state[au_ch]=LitePullState::completed;expected=R::pull_state;}
        if(oi_has("absent")){full_keys[au_ch]=.2f;for(unsigned n=0;n<64;++n){delay(1);ADC_DMA_poll();}CHECK(!Motion_control_filament_present(au_ch));expected=R::absent;}
        if(oi_has("nondurable")){durable_owner=255;expected=R::nondurable;}
        full_rx(full_motion(request_ch,3,0x3f,au_cmd));
        if(!full_reply.empty()){bus_port_to_host.send_data_len=uint16_t(full_reply.size());bus_port_to_host.send_package();repair_complete_tx();}
        auto& d=flight::diagnostic();CHECK(d.seen_count==1&&d.first_refusal==uint8_t(expected)&&d.refusal_command==au_cmd&&d.refusal_channel==request_ch);
        CHECK(!(d.seen_flags&6)&&!d.pwm_observed_mask&&!(flight::report.flags&oi_mask));
        if(expected==R::not_ready)CHECK(d.refusal_context&1);
        if(expected==R::absent){repair_step();CHECK(full_load_fault()==FullLoadFault::inputs&&flight::saved()&&flight::report.input_detail==uint16_t(18u|(au_ch<<8)));return oi_export();}
        ams[0].online=true;metadata_recovery=false;intent=JournalIntent::none;durable_owner=uint8_t(au_ch);ams[0].now_filament_num=uint8_t(au_ch);lite_pull_state[au_ch]=LitePullState::ready;
        repair_request(short_packet(0x20));au_request(au_ch,3,0x3f,au_cmd);au_tick(5);
        CHECK(d.first_refusal==uint8_t(expected)&&d.seen_count==2&&(d.seen_flags&6)==6&&d.admitted_channel==au_ch);
        CHECK(d.first_tx==(expected==R::offline?0u:3u));
        flight::operator_request(uint8_t((au_ch+1)%4));repair_step();CHECK(flight::saved()&&flight::complete(host_log)&&full_stopped());return oi_export();
    }
    if(oi_has("gesture-")){
        const unsigned candidate=oi_has("selected")?au_ch:(au_ch+1)%4;
        const auto writes=host_programs;std::array<uint8_t,4096> saved_nvm{};std::memcpy(saved_nvm.data(),host_nvm,4096);
        if(oi_has("present")){full_keys[candidate]=1.8f;au_tick(10);}
        if(oi_has("helper"))auto_unload_arm[(au_ch+2)%4]=1;
        if(oi_has("reservation"))intent=JournalIntent::reserve;
        if(oi_has("lease-inactive"))bambubus_heartbeat_reset_armed=false;
        full_pulls[candidate]=1.4f;
        if(oi_has("short")){au_tick(4000);full_pulls[candidate]=1.65f;au_tick(20);}
        else if(oi_has("noisy")){for(unsigned n=0;n<6;++n){full_pulls[candidate]=1.4f;au_tick(800);full_pulls[candidate]=1.65f;au_tick(20);}}
        else if(oi_has("reservation")||oi_has("helper")){
            // Exercise the actual gesture/controller body while retaining the
            // deliberately denied helper/reservation state, without a fake journal dispatch.
            for(unsigned n=0;n<5200;++n){delay(1);Motion_control_run(0);}
        }
        else if(oi_has("lease-inactive"))for(unsigned n=0;n<5300&&!flight::finished;++n){delay(1);repair_step();}
        else au_tick(5200);
        if(oi_has("lease-inactive")){
            CHECK(flight::operator_pending()&&flight::saved()&&flight::complete(host_log));
            CHECK(!(flight::diagnostic().before_flags&4)&&diagnostic_green_deliveries);return oi_export();
        }
        CHECK(!flight::operator_pending()&&host_programs==writes&&!std::memcmp(saved_nvm.data(),host_nvm,4096));
        if(!oi_has("selected"))CHECK(!flight::finished&&!flight::complete(host_log));
        return oi_export();
    }
#endif
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
    ud_global(au_cmd,0,true);CHECK(!(flight::report.flags&oi_seen));
#endif
    au_request(255,1,0,au_cmd);CHECK(flight::report.flags&oi_seen);
    if(oi_has("base-alternation")){
        const unsigned off=au_cmd==3?7:19;CHECK(full_reply[off]==4&&full_reply[off+1]==255);
        ud_global(au_cmd,0,false);CHECK(!flight::complete(host_log));
        au_request(255,7,0x99,au_cmd);CHECK(full_load_fault()==FullLoadFault::unsupported&&flight::complete(host_log));return oi_export();
    }
    if(oi_has("base-header-only")){CHECK(!flight::complete(host_log)&&!flight::finished);return oi_export();}
    for(unsigned n=0;n<3;++n)for(unsigned cmd:{3u,4u})for(unsigned motion:{0u,0x26u,0x66u,0x99u})ud_global(cmd,motion,true);
    for(unsigned cmd:{3u,4u}){
        for(unsigned ch=0;ch<4;++ch)oi_packet(cmd,ch,7,0x99,false);
        oi_packet(cmd,au_ch,7,0x26,false);oi_packet(cmd,au_ch,7,0x66,false);
        oi_packet(cmd,au_ch,3,0xbf,false);oi_packet(cmd,255,1,0,false);oi_packet(cmd,au_ch,7,0,false);
    }
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
    CHECK(!flight::diagnostic().seen_count&&!flight::diagnostic().seen_flags);
    if(oi_has("cut-at")){
        const unsigned cut=unsigned(std::stoul(scenario.substr(scenario.find("cut-at")+6)));
        flight::operator_request(uint8_t((au_ch+1)%4));log_cut=int(cut);bool interrupted=false;
        try{flight::finish_native(0);}catch(const PowerCut&){interrupted=true;}log_cut=-1;
        CHECK(flight::operator_pending()&&full_stopped()&&!full_load_ready()&&!lite_motion_ready());
        CHECK(!diagnostic_green_deliveries);
        if(interrupted)CHECK(!flight::saved());else CHECK(flight::saved()&&flight::complete(host_log));
        return oi_export();
    }
    if(oi_has("program-fail")){
        flight::operator_request(uint8_t((au_ch+1)%4));host_fail_program=true;
        CHECK(!flight::finish_native(0)&&!flight::saved()&&!flight::complete(host_log));host_fail_program=false;
#if !BMCU_USB_CONTINUOUS_JOURNAL
        try{capture_park();}catch(const TerminalCapture&){}CHECK(!diagnostic_green_deliveries&&full_stopped()&&!bus_started);
#endif
        return oi_export();
    }
    if(oi_has("busy-wait")){
        flight::operator_request(uint8_t((au_ch+1)%4));FLASH->STATR|=FLASH_FLAG_BSY;static unsigned waits=0;waits=0;
        host_time_hook=[](){++host_clock;raw_dma_boundary();if(FLASH->STATR&FLASH_FLAG_BSY){CHECK(flight::operator_pending()&&!flight::saved()&&full_stopped()&&!full_load_ready()&&!lite_motion_ready());if(++waits==20)FLASH->STATR&=~FLASH_FLAG_BSY;}};
        repair_step();CHECK(waits==20&&flight::saved()&&flight::complete(host_log)&&diagnostic_green_deliveries&&diagnostic_amber_deliveries);host_time_hook=[](){++host_clock;raw_dma_boundary();};return oi_export();
    }
    if(oi_has("busy-denied")){
        flight::operator_request(uint8_t((au_ch+1)%4));CHECK(flight::operator_pending()&&full_stopped());
        const auto writes=log_programs;g_flash_job.kind=2;
#if BMCU_LITE_LEASE_FEEDBACK
        CHECK(!flight::finish_native(0)&&!flight::finished&&!flight::saved()&&!flight::complete(host_log)&&log_programs==writes);
        g_flash_job={};repair_step();CHECK(flight::saved()&&flight::complete(host_log)&&!bus_started&&full_stopped());return oi_export();
#else
        CHECK(!flight::finish_native(0)&&flight::finished&&!flight::saved()&&!flight::complete(host_log)&&log_programs==writes);
        g_flash_job={};CHECK(!flight::finish_native(6)&&!flight::saved()&&!flight::complete(host_log)&&log_programs==writes);
#endif
        const auto before=ams[0];lite_set_motion(au_ch,7,0,au_cmd);CHECK(!std::memcmp(&before,&ams[0],sizeof(before)));
        CHECK(!full_load_ready()&&!lite_motion_ready());return au_done();
    }
    if(oi_has("cancel-later")){
        au_request(255,1,1,au_cmd);au_tick(2);CHECK(flight::report.flags&64);
        CHECK(!flight::finished&&!flight::complete(host_log));
        au_request(au_ch,7,0,au_cmd);CHECK(ams[0].now_filament_num==au_ch);
        flight::operator_request(uint8_t((au_ch+1)%4));repair_step();CHECK(flight::saved()&&flight::complete(host_log));
        return oi_export();
    }
    if(oi_has("pull")){
        au_request((au_ch+1)%4,3,0x3f,au_cmd);
        auto& d=flight::diagnostic();CHECK(d.seen_count==1&&d.first_refusal==uint8_t(flight::PullRefusal::loaded_mismatch)&&!(flight::report.flags&oi_mask));
        CHECK(d.first_tx==3&&d.response_command==au_cmd&&d.response_length==(au_cmd==3?44u:60u));
        au_request(au_ch,3,0x3f,au_cmd);au_tick(5);
        CHECK(d.seen_count==2&&(d.seen_flags&2)&&d.admitted_channel==au_ch&&(flight::report.flags&oi_mask)==(1u<<(16u+au_ch)));
        CHECK(d.seen_flags&4);CHECK(d.first_pwm[0]!=d.first_pwm[1]&&d.pwm_channel==au_ch);
        return au_done();
    }
    if(oi_has("manual")){
        const unsigned gesture=(au_ch+1)%4;const auto np=host_programs;
        const auto before_owner=owner;std::array<uint8_t,4096> nvm{};std::memcpy(nvm.data(),host_nvm,4096);
        au_tick(10);full_pulls[gesture]=1.4f;au_tick(4990);CHECK(!flight::operator_pending()&&!flight::finished);
        for(unsigned n=0;n<300&&!flight::finished;++n)au_tick();
        CHECK(flight::operator_pending()&&flight::saved()&&flight::complete(host_log)&&full_stopped()&&!bus_started);
        CHECK(diagnostic_green_deliveries&&diagnostic_amber_deliveries);
        for(const WS2812_class* led:{&SYS_RGB,&RGBOUT[0],&RGBOUT[1],&RGBOUT[2],&RGBOUT[3]}){
            auto last=std::find_if(deliveries.rbegin(),deliveries.rend(),[&](const LedEvent&e){return e.led==led;});
            CHECK(last!=deliveries.rend()&&last->rgb[0]==0x002000u);if(led!=&SYS_RGB)CHECK(last->rgb[1]==0x002000u);
        }
        auto& d=flight::diagnostic();CHECK(d.version==1&&d.snapshot_kind==1&&d.gesture_channel==gesture&&d.before_owner==before_owner&&d.before_selected==au_ch&&d.before_motion==uint8_t(_filament_motion::on_use));
        CHECK(!d.seen_count&&!(d.seen_flags&1)&&!(flight::report.flags&oi_mask));
        CHECK(owner==before_owner&&host_programs==np&&!std::memcmp(nvm.data(),host_nvm,4096));
        const auto writes=log_programs;CHECK(flight::finish_native(0)&&flight::saved()&&log_programs==writes);
        return oi_export();
    }
#endif
    return oi_export();
}
#endif
