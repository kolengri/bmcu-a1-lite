#if BMCU_LITE_V11_FEED_LATCH && BMCU_LITE_AUTHORITY_OBSERVATION && BMCU_USB_FLIGHT_RECORDER
// Synthetic schedules through the production retained-owner boot/parser/main,
// journal, controller, PWM and recorder. These are not physical Unload evidence.
#include "exact5_serializers.h"
static int global_serializers();
static bool global_has(const char* p){return scenario.find(p)!=std::string::npos;}
static void global_no_effect(unsigned m){
    auto before=ams[0];if(before.now_filament_num<4)before.filament[before.now_filament_num].meters=Motion_control_filament_meters(before.now_filament_num);const auto own=owner,durable=durable_owner;
    const auto journal=intent;const bool pending=waiting_send,grant=forward_grant;
    const auto stamp=forward_received;const auto count=authority_observation::count;
    const auto native=flight::report.flags&128u;const auto programs=host_programs;
    const auto first=full_load_fault();std::array<std::array<int,4>,3> pwm{};for(unsigned t=0;t<3;++t)for(unsigned c=0;c<4;++c)pwm[t][c]=compare_values[t][c];
    // Actual parser plus serializer, isolated from another main/controller tick.
    full_rx(full_motion(255,7,m,au_cmd));
    CHECK(!std::memcmp(&before,&ams[0],sizeof(before)));
    CHECK(owner==own&&durable_owner==durable&&intent==journal&&waiting_send==pending&&forward_grant==grant);
    CHECK(forward_received==stamp&&authority_observation::count==count&&host_programs==programs);
    CHECK((flight::report.flags&128u)==native&&full_load_fault()==first);
    for(unsigned t=0;t<3;++t)for(unsigned c=0;c<4;++c)CHECK(compare_values[t][c]==pwm[t][c]);
}
static int global_phase_verify(){
    au_ch=2;au_cmd=global_has("sender04")?4u:3u;
    au_trace=std::fopen((std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv").c_str(),"wx");CHECK(au_trace);
    host_clock=((host_clock+999)/1000)*1000;
    host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto& p:full_pulls)p=float(2048)*(3.3f/4095.f);
    repair_request(short_packet(0x20));au_tick(20);
    CHECK(full_load_owner()==2&&full_load_durable(2)&&!v11_local_eligible());
    CHECK(ams[0].now_filament_num==255&&!ams[0].filament_use_flag&&full_stopped());
    CHECK(!forward_grant&&!waiting_send&&!authority_observation::count&&!(flight::report.flags&128u));
    for(auto& f:ams[0].filament)CHECK(f.motion==_filament_motion::idle);
    if(global_has("serializers"))return global_serializers();
    au_request(255,1,0,au_cmd);
    CHECK(full_load_fault()==FullLoadFault::none&&full_load_durable(2)&&full_stopped());
    if(global_has("base26")||global_has("base66")){
        au_request(255,7,global_has("base66")?0x66:0x26,au_cmd);
        CHECK(full_load_fault()==FullLoadFault::unsupported&&!forward_grant&&full_stopped());
        uint8_t h[80];phase_extension(h);CHECK(h[60]==2&&h[62]==au_cmd&&h[63]==255&&h[64]==7&&h[65]==(global_has("base66")?0x66:0x26));
        CHECK(h[66]==au_cmd&&h[67]==255&&h[68]==1&&h[69]==0);
        phase_persist_export();return au_done();
    }
#if BMCU_LITE_GLOBAL_PHASE_COMPAT
    global_no_effect(0x26);global_no_effect(0x66);
    CHECK(!flight::finished&&!flight::native_checkpoint_pending()&&!flight::complete(host_log));
    CHECK(!host_programs&&!host_erases&&full_stopped());
    if(global_has("invalid")||global_has("unknown")||global_has("cancel")){
        const unsigned ch=global_has("invalid4-")?4u:global_has("invalid254-")?254u:255u;
        const unsigned state=global_has("unknown-state")?6u:global_has("cancel")?0u:7u;
        const unsigned m=global_has("unknown-motion")?0x27u:global_has("cancel")?0u:global_has("66-")?0x66u:0x26u;
        au_request(ch,state,m,au_cmd);
        CHECK(full_load_fault()==FullLoadFault::unsupported&&full_stopped()&&!forward_grant);
        phase_persist_export();return au_done();
    }
    if(global_has("terminal")){
        full_load_fail(FullLoadFault::controller);const auto first=full_load_fault();
        // The normal terminal loop stops servicing input. Exercise classifier
        // entry explicitly to prove that even forced later dispatch cannot revive.
        set_motion(255,7,0x26,0,au_cmd);set_motion(255,7,0x66,0,au_cmd);
        CHECK(full_load_fault()==first&&full_stopped()&&!forward_grant);
        phase_persist_export();return au_done();
    }
    if(global_has("lease")){
        bambubus_heartbeat_reset_armed=false;
        global_no_effect(0x26);global_no_effect(0x66);
        set_motion(2,3,0x3f,0,au_cmd);
        CHECK(full_load_fault()==FullLoadFault::none&&ams[0].now_filament_num==255&&full_stopped());
        CHECK(!forward_grant&&full_load_durable(2));return au_done();
    }
    if(global_has("metadata")){
        metadata_recovery=true;global_no_effect(0x26);global_no_effect(0x66);
        set_motion(2,3,0x3f,0,au_cmd);
        CHECK(metadata_recovery&&ams[0].now_filament_num==255&&full_stopped()&&full_load_durable(2));
        CHECK(full_load_fault()==FullLoadFault::none&&!forward_grant&&!host_programs);return au_done();
    }
    if(global_has("flash")){
        g_flash_fault=true;global_no_effect(0x26);global_no_effect(0x66);
        repair_step();CHECK(full_load_fault()==FullLoadFault::storage&&full_stopped());
        CHECK(!flight::complete(host_log));
        set_motion(255,7,0x26,0,au_cmd);set_motion(2,3,0x3f,0,au_cmd);
        CHECK(full_load_fault()==FullLoadFault::storage&&full_stopped()&&!forward_grant);
        CHECK(host_terminal_seen&&flight::finished&&!flight::complete(host_log));
        // Permanent injected Flash fault: preserve complete live diagnostics and
        // truthful incomplete wire frame; frozen persist never retries after failure.
        flight_save((scenario+".live-report388").c_str(),&flight::report,sizeof(flight::report));
        flight::export_frame();flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));
        return au_done();
    }
    if(global_has("health")){
        // Fail a different channel's actual encoder publication, preserving all4 gate.
        full_encoder_valid&=uint8_t(~(1u<<3));au_tick(3);
        CHECK(full_load_fault()==FullLoadFault::inputs&&full_stopped());
        set_motion(255,7,0x26,0,au_cmd);set_motion(2,3,0x3f,0,au_cmd);
        CHECK(full_load_fault()==FullLoadFault::inputs&&full_stopped());
        phase_persist_export();return au_done();
    }
    if(global_has("absent")){
        filament_channel_inserted[2]=false;
        set_motion(2,3,0x3f,0,au_cmd);
        CHECK(full_load_fault()==FullLoadFault::inputs&&full_stopped());
        phase_persist_export();return au_done();
    }
    if(global_has("foreign")){
        full_keys[1]=1.8f;au_tick(10);au_request(1,3,0x3f,au_cmd);
        CHECK(full_load_fault()==FullLoadFault::none&&ams[0].now_filament_num==255&&full_stopped());
        CHECK(full_load_durable(2)&&!forward_grant&&!host_programs);return au_done();
    }
    // Real normal pull following retained boot -> idle -> both global phases.
    au_request(2,3,0x3f,au_cmd);au_tick(3);
    CHECK(full_load_fault()==FullLoadFault::none&&full_load_durable(2));
    CHECK(ams[0].now_filament_num==2&&ams[0].filament[2].motion==_filament_motion::pull_back);
    CHECK(lite_pull_state[2]==LitePullState::active&&!forward_grant&&!waiting_send);
    CHECK(!authority_observation::count&&(flight::report.flags&128u)&&!host_programs);
    global_no_effect(0x26);global_no_effect(0x66);
    au_request(255,1,0,au_cmd);CHECK(lite_recovery_fault()&&full_stopped());
    const auto selected=ams[0].now_filament_num;
    set_motion(255,7,0x26,0,au_cmd);set_motion(255,7,0x66,0,au_cmd);set_motion(2,3,0x3f,0,au_cmd);
    CHECK(lite_recovery_fault()&&full_stopped()&&ams[0].now_filament_num==selected&&full_load_durable(2));
    // Nonterminal interrupted pull persists under the existing native-cancel cut.
    au_tick(3);CHECK(flight::complete(host_log));flight::export_frame();
    flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));
    return au_done();
#else
    CHECK(false);return 1;
#endif
}
#endif
#if BMCU_LITE_V11_FEED_LATCH && BMCU_LITE_AUTHORITY_OBSERVATION && BMCU_USB_FLIGHT_RECORDER
static int global_serializers(){
    full_heartbeat();host_time_hook=nullptr;host_delay_hook=nullptr;
    const int64_t counts[]={0,1,-1,1000,-1000,4096,-4096,-34910,2147483645ll,2147483650ll,
        4294967290ll,4294967301ll,-4294967290ll,-4294967301ll,INT64_MAX,INT64_MIN};
    unsigned comparisons=0;
    // All possible selected-byte values, all online masks, flags and signed counts.
    for(int64_t count:counts)for(unsigned selected=0;selected<256;++selected)
    for(unsigned mask=0;mask<16;++mask)for(unsigned flag:{0u,2u,4u}){
        for(unsigned ch=0;ch<4;++ch){
            as5600_odometer_count[ch]=count;ams[0].filament[ch].meters=123.f+float(ch);
            ams[0].filament[ch].online=bool(mask&(1u<<ch));
            ams[0].filament[ch].color_R=uint8_t(17u*ch+mask);ams[0].filament[ch].name[0]=char('A'+ch);
        }
        ams[0].now_filament_num=uint8_t(selected);ams[0].filament_use_flag=uint8_t(flag);
        ams[0].pressure=uint16_t(0x1234u+mask);const auto before=ams[0];
        const auto packet=full_motion(2,7,0x66,au_cmd);const auto programs=host_programs,erases=host_erases;
        std::memset(bus_port_to_host.tx_build_buf(),mask&2u?0xa5:0x5a,1280);
        package_num=uint8_t(mask);bus_port_to_host.send_data_len=0;
        if(au_cmd==3)exact5_get_package_motion((bambubus_printer_motion_package_struct*)packet.data(),packet.size());
        else exact5_get_package_stu_motion((bambubus_printer_stu_motion_package_struct*)packet.data(),packet.size());
        const unsigned n=bus_port_to_host.send_data_len;CHECK(n==(au_cmd==3?44u:60u));
        const Packet expected(bus_port_to_host.tx_build_buf(),bus_port_to_host.tx_build_buf()+n);
        const Packet expected_buffer(bus_port_to_host.tx_build_buf(),bus_port_to_host.tx_build_buf()+1280);const auto after=ams[0];
        std::memset(bus_port_to_host.tx_build_buf(),mask&2u?0xa5:0x5a,1280);
        ams[0]=before;package_num=uint8_t(mask);bus_port_to_host.send_data_len=0;
        if(au_cmd==3)get_package_motion((bambubus_printer_motion_package_struct*)packet.data(),packet.size());
        else get_package_stu_motion((bambubus_printer_stu_motion_package_struct*)packet.data(),packet.size());
        CHECK(bus_port_to_host.send_data_len==n&&!std::memcmp(expected.data(),bus_port_to_host.tx_build_buf(),n));
        CHECK(!std::memcmp(expected_buffer.data(),bus_port_to_host.tx_build_buf(),1280));
        CHECK(package_check_crc16(bus_port_to_host.tx_build_buf(),n)&&!std::memcmp(&ams[0],&after,sizeof(after)));
        CHECK(host_programs==programs&&host_erases==erases&&full_load_fault()==FullLoadFault::none);
        for(unsigned ch=0;ch<4;++ch)CHECK(as5600_odometer_count[ch]==count);
        ++comparisons;
    }
    CHECK(comparisons==196608u);bus_port_to_host.send_data_len=0;
    std::printf("EXACT2705 serializer full-frame comparisons=%u sender=%u all256selected all16online masks signedcounts no expected rewrites\n",comparisons,au_cmd);
    return au_done();
}
#endif
