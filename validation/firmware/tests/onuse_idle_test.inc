#if BMCU_USB_FLIGHT_RECORDER && BMCU_LITE_CONTINUOUS_NATIVE && BMCU_LITE_V11_FEED_LATCH
#include "accepted04_serializers.h"
static bool oi_has(const char* s){return scenario.find(s)!=std::string::npos;}
static constexpr uint32_t oi_seen=1u<<14,oi_mask=15u<<16;
static void oi_save(){
    flight_save((scenario+".live-report388").c_str(),&flight::report,sizeof(flight::report));
    flight::export_frame();flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));
}
static int oi_export(){
    oi_save();std::array<uint8_t,512> log{};std::memcpy(log.data(),host_log,512);
    std::array<uint8_t,4096> nvm{};std::memcpy(nvm.data(),host_nvm,4096);const auto lp=log_programs,np=host_programs;
    flight_reset();bus_started=false;g_adc_dma_inited=false;host_dma1.CFGR=0;
    host_time_hook=[](){++host_clock;if(log_tx.size()>=536)throw LoopComplete{};};
    try{firmware_main();CHECK(false);}catch(const LoopComplete&){}
    CHECK(!bus_started&&!g_adc_dma_inited&&!host_dma1.CFGR&&!flight::capturing);
    CHECK(log_programs==lp&&host_programs==np&&!std::memcmp(log.data(),host_log,512)&&!std::memcmp(nvm.data(),host_nvm,4096));
    CHECK(log_tx.size()==536&&!std::memcmp(log_tx.data()+20,log.data(),512));
    for(auto& row:compare_values)for(auto v:row)CHECK(!v);
    return au_done();
}
// Oracle uses the exact accepted04 serializer on the already-classified state.
// For the declared new global response only, apply Tiny's independent FF/zero fields.
static unsigned oi_packets=0;
static bool oi_save_packets=true;
static void oi_packet(unsigned cmd,unsigned ch,unsigned state,unsigned motion,bool preserved){
    CHECK(bus_port_to_host.idle&&!bus_port_to_host.send_data_len);
    const auto seq=package_num;const auto before=ams[0];const auto own=owner,dur=durable_owner;const auto journal=intent;
    const bool grant=forward_grant,waiting=waiting_send,finished=flight::finished,checkpoint=flight::native_checkpoint_pending();
    const auto count=authority_observation::count;const auto stamp=forward_received;const auto flags=flight::report.flags;
    const auto deadline=bambubus_heartbeat_deadline;const bool armed=bambubus_heartbeat_reset_armed;
    std::array<std::array<int,4>,3> pwm{};for(unsigned t=0;t<3;++t)for(unsigned c=0;c<4;++c)pwm[t][c]=compare_values[t][c];
    std::array<unsigned,4> helpers{},loaded{},pulls{};for(unsigned c=0;c<4;++c){helpers[c]=dm_auto_state[c];loaded[c]=dm_loaded[c];pulls[c]=unsigned(lite_pull_state[c]);}
    const auto np=host_programs,lp=log_programs,ne=host_erases;std::array<uint8_t,4096> nvm{};std::memcpy(nvm.data(),host_nvm,4096);
    std::array<uint8_t,512> log{};std::memcpy(log.data(),host_log,512);
    const auto p=full_motion(ch,state,motion,cmd);++oi_packets;if(oi_save_packets)flight_save((scenario+".request-"+std::to_string(oi_packets)+".bin").c_str(),p.data(),p.size());
    const auto diagnostic_reads_before=host_onuse_meter_reads;
    host_meter_reads=0;host_meter_trace=true;append(p);bus_rx_poll();bambubus_run();host_meter_trace=false;
    const auto selected=ams[0].now_filament_num;CHECK(bus_port_to_host.send_data_len==(cmd==3?44u:60u));
    const auto reads=host_meter_reads-(host_onuse_meter_reads-diagnostic_reads_before);Packet actual(bus_port_to_host.tx_build_buf(),bus_port_to_host.tx_build_buf()+bus_port_to_host.send_data_len);
    bus_port_to_host.send_package();CHECK(!bus_port_to_host.idle&&(DMA1_Channel4->CFGR&DMA_CFGR1_EN));const auto tx=repair_tx;repair_complete_tx();CHECK(repair_tx==tx+1&&bus_port_to_host.idle&&full_reply==actual);
    CHECK(package_num==(seq<7?seq+1:0));const auto after=ams[0];const auto staged=flight::staged_kind;
    package_num=seq;host_meter_trace=true;host_meter_reads=0;
    if(cmd==3)accepted04_get_package_motion((bambubus_printer_motion_package_struct*)p.data(),p.size());
    else accepted04_get_package_stu_motion((bambubus_printer_stu_motion_package_struct*)p.data(),p.size());
    host_meter_trace=false;Packet expected(bus_port_to_host.tx_build_buf(),bus_port_to_host.tx_build_buf()+bus_port_to_host.send_data_len);
    CHECK(package_num==(seq<7?seq+1:0));bus_port_to_host.send_data_len=0;flight::staged_kind=staged;
    if(preserved){
        // Both internal slots and every controller/ownership/storage boundary persist.
        CHECK(owner==own&&durable_owner==dur&&intent==journal&&forward_grant==grant&&waiting_send==waiting);
        CHECK(authority_observation::count==count&&forward_received==stamp&&selected==before.now_filament_num);
        CHECK(!std::memcmp(&before,&after,sizeof(before))&&reads==0);CHECK(after.filament_use_flag==4&&after.filament[selected].motion==_filament_motion::on_use);
        CHECK(host_programs==np&&log_programs==lp&&host_erases==ne&&!std::memcmp(nvm.data(),host_nvm,4096)&&!std::memcmp(log.data(),host_log,512));
        CHECK(flight::finished==finished&&flight::native_checkpoint_pending()==checkpoint&&flight::report.flags==(flags|oi_seen));
        CHECK(bambubus_heartbeat_deadline==deadline&&bambubus_heartbeat_reset_armed==armed);
        for(unsigned t=0;t<3;++t)for(unsigned c=0;c<4;++c)CHECK(compare_values[t][c]==pwm[t][c]);
        for(unsigned c=0;c<4;++c)CHECK(unsigned(dm_auto_state[c])==helpers[c]&&unsigned(dm_loaded[c])==loaded[c]&&unsigned(lite_pull_state[c])==pulls[c]);
        CHECK(!flight::native_checkpoint_pending()&&!flight::finished&&!(flight::report.flags&64u));
        const unsigned off=cmd==3?7:19;expected[off]=4;expected[off+1]=255;std::memset(expected.data()+off+2,0,4);expected[off+6]=expected[off+7]=255;
        if(cmd==3)expected[offsetof(bambubus_ams_motion_package_struct,filament_channel_2)]=255;
        package_add_crc(expected.data(),expected.size());
        CHECK(actual[off]==4&&actual[off+1]==255&&!std::memcmp(actual.data()+off+2,"\0\0\0\0",4)&&actual[off+6]==255&&actual[off+7]==255);
        ams[0]=after; // The accepted oracle's selected-meter read is not part of the new global reply.
    }else{
        CHECK(reads==(selected<4?1u:0u));CHECK(!std::memcmp(&after,&ams[0],sizeof(after)));
    }
    if(actual!=expected){flight_save("oracle-failure-actual.bin",actual.data(),actual.size());flight_save("oracle-failure-expected.bin",expected.data(),expected.size());}
    CHECK(actual==expected&&package_check_crc16(actual.data(),actual.size()));
    if(oi_save_packets)flight_save((scenario+".reply-"+std::to_string(oi_packets)+".bin").c_str(),actual.data(),actual.size());
}
static void oi_start(){au_start();au_request(au_ch,7,0,au_cmd);CHECK(full_load_durable(au_ch)&&ams[0].now_filament_num==au_ch&&ams[0].filament_use_flag==4);}
static void oi_poll(){for(unsigned n=0;n<4;++n)for(unsigned cmd:{3u,4u})oi_packet(cmd,255,1,0,true);}
static int onuse_workflow(){
    oi_start();au_request(255,7,0,au_cmd);oi_poll();CHECK(!(flight::report.flags&oi_mask));
    // Logical on_use retains the unchanged physical controller/PWM eligibility.
    full_pulls[au_ch]=1.8f;au_tick(12);std::printf("ONUSE off-band pct=%.3f fault=%u motion=%u stopped=%u\n",MC_PULL_pct_f[au_ch],unsigned(full_load_fault()),unsigned(MOTOR_CONTROL[au_ch].motion),full_stopped());CHECK(!full_stopped()&&full_load_durable(au_ch)&&!forward_grant&&full_load_fault()==FullLoadFault::none);
    oi_poll();full_pulls[au_ch]=float(2048)*(3.3f/4095.f);au_tick(8);
    au_request(au_ch,3,0x3f,au_cmd);au_tick(3);CHECK(lite_pull_state[au_ch]==LitePullState::active&&!forward_grant);
    CHECK((flight::report.flags&oi_mask)==(1u<<(16u+au_ch)));
    full_keys[au_ch]=.2f;for(unsigned n=0;n<30&&filament_now_position[au_ch]!=filament_redetect;++n)au_tick();
    CHECK(filament_now_position[au_ch]==filament_redetect&&full_load_owner()==au_ch);au_saved(au_ch);
    au_tick(100);full_keys[au_ch]=1.8f;bool stored=false;
    for(unsigned n=0;n<80;++n){au_tick();uint8_t ch=0;if(intent==JournalIntent::release&&Flash_background_idle()&&Flash_AMS_state_read(&ch)&&ch==255){stored=true;break;}}
    CHECK(stored&&lite_pull_state[au_ch]==LitePullState::completed&&host_programs==8);
    const auto p=full_motion(255,1,0,au_cmd);append(p);cont_step();CHECK(full_load_empty()&&full_stopped()&&flight::native_checkpoint_pending()&&!flight::finished);
    const auto lp=log_programs;cont_step();CHECK(log_programs==lp&&!flight::finished);repair_complete_tx();cont_step();
    CHECK(flight::finished&&flight::complete(host_log)&&!host_terminal_seen&&(flight::report.flags&oi_seen)&&(flight::report.flags&oi_mask)==(1u<<(16u+au_ch)));
    const auto frozen=flight::report;const auto writes=log_programs;
    for(unsigned cmd:{3u,4u})oi_packet(cmd,255,1,0,false);
    CHECK(!std::memcmp(&frozen,&flight::report,sizeof(frozen))&&log_programs==writes);return oi_export();
}
static int onuse_verify(){
    au_ch=oi_has("owner0")?0:oi_has("owner1")?1:oi_has("owner3")?3:2;au_cmd=oi_has("sender04")?4:3;
    au_trace=std::fopen((std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv").c_str(),"wx");CHECK(au_trace);
    host_clock=((host_clock+999)/1000)*1000;host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto& p:full_pulls)p=float(2048)*(3.3f/4095.f);repair_request(short_packet(0x20));au_tick(20);
    if(oi_has("oracle-grid")){
        oi_save_packets=false;const auto saved=ams[0];
        for(unsigned online=0;online<16;++online)for(unsigned slot=0;slot<5;++slot)for(unsigned flag:{0u,2u,4u,255u})for(unsigned values=0;values<3;++values){
            ams[0]=saved;ams[0].now_filament_num=slot<4?slot:255;ams[0].filament_use_flag=flag;ams[0].pressure=uint16_t(values*13000);
            for(unsigned ch=0;ch<4;++ch){ams[0].filament[ch].online=(online&(1u<<ch))!=0;ams[0].filament[ch].meters=123.f+ch;ams[0].filament[ch].compartment_temperature=float(values*15+ch);ams[0].filament[ch].compartment_humidity=values*30+ch;as5600_odometer_count[ch]=values?10000u*(values+ch):0u;}
            for(unsigned cmd:{3u,4u})oi_packet(cmd,0,7,0x99,false);
        }
        CHECK(oi_packets==1920);ams[0]=saved;std::printf("ORACLE GRID %u complete accepted04 response comparisons; all masks/selections/flags/metadata, CRC/sequence and exact meter-read/mutation boundaries\n",oi_packets);return oi_export();
    }
    if(oi_has("baseline-observed")){
        oi_start();au_request(255,7,0,3);au_request(255,1,0,au_cmd);au_tick(2);
        CHECK(full_load_fault()==FullLoadFault::none&&full_stopped()&&ams[0].now_filament_num==255&&ams[0].filament_use_flag==0&&flight::finished&&flight::complete(host_log)&&(flight::report.flags&64));
        const auto frozen=flight::report;oi_save();
        au_request(au_ch,7,0,au_cmd);CHECK(ams[0].now_filament_num==au_ch&&ams[0].filament[au_ch].motion==_filament_motion::on_use);
        au_request(au_ch,3,0x3f,au_cmd);au_tick(3);CHECK(lite_pull_state[au_ch]==LitePullState::active&&full_load_fault()==FullLoadFault::none);
        CHECK(!std::memcmp(&frozen,&flight::report,sizeof(frozen)));std::puts("BASELINE later same-owner use and pull remain admissible after early idle checkpoint");return oi_export();
    }
    if(oi_has("cold-loaded")||oi_has("empty")||oi_has("helper")||oi_has("reservation")){
        if(oi_has("helper")){full_keys[1]=1.55f;au_tick(180);CHECK(v11_pwm(1));}
        if(oi_has("reservation")){full_keys[au_ch]=1.8f;au_tick(10);au_bf();CHECK(full_load_journal_pending());}
        oi_packet(3,255,1,0,false);oi_packet(4,255,1,0,false);CHECK(!(flight::report.flags&(oi_seen|oi_mask)));return oi_export();
    }
    if(oi_has("workflow"))return onuse_workflow();
    oi_start();
    if(oi_has("refused-pulls")){
        const auto selected=ams[0].now_filament_num,own=owner;const auto np=host_programs,lp=log_programs;
        au_request((au_ch+1)%4,3,0x3f,3);CHECK(!(flight::report.flags&oi_mask)&&owner==own&&ams[0].now_filament_num==selected);
        metadata_recovery=true;full_rx(full_motion(au_ch,3,0x3f,4));CHECK(!(flight::report.flags&oi_mask)&&lite_pull_state[au_ch]==LitePullState::ready);metadata_recovery=false;
        intent=JournalIntent::reserve;full_rx(full_motion(au_ch,3,0x3f,3));CHECK(!(flight::report.flags&oi_mask)&&lite_pull_state[au_ch]==LitePullState::ready);intent=JournalIntent::none;
        CHECK(host_programs==np&&log_programs==lp&&full_load_fault()==FullLoadFault::none);
        au_request(au_ch,3,0x3f,4);au_tick(3);CHECK((flight::report.flags&oi_mask)==(1u<<(16u+au_ch))&&lite_pull_state[au_ch]==LitePullState::active);
        au_request(255,1,0,3);repair_step();CHECK(lite_recovery_fault()&&flight::complete(host_log)&&full_stopped());return oi_export();
    }
    if(oi_has("frozen-first-snapshot")){
        au_request(255,1,1,au_cmd);au_tick(2);CHECK(flight::finished&&flight::complete(host_log));const auto frozen=flight::report;
        CHECK(!(frozen.flags&(oi_seen|oi_mask)));
        au_request(au_ch,7,0,4);au_request(au_ch,3,0x3f,3);au_tick(3);CHECK(lite_pull_state[au_ch]==LitePullState::active);
        CHECK(!std::memcmp(&frozen,&flight::report,sizeof(frozen)));return oi_export();
    }
    if(oi_has("corpus")){
        // Every valid owner, both commands, ordinary/BF/use/phase99/global families.
        for(unsigned cmd:{3u,4u}){
            oi_packet(cmd,au_ch,7,0,false);
            for(unsigned ch=0;ch<4;++ch)oi_packet(cmd,ch,7,0x99,false);
            for(unsigned m:{0u,0x26u,0x66u})oi_packet(cmd,255,7,m,false);
            oi_packet(cmd,au_ch,7,0x26,false);oi_packet(cmd,au_ch,7,0x66,false);
            oi_packet(cmd,au_ch,3,0xbf,false);oi_packet(cmd,au_ch,7,0x26,false);oi_packet(cmd,255,1,0,false); // Existing send_out idle exception.
            oi_packet(cmd,au_ch,7,0,false);
        }
        return oi_export();
    }
    if(oi_has("gate-")){
        if(oi_has("wrong-owner"))durable_owner=uint8_t((au_ch+1)%4);
        if(oi_has("missing-cal"))g_valid_mask&=uint8_t(~(1u<<au_ch));
        if(oi_has("missing-presence")){full_keys[au_ch]=.2f;au_tick(10);}
        if(oi_has("metadata"))metadata_recovery=true;
        if(oi_has("journal"))intent=JournalIntent::reserve;
        if(oi_has("storage"))g_flash_fault=true;
        if(oi_has("terminal"))full_load_fail(FullLoadFault::controller);
        if(oi_has("lease")){bambubus_heartbeat_deadline=time_ticks32()-1;repair_step();CHECK(!bambubus_heartbeat_reset_armed);}
        if(oi_has("stopped"))au_request(255,1,1,au_cmd);
        if(oi_has("input")){full_encoder_valid=7;for(unsigned n=0;n<kAS5600_FAIL_TRIP&&!host_terminal_seen;++n){delay(1);repair_step();}}
        // Parser-only inhibition seam preserves the actual terminal main boundary.
        oi_packet(3,255,1,0,false);oi_packet(4,255,1,0,false);CHECK(!(flight::report.flags&(oi_seen|oi_mask)));return oi_export();
    }
    if(oi_has("negative-")){
        const unsigned cmd=oi_has("synthetic")?0:au_cmd,ch=oi_has("channel4")?4:255,state=oi_has("state0")?0:1,m=oi_has("motion1")?1:0;
        if(!cmd)lite_set_motion(ch,state,m,cmd);else oi_packet(cmd,ch,state,m,false);
        CHECK(!(flight::report.flags&(oi_seen|oi_mask)));return oi_export();
    }
    oi_poll();
    if(oi_has("cut-active"))return oi_export();
    if(oi_has("fault-input")){full_encoder_valid=7;for(unsigned n=0;n<kAS5600_FAIL_TRIP&&!host_terminal_seen;++n){delay(1);repair_step();}CHECK(full_load_fault()==FullLoadFault::inputs&&flight::report.input_detail==0x0309u);}
    else if(oi_has("fault-unsupported")){au_request(255,0,0,au_cmd);CHECK(full_load_fault()==FullLoadFault::unsupported);}
    else if(oi_has("interrupted-pull")){au_request(au_ch,3,0x3f,au_cmd);au_tick(3);CHECK(lite_pull_state[au_ch]==LitePullState::active);au_request(255,1,0,au_cmd);CHECK(lite_recovery_fault()&&full_stopped()&&lite_pull_state[au_ch]==LitePullState::interrupted);}
    else if(oi_has("lease-stop")){bambubus_heartbeat_deadline=time_ticks32()-1;repair_step();CHECK(full_stopped()&&!bambubus_heartbeat_reset_armed);const auto own=owner;oi_packet(3,255,1,0,false);CHECK(owner==own&&ams[0].now_filament_num==255);return oi_export();}
    else CHECK(false);
    repair_step();CHECK(flight::finished&&flight::complete(host_log)&&full_stopped());return oi_export();
}
#endif
