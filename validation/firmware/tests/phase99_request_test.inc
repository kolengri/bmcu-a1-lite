#if BMCU_LITE_FULL_LOAD && BMCU_LITE_PRECUT_PHASE_CAPTURE && BMCU_USB_CONTINUOUS_JOURNAL
#include "parent3103_serializers.h"
// Compatibility-hypothesis contract. Synthetic odometry is not printer evidence.
static unsigned request99_checks=0;
static Packet request99_expected(unsigned ch,unsigned cmd){
    CHECK(bus_port_to_host.idle&&!bus_port_to_host.send_data_len);
    const auto saved=ams[0];const auto seq=package_num,seen=lite_seen;
    const auto packet=full_motion(ch,7,0x99,cmd);
    if(cmd==3u)parent3103_get_package_motion((bambubus_printer_motion_package_struct*)packet.data(),packet.size());
    else parent3103_get_package_stu_motion((bambubus_printer_stu_motion_package_struct*)packet.data(),packet.size());
    Packet expected(bus_port_to_host.tx_build_buf(),bus_port_to_host.tx_build_buf()+bus_port_to_host.send_data_len);
    bus_port_to_host.send_data_len=0;package_num=seq;ams[0]=saved;lite_seen=seen;
    const unsigned off=cmd==3u?7u:19u;
    const float meter=ch<4u?1.f-Motion_control_filament_meters(ch):0.f;
    expected[off+1u]=uint8_t(ch);std::memcpy(expected.data()+off+2u,&meter,4u);
    if(cmd==3u)expected[offsetof(bambubus_ams_motion_package_struct,filament_channel_2)]=uint8_t(ch);
    package_add_crc(expected.data(),expected.size());return expected;
}
static void request99_check(unsigned ch,unsigned cmd,bool forced=false){
    journal_drain(5000);repair_step(); // Settle elapsed-time controller work before the query-only snapshot.
    const auto expected=request99_expected(ch,cmd);const auto before=ams[0];
    const auto own=owner,dur=durable_owner,seq=package_num;const auto journal=intent;
    const auto stamp=forward_received;const auto deadline=bambubus_heartbeat_deadline;
    const bool grant=forward_grant,waiting=waiting_send,armed=bambubus_heartbeat_reset_armed;
    const auto np=host_programs,ne=host_erases,lp=log_programs;const auto grants=authority_observation::count;
    const auto fault=full_load_fault();const auto frozen=flight::report;const bool finished=flight::finished;
    int pwm[3][4];std::memcpy(pwm,compare_values,sizeof(pwm));
    std::array<uint8_t,4096> nvm{};std::memcpy(nvm.data(),host_nvm,nvm.size());
    const auto packet=full_motion(ch,7,0x99,cmd);CHECK(package_check_crc16(const_cast<uint8_t*>(packet.data()),packet.size()));
    if(forced){ // Validated handler seam after terminal RX shutdown; no DMA-arrival claim.
        CHECK(get_packge_type(const_cast<uint8_t*>(packet.data()),packet.size())==(cmd==3u?bambubus_package_type::filament_motion_short:bambubus_package_type::filament_motion_long));
        flight::pull_wire(cmd,ch,7u,0x99u);
        if(cmd==3u)get_package_motion((bambubus_printer_motion_package_struct*)packet.data(),packet.size());
        else get_package_stu_motion((bambubus_printer_stu_motion_package_struct*)packet.data(),packet.size());
        full_reply.assign(bus_port_to_host.tx_build_buf(),bus_port_to_host.tx_build_buf()+bus_port_to_host.send_data_len);bus_port_to_host.send_data_len=0;
    }else repair_request(packet);
    CHECK(full_reply.size()==expected.size()&&package_check_crc16(full_reply.data(),full_reply.size()));
    const auto suffix=std::to_string(++request99_checks);
    flight_save((scenario+".request-"+suffix+".bin").c_str(),packet.data(),packet.size());
    flight_save((scenario+".reply-"+suffix+".bin").c_str(),full_reply.data(),full_reply.size());
    flight_save((scenario+".expected-"+suffix+".bin").c_str(),expected.data(),expected.size());
    const unsigned off=cmd==3u?7u:19u;
    std::printf("REQUEST99 hypothesis owner=%u selected=%u request=%u sender=%u reply=%u use=%u meter=%02x%02x%02x%02x\n",unsigned(own),unsigned(before.now_filament_num),ch,cmd,unsigned(full_reply[off+1]),unsigned(full_reply[off]),full_reply[off+2],full_reply[off+3],full_reply[off+4],full_reply[off+5]);std::fflush(stdout);
    CHECK(full_reply==expected); // Full wire/CRC hypothesis assertion, including the red parent case.
    CHECK(package_num==(seq<7u?seq+1u:0u));
    CHECK(!std::memcmp(&before,&ams[0],sizeof(before)));
    CHECK(owner==own&&durable_owner==dur&&intent==journal&&full_load_fault()==fault);
    CHECK(forward_received==stamp&&forward_grant==grant&&waiting_send==waiting&&authority_observation::count==grants);
    CHECK(bambubus_heartbeat_deadline==deadline&&bambubus_heartbeat_reset_armed==armed);
    CHECK(host_programs==np&&host_erases==ne&&log_programs==lp&&!std::memcmp(nvm.data(),host_nvm,nvm.size()));
    CHECK(!std::memcmp(pwm,compare_values,sizeof(pwm)));
    if(finished)CHECK(!std::memcmp(&frozen,&flight::report,sizeof(frozen)));
}
static void request99_busy(unsigned first){
    const auto before=ams[0];const auto seq=package_num;
    const auto np=host_programs,ne=host_erases,lp=log_programs;const auto stamp=forward_received;
    const auto deadline=bambubus_heartbeat_deadline;const auto grants=authority_observation::count;
    int pwm[3][4];std::memcpy(pwm,compare_values,sizeof(pwm));
    const unsigned cmds[3]={first,7u-first,first},slots[3]={0,1,3};
    Packet expected[3],requests[3];
    for(unsigned n=0;n<3u;++n){package_num=(seq+n)&7u;expected[n]=request99_expected(slots[n],cmds[n]);requests[n]=full_motion(slots[n],7,0x99,cmds[n]);}
    package_num=seq;
    auto inflight=[&](unsigned n){
        CHECK(!bus_port_to_host.idle&&DMA1_Channel4->CNTR==expected[n].size());
        const auto* bytes=reinterpret_cast<const uint8_t*>(DMA1_Channel4->MADDR);
        CHECK(!std::memcmp(bytes,expected[n].data(),expected[n].size()));
    };
    append(requests[0]);full_load_step();inflight(0);
    append(requests[1]);full_load_step();inflight(0);
    CHECK(bus_port_to_host.send_data_len==expected[1].size());
    CHECK(!std::memcmp(bus_port_to_host.tx_build_buf(),expected[1].data(),expected[1].size()));
    append(requests[2]);full_load_step();inflight(0);
    CHECK(bus_port_to_host.send_data_len==expected[1].size());
    CHECK(!std::memcmp(bus_port_to_host.tx_build_buf(),expected[1].data(),expected[1].size()));
    repair_complete_tx();CHECK(full_reply==expected[0]);
    full_load_step();inflight(1);repair_complete_tx();CHECK(full_reply==expected[1]);
    full_load_step();inflight(2);repair_complete_tx();CHECK(full_reply==expected[2]);
    CHECK(!std::memcmp(&before,&ams[0],sizeof(before))&&!std::memcmp(pwm,compare_values,sizeof(pwm)));
    CHECK(host_programs==np&&host_erases==ne&&log_programs==lp&&full_load_owner()==au_ch&&full_load_durable(au_ch));
    CHECK(forward_received==stamp&&bambubus_heartbeat_deadline==deadline&&authority_observation::count==grants);
    CHECK(flight::capturing&&!flight::finished&&!host_terminal_seen);
}
static int request99_export(){
    journal_drain(5000);flight::jexport_frame();flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));return 0;
}
static int request99_verify(){
    unsigned own=2,requested=0,cmd=oi_has("sender04")?4u:3u;
    std::sscanf(scenario.c_str(),"full-request99-owner%u-slot%u-sender%u",&own,&requested,&cmd);
    CHECK(own<4u&&requested<4u&&(cmd==3u||cmd==4u));au_ch=own;au_cmd=cmd;
    au_trace=std::fopen((std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv").c_str(),"wx");CHECK(au_trace);
    host_clock=((host_clock+999u)/1000u)*1000u;
    host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto& p:full_pulls)p=float(2048)*(3.3f/4095.f);
    repair_request(short_packet(0x20));au_tick(20);
    if(!oi_has("empty")){au_start();au_request(own,7,0,cmd);}
    freeze_encoder=true;
    for(unsigned ch=0;ch<4u;++ch){as5600_odometer_count[ch]=1000u+4096u*ch;ams[0].filament[ch].meters=500.f+ch;}
    if(oi_has("zero"))as5600_odometer_count[requested]=0u;
    if(oi_has("lease")){
        bambubus_heartbeat_deadline=time_ticks32()-1u;repair_step();
        CHECK(!bambubus_heartbeat_reset_armed&&ams[0].now_filament_num==255u&&full_stopped()&&full_load_durable(own));
    }
    if(oi_has("busy")){request99_busy(cmd);return request99_export();}
    if(oi_has("fault")){
        full_load_fail(FullLoadFault::controller);CHECK(full_stopped());
        request99_check(requested,cmd);repair_step();
        CHECK(!host_terminal_seen&&!flight::finished&&full_load_fault()==FullLoadFault::controller);
        request99_check((requested+1u)%4u,7u-cmd);
        full_rx(short_packet(0x20));full_rx(full_motion(requested,3,0xbf,cmd));full_rx(full_motion(own,3,0x3f,3));Motion_control_run(0);
        CHECK(full_load_fault()==FullLoadFault::controller&&full_stopped()&&full_load_owner()==own);
        return request99_export();
    }
    request99_check(oi_has("global")?255u:requested,cmd);
    if(oi_has("ownership")){
        const auto stamp=forward_received;const auto grants=authority_observation::count;
        au_request(requested,3,0x3f,cmd);
        CHECK(full_load_fault()==FullLoadFault::none&&full_load_owner()==own&&ams[0].now_filament_num==own);
        CHECK(lite_pull_state[requested]==LitePullState::ready&&lite_pull_state[own]==LitePullState::ready);
        CHECK(forward_received==stamp&&authority_observation::count==grants);
        journal_drain(5000);CHECK(retained_event(8,uint8_t(requested),uint32_t(flight::PullRefusal::loaded_mismatch)|(3u<<16)|(0x3fu<<24)));
        au_request(own,3,0x3f,3);au_tick(3);
        CHECK(lite_pull_state[own]==LitePullState::active&&ams[0].now_filament_num==own&&full_load_durable(own));
    }
    return request99_export();
}
#endif
