#if BMCU_USB_FLIGHT_RECORDER && BMCU_LITE_V11_FEED_LATCH
// Synthetic peripheral schedules through actual CRC parser, main, controller,
// journal and DMA/TC. They do not establish physical Unload or stock99 meaning.
static bool p99all_has(const char* s){return scenario.find(s)!=std::string::npos;}
static void p99all_emit(){
    flight_save((scenario+".live-report388").c_str(),&flight::report,sizeof(flight::report));
    flight::export_frame();flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));
}
static void p99all_fault_export(){
    repair_step();CHECK(host_terminal_seen&&flight::finished&&flight::complete(host_log));
    p99all_emit();phase_persist_export(); // Preserve live diagnostics before modeled RAM loss.
}
static unsigned p99all_calls=0;
static void p99all_check(unsigned cmd,unsigned ch,const Packet& baseline,bool deliver){
    const auto suffix=std::to_string(++p99all_calls);
    if(ams[0].now_filament_num<4)ams[0].filament[ams[0].now_filament_num].meters=Motion_control_filament_meters(ams[0].now_filament_num);
    const auto before=ams[0];const auto own=owner,durable=durable_owner;const auto journal=intent;
    const auto stamp=forward_received;const bool grant=forward_grant,waiting=waiting_send;
    const auto deadline=bambubus_heartbeat_deadline;const bool armed=bambubus_heartbeat_reset_armed;
    const auto programs=host_programs,erases=host_erases,lp=log_programs;const auto count=authority_observation::count;
    const auto flags=flight::report.flags;const auto fault=full_load_fault();const bool end=flight::native_end_pending(),checkpoint=flight::native_checkpoint_pending(),finished=flight::finished;
    const auto selected=before.now_filament_num,flag=before.filament_use_flag;const auto sequence=package_num;
    std::array<uint8_t,4096> nvm{};std::memcpy(nvm.data(),host_nvm,4096);
    std::array<uint8_t,512> log{};std::memcpy(log.data(),host_log,512);
    std::array<std::array<int,4>,3> pwm{};for(unsigned t=0;t<3;++t)for(unsigned c=0;c<4;++c)pwm[t][c]=compare_values[t][c];
    std::array<unsigned,4> helpers{},loaded{},pulls{};for(unsigned c=0;c<4;++c){helpers[c]=dm_auto_state[c];loaded[c]=dm_loaded[c];pulls[c]=unsigned(lite_pull_state[c]);}
    const auto packet=full_motion(ch,7,0x99,cmd);CHECK(package_check_crc16(const_cast<uint8_t*>(packet.data()),packet.size()));
    flight_save((scenario+".request-"+suffix+".bin").c_str(),packet.data(),packet.size());
    const auto size=cmd==3?44u:60u;
    if(deliver){
        CHECK(bus_port_to_host.idle&&!bus_port_to_host.send_data_len);append(packet);bus_rx_poll();bambubus_run();
        CHECK(bus_port_to_host.send_data_len==size);bus_port_to_host.send_package();
        CHECK(!bus_port_to_host.idle&&(DMA1_Channel4->CFGR&DMA_CFGR1_EN)&&DMA1_Channel4->CNTR==size);
        const auto tx=repair_tx;repair_complete_tx();CHECK(repair_tx==tx+1&&bus_port_to_host.idle);
    }else full_rx(packet); // Busy/pending forced parser seam; no delivery claim.
    // Preserve the actual archived-parent fault before the desired no-fault assertion.
    if(fault==FullLoadFault::none&&full_load_fault()!=fault){p99all_fault_export();CHECK(full_load_fault()==FullLoadFault::none);}
    CHECK(full_load_fault()==fault&&owner==own&&durable_owner==durable&&intent==journal);
    CHECK(!std::memcmp(&before,&ams[0],sizeof(before)));
    CHECK(forward_received==stamp&&forward_grant==grant&&waiting_send==waiting&&authority_observation::count==count);
    CHECK(bambubus_heartbeat_deadline==deadline&&bambubus_heartbeat_reset_armed==armed);
    CHECK(host_programs==programs&&host_erases==erases&&log_programs==lp&&!std::memcmp(nvm.data(),host_nvm,4096)&&!std::memcmp(log.data(),host_log,512));
    CHECK(flight::report.flags==flags&&flight::native_end_pending()==end&&flight::native_checkpoint_pending()==checkpoint&&flight::finished==finished);
    for(unsigned t=0;t<3;++t)for(unsigned c=0;c<4;++c)CHECK(compare_values[t][c]==pwm[t][c]);
    for(unsigned c=0;c<4;++c)CHECK(unsigned(dm_auto_state[c])==helpers[c]&&unsigned(dm_loaded[c])==loaded[c]&&unsigned(lite_pull_state[c])==pulls[c]);
    CHECK(full_reply.size()==size&&package_check_crc16(full_reply.data(),size)&&full_reply[4]==cmd);
    // Legacy accepted slot0 provides every serialized byte. Only the existing
    // sequence flag, command04 requested-slot echo and their CRCs may differ.
    auto expected=baseline;CHECK(expected.size()==size);expected[1]=0xC0|uint8_t(sequence<<3);
    if(cmd==4)expected[offsetof(bambubus_ams_stu_motion_package_struct,filament_channel_stu)]=ch;
    package_add_crc(expected.data(),expected.size());CHECK(expected==full_reply);
    CHECK(package_num==(sequence<7?sequence+1:0));
    if(cmd==3){bambubus_ams_motion_package_struct r{};std::memcpy(&r,full_reply.data(),sizeof(r));CHECK(r.filament_channel_2==selected&&r.filament_channel==selected&&r.filament_use_flag==flag);}
    else{bambubus_ams_stu_motion_package_struct r{};std::memcpy(&r,full_reply.data(),sizeof(r));CHECK(r.filament_channel_stu==ch&&r.filament_channel==selected&&r.filament_use_flag==flag);}
    flight_save((scenario+".reply-"+suffix+".bin").c_str(),full_reply.data(),full_reply.size());au_event(deliver?"REPLY":"PARSER",cmd,ch);
}
static void p99all_family(bool deliver=true){
    p9903_check(deliver);const auto baseline03=full_reply;
    p99_check(deliver);const auto baseline04=full_reply;
    for(unsigned cmd:{3u,4u})for(unsigned ch=0;ch<4;++ch)p99all_check(cmd,ch,cmd==3?baseline03:baseline04,deliver);
    // Adjacent sender swaps and repeated full polling families, including the
    // selected slot and all three other slots, carry no authority or preemption.
    for(unsigned ch=0;ch<4;++ch){p99all_check(3,ch,baseline03,deliver);p99all_check(4,ch,baseline04,deliver);}
}
static int p99all_workflow(){
    au_start();au_request(2,7,0,au_cmd);CHECK(full_load_durable(2)&&ams[0].now_filament_num==2&&ams[0].filament[2].motion==_filament_motion::on_use&&ams[0].filament_use_flag==4);
    for(unsigned cmd:{3u,4u}){
        const auto stamp=forward_received;const auto grants=authority_observation::count;
        au_request(255,7,0x26,cmd);au_request(255,7,0x66,cmd);
        au_request(2,7,0x66,cmd);au_request(2,7,0x26,cmd);
        CHECK(full_load_durable(2)&&ams[0].now_filament_num==2&&ams[0].filament[2].motion==_filament_motion::on_use&&ams[0].filament_use_flag==4&&forward_received==stamp&&authority_observation::count==grants);
    }
    p99all_family();
    uint8_t h[80];phase_extension(h);CHECK(h[60]==2&&h[62]==4&&h[63]==3&&h[64]==7&&h[65]==0x99&&h[66]==3&&h[67]==3&&h[68]==7&&h[69]==0x99);
    for(unsigned n=0;n<3;++n){p99all_family();} // Repetition and both04→03/03→04 orders.
    au_request(2,3,0x3f,au_cmd);au_tick(3);CHECK(lite_pull_state[2]==LitePullState::active&&!forward_grant);
    full_keys[2]=.2f;for(unsigned n=0;n<30&&filament_now_position[2]!=filament_redetect;++n)au_tick();
    CHECK(filament_now_position[2]==filament_redetect&&full_load_owner()==2);au_saved(2);
    au_tick(100);full_keys[2]=1.8f;bool stored=false;
    for(unsigned n=0;n<80;++n){au_tick();uint8_t ch=0;if(intent==JournalIntent::release&&Flash_background_idle()&&Flash_AMS_state_read(&ch)&&ch==255){stored=true;break;}}
    CHECK(stored&&full_load_owner()==2&&host_programs==8&&lite_pull_state[2]==LitePullState::completed);
    const auto packet=full_motion(255,1,0,3);append(packet);cont_step();
    CHECK(!host_terminal_seen&&full_load_empty()&&ams[0].now_filament_num==255&&full_stopped());au_saved(255);
    CHECK(flight::native_end_pending()&&flight::native_checkpoint_pending()&&!flight::finished&&!flight::complete(host_log));
    CHECK(!lite_motion_ready()&&!v11_local_eligible()&&!v11_local_drive_allowed(1));
    const auto lp=log_programs;p99all_family(false);CHECK(flight::native_end_pending()&&log_programs==lp);
    cont_step();CHECK(flight::native_end_pending()&&log_programs==lp&&full_stopped());
    repair_complete_tx();cont_step();CHECK(!host_terminal_seen&&flight::finished&&flight::complete(host_log)&&!flight::native_end_pending());
    const auto frozen=flight::report;const auto saved=std::vector<uint8_t>(host_log,host_log+512);const auto writes=log_programs;
    p99all_family();CHECK(!std::memcmp(&frozen,&flight::report,sizeof(frozen))&&!std::memcmp(saved.data(),host_log,512)&&log_programs==writes);
    for(unsigned cmd:{3u,4u}){const auto tx=repair_tx;au_request(255,1,0,cmd);cont_reply_empty(cmd);CHECK(repair_tx==tx+1&&bus_port_to_host.idle&&!host_terminal_seen);}
    CHECK(full_load_empty()&&full_stopped()&&host_programs==8&&log_programs==writes);
    return cont_save();
}
static int phase99_all_slots_verify(){
    au_ch=2;au_cmd=p99all_has("sender04")?4u:3u;au_trace=std::fopen((std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv").c_str(),"wx");CHECK(au_trace);
    host_clock=((host_clock+999)/1000)*1000;host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto& p:full_pulls)p=float(2048)*(3.3f/4095.f);
    repair_request(short_packet(0x20));au_tick(20);
    if(p99all_has("owner")){
        au_ch=p99all_has("owner0")?0:p99all_has("owner1")?1:p99all_has("owner3")?3:2;
        au_start();p99all_family();CHECK(full_load_durable(au_ch)&&ams[0].now_filament_num==au_ch&&ams[0].filament[au_ch].motion==_filament_motion::send_out);
        au_request(au_ch,7,0,au_cmd);p99all_family();CHECK(full_load_durable(au_ch)&&ams[0].now_filament_num==au_ch&&ams[0].filament[au_ch].motion==_filament_motion::on_use);
        p99all_emit();return au_done();
    }
    if(p99all_has("negative-sender")){
        au_start();au_request(2,7,0,3);const auto sender=p99all_has("sender00")?0u:p99all_has("senderff")?255u:2u;
        // Receiver dispatch admits only03/04. Exercise the classifier directly
        // for a synthetic/deferred/unknown sender; never fabricate wire routing.
        CHECK(lite_set_motion(1,7,0x99,sender));
        CHECK(full_load_fault()==FullLoadFault::unsupported&&full_stopped()&&!forward_grant&&full_load_durable(2));
        p99all_fault_export();return au_done();
    }
    if(p99all_has("workflow"))return p99all_workflow();
    if(scenario=="full-phase99-negative-cmd3"){au_start();au_request(2,7,0,3);p99all_family();CHECK(full_load_fault()==FullLoadFault::none&&full_load_durable(2)&&ams[0].now_filament_num==2&&ams[0].filament_use_flag==4);p99all_emit();return au_done();}
    if(p99all_has("negative-cancel")){
        au_start();const auto own=full_load_owner();au_request(255,0,0,au_cmd);
        CHECK(full_load_fault()==FullLoadFault::unsupported&&full_stopped()&&!forward_grant&&full_load_durable(own)&&host_terminal_seen&&flight::finished&&flight::complete(host_log));
        const auto frozen=flight::report;uint8_t h[80];phase_extension(h);CHECK(h[6]==6&&h[62]==au_cmd&&h[63]==255&&h[64]==0&&h[65]==0);
        // Forced parser seam after terminal park: status can neither revive nor relabel first fault.
        p99all_family();CHECK(full_load_fault()==FullLoadFault::unsupported&&full_stopped()&&!forward_grant&&full_load_owner()==own&&full_load_durable(own)&&!std::memcmp(&frozen,&flight::report,sizeof(frozen)));
        p99all_fault_export();return au_done();
    }
    if(p99all_has("negative")){
        au_start();au_request(2,7,0,3);au_request(255,7,0x26,3);
        unsigned cmd=au_cmd,ch=p99all_has("ch254")?254:p99all_has("ch255")?255:p99all_has("ch17")?17:p99all_has("ch4")?4:p99all_has("ch3")?3:p99all_has("ch2")?2:p99all_has("ch1")?1:0;
        unsigned state=p99all_has("state6")?6:p99all_has("state8")?8:7,m=p99all_has("motion98")?0x98:p99all_has("motion9a")?0x9a:0x99;
        au_request(ch,state,m,cmd);CHECK(full_load_fault()==FullLoadFault::unsupported&&full_stopped()&&!forward_grant&&full_load_durable(2));
        uint8_t h[80];phase_extension(h);CHECK(h[6]==6&&h[62]==cmd&&h[63]==ch&&h[64]==state&&h[65]==m);
        p99all_fault_export();return au_done();
    }
    if(p99all_has("retained")){CHECK(full_load_durable(2)&&ams[0].now_filament_num==255&&full_stopped());p99all_family();p99all_emit();return au_done();}
    if(p99all_has("empty")){CHECK(full_load_empty()&&v11_local_eligible());p99all_family();CHECK(full_load_empty()&&full_stopped());p99all_emit();return au_done();}
    if(p99all_has("local")){full_keys[1]=1.55f;au_tick(180);CHECK(dm_auto_state[1]==DM_AUTO_S1_PUSH&&v11_pwm(1));p99all_family();CHECK(v11_pwm(1)&&full_load_empty());p99all_emit();return au_done();}
    if(p99all_has("reserve")){full_keys[2]=1.8f;au_tick(10);au_bf();CHECK(waiting_send&&intent==JournalIntent::reserve&&!full_load_durable(2)&&full_stopped());p99all_family();CHECK(waiting_send&&full_stopped());p99all_emit();return au_done();}
    if(p99all_has("metadata")){metadata_recovery=true;p99all_family();full_rx(full_motion(2,3,0xbf,4));CHECK(full_load_empty()&&!forward_grant&&ams[0].now_filament_num==255&&full_stopped());p99all_emit();return au_done();}
    au_start();
    if(!p99all_has("forward")&&!p99all_has("lease"))au_request(2,7,0,3);
    if(p99all_has("stopped-loaded")){au_request(255,1,0,au_cmd);CHECK(full_stopped()&&full_load_durable(2)&&ams[0].now_filament_num==255&&!v11_local_eligible());p99all_family();p99all_emit();return au_done();}
    if(p99all_has("pull")||p99all_has("recovery")){au_request(2,3,0x3f,3);au_tick(3);CHECK(lite_pull_state[2]==LitePullState::active);}
    if(p99all_has("recovery")){au_request(255,1,0,3);CHECK(lite_recovery_fault()&&full_stopped()&&full_load_durable(2));}
    if(p99all_has("lease")){bambubus_heartbeat_deadline=time_ticks32()-1;repair_step();CHECK(!bambubus_heartbeat_reset_armed&&full_stopped()&&full_load_fault()==FullLoadFault::none);}
    if(p99all_has("terminal"))full_load_fail(FullLoadFault::controller);
    if(p99all_has("storage"))g_flash_fault=true;
    if(p99all_has("input"))full_encoder_valid=7;
    p99all_family();
    if(p99all_has("forward")){const auto stamp=forward_received;bambubus_heartbeat_deadline=time_ticks32()+2000;p99all_family();delay(3);repair_step();CHECK(!bambubus_heartbeat_reset_armed&&!forward_grant&&full_stopped()&&forward_received==stamp);p99all_family();}
    if(p99all_has("storage")){delay(1);repair_step();CHECK(full_load_fault()==FullLoadFault::storage&&full_stopped()&&host_terminal_seen);return cont_save();}
    if(p99all_has("input")){for(unsigned n=0;n<kAS5600_FAIL_TRIP&&!host_terminal_seen;++n){delay(1);repair_step();}CHECK(full_load_fault()==FullLoadFault::inputs&&full_stopped()&&host_terminal_seen);p99all_fault_export();return au_done();}
    if(p99all_has("terminal")){repair_step();CHECK(host_terminal_seen&&flight::finished&&flight::complete(host_log));const auto frozen=flight::report;p99all_family();CHECK(!std::memcmp(&frozen,&flight::report,sizeof(frozen)));p99all_fault_export();return au_done();}
    p99all_emit();return au_done();
}
#endif
