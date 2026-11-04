#if BMCU_USB_FLIGHT_RECORDER && BMCU_LITE_V11_FEED_LATCH
static bool cont_has(const char* s){return scenario.find(s)!=std::string::npos;}
static void cont_step(const Packet* p=nullptr){
    if(p)append(*p);
    try{full_load_step();}catch(const TerminalCapture&){}
}
static void cont_reply_empty(unsigned cmd){
    CHECK(full_reply.size()==(cmd==3?44u:60u)&&package_check_crc16(full_reply.data(),full_reply.size()));
    CHECK(full_reply[cmd==3?7:19]==0&&full_reply[cmd==3?8:20]==255);
    if(cmd==3){bambubus_ams_motion_package_struct r{};std::memcpy(&r,full_reply.data(),sizeof(r));CHECK(r.filament_channel_2==255);}
}
static int cont_save(){
    const auto fault=full_load_fault();CHECK(full_stopped());
    const bool complete=flight::complete(host_log);
    if(complete){flight::Report r{};std::memcpy(&r,host_log+32,sizeof(r));CHECK(r.native_fault==uint8_t(fault)&&(r.flags&32u));}
    flight_save((scenario+".live-report388").c_str(),&flight::report,sizeof(flight::report));
    flight::export_frame();flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));
    std::array<uint8_t,512> saved{};std::memcpy(saved.data(),host_log,512);
    std::array<uint8_t,4096> nvm{};std::memcpy(nvm.data(),host_nvm,4096);
    const auto programs=host_programs,lp=log_programs;
    flight_reset();bus_started=false;g_adc_dma_inited=false;host_dma1.CFGR=0;
    host_time_hook=[](){++host_clock;if(log_tx.size()>=536)throw LoopComplete{};};
    try{firmware_main();CHECK(false);}catch(const LoopComplete&){}
    CHECK(!bus_started&&!g_adc_dma_inited&&!host_dma1.CFGR&&host_programs==programs&&log_programs==lp);
    CHECK(!std::memcmp(saved.data(),host_log,512)&&!std::memcmp(nvm.data(),host_nvm,4096));
    CHECK(log_tx.size()==536&&!std::memcmp(log_tx.data()+20,saved.data(),512));
    for(auto& row:compare_values)for(auto v:row)CHECK(!v);
    std::printf("CONTINUE PASS %s fault=%u complete=%u NVM-programs=%u LOG-programs=%u actualDMA-replies=%u\n",scenario.c_str(),unsigned(fault),complete,programs,lp,repair_tx);
    return au_done();
}
static int unload_continue_verify(){
    au_ch=2;au_cmd=cont_has("sender04")?4u:3u;
    au_trace=std::fopen((std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv").c_str(),"wx");CHECK(au_trace);
    host_clock=((host_clock+999)/1000)*1000;
    host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto& p:full_pulls)p=float(2048)*(3.3f/4095.f);
    repair_request(short_packet(0x20));au_tick(20);CHECK(v11_local_eligible());au_start();
    au_request(2,3,0x3f,au_cmd);au_tick(3);CHECK(lite_pull_state[2]==LitePullState::active&&!forward_grant);
    full_keys[2]=.2f;for(unsigned n=0;n<30&&filament_now_position[2]!=filament_redetect;++n)au_tick();
    CHECK(filament_now_position[2]==filament_redetect&&full_load_owner()==2);au_saved(2);
    au_tick(100);full_keys[2]=1.8f;
    // Observe the real completion callback and release journal, stopping after
    // the Flash job has verified FF but before the next full_load_run reader.
    bool stored=false;
    for(unsigned n=0;n<80;++n){au_tick();uint8_t ch=0;
        if(intent==JournalIntent::release&&Flash_background_idle()&&Flash_AMS_state_read(&ch)&&ch==255){stored=true;break;}}
    CHECK(stored&&full_load_owner()==2&&host_programs==8&&lite_pull_state[2]==LitePullState::completed);
    const auto packet=full_motion(255,1,0,au_cmd);append(packet);
    // Actual production iteration: TX response is built before durable release
    // observation. Hold its DMA/USART TC so the common checkpoint cannot run yet.
    cont_step();
    CHECK(!host_terminal_seen); // Actual base6 fails here: it terminal-parks on success.
    CHECK(full_load_empty()&&ams[0].now_filament_num==255&&full_stopped());au_saved(255);
    CHECK(flight::native_end_pending()&&flight::native_checkpoint_pending()&&!flight::finished&&!flight::complete(host_log));
    CHECK(!lite_motion_ready()&&!v11_local_eligible()&&!v11_local_drive_allowed(1));
    CHECK(!bus_port_to_host.idle&&(DMA1_Channel4->CFGR&DMA_CFGR1_EN)&&DMA1_Channel4->CNTR);
    const auto grants=authority_observation::count;const auto origin=forward_received;const auto writes=host_programs;const auto lp=log_programs;
    const auto flags=flight::report.flags;
    // CRC-valid requests enter the real parser while the old TX keeps bus busy.
    for(unsigned cmd:{3u,4u})for(auto sm:std::array<std::array<unsigned,2>,3>{{{{3,0xbf}},{{7,0}},{{3,0x3f}}}}){
        full_rx(full_motion(1,sm[0],sm[1],cmd));
        CHECK(full_load_empty()&&!waiting_send&&!forward_grant&&intent==JournalIntent::none&&ams[0].now_filament_num==255&&full_stopped());
        CHECK(authority_observation::count==grants&&forward_received==origin&&host_programs==writes&&flight::report.flags==flags);
    }
    full_rx(full_motion(255,7,0x26,3));full_rx(full_motion(255,7,0x66,4));
    CHECK(flight::native_end_pending()&&!flight::finished&&log_programs==lp);
    if(cont_has("input"))full_encoder_valid=7;
    if(cont_has("storage"))g_flash_fault=true;
    if(cont_has("unknown")){const auto bad=full_motion(255,0,0,au_cmd);cont_step(&bad);}
    else if(cont_has("local-pwm")){Motion_control_set_PWM(1,500);cont_step();}
    else if(cont_has("input")){for(unsigned n=0;n<kAS5600_FAIL_TRIP&&!host_terminal_seen;++n){delay(1);cont_step();}}
    else if(cont_has("storage")){delay(1);cont_step();}
    if(cont_has("input")||cont_has("storage")||cont_has("unknown")||cont_has("local-pwm")){
        const auto want=cont_has("input")?FullLoadFault::inputs:cont_has("storage")?FullLoadFault::storage:cont_has("unknown")?FullLoadFault::unsupported:FullLoadFault::capability;
        CHECK(full_load_fault()==want&&host_terminal_seen&&full_stopped()&&!lite_motion_ready());
        CHECK(flight::report.native_fault==uint8_t(want));
        return cont_save();
    }
    // Still busy: another real iteration must not persist or admit motion.
    cont_step();CHECK(flight::native_end_pending()&&log_programs==lp&&full_stopped());
    if(cont_has("lease")){bambubus_heartbeat_deadline=time_ticks32()-1;cont_step();CHECK(!bambubus_heartbeat_reset_armed&&full_load_fault()==FullLoadFault::none);}
    repair_complete_tx();CHECK(bus_port_to_host.idle); // Actual TC/DMA path, not a synthetic reply.
    if(cont_has("logdeny"))host_fail_program=true;
    cont_step();
    if(cont_has("logdeny")){
        CHECK(full_load_fault()==FullLoadFault::storage&&full_stopped()&&!lite_motion_ready()&&!flight::complete(host_log));
        const auto denied=log_programs;cont_step();CHECK(host_terminal_seen&&log_programs==denied&&flight::report.native_fault==uint8_t(FullLoadFault::storage));
        return cont_save();
    }
    CHECK(!host_terminal_seen&&flight::finished&&flight::complete(host_log)&&!flight::native_end_pending()&&!flight::native_checkpoint_pending());
    CHECK(full_load_empty()&&full_stopped()&&host_programs==8);
    const auto saved=std::vector<uint8_t>(host_log,host_log+512);const auto logged=log_programs;
    if(cont_has("lease")){
        CHECK(!lite_motion_ready());set_motion(2,3,0xbf,0,au_cmd);CHECK(full_load_empty()&&!forward_grant&&host_programs==8);
        repair_request(short_packet(0x20));CHECK(lite_motion_ready());
    }
    for(unsigned cmd:{3u,4u}){
        const auto tx=repair_tx;au_request(255,1,0,cmd);cont_reply_empty(cmd);CHECK(repair_tx==tx+1&&bus_port_to_host.idle);
        CHECK(!host_terminal_seen&&log_programs==logged&&!std::memcmp(saved.data(),host_log,512));
    }
    CHECK(v11_local_eligible());
    // Original fresh-BF authority and durable reservation barrier still apply.
    const auto previous=authority_observation::count;au_bf(au_cmd);
    CHECK(waiting_send&&intent==JournalIntent::reserve&&!full_load_durable(2)&&full_stopped()&&ams[0].now_filament_num==255);
    // AO4 count is intentionally frozen after its first driven-stop fact.
    CHECK(authority_observation::count==previous&&forward_grant&&forward_received>origin&&host_programs==8);
    for(unsigned n=0;n<40&&full_load_journal_pending();++n)au_tick();au_tick(3);au_live();au_saved(2);
    CHECK(host_programs==12&&log_programs==logged&&!std::memcmp(saved.data(),host_log,512));
    // Stop through the unchanged normal idle policy before export. First durable
    // snapshot remains completion, not a report of subsequent operations.
    au_request(2,7,0,au_cmd);au_request(255,1,0,au_cmd);CHECK(full_stopped());
    return cont_save();
}
#endif
