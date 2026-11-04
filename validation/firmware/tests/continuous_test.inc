static int continuous_setup_fault_verify(){
    CHECK(phase==SetupPhase::fault&&!bus_started&&fault!=SetupFault::none);
    if(is("cal-invalid"))CHECK(fault==SetupFault::adc);
    if(is("direction-timeout"))CHECK(fault==SetupFault::direction);
    if(is("journal-error"))CHECK(fault==SetupFault::storage); // Existing live storage poll wins before setup write timeout.
    std::printf("SETUP FAULT DIAGNOSTIC stopped=%d flags=%u finished=%d fault=%u\n",full_stopped(),flight::report.flags,flight::finished,unsigned(fault));
    CHECK(full_stopped()&&!(flight::report.flags&8192u)&&(flight::report.flags&2u));
    CHECK(flight::finished);
    // Program failure can inhibit the report; a partial payload is never complete.
    if(!is("journal-error"))CHECK(flight::complete(host_log));
    flight::export_frame();flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));
    std::puts("CONTINUOUS terminal setup failure retained; no native handoff or setup-success marker");return 0;
}
// One runtime from actual fresh calibration through strict native init and wire flows.
static int continuous_cut_export(){
    CHECK(!flight::complete(host_log)&&!flight::finished);
    flight_save((scenario+".live-report388").c_str(),&flight::report,sizeof(flight::report));
    flight::export_frame();flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));
    std::array<uint8_t,512> saved{};std::memcpy(saved.data(),host_log,512);
    std::array<uint8_t,4096> nvm{};std::memcpy(nvm.data(),host_nvm,4096);
    const auto writes=host_programs,erases=host_erases,lp=log_programs;
    flight_reset();bus_started=false;g_adc_dma_inited=false;host_dma1.CFGR=0;
    host_time_hook=[](){++host_clock;if(log_tx.size()>=536)throw LoopComplete{};};
    try{firmware_main();CHECK(false);}catch(const LoopComplete&){}
    CHECK(!bus_started&&!g_adc_dma_inited&&!host_dma1.CFGR&&!flight::capturing);
    CHECK(host_programs==writes&&host_erases==erases&&log_programs==lp);
    CHECK(!std::memcmp(nvm.data(),host_nvm,4096)&&!std::memcmp(saved.data(),host_log,512));
    for(auto& row:compare_values)for(auto v:row)CHECK(!v);
    CHECK(log_tx.size()==536&&!std::memcmp(log_tx.data()+20,saved.data(),512));
    std::puts("CONTINUOUS RAM-loss USB actual-main export: incomplete payload unavailable, no sensor/bus/owner initialization");return 0;
}
static int continuous_verify(){
    CHECK(phase==SetupPhase::operation&&fault==SetupFault::none&&full_load_ready()&&bus_started);
    CHECK(reached==5&&flight::report.reached==5&&(flight::report.flags&8192u)&&!(flight::report.flags&2u));
    CHECK(!flight::finished&&!flight::complete(host_log)&&flight::capturing&&!flight::acknowledged);
    CHECK(flight::session==2&&flight::get32(host_log+20)==0xffffffffu);
    for(unsigned n=24;n<508;++n)CHECK(host_log[n]==255); // No setup payload or commit consumed.
    CHECK(host_programs==60&&host_erases==2&&cal_retained()&&Motion_control_setup_retained());
    uint8_t stored=0;CHECK(Flash_AMS_state_read(&stored)&&stored==255&&full_load_empty()&&full_stopped());
    CHECK(queued_green&&accepted_tick&&cal_start&&direction_start);
    for(unsigned ch=0;ch<4;++ch){CHECK(drives[ch]>0&&MC_PULL_calibration_is_valid(ch));CHECK(MOTOR_CONTROL[ch].motion==filament_motion_enum::filament_motion_stop);}
    flight_save((scenario+".calibrated-nvm4096").c_str(),host_nvm,4096);
    flight_save((scenario+".after-setup-report388").c_str(),&flight::report,sizeof(flight::report));
    if(scenario.find("cut-after-setup")!=std::string::npos)return continuous_cut_export();
    // Native counters start here; all setup writes were asserted above.
    host_programs=host_erases=0;
    au_ch=2;au_cmd=scenario.find("sender04")!=std::string::npos?4:3;
    au_trace=std::fopen((std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv").c_str(),"wx");CHECK(au_trace);
    host_clock=((host_clock+999)/1000)*1000;
    host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto& p:full_pulls)p=float(2048)*(3.3f/4095.f);
    repair_request(short_packet(0x20));au_tick(20);
    if(scenario.find("onuse-workflow")!=std::string::npos)return onuse_workflow();
    if(scenario.find("workflow")!=std::string::npos)return ff07_workflow();
    au_start();au_request(2,7,0,au_cmd);ff07_family();
    if(scenario.find("cut-active")!=std::string::npos){CHECK(full_load_durable(2));const int r=continuous_cut_export();return r;}
    if(scenario.find("fault-input")!=std::string::npos){
        full_encoder_valid=7;for(unsigned n=0;n<kAS5600_FAIL_TRIP&&!host_terminal_seen;++n){delay(1);repair_step();}
        CHECK(full_load_fault()==FullLoadFault::inputs&&flight::report.input_detail==0x0309u);
    }else{
        au_request(255,0,0,au_cmd);CHECK(full_load_fault()==FullLoadFault::unsupported);
    }
    ff07_fault_export();CHECK((flight::get32(host_log+32)&8192u)&&!(flight::get32(host_log+32)&2u));
    return au_done();
}
