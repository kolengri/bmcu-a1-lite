#if BMCU_LITE_AUTOMATIC_CAPTURE
static int automatic_verify(){
    au_ch=oi_has("owner0")?0:oi_has("owner1")?1:oi_has("owner3")?3:2;
    au_cmd=oi_has("sender04")?4:3;
    au_trace=std::fopen((std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv").c_str(),"wx");CHECK(au_trace);
    host_clock=((host_clock+999)/1000)*1000;
    host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto& p:full_pulls)p=float(2048)*(3.3f/4095.f);
    repair_request(short_packet(0x20));au_tick(20);
    CHECK(!(flight::report.flags&(1u<<20)));
    au_start();CHECK(!(flight::report.flags&(1u<<20)));
    if(oi_has("no-use")){
        const auto writes=log_programs;host_clock+=uint64_t(time_hw_tpms)*180000u;
        flight::automatic_capture();
        CHECK(!flight::operator_pending()&&!flight::finished&&log_programs==writes);
        return oi_export();
    }
    // Arm through the actual parser, never a synthetic timer setter.
    const uint64_t first_use=host_clock;
    au_request(au_ch,7,0,au_cmd);
    CHECK(flight::report.flags&(1u<<20));
    CHECK(flight::automatic_deadline>=first_use+uint64_t(time_hw_tpms)*120000u);
    CHECK(flight::automatic_deadline<=host_clock+uint64_t(time_hw_tpms)*120000u);
    const uint64_t deadline=flight::automatic_deadline;
    for(unsigned i=0;i<5;++i){au_tick(10);au_request(au_ch,7,0,au_cmd);au_request(255,7,0x66,au_cmd);}
    CHECK(flight::automatic_deadline==deadline&&!flight::finished);
    if(oi_has("manual-disabled")){
        full_pulls[(au_ch+1)%4]=1.4f;au_tick(5300);
        CHECK(!flight::operator_pending()&&!flight::finished&&!diagnostic_green_deliveries);
        full_pulls[(au_ch+1)%4]=1.65f;
    }
    if(oi_has("lease-gap")){
        bambubus_heartbeat_deadline=time_ticks32()-1;repair_step();
        CHECK(!lite_lease_active()&&full_stopped()&&!flight::finished);
        repair_request(short_packet(0x20));
        CHECK(flight::automatic_deadline==deadline);
    }
    if(oi_has("pull")){
        au_request(au_ch,3,0x3f,au_cmd);au_tick(5);
        CHECK(lite_pull_state[au_ch]==LitePullState::active);
        CHECK((flight::diagnostic().seen_flags&6u)==6u);
    }
    if(oi_has("fault")){
        full_load_fail(FullLoadFault::inputs);repair_step();
        CHECK(flight::saved()&&flight::diagnostic().snapshot_kind==2&&!flight::operator_pending());
        const auto bytes=log_programs;host_clock=deadline;flight::automatic_capture();
        CHECK(log_programs==bytes&&flight::diagnostic().snapshot_kind==2);
        return oi_export();
    }
    if(oi_has("long")){
        // Run actual main/controller/ADC for the full virtual interval; avoid a
        // multi-megabyte per-PWM trace while keeping every assertion/seam active.
        FILE* trace=au_trace;au_trace=nullptr;
        while(host_clock+uint64_t(time_hw_tpms)*2u<deadline){
            delay(1);if(++au_elapsed%400==0)repair_request(short_packet(0x20));repair_step();
            CHECK(!flight::operator_pending()&&!flight::finished);
        }
        au_trace=trace;
    }
    std::array<uint8_t,4096> nvm{};std::memcpy(nvm.data(),host_nvm,4096);
    const auto np=host_programs,lp=log_programs,ne=host_erases;
    // No save immediately before the boundary, including a 32-bit tick wrap.
    host_clock=deadline-1;flight::automatic_capture();
    CHECK(!flight::operator_pending()&&!flight::finished&&log_programs==lp);
    if(oi_has("pending")){
        g_flash_job={};g_flash_job.kind=2;g_flash_job.addr=0x0800f610u;
        g_flash_job.halfwords=4;g_flash_job.channel=uint8_t(au_ch);
        g_flash_job.words[0]=0x1234abcd;g_flash_job.words[1]=0x5678efab;
        intent=JournalIntent::release; // Automatic cutoff must not wait for it.
    }
    if(oi_has("write-fail"))host_fail_program=true;
    if(oi_has("permanent-busy"))FLASH->STATR|=FLASH_FLAG_BSY;
    if(oi_has("cut-"))log_cut=int(std::stoul(scenario.substr(scenario.find("cut-")+4)));
    const auto old_selected=ams[0].now_filament_num,old_owner=full_load_owner();
    const auto old_motion=old_selected<4u?uint8_t(ams[0].filament[old_selected].motion):255u;
    // A queued new native command must not run at/after the cutoff.
    append(full_motion(au_ch,3,0xbf,au_cmd));host_clock=deadline;
    bool power_cut=false;
    try{repair_step();}catch(const PowerCut&){power_cut=true;}
    log_cut=-1;
    CHECK(flight::operator_pending()&&full_stopped()&&!full_load_ready()&&!lite_motion_ready());
    CHECK(!bus_started&&host_programs==np&&host_erases==ne&&!std::memcmp(nvm.data(),host_nvm,4096));
    const auto& d=flight::diagnostic();
    CHECK(d.snapshot_kind==3&&d.gesture_channel==255&&d.before_selected==old_selected&&d.before_owner==old_owner&&d.before_motion==old_motion);
    if(power_cut||oi_has("write-fail")||oi_has("permanent-busy")){
        CHECK(!flight::saved()&&!flight::complete(host_log)&&!diagnostic_green_deliveries);
    }else{
        CHECK(flight::saved()&&flight::complete(host_log)&&host_terminal_seen&&diagnostic_amber_deliveries&&diagnostic_green_deliveries);
        const auto writes=log_programs;flight::automatic_capture();
        CHECK(log_programs==writes&&flight::saved());
    }
    std::printf("AUTO cutoff=%llu capture_kind=%u saved=%u power_cut=%u pending=%u\n",(unsigned long long)deadline,d.snapshot_kind,unsigned(flight::saved()),power_cut,unsigned(full_load_journal_pending()));
    return oi_export();
}
#endif
