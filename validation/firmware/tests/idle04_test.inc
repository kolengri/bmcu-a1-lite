#if BMCU_LITE_NATIVE_FEED_AUTHORITY
// Affected policy only. Real CRC parser, main, RAM-backed production journal,
// ADC/filter, controller and PWM run through the accepted harness.
// Time, raw sensor samples, Flash MMIO and DMA/USART completion are modeled.
static bool idle04_has(const char* s){return scenario.find(s)!=std::string::npos;}
static void idle04_quiet_record(){
#if BMCU_LITE_AUTHORITY_OBSERVATION
    CHECK(!authority_observation::mask&&!authority_observation::terminal_fault);
#endif
}
static void idle04_live(){au_live();CHECK(!full_stopped());idle04_quiet_record();}
static void idle04_idle(unsigned command){
    const auto grant=forward_received;const bool had=forward_grant;
    au_request(0xff,1,0,command);
    CHECK(forward_received==grant); // Neither creation nor renewal from an idle.
    if(command==3u||BMCU_LITE_NATIVE_IDLE_04){CHECK(forward_grant==had);idle04_live();}
    else CHECK(!forward_grant&&full_stopped()&&ams[0].now_filament_num==0xff);
}
static void idle04_reply_selected(unsigned command){
    if(command==3){
        CHECK(full_reply.size()==sizeof(bambubus_ams_motion_package_struct));
        bambubus_ams_motion_package_struct r{};std::memcpy(&r,full_reply.data(),sizeof(r));
        CHECK(r.filament_channel==2&&r.filament_channel_2==2&&r.filament_use_flag==2);
    }else{
        CHECK(full_reply.size()==sizeof(bambubus_ams_stu_motion_package_struct));
        bambubus_ams_stu_motion_package_struct r{};std::memcpy(&r,full_reply.data(),sizeof(r));
        CHECK(r.filament_channel==2&&r.filament_use_flag==2);
    }
}
static void idle04_retained(){CHECK(full_load_owner()==2);au_saved(2);}
static void idle04_export(){
    au_tick(3);
#if BMCU_LITE_AUTHORITY_OBSERVATION
    if(time_hw_tpms==1000&&!time_hw_slow){
        if(idle04_has("retained-use"))CHECK(!authority_observation::count&&!authority_observation::mask);
        else CHECK(authority_observation::mask);
        ob_export();
    }
#endif
}
static void idle04_terminal(){
    CHECK(full_load_fault()==FullLoadFault::command_authority&&full_stopped());
    idle04_retained();const auto writes=host_programs;
    // Later BF, heartbeat and idle cannot revive a terminal expiry or release NVM.
    for(unsigned cmd:{3u,4u}){au_request(2,3,0xbf,cmd);repair_request(short_packet(0x20));au_request(0xff,1,0,cmd);}
    au_tick(2);CHECK(full_load_fault()==FullLoadFault::command_authority&&full_stopped()&&host_programs==writes);
    CHECK(!forward_grant);idle04_retained();
}
static int idle04_done(){
    idle04_export();
#if BMCU_USB_FLIGHT_RECORDER
    const bool cancelled=full_stopped()&&ams[0].now_filament_num==0xff&&full_load_durable(2);
    if(full_load_fault()!=FullLoadFault::none||full_load_empty()||cancelled){
        CHECK(flight::finished);
        if(full_load_fault()!=FullLoadFault::none||full_load_empty())CHECK(host_terminal_seen);
        else {
            CHECK(!host_terminal_seen&&flight::complete(host_log));
            if(idle04_has("pull-lease"))CHECK((flight::report.flags&4096)&&!(flight::report.flags&64));
            else CHECK(flight::report.flags&64);
        }
        if(flight::complete(host_log)){
            flight::Report r;std::memcpy(&r,host_log+32,sizeof(r));
            CHECK(r.native_fault==uint8_t(full_load_fault())&&r.phase==8&&(r.flags&16));
            if(full_load_empty())CHECK((r.flags&32)&&full_stopped()&&durable_owner==255);
            CHECK(package_check_crc16(r.ao4,80)&&r.ao4[4]==authority_observation::mask);
            flight::export_frame();flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));
            // Power removal/reset loses every recorder RAM field. The real main
            // must still export the committed stop facts without touching NVM.
            std::array<uint8_t,512> saved;std::memcpy(saved.data(),host_log,512);
            std::array<uint8_t,4096> nvm;std::memcpy(nvm.data(),host_nvm,4096);
            flight_reset();bus_started=false;g_adc_dma_inited=false;host_dma1.CFGR=0;
            host_time_hook=[](){++host_clock;if(log_tx.size()>=536)throw LoopComplete{};};
            try{firmware_main();CHECK(false);}catch(const LoopComplete&){}
            CHECK(!bus_started&&!g_adc_dma_inited&&!host_dma1.CFGR);
            CHECK(!std::memcmp(saved.data(),host_log,512)&&!std::memcmp(nvm.data(),host_nvm,4096));
            CHECK(log_tx.size()==536&&!std::memcmp(log_tx.data()+20,saved.data(),512));
            CHECK(flight::complete(log_tx.data()+20));
            for(auto&row:compare_values)for(auto v:row)CHECK(v==0);
        }
    }
#endif
    return au_done();
}
static int idle04_verify(){
    au_ch=ob_ch=2;au_cmd=idle04_has("sender04")?4u:3u;
    host_clock=((host_clock+999)/1000)*1000;
    host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    const auto path=std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv";
    au_trace=std::fopen(path.c_str(),"wx");CHECK(au_trace);
    for(auto &p:full_pulls)p=float(2048)*(3.3f/4095.f);
    repair_request(short_packet(0x20));au_tick(20);
#if BMCU_USB_FLIGHT_RECORDER
    if(idle04_has("retained-")){
        CHECK(!v11_local_eligible()&&full_load_durable(2)&&ams[0].now_filament_num==0xff);
        if(idle04_has("retained-idle")){
            const auto writes=log_programs;
            for(unsigned cmd:{3u,4u}){au_request(0xff,1,0,cmd);au_tick(3);}
            CHECK(!flight::finished&&!flight::native_checkpoint_pending()&&!flight::complete(host_log));
            CHECK(!(flight::report.flags&(64u|128u))&&log_programs==writes&&full_stopped());
            idle04_retained();return au_done();
        }
        // A real use can be the first native command after retained-owner boot:
        // no BF/grant counter may be assumed as its provenance.
        au_request(2,7,0,au_cmd);au_tick(3);CHECK(ams[0].filament[2].motion==_filament_motion::on_use);
        bambubus_heartbeat_deadline=time_ticks32()-1;au_request(0xff,1,0,4);
        CHECK(full_stopped()&&!forward_grant&&full_load_fault()==FullLoadFault::none);
        CHECK(!flight::finished&&!flight::native_checkpoint_pending());
        repair_request(short_packet(0x20));au_request(0xff,1,0,4);au_tick(2);
        CHECK(full_stopped()&&full_load_fault()==FullLoadFault::none);idle04_retained();return idle04_done();
    }
#endif
    CHECK(v11_local_eligible());
    if(idle04_has("empty")){
        for(unsigned cmd:{3u,4u})au_request(0xff,1,0,cmd);
        CHECK(!forward_grant&&full_stopped()&&full_load_empty()&&!host_programs);
        idle04_quiet_record();return au_done();
    }
    if(idle04_has("pending")){
        full_keys[2]=full_keys[1]=1.8f;au_tick(10);au_bf(au_cmd);
        CHECK(waiting_send&&full_load_journal_pending()&&!full_load_durable(2)&&full_stopped());
        const auto original=forward_received;
        if(idle04_has("stale")){
            // No main/deferred execution until exact expiry, then queue a real BF.
            delay(100);au_request(2,3,0xbf,au_cmd);
            CHECK(full_load_fault()==FullLoadFault::command_authority&&full_stopped()&&durable_owner==0xff);
            const auto writes=host_programs;au_tick(5);au_request(0xff,1,0,4);
            CHECK(host_programs==writes&&full_load_owner()==2&&full_stopped());
            return idle04_done();
        }
        au_request(0xff,1,0,au_cmd);CHECK(!waiting_send&&!forward_grant&&full_stopped());
        if(idle04_has("rearm")){
            delay(60);au_bf(au_cmd==3?4:3);CHECK(waiting_send&&forward_received>original);
            const auto renewed=forward_received;au_request(1,3,0xbf,au_cmd);
            CHECK(forward_received==renewed&&full_load_owner()==2);
            for(unsigned i=0;i<30&&full_load_journal_pending();++i)au_tick();au_tick(3);au_live();CHECK(!full_stopped());
            CHECK(host_clock<original+100000);au_at(99);au_live();au_at(100);idle04_terminal();
        }else{
            au_tick(120);CHECK(!forward_grant&&full_stopped()&&full_load_fault()==FullLoadFault::none);
            CHECK(full_load_durable(2)&&host_programs==4);idle04_retained();
            // A genuinely new request may rearm a nonterminal canceled reservation.
            au_bf(au_cmd);au_tick(2);au_live();CHECK(!full_stopped());au_request(2,7,0,au_cmd);
        }
        return idle04_done();
    }
    au_start();CHECK(au_ch==2&&full_load_owner()==2);idle04_quiet_record();
    if(idle04_has("pull-lease")){
        au_request(2,3,0x3f,au_cmd);au_tick(3);CHECK(lite_pull_state[2]==LitePullState::active);
        bambubus_heartbeat_deadline=time_ticks32()-1;au_tick(3);
        CHECK(lite_pull_state[2]==LitePullState::interrupted&&lite_recovery_fault());
        CHECK(full_load_fault()==FullLoadFault::none&&full_stopped()&&!forward_grant&&ams[0].now_filament_num==0xff);
#if BMCU_USB_FLIGHT_RECORDER
        CHECK(flight::finished&&flight::complete(host_log));
        CHECK((flight::report.flags&4096)&&!(flight::report.flags&64));
        const auto reports=log_programs;
#endif
        std::array<uint8_t,4096> retained;std::memcpy(retained.data(),host_nvm,4096);
        const auto writes=host_programs;
        repair_request(short_packet(0x20));
        for(unsigned cmd:{3u,4u}){au_request(0xff,1,0,cmd);au_bf(cmd);au_request(2,3,0x3f,cmd);au_tick(3);}
        lite_unload_complete(2); // Recovery must also refuse the completion callback.
        CHECK(!lite_motion_ready()&&lite_recovery_fault()&&full_stopped()&&!forward_grant);
        CHECK(full_load_fault()==FullLoadFault::none&&full_load_owner()==2&&host_programs==writes);
        CHECK(!std::memcmp(retained.data(),host_nvm,4096));idle04_retained();
#if BMCU_USB_FLIGHT_RECORDER
        CHECK(log_programs==reports&&!(flight::report.flags&64));
#endif
        return idle04_done();
    }
    if(idle04_has("observed")){
        // Counter/gap-compatible synthetic schedule, NOT a recovered wire trace.
        for(unsigned i=1;i<115;++i){au_at(i==30?21:20);au_bf(au_cmd);}
        au_at(69);idle04_idle(3);idle04_reply_selected(3);
        au_at(70);idle04_idle(4);
        if(!BMCU_LITE_NATIVE_IDLE_04)return idle04_done();
        idle04_reply_selected(4);
#if BMCU_LITE_AUTHORITY_OBSERVATION
        CHECK(authority_observation::count==115&&authority_observation::max_gap==21&&authority_observation::last_gap==20);
        CHECK(authority_observation::age(host_clock)==70&&!authority_observation::pressure_flags);
        CHECK(authority_observation::origin[0]==au_cmd&&authority_observation::origin[1]==2&&authority_observation::phase_flags&4);
#endif
        au_at(90);au_bf(au_cmd==3?4:3);idle04_live();
        au_at(99);idle04_idle(4);idle04_reply_selected(4);
        if(idle04_has("cancel")){
            au_request(0xff,1,1,4);CHECK(full_stopped()&&!forward_grant&&full_load_fault()==FullLoadFault::none);
            au_tick(110);CHECK(full_load_fault()==FullLoadFault::none);idle04_retained();
#if BMCU_LITE_AUTHORITY_OBSERVATION
            ob_fact(1,authority_observation::idle);CHECK(authority_observation::records[1][15]==1);
#endif
        }else{au_at(100);idle04_terminal();}
        return idle04_done();
    }
    if(idle04_has("alias")||idle04_has("boundary")){
        au_bf(au_cmd);const unsigned idle=idle04_has("idle03")?3u:4u;
        idle04_idle(idle);
        if(idle==4&&!BMCU_LITE_NATIVE_IDLE_04){idle04_retained();return idle04_done();}
        au_at(99);idle04_idle(idle);idle04_live();
        const unsigned age=idle04_has("101")?101:100;
        // Enter main with the CRC-valid BF/idle already queued at 100 or 101 ms.
        // Exact >= expiry must run before either packet can renew/bypass.
        delay(age-99);const auto stamp=forward_received;
        au_request(idle04_has("queued-bf")?2:0xff,idle04_has("queued-bf")?3:1,idle04_has("queued-bf")?0xbf:0,au_cmd);
        CHECK(forward_received==stamp);idle04_terminal();return idle04_done();
    }
    if(idle04_has("fault")||idle04_has("lease")){
        FullLoadFault expected=FullLoadFault::inputs;
        if(idle04_has("at100")){au_at(99);delay(1);}
        if(idle04_has("storage")){g_flash_fault=true;expected=FullLoadFault::storage;}
        else if(idle04_has("clock-scale")){time_hw_tpms=1001;expected=FullLoadFault::clock;}
        else if(idle04_has("clock-back")){host_clock-=2000;expected=FullLoadFault::clock;}
        else if(idle04_has("lease")){
            if(idle04_has("lease-use")){au_request(2,7,0,au_cmd);au_tick(3);CHECK(ams[0].filament[2].motion==_filament_motion::on_use);}
            bambubus_heartbeat_deadline=time_ticks32()-1;expected=FullLoadFault::none;
        }
        else if(idle04_has("encoder")){
            full_encoder_valid=uint8_t(15u&~(1u<<2));
            // Existing production encoder health requires consecutive failed polls,
            // not one long elapsed delay. Idle cannot bypass the established trip.
            for(unsigned i=0;i<10&&full_load_fault()==FullLoadFault::none;++i){delay(1);au_request(0xff,1,0,4);}
        }
        else if(idle04_has("adc-stale")){full_adc_stopped=true;delay(101);}
        else{full_pulls[1]=NAN;delay(10);}
        au_request(0xff,1,0,4);
        if(idle04_has("encoder")){std::printf("ENCODER fault=%u expected=%u stopped=%u grant=%u good=%u online=%u magnet=%u t=%llu\n",unsigned(full_load_fault()),unsigned(expected),full_stopped(),forward_grant,AS5600_is_good(2),MC_AS5600.online[2],unsigned(MC_AS5600.magnet_stu[2]),(unsigned long long)host_clock);std::fflush(stdout);}
        CHECK(full_load_fault()==expected&&full_stopped()&&!forward_grant);
#if BMCU_USB_FLIGHT_RECORDER
        if(idle04_has("lease"))CHECK(!flight::finished&&!flight::native_checkpoint_pending());
#endif
        const auto writes=host_programs;
        repair_request(short_packet(0x20));au_request(0xff,1,0,4);au_tick(2);
        CHECK(full_stopped()&&full_load_fault()==expected&&host_programs==writes);
        if(expected!=FullLoadFault::storage)idle04_retained();else CHECK(full_load_owner()==2&&g_sta_saved_loaded==2);
        return idle04_done();
    }
    if(idle04_has("refused")){
        full_keys[1]=1.8f;au_tick(10);au_bf();au_at(80);const auto grant=forward_received;
        au_request(1,3,0xbf,4);au_request(2,3,0xbf,3,1);
        auto bad=full_motion(2,3,0xbf,4);bad.back()^=1;repair_request(bad);
        CHECK(forward_received==grant);idle04_idle(4);
        if(BMCU_LITE_NATIVE_IDLE_04){au_at(100);idle04_terminal();}
        return idle04_done();
    }
    if(idle04_has("wrong-selected")||idle04_has("no-grant")){
        // Explicit RAM state-boundary injection, not a claimed reachable wire sequence.
        if(idle04_has("wrong-selected")){ams[0].now_filament_num=1;ams[0].filament[1].motion=_filament_motion::send_out;}
        else full_load_cancel(); // Actual cancel body removes authority without a new grant.
        au_request(0xff,1,0,4);CHECK(full_stopped()&&ams[0].now_filament_num==0xff&&!forward_grant);
        CHECK(full_load_fault()==FullLoadFault::none);idle04_retained();return idle04_done();
    }
    if(idle04_has("variant")){
        au_request(0xff,1,1,au_cmd);CHECK(full_stopped()&&!forward_grant&&ams[0].now_filament_num==0xff);
        idle04_retained();return idle04_done();
    }
    const bool use=idle04_has("use");
#if BMCU_USB_FLIGHT_RECORDER
    std::array<uint8_t,4096> before_log;std::memcpy(before_log.data(),host_nvm,4096);
#endif
    au_request(2,use?7:3,use?0:0x3f,au_cmd);CHECK(!forward_grant);
    if(use){CHECK(ams[0].filament[2].motion==_filament_motion::on_use);au_tick(3);}
    else{
        au_tick(3);CHECK(lite_pull_state[2]==LitePullState::active);
        if(idle04_has("redetect")||idle04_has("complete")){
            full_keys[2]=.2f;
            for(unsigned i=0;i<30&&filament_now_position[2]!=filament_redetect;++i)au_tick();
            CHECK(filament_now_position[2]==filament_redetect);idle04_retained();
        }
    }
    if(idle04_has("complete")){
        au_tick(100);idle04_retained();CHECK(full_load_owner()==2);
        full_keys[2]=1.8f;
        for(unsigned i=0;i<40&&full_load_owner()!=0xff;++i)au_tick();
        CHECK(full_load_empty()&&lite_pull_state[2]==LitePullState::completed);au_saved(0xff);
        CHECK(host_programs==8); // One verified reserve and one completion-only release.
    }else{
#if BMCU_USB_FLIGHT_RECORDER
        if(idle04_has("logfail"))host_fail_program=true;
#endif
        au_request(0xff,1,0,4);CHECK(full_stopped()&&ams[0].now_filament_num==0xff);
        if(!idle04_has("logfail"))idle04_retained();CHECK(host_programs==4);
        if(!use)CHECK(lite_recovery_fault()&&lite_pull_state[2]==LitePullState::interrupted);
    }
#if BMCU_USB_FLIGHT_RECORDER
    if(idle04_has("logfail")){
        au_tick(3);CHECK(full_load_fault()==FullLoadFault::storage&&full_stopped()&&host_terminal_seen);
        CHECK(!flight::complete(host_log)&&flight::header_valid(host_log));
        CHECK(full_load_owner()==2&&durable_owner==2&&g_sta_saved_loaded==2);
        CHECK(!std::memcmp(before_log.data(),host_nvm,4096));
        au_bf(4);au_tick(3);CHECK(full_stopped()&&full_load_fault()==FullLoadFault::storage);
    }
    if(idle04_has("resume")){
        au_tick(3);CHECK(flight::complete(host_log)&&!host_terminal_seen);
        std::array<uint8_t,512> first;std::memcpy(first.data(),host_log,512);const auto writes=log_programs;
        au_bf(4);au_tick(3);au_live();CHECK(!full_stopped());
        au_request(2,7,0,4);au_tick(3);CHECK(ams[0].filament[2].motion==_filament_motion::on_use);
        au_request(0xff,1,0,4);au_tick(3);CHECK(full_stopped()&&!host_terminal_seen);
        CHECK(log_programs==writes&&!std::memcmp(first.data(),host_log,512));idle04_retained();
    }
#endif
    return idle04_done();
}
#endif
