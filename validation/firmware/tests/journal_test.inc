// Actual production scheduler, journal, parser/controller and boot/export bodies.
static void journal_drain(unsigned bound=5000){
    while(bound--){host_clock+=100;Flash_background_run();flight::journal_step();
        if(!flight::jqcount&&!flight::jstate&&!flight::jwaiting&&Flash_background_idle())return;}
    CHECK(false);
}
static unsigned journal_records(){
    unsigned result=0;for(unsigned page=0;page<2;++page){const uint8_t* p=host_log+page*256;
        if(!flight::jvalid(p))continue;
        for(unsigned slot=0;slot<14;++slot){auto* event=p+32+slot*16;
            if(flight::jget(event+12)==flight::jcrc(event,12,flight::jget(p+4)))++result;}}
    return result;
}
static void journal_inhibited_export(){
    std::array<uint8_t,512> before{};memcpy(before.data(),host_log,512);
    std::array<uint8_t,4096> nvm{};memcpy(nvm.data(),host_nvm,4096);
    const auto programs=log_programs,erases=log_erases;
    flight_reset();bus_started=false;g_adc_dma_inited=false;host_dma1.CFGR=0;
#if BMCU_USB_WORKING_MODE
#if BMCU_USB_ISP_EXPORT
    working_marker_prepare();
#endif
    written=0;host_time_hook=working_export_hook;
#else
    host_time_hook=[](){++host_clock;if(log_tx.size()>=536)throw LoopComplete{};};
#endif
    try{firmware_main();CHECK(false);}catch(const LoopComplete&){}
    CHECK(!bus_started&&!g_adc_dma_inited&&!flight::capturing&&!host_dma1.CFGR);
    CHECK(log_programs==programs&&log_erases==erases&&!memcmp(host_log,before.data(),512));
    CHECK(!memcmp(host_nvm,nvm.data(),4096)&&full_stopped());
    CHECK(log_tx.size()==536&&!memcmp(log_tx.data(),"BFL2",4)&&!memcmp(log_tx.data()+20,host_log,512));
    CHECK(flight::jget(log_tx.data()+532)==flight::jcrc(log_tx.data(),532));
    flight_save((scenario+".frame").c_str(),log_tx.data(),536);
}
static int journal_case(){
    if(is("full-journal-setup-fault")){
        CHECK(phase==SetupPhase::fault&&!bus_started&&full_stopped());
        CHECK(journal_records()>=2);journal_inhibited_export();return 0;
    }
    CHECK(phase==SetupPhase::operation&&full_load_ready()&&bus_started&&flight::capturing);
    if(is("full-journal-run")||is("full-journal-fault")||is("full-journal-traffic")){
        host_time_hook=[](){raw_dma_boundary();};
        repair_request(short_packet(0x20));repair_tick(20,false,false);
        full_present(2);repair_request(full_motion(2,3,0xbf));repair_tick(60,true,false);
        CHECK(full_load_durable(2)&&ams[0].now_filament_num==2);
        repair_request(full_motion(2,7,0));repair_tick(20,true,false);
        CHECK(ams[0].filament[2].motion==_filament_motion::on_use);
        if(is("full-journal-fault")){
            full_load_input_fail(InputFaultSource::encoder_not_good,2);repair_tick(150,true,false);
            CHECK(full_load_fault()==FullLoadFault::inputs&&full_stopped()&&bus_started&&!host_terminal_seen);
            bool found=false;for(unsigned p=0;p<2;++p)for(unsigned s=0;s<14;++s){auto* e=host_log+p*256+32+s*16;
                if(flight::jvalid(host_log+p*256)&&flight::jget(e+12)==flight::jcrc(e,12,flight::jget(host_log+p*256+4))&&(e[8]==9||e[8]==12))found=true;}
            CHECK(found);journal_inhibited_export();return 0;
        }
        const auto start=host_clock;const auto tx=repair_tx;
        for(unsigned ms=0;ms<125000;++ms){
            delay(1);++repair_elapsed;
            if(ms%400==0)repair_request(short_packet(0x20));
            if(ms%100==0)repair_request(full_motion(255,7,0,3+(ms/100)%2));else repair_step();
            CHECK(!flight::operator_pending()&&!host_terminal_seen&&bus_started);
        }
        CHECK(host_clock-start>=uint64_t(time_hw_tpms)*125000u&&repair_tx>tx+1000);
        CHECK(full_load_ready()&&full_load_durable(2)&&ams[0].filament[2].motion==_filament_motion::on_use);
        CHECK(journal_records()<28); // Unchanged polls, alternating senders and meters do not erase on every reply.
        repair_request(full_motion(2,3,0x3f));repair_tick(5,true,false);
        CHECK(lite_pull_state[2]==LitePullState::active&&full_load_fault()==FullLoadFault::none);
        journal_drain();journal_inhibited_export();return 0;
    }
    bus_started=false;host_time_hook=nullptr;journal_drain();
    if(is("full-journal-bootcuts")){
        std::array<uint8_t,512> saved{};memcpy(saved.data(),host_log,512);
        unsigned cases=0;
        // Rearm preserves the previous valid page throughout all erase/program prefixes.
        for(int cut=0;cut<120;++cut){
            memcpy(host_log,saved.data(),512);flight_reset();log_cut=cut;bool interrupted=false;
            try{flight::jprepare(1,2,2);}catch(const PowerCut&){interrupted=true;}log_cut=-1;
            CHECK(flight::jvalid(host_log));flight_reset();bool running=flight::boot();
            if(running){CHECK(flight::jget(host_log+256+24)==flight::jconsumed);flight_reset();CHECK(!flight::boot());}
            ++cases;if(!interrupted)break;
        }
        // Each consumed-marker halfword/byte boundary must never admit two sessions.
        for(int cut=0;cut<8;++cut){
            memcpy(host_log,saved.data(),512);flight_reset();CHECK(flight::jprepare(1,2,2));
            flight_reset();log_cut=cut;try{flight::boot();}catch(const PowerCut&){}log_cut=-1;
            flight_reset();bool running=flight::boot();
            if(running){flight_reset();CHECK(!flight::boot());}
            ++cases;
        }
        CHECK(cases>100);std::printf("Rearm/consume power-cut prefixes checked: %u\n",cases);return 0;
    }
    if(is("full-journal-busy")){
        CHECK(!Flash_diagnostic_submit(0x0800f000u,0,false));
        CHECK(!Flash_diagnostic_submit(0x0800ee01u,0,false));
        flight::jevent(7,2,0x12345678);flight::journal_step();CHECK(g_flash_job.kind==5);
        Flash_background_run();CHECK(g_flash_job.phase==2&&time_hw_slow);
        FLASH->STATR|=FLASH_FLAG_BSY;bus_started=true;
        host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
        // Modeled receive progresses while a peripheral operation is outstanding.
        // This is explicitly not evidence that target Flash instruction fetch cannot stall.
        repair_request(short_packet(0x20));CHECK(bambubus_heartbeat_reset_armed&&full_load_ready());
        CHECK(g_flash_job.phase==2&&full_stopped());
        FLASH->STATR&=~FLASH_FLAG_BSY;repair_step();CHECK(!Flash_setup_fault()&&full_load_ready());
        bus_started=false;journal_drain();journal_inhibited_export();return 0;
    }
    if(is("full-journal-loss")){
        for(unsigned n=0;n<30;++n)flight::jevent(7,2,n);
        CHECK(flight::jlost==22);journal_drain();flight::jevent(15,255,0);journal_drain();
        bool found=false;for(unsigned p=0;p<2;++p)for(unsigned s=0;s<14;++s){auto* e=host_log+p*256+32+s*16;
            if(flight::jvalid(host_log+p*256)&&flight::jget(e+12)==flight::jcrc(e,12,flight::jget(host_log+p*256+4))&&(flight::jget(e+8)>>16)==22)found=true;}
        CHECK(found);journal_inhibited_export();return 0;
    }
    if(is("full-journal-diagnostic-error")){
        const auto core=host_programs;host_fail_program=true;flight::jevent(7,2,99);
        for(unsigned i=0;i<20;++i){Flash_background_run();flight::journal_step();}
        CHECK(flight::jfailed&&!Flash_setup_fault()&&full_load_ready()&&host_programs==core);
        host_fail_program=false;CHECK(Flash_AMS_state_read(&durable_owner));return 0;
    }
    if(is("full-journal-contention")){
        flight::jevent(7,2,99);flight::journal_step();CHECK(g_flash_job.kind==5);
        owner=2;intent=JournalIntent::reserve;CHECK(!Flash_AMS_state_write(2));Flash_background_run();Flash_background_run();
        CHECK(Flash_background_idle());CHECK(!Flash_AMS_state_write(2));CHECK(g_flash_job.kind==2);
        const auto word=flight::jword;flight::journal_step();CHECK(flight::jword==word+1&&g_flash_job.kind==2);
        // Core request priority remains the existing target/lease capability guard.
        g_flash_job={};return 0;
    }
    if(is("full-journal-boot")){
        journal_inhibited_export();host_time_hook=nullptr;
        uint8_t ack[20];memcpy(ack,"BFA2",4);flight::jput(ack+4,flight::build);flight::jput(ack+8,flight::session);
        flight::jput(ack+12,flight::frame_crc);flight::jput(ack+16,flight::jcrc(ack,16));
        for(auto v:ack)log_rx.push_back(v);for(unsigned n=0;n<4;++n)flight::export_step();
        CHECK(flight::acknowledged);flight_reset();host_rcc.RSTSCKR.value=0x04000000;
        CHECK(!flight::boot()&&!flight::capturing);flight_reset();CHECK(flight::boot());
        journal_drain();journal_inhibited_export();return 0;
    }
    if(is("full-journal-wrap")){
        host_clock=(uint64_t(UINT32_MAX)-5u)*time_hw_tpms;
        flight::jevent(7,2,1);journal_drain();host_clock+=20u*time_hw_tpms;
        flight::jevent(7,2,2);journal_drain();CHECK(!flight::jfailed);journal_inhibited_export();return 0;
    }
    // Fill the first page. Rollover always preserves its independently valid history.
    while(flight::jslot<14){flight::jevent(7,2,flight::jslot);journal_drain();}
    std::array<uint8_t,512> initial{};memcpy(initial.data(),host_log,512);
    if(is("full-journal-powercuts")){
        unsigned cuts=0;for(int cut=0;cut<250;++cut){
            memcpy(host_log,initial.data(),512);flight_reset();flight::capturing=true;flight::jgeneration=1;flight::jsession=1;flight::jslot=14;
            phase=SetupPhase::operation;flight::jevent(7,2,999);log_cut=cut;bool interrupted=false;
            try{journal_drain();}catch(const PowerCut&){interrupted=true;}log_cut=-1;
            CHECK(flight::jvalid(host_log));CHECK(journal_records()>=14);
            const auto before=log_programs;flight_reset();
#if BMCU_USB_WORKING_MODE
            CHECK(!working_export_boot());
#else
            CHECK(!flight::boot());
#endif
            CHECK(log_programs==before&&!flight::capturing);
            ++cuts;if(!interrupted){CHECK(journal_records()==15);break;}
        }
        CHECK(cuts>100);std::printf("Power-cut prefixes checked: %u\n",cuts);return 0;
    }
    CHECK(is("full-journal-rollover"));
    for(unsigned n=0;n<90;++n){flight::jevent(7,2,n);journal_drain();}
    CHECK(flight::jgeneration>2&&journal_records()<=28&&journal_records()>=14);
    for(unsigned p=0;p<2;++p)CHECK(flight::jget(host_log+p*256+20)!=flight::jpermit);
    journal_inhibited_export();return 0;
}
