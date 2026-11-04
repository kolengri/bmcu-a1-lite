static bool retained_event(uint8_t kind,uint8_t channel,uint32_t value){
    for(unsigned p=0;p<2;++p){const uint8_t* page=host_log+p*256;if(!flight::jvalid(page))continue;
        for(unsigned s=0;s<14;++s){const uint8_t* e=page+32+s*16;
            if(flight::jget(e+12)==flight::jcrc(e,12,flight::jget(page+4))&&e[8]==kind&&e[9]==channel&&flight::jget(e+4)==value)return true;
        }}return false;
}
static unsigned retained_count(uint8_t kind,uint8_t channel,uint32_t value,bool require_loss=false){
    unsigned count=0;
    for(unsigned p=0;p<2;++p){const uint8_t* page=host_log+p*256;if(!flight::jvalid(page))continue;
        for(unsigned s=0;s<14;++s){const uint8_t* e=page+32+s*16;
            if(flight::jget(e+12)==flight::jcrc(e,12,flight::jget(page+4))&&e[8]==kind&&e[9]==channel&&
               flight::jget(e+4)==value&&(!require_loss||(flight::jget(e+8)>>16)!=0u))++count;
        }}return count;
}
static void retention_metrics(const char* suffix,const std::string& contents){
    flight_save((scenario+suffix).c_str(),contents.data(),contents.size());
}
static void retention_start(){
    host_time_hook=[](){raw_dma_boundary();};
    repair_request(short_packet(0x20));repair_tick(20,false,false);full_present(2);
    repair_request(full_motion(2,3,0xbf));repair_tick(60,true,false);
    CHECK(full_load_durable(2)&&ams[0].now_filament_num==2);
    repair_request(full_motion(2,7,0));repair_tick(20,true,false);
    CHECK(ams[0].filament[2].motion==_filament_motion::on_use);
    freeze_encoder=true;
    for(unsigned ch=0;ch<4;++ch)as5600_odometer_count[ch]=1000+4096*ch;
}
static int retention_case(){
    CHECK(phase==SetupPhase::operation&&full_load_ready()&&bus_started&&flight::capturing);
    retention_start();journal_drain();
    if(is("full-retention-pair-crosspage")){
        while(flight::jslot<13){flight::jevent(15,255,flight::jslot);journal_drain();}
        CHECK(flight::jslot==13);repair_request(full_motion(0,7,0x99));journal_drain();
        CHECK(flight::jgeneration==2&&flight::jslot==1);journal_inhibited_export();return 0;
    }
    if(is("full-retention-pair-boundary")||is("full-retention-pair-wrap")){
        // Actual serializer bytes, observed at the same DMA-start boundary.
        repair_request(full_motion(0,7,0x99));Packet response=full_reply;
        flight_reset();memset(host_log,255,sizeof(host_log));CHECK(flight::jprepare(0,1,1));
        CHECK(Flash_log_write(flight::base+24,flight::jconsumed,false));
        // This fixture resets only recorder state inside an already admitted
        // controller session. Reconstruct its operation phase after reset.
        phase=SetupPhase::operation;
        flight::capturing=true;flight::jgeneration=flight::jsession=1;
        const uint64_t origin=is("full-retention-pair-wrap")?(uint64_t(UINT32_MAX)-1u)*1000u:236804000u;
        host_clock=origin;host_time_hook=[](){host_clock+=1000;};
        flight::pull_wire(3,0,7,0x99);host_clock=origin;
        flight::tx_start(response.data(),response.size());host_time_hook=nullptr;
        CHECK(flight::jqcount==2);const auto tx=flight::jq[flight::jqhead],meter=flight::jq[(flight::jqhead+1)%flight::jqueue_size];
        retention_metrics(".metrics.json","{\"tx_ms\":"+std::to_string(tx.ms)+",\"meter_ms\":"+std::to_string(meter.ms)+"}\n");
        CHECK(tx.ms==meter.ms);CHECK((meter.meta&255u)==18&&meter.value==flight::jget(response.data()+9));
        bus_started=false;journal_drain();journal_inhibited_export();return 0;
    }
    if(is("full-retention-consumed-pull")){
        // Every changed eligible attempt needs its actual refusal, even if its
        // reason matches an earlier attempt. Identical consecutive retries coalesce.
        const uint32_t offline=uint32_t(flight::PullRefusal::offline)|(3u<<16)|(0x3fu<<24);
        ams[0].online=false;repair_request(full_motion(0,3,0x3f));repair_tick(50,true,false);
        const auto first=retained_count(8,0,offline);CHECK(first==1);
        repair_request(full_motion(255,7,0));repair_tick(50,true,false);
        const uint32_t at=flight::jmillis();ams[0].online=false;repair_request(full_motion(0,3,0x3f));repair_tick(100,true,false);journal_drain();
        bool seen=retained_event(19,0,3u|(0x3fu<<8)|(3u<<16));
        const auto second=retained_count(8,0,offline);
        retention_metrics(".retry-metrics.json","{\"first_refusals\":"+std::to_string(first)+",\"after_changed_retry\":"+std::to_string(second)+"}\n");
        CHECK(seen&&second==first+1);
        for(unsigned n=0;n<10;++n){ams[0].online=false;repair_request(full_motion(0,3,0x3f));}repair_tick(50,true,false);journal_drain();
        CHECK(retained_count(8,0,offline)==second);
        ams[0].online=false;repair_request(full_motion(1,3,0x3f));repair_tick(50,true,false);journal_drain();CHECK(retained_count(8,1,offline)==1);
        ams[0].online=false;repair_request(full_motion(1,3,0x3f,4));repair_tick(50,true,false);journal_drain();
        const auto command_reasons=retained_count(8,1,offline);
        if(command_reasons!=2){
            retention_metrics(".refusal-debug.bin",std::string((const char*)host_log,512));
            std::fprintf(stderr,"REFUSAL command-stage count=%u current=%u/%u/%08x pending=%u online=%u fault=%u queue=%u state=%u flashjob=%u\n",command_reasons,flight::jrequest_command,flight::jrequest_channel,flight::jrequest,flight::jrefusal_pending,ams[0].online,unsigned(full_load_fault()),flight::jqcount,flight::jstate,g_flash_job.kind);
        }
        CHECK(command_reasons==2);
        // The next real attempt has a different reason and fails to enqueue while
        // full. Its first successful identical retry must retain reason + losses.
        repair_request(full_motion(255,7,0));repair_tick(50,true,false);ams[0].online=true;
        repair_request(short_packet(0x20));journal_drain();
        const uint32_t mismatch=uint32_t(flight::PullRefusal::loaded_mismatch)|(3u<<16)|(0x3fu<<24);
        for(unsigned n=0;n<flight::jqueue_size;++n)flight::jevent(15,255,n);
        CHECK(flight::jqcount==flight::jqueue_size);
        repair_request(full_motion(0,3,0x3f));journal_drain();
        CHECK(!retained_count(8,0,mismatch)&&flight::jlost>=2);
        repair_request(full_motion(0,3,0x3f));repair_tick(100,true,false);journal_drain();
        CHECK(retained_count(8,0,mismatch)==1&&retained_count(8,0,mismatch,true)==1);
        for(unsigned n=0;n<10;++n)repair_request(full_motion(0,3,0x3f));repair_tick(50,true,false);journal_drain();
        CHECK(retained_count(8,0,mismatch)==1);
        CHECK(full_load_owner()==2&&full_load_durable(2)&&ams[0].now_filament_num==2);
        CHECK(lite_pull_state[0]==LitePullState::ready&&lite_pull_state[1]==LitePullState::ready&&lite_pull_state[2]==LitePullState::ready);
        CHECK(full_load_fault()==FullLoadFault::none&&!host_terminal_seen&&bus_started);
        retention_metrics(".metrics.json","{\"eligible_pull_retained\":true,\"changed_attempt_reasons\":true,\"consecutive_retries_coalesced\":true,\"queue_failure_retried_with_reason_and_loss\":true,\"retry_at_ms\":"+std::to_string(at)+"}\n");
        journal_inhibited_export();return 0;
    }
    CHECK(is("full-retention-scan"));
    // A meaningful real phase precedes both the wrong-owner pull and scan tail.
    repair_request(full_motion(2,7,0x66));repair_tick(100,true,false);
    const uint32_t phase_tuple=7u|(0x66u<<8)|(4u<<16)|(2u<<24);
    CHECK(retained_event(16,2,phase_tuple));
    repair_request(full_motion(0,3,0x3f));repair_tick(100,true,false);
    CHECK(full_load_owner()==2&&full_load_durable(2)&&ams[0].now_filament_num==2);
    CHECK(lite_pull_state[0]==LitePullState::ready&&lite_pull_state[2]==LitePullState::ready);
    const auto stamp=forward_received;const auto grants=authority_observation::count;
    const uint32_t started=flight::jmillis();const unsigned before_programs=log_programs,before_erases=log_erases;
    unsigned sent=0;
    for(unsigned ms=0;ms<240000;++ms){
        delay(1);++repair_elapsed;
        if(ms%400==0)repair_request(short_packet(0x20));
        if(ms%100==0){
            const unsigned ordinal=ms/100,key=(ordinal/2)%5,cmd=3+(ordinal%2),ch=key==4?255:key;
            for(unsigned c=0;c<4;++c)as5600_odometer_count[c]=1000+4096*c+(ordinal&1u);
            repair_request(full_motion(ch,7,ch==255?0:0x99,cmd));++sent;
#if BMCU_LITE_PHASE99_REQUEST_FEEDBACK
            const unsigned off=cmd==3?7:19;CHECK(full_reply[off+1]==ch);
            const float expected=ch<4?1.f-Motion_control_filament_meters(ch):0.f;
            CHECK(!memcmp(full_reply.data()+off+2,&expected,4));
#endif
        }else repair_step();
        CHECK(!host_terminal_seen&&bus_started&&full_load_ready()&&full_load_durable(2));
        CHECK(ams[0].now_filament_num==2&&ams[0].filament[2].motion==_filament_motion::on_use);
        CHECK(lite_pull_state[0]==LitePullState::ready&&lite_pull_state[2]==LitePullState::ready);
    }
    journal_drain();const bool phase_seen=retained_event(16,2,phase_tuple);
    const uint32_t refusal=uint32_t(flight::PullRefusal::loaded_mismatch)|(3u<<16)|(0x3fu<<24);
    const bool refusal_seen=retained_event(8,0,refusal);
    CHECK(forward_received==stamp&&authority_observation::count==grants);
    const auto elapsed=uint32_t(flight::jmillis()-started),generation=flight::jgeneration;
    retention_metrics(".metrics.json","{\"elapsed_ms\":"+std::to_string(elapsed)+",\"scan_requests\":"+std::to_string(sent)+",\"generation\":"+std::to_string(generation)+",\"program_halfwords\":"+std::to_string(log_programs-before_programs)+",\"page_erases\":"+std::to_string(log_erases-before_erases)+",\"prepull_phase_retained\":"+(phase_seen?"true":"false")+",\"pull_refusal_retained\":"+(refusal_seen?"true":"false")+"}\n");
    journal_inhibited_export();CHECK(phase_seen&&refusal_seen);return 0;
}
