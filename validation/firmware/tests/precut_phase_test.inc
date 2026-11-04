#if BMCU_LITE_PRECUT_PHASE_CAPTURE
static unsigned precut_class(unsigned ch,unsigned motion){
    return (motion==0?0:motion==0x26?1:motion==0x66?2:3)*2+(ch!=255);
}
static void precut_check(const Packet& actual,unsigned ch,unsigned motion,unsigned cmd,unsigned ordinal,uint64_t started){
    const unsigned index=precut_class(ch,motion),off=cmd==3?7:19;
    const auto& entry=flight::report.precut[index];
    if(entry.request!=((ch&15)|((cmd-3)<<4)|(ordinal<<5)))std::fprintf(stderr,"PRECUT mismatch index=%u ch=%u cmd=%u ordinal=%u actual=%u count=%u consumed=%x submitted=%x\n",index,ch,cmd,ordinal,unsigned(entry.request),unsigned(flight::phase_count),unsigned(flight::report.precut_consumed),unsigned(flight::report.precut_submitted));
    CHECK(entry.request==((ch&15)|((cmd-3)<<4)|(ordinal<<5)));
    CHECK(entry.feedback==uint8_t(actual[off]|(actual[off+1]<<4)));
    CHECK(!std::memcmp(entry.meters,actual.data()+off+2,4));
    CHECK(entry.elapsed_10ms==uint32_t(started-flight::phase_origin)/(time_hw_tpms*10u));
    CHECK(flight::report.precut_submitted&(1u<<(index*2+cmd-3)));
}
static int precut_verify(){
    au_ch=oi_has("owner0")?0:oi_has("owner1")?1:oi_has("owner3")?3:2;
    au_cmd=oi_has("sender04")?4:3;
    au_trace=std::fopen((std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv").c_str(),"wx");CHECK(au_trace);
    host_clock=((host_clock+999)/1000)*1000;
    host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto& p:full_pulls)p=float(2048)*(3.3f/4095.f);
    repair_request(short_packet(0x20));au_tick(20);
    au_request(255,7,0x26,3);au_request(255,7,0x99,4);
    CHECK(!flight::report.precut_consumed&&!flight::report.precut_submitted&&!flight::phase_count);
    au_start();
    const auto log_writes=log_programs;
    au_request(au_ch,7,0,au_cmd);
    CHECK(flight::report.precut_consumed==(1u<<(2+au_cmd-3)));
    CHECK(flight::report.precut_submitted==flight::report.precut_consumed&&flight::phase_count==1);
    precut_check(full_reply,au_ch,0,au_cmd,0,host_clock);
    const auto first=flight::report.precut[1];
    const uint64_t deadline=flight::automatic_deadline;
    // A real consumed request refused before construction has no submitted evidence.
    ams[0].online=false;au_request(255,7,0x66,4);ams[0].online=true;
    CHECK(flight::report.precut_consumed&(1u<<9));CHECK(!(flight::report.precut_submitted&(3u<<8)));
    CHECK(!flight::report.precut[4].request&&!flight::report.precut[4].feedback);
    // A constructed response held behind an actual in-flight DMA is not yet submitted.
    append(full_motion(au_ch,7,0,au_cmd));bus_rx_poll();bambubus_run();bus_port_to_host.send_package();
    CHECK(!bus_port_to_host.idle);
    append(full_motion(255,7,0x66,4));bus_rx_poll();bambubus_run();
    CHECK(bus_port_to_host.send_data_len==60);bus_port_to_host.send_package();
    CHECK(bus_port_to_host.send_data_len==60&&!(flight::report.precut_submitted&(3u<<8)));
    Packet staged(bus_port_to_host.tx_build_buf(),bus_port_to_host.tx_build_buf()+60);
    repair_complete_tx();
    bus_started=false;bus_port_to_host.send_package(); // Actual DMA function refuses a stopped peripheral.
    CHECK(bus_port_to_host.send_data_len==60&&!(flight::report.precut_submitted&(3u<<8)));
    bus_started=true;
    const auto submitted_at=host_clock;
    bus_port_to_host.send_package();CHECK(!bus_port_to_host.send_data_len);
    precut_check(staged,255,0x66,4,1,submitted_at);
    bus_tx_finish(true);CHECK(bus_port_to_host.idle&&rx_fault); // Actual abort hook retains first submitted bytes.
    bus_rx_poll();CHECK(!rx_fault&&DMA1_Channel5->CNTR==RX_SIZE);
    written=0; // Match the actual peripheral DMA cursor reset, as in RX overflow fixtures.
    CHECK(flight::report.precut[4].elapsed_10ms==first.elapsed_10ms);
    // The first response survives different later sender/channel and exact odometry changes.
    const auto global66=flight::report.precut[4];
    au_request(255,7,0x66,3);CHECK(!std::memcmp(&global66,&flight::report.precut[4],sizeof(global66)));
    unsigned ordinal=2;
    for(unsigned motion:{0u,0x26u,0x66u,0x99u})for(unsigned ch:{255u,au_ch}){
        const unsigned index=precut_class(ch,motion);
        if(index!=1&&index!=4){
            au_tick(13);const unsigned cmd=(ordinal&1)?4:3;
            au_request(ch,7,motion,cmd);precut_check(full_reply,ch,motion,cmd,ordinal++,host_clock);
        }
        for(unsigned cmd:{3u,4u})au_request(ch,7,motion,cmd);
    }
    CHECK(ordinal==8&&flight::phase_count==8);
    CHECK(flight::report.precut_consumed==65535u&&flight::report.precut_submitted==65535u);
    CHECK(!std::memcmp(&first,&flight::report.precut[1],sizeof(first)));
    // All actual requested addressed channels0..3 are independently consumed;
    // per-class first-response records remain frozen after the first channel.
    const auto tail=flight::report;
    for(unsigned ch=0;ch<4;++ch)for(unsigned cmd:{3u,4u})au_request(ch,7,0x99,cmd);
    CHECK(!std::memcmp(tail.precut,flight::report.precut,sizeof(tail.precut)));
    CHECK(log_programs==log_writes&&flight::automatic_deadline==deadline);
    // Ordinary/non-motion requests and both native-idle replies cannot reuse
    // a previous phase association; retain the existing last idle endpoint.
    CHECK(!flight::staged_phase);repair_request(short_packet(0x20));
    for(unsigned cmd:{3u,4u}){
        au_request(255,1,0,cmd);CHECK(flight::report.idle_command==cmd);
        CHECK(!std::memcmp(tail.precut,flight::report.precut,sizeof(tail.precut)));
        CHECK(flight::report.precut_consumed==65535u&&flight::report.precut_submitted==65535u&&!flight::staged_phase);
        CHECK(!std::memcmp(flight::report.native_idle,full_reply.data()+(cmd==3?7:19),8));
    }
    if(oi_has("wrap"))CHECK(uint32_t(deadline)<flight::phase_origin);
    if(oi_has("deadline-irq")){
        // Model the legal boundary where the actual DMA begins after the
        // deadline check in main, before its next terminal-capture iteration.
        // Clear one class only as a marked fixture setup to probe timestamp range.
        flight::report.precut_submitted&=~(3u<<12);flight::phase_count=7;
        append(full_motion(255,7,0x99,4));bus_rx_poll();bambubus_run();
        host_clock=deadline+uint64_t(time_hw_tpms)*7u;
        const auto at=host_clock;bus_port_to_host.send_package();
        precut_check(Packet(reinterpret_cast<uint8_t*>(DMA1_Channel4->MADDR),reinterpret_cast<uint8_t*>(DMA1_Channel4->MADDR)+DMA1_Channel4->CNTR),255,0x99,4,7,at);
        CHECK(flight::report.precut[6].elapsed_10ms==12000);repair_complete_tx();
    }
    host_clock=deadline+uint64_t(time_hw_tpms)*10u;repair_step();
    CHECK(flight::saved()&&flight::complete(host_log)&&flight::finished&&host_terminal_seen);
    const auto frozen=flight::report;const auto lp=log_programs;
    CHECK(!std::memcmp(host_log+32,&frozen,sizeof(frozen)));
    // Frozen hooks must not mutate the chronological tail, including IRQ paths.
    flight::pull_wire(3,255,7,0x26);flight::response(255,7,0x26);
    uint8_t bytes[60]={};flight::tx_start(bytes,60);flight::tx_finish(false);
    CHECK(!std::memcmp(&frozen,&flight::report,sizeof(frozen))&&log_programs==lp);
    oi_save();
    CHECK(!std::memcmp(flight::frame+52,&frozen,sizeof(frozen)));
    CHECK(flight::frame[34]==(sizeof(frozen)&255)&&flight::frame[35]==(sizeof(frozen)>>8));
    return oi_export();
}
#endif
