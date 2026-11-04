#if BMCU_LITE_FULL_LOAD && BMCU_LITE_AUTOMATIC_CAPTURE
// Reuse the accepted fixture's actual firmware bodies and hardware boundaries.
// No queue/parser/controller implementation or request admission is substituted.
static Packet rxq_dma_bytes(uint32_t first,uint32_t last){
    Packet bytes;for(auto i=first;i!=last;++i)bytes.push_back(uint8_t(rx_dma[i&(RX_SIZE-1u)]));return bytes;
}
static void rxq_head(const Packet& packet){
    CHECK(bus_port_to_host.recv_data_len==int(packet.size()));
    CHECK(!std::memcmp(bus_port_to_host.bus_recv_data_ptr,packet.data(),packet.size()));
}
static void rxq_dispatch(const Packet& packet,bool reply=true){
    if(!bus_port_to_host.recv_data_len)bus_rx_poll();
    rxq_head(packet);const auto tx=repair_tx;repair_step();
    CHECK(repair_tx==tx+unsigned(reply));
    if(reply)CHECK(full_reply[4]==packet[4]);
}
static void rxq_no_pull(){
    CHECK(!(flight::diagnostic().seen_flags&7u)&&!flight::diagnostic().seen_count);
    CHECK(ams[0].now_filament_num==au_ch&&ams[0].filament[au_ch].motion==_filament_motion::on_use);
    CHECK(ams[0].filament_use_flag==4&&full_load_durable(au_ch)&&!forward_grant);
    CHECK(lite_pull_state[au_ch]!=LitePullState::active);
}
static void rxq_pull(){
    au_tick(5);const auto& d=flight::diagnostic();
    CHECK((d.seen_flags&7u)==7u&&d.seen_count==1u);
    CHECK(d.seen_command==au_cmd&&d.seen_channel==au_ch&&d.admitted_command==au_cmd&&d.admitted_channel==au_ch);
    CHECK(d.pwm_channel==au_ch&&d.first_pwm[0]!=d.first_pwm[1]);
    CHECK(lite_pull_state[au_ch]==LitePullState::active&&v11_pwm(au_ch)&&!forward_grant);
}
template<class Report,class Port> static void rxq_snapshot_counters(const Report& report,const Port& port){
    // Dependent members keep the same regression compilable against immutable
    // accepted source, which has the original 388-byte report and no counters.
    if constexpr(sizeof(Report)==400 || sizeof(Report)==472){
#if BMCU_LITE_ENCODER_RECOVERY
        CHECK(flight::build==0x260a3103u);
#else
        CHECK(flight::build==(sizeof(Report)==472?0x260a3102u:0x260a3101u));
#endif
        CHECK(report.rx_dropped==port.rx_dropped&&report.rx_queue_pauses==port.rx_queue_pauses);
        CHECK(report.rx_header_crc_errors==port.rx_header_crc_errors&&report.rx_queue_pauses>0u);
        if(oi_has("invalid"))CHECK(report.rx_header_crc_errors==1u&&report.rx_dropped==0u);
        CHECK(reinterpret_cast<const uint8_t*>(&report.rx_dropped)-reinterpret_cast<const uint8_t*>(&report)==388);
        std::printf("RX-COUNTERS dropped=%u queue_pauses=%u header_crc_errors=%u\n",unsigned(report.rx_dropped),unsigned(report.rx_queue_pauses),unsigned(report.rx_header_crc_errors));
    }else CHECK(sizeof(Report)==388&&flight::build==0x260a2902u);
}
static int rxq_snapshot(){
    host_clock=flight::automatic_deadline;repair_step();
    CHECK(flight::saved()&&flight::complete(host_log)&&full_stopped());
    CHECK(flight::diagnostic().snapshot_kind==3&&flight::diagnostic().before_selected==au_ch);
    if(oi_has("loss-snapshot"))CHECK(flight::diagnostic().before_motion==uint8_t(_filament_motion::on_use));
    rxq_snapshot_counters(flight::report,bus_port_to_host);
    flight::Report saved{};std::memcpy(&saved,host_log+32,sizeof(saved));
    CHECK(!std::memcmp(&saved,&flight::report,sizeof(saved)));
    const auto writes=log_programs;const auto frozen=flight::report;
    flight::automatic_capture();
    CHECK(log_programs==writes&&!std::memcmp(&frozen,&flight::report,sizeof(frozen)));
    std::printf("RX-SNAPSHOT build=%08x bytes=%zu seen=%u before_motion=%u dropped=%u saved=1\n",unsigned(flight::build),sizeof(saved),unsigned(flight::diagnostic().seen_flags),unsigned(flight::diagnostic().before_motion),unsigned(bus_port_to_host.rx_dropped));
    return oi_export();
}
static int rx_queue_verify(){
    CHECK(phase==SetupPhase::operation&&fault==SetupFault::none&&full_load_ready()&&bus_started);
    CHECK(!host_programs&&!host_erases&&full_stopped());
    au_ch=oi_has("owner0")?0:oi_has("owner3")?3:2;au_cmd=oi_has("sender04")?4:3;
    au_trace=std::fopen((std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv").c_str(),"wx");CHECK(au_trace);
    host_clock=((host_clock+999)/1000)*1000;
    host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto& p:full_pulls)p=float(2048)*(3.3f/4095.f);
    repair_request(short_packet(0x20));au_tick(20);oi_start();au_tick(8);rxq_no_pull();
    CHECK(bus_port_to_host.idle&&!bus_port_to_host.send_data_len&&!bus_port_to_host.recv_data_len);
    const auto programs=host_programs,erases=host_erases,log_writes=log_programs;
    std::array<uint8_t,4096> nvm{};std::memcpy(nvm.data(),host_nvm,4096);
    const auto idle03=full_motion(255,1,0,3),idle04=full_motion(255,7,0x66,4),pull=full_motion(au_ch,3,0x3f,au_cmd);

    if(oi_has("busy-wrap")){
        const auto padding=(RX_SIZE-6u-(written&(RX_SIZE-1u)))&(RX_SIZE-1u);
        append(Packet(padding,0x55));bus_rx_poll();CHECK(rx_consumed==written);
        const auto first=rx_consumed,wraps=rx_wraps;
        for(const auto* packet:{&idle03,&idle04,&idle03,&idle04,&pull})append(*packet);
        bus_rx_poll();CHECK(rx_wraps==wraps+1u&&rx_consumed==first+idle03.size()+idle04.size());
        const auto unread=rxq_dma_bytes(rx_consumed,written);CHECK(!unread.empty());rxq_head(idle03);
        full_load_step();CHECK(!bus_port_to_host.idle&&!bus_port_to_host.send_data_len);
        const auto* dma=reinterpret_cast<const uint8_t*>(DMA1_Channel4->MADDR);
        const Packet in_flight(dma,dma+DMA1_Channel4->CNTR);CHECK(in_flight[4]==3);
        rxq_head(idle04);full_load_step();CHECK(!bus_port_to_host.idle&&bus_port_to_host.send_data_len==60);
        const Packet staged(bus_port_to_host.tx_build_buf(),bus_port_to_host.tx_build_buf()+bus_port_to_host.send_data_len);
        bus_rx_poll();CHECK(rx_consumed==written-pull.size());
        const auto blocked=rx_consumed;rxq_head(idle03);
        for(unsigned i=0;i<4;++i){
            full_load_step();CHECK(rx_consumed==blocked);rxq_head(idle03);rxq_no_pull();
            CHECK(DMA1_Channel4->CNTR==in_flight.size()&&!std::memcmp(dma,in_flight.data(),in_flight.size()));
            CHECK(bus_port_to_host.send_data_len==int(staged.size())&&!std::memcmp(bus_port_to_host.tx_build_buf(),staged.data(),staged.size()));
        }
        CHECK(rxq_dma_bytes(first+idle03.size()+idle04.size(),written)==unread);
        repair_complete_tx();CHECK(full_reply==in_flight);
        full_load_step();rxq_head(idle03);CHECK(!bus_port_to_host.idle&&!bus_port_to_host.send_data_len);
        repair_complete_tx();CHECK(full_reply==staged);
        rxq_dispatch(idle03);rxq_no_pull();rxq_dispatch(idle04);rxq_no_pull();rxq_dispatch(pull);rxq_pull();
    }else if(oi_has("overflow")){
        append(idle03);append(idle04);bus_rx_poll();rxq_head(idle03);
        const auto dropped=bus_port_to_host.rx_dropped;
        append(Packet(RX_SIZE,0x55));append(pull);bus_rx_poll();
        CHECK(rx_fault);rxq_no_pull();bus_rx_poll();
        CHECK(!rx_fault&&bus_port_to_host.rx_dropped==dropped+1u&&rx_consumed==0u&&rx_wraps==0u);
        // Actual recovery resets the peripheral DMA cursor. Match only that
        // physical boundary in the existing append fixture, never replay bytes.
        written=0;rxq_dispatch(idle03);rxq_dispatch(idle04);repair_step();rxq_no_pull();
        CHECK(!bus_port_to_host.recv_data_len&&!bus_port_to_host.receiving());
    }else if(oi_has("invalid")){
        auto header=pull;header[3]^=1u;
        auto bad_crc=pull;bad_crc.back()^=1u;
        append(idle03);append(idle04);
        append(Packet{0,0x55,0xaa,0x7f});append(header);
        append(Packet{0x3d,0xc5,6}); // Short length below the actual parser minimum.
        append(Packet{0x3d,5,0,0,1,5}); // Long length 1281 exceeds the slot.
        append(bad_crc);append(pull);
        bus_rx_poll();rxq_dispatch(idle03);rxq_no_pull();rxq_dispatch(idle04);rxq_no_pull();
        rxq_dispatch(bad_crc,false);rxq_no_pull();rxq_dispatch(pull);rxq_pull();
    }else if(oi_has("fragment")){
        append(idle03);append(idle04);const auto first=rx_consumed;
        const auto split=pull.size()/2u;append(Packet(pull.begin(),pull.begin()+split));bus_rx_poll();
        CHECK(rx_consumed==first+idle03.size()+idle04.size()&&!bus_port_to_host.receiving());
        CHECK(rxq_dma_bytes(rx_consumed,written)==Packet(pull.begin(),pull.begin()+split));
        rxq_dispatch(idle03);rxq_no_pull();bus_rx_poll();
        CHECK(rx_consumed==written&&bus_port_to_host.receiving());
        rxq_dispatch(idle04);rxq_no_pull();
        append(Packet(pull.begin()+split,pull.end()));bus_rx_poll();rxq_dispatch(pull);rxq_pull();
    }else if(oi_has("pull-first")){
        append(pull);append(idle04);append(idle03);bus_rx_poll();
        rxq_dispatch(pull);CHECK((flight::diagnostic().seen_flags&3u)==3u&&lite_pull_state[au_ch]==LitePullState::active);
        delay(2);rxq_dispatch(idle04);
        CHECK((flight::diagnostic().seen_flags&7u)==7u&&v11_pwm(au_ch)&&lite_pull_state[au_ch]==LitePullState::active);
        // FF/01/00 preserves on_use but remains the accepted fail-closed stop
        // during an active pull. FIFO must admit the pull before this cancel.
        rxq_dispatch(idle03);lease_stopped();
        CHECK(lite_pull_state[au_ch]==LitePullState::interrupted&&full_load_durable(au_ch));
        CHECK((flight::diagnostic().seen_flags&7u)==7u&&flight::diagnostic().seen_count==1u);
    }else{
        const auto first=rx_consumed,dropped=bus_port_to_host.rx_dropped;
        append(idle03);append(idle04);append(pull);bus_rx_poll();
        const auto consumed_after_poll=rx_consumed;const auto unread=rxq_dma_bytes(rx_consumed,written);
        // Drain even on accepted source so its concrete failure signature is
        // printed before the preservation assertion fails.
        rxq_dispatch(idle03);rxq_dispatch(idle04);repair_step();au_tick(5);
        std::printf("RX-BURST owner=%u command=%u produced=%u consumed_after_poll=%u dropped_delta=%u seen=%u selected=%u motion=%u\n",au_ch,au_cmd,unsigned(written-first),unsigned(consumed_after_poll-first),unsigned(bus_port_to_host.rx_dropped-dropped),unsigned(flight::diagnostic().seen_flags),unsigned(ams[0].now_filament_num),unsigned(ams[0].filament[au_ch].motion));std::fflush(stdout);
        if(oi_has("loss-snapshot")){
            CHECK(bus_port_to_host.rx_dropped==dropped+1u&&consumed_after_poll==written);rxq_no_pull();
            return rxq_snapshot();
        }
        CHECK(consumed_after_poll==first+idle03.size()+idle04.size()&&unread==pull);
        CHECK(bus_port_to_host.rx_dropped==dropped);rxq_pull();
    }
    CHECK(rx_consumed==written&&!bus_port_to_host.recv_data_len&&!bus_port_to_host.receiving());
    CHECK(host_programs==programs&&host_erases==erases&&log_programs==log_writes&&!std::memcmp(nvm.data(),host_nvm,4096));
    if(oi_has("snapshot"))return rxq_snapshot();
    std::printf("RX-QUEUE PASS %s consumed=%u dropped=%u seen=%u\n",scenario.c_str(),unsigned(rx_consumed),unsigned(bus_port_to_host.rx_dropped),unsigned(flight::diagnostic().seen_flags));
    return au_done();
}
#endif
