#if BMCU_LITE_V11_FEED_LATCH && BMCU_LITE_AUTHORITY_OBSERVATION
// ON expectations are separate from the retained legacy receipt-age cases.
static bool latch_has(const char* name){return scenario.find(name)!=std::string::npos;}
static void latch_live(){au_live();CHECK(forward_grant&&full_load_forward_live());}
static int latch_verify(){
    au_ch=latch_has("ch0")?0u:latch_has("ch1")?1u:latch_has("ch3")?3u:2u;
    au_cmd=latch_has("sender04")?4u:3u;
    const auto path=std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv";
    au_trace=std::fopen(path.c_str(),"wx");CHECK(au_trace);
    host_clock=((host_clock+999)/1000)*1000;
    host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto &p:full_pulls)p=float(2048)*(3.3f/4095.f);
    repair_request(short_packet(0x20));au_tick(20);CHECK(v11_local_eligible());
    if(latch_has("pending")){
        full_keys[au_ch]=1.8f;au_tick(10);au_bf(au_cmd);
        CHECK(waiting_send&&full_load_journal_pending()&&!full_load_durable(au_ch)&&full_stopped());
        const auto stamp=forward_received;const auto grants=authority_observation::count;
        if(latch_has("no-adc")){
            delay(150);repair_step();CHECK(full_load_fault()==FullLoadFault::inputs&&full_stopped()&&!forward_grant);
            CHECK(!full_load_durable(au_ch));return au_done();
        }
        if(latch_has("cancel")){
            au_request(255,1,0,au_cmd);CHECK(!forward_grant&&!waiting_send&&full_stopped());
            au_tick(180);CHECK(full_load_durable(au_ch)&&full_stopped()&&host_programs==4);
            CHECK(full_load_fault()==FullLoadFault::none&&!forward_grant);
            // A heartbeat cannot recreate the canceled intent.
            repair_request(short_packet(0x20));au_tick(100);CHECK(full_stopped()&&!forward_grant);
            CHECK(authority_observation::count==grants&&forward_received==stamp);
        }else{
            // Delay reservation dispatch, keeping real DMA/filter publication and
            // the existing freshness check alive; no BF or deferred admission.
            for(unsigned n=0;n<150;++n){delay(1);(void)ADC_DMA_get_value();CHECK(full_load_check_inputs());}
            repair_step();CHECK(full_load_fault()==FullLoadFault::none&&full_stopped());
            for(unsigned i=0;i<40&&full_load_journal_pending();++i)au_tick();au_tick(5);
            latch_live();CHECK(!full_stopped()&&forward_received==stamp&&authority_observation::count==grants);
            au_request(2,7,0,au_cmd); // Explicit state departure, not a synthetic BF.
            CHECK(!forward_grant);
        }
        return au_done();
    }
    if(latch_has("preempt")){
        full_keys[1]=1.55f;au_tick(180);CHECK(dm_auto_state[1]==DM_AUTO_S1_PUSH&&v11_pwm(1));
    }
    au_start();latch_live();
    if(latch_has("preempt"))CHECK(!v11_pwm(1)&&dm_auto_state[1]==DM_AUTO_IDLE&&!v11_local_eligible());
    if(latch_has("observed")){
        // Owner receipt: 115 admitted BF requests at 20ms, then exact idle03/04
        // at 69/70ms after the final BF. No BF is fabricated during the quiet tail.
        const auto initial_encoder=encoder_count(au_ch);
        for(unsigned n=1;n<115;++n){au_at(20);au_bf(au_cmd);latch_live();}
        CHECK(authority_observation::count==115&&authority_observation::max_gap==20&&authority_observation::last_gap==20);
        const auto stamp=forward_received;const auto bf=full_reply;
        const float expected_meter=-Motion_control_filament_meters(au_ch);
        CHECK(count_distance(encoder_count(au_ch),initial_encoder)>0);
        au_at(69);au_request(255,1,0,3);latch_live();
        au_at(70);au_request(255,1,0,4);latch_live();
        const auto idle=full_reply;
        au_tick(1500);latch_live();CHECK(!full_stopped());
        CHECK(forward_received==stamp&&authority_observation::count==115);
        CHECK(MOTOR_CONTROL[au_ch].motor_stop_time>=time_ms_fast());
        CHECK(MOTOR_CONTROL[au_ch].motor_stop_time<=time_ms_fast()+100);
#if BMCU_USB_FLIGHT_RECORDER
        CHECK(flight::report.bf_length==bf.size()&&!std::memcmp(flight::report.native_bf,bf.data(),bf.size()));
        CHECK(flight::report.idle_command==4&&flight::report.idle_length==idle.size());
        CHECK(!std::memcmp(flight::report.native_idle,idle.data()+19,8));
        CHECK(flight::report.native_tx[0]==3&&flight::report.native_tx[1]==3);
        CHECK(flight::report.tx_completed==117&&!flight::report.tx_aborted);
        const unsigned meter_offset=au_cmd==3u?9u:21u;
        float encoded;std::memcpy(&encoded,bf.data()+meter_offset,4);
        CHECK(std::isfinite(encoded)&&encoded==expected_meter);
        // Derived from the existing real encoder path; no independent measurement.
#endif
    }else{
        // More than ten legacy receipt-age periods, with only real heartbeats.
        const auto stamp=forward_received;au_tick(1200);latch_live();CHECK(!full_stopped());
        CHECK(forward_received==stamp&&authority_observation::count==1);
    }
#if BMCU_USB_FLIGHT_RECORDER
    if(latch_has("tx-abort")){
        const auto completed=flight::report.tx_completed,aborted=flight::report.tx_aborted;
        // Constructing a target response alone does not update submitted bytes.
        const auto before=flight::report;
        flight::response(au_ch,3,0xbf);
        CHECK(!std::memcmp(before.native_bf,flight::report.native_bf,60));
        auto packet=full_motion(au_ch,3,0xbf,au_cmd);append(packet);full_load_step();
        CHECK(!bus_port_to_host.idle&&flight::report.native_tx[0]==1);
        CHECK(flight::report.tx_completed==completed&&flight::report.tx_aborted==aborted);
        // Actual driver's DMA TE path, rather than a constructed-only annotation.
        DMA1->INTFR|=DMA1_FLAG_TE4;bus_rx_poll();
        CHECK(bus_port_to_host.idle&&flight::report.native_tx[0]==5&&flight::report.tx_aborted==uint16_t(aborted+1));
        CHECK(flight::report.tx_completed==completed);
        CHECK(DMA1_Channel5->CNTR==RX_SIZE&&rx_consumed==0);
        written=0; // Hardware seam: production reset restarted the modeled DMA at index0.
        au_tick(3);
    }
#endif
#if BMCU_USB_FLIGHT_RECORDER
    if(latch_has("tx-other")){
        const auto prior=flight::report;
        repair_request(short_packet(5,8));CHECK(full_reply.size()==29&&full_reply[4]==5);
        CHECK(!flight::staged_kind&&!flight::active_kind);
        CHECK(!std::memcmp(prior.native_bf,flight::report.native_bf,sizeof(flight::Report)-308));
    }
#endif
    if(latch_has("lease")){
        const auto stamp=forward_received;
        // Preserve the legacy >0 lease boundary: equality is still live,
        // the first later tick cancels before a queued BF/heartbeat and PWM.
        const uint64_t deadline=bambubus_heartbeat_deadline;
        while(host_clock+1000<=deadline){delay(1);repair_step();}
        host_clock=deadline;repair_step();latch_live();
        CHECK(lite_motion_ready());
        ++host_clock;
        raw_dma_boundary();
        if(latch_has("queued-heartbeat"))repair_request(short_packet(0x20));else au_bf(au_cmd);
        CHECK(!forward_grant&&full_stopped()&&ams[0].now_filament_num==255&&full_load_fault()==FullLoadFault::none);
        CHECK(forward_received==stamp);
        repair_request(short_packet(0x20));au_tick(30);CHECK(!forward_grant&&full_stopped());
        au_saved(au_ch);return au_done();
    }
    if(latch_has("watchdog")){
        const auto deadline=MOTOR_CONTROL[au_ch].motor_stop_time;
        MOTOR_CONTROL[au_ch].run(.001f,deadline+1);
        CHECK(MOTOR_CONTROL[au_ch].motion==filament_motion_enum::filament_motion_stop&&full_stopped());
        CHECK(forward_grant); // The independent controller deadline still stops PWM.
        au_request(255,1,1,au_cmd);CHECK(!forward_grant);return au_done();
    }
    if(latch_has("refused")){
        const auto stamp=forward_received;const auto count=authority_observation::count;
        au_request((au_ch+1)%4,3,0xbf,au_cmd);au_request(au_ch,3,0xbf,au_cmd,1);
        auto bad=full_motion(au_ch,3,0xbf,au_cmd);bad.back()^=1;repair_request(bad);
        au_tick(150);latch_live();CHECK(forward_received==stamp&&authority_observation::count==count);
    }
    if(latch_has("unsupported")){
        au_request(au_ch,2,0x42,au_cmd);CHECK(!forward_grant&&full_stopped()&&full_load_fault()==FullLoadFault::unsupported);return au_done();
    }
    if(latch_has("use")){
        au_request(au_ch,7,0,au_cmd);CHECK(!forward_grant&&ams[0].filament[au_ch].motion==_filament_motion::on_use);
        au_tick(150);CHECK(!forward_grant&&full_load_fault()==FullLoadFault::none);
    }
    // Nonexempt idle variant explicitly cancels even with a healthy lease.
    au_request(255,1,1,au_cmd);CHECK(!forward_grant&&full_stopped()&&ams[0].now_filament_num==255);
    au_saved(au_ch);
#if BMCU_USB_FLIGHT_RECORDER
    au_tick(3);CHECK(flight::finished&&flight::complete(host_log));
    flight::export_frame();flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));
    const auto frozen=flight::report;
    // Direct producer calls after freeze cannot mutate the native snapshot.
    flight::response(au_ch,3,0xbf);flight::tx_start(full_reply.data(),uint16_t(full_reply.size()));flight::tx_finish(true);
    CHECK(!std::memcmp(&frozen.native_bf,&flight::report.native_bf,sizeof(flight::Report)-308));
#endif
    return au_done();
}
#endif
