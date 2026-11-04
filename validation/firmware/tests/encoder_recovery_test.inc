static int encoder_recovery_verify(){
    au_ch=oi_has("owner0")?0:oi_has("owner1")?1:oi_has("owner3")?3:2;
    au_cmd=oi_has("sender04")?4:3;
    au_trace=std::fopen((std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv").c_str(),"wx");CHECK(au_trace);
    host_clock=((host_clock+999u)/1000u)*1000u;
    host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto& p:full_pulls)p=float(2048)*(3.3f/4095.f);
    repair_request(short_packet(0x20));au_tick(20);au_start();
    freeze_encoder=true;
    if(oi_has("driver-equivalence")){
        encoder_event_trace=true;
        for(unsigned raw:{0u,4095u,4096u})for(unsigned mask:{0u,1u,2u,4u,8u,15u})for(unsigned stage:{0u,1u,2u}){
            encoder_forced_raw=int(raw);encoder_fault_mask=mask;encoder_fault_remaining=1u;encoder_fault_stage=stage;
            const uint32_t ticks=host_stk.value;
            const auto valid=MC_AS5600.updata_angle();
            CHECK(valid==(raw<=4095u?(15u&~mask):0u));
            for(unsigned ch=0;ch<4u;++ch)CHECK(MC_AS5600.online[ch]==bool(valid&(1u<<ch)));
            encoder_events.push_back(host_stk.value-ticks);
        }
        for(unsigned mask:{0u,1u,2u,4u,8u,15u})for(unsigned stage:{0u,1u,2u}){
            encoder_status_fault=mask;encoder_fault_stage=stage;
            const uint32_t ticks=host_stk.value;
            MC_AS5600.updata_stu();
            for(unsigned ch=0;ch<4u;++ch){
                CHECK(MC_AS5600.online[ch]==!(mask&(1u<<ch)));
                CHECK(MC_AS5600.magnet_stu[ch]==(mask&(1u<<ch)?AS5600_soft_IIC_many::offline:AS5600_soft_IIC_many::normal));
            }
            encoder_events.push_back(host_stk.value-ticks);
        }
        encoder_event_trace=false;
        flight_save("driver-events.bin",reinterpret_cast<const uint8_t*>(encoder_events.data()),encoder_events.size()*sizeof(uint32_t));
        std::printf("DRIVER-EQUIVALENCE trials=72 events=%zu\n",encoder_events.size());
        return au_done();
    }
    if(oi_has("deadline"))au_request(au_ch,7,0,au_cmd);
    if(oi_has("wrap")){
        // Synthetic low32 boundary; keep64-bit authority/ADC freshness coherent.
        host_clock=uint64_t(UINT32_MAX)-999u;
        publication_ticks=previous_ticks=host_clock;
        bambubus_heartbeat_deadline=time_ticks32()+1000000u;
    }
    const unsigned bad=oi_has("other")?(au_ch+1u)%4u:au_ch;
    const uint32_t before=encoder_count(bad),raw_before=MC_AS5600.raw_angle[bad];
    const auto programs=host_programs,erases=host_erases;
    if(oi_has("magnet")){
        encoder_status_fault=1u<<bad;MC_AS5600.updata_stu();
        CHECK(MC_AS5600.magnet_stu[bad]==AS5600_soft_IIC_many::offline);
        delay(1);repair_step();
        CHECK(full_load_fault()==FullLoadFault::inputs&&full_stopped());
        CHECK(flight::report.input_detail==uint16_t(11u|(bad<<8)));
        return oi_export();
    }
    encoder_bad_raw=oi_has("raw");encoder_fault_mask=1u<<bad;
    encoder_fault_remaining=oi_has("persistent")?2u:1u;
    unsigned priority=0;
    if(std::sscanf(scenario.c_str(),"full-encoder-priority-s%u",&priority)==1){
        CHECK(priority>=1u&&priority<=9u);
        input_source=priority;input_channel=(bad+1u)%4u;input_active=true;
    }
    const auto reads=encoder_angle_reads;
    delay(1);repair_step();
    input_active=false;
    CHECK(encoder_angle_reads==reads+1u);
    CHECK(!MC_AS5600.online[bad]&&AS5600_is_good(bad)&&g_as5600_fail[bad]==1u);
    CHECK(encoder_count(bad)==before&&MC_AS5600.raw_angle[bad]==raw_before);
    CHECK(full_stopped());
    if(priority){
        CHECK(full_load_fault()==FullLoadFault::inputs);
        CHECK(flight::report.input_detail==uint16_t(priority|((priority<=2u?255u:input_channel)<<8)));
        return oi_export();
    }
#if !BMCU_LITE_ENCODER_RECOVERY
    CHECK(full_load_fault()==FullLoadFault::inputs);
    CHECK(flight::report.input_detail==uint16_t(10u|(bad<<8)));
    return oi_export();
#else
    CHECK(full_load_fault()==FullLoadFault::none);
    CHECK(as5600_invalid==1u);
    CHECK((flight::report.flags&(3u<<21))==(1u<<21));
    // No intervening loop or queued request may use the stale good latch.
    for(unsigned n=0;n<3u;++n){
        Motion_control_run(0);CHECK(encoder_angle_reads==reads+1u&&full_stopped());
        au_request(au_ch,3,0xbf,n%2u?3u:4u);CHECK(full_stopped());
        full_load_dispatch(true);Motion_control_set_PWM(au_ch,500);full_load_dispatch(false);
        CHECK(full_stopped()&&full_load_fault()==FullLoadFault::none);
    }
    if(oi_has("deadline")){
        const auto deadline=flight::automatic_deadline;
        CHECK(deadline>host_clock);
        append(full_motion(au_ch,3,0xbf,au_cmd));host_clock=deadline;
        repair_step();
        CHECK(flight::finished&&flight::saved()&&full_stopped()&&encoder_angle_reads==reads+1u);
        CHECK(flight::diagnostic().snapshot_kind==3u&&(flight::report.flags&(3u<<21))==(1u<<21));
    }else if(oi_has("clock")){
        ++time_hw_tpms;delay(1);repair_step();
        CHECK(full_load_fault()==FullLoadFault::clock&&full_stopped());
    }else if(oi_has("flash")){
        g_flash_fault=true;delay(1);repair_step();
        CHECK(full_load_fault()==FullLoadFault::storage&&full_stopped());
    }else if(oi_has("lease")){
        bambubus_heartbeat_deadline=time_ticks32();
        append(short_packet(0x20)); // A queued late heartbeat cannot resume the interrupted intent.
        delay(1);repair_step();
        CHECK(full_stopped()&&ams[0].now_filament_num==255u&&!forward_grant);
        CHECK(full_load_durable(au_ch));
    }else{
        const unsigned second_bad=oi_has("alternating")?(bad==au_ch?(au_ch+1u)%4u:au_ch):bad;
        if(oi_has("alternating")){encoder_fault_mask=1u<<second_bad;encoder_fault_remaining=1u;}
        if(oi_has("odometry"))encoder_forced_raw=(raw_before+64u)&4095u;
        delay(1);repair_step();
        CHECK(encoder_angle_reads==reads+2u);
        if(oi_has("persistent")){
            CHECK(full_load_fault()==FullLoadFault::inputs&&full_stopped());
            CHECK(as5600_invalid==2u);
            if(oi_has("alternating")){
                CHECK(MC_AS5600.online[bad]&&g_as5600_fail[bad]==0u);
                CHECK(!MC_AS5600.online[second_bad]&&g_as5600_fail[second_bad]==1u);
            }else CHECK(g_as5600_fail[bad]==2u);
            CHECK(AS5600_is_good(second_bad));
            CHECK(flight::report.input_detail==uint16_t(10u|(second_bad<<8)));
            encoder_fault_remaining=0;delay(1);Motion_control_run(0);
            CHECK(as5600_invalid==0u&&MC_AS5600.online[second_bad]);
            repair_request(short_packet(0x20));au_bf(au_cmd);
            CHECK(full_load_fault()==FullLoadFault::inputs&&full_stopped());
            CHECK((flight::report.flags&(3u<<21))==(1u<<21));
        }else{
            CHECK(full_load_fault()==FullLoadFault::none&&MC_AS5600.online[bad]&&g_as5600_fail[bad]==0u);
            CHECK(!full_stopped()&&encoder_count(bad)==before+(oi_has("odometry")?64u:0u));
            CHECK((flight::report.flags&(3u<<21))==(3u<<21));
        }
    }
    if(oi_has("complete")&&!flight::finished){
        Motion_control_lite_cancel();
        CHECK(flight::finish_native(0)&&flight::complete(host_log));
    }
    if(flight::finished){
        const auto flags=flight::report.flags;
        flight::native_started(3u<<21);
        CHECK(flight::report.flags==flags);
    }
    CHECK(host_programs==programs&&host_erases==erases);
    return oi_export();
#endif
}
