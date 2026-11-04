// New desired behavior against actual production bodies; no OEM/printer oracle.
static void lease_stopped(){
    CHECK(full_stopped()&&!forward_grant&&ams[0].now_filament_num==255&&ams[0].filament_use_flag==0);
    for(auto& f:ams[0].filament)CHECK(f.motion==_filament_motion::idle);
    for(unsigned t=0;t<3;++t)for(unsigned c=0;c<(t==2?4u:2u);c+=2){CHECK(compare_values[t][c]==compare_values[t][c+1]);CHECK(compare_values[t][c]==0||compare_values[t][c]==1000);}
}
static void lease_expire(bool queued=false){
    repair_request(short_packet(0x20));
    if(queued){
        while(time_diff32(bambubus_heartbeat_deadline,time_ticks32())>int32_t(ms_to_ticks32(1u))){delay(1);repair_step();}
        const auto arrival=time_ticks32(),deadline=bambubus_heartbeat_deadline;
        append(short_packet(0x20));bus_rx_poll();CHECK(bus_port_to_host.recv_data_len);
        delay(2);repair_step();CHECK(time_diff32(arrival,deadline)<=0&&time_diff32(time_ticks32(),deadline)>0);
        std::printf("QUEUED arrival=%u deadline=%u dispatch=%u\n",arrival,deadline,time_ticks32());
    }else for(unsigned n=1;n<=1200;++n){delay(1);if(n%20==0)au_request(255,7,0,au_cmd);else repair_step();}
    lease_stopped();CHECK(full_load_durable(au_ch)&&full_load_owner()==au_ch);
}
static void lease_reply(unsigned cmd,unsigned request_ch,unsigned state,unsigned motion,bool loaded,bool global){
    const auto before=ams[0];const auto owner_before=full_load_owner(),durable_before=durable_owner;
    const auto np=host_programs,lp=log_programs;const auto count=authority_observation::count;
    au_request(request_ch,state,motion,cmd);
    const unsigned off=cmd==3?7:19;
    CHECK(full_reply.size()==(cmd==3?44u:60u)&&package_check_crc16(full_reply.data(),full_reply.size()));
    CHECK(full_reply[off]==(loaded?4u:0u));
    CHECK(full_reply[off+1]==(loaded&&!global?au_ch:255u));
    if(global||!loaded)CHECK(!std::memcmp(full_reply.data()+off+2,"\0\0\0\0",4));
    else{const float expected=1.f-Motion_control_filament_meters(au_ch);CHECK(!std::memcmp(full_reply.data()+off+2,&expected,4));}
    if(cmd==3)CHECK(full_reply[offsetof(bambubus_ams_motion_package_struct,filament_channel_2)]==(loaded&&!global?au_ch:255u));
    else CHECK(full_reply[offsetof(bambubus_ams_stu_motion_package_struct,filament_channel_stu)]==request_ch);
    CHECK(ams[0].now_filament_num==before.now_filament_num&&ams[0].filament_use_flag==before.filament_use_flag);
    CHECK(full_load_owner()==owner_before&&durable_owner==durable_before&&host_programs==np&&log_programs==lp);
    CHECK(authority_observation::count==count&&!flight::finished&&!flight::native_checkpoint_pending());
    lease_stopped();
}
static int lease_verify(){
#if BMCU_LITE_LEASE_FEEDBACK
    if(oi_has("lease-meter")){
        au_start();freeze_encoder=true;
        if(oi_has("no-observation")||oi_has("invalid-sample")){
            const auto packet=full_motion(au_ch,7,0,au_cmd);full_rx(packet);
            CHECK((flight::diagnostic().reserved&3u)==1u);
            if(oi_has("invalid-sample")){
                const auto mask=full_encoder_valid;full_encoder_valid&=uint8_t(~(1u<<au_ch));
                delay(1);AS5600_distance_updata(time_ticks32());CHECK((flight::diagnostic().reserved&3u)==1u);
                full_encoder_valid=mask;delay(1);AS5600_distance_updata(time_ticks32());CHECK((flight::diagnostic().reserved&3u)==3u);
            }
        }else{
            au_request(au_ch,7,0,au_cmd);au_tick(4);
            auto& d=flight::diagnostic();CHECK((d.reserved&3u)==3u);
            CHECK(d.onuse_meters[0]==d.onuse_meters[1]&&d.onuse_meters[2]==0.f);
            if(oi_has("moving")){
                MC_AS5600.raw_angle[au_ch]=(MC_AS5600.raw_angle[au_ch]+128)&4095;au_tick(2);
                CHECK(d.onuse_meters[1]!=d.onuse_meters[0]&&d.onuse_meters[2]>0.f);
                const auto peak=d.onuse_meters[2];
                MC_AS5600.raw_angle[au_ch]=(MC_AS5600.raw_angle[au_ch]-128)&4095;au_tick(2);
                CHECK(d.onuse_meters[1]==d.onuse_meters[0]&&d.onuse_meters[2]==peak);
            }
        }
        const auto before=flight::diagnostic();flight::onuse_meter(uint8_t((au_ch+1)%4),false);
        CHECK(!std::memcmp(&before,&flight::diagnostic(),sizeof(before)));
        flight::operator_request(uint8_t((au_ch+1)%4));repair_step();
        CHECK(flight::saved()&&flight::complete(host_log)&&flight::diagnostic().reserved==before.reserved);
        CHECK(!std::memcmp(flight::diagnostic().onuse_meters,before.onuse_meters,sizeof(before.onuse_meters)));
        return oi_export();
    }
    if(oi_has("lease-pending")){
        full_keys[au_ch]=1.8f;au_tick(10);au_bf(au_cmd);CHECK(waiting_send&&full_load_journal_pending());
        bambubus_heartbeat_deadline=time_ticks32()-1u;repair_step();CHECK(lite_loaded_feedback==255u);lease_stopped();
        repair_request(short_packet(0x20));
        for(unsigned cmd:{3u,4u}){au_request(255,7,0,cmd);CHECK(full_reply[cmd==3?7:19]==0u);lease_stopped();}
        CHECK(lite_loaded_feedback==255u);return au_done();
    }
#endif

    if(oi_has("lease-forward")){
        au_start();lease_expire();for(unsigned cmd:{3u,4u})lease_reply(cmd,255,7,0,false,true);
#if BMCU_LITE_LEASE_FEEDBACK
        CHECK(!(flight::diagnostic().reserved&1u));
        flight::operator_request(uint8_t((au_ch+1)%4));repair_step();CHECK(flight::saved()&&!(flight::diagnostic().reserved&1u));return oi_export();
#else
        return au_done();
#endif
    }
    if(oi_has("lease-boot")){
        CHECK(full_load_durable(2)&&ams[0].now_filament_num==255);au_ch=2;
        // Retained physical ownership cannot arm the new state, even if RAM is
        // made to resemble on_use without an admitted addressed request.
        ams[0].now_filament_num=2;ams[0].filament[2].motion=_filament_motion::on_use;ams[0].filament_use_flag=4;
        lease_expire();for(unsigned cmd:{3u,4u})lease_reply(cmd,255,7,0,false,true);return au_done();
    }
    oi_start();const auto np=host_programs,lp=log_programs;std::array<uint8_t,4096> nvm{};std::memcpy(nvm.data(),host_nvm,4096);
    if(oi_has("lease-terminal")){
        // Valid unissued ownership work must be discarded, never completed just
        // to make the job idle; one issued operation may finish and be verified.
        g_flash_job={};g_flash_job.kind=2;g_flash_job.addr=0x0800f610u;g_flash_job.halfwords=4;g_flash_job.channel=uint8_t(au_ch);g_flash_job.slot=2;
        g_flash_job.words[0]=0x1234abcd;g_flash_job.words[1]=0x5678efab;
        if(oi_has("terminal-issued")){
            const unsigned index=flash_job_halfword(g_flash_job);
            host_store16(g_flash_job.addr+index*2u,uint16_t(g_flash_job.words[index>>1]>>((index&1u)*16u)));
            g_flash_job.phase=2;g_flash_job.started=time_ticks32();time_hw_slow=1;FLASH->STATR|=FLASH_FLAG_BSY;
        }
        const auto issued_programs=host_programs;std::memcpy(nvm.data(),host_nvm,4096);
        static unsigned waits;waits=0;
        static bool permanent,failed;permanent=oi_has("permanent");failed=oi_has("verify-fail");
        if(failed)host_flash_ptr(g_flash_job.addr+flash_job_halfword(g_flash_job)*2u)[0]^=1u;
        std::memcpy(nvm.data(),host_nvm,4096);
        host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY){++host_clock;if(!bus_started){CHECK(full_stopped()&&!flight::saved());if(++waits==20&&!permanent)FLASH->STATR&=~FLASH_FLAG_BSY;}}raw_dma_boundary();};
        full_load_fail(FullLoadFault::inputs);CHECK(full_stopped()&&!lite_motion_ready());
        // Call actual step: the pre-terminal background call must not issue work.
        repair_step();CHECK(host_terminal_seen&&!bus_started&&full_stopped()&&host_programs==issued_programs);
        CHECK(full_load_owner()==au_ch&&durable_owner==au_ch&&!std::memcmp(nvm.data(),host_nvm,4096));
        if(permanent||failed){CHECK(!flight::saved()&&!flight::complete(host_log)&&!diagnostic_green_deliveries);if(permanent)CHECK((FLASH->STATR&FLASH_FLAG_BSY)&&time_hw_slow);}
        else{CHECK(flight::saved()&&flight::complete(host_log)&&flight::report.native_fault==uint8_t(FullLoadFault::inputs)&&diagnostic_green_deliveries&&Flash_background_idle()&&!time_hw_slow);}
        std::printf("TERMINAL waits=%u programs=%u saved=%u native_fault=%u\n",waits,host_programs,unsigned(flight::saved()),flight::report.native_fault);
        return oi_export();
    }
    const bool healthy=oi_has("lease-healthy");
    if(healthy){
        repair_request(short_packet(0x20));
        for(unsigned n=1;n<=1200;++n){delay(1);if(n%400==0)repair_request(short_packet(0x20));if(n%20==0)au_request(255,7,0,au_cmd);else repair_step();}
        CHECK(ams[0].now_filament_num==au_ch&&ams[0].filament[au_ch].motion==_filament_motion::on_use&&ams[0].filament_use_flag==4);
        for(unsigned cmd:{3u,4u}){au_request(255,7,0,cmd);unsigned off=cmd==3?7:19;CHECK(full_reply[off]==4&&full_reply[off+1]==255);}
        CHECK(host_programs==np&&log_programs==lp&&!std::memcmp(nvm.data(),host_nvm,4096));return au_done();
    }
    lease_expire(oi_has("queued"));
    CHECK(host_programs==np&&log_programs==lp&&!std::memcmp(nvm.data(),host_nvm,4096)&&!flight::finished);
    repair_request(short_packet(0x20));lease_stopped();CHECK(lite_lease_active());
    if(oi_has("invalidate")){
#if BMCU_LITE_LEASE_FEEDBACK
        CHECK(lite_loaded_feedback==au_ch+4);
#endif
        if(oi_has("direct-cancel"))Motion_control_lite_cancel();
        else if(oi_has("genuine-cancel"))au_request(255,1,1,au_cmd);
        else if(oi_has("new-work")){au_request(au_ch,7,0,au_cmd);CHECK(ams[0].now_filament_num==au_ch);Motion_control_lite_cancel();ams[0].now_filament_num=255;ams[0].filament_use_flag=0;for(auto&f:ams[0].filament)f.motion=_filament_motion::idle;}
        else if(oi_has("nondurable"))durable_owner=255;
        else if(oi_has("metadata"))metadata_recovery=true;
        else if(oi_has("storage"))g_flash_fault=true;
        else if(oi_has("calibration"))g_valid_mask&=~(1u<<au_ch);
        else if(oi_has("absent")){full_keys[au_ch]=.2f;for(unsigned n=0;n<64;++n){delay(1);ADC_DMA_poll();}}
        else CHECK(false);
        // Serialize directly so a deliberately invalid storage/metadata gate
        // cannot be serviced/changed by the main background loop before observation.
        const auto packet=full_motion(255,7,0,au_cmd);bus_port_to_host.send_data_len=0;
        if(au_cmd==3)get_package_motion((bambubus_printer_motion_package_struct*)packet.data(),packet.size());
        else get_package_stu_motion((bambubus_printer_stu_motion_package_struct*)packet.data(),packet.size());
        auto* out=bus_port_to_host.tx_build_buf();CHECK(out[au_cmd==3?7:19]==0);
#if BMCU_LITE_LEASE_FEEDBACK
        CHECK(lite_loaded_feedback==255);
#endif
        durable_owner=uint8_t(au_ch);metadata_recovery=false;g_flash_fault=false;g_valid_mask|=1u<<au_ch;full_keys[au_ch]=1.8f;
        bus_port_to_host.send_data_len=0;for(unsigned n=0;n<64;++n){delay(1);ADC_DMA_poll();}
        for(unsigned cmd:{3u,4u})lease_reply(cmd,255,7,0,false,true);return au_done();
    }
    for(unsigned cmd:{3u,4u}){
        for(unsigned motion:{0u,0x26u,0x66u,0x99u})lease_reply(cmd,255,7,motion,true,true);
        lease_reply(cmd,255,1,0,true,true);
        for(unsigned slot=0;slot<4;++slot)lease_reply(cmd,slot,7,0x99,true,false);
        // Refused cross-owner pull preserves the feedback bookmark and control.
        lease_reply(cmd,(au_ch+1)%4,3,0x3f,true,false);
    }
    CHECK(host_programs==np&&log_programs==lp&&!std::memcmp(nvm.data(),host_nvm,4096));
    au_request(au_ch,3,0x3f,au_cmd);au_tick(5);CHECK(lite_pull_state[au_ch]==LitePullState::active&&flight::diagnostic().admitted_channel==au_ch&&(flight::diagnostic().seen_flags&6u)==6u);
    CHECK(!forward_grant);return au_done();
}
