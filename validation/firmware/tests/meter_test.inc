#if BMCU_LITE_V11_FEED_LATCH && BMCU_USB_FLIGHT_RECORDER && BMCU_LITE_NATIVE_PHASE_COMPAT
static uint32_t meter_bits(float f){uint32_t u;std::memcpy(&u,&f,4);return u;}
static float meter_wire(const Packet& p,unsigned cmd){float f;std::memcpy(&f,p.data()+(cmd==3?9:21),4);return f;}
static void meter_distance(unsigned ch,int64_t count,float wire){
    CHECK(as5600_odometer_count[ch]==count&&encoder_count(ch)==uint32_t(count));
    CHECK(meter_bits(Motion_control_filament_meters(ch))==meter_bits(baseline_meter_value(ch)));
    CHECK(std::isfinite(wire));
#if BMCU_LITE_METER_ORIGIN_ZERO
    // Independent geometry oracle, deliberately not the rounded cache formula.
    // f32 conversion, multiplication and cancellation permit this bounded error.
    const double travel=double(count)*(3.14159265358979323846*7.5/(4096.*1000.));
    CHECK(std::fabs(double(wire)-travel)<=1.e-7+std::fabs(travel)*3.e-7);
    if(!count)CHECK(meter_bits(wire)==0u); // 1.f - 1.f is positive zero.
    if(count==1)CHECK(wire>0.f&&wire<.000006f);
    if(count==-1)CHECK(wire<0.f&&wire>-.000006f);
#else
    if(!count)CHECK(meter_bits(wire)==0xbf800000u);
#endif
}
static int meter_responses(){
    full_heartbeat();host_time_hook=nullptr;host_delay_hook=nullptr;
    // Include owner-report equivalent counts, signed small deltas, 32-bit
    // wrap neighborhoods, and finite int64 extremes. These are synthetic counts.
    const int64_t counts[]={0,1,-1,1000,-1000,4096,-4096,-34910,-34936,-34811,65535,-65535,
        2147483645ll,2147483650ll,4294967290ll,4294967301ll,-4294967290ll,-4294967301ll,
        1099511627776ll,-1099511627776ll,1152921504606846976ll,-1152921504606846976ll,
        INT64_MAX,INT64_MIN};
    unsigned comparisons=0,valid=0,invalid=0;
    for(int64_t count:counts)for(unsigned selected:{au_ch,4u,254u,255u})for(unsigned flag:{0u,2u,4u}){
        for(unsigned ch=0;ch<4;++ch){as5600_odometer_count[ch]=count;ams[0].filament[ch].meters=123.f+float(ch);}
        ams[0].now_filament_num=uint8_t(selected);ams[0].filament_use_flag=uint8_t(flag);
        const auto before=ams[0];const auto packet=full_motion(au_ch,7,0x66,au_cmd);
        const auto programs=host_programs,erases=host_erases;const auto scale_before=time_hw_tpms;
        package_num=3;bus_port_to_host.send_data_len=0;
        if(au_cmd==3)baseline_get_package_motion((bambubus_printer_motion_package_struct*)packet.data(),packet.size());
        else baseline_get_package_stu_motion((bambubus_printer_stu_motion_package_struct*)packet.data(),packet.size());
        const unsigned n=bus_port_to_host.send_data_len;CHECK(n==(au_cmd==3?44u:60u));
        Packet expected(bus_port_to_host.tx_build_buf(),bus_port_to_host.tx_build_buf()+n);const auto baseline_after=ams[0];
        const float old_wire=meter_wire(expected,au_cmd);
#if BMCU_LITE_METER_ORIGIN_ZERO
        if(selected<4u){const float corrected=1.0f+old_wire;std::memcpy(expected.data()+(au_cmd==3?9:21),&corrected,4);package_add_crc(expected.data(),expected.size());}
#endif
        ams[0]=before;package_num=3;bus_port_to_host.send_data_len=0;
        if(au_cmd==3)get_package_motion((bambubus_printer_motion_package_struct*)packet.data(),packet.size());
        else get_package_stu_motion((bambubus_printer_stu_motion_package_struct*)packet.data(),packet.size());
        Packet actual(bus_port_to_host.tx_build_buf(),bus_port_to_host.tx_build_buf()+bus_port_to_host.send_data_len);
        CHECK(actual==expected&&package_check_crc16(actual.data(),actual.size()));
        CHECK(!std::memcmp(&ams[0],&baseline_after,sizeof(ams[0])));
        CHECK(host_programs==programs&&host_erases==erases&&time_hw_tpms==scale_before&&full_load_fault()==FullLoadFault::none);
        for(unsigned ch=0;ch<4;++ch)CHECK(as5600_odometer_count[ch]==count&&meter_bits(Motion_control_filament_meters(ch))==meter_bits(baseline_meter_value(ch)));
        if(selected<4u){meter_distance(selected,count,meter_wire(actual,au_cmd));++valid;}else ++invalid;
        ++comparisons;
    }
    CHECK(comparisons==288&&valid==72&&invalid==216);
    std::printf("METER responses=%u valid=%u invalid=%u sender=%u channel=%u cache/controller unchanged; baseline full frame differs only expected meter and CRC\n",comparisons,valid,invalid,au_cmd,au_ch);
    bus_port_to_host.send_data_len=0;return 0;
}
static int meter_sequence(){
    const auto path=std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".csv";
    au_trace=std::fopen(path.c_str(),"wx");CHECK(au_trace);
    host_clock=((host_clock+999)/1000)*1000;host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto &p:full_pulls)p=float(2048)*(3.3f/4095.f);
    repair_request(short_packet(0x20));au_tick(20);au_start();latch_live();freeze_encoder=true;
    const int64_t count=-34910;as5600_odometer_count[au_ch]=count;
    const auto writes=host_programs;const auto first=Motion_control_filament_meters(au_ch);
    for(unsigned cmd:{3u,4u}){
        for(unsigned i=0;i<3;++i){au_request(au_ch,3,0xbf,cmd);latch_live();meter_distance(au_ch,count,meter_wire(full_reply,cmd));
            CHECK(flight::report.native_tx[0]==3&&flight::report.bf_length==full_reply.size());
            CHECK(!std::memcmp(flight::report.native_bf,full_reply.data(),full_reply.size()));}
        au_request(255,1,0,cmd);latch_live();meter_distance(au_ch,count,meter_wire(full_reply,cmd));
        CHECK(flight::report.native_tx[1]==3&&flight::report.idle_command==cmd&&flight::report.idle_length==full_reply.size());
        CHECK(!std::memcmp(flight::report.native_idle,full_reply.data()+(cmd==3?7:19),8));
        au_request(au_ch,7,0,cmd);CHECK(!forward_grant&&ams[0].filament[au_ch].motion==_filament_motion::on_use);
        meter_distance(au_ch,count,meter_wire(full_reply,cmd));
        au_request(au_ch,7,0,cmd);meter_distance(au_ch,count,meter_wire(full_reply,cmd));
        CHECK(meter_bits(Motion_control_filament_meters(au_ch))==meter_bits(first)&&host_programs==writes);
    }
    // Production AS5600 update performs signed raw-angle wrap without rebasing
    // the cumulative counter, including crossing the uint32 consumer boundary.
    const int64_t wrap_start=4294967290ll;as5600_odometer_count[au_ch]=wrap_start;
    as5600_distance_save[au_ch]=4090;MC_AS5600.raw_angle[au_ch]=5;
    host_clock+=1000;AS5600_distance_updata(uint32_t(host_clock));CHECK(as5600_odometer_count[au_ch]==wrap_start+11&&encoder_count(au_ch)==5);
    CHECK(count_distance(encoder_count(au_ch),uint32_t(wrap_start))==11);
    au_request(au_ch,7,0x66,au_cmd);meter_distance(au_ch,wrap_start+11,meter_wire(full_reply,au_cmd));
    MC_AS5600.raw_angle[au_ch]=4090;host_clock+=1000;AS5600_distance_updata(uint32_t(host_clock));CHECK(as5600_odometer_count[au_ch]==wrap_start);
    au_request(au_ch,7,0x66,au_cmd);meter_distance(au_ch,wrap_start,meter_wire(full_reply,au_cmd));
    std::printf("METER sequence sender=%u channel=%u counter=%lld controller_bits=%08x owner=%u durable=%u writes=%u fault=%u\n",au_cmd,au_ch,(long long)as5600_odometer_count[au_ch],meter_bits(Motion_control_filament_meters(au_ch)),owner,durable_owner,host_programs,unsigned(full_load_fault()));
    // Retain the current unsupported-Cancel policy; no compatibility repair here.
    au_request(255,0,0,3);CHECK(full_load_fault()==FullLoadFault::unsupported&&full_stopped());
    phase_persist_export();return au_done();
}
static int meter_verify(){
    au_ch=scenario.find("ch0")!=std::string::npos?0u:scenario.find("ch1")!=std::string::npos?1u:scenario.find("ch3")!=std::string::npos?3u:2u;
    au_cmd=scenario.find("sender04")!=std::string::npos?4u:3u;
    if(scenario.find("response")!=std::string::npos)return meter_responses();
    return meter_sequence();
}
#endif
