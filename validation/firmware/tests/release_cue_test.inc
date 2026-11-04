#if BMCU_USB_FLIGHT_RECORDER
static void release_control_sample(){
    if(HOST_EXPECT_RELEASE_CUE&&release_closed&&host_clock>=release_closed+500000&&setup_trial_phase()==SetupPhase::gesture){
        for(int ch=-1;ch<4;++ch){const auto* led=ch<0?&SYS_RGB:&RGBOUT[ch];CHECK(leds[led][0]==0x1800u);if(ch>=0)CHECK(leds[led][1]==0x1800u);bool found=false;
            for(auto it=deliveries.rbegin();it!=deliveries.rend();++it)if(it->led==led){CHECK(it->rgb[0]==0x1800u);if(ch>=0)CHECK(it->rgb[1]==0x1800u);found=true;break;}CHECK(found);}
        release_final_stable_checked=true;
    }
    // Observe actual states at raw-input publication; exclude LED state/build ID.
    auto mix=[](uint64_t v){for(unsigned n=0;n<8;++n){release_control_digest^=(v>>(8*n))&255u;release_control_digest*=1099511628211ull;}};
    mix(host_clock);mix(unsigned(phase));mix(unsigned(fault));mix(gesture_ack_channel);mix(gesture_ack_start);
    mix(host_programs);mix(host_erases);mix(bus_started);mix(cal_start);mix(direction_start);
    for(unsigned ch=0;ch<4;++ch){mix(drives[ch]);mix(prompts[ch][0]);mix(prompts[ch][1]);}
    for(auto& timer:compare_values)for(auto v:timer)mix(unsigned(v));
    const auto* p=reinterpret_cast<const uint8_t*>(&flight::report);for(unsigned n=0;n<sizeof(flight::report);++n)mix(p[n]);
}
static int release_led_id(const WS2812_class* led){if(led==&SYS_RGB)return 0;for(int ch=0;ch<4;++ch)if(led==&RGBOUT[ch])return ch+1;CHECK(false);return -1;}
static void release_trace(const char* kind,const std::vector<LedEvent>& events){
    FILE* f=std::fopen((std::string(std::getenv("FLIGHT_TRACE_DIR"))+"/"+scenario+"-"+kind+".csv").c_str(),"wx");CHECK(f);
    std::fprintf(f,"tick,led,phase,rgb0,rgb1\n");std::map<int,std::pair<std::array<uint32_t,2>,SetupPhase>> last;
    for(const auto& e:events){const int id=release_led_id(e.led);const auto v=std::make_pair(e.rgb,e.phase);
        if(!last.count(id)||last[id]!=v)std::fprintf(f,"%llu,%d,%u,%06x,%06x\n",(unsigned long long)e.tick,id,unsigned(e.phase),e.rgb[0],e.rgb[1]);last[id]=v;}
    CHECK(!std::fclose(f));
}
static int release_verify(){
    CHECK(detail_schedule);const auto expected=*detail_schedule;
    CHECK(unsigned(fault)==expected.parent&&boot_admission_detail==expected.detail);
    CHECK(!bus_started&&gesture_ack_channel==255&&gesture_ack_start==0);
    CHECK(release_centers==4&&release_window&&release_samples>500&&max_gesture_poll_gap<3000);
    CHECK(bool(queued_green)==expected.ack);
    CHECK(!expected.ack||accepted_tick>=uint64_t(gesture_start+3000)*1000);
    if(expected.ack)CHECK(flight::report.hold_ms>=1000);
    else CHECK(flight::report.hold_ms<1000);
    if(is("release-near-hold"))CHECK(accepted_tick>=uint64_t(gesture_start+5000)*1000);
    if(is("release-retry"))CHECK(accepted_tick>=uint64_t(gesture_start+4000)*1000);
    if(is("release-hold-reset"))CHECK(accepted_tick>=uint64_t(gesture_start+4700)*1000);
    const bool queued=std::string(expected.fault)=="queued";
    std::array<unsigned,5> greens{};std::array<uint32_t,5> previous{};
    std::array<uint64_t,5> first{};
    bool admission=false,blue=false,final_stable_green=false;
    for(const auto& e:deliveries){
        const int id=release_led_id(e.led);if(id&&e.phase==SetupPhase::gesture)CHECK(e.rgb[0]==e.rgb[1]);
        if((e.phase==SetupPhase::inspect||e.phase==SetupPhase::gesture)&&e.rgb[0]==0x180018u&&e.tick<release_window)admission=true;
        if(e.phase==SetupPhase::gesture){
            if(e.rgb[0]==0x18u&&e.tick>=release_window&&(!accepted_tick||e.tick<accepted_tick))blue=true;
            if(e.rgb[0]==0x1800u){if(previous[id]!=0x1800u){++greens[id];if(!first[id])first[id]=e.tick;}}
            if(HOST_EXPECT_RELEASE_CUE&&first[id]&&(!fault_tick||e.tick<fault_tick)){
                // A queued clear immediately before CAL need not transmit black.
                CHECK(e.rgb[0]==0x1800u||e.rgb[0]==0u);
                if(e.rgb[0]==0u)CHECK(release_closed&&e.tick>=release_closed+1000000);
            }
            if(HOST_EXPECT_RELEASE_CUE&&release_closed&&e.tick>=release_closed+500000&&e.rgb[0]==0x1800u)final_stable_green=true;
            previous[id]=e.rgb[0];
        }
        if(e.phase==SetupPhase::calibration&&id)CHECK(e.rgb[0]!=0x1800u);
        if(fault_tick&&e.tick>=fault_tick)CHECK(e.rgb[0]==setup_trial_led_rgb(e.tick));
    }
    CHECK(admission&&blue);
    if(HOST_EXPECT_RELEASE_CUE){
        for(unsigned id=0;id<5;++id)CHECK(greens[id]==(expected.ack&&!queued?1u:0u));
        if(expected.ack&&!queued)for(auto tick:first)CHECK(tick>=accepted_tick&&tick-accepted_tick<3000);
    }else{
        CHECK(!greens[0]);for(unsigned id=1;id<5;++id)CHECK(greens[id]==(expected.ack&&!queued&&id==expected.channel+1?2u:0u));
    }
    if(!expected.parent){
        CHECK(phase==SetupPhase::success&&release_cal&&cal_start&&release_closed);
        CHECK(release_cal>=release_closed+1000000&&cal_start>=release_cal/1000);
        CHECK(host_programs==60&&host_erases==2&&cal_retained()&&Motion_control_setup_retained());
        uint8_t owner=0;CHECK(Flash_AMS_state_read(&owner)&&owner==255);
        for(unsigned ch=0;ch<4;++ch)CHECK(drives[ch]&&prompts[ch][0]>=release_cal/1000&&prompts[ch][1]>prompts[ch][0]);
        CHECK(!HOST_EXPECT_RELEASE_CUE||release_final_stable_checked);
        if(is("release-neutral-bounce"))CHECK(cal_start>=gesture_start+7700);
        if(is("release-near-release"))CHECK(cal_start>=gesture_start+7900);
        CHECK(flight::report.candidate==expected.channel&&flight::report.progress==7&&flight::report.release_ms>=1200);
    }else{
        CHECK(phase==SetupPhase::fault&&!release_cal&&!cal_start&&!host_programs&&!host_erases);
        for(auto d:drives)CHECK(!d);zeros();
        CHECK(fault_tick&&gesture_ack_channel==255);
    }
    if(is("release-long-held")||is("release-held-timeout"))CHECK(!HOST_EXPECT_RELEASE_CUE||release_expiry_checked);
    const auto report=flight::report;const bool complete=flight::complete(host_log);
    if(complete){flight::Report saved{};std::memcpy(&saved,host_log+32,sizeof(saved));CHECK(!std::memcmp(&saved,&report,sizeof(saved)));}
    flight_save((scenario+".live-report388").c_str(),&report,sizeof(report));
    flight::export_frame();flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));
    release_trace("queued",release_queue);release_trace("delivered",deliveries);
    // Actual main export after losing every recorder RAM field. No bus/ADC/PWM or
    // NVM work is admitted; complete report retains candidate/progress/fault bytes.
    std::array<uint8_t,512> saved{};std::memcpy(saved.data(),host_log,512);
    std::array<uint8_t,4096> nvm{};std::memcpy(nvm.data(),host_nvm,4096);
    const auto nvm_crc=flight::crc(host_nvm,4096),report_crc=flight::crc(&report,sizeof(report));
    FILE* result=std::fopen((std::string(std::getenv("FLIGHT_TRACE_DIR"))+"/"+scenario+".json").c_str(),"wx");CHECK(result);
    std::fprintf(result,"{\"phase\":%u,\"fault\":%u,\"detail\":%u,\"candidate\":%u,\"progress\":%u,\"accepted_tick\":%llu,\"closed_tick\":%llu,\"cal_tick\":%llu,\"gesture_start_ms\":%llu,\"cal_start_ms\":%llu,\"direction_start_ms\":%llu,\"programs\":%u,\"erases\":%u,\"complete\":%s,\"nvm_crc\":%u,\"report_crc\":%u,\"control_digest\":\"%016llx\",\"expiry_checked\":%s}\n",unsigned(phase),unsigned(fault),boot_admission_detail,report.candidate,report.progress,(unsigned long long)accepted_tick,(unsigned long long)release_closed,(unsigned long long)release_cal,(unsigned long long)gesture_start,(unsigned long long)cal_start,(unsigned long long)direction_start,host_programs,host_erases,complete?"true":"false",nvm_crc,report_crc,(unsigned long long)release_control_digest,release_expiry_checked?"true":"false");CHECK(!std::fclose(result));
    flight_reset();bus_started=false;g_adc_dma_inited=false;host_dma1.CFGR=0;
    host_time_hook=[](){++host_clock;if(log_tx.size()>=536)throw LoopComplete{};};
    try{firmware_main();CHECK(false);}catch(const LoopComplete&){}
    CHECK(!bus_started&&!g_adc_dma_inited&&!host_dma1.CFGR&&!std::memcmp(saved.data(),host_log,512)&&!std::memcmp(nvm.data(),host_nvm,4096));
    CHECK(log_tx.size()==536&&!std::memcmp(log_tx.data()+20,saved.data(),512));
    for(auto& timer:compare_values)for(auto v:timer)CHECK(!v);
    std::printf("RELEASE PASS %s desired=%u candidate=%u fault=%u complete=%u steady-expiry=%u control=%016llx\n",scenario.c_str(),HOST_EXPECT_RELEASE_CUE,report.candidate,report.fault,complete,release_expiry_checked,(unsigned long long)release_control_digest);
    return 0;
}
#endif
