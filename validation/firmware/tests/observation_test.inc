#if BMCU_LITE_V11_FEATURES
static FILE* ob_trace=nullptr;static FILE* ob_optical=nullptr;
static bool ob_media=false;static uint64_t ob_tail_digest=1469598103934665603ull,ob_tail_events=0;
static uint64_t ob_epoch=0,ob_last=0;static unsigned ob_ms=0,ob_ch=2,ob_command=3;
static uint32_t ob_last_rgb=0xffffffffu;
static void ob_event(const char* kind,unsigned a=0,int b=0){
    if(!ob_trace)return;
    char row[2048];unsigned used=unsigned(std::snprintf(row,sizeof(row),"%s,%u,%d,%u,%u,%u,%u,%u,%u",kind,a,b,full_load_owner(),durable_owner,unsigned(intent),ams[0].now_filament_num,unsigned(full_load_fault()),host_programs));
    for(unsigned ch=0;ch<4;++ch)used+=unsigned(std::snprintf(row+used,sizeof(row)-used,",%u,%u,%u,%u",unsigned(MOTOR_CONTROL[ch].motion),encoder_count(ch),unsigned(dm_auto_state[ch]),dm_loaded[ch]));
    for(const auto &timer:compare_values)for(int v:timer)used+=unsigned(std::snprintf(row+used,sizeof(row)-used,",%d",v));
    if(kind[0]=='R')for(auto v:full_reply)used+=unsigned(std::snprintf(row+used,sizeof(row)-used,",%02x",v));
    CHECK(used+1<sizeof(row));row[used++]='\n';
    if(ob_media){for(unsigned i=0;i<used;++i){ob_tail_digest^=uint8_t(row[i]);ob_tail_digest*=1099511628211ull;}++ob_tail_events;}
    else CHECK(std::fwrite(row,1,used,ob_trace)==used);
}
static void observation_pwm(unsigned ch,int pwm){ob_event("PWM",ch,pwm);}
static void observation_ws(const WS2812_class* led,const std::array<uint32_t,2>& rgb){
    if(ob_optical&&led==&RGBOUT[0]&&rgb[0]!=ob_last_rgb){
        ob_last_rgb=rgb[0];std::fprintf(ob_optical,"%.6f,%u,%u,%u\n",double(host_clock-ob_epoch)/1000000.,unsigned(rgb[0]>>16),unsigned((rgb[0]>>8)&255),unsigned(rgb[0]&255));
    }
}
static void ob_request(const Packet& p){repair_request(p);ob_event("REPLY",p[4],p.size());}
static void ob_bf(unsigned cmd=3,unsigned ch=99){ob_last=host_clock;ob_request(full_motion(ch==99?ob_ch:ch,3,0xbf,cmd));}
static void ob_tick(unsigned ms=1,bool renew=false){
    for(unsigned i=0;i<ms;++i){delay(1);++ob_ms;if(ob_ms%400==0)ob_request(short_packet(0x20));if(renew&&ob_ms%40==0)ob_bf(ob_ms%80?3:4);repair_step();ob_event("TICK");}
}
static void ob_start(){
    full_keys[ob_ch]=1.8f;ob_tick(10);ob_bf(ob_command);
    CHECK(full_stopped()&&full_load_journal_pending()&&!full_load_durable(ob_ch)&&ams[0].now_filament_num==0xff);
    for(unsigned i=0;i<30&&full_load_journal_pending();++i)ob_tick();ob_tick(10);
    CHECK(full_load_durable(ob_ch)&&!full_stopped()&&full_load_fault()==FullLoadFault::none);
}
static void ob_until_age(unsigned age){while(host_clock<ob_last+uint64_t(age)*1000)ob_tick();}
#if BMCU_LITE_AUTHORITY_OBSERVATION
static unsigned ob_count(const uint8_t* r){return unsigned(r[6])|(unsigned(r[7])<<8);}
static void ob_fact(unsigned slot,unsigned reason){CHECK(authority_observation::mask&(1u<<slot));const auto* r=authority_observation::records[slot];CHECK(r[1]==reason&&r[4]==ob_ch);CHECK(slot==0?!(r[0]&2):(r[0]&2));}
static void ob_export(){
    // Actual production publisher/CRC, not a host reimplementation of the frame.
#if BMCU_USB_FLIGHT_RECORDER
    authority_observation::snapshot(authority_observation::frame);
#else
    authority_observation::render();
#endif
    CHECK(authority_observation::frame[3]==80);
#if BMCU_LITE_NATIVE_PHASE_COMPAT
    CHECK(authority_observation::frame[60]==authority_observation::history_count&&authority_observation::frame[61]==0xa1);
    CHECK(!std::memcmp(authority_observation::frame+62,authority_observation::history,8));
#else
    CHECK(authority_observation::frame[60]==authority_observation::post_count&&authority_observation::frame[61]==authority_observation::post_flags);
#endif
    CHECK(package_check_crc16(authority_observation::frame,80));
    const auto path=std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+".bin";FILE*f=std::fopen(path.c_str(),"wb");CHECK(f);CHECK(std::fwrite(authority_observation::frame,1,80,f)==80);CHECK(!std::fclose(f));
}
#endif
static int observation_verify(){
    ob_command=scenario.find("-04")!=std::string::npos?4u:3u;ob_ch=scenario.find("-ch0")!=std::string::npos?0u:2u;
    host_clock=((host_clock+999)/1000)*1000;ob_epoch=host_clock;
    host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    const auto trace=std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+
#if BMCU_LITE_AUTHORITY_OBSERVATION
    "-ON.csv";
#else
    "-OFF.csv";
#endif
    ob_trace=std::fopen(trace.c_str(),"wx");CHECK(ob_trace);
    for(auto &p:full_pulls)p=float(2048)*(3.3f/4095.f);
    ob_request(short_packet(0x20));ob_tick(20);CHECK(v11_local_eligible());
    if(scenario.find("local")!=std::string::npos){full_keys[ob_ch]=1.55f;ob_tick(180);CHECK(dm_auto_state[ob_ch]==DM_AUTO_S1_PUSH&&v11_pwm(ob_ch));full_keys[ob_ch]=1.8f;ob_tick(1200);CHECK(dm_loaded[ob_ch]&&full_load_empty()&&!host_programs);}
    if(scenario.find("pending")!=std::string::npos){
        full_keys[ob_ch]=full_keys[1]=1.8f;ob_tick(10);ob_bf(ob_command);CHECK(waiting_send&&full_stopped());
        if(scenario.find("busy")!=std::string::npos){
            host_hang=true;for(unsigned i=0;i<30&&full_load_fault()==FullLoadFault::none;++i)ob_tick();
            CHECK(full_load_fault()==FullLoadFault::storage&&full_stopped()&&time_hw_slow&&(FLASH->STATR&FLASH_FLAG_BSY));
            const auto writes=host_programs;const auto delivered=deliveries.size();ob_tick(10);ob_bf(4);CHECK(host_programs==writes&&deliveries.size()==delivered&&full_stopped());
            CHECK(leds[&SYS_RGB][0]==0x200000u||leds[&SYS_RGB][0]==0x202000u);
#if BMCU_LITE_AUTHORITY_OBSERVATION
            ob_fact(0,authority_observation::fault);CHECK(authority_observation::records[0][2]==1&&!(authority_observation::records[0][0]&2));
#endif
        }else{
        ob_request(full_motion(0xff,1,0,ob_command));CHECK(!waiting_send&&full_stopped());
#if BMCU_LITE_AUTHORITY_OBSERVATION
        ob_fact(0,authority_observation::idle);std::array<uint8_t,26> pending{};std::memcpy(pending.data(),authority_observation::records[0],26);
        CHECK(!(pending[0]&2)&&pending[11]&1&&pending[12]==ob_command&&pending[13]==0xff&&pending[20]==ob_command&&pending[21]==ob_ch);
#endif
        ob_tick(3);ob_bf(ob_command==3?4:3);ob_request(full_motion(1,3,0xbf,ob_command));
        for(unsigned i=0;i<30&&full_load_journal_pending();++i)ob_tick();ob_tick(10);CHECK(full_load_durable(ob_ch)&&!full_stopped());
        ob_until_age(100);CHECK(unsigned(full_load_fault())==7&&full_stopped());
#if BMCU_LITE_AUTHORITY_OBSERVATION
        CHECK(!std::memcmp(pending.data(),authority_observation::records[0],26));ob_fact(1,authority_observation::authority);
        const auto* r=authority_observation::records[1];CHECK(r[20]==(ob_command==3?4:3)&&r[21]==ob_ch);CHECK(r[12]==ob_command&&r[13]==1);CHECK(r[11]&4);
#endif
        }
    }else{
        ob_start();
        if(scenario.find("renew")!=std::string::npos){
            ob_tick(900,true);ob_request(full_motion(ob_ch,7,0,ob_command));CHECK(full_load_fault()==FullLoadFault::none);
#if BMCU_LITE_AUTHORITY_OBSERVATION
            ob_fact(1,authority_observation::use);std::printf("RENEW OBS count=%u max=%u last=%u age=%u\n",ob_count(authority_observation::records[1]),authority_observation::records[1][9],authority_observation::records[1][10],authority_observation::records[1][8]);std::fflush(stdout);CHECK(ob_count(authority_observation::records[1])==23&&authority_observation::records[1][9]==50&&authority_observation::records[1][10]==40&&authority_observation::records[1][8]==26);
#endif
        }else if(scenario.find("idle04")!=std::string::npos){
            ob_request(full_motion(0xff,1,0,4));
#if BMCU_LITE_NATIVE_IDLE_04
            // Explicit changed policy ONLY in this fresh, durable native forward state.
            CHECK(!full_stopped()&&full_load_fault()==FullLoadFault::none);
#if BMCU_LITE_AUTHORITY_OBSERVATION
            CHECK(!authority_observation::mask);
#endif
            ob_until_age(100);CHECK(full_stopped()&&unsigned(full_load_fault())==7);
#if BMCU_LITE_AUTHORITY_OBSERVATION
            ob_fact(1,authority_observation::authority);CHECK(authority_observation::records[1][12]==4);
#endif
#else
            CHECK(full_stopped()&&full_load_fault()==FullLoadFault::none);
#if BMCU_LITE_AUTHORITY_OBSERVATION
            ob_fact(1,authority_observation::idle);CHECK(authority_observation::records[1][12]==4);
#endif
#endif
        }else if(scenario.find("hold")!=std::string::npos||scenario.find("hard")!=std::string::npos){
            const bool hard=scenario.find("hard")!=std::string::npos;
            if(hard){full_pulls[ob_ch]=float(3400)*(3.3f/4095.f);ob_tick(40,true);CHECK(MOTOR_CONTROL[ob_ch].send_hard&&full_stopped());}
            else{full_pulls[ob_ch]=float(2310)*(3.3f/4095.f);ob_tick(40,true);CHECK(MOTOR_CONTROL[ob_ch].send_stop_latch);full_pulls[ob_ch]=float(2345)*(3.3f/4095.f);ob_tick(40,true);CHECK(full_stopped());}
            CHECK(full_load_fault()==FullLoadFault::none);
#if BMCU_LITE_AUTHORITY_OBSERVATION
            ob_fact(1,hard?authority_observation::hard:authority_observation::hold_zero);CHECK(authority_observation::records[1][24]&(hard?2:1));
#endif
        }else if(scenario.find("fault")!=std::string::npos){
            FullLoadFault expected=FullLoadFault::inputs;
            if(scenario.find("storage")!=std::string::npos){g_flash_fault=true;expected=FullLoadFault::storage;}
            else if(scenario.find("clock")!=std::string::npos){time_hw_tpms=1001;expected=FullLoadFault::clock;}
            else if(scenario.find("encoder")!=std::string::npos)full_encoder_valid=uint8_t(15u&~(1u<<ob_ch));
            else if(scenario.find("capability")!=std::string::npos){Motion_control_set_PWM(1,500);expected=FullLoadFault::capability;}
            else if(scenario.find("unsupported")!=std::string::npos){ob_request(full_motion(ob_ch,9,1));expected=FullLoadFault::unsupported;}
            else full_pulls[1]=NAN;
            ob_tick(10);CHECK(full_load_fault()==expected&&full_stopped());
#if BMCU_LITE_AUTHORITY_OBSERVATION
            ob_fact(1,authority_observation::fault);CHECK(authority_observation::records[1][2]==unsigned(expected));
#endif
        }else{
            if(scenario.find("exempt")!=std::string::npos){for(unsigned i=0;i<3;++i){ob_tick(20);ob_request(full_motion(0xff,1,0,3));CHECK(!full_stopped()&&full_load_fault()==FullLoadFault::none);}}
            if(scenario.find("wrong")!=std::string::npos){full_keys[1]=1.8f;ob_tick(10);ob_bf(4,1);}
            ob_until_age(scenario.find("wrong")!=std::string::npos?90:100);CHECK(unsigned(full_load_fault())==7&&full_stopped());
#if BMCU_LITE_AUTHORITY_OBSERVATION
            ob_fact(1,authority_observation::authority);const auto* r=authority_observation::records[1];CHECK(r[2]==7&&ob_count(r)==1&&r[8]==100);CHECK(r[20]==ob_command&&r[21]==ob_ch);
            if(scenario.find("wrong")!=std::string::npos)CHECK(r[12]==4&&r[13]==1&&r[5]==ob_ch);
            std::array<uint8_t,26> first{};std::memcpy(first.data(),r,26);
#endif
            if(scenario.find("filtering")!=std::string::npos){
                auto invalid=full_motion(ob_ch,3,0xbf,4);invalid.back()^=1;ob_request(invalid);ob_request(full_motion(ob_ch,3,0xbf,4,1));
#if BMCU_LITE_AUTHORITY_OBSERVATION
                CHECK(authority_observation::post_count==0);
#endif
                ob_tick(37);ob_request(full_motion(1,3,0xbf,4));CHECK(full_stopped()&&unsigned(full_load_fault())==7);
#if BMCU_LITE_AUTHORITY_OBSERVATION
                CHECK(authority_observation::post_count==1&&authority_observation::post[0]==4&&authority_observation::post[1]==1&&authority_observation::post[2]==37&&authority_observation::post[3]==0);
#endif
            }
            if(scenario.find("queue")!=std::string::npos){
                // Produce an actual reply but withhold TX dispatch/completion.
                repair_complete_tx();append(full_motion(0xff,1,0,4));bus_rx_poll();bambubus_run();CHECK(bus_port_to_host.send_data_len);
                append(full_motion(ob_ch,3,0xbf,3));bus_rx_poll();CHECK(bus_port_to_host.recv_data_len);
                for(unsigned i=0;i<3;++i)bambubus_run();
#if BMCU_LITE_AUTHORITY_OBSERVATION
                CHECK(authority_observation::post_count==0);
#endif
                bus_port_to_host.send_package();repair_complete_tx();bambubus_run();bambubus_run();bus_port_to_host.send_package();repair_complete_tx();
#if BMCU_LITE_AUTHORITY_OBSERVATION
                CHECK(authority_observation::post_count==1);
#endif
            }
            if(scenario.find("later")!=std::string::npos||scenario.find("optical")!=std::string::npos){
                for(unsigned i=0;i<6;++i){ob_tick(50);ob_request(full_motion(i%3,3,0xbf,i%2?4:3));CHECK(full_stopped()&&unsigned(full_load_fault())==7);}
                auto invalid=full_motion(ob_ch,3,0xbf,4);invalid.back()^=1;ob_request(invalid);ob_request(full_motion(ob_ch,3,0xbf,4,1));
#if BMCU_LITE_AUTHORITY_OBSERVATION
                CHECK(authority_observation::post_count==4&&authority_observation::post_flags&1);CHECK(!std::memcmp(first.data(),authority_observation::records[1],26));
                for(unsigned i=0;i<4;++i){const auto* p=authority_observation::post+4*i;CHECK(p[0]==(i%2?4:3)&&p[1]==i%3&&(unsigned(p[2])|(unsigned(p[3])<<8))==(i+1)*50);}
#endif
            }
        }
    }
    // Equal real-main quiet publication interval in ON and OFF. Export must not
    // add hidden ON-only controller/PWM iterations or read an old cached frame.
    ob_tick(3);
#if BMCU_LITE_AUTHORITY_OBSERVATION
    const auto frozen=authority_observation::mask;CHECK(frozen);
    if(scenario.find("clock")==std::string::npos&&scenario.find("busy")==std::string::npos)ob_export();
#endif
    if(scenario.find("optical")!=std::string::npos){
#if BMCU_LITE_AUTHORITY_OBSERVATION
        const auto name=std::string(std::getenv("CUE_TRACE_DIR"))+"/optical-events.csv";ob_optical=std::fopen(name.c_str(),"wx");CHECK(ob_optical);std::fprintf(ob_optical,"seconds,r,g,b\n");
        authority_observation::cursor=0;authority_observation::dirty=true;
#endif
        // Full actual event ordering is hashed during the long media tail to
        // avoid archiving megabytes of identical stopped timer writes.
        ob_media=true;ob_tick(28000);ob_media=false;
        std::fprintf(ob_trace,"MEDIA_TAIL,%llu,%llu\n",(unsigned long long)ob_tail_events,(unsigned long long)ob_tail_digest);
#if BMCU_LITE_AUTHORITY_OBSERVATION
        CHECK(!std::fclose(ob_optical));ob_optical=nullptr;ob_export();
#endif
        CHECK(full_stopped()&&unsigned(full_load_fault())==7&&host_programs==4);
    }
    std::printf("OBSERVATION PASS %s fault=%u owner=%u durable=%u writes=%u stopped=%u\n",scenario.c_str(),unsigned(full_load_fault()),full_load_owner(),durable_owner,host_programs,full_stopped());
    CHECK(!std::fclose(ob_trace));ob_trace=nullptr;return 0;
}
#endif
