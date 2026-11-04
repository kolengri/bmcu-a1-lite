#if BMCU_LITE_V11_FEATURES
// Actual production main/parser/ADC/filter/journal/controller/PWM/serializer.
// Only raw peripheral inputs, integer time and DMA/USART completion are modeled.
static FILE* au_trace=nullptr;
static void au_event(const char* kind,unsigned a=0,int b=0){
    if(!au_trace)return;
    std::fprintf(au_trace,"%s,%u,%d,%u,%u,%u,%u,%u,%u",kind,a,b,full_load_owner(),durable_owner,unsigned(intent),ams[0].now_filament_num,unsigned(full_load_fault()),host_programs);
    for(unsigned ch=0;ch<4;++ch)std::fprintf(au_trace,",%u,%u,%u,%u",unsigned(MOTOR_CONTROL[ch].motion),encoder_count(ch),unsigned(dm_auto_state[ch]),dm_loaded[ch]);
    for(const auto &timer:compare_values)for(int v:timer)std::fprintf(au_trace,",%d",v);
    if(kind[0]=='R')for(auto v:full_reply)std::fprintf(au_trace,",%02x",v);
    std::fputc('\n',au_trace);
}
static void authority_pwm(unsigned ch,int pwm){
    if(!au_trace)return;
#if BMCU_LITE_NATIVE_FEED_AUTHORITY
    if(pwm&&!v11_local_drive_allowed(ch)&&ams[0].filament[ch].motion==_filament_motion::send_out)
        CHECK(forward_grant&&(BMCU_LITE_V11_FEED_LATCH||host_clock-forward_received<100000)&&full_load_durable(ch)&&dispatching);
#endif
    au_event("PWM",ch,pwm);
}
static uint64_t au_origin=0,au_last=0;
static unsigned au_ch=0,au_cmd=3,au_elapsed=0;
static void au_tick(unsigned ms=1){
    for(unsigned i=0;i<ms;++i){delay(1);++au_elapsed;if(au_elapsed%400==0)repair_request(short_packet(0x20));repair_step();au_event("TICK");}
}
static void au_request(unsigned ch,unsigned state,unsigned motion,unsigned cmd=3,unsigned unit=0){
    const auto packet=full_motion(ch,state,motion,cmd,unit);repair_request(packet);au_event("REPLY",cmd,int(ch));
}
static void au_bf(unsigned cmd=3){au_last=host_clock;au_request(au_ch,3,0xbf,cmd);}
static void au_saved(unsigned ch){
#if BMCU_USB_WORKING_MODE
    // Strict retained reads reject any outstanding diagnostic Flash operation.
    // Settle that actual scheduler job before asserting the durable NVM bytes.
    if(!Flash_background_idle())std::printf("Retained-read checkpoint waits kind=%u phase=%u core-programs=%u\n",g_flash_job.kind,g_flash_job.phase,host_programs);
    journal_drain(5000);
#endif
    uint8_t got=0xff;CHECK(Flash_AMS_state_read(&got)&&got==ch);
}
static void au_live(){if(!(full_load_fault()==FullLoadFault::none&&full_load_durable(au_ch)&&ams[0].now_filament_num==au_ch&&ams[0].filament[au_ch].motion==_filament_motion::send_out)){std::printf("AUTHORITY LIVE FAILURE t=%llu last=%llu fault=%u intent=%u owner=%u durable=%u selected=%u motion=%u waiting=%u writes=%u\n",(unsigned long long)host_clock,(unsigned long long)au_last,unsigned(full_load_fault()),unsigned(intent),full_load_owner(),durable_owner,ams[0].now_filament_num,unsigned(ams[0].filament[au_ch].motion),waiting_send,host_programs);std::fflush(stdout);}CHECK(full_load_fault()==FullLoadFault::none&&full_load_durable(au_ch)&&ams[0].now_filament_num==au_ch&&ams[0].filament[au_ch].motion==_filament_motion::send_out);}
static void au_start(){
    full_keys[au_ch]=1.8f;au_tick(10);au_bf(au_cmd);
    CHECK(full_load_owner()==au_ch&&full_load_journal_pending()&&!full_load_durable(au_ch)&&full_stopped());
    CHECK(ams[0].now_filament_num==0xff&&!host_programs);
    for(unsigned i=0;i<40&&full_load_journal_pending();++i)au_tick();
    au_tick(10);au_live();au_saved(au_ch);CHECK(!full_stopped()&&host_programs==4&&!host_erases);
}
static void au_at(unsigned ms){
    const uint64_t target=au_last+uint64_t(ms)*1000;
    CHECK(host_clock<=target);while(host_clock+1000<=target)au_tick();
    host_clock=target;repair_step();
}
static void au_expired(){
    CHECK(unsigned(full_load_fault())==7u&&full_stopped()&&full_load_owner()==au_ch);
    const auto writes=host_programs;
    repair_request(short_packet(0x20));au_bf(4);au_request(0xff,1,0);au_tick(5);
    CHECK(unsigned(full_load_fault())==7u&&full_stopped()&&full_load_owner()==au_ch&&host_programs==writes);
}
static int au_done(){
    std::printf("AUTHORITY PASS %s owner=%u durable=%u selected=%u fault=%u writes=%u time=%llu tx=%u\n",scenario.c_str(),full_load_owner(),durable_owner,ams[0].now_filament_num,unsigned(full_load_fault()),host_programs,(unsigned long long)host_clock,repair_tx);
    CHECK(!std::fclose(au_trace));au_trace=nullptr;return 0;
}
static int authority_verify(){
    const auto trace=std::string(std::getenv("CUE_TRACE_DIR"))+"/"+scenario+
#if BMCU_LITE_NATIVE_FEED_AUTHORITY
    "-ON.csv";
#else
    "-OFF.csv";
#endif
    au_trace=std::fopen(trace.c_str(),"wx");CHECK(au_trace);
    au_ch=scenario.find("-ch2")!=std::string::npos?2u:scenario.find("-ch1")!=std::string::npos?1u:scenario.find("-ch3")!=std::string::npos?3u:0u;
    au_cmd=scenario.find("-04")!=std::string::npos?4u:3u;
    host_clock=((host_clock+999)/1000)*1000;au_origin=host_clock;
    host_time_hook=[](){if(FLASH->STATR&FLASH_FLAG_BSY)++host_clock;raw_dma_boundary();};
    for(auto &p:full_pulls)p=float(2048)*(3.3f/4095.f);
    repair_request(short_packet(0x20));au_tick(20);CHECK(v11_local_eligible());
    if(scenario.find("local")!=std::string::npos){
        full_keys[au_ch]=1.55f;au_tick(180);CHECK(dm_auto_state[au_ch]==DM_AUTO_S1_PUSH&&v11_pwm(au_ch));
        for(unsigned i=0;i<15;++i){au_request(0xff,1,0,i%2?3:4);au_tick(5);}
        CHECK(dm_auto_state[au_ch]==DM_AUTO_S1_PUSH&&v11_pwm(au_ch)&&!dm_loaded[au_ch]);
        full_keys[au_ch]=1.8f;au_tick(1200);CHECK(dm_loaded[au_ch]&&dm_auto_state[au_ch]==DM_AUTO_IDLE);
        CHECK(full_load_empty()&&!host_programs); // Local staging never reserves native ownership.
        if(scenario.find("control")!=std::string::npos)return au_done();
    }
    if(scenario.find("pending")!=std::string::npos){
        full_keys[au_ch]=full_keys[(au_ch+1)%4]=1.8f;au_tick(10);au_bf(au_cmd);
        CHECK(waiting_send&&full_stopped()&&full_load_journal_pending()&&!full_load_durable(au_ch));
        const uint64_t original=au_last;
        if(scenario.find("write-error")!=std::string::npos||scenario.find("busy")!=std::string::npos){
            const bool busy=scenario.find("busy")!=std::string::npos;
            host_fail_program=!busy;host_hang=busy;
            for(unsigned i=0;i<30&&full_load_fault()==FullLoadFault::none;++i)au_tick();
            CHECK(full_load_fault()==FullLoadFault::storage&&full_stopped()&&full_load_owner()==au_ch&&durable_owner==0xff);
            const auto issued=host_programs;const auto delivered=deliveries.size();au_tick(10);au_bf(4);
            CHECK(host_programs==issued&&full_load_fault()==FullLoadFault::storage&&full_stopped());
            uint8_t got=0xff;CHECK(!Flash_AMS_state_read(&got));
            if(busy){CHECK((FLASH->STATR&FLASH_FLAG_BSY)&&time_hw_slow&&deliveries.size()==delivered);CHECK(leds[&SYS_RGB][0]==0x200000u||leds[&SYS_RGB][0]==0x202000u);}
        }else if(scenario.find("cancel")!=std::string::npos){
            au_request(0xff,1,0,au_cmd);CHECK(!waiting_send&&full_stopped());
            delay(120);au_tick(10);CHECK(full_load_fault()==FullLoadFault::none&&full_stopped());
            au_bf(au_cmd);au_tick(10);au_live();CHECK(!full_stopped());
        }else if(scenario.find("rearm")!=std::string::npos){
            au_request(0xff,1,0,au_cmd);CHECK(!waiting_send);
            delay(60);au_bf(au_cmd==3?4:3);CHECK(waiting_send);
            au_request((au_ch+1)%4,3,0xbf,au_cmd);CHECK(full_load_owner()==au_ch);
            au_tick(10);au_live();au_at(99);au_live();au_tick();au_expired();
        }else if(scenario.find("repeat")!=std::string::npos){
            delay(70);au_bf(au_cmd==3?4:3);CHECK(waiting_send&&full_stopped()&&host_programs<=1);
            au_tick(10);au_live();au_at(99);au_live();au_tick();au_expired();
        }else if(scenario.find("commit-edge")!=std::string::npos){
            // Advance only at the real verified-reservation -> dispatch boundary.
            host_time_hook=[](){raw_dma_boundary();if(durable_owner==au_ch&&intent==JournalIntent::none&&ams[0].now_filament_num==0xff)host_clock=au_last+100000;};
            for(unsigned i=0;i<20&&full_load_fault()==FullLoadFault::none;++i)au_tick();
            CHECK(full_stopped()&&ams[0].now_filament_num==0xff&&full_load_durable(au_ch));au_saved(au_ch);au_expired();
        }else if(scenario.find("stale")!=std::string::npos){
            delay(100);repair_step();CHECK(full_stopped());au_expired();
            CHECK(ams[0].now_filament_num==0xff); // No synthetic fresh admission after expiry.
        }else{
            delay(85); // Real main/journal paused, with no prior native differential PWM.
            for(unsigned i=0;i<10&&full_load_journal_pending();++i)au_tick();au_live();CHECK(au_last==original&&host_programs==4);
            au_at(99);au_live();au_tick();au_expired();
        }
    }else{
        if(scenario.find("cross-wrap")!=std::string::npos){host_clock=0xffffffffull-70000;repair_request(short_packet(0x20));au_tick(5);}
        au_start();
        if(scenario.find("queued")!=std::string::npos){
            au_at(99);host_clock=au_last+100000;au_bf(4);au_expired();
        }else if(scenario.find("pwm-edge")!=std::string::npos){
            au_at(99);host_clock=au_last+100000;full_load_dispatch(true);Motion_control_set_PWM(au_ch,500);full_load_dispatch(false);au_expired();
        }else if(scenario.find("cross-wrap")!=std::string::npos){
            au_tick(60);au_bf(4);au_tick(60);au_bf(3);au_live();CHECK(host_clock>0x100000000ull);
            au_at(99);au_live();au_tick();au_expired();
        }else if(scenario.find("renew")!=std::string::npos){
            const auto helper=filament_pull_back_count[au_ch];
            for(unsigned i=0;i<30;++i){au_tick(60);au_bf(i%2?3:4);au_live();CHECK(!full_stopped()&&host_programs==4);}
            CHECK(filament_pull_back_count[au_ch]==helper);if(scenario.find("control")!=std::string::npos)return au_done();au_at(99);au_live();au_tick();au_expired();
        }else if(scenario.find("observed")!=std::string::npos||scenario.find("alias")!=std::string::npos){
            // Measured semantic prefix; intervening BF cadence is an explicit synthetic assumption.
            // No inference that the owner received 60ms BF renewals.
            for(unsigned i=0;i<30;++i){au_tick(60);au_bf(au_cmd);}
            au_at(99);au_request(0xff,1,0,3);au_live();CHECK(!full_stopped());
            au_tick();au_expired(); // An Abort alias is delayed only the remaining grant.
        }else if(scenario.find("idle04")!=std::string::npos||scenario.find("variant")!=std::string::npos||scenario.find("rearm")!=std::string::npos){
            au_request(0xff,1,scenario.find("variant")!=std::string::npos?1:0,scenario.find("idle04")!=std::string::npos?4:3);
#if BMCU_LITE_NATIVE_IDLE_04
            if(scenario.find("idle04")!=std::string::npos){
                // Explicit changed policy ONLY in this fresh, durable native forward state.
                au_live();CHECK(!full_stopped());au_at(100);au_expired();return au_done();
            }
#endif
            CHECK(full_stopped()&&ams[0].now_filament_num==0xff&&full_load_fault()==FullLoadFault::none);au_saved(au_ch);
            delay(120);au_tick();CHECK(full_load_fault()==FullLoadFault::none&&full_stopped());
            if(scenario.find("rearm")!=std::string::npos){au_bf(4);au_tick(5);au_live();CHECK(!full_stopped()&&host_programs==4);}
        }else if(scenario.find("refused")!=std::string::npos){
            full_keys[(au_ch+1)%4]=1.8f;au_tick(10);au_at(80);
            au_request((au_ch+1)%4,3,0xbf,4);au_request(au_ch,3,0xbf,3,1);
            auto bad=full_motion(au_ch,3,0xbf,4);bad.back()^=1;repair_request(bad);
            au_at(99);au_live();au_tick();au_expired();au_saved(au_ch);
        }else if(scenario.find("use")!=std::string::npos||scenario.find("pull")!=std::string::npos){
            if(scenario.find("use")!=std::string::npos){au_request(au_ch,7,0,au_cmd);CHECK(ams[0].filament[au_ch].motion==_filament_motion::on_use);au_tick(120);CHECK(full_load_fault()==FullLoadFault::none);
                if(scenario.find("use-idle")!=std::string::npos){au_request(0xff,1,0,au_cmd);CHECK(full_stopped()&&ams[0].now_filament_num==0xff&&full_load_owner()==au_ch&&full_load_fault()==FullLoadFault::none);au_saved(au_ch);return au_done();}
            }
            au_request(au_ch,3,0x3f,au_cmd);au_tick(3);CHECK(lite_pull_state[au_ch]==LitePullState::active);
            if(scenario.find("pull-active-cancel")!=std::string::npos){au_request(0xff,1,0,au_cmd);CHECK(full_stopped()&&lite_recovery_fault()&&full_load_owner()==au_ch&&lite_pull_state[au_ch]==LitePullState::interrupted);au_saved(au_ch);return au_done();}
            full_keys[au_ch]=.2f;
            for(unsigned i=0;i<30&&filament_now_position[au_ch]!=filament_redetect;++i){au_tick();au_request(au_ch,3,0x3f,i%2?3:4);}
            CHECK(filament_now_position[au_ch]==filament_redetect&&full_load_owner()==au_ch);au_saved(au_ch);
            if(scenario.find("cancel")!=std::string::npos){au_request(0xff,1,0,3);CHECK(full_stopped()&&lite_recovery_fault()&&full_load_owner()==au_ch);}
            else{
                for(unsigned i=0;i<120;++i){au_tick();if(i%20==0)au_request(au_ch,3,0x3f,i%2?3:4);}
                CHECK(full_load_owner()==au_ch&&full_load_fault()==FullLoadFault::none);
                full_keys[au_ch]=1.8f;
                for(unsigned i=0;i<40&&full_load_owner()!=0xff;++i)au_tick();
                CHECK(full_load_empty()&&lite_pull_state[au_ch]==LitePullState::completed);au_saved(0xff);
            }
        }else if(scenario.find("fault")!=std::string::npos){
            FullLoadFault expected=FullLoadFault::inputs;
            if(scenario.find("clock-scale")!=std::string::npos){time_hw_tpms=1001;expected=FullLoadFault::clock;}
            else if(scenario.find("clock-back")!=std::string::npos){host_clock-=2000;expected=FullLoadFault::clock;}
            else if(scenario.find("storage")!=std::string::npos){g_flash_fault=true;expected=FullLoadFault::storage;}
            else if(scenario.find("unsupported")!=std::string::npos){au_request(au_ch,9,2);expected=FullLoadFault::unsupported;}
            else if(scenario.find("capability")!=std::string::npos){Motion_control_set_PWM((au_ch+1)%4,500);expected=FullLoadFault::capability;}
            else if(scenario.find("encoder")!=std::string::npos)full_encoder_valid=uint8_t(15u&~(1u<<au_ch));
            else if(scenario.find("jam")!=std::string::npos){g_on_use_jam_latch[au_ch]=1;expected=FullLoadFault::controller;}
            else full_pulls[(au_ch+1)%4]=NAN;
            au_tick(10);CHECK(full_load_fault()==expected&&full_stopped()&&full_load_owner()==au_ch);
            const auto writes=host_programs;au_bf(4);au_request(0xff,1,0);au_tick(110);
            CHECK(full_load_fault()==expected&&full_stopped()&&host_programs==writes);
            if(expected==FullLoadFault::storage){
                uint8_t got=0xff;CHECK(!Flash_AMS_state_read(&got));
                Setup_sta_record retained;CHECK(setup_sta_read(g_sta_saved_slot,retained)&&retained.channel==au_ch&&g_sta_saved_loaded==au_ch);
            }else au_saved(au_ch);
        }else if(scenario.find("lease")!=std::string::npos){
            // Real armed lease is placed one ms away (boundary setup, not a fake parser).
            bambubus_heartbeat_deadline=time_ticks32()+1000;au_tick(2);
            CHECK(!lite_motion_ready()&&full_stopped()&&full_load_fault()==FullLoadFault::none);
            repair_request(short_packet(0x20));au_tick(110);CHECK(full_stopped()&&full_load_fault()==FullLoadFault::none);au_saved(au_ch);
        }else if(scenario.find("wrap")!=std::string::npos){
            au_at(99);au_tick();au_expired();
            for(uint64_t extra:{0x80000000ull,0x100000000ull}){host_clock+=extra;au_bf(4);repair_request(short_packet(0x20));CHECK(unsigned(full_load_fault())==7u&&full_stopped());}
        }else if(scenario.find("idle-only")!=std::string::npos){
            for(unsigned i=0;i<4;++i){au_tick(20);au_request(0xff,1,0,3);au_live();CHECK(!full_stopped());}
            au_at(99);au_request(0xff,1,0,3);au_live();au_tick();au_expired();
        }else if(scenario.find("heartbeat")!=std::string::npos){
            au_at(80);repair_request(short_packet(0x20));au_at(99);au_live();au_tick();au_expired();
        }else{
            au_at(99);au_live();CHECK(!full_stopped());
            if(scenario.find("idle")!=std::string::npos)au_request(0xff,1,0,3);
            au_tick();
            if(scenario.find("exact101")!=std::string::npos){
                CHECK(host_clock==au_last+100000&&unsigned(full_load_fault())==7u&&full_stopped());
                au_tick();CHECK(host_clock==au_last+101000&&unsigned(full_load_fault())==7u&&full_stopped());
            }
            au_expired(); // exact 100ms; 101ms cannot revive even with BF.
            au_tick();au_bf(4);CHECK(unsigned(full_load_fault())==7u&&full_stopped());
        }
    }
    return au_done();
}
#endif
