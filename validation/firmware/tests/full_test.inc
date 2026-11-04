#if BMCU_LITE_FULL_LOAD
static float full_keys[4]={.2f,.2f,.2f,.2f};
static float full_pulls[4]={1.65f,1.65f,1.65f,1.65f};
static bool full_adc_stopped=false;
static Packet full_reply;
static void full_input(){
#if BMCU_LITE_V11_FEATURES
    if(maintenance)for(auto &p:full_pulls)p=1.65f;
#endif
    if(phase!=SetupPhase::inspect&&phase!=SetupPhase::operation)return;
    for(unsigned ch=0;ch<4;++ch){adc[6-2*ch]=full_pulls[ch];adc[7-2*ch]=full_keys[ch];}
    if(full_adc_stopped)host_dma1.CFGR=0;
}
static void full_boot_seed(const char* path){
    if(is("full-fresh"))return;
    if(scenario=="full-onuse-cold-loaded"){seed_cal();seed_motion();seed_owner(2,1,1);full_keys[2]=1.8f;return;}
    if(scenario=="full-unload-diagnostic-cold-no-provenance"||scenario.rfind("full-unload-diagnostic-lease-boot-",0)==0){seed_cal();seed_motion();seed_owner(2,1,1);full_keys[2]=1.8f;return;}
    if(is("full-cold-reserved")){full_keys[2]=1.8f;FILE* f=std::fopen(path,"rb");CHECK(f);CHECK(std::fread(host_nvm,1,4096,f)==4096);std::fclose(f);return;}
    if(is("full-cold-produced")){FILE* f=std::fopen(path,"rb");CHECK(f);CHECK(std::fread(host_nvm,1,4096,f)==4096);std::fclose(f);return;}
#if BMCU_USB_WORKING_MODE
    if(scenario.rfind("full-working-reboot-",0)==0){FILE*f=fopen(path,"rb");CHECK(f&&fread(host_nvm,1,4096,f)==4096&&!fclose(f));full_keys[unsigned(scenario.back()-'0')]=1.8f;return;}
#endif
    seed_cal();seed_motion();seed_owner(0xff);
    if(scenario.rfind("full-global-",0)==0||(scenario=="full-phase99-retained"||scenario=="full-phase99-03-retained"||scenario=="full-phase99-all-retained"||scenario=="full-ff07-retained")){seed_owner(2,1,1);full_keys[2]=1.8f;}
#if BMCU_USB_FLIGHT_RECORDER
    if(scenario.rfind("full-idle04-retained-",0)==0){seed_owner(2,1,1);full_keys[2]=1.8f;}
#endif
#if BMCU_LITE_V11_FEATURES
    if(is("full-v11-loaded-gates"))seed_owner(2,1,1);
    if(is("full-v11-cold-idle")){full_keys[3]=1.8f;full_pulls[3]=1.5f;}
    if(is("full-v11-metadata-cold")||is("full-v11-reset-interrupted-cold")||is("full-v11-metadata-invalid-cold")){
        FILE*f=std::fopen(path,"rb");CHECK(f);CHECK(std::fread(host_nvm,1,4096,f)==4096);std::fclose(f);
    }
#endif

    if((scenario.rfind("full-loaded-",0)==0&&scenario.back()>='0'&&scenario.back()<='3')){const unsigned ch=scenario.back()-'0';seed_owner(ch,1,1);full_keys[ch]=1.8f;}
#if BMCU_USB_WORKING_MODE
    if(scenario.rfind("full-working-retained-",0)==0){unsigned ch=unsigned(scenario.back()-'0');seed_owner(ch,1,1);full_keys[ch]=1.8f;}
#endif
    if(is("full-corrupt")||is("full-journal-setup-fault"))host_nvm[0]^=1;
    if(is("full-ambiguous"))seed_owner(0,0,1);
    if(is("full-loaded-empty"))seed_owner(2,1,1);
    if(scenario.rfind("full-r1-initial-absent-",0)==0||scenario.rfind("full-r1-wrong-owner-",0)==0)seed_owner(2,1,1);
    if(is("full-rollover"))for(unsigned i=0;i<STA_TOTAL_SLOTS;++i)seed_owner(0xff,uint16_t(i),i);
    if(is("full-startup-busy"))FLASH->STATR|=FLASH_FLAG_BSY;
    if(is("full-mask1")){auto* h=reinterpret_cast<NVM256_HDR*>(host_nvm);h->rsv=1u<<8;const auto crc=crc32_hw_words(host_nvm,252);std::memcpy(host_nvm+252,&crc,4);}
    if(is("full-bad-cal3")){Flash_CAL_payload c{};std::memcpy(&c,host_nvm+sizeof(NVM256_HDR),sizeof(c));c.offs[3]=NAN;seed_page(0x0800f000,MAGIC_CAL,2,&c,sizeof(c),15u<<8);}
    if(is("full-bad-mot3")){Motion_control_save_struct m{};std::memcpy(&m,host_flash_ptr(0x0800f100)+sizeof(NVM256_HDR),sizeof(m));m.Motion_control_dir[3]=0;seed_page(0x0800f100,MAGIC_MOT,VER_1,&m,sizeof(m));}
    if(is("full-incomplete"))std::memset(host_flash_ptr(0x0800f100),0xff,256);
}
static bool full_stopped(){for(unsigned t=0;t<3;++t)for(unsigned c=0;c<(t==2?4u:2u);c+=2)if(compare_values[t][c]!=compare_values[t][c+1])return false;return true;}
static bambubus_package_type full_rx(const Packet& packet){
    append(packet);bus_rx_poll();auto result=bambubus_run();
    auto* p=bus_port_to_host.tx_build_buf();full_reply.assign(p,p+bus_port_to_host.send_data_len);
    bus_port_to_host.send_data_len=0;
    if(!full_reply.empty())CHECK(package_check_crc16(full_reply.data(),full_reply.size()));
    return result;
}
static Packet full_motion(unsigned ch,unsigned state,unsigned flags,unsigned command=3,unsigned unit=0){
    auto p=short_packet(command,command==3?12:13);p[5]=unit;p[6]=state;if(command==3){p[7]=ch;p[8]=flags;}else{p[7]=flags;p[9]=ch;}package_add_crc(p.data(),p.size());return p;
}
static void full_heartbeat(){const bool faulted=lite_recovery_fault();const auto r=full_rx(short_packet(0x20));CHECK(r==(faulted?bambubus_package_type::error:bambubus_package_type::heartbeat));CHECK(bambubus_heartbeat_reset_armed);}
static void full_tick(unsigned ms=1,bool heartbeat=true){
    for(unsigned i=0;i<ms;++i){
        delay(1);if(heartbeat&&host_clock/1000%400==0)full_heartbeat();
        full_load_step();
    }
}
static void full_fault_lights(){
    CHECK(full_load_fault()!=FullLoadFault::none);const auto first=full_load_fault();
    full_tick(3,false);CHECK(!time_hw_slow);
    for(int ch=-1;ch<4;++ch){const auto* led=ch<0?&SYS_RGB:&RGBOUT[ch];bool found=false;
        for(auto it=deliveries.rbegin();it!=deliveries.rend();++it)if(it->led==led){CHECK(it->rgb[0]==0x200000u||it->rgb[0]==0x202000u);if(ch>=0)CHECK(it->rgb[1]==it->rgb[0]);found=true;break;}CHECK(found);}
    full_load_fail(FullLoadFault::capability);CHECK(full_load_fault()==first&&full_stopped());
}
static void full_present(unsigned ch){full_keys[ch]=1.8f;full_tick(10);CHECK(Motion_control_filament_present(ch));std::printf("PRESENT fault=%u ready=%d lease=%d recovery=%d readyADC=%d keys=%u/%u/%u/%u owner=%u t=%llu\n",unsigned(full_load_fault()),full_load_ready(),bambubus_heartbeat_reset_armed,lite_recovery_fault(),ADC_DMA_ready(),MC_ONLINE_key_stu[0],MC_ONLINE_key_stu[1],MC_ONLINE_key_stu[2],MC_ONLINE_key_stu[3],full_load_owner(),(unsigned long long)host_clock);std::fflush(stdout);}
static void full_start(unsigned ch){
    full_heartbeat();full_present(ch);
    CHECK(full_rx(full_motion(ch,3,0xbf))!=bambubus_package_type::error);
    CHECK(full_load_owner()==ch&&intent==JournalIntent::reserve&&!full_load_durable(ch)&&full_stopped());
    CHECK(ams[0].now_filament_num==0xff&&ams[0].filament_use_flag==0);
    const auto programs=host_programs;
    for(unsigned i=0;i<3;++i){full_rx(full_motion(ch,3,0xbf));CHECK(intent==JournalIntent::reserve&&host_programs==programs&&full_stopped());}
    for(unsigned i=0;i<40&&full_load_journal_pending();++i)full_tick();
    CHECK(full_load_durable(ch)&&full_load_owner()==ch&&!full_load_journal_pending());
    uint8_t stored=0xff;CHECK(Flash_AMS_state_read(&stored)&&stored==ch);
    full_tick(3);CHECK(ams[0].now_filament_num==ch&&ams[0].filament[ch].motion==_filament_motion::send_out);
}
#include "v11_test.inc"
#include "repair_idle_test.inc"
#include "authority_test.inc"
#include "observation_test.inc"
#include "idle04_test.inc"
#include "latch_test.inc"
#include "phase_test.inc"
#include "input_detail_test.inc"
#include "meter_test.inc"
#include "global_phase_test.inc"
#include "unload_continue_test.inc"
#include "phase99_test.inc"
#include "phase99_03_test.inc"
#include "phase99_all_slots_test.inc"
#include "ff07_test.inc"
#if BMCU_LITE_CONTINUOUS_NATIVE
#include "onuse_idle_test.inc"
#include "unload_diagnostic_test.inc"
#include "continuous_test.inc"
#endif
#include "automatic_test.inc"
#include "phase99_request_test.inc"
static int full_verify(const char* path){
#if BMCU_USB_CONTINUOUS_JOURNAL
#if BMCU_USB_WORKING_MODE
    if(scenario.rfind("full-working-",0)==0)return working_runtime(path);
#endif
    if(scenario.rfind("full-journal-",0)==0)return journal_case();
    if(scenario.rfind("full-retention-",0)==0)return retention_case();
    if(scenario.rfind("full-request99-",0)==0)return request99_verify();
#endif
    if(is("full-fresh")){
        CHECK(phase==SetupPhase::success&&fault==SetupFault::none&&!bus_started);
        CHECK(host_programs==60&&host_erases==2&&cal_retained()&&Motion_control_setup_retained());
        uint8_t stored=0;CHECK(Flash_AMS_state_read(&stored)&&stored==0xff);
        for(unsigned ch=0;ch<4;++ch)CHECK(drives[ch]>0);
        FILE* f=std::fopen(path,"wb");CHECK(f);CHECK(std::fwrite(host_nvm,1,4096,f)==4096);std::fclose(f);
        std::puts("Full profile: actual all4 gesture/CAL/direction/strictFF production fixture saved");return 0;
    }
    if(is("full-v11-reset-interrupted-cold")||is("full-v11-metadata-invalid-cold")||is("full-corrupt")||is("full-ambiguous")||is("full-incomplete")||is("full-startup-busy")||is("full-mask1")||is("full-bad-cal3")||is("full-bad-mot3")){
        CHECK(phase==SetupPhase::fault&&!bus_started&&!host_programs&&!host_erases&&full_stopped());
        std::puts("Full profile: incomplete/corrupt/ambiguous startup parked without writes");return 0;
    }
    CHECK(phase==SetupPhase::operation&&fault==SetupFault::none&&full_load_ready()&&bus_started);
    CHECK(!host_programs&&!host_erases&&full_stopped());
    for(unsigned ch=0;ch<4;++ch){CHECK(MC_PULL_calibration_is_valid(ch));CHECK(MOTOR_CONTROL[ch].dir==float(ch%2?-1:1));CHECK(filament_channel_inserted[ch]);CHECK(MOTOR_CONTROL[ch].motion==filament_motion_enum::filament_motion_stop);}
    // Stop only the bounded main-loop exit seam; hardware raw DMA/time hooks continue.
    host_time_hook=[](){++host_clock;raw_dma_boundary();};
#if BMCU_LITE_V11_FEATURES
#if BMCU_LITE_V11_FEED_LATCH && BMCU_LITE_AUTHORITY_OBSERVATION
#if BMCU_USB_FLIGHT_RECORDER
    if(scenario.rfind("full-ff07-",0)==0)return ff07_verify();
#if BMCU_LITE_CONTINUOUS_NATIVE
#if BMCU_LITE_AUTOMATIC_CAPTURE
    if(scenario.rfind("full-auto-",0)==0)return automatic_verify();
#endif
    if(scenario.rfind("full-unload-diagnostic-",0)==0)return unload_diagnostic_verify();
    if(scenario.rfind("full-onuse-",0)==0)return onuse_verify();
#endif
    if(scenario.rfind("full-phase99-all-",0)==0)return phase99_all_slots_verify();
    if(scenario.rfind("full-phase99-03-",0)==0)return phase99_03_verify();
#if BMCU_LITE_PHASE99_03_COMPAT
    // The former exact-cmd3 negative changes expectation only under its new child flag.
    if(scenario=="full-phase99-negative-cmd3")return phase99_03_verify();
#endif
    if(scenario.rfind("full-phase99-",0)==0)return phase99_verify();
    if(scenario.rfind("full-continue-",0)==0)return unload_continue_verify();
    if(scenario.rfind("full-global-",0)==0)return global_phase_verify();
    if(scenario.rfind("full-meter-",0)==0)return meter_verify();
    if(scenario=="full-adc-loop")return input_adc_verify();
    if(scenario=="full-input-none"||scenario=="full-input-noninput"||scenario=="full-input-first-snapshot")return input_control_verify();
    if(scenario.rfind("full-input-",0)==0)return input_detail_verify();
    if(scenario.rfind("full-phase-",0)==0)return phase_verify();
#endif
    if(scenario.rfind("full-latch-",0)==0)return latch_verify();
#endif
#if BMCU_LITE_NATIVE_FEED_AUTHORITY
    if(scenario.rfind("full-idle04-",0)==0)return idle04_verify();
#endif
    if(scenario.rfind("full-observe-",0)==0)return observation_verify();
    if(scenario.rfind("full-authority-",0)==0)return authority_verify();
    if(scenario.rfind("full-r1-idle-",0)==0)return repair_idle_verify();
#endif
    if(is("full-v11-pickup-required")){
        full_heartbeat();full_tick(20);full_keys[0]=1.55f;full_tick(180);
        CHECK(compare_values[TIM4][2]!=compare_values[TIM4][3]);
        CHECK(full_load_owner()==0xff&&!host_programs&&!host_erases);
        std::puts("Required physical first-switch pickup reached real PWM without a shared ownership write");return 0;
    }

#if BMCU_LITE_V11_FEATURES
    if(scenario.rfind("full-v11-",0)==0){
        const int result=v11_verify(path);
        std::printf("LOCAL-OUTCOME %s owner=%u writes=%u erases=%u",scenario.c_str(),full_load_owner(),host_programs,host_erases);
        for(unsigned ch=0;ch<4;++ch)std::printf(" ch%u=%u/%u/%u/%u/%u/%u",ch,drives[ch],unsigned(dm_auto_state[ch]),dm_loaded[ch],dm_auto_try[ch],dm_fail_latch[ch],dm_auto_remain_count[ch]);std::puts("");
        return result;
    }
#endif
    if((scenario.rfind("full-loaded-",0)==0&&scenario.back()>='0'&&scenario.back()<='3')){
        const unsigned ch=scenario.back()-'0';CHECK(full_load_owner()==ch&&full_load_durable(ch));
        full_heartbeat();full_tick(20);CHECK(full_stopped()&&ams[0].now_filament_num==0xff);
        const unsigned other=(ch+1)%4;full_present(other);full_rx(full_motion(other,3,0xbf));full_tick(20);
        CHECK(full_load_owner()==ch&&full_stopped()&&!host_programs&&!host_erases);
        full_keys[ch]=.2f;full_tick(20);CHECK(full_load_owner()==ch&&!host_programs);
        std::puts("Full profile: loaded/partial cold owner restored; no boot use/feed/cross-channel or absence clear");return 0;
    }
    if(is("full-cold-reserved")||is("full-loaded-empty")){CHECK(full_load_owner()==2&&full_load_durable(2));full_heartbeat();full_present(1);full_rx(full_motion(1,3,0xbf));full_tick(20);CHECK(full_stopped()&&full_load_owner()==2&&!host_programs&&!host_erases);return 0;}
    if(is("full-cold-produced")){CHECK(full_load_owner()==0xff&&g_valid_mask==15&&!host_programs&&full_stopped());std::puts("Actual setup-produced CAL/MOT/FF loaded all4 without interactive/default initialization");return 0;}
    if(is("full-native-only")){
        full_heartbeat();for(auto& key:full_keys)key=1.8f;for(auto& p:full_pulls)p=1.9f;
        full_tick(1000);CHECK(full_stopped()&&!host_programs&&full_load_owner()==0xff);
        std::puts("No local autoload/idle assist or virtual physical presence admits motion");return 0;
    }
    if(is("full-status")){
        full_heartbeat();for(unsigned i=0;i<4;++i)full_present(i);
        full_rx(full_motion(0xff,1,0,4));CHECK(full_reply.size()==sizeof(bambubus_ams_stu_motion_package_struct));
        bambubus_ams_stu_motion_package_struct r;std::memcpy(&r,full_reply.data(),sizeof(r));
        CHECK(r.filament_online_flag[0]==15&&r.filament_online_flag[1]==15&&r.filament_online_flag[2]==15&&r.filament_stu_flag==0x55&&r.filament_channel==0xff&&r.filament_use_flag==0);
        full_start(3);full_rx(full_motion(3,3,0xbf,4));std::memcpy(&r,full_reply.data(),sizeof(r));
        CHECK(r.ams_num==0&&r.filament_channel==3&&r.filament_use_flag==2&&r.filament_stu_flag==0x55);
        CHECK(full_load_owner()==3&&full_load_durable(3));std::puts("CRC-valid real03/04 serializers: all4 presence bits, fixed unit0, pending/active state coherent");return 0;
    }
    if(is("full-crc")){
        full_heartbeat();full_present(0);
        int previous_pwm[3][4];std::memcpy(previous_pwm,compare_values,sizeof(previous_pwm));
        const auto unchanged=[&](){
#if BMCU_LITE_V11_FEATURES
            return std::memcmp(previous_pwm,compare_values,sizeof(previous_pwm))==0;
#else
            return full_stopped();
#endif
        };
        auto p=full_motion(0,3,0xbf);p.back()^=1;full_rx(p);CHECK(full_load_owner()==0xff&&unchanged()&&!host_programs);
        p=full_motion(0,3,0xbf);p[3]^=1;const auto crc=bus_crc16(p.data(),p.size()-2);p[p.size()-2]=crc;p.back()=crc>>8;full_rx(p);CHECK(full_load_owner()==0xff&&unchanged()&&!host_programs);
        p=full_motion(0,3,0xbf,3,1);full_rx(p);CHECK(full_reply.empty()&&full_load_owner()==0xff&&!host_programs&&unchanged());
        p=full_motion(0,3,0xbf);full_rx(Packet(p.begin(),p.begin()+6));CHECK(full_load_owner()==0xff&&unchanged());full_rx(Packet(p.begin()+6,p.end()));CHECK(intent==JournalIntent::reserve&&full_load_owner()==0&&!full_load_durable(0)&&full_stopped());
        std::puts("Real DMA parser: CRC8/CRC16, wrong fixed unit and split native frame guards");return 0;
    }
    if(is("full-bounds")){
        full_heartbeat();full_rx(full_motion(4,3,0xbf));CHECK(full_load_fault()==FullLoadFault::unsupported&&full_stopped()&&full_load_owner()==0xff&&!host_programs);return 0;
    }
    if(is("full-empty-gesture")){
        full_heartbeat();full_pulls[0]=1.3f;full_tick(500);full_pulls[0]=1.65f;full_tick(100);
        CHECK(!Motion_control_filament_present(0)&&MC_ONLINE_key_stu[0]==0);full_rx(full_motion(0,3,0xbf));full_tick(10);CHECK(full_load_owner()==0xff&&full_stopped()&&!host_programs);return 0;
    }
    const unsigned ch=scenario.rfind("full-channel-",0)==0?unsigned(scenario.back()-'0'):2u;
    if(is("full-pending")||is("full-write-error")||is("full-write-busy")||scenario.rfind("full-cut-",0)==0){
        full_heartbeat();full_present(ch);full_rx(full_motion(ch,3,0xbf));
        CHECK(full_load_owner()==ch&&!full_load_durable(ch)&&full_stopped());
        if(is("full-write-error"))host_fail_program=true;
        if(is("full-write-busy"))host_hang=true;
        if(scenario.rfind("full-cut-",0)==0)host_powercut=scenario.back()-'0';
        try{
            for(unsigned i=0;i<30;++i){full_rx(full_motion(ch,3,0xbf));full_tick();if(!full_load_durable(ch))CHECK(full_stopped());}
        }catch(const PowerCut&){
            CHECK(full_stopped());g_flash_job={};FLASH->STATR=0;time_hw_slow=0;Flash_saves_init();
            const auto s=Flash_setup_probe();const unsigned cut=scenario.back()-'0';
            CHECK(cut==0?s.journal_valid&&s.owner==0xff:!s.journal_valid&&s.journal_invalid);
            std::puts("Real journal halfword power cut: no pre-durable PWM, strict restart refusal/old FF");return 0;
        }
        if(is("full-write-error")){CHECK(full_load_fault()==FullLoadFault::storage&&full_load_owner()==ch&&full_stopped());full_fault_lights();}
        else if(is("full-write-busy")){CHECK(full_stopped()&&full_load_owner()==ch&&!full_load_durable(ch));full_tick(20);CHECK(full_load_fault()==FullLoadFault::storage&&full_stopped());
            CHECK(time_hw_slow);const auto transmitted=deliveries.size();full_tick(3,false);CHECK(deliveries.size()==transmitted);
            CHECK(leds[&SYS_RGB][0]==0x200000u||leds[&SYS_RGB][0]==0x202000u);
            std::puts("LIMIT: permanently busy flash blocks real WS2812 transmission while fault color is queued");
            FLASH->STATR&=~FLASH_FLAG_BSY;host_hang=false;full_tick(3,false);full_fault_lights();
        }
        else CHECK(full_load_durable(ch));
        std::puts("Async reservation: repeated request idempotence, durable barrier/busy/write failure checked");return 0;
    }
    if(is("full-pending-fault")){
        full_heartbeat();full_present(ch);full_rx(full_motion(ch,3,0xbf));full_tick();
        CHECK(g_flash_job.kind&&g_flash_job.phase&&full_stopped());
        full_rx(full_motion(ch,7,0x66));CHECK(full_load_fault()==FullLoadFault::unsupported);
        const auto issued=host_programs;full_tick(30);CHECK(host_programs==issued&&full_stopped()&&full_load_owner()==ch&&!full_load_durable(ch));
        full_heartbeat();full_rx(full_motion(ch,3,0xbf));full_tick(20);CHECK(host_programs==issued&&full_stopped()&&full_load_fault()==FullLoadFault::unsupported);
        CHECK(!time_hw_slow);full_fault_lights();std::puts("Pending reservation fault: issued write cleaned up, no later halfword/retry, first fault and conservative owner retained");return 0;
    }
    if(is("full-pending-idle")||is("full-pending-lease")){
        full_heartbeat();full_present(ch);full_rx(full_motion(ch,3,0xbf));full_tick();
        if(is("full-pending-idle"))full_rx(full_motion(0xff,1,0));
        else{host_clock=uint64_t(bambubus_heartbeat_deadline)+1;full_rx({});}
        Motion_control_run(0);CHECK(!waiting_send&&full_stopped());full_tick(30,false);
        full_heartbeat();full_tick(20);CHECK(full_load_owner()==ch&&full_load_durable(ch)&&full_stopped()&&ams[0].now_filament_num==0xff);
        std::puts("Pending idle/lease: accepted reservation may commit; canceled send cannot auto-resume");return 0;
    }
    if(scenario.rfind("full-r1-initial-absent-",0)==0||scenario.rfind("full-r1-wrong-owner-",0)==0){
        const unsigned command=scenario.back()=='3'?3u:4u;
        full_heartbeat();full_tick(10);CHECK(full_load_owner()==ch&&full_load_durable(ch)&&full_stopped()&&!Motion_control_filament_present(ch));
        const unsigned target=scenario.rfind("full-r1-wrong-owner-",0)==0?(ch+1u)%4u:ch;
        full_rx(full_motion(target,3,0x3f,command));full_tick(3);
        if(target==ch)CHECK(full_load_fault()==FullLoadFault::inputs&&lite_pull_state[ch]==LitePullState::ready);
        else CHECK(full_load_fault()==FullLoadFault::none&&lite_pull_state[target]==LitePullState::ready&&ams[0].now_filament_num==0xff);
        uint8_t stored=0xff;CHECK(full_stopped()&&full_load_owner()==ch&&!host_programs&&!host_erases&&Flash_AMS_state_read(&stored)&&stored==ch);
        std::printf("R1 initial/owner exclusion native%02u target=%u fault=%u owner=%u stopped=%d\n",command,target,unsigned(full_load_fault()),full_load_owner(),full_stopped());return 0;
    }
    full_start(ch);
    if(scenario.rfind("full-r1-repeat-",0)==0){
        const unsigned command=scenario.find("-03-")!=std::string::npos?3u:4u;
        const bool at_redetect=scenario.find("-redetect")!=std::string::npos;
        full_rx(full_motion(ch,3,0x3f,command));full_tick(3);
        CHECK(lite_pull_state[ch]==LitePullState::active&&filament_now_position[ch]==filament_pulling_back);
        full_keys[ch]=.2f;
        for(unsigned i=0;i<40&&Motion_control_filament_present(ch);++i)full_tick();
        CHECK(!Motion_control_filament_present(ch));
        if(at_redetect)for(unsigned i=0;i<40&&filament_now_position[ch]!=filament_redetect;++i)full_tick();
        const auto expected=at_redetect?filament_redetect:filament_pulling_back;
        CHECK(filament_now_position[ch]==expected&&lite_pull_state[ch]==LitePullState::active&&full_load_fault()==FullLoadFault::none);
        const auto origin_count=filament_pull_back_count[ch],target=filament_pull_back_target[ch];
        const auto origin_ms=MOTOR_CONTROL[ch].pull_start_ms;const auto programs=host_programs;
        for(unsigned repeat=0;repeat<3;++repeat){
            full_rx(full_motion(ch,3,0x3f,command));
            std::printf("R1 repeat native%02u interval=%s fault=%u pullstate=%u owner=%u stopped=%d\n",command,at_redetect?"redetect":"pulling",unsigned(full_load_fault()),unsigned(lite_pull_state[ch]),full_load_owner(),full_stopped());std::fflush(stdout);
            CHECK(full_load_fault()==FullLoadFault::none&&lite_pull_state[ch]==LitePullState::active&&filament_now_position[ch]==expected);
            CHECK(filament_pull_back_count[ch]==origin_count&&filament_pull_back_target[ch]==target&&MOTOR_CONTROL[ch].pull_start_ms==origin_ms&&host_programs==programs&&full_load_owner()==ch&&!full_load_journal_pending());
        }
        full_tick();
        for(unsigned i=0;i<40&&filament_now_position[ch]!=filament_redetect;++i)full_tick();
        CHECK(filament_now_position[ch]==filament_redetect&&full_load_owner()==ch&&full_load_fault()==FullLoadFault::none&&!full_load_journal_pending());
        full_tick(3);CHECK(full_load_owner()==ch); // Neither absent interval nor a repeat is completion.
        full_keys[ch]=1.8f;
        for(unsigned i=0;i<40&&!full_load_journal_pending();++i)full_tick();
        CHECK(full_load_journal_pending()&&full_load_owner()==ch&&full_stopped());
        for(unsigned i=0;i<40&&full_load_journal_pending();++i)full_tick();
        uint8_t stored=0;CHECK(full_load_fault()==FullLoadFault::none&&lite_pull_state[ch]==LitePullState::completed&&full_load_owner()==0xff&&durable_owner==0xff&&Flash_AMS_state_read(&stored)&&stored==0xff&&host_programs==programs+4&&full_stopped());
        full_keys[ch]=.2f;full_tick(10);CHECK(!Motion_control_filament_present(ch));
        full_rx(full_motion(ch,3,0x3f,command));full_tick(3);
        CHECK(full_load_fault()==FullLoadFault::none&&lite_pull_state[ch]==LitePullState::completed&&full_load_owner()==0xff&&host_programs==programs+4&&full_stopped());
        std::puts("R1 verifiedFF completion and absent completed repeat without fault or extra journal write");return 0;
    }
    if(is("full-save-reserved")){FILE* f=std::fopen(path,"wb");CHECK(f);CHECK(std::fwrite(host_nvm,1,4096,f)==4096);std::fclose(f);return 0;}
    if(is("full-rollover")){CHECK(host_erases==1);full_rx(full_motion(0xff,1,0));CHECK(full_stopped());const auto scan=Flash_setup_probe();CHECK(scan.journal_valid&&!scan.journal_invalid&&scan.owner==ch);std::puts("Existing async journal rollover retains old owner until verified new record; strict rescan valid");return 0;}
    if(is("full-feed")||scenario.rfind("full-channel-",0)==0){
        full_tick(2500);CHECK(full_load_fault()==FullLoadFault::none);CHECK(drives[ch]>0);CHECK(!full_stopped());
        const auto start=MOTOR_CONTROL[ch].send_start_ms;const auto count=MOTOR_CONTROL[ch].send_start_count;
        for(unsigned i=0;i<5;++i){full_rx(full_motion(ch,3,0xbf));full_tick(10);CHECK(MOTOR_CONTROL[ch].send_start_ms==start&&MOTOR_CONTROL[ch].send_start_count==count);}
        CHECK(count_distance(encoder_count(ch),count)>distance_counts(.01f));
        std::puts("Full native send exceeds former2s/10mm trial with unchanged V11 helpers; repeated send origins retained");return 0;
    }
    if(is("full-use")){full_rx(full_motion(ch,7,0));full_tick(10);CHECK(ams[0].filament[ch].motion==_filament_motion::on_use&&full_load_owner()==ch&&full_load_durable(ch));std::puts("Native use retains durable reservation and uses V11 active controller");return 0;}
    if(is("full-idle")){full_rx(full_motion(0xff,1,0));full_tick(5);CHECK(full_stopped()&&full_load_owner()==ch&&full_load_durable(ch));std::puts("FF01 stops feed without clearing retained ownership");return 0;}
    if(is("full-lease")){
        const auto deadline=bambubus_heartbeat_deadline;host_clock=uint64_t(deadline)+1;full_rx({});Motion_control_run(0);CHECK(full_stopped()&&full_load_owner()==ch&&!lite_motion_ready());
        for(uint64_t extra:{0x80000000ull,0x100000000ull}){host_clock=uint64_t(deadline)+extra+2;full_rx(full_motion(ch,3,0xbf));Motion_control_run(0);CHECK(!lite_motion_ready()&&full_stopped()&&full_load_owner()==ch);}
        std::puts("Lease latch remains expired across half/full timer wraps; occupancy retained");return 0;
    }
    if(is("full-capability")){Motion_control_set_PWM((ch+1)%4,500);CHECK(full_load_fault()==FullLoadFault::capability&&full_stopped()&&full_load_owner()==ch);full_fault_lights();return 0;}
    if(is("full-unsupported")){full_rx(full_motion(ch,7,0x66));full_tick();CHECK(full_load_fault()==FullLoadFault::unsupported&&full_stopped()&&(lite_observations()&LITE_OTHER));full_fault_lights();return 0;}
    if(is("full-adc")){full_pulls[1]=NAN;full_tick(10);CHECK(full_load_fault()==FullLoadFault::inputs&&full_stopped()&&full_load_owner()==ch);full_fault_lights();return 0;}
    if(is("full-encoder")){full_encoder_valid=11;full_tick(10);CHECK(full_load_fault()==FullLoadFault::inputs&&full_stopped()&&full_load_owner()==ch);full_fault_lights();return 0;}
    if(is("full-clock")){time_hw_tpms=1001;full_tick();CHECK(full_load_fault()==FullLoadFault::clock&&full_stopped()&&full_load_owner()==ch);full_fault_lights();return 0;}
    if(is("full-jam")){g_on_use_jam_latch[ch]=1;full_tick();CHECK(full_load_fault()==FullLoadFault::controller&&full_stopped()&&full_load_owner()==ch);full_fault_lights();return 0;}
    if(is("full-absence")){full_keys[ch]=.2f;full_tick(10);CHECK(full_stopped()&&full_load_owner()==ch);uint8_t saved=0xff;CHECK(Flash_AMS_state_read(&saved)&&saved==ch);return 0;}
    if(scenario.rfind("full-unload",0)==0){
        full_rx(full_motion(ch,3,0x3f));full_tick(3);
        CHECK(lite_pull_state[ch]==LitePullState::active&&filament_now_position[ch]==filament_pulling_back);
        const auto origin_count=filament_pull_back_count[ch];full_rx(full_motion(ch,3,0x3f));full_tick(3);CHECK(filament_pull_back_count[ch]==origin_count);
        if(is("full-unload-idle")||is("full-unload-lease")){
            if(is("full-unload-idle"))full_rx(full_motion(0xff,1,0));
            else{host_clock=uint64_t(bambubus_heartbeat_deadline)+1;full_rx({});}
            Motion_control_run(0);CHECK(lite_recovery_fault()&&full_stopped()&&full_load_owner()==ch);full_heartbeat();full_rx(full_motion(ch,3,0xbf));CHECK(lite_recovery_fault()&&full_stopped()&&full_load_owner()==ch);return 0;
        }
        // Actual sensor schedule drives V11 distance/empty-publication -> redetect -> present completion.
        full_keys[ch]=.2f;
        for(unsigned i=0;i<30&&filament_now_position[ch]!=filament_redetect;++i)full_tick();
        CHECK(filament_now_position[ch]==filament_redetect&&full_load_owner()==ch&&!full_load_journal_pending());
        full_tick(3);CHECK(full_load_owner()==ch); // Absence/redetect alone cannot clear ownership.
        full_keys[ch]=1.8f;
        for(unsigned i=0;i<30&&!full_load_journal_pending();++i)full_tick();
        CHECK(full_load_journal_pending()&&full_load_owner()==ch&&full_stopped());
        if(is("full-unload-pending-fault")){
            full_tick();CHECK(g_flash_job.kind&&g_flash_job.phase);
            full_rx(full_motion(ch,7,0x66));CHECK(full_load_fault()==FullLoadFault::unsupported);
            const auto issued=host_programs;full_tick(30);CHECK(host_programs==issued&&full_stopped()&&full_load_owner()==ch&&durable_owner==ch);
            uint8_t stored=0xff;CHECK(!Flash_AMS_state_read(&stored)); // A suspended journal is still busy to strict consumers.
            Setup_sta_record retained;CHECK(setup_sta_read(g_sta_saved_slot,retained)&&retained.channel==ch&&g_sta_saved_loaded==ch);
            full_heartbeat();full_tick(20);CHECK(host_programs==issued&&full_stopped()&&full_load_fault()==FullLoadFault::unsupported);
            full_fault_lights();std::puts("Pending release fault: no further FF writes/retry, retained durable owner and first fault survive");return 0;
        }
        if(is("full-unload-write-error"))host_fail_program=true;
        if(is("full-unload-write-busy"))host_hang=true;
        if(scenario.rfind("full-unload-cut-",0)==0)host_powercut=scenario.back()-'0';
        try{for(unsigned i=0;i<40&&full_load_journal_pending();++i)full_tick();}
        catch(const PowerCut&){CHECK(full_stopped()&&full_load_owner()==ch);g_flash_job={};FLASH->STATR=0;time_hw_slow=0;Flash_saves_init();const auto scan=Flash_setup_probe();const unsigned cut=scenario.back()-'0';CHECK(cut==0?scan.journal_valid&&scan.owner==ch:!scan.journal_valid&&scan.journal_invalid);return 0;}
        if(is("full-unload-write-busy")){
            CHECK(full_load_fault()==FullLoadFault::storage&&full_stopped()&&full_load_owner()==ch&&durable_owner==ch&&time_hw_slow);
            const auto issued=host_programs;const auto transmitted=deliveries.size();full_tick(20,false);
            CHECK(host_programs==issued&&deliveries.size()==transmitted&&full_stopped()&&full_load_owner()==ch);
            CHECK(leds[&SYS_RGB][0]==0x200000u||leds[&SYS_RGB][0]==0x202000u);
            FLASH->STATR&=~FLASH_FLAG_BSY;host_hang=false;full_tick(3,false);
            CHECK(full_load_owner()==ch&&durable_owner==ch&&full_load_fault()==FullLoadFault::storage);full_fault_lights();
            std::puts("Busy release: durable owner retained, zero differential motion, first storage fault queued but optically blocked until modeled BSY clear");return 0;
        }
        if(is("full-unload-write-error")){full_tick(2);CHECK(full_load_fault()==FullLoadFault::storage&&full_load_owner()==ch&&full_stopped());return 0;}
        CHECK(full_load_owner()==0xff&&durable_owner==0xff&&full_stopped());uint8_t saved=0;CHECK(Flash_AMS_state_read(&saved)&&saved==0xff);
        full_rx(full_motion(ch,3,0x3f));full_tick(3);CHECK(full_load_owner()==0xff&&full_stopped()&&lite_pull_state[ch]==LitePullState::completed);
        std::puts("Actual V11 pull/redetect completion alone commits FF; repeated pull remains idempotent");return 0;
    }
    CHECK(false);return 1;
}
#else
static void full_input(){
#if BMCU_LITE_V11_FEATURES
    if(maintenance)for(auto &p:full_pulls)p=1.65f;
#endif
}
static void full_boot_seed(const char*){}
static int full_verify(const char*){CHECK(false);return 1;}
#endif
