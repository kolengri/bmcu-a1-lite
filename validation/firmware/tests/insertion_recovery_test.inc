// Actual main/setup/calibration/Flash/PWM bodies. Only external inputs and time
// are supplied here; permission hooks do not replace decisions or return values.
static uint64_t insertion_at=0, insertion_removed=0, insertion_cue_at=0;
static unsigned insertion_pauses=0,insertion_cal_runs=0,insertion_direction_runs=0;
static unsigned insertion_pause_programs=0,insertion_pause_erases=0;
static bool insertion_seen_pause=false,insertion_write_triggered=false;
static bool insertion_has(const char* word){return scenario.find(word)!=std::string::npos;}
static unsigned insertion_channel(){return unsigned(scenario.back()-'0');}
static void host_insertion_stage(unsigned stage){
    if(!insertion_case())return;
    if(stage==1)gesture_start=0;
    if(stage==2){++insertion_cal_runs;memset(prompts,0,sizeof(prompts));cal_start=0;}
    if(stage==3){++insertion_direction_runs;seen_drive=false;}
}
static void insertion_cue(uint32_t rgb){
    if(phase==SetupPhase::calibration&&g_valid_mask==15u&&(rgb==0x001600u||rgb==0x161600u)&&!insertion_cue_at)
        insertion_cue_at=host_clock/1000;
#if BMCU_SETUP_INSERTION_RECOVERY
    if(phase==SetupPhase::calibration)CHECK(rgb!=0x001600u);
#endif
}
static void insertion_input(){
    const auto ms=host_clock/1000;
    if(!insertion_at){
        bool trigger=(insertion_has("-inspect-")&&phase==SetupPhase::inspect)||
            (insertion_has("-gesture-")&&phase==SetupPhase::gesture&&gesture_start&&ms-gesture_start>2300)||
            (insertion_has("-postcue-")&&insertion_cue_at)||
            (insertion_has("-postcal-")&&phase==SetupPhase::calibration&&calibrated)||
            (insertion_has("-cal-sample-")&&phase==SetupPhase::calibration&&prompts[0][0]&&ms-prompts[0][0]>200)||
            (insertion_has("-direction-")&&phase==SetupPhase::direction&&seen_drive)||
            (insertion_has("-journalsettle-")&&phase==SetupPhase::journal);
        if(trigger)insertion_at=ms;
    }
    if(!insertion_at)return;
    const bool present=ms-insertion_at<(insertion_has("-postcue-")?2200u:600u);
    if(present){adc[7-2*insertion_channel()]=1.8f;
        if(insertion_has("-deflected-"))adc[6-2*insertion_channel()]=1.92f;
    }else if(!insertion_removed)insertion_removed=ms;
    if(insertion_has("-fault-adc-")&&ms-insertion_at>100)adc[0]=NAN;
    if(insertion_has("-fault-encoder-")&&ms-insertion_at>100)MC_AS5600.magnet_stu[1]=AS5600_soft_IIC_many::offline;
    if(insertion_has("-fault-clock-")&&ms-insertion_at>100)time_hw_tpms=1001;
    if(insertion_has("-fault-storage-")&&ms-insertion_at>100)g_flash_fault=true;
}
static void host_insertion_write(uint32_t address){
    if(!insertion_case()||insertion_write_triggered)return;
    const bool cal=insertion_has("-calwrite-")&&address>=0x0800f010u&&address<0x0800f100u;
    const bool motion=insertion_has("-motionwrite-")&&address>=0x0800f110u&&address<0x0800f200u;
    const bool journal=insertion_has("-journalcommit-")&&address==0x0800f600u&&g_flash_job.kind==2u;
    if(!cal&&!motion&&!journal)return;
    CHECK(full_stopped());insertion_write_triggered=true;insertion_at=host_clock/1000;
    // Model an input already published before this actual permission check.
    for(unsigned n=0;n<40;++n){delay(1);ADC_DMA_poll();}
}
static void insertion_tick(){
#if BMCU_SETUP_INSERTION_RECOVERY
    if(insertion_paused){
        CHECK(full_stopped()&&!bus_started);
        if(!insertion_seen_pause){insertion_seen_pause=true;++insertion_pauses;
            insertion_pause_programs=host_programs;insertion_pause_erases=host_erases;}
        CHECK(host_programs==insertion_pause_programs&&host_erases==insertion_pause_erases);
    }else if(insertion_seen_pause){
        CHECK(insertion_removed&&host_clock/1000>=insertion_removed+1500);
        insertion_seen_pause=false;
    }
#endif
}
static int insertion_verify(const char*){
    CHECK(insertion_at);
    if(insertion_has("-fault-")){
        CHECK(phase==SetupPhase::fault&&!bus_started&&full_stopped());
        const auto expected=insertion_has("-fault-adc-")?SetupFault::adc:
            insertion_has("-fault-encoder-")?SetupFault::encoder:
            insertion_has("-fault-clock-")?SetupFault::clock:SetupFault::storage;
        CHECK(fault==expected);return 0;
    }
    CHECK(phase==SetupPhase::operation&&fault==SetupFault::none&&bus_started&&full_load_ready());
    CHECK(cal_retained()&&Motion_control_setup_retained()&&full_load_empty()&&full_stopped());
    CHECK(host_programs==60&&host_erases==2);
#if BMCU_SETUP_INSERTION_RECOVERY
    CHECK(insertion_pauses==1&&insertion_epoch==1);
    if(insertion_has("-cal-sample-"))CHECK(insertion_cal_runs==2);
    if(insertion_has("-direction-"))CHECK(insertion_direction_runs==2);
#endif
    const auto writes=host_programs,erases=host_erases;
    host_time_hook=[](){raw_dma_boundary();};
    journal_drain(); // Readiness includes completion of the boot diagnostic queue.
    for(auto& pull:full_pulls)pull=1.65f;
    full_heartbeat();full_tick(200);CHECK(v11_local_eligible());
    for(unsigned ch=0;ch<4;++ch)CHECK(!dm_autoload_gate[ch]); // Existing fresh-empty publication admission.
    for(unsigned ch=0;ch<4;++ch){
        full_keys[ch]=1.55f;full_tick(180);
        std::printf("LOCAL ch=%u key=%u state=%u gate=%u loaded=%u blocked=%u fault=%u flash=%u ready=%u lease=%u hadlocal=%u\n",ch,MC_ONLINE_key_stu[ch],dm_auto_state[ch],dm_autoload_gate[ch],dm_loaded[ch],v11_helper_blocked[ch],unsigned(full_load_fault()),g_flash_job.kind,lite_motion_ready(),lite_lease_active(),v11_had_local);std::fflush(stdout);
        CHECK(dm_key_to_state(ch,ADC_DMA_get_value()[7-2*ch])==2);CHECK(dm_auto_state[ch]==DM_AUTO_S1_PUSH);CHECK(v11_pwm(ch));
        // Real journal producer while local S1 is active: never submit diagnostic
        // Flash until local stages/PWM are idle; genuine Flash-busy guard stays.
        if(ch==0){++bus_port_to_host.rx_dropped;full_tick(100);
            CHECK(dm_auto_state[ch]==DM_AUTO_S1_PUSH&&v11_pwm(ch));
            CHECK(flight::jqcount&&Flash_background_idle());}
        if(ch==0&&insertion_has("-localfault-")){
            full_keys[1]=NAN;full_tick(50);
            CHECK(full_load_fault()==FullLoadFault::inputs&&full_stopped());
            CHECK(!v11_had_local&&dm_auto_state[0]==DM_AUTO_IDLE&&v11_diagnostic_idle());
            journal_drain();bool recorded=false;
            for(unsigned page=0;page<2;++page){auto* p=host_log+page*256;if(!flight::jvalid(p))continue;
                for(unsigned slot=0;slot<14;++slot){auto* e=p+32+slot*16;
                    if(flight::jget(e+12)==flight::jcrc(e,12,flight::jget(p+4))&&
                        uint8_t(flight::jget(e+8))==12&&uint8_t(flight::jget(e+4))==uint8_t(FullLoadFault::inputs))recorded=true;}}
            CHECK(recorded);std::puts("INSERTION local input fault stops and journal cause commits");return 0;
        }
        full_keys[ch]=1.8f;full_tick(60);
        CHECK(dm_auto_state[ch]==DM_AUTO_S2_PUSH&&v11_pwm(ch));
        if(ch==0){++bus_port_to_host.rx_dropped;full_tick(100);
            CHECK(dm_auto_state[ch]==DM_AUTO_S2_PUSH&&v11_pwm(ch));
            CHECK(flight::jqcount&&Flash_background_idle());}
        full_tick(1200);
        CHECK(dm_loaded[ch]&&dm_auto_state[ch]==DM_AUTO_IDLE&&!v11_pwm(ch));v11_expect_ff();
    }
    full_tick(250);CHECK(!flight::jqcount&&!flight::jwaiting&&!flight::jstate);
    CHECK(host_programs==writes&&host_erases==erases&&full_load_fault()==FullLoadFault::none);
    std::printf("INSERTION recovered freshsetup calattempts=%u dirattempts=%u; all4 actual localS1/S2 complete\n",insertion_cal_runs,insertion_direction_runs);
    return 0;
}
