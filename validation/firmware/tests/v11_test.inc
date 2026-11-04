#if BMCU_LITE_V11_FEATURES
static bool v11_pwm(unsigned ch){int t=ch==3?TIM2:ch==2?TIM3:TIM4,c=ch==0?2:0;return compare_values[t][c]!=compare_values[t][c+1];}
static void v11_expect_ff(){uint8_t owner=0;CHECK(Flash_AMS_state_read(&owner)&&owner==0xff&&full_load_empty());CHECK(ams[0].now_filament_num==0xff);}
static void v11_insert(unsigned ch){full_keys[ch]=1.55f;full_tick(180);CHECK(dm_key_to_state(ch,ADC_DMA_get_value()[7-2*ch])==2);CHECK(dm_auto_state[ch]==DM_AUTO_S1_PUSH);CHECK(v11_pwm(ch));}
static void v11_material(unsigned ch,unsigned value){
    auto p=short_packet(8,45);p[5]=ch;p[15]=value;p[16]=83;p[17]=149;p[18]=255;p[19]=230;p[21]=250;
    const char name[]="PETG";std::memcpy(&p[23],name,sizeof(name));package_add_crc(p.data(),p.size());
    CHECK(full_rx(p)==bambubus_package_type::set_filament_info);CHECK(g_fil_dirty&(1u<<ch));
}
static Packet v11_long(unsigned type,std::vector<uint8_t> payload){
    bambubus_long_packge_data data{};data.package_number=1;data.target_address=kLiteAddress;data.source_address=0x0300;data.type=type;data.datas=payload.data();data.data_length=payload.size();
    bambubus_long_package_get(&data);auto* buf=bus_port_to_host.tx_build_buf();Packet p(buf,buf+bus_port_to_host.send_data_len);bus_port_to_host.send_data_len=0;
    p[1]=5;package_add_crc(p.data(),p.size());return p;
}
static void v11_save(const char* path){FILE*f=std::fopen(path,"wb");CHECK(f);CHECK(std::fwrite(host_nvm,1,4096,f)==4096);std::fclose(f);}
static int v11_verify(const char* path){
    full_heartbeat();
    if(is("full-v11-loaded-gates")){
        for(auto &key:full_keys)key=1.55f;full_tick(300);CHECK(full_stopped()&&!v11_local_eligible());
        full_pulls[0]=1.35f;full_tick(5500);CHECK(!maintenance&&full_stopped());CHECK(full_load_owner()==2&&!host_programs&&!host_erases);return 0;
    }
    if(is("full-v11-metadata-cold")){
        CHECK(metadata_recovery);full_tick(200);CHECK(!metadata_recovery&&Flash_background_idle());
        Flash_FilamentInfo info{};CHECK(Flash_AMS_filament_read(2,&info)&&info.color_R==99);v11_expect_ff();return 0;
    }
    full_tick(20);CHECK(v11_local_eligible());
    if(is("full-v11-cold-idle")){CHECK(v11_pwm(3)&&!host_programs);v11_expect_ff();return 0;}
    if(is("full-v11-s1-fail")){
        v11_insert(0);full_tick(5000);CHECK(dm_fail_latch[0]&&dm_auto_state[0]==DM_AUTO_S1_FAIL_RETRACT&&v11_pwm(0));
        full_keys[0]=.2f;full_tick(30);CHECK(!dm_fail_latch[0]&&!v11_pwm(0)&&!dm_autoload_gate[0]);v11_expect_ff();return 0;
    }
    if(is("full-v11-retry")){
        v11_insert(0);full_keys[0]=1.8f;full_tick(20);
        for(unsigned attempt=1;attempt<=3;++attempt){
            full_pulls[0]=1.815f;full_tick(30);CHECK(dm_auto_try[0]==attempt);
            if(attempt<3){CHECK(dm_auto_state[0]==DM_AUTO_S2_RETRACT);full_pulls[0]=1.64f;full_tick(60);CHECK(dm_auto_state[0]==DM_AUTO_S2_PUSH);}
        }
        CHECK(dm_fail_latch[0]&&dm_auto_state[0]==DM_AUTO_S2_FAIL_RETRACT);
        full_tick(1600);CHECK(!v11_pwm(0));CHECK(full_load_fault()==FullLoadFault::none);v11_expect_ff();return 0;
    }
    if(scenario.rfind("full-v11-fault-",0)==0){
        v11_insert(0);
        if(is("full-v11-fault-adc"))full_pulls[1]=NAN;
        if(is("full-v11-fault-encoder"))full_encoder_valid=14;
        if(is("full-v11-fault-clock"))time_hw_tpms=1001;
        if(is("full-v11-fault-storage"))g_flash_fault=true;
        if(is("full-v11-fault-unsupported"))full_rx(full_motion(0,9,1));
        full_tick(30);CHECK(full_load_fault()!=FullLoadFault::none&&full_stopped()&&!v11_had_local);
        const auto first=full_load_fault();const auto writes=host_programs;full_heartbeat();full_tick(100);CHECK(full_load_fault()==first&&host_programs==writes&&full_stopped());return 0;
    }
    if(is("full-v11-metadata-cut")||is("full-v11-metadata-cut-early")||is("full-v11-metadata-address")||is("full-v11-metadata-native-fault")||is("full-v11-metadata-arbitrate")||is("full-v11-metadata-busy")||is("full-v11-metadata-error")){
        if(is("full-v11-metadata-cut")||is("full-v11-metadata-cut-early")){
            for(unsigned i=0;i<FIL_SLOTS_PER_PAGE;++i){v11_material(2,10+i);full_tick(70);CHECK(!g_fil_dirty);}
            v11_material(2,99);
            if(is("full-v11-metadata-cut-early")){
                host_powercut=1;try{full_tick(20);CHECK(false);}catch(const PowerCut&){}v11_save(path);return 0;
            }
            // Stop between durable scratch commit and replacement page copy.
            for(unsigned i=0;i<100&&!(g_flash_job.kind==1u&&g_flash_job.rollover);++i)full_tick();
            CHECK(g_flash_job.kind==1u&&g_flash_job.rollover);host_powercut=0;
            try{full_tick(10);CHECK(false);}catch(const PowerCut&){}v11_save(path);return 0;
        }
        v11_material(2,42);full_tick(1);CHECK(g_flash_job.kind==1u&&full_stopped());
        if(is("full-v11-metadata-arbitrate")){
            full_keys[0]=1.8f;full_tick(4);full_rx(full_motion(0,3,0xbf));CHECK(full_load_journal_pending()&&full_stopped());
            full_tick(100);CHECK(full_load_durable(0));
            Flash_FilamentInfo committed{};CHECK(Flash_AMS_filament_read(2,&committed)&&committed.color_R==42);
            CHECK(g_fil_dirty==4u); // Completion acknowledgment is deferred behind native ownership.
            return 0;
        }
        if(is("full-v11-metadata-address")){
            const auto writes=host_programs;g_flash_job.addr=FLASH_NVM_CAL_ADDR;full_tick(20);
            CHECK(full_load_fault()==FullLoadFault::capability&&host_programs==writes&&full_stopped());return 0;
        }
        if(is("full-v11-metadata-native-fault")){
            full_keys[0]=1.8f;full_tick(4);full_rx(full_motion(0,3,0xbf));CHECK(full_load_journal_pending()&&full_load_owner()==0&&full_stopped());
            host_fail_program=true;full_tick(30);CHECK(full_load_fault()==FullLoadFault::storage&&full_load_owner()==0&&!full_load_durable(0)&&full_stopped());
            const auto writes=host_programs;full_tick(50);CHECK(host_programs==writes&&g_sta_saved_loaded==0xff);return 0;
        }
        const auto optical_before=deliveries.size();std::array<uint8_t,2048> journal{};std::memcpy(journal.data(),host_flash_ptr(0x0800f600),journal.size());
        if(is("full-v11-metadata-busy"))host_hang=true;else host_fail_program=true;
        full_tick(30);CHECK(full_load_fault()==FullLoadFault::storage&&full_stopped());
        const unsigned writes=host_programs;full_tick(20,false);CHECK(host_programs==writes);
        uint8_t owner=0;CHECK(g_sta_have_saved&&g_sta_saved_loaded==0xff);
        if(host_hang){CHECK(time_hw_slow&&(FLASH->STATR&FLASH_FLAG_BSY));CHECK(!deliveries.empty());}
        else CHECK(!Flash_AMS_state_read(&owner)); // The real strict reader refuses a latched flash fault.
        CHECK(!std::memcmp(journal.data(),host_flash_ptr(0x0800f600),journal.size()));
        if(host_hang){CHECK(deliveries.size()==optical_before);CHECK(leds[&SYS_RGB][0]==0x200000u||leds[&SYS_RGB][0]==0x202000u);}
        if(!host_hang)v11_save(path);
        return 0;
    }
    if(is("full-v11-reset-cut")){
        host_powercut=1;host_time_hook=[](){++host_clock;raw_dma_boundary();};full_pulls[0]=1.35f;
        try{full_tick(7000);CHECK(false);}catch(const PowerCut&){}CHECK(maintenance&&!bus_started&&full_stopped());v11_save(path);return 0;
    }
    if(is("full-v11-material-all")){
        for(unsigned ch=0;ch<4;++ch){
            v11_material(ch,20+ch);full_tick(70);
            std::vector<uint8_t> data(34);data[1]=ch;data[10]=40+ch;data[11]=91;data[12]=151;data[13]=255;data[14]=220;data[16]=250;data[18]='P';data[19]='L';data[20]='A';
            CHECK(full_rx(v11_long(0x218,data))==bambubus_package_type::set_filament_info_type2);full_tick(70);
            Flash_FilamentInfo saved{};CHECK(Flash_AMS_filament_read(ch,&saved)&&saved.color_R==40+ch&&saved.color_G==91&&std::string(saved.name)=="PLA");
            CHECK(full_rx(v11_long(0x211,{0,uint8_t(ch)}))==bambubus_package_type::read_filament_info);
            bambubus_long_packge_data response{};bambubus_long_package_analysis(full_reply.data(),full_reply.size(),&response);
            CHECK(response.data_length==sizeof(long_packge_filament)&&response.datas[1]==ch&&response.datas[59]==40+ch&&response.datas[60]==91);
        }
        CHECK(!g_fil_dirty);v11_expect_ff();return 0;
    }
    if(is("full-v11-manual-empty")){
        full_pulls[0]=1.9f;full_tick(40);CHECK(v11_pwm(0)&&!auto_unload_active[0]);v11_expect_ff();
        full_tick(1100,false);CHECK(full_stopped());full_heartbeat();full_tick(30);CHECK(full_stopped());
        full_pulls[0]=1.65f;full_tick(30);full_pulls[0]=1.9f;full_tick(30);CHECK(v11_pwm(0));v11_expect_ff();return 0;
    }
    if(is("full-v11-autoload-all")){
        for(unsigned ch=0;ch<4;++ch){
            v11_insert(ch);full_keys[ch]=1.8f;full_tick(1200);
            CHECK(dm_loaded[ch]&&dm_auto_state[ch]==DM_AUTO_IDLE);CHECK(!v11_pwm(ch));v11_expect_ff();
        }
        CHECK(!host_programs&&!host_erases);std::puts("V11 four independent staged channels; actual S1/S2/120mm completion; no shared reservation");return 0;
    }
    if(is("full-v11-preempt")){
        v11_insert(1);full_keys[0]=1.8f;full_tick(10);
        full_rx(full_motion(0,3,0xbf,4));CHECK(full_stopped()&&!v11_had_local&&dm_auto_state[1]==DM_AUTO_IDLE&&dm_autoload_gate[1]);
        const unsigned before=host_programs;full_rx(full_motion(0,3,0xbf,3));CHECK(host_programs==before&&full_stopped());
        full_tick(50);CHECK(full_load_durable(0)&&!v11_pwm(1));
        full_rx(full_motion(0xff,1,0));full_tick(30);CHECK(full_stopped()&&full_load_owner()==0);return 0;
    }
    if(is("full-v11-lease-rearm")){
        v11_insert(2);full_tick(1100,false);CHECK(!lite_motion_ready()&&full_stopped()&&dm_autoload_gate[2]);
        full_heartbeat();full_tick(100);CHECK(!v11_pwm(2)&&dm_auto_state[2]==DM_AUTO_IDLE);
        full_keys[2]=.2f;full_tick(50);v11_insert(2);CHECK(!host_programs);return 0;
    }
    if(is("full-v11-idle-gesture")){
        full_keys[3]=1.8f;full_tick(20);full_pulls[3]=1.5f;full_tick(200);
        CHECK(v11_pwm(3)&&MOTOR_CONTROL[3].motion==filament_motion_enum::filament_motion_pressure_ctrl_idle);v11_expect_ff();
        full_pulls[0]=1.35f;full_tick(180);full_pulls[0]=1.65f;full_tick(40);
        CHECK(MC_ONLINE_key_stu[0]==2u&&!Motion_control_filament_present(0));
        full_rx(full_motion(0,3,0xbf,4));CHECK(full_load_empty()&&!full_load_journal_pending());
        bambubus_ams_stu_motion_package_struct status{};CHECK(full_reply.size()==sizeof(status));std::memcpy(&status,full_reply.data(),sizeof(status));
        CHECK(status.filament_online_flag[0]==8u&&status.filament_online_flag[1]==8u&&status.filament_online_flag[2]==8u&&status.filament_use_flag==0u);
        full_tick(20);CHECK(!Motion_control_filament_present(0));return 0;
    }
    if(is("full-v11-local-unload")){
        full_keys[0]=1.8f;full_tick(50);full_pulls[0]=1.94f;full_tick(70);full_pulls[0]=1.65f;full_tick(100);
        CHECK(auto_unload_active[0]&&v11_pwm(0));v11_expect_ff();
        full_keys[0]=.2f;full_tick(2000);CHECK(!auto_unload_active[0]);v11_expect_ff();CHECK(!host_programs);return 0;
    }
    if(is("full-v11-reset")){
        // Keep real DMA/time and the ordinary empty hold. Only bound terminal park.
        host_time_hook=[](){++host_clock;raw_dma_boundary();if(phase==SetupPhase::success||phase==SetupPhase::fault)throw LoopComplete{};};
        full_pulls[0]=1.35f;
        try{full_tick(7000);}catch(const LoopComplete&){}
        std::printf("RESET maintenance=%d phase=%u fault=%u fullfault=%u programs=%u erases=%u t=%llu\n",maintenance,unsigned(phase),unsigned(fault),unsigned(full_load_fault()),host_programs,host_erases,(unsigned long long)host_clock);std::fflush(stdout);
        CHECK(maintenance&&phase==SetupPhase::success&&fault==SetupFault::none&&!bus_started);
        CHECK(cal_retained()&&Motion_control_setup_retained());uint8_t saved=0;CHECK(Flash_AMS_state_read(&saved)&&saved==0xff);
        CHECK(host_erases==4&&host_programs==96);FILE*f=std::fopen(path,"wb");CHECK(f);CHECK(std::fwrite(host_nvm,1,4096,f)==4096);std::fclose(f);
        std::puts("V11 real 5s empty reset -> RAM maintenance -> original gesture/all4 CAL/direction -> retained verified FF");return 0;
    }
    if(is("full-v11-metadata")){
        Flash_FilamentInfo expected{};expected.color_R=17;expected.color_G=83;expected.color_B=149;expected.color_A=255;std::strcpy(expected.name,"PETG");expected.temperature_min=230;expected.temperature_max=250;
        flashinfo_to_ram(2,&expected);ams_datas_set_need_to_save_filament(2);full_tick(120);
        Flash_FilamentInfo saved{};CHECK(Flash_AMS_filament_read(2,&saved)&&!std::memcmp(&saved,&expected,sizeof(saved)));CHECK(!g_fil_dirty&&Flash_background_idle());v11_expect_ff();
        const auto programs=host_programs;full_tick(30);CHECK(host_programs==programs);return 0;
    }
    CHECK(false);return 1;
}
#endif
