#if BMCU_LITE_V11_FEATURES
// Fixed integer schedules through real full_load_step, parser, serializer, ADC,
// controller and PWM. Only DMA completion/USART TC are hardware-boundary events.
static unsigned repair_tx=0,repair_idle_frames=0,repair_elapsed=0,repair_command=0;
static uint64_t repair_digest=1469598103934665603ull;
static FILE* repair_trace_file=nullptr;
static std::string repair_trace_name;
static void repair_complete_tx(){
    if(bus_port_to_host.idle)return;
    CHECK(DMA1_Channel4->CNTR>0&&(DMA1_Channel4->CFGR&DMA_CFGR1_EN));
    auto* data=reinterpret_cast<uint8_t*>(DMA1_Channel4->MADDR);
    full_reply.assign(data,data+DMA1_Channel4->CNTR);
    CHECK(package_check_crc16(full_reply.data(),full_reply.size()));
    if(full_load_empty()&&ams[0].now_filament_num==0xff){
        if(full_reply[4]==3){
            CHECK(full_reply.size()==sizeof(bambubus_ams_motion_package_struct));
            bambubus_ams_motion_package_struct status{};std::memcpy(&status,full_reply.data(),sizeof(status));
#if BMCU_LITE_PHASE99_REQUEST_FEEDBACK
            const unsigned reply_channel=(host_active_request_phase&0xe0u)==0xe0u&&(host_active_request_phase&15u)<4u?host_active_request_phase&15u:255u;
            CHECK(status.filament_channel==reply_channel&&status.filament_channel_2==reply_channel&&status.filament_use_flag==0);
#else
            CHECK(status.filament_channel==0xff&&status.filament_channel_2==0xff&&status.filament_use_flag==0);
#endif
        }else if(full_reply[4]==4){
            CHECK(full_reply.size()==sizeof(bambubus_ams_stu_motion_package_struct));
            bambubus_ams_stu_motion_package_struct status{};std::memcpy(&status,full_reply.data(),sizeof(status));
#if BMCU_LITE_PHASE99_REQUEST_FEEDBACK
            const unsigned reply_channel=(host_active_request_phase&0xe0u)==0xe0u&&(host_active_request_phase&15u)<4u?host_active_request_phase&15u:255u;
            CHECK(status.filament_channel==reply_channel&&status.filament_use_flag==0);
            if(reply_channel<4u)CHECK(status.filament_channel_stu==reply_channel);
#else
            CHECK(status.filament_channel==0xff&&status.filament_use_flag==0);
#endif
            unsigned online=0;for(unsigned ch=0;ch<4;++ch)if(ams[0].filament[ch].online)online|=1u<<ch;
            for(unsigned i=0;i<3;++i)CHECK(status.filament_online_flag[i]==online);
        }
    }
    ++repair_tx;DMA1_Channel4->CNTR=0;USART1->STATR|=USART_FLAG_TC;
    USART1_IRQHandler();CHECK(bus_port_to_host.idle);
}
static void repair_step(const Packet* packet=nullptr){
#if BMCU_USB_FLIGHT_RECORDER
    if(host_terminal_seen)return; // The real terminal loop never services another frame.
#endif
    repair_complete_tx();if(packet)append(*packet);
#if BMCU_USB_FLIGHT_RECORDER
    try{full_load_step();}catch(const TerminalCapture&){}
#else
    full_load_step();
#endif
    repair_complete_tx();
}
static void repair_request(const Packet& packet){repair_step(&packet);}
static void repair_trace(){
    auto mix=[](uint64_t v){for(unsigned i=0;i<8;++i){repair_digest^=(v>>(i*8))&255u;repair_digest*=1099511628211ull;}};
    for(unsigned ch=0;ch<4;++ch){
        mix(dm_auto_state[ch]);mix(dm_loaded[ch]);mix(dm_autoload_gate[ch]);mix(dm_auto_remain_count[ch]);
        mix(dm_auto_try[ch]);mix(dm_fail_latch[ch]);mix(auto_unload_arm[ch]);mix(auto_unload_active[ch]);
        mix(unsigned(MOTOR_CONTROL[ch].motion));mix(encoder_count(ch));mix(drives[ch]);
    }
    for(const auto &timer:compare_values)for(int v:timer)mix(v);
    if(repair_trace_file){
        std::fprintf(repair_trace_file,"%u",repair_elapsed);
        for(unsigned ch=0;ch<4;++ch)std::fprintf(repair_trace_file,",%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",unsigned(dm_auto_state[ch]),dm_loaded[ch],dm_autoload_gate[ch],dm_auto_remain_count[ch],dm_auto_try[ch],dm_fail_latch[ch],auto_unload_arm[ch],auto_unload_active[ch],unsigned(MOTOR_CONTROL[ch].motion),encoder_count(ch),drives[ch]);
        for(const auto &timer:compare_values)for(int v:timer)std::fprintf(repair_trace_file,",%d",v);
        std::fputc('\n',repair_trace_file);
    }
}
static void repair_tick(unsigned ms,bool heartbeat=true,bool poll=true){
    for(unsigned i=0;i<ms;++i){
        delay(1);++repair_elapsed;
        if(heartbeat&&repair_elapsed%400==0)repair_request(short_packet(0x20));
        if(poll&&repair_command&&repair_elapsed%20==0){
            const auto p=full_motion(0xff,1,0,repair_command);repair_request(p);++repair_idle_frames;
        }else repair_step();
        repair_trace();
    }
}
static int repair_idle_verify(){
    repair_command=unsigned(scenario.back()-'0');CHECK(repair_command==0||repair_command==3||repair_command==4);
    if(scenario.find("stage-")!=std::string::npos||scenario.find("assist-")!=std::string::npos||scenario.find("unload-")!=std::string::npos){
        for(unsigned i=1;!repair_trace_file;++i){repair_trace_name=scenario+"-"+std::to_string(i)+".csv";
            const auto path=std::string(std::getenv("CUE_TRACE_DIR"))+"/"+repair_trace_name;
            repair_trace_file=std::fopen(path.c_str(),"wx");CHECK(repair_trace_file||i<1000);
        }
    }
    // Clock reads do not consume simulated wall time; delay advances exact 1ms.
    // Raw DMA/filter publications and modeled encoder response remain real inputs.
    host_time_hook=[](){raw_dma_boundary();};
    repair_request(short_packet(0x20));repair_tick(20);CHECK(v11_local_eligible());
    if(scenario.find("stage-")!=std::string::npos){
        full_keys[0]=1.55f;repair_tick(180);
        CHECK(dm_auto_state[0]==DM_AUTO_S1_PUSH&&v11_pwm(0)&&!dm_loaded[0]);
        CHECK(dm_auto_remain_count[0]==0); // No fabricated S2 progress on one switch.
        repair_tick(100);CHECK(dm_auto_state[0]==DM_AUTO_S1_PUSH&&!dm_loaded[0]);
        full_keys[0]=1.8f;repair_tick(20);
        CHECK(dm_auto_state[0]==DM_AUTO_S2_PUSH&&dm_auto_remain_count[0]>0&&v11_pwm(0));
        CHECK(!dm_loaded[0]); // Both switches alone must not fake completed staging.
        repair_tick(1200);CHECK(dm_auto_state[0]==DM_AUTO_IDLE&&dm_loaded[0]&&!v11_pwm(0));
        full_pulls[0]=1.5f;repair_tick(200);CHECK(v11_pwm(0));
    }else if(scenario.find("assist-")!=std::string::npos){
        full_keys[2]=1.8f;repair_tick(30);full_pulls[2]=1.5f;repair_tick(1600);
        CHECK(v11_pwm(2)&&MOTOR_CONTROL[2].motion==filament_motion_enum::filament_motion_pressure_ctrl_idle);
        CHECK(dm_auto_state[2]==DM_AUTO_IDLE); // Original uncapped idle; no new travel envelope.
    }else if(scenario.find("unload-")!=std::string::npos){
        full_keys[0]=1.8f;repair_tick(50);full_pulls[0]=1.94f;repair_tick(70);
        full_pulls[0]=1.65f;repair_tick(100);CHECK(auto_unload_active[0]&&v11_pwm(0));
        full_keys[0]=.2f;repair_tick(2000);CHECK(!auto_unload_active[0]&&!v11_pwm(0));
    }else{
        // Start local pickup, then exercise cancellation paths using native frames.
        full_keys[1]=1.55f;repair_tick(180);CHECK(dm_auto_state[1]==DM_AUTO_S1_PUSH&&v11_pwm(1));
        if(scenario.find("lease-")!=std::string::npos){
            repair_tick(1100,false);CHECK(!lite_motion_ready()&&full_stopped()&&dm_autoload_gate[1]);
            repair_request(short_packet(0x20));repair_tick(100);CHECK(full_stopped()&&dm_auto_state[1]==DM_AUTO_IDLE);
            full_keys[1]=.2f;repair_tick(50);full_keys[1]=1.55f;repair_tick(180);
            CHECK(dm_auto_state[1]==DM_AUTO_S1_PUSH&&v11_pwm(1));
        }else if(scenario.find("fault-")!=std::string::npos){
            const bool storage=scenario.find("storage-")!=std::string::npos;
            if(storage)g_flash_fault=true;else full_pulls[3]=NAN;
            repair_tick(30);
            CHECK(full_load_fault()==(storage?FullLoadFault::storage:FullLoadFault::inputs)&&full_stopped()&&!v11_had_local);
            const auto first=full_load_fault();const auto programs=host_programs;
            repair_request(short_packet(0x20));repair_tick(100);CHECK(full_load_fault()==first&&full_stopped()&&host_programs==programs);
        }else{
            full_keys[0]=1.8f;repair_tick(10);
            repair_request(full_motion(0,3,0xbf,repair_command));
            CHECK(full_load_journal_pending()&&full_stopped()&&dm_auto_state[1]==DM_AUTO_IDLE&&dm_autoload_gate[1]);
            if(scenario.find("pending-")==std::string::npos){
                repair_tick(50,true,false);CHECK(full_load_durable(0)&&ams[0].now_filament_num==0&&v11_pwm(0));
                if(scenario.find("pull-")!=std::string::npos){
                    repair_request(full_motion(0,3,0x3f,repair_command));repair_tick(10,true,false);
                    CHECK(lite_pull_state[0]==LitePullState::active);
                }
            }
            repair_request(full_motion(0xff,1,0,repair_command));
            CHECK(full_stopped()&&ams[0].now_filament_num==0xff&&!waiting_send);
            if(scenario.find("pull-")!=std::string::npos)CHECK(lite_pull_state[0]==LitePullState::interrupted);
            repair_tick(80);CHECK(full_load_durable(0)&&full_stopped()&&full_load_owner()==0);
            const auto programs=host_programs;
            full_keys[2]=1.8f;repair_tick(10);repair_request(full_motion(2,3,0xbf,repair_command));
            repair_tick(30);CHECK(full_load_owner()==0&&full_stopped()&&host_programs==programs);
            // Heartbeat and idle cannot silently resume the canceled local stage or native feed.
            repair_request(short_packet(0x20));repair_tick(50);CHECK(full_stopped()&&dm_auto_state[1]==DM_AUTO_IDLE);
            uint8_t owner=0xff;CHECK(Flash_AMS_state_read(&owner)&&owner==0);
            std::printf("REPAIR-GUARD %s owner=0 programs=%u tx=%u\n",scenario.c_str(),host_programs,repair_tx);return 0;
        }
    }
    if(full_load_fault()==FullLoadFault::none){v11_expect_ff();CHECK(!full_load_journal_pending());}
    CHECK(!host_programs&&!host_erases);
    CHECK(!repair_command||(repair_idle_frames>0&&repair_tx==repair_idle_frames));
    if(repair_trace_file){CHECK(std::fclose(repair_trace_file)==0);std::printf("REPAIR-CSV %s %s\n",scenario.c_str(),repair_trace_name.c_str());}
    std::printf("REPAIR-TRACE %s digest=%016llx elapsed=%u idle=%u tx=%u owner=%u fault=%u\n",scenario.c_str(),(unsigned long long)repair_digest,repair_elapsed,repair_idle_frames,repair_tx,full_load_owner(),unsigned(full_load_fault()));return 0;
}
#endif
