#if BMCU_USB_ISP_EXPORT
static void working_marker_prepare(){uint32_t words[4]={0x33494642u,flight::build,0x00010001u,0};words[3]=flight::jcrc(words,12);memcpy(host_marker,words,16);}
#endif
// Actual main/bootstrap/parser/scheduler; only hardware/time are modeled.
static Packet working_request(){Packet p(15);p[0]=0x3d;p[1]=0xc5;p[2]=15;p[4]=0xfe;memcpy(p.data()+5,"BFX2",4);memcpy(p.data()+9,&flight::build,4);package_add_crc(p.data(),p.size());return p;}
static void working_bootstrap(){static bool injected=false;if(!bus_passive){injected=false;return;}if(!injected){written=0;append(short_packet(0x20));injected=true;}}
static void working_export_hook(){++host_clock;
#if !BMCU_USB_ISP_EXPORT
if(bus_passive&&written==0)append(working_request());
#endif
if(log_tx.size()>=536)throw LoopComplete{};}
static bool working_export_boot(){
#if BMCU_USB_ISP_EXPORT
flight_reset(); // Model the actual cold boot before consuming new host intent.
working_marker_prepare();
#endif
written=0;host_time_hook=working_export_hook;bool ran=flight::boot();host_time_hook=nullptr;return ran;}
static Packet working_input;static unsigned working_calls;static bool working_injected;
static void working_hook(){++host_clock;++working_calls;
    CHECK(full_stopped()&&!g_adc_dma_inited&&!host_dma1.CFGR&&bus_port_to_host.idle&&!(DMA1_Channel4->CFGR&DMA_CFGR1_EN));
    if(bus_passive){CHECK(!(USART1->CTLR1&USART_Mode_Tx)&&USART1->CTLR1&USART_Mode_Rx);
        bus_port_to_host.send_data_len=7;bus_port_to_host.send_package();CHECK(bus_port_to_host.idle&&bus_port_to_host.send_data_len==7);bus_port_to_host.send_data_len=0;
        CHECK(!bus_uart1_dma_send(nullptr,0));uint8_t p[7]={};CHECK(!bus_uart1_dma_send(p,7));bus_port_to_host.send_package(p,7);CHECK(bus_port_to_host.idle);
        if(!working_injected){written=0;append(working_input);working_injected=true;}}
    if(working_calls>1000)throw LoopComplete{};
}
static bool working_select(const Packet& p){flight_reset();bus_started=false;DMA1_Channel4->CFGR=0;g_adc_dma_inited=false;host_dma1.CFGR=0;working_input=p;working_calls=0;working_injected=false;host_time_hook=working_hook;return flight::boot();}
static int working_case(const char*){
    for(unsigned i=0;i<4096;++i)host_nvm[i]=uint8_t(i*17+3);
    const auto nvm=std::vector<uint8_t>(host_nvm,host_nvm+4096);
    if(is("working-invalid")){
        std::vector<Packet> packets;auto hb=short_packet(0x20);auto bad=hb;bad.back()^=1;packets.push_back(bad);bad=hb;bad[3]^=1;packets.push_back(bad);
        packets.push_back(Packet(hb.begin(),hb.begin()+4));packets.push_back(full_motion(0,3,0xbf,3,1));packets.push_back(short_packet(0xfe));
        bad=working_request();bad[9]^=1;package_add_crc(bad.data(),bad.size());packets.push_back(bad);packets.push_back(Packet{0,1,2,3,4,5,6});
#if BMCU_USB_ISP_EXPORT
        packets.push_back(working_request()); // Even exact BFX2 cannot select export anymore.
#endif
        Packet ahub(12);ahub[0]=0x33;ahub[6]=bus_crc8(ahub.data(),6);packets.push_back(ahub);
        auto discovery=short_packet(5,26);discovery[5]=1;discovery[6]=1;package_add_crc(discovery.data(),discovery.size());packets.push_back(discovery);
        for(const auto& p:packets){const auto lp=log_programs,le=log_erases;try{working_select(p);CHECK(false);}catch(const LoopComplete&){}
            CHECK(!flight::capturing&&bus_passive&&log_tx.empty()&&full_stopped());CHECK(log_programs==lp&&log_erases==le&&!memcmp(nvm.data(),host_nvm,4096));
            for(unsigned i=0;i<512;++i)CHECK(host_log[i]==255);}
        std::puts("Passive invalid/noise/partial/wrong-address/build rejected; all TX APIs/no sensors/no flash/no PWM");return 0;
    }
    if(is("working-export-virgin")||is("working-export-saved")){
        if(is("working-export-saved")){flight_reset();flight::wstart();CHECK(flight::capturing);phase=SetupPhase::operation;flight::jevent(8,2,123);journal_drain();}
        const auto saved=std::vector<uint8_t>(host_log,host_log+512);const auto lp=log_programs,le=log_erases;
        #if BMCU_USB_ISP_EXPORT
        working_marker_prepare();
#endif
        CHECK(!working_select(working_request()));CHECK(!bus_started&&!flight::capturing&&!flight::acknowledged);
        host_time_hook=nullptr;flight::uart_init();for(unsigned i=0;i<100;++i)flight::export_step();CHECK(log_tx.size()>=536);
        CHECK(!memcmp(log_tx.data(),"BFL2",4)&&!memcmp(log_tx.data()+20,saved.data(),512));flight_save((scenario+".frame").c_str(),log_tx.data(),536);
#if !BMCU_USB_ISP_EXPORT
        flight_save("working-export-request.bin",working_request().data(),15);
#endif
        uint8_t ack[20];memcpy(ack,"BFA2",4);flight::jput(ack+4,flight::build);flight::jput(ack+8,flight::session);flight::jput(ack+12,flight::frame_crc);flight::jput(ack+16,flight::jcrc(ack,16));
        for(auto b:ack)log_rx.push_back(b);for(unsigned i=0;i<4;++i)flight::export_step();CHECK(!flight::acknowledged&&!memcmp(flight::reply,"BFE2",4));
        CHECK(log_programs==lp&&log_erases==le&&!memcmp(saved.data(),host_log,512)&&!memcmp(nvm.data(),host_nvm,4096));return 0;
    }
    if(is("working-select")){
        std::vector<Packet> packets={short_packet(0x20),short_packet(5),full_motion(0,3,0xbf)};
        Packet discovery(16);discovery[0]=0x3d;discovery[1]=5;discovery[4]=16;discovery[7]=0;discovery[8]=0x12;discovery[11]=3;discovery[12]=1;package_add_crc(discovery.data(),discovery.size());packets.push_back(discovery);
        for(const auto&p:packets){memset(host_log,255,512);CHECK(working_select(p));CHECK(flight::capturing&&flight::jsession==1&&flight::jgeneration==1&&!bus_started&&!bambubus_heartbeat_reset_armed&&full_stopped());CHECK(!memcmp(nvm.data(),host_nvm,4096));}
        std::puts("Heartbeat, addressedmotion and discovery select initialization; triggeringpacket never dispatched");return 0;
    }
    if(is("working-sessions")){
        for(unsigned run=1;run<=5;++run){const auto before=std::vector<uint8_t>(host_log,host_log+512);const int previous=flight::jnewest(host_log);
            CHECK(working_select(short_packet(0x20)));CHECK(flight::jsession==run&&flight::jgeneration==run);
            if(previous>=0)CHECK(!memcmp(host_log+previous*256,before.data()+previous*256,256));
            phase=SetupPhase::operation;host_time_hook=nullptr;flight::jevent(8,2,run);journal_drain();CHECK(!memcmp(nvm.data(),host_nvm,4096));}
        CHECK(!working_export_boot());flight_save("working-sessions.frame",flight::frame,536);return 0;
    }
    if(is("working-virgincuts")){
        unsigned cuts=0;for(int cut=0;cut<200;++cut){memset(host_log,255,512);flight_reset();log_cut=cut;bool interrupted=false;
            try{flight::wstart();}catch(const PowerCut&){interrupted=true;}log_cut=-1;
            const auto torn=std::vector<uint8_t>(host_log,host_log+512);flight_reset();CHECK(!working_export_boot());CHECK(!memcmp(torn.data(),host_log,512));
            CHECK(working_select(short_packet(0x20)));CHECK(!memcmp(nvm.data(),host_nvm,4096));
            CHECK(flight::capturing || (flight::storage_error&&flight::jfailed));++cuts;if(!interrupted)break;
        }CHECK(cuts==96);std::printf("Virgin preparation cutprefixes=%u readonly retrieval; torn diagnostic cannot grant control\n",cuts);return 0;
    }
    if(is("working-bootcuts")){
        CHECK(working_select(short_packet(0x20)));phase=SetupPhase::operation;host_time_hook=nullptr;journal_drain();
        const auto saved=std::vector<uint8_t>(host_log,host_log+512);unsigned cuts=0;
        for(int cut=0;cut<200;++cut){memcpy(host_log,saved.data(),512);flight_reset();log_cut=cut;bool interrupted=false;
            try{flight::wstart();}catch(const PowerCut&){interrupted=true;}log_cut=-1;
            CHECK(flight::jvalid(host_log)&&!memcmp(host_log,saved.data(),256));
            const auto torn=std::vector<uint8_t>(host_log,host_log+512);flight_reset();CHECK(!working_export_boot());CHECK(!memcmp(torn.data(),host_log,512));
            CHECK(working_select(short_packet(0x20)));CHECK(flight::capturing&&!memcmp(nvm.data(),host_nvm,4096));
            ++cuts;if(!interrupted)break;}
        CHECK(cuts==96);std::printf("Working session preparation cutprefixes=%u previouspage retained, export readonly, next validprinter selects safely\n",cuts);return 0;
    }
    if(is("working-diagnostic-failure")){
        memset(host_log,0x37,512);CHECK(working_select(short_packet(0x20)));CHECK(!flight::capturing&&flight::storage_error&&flight::jfailed&&full_stopped());
        memset(host_log,255,512);flight_reset();host_fail_program=true;flight::wstart();CHECK(!flight::capturing&&flight::storage_error&&full_stopped());CHECK(!memcmp(nvm.data(),host_nvm,4096));return 0;
    }
    CHECK(false);return 1;
}
static int working_runtime(const char* path){
    CHECK(phase==SetupPhase::operation&&full_load_ready()&&full_stopped()&&bus_started&&flight::capturing&&!host_programs&&!host_erases);
    host_time_hook=[](){raw_dma_boundary();};
    if(scenario.rfind("full-working-retained-",0)==0||scenario.rfind("full-working-reboot-",0)==0){unsigned ch=unsigned(scenario.back()-'0');CHECK(full_load_owner()==ch&&full_load_durable(ch)&&ams[0].now_filament_num==255);repair_tick(1500,true,false);CHECK(full_stopped()&&full_load_owner()==ch&&!host_programs);FILE*f=fopen(path,"wb");CHECK(f&&fwrite(host_nvm,1,4096,f)==4096&&!fclose(f));return 0;}
    for(auto& p:full_pulls)p=float(2048)*(3.3f/4095.f);repair_request(short_packet(0x20));repair_tick(20);
    for(unsigned ch=0;ch<4;++ch){full_keys[ch]=1.8f;repair_tick(10);repair_request(full_motion(ch,3,0xbf,3+ch%2));repair_tick(60);CHECK(full_load_durable(ch));
        repair_request(full_motion(ch,7,0,3+ch%2));repair_tick(20);CHECK(ams[0].filament[ch].motion==_filament_motion::on_use);
        repair_request(full_motion(ch,3,0x3f,3+ch%2));repair_tick(3);CHECK(lite_pull_state[ch]==LitePullState::active);
        full_keys[ch]=.2f;for(unsigned n=0;n<30&&filament_now_position[ch]!=filament_redetect;++n)repair_tick(1);CHECK(filament_now_position[ch]==filament_redetect);
        repair_tick(100);full_keys[ch]=1.8f;for(unsigned n=0;n<200&&!full_load_empty();++n)repair_tick(1);
        CHECK(full_load_empty()&&lite_pull_state[ch]==LitePullState::completed&&full_stopped()&&!host_terminal_seen&&bus_started);full_keys[ch]=.2f;repair_tick(20);
    }
    journal_drain();journal_inhibited_export();std::puts("Actual Load->onuse->Unload->differentchannel completed all4 without terminalpark");return 0;
}
