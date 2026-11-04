// Actual recorder + flash + main, modeling power loss only at physical byte/erase seams.
static int continuous_boot_case(){
#if !BMCU_LITE_CONTINUOUS_NATIVE
    CHECK(false);return 1;
#else
    std::array<uint8_t,4096> independent{};std::memcpy(independent.data(),host_nvm,4096);
    if(is("flight-continuous-boot")){
        const auto nvm=host_programs,erases=host_erases;
        host_time_hook=[](){++host_clock;if(log_tx.size()>=536)throw LoopComplete{};};
        try{firmware_main();CHECK(false);}catch(const LoopComplete&){}
        CHECK(!flight::capturing&&!bus_started&&!g_adc_dma_inited&&!host_dma1.CFGR);
        CHECK(flight::header_valid(host_log)&&flight::get32(host_log+20)==0x6a51ffffu);
        CHECK(!flight::complete(host_log)&&host_programs==nvm&&host_erases==erases);
        for(auto& row:compare_values)for(auto v:row)CHECK(!v);
        flight_save("virgin-inhibited.frame",log_tx.data(),536);
        std::array<uint8_t,512> prepared{};std::memcpy(prepared.data(),host_log,512);
        // No BFA1 is sent. Pin/software/watchdog/unknown causes cannot consume it.
        for(uint32_t cause:{0u,0x04000000u,0x10000000u,0x20000000u,0x40000000u,0x80000000u,0x18000000u}){
            flight_reset();std::memcpy(host_log,prepared.data(),512);RCC->RSTSCKR.value=cause|1u;
            CHECK(!flight::boot()&&!flight::capturing&&uint32_t(RCC->RSTSCKR)==1u);
            CHECK(!std::memcmp(host_log,prepared.data(),512));
        }
        for(uint32_t cause:{0x08000000u,0x0c000000u}){
            flight_reset();std::memcpy(host_log,prepared.data(),512);RCC->RSTSCKR.value=cause;
            CHECK(flight::boot()&&flight::capturing&&flight::session==2&&!flight::acknowledged);
            CHECK(flight::get32(host_log+20)==0xffffffffu&&!flight::complete(host_log));
            std::array<uint8_t,512> consumed{};std::memcpy(consumed.data(),host_log,512);
            for(uint32_t again:{0x08000000u,0x04000000u,0x10000000u,0x18000000u}){
                flight_reset();RCC->RSTSCKR.value=again;CHECK(!flight::boot()&&!flight::capturing);
                CHECK(!std::memcmp(host_log,consumed.data(),512));
            }
        }
        CHECK(!std::memcmp(host_nvm,independent.data(),4096));
        std::puts("CONTINUOUS virgin actual-main inhibited; permit consumed once without ACK; direct-A1-first exports and needs extra cycle; USB POR indistinguishable");return 0;
    }
    if(is("flight-continuous-cuts")){
        unsigned preparecuts=0,consumecuts=0;
        for(int cut=0;cut<1000;++cut){
            flight_reset();std::memset(host_log,255,512);log_cut=cut;bool lost=false;
            try{CHECK(!flight::boot()&&!flight::capturing);}catch(const PowerCut&){lost=true;}
            log_cut=-1;CHECK(!std::memcmp(host_nvm,independent.data(),4096));
            std::array<uint8_t,512> torn{};std::memcpy(torn.data(),host_log,512);
            flight_reset();const bool ran=flight::boot();
            if(ran)CHECK(flight::header_valid(torn.data())&&flight::get32(torn.data()+20)==0x6a51ffffu);
            else CHECK(!flight::capturing);
            if(!lost){preparecuts=cut;break;}
        }
        flight_reset();std::memset(host_log,255,512);CHECK(!flight::boot());
        std::array<uint8_t,512> prepared{};std::memcpy(prepared.data(),host_log,512);
        for(int cut=0;cut<1000;++cut){
            flight_reset();std::memcpy(host_log,prepared.data(),512);log_cut=cut;bool lost=false;
            try{CHECK(flight::boot()&&flight::capturing);}catch(const PowerCut&){lost=true;}
            log_cut=-1;CHECK(!std::memcmp(host_nvm,independent.data(),4096));
            std::array<uint8_t,512> torn{};std::memcpy(torn.data(),host_log,512);
            flight_reset();const bool ran=flight::boot();
            // A fully armed valid record is still pre-execution. No changed,
            // unarmed/corrupt/partial record may automatically run or re-arm.
            if(ran)CHECK(flight::header_valid(torn.data())&&flight::get32(torn.data()+20)==0x6a51ffffu);
            else CHECK(!flight::capturing);
            if(!lost){consumecuts=cut;break;}
        }
        CHECK(preparecuts>25&&consumecuts>150);
        std::printf("CONTINUOUS initial-permit cuts=%u consumption cuts=%u; NVM retained, no repeated automatic permit\n",preparecuts,consumecuts);
        // Every protected header/permit byte corruption inhibits later cold execution.
        for(unsigned n=0;n<24;++n){
            if(n==20||n==21)continue; // Existing adjacent unused halfword is not protected by arm semantics.
            flight_reset();std::memcpy(host_log,prepared.data(),512);host_log[n]^=1;CHECK(!flight::boot()&&!flight::capturing);
        }
        return 0;
    }
    if(is("flight-continuous-persist-cuts")){
        flight_reset();std::memset(host_log,255,512);CHECK(!flight::boot());flight_reset();CHECK(flight::boot());
        std::array<uint8_t,512> active{};std::memcpy(active.data(),host_log,512);unsigned cuts=0;
        for(int cut=0;cut<2000;++cut){
            flight_reset();std::memcpy(host_log,active.data(),512);flight::capturing=true;
            flight::setup_completed();flight::input_fault(0x0309u);log_cut=cut;bool lost=false;
            try{flight::finish_native(2);}catch(const PowerCut&){lost=true;}
            log_cut=-1;CHECK(!std::memcmp(host_nvm,independent.data(),4096));
            if(flight::complete(host_log)){flight::Report r{};std::memcpy(&r,host_log+32,sizeof(r));CHECK(r.native_fault==2&&(r.flags&8192u)&&!(r.flags&2u)&&r.input_detail==0x0309u);}
            flight_reset();CHECK(!flight::boot()&&!flight::capturing);
            if(!lost){cuts=cut;break;}
        }
        CHECK(cuts>500);std::printf("CONTINUOUS native persistence byte/commit cuts=%u\n",cuts);return 0;
    }
    CHECK(false);return 1;
#endif
}
