// Actual boot/Flash controller; modeled ISP row writes and physical cut prefixes.
static unsigned marker_calls=0;
static void marker_hook(){host_clock+=1000;++marker_calls;
    CHECK(full_stopped()&&!g_adc_dma_inited&&!host_dma1.CFGR);
    if(log_tx.size()>=536 || marker_calls>150)throw LoopComplete{};
}
static void marker_reset(){flight_reset();bus_started=false;g_adc_dma_inited=false;host_dma1.CFGR=0;DMA1_Channel4->CFGR=DMA1_Channel5->CFGR=0;marker_calls=0;host_time_hook=marker_hook;}
static void marker_expected(uint32_t words[4]){words[0]=0x33494642u;words[1]=flight::build;words[2]=0x00010001u;words[3]=flight::jcrc(words,12);CHECK(words[3]==(flight::build==0x260b0404u?0x1987ba7bu:0x1342b362u));}
static bool marker_valid(){uint32_t expected[4];marker_expected(expected);return !memcmp(host_marker,expected,16);}
static void marker_blank(){for(unsigned n=0;n<256;n+=4){uint32_t v;memcpy(&v,host_marker+n,4);CHECK(v==0xffffffffu||v==0xe339e339u);}}
static void marker_passive(){const auto de=marker_export_enables,erases=marker_erases;try{flight::boot();CHECK(false);}catch(const LoopComplete&){}
    CHECK(bus_started&&bus_passive&&!flight::capturing&&marker_export_enables==de&&marker_erases==erases&&full_stopped());}
static int marker_case(){
    for(unsigned i=0;i<4096;++i)host_nvm[i]=uint8_t(i*17+3);
    const auto nvm=std::vector<uint8_t>(host_nvm,host_nvm+4096);
    auto preserved=[&](){CHECK(!memcmp(nvm.data(),host_nvm,4096));};
    if(is("marker-admission")||is("marker-quiesce")){
        marker_reset();working_marker_prepare();uint32_t expected[4];marker_expected(expected);flight_save("isp-marker.bin",expected,16);
        uint8_t row[56];memset(row,255,sizeof row);memcpy(row,expected,16);flight_save("isp-marker-row.bin",row,56);
        if(is("marker-quiesce")){USART1->CTLR1=USART_Mode_Tx|USART_Mode_Rx;USART1->CTLR3=USART_DMAReq_Tx|USART_DMAReq_Rx;DMA1_Channel4->CFGR=DMA1_Channel5->CFGR=DMA_CFGR1_EN;bus_started=false;}
        const auto lp=log_programs,le=log_erases,me=marker_erases,tx=marker_export_enables;
        try{firmware_main();CHECK(false);}catch(const LoopComplete&){}
        CHECK(marker_erases==me+1&&marker_export_enables==tx+1&&!bus_started&&!g_adc_dma_inited&&full_stopped());marker_blank();
        CHECK(log_programs==lp&&log_erases==le&&!memcmp(log_tx.data(),"BFL2",4)&&log_tx.size()==536);preserved();
        flight_save((scenario+".frame").c_str(),log_tx.data(),536);return 0;
    }
    if(is("marker-invalid")){
        uint32_t expected[4];marker_expected(expected);
        for(unsigned n=0;n<16;++n){memset(host_marker,255,256);memcpy(host_marker,expected,16);host_marker[n]^=1;marker_reset();marker_passive();preserved();}
        for(unsigned field:{1u,2u}){uint32_t wrong[4];memcpy(wrong,expected,16);wrong[field]^=1;wrong[3]=flight::jcrc(wrong,12);memset(host_marker,255,256);memcpy(host_marker,wrong,16);marker_reset();marker_passive();preserved();}
        for(uint32_t cause:{0u,0x08000000u,0x04000000u,0x80000000u}){memset(host_marker,255,256);marker_reset();host_rcc.RSTSCKR.value=cause;marker_passive();preserved();}
        // A retained exact marker is explicit intent independent of reset flags.
        for(uint32_t cause:{0u,0x08000000u,0x04000000u}){marker_reset();working_marker_prepare();host_rcc.RSTSCKR.value=cause;CHECK(!flight::boot());marker_blank();preserved();}
        std::puts("All16 marker-byte corruptions + absent/resetflags reject; validintent independentofresetcause");return 0;
    }
    if(is("marker-guards")){
        marker_reset();bus_boot_quiesce();uint32_t expected[4];marker_expected(expected);
        CHECK(!Flash_log_write(0x0800ed00u,0,false)&&!Flash_log_write(0x0800ed00u,0,true));
        CHECK(!Flash_diagnostic_submit(0x0800ed00u,0,false)&&!Flash_diagnostic_submit(0x0800ed00u,0,true));
        CHECK(!flash256_erase(0x0800ed00u,true)&&!flash_word_prog_std(0x0800ed00u,0,true));
        CHECK(!setup_trial_flash_address(0x0800ed00u));
        CHECK(Flash_boot_export_take(expected)==BootExportMarker::absent);working_marker_prepare();
        CHECK(Flash_boot_export_take(expected)==BootExportMarker::failed&&marker_valid());
        marker_reset();bus_boot_quiesce();phase=SetupPhase::operation;CHECK(Flash_boot_export_take(expected)==BootExportMarker::failed&&marker_valid());
        marker_reset();bus_boot_quiesce();compare_values[0][0]=compare_values[0][1]=1;CHECK(Flash_boot_export_take(expected)==BootExportMarker::failed&&marker_valid());
        CHECK(!marker_erases&&!marker_export_enables);preserved();return 0;
    }
    if(is("marker-failures")){
        for(unsigned mode=0;mode<4;++mode){marker_reset();working_marker_prepare();const auto tx=marker_export_enables,lp=log_programs,le=log_erases;
            if(mode==0)marker_erase_fail=true;if(mode==1)marker_verify_fail=true;if(mode==2)host_hang=true;if(mode==3)FLASH->STATR|=FLASH_FLAG_BSY;
            try{flight::boot();CHECK(false);}catch(const LoopComplete&){}
            CHECK(!bus_started&&!bus_passive&&bus_boot_quiet()&&marker_export_enables==tx&&full_stopped());CHECK(log_programs==lp&&log_erases==le);preserved();
            if(mode==2)CHECK(time_hw_slow&&FLASH->STATR&FLASH_FLAG_BSY);
            else if(mode!=3)CHECK(!time_hw_slow);
        }std::puts("Eraseerror/verifyerror/peripheralhang/preexistingbusy allinhibited withoutDEhigh/setup/motion");return 0;
    }
    if(is("marker-programcuts")){
        uint8_t row[56];memset(row,255,56);uint32_t expected[4];marker_expected(expected);memcpy(row,expected,16);
        const auto saved=std::vector<uint8_t>(host_log,host_log+512);
        for(int cut=0;cut<=84;++cut){memset(host_marker,255,256);marker_reset();log_cut=cut;
            try{for(unsigned n=0;n<56;n+=2)host_store16(0x0800ed00u+n,uint16_t(row[n]|uint16_t(row[n+1])<<8));}catch(const PowerCut&){}log_cut=-1;
            const bool valid=marker_valid();marker_reset();
            if(valid){CHECK(!flight::boot());marker_blank();}else marker_passive();
            CHECK(!memcmp(saved.data(),host_log,512));preserved();
        }std::puts("Canonical56B ISPpayload modeledhalfword/byte powercuts=85; onlyexact16Bintentselects andlogsunchanged");return 0;
    }
    if(is("marker-erasecuts")){
        const auto saved=std::vector<uint8_t>(host_log,host_log+512);uint8_t row[56];memset(row,255,56);uint32_t expected[4];marker_expected(expected);memcpy(row,expected,16);
        for(int cut=0;cut<=65;++cut){marker_reset();memset(host_marker,255,256);working_marker_prepare();log_cut=cut;
            try{flight::boot();}catch(const PowerCut&){}log_cut=-1;
            CHECK(!memcmp(saved.data(),host_log,512));preserved();
            const bool residual=marker_valid();marker_reset();
            if(residual){CHECK(!flight::boot());marker_blank();}else marker_passive();
            // Canonicalsamebuildretry is monotonic; no erasefallback is needed.
            marker_reset();for(unsigned n=0;n<56;n+=2)host_store16(0x0800ed00u+n,uint16_t(row[n]|uint16_t(row[n+1])<<8));CHECK(marker_valid());
            marker_reset();CHECK(!flight::boot());marker_blank();CHECK(!memcmp(saved.data(),host_log,512));preserved();
        }std::puts("Consumeerase powercuts=66; residualvalidintent staysUSBauthorized, partialerase neverexports, canonicalretry verified");return 0;
    }
    CHECK(false);return 1;
}
