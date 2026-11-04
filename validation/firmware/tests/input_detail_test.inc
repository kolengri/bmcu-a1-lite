#if BMCU_USB_FLIGHT_RECORDER && BMCU_LITE_NATIVE_PHASE_COMPAT
static unsigned input_source=0,input_channel=0;
static bool input_active=false,input_trace_active=false;
static std::vector<int> input_reads;
#include "baseline_adc_assign.h"
static bool input_adc_testing=false;
static unsigned input_adc_rd=0,input_adc_generation=0,input_adc_barriers=0;
static float input_adc_expected[8];
static void host_input_detail_barrier(const float* out,unsigned wr){
    if(!input_adc_testing)return;
    CHECK(wr==(input_adc_rd^1u)&&g_v_rd==input_adc_rd&&g_publication==input_adc_generation&&g_acc_dirty);
    CHECK(!std::memcmp(out,input_adc_expected,sizeof(input_adc_expected)));++input_adc_barriers;
}
static int input_adc_verify(){
    host_time_hook=nullptr;host_delay_hook=nullptr;host_dma1.CFGR=0;DMA1->INTFR=0;
    const uint32_t values[]={0,1,4095,65535,0x7fffffff,0x80000000,0xfffffffe,0xffffffff};
    const float scales[]={0.f,-0.f,1.f,3.3f/4095.f,1.e-30f,1.e30f,INFINITY,NAN};
    unsigned comparisons=0;
    for(float factor:scales)for(unsigned rotate=0;rotate<8;++rotate){
        for(unsigned ch=0;ch<8;++ch)g_acc_sum[ch]=values[(ch+rotate)%8];
        baseline_adc_assign(input_adc_expected,g_acc_sum,factor);g_scale=factor;g_acc_dirty=1;
        input_adc_rd=g_v_rd;input_adc_generation=g_publication;input_adc_barriers=0;
        input_reads.clear();input_trace_active=true;input_adc_testing=true;
        const float* result=ADC_DMA_get_value();input_adc_testing=false;input_trace_active=false;
        CHECK(input_adc_barriers==1&&g_v_rd==(input_adc_rd^1u)&&g_publication==input_adc_generation+1&&!g_acc_dirty);
        CHECK(result==g_v[g_v_rd]&&!std::memcmp(result,input_adc_expected,sizeof(input_adc_expected)));
        CHECK((input_reads==std::vector<int>{1,8}));comparisons+=8;
        input_reads.clear();input_trace_active=true;CHECK(ADC_DMA_get_value()==result);input_trace_active=false;
        CHECK(g_publication==input_adc_generation+1&&!g_acc_dirty&&(input_reads==std::vector<int>{1,8}));
    }
    std::printf("ADC baseline-body bit comparisons=%u; poll/barrier/destination/publication order exact, clean reread unchanged\n",comparisons);return 0;
}
static int input_control_verify(){
    host_time_hook=nullptr;host_delay_hook=nullptr;
    if(scenario=="full-input-noninput"){
        full_load_fail(FullLoadFault::controller);
#if BMCU_LITE_INPUT_FAULT_DETAIL
        full_load_input_fail(InputFaultSource::pull_absent,3);CHECK(flight::report.input_detail==0);
#endif
        CHECK(full_load_fault()==FullLoadFault::controller);
    }else CHECK(full_load_fault()==FullLoadFault::none);
    CHECK(flight::finish_native(uint8_t(full_load_fault()))&&flight::complete(host_log));
    std::array<uint8_t,512> first{};std::memcpy(first.data(),host_log,512);
    if(scenario=="full-input-first-snapshot"){
#if BMCU_LITE_INPUT_FAULT_DETAIL
        full_load_input_fail(InputFaultSource::pull_absent,2);
#endif
        CHECK(flight::finish_native(uint8_t(full_load_fault()))&&!std::memcmp(first.data(),host_log,512));
    }
    flight::export_frame();flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));return 0;
}
static void host_input_detail_read(int kind){if(input_trace_active)input_reads.push_back(kind);}
static void host_input_detail_hook(int stage){
    if(!input_active)return;
    const unsigned ch=input_channel;
    if(stage==1){
        g_acc_dirty=0;
        if(input_source==1)g_blocks_filled=0;
        if(input_source==2){publication=g_publication;publication_ticks=host_clock-100001;}
        if(input_source==3)filament_channel_inserted[ch]=false;
        if(input_source==4)g_valid_mask&=uint8_t(~(1u<<ch));
        if(input_source==5)g_v[g_v_rd][6-2*ch]=scenario.find("inf")!=std::string::npos?INFINITY:NAN;
        if(input_source==6)g_v[g_v_rd][6-2*ch]=scenario.find("high")!=std::string::npos?3.26f:.04f;
        if(input_source==7)g_v[g_v_rd][7-2*ch]=scenario.find("inf")!=std::string::npos?-INFINITY:NAN;
        if(input_source==8)g_v[g_v_rd][7-2*ch]=scenario.find("high")!=std::string::npos?3.31f:-.01f;
        if(input_source==9)g_as5600_good[ch]=false;
        if(input_source==10)MC_AS5600.online[ch]=false;
        if(input_source==11)MC_AS5600.magnet_stu[ch]=AS5600_soft_IIC_many::offline;
        // Earlier predicate and earlier channel must win in simultaneous failures.
        if(scenario.find("multi")!=std::string::npos){g_v[g_v_rd][7-2*ch]=NAN;if(ch<3)filament_channel_inserted[ch+1]=false;}
    }
    if(stage==2){
        if(input_source==15)MC_AS5600.online[ch]=false;
        if(input_source==16)MC_AS5600.magnet_stu[ch]=AS5600_soft_IIC_many::offline;
        if(input_source==17)filament_channel_inserted[ch]=false;
    }
}
static int input_detail_verify(){
    unsigned s=0,ch=0;CHECK(std::sscanf(scenario.c_str(),"full-input-s%u-ch%u",&s,&ch)==2&&s<=18&&ch<4);
    input_source=s;input_channel=ch;input_active=true;
    host_time_hook=nullptr;host_delay_hook=nullptr;
    full_heartbeat();owner=durable_owner=uint8_t(ch);intent=JournalIntent::none;
    ams[0].now_filament_num=uint8_t(ch);ams[0].filament[ch].motion=_filament_motion::on_use;
    input_reads.clear();input_trace_active=true;
    if(s>=14&&s<=17){
        if(s==14)full_encoder_valid=uint8_t(15u&~(1u<<ch));
        Motion_control_init();
    }else if(s==18){
        lite_pull_state[ch]=LitePullState::ready;MC_ONLINE_key_stu[ch]=0;g_v[g_v_rd][7-2*ch]=.2f;g_acc_dirty=0;
        CHECK(full_rx(full_motion(ch,3,0x3f))==bambubus_package_type::error);
    }else{
        if(s==12)durable_owner=0xffu;
        Motion_control_run(0);
    }
    input_trace_active=false;input_active=false;
    if(s==0)CHECK(full_load_fault()==FullLoadFault::inputs); // Healthy inputs, selected physical presence is absent.
    else CHECK(full_load_fault()==FullLoadFault::inputs);
    const unsigned expected=s?s:13u;
#if BMCU_LITE_INPUT_FAULT_DETAIL
    CHECK(flight::report.input_detail==(expected|((s<=2&&s?255u:ch)<<8)));
    const auto first=flight::report.input_detail;
    // Subsequent actual source and non-input faults cannot replace the first detail.
    full_load_input_fail(InputFaultSource::pull_absent,uint8_t((ch+1)%4));
    full_load_fail(FullLoadFault::controller);CHECK(flight::report.input_detail==first);
#endif
    CHECK(full_stopped());
    std::printf("INPUT-TRACE %s fault=%u owner=%u selected=%u pending=%d writes=%u erases=%u polls=",scenario.c_str(),unsigned(full_load_fault()),owner,ams[0].now_filament_num,full_load_journal_pending(),host_programs,host_erases);
    for(int r:input_reads)std::printf("%d,",r);std::puts("");
    CHECK(flight::finish_native(uint8_t(full_load_fault()))&&flight::complete(host_log));
    flight::export_frame();flight_save((scenario+".frame").c_str(),flight::frame,sizeof(flight::frame));
    std::array<uint8_t,512> saved{};std::memcpy(saved.data(),host_log,512);
    std::array<uint8_t,4096> nvm{};std::memcpy(nvm.data(),host_nvm,4096);
    flight_reset();std::memset(authority_observation::history,0,8);authority_observation::terminal_fault=0;
    bus_started=false;g_adc_dma_inited=false;host_dma1.CFGR=0;
    host_time_hook=[](){++host_clock;if(log_tx.size()>=536)throw LoopComplete{};};
    try{firmware_main();CHECK(false);}catch(const LoopComplete&){}
    CHECK(!bus_started&&!g_adc_dma_inited&&!host_dma1.CFGR&&!std::memcmp(saved.data(),host_log,512)&&!std::memcmp(nvm.data(),host_nvm,4096));
    CHECK(log_tx.size()==536&&!std::memcmp(log_tx.data()+20,saved.data(),512));
    return 0;
}
#else
static void host_input_detail_barrier(const float*,unsigned){}
static void host_input_detail_read(int){}
static void host_input_detail_hook(int){}
#endif
