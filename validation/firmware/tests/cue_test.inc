// Observer extensions. All gesture, renderer, ADC/filter and storage decisions remain production bodies.
#include "cue_schedules.h"
#include <cstdlib>
static void zeros();
static bool cue_parked=false;
static const CueSchedule* cue_schedule=nullptr;
static uint64_t cue_window=0,cue_closed=0,cue_first_sample=0,cue_first_blue=0,cue_last_center=0,cue_fault=0;
static unsigned cue_centers=0,cue_entries=0,cue_samples=0;
static std::array<float,4> cue_center_values{};
static bool cue_case(){return scenario.rfind("cue-",0)==0;}
static void host_center_observe(unsigned ch,float value){
    if(release_case()){CHECK(ch==release_centers&&ch<4);++release_centers;return;}
    if(!cue_case())return;
    CHECK(ch==cue_centers&&ch<4);cue_center_values[ch]=value;++cue_centers;cue_last_center=host_clock;
}
static void host_window_observe(uint64_t start){
#if BMCU_LITE_V11_FEATURES
    if(maintenance)cue_window=start;
#endif
    if(release_case()){CHECK(release_centers==4&&!release_window);release_window=start;return;}
    if(!cue_case())return;
    CHECK(cue_centers==4&&cue_last_center<=start);CHECK(++cue_entries==1);cue_window=start;
}
static void host_gesture_sample(){
    if(release_case()){
        if(!release_samples++){
            CHECK(release_centers==4&&release_window&&host_clock>=release_window);
            for(int ch=-1;ch<4;++ch){const auto* led=ch<0?&SYS_RGB:&RGBOUT[ch];bool found=false;
                for(auto it=deliveries.rbegin();it!=deliveries.rend();++it)if(it->led==led){CHECK(it->rgb[0]==0x18u);if(ch>=0)CHECK(it->rgb[1]==0x18u);found=true;break;}CHECK(found);}
        }
        if(HOST_EXPECT_RELEASE_CUE&&accepted_tick&&!time_hw_slow&&host_clock>=accepted_tick+2500000&&!release_expiry_checked){
            for(int ch=-1;ch<4;++ch){const auto* led=ch<0?&SYS_RGB:&RGBOUT[ch];bool found=false;
                for(auto it=deliveries.rbegin();it!=deliveries.rend();++it)if(it->led==led){CHECK(it->rgb[0]==0x1800u);if(ch>=0)CHECK(it->rgb[1]==0x1800u);found=true;break;}CHECK(found);}
            release_expiry_checked=true;
        }
        return;
    }
    if(!cue_case())return;
    ++cue_samples;
    if(is("cue-park-reentry")){
        cue_closed=host_clock;cue_parked=true;host_clock_calls_before_exit=20;
        try{setup_trial_boot();CHECK(false);}catch(const LoopComplete&){}
        host_clock_calls_before_exit=0;throw LoopComplete{};
    }
#ifdef HOST_OUTCOME_ONLY
    return;
#endif
    if(!cue_first_sample){
        cue_first_sample=host_clock;
        CHECK(cue_centers==4&&cue_window&&cue_first_sample-cue_window<3000);
        // Actual completed transmissions, before any scheduled owner movement.
        for(int ch=-1;ch<4;++ch){
            const auto* led=ch<0?&SYS_RGB:&RGBOUT[ch];bool found=false;
            for(auto it=deliveries.rbegin();it!=deliveries.rend();++it)if(it->led==led){
                CHECK(it->phase==SetupPhase::gesture&&it->tick>=cue_window&&it->rgb[0]==0x18);
                if(ch>=0)CHECK(it->rgb[1]==0x18);found=true;break;
            }CHECK(found);
        }
    }
}
static void host_release_observe(){if(release_case()){CHECK(release_window&&!release_closed);release_closed=host_clock;}if(cue_case()){CHECK(cue_window&&!cue_closed);cue_closed=host_clock;}}
static void host_cal_observe(){
    if(release_case()){
        release_cal=host_clock;CHECK(release_closed&&release_cal>=release_closed+1000000);
        CHECK(leds[&SYS_RGB][0]==0);for(auto& led:RGBOUT)CHECK(leds[&led][0]==0&&leds[&led][1]==0);return;
    }
    if(!cue_case())return;
    CHECK(leds[&SYS_RGB][0]==0);
    for(auto& led:RGBOUT)CHECK(leds[&led][0]==0&&leds[&led][1]==0);
}
static void host_fault_observe(){
    if(release_case()){
        if(!fault_tick)fault_tick=host_clock;
        CHECK(leds[&SYS_RGB][0]==0);for(auto& led:RGBOUT)CHECK(leds[&led][0]==0&&leds[&led][1]==0);return;
    }
    if(!cue_case())return;
    if(!cue_fault)cue_fault=host_clock;
#ifdef HOST_OUTCOME_ONLY
    return;
#endif
    // No stale blue/green is left in any real WS2812 queue at first-fault transition.
    CHECK(leds[&SYS_RGB][0]==0);
    for(auto& led:RGBOUT)CHECK(leds[&led][0]==0&&leds[&led][1]==0);
}
static void cue_queue_observe(const WS2812_class*,uint32_t rgb){
    if(setup_trial_phase()==SetupPhase::gesture&&rgb==0x18){
        CHECK(cue_centers==4&&cue_window&&host_clock>=cue_window&&!cue_closed&&!cue_fault);
        if(!cue_first_blue)cue_first_blue=host_clock;
    }
}
static void cue_input(SetupPhase mode,uint64_t ms){
    if(!cue_schedule)for(const auto& s:cue_schedules)if(scenario==s.name)cue_schedule=&s;
    CHECK(cue_schedule);
    if(mode!=SetupPhase::inspect&&mode!=SetupPhase::gesture)return;
    for(unsigned ch=0;ch<4;++ch)adc[6-2*ch]=cue_schedule->baseline[ch]*(3.3f/4095.f);
    if(mode!=SetupPhase::gesture)return;
    const auto elapsed=cue_window&&host_clock>=cue_window?(host_clock-cue_window)/1000:0;
    if(cue_window)for(unsigned i=0;i<cue_schedule->count;++i){const auto& s=cue_schedule->segments[i];
        if(elapsed>=s.begin&&elapsed<s.end)adc[6-2*s.channel]=s.raw*(3.3f/4095.f);
    }
    if(is("cue-queued-fault")&&cue_window&&elapsed>=2250)time_hw_slow=1;
    const std::string stage=cue_schedule->stage;
    const bool active=stage=="baseline"?gesture_start&&ms>=gesture_start+400:
        stage=="window"?cue_window&&elapsed>=1000:stage=="ack"&&cue_window&&elapsed>=2600;
    if(active){const std::string why=cue_schedule->fault;
        if(why=="adc")adc[5]=NAN;
        if(why=="key")adc[3]=1.8f;
        if(why=="storage")g_flash_fault=true;
        if(why=="clock")time_hw_tpms=1001;
    }
}
static int cue_verify(){
    CHECK(cue_schedule);const auto expected=*cue_schedule;
    CHECK(unsigned(fault)==expected.parent&&boot_admission_detail==expected.detail);
    CHECK(gesture_ack_channel==0xff&&gesture_ack_start==0&&!bus_started);
    CHECK(bool(queued_green)==expected.ack);
#ifdef HOST_OUTCOME_ONLY
    CHECK(cue_entries==(std::string(expected.stage)=="baseline"?0u:1u));
    if(cue_parked){CHECK(phase==SetupPhase::gesture&&!cal_start&&!host_programs&&!host_erases);for(auto n:drives)CHECK(!n);zeros();}
    else if(expected.parent){CHECK(phase==SetupPhase::fault&&!cal_start&&!host_programs&&!host_erases);for(auto n:drives)CHECK(!n);zeros();}
    else{CHECK(phase==SetupPhase::success&&cal_start&&host_programs==60&&host_erases==2&&cal_retained()&&Motion_control_setup_retained());for(auto n:drives)CHECK(n);zeros();const auto s=Flash_setup_probe();CHECK(s.journal_valid&&s.owner==0xff);}
    if(expected.parent==9)CHECK(cue_fault-cue_window>=15000000&&cue_fault-cue_window<15003000);
    std::printf("BASELINE-OUTCOME %s parent=%u detail=%u ACK=%u programs=%u erases=%u\n",scenario.c_str(),unsigned(fault),unsigned(boot_admission_detail),bool(queued_green),host_programs,host_erases);
    return 0;
#endif
    const bool baseline=std::string(expected.stage)=="baseline";
    CHECK(cue_entries==(baseline?0u:1u));
    bool magenta=false;std::array<uint32_t,4> prior{};std::array<unsigned,4> greens{};std::vector<uint64_t> edges;
    unsigned selected_ch=expected.count?expected.segments[0].channel:2;
    for(const auto& e:deliveries){
        if(e.phase==SetupPhase::gesture){
            if(e.tick<cue_window||!cue_window){CHECK(e.rgb[0]==0x180018);magenta=true;}
            if(e.rgb[0]==0x18){CHECK(cue_window&&e.tick>=cue_window&&e.tick-cue_window<15000000);CHECK(!cue_closed||e.tick<cue_closed);CHECK(!cue_fault||e.tick<cue_fault);}
            for(unsigned ch=0;ch<4;++ch)if(e.led==&RGBOUT[ch]){
                CHECK(e.rgb[0]==e.rgb[1]);const auto rgb=e.rgb[0];
                if(ch==selected_ch&&accepted_tick&&e.tick>=accepted_tick&&e.tick-accepted_tick<2000000){
                    const uint32_t want=(e.tick-accepted_tick)%1000000<500000?0x1800u:0u;
                    CHECK(rgb==want); // Global blue/magenta cannot fill either ACK dark gap.
                }
                if(rgb==0x1800&&prior[ch]!=rgb)++greens[ch];
                if(ch==selected_ch&&rgb!=prior[ch]&&(rgb==0x1800||prior[ch]==0x1800))edges.push_back(e.tick);
                prior[ch]=rgb;
            }
        }
        if(cue_fault&&e.tick>=cue_fault){CHECK(e.phase!=SetupPhase::gesture);CHECK(e.rgb[0]==setup_trial_led_rgb(e.tick));}
        if(e.phase==SetupPhase::calibration&&e.led!=&SYS_RGB)CHECK(e.rgb[0]!=0x1800&&e.rgb[1]!=0x1800);
    }
    CHECK(magenta);
    if(!baseline){
        CHECK(cue_first_blue>=cue_last_center&&cue_first_blue>=cue_window&&cue_first_blue-cue_window<3000&&cue_samples);
        for(unsigned ch=0;ch<4;++ch)CHECK(std::fabs(cue_center_values[ch]-expected.baseline[ch]*(3.3f/4095.f))<.00001f);
        // A steady color stays latched between deliveries. Check its completed cutoff,
        // rather than treating the absence of another blue write as optical cleanup.
        const uint64_t end=cue_closed?cue_closed:cue_fault;
        for(int ch=-1;ch<4;++ch){
            const auto* led=ch<0?&SYS_RGB:&RGBOUT[ch];bool found=false;
            uint32_t latched=0;
            for(const auto& e:deliveries)if(e.led==led&&e.tick<end)latched=e.rgb[0];
            if(cue_closed&&latched!=0x18)found=true; // ACK may already be green/dark at close.
            if(!found)for(const auto& e:deliveries)if(e.led==led&&e.tick>=end&&e.rgb[0]!=0x18){
                CHECK(e.tick-end<3000);found=true;break;
            }CHECK(found);
        }
    }
    for(unsigned ch=0;ch<4;++ch)if(ch!=selected_ch)CHECK(!greens[ch]);
    const bool full_ack=expected.ack&&expected.parent!=5&&std::string(expected.stage)!="ack";
    if(full_ack){CHECK(greens[selected_ch]==2&&edges.size()==4);for(unsigned i=0;i<4;++i){const auto d=int64_t(edges[i]-accepted_tick)-int64_t(i)*500000;CHECK(d>=0&&d<3000);}}
    if(is("cue-queued-fault"))CHECK(greens[selected_ch]==0&&accepted_tick);
    if(cue_parked){CHECK(phase==SetupPhase::gesture&&!cal_start&&!host_programs&&!host_erases);CHECK(setup_trial_led_rgb(host_clock)==0x180018);for(auto n:drives)CHECK(!n);zeros();}
    else if(expected.parent){
        CHECK(phase==SetupPhase::fault&&!cal_start&&!host_programs&&!host_erases);for(auto n:drives)CHECK(!n);zeros();
        if(expected.parent==9){CHECK(cue_fault-cue_window>=15000000&&cue_fault-cue_window<15003000);}
    }else{
        CHECK(phase==SetupPhase::success&&cue_closed&&cal_start&&host_programs>40&&host_erases==2);
        CHECK(cue_closed-cue_window<15000000);CHECK(cal_start*1000>=cue_closed+1000000);
        for(auto n:drives)CHECK(n);zeros();CHECK(cal_retained()&&Motion_control_setup_retained());
        const auto s=Flash_setup_probe();CHECK(s.journal_valid&&s.owner==0xff);
        if(is("cue-late-release"))CHECK(cal_start*1000>cue_window+15000000);
    }
    if(is("cue-delayed-admission")){
        CHECK(uint64_t(gesture_start)*1000<origin+2000000&&cue_window>origin+2000000);
        for(int ch=-1;ch<4;++ch){
            const auto* led=ch<0?&SYS_RGB:&RGBOUT[ch];bool found=false;
            for(auto it=deliveries.rbegin();it!=deliveries.rend();++it)if(it->led==led&&it->tick<=origin+2000000){CHECK(it->rgb[0]==0x180018);found=true;break;}
            CHECK(found);
        }
        const auto saved=phase;const auto saved_fault=fault;phase=SetupPhase::gesture;fault=SetupFault::none;
        CHECK(setup_trial_led_rgb(origin+2000000)==0x180018);phase=saved;fault=saved_fault;
    }
    const auto captured=fault;const auto detail=boot_admission_detail;const auto entries=cue_entries;const auto before=host_programs;
    host_time_hook=nullptr;host_delay_hook=nullptr;time_hw_slow=0;time_hw_tpms=scale;
    // Re-enter the real selected-boot PARK three times; no second gesture entry.
    for(unsigned i=0;i<3;++i){host_clock_calls_before_exit=20;try{setup_trial_boot();CHECK(false);}catch(const LoopComplete&){}host_clock_calls_before_exit=0;}
    CHECK(cue_entries==entries&&fault==captured&&boot_admission_detail==detail&&host_programs==before&&!bus_started);
    if(expected.parent){setup_trial_fail(SetupFault::capability);CHECK(fault==captured&&boot_admission_detail==detail);display(true);}
    const char* trace_dir=std::getenv("CUE_TRACE_DIR");CHECK(trace_dir);
    const std::string path=std::string(trace_dir)+"/"+scenario+".json";
    FILE* trace=std::fopen(path.c_str(),"w");CHECK(trace);
    std::fprintf(trace,"{\"case\":\"%s\",\"window_tick\":%llu,\"last_center_tick\":%llu,\"first_blue_tick\":%llu,\"first_sample_tick\":%llu,\"closed_tick\":%llu,\"fault_tick\":%llu,\"events\":[",scenario.c_str(),(unsigned long long)cue_window,(unsigned long long)cue_last_center,(unsigned long long)cue_first_blue,(unsigned long long)cue_first_sample,(unsigned long long)cue_closed,(unsigned long long)cue_fault);
    bool first=true;for(const auto& e:deliveries){int ch=-1;for(int i=0;i<4;++i)if(e.led==&RGBOUT[i])ch=i;
        std::fprintf(trace,"%s{\"tick\":%llu,\"channel\":%d,\"phase\":%u,\"rgb\":[%u,%u]}",first?"":",",(unsigned long long)e.tick,ch,unsigned(e.phase),e.rgb[0],e.rgb[1]);first=false;}
    std::fprintf(trace,"]}\n");std::fclose(trace);
    std::printf("CUE %s centers=%u window=%llu blue=%llu first-sample=%llu closed=%llu fault-tick=%llu cal-ms=%llu ACK-edges=%zu polls=%u max-gap=%llu raw-samples=%u PARK-entries=%u\n",scenario.c_str(),cue_centers,(unsigned long long)cue_window,(unsigned long long)cue_first_blue,(unsigned long long)cue_first_sample,(unsigned long long)cue_closed,(unsigned long long)cue_fault,(unsigned long long)cal_start,edges.size(),cue_samples,(unsigned long long)max_gesture_poll_gap,sample_events,cue_entries);
    return 0;
}
