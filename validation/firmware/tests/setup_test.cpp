#include <host_hal.h>
#include <host_flash.h>
#include <array>
#include <map>
#include <string>
#include <vector>
#include <cstring>
#include <cmath>
#include <algorithm>
static uint8_t host_active_request_phase=0;
static uint8_t host_uid[12]={1,2,3,4,5,6,7,8,9,10,11,12};
static void host_input_detail_hook(int);
#if BMCU_USB_CONTINUOUS_JOURNAL
static int journal_case();
static int retention_case();
#if BMCU_USB_WORKING_MODE
static void journal_drain(unsigned);
#if BMCU_USB_ISP_EXPORT
static void working_marker_prepare();
static int marker_case();
#endif
static void working_bootstrap();
static void working_export_hook();
static bool working_export_boot();
static int working_case(const char*);
static int working_runtime(const char*);
#endif
static void journal_drain(unsigned);
#if BMCU_USB_ISP_EXPORT
static void working_marker_prepare();
static int marker_case();
#endif
static bool retained_event(uint8_t,uint8_t,uint32_t);
#endif
static void host_input_detail_read(int);
static void host_input_detail_barrier(const float*,unsigned);
#include "_bus_hardware.cpp"
#include "bambu_bus_ams.cpp"
static void host_adc_poll_observe();
#include "ADC_DMA.cpp"
static void host_admission_observe(int);
static void host_pwm_observe(unsigned,int);
static unsigned host_meter_reads=0,host_onuse_meter_reads=0;
static bool host_meter_trace=false;
#include "Motion_control.cpp"
#include "MC_PULL_calibration.cpp"
#if BMCU_LITE_FULL_LOAD
#define MC_PULL_calibration_load_retained_trial MC_PULL_calibration_load_retained_full
#endif
#define main firmware_main
#include "main.cpp"
#undef main
#include "single_channel_trial.cpp"
#include "Flash_saves.cpp"
#include "full_load.cpp"
#include "authority_observation.cpp"
#if BMCU_USB_FLIGHT_RECORDER
#include "flight_hal.h"
#include "flight_recorder.cpp"
#endif
static void host_center_observe(unsigned,float);
static void host_window_observe(uint64_t);
static void host_gesture_sample();
static void host_release_observe();
static void host_fault_observe();
static void host_cal_observe();
#include "setup_trial.cpp"
static void host_ws_queued(const WS2812_class*,uint32_t,uint8_t);
static void host_ws_delivered(const WS2812_class*,const uint32_t*,uint8_t);
#include "ws2812.cpp"
#include "startup_schedules.h"
#include "gesture_schedules.h"
static std::string scenario;
static const StartupSchedule* schedule=nullptr;
static const GestureSchedule* detail_schedule=nullptr;
static unsigned sample_events=0,admission_calls=0;
static uint64_t last_sample_ms=0;
static void raw_dma_boundary();
static float adc[8]={1.65f,.20f,1.65f,.20f,1.65f,.20f,1.65f,.20f};
static bool adc_ready=true, seen_drive=false, injected=false, finished=false, freeze_encoder=false;
static unsigned drives[4]={},bus_enables=0;
static int host_pwm_value=0;
static uint64_t gesture_start=0,cal_start=0,direction_start=0,park_start=0;
static uint64_t prompts[4][2]={};
static std::map<const WS2812_class*,std::array<uint32_t,2>> leds;
struct LedEvent {uint64_t tick;const WS2812_class* led;std::array<uint32_t,2> rgb;SetupPhase phase;};
static std::vector<LedEvent> deliveries;
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
static unsigned diagnostic_green_deliveries=0,diagnostic_amber_deliveries=0;
static bool full_stopped();
#endif
static uint64_t accepted_tick=0,fault_tick=0,previous_gesture_poll=0,max_gesture_poll_gap=0;
static unsigned queued_green=0,gesture_polls=0;
#ifndef HOST_EXPECT_RELEASE_CUE
#define HOST_EXPECT_RELEASE_CUE 0
#endif
static bool release_case(){return scenario.rfind("release-",0)==0;}
static uint64_t release_window=0,release_closed=0,release_cal=0,release_control_digest=1469598103934665603ull;
static unsigned release_centers=0,release_samples=0;
static std::vector<LedEvent> release_queue;
static std::map<const WS2812_class*,std::pair<uint32_t,SetupPhase>> release_queue_last;
static bool release_expiry_checked=false,release_final_stable_checked=false;
static void release_control_sample();
static int release_verify();
static bool continuous_fresh(){return scenario.rfind("continuous-fresh-",0)==0;}
static bool any_case(){return scenario.rfind("any-",0)==0||continuous_fresh();}
static bool any_success(){return continuous_fresh()||scenario=="detail-normal"||scenario=="any-0"||scenario=="any-1"||scenario=="any-2"||scenario=="any-3"||scenario=="any-baseline"||scenario=="any-retry";}
static unsigned gesture_ch(){return !continuous_fresh()&&any_success()&&scenario!="any-retry"&&scenario!="any-baseline"?unsigned(scenario.back()-'0'):2u;}
static bool is(const char* s){return scenario==s||(continuous_fresh()&&scenario==std::string("continuous-fresh-")+s);}
static unsigned channel(int t,int c){return t==TIM2?3:t==TIM3?2:c<2?1:0;}
static void compare(int t,int c,uint16_t v){
    const unsigned ch=channel(t,c);
    const int other=compare_values[t][c^1];
    if(v!=other && v!=compare_values[t][c]){
#if BMCU_LITE_FULL_LOAD
        if(setup_trial_phase()==SetupPhase::operation){
            CHECK(v<=1000);if(host_pwm_value){CHECK(
#if BMCU_LITE_V11_FEATURES
                v11_local_drive_allowed(ch) ||
#endif
                (full_load_durable(ch)&&dispatching));++drives[ch];}
        } else
#endif
        if(setup_trial_phase()==SetupPhase::trial){CHECK(ch==0);CHECK(v<=1000);}
        else {
            CHECK(setup_trial_phase()==SetupPhase::direction || v==0);
            if(v && !other){CHECK(!setup_direction_pwm_call);CHECK(setup_trial_fault()==SetupFault::none);++drives[ch];seen_drive=true;}
        }
    }
}
#include "cue_test.inc"
static bool full_case(){return scenario.rfind("full-",0)==0
#if BMCU_LITE_CONTINUOUS_NATIVE
    ||(continuous_fresh()&&phase==SetupPhase::operation)
#endif
    ;}
static void full_input();
static uint8_t full_encoder_valid=15;
static void full_boot_seed(const char*);
static int full_verify(const char*);
static void authority_pwm(unsigned,int);
static void observation_pwm(unsigned,int);
static void observation_ws(const WS2812_class*,const std::array<uint32_t,2>&);
static void host_pwm_observe(unsigned ch,int pwm){
    host_pwm_value=pwm;
#if BMCU_LITE_V11_FEATURES
    authority_pwm(ch,pwm);
    observation_pwm(ch,pwm);
#endif
#if BMCU_LITE_FULL_LOAD
    if(setup_trial_phase()==SetupPhase::operation && pwm){CHECK(
#if BMCU_LITE_V11_FEATURES
                v11_local_drive_allowed(ch) ||
#endif
                (full_load_durable(ch)&&dispatching));CHECK(!full_load_journal_pending());}
#endif
}
static void input(){
    const auto mode=setup_trial_phase();const uint64_t ms=host_clock/1000;
    if(mode==SetupPhase::gesture && !gesture_start)gesture_start=ms;
    if(mode==SetupPhase::calibration && !cal_start)cal_start=ms;
    if(mode==SetupPhase::direction && !direction_start)direction_start=ms;
    for(int ch=0;ch<4;++ch){adc[6-2*ch]=1.65f;adc[7-2*ch]=.20f;}
    // These fixtures describe a baseline already present before admission.
    if(mode==SetupPhase::inspect){
        if(is("any-baseline"))adc[6-2*gesture_ch()]+=.15f;
        if(is("any-preheld"))adc[6-2*gesture_ch()]-=.20f;
    }
    if(is("filament"))adc[7]=1.8f;
    if(is("unknown"))adc[5]=NAN;
    if(is("missing-module") || is("ready-missing-module"))adc[2]=0;
    if(is("adc-stale"))adc_ready=false;
    if(is("uncentered"))adc[0]=1.95f;
    if(is("unstable") && host_clock/1000%200<100)adc[2]+=.06f;
    if(mode==SetupPhase::trial && scenario.rfind("trial-",0)==0)adc[7]=1.8f;
    if(mode==SetupPhase::gesture && gesture_start){
        auto elapsed=ms-gesture_start;
#if BMCU_LITE_V11_FEATURES
        if(maintenance)elapsed=cue_window?ms-cue_window/1000+1500:0;
#endif
        if(detail_schedule){
            for(unsigned i=0;i<detail_schedule->count;++i){const auto& part=detail_schedule->segments[i];
                if(elapsed>=part.begin&&elapsed<part.end)adc[6-2*detail_schedule->channel]+=part.delta;
            }
            if(elapsed>=detail_schedule->fault_ms){
                const std::string why=detail_schedule->fault;
                if(why=="adc")adc[5]=NAN; // Invalid raw packed DMA samples, not a Boolean ADC stub.
                if(why=="filament")adc[3]=1.8f;
                if(why=="storage")g_flash_fault=true;
                if(why=="clock")time_hw_tpms=1001;
                if(release_case()&&why=="queued")adc[3]=1.8f;
            }
            if(release_case()&&std::string(detail_schedule->fault)=="queued"&&elapsed>=2700)time_hw_slow=1;
        }else if(any_case()){
            const unsigned ch=gesture_ch();
            if(is("any-baseline"))adc[6-2*ch]+=.15f;
            if(is("any-preheld"))adc[6-2*ch]-=.20f;
            else if(is("any-preheld-invalid"))adc[6-2*ch]-=.40f;
            else if(!is("any-none")){
                uint64_t begin=is("any-late")?15000:2000;
                uint64_t end=is("any-short")?2300:(is("any-held")||is("any-late"))?30000:4400;
                if(is("any-retry")){
                    if((elapsed>=2000&&elapsed<2300)||(elapsed>=3000&&elapsed<5400))adc[6-2*ch]-=.20f;
                }else if(is("any-switch")){
                    if(elapsed>=2000&&elapsed<2300)adc[6-2*ch]-=.20f;
                    if(elapsed>=3000)adc[4]-=.20f;
                }else if(elapsed>=begin&&elapsed<end)adc[6-2*ch]+=(is("any-1")||is("any-3"))?.20f:-.20f;
                if((is("any-multiple")&&elapsed>=2000)||(is("any-multiple-after")&&elapsed>=3000))adc[4]-=.20f;
            }
            if(is("any-release-unstable")&&elapsed>=5700&&elapsed<5850)adc[0]+=.06f;
            const bool fail=elapsed>=3000;
            if(is("any-key")&&fail)adc[3]=1.8f;
            if(is("any-adc")&&fail)adc[5]=NAN;
            // any-adc-stale freezes real DMA completion below.
            if(is("any-storage")&&fail)g_flash_fault=true;
            if(is("any-busy")&&fail)FLASH->STATR|=FLASH_FLAG_BSY;
            if(is("any-module")&&fail)filament_channel_inserted[1]=false;
            if(is("any-clock-back")&&fail&&!injected){host_clock-=100000;injected=true;}
            if(is("any-clock-scale")&&fail)time_hw_tpms=1001;
            if(elapsed>=4300){
                if(is("any-ack-clock"))time_hw_tpms=1001;
                if(is("any-ack-adc"))adc[5]=NAN; // Raw packed DMA fault, not the unused legacy Boolean.
                if(is("any-ack-storage"))g_flash_fault=true;
            }
            if(is("any-fault-after-ack")&&elapsed>=5100)adc[3]=1.8f;
            if(is("any-queued-fault")){if(elapsed>=3900)time_hw_slow=1;if(elapsed>=4400)adc[3]=1.8f;}
        }else if(!cue_case()&&!is("gesture-timeout")){
            uint64_t end=is("gesture-short")?2300:4400;
            if(is("gesture-held"))end=30000;
            if(elapsed>=2000 && elapsed<end)adc[6]-=.20f;
        }
        if(is("gesture-key") && elapsed>2200)adc[3]=1.8f;
    }
    if(cue_case())cue_input(mode,ms);
    if(full_case())full_input();
    if(mode==SetupPhase::calibration && !is("cal-timeout")){
        for(int ch=0;ch<4;++ch)for(int k=0;k<2;++k)if(prompts[ch][k]){
            const auto elapsed=ms-prompts[ch][k];
            if(elapsed<500)adc[6-2*ch]+=(k?.30f:-.30f);
            if(is("cal-invalid") && !k && elapsed<300)adc[6-2*ch]=.01f;
        }
    }
    if(mode==SetupPhase::direction && seen_drive){
        if(is("direction-busy"))FLASH->STATR|=FLASH_FLAG_BSY;
        if(is("direction-direct") && !injected){injected=true;Motion_control_set_PWM(3,500);}
        if(is("direction-key"))adc[1]=1.8f;
        if(!injected && is("direction-clock-back")){host_clock-=100000;injected=true;}
        if(!injected && is("direction-clock-scale")){time_hw_tpms=1001;injected=true;}
    }
    if(mode==SetupPhase::direction && is("motion-error"))host_fail_program=true;
    if(mode==SetupPhase::calibration && is("cal-key") && prompts[0][0])adc[5]=1.8f;
    if(mode==SetupPhase::journal && is("journal-error"))host_fail_program=true;
    if(mode==SetupPhase::journal && is("journal-busy"))host_hang=true;
    if(mode==SetupPhase::calibration && is("program-error") && cal_start && ms-cal_start>10000)host_fail_program=true;
#if BMCU_USB_FLIGHT_RECORDER
    if(release_case())release_control_sample();
#endif
}
#ifdef HOST_ENCODER_DRIVER
#include "encoder_driver_model.inc"
#else
AS5600_soft_IIC_many::AS5600_soft_IIC_many(){online=online_buf;magnet_stu=magnet_buf;raw_angle=raw_buf;for(int i=0;i<4;++i){online[i]=true;magnet_stu[i]=normal;raw_angle[i]=2048;}}
AS5600_soft_IIC_many::~AS5600_soft_IIC_many(){}
void AS5600_soft_IIC_many::init(GPIO_TypeDef*const*,const uint16_t*,GPIO_TypeDef*const*,const uint16_t*,int n){numbers=n;if(is("cue-delayed-admission"))delay(600);}
void AS5600_soft_IIC_many::updata_stu(){host_input_detail_read(7);}
uint8_t AS5600_soft_IIC_many::updata_angle(){
    host_input_detail_read(6);
    input();static uint64_t previous=0;const auto ms=host_clock/1000;const auto dt=ms-previous;previous=ms;
    if(is("any-ack-encoder")&&setup_trial_phase()==SetupPhase::gesture&&gesture_start&&host_clock/1000-gesture_start>=4300)return 14;
    if(is("any-encoder")&&setup_trial_phase()==SetupPhase::gesture&&gesture_start&&host_clock/1000-gesture_start>=3000)return 14;
    if(is("encoder-entry") || is("ready-encoder") || (is("cal-encoder") && prompts[0][0]))return 7;
    if(is("direction-encoder") && seen_drive)return 14;
    if(!is("direction-timeout") && !freeze_encoder)for(int ch=0;ch<4;++ch){
        int t=ch==3?TIM2:ch==2?TIM3:TIM4;int c=ch==0?2:0;
        if(compare_values[t][c]!=compare_values[t][c+1])raw_angle[ch]=(raw_angle[ch]+int(dt)*20*(ch%2?-1:1))&4095;
    }
    if(full_case())return full_encoder_valid;
    return 15;
}
#endif
static void host_ws_queued(const WS2812_class* led,uint32_t rgb,uint8_t i){
    CHECK(i<2);leds[led][i]=rgb;
    if(release_case()&&i==0){const auto v=std::make_pair(rgb,setup_trial_phase());auto it=release_queue_last.find(led);
        if(it==release_queue_last.end()||it->second!=v){release_queue.push_back({host_clock,led,{rgb,rgb},setup_trial_phase()});release_queue_last[led]=v;}}

    if(cue_case())cue_queue_observe(led,rgb);
    if(setup_trial_phase()==SetupPhase::gesture&&led!=&SYS_RGB&&rgb==0x1800){++queued_green;CHECK(gesture_ack_channel<4);if(!continuous_fresh()&&(!release_case()||!HOST_EXPECT_RELEASE_CUE))CHECK(led==&RGBOUT[gesture_ack_channel]);if(!accepted_tick)accepted_tick=gesture_ack_start;}
    if(setup_trial_phase()==SetupPhase::calibration&&i==1)for(int ch=0;ch<4;++ch)if(led==&RGBOUT[ch]){
        if(rgb==0x10&&!prompts[ch][0])prompts[ch][0]=host_clock/1000;
        if(rgb==0x100000&&!prompts[ch][1])prompts[ch][1]=host_clock/1000;
    }
}
static void host_ws_delivered(const WS2812_class* led,const uint32_t* grb,uint8_t n){
    std::array<uint32_t,2> rgb{};for(unsigned i=0;i<n;++i)rgb[i]=((grb[i]&0xff00)<<8)|((grb[i]&0xff0000)>>8)|(grb[i]&0xff);
    deliveries.push_back({host_clock,led,rgb,setup_trial_phase()});
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
    if(setup_trial_phase()==SetupPhase::operation&&led==&SYS_RGB){
        if(rgb[0]==0x002000u){CHECK(flight::saved()&&flight::complete(host_log)&&full_stopped()&&!bus_started);++diagnostic_green_deliveries;}
        if(rgb[0]==0x201000u){CHECK((flight::operator_pending()
#if BMCU_LITE_LEASE_FEEDBACK
        ||full_load_fault()!=FullLoadFault::none
#endif
        )&&!flight::saved()&&full_stopped()&&!bus_started);++diagnostic_amber_deliveries;}
    }
#endif
#if BMCU_LITE_V11_FEATURES
    observation_ws(led,rgb);
#endif
}
static void host_admission_observe(int stage){
    ++admission_calls;
    std::printf("ADMISSION stage%d adc_age_ms=%llu ready=%d inited=%d gen=%u blocks=%u flags=%u/%u/%u/%u\n",stage,(unsigned long long)((host_clock-host_adc_start)/1000),ADC_DMA_ready(),ADC_DMA_is_inited(),ADC_DMA_generation(),g_blocks_filled,filament_channel_inserted[0],filament_channel_inserted[1],filament_channel_inserted[2],filament_channel_inserted[3]);std::fflush(stdout);
}
static void host_adc_poll_observe(){
    if(setup_trial_phase()==SetupPhase::gesture){++gesture_polls;if(previous_gesture_poll&&host_clock>=previous_gesture_poll)max_gesture_poll_gap=std::max(max_gesture_poll_gap,host_clock-previous_gesture_poll);previous_gesture_poll=host_clock;CHECK(!bus_started);
#if BMCU_LITE_V11_FEATURES
        if(!maintenance)
#endif
        {CHECK(!host_programs&&!host_erases);for(auto n:drives)CHECK(!n);}}
}
static void raw_dma_boundary(){
    if(is("any-adc-stale")&&phase==SetupPhase::gesture&&gesture_start&&host_clock/1000>=gesture_start+3000)return;
    if(!(host_dma1.CFGR&1u)||!host_adc1.started||!host_adc2.started||!host_adc_start)return;
    const uint64_t ms=(host_clock-host_adc_start)/1000;
    if(ms<=last_sample_ms)return;last_sample_ms=ms;
    if(schedule&&ms<schedule->activity)return;
    if(schedule&&schedule->stream==1){DMA1->INTFR|=DMA1_FLAG_TE1;return;}
    const unsigned half=sample_events++%2;
    if(!schedule||schedule->stream!=2){
        input();auto*buf=reinterpret_cast<volatile uint32_t*>(host_dma1.MADDR);
        for(unsigned row=0;row<32;++row)for(unsigned idx=0;idx<8;++idx){
            float v=adc[idx];
            if(schedule&&idx%2==1)v=schedule->key_v;
            if(schedule&&idx%2==0&&(ms<schedule->in_range||int(3-idx/2)==schedule->bad))v=0;
            uint16_t raw=std::isfinite(v)?uint16_t(std::max(0.f,std::min(3.3f,v))*(4095.f/3.3f)+.5f):0xffffu;
            buf[half*256+row*8+idx]=uint32_t(raw)|(uint32_t(raw)<<16);
        }
    }
    DMA1->INTFR|=half?DMA1_FLAG_TC1:DMA1_FLAG_HT1;
}
static void time_hook(){
    ++host_clock;raw_dma_boundary();
#if BMCU_USB_WORKING_MODE
    working_bootstrap();
#endif
    if(continuous_fresh()&&((scenario.find("cut-calibration")!=std::string::npos&&phase==SetupPhase::calibration&&prompts[0][0]) || (scenario.find("cut-direction")!=std::string::npos&&phase==SetupPhase::direction&&seen_drive) || (scenario.find("cut-journal")!=std::string::npos&&phase==SetupPhase::journal)))throw PowerCut{};
    if(cue_case()&&setup_trial_phase()==SetupPhase::fault){if(!fault_tick)fault_tick=host_clock;if(is("cue-queued-fault"))time_hw_slow=0;}
    if(schedule&&setup_trial_phase()==SetupPhase::gesture)throw LoopComplete{};
#if BMCU_LITE_FULL_LOAD
    if(full_case()&&setup_trial_phase()==SetupPhase::operation&&bus_started
#if BMCU_USB_WORKING_MODE
       && !bus_passive
#endif
    )throw LoopComplete{};
#endif
    if(release_case()&&setup_trial_phase()==SetupPhase::fault){if(!fault_tick)fault_tick=host_clock;time_hw_slow=0;}
    if(any_case()&&setup_trial_phase()==SetupPhase::fault){if(!fault_tick)fault_tick=host_clock;if(is("any-queued-fault"))time_hw_slow=0;}

    if(!finished &&
#if BMCU_LITE_CONTINUOUS_NATIVE
       (!continuous_fresh()||(flight::report.flags&2u)) &&
#endif
       (setup_trial_phase()==SetupPhase::fault || setup_trial_phase()==SetupPhase::success)){
        if(!park_start)park_start=host_clock;
        if(host_clock-park_start>(is("full-journal-setup-fault")?100000u:10000u))throw LoopComplete{};
    }
    if(!finished && setup_trial_phase()==SetupPhase::trial && host_clock>500000)throw LoopComplete{};
    if(!finished)CHECK(host_clock<((scenario=="full-auto-wrap"||scenario=="full-precut-wrap")?uint64_t(UINT32_MAX)+400000000ull:400000000ull));
}
static void seed_page(uint32_t address,uint32_t magic,uint16_t version,const void* payload,uint16_t bytes,uint32_t flags=0){
    auto* raw=host_flash_ptr(address);std::memset(raw,0xff,256);
    NVM256_HDR h{magic,version,bytes,flags};std::memcpy(raw,&h,sizeof(h));std::memcpy(raw+sizeof(h),payload,bytes);
    const uint32_t crc=crc32_hw_words(raw,252);std::memcpy(raw+252,&crc,4);
}
static void seed_cal(){Flash_CAL_payload c{};for(int i=0;i<4;++i){c.offs[i]=0;c.vmin[i]=1.35f;c.vmax[i]=1.95f;c.dm_none[i]=.6f;}seed_page(0x0800f000,MAGIC_CAL,2,&c,sizeof(c),15u<<8);}
static void seed_motion(){Motion_control_save_struct m{};m.check=0x40614061;for(int i=0;i<4;++i){m.Motion_control_dir[i]=i%2?-1:1;m.dm_key_none_cv[i]=60;}seed_page(0x0800f100,MAGIC_MOT,VER_1,&m,sizeof(m));}
static void seed_owner(uint8_t ch,uint16_t seq=0,unsigned slot=0){CHECK(slot<STA_TOTAL_SLOTS);uint32_t words[2]={(STA_TAG<<24)|(uint32_t(seq)<<8)|ch,0};words[1]=words[0]^MAGIC_STA;std::memcpy(host_flash_ptr(sta_slot_addr(slot)),words,8);}
// Construct integrity-valid inputs with the production CRC primitive; production readers do all validation.
static bool ordering_ready(){return is("order-journal-full-wrap")||is("order-journal-one")||is("order-journal-wrap")||is("order-journal-duplicate")||is("order-cal-one")||is("order-cal-wrap")||is("order-cal-duplicate")||is("order-cal-newest");}
static bool ordering_loaded(){return is("order-journal-newest-loaded")||is("order-journal-duplicate-loaded");}
static bool ordering_journal_bad(){return scenario.rfind("order-journal-",0)==0&&!ordering_ready()&&!ordering_loaded();}
static bool ordering_cal_bad(){return scenario.rfind("order-cal-",0)==0&&!ordering_ready();}
static void ordering_seed(){
    seed_cal();seed_motion();seed_owner(0xff);
    if(is("order-journal-tie")||is("order-journal-cache"))seed_owner(0,0,1);
    if(is("order-journal-half"))seed_owner(0,0x8000,1);
    if(is("order-journal-older-tie")){seed_owner(0,0,1);seed_owner(0xff,2,2);}
    if(is("order-journal-cycle")){seed_owner(0xff,0x6000,1);seed_owner(0xff,0xc000,2);}
    if(is("order-journal-cycle-reverse")){seed_owner(0xff,0xc000,1);seed_owner(0xff,0x6000,2);}
    if(is("order-journal-wrap")){seed_owner(0,0xfffe,0);seed_owner(1,0xffff,1);seed_owner(0xff,0,2);}
    if(is("order-journal-duplicate"))seed_owner(0xff,0,1);
    if(is("order-journal-newest-loaded")){seed_owner(0xff,0xffff,0);seed_owner(2,0,1);}
    if(is("order-journal-duplicate-loaded")){seed_owner(2,0,0);seed_owner(2,0,1);}
    if(is("order-journal-full-wrap"))for(unsigned i=0;i<STA_TOTAL_SLOTS;++i)seed_owner(i==STA_TOTAL_SLOTS-1?0xff:0,uint16_t(65500u+i),i);
    if(is("order-journal-full"))for(unsigned i=0;i<STA_TOTAL_SLOTS;++i)seed_owner(0xff,uint16_t(i*257u),i);
    if(scenario.rfind("order-cal-",0)==0&&!is("order-cal-one")){
        Flash_CAL_payload c;std::memcpy(&c,host_flash_ptr(FLASH_NVM_CAL_ADDR)+sizeof(NVM256_HDR),sizeof(c));
        uint16_t first=0,second=0;uint32_t flags=15u<<8;
        if(!is("order-cal-duplicate"))c.offs[0]=.02f;
        if(is("order-cal-half"))second=0x8000;
        if(is("order-cal-wrap"))first=0xffff;
        if(is("order-cal-newest"))second=1;
        if(is("order-cal-flags-tie")){c.offs[0]=0;flags|=1;}
        auto* h=reinterpret_cast<NVM256_HDR*>(host_flash_ptr(FLASH_NVM_CAL_ADDR));h->rsv|=uint32_t(first)<<16;
        uint32_t crc=crc32_hw_words(host_flash_ptr(FLASH_NVM_CAL_ADDR),252);std::memcpy(host_flash_ptr(FLASH_NVM_CAL_ADDR)+252,&crc,4);
        seed_page(CAL_BACKUP_ADDR,MAGIC_CAL,2,&c,sizeof(c),(uint32_t(second)<<16)|flags);
    }
}
static void zeros(){
    for(int t=0;t<3;++t)for(int c=0;c<(t==2?4:2);++c){
        const unsigned ch=channel(t,c);CHECK(compare_values[t][c]==(setup_motor_energized[ch]?1000:0));
    }
}
using Packet=std::vector<uint8_t>;
static unsigned written=0;
static Packet short_packet(uint8_t cmd,unsigned bytes=8){Packet p(bytes);p[0]=0x3d;p[1]=0xc5;p[2]=bytes;p[4]=cmd;package_add_crc(p.data(),p.size());return p;}
static void append(const Packet& p){for(auto b:p){rx_dma[written&(RX_SIZE-1u)]=b;++written;DMA1_Channel5->CNTR=RX_SIZE-(written&(RX_SIZE-1u));if(!(written&(RX_SIZE-1u))){DMA1->INTFR|=DMA1_FLAG_TC5;DMA1_Channel5_IRQHandler();}}}
static bambubus_package_type dispatch(const Packet& p){append(p);bus_rx_poll();auto r=bambubus_run();bus_port_to_host.send_data_len=0;return r;}
static void motion_request(uint8_t state,uint8_t flags){auto p=short_packet(3,12);p[5]=0;p[6]=state;p[7]=0;p[8]=flags;package_add_crc(p.data(),p.size());dispatch(p);}
static void stopped_trial(){for(int t=0;t<3;++t)for(int c=0;c<(t==2?4:2);c+=2)CHECK(compare_values[t][c]==compare_values[t][c+1]);for(int t=0;t<3;++t)for(int c=0;c<2;++c)CHECK(compare_values[t][c]==0);}
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL
static void native_trial(){
    auto hb=short_packet(0x20);auto bad=hb;bad.back()^=1;
    dispatch(bad);CHECK(!bambubus_heartbeat_reset_armed);
    append(Packet(hb.begin(),hb.begin()+4));bus_rx_poll();bambubus_run();CHECK(!bambubus_heartbeat_reset_armed);
    dispatch(Packet(hb.begin()+4,hb.end()));CHECK(bambubus_heartbeat_reset_armed);
    CHECK(Motion_control_trial_eligibility()==TrialStop::none);
    motion_request(3,0xbf);CHECK(single_trial_last_verb()==TrialVerb::feed && single_trial_active());
    for(int i=0;i<4;++i){host_clock+=1000;Motion_control_run(0);}
    CHECK(compare_values[TIM4][2]!=compare_values[TIM4][3]);CHECK(std::max(compare_values[TIM4][2],compare_values[TIM4][3])<=600);
    const auto start=trial.start_ticks;
    if(is("trial-use-pull")){
        motion_request(7,0);CHECK(single_trial_last_verb()==TrialVerb::use && trial.start_ticks==start);
        Motion_control_run(0);motion_request(3,0x3f);CHECK(single_trial_last_verb()==TrialVerb::pull && trial.start_ticks==start);Motion_control_run(0);
    }
    if(is("trial-distance")){
        MC_AS5600.raw_angle[0]=(MC_AS5600.raw_angle[0]+1740)&4095;host_clock+=1000;Motion_control_run(0);
        CHECK(single_trial_reason()==TrialStop::travel_cap);
    }else if(is("trial-lease-wrap")){
        host_clock=uint64_t(bambubus_heartbeat_deadline)+1;bambubus_run();Motion_control_run(-1);CHECK(single_trial_reason()==TrialStop::lease);
        const auto deadline=bambubus_heartbeat_deadline;
        for(uint64_t delta:{uint64_t(1)<<31,uint64_t(1)<<32}){
            host_clock=uint64_t(deadline)+delta;dispatch(bad);motion_request(3,0xbf);Motion_control_run(-1);
            CHECK(!bambubus_heartbeat_reset_armed && single_trial_reason()==TrialStop::lease);
        }
        dispatch(hb);CHECK(single_trial_reason()==TrialStop::lease);
    }else{
        freeze_encoder=true;
        host_clock=start+uint64_t(time_hw_tpms)*700;dispatch(hb);
        host_clock=start+uint64_t(time_hw_tpms)*1400;dispatch(hb);
        host_clock=start+uint64_t(time_hw_tpms)*2001;dispatch(hb);Motion_control_run(0);CHECK(single_trial_reason()==TrialStop::time_cap);
    }
    stopped_trial();CHECK(!host_programs && !host_erases);
    std::puts("combined boot -> validated DMA/native request -> real controller/PWM/cap; no setup capabilities");
}
#endif
#if BMCU_USB_FLIGHT_RECORDER
static void flight_save(const char*,const void*,size_t);
static void flight_reset();
#endif
#include "full_test.inc"
#if !BMCU_LITE_FULL_LOAD
// Input-detail observers belong to the full-load fixture. Other profiles still
// compile the shared physical input paths without enabling those observers.
static void host_input_detail_hook(int){}
static void host_input_detail_read(int){}
static void host_input_detail_barrier(const float*,unsigned){}
#endif
#if BMCU_USB_FLIGHT_RECORDER
#include "flight_test.inc"
#endif
#include "release_cue_test.inc"
#include "rx_queue_test.inc"
#include "precut_phase_test.inc"
#ifdef HOST_ENCODER_DRIVER
#include "encoder_recovery_test.inc"
#endif
#if BMCU_USB_CONTINUOUS_JOURNAL
#include "journal_test.inc"
#include "retention_test.inc"
#endif
#if BMCU_USB_WORKING_MODE
#include "working_test.inc"
#if BMCU_USB_ISP_EXPORT
#include "marker_test.inc"
#endif
#endif
int main(int argc,char**argv){
    CHECK(argc==3);scenario=argv[1];time_hw_tpms=1000;time_hw_tpus=1;host_clock=100000;
    if((scenario=="full-auto-wrap"||scenario=="full-precut-wrap"))host_clock=UINT32_MAX-10000000ull;
    host_time_hook=time_hook;host_compare_hook=compare;host_delay_hook=[](uint32_t){raw_dma_boundary();};
    for(const auto& s:schedules)if(scenario==s.name)schedule=&s;
    for(const auto& s:gesture_schedules)if(scenario==s.name)detail_schedule=&s;
    std::memset(host_nvm,0xff,sizeof(host_nvm));
#if BMCU_USB_FLIGHT_RECORDER
    std::memset(host_log,0xff,sizeof(host_log));
#if BMCU_USB_ISP_EXPORT
    memset(host_marker,255,sizeof(host_marker));
    if(scenario.rfind("marker-",0)==0)return marker_case();
#endif
#if BMCU_USB_WORKING_MODE
    if(scenario.rfind("working-",0)==0)return working_case(argv[2]);
#else
    if(scenario.rfind("flight-",0)==0)return flight_case(argv[2]);
    // Model the supported installation flow using the actual preparation/ACK
    // bodies, then a separate POR/PDR before running actual main/setup.
    CHECK(!flight::boot()&&!flight::capturing);
#if BMCU_USB_CONTINUOUS_JOURNAL
    if(scenario=="full-journal-boot")flight_save("virgin.frame",flight::frame,sizeof(flight::frame));
#endif
#if !BMCU_LITE_CONTINUOUS_NATIVE
    flight_ack();CHECK(flight::acknowledged);
#else
    CHECK(flight::get32(host_log+20)==0x6a51ffffu&&!flight::acknowledged);
#endif
    flight_reset();host_time_hook=time_hook;host_compare_hook=compare;
#endif
#endif
    if(full_case())full_boot_seed(argv[2]);
    if(scenario.rfind("pipeline-",0)==0){
        ADC_DMA_init();ADC_DMA_wait_full();CHECK(ADC_DMA_ready()&&ADC_DMA_is_inited());
        CHECK(ADC_DMA_generation()==0);CHECK(ADC_DMA_get_value()[0]>.3f);const auto gen=ADC_DMA_generation();CHECK(gen==1);
        CHECK(ADC_DMA_get_value()[0]>.3f&&ADC_DMA_generation()==gen);
        ADC_DMA_filter_reset();CHECK(!ADC_DMA_ready()&&ADC_DMA_generation()==gen);CHECK(ADC_DMA_get_value()[0]==0&&ADC_DMA_generation()==gen);
        if(is("pipeline-stopped"))host_dma1.CFGR=0;
        ADC_DMA_wait_full();
        if(is("pipeline-stopped")){CHECK(!ADC_DMA_ready()&&ADC_DMA_get_value()[0]==0&&ADC_DMA_generation()==gen);}
        else{
            CHECK(ADC_DMA_ready()&&ADC_DMA_generation()==gen);CHECK(ADC_DMA_get_value()[0]>.3f&&ADC_DMA_generation()==gen+1);
            DMA1->INTFR|=DMA1_FLAG_TE1;ADC_DMA_poll();CHECK(!ADC_DMA_ready());const auto reset_gen=ADC_DMA_generation();CHECK(ADC_DMA_get_value()[0]==0&&ADC_DMA_generation()==reset_gen);
            ADC_DMA_wait_full();CHECK(ADC_DMA_ready());CHECK(ADC_DMA_get_value()[0]>.3f&&ADC_DMA_generation()==reset_gen+1);
        }
        CHECK(!host_programs&&!host_erases&&!bus_started);std::puts("real ADC reset/publication/TE/acquisition control passed");return 0;
    }
    if(scenario.rfind("order-",0)==0){
        // Cache a real positive result, then force the actual classification rescan to observe new ambiguity.
        if(is("order-journal-cache")||is("order-cal-cache")){
            seed_cal();seed_motion();seed_owner(0xff);Flash_saves_init();uint8_t ch=0;
            CHECK(Flash_AMS_state_read(&ch)&&ch==0xff);CHECK(Flash_AMS_state_read(&ch)&&ch==0xff);
            if(is("order-cal-cache")){float o[4],l[4],h[4],d[4];int8_t p[4];uint8_t mask;CHECK(Flash_MC_PULL_cal_read(o,l,h,p,d,&mask));CHECK(Flash_MC_PULL_cal_read(o,l,h,p,d,&mask));}
        }
        ordering_seed();
        if(is("order-journal-cache")||is("order-cal-cache")){
            const auto refreshed=Flash_setup_probe();uint8_t ch=0;
            if(is("order-journal-cache")){CHECK(!refreshed.journal_valid&&refreshed.journal_invalid);CHECK(!Flash_AMS_state_read(&ch));CHECK(!Flash_AMS_state_read(&ch));}
            else {CHECK(refreshed.cal_invalid);float o[4],l[4],h[4],d[4];int8_t p[4];uint8_t mask;CHECK(!Flash_MC_PULL_cal_read(o,l,h,p,d,&mask));CHECK(!Flash_MC_PULL_cal_read(o,l,h,p,d,&mask));}
        }
    }
    if(scenario.rfind("loaded-",0)==0)seed_owner(uint8_t(scenario.back()-'0'));
    if(is("corrupt-journal"))host_nvm[0x600]=1;
    if(is("partial-cal"))host_nvm[0]=1;
    if(is("partial-motion"))host_nvm[0x100]=1;
    if(is("partial-journal"))host_nvm[0x604]=1;
    if(is("valid-ff-incomplete"))seed_owner(0xff);
    if(is("valid-ff-tainted")){seed_owner(0xff);host_nvm[0x608]=1;}
    if(is("cached-negative")){Flash_saves_init();uint8_t ch=0;CHECK(!Flash_AMS_state_read(&ch));seed_owner(1);const auto p=Flash_setup_probe();CHECK(p.journal_valid && p.owner==1);}
    if(is("cached-valid")){seed_owner(0xff);Flash_saves_init();uint8_t ch;CHECK(Flash_AMS_state_read(&ch));host_nvm[0x600]=0;const auto p=Flash_setup_probe();CHECK(p.journal_invalid && !p.journal_valid);}
    if(is("cold-trial") || is("after-cut")){
        FILE*f=std::fopen(argv[2],"rb");CHECK(f);CHECK(std::fread(host_nvm,1,sizeof(host_nvm),f)==sizeof(host_nvm));std::fclose(f);
    }
    if(scenario.rfind("trial-",0)==0 || scenario.rfind("ready-",0)==0){seed_cal();seed_motion();seed_owner(0xff);}
    if(is("cut-cal"))host_powercut=2;
    if(is("cut-after-cal"))host_powercut=40;
    if(is("cut-motion"))host_powercut=43;
    if(is("cut-journal"))host_powercut=58;
    if(scenario.rfind("boundary-",0)==0){
        seed_cal();seed_motion();seed_owner(0xff);
        auto* payload=reinterpret_cast<Flash_CAL_payload*>(host_nvm+sizeof(NVM256_HDR));
        float value=0.01f*60.0f;
        if(is("boundary-below"))value=std::nextafter(value,-INFINITY);
        if(is("boundary-nan"))value=NAN;
        if(is("boundary-infinite"))value=INFINITY;
        if(is("boundary-upper"))value=1.39f;
        if(is("boundary-above"))value=std::nextafter(1.39f,INFINITY);
        for(float &v:payload->dm_none)v=value;
        const uint32_t crc=crc32_hw_words(host_nvm,252);std::memcpy(host_nvm+252,&crc,4);
        const bool expected=is("boundary-min")||is("boundary-upper");
        Flash_saves_init();uint8_t owner=0;
        CHECK(Flash_AMS_state_read(&owner)&&owner==0xff);CHECK(Flash_AMS_state_read(&owner)&&owner==0xff);
        CHECK(MC_PULL_calibration_load_retained_trial()==expected);CHECK(MC_PULL_calibration_load_retained_trial()==expected);
        const auto s=Flash_setup_probe();CHECK(s.journal_valid&&s.owner==0xff);CHECK(cal_retained()==expected);
        uint32_t bits;std::memcpy(&bits,&value,4);std::printf("boundary=%.17g bits=%08x accepted=%d; fresh and cached journal, repeated real retained loader\n",double(value),bits,expected);return 0;
    }
    if(is("any-renderer")){
        host_time_hook=nullptr;RGB_init();scale=1000;origin=host_clock;phase=SetupPhase::gesture;reached=1;gesture_ack_channel=3;gesture_ack_start=host_clock;
        unsigned on=0,magenta=0;const auto start=host_clock;
        for(unsigned ms=0;ms<3000;++ms){host_clock=start+uint64_t(ms)*1000;display(false);on+=leds[&RGBOUT[3]][0]==0x1800;for(unsigned ch=0;ch<3;++ch)CHECK(leds[&RGBOUT[ch]][0]==0x180018&&leds[&RGBOUT[ch]][1]==0x180018);magenta+=leds[&SYS_RGB][0]==0x180018;}
        CHECK(on==1000);CHECK(magenta==3000);
        host_clock=start+3100000;gesture_ack_start=host_clock;display(false);CHECK(leds[&RGBOUT[3]][0]==0x1800);
        setup_trial_fail(SetupFault::adc);CHECK(gesture_ack_channel==0xff&&leds[&RGBOUT[3]][0]==0&&leds[&RGBOUT[3]][1]==0);
        display(true);CHECK(leds[&RGBOUT[3]][0]==setup_trial_led_rgb(host_clock));
        phase=SetupPhase::trial;const auto before=leds[&RGBOUT[3]];gesture_ack_clear();CHECK(leds[&RGBOUT[3]]==before);
        std::puts("actual renderer/queue/transmitter: exact2x500ms green, two500ms gaps; fault clear and trial no stale ack");return 0;
    }
    if(is("led")){
        scale=1000;origin=100;phase=SetupPhase::fault;fault=SetupFault::capability;reached=5;
        unsigned white=0,red=0,green=0;
        for(uint32_t ms=0;ms<30000;++ms){auto rgb=setup_trial_led_rgb(origin+uint64_t(ms)*scale);white+=rgb==0x101010;red+=rgb==0x180000;green+=rgb==0x001800;}
        CHECK(white==600 && red==7000 && green==2500);std::puts("LED maximum14 pulses and 30s frame verified");return 0;
    }
    try{firmware_main();CHECK(false);}catch(const PowerCut&){
        FILE*f=std::fopen(argv[2],"wb");CHECK(f);CHECK(std::fwrite(host_nvm,1,sizeof(host_nvm),f)==sizeof(host_nvm));std::fclose(f);
        std::printf("power cut after %u actual halfword programs; persistent bytes saved\n",host_programs);
#if BMCU_LITE_CONTINUOUS_NATIVE
        if(continuous_fresh())return continuous_cut_export();
#endif
        return 0;
    }catch(const LoopComplete&){}
#if BMCU_USB_FLIGHT_RECORDER
#if !BMCU_USB_CONTINUOUS_JOURNAL
    flight_setup_verify();
#endif
#endif
    finished=true;
    std::printf("phase=%u fault=%u programs=%u erases=%u drives=%u/%u/%u/%u bus=%d\n",unsigned(phase),unsigned(fault),host_programs,host_erases,drives[0],drives[1],drives[2],drives[3],bus_started);
    std::fflush(stdout);
#if BMCU_LITE_CONTINUOUS_NATIVE
    if(continuous_fresh())return phase==SetupPhase::fault?continuous_setup_fault_verify():continuous_verify();
#endif
#if BMCU_LITE_FULL_LOAD && BMCU_LITE_AUTOMATIC_CAPTURE
    #ifdef HOST_ENCODER_DRIVER
    if(scenario.rfind("full-encoder-",0)==0)return encoder_recovery_verify();
    #endif
    if(scenario.rfind("full-rx-",0)==0)return rx_queue_verify();
#if BMCU_LITE_PRECUT_PHASE_CAPTURE
    if(scenario.rfind("full-precut-",0)==0)return precut_verify();
#endif
#endif
    if(full_case())return full_verify(argv[2]);
    if(cue_case())return cue_verify();
#if BMCU_USB_FLIGHT_RECORDER
    if(release_case())return release_verify();
#endif
    if(schedule){
        CHECK(!bus_started);
#if BMCU_LITE_V11_FEATURES
        if(!maintenance)
#endif
        {CHECK(!host_programs&&!host_erases);for(auto n:drives)CHECK(!n);}zeros();
        std::printf("STARTUP %s phase=%u fault=%u ready=%d inited=%d generation=%u samples=%u\n",scenario.c_str(),unsigned(phase),unsigned(fault),ADC_DMA_ready(),ADC_DMA_is_inited(),ADC_DMA_generation(),sample_events);std::fflush(stdout);
        if(schedule->positive)CHECK(phase==SetupPhase::gesture&&fault==SetupFault::none);
        else CHECK(phase==SetupPhase::fault&&unsigned(fault)==schedule->parent);
#ifndef HOST_BEFORE_FIX
        CHECK(boot_admission_detail==schedule->detail);
        // Real acquisition may recover after the parked failure; snapshot and parent reason stay frozen.
        if(!schedule->positive){
            const auto detail=boot_admission_detail;const auto parent=fault;
            host_time_hook=nullptr;schedule=nullptr;
            for(int i=0;i<8;++i){delay(1);ADC_DMA_poll();}ADC_DMA_wait_full();MC_PULL_detect_channels_inserted();
            CHECK(ADC_DMA_ready()&&ADC_DMA_is_inited());for(bool flag:filament_channel_inserted)CHECK(flag);
            setup_trial_admission_fail(SetupFault::adc,6);CHECK(fault==parent&&boot_admission_detail==detail);
            unsigned white=0,red=0,green=0,blue=0;
            for(uint32_t ms=0;ms<30000;++ms){const auto rgb=setup_trial_led_rgb(origin+uint64_t(ms)*scale);white+=rgb==0x101010;red+=rgb==0x180000;green+=rgb==0x001800;blue+=rgb==0x18;if(rgb==0x18)CHECK(ms>=24000&&ms<29500);}
            CHECK(white==600&&green==0&&red==unsigned(parent)*500&&blue==unsigned(detail)*500);
            host_clock=origin+uint64_t(24000)*scale;display(true);for(const auto& led:RGBOUT)CHECK(leds[&led][0]==(detail?0x18u:0u)&&leds[&led][1]==(detail?0x18u:0u));
            CHECK(!bus_started);
#if BMCU_LITE_V11_FEATURES
        if(!maintenance)
#endif
        {CHECK(!host_programs&&!host_erases);for(auto n:drives)CHECK(!n);}
            std::printf("frozen blue%u survives healthy reacquisition; exact30s groups verified\n",detail);
        }else{
            setup_trial_admission_fail(SetupFault::module,3);CHECK(boot_admission_detail==0); // Later gesture fault adds no early detail.
        }
#endif
        return 0;
    }
    if(detail_schedule){
        const auto expected=*detail_schedule;
        CHECK(unsigned(fault)==expected.parent);
        CHECK(boot_admission_detail==expected.detail);
        CHECK(gesture_ack_channel==0xff&&gesture_ack_start==0&&!bus_started);
        CHECK(bool(queued_green)==expected.ack);
        std::array<unsigned,4> green_edges{};std::array<uint32_t,4> prior{};
        std::vector<uint64_t> edges;
        for(const auto& event:deliveries){
            if(event.phase==SetupPhase::gesture)for(unsigned ch=0;ch<4;++ch)if(event.led==&RGBOUT[ch]){
                CHECK(event.rgb[0]==event.rgb[1]);const auto color=event.rgb[0];
                if(color==0x1800&&prior[ch]!=color)++green_edges[ch];
                if(ch==expected.channel&&color!=prior[ch]&&(color==0x1800||prior[ch]==0x1800))edges.push_back(event.tick);
                prior[ch]=color;
            }
            if(event.phase==SetupPhase::calibration&&event.led!=&SYS_RGB)CHECK(event.rgb[0]!=0x1800&&event.rgb[1]!=0x1800);
        }
        for(unsigned ch=0;ch<4;++ch)CHECK(green_edges[ch]==(expected.ack&&ch==expected.channel?2u:0u));
        if(expected.ack){
            CHECK(edges.size()==4&&accepted_tick>=uint64_t(gesture_start+4000)*1000);
            for(unsigned i=0;i<4;++i){const int64_t error=int64_t(edges[i]-accepted_tick)-int64_t(i)*500000;CHECK(error>=0&&error<3000);}
        }
        CHECK(gesture_polls>1000&&max_gesture_poll_gap<3000);
        if(expected.parent){
            CHECK(phase==SetupPhase::fault&&!cal_start&&!host_programs&&!host_erases);for(auto n:drives)CHECK(!n);zeros();
            // Later healthy DMA publication and retry/reinitialization cannot append or replace a detail.
            const auto captured=fault;const auto detail=boot_admission_detail;
            detail_schedule=nullptr;host_time_hook=nullptr;time_hw_tpms=scale;g_flash_fault=false;
            for(int ch=0;ch<4;++ch){adc[6-2*ch]=1.65f;adc[7-2*ch]=.20f;}
            ADC_DMA_filter_reset();ADC_DMA_wait_full();CHECK(ADC_DMA_ready());CHECK(ADC_DMA_get_value()[0]>.3f);
            host_clock_calls_before_exit=5;
            try{setup_trial_boot();CHECK(false);}catch(const LoopComplete&){}
            host_clock_calls_before_exit=0;
            CHECK(!gesture());setup_trial_admission_fail(SetupFault::module,4);
            CHECK(fault==captured&&boot_admission_detail==detail);
            if(captured!=SetupFault::clock){
                unsigned white=0,green=0,red=0,blue=0;
                for(unsigned ms=0;ms<30000;++ms){
                    const auto rgb=setup_trial_led_rgb(origin+uint64_t(ms)*scale);
                    const auto want=ms<600?0x101010u:
                        (ms>=2000&&ms<2000+unsigned(reached)*1000&&(ms-2000)%1000<500)?0x1800u:
                        (ms>=9000&&ms<9000+unsigned(captured)*1000&&(ms-9000)%1000<500)?0x180000u:
                        (ms>=24000&&ms<24000+unsigned(detail)*1000&&(ms-24000)%1000<500)?0x18u:0u;
                    CHECK(rgb==want);CHECK(setup_trial_led_rgb(origin+uint64_t(ms+30000)*scale)==want);
                    white+=rgb==0x101010;green+=rgb==0x1800;red+=rgb==0x180000;blue+=rgb==0x18;
                }
                CHECK(white==600&&green==unsigned(reached)*500&&red==unsigned(captured)*500&&blue==unsigned(detail)*500);
                // Observe the real queue and completed WS2812 delivery at every blue ON/OFF edge.
                for(unsigned i=0;i<=unsigned(detail)*2;++i){
                    host_clock=origin+uint64_t(24000+i*500)*scale;display(true);
                    const uint32_t want=i<unsigned(detail)*2&&i%2==0?0x18u:0u;
                    for(const auto& led:RGBOUT){CHECK(leds[&led][0]==want&&leds[&led][1]==want);bool found=false;
                        for(auto it=deliveries.rbegin();it!=deliveries.rend();++it)if(it->led==&led){CHECK(it->rgb[0]==want&&it->rgb[1]==want);found=true;break;}CHECK(found);}
                }
            }else CHECK(setup_trial_led_rgb(origin+24000ull*scale)==0x180000u);
            bus_init();bus_uart1_init();USART1_IRQHandler();DMA1_Channel4_IRQHandler();DMA1_Channel5_IRQHandler();
            CHECK(bambubus_run()==bambubus_package_type::none);
            CHECK(!Flash_AMS_state_write(0xff));Flash_background_run();Motion_control_set_PWM(2,1000);
            CHECK(fault==captured&&boot_admission_detail==detail);
            CHECK(!host_programs&&!host_erases&&!bus_started);zeros();
            std::printf("detail parent%u blue%u: frozen, no capabilities, ACK=%u, renderer/delivery verified\n",unsigned(captured),unsigned(detail),expected.ack);
            return 0;
        }
        CHECK(phase==SetupPhase::success&&cal_start>=gesture_start+6600);
        std::puts("detail normal: exact ACK, stable release, manual calibration follows unchanged");
    }
    if(any_case()){
        CHECK(gesture_ack_channel==0xff&&gesture_ack_start==0);CHECK(!bus_started);
        std::array<unsigned,4> greens{};std::array<uint32_t,4> last{};
        std::vector<uint64_t> edges;
        for(const auto& e:deliveries){
            if(e.phase==SetupPhase::gesture){
                for(unsigned ch=0;ch<4;++ch)if(e.led==&RGBOUT[ch]){
                    CHECK(e.rgb[0]==e.rgb[1]);
                    if(e.rgb[0]==0x1800){CHECK(ch==gesture_ch());if(last[ch]!=0x1800)++greens[ch];}
                    if(ch==gesture_ch()&&e.rgb[0]!=last[ch]&&(e.rgb[0]==0x1800||last[ch]==0x1800))edges.push_back(e.tick);
                    last[ch]=e.rgb[0];
                }
            }
            if(e.phase==SetupPhase::calibration&&e.led!=&SYS_RGB)CHECK(e.rgb[0]!=0x1800&&e.rgb[1]!=0x1800);
            if(fault_tick&&e.tick>=fault_tick&&e.led!=&SYS_RGB)CHECK(e.rgb[0]==setup_trial_led_rgb(e.tick)&&e.rgb[1]==setup_trial_led_rgb(e.tick));
        }
        for(unsigned ch=0;ch<4;++ch)if(ch!=gesture_ch())CHECK(!greens[ch]);
        const bool acknowledged=(BMCU_LITE_SETUP_GESTURE_1S&&is("any-adc-stale"))||any_success()||is("any-held")||is("any-release-unstable")||is("any-fault-after-ack")||is("any-queued-fault")||scenario.rfind("any-ack-",0)==0;
        CHECK(bool(queued_green)==acknowledged);
        if(acknowledged){CHECK(accepted_tick);CHECK(accepted_tick>=uint64_t(gesture_start+(is("any-retry")?3000:2000)+(BMCU_LITE_SETUP_GESTURE_1S?1000:2000))*1000);}
        if(any_success()||is("any-held")||is("any-release-unstable")){
            CHECK(greens[gesture_ch()]==2&&edges.size()==4);
            for(unsigned i=0;i<4;++i){const int64_t offset=int64_t(edges[i]-accepted_tick)-int64_t(i)*500000;CHECK(offset>=0&&offset<3000);}
        }
        if(!acknowledged||is("any-queued-fault"))CHECK(greens[gesture_ch()]==0);
        CHECK(max_gesture_poll_gap<3000&&gesture_polls>(is("any-preheld-invalid")?0u:1000u));
        if(!any_success()){CHECK(phase==SetupPhase::fault&&!host_programs&&!host_erases);for(auto n:drives)CHECK(!n);}
        if(any_success()){
            CHECK(cal_start>=gesture_start+(is("any-retry")?7600:6600));
            std::printf("any-buffer channel%u: two delivered pulses, genuine CAL/MOT/FF, no pre-CAL capabilities; poll gap%llu ticks\n",gesture_ch(),(unsigned long long)max_gesture_poll_gap);
        }else std::printf("any-buffer refusal: queued=%u delivered-pulses=%u poll gap%llu ticks\n",queued_green,greens[gesture_ch()],(unsigned long long)max_gesture_poll_gap);
    }
    if(scenario.rfind("order-",0)==0){
        const auto storage=Flash_setup_probe();uint8_t owner=0;
        const bool j1=Flash_AMS_state_read(&owner),j2=Flash_AMS_state_read(&owner);
        float o[4],l[4],h[4],d[4];int8_t p[4];uint8_t mask=0;
        const bool c1=Flash_MC_PULL_cal_read(o,l,h,p,d,&mask),c2=Flash_MC_PULL_cal_read(o,l,h,p,d,&mask);
        std::printf("production ordering: journal=%d/%d invalid=%d cal=%d/%d invalid=%d owner=%u\n",j1,j2,storage.journal_invalid,c1,c2,storage.cal_invalid,owner);std::fflush(stdout);
        CHECK(j1==!ordering_journal_bad()&&j2==j1);CHECK(c1==!ordering_cal_bad()&&c2==c1);
        CHECK(storage.journal_valid==j1&&storage.journal_invalid==ordering_journal_bad());
        CHECK(storage.cal_invalid==ordering_cal_bad());
        CHECK(!host_programs&&!host_erases);for(auto n:drives)CHECK(!n);if(ordering_ready())stopped_trial();else zeros();
        if(ordering_ready()){
            CHECK(phase==SetupPhase::trial&&fault==SetupFault::none&&bus_started);CHECK(owner==0xff);
            if(is("order-cal-wrap")||is("order-cal-newest"))CHECK(o[0]==.02f);
        }else{
            CHECK(phase==SetupPhase::fault&&fault==(ordering_loaded()?SetupFault::loaded_owner:SetupFault::storage));CHECK(!bus_started);
            bus_init();bus_uart1_init();USART1_IRQHandler();DMA1_Channel4_IRQHandler();DMA1_Channel5_IRQHandler();
            CHECK(!bus_started&&bambubus_run()==bambubus_package_type::none);CHECK(!USART1->CTLR1&&!USART1->CTLR3&&!DMA1_Channel5->CFGR);
            Motion_control_set_PWM(0,500);CHECK(!Flash_AMS_state_write(0xff));Flash_background_run();zeros();CHECK(!host_programs&&!host_erases);
        }
        return 0;
    }
    if(is("success")||any_success()){
        if(phase!=SetupPhase::success){
            float o[4],l[4],h[4],d[4];int8_t pol[4];uint8_t mask=0;
            bool r=Flash_MC_PULL_cal_read(o,l,h,pol,d,&mask);
            std::printf("calread=%d mask=%u gvalid=%u flashfault=%d crc=%08x stored=%08x\n",r,mask,g_valid_mask,g_flash_fault,crc32_hw_words(host_nvm,252),*(uint32_t*)(host_nvm+252));
            if(r)for(int ch=0;ch<4;++ch)std::printf("ch%d off%f low%f high%f dm%f pol%d\n",ch,o[ch],l[ch],h[ch],d[ch],pol[ch]);
        }
        CHECK(phase==SetupPhase::success && fault==SetupFault::none);CHECK(!bus_started);CHECK(host_programs>40);CHECK(host_erases==2);
        for(auto n:drives)CHECK(n>0);zeros();const auto s=Flash_setup_probe();CHECK(s.journal_valid && s.owner==0xff && !s.journal_invalid);CHECK(cal_retained() && Motion_control_setup_retained());
        float o[4],l[4],h[4],d[4];int8_t pol[4];uint8_t mask=0;
        CHECK(Flash_MC_PULL_cal_read(o,l,h,pol,d,&mask));CHECK(d[0]==0.01f*60.0f && d[0]<.6f);
        CHECK(MC_PULL_calibration_load_retained_trial());CHECK(MC_PULL_calibration_load_retained_trial());
        uint8_t ch=0;CHECK(Flash_AMS_state_read(&ch)&&ch==0xff);CHECK(Flash_AMS_state_read(&ch)&&ch==0xff);
        const auto fresh=Flash_setup_probe();CHECK(fresh.journal_valid&&fresh.owner==0xff);
        std::printf("genuine V11 calibration write/read/retained round trip: %.17g\n",double(d[0]));
        FILE*f=std::fopen(argv[2],"wb");CHECK(f);CHECK(std::fwrite(host_nvm,1,sizeof(host_nvm),f)==sizeof(host_nvm));std::fclose(f);
    }
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL
    else if(is("cold-trial") || scenario.rfind("trial-",0)==0){
        CHECK(gesture_ack_channel==0xff&&gesture_ack_start==0);
        CHECK(phase==SetupPhase::trial && bus_started && fault==SetupFault::none);CHECK(!single_trial_fault());CHECK(!host_programs && !host_erases);for(auto n:drives)CHECK(!n);
        const auto saved=std::array<int,4>{compare_values[2][0],compare_values[2][1],compare_values[2][2],compare_values[2][3]};
        float off[4],low[4],high[4],dm[4];int8_t pol[4];uint8_t mask=0;
        CHECK(Flash_MC_PULL_cal_read(off,low,high,pol,dm,&mask));
        CHECK(!Flash_MC_PULL_cal_write_all(off,low,high,pol,dm,mask));
        CHECK(!flash256_erase(0x0800f000));CHECK(!flash_word_prog_std(0x0800f000,0));
        Flash_background_run();
        CHECK(!Flash_AMS_state_write(0xff));CHECK(!Flash_Motion_write(nullptr,0));CHECK(!Flash_NVM_full_clear());CHECK(!Flash_Motion_clear());CHECK(!Flash_MC_PULL_cal_clear());CHECK(!Flash_AMS_filament_write(0,nullptr));
        CHECK(boot_admission_detail==0);setup_trial_admission_fail(SetupFault::module,3);CHECK(!gesture());CHECK(boot_admission_detail==0);
        Motion_control_setup_stop();MOTOR_init();MC_PULL_calibration_boot();CHECK(!Motion_control_setup_direction());setup_trial_boot();setup_trial_fail(SetupFault::storage);
        CHECK(phase==SetupPhase::trial && fault==SetupFault::none && !host_programs && !host_erases);
        for(int i=0;i<4;++i)CHECK(saved[i]==compare_values[2][i]);
        if(scenario.rfind("trial-",0)==0 && !is("trial-capabilities"))native_trial();
    }
#endif
    else{
        CHECK(phase==SetupPhase::fault && fault!=SetupFault::none);CHECK(!bus_started);zeros();
        if(is("filament") || is("gesture-key") || is("cal-key") || is("direction-key"))CHECK(fault==SetupFault::filament);
        if(is("unknown") || is("adc-stale") || is("adc-generation-stale") || is("uncentered") || is("unstable") || is("cal-invalid"))CHECK(fault==SetupFault::adc);
        if(is("missing-module") || is("ready-missing-module"))CHECK(fault==SetupFault::module);
        if(is("any-ack-encoder")&&setup_trial_phase()==SetupPhase::gesture&&gesture_start&&host_clock/1000-gesture_start>=4300)return 14;
    if(is("any-encoder")&&setup_trial_phase()==SetupPhase::gesture&&gesture_start&&host_clock/1000-gesture_start>=3000)return 14;
    if(is("encoder-entry") || is("ready-encoder") || is("cal-encoder") || is("direction-encoder"))CHECK(fault==SetupFault::encoder);
        if(scenario.rfind("loaded-",0)==0 || is("cached-negative"))CHECK(fault==SetupFault::loaded_owner);
        if(is("gesture-timeout") || is("gesture-held") || is("gesture-short"))CHECK(fault==SetupFault::gesture_timeout);
        if(is("cal-timeout"))CHECK(fault==SetupFault::calibration);
        if(is("direction-clock-back") || is("direction-clock-scale"))CHECK(fault==SetupFault::clock);
        if(is("direction-direct"))CHECK(fault==SetupFault::capability);
        if(!host_programs && !is("after-cut"))CHECK(!host_erases);
        const unsigned before=host_programs;bus_init();bus_uart1_init();USART1_IRQHandler();DMA1_Channel4_IRQHandler();DMA1_Channel5_IRQHandler();CHECK(!bus_started);CHECK(bambubus_run()==bambubus_package_type::none);CHECK(!USART1->CTLR1 && !USART1->CTLR3 && !DMA1_Channel5->CFGR);
        CHECK(!Flash_AMS_state_write(0xff));Flash_background_run();CHECK(host_programs==before);
        const auto first=fault;Motion_control_set_PWM(2,1000);CHECK(fault==first);zeros();
        if(scenario.rfind("direction-",0)!=0 && scenario.rfind("journal-",0)!=0 && !is("program-error") && !is("motion-error")){for(auto n:drives)CHECK(!n);}
    }
}
