#include "flight_recorder.h"
#if BMCU_USB_FLIGHT_RECORDER
#include "authority_observation.h"
#include "Flash_saves.h"
#include "Motion_control.h"
#if BMCU_LITE_LEASE_FEEDBACK
#include "app_api.h"
#endif
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
#include "ams.h"
#include "bambu_bus_ams.h"
#include "full_load.h"
#endif
#include "setup_trial.h"
#include "hal/time_hw.h"
#include "hal/irq_wch.h"
#include "ch32v20x_flash.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_rcc.h"
#include "ch32v20x_usart.h"
#if BMCU_LITE_RX_BACKPRESSURE
#include "_bus_hardware.h"
#endif
#include <string.h>
#include <math.h>
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE && defined(__riscv)
// GCC12.2 libgcc's own public ABI wrappers share its exact combined helper.
// No division implementation, zero-divisor policy or rounding is recreated here.
extern "C" uint64_t __udivmoddi4(uint64_t,uint64_t,uint64_t*);
extern "C" __attribute__((used,externally_visible,noinline)) uint64_t __udivdi3(uint64_t n,uint64_t d){return __udivmoddi4(n,d,nullptr);}
extern "C" __attribute__((used,externally_visible,noinline)) uint64_t __umoddi3(uint64_t n,uint64_t d){uint64_t r;__udivmoddi4(n,d,&r);return r;}
extern "C" float __floatundisf(uint64_t);
extern "C" __attribute__((used,externally_visible,noinline)) float __floatdisf(int64_t x){
    uint64_t magnitude=uint64_t(x);
    if(x<0)magnitude=UINT64_C(0)-magnitude;
    float value=__floatundisf(magnitude);uint32_t bits;
    memcpy(&bits,&value,4);if(x<0)bits^=0x80000000u;memcpy(&value,&bits,4);
    return value;
}
#endif
#if BMCU_USB_WORKING_MODE
bambubus_package_type get_packge_type(uint8_t*, int);
#endif
namespace flight {
static constexpr uint32_t magic=0x314c4642u, commit=0x51c0da7au, guard=0x6c726731u;
static constexpr uint16_t armed=0x6a51u;
static_assert(base+512u==FLASH_NVM_BASE_ADDR,"Recorder must end before original NVM");
static_assert(32u+sizeof(Report)<508u,"Report must not overlap rearm guard");
static Report report;
alignas(4) static uint8_t frame[536],ack[20],reply[20];
static uint32_t session=0,frame_crc=0,window_scale=0;
static uint64_t window_start=0,last_sample=0;
static float comparator_delta[4];
static unsigned tx=0,rx=0;
static uint32_t tx_at=0,rx_at=0;
static bool capturing=false,acknowledged=false,storage_error=false;
#if BMCU_USB_CONTINUOUS_JOURNAL
#include "continuous_journal.inc"
#endif
#if BMCU_LITE_V11_FEED_LATCH
static volatile bool finished=false;
// DMA submission and completion hooks run under the bus IRQ guard. Only status
// counters/flags are written by the ISR. finished freezes them before Flash writes.
// Full-load main calls only queued send_package(): while send_data_len is nonzero,
// bambubus_run cannot consume another request or replace its response (fixed unit0,
// BMCU_AMS_AUTO=0). Each successful DMA submission consumes staged_kind, including
// non-motion sends. The only setters are the two bounded serializers above their
// send_data_len publication; no constructed response is labeled submitted.
static uint8_t staged_kind=0,active_kind=0;
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
static bool verified_saved=false,pull_first=false;
#if BMCU_LITE_AUTOMATIC_CAPTURE
static uint64_t automatic_deadline=0;

#endif
static volatile bool operator_stop=false; // Single main writer, byte-atomic IRQ readers.
#if BMCU_LITE_PRECUT_PHASE_CAPTURE
static uint8_t wire_phase=0,staged_phase=0,phase_count=0;
static uint32_t phase_origin=0;
// Current request: class bits5..7, sender04 bit4, requested channel0..3/F.
// Every valid encoding is nonzero; actual DMA start alone fills a record.
static void phase_consumed(){
    if(!finished && !operator_stop && (report.flags&(1u<<20)) && wire_phase>2u)
        report.precut_consumed|=1u<<(wire_phase>>4);
}
#endif
static uint8_t pull_command=0,pull_channel=0;
static DiagnosticV1& diagnostic(){return report.diagnostic;}
bool operator_pending(){return operator_stop;}
bool saved(){return verified_saved;}
bool operator_eligible(){
#if BMCU_USB_CONTINUOUS_JOURNAL
    return false;
#endif

    return !finished && (report.flags&128u) && full_load_ready() && // ready() includes the same permanent operator-stop gate.
        full_load_durable(full_load_owner()) && !full_load_journal_pending() &&
        !Flash_metadata_recovery_pending() && !Flash_setup_fault();
}
void pull_wire(uint8_t command,uint8_t ch,uint8_t state,uint8_t motion){
#if BMCU_USB_CONTINUOUS_JOURNAL
    const uint32_t tuple=uint32_t(state)|(uint32_t(motion)<<8);
    const bool changed=tuple!=jrequest||ch!=jrequest_channel||command!=jrequest_command;
    jrequest=tuple;jrequest_channel=ch;jrequest_command=command;
    // Existing CRC/addressed/consumed boundary, before offline/admission guards.
    if(ch<4u&&state==3u&&motion==0x3fu&&changed){
        jrefusal_pending=true; // First actual refusal for this attempt, before any guard runs.
        jevent(19,ch,tuple|(uint32_t(command)<<16));
    }
    return;
#endif

#if BMCU_LITE_PRECUT_PHASE_CAPTURE
    authority_observation::wire(command,ch,state,motion); // Same boundary/order; one argument dispatch for both observers.
    uint8_t phase=motion==0u?0u:motion==0x26u?1u:motion==0x66u?2u:motion==0x99u?3u:255u;
    wire_phase=state==7u && phase<4u && (ch<4u || ch==255u) ?
        uint8_t((phase<<6)|(ch==255u?15u:(32u|ch))|((command-3u)<<4)):
        (ch==255u && state==1u && motion==0u?2u:0u);
    phase_consumed();
#endif
    pull_command=ch<4u && state==3u && motion==0x3fu?command:0u; // Zero encodes no eligible current request; real commands are03/04.
    pull_first=false;
    if(finished || operator_stop || !pull_command)return;
    auto& d=diagnostic();++d.seen_count; // Schema is stamped only in the frozen snapshot.
    if(!(d.seen_flags&1u)){d.seen_flags|=1u;d.seen_command=command;d.seen_channel=ch;pull_first=true;}
    pull_channel=ch;
}
void pull_refused(PullRefusal reason,uint8_t context){
#if BMCU_USB_CONTINUOUS_JOURNAL
    static uint32_t previous=0xffffffffu;const uint32_t value=uint32_t(reason)|(uint32_t(context)<<8)|jrequest<<16;
    if((jrefusal_pending||value!=previous)&&jevent(8,jrequest_channel,value)){
        previous=value;jrefusal_pending=false; // A full queue must not poison refusal suppression.
    }
    return;
#endif

    if(finished || operator_stop || !pull_command)return;
    auto& d=diagnostic();
    if(!d.first_refusal){d.first_refusal=uint8_t(reason);d.refusal_context=context;
        d.refusal_command=pull_command;d.refusal_channel=pull_channel;}
}
void pull_admitted(uint8_t command,uint8_t ch){
#if BMCU_USB_CONTINUOUS_JOURNAL
    jevent(7,ch,command);return;
#endif

    if(finished || operator_stop)return;
    auto& d=diagnostic(); // Called only after the real pull request's accepted mutation.
    if(!(d.seen_flags&2u)){d.seen_flags|=2u;d.admitted_command=command;d.admitted_channel=ch;}
}
void pull_pwm(uint8_t ch,uint16_t first,uint16_t second){
#if BMCU_USB_CONTINUOUS_JOURNAL
    static uint8_t seen=0;if(first!=second&&!(seen&(1u<<ch))){seen|=1u<<ch;jevent(11,ch,uint32_t(first)|(uint32_t(second)<<16));}return;
#endif

    if(finished || operator_stop || ams[0].filament[ch].motion!=_filament_motion::pull_back)return;
    auto& d=diagnostic();d.pwm_observed_mask|=1u<<ch;
    if(first!=second && !(d.seen_flags&4u)){d.seen_flags|=4u;d.pwm_channel=ch;d.first_pwm[0]=first;d.first_pwm[1]=second;}
}
static void diagnostic_snapshot(uint8_t kind,uint8_t gesture){
#if BMCU_LITE_RX_BACKPRESSURE
    report.rx_dropped=bus_port_to_host.rx_dropped;
    report.rx_queue_pauses=bus_port_to_host.rx_queue_pauses;
    report.rx_header_crc_errors=bus_port_to_host.rx_header_crc_errors;
#endif
    auto& d=diagnostic();d.version=1;d.snapshot_kind=kind;d.gesture_channel=gesture;
    const auto& a=ams[0];const uint8_t selected=a.now_filament_num;
    d.before_selected=selected;d.before_owner=full_load_owner();
    d.before_motion=selected<4u?uint8_t(a.filament[selected].motion):255u;
    d.before_pull=lite_pull_status(selected);d.before_fault=uint8_t(full_load_fault());
    d.before_flags=uint8_t(lite_motion_ready()) | (uint8_t(lite_recovery_fault())<<1) | (uint8_t(lite_lease_active())<<2);
    d.before_pwm[0]=TIM4->CH3CVR;d.before_pwm[1]=TIM4->CH4CVR;
    d.before_pwm[2]=TIM4->CH1CVR;d.before_pwm[3]=TIM4->CH2CVR;
    d.before_pwm[4]=TIM3->CH1CVR;d.before_pwm[5]=TIM3->CH2CVR;
    d.before_pwm[6]=TIM2->CH1CVR;d.before_pwm[7]=TIM2->CH2CVR;
}
#if BMCU_LITE_AUTOMATIC_CAPTURE
static void terminal_request(uint8_t kind,uint8_t gesture){
    diagnostic_snapshot(kind,gesture);
#else
void operator_request(uint8_t gesture){
#if BMCU_USB_CONTINUOUS_JOURNAL
    (void)gesture;return;
#endif

    if(!operator_eligible())return;
    diagnostic_snapshot(1u,gesture);
#endif
    authority_observation::snapshot(report.ao4);
    __asm__ volatile("" ::: "memory"); // Publish the pre-stop RAM evidence before the IRQ-visible freeze.
    operator_stop=true; // Set before cancellation: never permit deferred/local/PWM revival.
    __asm__ volatile("" ::: "memory");
    Motion_control_lite_cancel();
}
#if BMCU_LITE_AUTOMATIC_CAPTURE
void operator_request(uint8_t gesture){
#if BMCU_USB_CONTINUOUS_JOURNAL
    (void)gesture;return;
#endif

    if(operator_eligible())terminal_request(1u,gesture);
}
void automatic_capture(){
#if BMCU_USB_CONTINUOUS_JOURNAL
    return;
#endif

    // No renewal by polls, heartbeat recovery, repeated use, or a second channel.
    // A software journal may be pending: terminal shutdown discards unissued work.
    if(!finished && !operator_stop && (report.flags&(1u<<20)) &&
       time_ticks64()>=automatic_deadline)
        terminal_request(3u,255u);
}
#endif
#endif
__attribute__((noinline)) void response(uint8_t ch,uint8_t state,uint8_t motion){
#if BMCU_USB_CONTINUOUS_JOURNAL
    return; // The actual DMA-start hook owns the compact journal event.
#endif
#if BMCU_LITE_PRECUT_PHASE_CAPTURE
    staged_phase=wire_phase; // Only a constructed motion response stages the boundary request.
    staged_kind=pull_first?1u:wire_phase==2u?2u:0u;
    pull_first=false;
#elif BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
    staged_kind=pull_first?1u:(ch==255u&&state==1u&&motion==0u)?2u:0u;
    pull_first=false;
#else
    staged_kind=(state==3u&&motion==0xbfu)?1u:(ch==255u&&state==1u&&motion==0u)?2u:0u;
#endif
}
void tx_start(const uint8_t* data,uint16_t length){
#if BMCU_USB_CONTINUOUS_JOURNAL
    if((data[4]==3u||data[4]==4u)&&length>=(data[4]==3u?15u:27u))jtxevent(data);
    return;
#endif

    active_kind=staged_kind;staged_kind=0;
#if BMCU_LITE_PRECUT_PHASE_CAPTURE
    const uint8_t phase=staged_phase;staged_phase=0;
    if(finished || operator_stop)return;
    const uint8_t* feedback=data+(data[4]==3u?7u:19u);

#endif
#if BMCU_LITE_PRECUT_PHASE_CAPTURE
    if(active_kind){ // Common terminal gate already checked above.
#else
    if(finished
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
       || operator_stop
#endif
       ||!active_kind)return;
#endif
    if(active_kind==1u){
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
        auto& d=diagnostic();d.response_command=data[4];d.response_length=uint8_t(length);d.first_tx=1u;
#else
        memcpy(report.native_bf,data,length);report.bf_length=uint8_t(length);
#endif
    }
    else {memcpy(report.native_idle,
#if BMCU_LITE_PRECUT_PHASE_CAPTURE
        feedback,
#else
        data+(data[4]==3u?7u:19u),
#endif
        8u);report.idle_command=data[4];report.idle_length=uint8_t(length);}
    report.native_tx[active_kind-1u]=1u;
#if BMCU_LITE_PRECUT_PHASE_CAPTURE
    }
    if(phase>2u && (report.flags&(1u<<20))){
        const uint8_t index=phase>>5;
        const bool first=!(report.precut_submitted&(3u<<((phase>>4)&14u)));
        report.precut_submitted|=1u<<(phase>>4);
        if(first){
            auto& entry=report.precut[index];
            // This profile uses144MHz/HCLK8:18000tpms. Its120s window is
            // 2.16e9 ticks, below one uint32 wrap (238.6s). Keep the64-bit
            // control deadline. lite_profile.h rejects other target clock profiles.
            entry.elapsed_10ms=uint16_t(uint32_t(time_ticks32()-phase_origin)/(time_hw_tpms*10u));
            entry.request=uint8_t((phase&31u)|(phase_count++<<5));
            entry.feedback=uint8_t(feedback[0]|(feedback[1]<<4));
            memcpy(entry.meters,feedback+2,4);
        }
    }
#endif
}
#if BMCU_LITE_INPUT_FAULT_DETAIL
void input_fault(uint16_t detail){
#if BMCU_USB_CONTINUOUS_JOURNAL
    jevent(9,uint8_t(detail>>8),uint8_t(detail));return;
#endif
report.input_detail=detail;}
#endif
void tx_finish(bool aborted){
#if BMCU_USB_CONTINUOUS_JOURNAL
    if(aborted)jevent(13,jrequest_channel,jrequest_command);
    return;
#endif

    if(!finished
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
       && !operator_stop
#endif
       &&active_kind){
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
        if(active_kind==1u)diagnostic().first_tx|=aborted?4u:2u;
#endif
        report.native_tx[active_kind-1u]|=aborted?4u:2u;
        if(aborted)++report.tx_aborted;else ++report.tx_completed;
    }
    active_kind=0;
}

#else
static bool finished=false;
#endif
static uint32_t crc(const void* data,unsigned n) {
    uint32_t value=0xffffffffu;const uint8_t* p=(const uint8_t*)data;
    while(n--){value^=*p++;for(unsigned i=0;i<8;++i)value=(value>>1)^((value&1u)?0xedb88320u:0u);}
    return ~value;
}
static uint32_t get32(const uint8_t* p){uint32_t v;memcpy(&v,__builtin_assume_aligned(p,4),4);return v;}
static void put32(uint8_t* p,uint32_t v){memcpy(__builtin_assume_aligned(p,4),&v,4);}
static bool blank(uint32_t value){return value==0xffffffffu||value==0xe339e339u;}
static bool erased(){for(unsigned n=0;n<512;n+=4)if(!blank(get32(frame+20+n)))return false;return true;}
static bool write(uint32_t address,uint32_t value,bool erase=false){
    if(storage_error)return false;
    if(!Flash_log_write(address,value,erase)){storage_error=true;return false;}
    return true;
}
static bool header_valid(const uint8_t* p){
    return get32(p)==magic&&get32(p+4)==build&&get32(p+8)!=0&&get32(p+8)!=0xffffffffu&&
        p[12]==version&&p[13]==0&&p[14]==(sizeof(Report)&255u)&&p[15]==(sizeof(Report)>>8)&&get32(p+16)==crc(p,16);
}
static bool complete(const uint8_t* p){
    return header_valid(p)&&get32(p+24)==commit&&get32(p+28)==crc(p+32,sizeof(Report))&&get32(p+508)==guard;
}
static void export_frame(){
    memcpy(frame,"BFL1",4);put32(frame+4,(512u<<16)|version);
    put32(frame+8,build);put32(frame+12,session);
    Flash_log_read(frame+20);
    put32(frame+16,(header_valid(frame+20)?1u:0u)|(complete(frame+20)?2u:0u)|(storage_error?4u:0u));
    frame_crc=crc(frame,532);put32(frame+532,frame_crc);tx=0;
}
bool boot(){
#if BMCU_USB_CONTINUOUS_JOURNAL
    return jboot();
#endif

    // MC_PWM_init precedes this function. No sensor, bus, calibration or owner
    // initialization has run. No mode is inferred from USB or wire silence.

    // WCH RCC reset flags are sticky. Consume them on every boot, including
    // export, so a later NRST cannot inherit the preceding POR/PDR flag.
    const uint32_t reset=RCC->RSTSCKR;
    RCC->RSTSCKR|=1u<<24; // RMVF clears reset flags; preserve oscillator controls.
    const bool power_cycle=(reset&0xf8000000u)==0x08000000u;
    Flash_log_read(frame+20);
    const bool valid=header_valid(frame+20);
    session=valid?get32(frame+28):0;
    uint16_t arm;memcpy(&arm,frame+42,2);
    const bool initial=erased();
    if(initial || (power_cycle&&valid&&arm==armed&&session<0xfffffffeu)){
        const uint32_t next=initial?1:session+1;
        if(!initial){
            // B stays nonblank while A is erased. A is durable before erasing B.
            // Any interrupted erase/write routes to export, never another test.
            if(!write(base+508,guard)||!write(base,0,true)){export_frame();return false;}
        }
        alignas(4) uint8_t header[20];put32(header,magic);put32(header+4,build);put32(header+8,next);
        put32(header+12,(uint32_t(sizeof(Report))<<16)|version);put32(header+16,crc(header,16));
        bool ok=true;for(unsigned n=0;n<20&&ok;n+=4)ok=write(base+n,get32(header+n));
        if(ok&&!initial)ok=write(base+256,0,true);
        session=next;
        // The stock flasher resets into this image while still USB-powered.
        // Virgin preparation therefore exports too; every test requires an
        // explicit saved-report ACK followed by a POR/PDR-qualified boot.
#if BMCU_LITE_CONTINUOUS_NATIVE
        // The first boot (including the flasher's USB reset) is always inhibited.
        // Prepare exactly one later POR/PDR session; no source detector exists.
        // A torn permit or any nonvirgin record must export rather than auto-arm.
        if(ok&&initial)ok=write(base+20,0x6a51ffffu);
#endif
        if(ok&&!initial){capturing=true;report.candidate=0xffu;return true;}
    }
    export_frame();return false;
}
// Window entry is reached only after setup validates its nonzero frozen scale.
static uint32_t ms(uint64_t ticks){return uint32_t(ticks/window_scale);}
void window(const float* center,uint64_t start,uint32_t gen,uint32_t scale){
#if BMCU_USB_CONTINUOUS_JOURNAL
    return;
#endif

    window_start=last_sample=start;window_scale=scale;
    report.flags|=1u;report.window_ms=ms(start);report.gen_first=report.gen_last=gen;
    report.core_hz=SystemCoreClock;report.ticks_per_ms=scale;
    report.rcc_cfgr0=RCC->CFGR0;report.exten_ctlr=EXTEN->EXTEN_CTR;report.systick_ctlr=STK_CTLR;
    for(unsigned i=0;i<4;++i)report.baseline[i]=report.minimum[i]=report.maximum[i]=center[i];
}
void sample(uint64_t now,uint32_t gen,uint64_t age){
#if BMCU_USB_CONTINUOUS_JOURNAL
    return;
#endif

    ++report.samples;
    if(now<last_sample){report.flags|=8u;return;}
    const uint32_t gap=ms(now-last_sample);if(gap>report.max_gap_ms)report.max_gap_ms=gap;
    last_sample=now;
    const uint32_t aged=ms(age);if(aged>report.max_age_ms)report.max_age_ms=aged;
    if(gen!=report.gen_last){report.gen_last=gen;}
}
void pull(uint8_t ch,float value,float delta){
#if BMCU_USB_CONTINUOUS_JOURNAL
    return;
#endif

    comparator_delta[ch]=delta;
    report.flags|=uint32_t(1u)<<(8+ch);
    if(value<report.minimum[ch])report.minimum[ch]=value;
    if(value>report.maximum[ch])report.maximum[ch]=value;
    if(delta>report.peak_delta[ch])report.peak_delta[ch]=delta;
}

void progress(uint8_t candidate,uint64_t held,bool confirmed,uint64_t released,uint64_t now){
#if BMCU_USB_CONTINUOUS_JOURNAL
    return;
#endif

    const uint8_t bits=(held?1:0)|(confirmed?2:0)|(released?4:0);
    const uint32_t elapsed=ms(now-window_start);
    if(candidate!=report.candidate||bits!=report.progress){
        if(report.event_count<12){
            report.event_delta[report.event_count]=candidate<4u?comparator_delta[candidate]:0.f;
            report.events[report.event_count++]={uint16_t(elapsed>65535?65535:elapsed),bits,candidate};
        }
        else report.flags|=4u;
        if(confirmed&&!(report.progress&2u))report.confirmed_ms=elapsed;
    }
    if(held&&!(report.progress&2u)){const auto n=ms(now-held);if(n>report.hold_ms)report.hold_ms=n;}
    if(released){const auto n=ms(now-released);if(n>report.release_ms)report.release_ms=n;}
    report.candidate=candidate;report.progress=bits;
}
static bool persist(){
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
    if(finished||!capturing)return verified_saved;
#else
    if(finished||!capturing)return !storage_error;
#endif
#if BMCU_LITE_LEASE_FEEDBACK
    if(!Flash_background_idle())return false; // A busy refusal must not freeze a retryable capture.
#endif
#if BMCU_LITE_V11_FEED_LATCH
    const uint32_t irq=irq_save_wch();
    finished=true;irq_restore_wch(irq);
#else
    finished=true;
#endif
#if !BMCU_LITE_LEASE_FEEDBACK
    if(!Flash_background_idle())return false; // Marker truthfully remains incomplete.
#endif
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
    if(!operator_stop){diagnostic_snapshot(2u,255u);authority_observation::snapshot(report.ao4);}
#else
    authority_observation::snapshot(report.ao4);
#endif
    bool ok=write(base+508,guard);
    const uint8_t* p=(const uint8_t*)&report;
    for(unsigned n=0;n<sizeof(Report)&&ok;n+=4)ok=write(base+32+n,get32(p+n));
    if(ok)ok=write(base+28,crc(&report,sizeof(report)));
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
    if(ok)ok=write(base+24,commit);
    if(ok){Flash_log_read(frame+20);verified_saved=complete(frame+20);}
    return verified_saved;
#else
    return ok&&write(base+24,commit); // Commit last, after all verified halfwords.
#endif
}
void finish_setup(uint8_t phase,uint8_t reached,uint8_t fault,uint8_t detail){
#if BMCU_USB_CONTINUOUS_JOURNAL
    jevent(2,255,uint32_t(phase)|(uint32_t(reached)<<8)|(uint32_t(fault)<<16)|(uint32_t(detail)<<24));return;
#endif

    report.phase=phase;report.reached=reached;report.fault=fault;report.detail=detail;report.flags|=2u;
    persist();
}
#if BMCU_LITE_CONTINUOUS_NATIVE
#if BMCU_LITE_ONUSE_IDLE_COMPAT
void native_started(uint32_t flags){
#if BMCU_USB_CONTINUOUS_JOURNAL
    static uint32_t previous=0;if(flags!=previous){previous=flags;jevent((flags&((1u<<21)|(1u<<22)))?10:3,255,flags);}return;
#endif
if(!finished)report.flags|=flags;}
#endif
void setup_completed(){
#if BMCU_USB_CONTINUOUS_JOURNAL
    jevent(2,255,uint32_t(SetupPhase::success)|(5u<<8));return;
#endif
    report.flags|=8192u;report.reached=5u;
}
#endif
void native_completed(){
#if BMCU_USB_CONTINUOUS_JOURNAL
    jevent(6,255,0);return;
#endif
report.flags|=32u;}
bool native_end_pending(){return (report.flags&32u)
#if BMCU_LITE_NATIVE_UNLOAD_CONTINUE
    &&!finished
#endif
    ;}
#if !BMCU_LITE_ONUSE_IDLE_COMPAT
void native_started(){report.flags|=128u;}
#endif
void native_interrupted(){
#if BMCU_USB_CONTINUOUS_JOURNAL
    static bool seen=false;if(!seen){seen=true;jevent(4,255,0);}return;
#endif
if(report.flags&128u)report.flags|=4096u;}
void native_cancelled(){
#if BMCU_USB_CONTINUOUS_JOURNAL
    jevent(5,255,0);return;
#endif
if(report.flags&128u)report.flags|=64u;}
bool native_checkpoint_pending(){
#if BMCU_USB_CONTINUOUS_JOURNAL
    return false;
#endif
return (report.flags&(
#if !BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
    64u|
#endif
    4096u
#if BMCU_LITE_NATIVE_UNLOAD_CONTINUE
    |32u
#endif
    ))&&!finished;}
#if BMCU_LITE_LEASE_FEEDBACK
void onuse_meter(uint8_t ch,bool initial){
#if BMCU_USB_CONTINUOUS_JOURNAL
    (void)ch;(void)initial;return;
#endif

    auto& d=report.diagnostic;
    if(finished || !full_load_ready())return;
    if(initial){
#if BMCU_LITE_AUTOMATIC_CAPTURE
        if(!(report.flags&(1u<<20))){
#if BMCU_LITE_PRECUT_PHASE_CAPTURE
            const uint64_t origin=time_ticks64();phase_origin=uint32_t(origin);automatic_deadline=origin+uint64_t(time_hw_tpms)*120000u;
#else
            automatic_deadline=time_ticks64()+uint64_t(time_hw_tpms)*120000u;
#endif
            report.flags|=1u<<20;
#if BMCU_LITE_PRECUT_PHASE_CAPTURE
            phase_consumed(); // First on_use is now admitted; include its boundary request.
#endif
        }
#endif
        d.reserved=uint8_t(1u|(ch<<2));
        d.onuse_meters[0]=d.onuse_meters[1]=Motion_control_filament_meters(ch);
        d.onuse_meters[2]=0.f;
    }else if((d.reserved&13u)==(1u|(ch<<2))){
        d.reserved|=2u; // A later valid existing encoder publication was observed.
        d.onuse_meters[1]=Motion_control_filament_meters(ch);
        const float change=fabsf(d.onuse_meters[1]-d.onuse_meters[0]);
        if(change>d.onuse_meters[2])d.onuse_meters[2]=change;
    }
}
#endif
bool finish_native(uint8_t fault){
#if BMCU_USB_CONTINUOUS_JOURNAL
    jevent(2,255,uint32_t(fault)<<16);return true;
#endif

#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
    if(finished)return verified_saved;
#endif
#if BMCU_LITE_INPUT_FAULT_DETAIL
    report.native_reserved=0xa4u;
#endif
    report.phase=uint8_t(SetupPhase::operation);report.native_fault=fault;report.flags|=16u;
    return persist();
}
void uart_init(){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1,ENABLE);
    USART_DeInit(USART1);
    GPIO_InitTypeDef gpio={0};gpio.GPIO_Speed=GPIO_Speed_50MHz;
    GPIOA->BSHR=GPIO_Pin_12;gpio.GPIO_Pin=GPIO_Pin_12;gpio.GPIO_Mode=GPIO_Mode_Out_PP;GPIO_Init(GPIOA,&gpio);
    gpio.GPIO_Pin=GPIO_Pin_9;gpio.GPIO_Mode=GPIO_Mode_AF_PP;GPIO_Init(GPIOA,&gpio);
    gpio.GPIO_Pin=GPIO_Pin_10;gpio.GPIO_Mode=GPIO_Mode_IN_FLOATING;GPIO_Init(GPIOA,&gpio);
    USART_InitTypeDef uart={0};uart.USART_BaudRate=115200;uart.USART_WordLength=USART_WordLength_8b;
    uart.USART_StopBits=USART_StopBits_1;uart.USART_Parity=USART_Parity_No;
    uart.USART_HardwareFlowControl=USART_HardwareFlowControl_None;uart.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
    USART_Init(USART1,&uart);USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
    USART_DMACmd(USART1,USART_DMAReq_Rx|USART_DMAReq_Tx,DISABLE);USART_Cmd(USART1,ENABLE);
}
void export_step(){
#if BMCU_USB_CONTINUOUS_JOURNAL
    jexport_step();return;
#endif

    // Fixed RX length, bounded per-step work and 250ms partial-command expiry.
    const uint32_t now=time_ticks32();
    if(rx&&(uint32_t)(now-rx_at)>ms_to_ticks32(250))rx=0;
    for(unsigned i=0;i<8&&USART_GetFlagStatus(USART1,USART_FLAG_RXNE)!=RESET;++i){
        const uint8_t v=uint8_t(USART_ReceiveData(USART1));rx_at=now;
        if(!rx&&v!='B')continue;
        ack[rx++]=v;
        if(rx==20){
            rx=0;
            if(memcmp(ack,"BFA1",4)||get32(ack+4)!=build||get32(ack+8)!=session||
               get32(ack+12)!=frame_crc||get32(ack+16)!=crc(ack,16))continue;
            bool ok=acknowledged;
            if(!ok&&header_valid(frame+20)&&!storage_error){
                // Rewrite the adjacent erased halfword exactly as read; never erase
                // on USB. A failed/torn ACK cannot arm immediate execution.
                const uint32_t state=get32(frame+40);
                if(blank(state))ok=write(base+20,(state&0xffffu)|(uint32_t(armed)<<16));
                else ok=uint16_t(state>>16)==armed;
            }
            memcpy(reply,ok?"BFK1":"BFE1",4);put32(reply+4,build);put32(reply+8,session);
            put32(reply+12,frame_crc);put32(reply+16,crc(reply,16));
            acknowledged=ok;tx=0;tx_at=now;
        }
    }
    const bool responding=reply[0]!=0;const unsigned size=responding?20:sizeof(frame);
    if(tx==size){if((uint32_t)(now-tx_at)<ms_to_ticks32(100))return;tx=0;}
    for(unsigned i=0;i<8&&tx<size&&USART_GetFlagStatus(USART1,USART_FLAG_TXE)!=RESET;++i)
        USART_SendData(USART1,(responding?reply:frame)[tx++]);
    if(tx==size)tx_at=now;
}
}
#endif
