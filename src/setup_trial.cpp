#include "lite_profile.h"
#if BMCU_LITE_SETUP_TRIAL
#include "setup_trial.h"
#include "flight_recorder.h"
#include "Flash_saves.h"
#include "MC_PULL_calibration.h"
#include "ADC_DMA.h"
#include "hal/time_hw.h"
#include "hal/irq_wch.h"
#include "app_api.h"
#if BMCU_LITE_V11_FEATURES
#include "Motion_control.h"
#include "_bus_hardware.h"
#endif
#include "ws2812.h"
#include <math.h>
extern WS2812_class SYS_RGB;
extern WS2812_class RGBOUT[4];
extern void RGB_update();
static void display(bool all);
static uint8_t gesture_ack_channel=0xffu;
static uint64_t gesture_ack_start=0;
static bool gesture_window_open=false;
static void gesture_ack_clear() {
    if (gesture_ack_channel<4u)
        for (uint8_t i=0;i<2;++i) RGBOUT[gesture_ack_channel].set_RGB(0,0,0,i);
    gesture_ack_channel=0xffu;
    gesture_ack_start=0;
}
static void gesture_cue_clear() {
    gesture_window_open=false;
    gesture_ack_clear();
    SYS_RGB.set_RGB(0,0,0,0);
    for (auto &led:RGBOUT) for (uint8_t i=0;i<2;++i) led.set_RGB(0,0,0,i);
}
static SetupPhase phase = SetupPhase::inspect;
static SetupFault fault = SetupFault::none;
static bool selected = false, calibrated = false;
#if BMCU_LITE_V11_FEATURES
static bool maintenance=false, maintenance_clearing=false;
bool setup_trial_maintenance_clearing() { return maintenance && maintenance_clearing; }
#endif

static uint8_t reached = 0;
// Frozen first-fault blue detail: admission4/6 or gesture timeout9; parent-specific.
static uint8_t boot_admission_detail = 0;
static uint32_t scale = 0, generation = 0;
static uint64_t last_ticks = 0, origin = 0, phase_origin = 0, adc_progress = 0;
SetupPhase setup_trial_phase() { return phase; }
SetupFault setup_trial_fault() { return fault; }
bool setup_trial_bus_allowed() { return selected && phase ==
#if BMCU_LITE_FULL_LOAD
    SetupPhase::operation;
#else
    SetupPhase::trial;
#endif
}
bool setup_trial_selected() { return selected; }
void setup_trial_fail(SetupFault why) {
    const uint32_t irq=irq_save_wch();
#if BMCU_LITE_FULL_LOAD
    if (phase == SetupPhase::operation) { full_load_fail(FullLoadFault::capability); irq_restore_wch(irq); return; }
#else
    if (phase == SetupPhase::trial) { irq_restore_wch(irq); return; }
#endif
    if (fault == SetupFault::none) fault = why;
    gesture_cue_clear(); // Replace pending live/ACK colors before terminal delivery.
    phase = SetupPhase::fault;
    Motion_control_setup_stop();
    irq_restore_wch(irq);
}
void setup_trial_admission_fail(SetupFault why,uint8_t detail) {
    const uint32_t irq=irq_save_wch();
    if (phase==SetupPhase::inspect && fault==SetupFault::none &&
        ((why==SetupFault::adc && detail==6u) ||
         (why==SetupFault::module && detail>=1u && detail<=5u)))
        boot_admission_detail=detail; // Snapshot before stop/cleanup; never recalculate.
    setup_trial_fail(why);
    irq_restore_wch(irq);
}
static bool clock_ok() {
    const uint64_t now = time_ticks64();
    if (!scale || time_hw_tpms != scale || now < last_ticks ||
        now-origin > uint64_t(scale)*300000u) {
        setup_trial_fail(SetupFault::clock); return false;
    }
    last_ticks = now;
    return true;
}
static bool cal_retained() {
    float o[4], lo[4], hi[4], dm[4]; int8_t pol[4]; uint8_t valid = 0;
    if (!Flash_MC_PULL_cal_read(o,lo,hi,pol,dm,&valid) || valid != 15u) return false;
    for (uint8_t ch=0; ch<4; ++ch)
        if (!isfinite(o[ch]) || 1.65f-o[ch]<.05f || 1.65f-o[ch]>3.25f ||
            !isfinite(lo[ch]) || !isfinite(hi[ch]) || 1.65f-lo[ch]<.05f || hi[ch]-1.65f<.05f ||
            hi[ch]-lo[ch]<.12f || !isfinite(dm[ch]) || dm[ch]<0.01f*60.0f || dm[ch]>1.39f ||
            (pol[ch]!=1 && pol[ch]!=-1)) return false;
    return true;
}
bool setup_trial_poll(bool centered) {
#if BMCU_LITE_FULL_LOAD
    if (phase == SetupPhase::operation) return false;
#endif
    if (phase == SetupPhase::trial || phase == SetupPhase::fault || phase == SetupPhase::success) return false;
    if (!clock_ok()) return false;
    const uint64_t now=last_ticks;
    if (phase == SetupPhase::direction && now-phase_origin > uint64_t(scale)*2500u) {
        setup_trial_fail(SetupFault::direction); return false;
    }
    ADC_DMA_poll();
    if (!ADC_DMA_ready()) { setup_trial_fail(SetupFault::adc); return false; }
    const uint32_t got=ADC_DMA_generation();
    if (got!=generation) { generation=got; adc_progress=now; }
    if (now-adc_progress > uint64_t(scale)*100u) { setup_trial_fail(SetupFault::adc); return false; }
    if (Flash_setup_fault() || (phase!=SetupPhase::journal && !Flash_background_idle())) { setup_trial_fail(SetupFault::storage); return false; }
    const bool ok=Motion_control_setup_inputs(centered,calibrated);
    if (!ok || phase==SetupPhase::fault || !clock_ok()) return false;
    display(false);
    return phase!=SetupPhase::fault;
}
bool setup_trial_write_allowed(SetupWrite kind) {
    const bool allowed = (kind==SetupWrite::cal && phase==SetupPhase::calibration) ||
        (kind==SetupWrite::motion && phase==SetupPhase::direction) ||
        (kind==SetupWrite::journal && phase==SetupPhase::journal);
    return allowed && setup_trial_poll(true);
}
bool setup_trial_flash_address(uint32_t address) {
#if BMCU_LITE_V11_FEATURES
    if (maintenance && phase==SetupPhase::calibration)
        return ((address>=0x0800f000u && address<0x0800f100u) ||
                (address>=0x0800ff00u && address<0x08010000u)) && setup_trial_poll(true);
#endif
    if (phase==SetupPhase::calibration) return address>=0x0800f000u && address<0x0800f100u && setup_trial_poll(true);
    if (phase==SetupPhase::direction) return address>=0x0800f100u && address<0x0800f200u && setup_trial_poll(true);
    // Fresh setup only uses its first erased journal slot; rollover is never permitted.
    if (phase==SetupPhase::journal) return address>=0x0800f600u && address<0x0800f608u && setup_trial_poll(true);
    return false;
}
bool setup_trial_drive_allowed(uint8_t ch,int pwm) {
    if (phase!=SetupPhase::direction || ch>=4 || pwm<0 || pwm>1000) {
        setup_trial_fail(SetupFault::capability); return false;
    }
    return setup_trial_poll(true);
}
uint32_t setup_trial_led_rgb(uint64_t ticks) {
    if (!scale || fault==SetupFault::clock) return 0x180000u; // Timing cannot be trusted.
    if (fault==SetupFault::none && (phase==SetupPhase::inspect || phase==SetupPhase::gesture)) {
#if BMCU_LITE_SETUP_RELEASE_CUE
        if (phase==SetupPhase::gesture && gesture_ack_channel<4u) return 0x001800u;
#endif
        return gesture_window_open?0x000018u:0x180018u;
    }
    const uint32_t ms = uint32_t(((ticks>=origin?ticks-origin:0u)/scale)%30000u);
    if (ms<600u) return 0x101010u;
    const uint8_t stage=reached; // Preserve the last reached stage after a fault.
    if (ms>=2000 && ms<2000u+uint32_t(stage)*1000u && (ms-2000)%1000<500) return 0x001800u;
    if (ms>=9000 && ms<9000u+uint32_t(fault)*1000u && (ms-9000)%1000<500) return 0x180000u;
    if (phase==SetupPhase::fault && boot_admission_detail && ms>=24000u &&
        ms<24000u+uint32_t(boot_admission_detail)*1000u && (ms-24000u)%1000u<500u) return 0x000018u;
    return 0;
}
static void display(bool all) {
    const uint64_t now=time_ticks64();
    const uint32_t rgb=setup_trial_led_rgb(now);
    SYS_RGB.set_RGB(rgb>>16,rgb>>8,rgb,0);
#if !BMCU_LITE_SETUP_RELEASE_CUE
    if (phase==SetupPhase::gesture) {
        const uint64_t elapsed=now>=gesture_ack_start?now-gesture_ack_start:0;
        const bool ack=gesture_ack_channel<4u && scale && now>=gesture_ack_start &&
            elapsed<uint64_t(scale)*2000u;
        for (uint8_t ch=0;ch<4;++ch) for (uint8_t i=0;i<2;++i) {
            const uint32_t color=(ack && ch==gesture_ack_channel)?
                ((elapsed/scale)%1000u<500u?0x001800u:0u):rgb;
            RGBOUT[ch].set_RGB(color>>16,color>>8,color,i);
        }
    }
#else
    all = all || phase==SetupPhase::gesture; // One steady release cue on every LED.
#endif
    if (all) for (auto &led:RGBOUT) for(uint8_t i=0;i<2;++i) led.set_RGB(rgb>>16,rgb>>8,rgb,i);
    RGB_update();
}
static void wait_tick(bool all=false) { display(all); delay(1); }
[[noreturn]] static void park() {
    gesture_cue_clear(); // PARK never advertises a live input window or pending ACK.
    Motion_control_setup_stop();
#if BMCU_USB_FLIGHT_RECORDER
    flight::finish_setup(uint8_t(phase),reached,uint8_t(fault),boot_admission_detail);
#endif
    for (;;) {
#if BMCU_USB_CONTINUOUS_JOURNAL
        Flash_background_run();flight::journal_step(); // Stopped outputs; diagnostic writes only.
#endif
        wait_tick(true); // No bus/control revival; power cycle is required.
    }
}
static bool stable_empty(uint32_t ms) {
    float low[4], high[4];
    for (uint8_t ch=0;ch<4;++ch) low[ch]=high[ch]=ADC_DMA_get_value()[6-2*ch];
    const uint64_t start=time_ticks64();
    while (time_ticks64()-start < uint64_t(scale)*ms) {
        if (!setup_trial_poll(true)) return false;
        const float *v=ADC_DMA_get_value();
        for(uint8_t ch=0;ch<4;++ch) {
            const float raw=v[6-2*ch]; if(raw<low[ch])low[ch]=raw; if(raw>high[ch])high[ch]=raw;
            if(high[ch]-low[ch]>.04f){setup_trial_fail(SetupFault::adc);return false;}
        }
        wait_tick();
    }
    return true;
}
static bool gesture() {
    // Real empty/neutral admission precedes intent. Each buffer uses its own
    // measured baseline; a short hold may retry only the locked candidate.
    if (!stable_empty(1500)) return false;
    float center[4];
    for (uint8_t ch=0;ch<4;++ch) center[ch]=ADC_DMA_get_value()[6-2*ch];
    const uint64_t start=time_ticks64(); uint64_t held=0,released=0;
#if BMCU_USB_FLIGHT_RECORDER
    flight::window(center,start,ADC_DMA_generation(),scale);
#endif
    gesture_window_open=true; // All four center floats are frozen; budget starts above.
    display(false); // Queue and deliver readiness before the first gesture sample.
    uint8_t candidate=0xffu;
    bool confirmed=false;
    while (time_ticks64()-start < uint64_t(scale)*15000u) {
        if (!setup_trial_poll()) return false;
        const float *v=ADC_DMA_get_value();
#if BMCU_USB_FLIGHT_RECORDER
        flight::sample(last_ticks,ADC_DMA_generation(),last_ticks-adc_progress);
#endif
        uint8_t moving=0xffu;
        for (uint8_t ch=0;ch<4;++ch) {
#if BMCU_USB_FLIGHT_RECORDER
            const float raw=v[6-2*ch], delta=fabsf(raw-center[ch]);
            flight::pull(ch,raw,delta); // The exact value/delta tested below, before early rejection.
            if (delta>=.15f) {
#else
            if (fabsf(v[6-2*ch]-center[ch])>=.15f) {
#endif
            if (moving<4u || (candidate<4u && ch!=candidate)) {
                setup_trial_fail(SetupFault::adc); return false;
            }
            moving=ch;
            }
        }
        if (candidate>=4u && moving<4u) candidate=moving;
        for (uint8_t ch=0;ch<4;++ch) if (ch!=candidate && fabsf(v[6-2*ch]-1.65f)>.25f) {
            setup_trial_fail(SetupFault::adc); return false;
        }
        const uint64_t now=time_ticks64();
        const float delta=candidate<4u?v[6-2*candidate]-center[candidate]:0.f;
        if (!confirmed) {
            if (fabsf(delta)>=.15f) {
                if (!held) held=now;
                if (now-held>=uint64_t(scale)*(BMCU_LITE_SETUP_GESTURE_1S?1000u:2000u)) {
                    confirmed=true;
                    gesture_ack_channel=candidate;
                    gesture_ack_start=now; // One two-pulse sequence; never a capability.
                }
            } else held=0;
        } else {
            if (fabsf(delta)<=.04f) {
                if (!released) released=now;
                if (now-released>=uint64_t(scale)*1200u) {
#if BMCU_USB_FLIGHT_RECORDER
                    flight::progress(candidate,held,confirmed,released,now);
#endif
                    gesture_window_open=false; // Final stability is outside the input window.
                    display(false); // Replace blue while preserving the full accepted-hold ACK.
                    const bool ready=stable_empty(1000);
                    gesture_cue_clear(); // No stale delivery over manual CAL prompts.
                    return ready;
                }
            } else released=0;
        }
#if BMCU_USB_FLIGHT_RECORDER
        flight::progress(candidate,held,confirmed,released,now);
#endif
        wait_tick(); // Existing1ms cadence, with live safety polls on every iteration.
    }
    const uint32_t irq=irq_save_wch();
    if (phase==SetupPhase::gesture && fault==SetupFault::none)
        boot_admission_detail=confirmed?3u:(candidate<4u?2u:1u);
    setup_trial_fail(SetupFault::gesture_timeout); // Freeze locals before stop/ACK cleanup.
    irq_restore_wch(irq);
    return false;
}

#if BMCU_LITE_V11_FEATURES
static void run_empty_setup() {
    if (!scale || !Motion_control_setup_init()) { if (fault==SetupFault::none) setup_trial_fail(SetupFault::encoder); park(); }
    phase=SetupPhase::gesture; reached=1;
    if (!gesture()) park();
    phase=SetupPhase::calibration; reached=2; phase_origin=time_ticks64();
    MC_PULL_calibration_boot();
    if (fault!=SetupFault::none) park();
    if (!cal_retained()) { setup_trial_fail(SetupFault::calibration); park(); }
    for(uint8_t ch=0;ch<4;++ch) if (!MC_PULL_calibration_is_valid(ch)) {
        setup_trial_fail(SetupFault::calibration); park();
    }
    calibrated=true;
    if (!stable_empty(1500)) park();
    phase=SetupPhase::direction; reached=3; phase_origin=time_ticks64();
    if (!Motion_control_setup_direction()) { if(fault==SetupFault::none)setup_trial_fail(SetupFault::direction); park(); }
    if (!Motion_control_setup_retained()) { setup_trial_fail(SetupFault::verification); park(); }
    phase=SetupPhase::journal; reached=4; phase_origin=time_ticks64();
    if (!stable_empty(1500)) park();
    const SetupStorage before=Flash_setup_probe();
    const bool journal_ok=maintenance ? (before.journal_valid && before.owner==0xffu && !before.journal_invalid) :
        (before.journal_erased && !before.journal_valid && !before.journal_invalid);
    if (before.fault || before.busy || !journal_ok ||
        !cal_retained() || !Motion_control_setup_retained()) { setup_trial_fail(SetupFault::verification); park(); }
    const uint64_t start=time_ticks64();
    bool committed=maintenance;
    while (!committed && time_ticks64()-start < uint64_t(scale)*500u) {
        if (!setup_trial_poll(true)) park();
        if (Flash_AMS_state_write(0xffu)) { committed=true; break; }
        Flash_background_run(); wait_tick();
    }
    if (!committed) { setup_trial_fail(SetupFault::write); park(); }
    const SetupStorage after=Flash_setup_probe();
    if (after.fault || after.busy || after.journal_invalid || !after.journal_valid || after.owner!=0xffu ||
        !cal_retained() || !Motion_control_setup_retained() || !setup_trial_poll(true)) {
        setup_trial_fail(SetupFault::verification); park();
    }
    phase=SetupPhase::success; reached=5;
#if BMCU_LITE_CONTINUOUS_NATIVE
    if (!maintenance) {
        Motion_control_setup_stop(); // Gate remains closed until every output is zero.
        flight::setup_completed();
        phase=SetupPhase::operation;
        return; // Existing main performs strict retained readers before bus admission.
    }
#endif
    park();
}
void setup_trial_empty_maintenance() {
    // An explicit empty runtime transition, not a new permissive cold-boot class.
    if (phase!=SetupPhase::operation || !v11_maintenance_allowed() || !Flash_background_idle()) return;
    const SetupStorage before=Flash_setup_probe();
    if (before.fault || before.busy || before.journal_invalid || !before.journal_valid || before.owner!=0xffu) {
        full_load_fail(FullLoadFault::storage); return;
    }
    for (uint8_t ch=0;ch<4;++ch) if (Motion_control_filament_present(ch)) return;
    v11_local_preempt(); Motion_control_setup_stop(); bus_shutdown();
    maintenance=true; maintenance_clearing=true; calibrated=false;
    scale=time_hw_tpms; origin=last_ticks=phase_origin=adc_progress=time_ticks64();
    generation=ADC_DMA_generation();
    phase=SetupPhase::gesture; reached=1;
    // The reset is requested while held. Wait for a stable release before
    // invoking strict setup admission; no erase occurs during this wait.
    float low[4],high[4];
    for (uint8_t ch=0;ch<4;++ch) low[ch]=high[ch]=ADC_DMA_get_value()[6-2*ch];
    uint64_t released=time_ticks64();
    for (;;) {
        if (!setup_trial_poll()) park();
        bool stable=true;
        const float *v=ADC_DMA_get_value();
        for (uint8_t ch=0;ch<4;++ch) {
            if (v[6-2*ch]<low[ch]) low[ch]=v[6-2*ch];
            if (v[6-2*ch]>high[ch]) high[ch]=v[6-2*ch];
            stable &= fabsf(v[6-2*ch]-1.65f)<=.25f && high[ch]-low[ch]<=.04f;
        }
        if (!stable) {
            released=time_ticks64();
            for (uint8_t ch=0;ch<4;++ch) low[ch]=high[ch]=v[6-2*ch];
        } else if (time_ticks64()-released>=uint64_t(scale)*1500u) break;
        wait_tick();
    }
    phase=SetupPhase::calibration;
    if (!MC_PULL_calibration_clear()) { setup_trial_fail(SetupFault::write); park(); }
    phase=SetupPhase::direction; phase_origin=time_ticks64();
    if (!Flash_Motion_clear()) { setup_trial_fail(SetupFault::write); park(); }
    maintenance_clearing=false; phase=SetupPhase::inspect; phase_origin=time_ticks64();
    // Keep the verified FF journal and all filament settings. Interrupted
    // maintenance remains incomplete and parks on cold boot under the old rules.
    run_empty_setup();
}
#endif
void setup_trial_boot() {
#if BMCU_LITE_FULL_LOAD
    if (selected) { if (setup_trial_bus_allowed()) return; park(); }
#else
    if (selected) { if (phase==SetupPhase::trial) return; park(); }
#endif
    if (phase!=SetupPhase::inspect) { selected=true; park(); }
    selected=true; scale=time_hw_tpms; origin=last_ticks=phase_origin=adc_progress=time_ticks64();
    generation=ADC_DMA_generation();
    if (!scale) { setup_trial_fail(SetupFault::clock); park(); }
    const SetupStorage s=Flash_setup_probe();
    if (s.fault || s.busy) { setup_trial_fail(SetupFault::storage); park(); }
#if !BMCU_LITE_FULL_LOAD
    if (s.journal_valid && s.owner!=0xffu) { setup_trial_fail(SetupFault::loaded_owner); park(); }
#endif
    if (s.journal_invalid || s.cal_invalid || (!s.motion_erased && !s.motion_schema)) {
        setup_trial_fail(SetupFault::storage); park();
    }
    const bool real_cal=s.cal_schema && cal_retained();
    const bool real_motion=s.motion_schema && Motion_control_setup_retained();
    if (s.journal_valid &&
#if !BMCU_LITE_FULL_LOAD
        s.owner==0xffu &&
#endif
        real_cal && real_motion) {
        if (!Motion_control_setup_init(false) || !clock_ok()) park();
#if BMCU_LITE_FULL_LOAD
        phase=SetupPhase::operation; return;
#else
        phase=SetupPhase::trial; return;
#endif
    }
    if (!s.journal_erased || !s.cal_erased || !s.motion_erased) {
        setup_trial_fail(SetupFault::incomplete); park();
    }
#if BMCU_LITE_V11_FEATURES
    run_empty_setup();
#else
    if (!scale || !Motion_control_setup_init()) { if (fault==SetupFault::none) setup_trial_fail(SetupFault::encoder); park(); }
    phase=SetupPhase::gesture; reached=1;
    if (!gesture()) park();
    phase=SetupPhase::calibration; reached=2; phase_origin=time_ticks64();
    MC_PULL_calibration_boot();
    if (fault!=SetupFault::none) park();
    if (!cal_retained()) { setup_trial_fail(SetupFault::calibration); park(); }
    for(uint8_t ch=0;ch<4;++ch) if (!MC_PULL_calibration_is_valid(ch)) {
        setup_trial_fail(SetupFault::calibration); park();
    }
    calibrated=true;
    if (!stable_empty(1500)) park();
    phase=SetupPhase::direction; reached=3; phase_origin=time_ticks64();
    if (!Motion_control_setup_direction()) { if(fault==SetupFault::none)setup_trial_fail(SetupFault::direction); park(); }
    if (!Motion_control_setup_retained()) { setup_trial_fail(SetupFault::verification); park(); }
    phase=SetupPhase::journal; reached=4; phase_origin=time_ticks64();
    if (!stable_empty(1500)) park();
    const SetupStorage before=Flash_setup_probe();
    if (before.fault || before.busy || !before.journal_erased || before.journal_valid || before.journal_invalid ||
        !cal_retained() || !Motion_control_setup_retained()) { setup_trial_fail(SetupFault::verification); park(); }
    const uint64_t start=time_ticks64();
    bool committed=false;
    while (time_ticks64()-start < uint64_t(scale)*500u) {
        if (!setup_trial_poll(true)) park();
        if (Flash_AMS_state_write(0xffu)) { committed=true; break; }
        Flash_background_run(); wait_tick();
    }
    if (!committed) { setup_trial_fail(SetupFault::write); park(); }
    const SetupStorage after=Flash_setup_probe();
    if (after.fault || after.busy || after.journal_invalid || !after.journal_valid || after.owner!=0xffu ||
        !cal_retained() || !Motion_control_setup_retained() || !setup_trial_poll(true)) {
        setup_trial_fail(SetupFault::verification); park();
    }
    phase=SetupPhase::success; reached=5;
    park();
#endif
}
#endif
