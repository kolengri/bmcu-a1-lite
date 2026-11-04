#include "lite_profile.h"
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL
#include "hal/time_hw.h"
#include "ams.h"
#include "app_api.h"
#if BMCU_LITE_OWNERSHIP_DETAIL
#include "hal/irq_wch.h"
#endif

// RAM-only, one session per power-on. No reset/fault-clear wire command exists.
struct TrialState {
    bool boot_checked = false;
    bool active = false;
    bool dispatched = false;
    bool started = false;
    TrialVerb last = TrialVerb::none;
    TrialStop stop = TrialStop::none;
#if BMCU_LITE_OWNERSHIP_DETAIL
    TrialOwnershipDetail ownership_detail = TrialOwnershipDetail::none;
#endif
    uint64_t start_ticks = 0;
    uint64_t last_ticks = 0;
    uint32_t scale = 0;
    uint32_t previous_count = 0;
    uint32_t traveled_count = 0;
};
static TrialState trial;

bool single_trial_fault() { return trial.stop != TrialStop::none; }
bool single_trial_active() { return trial.active && !single_trial_fault(); }
TrialStop single_trial_reason() { return trial.stop; }
TrialVerb single_trial_last_verb() { return trial.last; }

#if BMCU_LITE_OWNERSHIP_DETAIL
TrialOwnershipDetail single_trial_ownership_detail() { return trial.ownership_detail; }
#endif
void single_trial_stop(TrialStop reason
#if BMCU_LITE_OWNERSHIP_DETAIL
    , TrialOwnershipDetail detail
#endif
) {
#if BMCU_LITE_OWNERSHIP_DETAIL
    const uint32_t irq = irq_save_wch();
    if (reason == TrialStop::none || single_trial_fault()) { irq_restore_wch(irq); return; }
    trial.ownership_detail = reason == TrialStop::ownership ? detail : TrialOwnershipDetail::none;
    trial.stop = reason; // Publish parent/detail under the same first-fault latch.
    irq_restore_wch(irq);
#else
    if (reason == TrialStop::none || single_trial_fault()) return;
    trial.stop = reason; // First terminal cause survives later retries/heartbeats.
#endif
    trial.active = false;
    trial.dispatched = false;
    Motion_control_lite_cancel();
    for (auto &f : ams[0].filament) f.motion = _filament_motion::idle;
    ams[0].now_filament_num = 0xFFu;
    ams[0].filament_use_flag = 0;
    // Do not claim completion or clear/remap the retained physical ownership.
}
void single_trial_boot_ownership(bool readable, uint8_t loaded) {
    if (trial.boot_checked) return; // Re-initialization cannot clear the budget/fault.
    trial.boot_checked = true;
    if (!readable || (loaded != 0xFFu && loaded != 0u)) {
#if BMCU_LITE_OWNERSHIP_DETAIL
        TrialOwnershipDetail detail = TrialOwnershipDetail::boot_unclassified_refusal;
        if (!readable) {
            switch (Flash_trial_ownership_read_observation()) {
            case TrialOwnershipRead::flash_fault: detail = TrialOwnershipDetail::boot_flash_fault; break;
            case TrialOwnershipRead::flash_busy: detail = TrialOwnershipDetail::boot_flash_busy; break;
            case TrialOwnershipRead::no_valid_record: detail = TrialOwnershipDetail::boot_no_valid_record; break;
            default: break; // Do not normalize an unclassified refusal to an empty journal.
            }
        } else if (loaded == 1u) detail = TrialOwnershipDetail::boot_owner_UI2;
        else if (loaded == 2u) detail = TrialOwnershipDetail::boot_owner_UI3;
        else if (loaded == 3u) detail = TrialOwnershipDetail::boot_owner_UI4;
        single_trial_stop(TrialStop::ownership, detail);
#else
        single_trial_stop(TrialStop::ownership);
#endif
    }
}
void single_trial_poll() {
    if (!trial.active || single_trial_fault()) return;
    const uint64_t now = time_ticks64();
    if (!time_hw_tpms || time_hw_tpms != trial.scale || now < trial.last_ticks) {
        single_trial_stop(TrialStop::controller_interlock);
        return;
    }
    trial.last_ticks = now; // Preserve observation history across retries and init calls.
    const uint32_t current = Motion_control_trial_encoder();
    const uint32_t delta = current - trial.previous_count;
    const uint32_t absolute = (delta & 0x80000000u) ? 0u - delta : delta;
    trial.previous_count = current;
    const uint32_t cap = Motion_control_trial_travel_cap();
    if (trial.traveled_count >= cap || absolute >= cap - trial.traveled_count) {
        single_trial_stop(TrialStop::travel_cap);
        return;
    }
    trial.traveled_count += absolute;
    if (now - trial.start_ticks >= uint64_t(kTrialMaxMs) * trial.scale)
        single_trial_stop(TrialStop::time_cap);
    else if (!lite_motion_ready()) single_trial_stop(TrialStop::lease);
    else {
        const auto refusal = Motion_control_trial_eligibility();
        if (refusal != TrialStop::none) single_trial_stop(refusal);
    }
}
bool single_trial_request(uint8_t ch, bool idle, TrialVerb verb) {
    if (single_trial_fault()) return false;
    single_trial_poll();
    if (single_trial_fault()) return false;
    if (idle) {
        if (trial.active) single_trial_stop(TrialStop::idle_tuple);
        return !single_trial_fault();
    }
    if (ch != 0u) {
#if BMCU_LITE_OWNERSHIP_DETAIL
        const auto detail = ch == 1u ? TrialOwnershipDetail::native_channel_UI2 :
            ch == 2u ? TrialOwnershipDetail::native_channel_UI3 :
            ch == 3u ? TrialOwnershipDetail::native_channel_UI4 : TrialOwnershipDetail::request_channel_out_of_range;
        single_trial_stop(TrialStop::ownership, detail);
#else
        single_trial_stop(TrialStop::ownership);
#endif
        return false;
    }
    if (verb == TrialVerb::none) { single_trial_stop(TrialStop::unsupported); return false; }
    if (!trial.boot_checked) {
#if BMCU_LITE_OWNERSHIP_DETAIL
        single_trial_stop(TrialStop::ownership, TrialOwnershipDetail::boot_check_missing);
#else
        single_trial_stop(TrialStop::ownership);
#endif
        return false;
    }
    if (!lite_motion_ready()) { single_trial_stop(TrialStop::lease); return false; }
    if ((ams_state_get_loaded() != 0xFFu && ams_state_get_loaded() != 0u) ||
        (ams[0].now_filament_num < 4u && ams[0].now_filament_num != 0u) ||
        (verb == TrialVerb::pull && ams_state_get_loaded() != 0u && ams[0].now_filament_num != 0u)) {
#if BMCU_LITE_OWNERSHIP_DETAIL
        // Observe the same existing predicate priority after refusal; no new guard.
        const uint8_t loaded = ams_state_get_loaded();
        const uint8_t selected = ams[0].now_filament_num;
        const auto detail = loaded != 0xFFu && loaded != 0u ? TrialOwnershipDetail::retained_owner_conflict :
            selected < 4u && selected != 0u ? TrialOwnershipDetail::current_selection_conflict : TrialOwnershipDetail::pull_without_owner;
        single_trial_stop(TrialStop::ownership, detail);
#else
        single_trial_stop(TrialStop::ownership);
#endif
        return false;
    }
    const auto refusal = Motion_control_trial_eligibility();
    if (refusal != TrialStop::none) { single_trial_stop(refusal); return false; }
    return true;
}
void single_trial_accept(TrialVerb verb) {
    if (single_trial_fault()) return;
    if (!trial.started) {
        trial.started = true;
        trial.start_ticks = time_ticks64();
        trial.last_ticks = trial.start_ticks;
        trial.scale = time_hw_tpms;
        trial.previous_count = Motion_control_trial_encoder();
        trial.traveled_count = 0;
    }
    trial.last = verb;
    trial.active = true;
    single_trial_poll();
}
void single_trial_dispatch(bool active) { trial.dispatched = active && single_trial_active(); }
int single_trial_filter_pwm(uint8_t ch, int pwm) {
    if (!pwm) return 0;
    single_trial_poll();
    if (single_trial_fault()) return 0;
    if (ch != 0u || !trial.dispatched || !single_trial_active()) {
        single_trial_stop(TrialStop::controller_interlock); return 0;
    }
    const auto refusal = Motion_control_trial_eligibility();
    if (refusal != TrialStop::none) { single_trial_stop(refusal); return 0; }
    if (!Motion_control_trial_output_kind(trial.last)) {
        single_trial_stop(TrialStop::controller_interlock); return 0;
    }
    return pwm > kTrialMaxPWM ? kTrialMaxPWM : pwm < -kTrialMaxPWM ? -kTrialMaxPWM : pwm;
}
uint32_t single_trial_led_rgb() {
#if BMCU_LITE_OWNERSHIP_DETAIL
    // Fixed 40 s frame, 500 ms on/500 ms off. Reserved group windows remain
    // present even for zero counts: white0, green2s, red7s, ownership blue18s.
    const uint32_t scale = time_hw_tpms ? time_hw_tpms : 1u;
    const uint32_t ms = uint32_t((time_ticks64() / scale) % kOwnershipDetailFrameMs);
    if (ms < 600u) return 0x101010u;
    if (ms >= 2000u && ms < 2000u + 1000u * uint8_t(trial.last) && (ms - 2000u) % 1000u < 500u)
        return 0x002000u;
    if (ms >= 7000u && ms < 7000u + 1000u * uint8_t(trial.stop) && (ms - 7000u) % 1000u < 500u)
        return 0x200000u;
    const uint8_t detail = uint8_t(trial.ownership_detail);
    if (trial.stop == TrialStop::ownership && detail <= kOwnershipDetailMax && ms >= 18000u &&
        ms < 18000u + 1000u * detail && (ms - 18000u) % 1000u < 500u)
        return 0x000020u;
    return 0;
#else
    // Fixed 10 s frame: long white marker, green last accepted verb (1..3),
    // red first terminal reason (1..9). Counts and pauses carry the meaning.
    const uint32_t scale = time_hw_tpms ? time_hw_tpms : 1u;
    const uint32_t ms = uint32_t((time_ticks64() / scale) % 10000u);
    if (ms < 600u) return 0x101010u;
    if (ms >= 1000u && ms < 1000u + 400u * uint8_t(trial.last) && (ms - 1000u) % 400u < 200u)
        return 0x002000u;
    if (ms >= 4000u && ms < 4000u + 400u * uint8_t(trial.stop) && (ms - 4000u) % 400u < 200u)
        return 0x200000u;
    return 0;
#endif
}
#endif
