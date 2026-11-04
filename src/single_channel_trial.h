#pragma once
#include <stdint.h>
#ifndef BMCU_LITE_SINGLE_CHANNEL_TRIAL
#define BMCU_LITE_SINGLE_CHANNEL_TRIAL 0
#endif
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL != 0 && BMCU_LITE_SINGLE_CHANNEL_TRIAL != 1
#error "Single-channel trial must be 0 or 1"
#endif
#include "ownership_detail.h"
#include "setup_trial.h"
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL
#if !BMCU_EXPERIMENTAL_AMS_LITE || BMCU_LITE_MOTORLOCK || BAMBU_BUS_AMS_NUM != 0
#error "Single-channel trial requires native experimental Lite, fixed unit 0, without motorlock"
#endif
enum class TrialVerb : uint8_t { none, feed, pull, use };
enum class TrialStop : uint8_t {
    none, idle_tuple, unsupported, lease, retained_calibration,
    physical_sensor, ownership, travel_cap, time_cap, controller_interlock
};
constexpr uint32_t kTrialMaxMs = 2000u;
constexpr int kTrialMaxPWM = 600;
void single_trial_boot_ownership(bool readable, uint8_t loaded);
void single_trial_stop(TrialStop reason
#if BMCU_LITE_OWNERSHIP_DETAIL
    , TrialOwnershipDetail detail = TrialOwnershipDetail::none
#endif
);
bool single_trial_fault();
bool single_trial_active();
TrialStop single_trial_reason();
TrialVerb single_trial_last_verb();
bool single_trial_request(uint8_t ch, bool idle, TrialVerb verb);
void single_trial_accept(TrialVerb verb);
void single_trial_poll();
void single_trial_dispatch(bool active);
int single_trial_filter_pwm(uint8_t ch, int pwm);
uint32_t single_trial_led_rgb();
uint32_t Motion_control_trial_encoder();
uint32_t Motion_control_trial_travel_cap();
TrialStop Motion_control_trial_eligibility();
bool Motion_control_trial_output_kind(TrialVerb verb);
bool MC_PULL_calibration_load_retained_trial();
#endif
