#pragma once
#include <stdint.h>
#ifndef BMCU_LITE_SETUP_TRIAL
#define BMCU_LITE_SETUP_TRIAL 0
#endif
#if BMCU_LITE_SETUP_TRIAL != 0 && BMCU_LITE_SETUP_TRIAL != 1
#error "Setup/trial flag must be 0 or 1"
#endif
#if BMCU_LITE_SETUP_TRIAL
#if !BMCU_EXPERIMENTAL_AMS_LITE || (!BMCU_LITE_SINGLE_CHANNEL_TRIAL && !BMCU_LITE_FULL_LOAD) || (BMCU_LITE_SINGLE_CHANNEL_TRIAL && BMCU_LITE_FULL_LOAD) || BMCU_LITE_MOTORLOCK || !BMCU_DM_TWO_MICROSWITCH || BAMBU_BUS_AMS_NUM != 0
#error "Setup/trial requires fixed unit0 native Lite, single-channel DM trial without motorlock"
#endif
// NOT FOR PRINTING. Mode is selected once, before the bus can start.
enum class SetupPhase : uint8_t { inspect, gesture, calibration, direction, journal, success, fault, trial
#if BMCU_LITE_FULL_LOAD
    , operation
#endif
};
enum class SetupFault : uint8_t { none, storage, loaded_owner, incomplete, module, filament,
    adc, encoder, clock, gesture_timeout, calibration, direction, write, verification, capability };
enum class SetupWrite : uint8_t { cal, motion, journal };
struct SetupStorage {
    bool fault, busy, journal_erased, journal_valid, journal_invalid;
    uint8_t owner;
    bool cal_erased, cal_schema, cal_invalid, motion_erased, motion_schema;
};
SetupPhase setup_trial_phase();
SetupFault setup_trial_fault();
bool setup_trial_bus_allowed();
bool setup_trial_selected();
void setup_trial_fail(SetupFault reason);
void setup_trial_admission_fail(SetupFault reason,uint8_t detail); // Frozen early-boot detail1..6 only.
bool setup_trial_poll(bool centered = false
#if BMCU_SETUP_INSERTION_RECOVERY
    , bool committing = false
#endif
);
#if BMCU_SETUP_INSERTION_RECOVERY
bool setup_trial_retry_pending();
#endif
bool setup_trial_write_allowed(SetupWrite kind);
bool setup_trial_flash_address(uint32_t address);
bool setup_trial_drive_allowed(uint8_t ch, int pwm);
void setup_trial_boot(); // Returns only for irrevocably selected trial.
#if BMCU_LITE_V11_FEATURES
bool setup_trial_maintenance_clearing();
void setup_trial_empty_maintenance(); // Intentional RAM-only transition; interrupted storage still parks.
#endif
uint32_t setup_trial_led_rgb(uint64_t ticks);
SetupStorage Flash_setup_probe(); // Uses production readers and scan/cache flags.
bool Flash_setup_fault();
bool Motion_control_setup_init(bool empty = true);
bool Motion_control_setup_inputs(bool centered, bool calibrated, bool empty = true
#if BMCU_SETUP_INSERTION_RECOVERY
    , const float* snapshot = nullptr
#endif
);
bool Motion_control_setup_retained();
bool Motion_control_setup_direction();
void Motion_control_setup_stop();
#endif
