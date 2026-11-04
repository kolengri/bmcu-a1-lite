#pragma once
#include <stdint.h>
#ifndef BMCU_LITE_OWNERSHIP_DETAIL
#define BMCU_LITE_OWNERSHIP_DETAIL 0
#endif
#if BMCU_LITE_OWNERSHIP_DETAIL != 0 && BMCU_LITE_OWNERSHIP_DETAIL != 1
#error "Ownership detail must be 0 or 1"
#endif
#if BMCU_LITE_OWNERSHIP_DETAIL
#if !BMCU_LITE_SINGLE_CHANNEL_TRIAL || !BMCU_EXPERIMENTAL_AMS_LITE || BMCU_LITE_MOTORLOCK || BAMBU_BUS_AMS_NUM != 0
#error "Ownership detail requires the fixed unit0 single-channel experimental Lite trial without motorlock"
#endif
// Stable blue pulse counts. These describe the first reason6 entry, never a
// new eligibility rule or a conclusion that erased/corrupted storage is empty.
enum class TrialOwnershipDetail : uint8_t {
    none = 0,
    boot_flash_fault = 1,
    boot_flash_busy = 2,
    boot_no_valid_record = 3,
    boot_owner_UI2 = 4,
    boot_owner_UI3 = 5,
    boot_owner_UI4 = 6,
    boot_unclassified_refusal = 7,
    native_channel_UI2 = 8,
    native_channel_UI3 = 9,
    native_channel_UI4 = 10,
    boot_check_missing = 11,
    retained_owner_conflict = 12,
    current_selection_conflict = 13,
    pull_without_owner = 14,
    request_channel_out_of_range = 15
};
// Observation written by the actual reader at its existing return paths.
enum class TrialOwnershipRead : uint8_t { unobserved, valid, flash_fault, flash_busy, no_valid_record };
TrialOwnershipRead Flash_trial_ownership_read_observation();
TrialOwnershipDetail single_trial_ownership_detail();
constexpr uint32_t kOwnershipDetailFrameMs = 40000u;
constexpr uint8_t kOwnershipDetailMax = 15u;
#endif
