#pragma once
#include <stdint.h>
#if BMCU_LITE_FULL_LOAD
// EXPERIMENTAL / NOT FOR PRINTING. Setup and operation share one image.
enum class FullLoadFault : uint8_t { none, storage, inputs, clock, controller, capability, unsupported
#if BMCU_LITE_NATIVE_FEED_AUTHORITY
, command_authority
#endif
};
FullLoadFault full_load_fault();
bool full_load_ready();
void full_load_fail(FullLoadFault reason);
#if BMCU_LITE_INPUT_FAULT_DETAIL
// Values are persistent diagnostic schema, not fault or admission policy.
enum class InputFaultSource : uint8_t { adc_not_ready=1, adc_stale, module_absent, calibration_invalid,
    comparator_nonfinite, comparator_range, key_nonfinite, key_range, encoder_not_good,
    encoder_offline, encoder_magnet, selected_not_durable, selected_absent,
    boot_encoder_valid, boot_encoder_offline, boot_encoder_magnet, boot_module_absent, pull_absent };
void full_load_input_fail(InputFaultSource source,uint8_t channel=0xffu);
#endif
void full_load_boot(uint8_t retained_owner);
uint8_t full_load_owner();
bool full_load_durable(uint8_t ch);
#if BMCU_LITE_NATIVE_FEED_AUTHORITY
bool full_load_reserve(uint8_t ch, uint8_t command=0, uint64_t received_at=0);
bool full_load_forward_check();
#if !BMCU_LITE_V11_FEED_LATCH
// Preserve the legacy symbol and exact control image; live is a compatible name.
#define full_load_forward_live full_load_forward_fresh
#endif
bool full_load_forward_live();
#else
bool full_load_reserve(uint8_t ch);
#endif
bool full_load_complete(uint8_t ch);
bool full_load_journal_pending();
bool full_load_journal_target(uint8_t ch);
void full_load_cancel();
void full_load_run();
bool full_load_check_inputs();
void full_load_dispatch(bool active);
bool full_load_drive_allowed(uint8_t ch);
#if BMCU_LITE_V11_FEATURES
bool full_load_empty();
bool v11_local_eligible();
bool v11_local_drive_allowed(uint8_t ch);
void v11_local_preempt();
bool v11_metadata_allowed();
bool v11_maintenance_allowed();
#endif
void lite_reserved_send_ready(uint8_t ch);
bool MC_PULL_calibration_load_retained_full();
#endif
