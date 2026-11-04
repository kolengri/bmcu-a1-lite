#include "lite_profile.h"
#include "flight_recorder.h"
#include "authority_observation.h"
#if BMCU_LITE_FULL_LOAD
#include "full_load.h"
#include "Flash_saves.h"
#include "ADC_DMA.h"
#include "ams.h"
#include "hal/time_hw.h"
static FullLoadFault operation_fault=FullLoadFault::none;
enum class JournalIntent : uint8_t { none, reserve, release };
static JournalIntent intent=JournalIntent::none;
static uint8_t owner=0xffu,durable_owner=0xffu;
static bool initialized=false,waiting_send=false,dispatching=false;
static uint32_t operation_scale=0,publication=0;
static uint64_t previous_ticks=0,publication_ticks=0;
#if BMCU_LITE_NATIVE_FEED_AUTHORITY
static bool forward_grant=false;
static uint64_t forward_received=0;
// Legacy engineering receipt-age bound is retained only in the default control.
// In V11 mode an admitted intent lives until an explicit state/interlock revokes it.
bool full_load_forward_check() {
    if (!forward_grant) return full_load_ready();
    if (Flash_setup_fault()) full_load_fail(FullLoadFault::storage);
    if (!full_load_check_inputs()) return false;
#if !BMCU_LITE_V11_FEED_LATCH
    if (time_ticks64()-forward_received >= uint64_t(operation_scale)*100u)
        full_load_fail(FullLoadFault::command_authority);
#endif
    return full_load_ready();
}
bool full_load_forward_live() {
    return forward_grant && full_load_forward_check();
}
#endif
FullLoadFault full_load_fault() { return operation_fault; }
bool full_load_ready() { return initialized && operation_fault==FullLoadFault::none
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
    && !flight::operator_pending()
#endif
    ; }
uint8_t full_load_owner() { return owner; }
bool full_load_journal_pending() { return intent!=JournalIntent::none; }
bool full_load_durable(uint8_t ch) { return ch<4u && durable_owner==ch && owner==ch && intent==JournalIntent::none; }
#if BMCU_LITE_V11_FEATURES
bool full_load_empty() { return full_load_ready() && owner==0xffu && durable_owner==0xffu && intent==JournalIntent::none; }
#endif
void full_load_cancel() { waiting_send=false; dispatching=false;
#if BMCU_LITE_NATIVE_FEED_AUTHORITY
    forward_grant=false;
#endif
}
void full_load_fail(FullLoadFault reason) {
#if BMCU_USB_CONTINUOUS_JOURNAL
    if(operation_fault==FullLoadFault::none)flight::journal_fault(uint8_t(reason));
#endif
#if BMCU_LITE_AUTHORITY_OBSERVATION
    if (operation_fault==FullLoadFault::none) authority_observation::stop(reason==FullLoadFault::command_authority?authority_observation::authority:authority_observation::fault,uint8_t(reason));
#endif

    if (operation_fault==FullLoadFault::none) operation_fault=reason;
    full_load_cancel();
    Motion_control_lite_cancel();
}
#if BMCU_LITE_INPUT_FAULT_DETAIL
void full_load_input_fail(InputFaultSource source,uint8_t channel) {
    if (operation_fault==FullLoadFault::none) flight::input_fault(uint16_t(source)|(uint16_t(channel)<<8));
    full_load_fail(FullLoadFault::inputs);
}
#endif
void full_load_boot(uint8_t retained_owner) {
    // No normalization or on-use synthesis. The caller used the strict production reader.
    if (initialized) return;
    owner=durable_owner=retained_owner;
    initialized=true;
    operation_scale=time_hw_tpms;
    previous_ticks=publication_ticks=time_ticks64();
    publication=ADC_DMA_generation();
    if ((!operation_scale) || (retained_owner>=4u && retained_owner!=0xffu))
        full_load_fail(FullLoadFault::storage);
}
bool full_load_reserve(uint8_t ch
#if BMCU_LITE_NATIVE_FEED_AUTHORITY
, uint8_t command, uint64_t received_at
#endif
) {
#if BMCU_LITE_NATIVE_FEED_AUTHORITY
    if (!full_load_forward_check()) return false;
#endif
    if (!full_load_ready() || !lite_motion_ready() || ch>=4u) return false;
#if BMCU_LITE_NATIVE_FEED_AUTHORITY
    if (command && (full_load_durable(ch) || (intent==JournalIntent::reserve && owner==ch) ||
                     (intent==JournalIntent::none && owner==0xFFu))) {

#if BMCU_LITE_AUTHORITY_OBSERVATION
        authority_observation::grant(ch,received_at,!waiting_send&&(ams[0].now_filament_num!=ch||ams[0].filament[ch].motion!=_filament_motion::send_out));
#endif
        forward_received=received_at;
        forward_grant=true;
#if BMCU_USB_FLIGHT_RECORDER
        flight::native_started(); // Includes a valid grant waiting on durable reservation.
#endif
    }
#endif
    if (full_load_durable(ch)) return true;
    if (intent==JournalIntent::reserve && owner==ch) { waiting_send=true; return false; }
    if (intent!=JournalIntent::none || owner!=0xffu) return false;
    owner=ch; // Conservative RAM lock while the persistent reservation is pending.
    intent=JournalIntent::reserve;
    waiting_send=true;
    return false;
}
bool full_load_complete(uint8_t ch) {
    if (!full_load_ready() || !full_load_durable(ch)) return false;
    // Called only by the active native pull's actual V11 redetect completion callback.
    intent=JournalIntent::release;
    waiting_send=false;
    return true;
}
bool full_load_journal_target(uint8_t ch) {
    return (intent==JournalIntent::reserve && ch==owner && owner<4u) ||
        (intent==JournalIntent::release && ch==0xffu && durable_owner==owner && owner<4u);
}
void full_load_run() {
    if (!full_load_ready()) return;
    if (Flash_setup_fault()) { full_load_fail(FullLoadFault::storage); return; }
#if BMCU_LITE_NATIVE_FEED_AUTHORITY
    if (!full_load_forward_check()) return;
#endif
    if (intent==JournalIntent::none || !lite_motion_ready()) return;
    const uint8_t target=intent==JournalIntent::reserve?owner:0xffu;
    // Submission returns false while queued/busy. The existing job verifies every
    // halfword before it updates the production reader's saved-record cache.
    if (!Flash_AMS_state_write(target)) return;
    uint8_t verified=0xffu;
    if (!Flash_AMS_state_read(&verified) || verified!=target) {
        full_load_fail(FullLoadFault::storage); return;
    }
    durable_owner=target;
    const bool resume=waiting_send && intent==JournalIntent::reserve;
    intent=JournalIntent::none;
    waiting_send=false;
    if (target==0xffu) {
        owner=0xffu;
        ams[0].now_filament_num=0xffu;
        ams[0].filament_use_flag=0u;
#if BMCU_USB_FLIGHT_RECORDER
        flight::native_completed(); // Only after the production reader verified durable FF.
#endif
    } else if (resume && lite_motion_ready()
#if BMCU_LITE_NATIVE_FEED_AUTHORITY
               && full_load_forward_live()
#endif
    ) {
        lite_reserved_send_ready(target);
    }
}
bool full_load_check_inputs() {
    if (!full_load_ready()) return false;
    const uint64_t now=time_ticks64();
    if (!operation_scale || time_hw_tpms!=operation_scale || now<previous_ticks) {
        full_load_fail(FullLoadFault::clock); return false;
    }
    previous_ticks=now;
    const uint32_t got=ADC_DMA_generation();
    if (got!=publication) { publication=got; publication_ticks=now; }
    // Reuse the accepted ADC publication freshness condition, without a session timeout.
#if BMCU_LITE_INPUT_FAULT_DETAIL
    const auto source=!ADC_DMA_ready()?InputFaultSource::adc_not_ready:
        now-publication_ticks>uint64_t(operation_scale)*100u?InputFaultSource::adc_stale:InputFaultSource(0);
    if (uint8_t(source)) { full_load_input_fail(source); return false; }
#else
    if (!ADC_DMA_ready() || now-publication_ticks>uint64_t(operation_scale)*100u) {
        full_load_fail(FullLoadFault::inputs); return false;
    }
#endif
    return true;
}
void full_load_dispatch(bool active) { dispatching=active; }
bool full_load_drive_allowed(uint8_t ch) {
#if BMCU_LITE_V11_FEATURES
    if (v11_local_drive_allowed(ch)) return true;
#endif
#if BMCU_LITE_NATIVE_FEED_AUTHORITY
    if (!full_load_forward_check()) return false;
#endif
    if (full_load_ready() && dispatching && full_load_durable(ch) && lite_motion_ready() &&
        ams[0].now_filament_num==ch) return true;
    full_load_fail(FullLoadFault::capability);
    return false;
}
#endif
