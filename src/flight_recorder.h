#pragma once
#include "lite_profile.h"
#if BMCU_USB_FLIGHT_RECORDER
namespace flight {
#if BMCU_LITE_AUTOMATIC_CAPTURE
#if BMCU_USB_CONTINUOUS_JOURNAL
#if BMCU_USB_ISP_EXPORT
constexpr uint32_t build=0x260b0403u;
#elif BMCU_USB_WORKING_MODE
constexpr uint32_t build=0x260b0402u;
#else
constexpr uint32_t build=0x260b0401u;
#endif
void journal_step();
void journal_fault(uint8_t fault);
#elif BMCU_LITE_ENCODER_RECOVERY
constexpr uint32_t build=0x260a3103u;
#elif BMCU_LITE_PRECUT_PHASE_CAPTURE
constexpr uint32_t build=0x260a3102u;
#elif BMCU_LITE_RX_BACKPRESSURE
constexpr uint32_t build=0x260a3101u;
#else
constexpr uint32_t build=0x260a2902u;
#endif
void automatic_capture();
#elif BMCU_LITE_LEASE_FEEDBACK
constexpr uint32_t build=0x260a2901u;
#elif BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
constexpr uint32_t build=0x260a2806u;
#elif BMCU_LITE_ONUSE_IDLE_COMPAT
constexpr uint32_t build=0x260a2805u;
#elif BMCU_LITE_CONTINUOUS_NATIVE
constexpr uint32_t build=0x260a2804u;
#elif BMCU_LITE_PHASE99_ALL_SLOTS
constexpr uint32_t build=0x260a2803u;
#elif BMCU_LITE_PHASE99_03_COMPAT
constexpr uint32_t build=0x260a2802u;
#elif BMCU_LITE_PHASE99_COMPAT
constexpr uint32_t build=0x260a2801u;
#elif BMCU_LITE_SETUP_RELEASE_CUE || BMCU_LITE_NATIVE_UNLOAD_CONTINUE
constexpr uint32_t build=0x260a2707u;
#elif BMCU_LITE_GLOBAL_PHASE_COMPAT
constexpr uint32_t build=0x260a2706u;
#elif BMCU_LITE_METER_ORIGIN_ZERO
constexpr uint32_t build=0x260a2705u;
#elif BMCU_LITE_INPUT_FAULT_DETAIL
constexpr uint32_t build=0x260a2704u;
#elif BMCU_LITE_NATIVE_PHASE_COMPAT
constexpr uint32_t build=0x260a2703u;
#elif BMCU_LITE_V11_FEED_LATCH
constexpr uint32_t build=0x260a2702u;
#else
constexpr uint32_t build=0x260a2701u;
#endif
constexpr uint32_t base=0x0800ee00u;
constexpr uint16_t version=1;
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
// Fixed version1 diagnostic area; no pointers, native padding or new wire offsets.
struct DiagnosticV1 {
    uint8_t version,seen_command,seen_channel,first_refusal;
    uint8_t refusal_command,refusal_channel,refusal_context,admitted_channel;
    uint8_t first_tx,response_command,response_length,pwm_channel;
    uint16_t first_pwm[2];
    uint8_t before_selected,before_owner,before_motion,before_pull;
    uint8_t before_fault,before_flags,gesture_channel,snapshot_kind;
    uint16_t before_pwm[8];
    uint8_t admitted_command,seen_flags,pwm_observed_mask,reserved;
    uint32_t seen_count;
#if BMCU_LITE_LEASE_FEEDBACK
    float onuse_meters[3]; // Initial, latest, max absolute change; cached real odometry only.
#else
    uint32_t reserved_words[3];
#endif
};
static_assert(sizeof(DiagnosticV1)==60,"diagnostic area changed");
#if BMCU_LITE_LEASE_FEEDBACK
static_assert(__builtin_offsetof(DiagnosticV1,onuse_meters)==48,"on-use evidence offset changed");
void onuse_meter(uint8_t ch,bool initial);
#endif
#endif
// Fixed LE layout, raw IEEE754 comparator floats (no extra sensor reads).
struct Report {
    uint32_t flags, window_ms, samples, gen_first, gen_last, max_gap_ms, max_age_ms;
    float baseline[4], minimum[4], maximum[4], peak_delta[4];
    uint32_t hold_ms, release_ms, confirmed_ms, core_hz;
    uint8_t candidate, progress, phase, reached, fault, detail, native_fault, event_count;
    struct Event { uint16_t ms; uint8_t kind, channel; } events[12];
    uint8_t ao4[80];
    uint32_t ticks_per_ms, rcc_cfgr0, exten_ctlr, systick_ctlr;
    float event_delta[12];
#if BMCU_LITE_V11_FEED_LATCH
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
    union { uint8_t native_bf[60]; DiagnosticV1 diagnostic; };
#else
    uint8_t native_bf[60]; // Full last BF-associated reply at actual DMA start.
#endif
    uint8_t native_idle[8]; // use flag, channel, wire meters, pressure; not a full frame.
    uint8_t idle_command,idle_length; // Exact FF/01/00 request; command 03 or 04.
    uint16_t tx_completed,tx_aborted;
    uint8_t bf_length,native_tx[2]; // flags: started=1, completed=2, aborted=4
    uint8_t native_reserved;
#if BMCU_LITE_INPUT_FAULT_DETAIL
    uint16_t input_detail; // Existing tail padding: source low byte, channel high byte.
#endif
#if BMCU_LITE_RX_BACKPRESSURE
    uint32_t rx_dropped,rx_queue_pauses,rx_header_crc_errors;
#if BMCU_LITE_PRECUT_PHASE_CAPTURE
    // First actual DMA response per state07 motion00/26/66/99 x global/addressed.
    // Request: channel0..3/F, sender04 bit4, first-submit ordinal bits5..7.
    // Feedback: exact use nibble low, wire selected channel0..3/F high.
    struct PhaseResponse { uint16_t elapsed_10ms; uint8_t request,feedback; uint8_t meters[4]; } precut[8];
    uint32_t precut_consumed,precut_submitted; // class*2 + sender04; consumed != constructed != DMA start.
#endif
#endif
#endif
};
#if BMCU_LITE_V11_FEED_LATCH
static_assert(sizeof(Report)==(BMCU_LITE_PRECUT_PHASE_CAPTURE?472:(BMCU_LITE_RX_BACKPRESSURE?400:388)), "wire layout changed");
#if BMCU_LITE_INPUT_FAULT_DETAIL
static_assert(__builtin_offsetof(Report,native_reserved)==385 && __builtin_offsetof(Report,input_detail)==386, "input detail layout changed");
#endif
#if BMCU_LITE_INPUT_FAULT_DETAIL
void input_fault(uint16_t detail);
#endif
void response(uint8_t ch,uint8_t state,uint8_t motion);
void tx_start(const uint8_t* data,uint16_t length);
void tx_finish(bool aborted);
#endif
#if BMCU_LITE_PRECUT_PHASE_CAPTURE
static_assert(__builtin_offsetof(Report,precut)==400 && __builtin_offsetof(Report,precut_consumed)==464 && __builtin_offsetof(Report,precut_submitted)==468,"precut tail layout changed");
#endif
static_assert(sizeof(Report)==(BMCU_LITE_V11_FEED_LATCH?(BMCU_LITE_PRECUT_PHASE_CAPTURE?472:(BMCU_LITE_RX_BACKPRESSURE?400:388)):308), "wire layout changed");
// true means run the one armed test; false means inhibited export forever.
bool boot();
void window(const float* center,uint64_t start,uint32_t gen,uint32_t scale);
void sample(uint64_t now,uint32_t gen,uint64_t age);
void pull(uint8_t ch,float value,float delta);
void progress(uint8_t candidate,uint64_t held,bool confirmed,uint64_t released,uint64_t now);
void finish_setup(uint8_t phase,uint8_t reached,uint8_t fault,uint8_t detail);
#if BMCU_LITE_CONTINUOUS_NATIVE
#if BMCU_LITE_ONUSE_IDLE_COMPAT
void native_started(uint32_t flags=128u); // Logical admission flags only, frozen after first snapshot.
#endif
void setup_completed(); // RAM-only; preserve the native durable snapshot slot.
#endif
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
enum class PullRefusal:uint8_t { offline=1,not_ready,metadata,journal,loaded_mismatch,selection_mismatch,pull_state,absent,nondurable,no_retained_selection };
void pull_wire(uint8_t command,uint8_t ch,uint8_t state,uint8_t motion);
void pull_refused(PullRefusal reason,uint8_t context=0);
void pull_admitted(uint8_t command,uint8_t ch);
void pull_pwm(uint8_t ch,uint16_t first,uint16_t second);
bool operator_eligible();
void operator_request(uint8_t gesture_channel);
bool operator_pending();
bool saved();
#endif
bool finish_native(uint8_t fault);
void native_cancelled();
#if !BMCU_LITE_ONUSE_IDLE_COMPAT
void native_started();
#endif
void native_interrupted();
bool native_checkpoint_pending();
void native_completed();
bool native_end_pending();
void uart_init();
void export_step();
}
#endif

