#pragma once

// Independent opt-in; ordinary and historical diagnostic profiles retain their behavior.
#ifndef BMCU_USB_CONTINUOUS_JOURNAL
#define BMCU_USB_CONTINUOUS_JOURNAL 0
#endif
#if BMCU_USB_CONTINUOUS_JOURNAL != 0 && BMCU_USB_CONTINUOUS_JOURNAL != 1
#error "BMCU_USB_CONTINUOUS_JOURNAL must be 0 or 1"
#endif
#if BMCU_USB_CONTINUOUS_JOURNAL && !BMCU_LITE_ENCODER_RECOVERY
#error "Continuous journal requires the installed encoder-recovery control baseline"
#endif
#include <stdint.h>
#include "ams_addressing.h"

// EXPERIMENTAL. Ordinary V11 is the default; this is not certified firmware.
#ifndef BMCU_EXPERIMENTAL_AMS_LITE
#define BMCU_EXPERIMENTAL_AMS_LITE 0
#endif
#ifndef BMCU_LITE_MOTORLOCK
#define BMCU_LITE_MOTORLOCK 0
#endif
#if (BMCU_EXPERIMENTAL_AMS_LITE != 0 && BMCU_EXPERIMENTAL_AMS_LITE != 1) || (BMCU_LITE_MOTORLOCK != 0 && BMCU_LITE_MOTORLOCK != 1)
#error "Lite options must be 0 or 1"
#endif
#if BMCU_LITE_MOTORLOCK && !BMCU_EXPERIMENTAL_AMS_LITE
#error "Motorlock requires the experimental Lite profile"
#endif
#if BMCU_EXPERIMENTAL_AMS_LITE && BAMBU_BUS_AMS_NUM != 0
#error "Experimental Lite requires fixed unit 0; AUTO/chains are unsupported"
#endif
#if BMCU_EXPERIMENTAL_AMS_LITE
constexpr uint16_t kLiteAddress = 0x1200u;
enum LiteObservation : uint8_t {
    LITE_DISCOVERY = 1u, LITE_VERSION = 2u, LITE_SERIAL = 4u,
    LITE_STATUS = 8u, LITE_CERT = 16u, LITE_AUTH = 32u,
    LITE_OTHER = 64u, LITE_WRONG_ADDRESS = 128u
};
uint8_t lite_observations();
uint32_t lite_observation_rgb();
bool lite_motion_ready();
bool lite_recovery_fault();
void lite_unload_interrupted();
void lite_unload_complete(uint8_t ch);
#if !BMCU_LITE_MOTORLOCK
void Motion_control_lite_cancel();
#endif
void MC_PWM_init();
#endif

#ifndef BMCU_LITE_FULL_LOAD
#define BMCU_LITE_FULL_LOAD 0
#endif
#if BMCU_LITE_FULL_LOAD != 0 && BMCU_LITE_FULL_LOAD != 1
#error "Full-load profile must be 0 or 1"
#endif
#include "single_channel_trial.h"
#if BMCU_LITE_FULL_LOAD
#if !BMCU_EXPERIMENTAL_AMS_LITE || !BMCU_LITE_SETUP_TRIAL || BMCU_LITE_SINGLE_CHANNEL_TRIAL || BMCU_LITE_MOTORLOCK || BMCU_LITE_OWNERSHIP_DETAIL || !BMCU_DM_TWO_MICROSWITCH || BAMBU_BUS_AMS_NUM != 0
#error "Full-load requires fixed unit0 native Lite DM setup, without trial/motorlock/detail"
#endif
#endif
#ifndef BMCU_LITE_V11_FEATURES
#define BMCU_LITE_V11_FEATURES 0
#endif
#if BMCU_LITE_V11_FEATURES != 0 && BMCU_LITE_V11_FEATURES != 1
#error "V11 features flag must be 0 or 1"
#endif
#if BMCU_LITE_V11_FEATURES && (!BMCU_LITE_FULL_LOAD || BMCU_SOFT_LOAD || BMCU_P1S)
#error "V11 features require the full-load standard DMPro A1 profile"
#endif
#ifndef BMCU_LITE_NATIVE_FEED_AUTHORITY
#define BMCU_LITE_NATIVE_FEED_AUTHORITY 0
#endif
#if BMCU_LITE_NATIVE_FEED_AUTHORITY != 0 && BMCU_LITE_NATIVE_FEED_AUTHORITY != 1
#error "Native feed authority must be 0 or 1"
#endif
#if BMCU_LITE_NATIVE_FEED_AUTHORITY && !BMCU_LITE_V11_FEATURES
#error "Native feed authority requires restored V11 features"
#endif
// Restore V11 state-driven native forward intent. The legacy receipt-age policy
// remains the default control; matching idle/Abort aliases are unqualified here.
#ifndef BMCU_LITE_V11_FEED_LATCH
#define BMCU_LITE_V11_FEED_LATCH 0
#endif
#if BMCU_LITE_V11_FEED_LATCH != 0 && BMCU_LITE_V11_FEED_LATCH != 1
#error "V11 feed latch must be 0 or 1"
#endif
#if BMCU_LITE_V11_FEED_LATCH && (!BMCU_LITE_NATIVE_FEED_AUTHORITY || !BMCU_LITE_V11_FEATURES || !BMCU_LITE_FULL_LOAD)
#error "V11 feed latch requires native feed authority and the restored full-load V11 profile"
#endif
#ifndef BMCU_LITE_AUTHORITY_OBSERVATION
#define BMCU_LITE_AUTHORITY_OBSERVATION 0
#endif
#if BMCU_LITE_AUTHORITY_OBSERVATION != 0 && BMCU_LITE_AUTHORITY_OBSERVATION != 1
#error "Authority observation must be 0 or 1"
#endif
#if BMCU_LITE_AUTHORITY_OBSERVATION && !BMCU_LITE_NATIVE_FEED_AUTHORITY
#error "Authority observation requires native feed authority"
#endif
#ifndef BMCU_LITE_NATIVE_IDLE_04
#define BMCU_LITE_NATIVE_IDLE_04 0
#endif
#if BMCU_LITE_NATIVE_IDLE_04 != 0 && BMCU_LITE_NATIVE_IDLE_04 != 1
#error "Native idle 04 must be 0 or 1"
#endif
#if BMCU_LITE_NATIVE_IDLE_04 && !BMCU_LITE_NATIVE_FEED_AUTHORITY
#error "Native idle 04 requires native feed authority"
#endif
#include "full_load.h"

#ifndef BMCU_USB_FLIGHT_RECORDER
#define BMCU_USB_FLIGHT_RECORDER 0
#endif
#if BMCU_USB_FLIGHT_RECORDER != 0 && BMCU_USB_FLIGHT_RECORDER != 1
#error "Flight recorder must be 0 or 1"
#endif
#if BMCU_USB_FLIGHT_RECORDER && (!BMCU_LITE_NATIVE_IDLE_04 || !BMCU_LITE_AUTHORITY_OBSERVATION)
#error "Flight recorder requires the accepted idle04 observation profile"
#endif

#ifndef BMCU_LITE_SETUP_GESTURE_1S
#define BMCU_LITE_SETUP_GESTURE_1S 0
#endif
#if BMCU_LITE_SETUP_GESTURE_1S != 0 && BMCU_LITE_SETUP_GESTURE_1S != 1
#error "One-second setup gesture must be 0 or 1"
#endif
#if BMCU_LITE_SETUP_GESTURE_1S && !BMCU_USB_FLIGHT_RECORDER
#error "One-second setup gesture requires the logged test profile"
#endif

// Historical Lite response phases; experimental and deliberately default-OFF.
#ifndef BMCU_LITE_NATIVE_PHASE_COMPAT
#define BMCU_LITE_NATIVE_PHASE_COMPAT 0
#endif
#if BMCU_LITE_NATIVE_PHASE_COMPAT != 0 && BMCU_LITE_NATIVE_PHASE_COMPAT != 1
#error "Native phase compatibility must be 0 or 1"
#endif
#if BMCU_LITE_NATIVE_PHASE_COMPAT && (!BMCU_USB_FLIGHT_RECORDER || !BMCU_LITE_V11_FEED_LATCH || !BMCU_LITE_NATIVE_FEED_AUTHORITY || !BMCU_LITE_V11_FEATURES || !BMCU_LITE_FULL_LOAD)
#error "Native phase compatibility requires the logged restored V11 native feed-latch profile"
#endif

// First input-fault provenance only; no additional sensor observations.
#ifndef BMCU_LITE_INPUT_FAULT_DETAIL
#define BMCU_LITE_INPUT_FAULT_DETAIL 0
#endif
#if BMCU_LITE_INPUT_FAULT_DETAIL != 0 && BMCU_LITE_INPUT_FAULT_DETAIL != 1
#error "Input fault detail must be 0 or 1"
#endif
#if BMCU_LITE_INPUT_FAULT_DETAIL && !BMCU_LITE_NATIVE_PHASE_COMPAT
#error "Input fault detail requires the logged restored V11 phase-compatible profile"
#endif

// Experimental Lite wire-meter origin only; controller odometry is unchanged.
#ifndef BMCU_LITE_METER_ORIGIN_ZERO
#define BMCU_LITE_METER_ORIGIN_ZERO 0
#endif
#if BMCU_LITE_METER_ORIGIN_ZERO != 0 && BMCU_LITE_METER_ORIGIN_ZERO != 1
#error "Lite meter origin must be 0 or 1"
#endif
#if BMCU_LITE_METER_ORIGIN_ZERO && !BMCU_LITE_INPUT_FAULT_DETAIL
#error "Lite meter origin requires the logged phase-compatible input-detail profile"
#endif

// Global Lite status phases never select a channel or admit motion.
#ifndef BMCU_LITE_GLOBAL_PHASE_COMPAT
#define BMCU_LITE_GLOBAL_PHASE_COMPAT 0
#endif
#if BMCU_LITE_GLOBAL_PHASE_COMPAT != 0 && BMCU_LITE_GLOBAL_PHASE_COMPAT != 1
#error "Global phase compatibility must be 0 or 1"
#endif
#if BMCU_LITE_GLOBAL_PHASE_COMPAT && !BMCU_LITE_METER_ORIGIN_ZERO
#error "Global phase compatibility requires the logged phase-compatible zero-origin profile"
#endif

// Visual release cue only; accepted-hold and stable-release decisions are unchanged.
#ifndef BMCU_LITE_SETUP_RELEASE_CUE
#define BMCU_LITE_SETUP_RELEASE_CUE 0
#endif
#if BMCU_LITE_SETUP_RELEASE_CUE != 0 && BMCU_LITE_SETUP_RELEASE_CUE != 1
#error "Setup release cue must be 0 or 1"
#endif
#if BMCU_LITE_SETUP_RELEASE_CUE && !BMCU_LITE_GLOBAL_PHASE_COMPAT
#error "Setup release cue requires the accepted logged global-phase profile"
#endif

// Continue bus service after the existing stopped successful-Unload checkpoint.
#ifndef BMCU_LITE_NATIVE_UNLOAD_CONTINUE
#define BMCU_LITE_NATIVE_UNLOAD_CONTINUE 0
#endif
#if BMCU_LITE_NATIVE_UNLOAD_CONTINUE != 0 && BMCU_LITE_NATIVE_UNLOAD_CONTINUE != 1
#error "Native unload continuation must be 0 or 1"
#endif
#if BMCU_LITE_NATIVE_UNLOAD_CONTINUE && !BMCU_LITE_GLOBAL_PHASE_COMPAT
#error "Native unload continuation requires the accepted logged global-phase profile"
#endif

// Reply to the exact observed phase99 status request without motion authority.
#ifndef BMCU_LITE_PHASE99_COMPAT
#define BMCU_LITE_PHASE99_COMPAT 0
#endif
#if BMCU_LITE_PHASE99_COMPAT != 0 && BMCU_LITE_PHASE99_COMPAT != 1
#error "Phase99 compatibility must be 0 or 1"
#endif
#if BMCU_LITE_PHASE99_COMPAT && (!BMCU_LITE_SETUP_RELEASE_CUE || !BMCU_LITE_NATIVE_UNLOAD_CONTINUE)
#error "Phase99 compatibility requires the accepted logged setup-release and unload-continuation profile"
#endif

// Reply to the exact observed03 phase99 counterpart without motion authority.
#ifndef BMCU_LITE_PHASE99_03_COMPAT
#define BMCU_LITE_PHASE99_03_COMPAT 0
#endif
#if BMCU_LITE_PHASE99_03_COMPAT != 0 && BMCU_LITE_PHASE99_03_COMPAT != 1
#error "Phase99 command03 compatibility must be 0 or 1"
#endif
#if BMCU_LITE_PHASE99_03_COMPAT && !BMCU_LITE_PHASE99_COMPAT
#error "Phase99 command03 compatibility requires the accepted command04 phase99 profile"
#endif

// Recognize phase99 polling for every valid Lite slot without motion authority.
#ifndef BMCU_LITE_PHASE99_ALL_SLOTS
#define BMCU_LITE_PHASE99_ALL_SLOTS 0
#endif
#if BMCU_LITE_PHASE99_ALL_SLOTS != 0 && BMCU_LITE_PHASE99_ALL_SLOTS != 1
#error "Phase99 all-slots compatibility must be 0 or 1"
#endif
#if BMCU_LITE_PHASE99_ALL_SLOTS && !BMCU_LITE_PHASE99_03_COMPAT
#error "Phase99 all-slots compatibility requires the accepted command03 phase99 profile"
#endif

// One-shot flash-to-calibration-to-native workflow and observed global idle poll.
#ifndef BMCU_LITE_CONTINUOUS_NATIVE
#define BMCU_LITE_CONTINUOUS_NATIVE 0
#endif
#if BMCU_LITE_CONTINUOUS_NATIVE != 0 && BMCU_LITE_CONTINUOUS_NATIVE != 1
#error "Continuous native flag must be 0 or 1"
#endif
#if BMCU_LITE_CONTINUOUS_NATIVE && (!BMCU_LITE_PHASE99_ALL_SLOTS || !BMCU_LITE_SETUP_RELEASE_CUE || !BMCU_LITE_NATIVE_UNLOAD_CONTINUE)
#error "Continuous native requires the accepted all-slots, setup-cue and unload-continuation profile"
#endif


#ifndef BMCU_LITE_ONUSE_IDLE_COMPAT
#define BMCU_LITE_ONUSE_IDLE_COMPAT 0
#endif
#if BMCU_LITE_ONUSE_IDLE_COMPAT != 0 && BMCU_LITE_ONUSE_IDLE_COMPAT != 1
#error "On-use idle compatibility flag must be 0 or 1"
#endif
#if BMCU_LITE_ONUSE_IDLE_COMPAT && !BMCU_LITE_CONTINUOUS_NATIVE
#error "On-use idle compatibility requires the accepted continuous-native profile"
#endif

// Child diagnostic build; OFF preserves the accepted05 image and protocol.
#ifndef BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
#define BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE 0
#endif
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE != 0 && BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE != 1
#error "BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE must be 0 or 1"
#endif
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE && !BMCU_LITE_ONUSE_IDLE_COMPAT
#error "Unload diagnostics require the accepted on-use idle parent"
#endif

// Feedback-only loaded state after an actual lease stop; no motion authority.
#ifndef BMCU_LITE_LEASE_FEEDBACK
#define BMCU_LITE_LEASE_FEEDBACK 0
#endif
#if BMCU_LITE_LEASE_FEEDBACK != 0 && BMCU_LITE_LEASE_FEEDBACK != 1
#error "Lease feedback must be 0 or 1"
#endif
#if BMCU_LITE_LEASE_FEEDBACK && !BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE
#error "Lease feedback requires the accepted unload-diagnostic parent"
#endif
#if BMCU_LITE_LEASE_FEEDBACK
bool lite_lease_active();
void lite_feedback_clear();
void lite_lease_stop(bool retain=true);
#endif

// Diagnostic-only terminal cutoff, measured from the first accepted on_use.
#ifndef BMCU_LITE_AUTOMATIC_CAPTURE
#define BMCU_LITE_AUTOMATIC_CAPTURE 0
#endif
#if BMCU_LITE_AUTOMATIC_CAPTURE != 0 && BMCU_LITE_AUTOMATIC_CAPTURE != 1
#error "Automatic capture must be 0 or 1"
#endif
#if BMCU_LITE_AUTOMATIC_CAPTURE && !BMCU_LITE_LEASE_FEEDBACK
#error "Automatic capture requires the accepted lease-feedback parent"
#endif

// Retain DMA bytes until a complete-frame queue slot is available.
#ifndef BMCU_LITE_RX_BACKPRESSURE
#define BMCU_LITE_RX_BACKPRESSURE 0
#endif
#if BMCU_LITE_RX_BACKPRESSURE != 0 && BMCU_LITE_RX_BACKPRESSURE != 1
#error "RX backpressure must be 0 or 1"
#endif
#if BMCU_LITE_RX_BACKPRESSURE && !BMCU_LITE_AUTOMATIC_CAPTURE
#error "RX backpressure requires the automatic diagnostic parent"
#endif

// Bounded passive pre-cut evidence; inherited timer, motion and feedback stay authoritative.
#ifndef BMCU_LITE_PRECUT_PHASE_CAPTURE
#define BMCU_LITE_PRECUT_PHASE_CAPTURE 0
#endif
#if BMCU_LITE_PRECUT_PHASE_CAPTURE != 0 && BMCU_LITE_PRECUT_PHASE_CAPTURE != 1
#error "Precut phase capture must be 0 or 1"
#endif
#if BMCU_LITE_PRECUT_PHASE_CAPTURE && !BMCU_LITE_RX_BACKPRESSURE
#error "Precut phase capture requires RX backpressure"
#endif

#if BMCU_LITE_PRECUT_PHASE_CAPTURE && defined(__riscv)
#if !defined(SYSCLK_FREQ_144MHz_HSI) || SYSCLK_FREQ_144MHz_HSI != 144000000
#error "Precut phase capture requires the fixed144MHz HSI clock profile"
#endif
#endif

// One stopped revalidation poll for an isolated invalid runtime angle sample.
#ifndef BMCU_LITE_ENCODER_RECOVERY
#define BMCU_LITE_ENCODER_RECOVERY 0
#endif
#if BMCU_LITE_ENCODER_RECOVERY != 0 && BMCU_LITE_ENCODER_RECOVERY != 1
#error "Encoder recovery must be 0 or 1"
#endif
#if BMCU_LITE_ENCODER_RECOVERY && (!BMCU_LITE_PRECUT_PHASE_CAPTURE || !BMCU_LITE_INPUT_FAULT_DETAIL)
#error "Encoder recovery requires the accepted precut/input-detail profile"
#endif

// Bounded compatibility experiment: addressed07/99 feedback uses its requested slot.
#ifndef BMCU_LITE_PHASE99_REQUEST_FEEDBACK
#define BMCU_LITE_PHASE99_REQUEST_FEEDBACK 0
#endif
#if BMCU_LITE_PHASE99_REQUEST_FEEDBACK != 0 && BMCU_LITE_PHASE99_REQUEST_FEEDBACK != 1
#error "Phase99 request feedback must be 0 or 1"
#endif
#if BMCU_LITE_PHASE99_REQUEST_FEEDBACK && (!BMCU_LITE_ENCODER_RECOVERY || !BMCU_LITE_PHASE99_ALL_SLOTS)
#error "Phase99 request feedback requires the encoder-recovery/all-slots parent"
#endif
#if BMCU_USB_CONTINUOUS_JOURNAL != BMCU_LITE_PHASE99_REQUEST_FEEDBACK
#error "This packet requires the journal-retention and requested-slot feedback profile together"
#endif

// Passive printer/export selection, enabled only by the working profile.
#ifndef BMCU_USB_WORKING_MODE
#define BMCU_USB_WORKING_MODE 0
#endif
#if BMCU_USB_WORKING_MODE != 0 && BMCU_USB_WORKING_MODE != 1
#error "Working mode must be 0 or 1"
#endif
#if BMCU_USB_WORKING_MODE && !BMCU_USB_CONTINUOUS_JOURNAL
#error "Working mode requires the journal-retention profile"
#endif

// Explicit ISP-prepared export intent, consumed before passive printer RX.
#ifndef BMCU_USB_ISP_EXPORT
#define BMCU_USB_ISP_EXPORT 0
#endif
#if BMCU_USB_ISP_EXPORT != 0 && BMCU_USB_ISP_EXPORT != 1
#error "ISP export must be 0 or 1"
#endif
#if BMCU_USB_ISP_EXPORT && !BMCU_USB_WORKING_MODE
#error "ISP export requires working mode"
#endif
