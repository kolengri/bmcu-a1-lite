#include "MC_PULL_calibration.h"
#include "ws2812.h"

#include "Flash_saves.h"
#include "Motion_control.h"
#include "ams_addressing.h"
#include "_bus_hardware.h"
#include "ams.h"
#include "ahub_bus.h"
#include "bambu_bus_ams.h"
#include "ADC_DMA.h"
#include "Debug_log.h"
#include <string.h>
#include "lite_profile.h"
#include "authority_observation.h"
#include "flight_recorder.h"

WS2812_class SYS_RGB;
WS2812_class RGBOUT[4];

void RGB_init()
{
    SYS_RGB.init(1, GPIOD, GPIO_Pin_1);
    RGBOUT[0].init(2, GPIOA, GPIO_Pin_11);
    RGBOUT[1].init(2, GPIOA, GPIO_Pin_8);
    RGBOUT[2].init(2, GPIOB, GPIO_Pin_1);
    RGBOUT[3].init(2, GPIOB, GPIO_Pin_0);
}

void RGB_update()
{
    if (!(SYS_RGB.is_dirty() ||
          RGBOUT[0].is_dirty() || RGBOUT[1].is_dirty() ||
          RGBOUT[2].is_dirty() || RGBOUT[3].is_dirty()))
        return;

    if (!bus_background_ready()) return;

    static uint32_t last = 0u;

    uint32_t min_gap = time_hw_tpms;
    if (!min_gap) min_gap = 1u;

    const uint32_t now = time_ticks32();
    if (last != 0u && (uint32_t)(now - last) < min_gap)
        return;

    last = now;

    SYS_RGB.updata();
    RGBOUT[0].updata();
    RGBOUT[1].updata();
    RGBOUT[2].updata();
    RGBOUT[3].updata();
}

#if !BMCU_LITE_MOTORLOCK
static uint8_t g_fil_dirty = 0;
static uint8_t g_loaded_ch = 0xFF;
static uint8_t g_state_dirty = 0;
static uint8_t g_state_write_value = 0xFF;
static uint8_t g_loaded_flash_pending = 0;

static inline void ram_to_flashinfo(uint8_t fil, Flash_FilamentInfo* o)
{
    const _filament* f = &ams[BMCU_LOCAL_AMS_INDEX].filament[fil];

    memcpy(o->bambubus_filament_id, f->bambubus_filament_id, sizeof(o->bambubus_filament_id));
    o->color_R = f->color_R;
    o->color_G = f->color_G;
    o->color_B = f->color_B;
    o->color_A = f->color_A;
    o->temperature_min = f->temperature_min;
    o->temperature_max = f->temperature_max;
    memcpy(o->name, f->name, sizeof(o->name));
}

static inline void flashinfo_to_ram(uint8_t fil, const Flash_FilamentInfo* i)
{
    _filament* f = &ams[BMCU_LOCAL_AMS_INDEX].filament[fil];

    memcpy(f->bambubus_filament_id, i->bambubus_filament_id, sizeof(i->bambubus_filament_id));
    f->color_R = i->color_R;
    f->color_G = i->color_G;
    f->color_B = i->color_B;
    f->color_A = i->color_A;
    f->temperature_min = i->temperature_min;
    f->temperature_max = i->temperature_max;

    memset(f->name, 0, sizeof(f->name));
    memcpy(f->name, i->name, sizeof(i->name));
    f->name[sizeof(f->name) - 1u] = 0;
}

bool ams_datas_read()
{
    bool any = false;

    for (uint8_t fil = 0; fil < 4u; fil++)
    {
        Flash_FilamentInfo fi;
        if (Flash_AMS_filament_read(fil, &fi))
        {
            flashinfo_to_ram(fil, &fi);
            any = true;
        }
    }

    return any;
}

void ams_datas_set_need_to_save()
{
    g_fil_dirty = 0x0Fu;
}

void ams_datas_set_need_to_save_filament(uint8_t filament_idx)
{
    if (filament_idx >= 4u) return;
    g_fil_dirty |= (uint8_t)(1u << filament_idx);
}

void ams_state_claim_loaded(uint8_t filament_ch)
{
#if BMCU_LITE_FULL_LOAD
    (void)full_load_reserve(filament_ch); return;
#endif
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL
    if (filament_ch != 0u) return;
#endif
#if BMCU_EXPERIMENTAL_AMS_LITE
    if (!lite_motion_ready()) return;
#endif
    if (filament_ch >= 4u) return;
    if (g_loaded_ch != 0xFFu && g_loaded_ch != filament_ch) return;

    if (g_loaded_ch == 0xFFu)
    {
        g_loaded_ch = filament_ch;
        g_loaded_flash_pending = 1u;
    }
}

void ams_state_set_loaded(uint8_t filament_ch)
{
#if BMCU_LITE_FULL_LOAD
    return; // Operation already requires a verified durable reservation; use cannot create one.
#endif
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL
    ams_state_claim_loaded(filament_ch); return; // Trial reservations are RAM-only, never a full-load commit.
#endif
#if BMCU_EXPERIMENTAL_AMS_LITE
    if (!lite_motion_ready()) return;
#endif
    if (filament_ch >= 4u) return;
    if (g_loaded_ch != 0xFFu && g_loaded_ch != filament_ch) return;

    if (g_loaded_ch == 0xFFu)
    {
        g_loaded_ch = filament_ch;
        g_loaded_flash_pending = 1u;
    }

    if (!g_loaded_flash_pending) return;

    g_state_write_value = filament_ch;
    g_state_dirty = 1u;
    g_loaded_flash_pending = 0u;
}

void ams_state_set_unloaded(uint8_t filament_ch)
{
#if BMCU_LITE_FULL_LOAD
    return; // Only actual V11 unload/redetect completion may request durable FF.
#endif
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL
    if (g_loaded_ch != 0u || filament_ch != 0u) return;
#endif
#if BMCU_EXPERIMENTAL_AMS_LITE
    if (!lite_motion_ready()) return;
#endif
    if (g_loaded_ch == 0xFFu) return;
    if (filament_ch < 4u && g_loaded_ch != filament_ch) return;
    g_loaded_ch = 0xFFu;
    g_loaded_flash_pending = 0u;
    g_state_write_value = 0xFFu;
    g_state_dirty = 1u;
}

uint8_t ams_state_get_loaded(void)
{
#if BMCU_LITE_FULL_LOAD
    return full_load_owner();
#else
    return g_loaded_ch;
#endif
}

static void ams_state_save_run()
{
#if BMCU_EXPERIMENTAL_AMS_LITE
    if (!lite_motion_ready()) return;
#endif
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL
    return;
#endif
    if (!g_state_dirty) return;

    if (Flash_AMS_state_write(g_state_write_value))
        g_state_dirty = 0u;
}

void ams_datas_save_run()
{
    if (!g_fil_dirty) return;

    uint8_t fil = 0xFFu;
    for (uint8_t i = 0; i < 4u; i++)
    {
        if (g_fil_dirty & (uint8_t)(1u << i))
        {
            fil = i;
            break;
        }
    }

    if (fil == 0xFFu) return;

    Flash_FilamentInfo now;
    ram_to_flashinfo(fil, &now);

    if (Flash_AMS_filament_write(fil, &now))
        g_fil_dirty &= (uint8_t)~(1u << fil);
}

#else
void ams_datas_set_need_to_save_filament(uint8_t) {}
void ams_state_claim_loaded(uint8_t) {}
void ams_state_set_loaded(uint8_t) {}
void ams_state_set_unloaded(uint8_t) {}
uint8_t ams_state_get_loaded() { return 0xFFu; }
float Motion_control_filament_meters(uint8_t) { return 0.0f; }
#endif
#if BMCU_EXPERIMENTAL_AMS_LITE
static void lite_diagnostic_rgb() {
    const uint32_t rgb = lite_observation_rgb();
    SYS_RGB.set_RGB(rgb >> 16, rgb >> 8, rgb, 0);
#if BMCU_LITE_AUTHORITY_OBSERVATION
    if(time_hw_slow || !setup_trial_bus_allowed())
#endif
    for (auto &led : RGBOUT)
        for (uint8_t i = 0; i < 2; ++i) led.set_RGB(rgb >> 16, rgb >> 8, rgb, i);
    RGB_update();
}
#endif
#if BMCU_LITE_FULL_LOAD
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE && !BMCU_USB_CONTINUOUS_JOURNAL
static void capture_rgb(uint8_t red,uint8_t green){
    bus_shutdown(); // Terminal evidence/control already frozen; shared by waiting and parked cues.
    SYS_RGB.set_RGB(red,green,0,0);
    for(auto& led:RGBOUT){for(uint8_t i=0;i<2;++i)led.set_RGB(red,green,0,i);led.updata();}
    SYS_RGB.updata(); // Terminal bus is shut down; do not leave a cadence-deferred cue queued.
}
static void capture_park(){
    const uint8_t green=uint8_t(flight::saved())<<5;
    capture_rgb(green^32u,green);
    for(;;)time_ticks32();
}
#if BMCU_LITE_LEASE_FEEDBACK
static void capture_finish(uint8_t fault){
    capture_rgb(32u,16u); // Shutdown precedes draining and persistence.
    if(Flash_background_stop() && bus_background_ready())flight::finish_native(fault);
    capture_park();
}
#endif
#endif
static void full_load_step() {
#if BMCU_LITE_AUTOMATIC_CAPTURE
    flight::automatic_capture(); // Freeze/stop before dispatching more native work.
#endif
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE && !BMCU_USB_CONTINUOUS_JOURNAL
    if(flight::operator_pending()) {
        // Evidence already frozen and every bridge stopped. This unchanged
        // terminal shutdown bounds/aborts TX; no ensuing RX is dispatched.
#if BMCU_LITE_LEASE_FEEDBACK
        capture_finish(0);
#else
        capture_rgb(32u,16u);
        while(!Flash_background_idle()){Flash_background_run();time_ticks32();}
        if(bus_background_ready())flight::finish_native(0);
        capture_park();
#endif
    }
#endif
    bus_rx_poll();
    const bambubus_package_type status=bambubus_run();
    bus_port_to_host.send_package();
    if (status!=bambubus_package_type::none && status!=bambubus_package_type::error)
        bus_host_device_type=0x1200u;
    const int error=(bus_host_device_type!=0x1200u || status==bambubus_package_type::error)?-1:0;
    full_load_run();
#if BMCU_USB_FLIGHT_RECORDER && !BMCU_LITE_NATIVE_UNLOAD_CONTINUE
    if (flight::native_end_pending()) {
        Motion_control_lite_cancel(); // Complete verified Unload; no next local/native action.
        flight::finish_native(0);
        for (;;) time_ticks32(); // Terminal capture: successful verified completion.
    }
#endif
#if BMCU_LITE_V11_FEATURES
    Flash_metadata_recover_run();
    if (!Flash_metadata_recovery_pending() && !full_load_journal_pending() &&
        v11_metadata_allowed() && bus_background_ready()) ams_datas_save_run();
#endif
    Motion_control_run(error);
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE && !BMCU_USB_CONTINUOUS_JOURNAL
    if(flight::operator_pending())return; // The next step exclusively owns terminal shutdown/save.
#endif
    Flash_background_run();
#if BMCU_USB_CONTINUOUS_JOURNAL
    if(lite_recovery_fault())flight::native_interrupted();
    flight::journal_step();
#endif
#if BMCU_USB_FLIGHT_RECORDER && !BMCU_USB_CONTINUOUS_JOURNAL
    if (full_load_fault()!=FullLoadFault::none) {
#if BMCU_LITE_LEASE_FEEDBACK
        capture_finish(uint8_t(full_load_fault()));
#else
        flight::finish_native(uint8_t(full_load_fault()));
#endif
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE && !BMCU_USB_CONTINUOUS_JOURNAL
        capture_park();
#else
        for (;;) time_ticks32(); // Terminal capture: preserve the existing stopped bridge state.
#endif
    }
    // With no terminal FullLoadFault, recovery means a latched interrupted pull.
    // It refuses later commands, so capture the stopped state independently.
    if (lite_recovery_fault()) flight::native_interrupted();
    // One nonterminal checkpoint after native cancellation/recovery. The durable
    // loaded owner blocks local helpers; selection blocks active native work.
    // A later Load/use phase may continue after a successful stopped commit.
    if (flight::native_checkpoint_pending() && ams[0].now_filament_num==0xffu &&
        (full_load_durable(full_load_owner())
#if BMCU_LITE_NATIVE_UNLOAD_CONTINUE
         || (flight::native_end_pending() && full_load_empty())
#endif
        ) && Flash_background_idle() && bus_background_ready()) {
        if (!flight::finish_native(0)) full_load_fail(FullLoadFault::storage);
#if BMCU_LITE_UNLOAD_DIAGNOSTIC_CAPTURE && !BMCU_USB_CONTINUOUS_JOURNAL
        if(lite_recovery_fault())capture_park(); // Verified save cue for terminal interrupted pull.
#endif
    }
#endif
    if (lite_recovery_fault()) lite_diagnostic_rgb();
    else if (bambubus_arrange_wait() || error) SYS_RGB.set_RGB(0x10,0,0,0);
    else SYS_RGB.set_RGB(0x38,0x35,0x32,0);

#if BMCU_LITE_AUTHORITY_OBSERVATION
    authority_observation::render();
#endif
    RGB_update();
}
#endif
int main(void)
{
    SystemInit();
    SystemCoreClockUpdate();
    time_hw_init();

    __enable_irq();

    WWDG_DeInit();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);

#if BMCU_LITE_MOTORLOCK || BMCU_LITE_SINGLE_CHANNEL_TRIAL || BMCU_LITE_FULL_LOAD
    MC_PWM_init(); // Zero compare values before enabling any PWM output.
#endif
#if BMCU_USB_FLIGHT_RECORDER
    if (!flight::boot()) {
        flight::uart_init();
        for (;;) flight::export_step();
    }
#endif
    RGB_init();
    delay(10);

#if BMCU_LITE_MOTORLOCK
    ams_init();
    ams[0].online = true;
    for (auto &f : ams[0].filament) f.online = false;
    bambubus_init();
    bus_init();
    while (1) {
        bus_rx_poll();
        bambubus_run();
        bus_port_to_host.send_package();
        lite_diagnostic_rgb();
    }
#else
    SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
    for (int i = 0; i < 4; i++) RGBOUT[i].set_RGB(0, 0, 0, 0);
    RGB_update();
    delay(50);

    DEBUG_init();
    ams_init();
    Flash_saves_init();

    ADC_DMA_init();
    ADC_DMA_wait_full();
#if BMCU_LITE_SETUP_TRIAL
    setup_trial_boot(); // Setup/fault/success never return or enable the bus.
#endif

#if BMCU_LITE_FULL_LOAD
    uint8_t retained=0xffu;
    if (!Flash_AMS_state_read(&retained) || !MC_PULL_calibration_load_retained_full())
        full_load_fail(FullLoadFault::storage);
    full_load_boot(retained);
#elif BMCU_LITE_SINGLE_CHANNEL_TRIAL
    // Read and reserve ownership before any absent-channel cleanup can run.
    uint8_t retained = 0xFFu;
    const bool retained_ok = Flash_AMS_state_read(&retained);
    if (retained_ok) g_loaded_ch = retained;
    single_trial_boot_ownership(retained_ok, retained);
    if (!MC_PULL_calibration_load_retained_trial()) single_trial_stop(TrialStop::retained_calibration);
#else
    MC_PULL_calibration_boot();
#endif
    ams_datas_read();

    #if !BMCU_LITE_SINGLE_CHANNEL_TRIAL && !BMCU_LITE_FULL_LOAD
    {
        uint8_t ch = 0xFFu;
        if (Flash_AMS_state_read(&ch))
        {
            g_loaded_ch = ch;

            if (ch < 4u)
            {
                _ams* a = &ams[BMCU_LOCAL_AMS_INDEX];

                a->now_filament_num  = ch;
                a->filament_use_flag = 0x04;
                a->pressure          = 0x2B00;

                for (uint8_t i = 0; i < 4u; i++)
                    a->filament[i].motion = _filament_motion::idle;

#if BMCU_EXPERIMENTAL_AMS_LITE
                // Persistent loaded ownership is retained, but boot is not a use command.
                a->now_filament_num = 0xFFu;
                a->filament_use_flag = 0;
#else
                a->filament[ch].motion = _filament_motion::on_use;
#endif
            }
        }
    }

    #endif
    Motion_control_init();
#if !BMCU_LITE_SINGLE_CHANNEL_TRIAL && !BMCU_LITE_FULL_LOAD
    if (g_loaded_ch < 4u && (!Motion_control_filament_present(g_loaded_ch) ||
        !MC_PULL_calibration_is_valid(g_loaded_ch))) {
        _ams* a = &ams[BMCU_LOCAL_AMS_INDEX];
        a->filament[g_loaded_ch].motion = _filament_motion::idle;
        a->now_filament_num = 0xFFu;
        a->filament_use_flag = 0u;
        a->pressure = 0xF9C6u;
        ams_state_set_unloaded(g_loaded_ch);
    }
    #endif
    bambubus_init();
    bus_init();

    DEBUG("START\n");

    while (1)
    {
#if BMCU_LITE_FULL_LOAD
        full_load_step();
#else
        bus_rx_poll();
#if !BMCU_EXPERIMENTAL_AMS_LITE
        const ahubus_package_type   ahub_stu     = ahubus_run();
#endif
        const bambubus_package_type bambubus_stu = bambubus_run();
        bus_port_to_host.send_package();

        if (bambubus_stu != bambubus_package_type::none && bambubus_stu != bambubus_package_type::error)
            bus_host_device_type = BMCU_EXPERIMENTAL_AMS_LITE ? 0x1200u : host_device_type_ams;
#if !BMCU_EXPERIMENTAL_AMS_LITE
        else if (ahub_stu != ahubus_package_type::none && ahub_stu != ahubus_package_type::error)
            bus_host_device_type = host_device_type_ahub;
#endif

        const int error = bus_host_device_type == (BMCU_EXPERIMENTAL_AMS_LITE ? 0x1200u : host_device_type_ams)
            ? (bambubus_stu == bambubus_package_type::error ? -1 : 0)
#if !BMCU_EXPERIMENTAL_AMS_LITE
            : bus_host_device_type == host_device_type_ahub
                ? (ahub_stu == ahubus_package_type::error ? -1 : 0) : -1;
#else
            : -1;
#endif
#if !BMCU_LITE_SINGLE_CHANNEL_TRIAL
#if BMCU_EXPERIMENTAL_AMS_LITE
        if (lite_recovery_fault()) lite_diagnostic_rgb();
        else
#endif
        if (bambubus_arrange_wait()) SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
        else if (error) SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
        else SYS_RGB.set_RGB(0x38, 0x35, 0x32, 0);

#endif

#if !BMCU_LITE_SINGLE_CHANNEL_TRIAL && !BMCU_LITE_FULL_LOAD
        if (g_state_dirty && bus_background_ready()) ams_state_save_run();
        if (g_fil_dirty && bus_background_ready()) ams_datas_save_run();
#endif

        Motion_control_run(error);
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL
        const uint32_t rgb = single_trial_led_rgb();
        SYS_RGB.set_RGB(rgb >> 16, rgb >> 8, rgb, 0);
        for (auto &led : RGBOUT) for (uint8_t i=0;i<2;++i) led.set_RGB(rgb >> 16, rgb >> 8, rgb, i);
#else
        Flash_background_run();
#endif
        RGB_update();
#endif
    }
#endif
}
