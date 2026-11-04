#include "lite_profile.h"
#if BMCU_LITE_V11_FEATURES
#include "ams.h"
#endif
#if !BMCU_LITE_MOTORLOCK
#include "Flash_saves.h"
#include "hal/irq_wch.h"
#include "hal/time_hw.h"
#include <string.h>
#include "_bus_hardware.h"

#include "ch32v20x_rcc.h"
#include "ch32v20x_crc.h"
#include "ch32v20x_flash.h"

static_assert(sizeof(Flash_FilamentInfo) == 32u, "Flash_FilamentInfo must be 32 bytes");
static_assert(alignof(Flash_FilamentInfo) >= 4u, "Flash_FilamentInfo must be 4-byte aligned");

static uint32_t crc32_hw_words(const void* data, uint32_t bytes)
{
    const uint32_t* p = (const uint32_t*)data;
    CRC->CTLR = 1u;
    for (uint32_t i = 0u; i < (bytes >> 2); i++) CRC->DATAR = p[i];
    return CRC->DATAR;
}

static inline uint32_t ams_fil_page(uint8_t filament_idx)
{
    return FLASH_NVM_AMS_ADDR + (uint32_t)filament_idx * FLASH_NVM256_PAGE_SIZE;
}

static constexpr uint32_t FLASH_ERASED_WORD = 0xE339E339u;

class Flash_clock_guard
{
    const uint32_t was_slow;
public:
    Flash_clock_guard() : was_slow(time_hw_slow) {
        if (!was_slow) time_hw_flash_clock(1u);
    }
    ~Flash_clock_guard() {
        if (!was_slow && !(FLASH->STATR & FLASH_FLAG_BSY)) time_hw_flash_clock(0u);
    }
};

static inline bool flash_word_is_blank(uint32_t v)
{
    return v == FLASH_ERASED_WORD || v == 0xFFFFFFFFu;
}

static bool flash_range_is_erased(uint32_t base_addr, uint32_t bytes)
{
    Flash_clock_guard clock;
    const uint32_t* p = (const uint32_t*)base_addr;

    for (uint32_t i = 0u; i < (bytes >> 2); i++)
    {
        if (!flash_word_is_blank(p[i])) return false;
    }

    return true;
}

static constexpr uint32_t FLASH_ERASE_TIMEOUT_US = 32000u;
static constexpr uint32_t FLASH_PROGRAM_TIMEOUT_US = 4000u;
static bool g_flash_fault = false;

static bool flash_wait_ready(uint32_t timeout_us)
{
    const uint32_t started = time_ticks32();
    const uint32_t timeout = us_to_ticks32(timeout_us);
#if BMCU_USB_FLIGHT_RECORDER
    uint32_t bound=2000000u;
    while (FLASH->STATR & FLASH_FLAG_BSY) {
        if (!--bound || (uint32_t)(time_ticks32() - started) >= timeout) return false;
    }
#else
    while (FLASH->STATR & FLASH_FLAG_BSY)
        if ((uint32_t)(time_ticks32() - started) >= timeout) return false;
#endif
    return (FLASH->STATR & FLASH_FLAG_WRPRTERR) == 0u;
}

#if BMCU_USB_ISP_EXPORT
static constexpr uint32_t boot_export_page=0x0800ed00u;
static bool boot_export_checked=false,boot_export_erasing=false;
static bool boot_export_stopped(){return
    TIM2->CH1CVR==0u && TIM2->CH2CVR==0u && TIM3->CH1CVR==0u && TIM3->CH2CVR==0u &&
    TIM4->CH1CVR==0u && TIM4->CH2CVR==0u && TIM4->CH3CVR==0u && TIM4->CH4CVR==0u;}
#endif
#if BMCU_USB_FLIGHT_RECORDER
static bool log_address(uint32_t address) {
    return address>=0x0800ee00u && address<0x0800f000u &&
        TIM2->CH1CVR==TIM2->CH2CVR && TIM3->CH1CVR==TIM3->CH2CVR &&
        TIM4->CH1CVR==TIM4->CH2CVR && TIM4->CH3CVR==TIM4->CH4CVR;
}
static bool flash256_erase(uint32_t page_addr,bool log=false)
#else
static bool flash256_erase(uint32_t page_addr)
#endif
{
    if (page_addr & (FLASH_NVM256_PAGE_SIZE - 1u)) return false;
#if BMCU_LITE_SETUP_TRIAL
#if BMCU_USB_FLIGHT_RECORDER
#if BMCU_USB_ISP_EXPORT
    // This private scope is opened only by the one-shot validated boot consumer.
    if(page_addr==boot_export_page){
        if(!boot_export_erasing || setup_trial_phase()!=SetupPhase::inspect || !boot_export_stopped() || !bus_boot_quiet())return false;
    }else
#endif
    if (log ? !log_address(page_addr) : !setup_trial_flash_address(page_addr)) return false;
#else
    if (!setup_trial_flash_address(page_addr)) return false;
#endif
#endif
    if (g_flash_fault || !Flash_background_idle()) return false;
    Flash_clock_guard clock;
    const uint32_t irq = irq_save_wch();
    FLASH_Unlock_Fast();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
    FLASH->CTLR |= 0x00020000u;
    FLASH->ADDR = page_addr;
    FLASH->CTLR |= 0x00000040u;
    const bool ok = flash_wait_ready(FLASH_ERASE_TIMEOUT_US);
    if (!(FLASH->STATR & FLASH_FLAG_BSY)) {
        FLASH->CTLR &= ~0x00020000u;
        FLASH_Lock_Fast();
        FLASH_Lock();
    }
    irq_restore_wch(irq);
    const bool verified = ok && flash_range_is_erased(page_addr, FLASH_NVM256_PAGE_SIZE);
    if (!verified) g_flash_fault = true;
    return verified;
}

#if BMCU_USB_ISP_EXPORT
BootExportMarker Flash_boot_export_take(const uint32_t expected[4]){
    if(boot_export_checked)return BootExportMarker::failed;
    boot_export_checked=true;
    if(!expected || setup_trial_phase()!=SetupPhase::inspect || !boot_export_stopped() ||
       !bus_boot_quiet() || g_flash_fault || !Flash_background_idle())return BootExportMarker::failed;
    Flash_clock_guard clock;
    if(memcmp((const void*)boot_export_page,expected,16))return BootExportMarker::absent;
    boot_export_erasing=true;const bool ok=flash256_erase(boot_export_page);boot_export_erasing=false;
    return ok?BootExportMarker::consumed:BootExportMarker::failed;
}
#endif

#if BMCU_USB_FLIGHT_RECORDER
static bool flash_word_prog_std(uint32_t addr, uint32_t data,bool log=false)
#else
static bool flash_word_prog_std(uint32_t addr, uint32_t data)
#endif
{
    if ((addr & 3u) || g_flash_fault || !Flash_background_idle()) return false;
#if BMCU_LITE_SETUP_TRIAL
#if BMCU_USB_FLIGHT_RECORDER
    if (log ? !log_address(addr) : !setup_trial_flash_address(addr)) return false;
#else
    if (!setup_trial_flash_address(addr)) return false;
#endif
#endif
    Flash_clock_guard clock;
    if (*(const volatile uint32_t*)addr == data) return true;
    const uint32_t irq = irq_save_wch();
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
    FLASH->CTLR |= 1u;
    *(volatile uint16_t*)addr = (uint16_t)data;
    bool ok = flash_wait_ready(FLASH_PROGRAM_TIMEOUT_US);
    if (ok) {
        *(volatile uint16_t*)(addr + 2u) = (uint16_t)(data >> 16);
        ok = flash_wait_ready(FLASH_PROGRAM_TIMEOUT_US);
    }
    if (!(FLASH->STATR & FLASH_FLAG_BSY)) {
        FLASH->CTLR &= ~1u;
        FLASH_Lock();
    }
    irq_restore_wch(irq);
    const bool verified = ok && *(const volatile uint32_t*)addr == data;
    if (!verified) g_flash_fault = true;
    return verified;
}

static bool flash_prog_words(uint32_t base_addr, const uint32_t* words, uint32_t count)
{
    if (base_addr & 3u) return false;

    for (uint32_t i = 0u; i < count; i++)
    {
        if (!flash_word_prog_std(base_addr + (i << 2), words[i]))
            return false;
    }

    return true;
}

static bool nvm256_write(uint32_t page_addr, uint32_t magic, uint16_t ver, uint32_t rsv,
                         const void* payload, uint16_t len)
{
    if (len > (uint16_t)(NVM256_CRC_OFF - sizeof(NVM256_HDR))) return false;
    if (g_flash_fault || !Flash_background_idle()) return false;
    Flash_clock_guard clock;

    alignas(4) uint32_t w[64];
    uint8_t* b = (uint8_t*)w;
    memset(b, 0xFF, FLASH_NVM256_PAGE_SIZE);

    NVM256_HDR h{};
    h.magic = magic;
    h.ver = ver;
    h.len = len;
    h.rsv = rsv;

    memcpy(b, &h, sizeof(h));
    if (len) memcpy(b + sizeof(h), payload, len);

    const uint32_t crc = crc32_hw_words(b, NVM256_CRC_OFF);
    memcpy(b + NVM256_CRC_OFF, &crc, 4u);

    if (!flash256_erase(page_addr)) return false;
    if (!flash_prog_words(page_addr + 4u, w + 1u, 63u)) return false;
    return flash_word_prog_std(page_addr, w[0]);
}

static bool nvm256_read(uint32_t page_addr, uint32_t magic, uint16_t ver,
                        void* out, uint16_t max_len, uint16_t* got_len, uint32_t* rsv_out)
{
    if (FLASH->STATR & FLASH_FLAG_BSY) return false;
    Flash_clock_guard clock;
    const uint8_t* b = (const uint8_t*)page_addr;

    const uint32_t stored = *(const uint32_t*)(b + NVM256_CRC_OFF);
    if (flash_word_is_blank(stored)) return false;

    const uint32_t crc = crc32_hw_words(b, NVM256_CRC_OFF);
    if (crc != stored) return false;

    NVM256_HDR h{};
    memcpy(&h, b, sizeof(h));

    if (h.magic != magic) return false;
    if (h.ver != ver) return false;
    if (h.len > max_len) return false;

    if (h.len) memcpy(out, b + sizeof(h), h.len);
    if (got_len) *got_len = h.len;
    if (rsv_out) *rsv_out = h.rsv;
    return true;
}

static constexpr uint32_t FIL_SLOT_WORDS = 10u;
static constexpr uint32_t FIL_SLOT_BYTES = FIL_SLOT_WORDS * 4u;
static constexpr uint32_t FIL_SLOTS_PER_PAGE = 6u;
static constexpr uint32_t FIL_SCRATCH_ADDR = FLASH_NVM_BASE_ADDR + 14u * FLASH_NVM256_PAGE_SIZE;
static constexpr uint32_t FIL_SCRATCH_MAGIC = 0x32524340u;

static_assert(FIL_SLOT_BYTES * FIL_SLOTS_PER_PAGE <= FLASH_NVM256_PAGE_SIZE, "FIL journal too large");

static inline uint32_t fil_page_addr(uint8_t filament_idx)
{
    return ams_fil_page(filament_idx);
}

static inline void fil_slot_pack(uint32_t w[FIL_SLOT_WORDS], const Flash_FilamentInfo* info)
{
    w[0] = MAGIC_FIL;
    memcpy(&w[1], info, sizeof(*info));
    w[FIL_SLOT_WORDS - 1u] = crc32_hw_words(w, (FIL_SLOT_WORDS - 1u) * 4u);
}

static inline bool fil_slot_valid(const uint32_t* p, Flash_FilamentInfo* out)
{
    if (p[0] != MAGIC_FIL) return false;
    if (crc32_hw_words(p, (FIL_SLOT_WORDS - 1u) * 4u) != p[FIL_SLOT_WORDS - 1u]) return false;
    if (out) memcpy(out, &p[1], sizeof(*out));
    return true;
}

static bool fil_scan_page(uint32_t base, Flash_FilamentInfo* last, uint32_t* first_empty)
{
    Flash_clock_guard clock;
    bool found = false;

    if (first_empty) *first_empty = FIL_SLOTS_PER_PAGE;

    for (uint32_t s = 0u; s < FIL_SLOTS_PER_PAGE; s++)
    {
        const uint32_t* p = (const uint32_t*)(base + s * FIL_SLOT_BYTES);

        bool blank = true;
        for (uint32_t i = 0u; i < FIL_SLOT_WORDS; i++)
            if (!flash_word_is_blank(p[i])) { blank = false; break; }
        if (blank)
        {
            if (first_empty && *first_empty == FIL_SLOTS_PER_PAGE)
                *first_empty = s;
            continue;
        }

        if (first_empty) *first_empty = FIL_SLOTS_PER_PAGE;
        Flash_FilamentInfo tmp;
        if (fil_slot_valid(p, &tmp))
        {
            if (last) *last = tmp;
            found = true;
        }
    }

    return found;
}

static uint8_t g_fil_have[4] = {0u, 0u, 0u, 0u};
static uint8_t g_fil_first_empty[4] = {0u, 0u, 0u, 0u};
static Flash_FilamentInfo g_fil_last[4];

static void fil_cache_load_one(uint8_t filament_idx)
{
    Flash_FilamentInfo last;
    uint32_t first_empty = FIL_SLOTS_PER_PAGE;
    const uint32_t base = fil_page_addr(filament_idx);

    if (fil_scan_page(base, &last, &first_empty))
    {
        g_fil_have[filament_idx] = 1u;
        g_fil_first_empty[filament_idx] = (uint8_t)first_empty;
        memcpy(&g_fil_last[filament_idx], &last, sizeof(last));
        return;
    }

    g_fil_have[filament_idx] = 0u;
    g_fil_first_empty[filament_idx] = (uint8_t)first_empty;
    memset(&g_fil_last[filament_idx], 0, sizeof(g_fil_last[filament_idx]));
}

static constexpr uint32_t STA_TAG = 0xA6u;
static constexpr uint32_t STA_PAGE_FIRST = 6u;
static constexpr uint32_t STA_PAGE_COUNT = 8u;
static constexpr uint32_t STA_SLOT_BYTES = 8u;
static constexpr uint32_t STA_SLOTS_PER_PAGE = (FLASH_NVM256_PAGE_SIZE / STA_SLOT_BYTES);
static constexpr uint32_t STA_TOTAL_SLOTS = (STA_PAGE_COUNT * STA_SLOTS_PER_PAGE);
static constexpr uint32_t CAL_BACKUP_ADDR = FLASH_NVM_BASE_ADDR + 15u * FLASH_NVM256_PAGE_SIZE;

static uint16_t g_sta_seq = 0u;
static uint16_t g_sta_slot = 0u;
static uint16_t g_sta_saved_slot = 0u;
static uint8_t g_sta_have_saved = 0u;
static uint8_t g_sta_saved_loaded = 0xFFu;
static bool g_sta_scanned = false;
#if BMCU_LITE_SETUP_TRIAL
static bool g_setup_journal_invalid = false;
static bool g_setup_cal_ambiguous = false;
#endif

static void flash_runtime_cache_clear(void)
{
    memset(g_fil_have, 0, sizeof(g_fil_have));
    memset(g_fil_first_empty, 0, sizeof(g_fil_first_empty));
    memset(g_fil_last, 0, sizeof(g_fil_last));

    g_sta_seq = 0u;
    g_sta_slot = 0u;
    g_sta_saved_slot = 0u;
    g_sta_have_saved = 0u;
    g_sta_saved_loaded = 0xFFu;
    g_sta_scanned = false;
}

bool Flash_NVM_full_clear(void)
{
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL || BMCU_LITE_FULL_LOAD
    return false; // Read-only trial: no erase, program or journal submission.
#else
    if (!Flash_background_idle()) return false;
    Flash_clock_guard clock;
    for (uint32_t i = 0u; i < FLASH_NVM_PAGE_COUNT; i++)
        if (!flash256_erase(FLASH_NVM_BASE_ADDR + i * FLASH_NVM256_PAGE_SIZE)) return false;

    flash_runtime_cache_clear();
    return true;
#endif
}

static inline uint32_t sta_page_addr(uint32_t page_i)
{
    return FLASH_NVM_BASE_ADDR + (STA_PAGE_FIRST + page_i) * FLASH_NVM256_PAGE_SIZE;
}

static inline uint32_t sta_slot_addr(uint32_t slot)
{
    const uint32_t page_i = slot / STA_SLOTS_PER_PAGE;
    const uint32_t slot_i = slot - page_i * STA_SLOTS_PER_PAGE;
    return sta_page_addr(page_i) + slot_i * STA_SLOT_BYTES;
}

struct Flash_job
{
    uint32_t words[FIL_SLOT_WORDS];
    uint32_t addr;
    uint32_t started;
    uint16_t slot;
    uint8_t kind;
    uint8_t channel;
    uint8_t halfwords;
    uint8_t index;
    uint8_t phase;
    bool erase;
    bool rollover;
};

static Flash_job g_flash_job = {};
#if BMCU_USB_CONTINUOUS_JOURNAL
static int diagnostic_result=0;
int Flash_diagnostic_result(){return diagnostic_result;}
static bool diagnostic_job_valid(const Flash_job& job){
    return job.kind==5u && job.addr>=0x0800ee00u && job.addr<0x0800f000u &&
        !(job.addr&1u) && (job.erase ? (!(job.addr&255u) && job.halfwords==0u) : job.halfwords==1u);
}
bool Flash_diagnostic_submit(uint32_t address,uint16_t value,bool erase){
    if(g_flash_fault || !Flash_background_idle() || diagnostic_result==1) return false;
    Flash_job job={};job.kind=5;job.addr=address;job.words[0]=value;job.erase=erase;job.halfwords=erase?0:1;
    if(!diagnostic_job_valid(job))return false;
    g_flash_job=job;diagnostic_result=1;return true;
}
#endif
#if BMCU_LITE_V11_FEATURES
static bool metadata_recovery=false;
bool Flash_metadata_recovery_pending() { return metadata_recovery; }
static bool metadata_job_valid(const Flash_job &job) {
    if (job.channel>=4u) return false;
    if (job.kind==4u) return job.addr==FIL_SCRATCH_ADDR && job.halfwords==0u && job.erase;
    if (job.halfwords!=FIL_SLOT_WORDS*2u) return false;
    if (job.kind==3u) return job.addr==FIL_SCRATCH_ADDR;
    if (job.kind!=1u) return false;
    const uint32_t base=fil_page_addr(job.channel);
    return job.addr>=base && job.addr+FIL_SLOT_BYTES<=base+FLASH_NVM256_PAGE_SIZE &&
        (job.addr-base)%FIL_SLOT_BYTES==0u;
}
void Flash_metadata_recover_run() {
    if (!metadata_recovery || !Flash_background_idle() || !full_load_ready() ||
        !lite_motion_ready() || full_load_journal_pending() || ams[0].now_filament_num<4u) return;
    // Existing valid scratch format is durable. Resume its existing copy/clear
    // job after startup/lease admission, never by synchronous boot flash writes.
    Flash_job &job=g_flash_job; job={}; job.kind=1u;
    const uint32_t *saved=(const uint32_t*)FIL_SCRATCH_ADDR;
    job.channel=saved[0]&3u; job.addr=fil_page_addr(job.channel);
    job.erase=true; job.rollover=true; job.halfwords=FIL_SLOT_WORDS*2u;
    fil_slot_pack(job.words,&g_fil_last[job.channel]);
}
#endif

static uint8_t flash_job_halfword(const Flash_job& job)
{
#if BMCU_USB_CONTINUOUS_JOURNAL
    if(job.kind==5u)return 0;
#endif
    const uint8_t index = job.index + 2u;
    return index < job.halfwords ? index : index - job.halfwords;
}

bool Flash_background_idle(void)
{
    return !g_flash_job.kind && !(FLASH->STATR & FLASH_FLAG_BSY);
}

#if BMCU_LITE_LEASE_FEEDBACK
static bool flash_stopping=false;
#endif
void Flash_background_run(void)
{
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL && !BMCU_LITE_SETUP_TRIAL
    return;
#else
#if BMCU_LITE_SETUP_TRIAL
#if BMCU_USB_CONTINUOUS_JOURNAL
    if(!diagnostic_job_valid(g_flash_job)) {
#endif
#if BMCU_LITE_LEASE_FEEDBACK
    if(!flash_stopping) {
#endif
#if BMCU_LITE_FULL_LOAD
    if (setup_trial_phase()==SetupPhase::operation) {
#if BMCU_LITE_V11_FEATURES
        const bool journal=g_flash_job.kind==2u && g_flash_job.halfwords==4u &&
            g_flash_job.addr>=0x0800f600u && g_flash_job.addr<0x0800fe00u &&
            full_load_journal_target(g_flash_job.channel);
        if (g_flash_job.kind && !journal
#if BMCU_USB_CONTINUOUS_JOURNAL
            && !diagnostic_job_valid(g_flash_job)
#endif
#if BMCU_LITE_V11_FEATURES
            && !metadata_job_valid(g_flash_job)
#endif
        ) {
#else
        if (g_flash_job.kind && (g_flash_job.kind!=2u || g_flash_job.halfwords!=4u ||
            g_flash_job.addr<0x0800f600u || g_flash_job.addr>=0x0800fe00u ||
            !full_load_journal_target(g_flash_job.channel))) {
#endif
            full_load_fail(FullLoadFault::capability); return;
        }
    } else
#endif
    {
    if (setup_trial_phase() != SetupPhase::journal || !setup_trial_write_allowed(SetupWrite::journal)) return;
    if (g_flash_job.kind && (g_flash_job.kind!=2u || g_flash_job.addr!=0x0800f600u ||
        g_flash_job.channel!=0xffu || g_flash_job.halfwords!=4u || g_flash_job.erase)) {
        setup_trial_fail(SetupFault::capability); return;
    }
    }
#endif
#if BMCU_LITE_LEASE_FEEDBACK
    }
#endif
#if BMCU_USB_CONTINUOUS_JOURNAL
    }
#endif
    Flash_job &job = g_flash_job;
    if (!job.kind) {
        if (time_hw_slow && !(FLASH->STATR & FLASH_FLAG_BSY)) {
            FLASH->CTLR &= ~(0x00020000u | 1u);
            FLASH_Lock_Fast();
            FLASH_Lock();
            time_hw_flash_clock(0u);
        }
        return;
    }
    if (job.phase) {
        if (FLASH->STATR & FLASH_FLAG_BSY) {
            const uint32_t timeout = job.phase == 1u ? FLASH_ERASE_TIMEOUT_US : FLASH_PROGRAM_TIMEOUT_US;
            if ((uint32_t)(time_ticks32() - job.started) >= us_to_ticks32(timeout)) {
#if BMCU_USB_CONTINUOUS_JOURNAL
                if(job.kind==5u)diagnostic_result=-1;else
#endif
                g_flash_fault = true;
            }
            return;
        }
        FLASH->CTLR &= ~(0x00020000u | 1u);
        FLASH_Lock_Fast();
        FLASH_Lock();
        bool ok = !g_flash_fault && !(FLASH->STATR & FLASH_FLAG_WRPRTERR);
#if BMCU_USB_CONTINUOUS_JOURNAL
        if(job.kind==5u && diagnostic_result<0)ok=false;
#endif
        if (job.phase == 1u) {
            ok = ok && flash_range_is_erased(job.addr & ~(FLASH_NVM256_PAGE_SIZE - 1u), FLASH_NVM256_PAGE_SIZE);
            job.erase = false;
        } else {
            const uint8_t index = flash_job_halfword(job);
            const uint16_t expected = (uint16_t)(job.words[index >> 1] >> ((index & 1u) * 16u));
            ok = ok && *(const volatile uint16_t*)(job.addr + (uint32_t)index * 2u) == expected;
            ++job.index;
        }
        job.phase = 0u;
        time_hw_flash_clock(0u);
        if (!ok) {
#if BMCU_USB_CONTINUOUS_JOURNAL
            if(job.kind==5u){
                diagnostic_result=-1;FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPRTERR);
                job.kind=0u;return;
            }
#endif
            g_flash_fault = true;
            job.kind = 0u;
            return;
        }
        if (job.index == job.halfwords) {
#if BMCU_USB_CONTINUOUS_JOURNAL
            if(job.kind==5u){diagnostic_result=2;job.kind=0u;return;}
#endif
            if (job.kind == 3u) {
                job.kind = 1u;
                job.addr = fil_page_addr(job.channel);
                job.words[0] = MAGIC_FIL;
                job.words[FIL_SLOT_WORDS - 1u] = crc32_hw_words(job.words, (FIL_SLOT_WORDS - 1u) * 4u);
                job.index = 0u;
                job.erase = true;
                return;
            }
            if (job.kind == 1u && job.rollover) {
                job.kind = 4u;
                job.addr = FIL_SCRATCH_ADDR;
                job.index = job.halfwords = 0u;
                job.erase = true;
                return;
            }
            if (job.kind == 1u || job.kind == 4u) {
                memcpy(&g_fil_last[job.channel], &job.words[1], sizeof(Flash_FilamentInfo));
                g_fil_have[job.channel] = 1u;
#if BMCU_LITE_V11_FEATURES
                if (job.rollover) g_fil_first_empty[job.channel]=1u;
                if (job.kind==4u) metadata_recovery=false;
#endif
            } else {
                g_sta_seq = (uint16_t)(g_sta_seq + 1u);
                g_sta_saved_slot = job.slot;
                g_sta_saved_loaded = job.channel;
                g_sta_have_saved = 1u;
            }
            job.kind = 0u;
            return;
        }
    }
#if BMCU_LITE_LEASE_FEEDBACK
    if(flash_stopping)return;
#endif
#if BMCU_LITE_FULL_LOAD
    // Finish/lock an already issued peripheral operation above, but a latched
    // fault or lost lease cannot issue another ownership erase/program below.
    if (
#if BMCU_USB_CONTINUOUS_JOURNAL
        // A real latched stop keeps an incomplete core record incomplete. It may
        // release unissued work after the issued operation above has settled.
        job.kind!=5u && full_load_fault()!=FullLoadFault::none) {job.kind=0u;return;}
    if (
        job.kind!=5u &&
#endif
        setup_trial_phase()==SetupPhase::operation &&
        (!full_load_ready() || !lite_motion_ready())) return;
#endif
    if (g_flash_fault || !bus_background_ready()) return;
    const uint32_t irq = irq_save_wch();
    time_hw_flash_clock(1u);
    job.started = time_ticks32();
    if (job.erase) {
        FLASH_Unlock_Fast();
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
        FLASH->CTLR |= 0x00020000u;
        FLASH->ADDR = job.addr & ~(FLASH_NVM256_PAGE_SIZE - 1u);
        job.phase = 1u;
        FLASH->CTLR |= 0x00000040u;
    } else {
        const uint8_t index = flash_job_halfword(job);
        FLASH_Unlock();
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
        FLASH->CTLR |= 1u;
        job.phase = 2u;
        *(volatile uint16_t*)(job.addr + (uint32_t)index * 2u) =
            (uint16_t)(job.words[index >> 1] >> ((index & 1u) * 16u));
    }
    irq_restore_wch(irq);
#endif
}

#if BMCU_LITE_LEASE_FEEDBACK
bool Flash_background_stop(){
    flash_stopping=true; // Terminal only: no subsequent ownership/metadata submission.
    if(!g_flash_job.phase && !flash_wait_ready(FLASH_ERASE_TIMEOUT_US)){
        g_flash_fault=true;return false; // Bound an untracked already-busy peripheral without issuing work.
    }
    do {Flash_background_run();time_ticks32();} while(g_flash_job.phase && !g_flash_fault);
    if(FLASH->STATR&FLASH_FLAG_BSY)return false;
    g_flash_job.kind=0u; // Discard unissued remainder; never complete it to obtain idle.
    Flash_background_run(); // Restore locks/clock even for a phase-zero job.
    return !g_flash_fault;
}
#endif
static bool fil_scratch_recover(void)
{
    if (g_flash_fault || !Flash_background_idle()) return false;
    Flash_clock_guard clock;
    const uint32_t* saved = (const uint32_t*)FIL_SCRATCH_ADDR;
    if ((saved[0] & ~3u) != FIL_SCRATCH_MAGIC ||
        crc32_hw_words(saved, (FIL_SLOT_WORDS - 1u) * 4u) != saved[FIL_SLOT_WORDS - 1u])
        return true;
    const uint8_t channel = saved[0] & 3u;
    Flash_FilamentInfo info;
    memcpy(&info, &saved[1], sizeof(info));
    const bool restored = g_fil_have[channel] && memcmp(&g_fil_last[channel], &info, sizeof(info)) == 0;
    g_fil_last[channel] = info;
    g_fil_have[channel] = 1u;
    if (!restored) {
        uint32_t words[FIL_SLOT_WORDS];
        fil_slot_pack(words, &info);
        const uint32_t addr = fil_page_addr(channel);
        if (!flash256_erase(addr) || !flash_prog_words(addr + 4u, words + 1u, FIL_SLOT_WORDS - 1u) ||
            !flash_word_prog_std(addr, words[0])) return false;
        g_fil_first_empty[channel] = 1u;
    }
    return flash256_erase(FIL_SCRATCH_ADDR);
}

void Flash_saves_init(void)
{
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL || BMCU_LITE_FULL_LOAD
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
    flash_runtime_cache_clear();
    g_flash_job = {};
    g_flash_fault = false;
    for (uint8_t i=0;i<4;++i) fil_cache_load_one(i);
#if BMCU_LITE_V11_FEATURES
    metadata_recovery=false;
    const uint32_t *scratch=(const uint32_t*)FIL_SCRATCH_ADDR;
    if (!flash_range_is_erased(FIL_SCRATCH_ADDR,FLASH_NVM256_PAGE_SIZE)) {
        if ((scratch[0]&~3u)!=FIL_SCRATCH_MAGIC ||
            crc32_hw_words(scratch,(FIL_SLOT_WORDS-1u)*4u)!=scratch[FIL_SLOT_WORDS-1u]) g_flash_fault=true;
        else {
            const uint8_t ch=scratch[0]&3u;
            memcpy(&g_fil_last[ch],&scratch[1],sizeof(Flash_FilamentInfo));
            g_fil_have[ch]=1; metadata_recovery=true;
        }
    }
#endif
    // No unlock/program/erase or ownership cleanup at startup.
#else
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

    flash_runtime_cache_clear();
    g_flash_job = {};
    g_flash_fault = false;
    Flash_clock_guard clock;
    FLASH_Unlock();
    FLASH_Unlock_Fast();
    FLASH_Access_Clock_Cfg(FLASH_Access_SYSTEM_HALF);
    FLASH_Lock_Fast();
    FLASH_Lock();

    for (uint8_t i = 0u; i < 4u; i++)
        fil_cache_load_one(i);
    uint8_t loaded;
    if (!Flash_AMS_state_read(&loaded)) g_flash_fault = true;
    if (!fil_scratch_recover()) g_flash_fault = true;
#endif
}

bool Flash_AMS_filament_write(uint8_t filament_idx, const Flash_FilamentInfo* info)
{
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL || (BMCU_LITE_FULL_LOAD && !BMCU_LITE_V11_FEATURES)
    return false; // Read-only trial: no erase, program or journal submission.
#else
#if BMCU_LITE_V11_FEATURES
    if (!v11_metadata_allowed() || metadata_recovery) return false;
#endif
    if (!info || filament_idx >= 4u) return false;
    if (g_flash_fault || !Flash_background_idle()) return false;

    if (g_fil_have[filament_idx] &&
        memcmp(&g_fil_last[filament_idx], info, sizeof(*info)) == 0)
        return true;

    uint32_t slot = g_fil_first_empty[filament_idx];
    const bool erase = slot >= FIL_SLOTS_PER_PAGE;
    if (erase) slot = 0u;
    Flash_job &job = g_flash_job;
    job = {};
    job.kind = erase ? 3u : 1u;
    job.channel = filament_idx;
    job.addr = erase ? FIL_SCRATCH_ADDR : fil_page_addr(filament_idx) + slot * FIL_SLOT_BYTES;
    job.erase = erase;
    job.rollover = erase;
    job.halfwords = FIL_SLOT_WORDS * 2u;
    fil_slot_pack(job.words, info);
    if (erase) {
        job.words[0] = FIL_SCRATCH_MAGIC | filament_idx;
        job.words[FIL_SLOT_WORDS - 1u] = crc32_hw_words(job.words, (FIL_SLOT_WORDS - 1u) * 4u);
    }
    g_fil_first_empty[filament_idx] = (uint8_t)(slot + 1u);
    return false;
#endif
}

bool Flash_AMS_filament_read(uint8_t filament_idx, Flash_FilamentInfo* out)
{
    if (!out || filament_idx >= 4u) return false;
    if (!g_fil_have[filament_idx]) return false;

    memcpy(out, &g_fil_last[filament_idx], sizeof(*out));
    return true;
}

bool Flash_AMS_filament_clear(uint8_t filament_idx)
{
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL || BMCU_LITE_FULL_LOAD
    return false; // Read-only trial: no erase, program or journal submission.
#else
    if (filament_idx >= 4u) return false;
    if (!flash256_erase(fil_page_addr(filament_idx))) return false;

    g_fil_have[filament_idx] = 0u;
    g_fil_first_empty[filament_idx] = 0u;
    memset(&g_fil_last[filament_idx], 0, sizeof(g_fil_last[filament_idx]));
    return true;
#endif
}

static uint32_t sta_find_slot(bool* erase)
{
    uint32_t slot = g_sta_slot;
    uint32_t scanned = 0u;
    for (; scanned < STA_TOTAL_SLOTS; scanned++) {
        if (flash_range_is_erased(sta_slot_addr(slot), STA_SLOT_BYTES)) break;
        slot = (slot + 1u) % STA_TOTAL_SLOTS;
    }
    *erase = scanned == STA_TOTAL_SLOTS;
    if (*erase) {
        uint32_t page = g_sta_slot / STA_SLOTS_PER_PAGE;
        if (g_sta_have_saved && page == g_sta_saved_slot / STA_SLOTS_PER_PAGE)
            page = (page + 1u) % STA_PAGE_COUNT;
        slot = page * STA_SLOTS_PER_PAGE;
    }
    return slot;
}

#if BMCU_LITE_OWNERSHIP_DETAIL
static TrialOwnershipRead g_trial_ownership_read = TrialOwnershipRead::unobserved;
TrialOwnershipRead Flash_trial_ownership_read_observation() { return g_trial_ownership_read; }
#endif

#if BMCU_LITE_SETUP_TRIAL
struct Setup_sta_record { uint16_t seq; uint8_t channel; };

// The combined reader and its ordering passes share this actual integrity validator.
// No record array or schema change: bounded rereads trade boot work for constant stack.
static bool setup_sta_read(uint32_t slot, Setup_sta_record& record)
{
    const uint32_t a = sta_slot_addr(slot);
    const uint32_t w0 = *(const volatile uint32_t*)(a + 0u);
    const uint32_t w1 = *(const volatile uint32_t*)(a + 4u);
    const bool blank = flash_word_is_blank(w0) && flash_word_is_blank(w1);
    const uint8_t ch = (uint8_t)w0;
    if ((w0 >> 24) != STA_TAG || (w0 ^ w1) != MAGIC_STA || (ch >= 4u && ch != 0xffu)) {
        if (!blank) g_setup_journal_invalid = true;
        return false;
    }
    record.seq = (uint16_t)(w0 >> 8);
    record.channel = ch;
    return true;
}
#endif

bool Flash_AMS_state_read(uint8_t* loaded_ch)
{
    if (!loaded_ch) return false;
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL || BMCU_LITE_FULL_LOAD
    // Trial eligibility needs a valid retained record, including a valid FF.
    // Cached sentinels must not turn an absent/invalid journal into empty ownership.
    if (g_flash_fault || !Flash_background_idle()) {
#if BMCU_LITE_OWNERSHIP_DETAIL
        // Preserve the exact existing guard and its short-circuit priority.
        g_trial_ownership_read = g_flash_fault ? TrialOwnershipRead::flash_fault : TrialOwnershipRead::flash_busy;
#endif
        return false;
    }
#endif
    if (g_sta_scanned) {
        *loaded_ch = g_sta_saved_loaded;
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL || BMCU_LITE_FULL_LOAD
#if BMCU_LITE_OWNERSHIP_DETAIL
        g_trial_ownership_read = g_sta_have_saved ? TrialOwnershipRead::valid : TrialOwnershipRead::no_valid_record;
#endif
        return g_sta_have_saved != 0u;
#else
        return true;
#endif
    }
    if (g_flash_fault || !Flash_background_idle()) {
#if BMCU_LITE_OWNERSHIP_DETAIL
        // Preserve the exact existing guard and its short-circuit priority.
        g_trial_ownership_read = g_flash_fault ? TrialOwnershipRead::flash_fault : TrialOwnershipRead::flash_busy;
#endif
        return false;
    }
    Flash_clock_guard clock;
    uint8_t best_ch = 0xFFu;
    uint16_t best_seq = 0u;
    uint32_t best_slot = 0u;
    bool have = false;
#if BMCU_LITE_SETUP_TRIAL
    g_setup_journal_invalid = false;
#endif

#if BMCU_LITE_SETUP_TRIAL
    for (uint32_t slot = 0u; slot < STA_TOTAL_SLOTS; slot++) {
        Setup_sta_record record;
        if (!setup_sta_read(slot, record)) continue;
        // Check every equal-sequence payload, including older records hidden by a newer FF.
        // Half-range pairs have no defined order even if the channel content agrees.
        for (uint32_t previous = 0u; previous < slot; previous++) {
            Setup_sta_record earlier;
            if (!setup_sta_read(previous, earlier)) continue;
            const uint16_t delta = (uint16_t)(record.seq - earlier.seq);
            if ((delta == 0u && record.channel != earlier.channel) || delta == 0x8000u)
                g_setup_journal_invalid = true;
        }
        const uint16_t delta = (uint16_t)(record.seq - best_seq);
        if (!have || (delta != 0u && delta < 0x8000u)) {
            have = true;
            best_seq = record.seq;
            best_ch = record.channel;
            best_slot = slot;
        }
    }
    // Pairwise serial order can cycle. A retained winner must dominate every distinct
    // sequence; a tournament's last candidate alone does not establish a newest record.
    if (have) for (uint32_t slot = 0u; slot < STA_TOTAL_SLOTS; slot++) {
        Setup_sta_record record;
        if (!setup_sta_read(slot, record)) continue;
        const uint16_t age = (uint16_t)(best_seq - record.seq);
        if (age >= 0x8000u) g_setup_journal_invalid = true;
    }
    // Cache refusal along with taint. Neither a cached FF nor a subsequent consumer
    // can treat ambiguous/integrity-invalid storage as valid retained ownership.
    if (g_setup_journal_invalid) have = false;
#else
    for (uint32_t slot = 0u; slot < STA_TOTAL_SLOTS; slot++) {
        const uint32_t a = sta_slot_addr(slot);
        const uint32_t w0 = *(const volatile uint32_t*)(a + 0u);
        const uint32_t w1 = *(const volatile uint32_t*)(a + 4u);
#if BMCU_LITE_SETUP_TRIAL
        const bool blank = flash_word_is_blank(w0) && flash_word_is_blank(w1);
#endif
        if ((w0 >> 24) != STA_TAG) {
#if BMCU_LITE_SETUP_TRIAL
            if (!blank) g_setup_journal_invalid = true;
#endif
            continue;
        }
        if ((w0 ^ w1) != MAGIC_STA) {
#if BMCU_LITE_SETUP_TRIAL
            if (!blank) g_setup_journal_invalid = true;
#endif
            continue;
        }
        const uint16_t seq = (uint16_t)((w0 >> 8) & 0xFFFFu);
        const uint8_t ch = (uint8_t)(w0 & 0xFFu);
        if (ch >= 4u && ch != 0xFFu) {
#if BMCU_LITE_SETUP_TRIAL
            g_setup_journal_invalid = true;
#endif
            continue;
        }
        if (!have || (int16_t)(seq - best_seq) > 0) {
            have = true;
            best_seq = seq;
            best_ch = ch;
            best_slot = slot;
        }
    }

#endif

    g_sta_seq = have ? (uint16_t)(best_seq + 1u) : 0u;
    g_sta_saved_slot = (uint16_t)best_slot;
    g_sta_slot = have ? (uint16_t)((best_slot + 1u) % STA_TOTAL_SLOTS) : 0u;
    g_sta_have_saved = have;
    g_sta_saved_loaded = best_ch;
    g_sta_scanned = true;
    *loaded_ch = best_ch;
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL || BMCU_LITE_FULL_LOAD
#if BMCU_LITE_OWNERSHIP_DETAIL
    g_trial_ownership_read = have ? TrialOwnershipRead::valid : TrialOwnershipRead::no_valid_record;
#endif
    return have;
#else
    return true;
#endif
}

bool Flash_AMS_state_write(uint8_t loaded_ch)
{
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL && !BMCU_LITE_SETUP_TRIAL
    return false; // Read-only trial: no erase, program or journal submission.
#else
#if BMCU_LITE_SETUP_TRIAL
#if BMCU_LITE_FULL_LOAD
    if (setup_trial_phase()==SetupPhase::operation) {
        if (!full_load_journal_target(loaded_ch) || !full_load_ready()) return false;
    } else
#endif
    if (loaded_ch!=0xffu || !setup_trial_write_allowed(SetupWrite::journal)) return false;
#endif
    if (loaded_ch >= 4u && loaded_ch != 0xFFu) return false;
    if (g_flash_fault || !Flash_background_idle()) return false;
    if (g_sta_have_saved && g_sta_saved_loaded == loaded_ch) return true;
    if (!bus_background_ready()) return false;
    Flash_clock_guard clock;
    bool erase;
    const uint32_t slot = sta_find_slot(&erase);
    Flash_job &job = g_flash_job;
    job = {};
    job.kind = 2u;
    job.channel = loaded_ch;
    job.slot = (uint16_t)slot;
    job.addr = sta_slot_addr(slot);
    job.erase = erase;
    job.halfwords = 4u;
    job.words[0] = (STA_TAG << 24) | ((uint32_t)g_sta_seq << 8) | loaded_ch;
    job.words[1] = job.words[0] ^ MAGIC_STA;
    g_sta_slot = (uint16_t)((slot + 1u) % STA_TOTAL_SLOTS);
    return false;
#endif
}

struct alignas(4) Flash_CAL_payload
{
    float offs[4];
    float vmin[4];
    float vmax[4];
    float dm_none[4];
};

static bool cal_latest(Flash_CAL_payload* out, uint32_t* rsv, uint32_t* addr)
{
    bool have = false;
#if BMCU_LITE_SETUP_TRIAL
    g_setup_cal_ambiguous = false;
    Flash_CAL_payload selected;
#endif
    for (uint8_t i = 0u; i < 2u; i++) {
        const uint32_t page = i ? CAL_BACKUP_ADDR : FLASH_NVM_CAL_ADDR;
        Flash_CAL_payload p;
        uint16_t got = 0u;
        uint32_t candidate = 0u;
        if (!nvm256_read(page, MAGIC_CAL, 2u, &p, sizeof(p), &got, &candidate) || got != sizeof(p)) continue;
#if BMCU_LITE_SETUP_TRIAL
        const uint16_t delta = (uint16_t)((candidate >> 16) - (*rsv >> 16));
        if (have && (delta == 0x8000u || (delta == 0u &&
            ((uint16_t)candidate != (uint16_t)*rsv || memcmp(&p, &selected, sizeof(p)) != 0))))
            g_setup_cal_ambiguous = true;
        // Equal-sequence duplicates require identical payload bits and all lower flags.
        // The existing nvm256_read above remains the sole CAL integrity validator.
        if (!have || (delta != 0u && delta < 0x8000u)) {
            selected = p;
#else
        if (!have || (int16_t)((uint16_t)(candidate >> 16) - (uint16_t)(*rsv >> 16)) > 0) {
#endif
            if (out) *out = p;
            *rsv = candidate;
            *addr = page;
            have = true;
        }
    }
#if BMCU_LITE_SETUP_TRIAL
    if (g_setup_cal_ambiguous) return false;
#endif
    return have;
}

static bool cal_commit(const Flash_CAL_payload& p, uint32_t flags)
{
    if (g_flash_fault || !Flash_background_idle()) return false;
    uint32_t rsv = 0u, addr = CAL_BACKUP_ADDR;
    const bool have = cal_latest(nullptr, &rsv, &addr);
#if BMCU_LITE_SETUP_TRIAL
    if (g_setup_cal_ambiguous) return false;
#endif
    const uint16_t seq = have ? (uint16_t)((rsv >> 16) + 1u) : 0u;
    return nvm256_write(addr == FLASH_NVM_CAL_ADDR ? CAL_BACKUP_ADDR : FLASH_NVM_CAL_ADDR,
        MAGIC_CAL, 2u, ((uint32_t)seq << 16) | flags, &p, sizeof(p));
}

bool Flash_MC_PULL_cal_write_all(const float offs[4], const float vmin[4], const float vmax[4], const int8_t pol[4], const float dm_none[4], uint8_t valid_mask)
{
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL && !BMCU_LITE_SETUP_TRIAL
    return false; // Read-only trial: no erase, program or journal submission.
#else
#if BMCU_LITE_SETUP_TRIAL
    if (!setup_trial_write_allowed(SetupWrite::cal)) return false;
#endif
    Flash_CAL_payload p;
    memcpy(p.offs, offs, sizeof(p.offs));
    memcpy(p.vmin, vmin, sizeof(p.vmin));
    memcpy(p.vmax, vmax, sizeof(p.vmax));
    memcpy(p.dm_none, dm_none, sizeof(p.dm_none));

    uint32_t rsv = (uint32_t)(valid_mask & 0x0Fu) << 8;
    for (uint8_t ch = 0u; ch < 4u; ch++)
    {
        if (pol && pol[ch] < 0) rsv |= (1u << ch);
    }

    return cal_commit(p, rsv);
#endif
}

bool Flash_MC_PULL_cal_read(float offs[4], float vmin[4], float vmax[4], int8_t pol[4], float dm_none[4], uint8_t* valid_mask)
{
    Flash_CAL_payload p;
    uint32_t rsv = 0u, addr = 0u;
    if (!cal_latest(&p, &rsv, &addr)) return false;
    if (valid_mask) *valid_mask = (uint8_t)((rsv >> 8) & 0x0Fu);

    memcpy(offs, p.offs, sizeof(p.offs));
    memcpy(vmin, p.vmin, sizeof(p.vmin));
    memcpy(vmax, p.vmax, sizeof(p.vmax));
    memcpy(dm_none, p.dm_none, sizeof(p.dm_none));

    if (pol)
    {
        for (uint8_t ch = 0u; ch < 4u; ch++)
            pol[ch] = (rsv & (1u << ch)) ? -1 : 1;
    }

    return true;
}

bool Flash_MC_PULL_cal_clear(void)
{
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL || (BMCU_LITE_FULL_LOAD && !BMCU_LITE_V11_FEATURES)
    return false; // Read-only trial: no erase, program or journal submission.
#else
#if BMCU_LITE_V11_FEATURES
    if (!setup_trial_maintenance_clearing() || setup_trial_phase()!=SetupPhase::calibration) return false;
#endif
    const Flash_CAL_payload p = {};
    return cal_commit(p, 0u);
#endif
}

bool Flash_Motion_write(const void* in, uint16_t bytes)
{
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL && !BMCU_LITE_SETUP_TRIAL
    return false; // Read-only trial: no erase, program or journal submission.
#else
#if BMCU_LITE_SETUP_TRIAL
    if (!setup_trial_write_allowed(SetupWrite::motion)) return false;
#endif
    if (!in || bytes == 0u) return false;
    return nvm256_write(FLASH_NVM_MOTION_ADDR, MAGIC_MOT, VER_1, 0u, in, bytes);
#endif
}

bool Flash_Motion_read(void* out, uint16_t bytes)
{
    if (!out || bytes == 0u) return false;

    uint16_t got = 0u;
    if (!nvm256_read(FLASH_NVM_MOTION_ADDR, MAGIC_MOT, VER_1,
                     out, bytes, &got, nullptr))
        return false;

    return (got != 0u) && (got <= bytes);
}

bool Flash_Motion_clear(void)
{
#if BMCU_LITE_SINGLE_CHANNEL_TRIAL || (BMCU_LITE_FULL_LOAD && !BMCU_LITE_V11_FEATURES)
    return false; // Read-only trial: no erase, program or journal submission.
#else
#if BMCU_LITE_V11_FEATURES
    if (!setup_trial_maintenance_clearing() || setup_trial_phase()!=SetupPhase::direction) return false;
#endif
    return flash256_erase(FLASH_NVM_MOTION_ADDR);
#endif
}

#if BMCU_LITE_SETUP_TRIAL
bool Flash_setup_fault() {
#if BMCU_USB_CONTINUOUS_JOURNAL
    // Exactly one scheduler owner exists. Its diagnostic error is classified and
    // cleared on completion; it must not masquerade as an ownership/CAL failure.
    if(g_flash_job.kind==5u)return g_flash_fault;
#endif
    return g_flash_fault || (FLASH->STATR & FLASH_FLAG_WRPRTERR);
}
SetupStorage Flash_setup_probe() {
    SetupStorage s{}; s.owner=0xffu;
    s.fault=Flash_setup_fault(); s.busy=!Flash_background_idle();
    if (s.fault || s.busy) return s;
    g_sta_scanned=false; // Force the production integrity/sequence scan, not cached FF.
    s.journal_valid=Flash_AMS_state_read(&s.owner);
    s.journal_invalid=g_setup_journal_invalid;
    s.journal_erased=flash_range_is_erased(sta_page_addr(0),STA_PAGE_COUNT*FLASH_NVM256_PAGE_SIZE);
    s.cal_erased=true;
    for (uint8_t i=0;i<2;++i) {
        const uint32_t address=i?CAL_BACKUP_ADDR:FLASH_NVM_CAL_ADDR;
        const bool blank=flash_range_is_erased(address,FLASH_NVM256_PAGE_SIZE);
        s.cal_erased &= blank;
        if (!blank) {
            Flash_CAL_payload data; uint16_t got=0;
            const bool valid=nvm256_read(address,MAGIC_CAL,2u,&data,sizeof(data),&got,nullptr) && got==sizeof(data);
            s.cal_schema |= valid; s.cal_invalid |= !valid;
        }
    }
    uint32_t cal_rsv=0u,cal_addr=0u;
    (void)cal_latest(nullptr,&cal_rsv,&cal_addr);
    s.cal_invalid |= g_setup_cal_ambiguous;
    s.motion_erased=flash_range_is_erased(FLASH_NVM_MOTION_ADDR,FLASH_NVM256_PAGE_SIZE);
    alignas(4) uint8_t motion[24]; uint16_t got=0;
    s.motion_schema=nvm256_read(FLASH_NVM_MOTION_ADDR,MAGIC_MOT,VER_1,motion,sizeof(motion),&got,nullptr) && got==sizeof(motion);
    s.fault=Flash_setup_fault(); s.busy=!Flash_background_idle();
    return s;
}
#endif

#endif

#if BMCU_USB_FLIGHT_RECORDER
void Flash_log_read(void* out) {
    Flash_clock_guard clock;
    memcpy(out,(const void*)0x0800ee00u,512);
}
bool Flash_log_write(uint32_t address,uint32_t value,bool erase) {
    if (!log_address(address)) return false;
    return erase ? flash256_erase(address,true) : flash_word_prog_std(address,value,true);
}
#endif
