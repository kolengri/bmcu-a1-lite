#pragma once
#include <host_hal.h>
#include <cstring>
// Peripheral/RAM boundary only. The production journal/cache/reader functions
// are compiled unchanged except translating physical flash load addresses here.
static uint32_t time_hw_slow=0;
inline void time_hw_flash_clock(uint32_t slow) { time_hw_slow=slow; }
alignas(4) static uint8_t host_nvm[4096];
alignas(4) static uint8_t host_log[512];
alignas(4) static uint8_t host_marker[256];
static unsigned marker_programs=0,marker_erases=0,marker_export_enables=0;
static bool marker_erase_fail=false,marker_verify_fail=false;
inline void host_export_de_high(){
#if BMCU_USB_ISP_EXPORT
    for(unsigned i=0;i<256;i+=4){uint32_t v;memcpy(&v,host_marker+i,4);CHECK(v==0xffffffffu||v==0xe339e339u);}
    for(const auto& row:compare_values)for(auto v:row)CHECK(v==0);
    ++marker_export_enables;
#endif
    GPIOA->BSHR=GPIO_Pin_12;
}
static unsigned log_programs=0,log_erases=0;
static int log_cut=-1;


static unsigned host_programs=0,host_erases=0;
static int host_powercut=-1;
static bool host_fail_program=false,host_hang=false;
struct PowerCut {};
static void log_step() {if(log_cut==0)throw PowerCut{};if(log_cut>0)--log_cut;}
static uint8_t* host_flash_ptr(uint32_t address) {
    if(address>=0x0800ed00u&&address<0x0800ee00u){CHECK(!(address&1));return host_marker+(address-0x0800ed00u);}
    if(address>=0x0800ee00u&&address<0x0800f000u){CHECK(!(address&1));return host_log+(address-0x0800ee00u);}
    CHECK(address>=0x0800f000u && address<=0x0800fffeu && !(address&1u));
    return host_nvm+(address-0x0800f000u);
}
enum { FLASH_FLAG_BSY=1,FLASH_FLAG_EOP=4,FLASH_FLAG_WRPRTERR=16, FLASH_Access_SYSTEM_HALF=0 };
struct HostFlash {
    uint32_t STATR=0,ADDR=0;
    struct Control {
        HostFlash* f; uint32_t value=0;
        void operator|=(uint32_t v) {
            value|=v;
            if ((v&0x40u) && (value&0x20000u)) {
                if(f->ADDR==0x0800ed00u){++marker_erases;CHECK(time_hw_slow==1u);for(const auto& row:compare_values)for(auto v:row)CHECK(v==0);CHECK(!(USART1->CTLR1&USART_Mode_Tx)&&!(DMA1_Channel4->CFGR&DMA_CFGR1_EN)&&!(DMA1_Channel5->CFGR&DMA_CFGR1_EN));}
                else if(f->ADDR<0x0800f000u){++log_erases;CHECK(f->ADDR==0x0800ee00u||f->ADDR==0x0800ef00u);}
                else ++host_erases;
                CHECK(f->ADDR>=0x0800ed00u && f->ADDR<=0x0800ff00u);
                if(f->ADDR==0x0800ed00u&&marker_erase_fail){f->STATR|=FLASH_FLAG_WRPRTERR;return;}
                if(host_hang) f->STATR|=FLASH_FLAG_BSY;
                else if(f->ADDR<0x0800f000u){
                    for(unsigned n=0;n<256;n+=4){log_step();std::memset(host_flash_ptr(f->ADDR+n),0xff,4);}log_step();
                    if(f->ADDR==0x0800ed00u&&marker_verify_fail)host_marker[200]=0;
                }else std::memset(host_flash_ptr(f->ADDR),0xff,256);
            }
        }
        void operator&=(uint32_t v) {value&=v;}
        operator uint32_t() const {return value;}
    } CTLR{this};
};
static HostFlash host_flash;
#define FLASH (&host_flash)
inline void FLASH_Unlock_Fast() {}
inline void FLASH_Unlock() {}
inline void FLASH_Lock_Fast() {}
inline void FLASH_Lock() {}
inline void FLASH_ClearFlag(uint32_t v) { FLASH->STATR&=~v; }
inline void FLASH_Access_Clock_Cfg(uint32_t) {}
inline void host_store16(uint32_t address,uint16_t data) {
    if(address<0x0800f000u)log_step();
    if(address>=0x0800f000u&&host_powercut==0)throw PowerCut{};
    if(address>=0x0800f000u&&host_powercut>0)--host_powercut;
    if(address<0x0800ee00u)++marker_programs;else if(address<0x0800f000u)++log_programs;else ++host_programs;
    if(host_hang){FLASH->STATR|=FLASH_FLAG_BSY;return;}
    if(host_fail_program){FLASH->STATR|=FLASH_FLAG_WRPRTERR;return;}
    auto* p=host_flash_ptr(address);
    p[0]&=uint8_t(data);if(address<0x0800f000u)log_step();p[1]&=uint8_t(data>>8);if(address<0x0800f000u)log_step();
}
// The hardware CRC peripheral uses the STM32/WCH polynomial and word writes.
// This models MMIO, not the journal validation or scan/cache logic.
struct HostCRC {
    uint32_t value=0xffffffffu;
    struct Control { uint32_t* value;void operator=(uint32_t v) {if(v&1u)*value=0xffffffffu;} } CTLR{&value};
    struct Data {
        uint32_t* value;
        void operator=(uint32_t v) {
            *value^=v;
            for(int bit=0;bit<32;++bit)*value=(*value<<1)^((*value&0x80000000u)?0x04c11db7u:0u);
        }
        operator uint32_t() const {return *value;}
    } DATAR{&value};
};
static HostCRC host_crc;
#define CRC (&host_crc)
