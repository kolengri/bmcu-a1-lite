#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstdio>

#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #x); std::abort(); } } while (0)
struct GPIO_TypeDef { uint32_t BCR=0, BSHR=0, CFGLR=0, CFGHR=0; };
static GPIO_TypeDef host_gpio_ports[4];
#define GPIOA (&host_gpio_ports[0])
#define GPIOB (&host_gpio_ports[1])
#define GPIOC (&host_gpio_ports[2])
#define GPIOD (&host_gpio_ports[3])
// Raw RCC boundary: reset flags are read-only except RMVF, which clears them.
struct HostResetStatus {
    uint32_t value=0x0c000000u;
    operator uint32_t() const {return value;}
    void operator|=(uint32_t bits) {if(bits&(1u<<24))value&=0x03ffffffu;}
};
static struct {HostResetStatus RSTSCKR;uint32_t CFGR0=0x003c9408u;} host_rcc;
static struct {uint32_t EXTEN_CTR=0x10u;} host_exten;
#define EXTEN (&host_exten)
static uint32_t SystemCoreClock=144000000u;
static uint32_t host_systick_ctlr=9u;
#define STK_CTLR host_systick_ctlr
#define RCC (&host_rcc)
enum {
    GPIO_Pin_0=1, GPIO_Pin_1=2, GPIO_Pin_2=4, GPIO_Pin_3=8, GPIO_Pin_4=16,
    GPIO_Pin_5=32, GPIO_Pin_6=64, GPIO_Pin_7=128, GPIO_Pin_8=256,
    GPIO_Pin_9=512, GPIO_Pin_10=1024, GPIO_Pin_11=2048, GPIO_Pin_12=4096, GPIO_Pin_13=8192, GPIO_Pin_14=16384, GPIO_Pin_15=32768,
    ENABLE=1, DISABLE=0, GPIO_Mode_AF_PP=1, GPIO_Speed_50MHz=1,
    RCC_APB2Periph_GPIOA=1, RCC_APB2Periph_GPIOB=2, RCC_APB2Periph_AFIO=4, RCC_APB2Periph_GPIOC=8, RCC_APB2Periph_GPIOD=16,
    RCC_APB1Periph_TIM2=1, RCC_APB1Periph_TIM3=2, RCC_APB1Periph_TIM4=4,
    RCC_APB1Periph_WWDG=8, NVIC_PriorityGroup_1=1, GPIO_Remap_PD01=1,
    GPIO_FullRemap_TIM2=2, GPIO_PartialRemap_TIM3=3, GPIO_Remap_TIM4=4,
    TIM_CounterMode_Up=1, TIM_OCMode_PWM1=1, TIM_OutputState_Enable=1,
    TIM_OCPolarity_High=1, TIM_OCPreload_Enable=1,
    TIM2=0, TIM3=1, TIM4=2
};
struct GPIO_InitTypeDef { int GPIO_Pin, GPIO_Mode, GPIO_Speed; };
struct TIM_TimeBaseInitTypeDef { int TIM_Period, TIM_Prescaler, TIM_ClockDivision, TIM_CounterMode; };
struct TIM_OCInitTypeDef { int TIM_OCMode, TIM_OutputState, TIM_Pulse, TIM_OCPolarity; };
static int compare_values[3][4];
static unsigned pwm_initializations;
static void (*host_compare_hook)(int,int,uint16_t)=nullptr;
static void (*host_delay_hook)(uint32_t)=nullptr;
static void (*host_time_hook)()=nullptr;
static unsigned host_analog_configs=0;
inline void GPIO_Init(GPIO_TypeDef* p, GPIO_InitTypeDef* g) {if(g->GPIO_Mode==4){CHECK(p==GPIOA&&g->GPIO_Pin==255);++host_analog_configs;}}
inline void RCC_APB2PeriphClockCmd(int, int) {}
inline void RCC_APB1PeriphClockCmd(int, int) {}
inline void GPIO_PinRemapConfig(int, int) {}
inline void TIM_TimeBaseInit(int, TIM_TimeBaseInitTypeDef*) {}
#define OC_INIT(n) inline void TIM_OC##n##Init(int t, TIM_OCInitTypeDef* p) { CHECK(p->TIM_Pulse == 0); compare_values[t][n-1]=p->TIM_Pulse; ++pwm_initializations; }
OC_INIT(1) OC_INIT(2) OC_INIT(3) OC_INIT(4)
#define OC_PRELOAD(n) inline void TIM_OC##n##PreloadConfig(int, int) {}
OC_PRELOAD(1) OC_PRELOAD(2) OC_PRELOAD(3) OC_PRELOAD(4)
#define COMPARE(n) inline void TIM_SetCompare##n(int t, uint16_t v) { CHECK(v <= 1000); if(host_compare_hook)host_compare_hook(t,n-1,v); compare_values[t][n-1]=v; }
COMPARE(1) COMPARE(2) COMPARE(3) COMPARE(4)
inline void TIM_CtrlPWMOutputs(int t, int) {
    for (int c=0; c<(t==TIM4 ? 4 : 2); ++c) CHECK(compare_values[t][c]==0);
}
inline void TIM_ARRPreloadConfig(int, int) {}
inline void TIM_Cmd(int, int) {}
inline void SystemInit() {}
inline void SystemCoreClockUpdate() {}
inline void __enable_irq() {}
inline void WWDG_DeInit() {}
inline void NVIC_PriorityGroupConfig(int) {}

struct LoopComplete {};
struct TerminalCapture {};
static bool host_terminal_seen=false;
static unsigned host_clock_calls_before_exit;
static uint64_t host_clock = 100;
static uint32_t time_hw_tpms = 1;
inline uint32_t time_ticks32() { if(host_time_hook)host_time_hook(); return host_clock; }
inline uint32_t ms_to_ticks32(uint32_t ms) { return ms * time_hw_tpms; }
inline int32_t time_diff32(uint32_t a, uint32_t b) { return int32_t(a-b); }
static bool host_irq_enabled=true;
static void (*host_irq_restore_hook)()=nullptr;
inline uint32_t irq_save_wch() {const uint32_t saved=host_irq_enabled?0x88u:0u;host_irq_enabled=false;return saved;}
inline void irq_restore_wch(uint32_t saved) {
    host_irq_enabled=(saved&0x88u)!=0;
    if(host_irq_enabled && host_irq_restore_hook){auto hook=host_irq_restore_hook;host_irq_restore_hook=nullptr;hook();}
}
inline void time_hw_init() {}
inline void delay_us(uint32_t us) { host_clock+=us/1000; }
inline void delay(uint32_t ms) {for(uint32_t i=0;i<ms;++i){host_clock+=time_hw_tpms;if(host_delay_hook)host_delay_hook(1);}}

static uint32_t time_hw_tpus = 1;
inline uint64_t time_ticks64() {
    if(host_time_hook)host_time_hook();
    if (host_clock_calls_before_exit && --host_clock_calls_before_exit==0) throw LoopComplete{};
    return host_clock;
}
inline uint32_t time_hw_ticks_per_ms() { return time_hw_tpms; }
inline void NVIC_SystemReset() { CHECK(false); }

inline uint32_t us_to_ticks32(uint32_t us) { return us * time_hw_tpus; }
inline bool time_reached32(uint32_t now,uint32_t deadline) { return int32_t(now-deadline)>=0; }

// MMIO-only substitutes. Transport/parser/ISR control flow comes from production.
enum {
    GPIO_Mode_IPU=2, GPIO_Mode_Out_PP=3, GPIO_Mode_Out_OD=5, RCC_APB2Periph_USART1=32,
    RCC_AHBPeriph_CRC=1, RCC_AHBPeriph_DMA1=2,
    USART_WordLength_9b=1, USART_StopBits_1=1, USART_Parity_Even=1,
    USART_HardwareFlowControl_None=0, USART_Mode_Rx=4, USART_Mode_Tx=8,
    USART_IT_TC=0x40, USART_IT_PE=0x100, USART_IT_ERR=0x200,
    USART_FLAG_TC=0x40, USART_FLAG_RXNE=0x20, USART_DMAReq_Tx=0x80, USART_DMAReq_Rx=0x40,
    USART1_IRQn=1, DMA1_Channel4_IRQn=2, DMA1_Channel5_IRQn=3,
    DMA_DIR_PeripheralDST=1, DMA_DIR_PeripheralSRC=0,
    DMA_Mode_Normal=0, DMA_Mode_Circular=1, DMA_PeripheralInc_Disable=0,
    DMA_MemoryInc_Enable=1, DMA_Priority_VeryHigh=3, DMA_M2M_Disable=0,
    DMA_MemoryDataSize_Byte=1, DMA_PeripheralDataSize_Byte=1,
    DMA_IT_TE=8, DMA_IT_TC=2, DMA_CFGR1_EN=1,
    DMA1_FLAG_GL4=0x1000, DMA1_FLAG_TE4=0x8000,
    DMA1_FLAG_GL5=0x10000, DMA1_FLAG_TC5=0x20000, DMA1_FLAG_TE5=0x80000
};
struct MockDMA {
    volatile uint32_t INTFR=0;
    struct Clear {
        volatile uint32_t* flags;
        void operator=(uint32_t mask) {
            if(mask & 1u) mask|=0xf;
            if(mask & DMA1_FLAG_GL4) mask|=0xf000;
            if(mask & DMA1_FLAG_GL5) mask|=0xf0000;
            *flags &= ~mask;
        }
    } INTFCR{&INTFR};
};
struct MockDMAChannel { uintptr_t MADDR=0; volatile uint32_t CFGR=0,CNTR=0; };
struct MockUSART { volatile uint16_t BRR=0,CTLR1=0,CTLR2=0,CTLR3=0,STATR=0,DATAR=0; };
static MockDMA host_dma;
static MockDMAChannel host_dma4,host_dma5;
static MockUSART host_usart;
#define DMA1 (&host_dma)
#define DMA1_Channel4 (&host_dma4)
#define DMA1_Channel5 (&host_dma5)
#define USART1 (&host_usart)
struct DMA_InitTypeDef {
    uintptr_t DMA_PeripheralBaseAddr=0,DMA_MemoryBaseAddr=0;
    uint32_t DMA_DIR=0,DMA_Mode=0,DMA_PeripheralInc=0,DMA_MemoryInc=0,DMA_Priority=0,
        DMA_M2M=0,DMA_MemoryDataSize=0,DMA_PeripheralDataSize=0,DMA_BufferSize=0;
};
struct USART_InitTypeDef { uint32_t USART_BaudRate,USART_WordLength,USART_StopBits,USART_Parity,USART_HardwareFlowControl,USART_Mode; };
struct NVIC_InitTypeDef { uint32_t NVIC_IRQChannel,NVIC_IRQChannelPreemptionPriority,NVIC_IRQChannelSubPriority,NVIC_IRQChannelCmd; };
inline void RCC_AHBPeriphClockCmd(int,int) {}
inline void USART_DeInit(MockUSART* u) { u->BRR=u->CTLR1=u->CTLR2=u->CTLR3=0; }
inline void USART_Init(MockUSART* u,USART_InitTypeDef* c) { u->CTLR1=c->USART_Mode; }
inline void USART_Cmd(MockUSART*,int) {}
inline void USART_ITConfig(MockUSART*,int,int) {}
inline void USART_DMACmd(MockUSART* u,int mask,int enable) { if(enable)u->CTLR3|=mask;else u->CTLR3&=~mask; }
inline void USART_ClearITPendingBit(MockUSART* u,int flag) { u->STATR&=~flag; }
inline void NVIC_Init(NVIC_InitTypeDef*) {}
inline void DMA_Init(MockDMAChannel* c,DMA_InitTypeDef* p) { c->CNTR=p->DMA_BufferSize;c->MADDR=p->DMA_MemoryBaseAddr; }
inline void DMA_ClearFlag(uint32_t flag) { DMA1->INTFCR=flag; }
inline void DMA_ITConfig(MockDMAChannel*,int,int) {}
inline void DMA_Cmd(MockDMAChannel* c,int enable) { if(enable)c->CFGR|=1;else c->CFGR&=~1u; }

// Free-running physical STK boundary for the unchanged WS2812 bit-loop body.
struct HostSTK {uint32_t value=0;operator uint32_t(){return ++value;}};
static HostSTK host_stk;
#define STK_CNTL (uint32_t(host_stk))
inline uint32_t time_hw_ticks_per_us(){return 18;}
inline void delayTicks32(uint32_t ticks){host_stk.value+=ticks;}

// ADC hardware boundary only; actual ADC_DMA.cpp owns all lifecycle/filter/publication logic.
enum {GPIO_Mode_AIN=4,RCC_APB2Periph_ADC1=64,RCC_APB2Periph_ADC2=128,RCC_PCLK2_Div4=4,
DMA1_FLAG_GL1=1,DMA1_FLAG_TC1=2,DMA1_FLAG_HT1=4,DMA1_FLAG_TE1=8,DMA_IT_HT=4,
DMA_PeripheralDataSize_Word=4,DMA_MemoryDataSize_Word=4,DMA_Priority_High=2,
ADC_Mode_RegSimult=1,ADC_Mode_Independent=2,ADC_ExternalTrigConv_None=0,ADC_DataAlign_Right=0,
ADC_OutputBuffer_Disable=0,ADC_Pga_1=1,ADC_SampleTime_71Cycles5=71};
static MockDMAChannel host_dma1;
#define DMA1_Channel1 (&host_dma1)
inline void DMA_DeInit(MockDMAChannel*){}
inline void DMA_SetCurrDataCounter(MockDMAChannel*p,uint32_t n){p->CNTR=n;}
struct ADC_TypeDef {bool started=false;unsigned rank_count=0;};
static ADC_TypeDef host_adc1,host_adc2;
#define ADC1 (&host_adc1)
#define ADC2 (&host_adc2)
static uint64_t host_adc_start=0;
struct ADC_InitTypeDef {int ADC_Mode=0,ADC_ScanConvMode=0,ADC_ContinuousConvMode=0,ADC_ExternalTrigConv=0,ADC_DataAlign=0,ADC_NbrOfChannel=0,ADC_OutputBuffer=0,ADC_Pga=0;};
inline void RCC_ADCCLKConfig(int n){CHECK(n==RCC_PCLK2_Div4);}
inline void ADC_DeInit(ADC_TypeDef*){}
inline void ADC_Init(ADC_TypeDef*,ADC_InitTypeDef*p){CHECK(p->ADC_NbrOfChannel==8);}
inline void ADC_RegularChannelConfig(ADC_TypeDef*p,uint8_t c,uint8_t r,int){CHECK(c==p->rank_count&&r==c+1);++p->rank_count;}
inline void ADC_Cmd(ADC_TypeDef*,int){}inline void ADC_BufferCmd(ADC_TypeDef*,int){}
inline void ADC_ResetCalibration(ADC_TypeDef*){}inline bool ADC_GetResetCalibrationStatus(ADC_TypeDef*){return false;}
inline void ADC_StartCalibration(ADC_TypeDef*){}inline bool ADC_GetCalibrationStatus(ADC_TypeDef*){return false;}
inline void ADC_DMACmd(ADC_TypeDef*,int){}
inline void ADC_SoftwareStartConvCmd(ADC_TypeDef*p,int on){p->started=on;if(host_adc1.started&&host_adc2.started&&!host_adc_start)host_adc_start=host_clock;}

