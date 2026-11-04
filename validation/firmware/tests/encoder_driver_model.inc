// Only SDA samples and the transaction boundary are modeled. Driver ACK/data
// loops, raw-range rejection, online/magnet state and odometry remain actual.
static unsigned encoder_angle_reads=0,encoder_status_reads=0;
static unsigned encoder_fault_mask=0,encoder_fault_remaining=0,encoder_status_fault=0;
static bool encoder_bad_raw=false;
static uint16_t encoder_wire[4];
static int encoder_forced_raw=-1;
static unsigned encoder_fault_stage=0;
static bool encoder_event_trace=false;
static std::vector<uint32_t> encoder_events;
static void host_i2c_event(unsigned event,unsigned argument){if(encoder_event_trace)encoder_events.push_back((event<<24)|argument);}
static unsigned encoder_bit[4],encoder_ack[4],encoder_active_fault=0,encoder_width=0;
static uint64_t encoder_previous_ms=0;
static void host_i2c_begin(uint8_t reg,unsigned width){
    CHECK((reg==0x0cu&&width==16u)||(reg==0x0bu&&width==8u));
    encoder_width=width;
    for(unsigned ch=0;ch<4;++ch){encoder_bit[ch]=0;encoder_ack[ch]=0;}
    if(width==8u){
        ++encoder_status_reads;encoder_active_fault=encoder_status_fault;
        for(auto& w:encoder_wire)w=0x20u;
        return;
    }
    ++encoder_angle_reads;host_input_detail_read(6);input();
    const uint64_t ms=host_clock/1000u,dt=ms-encoder_previous_ms;encoder_previous_ms=ms;
    encoder_active_fault=encoder_fault_remaining?encoder_fault_mask:0u;
    if(encoder_fault_remaining)--encoder_fault_remaining;
    for(unsigned ch=0;ch<4;++ch){
        const unsigned t=ch==3?TIM2:ch==2?TIM3:TIM4,c=ch==0?2:0;
        uint16_t raw=encoder_forced_raw<0?MC_AS5600.raw_angle[ch]:uint16_t(encoder_forced_raw);
        if(!freeze_encoder&&compare_values[t][c]!=compare_values[t][c+1])raw=(raw+int(dt)*20*(ch%2?-1:1))&4095u;
        encoder_wire[ch]=(encoder_bad_raw&&(encoder_active_fault&(1u<<ch)))?4096u:raw;
    }
}
static bool host_i2c_sample(unsigned ch,bool ack){
    CHECK(ch<4u);
    if(ack){
        const unsigned stage=encoder_ack[ch]++;
        return (encoder_width==8u||!encoder_bad_raw)&&stage==encoder_fault_stage&&(encoder_active_fault&(1u<<ch));
    }
    const unsigned bit=encoder_bit[ch]++;
    CHECK(bit<encoder_width);
    return (encoder_wire[ch]>>(encoder_width-1u-bit))&1u;
}
#include "many_soft_AS5600.cpp"
