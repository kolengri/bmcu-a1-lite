#include "authority_observation.h"
#if BMCU_LITE_AUTHORITY_OBSERVATION
#include "hal/time_hw.h"
#include "hal/irq_wch.h"
#include "ams.h"
#include "app_api.h"
#include "_bus_hardware.h"
#include "crc_bus.h"
#include <string.h>
namespace authority_observation {
// Two fixed first facts distinguish pending cancellation from a later driven stop.
// Every producer and the publisher run in main; no IRQ performs an acquisition.
static uint8_t records[2][26],frame[80],history[8],origin[4];
static uint8_t history_count=0,selected_grant=0xff,mask=0,coverage=0;
static uint8_t phase_flags=0,pressure_flags=0,driving=0,drive_channel=0xff;
static uint16_t count=0;
static uint8_t max_gap=0,last_gap=0;
static uint32_t scale=0;
static uint64_t received=0,due=0;
static bool dirty=false;
static uint8_t post[16],post_count=0,post_flags=0,terminal_fault=0,terminal_slot=0xff;
static uint64_t terminal_at=0;
static unsigned cursor=0;
static uint8_t previous=0;
static constexpr uint8_t preamble[13]={0,7,0,7,0,1,2,3,4,5,6,7,0};
static constexpr uint16_t duration[13]={100,600,400,400,400,200,200,200,200,200,200,200,200};
static uint8_t age(uint64_t now){
    if(!count||!scale||time_hw_tpms!=scale||now<received){coverage|=8;return 255;}
    const uint64_t ms=(now-received)/scale;
    if(ms>254){coverage|=32;return 254;}
    return uint8_t(ms);
}
void wire(uint8_t command,uint8_t ch,uint8_t state,uint8_t motion){
    // Validated *handled* frames, not an electrical arrival trace. This hook runs
    // once per consumed frame, before online/admission rejection and never grants motion permission.
#if !BMCU_USB_FLIGHT_RECORDER
    if(terminal_fault && full_load_fault()!=FullLoadFault::none && state==3u && motion==0xbfu){
        if(post_count<4){
            const uint64_t now=time_ticks64();uint16_t elapsed=65535;
            if(scale&&time_hw_tpms==scale&&now>=terminal_at){
                const uint64_t ms=(now-terminal_at)/scale;
                elapsed=ms>65534?65534:uint16_t(ms);if(ms>65534)post_flags|=4;
            }else post_flags|=2;
            uint8_t* p=post+4u*post_count++;p[0]=command;p[1]=ch;p[2]=uint8_t(elapsed);p[3]=uint8_t(elapsed>>8);dirty=true;
        }else if(!(post_flags&1)){post_flags|=1;dirty=true;}
    }
 #endif
#if BMCU_LITE_NATIVE_PHASE_COMPAT
    if(terminal_fault)return; // Keep handled tuples after the frozen first driven record.
#else
    if(mask&2u)return;
#endif
    memcpy(history+4,history,4);history[0]=command;history[1]=ch;history[2]=state;history[3]=motion;
    if(history_count<2)++history_count;
}
void grant(uint8_t ch,uint64_t at,bool fresh_origin){
    if(mask&2u)return;
    if(count){last_gap=age(at);if(last_gap>max_gap)max_gap=last_gap;}
    if(count!=65535)++count;else coverage|=16;
    if(fresh_origin||count==1){memcpy(origin,history,4);phase_flags=0;pressure_flags=0;}
    selected_grant=ch;scale=time_hw_tpms;received=at;
}
void deferred(){if(!(mask&2u))phase_flags|=4;}
void stop(uint8_t reason,uint8_t fault){
    if(!count&&reason!=authority_observation::fault&&reason!=authority)return;
    const uint8_t slot=coverage&2u?1u:0u;
    if((mask&(1u<<slot))&&!fault)return;
    if(!scale)scale=time_hw_tpms;
    const uint64_t now=time_ticks64();
    if(fault&&!terminal_fault){terminal_fault=fault;terminal_slot=mask&(1u<<slot)?2u:slot;terminal_at=now;dirty=true;}
    if(mask&(1u<<slot))return;
    const uint8_t elapsed=age(now);const uint32_t saved=irq_save_wch();uint8_t* p=records[slot];
    p[0]=coverage|uint8_t(history_count>0?64:0)|uint8_t(history_count>1?128:0);
    p[1]=reason;p[2]=fault;p[3]=ams[0].now_filament_num;p[4]=full_load_owner();p[5]=selected_grant;
    p[6]=uint8_t(count);p[7]=uint8_t(count>>8);p[8]=elapsed;p[9]=max_gap;p[10]=last_gap;
    p[11]=phase_flags|uint8_t(full_load_journal_pending()?1:0)|uint8_t(full_load_durable(p[3])?2:0);
    memcpy(p+12,history,8);memcpy(p+20,origin,4);p[24]=pressure_flags;p[25]=drive_channel;
    mask|=uint8_t(1u<<slot);dirty=true;irq_restore_wch(saved);
}
void pressure(uint8_t kind,uint8_t ch){
    if(mask&2u||!count||ams[0].now_filament_num!=ch||ams[0].filament[ch].motion!=_filament_motion::send_out)return;
    if(kind==0)pressure_flags|=1;
    else{if(kind==hard)pressure_flags|=2;stop(kind);}
}
void pwm(uint8_t ch,int value){
    if(mask&2u||!count||ams[0].now_filament_num!=ch||ams[0].filament[ch].motion!=_filament_motion::send_out)return;
    const bool prior=driving&(1u<<ch);
    if(value){driving|=uint8_t(1u<<ch);coverage|=2;drive_channel=ch;}
    else{driving&=uint8_t(~(1u<<ch));if(prior)stop(applied_zero);}
}
#if !BMCU_USB_FLIGHT_RECORDER
static void color(uint8_t c){
    for(auto &led:RGBOUT)for(uint8_t i=0;i<2;++i)led.set_RGB(c&1?32:0,c&2?32:0,c&4?32:0,i);
}
void render(){
    if(!mask||time_hw_slow||!scale||time_hw_tpms!=scale||!bus_background_ready())return;
    const uint64_t now=time_ticks64();
    if(dirty){
        const uint32_t saved=irq_save_wch();frame[0]='A';frame[1]='O';frame[2]=4;frame[3]=80;
        frame[4]=mask;frame[5]=0;frame[6]=terminal_fault;frame[7]=terminal_slot;
        for(uint8_t i=0;i<2;++i){if(mask&(1u<<i))memcpy(frame+8+26u*i,records[i],26);else memset(frame+8+26u*i,255,26);}
        frame[60]=post_count;frame[61]=post_flags;memcpy(frame+62,post,16);irq_restore_wch(saved);
        const uint16_t crc=bus_crc16(frame,78);frame[78]=uint8_t(crc);frame[79]=uint8_t(crc>>8);
        dirty=false;cursor=0;
    }
    if(!cursor)due=now;
    if(now<due)return;
    if(now-due>uint64_t(scale)*30u){post_flags|=8;dirty=true;cursor=0;return;}
    for(const auto &led:RGBOUT)if(led.is_dirty()){post_flags|=8;dirty=true;cursor=0;return;}
    if(cursor<13){color(preamble[cursor]);due+=uint64_t(scale)*duration[cursor];++cursor;return;}
    if(cursor==13)previous=0;
    const unsigned pos=cursor-13;
    if(pos==240){color(uint8_t((previous+1u)&7u));due+=uint64_t(scale)*100u;++cursor;return;}
    if(pos==241){cursor=0;return;}
    const uint8_t b=frame[pos/3];const unsigned digit=pos%3;
    const uint8_t d=digit==0?uint8_t(b/49):digit==1?uint8_t((b/7)%7):uint8_t(b%7);
    previous=uint8_t((previous+1u+d)&7u);color(previous);due+=uint64_t(scale)*100u;++cursor;
}
#else
void snapshot(uint8_t* out){
    out[0]='A';out[1]='O';out[2]=4;out[3]=80;out[4]=mask;out[5]=0;out[6]=terminal_fault;out[7]=terminal_slot;
    for(unsigned i=0;i<2;++i){if(mask&(1u<<i))memcpy(out+8+26*i,records[i],26);else memset(out+8+26*i,255,26);}
#if BMCU_LITE_NATIVE_PHASE_COMPAT
    out[60]=history_count;out[61]=0xa1u; // Explicit USB first-terminal history marker.
    memcpy(out+62,history,8);memset(out+70,0,8);
#else
    out[60]=post_count;out[61]=post_flags;memcpy(out+62,post,16);
#endif
    const uint16_t value=bus_crc16(out,78);out[78]=uint8_t(value);out[79]=uint8_t(value>>8);
}
void render(){}
#endif

}
#endif
