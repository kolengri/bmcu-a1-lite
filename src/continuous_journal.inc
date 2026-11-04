// Two independently committed pages. All addresses remain below original NVM.
// No allocation, diagnostic fault -> control mutation, or runtime synchronous flash loop.
static constexpr uint32_t jmagic=0x324a4642u,jpermit=0x6a51ffffu,jconsumed=0x31534e43u;
static constexpr unsigned jheader=32u,jrecords=14u,jqueue_size=8u;
struct JournalEvent {uint32_t ms,value,meta,crc;};
static_assert(sizeof(JournalEvent)==16,"journal wire record");
static JournalEvent jq[jqueue_size];
static uint8_t jqhead=0,jqcount=0,jpage=0,jslot=0,jword=0,jstate=0;
static uint16_t jlost=0;
static uint32_t jgeneration=0,jsession=0,jlast_heartbeat=0;
static uint32_t jrequest=0,jlast_rx=0;
// Per requested slot, not last polled slot. Equal03/04 semantics coalesce;
// the emitted event still records whichever command actually submitted it.
static uint32_t jlast_tuple[5]={};
static uint8_t jrequest_channel=255,jrequest_command=0;
static bool jrefusal_pending=false;
static uint8_t jtuple_valid=0;
static bool jwaiting=false,jfailed=false;
alignas(4) static uint8_t jbuffer[32];
static uint32_t jget(const void* p){uint32_t v;memcpy(&v,p,4);return v;}
static void jput(void* p,uint32_t v){memcpy(p,&v,4);}
static uint32_t jcrc(const void* data,unsigned length,uint32_t seed=0xffffffffu){
    const uint8_t* p=(const uint8_t*)data;uint32_t c=seed;
    for(unsigned n=0;n<length;++n){c^=p[n];for(unsigned b=0;b<8;++b)c=(c>>1)^((c&1u)?0xedb88320u:0u);}
    return ~c;
}
static uint32_t jmillis(){return time_hw_tpms?uint32_t(time_ticks64()/time_hw_tpms):0;}
static bool jblank(uint32_t v){return v==0xffffffffu||v==0xe339e339u;}
static bool jvalid(const uint8_t* p){
    return jget(p)==jmagic && jget(p+12)==build && jget(p+4)>0u && jget(p+4)<0xffffffffu &&
        jget(p+8)>0u && jget(p+8)<0xffffffffu && jget(p+16)==jcrc(p,16);
}
// Equal/ambiguous generations inhibit execution and rearm rather than picking arbitrarily.
static int jnewest(const uint8_t* raw){
    bool a=jvalid(raw),b=jvalid(raw+256);if(!a)return b?1:-1;if(!b)return 0;
    uint32_t delta=jget(raw+4)-jget(raw+260);
    if(!delta||delta==0x80000000u)return -2;
    return delta<0x80000000u?0:1;
}
static bool jevent_at(uint8_t kind,uint8_t ch,uint32_t value,uint32_t now){
    if(!capturing||jfailed)return false;
    const uint32_t irq=irq_save_wch();
    if(jqcount==jqueue_size){if(jlost<65535u)++jlost;irq_restore_wch(irq);return false;}
    jq[(jqhead+jqcount)%jqueue_size]={now,value,uint32_t(kind)|(uint32_t(ch)<<8)|(uint32_t(jlost)<<16),0};
    jlost=0;++jqcount;irq_restore_wch(irq);return true;
}
static bool jevent(uint8_t kind,uint8_t ch,uint32_t value){return jevent_at(kind,ch,value,jmillis());}
static void jtxevent(const uint8_t* data){
    if(!capturing||jfailed)return;
    const unsigned sender=data[4]-3u;const uint8_t* feedback=data+(sender==0?7u:19u);
    const uint32_t tuple=jrequest|(uint32_t(feedback[0])<<16)|(uint32_t(feedback[1])<<24);
    const uint32_t now=jmillis(),meter=jget(feedback+2);
    const unsigned key=jrequest_channel<4u?jrequest_channel:jrequest_channel==255u?4u:5u;
    const bool transition=key==5u||!(jtuple_valid&(1u<<key))||tuple!=jlast_tuple[key];
    if(transition){
        const uint32_t irq=irq_save_wch();
        // Reserve both entries: a retained TX always has its adjacent exact wire meter,
        // unless power is cut before the second independent CRC commits.
        if(jqcount>jqueue_size-2u){jlost=jlost>65533u?65535u:uint16_t(jlost+2u);}
        else {
            // One sample identifies both records, including a millisecond boundary.
            jevent_at(uint8_t(16u+sender),jrequest_channel,tuple,now);
            jevent_at(18,feedback[1],meter,now);
            if(key<5u){jtuple_valid|=1u<<key;jlast_tuple[key]=tuple;}
        }
        irq_restore_wch(irq);
    }
}
void journal_fault(uint8_t fault){jevent(12,ams[0].now_filament_num,uint32_t(fault)|(uint32_t(full_load_owner())<<8));}
static void jmake_header(uint32_t generation,uint32_t run){
    memset(jbuffer,255,sizeof(jbuffer));jput(jbuffer,jmagic);jput(jbuffer+4,generation);
    jput(jbuffer+8,run);jput(jbuffer+12,build);jput(jbuffer+16,jcrc(jbuffer,16));
}
// Only inhibited boot/export uses the parent's synchronous, stopped-bridge writer.
// The other page is never erased by this operation. Permit is the final write.
static bool jprepare(uint8_t page,uint32_t generation,uint32_t run){
    jmake_header(generation,run);uint32_t address=base+uint32_t(page)*256;
    if(!Flash_log_write(address,0,true))return false;
    for(unsigned n=4;n<20;n+=4)if(!Flash_log_write(address+n,jget(jbuffer+n),false))return false;
    #if BMCU_USB_WORKING_MODE
    return Flash_log_write(address,jmagic,false);
#else
    return Flash_log_write(address,jmagic,false) && Flash_log_write(address+20,jpermit,false);
#endif
}
static void jexport_frame(){
    memcpy(frame,"BFL2",4);jput(frame+4,(512u<<16)|2u);jput(frame+8,build);
    Flash_log_read(frame+20);const int newest=jnewest(frame+20);
    session=newest>=0?jget(frame+20+newest*256+8):0;jput(frame+12,session);
    // bit0: bounded rolling; bit1/2: valid pages; bit3: local storage failure;
    // bit4: oldest generations overwritten; bit5: ambiguous headers.
    uint32_t flags=1u|(jvalid(frame+20)?2u:0u)|(jvalid(frame+276)?4u:0u)|(storage_error?8u:0u);
    if(newest>=0&&jget(frame+20+newest*256+4)>2u)flags|=16u;
    if(newest==-2)flags|=32u;
    jput(frame+16,flags);
    frame_crc=jcrc(frame,532);jput(frame+532,frame_crc);tx=0;
}
#if BMCU_USB_WORKING_MODE
#include "working_boot.inc"
#endif
static bool jboot(){
#if BMCU_USB_WORKING_MODE
    return wboot();
#else
    const uint32_t reset=RCC->RSTSCKR;RCC->RSTSCKR|=1u<<24;
    const bool power_cycle=(reset&0xf8000000u)==0x08000000u;
    Flash_log_read(frame+20);int newest=jnewest(frame+20);bool virgin=true;
    for(unsigned n=0;n<512;n+=4)if(!jblank(jget(frame+20+n)))virgin=false;
    if(virgin){if(!jprepare(0,1,1))storage_error=true;jexport_frame();return false;}
    if(newest>=0){const uint8_t* p=frame+20+newest*256;
        // NRST, torn/absent permit, and a completed run always stay in export.
        if(power_cycle&&jget(p+20)==jpermit&&jblank(jget(p+24))){
            if(Flash_log_write(base+newest*256+24,jconsumed,false)){
                jpage=uint8_t(newest);jgeneration=jget(p+4);jsession=jget(p+8);session=jsession;
                // A prepared page is empty; refuse a tampered permit on nonempty history.
                bool empty=true;for(unsigned n=jheader;n<256;n+=4)if(!jblank(jget(p+n)))empty=false;
                if(empty){capturing=true;jevent(1,255,jgeneration);return true;}
            }else storage_error=true;
        }
    }
    jexport_frame();return false;
#endif
}
void journal_step(){
    if(!capturing||jfailed)return;
    const uint32_t now=jmillis();
    if(uint32_t(now-jlast_heartbeat)>=120000u){jlast_heartbeat=now;jevent(15,255,now);}
    if(bus_port_to_host.rx_dropped!=jlast_rx){jlast_rx=bus_port_to_host.rx_dropped;jevent(14,255,jlast_rx);}
    if(jwaiting){const int result=Flash_diagnostic_result();if(result==1)return;
        jwaiting=false;if(result!=2){jfailed=true;return;}++jword;
    }
    // All core submissions, metadata recovery and safety checks run before this step.
    // Never reserve the shared scheduler while control/ownership work is pending.
    if(!Flash_background_idle()||
       (full_load_fault()==FullLoadFault::none&&(full_load_journal_pending()||Flash_metadata_recovery_pending()))||
       (setup_trial_phase()!=SetupPhase::operation&&setup_trial_phase()!=SetupPhase::fault&&
        setup_trial_phase()!=SetupPhase::success)||!bus_background_ready())return;
    if(jstate==0){
        if(!jqcount)return;
        if(jslot==jrecords){
            if(jgeneration>=0xfffffffeu){jfailed=true;return;}
            jpage^=1u;++jgeneration;jmake_header(jgeneration,jsession);jstate=1;jword=0;
        }else{
            const uint32_t irq=irq_save_wch();JournalEvent event=jq[jqhead];
            jqhead=(jqhead+1u)%jqueue_size;--jqcount;irq_restore_wch(irq);
            event.crc=jcrc(&event,12,jgeneration);memcpy(jbuffer,&event,16);jstate=3;jword=0;
        }
    }
    uint32_t address=base+uint32_t(jpage)*256;
    if(jstate==1){
        if(jword){jstate=2;jword=0;}else {jwaiting=Flash_diagnostic_submit(address,0,true);return;}
    }
    if(jstate==2){
        if(jword==10u){jstate=0;jslot=0;jword=0;return;}
        // Commit magic last, after generation/session/build/CRC.
        const unsigned half=(jword+2u)%10u;
        jwaiting=Flash_diagnostic_submit(address+half*2u,uint16_t(jget(jbuffer+(half&~1u)*2u)>>((half&1u)*16u)),false);return;
    }
    if(jstate==3){
        if(jword==8u){++jslot;jstate=0;jword=0;return;}
        // Record CRC is last. Interrupted records are never called durable events.
        const unsigned half=jword;
        jwaiting=Flash_diagnostic_submit(address+jheader+jslot*16u+half*2u,
            uint16_t(jget(jbuffer+(half&~1u)*2u)>>((half&1u)*16u)),false);
    }
}
static void jexport_step(){
    const uint32_t now=time_ticks32();if(rx&&uint32_t(now-rx_at)>ms_to_ticks32(250))rx=0;
    for(unsigned n=0;n<8&&USART_GetFlagStatus(USART1,USART_FLAG_RXNE)!=RESET;++n){
        uint8_t v=uint8_t(USART_ReceiveData(USART1));rx_at=now;if(!rx&&v!='B')continue;ack[rx++]=v;
        if(rx!=20)continue;
        rx=0;
        if(memcmp(ack,"BFA2",4)||jget(ack+4)!=build||jget(ack+8)!=session||jget(ack+12)!=frame_crc||jget(ack+16)!=jcrc(ack,16))continue;
#if BMCU_USB_WORKING_MODE
        const bool ok=false; // Working boots never accept one-use rearming.
#else
        bool ok=acknowledged;const int newest=jnewest(frame+20);
        if(!ok&&!storage_error&&newest!=-2){
            if(newest>=0){const uint8_t* p=frame+20+newest*256;
                if(jget(p+20)==jpermit&&jblank(jget(p+24)))ok=true;
                else if(jget(p+4)<0xfffffffeu&&session<0xfffffffeu)
                    ok=jprepare(uint8_t(newest^1),jget(p+4)+1,session+1);
            }else ok=jprepare(0,1,1); // Explicit saved-frame ACK replaces legacy/unknown raw only here.
        }
        if(!ok)storage_error=true;
#endif
        memcpy(reply,ok?"BFK2":"BFE2",4);jput(reply+4,build);jput(reply+8,session);
        jput(reply+12,frame_crc);jput(reply+16,jcrc(reply,16));acknowledged=ok;tx=0;tx_at=now;
    }
    const bool responding=reply[0]!=0;const unsigned size=responding?20:sizeof(frame);
    if(tx==size){if(uint32_t(now-tx_at)<ms_to_ticks32(100))return;tx=0;}
    for(unsigned n=0;n<8&&tx<size&&USART_GetFlagStatus(USART1,USART_FLAG_TXE)!=RESET;++n)
        USART_SendData(USART1,(responding?reply:frame)[tx++]);
    if(tx==size)tx_at=now;
}
