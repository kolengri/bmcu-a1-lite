// No source inference: constant bus-format passive RX waits for validated input.
// Called after zero PWM initialization and before sensors, setup or owner reads.
#if !BMCU_USB_ISP_EXPORT
static bool wexport_request(uint8_t* p,unsigned n){
    return n==15u && p[0]==0x3du && p[1]==0xc5u && p[2]==15u &&
        p[3]==bus_crc8(p,3) && p[4]==0xfeu && !memcmp(p+5,"BFX2",4) &&
        jget(p+9)==build && uint16_t(p[13]|uint16_t(p[14])<<8)==bus_crc16(p,13);
}
#endif
static bool wprinter_packet(uint8_t* p,int n){
    const auto type=::get_packge_type(p,n);
    if(type==bambubus_package_type::heartbeat)return true;
    if(type==bambubus_package_type::none || type==bambubus_package_type::ETC)return false;
    // Long classification already validates the exact Lite target address.
    if(p[1]==4u || p[1]==5u)return true;
    if(p[1]!=0xc5u)return false;
    // Discovery's common request is addressed by its protocol operation, before registration.
    if(type==bambubus_package_type::online_detect)return p[5]==0u || p[5]==3u || (p[5]==1u && p[6]==bambubus_ams_num());
    return n>5 && p[5]==bambubus_ams_num();
}
static void wstart(){
    // This write happens only after positive printer selection, while PWM is stopped.
    Flash_log_read(frame+20);const int newest=jnewest(frame+20);
    bool virgin=true;for(unsigned n=0;n<512;n+=4)if(!jblank(jget(frame+20+n)))virgin=false;
    if(newest==-2 || (newest<0 && !virgin)){storage_error=jfailed=true;return;}
    const uint32_t generation=newest>=0?jget(frame+20+newest*256+4):0;
    const uint32_t run=newest>=0?jget(frame+20+newest*256+8):0;
    if(generation>=0xfffffffeu || run>=0xfffffffeu){storage_error=jfailed=true;return;}
    jpage=newest>=0?uint8_t(newest^1):0;jgeneration=generation+1;jsession=run+1;
    if(!jprepare(jpage,jgeneration,jsession)){storage_error=jfailed=true;return;}
    session=jsession;capturing=true;jevent(1,255,jgeneration);
}
static bool wboot(){
    RCC->RSTSCKR|=1u<<24; // Flags never select mode or grant execution.
#if BMCU_USB_ISP_EXPORT
    bus_boot_quiesce();
    alignas(4) uint32_t marker[4]={0x33494642u,build,0x00010001u,0};marker[3]=jcrc(marker,12);
    const auto selection=Flash_boot_export_take(marker);
    if(selection==BootExportMarker::consumed){jexport_frame();return false;}
    if(selection==BootExportMarker::failed){for(;;)(void)time_ticks32();} // No TX, motion, setup or retry on failed consume.
#endif
    bus_passive_init();
    for(;;){
        bus_rx_poll();
        if(!bus_port_to_host.recv_data_len)continue;
        uint8_t* p=bus_port_to_host.bus_recv_data_ptr;const int n=bus_port_to_host.recv_data_len;
#if BMCU_USB_ISP_EXPORT
        const bool export_selected=false; // No in-band USB selection on the shared RX net.
#else
        const bool export_selected=bus_port_to_host.bus_package_type==_bus_data_type::bambubus && wexport_request(p,n);
#endif
        const bool printer_selected=!export_selected && bus_port_to_host.bus_package_type==_bus_data_type::bambubus && wprinter_packet(p,n);
        bus_port_to_host.release_packet();
        if(!export_selected && !printer_selected)continue;
        bus_shutdown(); // Trigger and queued traffic are discarded by normal bus_init.
        if(export_selected){jexport_frame();return false;}
        wstart();return true; // Diagnostic failure cannot inhibit normal guarded control.
    }
}
