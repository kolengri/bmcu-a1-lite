static bool header(const uint8_t*p){return u32(p)==0x324a4642u&&u32(p+12)==BUILD&&u32(p+4)>0&&u32(p+4)<UINT32_MAX&&u32(p+8)>0&&u32(p+8)<UINT32_MAX&&u32(p+16)==crc(p,16);}
static int newest(const uint8_t*r){bool a=header(r),b=header(r+256);if(!a)return b?1:-1;if(!b)return 0;uint32_t d=u32(r+4)-u32(r+260);return !d||d==0x80000000u?-2:d<0x80000000u?0:1;}
static bool valid_frame(const uint8_t*p){
 if(memcmp(p,"BFL2",4)||u16(p+4)!=2||u16(p+6)!=512||u32(p+8)!=BUILD||u32(p+532)!=crc(p,532))return false;
 const uint8_t*r=p+20;int n=newest(r);uint32_t flags=1u|(header(r)?2u:0u)|(header(r+256)?4u:0u)|(u32(p+16)&8u);
 if(n>=0&&u32(r+n*256+4)>2)flags|=16u;
 if(n==-2)flags|=32u;
 return flags==u32(p+16)&&u32(p+12)==(n>=0?u32(r+n*256+8):0);
}
static bool record(const uint8_t*p,uint32_t generation){return u32(p+12)==~update(generation,p,12);}
static bool blank_record(const uint8_t*p){for(unsigned n=0;n<16;n+=4)if(u32(p+n)!=UINT32_MAX&&u32(p+n)!=0xe339e339u)return false;return true;}
static bool paired_meter(const uint8_t*r,unsigned page,unsigned slot){
 const uint8_t*h=r+page*256,*e=h+32+slot*16,*next;
 uint32_t generation=u32(h+4);
 if(slot<13u)next=e+16;
 else {
  const uint8_t*other=r+(page^1u)*256;
  if(!header(other)||u32(other+4)!=generation+1u||u32(other+8)!=u32(h+8))return false;
  generation=u32(other+4);next=other+32;
 }
 return record(e,u32(h+4))&&record(next,generation)&&next[8]==18u&&
  u32(next)==u32(e)&&next[9]==e[7]&&u16(next+10)==0u;
}
static bool decode(FILE*f,const uint8_t*p){
 const uint8_t*r=p+20;int latest=newest(r);unsigned count=0,invalid=0,losses=0;bool first=true;
 fprintf(f,"{\n\"format\":2,\"build\":\"%08x\",\"session\":%u,\"frame_crc32\":\"%08x\",\"transport_valid\":true,\"header_valid\":%s,\"complete\":false,\"firmware_storage_error\":%s,\n",BUILD,u32(p+12),u32(p+532),header(r)||header(r+256)?"true":"false",u32(p+16)&8?"true":"false");
 fprintf(f,"\"journal\":{\"valid\":%s,\"bounded_rolling\":true,\"capacity_events\":28,\"ambiguous_headers\":%s,\"prior_generations_discarded\":%s,\"newest_generation\":",latest>=0?"true":"false",latest==-2?"true":"false",u32(p+16)&16?"true":"false");
 if(latest>=0)fprintf(f,"%u",u32(r+latest*256+4));else fputs("null",f);
 fputs(",\"pages\":[",f);
 for(unsigned page=0;page<2;++page){const uint8_t*h=r+page*256;if(page)fputc(',',f);
  fprintf(f,"{\"page\":%u,\"valid\":%s",page,header(h)?"true":"false");
  if(header(h))fprintf(f,",\"generation\":%u,\"session\":%u,\"permit_armed\":%s,\"permit_consumed\":%s",u32(h+4),u32(h+8),u32(h+20)==0x6a51ffffu?"true":"false",u32(h+24)==0x31534e43u?"true":"false");
  fputc('}',f);
 }
 fputs("],\"events\":[",f);
 // Page generation then slot defines order. Milliseconds wrap and are not sorted.
 for(unsigned order=0;order<2;++order){unsigned page=latest>=0?(unsigned)(latest^(order==0?1:0)):order;const uint8_t*h=r+page*256;if(!header(h))continue;
  for(unsigned slot=0;slot<14;++slot){const uint8_t*e=h+32+slot*16;if(!record(e,u32(h+4))){if(!blank_record(e))++invalid;continue;}
   unsigned kind=e[8],channel=e[9],dropped=u16(e+10);++count;losses+=dropped;if(!first)fputc(',',f);first=false;
   fprintf(f,"{\"page\":%u,\"generation\":%u,\"session\":%u,\"slot\":%u,\"millis_mod4294967296\":%u,\"kind\":%u,\"channel\":%u,\"value\":%u,\"dropped_before_saturated65535\":%u",page,u32(h+4),u32(h+8),slot,u32(e),kind,channel,u32(e+4),dropped);
   if(kind==16||kind==17)fprintf(f,",\"submitted_response\":{\"command\":%u,\"requested_channel\":%u,\"state\":%u,\"motion\":%u,\"returned_use\":%u,\"returned_channel\":%u,\"meter_record_committed\":%s}",kind-13,channel,e[4],e[5],e[6],e[7],paired_meter(r,page,slot)?"true":"false");
   if(kind==18){uint32_t bits=u32(e+4);float value;memcpy(&value,&bits,4);fprintf(f,",\"wire_meters_bits\":\"%08x\",\"wire_meters\":",bits);if(isfinite(value))fprintf(f,"%.9g",(double)value);else fputs("null",f);}
   if(kind==19)fprintf(f,",\"eligible_pull\":{\"command\":%u,\"requested_channel\":%u,\"state\":%u,\"motion\":%u,\"admission_inferred\":false}",e[6],channel,e[4],e[5]);
   fputs(",\"raw\":",f);hex(f,e,16);fputc('}',f);
  }
 }
 fprintf(f,"],\"committed_events\":%u,\"invalid_nonblank_records\":%u,\"retained_dropped_count_sum\":%u,\"durability_lag\":\"Up to eight queued events plus one in-progress record; arbitrary additional loss during no flash-ready windows or diagnostic failure. A power cut can lose this uncommitted tail.\",\"limits\":\"At most28 retained events. Generation ordering includes normal boot sessions and runtime rollover; generation>2 means earlier page generations have been discarded, not a counted number of lost events. Dropped counters saturate and only describe retained records. TX means DMA started, not printer acceptance. A TX with no adjacent committed meter is explicitly incomplete; meter-only records also occur. No physical timing or full trace guarantee.\"},\n\"raw_flash\":",count,invalid,losses);
 hex(f,r,512);fputs("\n}\n",f);return !ferror(f);
}
