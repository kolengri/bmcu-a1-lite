// Offline-only BFL2 decoder. No serial, modem-line, reset, or live-export API.
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#define BUILD 0x260b0403u
#define FRAME 536
static uint32_t u32(const uint8_t*p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static unsigned u16(const uint8_t*p){return p[0]|(unsigned)p[1]<<8;}
static uint32_t update(uint32_t v,const void*data,size_t n){const uint8_t*p=data;while(n--){v^=*p++;for(unsigned i=0;i<8;i++)v=(v>>1)^((v&1)?0xedb88320u:0);}return v;}
static uint32_t crc(const void*data,size_t n){return ~update(~0u,data,n);}
static bool synced(FILE*f){return !ferror(f)&&!fflush(f)&&!fsync(fileno(f));}
static FILE* newfile(int dir,const char*name){int fd=openat(dir,name,O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW|O_CLOEXEC,0600);return fd<0?NULL:fdopen(fd,"wb");}
static void hex(FILE*f,const uint8_t*p,size_t n){fputc('"',f);while(n--)fprintf(f,"%02x",*p++);fputc('"',f);}
#include "journal_decode.inc"
static void usage(void){puts("bmcu-usb-logs260b0403 OFFLINE ONLY\n  --input REGULAR_FILE --output NEWDIR [--max-bytes 536..1048576]\nLive ISP preparation and capture belong to the host workflow. Port/rearm/boot-export options are rejected.");}
int main(int argc,char**argv){
 const char*input=NULL,*out=NULL;size_t limit=65536;
 for(int i=1;i<argc;i++){
  if(!strcmp(argv[i],"--help")){usage();return 0;}
  if(i+1>=argc){usage();return 2;}
  const char*k=argv[i],*v=argv[++i];char*end=NULL;
  if(!strcmp(k,"--input"))input=v;else if(!strcmp(k,"--output"))out=v;
  else if(!strcmp(k,"--max-bytes")){unsigned long n=strtoul(v,&end,10);if(!*v||*end||n<FRAME||n>1048576)return 2;limit=n;}
  else return 2;
 }
 if(!out||!input){usage();return 2;}
 // Reject devices/FIFOs/symlinks before opening. Recheck identity/type after open.
 struct stat before,after;if(lstat(input,&before)||!S_ISREG(before.st_mode))return 1;
 int source=open(input,O_RDONLY|O_NONBLOCK|O_NOFOLLOW|O_CLOEXEC);
 if(source<0||fstat(source,&after)||!S_ISREG(after.st_mode)||before.st_dev!=after.st_dev||before.st_ino!=after.st_ino){if(source>=0)close(source);return 1;}
 if(mkdir(out,0700)){close(source);fprintf(stderr,"Output directory must be new.\n");return 1;}
 int dir=open(out,O_RDONLY|O_DIRECTORY|O_NOFOLLOW|O_CLOEXEC);if(dir<0){close(source);return 1;}
 FILE*raw=newfile(dir,"raw.bin"),*report=newfile(dir,"report.json"),*meta=newfile(dir,"metadata.json");if(!raw||!report||!meta)return 1;
 uint32_t raw_crc=~0u;uint8_t buf[1024],pending[FRAME],selected[FRAME];size_t used=0,total=0,frames=0,corrupt=0,noise=0;bool have=false,failed=false;
 const char*reason="eof";
 while(!failed&&total<limit){
  size_t room=sizeof(buf);if(room>limit-total)room=limit-total;ssize_t n=read(source,buf,room);
  if(n<0){if(errno==EINTR)continue;failed=true;reason="read_error";break;}if(!n)break;
  if(fwrite(buf,1,(size_t)n,raw)!=(size_t)n){failed=true;reason="disk_error";break;}total+=(size_t)n;raw_crc=update(raw_crc,buf,(size_t)n);
  for(ssize_t j=0;j<n;j++){
   pending[used++]=buf[j];while(used>=4&&memcmp(pending,"BFL2",4)){memmove(pending,pending+1,--used);noise++;}
   if(used==FRAME){if(valid_frame(pending)){frames++;if(!have){memcpy(selected,pending,FRAME);have=true;}else if(memcmp(selected,pending,FRAME)){failed=true;reason="conflicting_reports";}used=0;}else{corrupt++;memmove(pending,pending+1,--used);}}
  }
 }
 if(total==limit)reason="size_limit";
 if(have){if(!decode(report,selected))failed=true;}else{fputs("{\"transport_valid\":false,\"complete\":false,\"setup\":null}\n",report);failed=true;}
 fprintf(meta,"{\"export_requested\":false,\"export_request_sent\":0,\"export_mode\":\"offline\",\"format\":1,\"mode\":\"offline\",\"bytes\":%zu,\"raw_crc32\":\"%08x\",\"capture_failed\":%s,\"frames\":%zu,\"corrupt_frames\":%zu,\"noise_bytes\":%zu,\"partial_tail_bytes\":%zu,\"reason\":\"%s\",\"ack_requested\":false,\"observe_stream\":false,\"frame_timestamps_available\":false,\"first_frame_seconds\":0.000000000,\"last_frame_seconds\":0.000000000}\n",total,~raw_crc,failed?"true":"false",frames,corrupt,noise,used,reason);
 if(!synced(raw)||!synced(report)||!synced(meta)||fsync(dir))failed=true;
 close(source);if(fclose(raw)||fclose(report)||fclose(meta))failed=true;close(dir);
 printf("Saved %zu bytes; %zu validated frames; %s.\n",total,frames,reason);return failed?1:0;
}
