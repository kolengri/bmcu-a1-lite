#pragma once
#include "lite_profile.h"
#if BMCU_LITE_AUTHORITY_OBSERVATION
namespace authority_observation {
enum Reason : uint8_t { authority=1,idle=2,fault=3,use=4,pull=5,hard=6,hold_zero=7,applied_zero=8,lease=9,bus_error=10,departure=11 };
void wire(uint8_t command,uint8_t ch,uint8_t state,uint8_t motion);
void grant(uint8_t ch,uint64_t received,bool new_origin);
void deferred();
void pressure(uint8_t kind,uint8_t ch);
void pwm(uint8_t ch,int value);
void stop(uint8_t reason,uint8_t fault=0);
void render();
#if BMCU_USB_FLIGHT_RECORDER
void snapshot(uint8_t* out);
#endif
}
#endif
