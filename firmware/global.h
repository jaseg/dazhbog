#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#define FIRMWARE_VERSION 2
#define HARDWARE_VERSION 2

#define TS_CAL1 (*(uint16_t *)0x1FFFF7B8)
#define VREFINT_CAL (*(uint16_t *)0x1FFFF7BA)

extern uint32_t sys_time;
extern uint32_t sys_time_seconds;

#endif/*__GLOBAL_H__*/
