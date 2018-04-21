#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#define COLOR_SPEC_WHITE            0x00
#define COLOR_SPEC_SINGLE_COLOR     0x01
#define COLOR_SPEC_RGB              0x02
#define COLOR_SPEC_RGBW             0x03
#define COLOR_SPEC_COLD_WARM_WHITE  0x04
#define COLOR_SPEC_WWA              0x05 /* cold white/warm white/amber */

#define OLSNDOT_V1 0x01

#define FIRMWARE_VERSION 2
#define HARDWARE_VERSION 2

/* Maximum bit count supported by serial command protocol. The brightness data is assumed to be of this bit width, but
 * only the uppermost NBITS bits are used. */
#define MAX_BITS 16

/* Bit count of this device. Note that to change this you will also have to adapt the per-bit timer period lookup table
 * in main.c.  */
#define NBITS 14

#define NCHANNELS 32
#define CHANNEL_SPEC 'H'
#define COLOR_SPEC COLOR_SPEC_RGBW
#define DEVICE_TYPE OLSNDOT_V1

#define TS_CAL1 (*(uint16_t *)0x1FFFF7B8)
#define VREFINT_CAL (*(uint16_t *)0x1FFFF7BA)

extern uint32_t sys_time;
extern uint32_t sys_time_seconds;

#endif/*__GLOBAL_H__*/
