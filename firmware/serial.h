/* Megumin LED display firmware
 * Copyright (C) 2018 Sebastian Götte <code@jaseg.net>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "global.h"

/* High-level stuff */
void serial_init(void);
void send_status_reply(void);

/* Internal low-level stuff */
void tx_char(uint8_t c);
void send_frame_formatted(uint8_t *buf, int len);

/* Error counters for debugging */
extern unsigned int uart_overruns;
extern unsigned int frame_overruns;
extern unsigned int invalid_frames;

union tx_buf_union {
    struct __attribute__((packed)) {
        uint8_t  firmware_version,
                 hardware_version;
        uint8_t  nbits;
        uint8_t  channel_spec;
        uint8_t color_spec;
        uint16_t nchannels;
        uint32_t uptime_s,
                 uart_overruns,
                 frame_overruns,
                 invalid_frames;
         int16_t vcc_mv,
                 temp_celsius;
    } desc_reply;
    struct __attribute__((packed)) {
        uint32_t mac;
        uint16_t  device_type;
    } device_type_reply;
    uint8_t byte_data[0];
};

union rx_buf_union {
    /* Right-aligned integer raw channel brightness values like so:
     *
     * bit index 31       ...       16 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
     *                                 | (MSB)      serial data put *here*     (LSB) |
     *           |<-utterly ignored->| |<-----------------MAX_BITS------------------>|
     *                                 |<----------------NBITS---------------->|  |<>|--ignored
     *                                 | (MSB)      brightness data      (LSB) |  |<>|--ignored
     */
    struct __attribute__((packed)) { uint16_t framebuf[NCHANNELS]; uint8_t end[0]; } set_fb_rq;
    uint8_t byte_data[0];
    uint32_t mac_data;
};
extern volatile union rx_buf_union rx_buf;
extern volatile uint8_t framebuf_out_of_sync;

#endif/*__SERIAL_H__*/
