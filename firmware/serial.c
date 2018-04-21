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

#include <stm32f0xx.h>
#include <stdint.h>
#include <system_stm32f0xx.h>
#include <stm32f0xx_ll_utils.h>
#include <math.h>

#include "serial.h"
#include "adc.h"

unsigned int uart_overruns = 0;
unsigned int frame_overruns = 0;
unsigned int invalid_frames = 0;

static union tx_buf_union tx_buf;
volatile union rx_buf_union rx_buf;
volatile uint8_t framebuf_out_of_sync;

void tx_char(uint8_t c) {
    while (!(USART1->ISR & USART_ISR_TC));
    USART1->TDR = c;
}

void send_frame_formatted(uint8_t *buf, int len) {
    uint8_t *p=buf, *q=buf, *end=buf+len;
    do {
        while (*q && q!=end)
            q++;
        tx_char(q-p+1);
        while (*p && p!=end)
            tx_char(*p++);
        p++, q++;
    } while (p <= end);
    tx_char('\0');
}

void send_status_reply(void) {
    tx_buf.desc_reply.firmware_version = FIRMWARE_VERSION;
    tx_buf.desc_reply.hardware_version = HARDWARE_VERSION;
    tx_buf.desc_reply.nbits = NBITS;
    tx_buf.desc_reply.channel_spec = CHANNEL_SPEC;
    tx_buf.desc_reply.nchannels = NCHANNELS;
    tx_buf.desc_reply.color_spec = COLOR_SPEC;
    tx_buf.desc_reply.uptime_s = sys_time_seconds;
    tx_buf.desc_reply.vcc_mv = adc_vcc_mv;
    tx_buf.desc_reply.temp_celsius = adc_temp_celsius;
    tx_buf.desc_reply.uart_overruns = uart_overruns;
    tx_buf.desc_reply.frame_overruns = frame_overruns;
    tx_buf.desc_reply.invalid_frames = invalid_frames;
    send_frame_formatted(tx_buf.byte_data, sizeof(tx_buf.desc_reply));
}

/* This is the higher-level protocol handler for the serial protocol. It gets passed the number of data bytes in this
 * frame (which may be zero) and returns a pointer to the buffer where the next frame should be stored.
 */
static volatile inline void packet_received(int len) {
    static enum {
        PROT_ADDRESSED = 0,
        PROT_EXPECT_FRAME_SECOND_HALF = 1,
        PROT_IGNORE = 2,
    } protocol_state = PROT_IGNORE; 
    /* Use mac frames as delimiters to synchronize this protocol layer */
    if (len == 1 && rx_buf.byte_data[0] == 0x00) { /* Discovery packet */
        if (sys_time < 100 && sys_time_seconds == 0) { /* Only respond during the first 100ms after boot */
            tx_buf.device_type_reply.mac = MAC_ADDR;
            tx_buf.device_type_reply.device_type = DEVICE_TYPE;
            send_frame_formatted(tx_buf.byte_data, sizeof(tx_buf.device_type_reply));
        }

    } else if (len == 1) { /* Command packet */
        if (protocol_state == PROT_ADDRESSED) {
            switch (rx_buf.byte_data[0]) {
            case 0x01:
                send_status_reply();
                break;
            }
        } else {
            invalid_frames++;
        }
        protocol_state = PROT_IGNORE;

    } else if (len == 4) { /* Address packet */
        if (rx_buf.mac_data == MAC_ADDR) { /* we are addressed */
            protocol_state = PROT_ADDRESSED; /* start listening for frame buffer data */
        } else { /* we are not addressed */
            protocol_state = PROT_IGNORE; /* ignore packet */
        }

    } else if (len == sizeof(rx_buf.set_fb_rq)) {
        if (protocol_state == PROT_ADDRESSED) { /* First of two half-framebuffer data frames */
            /* Kick off buffer transfer. This triggers the main loop to copy data out of the receive buffer and paste it
             * properly formatted into the frame buffer. */
            if (framebuf_out_of_sync == 0) {
                framebuf_out_of_sync = 1;
            } else {
                /* FIXME An overrun happend. What should we do? */
                frame_overruns++;
            }

            /* Go to "hang mode" until next zero-length packet. */
            protocol_state = PROT_IGNORE;
        }

    } else {
        /* FIXME An invalid packet has been received. What should we do? */
        invalid_frames++;
        protocol_state = PROT_IGNORE; /* go into "hang mode" until next zero-length packet */
    }
}

void USART1_IRQHandler(void) {
    /* Since a large amount of data will be shoved down this UART interface we need a more reliable and more efficient
     * way of framing than just waiting between transmissions.
     *
     * This code uses "Consistent Overhead Byte Stuffing" (COBS). For details, see its Wikipedia page[0] or the proper
     * scientific paper[1] published on it. Roughly, it works like this:
     *
     * * A frame is at most 254 bytes in length.
     * * The null byte 0x00 acts as a frame delimiter. There is no null bytes inside frames.
     * * Every frame starts with an "overhead" byte indicating the number of non-null payload bytes until the next null
     *   byte in the payload, **plus one**. This means this byte can never be zero.
     * * Every null byte in the payload is replaced by *its* distance to *its* next null byte as above.
     *
     * This means, at any point the receiver can efficiently be synchronized on the next frame boundary by simply
     * waiting for a null byte. After that, only a simple state machine is necessary to strip the overhead byte and a
     * counter to then count skip intervals.
     *
     * Here is Wikipedia's table of example values:
     *
     *    Unencoded data          Encoded with COBS
     *    00                      01 01 00
     *    00 00                   01 01 01 00
     *    11 22 00 33             03 11 22 02 33 00
     *    11 22 33 44             05 11 22 33 44 00
     *    11 00 00 00             02 11 01 01 01 00
     *    01 02 ...FE             FF 01 02 ...FE 00
     *
     * [0] https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
     * [1] Cheshire, Stuart; Baker, Mary (1999). "Consistent Overhead Byte Stuffing"
     *     IEEE/ACM Transactions on Networking. doi:10.1109/90.769765
     *     http://www.stuartcheshire.org/papers/COBSforToN.pdf
     */

    /* Index inside the current frame payload */
    static int rxpos = 0;
    /* COBS state machine. This implementation might be a little too complicated, but it works well enough and I find it
     * reasonably easy to understand. */
    static enum {
        COBS_WAIT_SYNC = 0,  /* Synchronize with frame */
        COBS_WAIT_START = 1, /* Await overhead byte */
        COBS_RUNNING = 2     /* Process payload */
    } cobs_state = 0;
    /* COBS skip counter. During payload processing this contains the remaining non-null payload bytes */
    static int cobs_count = 0;

    if (USART1->ISR & USART_ISR_ORE) { /* Overrun handling */
        uart_overruns++;
        /* Reset and re-synchronize. Retry next frame. */
        rxpos = 0;
        cobs_state = COBS_WAIT_SYNC;
        /* Clear interrupt flag */
        USART1->ICR = USART_ICR_ORECF;

    } else { /* Data received */
        uint8_t data = USART1->RDR; /* This automatically acknowledges the IRQ */

        if (data == 0x00) { /* End-of-packet */
            if (cobs_state != COBS_WAIT_SYNC) /* Has a packet been received? */
                /* Process higher protocol layers on this packet. */
                packet_received(rxpos);

            /* Reset for next packet. */
            cobs_state = COBS_WAIT_START;
            rxpos = 0;

        } else { /* non-null byte */
            if (cobs_state == COBS_WAIT_SYNC) { /* Wait for null byte */
                /* ignore data */

            } else if (cobs_state == COBS_WAIT_START) { /* Overhead byte */
                cobs_count = data;
                cobs_state = COBS_RUNNING;

            } else { /* Payload byte */
                if (--cobs_count == 0) { /* Skip byte */
                    cobs_count = data;
                    data = 0;
                }

                /* Write processed payload byte to current receive buffer */
                rx_buf.byte_data[rxpos++] = data;
            }
        }
    }
}

