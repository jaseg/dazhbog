#ifndef __USB_H__
#define __USB_H__

#include <libopencm3/usb/usbd.h>

usbd_device* usb_serial_init();
const char*  usb_serial_rx();
size_t       usb_serial_tx(const char*, size_t);


#endif

