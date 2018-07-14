

/*
 * Initialize USB ACM serial device, provide convenience
 * function for serial communication.
 *
 * Mostly plucked together from some opencm3 example code.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/nvic.h>

#include "usb.h"

#define STATUS_LED_PORT GPIOC
#define STATUS_LED_PIN  GPIO13

#define USBD_PORT GPIOA
#define USBDM     GPIO11
#define USBDP     GPIO12

#define RX_ECHO     1
#define RX_BUF_LEN  256

static char    rx_tmp[RX_BUF_LEN];
static char    rx_buf[RX_BUF_LEN];

static size_t  rx_tmp_len;
static uint8_t rx_buf_ready;

static usbd_device *usbd_dev;

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static const struct usb_device_descriptor device_descriptor = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0xfe, // USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x1209,
    .idProduct = 0x4001,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}};

static const struct usb_interface_descriptor comm_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = 0xfe,  // USB_CLASS_CDC,
    .bInterfaceSubClass = 0x00, // USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = 0x00, // USB_CDC_PROTOCOL_AT,
    .iInterface = 0,

    .endpoint = comm_endp,

    .extra = NULL, //  &cdcacm_functional_descriptors,
    .extralen = 0,
}};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,

    .endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = comm_iface,
}, {
    .num_altsetting = 1,
    .altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,

    .interface = ifaces,
};


static const char *usb_strings[] = {
    "Chaos Computer Club Berlin e.V.",
    "bluefnorf",
    "bf1",
};


void usb_gpio_init() {
    // Clock
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);

    // D+
    gpio_set_mode(USBD_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  USBDP);

    // LED
    gpio_set_mode(STATUS_LED_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  STATUS_LED_PIN);

}

void usb_status_led_toggle() {
    gpio_toggle(STATUS_LED_PORT, STATUS_LED_PIN);
}

static void usb_data_rx_cb(usbd_device *usbd_dev, uint8_t ep) {
    char buf[64];

    int len = usbd_ep_read_packet(usbd_dev, ep, buf, 64);

    if (len) {
        usbd_ep_write_packet(usbd_dev, 0x82, buf, len);
    }
}

static enum usbd_request_return_codes usb_control_request_cb(usbd_device *usbd_dev, struct usb_setup_data *req,
        uint8_t **buf, uint16_t *len, usbd_control_complete_callback *complete) {
    (void)usbd_dev;
    (void)req;
    (void)buf;
    (void)len;
    (void)complete;
    return 0;
}

static void usb_set_config_cb(usbd_device *usbd_dev, uint16_t wValue) {
    (void)wValue;

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, usb_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);

    usbd_register_control_callback(
        usbd_dev,
        USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
        usb_control_request_cb
    );
}

usbd_device *usb_serial_init() {
    // Initialize GPIO
    usb_gpio_init();

    // Initialize buffers
    memset(rx_tmp, 0, RX_BUF_LEN);
    memset(rx_buf, 0, RX_BUF_LEN);

    rx_tmp_len = 0;
    rx_buf_ready = 0;

    // Pull down D+
    gpio_clear(USBD_PORT, USBDP);
    for (int i = 0; i < 0x10000; i++) {
        __asm__("nop");
    }
    gpio_set(USBD_PORT, USBDP);

    // Initialize usb device
    usbd_dev = usbd_init(&st_usbfs_v1_usb_driver,
                         &device_descriptor,
                         &config,
                         usb_strings, 3,
                         usbd_control_buffer, sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(usbd_dev, usb_set_config_cb);

    // Pull down
    gpio_clear(USBD_PORT, USBDP);

    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);

    return usbd_dev;
}

void usb_lp_can_rx0_isr() {
    usbd_poll(usbd_dev);
    nvic_clear_pending_irq(NVIC_USB_LP_CAN_RX0_IRQ);
}
