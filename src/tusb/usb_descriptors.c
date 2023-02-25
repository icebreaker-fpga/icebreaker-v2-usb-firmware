/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "tusb.h"
#include "class/vendor/vendor_device.h"
#include "flash.h"

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0x0403,
    .idProduct          = 0x6014,
    .bcdDevice          = 0x0900,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}


//--------------------------------------------------------------------+
// Generic Descriptor Templates
//--------------------------------------------------------------------+

#define TUD_GEN_INTERFACE_DESCRIPTOR(_itfnum, _altsetting, _num_endpoints, _itfclass, _itfsubclass, _itfprotocol, _itfstridx) \
	9, TUSB_DESC_INTERFACE, _itfnum, _altsetting, _num_endpoints, _itfclass, _itfsubclass, _itfprotocol, _itfstridx

#define TUD_GEN_BULK_EP_DESCRIPTOR(_epnum, _epsize) \
	7, TUSB_DESC_ENDPOINT, _epnum, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

#define TUD_GEN_INTERRUPT_EP_DESCRIPTOR(_epnum, _epsize, _interval) \
	7, TUSB_DESC_ENDPOINT, _epnum, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_epsize), _interval


//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
enum
{
  ITF_NUM_MPSSE_0,
  ITF_NUM_TOTAL
};

// 1x (interface + 2 endpoints)
#define TUD_MPSSE_DESC_LEN  (9+7+7)
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_MPSSE_DESC_LEN)

#define EPNUM_MPSSE_0_IN      0x81
#define EPNUM_MPSSE_0_OUT     0x02



uint8_t const desc_fs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),
  TUD_GEN_INTERFACE_DESCRIPTOR(0, 0, 2, 0xFF, 0xFF, 0xFF , 2),
  TUD_GEN_BULK_EP_DESCRIPTOR(EPNUM_MPSSE_0_IN, CFG_TUD_VENDOR_EPSIZE),
  TUD_GEN_BULK_EP_DESCRIPTOR(EPNUM_MPSSE_0_OUT, CFG_TUD_VENDOR_EPSIZE),
 };


uint8_t const desc_hs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),
  TUD_GEN_INTERFACE_DESCRIPTOR(0, 0, 2, 0xFF, 0xFF, 0xFF , 2),
  TUD_GEN_BULK_EP_DESCRIPTOR(EPNUM_MPSSE_0_IN, CFG_TUD_VENDOR_EPSIZE),
  TUD_GEN_BULK_EP_DESCRIPTOR(EPNUM_MPSSE_0_OUT, CFG_TUD_VENDOR_EPSIZE),
};

// device qualifier is mostly similar to device descriptor since we don't change configuration based on speed
tusb_desc_device_qualifier_t const desc_device_qualifier =
{
  .bLength            = sizeof(tusb_desc_device_qualifier_t),
  .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
  .bcdUSB             = 0x0200,

  .bDeviceClass       = 0xFF,
  .bDeviceSubClass    = 0x00,
  .bDeviceProtocol    = 0x00,

  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
  .bNumConfigurations = 0x01,
  .bReserved          = 0x00
};

// Invoked when received GET DEVICE QUALIFIER DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete.
// device_qualifier descriptor describes information about a high-speed capable device that would
// change if the device were operating at the other speed. If not highspeed capable stall this request.
uint8_t const* tud_descriptor_device_qualifier_cb(void)
{
  return (uint8_t const*) &desc_device_qualifier;
}

// Invoked when received GET OTHER SEED CONFIGURATION DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
// Configuration descriptor in the other speed e.g if high speed then this is for full speed and vice versa
uint8_t const* tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
  (void) index; // for multiple configurations

  // if link speed is high return fullspeed config, and vice versa
  return (tud_speed_get() == TUSB_SPEED_HIGH) ?  desc_fs_configuration : desc_hs_configuration;
}


// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // for multiple configurations

#if TUD_OPT_HIGH_SPEED
  // Although we are highspeed, host may be fullspeed.
  return (tud_speed_get() == TUSB_SPEED_HIGH) ?  desc_hs_configuration : desc_fs_configuration;
#else
  return desc_fs_configuration;
#endif
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const* string_desc_arr [] =
{
  (const char[]) { 0x09, 0x04 },                // 0: is supported language is English (0x0409)
  "1BitSquared",                                // 1: Manufacturer
  "iCEBreaker V1.99a",                          // 2: Product
  "dev00001",                                   // 3: Serial
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;

  size_t chr_count;

  if ( index == 0)
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }
  else
  {

    if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

    const char* str = string_desc_arr[index];

    // Cap at max char
    chr_count = strlen(str);
    if ( chr_count > 39 ) {
      chr_count = 39;
    }

    // Convert ASCII string into UTF-16
    for(uint8_t i=0; i<chr_count; i++)
    {
      _desc_str[1+i] = str[i];
    }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (uint16_t)((((uint16_t)TUSB_DESC_STRING) << 8 ) | (2u*chr_count + 2u));

  return _desc_str;
}
