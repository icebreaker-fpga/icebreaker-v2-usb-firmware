// Copyright 2022 sakumisu
// Copyright 2022 Ahmed Charles <me@ahmedcharles.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <inttypes.h>

#include <ch32v30x_misc.h>
#include <ch32v30x_usart.h>

#include <usbd_core.h>

#include <usbd_cdc.h>

#include "cdc_acm_to_uart.h"

void UART6_IRQHandler() __attribute__((naked));

/*!< endpoint address */
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83

#define USBD_VID           0xFFFF
#define USBD_PID           0xFFFF
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

/*!< config descriptor size */
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

/*!< global descriptor */
// clang-format off
static const uint8_t cdc_descriptor[] = {
  USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
  USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
  CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, 0x02),

  ///////////////////////////////////////
  /// string0 descriptor
  ///////////////////////////////////////
  USB_LANGID_INIT(USBD_LANGID_STRING),

  ///////////////////////////////////////
  /// string1 descriptor - Manufacturer
  ///////////////////////////////////////
  0x18,                       /* bLength */
  USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
  '1', 0x00,                  /* wcChar0 */
  'B', 0x00,                  /* wcChar1 */
  'i', 0x00,                  /* wcChar2 */
  't', 0x00,                  /* wcChar3 */
  'S', 0x00,                  /* wcChar4 */
  'q', 0x00,                  /* wcChar5 */
  'u', 0x00,                  /* wcChar6 */
  'a', 0x00,                  /* wcChar7 */
  'r', 0x00,                  /* wcChar8 */
  'e', 0x00,                  /* wcChar9 */
  'd', 0x00,                  /* wcChar10 */

  ///////////////////////////////////////
  /// string2 descriptor - Product
  ///////////////////////////////////////
  0x24,                       /* bLength */
  USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
  'i', 0x00,                  /* wcChar0 */
  'C', 0x00,                  /* wcChar1 */
  'E', 0x00,                  /* wcChar2 */
  'B', 0x00,                  /* wcChar3 */
  'r', 0x00,                  /* wcChar4 */
  'e', 0x00,                  /* wcChar5 */
  'a', 0x00,                  /* wcChar6 */
  'k', 0x00,                  /* wcChar7 */
  'e', 0x00,                  /* wcChar8 */
  'r', 0x00,                  /* wcChar9 */
  ' ', 0x00,                  /* wcChar10 */
  'V', 0x00,                  /* wcChar11 */
  '1', 0x00,                  /* wcChar12 */
  '.', 0x00,                  /* wcChar13 */
  '9', 0x00,                  /* wcChar14 */
  '9', 0x00,                  /* wcChar15 */
  'a', 0x00,                  /* wcChar16 */

  ///////////////////////////////////////
  /// string3 descriptor - Serial
  ///////////////////////////////////////
  0x12,                       /* bLength */
  USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
  'i', 0x00,                  /* wcChar0 */
  'B', 0x00,                  /* wcChar1 */
  'S', 0x00,                  /* wcChar2 */
  'r', 0x00,                  /* wcChar3 */
  '6', 0x00,                  /* wcChar4 */
  'D', 0x00,                  /* wcChar5 */
  'v', 0x00,                  /* wcChar6 */
  'E', 0x00,                  /* wcChar7 */

#ifdef CONFIG_USB_HS
  ///////////////////////////////////////
  /// device qualifier descriptor
  ///////////////////////////////////////
  0x0a,
  USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x02,
  0x02,
  0x01,
  0x40,
  0x01,
  0x00,
#endif

  0x00,
};
// clang-format on

/*!< class */
usbd_class_t cdc_class;
/*!< interface one */
usbd_interface_t cdc_cmd_intf;
/*!< interface two */
usbd_interface_t cdc_data_intf;

// data terminal ready
volatile uint8_t dtr_enable = 0;

// data from host
void usbd_cdc_acm_out(uint8_t ep) {
  uint8_t data[64];
  uint32_t read_byte;

  usbd_ep_read(ep, data, 64, &read_byte);
  printf("read len: %" PRIu32 ": %.*s\r\n", read_byte, (int)read_byte, data);
  printf("read len: %d\r\n", (int)data[0]);
  for (uint32_t i = 0; i != read_byte; ++i) {
    USART_SendData(UART6, data[i]);
    while (USART_GetFlagStatus(UART6, USART_FLAG_TXE) == RESET) { /* waiting for sending finish */
    }
  }
  usbd_ep_read(ep, NULL, 0, NULL);
}

// data to host
void usbd_cdc_acm_in(uint8_t ep) {
  printf("in\r\n");
}

/*!< endpoint call back */
usbd_endpoint_t cdc_out_ep = { .ep_addr = CDC_OUT_EP, .ep_cb = usbd_cdc_acm_out };

usbd_endpoint_t cdc_in_ep = { .ep_addr = CDC_IN_EP, .ep_cb = usbd_cdc_acm_in };

/* function ------------------------------------------------------------------*/
void cdc_acm_init() {
  usbd_desc_register(cdc_descriptor);
  /*!< add interface */
  usbd_cdc_add_acm_interface(&cdc_class, &cdc_cmd_intf);
  usbd_cdc_add_acm_interface(&cdc_class, &cdc_data_intf);
  /*!< interface add endpoint */
  usbd_interface_add_endpoint(&cdc_data_intf, &cdc_out_ep);
  usbd_interface_add_endpoint(&cdc_data_intf, &cdc_in_ep);

  usb_dc_init();

  GPIO_InitTypeDef GPIO_InitStructure = { 0 };
  USART_InitTypeDef USART_InitStructure = { 0 };
  NVIC_InitTypeDef NVIC_InitStructure = { 0 };

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART6, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinRemapConfig(GPIO_PartialRemap_USART6, ENABLE);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

  USART_Init(UART6, &USART_InitStructure);
  USART_ITConfig(UART6, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = UART6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_Cmd(UART6, ENABLE);
}

void usbd_cdc_acm_set_dtr(bool dtr) {
  if (dtr) {
    dtr_enable = 1;
  } else {
    dtr_enable = 0;
  }
}

__attribute__ ((used)) void UART6_IRQHandler_impl() {
  if (USART_GetITStatus(UART6, USART_IT_RXNE) != RESET && dtr_enable) {
    uint8_t data = USART_ReceiveData(UART6);
    usbd_ep_write(CDC_IN_EP, &data, 1, NULL);
  }
}

void UART6_IRQHandler() {
  __asm volatile("call UART6_IRQHandler_impl; mret");
}
