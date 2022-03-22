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

#include <debug.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "tusb.h"


void cdc_task(void);

volatile uint8_t dtr_enable = 0;


//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

void USBHS_IRQHandler(void) __attribute__((naked));
void USBHS_IRQHandler(void) { 
        __asm volatile ("call USBHS_IRQHandler_impl; mret");
}

__attribute__ ((used)) void USBHS_IRQHandler_impl(void) { 
  tud_int_handler(0); 
}

void UART6_IRQHandler() __attribute__((naked));
__attribute__ ((used)) void UART6_IRQHandler_impl() {
  if (USART_GetITStatus(UART6, USART_IT_RXNE) != RESET && dtr_enable) {
    uint8_t data = USART_ReceiveData(UART6);
    tud_cdc_write(&data, 1);
    tud_cdc_write_flush();
  }
}

void UART6_IRQHandler() {
  __asm volatile("call UART6_IRQHandler_impl; mret");
}


uint32_t SysTick_Config(uint32_t ticks)
{
    NVIC_EnableIRQ(SysTicK_IRQn);
    SysTick->CTLR=0;
    SysTick->SR=0;
    SysTick->CNT=0;
    SysTick->CMP=ticks-1;
    SysTick->CTLR=0xF;
    return 0;
}

void board_init(void) {

  /* Disable interrupts during init */
  __disable_irq();


#if CFG_TUSB_OS == OPT_OS_NONE
  SysTick_Config(SystemCoreClock / 1000);
#endif

	USART_Printf_Init(115200);

  RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_USBPHY);
  RCC_USBHSPLLCLKConfig(RCC_HSBHSPLLCLKSource_HSE);
  RCC_USBHSConfig(RCC_USBPLL_Div2);
  RCC_USBHSPLLCKREFCLKConfig(RCC_USBHSPLLCKREFCLK_4M);
  RCC_USBHSPHYPLLALIVEcmd(ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure = {0};
  USART_InitTypeDef USART_InitStructure = { 0 };
  NVIC_InitTypeDef NVIC_InitStructure = { 0 };

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART6, ENABLE);

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

  /* Enable interrupts globaly */
  __enable_irq();
}

#if CFG_TUSB_OS == OPT_OS_NONE

volatile uint32_t system_ticks = 0;

/* Small workaround to support HW stack save/restore */
void SysTick_Handler(void) __attribute__((naked));
void SysTick_Handler(void) { 
      __asm volatile ("call SysTick_Handler_impl; mret");
}

__attribute__((used)) void SysTick_Handler_impl(void) { 
  SysTick->SR=0;
  system_ticks++;
}

uint32_t board_millis(void) { return system_ticks; }

#endif


int main() {
  
  board_init();
  tusb_init();

  while (1)
  {
    tud_task(); // tinyusb device task
    
    cdc_task();
  }



}



//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void)
{
  // connected() check for DTR bit
  // Most but not all terminal client set this when making connection
  // if ( tud_cdc_connected() )
  {
    // connected and there are data available
    if ( tud_cdc_available() )
    {
      // read datas
      char buf[64];
      uint32_t count = tud_cdc_read(buf, sizeof(buf));

      for (uint32_t i = 0; i != count; ++i) {
        USART_SendData(UART6, buf[i]);
        while (USART_GetFlagStatus(UART6, USART_FLAG_TXE) == RESET) { /* waiting for sending finish */
        }
      }
    }
  }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;
  (void) rts;

  if ( dtr )
  {
    dtr_enable = 1;
  }else
  {
    dtr_enable = 0;
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;
}
