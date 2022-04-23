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

#include "usb_ch32_usbhs_reg.h"

void cdc_task(void);
void blink_task(void);
void reset_task(void);

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
  RCC_USBHSConfig(RCC_USBPLL_Div4);
  RCC_USBHSPLLCKREFCLKConfig(RCC_USBHSPLLCKREFCLK_4M);
  RCC_USBHSPHYPLLALIVEcmd(ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure = {0};

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_Init(
    GPIOC, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_Out_PP,
      .GPIO_Speed = GPIO_Speed_10MHz,
      .GPIO_Pin = GPIO_Pin_2,
    }
  );

  GPIO_Init(
    GPIOE, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_IPU,
      .GPIO_Speed = GPIO_Speed_10MHz,
      .GPIO_Pin = GPIO_Pin_3,
    }
  );

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

    blink_task();
    reset_task();
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
      (void) count;

      // Echo back
      // Note: Skip echo by commenting out write() and write_flush()
      // for throughput test e.g
      //    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
      tud_cdc_write(buf, count);
      tud_cdc_write_flush();
    }
  }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;
  (void) rts;

  // TODO set some indicator
  if ( dtr )
  {
    // Terminal connected
  }else
  {
    // Terminal disconnected
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;
}


void blink_task(){
  static unsigned int _time;
  static bool _toggle;

  if((board_millis() - _time) > 250){
    _time = board_millis();

    if(_toggle ^= 1){
      GPIO_ResetBits(GPIOC, 4);
    }else{
      GPIO_SetBits(GPIOC, 4);
    }
  }
}

void reset_task(){
  if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3)){
    NVIC_SystemReset();
  }
}