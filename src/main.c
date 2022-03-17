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

#include <usbd_core.h>

#include <usbd_cdc.h>

void usb_dc_low_level_init(void) {
  RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_USBPHY);
  RCC_USBHSPLLCLKConfig(RCC_HSBHSPLLCLKSource_HSE);
  RCC_USBHSConfig(RCC_USBPLL_Div2);
  RCC_USBHSPLLCKREFCLKConfig(RCC_USBHSPLLCKREFCLK_4M);
  RCC_USBHSPHYPLLALIVEcmd(ENABLE);
#ifdef CONFIG_USB_HS
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
#else
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_OTG_FS, ENABLE);
#endif

  Delay_Us(100);
#ifndef CONFIG_USB_HS
  //EXTEN->EXTEN_CTR |= EXTEN_USBD_PU_EN;
  NVIC_EnableIRQ(OTG_FS_IRQn);
#else
  NVIC_EnableIRQ(USBHS_IRQn);
#endif
}

int main() {
  Delay_Init();
  USART_Printf_Init(115200);
  printf("SystemClk: %" PRIu32 "\r\n", SystemCoreClock);

  Delay_Ms(10);

  extern void cdc_acm_init(void);
  cdc_acm_init();

  while (!usb_device_is_configured()) {
  }
  while (1) {
    extern void cdc_acm_data_send_with_dtr_test();
    cdc_acm_data_send_with_dtr_test();
    Delay_Ms(500);
  }
}
