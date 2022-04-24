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
#include "flash.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+
typedef struct
{
	uint32_t address;
	uint32_t length;
} memory_offest;

memory_offest const alt_offsets[] = {
	{.address = 0x200000, .length = 0x600000}, /* Main Gateware */
	{.address = 0x800000, .length = 0x400000}, /* Main Firmawre */
	{.address = 0xC00000, .length = 0x400000}, /* Extra */
	{.address = 0x000000, .length = 0x200000}  /* Bootloader */
};


static int complete_timeout;
static bool bl_upgrade = false;

/* Blink pattern
 * - 1000 ms : device should reboot
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum
{
	BLINK_DFU_IDLE,
	BLINK_DFU_IDLE_BOOTLOADER,
	BLINK_DFU_DOWNLOAD,
	BLINK_DFU_ERROR,
	BLINK_DFU_SLEEP,
};

static uint32_t blink_interval_ms = BLINK_DFU_IDLE;


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

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
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
      .GPIO_Mode = GPIO_Mode_IN_FLOATING,
      .GPIO_Speed = GPIO_Speed_10MHz,
      .GPIO_Pin = GPIO_Pin_3,
    }
  );

  /* iCE Reset PA10 */
  GPIO_Init(
    GPIOA, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_Out_PP,
      .GPIO_Speed = GPIO_Speed_10MHz,
      .GPIO_Pin = GPIO_Pin_10,
    }
  );
  GPIO_ResetBits(GPIOA, GPIO_Pin_10);

  /* SPI config */

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  SPI_InitTypeDef  SPI_InitStructure = {0};

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_SetBits(GPIOC, GPIO_Pin_10);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_SetBits(GPIOC, GPIO_Pin_11);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA, GPIO_Pin_4);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  SPI_Cmd(SPI1, ENABLE);

  SPI_Flash_WAKEUP();

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

/* Implement a 4-press reset to bootloader, timeout from first press is 3s */
void reset_task(){
  enum reset_state {
    IDLE,
    WINDOW
  };
  static uint32_t _timeout;
  static enum reset_state _state = IDLE;
  static uint32_t _count;

  static bool last_pin_value = false;
  bool pin_value = (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3)) ? true : false;
  bool edge = (pin_value == true) && (last_pin_value == false);
  
  switch(_state){
    case IDLE:
    default:
      if(edge){
        _state = WINDOW;
        _timeout = board_millis();
        _count = 0;
      }
      break;
    
    case WINDOW:
      if(edge){
        if(++_count >= 4){
          NVIC_SystemReset();
        }
      }
      if((board_millis() - _timeout) > 3000){
        _state = IDLE;
      }
      break;
  }

  last_pin_value = pin_value;
}


//--------------------------------------------------------------------+
// DFU callbacks
// Note: alt is used as the partition number, in order to support multiple partitions like FLASH, EEPROM, etc.
//--------------------------------------------------------------------+

// Invoked right before tud_dfu_download_cb() (state=DFU_DNBUSY) or tud_dfu_manifest_cb() (state=DFU_MANIFEST)
// Application return timeout in milliseconds (bwPollTimeout) for the next download/manifest operation.
// During this period, USB host won't try to communicate with us.
uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state)
{
	if (state == DFU_DNBUSY)
	{
		return 1; /* Request we are polled in 1ms */
	}
	else if (state == DFU_MANIFEST)
	{
		// since we don't buffer entire image and do any flashing in manifest stage
		return 0;
	}

	return 0;
}

// Invoked when received DFU_DNLOAD (wLength>0) following by DFU_GETSTATUS (state=DFU_DNBUSY) requests
// This callback could be returned before flashing op is complete (async).
// Once finished flashing, application must call tud_dfu_finish_flashing()
void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, uint8_t const *data, uint16_t length)
{
	(void)alt;
	(void)block_num;

	blink_interval_ms = BLINK_DFU_DOWNLOAD;

	if ((block_num * CFG_TUD_DFU_XFER_BUFSIZE) >= alt_offsets[alt].length)
	{
		// flashing op for download length error
		tud_dfu_finish_flashing(DFU_STATUS_ERR_ADDRESS);

		blink_interval_ms = BLINK_DFU_ERROR;

		return;
	}

	uint32_t flash_address = alt_offsets[alt].address + block_num * CFG_TUD_DFU_XFER_BUFSIZE;

	/* First block in 64K erase block */
	if ((flash_address & (FLASH_64K_BLOCK_ERASE_SIZE - 1)) == 0)
	{

		//spiflash_write_enable();
		//spiflash_sector_erase(flash_address);

		/* While FLASH erase is in progress update LEDs */
		//while (spiflash_read_status_register() & 1)
		{
			blink_task();
		}
	}

	//printf("tud_dfu_download_cb(), alt=%u, block=%u, flash_address=%08x\n", alt, block_num, flash_address);

	for (int i = 0; i < CFG_TUD_DFU_XFER_BUFSIZE / 256; i++)
	{

		//spiflash_write_enable();
		//spiflash_page_program(flash_address, data, 256);
		flash_address += 256;
		data += 256;

		/* While FLASH erase is in progress update LEDs */
		//while (spiflash_read_status_register() & 1)
		{
			blink_task();
		}
	}

	// flashing op for download complete without error
	tud_dfu_finish_flashing(DFU_STATUS_OK);
}

// Invoked when download process is complete, received DFU_DNLOAD (wLength=0) following by DFU_GETSTATUS (state=Manifest)
// Application can do checksum, or actual flashing if buffered entire image previously.
// Once finished flashing, application must call tud_dfu_finish_flashing()
void tud_dfu_manifest_cb(uint8_t alt)
{
	(void)alt;
	blink_interval_ms = BLINK_DFU_DOWNLOAD;

	// flashing op for manifest is complete without error
	// Application can perform checksum, should it fail, use appropriate status such as errVERIFY.
	tud_dfu_finish_flashing(DFU_STATUS_OK);
}

// Invoked when the Host has terminated a download or upload transfer
void tud_dfu_abort_cb(uint8_t alt)
{
	(void)alt;
	blink_interval_ms = BLINK_DFU_ERROR;
}

// Invoked when a DFU_DETACH request is received
void tud_dfu_detach_cb(void)
{
	blink_interval_ms = BLINK_DFU_SLEEP;
	//complete_timeout = 100;
}