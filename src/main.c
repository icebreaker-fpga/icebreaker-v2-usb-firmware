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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flash.h"
#include "platform.h"
#include "time.h"
#include "tusb.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+
typedef struct {
  uint32_t address;
  uint32_t length;
} memory_offest;

memory_offest const alt_offsets[] = {
  { .address = 0x000000, .length = 0x800000 }, /* Main Gateware */
  { .address = 0x000000, .length = 0x800000 }, /* Main Firmawre */
};

/* Blink pattern
 * - 1000 ms : device should reboot
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum {
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

__attribute__((naked, aligned(4))) void USBHS_IRQHandler(void) {
  tud_int_handler(0);
  __asm volatile("mret");
}

uint32_t SysTick_Config(uint32_t ticks) {
  NVIC_EnableIRQ(SysTicK_IRQn);
  SysTick->CTLR = 0;
  SysTick->SR = 0;
  SysTick->CNT = 0;
  SysTick->CMP = ticks - 1;
  SysTick->CTLR = 0xF;
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
  RCC_USBHSPLLCKREFCLKConfig(RCC_USBHSPLLCKREFCLK_8M);
  RCC_USBHSPHYPLLALIVEcmd(ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

  GPIO_Init(GPIOC,
            &(GPIO_InitTypeDef){
              .GPIO_Mode = GPIO_Mode_Out_PP,
              .GPIO_Speed = GPIO_Speed_10MHz,
              .GPIO_Pin = GPIO_Pin_2,
            });

  GPIO_Init(GPIOC,
            &(GPIO_InitTypeDef){
              .GPIO_Mode = GPIO_Mode_Out_PP,
              .GPIO_Speed = GPIO_Speed_10MHz,
              .GPIO_Pin = GPIO_Pin_3,
            });

  GPIO_Init(GPIOE,
            &(GPIO_InitTypeDef){
              .GPIO_Mode = GPIO_Mode_IN_FLOATING,
              .GPIO_Speed = GPIO_Speed_10MHz,
              .GPIO_Pin = GPIO_Pin_3,
            });
  GPIO_Init(GPIOE,
            &(GPIO_InitTypeDef){
              .GPIO_Mode = GPIO_Mode_IN_FLOATING,
              .GPIO_Speed = GPIO_Speed_10MHz,
              .GPIO_Pin = GPIO_Pin_2,
            });

  GPIO_Init(GPIOE,
            &(GPIO_InitTypeDef){
              .GPIO_Mode = GPIO_Mode_Out_PP,
              .GPIO_Speed = GPIO_Speed_10MHz,
              .GPIO_Pin = GPIO_Pin_13,
            });
  GPIO_SetBits(GPIOE, GPIO_Pin_13);

  /* iCE Reset PA10 */
  GPIO_Init(GPIOA,
            &(GPIO_InitTypeDef){
              .GPIO_Mode = GPIO_Mode_Out_PP,
              .GPIO_Speed = GPIO_Speed_10MHz,
              .GPIO_Pin = GPIO_Pin_10,
            });
  GPIO_ResetBits(GPIOA, GPIO_Pin_10);

  /* iCE Clk PA8 */
  GPIO_Init(GPIOA,
            &(GPIO_InitTypeDef){
              .GPIO_Mode = GPIO_Mode_AF_PP,
              .GPIO_Speed = GPIO_Speed_50MHz,
              .GPIO_Pin = GPIO_Pin_8,
            });
  RCC_MCOConfig(RCC_MCO_XT1);

  /* Enable interrupts globaly */
  __enable_irq();
}

#if CFG_TUSB_OS == OPT_OS_NONE

volatile uint32_t system_ticks = 0;

/* Small workaround to support HW stack save/restore */
void SysTick_Handler(void) __attribute__((naked, aligned(4)));
void SysTick_Handler(void) {
  __asm volatile("call SysTick_Handler_impl; mret");
}

__attribute__((used, aligned(4))) void SysTick_Handler_impl(void) {
  SysTick->SR = 0;
  system_ticks++;
}

uint32_t board_millis(void) {
  return system_ticks;
}

#endif

__attribute__((aligned(4))) int main() {
  board_init();

  ice40_reset_hold();
  flash_spi_init();

  /* Read out UUID and cache it, used by USB system for descriptors */
  SPI_Flash_WAKEUP();
  SPI_Flash_ReadUUID(NULL);

  flash_spi_deinit();
  ice40_reset_release();

  ice40_spi_init();

  tusb_init();

  while (1) {
    tud_task(); // tinyusb device task

    blink_task();
    reset_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {}

// Invoked when device is unmounted
void tud_umount_cb(void) {}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {}

void blink_task() {
  static unsigned int _time;
  static bool _toggle;

  if ((board_millis() - _time) > 200) {
    _time = board_millis();

    if (_toggle ^= 1) {
      GPIO_ResetBits(GPIOC, GPIO_Pin_3);
      GPIO_SetBits(GPIOC, GPIO_Pin_2);
    } else {
      GPIO_SetBits(GPIOC, GPIO_Pin_3);
      GPIO_ResetBits(GPIOC, GPIO_Pin_2);
    }
  }
}

/* Implement a 4-press reset to bootloader, timeout from first press is 3s */
void reset_task() {
  enum reset_state {
    IDLE,
    WINDOW
  };
  static uint32_t _timeout;
  static enum reset_state _state = IDLE;
  static uint32_t _count;

  static bool last_pin_value = false;
  bool pin_value = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3);
  bool edge = (pin_value == true) && (last_pin_value == false);

  switch (_state) {
  case IDLE:
  default:
    if (edge) {
      _state = WINDOW;
      _timeout = board_millis();
      _count = 0;
    }
    break;

  case WINDOW:
    if (edge) {
      if (++_count >= 4) {
        NVIC_SystemReset();
      }
    }
    if ((board_millis() - _timeout) > 3000) {
      _state = IDLE;
    }
    break;
  }

  last_pin_value = pin_value;
}

enum mpsse_requests {
  REQUEST_RESET = 0x00,
  REQUEST_MODEM_CTRL = 0x01,
  REQUEST_SET_FLOW_CTRL = 0x02,
  REQUEST_SET_BAUD_RATE = 0x03,
  REQUEST_SET_DATA = 0x04,
  REQUEST_GET_MODEM_STAT = 0x05,
  REQUEST_SET_EVENT_CHAR = 0x06,
  REQUEST_SET_ERROR_CHAR = 0x07,
  REQUEST_SET_LAT_TIMER = 0x09,
  REQUEST_GET_LAT_TIMER = 0x0A,
  REQUEST_SET_BITMODE = 0x0B,
  FTDI_SIO_READ_EEPROM = 0x90,
};

uint8_t lat_timer = 0;

//--------------------------------------------------------------------+
// MPSSE use vendor class
//--------------------------------------------------------------------+

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool tud_vendor_control_xfer_cb(uint8_t rhport,
                                uint8_t stage,
                                tusb_control_request_t const* request) {
  // nothing to with DATA & ACK stage
  if (stage != CONTROL_STAGE_SETUP)
    return true;

  switch (request->bmRequestType_bit.type) {
  case TUSB_REQ_TYPE_VENDOR:
    switch (request->bRequest) {
    case REQUEST_RESET:
      return tud_control_xfer(rhport, request, NULL, 0);

    case REQUEST_GET_LAT_TIMER:
      if (request->wLength == 1) {
        return tud_control_xfer(rhport, request, (void*)(uint8_t[]){ lat_timer }, 1);
      }
      return false;

    case REQUEST_SET_LAT_TIMER:
      lat_timer = request->wValue;
      return tud_control_xfer(rhport, request, NULL, 0);

    case REQUEST_SET_BAUD_RATE: {
      uint8_t baud_low = request->wValue & 0xFF;
      uint8_t baud_high = (request->wValue >> 8) & 0xFF;
      uint8_t port = request->wIndex & 0xFF;
      uint8_t clock_div = (request->wIndex >> 8) & 0xFF;
    }
      return tud_control_xfer(rhport, request, NULL, 0);

    case REQUEST_SET_BITMODE:
      return tud_control_xfer(rhport, request, NULL, 0);

    case FTDI_SIO_READ_EEPROM:
      /* Return 0xFF bytes, like a real FTDI without EEPROM might */
      if (request->wLength == 2) {
        return tud_control_xfer(rhport, request, (void*)(uint8_t[]){ 0xFF, 0xFF }, 2);
      }
      return false;

    default:
      break;
    }
    break;

  default:
    break;
  }

  // stall unknown request
  return false;
}

/* MPSSE engine command definitions */
enum mpsse_cmd {
  /* Mode commands */
  MC_SETB_LOW = 0x80,     /* Set Data bits LowByte */
  MC_READB_LOW = 0x81,    /* Read Data bits LowByte */
  MC_SETB_HIGH = 0x82,    /* Set Data bits HighByte */
  MC_READB_HIGH = 0x83,   /* Read data bits HighByte */
  MC_LOOPBACK_EN = 0x84,  /* Enable loopback */
  MC_LOOPBACK_DIS = 0x85, /* Disable loopback */
  MC_SET_CLK_DIV = 0x86,  /* Set clock divisor */
  MC_FLUSH = 0x87,        /* Flush buffer fifos to the PC. */
  MC_WAIT_H = 0x88,       /* Wait on GPIOL1 to go high. */
  MC_WAIT_L = 0x89,       /* Wait on GPIOL1 to go low. */
  MC_TCK_X5 = 0x8A,       /* Disable /5 div, enables 60MHz master clock */
  MC_TCK_D5 = 0x8B,       /* Enable /5 div, backward compat to FT2232D */
  MC_EN_3PH_CLK = 0x8C,   /* Enable 3 phase clk, DDR I2C */
  MC_DIS_3PH_CLK = 0x8D,  /* Disable 3 phase clk */
  MC_CLK_N = 0x8E,        /* Clock every bit, used for JTAG */
  MC_CLK_N8 = 0x8F,       /* Clock every byte, used for JTAG */
  MC_CLK_TO_H = 0x94,     /* Clock until GPIOL1 goes high */
  MC_CLK_TO_L = 0x95,     /* Clock until GPIOL1 goes low */
  MC_EN_ADPT_CLK = 0x96,  /* Enable adaptive clocking */
  MC_DIS_ADPT_CLK = 0x97, /* Disable adaptive clocking */
  MC_CLK8_TO_H = 0x9C,    /* Clock until GPIOL1 goes high, count bytes */
  MC_CLK8_TO_L = 0x9D,    /* Clock until GPIOL1 goes low, count bytes */
  MC_TRI = 0x9E,          /* Set IO to only drive on 0 and tristate on 1 */
  /* CPU mode commands */
  MC_CPU_RS = 0x90, /* CPUMode read short address */
  MC_CPU_RE = 0x91, /* CPUMode read extended address */
  MC_CPU_WS = 0x92, /* CPUMode write short address */
  MC_CPU_WE = 0x93, /* CPUMode write extended address */
};

/* Transfer Command bits */

/* All byte based commands consist of:
 * - Command byte
 * - Length lsb
 * - Length msb
 *
 * If data out is enabled the data follows after the above command bytes,
 * otherwise no additional data is needed.
 * - Data * n
 *
 * All bit based commands consist of:
 * - Command byte
 * - Length
 *
 * If data out is enabled a byte containing bitst to transfer follows.
 * Otherwise no additional data is needed. Only up to 8 bits can be transferred
 * per transaction when in bit mode.
 */

/* b 0000 0000
 *   |||| |||`- Data out negative enable. Update DO on negative clock edge.
 *   |||| ||`-- Bit count enable. When reset count represents bytes.
 *   |||| |`--- Data in negative enable. Latch DI on negative clock edge.
 *   |||| `---- LSB enable. When set clock data out LSB first.
 *   ||||
 *   |||`------ Data out enable
 *   ||`------- Data in enable
 *   |`-------- TMS mode enable
 *   `--------- Special command mode enable. See mpsse_cmd enum.
 */

#define MC_DATA_TMS  (0x40) /* When set use TMS mode */
#define MC_DATA_IN   (0x20) /* When set read data (Data IN) */
#define MC_DATA_OUT  (0x10) /* When set write data (Data OUT) */
#define MC_DATA_LSB  (0x08) /* When set input/output data LSB first. */
#define MC_DATA_ICN  (0x04) /* When set receive data on negative clock edge */
#define MC_DATA_BITS (0x02) /* When set count bits not bytes */
#define MC_DATA_OCN  (0x01) /* When set update data on negative clock edge */

typedef enum {
  OP_CODE = 0,
  OP_PARAM0,
  OP_PARAM1,
  DATA_OUT
} mpsse_state_t;

mpsse_state_t mpsse_state;
uint8_t current_op = 0;
uint8_t param0 = 0;
uint8_t param1 = 0;

uint16_t out_cnt;

void process_mpsse(uint8_t c) {
  switch (mpsse_state) {
  case OP_CODE: {
    current_op = c;
    if (c & 0x80) {
      switch (c) {
      case MC_READB_LOW: {
        uint8_t gpio_l = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) << 4)    // CS
                         | (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) << 6)   // CDONE
                         | (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10) << 7); // CRESET
        //printf("gpio: %02x\r\n", gpio_l);
        tud_vendor_write((void*)(uint8_t[]){ gpio_l }, 1);
        tud_vendor_flush();
      } break;

      case MC_TCK_D5: {
        break;
      }

      case MC_SETB_LOW:
      case MC_SET_CLK_DIV:
      case MC_CLK_N8:
      case MC_CLK_N:
        mpsse_state = OP_PARAM0;
        break;

      default:
        printf("Unknown OP: %02x\r\n", c);
        break;
      }
    } else {
      if (c & MC_DATA_OUT) {
        mpsse_state = OP_PARAM0;
      }
    }
  } break;

  case OP_PARAM0:
    if (current_op & 0x80) {
      switch (current_op) {
      case MC_SETB_LOW:
      case MC_CLK_N8:
        param0 = c;
        mpsse_state = OP_PARAM1;
        break;

      case MC_SET_CLK_DIV:
        mpsse_state = OP_PARAM1;
        break;

      case MC_CLK_N: {
        param0 = c;
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
        GPIO_Init(GPIOB,
                  &(GPIO_InitTypeDef){
                    .GPIO_Mode = GPIO_Mode_Out_PP,
                    .GPIO_Speed = GPIO_Speed_50MHz,
                    .GPIO_Pin = GPIO_Pin_13,
                  });

        /* Transmit dummy bytes */
        for (int i = 0; i <= param0; i++) {
          /* Toggle Clock line */
          GPIO_ResetBits(GPIOB, GPIO_Pin_13);
          GPIO_SetBits(GPIOB, GPIO_Pin_13);
        }

        GPIO_Init(GPIOB,
                  &(GPIO_InitTypeDef){
                    .GPIO_Mode = GPIO_Mode_AF_PP,
                    .GPIO_Speed = GPIO_Speed_50MHz,
                    .GPIO_Pin = GPIO_Pin_13,
                  });
        mpsse_state = OP_CODE;
      } break;

      default:
        mpsse_state = OP_CODE;
        break;
      }
    } else {
      if (current_op & MC_DATA_OUT) {
        param0 = c;
        mpsse_state = OP_PARAM1;
      }
    }
    break;

  case OP_PARAM1:
    if (current_op & 0x80) {
      switch (current_op) {
      case MC_SETB_LOW:
        param1 = c;
        /* Set Pins */
        /* Param0 = GPIO, param1 = direction */

        /* CS */
        if (param0 & 0x10) {
          GPIO_SetBits(GPIOB, GPIO_Pin_12);
        } else {
          GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        }

        if (param1 & 0x10) {
          GPIO_Init(GPIOB,
                    &(GPIO_InitTypeDef){
                      .GPIO_Mode = GPIO_Mode_Out_PP,
                      .GPIO_Speed = GPIO_Speed_50MHz,
                      .GPIO_Pin = GPIO_Pin_12,
                    });
        } else {
          GPIO_Init(GPIOB,
                    &(GPIO_InitTypeDef){
                      .GPIO_Mode = GPIO_Mode_IN_FLOATING,
                      .GPIO_Speed = GPIO_Speed_50MHz,
                      .GPIO_Pin = GPIO_Pin_12,
                    });
        }

        /* Reset */
        if (param0 & 0x80) {
          GPIO_SetBits(GPIOA, GPIO_Pin_10);
        } else {
          GPIO_ResetBits(GPIOA, GPIO_Pin_10);
        }

        if (param1 & 0x80) {
          GPIO_Init(GPIOA,
                    &(GPIO_InitTypeDef){
                      .GPIO_Mode = GPIO_Mode_Out_PP,
                      .GPIO_Speed = GPIO_Speed_50MHz,
                      .GPIO_Pin = GPIO_Pin_10,
                    });
        } else {
          GPIO_Init(GPIOA,
                    &(GPIO_InitTypeDef){
                      .GPIO_Mode = GPIO_Mode_IN_FLOATING,
                      .GPIO_Speed = GPIO_Speed_50MHz,
                      .GPIO_Pin = GPIO_Pin_10,
                    });
        }

        mpsse_state = OP_CODE;
        break;

      case MC_SET_CLK_DIV:
        mpsse_state = OP_CODE;
        break;

      case MC_CLK_N8: {
        mpsse_state = OP_CODE;

        for (int i = 0; i <= param0; i++) {
          SPI_I2S_SendData(SPI2, 0xFF); /* 8 dummy clocks */
          while (!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE))
            ;
        }
      } break;

      default:
        mpsse_state = OP_CODE;
        break;
      }
    } else {
      if (current_op & MC_DATA_OUT) {
        param1 = c;
        mpsse_state = DATA_OUT;
        out_cnt = (param0 + (param1 << 8)) + 1;
      }
    }
    break;
  case DATA_OUT: {
    if (current_op & MC_DATA_OUT) {
      SPI_I2S_SendData(SPI2, c); /* 8 dummy clocks */
      while (!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE))
        ;

      if (--out_cnt == 0) {
        mpsse_state = OP_CODE;
      }
    }

  } break;
  }
}

// Invoked when received new data
void tud_vendor_rx_cb(uint8_t itf) {
  while (1) {
    uint8_t c;
    if (tud_vendor_read(&c, 1)) {
      process_mpsse(c);
    } else {
      break;
    }
  }
}

// Invoked when last rx transfer finished
void tud_vendor_tx_cb(uint8_t itf, uint32_t sent_bytes) {}