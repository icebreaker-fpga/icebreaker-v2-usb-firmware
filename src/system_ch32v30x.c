/********************************** (C) COPYRIGHT *******************************
 * File Name          : system_ch32v30x.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : CH32V30x Device Peripheral Access Layer System Source File.
 *                      For HSE = 8Mhz
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *********************************************************************************/

#include "ch32v30x.h"

// Uncomment the line corresponding to the desired System clock (SYSCLK)
// frequency (after reset the HSI is used as SYSCLK source).
// If none of the define below is enabled, the HSI is used as System clock
// source.

// #define SYSCLK_FREQ_HSE    HSE_VALUE
// #define SYSCLK_FREQ_24MHz  24000000
// #define SYSCLK_FREQ_48MHz  48000000
// #define SYSCLK_FREQ_56MHz  56000000
#define SYSCLK_FREQ_72MHz 72000000
// #define SYSCLK_FREQ_96MHz  96000000
// #define SYSCLK_FREQ_120MHz  120000000
//#define SYSCLK_FREQ_144MHz  144000000

/* Clock Definitions */
#ifdef SYSCLK_FREQ_HSE
uint32_t SystemCoreClock = SYSCLK_FREQ_HSE; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_24MHz
uint32_t SystemCoreClock = SYSCLK_FREQ_24MHz; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_48MHz
uint32_t SystemCoreClock = SYSCLK_FREQ_48MHz; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_56MHz
uint32_t SystemCoreClock = SYSCLK_FREQ_56MHz; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_72MHz
uint32_t SystemCoreClock = SYSCLK_FREQ_72MHz; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_96MHz
uint32_t SystemCoreClock = SYSCLK_FREQ_96MHz; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_120MHz
uint32_t SystemCoreClock = SYSCLK_FREQ_120MHz; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_144MHz
uint32_t SystemCoreClock = SYSCLK_FREQ_144MHz; /* System Clock Frequency (Core Clock) */
#else /* HSI Selected as System Clock source */
uint32_t SystemCoreClock = HSI_VALUE; /* System Clock Frequency (Core Clock) */
#endif

__I uint8_t AHBPrescTable[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9 };

/* system_private_function_proto_types */
static void SetSysClock();

#ifdef SYSCLK_FREQ_HSE
static void SetSysClockToHSE(void);
#elif defined SYSCLK_FREQ_24MHz || defined SYSCLK_FREQ_48MHz || defined SYSCLK_FREQ_56MHz          \
  || defined SYSCLK_FREQ_72MHz || defined SYSCLK_FREQ_96MHz || defined SYSCLK_FREQ_120MHz          \
  || defined SYSCLK_FREQ_144MHz
static void SetSysClockImpl(uint32_t ppre1_div, uint32_t pll_mull);
#endif

// Setup the microcontroller system Initialize the Embedded Flash Interface,
// the PLL and update the SystemCoreClock variable.
void SystemInit() {
  RCC->CTLR |= (uint32_t)0x00000001;

#ifdef CH32V30x_D8C
  RCC->CFGR0 &= (uint32_t)0xF8FF0000;
#else
  RCC->CFGR0 &= (uint32_t)0xF0FF0000;
#endif

  RCC->CTLR &= (uint32_t)0xFEF6FFFF;
  RCC->CTLR &= (uint32_t)0xFFFBFFFF;
  RCC->CFGR0 &= (uint32_t)0xFF80FFFF;

#ifdef CH32V30x_D8C
  RCC->CTLR &= (uint32_t)0xEBFFFFFF;
  RCC->INTR = 0x00FF0000;
  RCC->CFGR2 = 0x00000000;
#else
  RCC->INTR = 0x009F0000;
#endif
  SetSysClock();
}

// Update SystemCoreClock variable according to Clock Register Values.
void SystemCoreClockUpdate() {
  uint32_t tmp = 0, pllmull = 0, pllsource = 0, Pll_6_5 = 0;

  tmp = RCC->CFGR0 & RCC_SWS;

  switch (tmp) {
  case 0x00:
    SystemCoreClock = HSI_VALUE;
    break;
  case 0x04:
    SystemCoreClock = HSE_VALUE;
    break;
  case 0x08:
    pllmull = RCC->CFGR0 & RCC_PLLMULL;
    pllsource = RCC->CFGR0 & RCC_PLLSRC;
    pllmull = (pllmull >> 18) + 2;

#ifdef CH32V30x_D8
    if (pllmull == 17) {
      pllmull = 18;
    }
#else
    if (pllmull == 2) {
      pllmull = 18;
    }
    if (pllmull == 15) {
      pllmull = 13; /* *6.5 */
      Pll_6_5 = 1;
    }
    if (pllmull == 16) {
      pllmull = 15;
    }
    if (pllmull == 17) {
      pllmull = 16;
    }
#endif

    if (pllsource == 0x00) {
      SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
    } else {
      if ((RCC->CFGR0 & RCC_PLLXTPRE) != (uint32_t)RESET) {
        SystemCoreClock = (HSE_VALUE >> 1) * pllmull;
      } else {
        SystemCoreClock = HSE_VALUE * pllmull;
      }
    }

    if (Pll_6_5 == 1) {
      SystemCoreClock = (SystemCoreClock / 2);
    }

    break;
  default:
    SystemCoreClock = HSI_VALUE;
    break;
  }

  tmp = AHBPrescTable[((RCC->CFGR0 & RCC_HPRE) >> 4)];
  SystemCoreClock >>= tmp;
}

// Configures the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers.
static void SetSysClock() {
#ifdef SYSCLK_FREQ_HSE
  SetSysClockToHSE();
#elif defined SYSCLK_FREQ_24MHz
#ifdef CH32V30x_D8
  SetSysClockImpl(RCC_PPRE1_DIV1, RCC_PLLMULL3);
#else
  SetSysClockImpl(RCC_PPRE1_DIV1, RCC_PLLMULL3_EXTEN);
#endif
#elif defined SYSCLK_FREQ_48MHz
#ifdef CH32V30x_D8
  SetSysClockImpl(RCC_PPRE1_DIV2, RCC_PLLMULL6);
#else
  SetSysClockImpl(RCC_PPRE1_DIV2, RCC_PLLMULL6_EXTEN);
#endif
#elif defined SYSCLK_FREQ_56MHz
#ifdef CH32V30x_D8
  SetSysClockImpl(RCC_PPRE1_DIV2, RCC_PLLMULL7);
#else
  SetSysClockImpl(RCC_PPRE1_DIV2, RCC_PLLMULL7_EXTEN);
#endif
#elif defined SYSCLK_FREQ_72MHz
#ifdef CH32V30x_D8
  SetSysClockImpl(RCC_PPRE1_DIV2, RCC_PLLMULL9);
#else
  SetSysClockImpl(RCC_PPRE1_DIV2, RCC_PLLMULL9_EXTEN);
#endif
#elif defined SYSCLK_FREQ_96MHz
#ifdef CH32V30x_D8
  SetSysClockImpl(RCC_PPRE1_DIV2, RCC_PLLMULL12);
#else
  SetSysClockImpl(RCC_PPRE1_DIV2, RCC_PLLMULL12_EXTEN);
#endif
#elif defined SYSCLK_FREQ_120MHz
#ifdef CH32V30x_D8
  SetSysClockImpl(RCC_PPRE1_DIV2, RCC_PLLMULL15);
#else
  SetSysClockImpl(RCC_PPRE1_DIV2, RCC_PLLMULL15_EXTEN);
#endif
#elif defined SYSCLK_FREQ_144MHz
#ifdef CH32V30x_D8
  SetSysClockImpl(RCC_PPRE1_DIV2, RCC_PLLMULL18);
#else
  SetSysClockImpl(RCC_PPRE1_DIV2, RCC_PLLMULL18_EXTEN);
#endif
#endif
  // If none of the define above is enabled, the HSI is used as System clock
  // source (default after reset)
}

#ifdef SYSCLK_FREQ_HSE

// Sets HSE as System clock source and configure HCLK, PCLK2 and PCLK1 prescalers.
static void SetSysClockToHSE(void) {
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

  RCC->CTLR |= ((uint32_t)RCC_HSEON);

  /* Wait till HSE is ready and if Time out is reached exit */
  do {
    HSEStatus = RCC->CTLR & RCC_HSERDY;
    StartUpCounter++;
  } while ((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CTLR & RCC_HSERDY) != RESET) {
    HSEStatus = (uint32_t)0x01;
  } else {
    HSEStatus = (uint32_t)0x00;
  }

  if (HSEStatus == (uint32_t)0x01) {
    /* HCLK = SYSCLK */
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV1;

    /* Select HSE as system clock source */
    RCC->CFGR0 &= (uint32_t)((uint32_t) ~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_HSE;

    /* Wait till HSE is used as system clock source */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x04) {
    }
  } else {
    // If HSE fails to start-up, the application will have wrong clock
    // configuration. User can add here some code to deal with this error
  }
}

#elif defined SYSCLK_FREQ_24MHz || defined SYSCLK_FREQ_48MHz || defined SYSCLK_FREQ_56MHz || defined SYSCLK_FREQ_72MHz || defined SYSCLK_FREQ_96MHz || defined SYSCLK_FREQ_120MHz || defined SYSCLK_FREQ_144MHz

// Sets System clock frequency to X MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
static void SetSysClockImpl(uint32_t ppre1_div, uint32_t pll_mull) {
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

  RCC->CTLR |= RCC_HSEON;

  /* Wait till HSE is ready and if Time out is reached exit */
  do {
    HSEStatus = RCC->CTLR & RCC_HSERDY;
    StartUpCounter++;
  } while ((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CTLR & RCC_HSERDY) != RESET) {
    HSEStatus = (uint32_t)0x01;
  } else {
    HSEStatus = (uint32_t)0x00;
  }

  if (HSEStatus == (uint32_t)0x01) {
    /* HCLK = SYSCLK */
    RCC->CFGR0 |= RCC_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR0 |= RCC_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR0 |= ppre1_div;

    /* PLL configuration: PLLCLK = HSE * pll_mull = X MHz */
    RCC->CFGR0 &= ~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL);

    RCC->CFGR0 |= (RCC_PLLSRC_HSE | RCC_PLLXTPRE_HSE | pll_mull);

    /* Enable PLL */
    RCC->CTLR |= RCC_PLLON;

    /* Wait till PLL is ready */
    while ((RCC->CTLR & RCC_PLLRDY) == 0) {
    }

    /* Select PLL as system clock source */
    RCC->CFGR0 &= ~RCC_SW;
    RCC->CFGR0 |= RCC_SW_PLL;

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR0 & RCC_SWS) != (uint32_t)0x08) {
    }
  } else {
    // If HSE fails to start-up, the application will have wrong clock
    // configuration. User can add here some code to deal with this error
  }
}

#endif
