/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v30x_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main Interrupt Service Routines.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Copyright (c) 2022 Ahmed Charles
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include "ch32v30x_it.h"

void NMI_Handler() __attribute__((naked));
void HardFault_Handler() __attribute__((naked));

// This function handles NMI exception.
void NMI_Handler() {
  __asm volatile("mret");
}

// This function handles Hard Fault exception.
void HardFault_Handler() {
  __asm volatile("1: j 1b");
}
