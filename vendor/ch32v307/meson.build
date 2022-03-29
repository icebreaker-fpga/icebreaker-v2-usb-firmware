#  Copyright 2022 Ahmed Charles <me@ahmedcharles.com>
#  Copyright 2022 Greg Davill <greg.davill@gmail.com>
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

ch32v307_inc = include_directories([
  'core',
  'debug',
  'peripheral/inc',
])
ch32v307_src = files([
  'core/core_riscv.c',
  'debug/debug.c',
  'peripheral/src/ch32v30x_adc.c',
  'peripheral/src/ch32v30x_bkp.c',
  'peripheral/src/ch32v30x_can.c',
  'peripheral/src/ch32v30x_crc.c',
  'peripheral/src/ch32v30x_dac.c',
  'peripheral/src/ch32v30x_dbgmcu.c',
  'peripheral/src/ch32v30x_dma.c',
  'peripheral/src/ch32v30x_dvp.c',
  'peripheral/src/ch32v30x_eth.c',
  'peripheral/src/ch32v30x_exti.c',
  'peripheral/src/ch32v30x_flash.c',
  'peripheral/src/ch32v30x_fsmc.c',
  'peripheral/src/ch32v30x_gpio.c',
  'peripheral/src/ch32v30x_i2c.c',
  'peripheral/src/ch32v30x_iwdg.c',
  'peripheral/src/ch32v30x_misc.c',
  'peripheral/src/ch32v30x_opa.c',
  'peripheral/src/ch32v30x_pwr.c',
  'peripheral/src/ch32v30x_rcc.c',
  'peripheral/src/ch32v30x_rng.c',
  'peripheral/src/ch32v30x_rtc.c',
  'peripheral/src/ch32v30x_sdio.c',
  'peripheral/src/ch32v30x_spi.c',
  'peripheral/src/ch32v30x_tim.c',
  'peripheral/src/ch32v30x_usart.c',
  'peripheral/src/ch32v30x_wwdg.c',
  'startup/startup_ch32v30x_D8C.S',
])
linker_script = files('ld' / 'link.ld')
link_args += [
  '-T@0@'.format(meson.project_source_root() / '@0@'.format(linker_script[0])),
]
ch32v307_link_depends = linker_script
