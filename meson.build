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

project(
  'icebreaker-ch32v307-usb',
  'c',
  default_options : ['buildtype=minsize'],
  license : 'Apache-2.0',
  version : '0.0.1',
)

assert(
  meson.is_cross_build(),
  'Microcontroller projects can only be built in a cross build environment.',
)

add_global_arguments([
  '-flto',
  '-march=rv32imafc',
  '-mabi=ilp32f',
  '-msmall-data-limit=8',
  '-mno-save-restore',
  '-fmessage-length=0',
  '-fsigned-char',
  '-ffunction-sections',
  '-fdata-sections',
  '-DCFG_TUSB_MCU=OPT_MCU_CH32VF307',
	'-DBOARD_DEVICE_RHPORT_SPEED=OPT_MODE_HIGH_SPEED',
  '-DTUP_DCD_ENDPOINT_MAX=16',
], language: ['c', 'cpp'])
if get_option('usb_port') == 'hs'
  add_global_arguments('-DCONFIG_USB_HS', language: ['c', 'cpp'])
endif
add_global_link_arguments([
  '-flto',
  '-march=rv32imafc',
  '-mabi=ilp32f',
  '-msmall-data-limit=8',
  '-mno-save-restore',
  '-fmessage-length=0',
  '-fsigned-char',
  '-ffunction-sections',
  '-fdata-sections',
  '-nostartfiles',
  '-Xlinker',
  '--gc-sections',
  '--specs=nano.specs',
  '--specs=nosys.specs',
], language: ['c', 'cpp'])

objcopy = find_program('objcopy')
objdump = find_program('objdump')

subdir('vendor')
subdir('src')

name = 'icebreaker-ch32v307-usb'
exe = executable(
  name + '.elf',
  [src, ch32v307_src, tusb_src],
  include_directories : [inc, ch32v307_inc, tusb_inc],
)
hex = custom_target(
  name + '.hex',
  input : exe,
  output : name + '.hex',
  command : [objcopy, '-O', 'ihex', '@INPUT@', '@OUTPUT@'],
  depends : exe,
  build_by_default : true,
)
bin = custom_target(
  name + '.bin',
  input : exe,
  output : name + '.bin',
  command : [objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
  depends : exe,
  build_by_default : true,
)
lst = custom_target(
  name + '.lst',
  input : exe,
  output : name + '.lst',
  command : [objdump, '--all-headers', '--demangle', '--disassemble', '@INPUT@'],
  capture : true,
  depends : exe,
  build_by_default : true,
)
run_target('upload', command: [ find_program('python'), '-m', 'ch55xtool', '-f', bin, '-r'], depends: exe)
