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
  '-march=rv32imac',
  '-mabi=ilp32',
  '-msmall-data-limit=8',
  '-mno-save-restore',
  '-fmessage-length=0',
  '-fsigned-char',
  '-ffunction-sections',
  '-fdata-sections',
], language: ['c', 'cpp'])
if get_option('usb_port') == 'hs'
  add_global_arguments('-DCONFIG_USB_HS', language: ['c', 'cpp'])
endif
if get_option('usb_debug') == 'error'
  add_global_arguments('-DUSB_DBG_LEVEL=0', language: ['c', 'cpp'])
elif get_option('usb_debug') == 'warning'
  add_global_arguments('-DUSB_DBG_LEVEL=1', language: ['c', 'cpp'])
elif get_option('usb_debug') == 'info'
  add_global_arguments('-DUSB_DBG_LEVEL=2', language: ['c', 'cpp'])
elif get_option('usb_debug') == 'log'
  add_global_arguments('-DUSB_DBG_LEVEL=3', language: ['c', 'cpp'])
endif
if get_option('board') == 'ch32v307v-r1'
  add_global_arguments('-DDEBUG=1', language: ['c', 'cpp'])
elif get_option('board') == 'icebreaker-v1.99a'
  add_global_arguments('-DDEBUG=3', language: ['c', 'cpp'])
endif
add_global_link_arguments([
  '-march=rv32imac',
  '-mabi=ilp32',
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
  [src, ch32v307_src, cherryusb_src],
  include_directories : [inc, ch32v307_inc, cherryusb_inc],
)
hex = custom_target(
  name + '.hex',
  input : exe,
  output : name + '.hex',
  command : [objcopy, '-O', 'ihex', '@INPUT@', '@OUTPUT@'],
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
