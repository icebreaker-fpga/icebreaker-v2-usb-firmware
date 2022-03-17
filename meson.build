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