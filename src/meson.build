name = 'icebreaker-ch32v307-usb'
inc = include_directories('.')
src = files([
  'cdc_acm_template.c',
  'main.c',
  'ch32v30x_it.c',
  'system_ch32v30x.c',
])
exe = executable(
  name + '.elf',
  [src, core_src, debug_src, peripheral_src, startup_src, cherryusb_src],
  include_directories : [inc, core_inc, debug_inc, peripheral_inc, cherryusb_inc],
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