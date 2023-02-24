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

tusb_inc = include_directories([
    'tinyusb_src/src',
    'tinyusb_src/src/device',
    'tinyusb_src/src/common',
    'tinyusb_src/src/class/cdc',
    'tinyusb_src/src/portable/wch/ch32v307',
])
tusb_src = files([
    'tinyusb_src/src/tusb.c',
    'tinyusb_src/src/device/usbd.c',
    'tinyusb_src/src/device/usbd_control.c',
    'tinyusb_src/src/common/tusb_fifo.c',
    'tinyusb_src/src/class/cdc/cdc_device.c',
    'tinyusb_src/src/class/dfu/dfu_device.c',
    'tinyusb_src/src/class/dfu/dfu_rt_device.c',
    'tinyusb_src/src/class/hid/hid_device.c',
    'tinyusb_src/src/class/msc/msc_device.c',
    'tinyusb_src/src/portable/wch/ch32v307/dcd_usbhs.c',
])