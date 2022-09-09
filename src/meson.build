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

inc = include_directories([
  '.', 
  'tusb',
  'tusb/portable/wch/ch32v307',
  'platform'
])
src = files([
  'ch32v30x_it.c',
  'main.c',
  'system_ch32v30x.c',
  'flash.c',
  'time.c',
  'tusb/usb_descriptors.c',
  'tusb/portable/wch/ch32v307/usb_dc_usbhs.c',
  'platform/hw_1.99a.c'
])
