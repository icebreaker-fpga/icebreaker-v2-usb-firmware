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

cherryusb_inc = include_directories([
  'common',
  'core',
  'port/ch32',
  'class/cdc',
])
cherryusb_src = files([
  'class/cdc/usbd_cdc.c',
  'core/usbd_core.c',
])
if get_option('usb_port') == 'hs'
  cherryusb_src += files('port/ch32/usb_dc_usbhs.c')
elif get_option('usb_port') == 'fs'
  cherryusb_src += files('port/ch32/usb_dc_usbfs.c')
endif
