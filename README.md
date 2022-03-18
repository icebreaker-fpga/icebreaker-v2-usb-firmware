# icebreaker-v2-usb-firmware

Firmware to implement USB communications on the CH32V307 microcontroller.

## Prerequisites

- RISCV compiler (xPack/SiFive)
- Meson
- Ninja

## Building

```text
meson setup build/ --cross-file riscv-xpack # or riscv-sifive
meson compile -C build/
```

### High vs Full Speed ports

When creating a new build:

```text
meson setup build/ --cross-file riscv-xpack -Dusb_port=hs # or fs
```

Change an existing build:

```text
meson --reconfigure build/ -Dusb_port=hs # or fs
```

## License

Unless otherwise stated files are licensed as Apache-2.0:

```
   Copyright 2022 Ahmed Charles <me@ahmedcharles.com>
   Copyright 2022 Greg Davill <greg.davill@gmail.com>

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
```
