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

### Build Options

Options are specified via meson's options, when creating a new build:

```text
meson setup build/ --cross-file riscv-xpack -Doption_name=option_value
```

Change an existing build:

```text
meson --reconfigure build/ -Doption_name=option_value
```

| Option Name   | Option Value                    | Option Description        |
| ------------- | ------------------------------- | ------------------------- |
| board         | ch32v307v-r1, icebreaker-v1.99a | which board/pins are used |
| usb_port      | hs, fs                          | which usb port is used    |
| usb_debug     | error, warning, info, log       | which debug level is used |

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
