# icebreaker-v2-usb-firmware

Firmware to implement USB communications on the CH32V307 microcontroller

## Prerequisites

- RISCV compiler (xPack)
- Meson

## Building

```text
meson setup build/ --cross-file riscv-xpack
meson compile -C build/
```

## License

Unless otherwise stated files are licensed as Apache-2.0
