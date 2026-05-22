# ESP32-S31-Korvo Development Board

## User Guide

* ESP32-S31-Korvo-1 - [English](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s31/esp32-s31-korvo-1/user_guide.html) / [中文](https://docs.espressif.com/projects/esp-dev-kits/zh_CN/latest/esp32s31/esp32-s31-korvo-1/user_guide.html)

## Examples

The following examples are recommended to be developed with the ESP-IDF **master** branch.

* [Factory Demo](./examples/factory_demo/)

## Notes

* The current `factory_demo` keeps the camera pipeline enabled through the bundled temporary `esp_video` component under `examples/common_components/esp_video` for OV3660 initialization compatibility. It should be replaced when official ESP32-S31 support is available in the upstream component.
