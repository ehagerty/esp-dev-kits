# USB Camera LCD Example

An example for using LCD display video which captured from USB camera.

* Transfer uvc frame to wifi http if `ENABLE_UVC_WIFI_XFER` is set to `1`, the real-time image can be fetched through Wi-Fi softAP (ssid: ESP32S3-UVC, http: 192.168.4.1).
* Print log about SRAM and PSRAM memory if `LOG_MEM_INFO` is set to `1`, includes `Biggest/Free/Total` three types.

## How to use example

Please first read the [User Guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-lcd-ev-board/user_guide.html#esp32-s3-lcd-ev-board) of the ESP32-S3-LCD-EV-Board to learn about its software and hardware information.

The example can be directly flashed to the board by [ESP Launchpad](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://dl.espressif.com/AE/esp-dev-kits/config.toml).

<a href="https://espressif.github.io/esp-launchpad/?flashConfigURL=https://dl.espressif.com/AE/esp-dev-kits/config.toml&app=usb_camera_lcd">
    <img alt="Try it with ESP Launchpad" src="https://espressif.github.io/esp-launchpad/assets/try_with_launchpad.png" width="200" height="56">
</a>

### ESP-IDF Required

* The ESP-IDF v5.0.1 or later is required to use this example. For using the branch of ESP-IDF, the latest branch `release/v5.1` is recommended. For using the tag of ESP-IDF, the tag `v5.1.2` or later is recommended.
* Please follow the [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/index.html) to set up the development environment.

### Hardware Required

* An ESP32-S3-LCD-EV-Board development board with subboard3 (800x480) or subboard2 (480x480)
* An USB camera that can output 800x480 or 480x320 resolution image
* An USB Type-C cable for Power supply and programming (Please connect to UART port instead of USB port)

### Configurations

Run `idf.py menuconfig` and go to `Board Support Package`.

### Hardware Connection

| USB_DP | USB_DM |
| ------ | ------ |
| GPIO20 | GPIO19 |

**Note:** If the camera is connected to the USB port, diode D1 needs to be short-circuited. About this, please check [Power Supply over USB](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-lcd-ev-board/user_guide.html#power-supply-over-usb) part in board's schematic.

### Build and Flash

1. The project configure PSRAM with 80M DDR(Octal) by default. **Only for the version v1.4 and older boards with ESP32-S3-WROOM-1-N16R8 can enable PSRAM 120M DDR(Octal) feature by the following commands**, see [here](../../README.md#psram-120m-ddr) for more details.
    ```
    rm -rf build sdkconfig sdkconfig.old
    idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.psram_120m_ddr" reconfigure
    ```
2. Run `idf.py -p PORT flash monitor` to build, flash and monitor the project. **Note that it must be connected with UART port instead of USB port.**

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

The following animations show the example running on different development boards.

![usb_camera_lcd_480_480](https://dl.espressif.com/AE/esp-dev-kits/s3-lcd-ev-board_examples_usb_camera_lcd_480_480.gif)

![usb_camera_lcd_800_480](https://dl.espressif.com/AE/esp-dev-kits/s3-lcd-ev-board_examples_usb_camera_lcd_800_480_2.gif)

## Troubleshooting

* Program build failure
    * Error message with `error: static assertion failed: "FLASH and PSRAM Mode configuration are not supported"`: Please check [documentation](https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32s3/api-guides/flash_psram_config.html#all-supported-modes-and-speeds) to make sure the flash and PSRAM mode configuration is correct.
* Program upload failure
    * Hardware connection is not correct: run `idf.py -p PORT monitor`, and reboot your board to see if there are any output logs.
    * The baud rate for downloading is too high: lower your baud rate in the `menuconfig` menu, and try again.
    * Error message with `A fatal error occurred: Could not open /dev/ttyACM0, the port doesn't exist`: Please first make sure the development board connected, then make board into "Download Boot" by following steps:
        1. keep press "BOOT(SW2)" button
        2. short press "RST(SW1)" button
        3. release "BOOT(SW2)".
        4. upload program and reset
* Program runtime failure
    * Abnormal display on the sub-board2 screen (480x480), backlight is on but there is no image displayed: If the log level is configured as "Debug" or lower, please also increase the baud rate of log output as well (e.g., 2000000).
    * Get stuck in the boot process: The latest version (v1.5) of the ESP32-S3-LCD-EV-Board, equipped with the `ESP32-S3-WROOM-1-N16R16V` module, does not currently support the PSRAM 120M DDR(Octal) feature.
    * Warning message with `W (xxx) lcd_panel.io.3wire_spi: Delete but keep CS line inactive`: This is a normal message, please ignore it.

## Technical support and feedback

Please use the following feedback channels:

* For technical queries, go to the [esp32.com](https://esp32.com/) forum
* For a feature request or bug report, create a [GitHub issue](https://github.com/espressif/esp-dev-kits/issues)

We will get back to you as soon as possible.
