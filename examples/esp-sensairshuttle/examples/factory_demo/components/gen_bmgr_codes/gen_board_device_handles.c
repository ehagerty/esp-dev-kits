/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: LicenseRef-Espressif-Modified-MIT
 *
 * Auto-generated device handle definition file
 * DO NOT MODIFY THIS FILE MANUALLY
 *
 * See LICENSE file for details.
 */

#include <stddef.h>
#include "esp_board_device.h"
#include "dev_audio_codec.h"
#include "dev_custom.h"
#include "dev_display_lcd_spi.h"
#include "dev_lcd_touch_i2c.h"

// Device handle array
esp_board_device_handle_t g_esp_board_device_handles[] = {
    {
        .next = &g_esp_board_device_handles[1],
        .name = "display_lcd",
        .type = "display_lcd_spi",
        .device_handle = NULL,
        .init = dev_display_lcd_spi_init,
        .deinit = dev_display_lcd_spi_deinit
    },
    {
        .next = &g_esp_board_device_handles[2],
        .name = "lcd_touch",
        .type = "lcd_touch_i2c",
        .device_handle = NULL,
        .init = dev_lcd_touch_i2c_init,
        .deinit = dev_lcd_touch_i2c_deinit
    },
    {
        .next = &g_esp_board_device_handles[3],
        .name = "touch_button",
        .type = "custom",
        .device_handle = NULL,
        .init = dev_custom_init,
        .deinit = dev_custom_deinit
    },
    {
        .next = &g_esp_board_device_handles[4],
        .name = "fake_audio_dac",
        .type = "audio_codec",
        .device_handle = NULL,
        .init = dev_audio_codec_init,
        .deinit = dev_audio_codec_deinit
    },
    {
        .next = &g_esp_board_device_handles[5],
        .name = "audio_dac",
        .type = "custom",
        .device_handle = NULL,
        .init = dev_custom_init,
        .deinit = dev_custom_deinit
    },
    {
        .next = NULL,
        .name = "audio_adc",
        .type = "custom",
        .device_handle = NULL,
        .init = dev_custom_init,
        .deinit = dev_custom_deinit
    },
};
