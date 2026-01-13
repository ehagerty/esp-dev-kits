/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: LicenseRef-Espressif-Modified-MIT
 *
 * Auto-generated device configuration file
 * DO NOT MODIFY THIS FILE MANUALLY
 *
 * See LICENSE file for details.
 */

#include <stdlib.h>
#include "esp_board_device.h"
#include "dev_audio_codec.h"
#include "dev_custom.h"
#include "dev_display_lcd_spi.h"
#include "dev_lcd_touch_i2c.h"
#include "gen_board_device_custom.h"

// Device configuration structures
const static dev_display_lcd_spi_config_t esp_bmgr_display_lcd_cfg = {
    .name = "display_lcd",
    .chip = "ili9341",
    .type = "display_lcd_spi",
    .spi_name = "spi_display",
    .io_spi_config = {
        .cs_gpio_num = 25,
        .dc_gpio_num = 26,
        .spi_mode = 3,
        .pclk_hz = 20000000,
        .trans_queue_depth = 2,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .flags = {
            .dc_high_on_cmd = false,
            .dc_low_on_data = false,
            .dc_low_on_param = false,
            .octal_mode = false,
            .quad_mode = false,
            .sio_mode = true,
            .lsb_first = false,
            .cs_high_active = false,
        },
    },
    .panel_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
        .flags = {
            .reset_active_high = true,
        },
        .vendor_config = NULL,
    },
    .swap_xy = true,
    .mirror_x = true,
    .mirror_y = false,
    .x_max = 284,
    .y_max = 240,
    .invert_color = false,
    .need_reset = true,
};

const static dev_lcd_touch_i2c_config_t esp_bmgr_lcd_touch_cfg = {
    .name = "lcd_touch",
    .chip = "cst816s",
    .type = "lcd_touch_i2c",
    .i2c_name = "i2c_master",
    .i2c_addr = {0x15, 0x00},
    .io_i2c_config = {
        .dev_addr = 21,
        .control_phase_bytes = 1,
        .dc_bit_offset = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 0,
        .scl_speed_hz = 100000,
        .flags = {
            .dc_low_on_data = false,
            .disable_control_phase = true,
        },
    },
    .touch_config = {
        .x_max = 240,
        .y_max = 284,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = true,
            .mirror_x = true,
            .mirror_y = true,
        },
        .process_coordinates = NULL,
        .interrupt_callback = NULL,
        .user_data = NULL,
        .driver_data = NULL,
    },
};

const static dev_custom_touch_button_config_t esp_bmgr_touch_button_cfg = {
    .name = "touch_button",
    .type = "custom",
    .chip = "bs8112",
    .peripheral_count = 1,
    .peripheral_name = "i2c_master",
};

const static dev_audio_codec_config_t esp_bmgr_fake_audio_dac_cfg = {
    .name = "fake_audio_dac",
    .chip = "none",
    .type = "audio_codec",
    .adc_enabled = false,
    .adc_max_channel = 0,
    .adc_channel_mask = 0x3,
    .adc_channel_labels = "",
    .adc_init_gain = 0,
    .dac_enabled = false,
    .dac_max_channel = 0,
    .dac_channel_mask = 0x0,
    .dac_init_gain = 0,
    .pa_cfg = {
        .name = "none",
        .port = -1,
        .active_level = 0,
        .gain = 0.0,
    },
    .i2c_cfg = {
        .name = "",
        .port = 0,
        .address = 48,
        .frequency = 400000,
    },
    .i2s_cfg = {
        .name = "",
        .port = 0,
    },
    .metadata = NULL,
    .metadata_size = 0,
    .mclk_enabled = false,
    .aec_enabled = false,
    .eq_enabled = false,
    .alc_enabled = false,
};

const static dev_custom_audio_dac_config_t esp_bmgr_audio_dac_cfg = {
    .name = "audio_dac",
    .type = "custom",
    .chip = "none",
    .speaker_n_io = 8,
    .speaker_p_io = 7,
    .sample_rate_hz = 16000,
};

const static dev_custom_audio_adc_config_t esp_bmgr_audio_adc_cfg = {
    .name = "audio_adc",
    .type = "custom",
    .chip = "none",
    .adc_channel = 5,
    .sample_rate_hz = 16000,
};

// Device descriptor array
const esp_board_device_desc_t g_esp_board_devices[] = {
    {
        .next = &g_esp_board_devices[1],
        .name = "display_lcd",
        .type = "display_lcd_spi",
        .cfg = &esp_bmgr_display_lcd_cfg,
        .cfg_size = sizeof(esp_bmgr_display_lcd_cfg),
        .init_skip = false,
    },
    {
        .next = &g_esp_board_devices[2],
        .name = "lcd_touch",
        .type = "lcd_touch_i2c",
        .cfg = &esp_bmgr_lcd_touch_cfg,
        .cfg_size = sizeof(esp_bmgr_lcd_touch_cfg),
        .init_skip = false,
    },
    {
        .next = &g_esp_board_devices[3],
        .name = "touch_button",
        .type = "custom",
        .cfg = &esp_bmgr_touch_button_cfg,
        .cfg_size = sizeof(esp_bmgr_touch_button_cfg),
        .init_skip = false,
    },
    {
        .next = &g_esp_board_devices[4],
        .name = "fake_audio_dac",
        .type = "audio_codec",
        .cfg = &esp_bmgr_fake_audio_dac_cfg,
        .cfg_size = sizeof(esp_bmgr_fake_audio_dac_cfg),
        .init_skip = true,
    },
    {
        .next = &g_esp_board_devices[5],
        .name = "audio_dac",
        .type = "custom",
        .cfg = &esp_bmgr_audio_dac_cfg,
        .cfg_size = sizeof(esp_bmgr_audio_dac_cfg),
        .init_skip = false,
    },
    {
        .next = NULL,
        .name = "audio_adc",
        .type = "custom",
        .cfg = &esp_bmgr_audio_adc_cfg,
        .cfg_size = sizeof(esp_bmgr_audio_adc_cfg),
        .init_skip = false,
    },
};
