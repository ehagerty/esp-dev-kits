/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: LicenseRef-Espressif-Modified-MIT
 *
 * Auto-generated peripheral configuration file
 * DO NOT MODIFY THIS FILE MANUALLY
 *
 * See LICENSE file for details.
 */

#include <stdlib.h>
#include "esp_board_periph.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "hal/gpio_types.h"
#include "periph_gpio.h"
#include "periph_spi.h"

// Peripheral configuration structures
const static i2c_master_bus_config_t esp_bmgr_i2c_master_cfg = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = 2,
    .scl_io_num = 3,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .intr_priority = 1,
    .trans_queue_depth = 0,
    .flags = {
        .enable_internal_pullup = true,
    },
};

static periph_spi_config_t esp_bmgr_spi_display_cfg = {
    .spi_port = SPI2_HOST,
    .spi_bus_config = {
        .mosi_io_num = 23,
        .miso_io_num = -1,
        .sclk_io_num = 24,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .data_io_default_level = false,
        .max_transfer_sz = 3600,
        .flags = 0,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = 0,
    },
};

const static periph_gpio_config_t esp_bmgr_gpio_pa_control_cfg = {
    .gpio_config = {
        .pin_bit_mask = BIT64(1),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    },
    .default_level = 1,
};

// Peripheral descriptor array
const esp_board_periph_desc_t g_esp_board_peripherals[] = {
    {
        .next = &g_esp_board_peripherals[1],
        .name = "i2c_master",
        .type = "i2c",
        .format = NULL,
        .role = "master",
        .cfg = &esp_bmgr_i2c_master_cfg,
        .cfg_size = sizeof(esp_bmgr_i2c_master_cfg),
        .id = 0,
    },
    {
        .next = &g_esp_board_peripherals[2],
        .name = "spi_display",
        .type = "spi",
        .format = NULL,
        .role = "master",
        .cfg = &esp_bmgr_spi_display_cfg,
        .cfg_size = sizeof(esp_bmgr_spi_display_cfg),
        .id = 0,
    },
    {
        .next = NULL,
        .name = "gpio_pa_control",
        .type = "gpio",
        .format = NULL,
        .role = "io",
        .cfg = &esp_bmgr_gpio_pa_control_cfg,
        .cfg_size = sizeof(esp_bmgr_gpio_pa_control_cfg),
        .id = 0,
    },
};
