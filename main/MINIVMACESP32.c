/*
 Copyright (C) 2025  <uliuc@gmx.net >

 This program is free software; you can redistribute it and/or modify it
 under the terms of the GNU General Public License as published by the
 Free Software Foundation; either version 3 of the License, or (at your
 option) any later version.

 This program is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 for more details.

 For the complete text of the GNU General Public License see
 http://www.gnu.org/licenses/.
*/

#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_io_additions.h"
#include "esp_lcd_st7701.h"

#include "esp_lv_adapter.h"
#include "ESP32API.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
///               Guition ESP32-S3-4848S040 LCD configuration                  //////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LCD_H_RES               480
#define LCD_V_RES               480
#define LCD_BIT_PER_PIXEL       18
#define RGB_BIT_PER_PIXEL       16
#define RGB_DATA_WIDTH          16
#define RGB_BOUNCE_BUFFER_SIZE  LCD_H_RES * 10
#define LCD_IO_RGB_DISP         -1             // -1 if not used
#define LCD_IO_RGB_VSYNC        GPIO_NUM_17
#define LCD_IO_RGB_HSYNC        GPIO_NUM_16
#define LCD_IO_RGB_DE           GPIO_NUM_18
#define LCD_IO_RGB_PCLK         GPIO_NUM_21
#define LCD_IO_RGB_DATA0        GPIO_NUM_11      // B0
#define LCD_IO_RGB_DATA1        GPIO_NUM_12      // B1
#define LCD_IO_RGB_DATA2        GPIO_NUM_13      // B2
#define LCD_IO_RGB_DATA3        GPIO_NUM_14      // B3
#define LCD_IO_RGB_DATA4        GPIO_NUM_0       // B4
#define LCD_IO_RGB_DATA5        GPIO_NUM_8       // G0
#define LCD_IO_RGB_DATA6        GPIO_NUM_20      // G1
#define LCD_IO_RGB_DATA7        GPIO_NUM_3       // G2
#define LCD_IO_RGB_DATA8        GPIO_NUM_46      // G3
#define LCD_IO_RGB_DATA9        GPIO_NUM_9       // G4
#define LCD_IO_RGB_DATA10       GPIO_NUM_10      // G5
#define LCD_IO_RGB_DATA11       GPIO_NUM_4       // R0
#define LCD_IO_RGB_DATA12       GPIO_NUM_5       // R1
#define LCD_IO_RGB_DATA13       GPIO_NUM_6       // R2
#define LCD_IO_RGB_DATA14       GPIO_NUM_7       // R3
#define LCD_IO_RGB_DATA15       GPIO_NUM_15      // R4
#define LCD_IO_SPI_CS           GPIO_NUM_39
#define LCD_IO_SPI_SCL          GPIO_NUM_48
#define LCD_IO_SPI_SDA          GPIO_NUM_47
#define LCD_IO_RST              -1
#define PIN_NUM_BK_LIGHT        GPIO_NUM_38
#define LCD_BK_LIGHT_ON_LEVEL   1
#define LCD_BK_LIGHT_OFF_LEVEL  !LCD_BK_LIGHT_ON_LEVEL
#define LCD_PIXEL_CLOCK_HZ      (12 * 1000 * 1000)

// rotation
#define CONFIG_LCD_ROTATION     ESP_LV_ADAPTER_ROTATE_0

// i2c pins
#define PIN_SCL                 GPIO_NUM_45
#define PIN_SDA                 GPIO_NUM_19
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000

// frame buffer size
#define EMU_WIDTH   512
#define EMU_HEIGHT  342

static const char *TAG = "MINIVMACESP32";

// Guition ESP32-S3-4848S040 ST7701 initialization
static const st7701_lcd_init_cmd_t lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    {0xC0, (uint8_t []){0x3B, 0x00}, 2, 0},
    {0xC1, (uint8_t []){0x0D, 0x02}, 2, 0},
    {0xC2, (uint8_t []){0x31, 0x05}, 2, 0},
    {0xCD, (uint8_t []){0x00}, 1, 0},
    {0xB0, (uint8_t []){0x00, 0x11, 0x18, 0x0E, 0x11, 0x06, 0x07, 0x08, 0x07, 0x22, 0x04, 0x12, 0x0F, 0xAA, 0x31, 0x18}, 16, 0},
    {0xB1, (uint8_t []){0x00, 0x11, 0x19, 0x0E, 0x12, 0x07, 0x08, 0x08, 0x08, 0x22, 0x04, 0x11, 0x11, 0xA9, 0x32, 0x18}, 16, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    {0xB0, (uint8_t []){0x60}, 1, 0},
    {0xB1, (uint8_t []){0x32}, 1, 0},
    {0xB2, (uint8_t []){0x07}, 1, 0},
    {0xB3, (uint8_t []){0x80}, 1, 0},
    {0xB5, (uint8_t []){0x49}, 1, 0},
    {0xB7, (uint8_t []){0x85}, 1, 0},
    {0xB8, (uint8_t []){0x21}, 1, 0},
    {0xC1, (uint8_t []){0x78}, 1, 0},
    {0xC2, (uint8_t []){0x78}, 1, 0},
    {0xE0, (uint8_t []){0x00, 0x1B, 0x02}, 3, 0},
    {0xE1, (uint8_t []){0x08, 0xA0, 0x00, 0x00, 0x07, 0xA0, 0x00, 0x00, 0x00, 0x44, 0x44}, 11, 0},
    {0xE2, (uint8_t []){0x11, 0x11, 0x44, 0x44, 0xED, 0xA0, 0x00, 0x00, 0xEC, 0xA0, 0x00, 0x00}, 12, 0},
    {0xE3, (uint8_t []){0x00, 0x00, 0x11, 0x11}, 4, 0},
    {0xE4, (uint8_t []){0x44, 0x44}, 2, 0},
    {0xE5, (uint8_t []){0x0A, 0xE9, 0xD8, 0xA0, 0x0C, 0xEB, 0xD8, 0xA0, 0x0E, 0xED, 0xD8, 0xA0, 0x10, 0xEF, 0xD8, 0xA0}, 16, 0},
    {0xE6, (uint8_t []){0x00, 0x00, 0x11, 0x11}, 4, 0},
    {0xE7, (uint8_t []){0x44, 0x44}, 2, 0},
    {0xE8, (uint8_t []){0x09, 0xE8, 0xD8, 0xA0, 0x0B, 0xEA, 0xD8, 0xA0, 0x0D, 0xEC, 0xD8, 0xA0, 0x0F, 0xEE, 0xD8, 0xA0}, 16, 0},
    {0xEB, (uint8_t []){0x02, 0x00, 0xE4, 0xE4, 0x88, 0x00, 0x40}, 7, 0},
    {0xEC, (uint8_t []){0x3C, 0x00}, 2, 0},
    {0xED, (uint8_t []){0xAB, 0x89, 0x76, 0x54, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x20, 0x45, 0x67, 0x98, 0xBA}, 16, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
    {0xE5, (uint8_t []){0xE4}, 1, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
    {0x3A, (uint8_t []){0x60}, 1, 10},
    {0x11, (uint8_t []){0x00}, 0, 120},
    {0x29, (uint8_t []){0x00}, 0, 0},
};

// in ESP32API.c   
extern void display_task(void* Param);
   
// starts minivmac in his own task
static void emulator_task(void *param)
{
    minivmac_main(0, NULL); 
    vTaskDelete(NULL);
}   
   
// main
void app_main()
{
    // display init
    // first initialize i2c driver for IO extender
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    
#if PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif

    // init SPI bus
    ESP_LOGI(TAG, "Install 3-wire SPI panel IO");
    spi_line_config_t line_config = {
        .cs_io_type = IO_TYPE_GPIO,
        .cs_gpio_num = LCD_IO_SPI_CS,
        .scl_io_type = IO_TYPE_GPIO,
        .scl_gpio_num = LCD_IO_SPI_SCL,
        .sda_io_type = IO_TYPE_GPIO,
        .sda_gpio_num = LCD_IO_SPI_SDA,
        .io_expander = NULL,
    };
    
    esp_lcd_panel_io_3wire_spi_config_t io_config = ST7701_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
    io_config.spi_mode = 3;
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle));

    // install panel driver
    ESP_LOGI(TAG, "Install ST7701 panel driver");
    esp_lcd_panel_handle_t lcd_handle = NULL;

    // calculate required frame buffer count based on rotation
    esp_lv_adapter_rotation_t rotation = CONFIG_LCD_ROTATION;
    uint8_t num_fbs = esp_lv_adapter_get_required_frame_buffer_count(
                          ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE, rotation);

    esp_lcd_rgb_panel_config_t rgb_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .psram_trans_align = 64,
        .data_width = RGB_DATA_WIDTH,
        .bits_per_pixel = RGB_BIT_PER_PIXEL,
        .de_gpio_num = LCD_IO_RGB_DE,
        .pclk_gpio_num = LCD_IO_RGB_PCLK,
        .vsync_gpio_num = LCD_IO_RGB_VSYNC,
        .hsync_gpio_num = LCD_IO_RGB_HSYNC,
        .disp_gpio_num = LCD_IO_RGB_DISP,
        .data_gpio_nums = {
            LCD_IO_RGB_DATA0,
            LCD_IO_RGB_DATA1,
            LCD_IO_RGB_DATA2,
            LCD_IO_RGB_DATA3,
            LCD_IO_RGB_DATA4,
            LCD_IO_RGB_DATA5,
            LCD_IO_RGB_DATA6,
            LCD_IO_RGB_DATA7,
            LCD_IO_RGB_DATA8,
            LCD_IO_RGB_DATA9,
            LCD_IO_RGB_DATA10,
            LCD_IO_RGB_DATA11,
            LCD_IO_RGB_DATA12,
            LCD_IO_RGB_DATA13,
            LCD_IO_RGB_DATA14,
            LCD_IO_RGB_DATA15,
        },
        .timings = {
            .pclk_hz = LCD_PIXEL_CLOCK_HZ,
            .h_res = LCD_H_RES,
            .v_res = LCD_V_RES, 
            .hsync_back_porch = 50,
            .hsync_front_porch = 10,
            .hsync_pulse_width = 8,
            .vsync_back_porch = 20,
            .vsync_front_porch = 10,
            .vsync_pulse_width = 8,
            .flags.pclk_active_neg = false, },
        .flags.fb_in_psram = 1,
        .num_fbs = num_fbs,
        .bounce_buffer_size_px = RGB_BOUNCE_BUFFER_SIZE,
    };
    rgb_config.timings.h_res = LCD_H_RES;
    rgb_config.timings.v_res = LCD_V_RES;
    st7701_vendor_config_t vendor_config = {
        .rgb_config = &rgb_config,
        .init_cmds = lcd_init_cmds,      
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .auto_del_panel_io = 0,         /**
                                             * Set to 1 if panel IO is no longer needed after LCD initialization.
                                             * If the panel IO pins are sharing other pins of the RGB interface to save GPIOs,
                                             * Please set it to 1 to release the pins.
                                             */
            .mirror_by_cmd = 1,             // Set to 0 if `auto_del_panel_io` is enabled
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_IO_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
        
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7701(io_handle, &panel_config, &lcd_handle));
    
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_handle, true));

    // Initialize LVGL adapter
    ESP_LOGI(TAG, "Initialize LVGL adapter");
    esp_lv_adapter_config_t adapter_config = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(esp_lv_adapter_init(&adapter_config));

    // Register display to LVGL adapter
    ESP_LOGI(TAG, "Register display to LVGL adapter");
    esp_lv_adapter_display_config_t display_config = ESP_LV_ADAPTER_DISPLAY_RGB_DEFAULT_CONFIG(
                                                         lcd_handle,
                                                         io_handle,
                                                         LCD_H_RES,
                                                         LCD_V_RES,
                                                         rotation
                                                     );
    
    display_config.tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE;
                                                     
    lv_display_t *disp = esp_lv_adapter_register_display(&display_config);
    if (disp == NULL) {
        ESP_LOGE(TAG, "Failed to register display");
        return;
    }

    // Start LVGL adapter task
    ESP_LOGI(TAG, "Start LVGL adapter");
    ESP_ERROR_CHECK(esp_lv_adapter_start());

#if PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
#endif

    esp_lv_adapter_lock(-1);
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_make(148, 91, 89), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    esp_lv_adapter_unlock();

    // init emulator
    init_vmacmini_esp32();
    
    // start display on core 1
    BaseType_t display_task_created = xTaskCreatePinnedToCore(
        display_task, "display_task", 4096 * 2, NULL, 6, NULL, 1);
    ESP_LOGI(TAG, "Display task creation result=%d, handle=%p",
             (int)display_task_created, NULL);
        
    // start emulator task with bigger stack
    xTaskCreatePinnedToCore(emulator_task, "emulator_task", 12288, NULL, 5, NULL, 0);       
}
