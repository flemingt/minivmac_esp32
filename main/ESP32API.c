/*
 Copyright (C) 2025  <uliuc@gmx.net >
 Partially based on code by Tara K. https://github.com/TaraHoleInIt/MinivMacArduino

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

#include <string.h>

#include "ESP32API.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "esp_lv_adapter.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "SYSDEPNS.h"

static const char* TAG = "ESP32API";

// mouse and keyboard functions
FORWARDPROC DoKeyCode(int key_code, blnr down);
static uart_port_t s_rx_uart;
static int s_rx_tx; // S3 TX (geht zum ESP32 RX)
static int s_rx_rx; // S3 RX (kommt vom ESP32 TX)

// combine mouse events
static int g_mouse_dx = 0;
static int g_mouse_dy = 0;
static uint8_t g_mouse_buttons = 0;
static bool mouse_btn_state = false;
static int g_em_mouse_x = 256;
static int g_em_mouse_y = 171;

static SemaphoreHandle_t g_mouse_mutex;
static SemaphoreHandle_t upd_area_mutex = NULL;

void hid_link_uart_init_rx(uart_port_t uart, int tx_pin, int rx_pin, int baud)
{
    s_rx_uart = uart;
    s_rx_tx = tx_pin;
    s_rx_rx = rx_pin;
    
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(uart, &cfg);
    uart_set_pin(uart, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart, 2048, 0, 0, NULL, ESP_INTR_FLAG_IRAM);
}

bool hid_link_data_available(size_t min_bytes)
{
    size_t avail = 0;
    if (uart_get_buffered_data_len(s_rx_uart, &avail) == ESP_OK) {
        return avail >= min_bytes; 
    }
    return false;
}

bool hid_link_read_packet(char *type_out, uint8_t *buf, uint16_t buf_size, uint16_t *len_out, TickType_t timeout_ticks)
{
    if (!type_out || !buf || !len_out) return false;
    
    uint8_t hdr[3];
    int n = uart_read_bytes(s_rx_uart, hdr, 3, timeout_ticks);
    if (n != 3) return false;
    
    char type = (char)hdr[0];
    if (type != 'M' && type != 'K') {
        // unkown type
        return false;
    }
    
    uint16_t len = (uint16_t)(hdr[1] | (hdr[2] << 8));
    
    if (len > buf_size) {
        uint16_t remaining = len + 1; // +1 for checksum
        uint8_t dump[64];
        while (remaining) {
            int chunk = remaining > sizeof(dump) ? sizeof(dump) : remaining;
            int got = uart_read_bytes(s_rx_uart, dump, chunk, pdMS_TO_TICKS(10));
            if (got <= 0) break;
            remaining -= (uint16_t)got;
        }
        return false;
    }
    
    // read payload
    int got = uart_read_bytes(s_rx_uart, buf, len, timeout_ticks);
    if (got != len) return false;
    
    // read checksum
    uint8_t rx_chk = 0;
    got = uart_read_bytes(s_rx_uart, &rx_chk, 1, timeout_ticks);
    if (got != 1) return false;
    
    // XOR over header and payload
    uint8_t calc = hdr[0] ^ hdr[1] ^ hdr[2];
    for (uint16_t i = 0; i < len; ++i) {
        calc ^= buf[i];
    }
    
    if (calc != rx_chk) {
        // wrong checksum -> ignore packet
        return false;
    }
    
    *type_out = type;
    *len_out = len;
    return true;
}

// button helper
bool mouse_btn_left(uint8_t b)  { return (b & 0x01) != 0; }
bool mouse_btn_right(uint8_t b) { return (b & 0x02) != 0; }

void dump_raw(const uint8_t *data, uint16_t len)
{
    char buf[256];
    int pos = 0;

    pos += snprintf(buf + pos, sizeof(buf) - pos, "RAW (%d): ", len);

    for (int i = 0; i < len; i++) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X ", data[i]);
        if (pos >= sizeof(buf)) break;
    }

    ESP_LOGD("RAW", "%s", buf);
}

// mouse event parser 5 Bytes: [buttons][dx_lo][dx_hi][dy_lo][dy_hi]
bool mouse_parse_rel16_le(const uint8_t *data, uint16_t len, int16_t *dx, int16_t *dy, uint8_t *buttons)
{
    if (!data || !dx || !dy || !buttons) return false;
    if (len < 5) return false;
    
    *buttons = data[0];
    
    uint16_t x_u = (uint16_t)data[1] | ((uint16_t)data[2] << 8); // little-endian
    uint16_t y_u = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
   
    *dx = (int16_t)x_u; // to signed
    *dy = (int16_t)y_u;
    
    return true;
}

// keyboard event parser: hid standard keyboard descriptor
bool keyboard_parse(const uint8_t *data, uint16_t len, int16_t *keycode, int16_t *modifier) 
{
    if (len >=3) {
        
        *modifier = (int16_t)data[0];
        *keycode = (int16_t)data[2];
        
    } else return false;

    return true;
}
    
static int16_t prev_keycode = 0; 
static int16_t prev_modifier = 0;
    
void mousekeyboard_task(void *arg)
{
    g_mouse_mutex = xSemaphoreCreateMutex();
    if (!g_mouse_mutex) {
        ESP_LOGE(TAG, "mouse mutex creation failed");
        vTaskDelete(NULL);
        return;
    }

    while (1) {

        if (hid_link_data_available(3)) {

            char type;
            uint16_t len = 0;
            uint8_t payload[64];
            uint8_t buttons = 0;
            int16_t dx = 0;
            int16_t dy = 0;

            // non-blocking read, last argument timeout=0
            if (hid_link_read_packet(&type, payload, sizeof(payload), &len, 0)) {

                if (type == 'M') {
                    
                    dump_raw(payload, len); 
                    
                    if (mouse_parse_rel16_le(payload, len, &dx, &dy, &buttons)) {

                        ESP_LOGD(TAG, "X:%d, Y:%d, B:%d", dx, dy, buttons);

                        // accumulate movement
                        if (xSemaphoreTake(g_mouse_mutex, portMAX_DELAY)) {
                            g_mouse_dx += dx;
                            g_mouse_dy += dy;
                            g_mouse_buttons = buttons;
                            xSemaphoreGive(g_mouse_mutex);
                        }
                    }
                }
                else if (type == 'K') {
                    
                    ESP_LOGI(TAG, "keyboard event");
                    dump_raw(payload, len);
                    int16_t keycode = 0;
                    int16_t modifier = 0;
                    
                    if (keyboard_parse(payload, len, &keycode, &modifier)) {
                        
                        ESP_LOGD(TAG, "Keycode: %02X, Modifier: %02X", keycode, modifier);
                    
                        // modifier are own keys
                        if (modifier == 0) {
                            if (prev_modifier > 0) {
                                DoKeyCode(prev_modifier, falseblnr);
                                prev_modifier = 0;
                            }
                        } else {
                            if (modifier & 0x02) {
                            
                                // left shift keydown
                                DoKeyCode(0xE1, trueblnr);
                                prev_modifier = 0xE1;
                            } 
                            else if (modifier & 0x20) {
                                // right shift keydown
                                DoKeyCode(0xE5, trueblnr);
                                prev_modifier = 0xE5;
                            }
                            else if (modifier & 0x01) {
                                // left control keydown
                                DoKeyCode(0xE0, trueblnr);
                                prev_modifier = 0xE0;
                            }
                            else if (modifier & 0x01) {
                                // right control keydown
                                DoKeyCode(0xE4, trueblnr);
                                prev_modifier = 0xE4;
                            }
                            else if (modifier & 0x04) {
                                // left option keydown (left alt)
                                DoKeyCode(0xE2, trueblnr);
                                prev_modifier = 0xE2;
                            }
                            else if (modifier & 0x40) {
                                // right option keydown (right alt)
                                DoKeyCode(0xE6, trueblnr);
                                prev_modifier = 0xE6;
                            }
                            else if (modifier & 0x08) {
                                // left command keydown (left command/gui/meta)
                                DoKeyCode(0xE3, trueblnr);
                                prev_modifier = 0xE3;
                            }
                            else if (modifier & 0x80) {
                                // right command keydown (right command/gui/meta)
                                DoKeyCode(0xE7, trueblnr);
                                prev_modifier = 0xE7;
                            }
                        }
                    
                        if (keycode == 0) {
                            if (prev_keycode > 0) {
                                DoKeyCode(prev_keycode, falseblnr);
                                prev_keycode = 0;
                            }
                        } else { 
                            DoKeyCode(keycode, trueblnr);
                            prev_keycode = keycode;
                        }
                    }
                }
            }
        }

        // small delay to avoid starving other tasks
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// graphics
// mac display size
#define EMU_WIDTH   512
#define EMU_HEIGHT  342
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 480
#define DISPLAY_WIDTH SCREEN_WIDTH
#define DISPLAY_HEIGHT ((EMU_HEIGHT * SCREEN_WIDTH) / EMU_WIDTH)
#define X_OFF 0
#define Y_OFF ((SCREEN_HEIGHT - DISPLAY_HEIGHT) / 2)

#define GT911_I2C_PORT I2C_NUM_0
#define GT911_I2C_ADDRESS 0x5D
#define GT911_STATUS_REGISTER 0x814E
#define GT911_POINT_REGISTER (GT911_STATUS_REGISTER + 1)

static SemaphoreHandle_t newframe_sem = NULL;
const uint8_t* mac_fb = NULL;
static lv_color_t *emu_buf;
static lv_img_dsc_t emu_img_dsc;
static lv_obj_t *emu_img; 
static bool first_mac_frame_logged = false;
static bool first_display_wake_logged = false;
static bool first_display_lock_logged = false;
static bool first_display_update_logged = false;

// lookup-table
uint16_t lut[256][8]; // 256 Bytes for 8 Pixel each

// update area: minivmac sends info about partial screen updates
static int upd_area_l = EMU_WIDTH/2;
static int upd_area_t = EMU_HEIGHT/2;
static int upd_area_r = EMU_WIDTH/2;
static int upd_area_b = EMU_HEIGHT/2;
static int update_event_count = 0;

static esp_err_t gt911_read(uint16_t reg, uint8_t *data, size_t length)
{
    uint8_t address[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
    return i2c_master_write_read_device(GT911_I2C_PORT, GT911_I2C_ADDRESS,
                                        address, sizeof(address), data, length,
                                        pdMS_TO_TICKS(20));
}

static esp_err_t gt911_clear_status(void)
{
    uint8_t command[3] = {
        (uint8_t)(GT911_STATUS_REGISTER >> 8),
        (uint8_t)GT911_STATUS_REGISTER,
        0
    };
    return i2c_master_write_to_device(GT911_I2C_PORT, GT911_I2C_ADDRESS,
                                      command, sizeof(command),
                                      pdMS_TO_TICKS(20));
}

static void touch_mouse_task(void *arg)
{
    (void)arg;
    bool previous_touch = false;
    int previous_x = 0;
    int previous_y = 0;

    while (true) {
        uint8_t status = 0;
        if (gt911_read(GT911_STATUS_REGISTER, &status, 1) == ESP_OK &&
            (status & 0x80) != 0) {
            uint8_t point[8] = {0};
            uint8_t point_count = status & 0x0F;
            bool valid_point = point_count > 0 && point_count <= 5 &&
                               gt911_read(GT911_POINT_REGISTER, point,
                                          sizeof(point)) == ESP_OK;

            if (valid_point) {
                int touch_x = (int)point[1] | ((int)point[2] << 8);
                int touch_y = (int)point[3] | ((int)point[4] << 8);
                bool inside_mac_screen = touch_x < SCREEN_WIDTH &&
                                         touch_y >= Y_OFF &&
                                         touch_y < Y_OFF + DISPLAY_HEIGHT;

                if (inside_mac_screen) {
                    int mac_x = touch_x * EMU_WIDTH / DISPLAY_WIDTH;
                    int mac_y = (touch_y - Y_OFF) * EMU_HEIGHT / DISPLAY_HEIGHT;

                    if (xSemaphoreTake(g_mouse_mutex, portMAX_DELAY)) {
                        int base_x = previous_touch ? previous_x : g_em_mouse_x;
                        int base_y = previous_touch ? previous_y : g_em_mouse_y;
                        g_mouse_dx += mac_x - base_x;
                        g_mouse_dy += mac_y - base_y;
                        g_mouse_buttons = 1;
                        xSemaphoreGive(g_mouse_mutex);
                    }
                    previous_x = mac_x;
                    previous_y = mac_y;
                    previous_touch = true;
                } else {
                    previous_touch = false;
                    if (xSemaphoreTake(g_mouse_mutex, portMAX_DELAY)) {
                        g_mouse_buttons = 0;
                        xSemaphoreGive(g_mouse_mutex);
                    }
                }
            } else {
                previous_touch = false;
                if (xSemaphoreTake(g_mouse_mutex, portMAX_DELAY)) {
                    g_mouse_buttons = 0;
                    xSemaphoreGive(g_mouse_mutex);
                }
            }

            gt911_clear_status();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void display_task(void* Param) {
    ESP_LOGI(TAG, "Display task started; newframe semaphore=%p", newframe_sem);

    while (true) {
      
        ulTaskNotifyTake(pdTRUE, 0);
      
        // wait for new frame
        if (xSemaphoreTake(newframe_sem, portMAX_DELAY) != pdTRUE) {
                ESP32API_Yield();
                continue;
        }

        if (!first_display_wake_logged) {
            ESP_LOGI(TAG, "Display task received frame: fb=%p, canvas=%p, image=%p",
                 mac_fb, emu_buf, emu_img);
            first_display_wake_logged = true;
        }
      
        // this is for full frame update
        /*
        if (mac_fb) {
        
            lv_color_t* dst_fb = emu_buf;
    
            for (int y = 0; y < EMU_HEIGHT; y++) {
        
                const uint8_t* src = mac_fb + y * (EMU_WIDTH / 8);
        
                lv_color_t* dst = &dst_fb[y * EMU_WIDTH];
        
                for (int bx = 0; bx < EMU_WIDTH / 8; bx++) {
        
                    const uint8_t b = src[bx];
                    const uint16_t* p = lut[b];    // 8  RGB565 pixel
        
                    dst[0].full = p[0];
                    dst[1].full = p[1];
                    dst[2].full = p[2];
                    dst[3].full = p[3];
                    dst[4].full = p[4];
                    dst[5].full = p[5];
                    dst[6].full = p[6];
                    dst[7].full = p[7];
        
                    dst += 8;
                }
            }      
            
            esp_lv_adapter_lock(-1);
            lv_obj_invalidate(emu_img);     
            lv_refr_now(NULL);              
            esp_lv_adapter_unlock();
            
            // wait for vsync
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }*/
    
        // partial frame update
        if (mac_fb) {

            lv_color_t* dst_fb = emu_buf;
        
            // clamp area
            int x1 = 0;
            int x2 = 0;
            int y1 = 0;
            int y2 = 0;
            
            if (xSemaphoreTake(upd_area_mutex, portMAX_DELAY)) {
                y1 = upd_area_t;
                y2 = upd_area_b;
                x1 = upd_area_l;
                x2 = upd_area_r;
                update_event_count = 0;
                xSemaphoreGive(upd_area_mutex);
            }
        
            // Clamp the changed Mac region before scaling it to the panel.
            if (y1 < 0) y1 = 0;
            if (y2 >= EMU_HEIGHT) y2 = EMU_HEIGHT - 1;
            if (x1 < 0) x1 = 0;
            if (x2 >= EMU_WIDTH) x2 = EMU_WIDTH - 1;

            int x1_scaled = x1 * DISPLAY_WIDTH / EMU_WIDTH;
            int x2_scaled = ((x2 + 1) * DISPLAY_WIDTH + EMU_WIDTH - 1) / EMU_WIDTH - 1;
            int y1_scaled = y1 * DISPLAY_HEIGHT / EMU_HEIGHT;
            int y2_scaled = ((y2 + 1) * DISPLAY_HEIGHT + EMU_HEIGHT - 1) / EMU_HEIGHT - 1;

            for (int y = y1_scaled; y <= y2_scaled; y++) {
                int source_y = y * EMU_HEIGHT / DISPLAY_HEIGHT;
                lv_color_t *dst = &dst_fb[y * DISPLAY_WIDTH + x1_scaled];

                for (int x = x1_scaled; x <= x2_scaled; x++) {
                    int source_x = x * EMU_WIDTH / DISPLAY_WIDTH;
                    uint8_t source_byte = mac_fb[source_y * (EMU_WIDTH / 8) + source_x / 8];
                    dst->full = lut[source_byte][source_x & 7];
                    dst++;
                }
            }
        
            // invalidate area
            lv_area_t area;
            area.x1 = x1_scaled + X_OFF;
            area.x2 = x2_scaled + X_OFF;
            area.y1 = y1_scaled + Y_OFF;
            area.y2 = y2_scaled + Y_OFF;

            if (!first_display_lock_logged) {
                ESP_LOGI(TAG, "Updating Mac image area (%d,%d)-(%d,%d)",
                         area.x1, area.y1, area.x2, area.y2);
            }
            
            ESP_LOGD(TAG, "X1: %d, Y1: %d, X2: %d, Y2: %d", area.x1, area.y1, area.x2, area.y2);
        
            esp_lv_adapter_lock(-1);
            if (!first_display_lock_logged) {
                ESP_LOGI(TAG, "LVGL lock acquired for Mac image");
                first_display_lock_logged = true;
            }
            lv_obj_invalidate_area(emu_img, &area);
            lv_refr_now(NULL);
            esp_lv_adapter_unlock();

            if (!first_display_update_logged) {
                ESP_LOGI(TAG, "Rendered first Mac framebuffer update");
                first_display_update_logged = true;
            }
        
            vTaskDelay(pdMS_TO_TICKS(16));
        }
    }
}

void create_1bpp_loopup_table() {
    // create lookup table for mac framebuffer conversion
    for (int byte = 0; byte < 256; byte++) {
        for (int b = 0; b < 8; b++) {
            // bit proof
            lut[byte][b]  = (byte & (0x80 >> b)) ? 0x0000 : 0xFFFF;
        }
    }
}

void lvgl_emubuffer_init()
{
    emu_buf = (lv_color_t *)malloc(DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(lv_color_t));

    if (!emu_buf) {
        ESP_LOGE(TAG, "could not initialize emu buffer");
        return;
    }
    memset(emu_buf, 0xff, DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(lv_color_t));

    emu_img_dsc.header.always_zero = 0;
    emu_img_dsc.header.w = DISPLAY_WIDTH;
    emu_img_dsc.header.h = DISPLAY_HEIGHT;
    emu_img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;          // RGB565
    emu_img_dsc.data = (const uint8_t *)emu_buf;
    emu_img_dsc.data_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(lv_color_t);

    esp_lv_adapter_lock(-1);
    emu_img = lv_img_create(lv_scr_act());
    if (!emu_img) {
        esp_lv_adapter_unlock();
        ESP_LOGE(TAG, "could not initialize emu image");
        return;
    }

    lv_img_set_src(emu_img, &emu_img_dsc);
    lv_obj_set_style_opa(emu_img, LV_OPA_COVER, 0);
    lv_obj_set_pos(emu_img, X_OFF, Y_OFF);
    esp_lv_adapter_unlock();
}

void init_vmacmini_esp32() {

    // init SPIFFS 
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",          // mount path
        .partition_label = NULL,         // standard spiffs partition
        .max_files = 10,                 // max opened files
        .format_if_mount_failed = false  // do not format
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));    
    }

    // some info
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS total=%zu, used=%zu", total, used);
    } else {
        ESP_LOGW(TAG, "SPIFFS info failed: %s", esp_err_to_name(ret));
    }

    // create conversion lookup table for monochrome mac framebuffer
    create_1bpp_loopup_table();
    
    // init lvgl canvas
    lvgl_emubuffer_init();
  
    // new frame semaphore
    newframe_sem = xSemaphoreCreateBinary();
    ESP_LOGI(TAG, "Created newframe semaphore=%p", newframe_sem);
    
    // update area mutex
    upd_area_mutex = xSemaphoreCreateMutex();
    
    g_mouse_mutex = xSemaphoreCreateMutex();
    if (g_mouse_mutex) {
        ESP_LOGI(TAG, "starting GT911 touch mouse");
        xTaskCreate(touch_mouse_task, "touch_mouse_task", 4096, NULL, 10, NULL);
    } else {
        ESP_LOGE(TAG, "touch mouse mutex creation failed");
    }
 }

void ESP32API_GetMouseDelta(int* dx, int* dy) {

    *dx = 0;
    *dy = 0;

    if (!g_mouse_mutex)
        return;

    if (xSemaphoreTake(g_mouse_mutex, portMAX_DELAY)) {
        *dx = g_mouse_dx;
        *dy = g_mouse_dy;

        // reset after reading
        g_mouse_dx = 0;
        g_mouse_dy = 0;

        // button state
        mouse_btn_state = mouse_btn_left(g_mouse_buttons);

        xSemaphoreGive(g_mouse_mutex);
    }      
}

void ESP32API_GiveEmulatedMouseToESP32(int* EmMouseX, int* EmMouseY) {
    if (!g_mouse_mutex || !EmMouseX || !EmMouseY)
        return;

    if (xSemaphoreTake(g_mouse_mutex, portMAX_DELAY)) {
        g_em_mouse_x = *EmMouseX;
        g_em_mouse_y = *EmMouseY;
        xSemaphoreGive(g_mouse_mutex);
    }
}

int ESP32API_GetMouseButton(void) {
    int current_val = mouse_btn_state ? 1 : 0;
    
    return current_val;
}

uint64_t ESP32API_GetTimeMS(void)
{
    return 1591551981844ULL + (esp_timer_get_time() / 1000ULL);
}

void ESP32API_Yield(void)
{
    taskYIELD();
}

void ESP32API_Delay(uint32_t MSToDelay) {
    vTaskDelay(pdMS_TO_TICKS(MSToDelay)); 
}

ESP32File ESP32API_open(const char* Path, const char* Mode)
{
    char SPIFFSPath[256];
    snprintf(SPIFFSPath, sizeof(SPIFFSPath), "/spiffs/%s", Path);
    return (ESP32File) fopen(SPIFFSPath, Mode);
}

void ESP32API_close(ESP32File Handle)
{
    if (Handle) {
        fclose((FILE*) Handle);
    }
}

size_t ESP32API_read(void* Buffer, size_t Size, size_t Nmemb, ESP32File Handle)
{
    size_t BytesRead = 0;
    if (Handle) {
        BytesRead = fread(Buffer, Size, Nmemb, (FILE*) Handle);
    }

    return BytesRead;
}

size_t ESP32API_write(const void* Buffer, size_t Size, size_t Nmemb, ESP32File Handle)
{
    size_t BytesWritten = 0;
    if (Handle) {
        BytesWritten = fwrite(Buffer, Size, Nmemb, (FILE*) Handle);
    }
    return BytesWritten;
}

long ESP32API_tell(ESP32File Handle)
{
    long Offset = 0; // maybe -1
    if (Handle) {
        Offset = ftell((FILE*) Handle);
    }
    return Offset;
}

long ESP32API_seek(ESP32File Handle, long Offset, int Whence)
{
    if (Handle) {
        return fseek((FILE*) Handle, Offset, Whence);
    }
    return -1;
}

int ESP32API_eof(ESP32File Handle)
{
    if (Handle) {
        return feof((FILE*)Handle);
    }
    return 0;
}

void* ESP32API_malloc(size_t Size) {
    return heap_caps_malloc(Size, (Size >= 262144) ? MALLOC_CAP_SPIRAM : MALLOC_CAP_DEFAULT);
}

void* ESP32API_calloc(size_t Nmemb, size_t Size) {
    return heap_caps_calloc(Nmemb, Size, ((Size * Nmemb) >= 262144) ? MALLOC_CAP_SPIRAM : MALLOC_CAP_DEFAULT);
}

void ESP32API_free(void* Memory) {
    heap_caps_free(Memory);
}

void ESP32API_CheckForEvents( void ) {
 // not used
}

static bool Changed = false;

void ESP32API_ScreenChanged(int top, int left, int bottom, int right) {
   if (xSemaphoreTake(upd_area_mutex, portMAX_DELAY)) {
        
        if (update_event_count == 0) {
       
            update_event_count++; 
            upd_area_t = top;
            upd_area_l = left;
            upd_area_r = right;
            upd_area_b = bottom;
            
        } else {
        
            // screen not updated in time, expand events
            if (left < upd_area_l) upd_area_l = left;
            if (top < upd_area_t) upd_area_t = top;
            if (right > upd_area_r) upd_area_r = right;
            if (bottom > upd_area_b) upd_area_b = bottom;
       }
       
       Changed = true;
       xSemaphoreGive(upd_area_mutex);
    }
        
    ESP_LOGD(TAG, "T: %d, L: %d, B: %d, R: %d", top, left, bottom, right);
}

void ESP32API_DrawScreen(const uint8_t* new_fb) {
    if (Changed) {
        mac_fb = new_fb;
        Changed = false;

        if (!first_mac_frame_logged) {
            ESP_LOGI(TAG, "Received first Mac framebuffer update: fb=%p, semaphore=%p",
                     new_fb, newframe_sem);
            first_mac_frame_logged = true;
        }

        BaseType_t given = xSemaphoreGive(newframe_sem);
        if (first_mac_frame_logged && given != pdTRUE) {
            ESP_LOGW(TAG, "Could not signal display task for Mac framebuffer");
        }
    }
}