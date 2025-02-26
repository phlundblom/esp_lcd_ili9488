/*
 * SPDX-FileCopyrightText: 2022 atanisoft (github.com/atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#include "core/lv_obj.h"
#include "core/lv_obj_style_gen.h"
#include "display/lv_display.h"
#include "draw/lv_draw_buf.h"
#include "draw/lv_draw_rect.h"
#include "misc/lv_color.h"
#include "sdkconfig.h"
#include "widgets/scale/lv_scale.h"
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <esp_freertos_hooks.h>
#include <esp_lcd_ili9488.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lvgl.h>
#include <stdio.h>

// Uncomment the following line to enable using double buffering of LVGL color
// data.
// #define USE_DOUBLE_BUFFERING 1

// Funkar med 20MHz
// SPI2 MOSI 11 = 19
// SPI2 MISO 13 = 21
// SPI2 CLK  12 = 23
// SPI2 CS   10 = 24
// SPI2 RS/DC 9 = 18
//
// Funkar med 20MHz, inte bättre, klarar inte 25MHz
// SPI2 MOSI 10 = 19- nästan default, bara flyttat cs
// SPI2 MISO 13 = 21-
// SPI2 CLK  11 = 23-
// SPI2 CS   12 = 24
// SPI2 RS/DC 9 = 18

#define BYTE_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB888))

static const char *TAG = "main";

static const int DISPLAY_HORIZONTAL_PIXELS = 480;
static const int DISPLAY_VERTICAL_PIXELS = 320;
// static const int DISPLAY_COMMAND_BITS = 8;
static const int DISPLAY_COMMAND_BITS = 16;
// static const int DISPLAY_PARAMETER_BITS = 32;
static const int DISPLAY_PARAMETER_BITS = 16;
static const unsigned int DISPLAY_REFRESH_HZ = 20e6;
static const int DISPLAY_SPI_QUEUE_LEN = 10;
static const int SPI_MAX_TRANSFER_SIZE = 32768;

// Default to 25 lines of color data
static const size_t LV_BUFFER_SIZE = DISPLAY_HORIZONTAL_PIXELS * 25;
static const int LVGL_UPDATE_PERIOD_MS = 5;

static const ledc_mode_t BACKLIGHT_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static const ledc_channel_t BACKLIGHT_LEDC_CHANNEL = LEDC_CHANNEL_0;
static const ledc_timer_t BACKLIGHT_LEDC_TIMER = LEDC_TIMER_1;
static const ledc_timer_bit_t BACKLIGHT_LEDC_TIMER_RESOLUTION =
    LEDC_TIMER_10_BIT;
static const uint32_t BACKLIGHT_LEDC_FRQUENCY = 5000;

static esp_lcd_panel_io_handle_t lcd_io_handle = NULL;
static esp_lcd_panel_handle_t lcd_handle = NULL;
static lv_display_t *lv_disp_drv;
static lv_color_t *lv_buf_1 = NULL;
static lv_color_t *lv_buf_2 = NULL;
static lv_obj_t *meter = NULL;
static lv_style_t style_screen;
static lv_obj_t *needle_line;

static void update_meter_value(void *obj, int32_t v) {
  lv_scale_set_line_needle_value(obj, needle_line, 60, v);
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx) {
  lv_display_t *disp_driver = (lv_display_t *)user_ctx;
  lv_disp_flush_ready(disp_driver);
  return false;
}

static void lvgl_flush_cb(lv_display_t *display, const lv_area_t *area,
                          uint8_t *px_map) {
  esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(display);

  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;
  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1,
                            offsety2 + 1, px_map);
}

static void IRAM_ATTR lvgl_tick_cb(void *param) {
  lv_tick_inc(LVGL_UPDATE_PERIOD_MS);
}

static void display_brightness_init(void) {
  const ledc_channel_config_t LCD_backlight_channel = {
      .gpio_num = (gpio_num_t)CONFIG_TFT_BACKLIGHT_PIN,
      .speed_mode = BACKLIGHT_LEDC_MODE,
      .channel = BACKLIGHT_LEDC_CHANNEL,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = BACKLIGHT_LEDC_TIMER,
      .duty = 0,
      .hpoint = 0,
      .flags = {.output_invert = 0}};
  const ledc_timer_config_t LCD_backlight_timer = {
      .speed_mode = BACKLIGHT_LEDC_MODE,
      .duty_resolution = BACKLIGHT_LEDC_TIMER_RESOLUTION,
      .timer_num = BACKLIGHT_LEDC_TIMER,
      .freq_hz = BACKLIGHT_LEDC_FRQUENCY,
      .clk_cfg = LEDC_AUTO_CLK};
  ESP_LOGI(TAG, "Initializing LEDC for backlight pin: %d",
           CONFIG_TFT_BACKLIGHT_PIN);

  ESP_ERROR_CHECK(ledc_timer_config(&LCD_backlight_timer));
  ESP_ERROR_CHECK(ledc_channel_config(&LCD_backlight_channel));
}

void display_brightness_set(int brightness_percentage) {
  if (brightness_percentage > 100) {
    brightness_percentage = 100;
  } else if (brightness_percentage < 0) {
    brightness_percentage = 0;
  }
  ESP_LOGI(TAG, "Setting backlight to %d%%", brightness_percentage);

  // LEDC resolution set to 10bits, thus: 100% = 1023
  uint32_t duty_cycle = (1023 * brightness_percentage) / 100;
  ESP_ERROR_CHECK(
      ledc_set_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL, duty_cycle));
  ESP_ERROR_CHECK(
      ledc_update_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL));
}

void initialize_spi() {
  ESP_LOGI(TAG, "Initializing SPI bus (MOSI:%d, MISO:%d, CLK:%d)",
           CONFIG_SPI_MOSI, CONFIG_SPI_MISO, CONFIG_SPI_CLOCK);
  spi_bus_config_t bus = {
      .mosi_io_num = CONFIG_SPI_MOSI,
      .miso_io_num = CONFIG_SPI_MISO,
      .sclk_io_num = CONFIG_SPI_CLOCK,
      .quadwp_io_num = GPIO_NUM_NC,
      .quadhd_io_num = GPIO_NUM_NC,
      .data4_io_num = GPIO_NUM_NC,
      .data5_io_num = GPIO_NUM_NC,
      .data6_io_num = GPIO_NUM_NC,
      .data7_io_num = GPIO_NUM_NC,
      .max_transfer_sz = SPI_MAX_TRANSFER_SIZE,
      .flags = SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO |
               SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_MASTER,
      .intr_flags = ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM};

  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
}

void initialize_display() {
  const esp_lcd_panel_io_spi_config_t io_config = {
      .cs_gpio_num = CONFIG_TFT_CS_PIN,
      .dc_gpio_num = CONFIG_TFT_DC_PIN,
      .spi_mode = 0,
      .pclk_hz = DISPLAY_REFRESH_HZ,
      .trans_queue_depth = DISPLAY_SPI_QUEUE_LEN,
      .on_color_trans_done = notify_lvgl_flush_ready,
      .user_ctx = lv_disp_drv,
      .lcd_cmd_bits = DISPLAY_COMMAND_BITS,
      .lcd_param_bits = DISPLAY_PARAMETER_BITS,
      .flags = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
          .dc_as_cmd_phase = 0,
          .dc_low_on_data = 0,
          .octal_mode = 0,
          .lsb_first = 0
#else
          .dc_low_on_data = 0,
          .octal_mode = 0,
          .sio_mode = 0,
          .lsb_first = 0,
          .cs_high_active = 0
#endif
      }};

  const esp_lcd_panel_dev_config_t lcd_config = {
      .reset_gpio_num = CONFIG_TFT_RESET_PIN,
      .color_space = CONFIG_DISPLAY_COLOR_MODE,
      .bits_per_pixel = 18,
      .flags = {.reset_active_high = 0},
      .vendor_config = NULL};

  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST,
                                           &io_config, &lcd_io_handle));

  ESP_ERROR_CHECK(esp_lcd_new_panel_ili9488(lcd_io_handle, &lcd_config,
                                            LV_BUFFER_SIZE, &lcd_handle));

  ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd_handle, false));
  ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(lcd_handle, true));
  ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_handle, false, true));
  ESP_ERROR_CHECK(esp_lcd_panel_set_gap(lcd_handle, 0, 0));

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
  ESP_ERROR_CHECK(esp_lcd_panel_disp_off(lcd_handle, false));
#else
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_handle, true));
#endif
}

void initialize_lvgl_pre_driver() {
  ESP_LOGI(TAG, "Initializing LVGL");
  lv_init();

  lv_disp_drv =
      lv_display_create(DISPLAY_HORIZONTAL_PIXELS, DISPLAY_VERTICAL_PIXELS);
}

void initialize_lvgl_post_driver() {
  ESP_LOGI(TAG, "Allocating %zu bytes for LVGL buffer",
           LV_BUFFER_SIZE * sizeof(lv_color_t));
  lv_buf_1 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * BYTE_PER_PIXEL,
                                            MALLOC_CAP_DMA);
#if USE_DOUBLE_BUFFERING
  ESP_LOGI(TAG, "Allocating %zu bytes for second LVGL buffer",
           LV_BUFFER_SIZE * sizeof(lv_color_t));
  lv_buf_2 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * BYTE_PER_PIXEL,
                                            MALLOC_CAP_DMA);
#endif
  ESP_LOGI(TAG, "Initializing %dx%d display", DISPLAY_HORIZONTAL_PIXELS,
           DISPLAY_VERTICAL_PIXELS);
  lv_display_set_flush_cb(lv_disp_drv, lvgl_flush_cb);

  ESP_LOGI(TAG, "Creating LVLG display buffer");
  lv_display_set_buffers(lv_disp_drv, lv_buf_1, lv_buf_2,
                         LV_BUFFER_SIZE * BYTE_PER_PIXEL,
                         LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_display_set_user_data(lv_disp_drv, lcd_handle);

  ESP_LOGI(TAG, "Creating LVGL tick timer");
  const esp_timer_create_args_t lvgl_tick_timer_args = {
      .callback = &lvgl_tick_cb, .name = "lvgl_tick"};
  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(lvgl_tick_timer, LVGL_UPDATE_PERIOD_MS * 1000));
}

void create_demo_ui() {
  lv_obj_t *scr = lv_disp_get_scr_act(NULL);

  // Set the background color of the display to black.
  lv_style_init(&style_screen);
  // lv_style_set_bg_color(&style_screen, lv_color_black());
  lv_style_set_bg_color(&style_screen, lv_color_make(255, 0, 0));
  lv_obj_add_style(lv_scr_act(), &style_screen, LV_STATE_DEFAULT);

  // Create a meter which can be animated.
  meter = lv_scale_create(scr);
  lv_obj_center(meter);
  lv_obj_set_size(meter, 200, 200);

  // Add a scale first
  lv_scale_set_mode(meter, LV_SCALE_MODE_ROUND_OUTER);
  lv_obj_set_style_radius(meter, LV_RADIUS_CIRCLE, 0);
  lv_scale_set_total_tick_count(meter, 100);
  lv_scale_set_major_tick_every(meter, 10);

  lv_scale_section_t *section;

  // Add a blue arc to the start
  section = lv_scale_add_section(meter);
  lv_scale_section_set_range(section, 0, 20);

  /*    lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_BLUE),
   * 0);*/
  /*lv_meter_set_indicator_start_value(meter, indic, 0);*/
  /*lv_meter_set_indicator_end_value(meter, indic, 20);*/

  // Make the tick lines blue at the start of the scale
  /*indic =*/
  /*    lv_meter_add_scale_lines(meter, scale,
   * lv_palette_main(LV_PALETTE_BLUE),*/
  /*                             lv_palette_main(LV_PALETTE_BLUE), false, 0);*/
  /*lv_meter_set_indicator_start_value(meter, indic, 0);*/
  /*lv_meter_set_indicator_end_value(meter, indic, 20);*/

  // Add a red arc to the end
  /*indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_RED),
   * 0);*/
  /*lv_meter_set_indicator_start_value(meter, indic, 80);*/
  /*lv_meter_set_indicator_end_value(meter, indic, 100);*/

  // Make the tick lines red at the end of the scale
  /*indic =*/
  /*    lv_meter_add_scale_lines(meter, scale,
   * lv_palette_main(LV_PALETTE_RED),*/
  /*                             lv_palette_main(LV_PALETTE_RED), false, 0);*/
  /*lv_meter_set_indicator_start_value(meter, indic, 80);*/
  /*lv_meter_set_indicator_end_value(meter, indic, 100);*/

  // Add a needle line indicator
  /*indic = lv_meter_add_needle_line(meter, scale, 4,*/
  /*                                 lv_palette_main(LV_PALETTE_GREY), -10);*/
  needle_line = lv_line_create(meter);
  lv_obj_set_style_line_width(needle_line, 4, LV_PART_MAIN);
  lv_obj_set_style_line_rounded(needle_line, true, LV_PART_MAIN);
  lv_obj_set_style_bg_color(needle_line, lv_palette_main(LV_PALETTE_GREY),
                            LV_PART_MAIN);

  // Create an animation to set the value
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_exec_cb(&a, update_meter_value);
  lv_anim_set_var(&a, meter);
  lv_anim_set_values(&a, 0, 100);
  lv_anim_set_time(&a, 2000);
  lv_anim_set_repeat_delay(&a, 100);
  lv_anim_set_playback_time(&a, 500);
  lv_anim_set_playback_delay(&a, 100);
  lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
  lv_anim_start(&a);
}

void app_main() {
  display_brightness_init();
  display_brightness_set(0);
  initialize_spi();
  initialize_lvgl_pre_driver();
  initialize_display();
  initialize_lvgl_post_driver();
  create_demo_ui();
  display_brightness_set(75);

  // char buf[100 * 2];
  // memset(buf, 0xf0, sizeof(buf));
  //  esp_lcd_panel_draw_bitmap(lcd_handle, 0, 0, 10, 10, buf);
  //  esp_lcd_panel_draw_bitmap(lcd_handle, 100, 100, 110, 110, buf);

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10));
    lv_timer_handler();
  }
}
