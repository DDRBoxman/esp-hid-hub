#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "lvgl_demo_ui.h"

#include "seesaw.h"
#include "neokey_4x1.h"
#include "htu31d.h"
#include "usb.h"
#include "stemma_encoder.h"

#include "esp_app_trace.h"

static const char *TAG = "example";

#include "class/hid/hid_device.h"

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/

#define LCD_HOST SPI2_HOST

#define PIN_NUM_SCLK 36
#define PIN_NUM_MOSI 35
#define PIN_NUM_MISO 37
#define PIN_NUM_LCD_DC 40
#define PIN_NUM_LCD_RST 41
#define PIN_NUM_LCD_CS 42
#define PIN_NUM_BK_LIGHT 45
#define LCD_BK_LIGHT_ON_LEVEL 1

#define LCD_H_RES 240
#define LCD_V_RES 135

#define LCD_PIXEL_CLOCK_HZ (7 * 1000 * 1000)
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

#define TFT_I2C_POWER 7

#define I2C_MASTER_SCL_IO 4
#define I2C_MASTER_SDA_IO 3
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 80000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define I2C_ENCODER_ADDR 0x36
#define I2C_NEOKEY_ADDR 0x31

QueueHandle_t hid_queue;

static SemaphoreHandle_t encoder_mutex;
static int32_t encoder_diff = 0;
static uint32_t encoder_pressed = 0;

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void example_lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;

    switch (drv->rotated)
    {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK)
    {
        return err;
    }

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void hid_task(void *parameter)
{
    while (1)
    {
        uint32_t key;
        BaseType_t sucess = xQueueReceive(hid_queue, &key, 0);

        if (sucess && usb_mounted())
        {
            app_send_hid_demo(HID_KEY_1 + key);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void neokey_task(void *parameter)
{
    uint32_t last_buttons = 0;

    uint8_t data_wr[] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};

    while (1)
    {
        uint32_t buttons = neokey_read(I2C_NEOKEY_ADDR);

        uint8_t just_pressed = (buttons ^ last_buttons) & buttons;
        uint8_t just_released = (buttons ^ last_buttons) & ~buttons;
        if (just_pressed | just_released)
        {

            for (int b = 0; b < 4; b++)
            {
                if (just_pressed & (1 << b))
                {
                    xQueueSend(hid_queue, &b, 0);

                    data_wr[(b * 3)] = 0x22;
                }

                if (just_released & (1 << b))
                {
                    data_wr[(b * 3)] = 0x11;
                }
            }
        }

        last_buttons = buttons;

        seesaw_pixel_write(I2C_NEOKEY_ADDR, data_wr, 12);

        uint32_t pressed = stemma_encoder_read_button(I2C_ENCODER_ADDR);

        int32_t diff = stemma_encoder_get_diff(I2C_ENCODER_ADDR);

        xSemaphoreTake(encoder_mutex, portMAX_DELAY);
        encoder_diff += diff;
        encoder_pressed = pressed;
        xSemaphoreGive(encoder_mutex);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void lvgl_task(void *parms)
{
}

bool lvgl_encoder_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    xSemaphoreTake(encoder_mutex, portMAX_DELAY);
    data->enc_diff = encoder_diff;

    encoder_diff = 0;

    if (encoder_pressed != 0)
    {
        data->state = LV_INDEV_STATE_PR;
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }

    xSemaphoreGive(encoder_mutex);

    return false; /*No buffering now so no more data read*/
}

void app_main(void)
{
    esp_log_set_vprintf(esp_apptrace_vprintf);

    hid_queue = xQueueCreate(10, sizeof(uint32_t));
    encoder_mutex = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "Turn on tft");
    gpio_config_t tft_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << TFT_I2C_POWER};
    ESP_ERROR_CHECK(gpio_config(&tft_gpio_config));
    gpio_set_level(TFT_I2C_POWER, LCD_BK_LIGHT_ON_LEVEL);

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Wait for i2c devices to be ready
    vTaskDelay(500 / portTICK_PERIOD_MS);

    neokey_setup(I2C_NEOKEY_ADDR);

    stemma_encoder_setup(I2C_ENCODER_ADDR);

    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_LCD_DC,
        .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,
    };
    // Create LCD panel handle for ST7789, with the SPI IO device handle
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    esp_lcd_panel_set_gap(panel_handle, 40, 53);

    esp_lcd_panel_invert_color(panel_handle, true);

    // Turn on the screen
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = 135;
    disp_drv.ver_res = 240;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.drv_update_cb = example_lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;

    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    lv_disp_set_rotation(disp, LV_DISP_ROT_90);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = lvgl_encoder_read;
    /*Register the driver in LVGL and save the created input device object*/
    lv_indev_t *indev = lv_indev_drv_register(&indev_drv);

    ESP_LOGI(TAG, "Display LVGL Meter Widget");
    example_lvgl_demo_ui(disp, indev);

    //setup_usb(GPIO_NUM_0);

    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t pixels[] = {0x11, 0x00, 0x00, 0x11, 0x00, 0x00, 0x11, 0x00, 0x00, 0x11, 0x00, 0x00};
    seesaw_pixel_write(I2C_NEOKEY_ADDR, pixels, 12);

    uint8_t encoder_pixel[] = {0x11, 0x11, 0x11};
    seesaw_pixel_write(I2C_ENCODER_ADDR, encoder_pixel, 3);

    vTaskDelay(pdMS_TO_TICKS(5));

    xTaskCreatePinnedToCore(neokey_task, "neokey_task", 2048, NULL, 4, NULL, 1);

    xTaskCreatePinnedToCore(hid_task, "hid_task", 2048, NULL, 4, NULL, 1);

    uint32_t last_buttons = 0;

    while (1)
    {
        //  raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        //  The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
    }
}
