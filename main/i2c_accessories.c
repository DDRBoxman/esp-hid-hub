#include "driver/i2c.h"
#include "esp_log.h"

#include "i2c/seesaw.h"
#include "i2c/neokey_4x1.h"
#include "i2c/htu31d.h"
#include "i2c/stemma_encoder.h"

#include "pins.h"

#include "shared.h"

static const char *TAG = "i2c_accessories.c";

static SemaphoreHandle_t encoder_mutex;
static int32_t encoder_diff = 0;
static uint32_t encoder_pressed = 0;

int32_t get_encoder_diff() {
    xSemaphoreTake(encoder_mutex, portMAX_DELAY);
    uint32_t diff = encoder_diff;
    encoder_diff = 0;
    xSemaphoreGive(encoder_mutex);
    return diff;
}

uint32_t get_encoder_pressed() {
    return encoder_pressed;
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = 0,
        .scl_pullup_en = 0,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK)
    {
        return err;
    }

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void reset_i2c()
{
     ESP_LOGI(TAG, "i2c reset");

    i2c_reset_tx_fifo(I2C_MASTER_NUM);
    i2c_reset_rx_fifo(I2C_MASTER_NUM);

    i2c_driver_delete(I2C_MASTER_NUM);

    i2c_master_init();
}

void neokey_task(void *parameter)
{
    uint32_t last_buttons = 0;

    uint8_t data_wr[] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};

    while (1)
    {
        uint32_t buttons = 0;
        bool success = neokey_read(I2C_NEOKEY_ADDR, &buttons);

        if (!success)
        {
            reset_i2c();
            continue;
        }

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
        if (!success)
        {
            reset_i2c();
            continue;
        }

        success = seesaw_pixel_write(I2C_NEOKEY_ADDR, data_wr, 12);

        uint32_t pressed = 0;
        success = stemma_encoder_read_button(I2C_ENCODER_ADDR, &pressed);
        if (!success)
        {
            reset_i2c();
            continue;
        }

        int32_t diff = 0;
        success = stemma_encoder_get_diff(I2C_ENCODER_ADDR, &diff);
        if (!success)
        {
            reset_i2c();
            continue;
        }

        xSemaphoreTake(encoder_mutex, portMAX_DELAY);
        encoder_diff += diff;
        encoder_pressed = pressed;
        xSemaphoreGive(encoder_mutex);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void i2c_setup() {
    encoder_mutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    neokey_setup(I2C_NEOKEY_ADDR);

    stemma_encoder_setup(I2C_ENCODER_ADDR);

    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t pixels[] = {0x11, 0x00, 0x00, 0x11, 0x00, 0x00, 0x11, 0x00, 0x00, 0x11, 0x00, 0x00};
    seesaw_pixel_write(I2C_NEOKEY_ADDR, pixels, 12);

    uint8_t encoder_pixel[] = {0x11, 0x11, 0x11};
    seesaw_pixel_write(I2C_ENCODER_ADDR, encoder_pixel, 3);
}