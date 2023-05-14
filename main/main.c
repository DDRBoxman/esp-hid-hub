#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lvgl.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"

#include "lvgl_control.h"

#include "pins.h"

#include "i2c_accessories.h"

#include "usb.h"

#include "esp_app_trace.h"

#include "class/hid/hid_device.h"

#include "shared.h"

static const char *TAG = "example";

/* Signal Wi-Fi events on this event-group */
const int WIFI_CONNECTED_EVENT = BIT0;
static EventGroupHandle_t wifi_event_group;

QueueHandle_t hid_queue;

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

void lvgl_task(void *parms)
{
}

void app_main(void)
{
    ESP_LOGI(TAG, "Waiting for OpenOCD connection???");

   // esp_log_set_vprintf(esp_apptrace_vprintf);

   /* Initialize NVS partition */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* Initialize TCP/IP */
    ESP_ERROR_CHECK(esp_netif_init());

    hid_queue = xQueueCreate(10, sizeof(uint32_t));

    ESP_LOGI(TAG, "Turn on tft");
    gpio_config_t tft_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << TFT_I2C_POWER};
    ESP_ERROR_CHECK(gpio_config(&tft_gpio_config));
    gpio_set_level(TFT_I2C_POWER, LCD_BK_LIGHT_ON_LEVEL);

    // Wait for i2c devices to be ready
    vTaskDelay(500 / portTICK_PERIOD_MS);

    i2c_setup();

    // setup_usb(GPIO_NUM_0);

    lvgl_setup();

    vTaskDelay(pdMS_TO_TICKS(5));

    xTaskCreatePinnedToCore(neokey_task, "neokey_task", 2048, NULL, 4, NULL, 1);

    xTaskCreatePinnedToCore(hid_task, "hid_task", 2048, NULL, 4, NULL, 1);

    while (1)
    {
        //  raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        //  The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
    }
}
