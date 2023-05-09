#include <inttypes.h>
#include "esp_err.h"

/**
    float temperature = 0.0f;
    float humidity = 0.0f;

    htu31d_read_temp_rh(0x40, &temperature, &humidity);

    ESP_LOGW(TAG, "%f\n", humidity);
    ESP_LOGW(TAG, "%f\n", temperature);
*/
esp_err_t htu31d_read_temp_rh(uint8_t addr, float *ret_temperature, float *ret_humidity);

esp_err_t htu31d_heater_on(uint8_t addr);