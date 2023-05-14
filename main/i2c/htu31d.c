#import "htu31d.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "example";


//TODO Move???
#define I2C_MASTER_TIMEOUT_MS       1000
#define I2C_MASTER_NUM              0

static uint8_t htu31d_crc(uint16_t value) {
  uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
  uint32_t msb = 0x800000;
  uint32_t mask = 0xFF8000;
  uint32_t result = (uint32_t)value << 8; // Pad with zeros as specified in spec

  while (msb != 0x80) {
    // Check if msb of current value is 1 and apply XOR mask
    if (result & msb)
      result = ((result ^ polynom) & mask) | (result & ~mask);

    // Shift by one
    msb >>= 1;
    mask >>= 1;
    polynom >>= 1;
  }
  return result;
}

esp_err_t htu31d_heater_on(uint8_t addr)
{
    uint8_t cmd[] = {0x04};
    return i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t htu31d_read_temp_rh(uint8_t addr, float* ret_temperature, float* ret_humidity)
{

    uint8_t cmd[] = {0x40};
    i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    // TODO: find real value, or come back later???
    vTaskDelay(500 / portTICK_PERIOD_MS);

    uint8_t cmd2[] = {0};
    uint8_t buf[6];
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM,
                                 addr,
                                 cmd2, 1,
                                 buf, 6,
                                 I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    uint8_t test = buf[0];
    uint8_t test2 = buf[1];
    uint8_t test3 = buf[2];
    uint8_t test4 = buf[3];
    uint8_t test5 = buf[4];
    uint8_t test6 = buf[5];

     uint16_t raw_temp = buf[0];
    raw_temp <<= 8;
    raw_temp |= buf[1];

    uint8_t crc_tmp = htu31d_crc(raw_temp);

    uint16_t raw_humidity = ((uint16_t)buf[3] << 8) | (uint16_t)buf[4];

    uint8_t crc_hmd = htu31d_crc(raw_humidity);

    ESP_LOGW(TAG, "%d %d %d %d %d %d %d %d\n", test, test2, test3, test4, test5, test6, crc_tmp, crc_hmd);

    uint8_t temp_crc = buf[2];

    if (temp_crc != htu31d_crc(raw_temp)) {
        return -1;
    }

    float temperature = (float) raw_temp;
    temperature /= 65535.0;
    temperature *= 165.0;
    temperature -= 40.0;

    *ret_temperature = temperature;

    //uint16_t raw_humidity = ((uint16_t)buf[3] << 8) | (uint16_t)buf[4];

    uint8_t humidity_crc = buf[5];
    
    if (humidity_crc != htu31d_crc(raw_humidity)) {
        return -1;
    }

    float humidity = raw_humidity;
    humidity /= 65535.0;
    humidity *= 100.0;

    *ret_humidity = humidity;

    return ret;
}