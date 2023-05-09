#include <string.h>

#include "seesaw.h"
#include "driver/i2c.h"

#include "esp_err.h"
#include "esp_log.h"

#define I2C_MASTER_NUM 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */

#define ACK_VAL 0x0  /*!< I2C ack value */
#define NACK_VAL 0x1 /*!< I2C nack value */

static const char *TAG = "example";

void seesaw_pixel_init(uint8_t addr, uint8_t pin, uint8_t num_pixels)
{
    uint8_t cmd[] = {SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_PIN, pin};
    i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd, sizeof(cmd), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    uint8_t cmd2[] = {SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF_LENGTH, 0x0, num_pixels * 3};
    i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd2, sizeof(cmd2), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void seesaw_pixel_write(uint8_t addr, uint8_t pixels[], uint8_t len)
{
    seesaw_pixel_write_offset(addr, pixels, len, 0);
}

void seesaw_pixel_write_offset(uint8_t addr, uint8_t pixels[], uint8_t len, uint8_t offset)
{
    // TODO: Preallocate a larger buffer?
    uint8_t *data_wr = malloc(4 + len);
    uint8_t data[] = {SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF, 0x0, offset};
    memcpy(data_wr, data, 4);
    memcpy(data_wr + 4, pixels, len);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, 4 + len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // We need to disable auto ack checks for this command
    // The neokey 4x1 doesn't seem to hold them long enough?
    uint8_t cmd4[] = {SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_SHOW};
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | WRITE_BIT, 0);
    i2c_master_write(cmd, cmd4, 2, 0);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void seesaw_pin_mode_bulk(uint8_t addr, uint32_t pins, uint8_t mode)
{
    uint8_t cmd[] = {SEESAW_GPIO_BASE, 0, (uint8_t)(pins >> 24), (uint8_t)(pins >> 16),
                     (uint8_t)(pins >> 8), (uint8_t)pins};
    switch (mode)
    {
    case OUTPUT:
        cmd[1] = SEESAW_GPIO_DIRSET_BULK;
        i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd, sizeof(cmd), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        break;
    case INPUT:
        cmd[1] = SEESAW_GPIO_DIRCLR_BULK;
        i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd, sizeof(cmd), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        break;
    case INPUT_PULLUP:
        cmd[1] = SEESAW_GPIO_DIRCLR_BULK;
        i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd, sizeof(cmd), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        cmd[1] = SEESAW_GPIO_PULLENSET;
        i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd, sizeof(cmd), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        cmd[1] = SEESAW_GPIO_BULK_SET;
        i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd, sizeof(cmd), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        break;
    case INPUT_PULLDOWN:
        cmd[1] = SEESAW_GPIO_DIRCLR_BULK;
        i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd, sizeof(cmd), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        cmd[1] = SEESAW_GPIO_PULLENSET;
        i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd, sizeof(cmd), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        cmd[1] = SEESAW_GPIO_BULK_CLR;
        i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd, sizeof(cmd), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        break;
    }
}

void seesaw_pin_mode(uint8_t addr, uint8_t pin, uint8_t mode)
{
    /*if (pin >= 32)
      pinModeBulk(addr, 0, 1ul << (pin - 32), mode);
    else*/
    seesaw_pin_mode_bulk(addr, 1ul << pin, mode);
}

void seesaw_set_interrupts(uint8_t addr, uint32_t pins, bool enabled)
{
    uint8_t cmd[] = {SEESAW_GPIO_BASE, 0, (uint8_t)(pins >> 24), (uint8_t)(pins >> 16),
                     (uint8_t)(pins >> 8), (uint8_t)pins};
    if (enabled)
    {
        cmd[1] = SEESAW_GPIO_INTENSET;
    }
    else
    {
        cmd[1] = SEESAW_GPIO_INTENCLR;
    }

    i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd, sizeof(cmd), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

uint32_t seesaw_digital_read_bulk(uint8_t addr, uint32_t pins)
{
    uint8_t data_wr[] = {SEESAW_GPIO_BASE, SEESAW_GPIO_BULK};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, 2, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t buf[4];

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, buf, 3, ACK_VAL);
    i2c_master_read_byte(cmd, buf + 3, NACK_VAL);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    switch (err)
    {
    // case ESP_OK:
    case ESP_ERR_INVALID_ARG:
        ESP_LOGW(TAG, "invalid tag");
        break;
    case ESP_FAIL:
        ESP_LOGW(TAG, "fail");
        break;
    case ESP_ERR_INVALID_STATE:
        ESP_LOGW(TAG, "invalid state");
        break;
    case ESP_ERR_TIMEOUT:
        ESP_LOGW(TAG, "timeout");
        break;
    }

    uint32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                   ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
    return ret & pins;
}

uint32_t seesaw_get_version(uint8_t addr)
{
    uint8_t cmd[] = {SEESAW_STATUS_BASE, SEESAW_STATUS_VERSION};
    uint8_t buf[4];
    i2c_master_write_read_device(I2C_MASTER_NUM,
                                 addr,
                                 cmd, 2,
                                 buf, 4,
                                 I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    uint32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                   ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
    return ret;
}

uint8_t seesaw_get_hardware_type(uint8_t addr)
{
    uint8_t cmd[] = {SEESAW_STATUS_BASE, SEESAW_STATUS_HW_ID};
    uint8_t buf[1];
    i2c_master_write_read_device(I2C_MASTER_NUM,
                                 addr,
                                 cmd, 2,
                                 buf, 1,
                                 I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return buf[0];
}

uint8_t seesaw_get_address(uint8_t addr)
{
    uint8_t seesaw_cmd[] = {SEESAW_EEPROM_BASE, SEESAW_EEPROM_I2C_ADDR};
    uint8_t buf[1];

    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM,
                                                 addr,
                                                 seesaw_cmd, 2,
                                                 buf, 1,
                                                 I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    switch (ret)
    {
    // case ESP_OK:
    case ESP_ERR_INVALID_ARG:
        ESP_LOGW(TAG, "invalid tag");
        break;
    case ESP_FAIL:
        ESP_LOGW(TAG, "fail");
        break;
    case ESP_ERR_INVALID_STATE:
        ESP_LOGW(TAG, "invalid state");
        break;
    case ESP_ERR_TIMEOUT:
        ESP_LOGW(TAG, "timeout");
        break;
    }

    return buf[0];
}

bool seesaw_enable_encoder_interrupt(uint8_t addr)
{
    uint8_t cmd[] = {SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENSET, 0x01};
    i2c_master_write_to_device(I2C_MASTER_NUM, addr, cmd, sizeof(cmd), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return true;
}

int32_t seesaw_get_encoder_position(uint8_t addr)
{
    uint8_t cmd[] = {SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION};
    uint8_t buf[4];
    i2c_master_write_read_device(I2C_MASTER_NUM,
                                 addr,
                                 cmd, 2,
                                 buf, 4,
                                 I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    int32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                  ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];

    return ret;
}