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