#include <stdio.h>
#include <stdbool.h>

#define INPUT           (0x0ul)
#define OUTPUT          (0x1ul)
#define INPUT_PULLUP    (0x2ul)
#define INPUT_PULLDOWN  (0x4ul)

/** GPIO module function address registers
 */
enum {
  SEESAW_GPIO_DIRSET_BULK = 0x02,
  SEESAW_GPIO_DIRCLR_BULK = 0x03,
  SEESAW_GPIO_BULK = 0x04,
  SEESAW_GPIO_BULK_SET = 0x05,
  SEESAW_GPIO_BULK_CLR = 0x06,
  SEESAW_GPIO_BULK_TOGGLE = 0x07,
  SEESAW_GPIO_INTENSET = 0x08,
  SEESAW_GPIO_INTENCLR = 0x09,
  SEESAW_GPIO_INTFLAG = 0x0A,
  SEESAW_GPIO_PULLENSET = 0x0B,
  SEESAW_GPIO_PULLENCLR = 0x0C,
};

/** status module function address registers
 */
enum {
  SEESAW_STATUS_HW_ID = 0x01,
  SEESAW_STATUS_VERSION = 0x02,
  SEESAW_STATUS_OPTIONS = 0x03,
  SEESAW_STATUS_TEMP = 0x04,
  SEESAW_STATUS_SWRST = 0x7F,
};

/** neopixel module function address registers
 */
enum {
  SEESAW_NEOPIXEL_STATUS = 0x00,
  SEESAW_NEOPIXEL_PIN = 0x01,
  SEESAW_NEOPIXEL_SPEED = 0x02,
  SEESAW_NEOPIXEL_BUF_LENGTH = 0x03,
  SEESAW_NEOPIXEL_BUF = 0x04,
  SEESAW_NEOPIXEL_SHOW = 0x05,
};

/** Module Base Addreses
 *  The module base addresses for different seesaw modules.
 */
enum {
  SEESAW_STATUS_BASE = 0x00,
  SEESAW_GPIO_BASE = 0x01,
  SEESAW_SERCOM0_BASE = 0x02,

  SEESAW_TIMER_BASE = 0x08,
  SEESAW_ADC_BASE = 0x09,
  SEESAW_DAC_BASE = 0x0A,
  SEESAW_INTERRUPT_BASE = 0x0B,
  SEESAW_DAP_BASE = 0x0C,
  SEESAW_EEPROM_BASE = 0x0D,
  SEESAW_NEOPIXEL_BASE = 0x0E,
  SEESAW_TOUCH_BASE = 0x0F,
  SEESAW_KEYPAD_BASE = 0x10,
  SEESAW_ENCODER_BASE = 0x11,
  SEESAW_SPECTRUM_BASE = 0x12,
};

/** encoder module edge definitions
 */
enum {
  SEESAW_ENCODER_STATUS = 0x00,
  SEESAW_ENCODER_INTENSET = 0x10,
  SEESAW_ENCODER_INTENCLR = 0x20,
  SEESAW_ENCODER_POSITION = 0x30,
  SEESAW_ENCODER_DELTA = 0x40,
};

#define SEESAW_HW_ID_CODE_SAMD09 0x55 ///< seesaw HW ID code for SAMD09

#define SEESAW_HW_ID_CODE_TINY8X7 0x87 ///< seesaw HW ID code for ATtiny817

#define SEESAW_EEPROM_I2C_ADDR                                                 \
  0x3F ///< EEPROM address of i2c address to start up with (for devices that
       ///< support this feature)

void seesaw_pin_mode(uint8_t addr, uint8_t pin, uint8_t mode);

void seesaw_pin_mode_bulk(uint8_t addr, uint32_t pins, uint8_t mode);

void seesaw_set_interrupts(uint8_t addr, uint32_t pins, bool enabled);

//void seesaw_read_interrupts(addr, uint32_t pins);

uint32_t seesaw_digital_read_bulk(uint8_t addr, uint32_t pins);

void seesaw_pixel_init(uint8_t addr, uint8_t pin, uint8_t num_pixels);

void seesaw_pixel_write(uint8_t addr, uint8_t pixels[], uint8_t len);

void seesaw_pixel_write_offset(uint8_t addr, uint8_t pixels[], uint8_t len, uint8_t offset);

uint32_t seesaw_get_version(uint8_t addr);

uint8_t seesaw_get_hardware_type(uint8_t addr);

uint8_t seesaw_get_address(uint8_t addr);

bool seesaw_enable_encoder_interrupt(uint8_t addr);

int32_t seesaw_get_encoder_position(uint8_t addr);
