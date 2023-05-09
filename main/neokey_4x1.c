#include "neokey_4x1.h"
#include "seesaw.h"

#define NEOKEY_1X4_NEOPIN 3
#define NEOKEY_1X4_BUTTONA 4
#define NEOKEY_1X4_BUTTONB 5
#define NEOKEY_1X4_BUTTONC 6
#define NEOKEY_1X4_BUTTOND 7
#define NEOKEY_1X4_BUTTONMASK ((1 << 4) | (1 << 5) | (1 << 6) | (1 << 7))

void neokey_setup(uint8_t addr) {
  seesaw_pixel_init(addr, 3, 4);

  seesaw_pin_mode_bulk(addr, NEOKEY_1X4_BUTTONMASK, INPUT_PULLUP);
  // This seems to be required even if we don't read the interrupt register
  seesaw_set_interrupts(addr, NEOKEY_1X4_BUTTONMASK, 1);
}

void neokey_set_pixels() {

}

void neokey_read_interrupts(uint8_t addr) {
  //seesaw_read_interrupts(addr, )
}

uint32_t neokey_read(uint8_t addr) {
  uint32_t buttons = seesaw_digital_read_bulk(addr, NEOKEY_1X4_BUTTONMASK);
  buttons ^= NEOKEY_1X4_BUTTONMASK;
  buttons &= NEOKEY_1X4_BUTTONMASK;
  buttons >>= NEOKEY_1X4_BUTTONA;

  return buttons;
}