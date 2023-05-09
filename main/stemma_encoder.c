#include "stemma_encoder.h"
#include "seesaw.h"

#define SS_SWITCH        24
#define SS_NEOPIX        6

#define SS_BUTTONMASK (1ul << 24)


void stemma_encoder_setup(uint8_t addr) {

  seesaw_pixel_init(addr, SS_NEOPIX, 1);

  seesaw_pin_mode_bulk(addr, (uint32_t)1 << SS_SWITCH, INPUT_PULLUP);
  seesaw_set_interrupts(addr, (uint32_t)1 << SS_SWITCH, 1);

  seesaw_enable_encoder_interrupt(addr);
}

int32_t stemma_encoder_get_position(uint8_t addr) {
  return seesaw_get_encoder_position(addr);
}

int32_t stemma_encoder_get_diff(uint8_t addr) {
  return seesaw_get_encoder_diff(addr);
}

bool stemma_encoder_read_button(uint8_t addr) {
  uint32_t buttons = seesaw_digital_read_bulk(addr, SS_BUTTONMASK);

  return buttons != 0;
}