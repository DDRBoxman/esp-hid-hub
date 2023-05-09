#include <stdint.h>
#include <stdbool.h>

void stemma_encoder_setup(uint8_t addr);
int32_t stemma_encoder_get_position(uint8_t addr);
int32_t stemma_encoder_get_diff(uint8_t addr);
uint32_t stemma_encoder_read_button(uint8_t addr);