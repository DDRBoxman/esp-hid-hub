#include <stdint.h>
#include <stdbool.h>

void neokey_setup(uint8_t addr);

bool neokey_read(uint8_t addr, uint32_t *buttons);
