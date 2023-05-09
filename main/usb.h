#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

void setup_usb(gpio_num_t button);
void app_send_hid_demo(uint8_t number_key);
bool usb_mounted();