#include "freertos/queue.h"

extern QueueHandle_t hid_queue;

int32_t get_encoder_diff();
uint32_t get_encoder_pressed();