#pragma once

#include "usb_desc.h"
#include <stdint.h>

// C language implementation
#ifdef __cplusplus
extern "C" {
#endif
  extern volatile uint8_t usb_high_speed;
  void usb_custom_configure (void);
  void usb_custom_set_handler(uint32_t (*handler)(uint8_t *control_data, uint32_t len, uint8_t *return_data, uint32_t max_return_len));
  void usb_custom_init_bulk_transfer(uint8_t *buffer, uint32_t len, uint8_t transfer_id);
#ifdef __cplusplus
}
#endif