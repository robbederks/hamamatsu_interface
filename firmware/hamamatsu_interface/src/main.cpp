#include <Arduino.h>
#include <usb_custom.h>

#define SENSOR_ROWS 1024
#define SENSOR_COLUMNS 1024

typedef struct __attribute__((__packed__)) {
  uint32_t row;
  uint32_t col;
  bool busy;
  bool done;
} readout_state;
volatile readout_state state;

typedef struct __attribute__((__packed__)) {
  uint8_t command;
  uint32_t data_len;
  uint8_t data[64 - 5];
} control_req_t;

EXTMEM uint16_t pixel_buffer[SENSOR_ROWS][SENSOR_COLUMNS]; // External RAM is neccesary to fit the buffer

uint32_t faxitron_command(uint8_t* command, uint32_t command_len, uint8_t *response, uint32_t max_response_len) {
  if (command_len > 0) {
    Serial2.write(command, command_len);
    Serial2.write('\r');
    Serial2.flush();
  }
  return (uint32_t) Serial2.readBytesUntil('\r', response, max_response_len);
}

uint32_t usb_handler(uint8_t *control_data, uint32_t len, uint8_t *return_data, uint32_t max_return_len) {
  uint32_t return_len = 0;
  control_req_t *req = (control_req_t *)control_data;
  if (len < 5) {
    Serial.write("Invalid command with len %d\n", len);
    return_len = 0;
    goto end;
  }

  switch (req->command) {
    case 0x00: // Ping
      Serial.write("Ping!\n");
      return_data[0] = 0xA5;
      return_len = 1;
      break;
    case 0x01: // Check state
      memcpy(return_data, &state, sizeof(state));
      return_len = sizeof(state);
      break;
    case 0x02: // Get pixel buffer
      // setup bulk transfer
      usb_custom_init_bulk_transfer((uint8_t *) pixel_buffer, sizeof(pixel_buffer), 0);

      // return the size of the buffer
      *((uint32_t *)return_data) = sizeof(pixel_buffer);
      return_len = sizeof(uint32_t);
      break;
    case 0x10: // Get Faxitron status
      return_len = faxitron_command(req->data, req->data_len, return_data, 10);
      break;

    default:
      Serial.write("Invalid command %d\n", req->command);
      return_len = 0;
      break;
  }

end:
  return return_len;
}

void setup() {
  // Setup USB Serial
  Serial.begin(115200);

  // Setup Faxitron Serial
  Serial2.begin(9600, SERIAL_8N1);

  // USB handler
  usb_custom_set_handler(usb_handler);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}