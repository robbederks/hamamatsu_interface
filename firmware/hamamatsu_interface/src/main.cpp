#include <Arduino.h>
#include <usb_custom.h>
#include <imxrt.h>

#define SENSOR_ROWS 1056
#define SENSOR_COLUMNS 1056

#define PIN_RX_ENABLE 6
#define PIN_INT_EXT 11
#define PIN_EXT_TRIGGER 12
#define PIN_BIN0 24
#define PIN_BIN1 25
#define PIN_PCLK 26
#define PIN_DATA_1 19
#define PIN_DATA_2 18
#define PIN_DATA_3 14
#define PIN_DATA_4 15
#define PIN_DATA_5 40
#define PIN_DATA_6 41
#define PIN_DATA_7 17
#define PIN_DATA_8 16
#define PIN_DATA_9 22
#define PIN_DATA_10 23
#define PIN_DATA_11 20
#define PIN_DATA_12 21
#define PIN_HSYNC 39
#define PIN_VSYNC 38

#define STATE_IDLE 0
#define STATE_WAITING_ON_VSYNC 1
#define STATE_ACQUIRING 2
#define STATE_DONE 3

typedef struct __attribute__((__packed__)) {
  uint8_t state;
  uint32_t row;
  uint32_t col;
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

    case 0x03: // Start trigger
      if (state.state != STATE_IDLE && state.state != STATE_DONE) {
        Serial.write("Already busy!\n");
        return_data[0] = 0x01;
      } else {
        memset(pixel_buffer, 0xFF, sizeof(pixel_buffer));
        state.state = STATE_WAITING_ON_VSYNC;
        digitalWrite(PIN_EXT_TRIGGER, HIGH);
        return_data[0] = 0x00;
      }
      return_len = 1;
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

void pclk_interrupt() {
  if (state.state != STATE_ACQUIRING) {
    return;
  }

  // Read full port to get 12-bit data
  pixel_buffer[state.row][state.col] = (GPIO6_PSR >> 16) & 0xFFF;
  state.col++;
}

void vsync_interrupt() {
  if (state.state == STATE_WAITING_ON_VSYNC) {
    state.state = STATE_ACQUIRING;
    state.row = 0;
    state.col = 0;
  }
}

void hsync_interrupt() {
  if (state.state == STATE_ACQUIRING) {
    if (state.row % 100 == 0) {
      Serial.printf("Row %d cols %d\n", state.row-1, state.col);
    }

    state.col = 0;
    state.row++;
    if (state.row == SENSOR_ROWS) {
      state.state = STATE_DONE;
      Serial.write("Done!\n");
      Serial.flush();
    }

    if (state.row == SENSOR_ROWS / 2) {
      digitalWrite(PIN_EXT_TRIGGER, LOW);
    }
  }
}

void setup() {
  // Setup USB Serial
  Serial.begin(115200);

  // Setup Faxitron Serial
  Serial2.begin(9600, SERIAL_8N1);

  // GPIO
  pinMode(PIN_RX_ENABLE, OUTPUT);
  pinMode(PIN_INT_EXT, OUTPUT);
  pinMode(PIN_EXT_TRIGGER, OUTPUT);
  pinMode(PIN_BIN0, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_PCLK, INPUT);
  pinMode(PIN_DATA_1, INPUT);
  pinMode(PIN_DATA_2, INPUT);
  pinMode(PIN_DATA_3, INPUT);
  pinMode(PIN_DATA_4, INPUT);
  pinMode(PIN_DATA_5, INPUT);
  pinMode(PIN_DATA_6, INPUT);
  pinMode(PIN_DATA_7, INPUT);
  pinMode(PIN_DATA_8, INPUT);
  pinMode(PIN_DATA_9, INPUT);
  pinMode(PIN_DATA_10, INPUT);
  pinMode(PIN_DATA_11, INPUT);
  pinMode(PIN_DATA_12, INPUT);
  pinMode(PIN_HSYNC, INPUT);
  pinMode(PIN_VSYNC, INPUT);

  digitalWrite(PIN_RX_ENABLE, HIGH);
  digitalWrite(PIN_INT_EXT, HIGH);
  digitalWrite(PIN_EXT_TRIGGER, LOW);
  digitalWrite(PIN_BIN0, LOW);
  digitalWrite(PIN_BIN1, LOW);

  // Setup interrupt on pclk
  attachInterrupt(digitalPinToInterrupt(PIN_PCLK), pclk_interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_VSYNC), vsync_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_HSYNC), hsync_interrupt, RISING);

  // USB handler
  usb_custom_set_handler(usb_handler);
}

void loop() {}