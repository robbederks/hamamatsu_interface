#include <Arduino.h>
#include <usb_custom.h>
#include <imxrt.h>
#include <DMAChannel.h>
#include "reverse_table.h"

// C7942 native resolution is 2400x2400 at 12.5 MHz
// Using DTCM row buffer (fast RAM) for capture, then pack to PSRAM
// 12-bit packing: 2400x2400x1.5 = 8.64MB, crop to 2400x2320 = 8.35MB fits
#define SENSOR_ROWS 2320
#define SENSOR_COLUMNS 2400
#define SKIP_ROWS ((2400 - SENSOR_ROWS) / 2)  // Center the vertical crop

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

#define GPIO6_DATA_SHIFT 16

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

// 12-bit packed buffer in PSRAM, row buffer in fast DMAMEM for capture
// Packed size: ROWS * COLS * 3 / 2 bytes (2 pixels = 3 bytes)
#define PACKED_FRAME_SIZE ((uint32_t)SENSOR_ROWS * SENSOR_COLUMNS * 3 / 2)
uint8_t *packed_buffer = nullptr;

// DMA capture buffer - 32-bit to match GPIO register width
// 1x sampling - one DMA transfer per intended pixel
#define SAMPLES_PER_ROW SENSOR_COLUMNS  // 2400 samples

// Buffer for capture
uint32_t dma_row_buffer[SAMPLES_PER_ROW] __attribute__((aligned(32)));

// DMA channels for GPIO capture
DMAChannel dma_gpio6;
DMAChannel dma_gpio1;
volatile bool dma_complete = false;

void dma_isr() {
  dma_gpio6.clearInterrupt();
  dma_complete = true;
}

// External memory allocation helper
extern "C" uint8_t external_psram_size;

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
    case 0x00:
      Serial.write("Ping!\n");

      // DMA test - read one word from GPIO1 (not GPIO6 which has bus access issues)
      {
        uint32_t test_dest = 0xDEADBEEF;
        Serial.printf("DMA test: GPIO1=0x%08X @ 0x%08X, GPIO6=0x%08X @ 0x%08X\n",
                      GPIO1_PSR, (uint32_t)&GPIO1_PSR, GPIO6_PSR, (uint32_t)&GPIO6_PSR);

        // Clear any previous errors
        DMA_CERR = dma_gpio6.channel;

        dma_gpio6.TCD->SADDR = &GPIO1_PSR;  // Use GPIO1 instead of GPIO6
        dma_gpio6.TCD->SOFF = 0;
        dma_gpio6.TCD->ATTR = 0x0202;
        dma_gpio6.TCD->NBYTES = 4;
        dma_gpio6.TCD->SLAST = 0;
        dma_gpio6.TCD->DADDR = &test_dest;
        dma_gpio6.TCD->DOFF = 0;
        dma_gpio6.TCD->CITER = 1;
        dma_gpio6.TCD->BITER = 1;
        dma_gpio6.TCD->DLASTSGA = 0;
        dma_gpio6.TCD->CSR = 0;
        asm volatile("dsb");

        dma_gpio6.enable();
        dma_gpio6.triggerManual();
        delayMicroseconds(10);

        Serial.printf("DMA test: before=0xDEADBEEF, after=0x%08X\n", test_dest);
        Serial.printf("DMA test: CITER=%d, ERR=0x%08X, ES=0x%08X\n",
                      dma_gpio6.TCD->CITER, DMA_ERR, DMA_ES);
        dma_gpio6.disable();
      }

      return_data[0] = 0xA5;
      return_len = 1;
      break;

    case 0x01:
      memcpy(return_data, (const void*)&state, sizeof(state));
      return_len = sizeof(state);
      break;

    case 0x02:
      if (packed_buffer == nullptr) {
        *((uint32_t *)return_data) = 0;
        return_len = sizeof(uint32_t);
        Serial.write("ERROR: No pixel buffer allocated!\n");
      } else {
        usb_custom_init_bulk_transfer(packed_buffer, PACKED_FRAME_SIZE, 0);
        *((uint32_t *)return_data) = PACKED_FRAME_SIZE;
        return_len = sizeof(uint32_t);
      }
      break;

    case 0x03:
      if (packed_buffer == nullptr) {
        Serial.write("ERROR: No pixel buffer allocated!\n");
        return_data[0] = 0x02;  // Error: no buffer
      } else if (state.state != STATE_IDLE && state.state != STATE_DONE) {
        Serial.write("Already busy!\n");
        return_data[0] = 0x01;
      } else {
        // Initialize packed buffer to 0xFF (which unpacks to 0xFFF, 0xFFF)
        // This allows test_frame.py to detect uncaptured pixels
        memset(packed_buffer, 0xFF, PACKED_FRAME_SIZE);
        arm_dcache_flush_delete(packed_buffer, PACKED_FRAME_SIZE);
        state.state = STATE_WAITING_ON_VSYNC;
        state.row = 0;
        state.col = 0;
        digitalWrite(PIN_EXT_TRIGGER, HIGH);
        return_data[0] = 0x00;
      }
      return_len = 1;
      break;

    case 0x10:
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

// GPIO1 bit positions for sync signals (Pin 26=PCLK, Pin 38=VSYNC, Pin 39=HSYNC)
// Teensy 4.1: Pin 26 -> GPIO1 bit 30, Pin 38 -> GPIO1 bit 28, Pin 39 -> GPIO1 bit 29
#define GPIO1_PCLK_BIT  30
#define GPIO1_VSYNC_BIT 28
#define GPIO1_HSYNC_BIT 29
#define GPIO1_PCLK_MASK  (1UL << GPIO1_PCLK_BIT)
#define GPIO1_VSYNC_MASK (1UL << GPIO1_VSYNC_BIT)
#define GPIO1_HSYNC_MASK (1UL << GPIO1_HSYNC_BIT)

// Setup DMA with A_ON mode for oversampled capture
void setup_dma_capture() {
  // Configure DMA channel to read from GPIO1 (not GPIO6 - fast GPIO isn't DMA accessible)
  dma_gpio6.begin();
  dma_gpio6.source(GPIO1_PSR);
  dma_gpio6.destinationBuffer(dma_row_buffer, SAMPLES_PER_ROW * sizeof(uint32_t));
  dma_gpio6.transferSize(4);  // 32-bit transfers
  dma_gpio6.transferCount(SAMPLES_PER_ROW);
  dma_gpio6.interruptAtCompletion();
  dma_gpio6.attachInterrupt(dma_isr);

  // Configure DMAMUX for A_ON (always-on) mode - continuous triggering
  volatile uint32_t *dmamux = (volatile uint32_t *)0x400EC000;
  dma_gpio6.triggerAtHardwareEvent(0);  // Placeholder
  dmamux[dma_gpio6.channel] = (1 << 31) | (1 << 29);  // ENBL | A_ON

  Serial.println("DMA capture configured with A_ON mode");
  Serial.printf("  DMA channel = %d\n", dma_gpio6.channel);
  Serial.printf("  Samples per row = %d\n", SAMPLES_PER_ROW);
  Serial.printf("  DMAMUX = 0x%08X\n", dmamux[dma_gpio6.channel]);
}

// Pack two 12-bit values into 3 bytes
// Layout: byte0 = p0[7:0], byte1 = p1[3:0]<<4 | p0[11:8], byte2 = p1[11:4]
inline void pack_12bit(uint16_t p0, uint16_t p1, uint8_t *out) {
  out[0] = p0 & 0xFF;
  out[1] = ((p1 & 0x0F) << 4) | ((p0 >> 8) & 0x0F);
  out[2] = (p1 >> 4) & 0xFF;
}

// Temporary row buffer for extracted pixels
uint16_t pixel_row_buffer[SENSOR_COLUMNS];

// Capture using tight polling loop
// At 12.5 MHz pixel clock and 600 MHz CPU, we have 48 cycles per pixel
FASTRUN void acquire_frame() {
  if (packed_buffer == nullptr) {
    state.state = STATE_IDLE;
    return;
  }

  // Wait for VSYNC rising edge
  while (!(GPIO1_PSR & GPIO1_VSYNC_MASK)) {
    if (state.state != STATE_WAITING_ON_VSYNC) return;
  }

  state.state = STATE_ACQUIRING;

  // Pointer to current position in packed buffer
  uint8_t *pack_ptr = packed_buffer;

  // Skip initial rows to center the vertical crop (interrupts still enabled)
  for (uint32_t skip = 0; skip < SKIP_ROWS; skip++) {
    while (!(GPIO1_PSR & GPIO1_HSYNC_MASK)) {}
    while (GPIO1_PSR & GPIO1_HSYNC_MASK) {}
  }

  for (uint32_t row = 0; row < SENSOR_ROWS; row++) {
    state.row = row;

    // Wait for HSYNC high (start of row) with timeout
    uint32_t timeout = 10000000;  // ~17ms at 600MHz
    while (!(GPIO1_PSR & GPIO1_HSYNC_MASK)) {
      if (--timeout == 0) {
        // No more rows from sensor
        __enable_irq();
        Serial.printf("HSYNC timeout at row %lu\n", row);
        goto done;
      }
    }

    // Clear row buffer
    memset(dma_row_buffer, 0xFF, SAMPLES_PER_ROW * sizeof(uint32_t));

    // Disable interrupts only during pixel capture
    __disable_irq();

    // Wait for PCLK rising edge to synchronize
    while (!(GPIO1_PSR & GPIO1_PCLK_MASK)) {}  // Wait for PCLK high
    while (GPIO1_PSR & GPIO1_PCLK_MASK) {}      // Wait for PCLK low

    // Fixed-rate sampling - oversample slightly (faster than pixel clock)
    // to ensure we capture all pixels. Some duplicates are OK.
    // Loop body: LDR+STR ~10 cycles, SUBS+BNE ~4 = ~14 cycles
    // With 26 NOPs: ~40 cycles total → 15 MHz sample rate (1.2x oversample)
    volatile uint32_t *gpio6_ptr = &GPIO6_PSR;
    uint32_t *buf = dma_row_buffer;
    uint32_t count = SENSOR_COLUMNS;

    asm volatile(
      "1:                        \n\t"
      "  ldr r2, [%[gpio6]]      \n\t"   // Load GPIO6_PSR
      "  str r2, [%[buf]], #4    \n\t"   // Store and increment buf
      "  nop; nop; nop; nop      \n\t"   // 4
      "  subs %[cnt], #1         \n\t"   // Decrement counter
      "  bne 1b                  \n\t"   // Loop if not zero
      : [buf] "+r" (buf), [cnt] "+r" (count)
      : [gpio6] "r" (gpio6_ptr)
      : "r2", "memory", "cc"
    );

    uint32_t col = SENSOR_COLUMNS;  // We captured exactly SENSOR_COLUMNS

    // Re-enable interrupts
    __enable_irq();

    state.col = col;

    // Pack captured pixels (without bit reversal - do that after frame)
    uint32_t to_pack = (col < SENSOR_COLUMNS) ? col : SENSOR_COLUMNS;
    for (uint32_t c = 0; c < to_pack; c += 2) {
      uint16_t p0 = (dma_row_buffer[c] >> GPIO6_DATA_SHIFT) & 0xFFF;
      uint16_t p1 = (c + 1 < to_pack) ? ((dma_row_buffer[c + 1] >> GPIO6_DATA_SHIFT) & 0xFFF) : 0xFFF;
      pack_ptr[0] = p0 & 0xFF;
      pack_ptr[1] = ((p1 & 0x0F) << 4) | ((p0 >> 8) & 0x0F);
      pack_ptr[2] = (p1 >> 4) & 0xFF;
      pack_ptr += 3;
    }
    // Fill rest with 0xFFF (empty)
    for (uint32_t c = to_pack; c < SENSOR_COLUMNS; c += 2) {
      pack_ptr[0] = 0xFF;
      pack_ptr[1] = 0xFF;
      pack_ptr[2] = 0xFF;
      pack_ptr += 3;
    }

    // Turn off trigger at midpoint
    if (row == SENSOR_ROWS / 2) {
      GPIO7_DR_CLEAR = (1 << 1);  // Pin 12 is GPIO7 bit 1
    }
  }

done:
  // Post-process: apply bit-reversal to the entire packed buffer
  Serial.write("Applying bit-reversal...\n");
  for (uint32_t i = 0; i < PACKED_FRAME_SIZE; i += 3) {
    // Unpack two 12-bit values
    uint16_t p0 = packed_buffer[i] | ((packed_buffer[i+1] & 0x0F) << 8);
    uint16_t p1 = (packed_buffer[i+1] >> 4) | (packed_buffer[i+2] << 4);
    // Bit-reverse
    p0 = reverse_table[p0];
    p1 = reverse_table[p1];
    // Re-pack
    packed_buffer[i] = p0 & 0xFF;
    packed_buffer[i+1] = ((p1 & 0x0F) << 4) | ((p0 >> 8) & 0x0F);
    packed_buffer[i+2] = (p1 >> 4) & 0xFF;
  }

  state.state = STATE_DONE;
  Serial.write("Done!\n");
  Serial.flush();
  arm_dcache_flush_delete(packed_buffer, PACKED_FRAME_SIZE);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1);

  // Check PSRAM availability and allocate packed buffer
  // 12-bit packed: 2 pixels = 3 bytes
  uint32_t psram_size = external_psram_size * 1024 * 1024;

  Serial.printf("PSRAM size: %lu MB, packed frame: %lu bytes\n", psram_size / (1024*1024), PACKED_FRAME_SIZE);

  if (psram_size >= PACKED_FRAME_SIZE) {
    packed_buffer = (uint8_t *)extmem_malloc(PACKED_FRAME_SIZE);
    if (packed_buffer) {
      Serial.printf("Allocated %lu bytes in EXTMEM at %p\n", PACKED_FRAME_SIZE, packed_buffer);
    } else {
      Serial.println("ERROR: Failed to allocate EXTMEM!");
    }
  } else {
    Serial.printf("ERROR: PSRAM too small! Need %lu bytes but only have %lu\n", PACKED_FRAME_SIZE, psram_size);
  }

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
#ifdef BINNING_2X2
  digitalWrite(PIN_BIN0, HIGH);  // 2x2 binning
  digitalWrite(PIN_BIN1, LOW);
#else
  digitalWrite(PIN_BIN0, LOW);   // 1x1 no binning
  digitalWrite(PIN_BIN1, LOW);
#endif

  // Setup DMA capture triggered by PCLK
  setup_dma_capture();

  usb_custom_set_handler(usb_handler);
  state.state = STATE_IDLE;
}

void loop() {
  if (state.state == STATE_WAITING_ON_VSYNC) {
    acquire_frame();
  }
}
