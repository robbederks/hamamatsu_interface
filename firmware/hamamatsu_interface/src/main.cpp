#include <Arduino.h>
#include <usb_custom.h>
#include <imxrt.h>
#include "reverse_table.h"

// C7942 native resolution is 2400x2400 at 15.15 MHz PCLK
// Direct PSRAM writes via D-cache (WB-WA), FlexSPI2 clocked at 132 MHz
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
  uint8_t stop_reason;       // 0=normal, 1=hsync_timeout
  uint32_t dma_wait_total;   // total DMA wait loop iterations
  uint32_t overhead_max_us;  // max inter-row overhead in microseconds
} readout_state;
volatile readout_state state;

typedef struct __attribute__((__packed__)) {
  uint8_t command;
  uint32_t data_len;
  uint8_t data[64 - 5];
} control_req_t;

// 12-bit packed buffer in PSRAM — writes go directly via D-cache (WB-WA)
// Packed size: ROWS * COLS * 3 / 2 bytes (2 pixels = 3 bytes)
#define PACKED_FRAME_SIZE ((uint32_t)SENSOR_ROWS * SENSOR_COLUMNS * 3 / 2)
#define PACKED_ROW_BYTES (SENSOR_COLUMNS * 3 / 2)  // 3600 bytes per row
uint8_t *packed_buffer = nullptr;

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
        // Initialize packed buffer to 0xAA (which unpacks to 0xAAA, 0xAAA)
        // This allows test_frame.py to detect uncaptured pixels
        memset(packed_buffer, 0xAA, PACKED_FRAME_SIZE);
        arm_dcache_flush_delete(packed_buffer, PACKED_FRAME_SIZE);
        state.state = STATE_WAITING_ON_VSYNC;
        state.row = 0;
        state.col = 0;
        state.stop_reason = 0;
        state.dma_wait_total = 0;
        state.overhead_max_us = 0;
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

// Bit positions for sync signals (Pin 26=PCLK, Pin 38=VSYNC, Pin 39=HSYNC)
// Teensy 4.1: Pin 26 -> bit 30, Pin 38 -> bit 28, Pin 39 -> bit 29
#define PCLK_BIT  30
#define VSYNC_BIT 28
#define HSYNC_BIT 29
#define PCLK_MASK  (1UL << PCLK_BIT)
#define VSYNC_MASK (1UL << VSYNC_BIT)
#define HSYNC_MASK (1UL << HSYNC_BIT)


// PCLK-synchronized capture: direct PSRAM writes via D-cache (WB-WA)
// C7942: 15.15 MHz PCLK (66 ns/pixel, 39.6 CPU cycles/pixel at 600 MHz)
// FlexSPI2 clocked at 132 MHz (up from 88 MHz). D-cache absorbs strb writes;
// evictions write 32-byte cache lines to PSRAM. Cache write-allocate fills
// cause ~400 ns stalls per line (at 132 MHz), 113 lines/row = ~45 us overhead.
// Total row time: ~160 us capture + ~45 us overhead = ~205 us < 208 us period.
FASTRUN void acquire_frame() {
  if (packed_buffer == nullptr) {
    state.state = STATE_IDLE;
    return;
  }

  // Wait for VSYNC rising edge
  while (!(GPIO6_PSR & VSYNC_MASK)) {
    if (state.state != STATE_WAITING_ON_VSYNC) return;
  }

  state.state = STATE_ACQUIRING;

  uint8_t *psram_ptr = packed_buffer;
  uint32_t row_max_cycles = 0;

  // Disable interrupts for entire frame
  __disable_irq();

  // Skip initial rows to center the vertical crop
  for (uint32_t skip = 0; skip < SKIP_ROWS; skip++) {
    while (!(GPIO6_PSR & HSYNC_MASK)) {}
    while (GPIO6_PSR & HSYNC_MASK) {}
  }

  for (uint32_t row = 0; row < SENSOR_ROWS; row++) {
    state.row = row;

    // Wait for HSYNC high (start of row) with timeout
    uint32_t timeout = 10000000;
    while (!(GPIO6_PSR & HSYNC_MASK)) {
      if (--timeout == 0) {
        __enable_irq();
        state.stop_reason = 1;  // HSYNC timeout
        Serial.printf("HSYNC timeout at row %lu\n", row);
        goto done;
      }
    }

    uint32_t t0 = ARM_DWT_CYCCNT;

    // Capture directly to PSRAM via D-cache (WB-WA absorbs strb writes)
    volatile uint32_t *gpio6_ptr = &GPIO6_PSR;
    uint8_t *pack = psram_ptr;  // Write directly to PSRAM address
    uint32_t count = SENSOR_COLUMNS / 2;  // 1200 pixel pairs
    uint32_t pclk_mask = PCLK_MASK;

    // C7942 data sampling convention (see EXPERIMENTS.md Exp 3):
    // Tdd (34 ns) > Tppw (33 ns) means pixel N's data appears AFTER PCLK N falls.
    // When GPIO6_PSR shows PCLK=1, data bits = pixel N-1 (settled 32 ns ago).
    // We extract data from the SAME read that detected the edge — no re-read.
    // A "prime" read discards the first edge (pre-pixel-1 garbage), then
    // 1200 pairs read edges 2-2401, capturing pixels 1-2400.
    asm volatile(
      // -- Prime: read first PCLK edge, discard data --
      "0:                              \n\t"
      "  ldr r2, [%[gpio6]]           \n\t"
      "  tst r2, %[pclk]              \n\t"
      "  beq 0b                       \n\t"
      "5:                              \n\t"
      "  ldr r2, [%[gpio6]]           \n\t"
      "  tst r2, %[pclk]              \n\t"
      "  bne 5b                       \n\t"
      // -- Main loop: 1200 pixel pairs --
      // -- Pixel A: wait for PCLK rising, extract from edge-detect read --
      "1:                              \n\t"
      "  ldr r2, [%[gpio6]]           \n\t"
      "  tst r2, %[pclk]              \n\t"
      "  beq 1b                       \n\t"
      "  ubfx r3, r2, #16, #12        \n\t"  // pixel A: bits[27:16] of edge read
      // -- Wait for PCLK falling edge --
      "2:                              \n\t"
      "  ldr r2, [%[gpio6]]           \n\t"
      "  tst r2, %[pclk]              \n\t"
      "  bne 2b                       \n\t"
      // -- Pixel B: wait for PCLK rising, extract from edge-detect read --
      "3:                              \n\t"
      "  ldr r2, [%[gpio6]]           \n\t"
      "  tst r2, %[pclk]              \n\t"
      "  beq 3b                       \n\t"
      "  ubfx r4, r2, #16, #12        \n\t"  // pixel B: bits[27:16] of edge read
      // -- Pack pixel pair into 3 bytes (PSRAM via D-cache) --
      "  strb r3, [%[pack]], #1       \n\t"  // byte0 = pA[7:0]
      "  lsr r5, r3, #8              \n\t"   // r5 = pA[11:8]
      "  and r2, r4, #0x0F           \n\t"   // r2 = pB[3:0]
      "  orr r5, r5, r2, lsl #4      \n\t"   // r5 = pB[3:0]<<4 | pA[11:8]
      "  strb r5, [%[pack]], #1       \n\t"  // byte1
      "  lsr r5, r4, #4              \n\t"   // r5 = pB[11:4]
      "  strb r5, [%[pack]], #1       \n\t"  // byte2
      // -- Wait for PCLK falling edge --
      "4:                              \n\t"
      "  ldr r2, [%[gpio6]]           \n\t"
      "  tst r2, %[pclk]              \n\t"
      "  bne 4b                       \n\t"
      // -- Loop --
      "  subs %[cnt], #1              \n\t"
      "  bne 1b                       \n\t"
      : [pack] "+r" (pack), [cnt] "+r" (count)
      : [gpio6] "r" (gpio6_ptr), [pclk] "r" (pclk_mask)
      : "r2", "r3", "r4", "r5", "memory", "cc"
    );

    // Measure row time (capture + cache stall overhead)
    uint32_t row_cycles = ARM_DWT_CYCCNT - t0;
    if (row_cycles > row_max_cycles) row_max_cycles = row_cycles;

    psram_ptr += PACKED_ROW_BYTES;
    state.col = SENSOR_COLUMNS;

    // Turn off trigger at midpoint
    if (row == SENSOR_ROWS / 2) {
      GPIO7_DR_CLEAR = (1 << 1);  // Pin 12 is GPIO7 bit 1
    }
  }

  __enable_irq();

done:
  state.dma_wait_total = row_max_cycles;  // repurpose field: max row cycles
  state.overhead_max_us = row_max_cycles / 600;  // cycles → us at 600 MHz
  Serial.printf("Capture diag: rows=%lu, row_max=%lu cycles (%lu us)\n",
                state.row, row_max_cycles, row_max_cycles / 600);

  // Post-process: apply bit-reversal to the entire packed buffer
  // Data is in D-cache from the capture writes — no invalidation needed.
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

  // Increase FlexSPI2 (PSRAM) clock from 88 MHz to 132 MHz
  // Source: PLL2 = 528 MHz, PODF=3 → div=4 → 132 MHz (PSRAM rated for 133 MHz)
  // This reduces cache write-allocate fill latency by 1.5x (~300→200 ns per line)
  CCM_CBCMR = (CCM_CBCMR & ~CCM_CBCMR_FLEXSPI2_PODF_MASK) | CCM_CBCMR_FLEXSPI2_PODF(3);

  // Enable DWT cycle counter for timing measurements
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  usb_custom_set_handler(usb_handler);
  state.state = STATE_IDLE;
}

void loop() {
  if (state.state == STATE_WAITING_ON_VSYNC) {
    acquire_frame();
  }
}
