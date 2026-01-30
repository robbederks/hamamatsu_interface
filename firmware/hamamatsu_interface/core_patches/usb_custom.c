#include "usb_dev.h"
#include "usb_serial.h"
#include "debug/printf.h"
#include "avr/pgmspace.h" // for PROGMEM, DMAMEM, FASTRUN
#include <string.h>

#define RX_NUM  8
#define CUSTOM_RX_SIZE 64

static uint16_t tx_packet_size = 0;
static transfer_t tx_transfer[RX_NUM] __attribute__ ((used, aligned(32)));
static transfer_t rx_transfer[RX_NUM] __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t rx_buffer[RX_NUM][CUSTOM_RX_SIZE] __attribute__ ((aligned(32)));
extern volatile uint8_t usb_high_speed;

uint32_t (*control_handler)(uint8_t *control_data, uint32_t len, uint8_t *return_data, uint32_t max_return_len) = NULL;

static void rx_queue_transfer(int i) {
	NVIC_DISABLE_IRQ(IRQ_USB1);
	usb_prepare_transfer(&rx_transfer[i], &rx_buffer[i], CUSTOM_RX_SIZE, i);
	arm_dcache_delete(&rx_buffer[i], CUSTOM_RX_SIZE);
	usb_receive(CUSTOM_RX_ENDPOINT, &rx_transfer[i]);
	NVIC_ENABLE_IRQ(IRQ_USB1);
}

DMAMEM static uint8_t return_data[512] __attribute__ ((aligned(32)));
static void rx_event(transfer_t *t) {
	int len = CUSTOM_RX_SIZE - ((t->status >> 16) & 0x7FFF);
	int i = t->callback_param;

  uint32_t return_len = 0;
  if(control_handler != NULL) {
    return_len = control_handler(rx_buffer[i], len, &return_data[4], tx_packet_size - 4);
    if (return_len > tx_packet_size - 4) {
      printf("return_len > tx_packet_size\n");
      return_len = tx_packet_size - 4;
    }
    memcpy(return_data, &return_len, sizeof(uint32_t));
  } else {
    return_data[0] = 0x00;
    return_len = 1;
    printf("control_handler == NULL\n");
  }

  // queue response transfer
  NVIC_DISABLE_IRQ(IRQ_USB1);
  usb_prepare_transfer(&tx_transfer[i], return_data, return_len + 4, 0);
  arm_dcache_flush_delete(return_data, return_len + 4);
  usb_transmit(CUSTOM_TX_ENDPOINT, &tx_transfer[i]);
  NVIC_ENABLE_IRQ(IRQ_USB1);

  // Put buffer back in
  rx_queue_transfer(i);
}

void usb_custom_set_handler(uint32_t (*handler)(uint8_t *control_data, uint32_t len, uint8_t *return_data, uint32_t max_return_len)) {
  control_handler = handler;
}

static transfer_t bulk_transfer __attribute__ ((used, aligned(32)));
static uint8_t *bulk_data __attribute__ ((aligned(32))) = NULL;
static uint32_t bulk_len_remaining = 0;

void usb_custom_init_bulk_transfer(uint8_t *buffer, uint32_t len, uint8_t transfer_id) {
  (void)transfer_id; // unused for now

  // On first call, flush the entire buffer to ensure PSRAM has current data
  // (caller should have already flushed for EXTMEM buffers, but this ensures internal RAM works too)
  if (bulk_data != buffer || bulk_len_remaining == 0) {
    bulk_data = buffer;
    bulk_len_remaining = len;
    if (buffer != NULL && len > 0) {
      arm_dcache_flush_delete(buffer, len);
    }
  }

  if (bulk_data == NULL || bulk_len_remaining == 0) {
    return;
  }

  // queue transfer
  NVIC_DISABLE_IRQ(IRQ_USB1);
  uint32_t transfer_len = bulk_len_remaining > tx_packet_size ? tx_packet_size : bulk_len_remaining;

  usb_prepare_transfer(&bulk_transfer, bulk_data, transfer_len, 0);
  usb_transmit(CUSTOM_BULK_ENDPOINT, &bulk_transfer);
  NVIC_ENABLE_IRQ(IRQ_USB1);

  // update state
  bulk_data += transfer_len;
  bulk_len_remaining -= transfer_len;
}

static void bulk_event(transfer_t *t) {
  // queue next transfer
  usb_custom_init_bulk_transfer(bulk_data, bulk_len_remaining, 0);
}

void usb_custom_configure (void) {
  tx_packet_size = usb_high_speed ? 512 : 64;

  usb_config_rx(CUSTOM_RX_ENDPOINT, CUSTOM_RX_SIZE, 0, rx_event);
  usb_config_tx(CUSTOM_TX_ENDPOINT, tx_packet_size, 0, NULL);
  usb_config_tx(CUSTOM_BULK_ENDPOINT, tx_packet_size, 0, bulk_event);

  // init some rx transfers
  for (int i=0; i < RX_NUM; i++) rx_queue_transfer(i);

  printf("Custom USB configured\n");
}