#ifndef BRIDGE_H
#define BRIDGE_H

#include "driver/spi_common.h"
#include "driver/spi_slave.h"

// will need a spi interface
// configure, init, add device

// when it gets an a2dp stream it should decode and send over spi :)

#define MAX_SPI_TRANSFER_CHUNK 2048 //for a2dp

#define BRIDGE_DEV SPI2_HOST

#define EXT_MCU_ON_FLAG (0x01 << 7)
#define EXT_MCU_BT_FLAG (0x01 << 1)

extern uint8_t status_flags;

esp_err_t init_SPI(void);

// void transmit_SPI(uint8_t* data, uint32_t len);


#endif
