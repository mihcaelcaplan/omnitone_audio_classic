#ifndef BRIDGE_H
#define BRIDGE_H

#include <stdint.h>
#include "driver/spi_common.h"
#include "driver/spi_slave.h"

// SPI slave interface to the nRF5340. The nRF is the master and polls us; we
// answer one transaction late (see bt_spi_task_handler in bt_app_core.c).

#define MAX_SPI_TRANSFER_CHUNK 2048 // largest OTA_DATA payload

// Frame layout. The master pads every frame to a multiple of 4 bytes because
// the ESP32 slave DMA only lands whole words in rx_buf.
#define BRIDGE_WORD_LEN          4
#define BRIDGE_CMD_ONLY_LEN      4  // any command with no payload: [cmd] + 3 pad
#define BRIDGE_OTA_BEGIN_LEN     8  // [cmd][size u32] + 3 pad
#define BRIDGE_OTA_DATA_HDR_LEN  5  // [cmd][seq u16][len u16] ahead of the payload
#define BRIDGE_OTA_DATA_CRC_LEN  4  // crc32 after the payload
#define BRIDGE_ROUND_UP(n, m)    ((((n) + (m) - 1) / (m)) * (m))
// what the master clocks for an OTA_DATA frame carrying len payload bytes
#define BRIDGE_OTA_DATA_FRAME_LEN(len) \
    BRIDGE_ROUND_UP(BRIDGE_OTA_DATA_HDR_LEN + (len) + BRIDGE_OTA_DATA_CRC_LEN, BRIDGE_WORD_LEN)

// Largest frame either side will ever clock in one transaction. OTA_DATA is
// the biggest: hdr + payload + crc = 2057, padded to a word boundary. Both DMA
// buffers are this size - tx as well as rx, because the DMA reads .length
// bits from tx_buffer no matter how short the reply is, and a smaller tx
// buffer would clock adjacent heap out to the master.
#define BRIDGE_FRAME_MAX (MAX_SPI_TRANSFER_CHUNK + 16)

#define BRIDGE_DEV SPI2_HOST

// slave pins; CS is also read as a plain GPIO before each transaction is armed
#define BRIDGE_PIN_MISO 12
#define BRIDGE_PIN_MOSI 13
#define BRIDGE_PIN_CLK  14
#define BRIDGE_PIN_CS   15

// status_flags bits, read via BRIDGE_CMD_STATUS
#define EXT_MCU_ON_FLAG          (0x01 << 7)
#define EXT_MCU_OTA_ERROR_FLAG   (0x01 << 4) // last OTA attempt failed or was rolled back; details via OTA_STATE
#define EXT_MCU_OTA_PENDING_FLAG (0x01 << 3) // new image booted, waiting for OTA_CONFIRM
#define EXT_MCU_OTA_BUSY_FLAG    (0x01 << 2) // erase / receive / verify in progress, BT is quiesced
#define EXT_MCU_BT_FLAG          (0x01 << 1)

// Command byte, first byte of every frame from the master. Replies are tagged
// with (cmd | BRIDGE_REPLY_TAG) in their first byte so the master can tell
// which command a reply answers.
typedef enum {
    BRIDGE_CMD_NONE        = 0x00, // empty poll: master is only clocking out our last reply
    BRIDGE_CMD_STATUS      = 0x0A, // -> [0x8A][status_flags]
    BRIDGE_CMD_VERSION     = 0x0B, // -> [0x8B][32-byte app version string, NUL padded]
    BRIDGE_CMD_OTA_BEGIN   = 0x20, // [size u32 LE]                              -> [0xA0][result]
    BRIDGE_CMD_OTA_DATA    = 0x21, // [seq u16][len u16][payload <= 2048][crc32]  -> [0xA1][result][next_seq u16]
    BRIDGE_CMD_OTA_END     = 0x22, //                                             -> [0xA2][result]
    BRIDGE_CMD_OTA_STATE   = 0x23, //                                             -> [0xA3][state][err u16][next_seq u16][percent]
    BRIDGE_CMD_OTA_REBOOT  = 0x24, //                                             -> [0xA4][result], restart follows ~500ms later
    BRIDGE_CMD_OTA_CONFIRM = 0x25, //                                             -> [0xA5][result]
    BRIDGE_CMD_OTA_ABORT   = 0x26, //                                             -> [0xA6][result]
} bridge_cmd_t;

#define BRIDGE_REPLY_TAG 0x80

// Reply payload byte for a command byte we don't recognise
#define BRIDGE_RESULT_UNKNOWN_CMD 0xFF

extern uint8_t status_flags;

esp_err_t init_SPI(void);

// void transmit_SPI(uint8_t* data, uint32_t len);


#endif
