#include "bridge.h"


#include <string.h> //for memset
#include <math.h>
#include "esp_log.h"

uint8_t status_flags = EXT_MCU_ON_FLAG;

spi_bus_config_t buscfg = {
    .miso_io_num = BRIDGE_PIN_MISO,
    .mosi_io_num = BRIDGE_PIN_MOSI,
    .sclk_io_num = BRIDGE_PIN_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};

size_t max_len = 0;
 
//Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = BRIDGE_PIN_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL
    };

esp_err_t init_SPI(void){
    esp_err_t ret = 0;

    ret = spi_slave_initialize(BRIDGE_DEV, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    
    return ret;
}
