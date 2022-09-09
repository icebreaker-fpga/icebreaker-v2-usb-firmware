
#include <stdint.h>
#include <stdbool.h>


void flash_spi_init();
void flash_spi_deinit();
bool is_flash_spi_inited();

void ice40_reset_hold(void);
void ice40_reset_release(void);



void ice40_spi_init();
void ice40_spi_deinit();
bool is_ice40_spi_inited(void);