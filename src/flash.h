#ifndef FLASH_H_
#define FLASH_H_

#include "ch32v30x.h"

void SPI_Flash_ReadUUID(u8* uuid);
void SPI_Flash_WAKEUP(void);

#define FLASH_64K_BLOCK_ERASE_SIZE (64*1024)
#define FLASH_4K_BLOCK_ERASE_SIZE (4*1024)

#endif /* FLASH_H_ */
