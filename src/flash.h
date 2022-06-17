#ifndef FLASH_H_
#define FLASH_H_

#include "ch32v30x.h"

void SPI_FLASH_Write_SR(u8 sr);
void SPI_Flash_Wait_Busy(void);
void SPI_FLASH_Write_Enable(void);
void SPI_FLASH_Write_Disable(void);
void SPI_Flash_ReadUUID(u8* uuid);
void SPI_Flash_Erase_Sector(u32 Dst_Addr);
void SPI_Flash_Read(u8 *pBuffer, u32 ReadAddr, u16 size);
void SPI_Flash_Write_Page(u8 *pBuffer, u32 WriteAddr, u16 size);
void SPI_Flash_Write_NoCheck(u8 *pBuffer, u32 WriteAddr, u16 size);
void SPI_Flash_Write(u8 *pBuffer, u32 WriteAddr, u16 size);
void SPI_Flash_Erase_Chip(void);
void SPI_Flash_PowerDown(void);
void SPI_Flash_WAKEUP(void);

#define FLASH_64K_BLOCK_ERASE_SIZE (64*1024)
#define FLASH_4K_BLOCK_ERASE_SIZE (4*1024)

#endif /* FLASH_H_ */
