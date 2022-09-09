/*  
 *  Copyright 2020 Gregory Davill <greg.davill@gmail.com>
 *
 * Adapted from: https://github.com/norbertthiel
 * src: https://github.com/litex-hub/litespi/issues/52#issuecomment-890787356
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "ch32v30x.h"

#include "flash.h"


/* Winbond SPIFalsh ID */
#define W25Q80                   0XEF13
#define W25Q16                   0XEF14
#define W25Q32                   0XEF15
#define W25Q64                   0XEF16
#define W25Q128                  0XEF17

/* Winbond SPIFalsh Instruction List */
#define W25X_WriteEnable         0x06
#define W25X_WriteDisable        0x04
#define W25X_ReadStatusReg       0x05
#define W25X_WriteStatusReg      0x01
#define W25X_ReadData            0x03
#define W25X_FastReadData        0x0B
#define W25X_FastReadDual        0x3B
#define W25X_PageProgram         0x02
#define W25X_BlockErase          0xD8
#define W25X_SectorErase         0x20
#define W25X_ChipErase           0xC7
#define W25X_PowerDown           0xB9
#define W25X_ReleasePowerDown    0xAB
#define W25X_DeviceID            0xAB
#define W25X_ManufactDeviceID    0x90
#define W25X_JedecDeviceID       0x9F

/* Global define */

/* Global Variable */
u8       SPI_FLASH_BUF[4096];
const u8 TEXT_Buf[] = {"CH32F103 SPI FLASH W25Qxx"};
#define SIZE    sizeof(TEXT_Buf)

/*********************************************************************
 * @fn      SPI1_ReadWriteByte
 *
 * @brief   SPI1 read or write one byte.
 *
 * @param   TxData - write one byte data.
 *
 * @return  Read one byte data.
 */
u8 SPI1_ReadWriteByte(u8 TxData)
{
    u8 i = 0;

    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
    {
        i++;
        if(i > 200)
            return 0;
    }

    SPI_I2S_SendData(SPI1, TxData);
    i = 0;

    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
    {
        i++;
        if(i > 200)
            return 0;
    }

    return SPI_I2S_ReceiveData(SPI1);
}


/*********************************************************************
 * @fn      SPI_Flash_ReadSR
 *
 * @brief   Read W25Qxx status register.
 *        ����BIT7  6   5   4   3   2   1   0
 *        ����SPR   RV  TB  BP2 BP1 BP0 WEL BUSY
 *
 * @return  byte - status register value.
 */
u8 SPI_Flash_ReadSR(void)
{
    u8 byte = 0;

    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
    SPI1_ReadWriteByte(W25X_ReadStatusReg);
    byte = SPI1_ReadWriteByte(0Xff);
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);

    return byte;
}

/*********************************************************************
 * @fn      SPI_FLASH_Write_SR
 *
 * @brief   Write W25Qxx status register.
 *
 * @param   sr - status register value.
 *
 * @return  none
 */
void SPI_FLASH_Write_SR(u8 sr)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
    SPI1_ReadWriteByte(W25X_WriteStatusReg);
    SPI1_ReadWriteByte(sr);
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
}

/*********************************************************************
 * @fn      SPI_Flash_Wait_Busy
 *
 * @brief   Wait flash free.
 *
 * @return  none
 */
void SPI_Flash_Wait_Busy(void)
{
    while((SPI_Flash_ReadSR() & 0x01) == 0x01)
        ;
}

/*********************************************************************
 * @fn      SPI_FLASH_Write_Enable
 *
 * @brief   Enable flash write.
 *
 * @return  none
 */
void SPI_FLASH_Write_Enable(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
    SPI1_ReadWriteByte(W25X_WriteEnable);
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
}

/*********************************************************************
 * @fn      SPI_FLASH_Write_Disable
 *
 * @brief   Disable flash write.
 *
 * @return  none
 */
void SPI_FLASH_Write_Disable(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
    SPI1_ReadWriteByte(W25X_WriteDisable);
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
}

/*********************************************************************
 * @fn      SPI_Flash_ReadID
 *
 * @brief   Read flash ID.
 *
 * @return  Temp - FLASH ID.
 */
u16 SPI_Flash_ReadID(void)
{
    u16 Temp = 0;

    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
    SPI1_ReadWriteByte(W25X_ManufactDeviceID);
    SPI1_ReadWriteByte(0x00);
    SPI1_ReadWriteByte(0x00);
    SPI1_ReadWriteByte(0x00);
    Temp |= SPI1_ReadWriteByte(0xFF) << 8;
    Temp |= SPI1_ReadWriteByte(0xFF);
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);

    return Temp;
}


/*********************************************************************
 * @fn      SPI_Flash_ReadUUID
 *
 * @brief   Read flash UUID.
 * 
 * @param   buf - buffer to store UUID
 *
 * @return  none
 */
void SPI_Flash_ReadUUID(u8* uuid)
{
    static u8 _uuid[8];
    static bool _cached = false;
    
    if(_cached == false){
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
        SPI1_ReadWriteByte(0x48);
        SPI1_ReadWriteByte(0x00);
        SPI1_ReadWriteByte(0x00);
        SPI1_ReadWriteByte(0x00);
        SPI1_ReadWriteByte(0x00);

        for(int i=0; i < 8; i++)
            _uuid[i] = SPI1_ReadWriteByte(0xFF);

        GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
        _cached = true;
    }

    if(uuid){
        for(int i=0; i < 8; i++)
            uuid[i] = _uuid[i];
    }
}

/*********************************************************************
 * @fn      SPI_Flash_Erase_Sector
 *
 * @brief   Erase one sector(4Kbyte).
 *
 * @param   Dst_Addr - 0 ���� 2047
 *
 * @return  none
 */
void SPI_Flash_Erase_Sector(u32 Dst_Addr)
{
    SPI_FLASH_Write_Enable();
    SPI_Flash_Wait_Busy();
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
    SPI1_ReadWriteByte(W25X_SectorErase);
    SPI1_ReadWriteByte((u8)((Dst_Addr) >> 16));
    SPI1_ReadWriteByte((u8)((Dst_Addr) >> 8));
    SPI1_ReadWriteByte((u8)Dst_Addr);
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
    SPI_Flash_Wait_Busy();
}

/*********************************************************************
 * @fn      SPI_Flash_Read
 *
 * @brief   Read data from flash.
 *
 * @param   pBuffer -
 *          ReadAddr -Initial address(24bit).
 *          size - Data length.
 *
 * @return  none
 */
void SPI_Flash_Read(u8 *pBuffer, u32 ReadAddr, u16 size)
{
    u16 i;

    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
    SPI1_ReadWriteByte(W25X_ReadData);
    SPI1_ReadWriteByte((u8)((ReadAddr) >> 16));
    SPI1_ReadWriteByte((u8)((ReadAddr) >> 8));
    SPI1_ReadWriteByte((u8)ReadAddr);

    for(i = 0; i < size; i++)
    {
        pBuffer[i] = SPI1_ReadWriteByte(0XFF);
    }

    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
}

/*********************************************************************
 * @fn      SPI_Flash_Write_Page
 *
 * @brief   Write data by one page.
 *
 * @param   pBuffer -
 *          WriteAddr - Initial address(24bit).
 *          size - Data length.
 *
 * @return  none
 */
void SPI_Flash_Write_Page(u8 *pBuffer, u32 WriteAddr, u16 size)
{
    u16 i;

    SPI_FLASH_Write_Enable();
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
    SPI1_ReadWriteByte(W25X_PageProgram);
    SPI1_ReadWriteByte((u8)((WriteAddr) >> 16));
    SPI1_ReadWriteByte((u8)((WriteAddr) >> 8));
    SPI1_ReadWriteByte((u8)WriteAddr);

    for(i = 0; i < size; i++)
    {
        SPI1_ReadWriteByte(pBuffer[i]);
    }

    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
    SPI_Flash_Wait_Busy();
}

/*********************************************************************
 * @fn      SPI_Flash_Write_NoCheck
 *
 * @brief   Write data to flash.(need Erase)
 *          All data in address rang is 0xFF.
 *
 * @param   pBuffer -
 *          WriteAddr - Initial address(24bit).
 *          size - Data length.
 *
 * @return  none
 */
void SPI_Flash_Write_NoCheck(u8 *pBuffer, u32 WriteAddr, u16 size)
{
    u16 pageremain;

    pageremain = 256 - WriteAddr % 256;

    if(size <= pageremain)
        pageremain = size;

    while(1)
    {
        SPI_Flash_Write_Page(pBuffer, WriteAddr, pageremain);

        if(size == pageremain)
        {
            break;
        }
        else
        {
            pBuffer += pageremain;
            WriteAddr += pageremain;
            size -= pageremain;

            if(size > 256)
                pageremain = 256;
            else
                pageremain = size;
        }
    }
}

/*********************************************************************
 * @fn      SPI_Flash_Write
 *
 * @brief   Write data to flash.(no need Erase)
 *
 * @param   pBuffer -
 *          WriteAddr - Initial address(24bit).
 *          size - Data length.
 *
 * @return  none
 */
void SPI_Flash_Write(u8 *pBuffer, u32 WriteAddr, u16 size)
{
    u32 secpos;
    u16 secoff;
    u16 secremain;
    u16 i;

    secpos = WriteAddr / 4096;
    secoff = WriteAddr % 4096;
    secremain = 4096 - secoff;

    if(size <= secremain)
        secremain = size;

    while(1)
    {
        SPI_Flash_Read(SPI_FLASH_BUF, secpos * 4096, 4096);

        for(i = 0; i < secremain; i++)
        {
            if(SPI_FLASH_BUF[secoff + i] != 0XFF)
                break;
        }

        if(i < secremain)
        {
            SPI_Flash_Erase_Sector(secpos);

            for(i = 0; i < secremain; i++)
            {
                SPI_FLASH_BUF[i + secoff] = pBuffer[i];
            }

            SPI_Flash_Write_NoCheck(SPI_FLASH_BUF, secpos * 4096, 4096);
        }
        else
        {
            SPI_Flash_Write_NoCheck(pBuffer, WriteAddr, secremain);
        }

        if(size == secremain)
        {
            break;
        }
        else
        {
            secpos++;
            secoff = 0;

            pBuffer += secremain;
            WriteAddr += secremain;
            size -= secremain;

            if(size > 4096)
            {
                secremain = 4096;
            }
            else
            {
                secremain = size;
            }
        }
    }
}

/*********************************************************************
 * @fn      SPI_Flash_Erase_Chip
 *
 * @brief   Erase all FLASH pages.
 *
 * @return  none
 */
void SPI_Flash_Erase_Chip(void)
{
    SPI_FLASH_Write_Enable();
    SPI_Flash_Wait_Busy();
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
    SPI1_ReadWriteByte(W25X_ChipErase);
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
    SPI_Flash_Wait_Busy();
}

/*********************************************************************
 * @fn      SPI_Flash_PowerDown
 *
 * @brief   Enter power down mode.
 *
 * @return  none
 */
void SPI_Flash_PowerDown(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
    SPI1_ReadWriteByte(W25X_PowerDown);
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
}

/*********************************************************************
 * @fn      SPI_Flash_WAKEUP
 *
 * @brief   Power down wake up.
 *
 * @return  none
 */
void SPI_Flash_WAKEUP(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);
    SPI1_ReadWriteByte(W25X_ReleasePowerDown);
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);
}
