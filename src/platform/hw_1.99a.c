
#include "hw_1.99a.h"
#include "ch32v30x.h"

static bool flash_spi_inited = false;
static bool ice40_spi_inited = false;

void flash_spi_init(){

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  GPIO_SetBits(GPIOC, GPIO_Pin_10);
  GPIO_Init(
    GPIOC, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_Out_PP,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_10,
    }
  );

  GPIO_SetBits(GPIOC, GPIO_Pin_11);
  GPIO_Init(
    GPIOC, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_Out_PP,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_11,
    }
  );

  GPIO_SetBits(GPIOA, GPIO_Pin_4);
  GPIO_Init(
    GPIOA, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_Out_PP,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_4,
    }
  );

  GPIO_Init(
    GPIOA, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_AF_PP,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_5,
    }
  );
  GPIO_Init(
    GPIOA, 
    &(GPIO_InitTypeDef) {
      /* Signal: POCI - Leave floating, AF is picked up automatically on inputs */
      .GPIO_Mode = GPIO_Mode_IN_FLOATING, 
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_6,
    }
  );
  GPIO_Init(
    GPIOA, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_AF_PP,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_7,
    }
  );


  /* Typical SPI0 configuration */
  SPI_Init(SPI1, &(SPI_InitTypeDef){
    .SPI_Direction = SPI_Direction_2Lines_FullDuplex,
    .SPI_Mode = SPI_Mode_Master,
    .SPI_DataSize = SPI_DataSize_8b,
    .SPI_CPOL = SPI_CPOL_High,
    .SPI_CPHA = SPI_CPHA_2Edge,
    .SPI_NSS = SPI_NSS_Soft,
    .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4,
    .SPI_FirstBit = SPI_FirstBit_MSB,
    .SPI_CRCPolynomial = 7,
  });

  SPI_Cmd(SPI1, ENABLE);

  flash_spi_inited = true;
}


void flash_spi_deinit(){

  GPIO_Init(
    GPIOC, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_IPU,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_10,
    }
  );

  GPIO_Init(
    GPIOC, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_IPU, 
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_11,
    }
  );

  GPIO_Init(
    GPIOA, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_IN_FLOATING, 
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_4,
    }
  );
  GPIO_SetBits(GPIOA, GPIO_Pin_4);

  GPIO_Init(
    GPIOA, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_IN_FLOATING, 
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_5,
    }
  );
  GPIO_Init(
    GPIOA, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_IN_FLOATING, 
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_6,
    }
  );
  GPIO_Init(
    GPIOA, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_IN_FLOATING,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_7,
    }
  );

  SPI_Cmd(SPI1, DISABLE);

  flash_spi_inited = false;
}

bool is_flash_spi_inited(void) {
  return flash_spi_inited;
}

void ice40_spi_init(){

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  //GPIO_SetBits(GPIOB, GPIO_Pin_12);
  GPIO_Init(
    GPIOB, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_Out_PP,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_12,
    }
  );

  GPIO_Init(
    GPIOB, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_AF_PP,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_13,
    }
  );
  GPIO_Init(
    GPIOB, 
    &(GPIO_InitTypeDef) {
      /* Signal: POCI - Leave floating, AF is picked up automatically on inputs */
      .GPIO_Mode = GPIO_Mode_IN_FLOATING, 
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_14,
    }
  );
  GPIO_Init(
    GPIOB, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_AF_PP,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_15,
    }
  );


  /* Typical SPI2 configuration */
  SPI_Init(SPI2, &(SPI_InitTypeDef){
    .SPI_Direction = SPI_Direction_2Lines_FullDuplex,
    .SPI_Mode = SPI_Mode_Master,
    .SPI_DataSize = SPI_DataSize_8b,
    .SPI_CPOL = SPI_CPOL_High,
    .SPI_CPHA = SPI_CPHA_2Edge,
    .SPI_NSS = SPI_NSS_Soft,
    .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4,
    .SPI_FirstBit = SPI_FirstBit_MSB,
    .SPI_CRCPolynomial = 7,
  });

  SPI_Cmd(SPI2, ENABLE);

  ice40_spi_inited = true;
}


void ice40_spi_deinit(){

  GPIO_Init(
    GPIOB, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_IPU,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_12,
    }
  );

  GPIO_Init(
    GPIOB, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_IN_FLOATING, 
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_13,
    }
  );

  GPIO_Init(
    GPIOB, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_IN_FLOATING, 
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_14,
    }
  );

  GPIO_Init(
    GPIOB, 
    &(GPIO_InitTypeDef) {
      .GPIO_Mode = GPIO_Mode_IN_FLOATING, 
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Pin = GPIO_Pin_15,
    }
  );
  
  SPI_Cmd(SPI2, DISABLE);

  ice40_spi_inited = false;
}

bool is_ice40_spi_inited(void) {
  return ice40_spi_inited;
}


void ice40_reset_hold(){
  /* Hold iCE40 in reset */
  GPIO_ResetBits(GPIOA, GPIO_Pin_10);
}

void ice40_reset_release(){
  /* Release iCE40 from reset */
  GPIO_SetBits(GPIOA, GPIO_Pin_10);
}