/*
 * AD7779.c
 *
 *  Created on: Jun 1, 2020
 *      Author: derryn
 */

#include "main.h"


#define GENERAL_USER_CONFIG_1 0x011
#define GENERAL_USER_CONFIG_2 0x012
#define GENERAL_USER_CONFIG_3 0x013
#define DOUT_FORMAT 0x014
#define ADC_MUX_CONFIG 0x015
#define BUFFER_CONFIG_1 0x019

#define SRC_N_MSB 0x060
#define SRC_N_LSB 0x061
#define SRC_UPDATE 0x064

SPI_HandleTypeDef *hspi;
  /*
   * Note using USART, which has a reversed bit order to normal SPI.
   *
   * Therefore all transactions need the byte bit order reversed.
   */

  // from https://stackoverflow.com/questions/2602823/in-c-c-whats-the-simplest-way-to-reverse-the-order-of-bits-in-a-byte
  unsigned char reverse(unsigned char b) {
     b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
     b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
     b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
     return b;
  }

  void AD7779_reset(void){
	  HAL_GPIO_WritePin(ADC_nRESET_GPIO_Port, ADC_nRESET_Pin, RESET);
	  HAL_GPIO_WritePin(SPI_nCS_GPIO_Port, SPI_nCS_Pin, SET);

	  HAL_Delay(100);
	  HAL_GPIO_WritePin(ADC_nRESET_GPIO_Port, ADC_nRESET_Pin, SET);
	  HAL_Delay(100);

  }




  void AD7779_write_register(uint8_t reg_address, uint8_t data){


	  uint8_t address = (reg_address & ~(1 << 7)); // Clear the R/nW bit
	  uint8_t rev_data = (data);

  	  HAL_GPIO_WritePin(SPI_nCS_GPIO_Port, SPI_nCS_Pin, RESET);
//  	  HAL_USART_Transmit(husart, &address, 1, 5);
//  	  HAL_USART_Transmit(husart, &rev_data, 1, 5);
  	  HAL_SPI_Transmit(hspi, &address, 1, 5);
  	  HAL_SPI_Transmit(hspi, &rev_data, 1, 5);
  	  HAL_GPIO_WritePin(SPI_nCS_GPIO_Port, SPI_nCS_Pin, SET);

  }



  uint8_t AD7779_read_register(uint8_t reg_address){

	  uint8_t rec_data;
	  uint8_t address = reg_address | (1 << 7); // Set the read bit

  	  HAL_GPIO_WritePin(SPI_nCS_GPIO_Port, SPI_nCS_Pin, RESET);
  	  //HAL_USART_Transmit(husart, &address, 1, 5);
  	  HAL_SPI_Transmit(hspi, &address, 1, 5);

  	  // Clear the current SPI DR
  	  do {
  		  hspi->Instance->DR;
  	  } while(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE) != RESET);

  	  HAL_SPI_Receive(hspi, &rec_data, 1, 5);
  	  //HAL_USART_Receive(husart, &rec_data, 1, 5);

  	  HAL_GPIO_WritePin(SPI_nCS_GPIO_Port, SPI_nCS_Pin, SET);

  	  return (rec_data);
  }

  void AD7779_read_channel_status_reg(uint8_t *status_regs){
	  for(int i=0; i < 4; i++){
		  *(status_regs + i) = AD7779_read_register(0x4C + i);
	  }
  }

volatile uint8_t reg;

  void AD7779_init(SPI_HandleTypeDef *spi_handle){
	  hspi = spi_handle;

	  AD7779_reset();

	  reg = AD7779_read_register(GENERAL_USER_CONFIG_1);


	  AD7779_write_register(GENERAL_USER_CONFIG_1, 0x74); // Enable high res mode
	  AD7779_write_register(BUFFER_CONFIG_1, 0x10); // Enable +ve reference input buffer, disable negative (as connected straight to ground.)


	  AD7779_write_register(ADC_MUX_CONFIG, 0x00); // Set to external reference (which is connected to the internal reference via LP filter.)

	  AD7779_write_register(DOUT_FORMAT, 0xE0); // Set to single data lane, CRC header format, 1 MCLK div for dout

	  // Set sps to 12800
	  AD7779_write_register(SRC_N_MSB, 0x00);
	  AD7779_write_register(SRC_N_LSB, 0xA0);
	  // Load the new div values
	  AD7779_write_register(SRC_UPDATE, 0x01);
	  HAL_Delay(1);
	  AD7779_write_register(SRC_UPDATE, 0x00);
  }

