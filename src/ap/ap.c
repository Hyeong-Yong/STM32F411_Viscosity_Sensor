/*
 * ap.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "ap.h"




void apInit(void)
{
  cliOpen(_DEF_UART1, 57600);
  uartOpen(_DEF_UART2, 57600);
  i2cOpen(_DEF_I2C1, 400);  // mcp4725,  DAC to current
  i2cOpen(_DEF_I2C2, 400);	// ina219,   current sensor
  spiOpen(_DEF_SPI1);		// max31865, temperature sensor
}

void apMain(void)
{
  uint32_t pre_time;
  uint32_t rx_data;

  pre_time = millis();
  while(1)
  {
    if (millis()-pre_time >= 500)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);
    }

    if (uartAvailable(_DEF_UART2)>0){
    	rx_data = uartRead(_DEF_UART2);
    	uartPrintf(_DEF_UART2, "Rx : %d \n", rx_data);
    }


    cliMain();
  }
}

