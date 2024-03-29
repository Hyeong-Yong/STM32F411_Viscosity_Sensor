/*
 * ap.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "ap.h"





void apInit(void)
{
  //cliOpen(_DEF_UART1, 57600);
  uartOpen(_DEF_UART1, 57600);
  uartOpen(_DEF_UART2, 57600);
  i2cOpen(_DEF_I2C1, 400);  // mcp4725,  DAC to current
  //i2cOpen(_DEF_I2C2, 400);	// ina219,   current sensor
  spiOpen(_DEF_SPI1);		// max31865, temperature sensor
}

void apMain(void)
{
  uint32_t pre_time;

  uint8_t rx_data;
  uint8_t buf[4];
  uint8_t i =0;

  uint32_t viscosity;
  uint16_t DAC_viscosity;

  float temperature;
  float* p_temperature = &temperature;

  pre_time = millis();
  while(1)
  {

    if (uartAvailable(_DEF_UART1)>0){
     	rx_data = uartRead(_DEF_UART1);
    	buf[i]= rx_data;
    	i++;
    	if(i == 4){
    		viscosity  = buf[0];
    		viscosity |= buf[1] <<8;
    		viscosity |= buf[2] <<16;
    		viscosity |= buf[3] <<24;
    		i=0;
    		if (viscosity<=2000 && viscosity >=0){
    		DAC_viscosity = (uint16_t)viscosity+1000;
    		uartPrintf(_DEF_UART2, "Rx : %d \n", (DAC_viscosity-1000)/10);
    		mcp4725_setValue(DAC_viscosity, MCP4725_FAST_MODE, MCP4725_POWER_DOWN_OFF);
    		}
   		}
    }

    if (millis()-pre_time>= 1000){
      pre_time = millis();
      ledToggle(_DEF_LED1);


      bool spi_ret = max31865_readTempC(p_temperature);
      if (spi_ret == true){
    	  __IO uint16_t temperature_int;

    	  temperature_int=(uint16_t)(temperature*100);
    	  uint8_t tx_data[2];

		  tx_data[0] = (uint8_t)temperature_int;
		  tx_data[1] = (uint8_t)(temperature_int>>8);
   	  	  uartWrite(_DEF_UART1, tx_data, 2);

      	  }
      }

     //cliMain();

    }


}

