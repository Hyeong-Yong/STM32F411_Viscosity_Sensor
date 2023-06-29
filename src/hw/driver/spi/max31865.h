/*
 * max31865.h
 *
 *  Created on: 2023. 6. 29.
 *      Author: hwang
 */

#ifndef SRC_HW_DRIVER_SPI_MAX31865_H_
#define SRC_HW_DRIVER_SPI_MAX31865_H_


#include <stdbool.h>
#include <math.h>
#include "spi.h"


#include "gpio.h"


#define _MAX31865_RREF      430.0f
#define _MAX31865_RNOMINAL  100.0f

//#########################################################################################################################
typedef struct
{
  GPIO_TypeDef      *cs_gpio;
  uint16_t          cs_pin;
  SPI_HandleTypeDef *spi;
  uint8_t           lock;

}Max31865_t;
//#########################################################################################################################
void  Max31865_init(Max31865_t *max31865,SPI_HandleTypeDef *spi,GPIO_TypeDef  *cs_gpio,uint16_t cs_pin,uint8_t  numwires, uint8_t filterHz);
bool  Max31865_readTempC(Max31865_t *max31865,float *readTemp);
bool  Max31865_readTempF(Max31865_t *max31865,float *readTemp);
float Max31865_Filter(float	newInput, float	lastOutput, float efectiveFactor);
//#########################################################################################################################





#endif /* SRC_HW_DRIVER_SPI_MAX31865_H_ */
