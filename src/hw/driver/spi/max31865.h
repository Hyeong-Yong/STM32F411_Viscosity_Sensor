/*
 * max31865.h
 *
 *  Created on: 2023. 6. 29.
 *      Author: hwang
 */

#ifndef SRC_HW_DRIVER_SPI_MAX31865_H_
#define SRC_HW_DRIVER_SPI_MAX31865_H_


#include "hw_def.h"
#include "spi.h"
#include <math.h>

#ifdef _USE_HW_MAX31865

#define _MAX31865_RREF      430.0f
#define _MAX31865_RNOMINAL  100.0f

//#########################################################################################################################


//#########################################################################################################################
void  max31865_init();
bool  max31865_readTempC(float *readTemp);
bool  max31865_readTempF(float *readTemp);
float max31865_Filter(float	newInput, float	lastOutput, float efectiveFactor);
//#########################################################################################################################

#endif


#endif /* SRC_HW_DRIVER_SPI_MAX31865_H_ */
