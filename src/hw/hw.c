/*
 * hw.c
 *
 *  Created on: Dec 31, 2021
 *      Author: HYJH
 */

#include "hw.h"

void hwInit(void)
{
  bspInit();
  cliInit();
  ledInit();
  cdcInit();
  usbInit();
  uartInit();
  buttonInit();
  i2cInit();
  mcp4725_init();

  gpioInit();
  spiInit();
  delay(500);
  //adcInit();
  //pwmInit();

}


