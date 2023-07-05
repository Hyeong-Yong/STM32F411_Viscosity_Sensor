/*
 * hw.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "hw.h"





void hwInit(void)
{
  bspInit();

  cliInit();
  ledInit();
  usbInit();
  uartInit();
  buttonInit();
  gpioInit();
  mcp4725_init();
  INA219_Init();
}
