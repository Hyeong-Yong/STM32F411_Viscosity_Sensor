/*
 * hw.h
 *
 *  Created on: Dec 31, 2021
 *      Author: HYJH
 */

#ifndef SRC_HW_HW_H_
#define SRC_HW_HW_H_

#include "hw_def.h"


#include "led.h"
#include "uart.h"
#include "usb.h"
#include "reset.h"
#include "rtc.h"
#include "cdc.h"
#include "flash.h"
#include "cli.h"
#include "button.h"
#include "gpio.h"
#include "fatfs.h"
#include "spi.h"
//#include "adc.h"
#include "i2c.h"
#include "mcp4725.h"
//#include "pwm.h"
#include "ina219.h"

#include "max31865.h"


void hwInit(void);


#endif /* SRC_HW_HW_H_ */
