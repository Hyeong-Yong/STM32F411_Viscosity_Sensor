/*
 * hw_def.h
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#ifndef SRC_HW_HW_DEF_H_
#define SRC_HW_HW_DEF_H_


#include "def.h"
#include "bsp.h"




#define _USE_HW_USB
#define _USE_HW_CDC


#define _USE_HW_LED
#define      HW_LED_MAX_CH          1

#define _USE_HW_UART
#define      HW_UART_MAX_CH         2

#define _USE_HW_CLI
#define      HW_CLI_CMD_LIST_MAX    16
#define      HW_CLI_CMD_NAME_MAX    16
#define      HW_CLI_LINE_HIS_MAX    4
#define      HW_CLI_LINE_BUF_MAX    32

#define _USE_HW_BUTTON
#define      HW_BUTTON_MAX_CH       1

#define _USE_HW_GPIO
#define      HW_GPIO_MAX_CH         2

#define _USE_HW_I2C
#define      HW_I2C_MAX_CH	2
#define _USE_HW_MCP4725
#define _USE_HW_INA219

#define _USE_HW_SPI

#define _PIN_GPIO_SD_CD             0

#define _USE_HW_SPI
#define      HW_SPI_MAX_CH			2
#define _USE_HW_MAX31865

#endif /* SRC_HW_HW_DEF_H_ */
