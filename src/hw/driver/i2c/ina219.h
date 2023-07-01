/*
 * ina219.h
 *
 *  Created on: 2023. 6. 28.
 *      Author: hwang
 */

#ifndef SRC_HW_DRIVER_I2C_INA219_H_
#define SRC_HW_DRIVER_I2C_INA219_H_

#include "hw.h"
#include "cli.h"

/*!
@brief  default I2C address
*/

typedef enum
{
  I2C_ADDRESS1  = 0x40,    // 1000000 (A0=GND, A1=GND)
  I2C_ADDRESS2  = 0x41,    // 1000001 (A0=VCC, A1=GND)
  I2C_ADDRESS3  = 0x44,    // 1000100 (A0=GND, A1=VCC)
  I2C_ADDRESS4  = 0x45,    // 1000101 (A0=VCC, A1=VCC)
} INA219_ADDRESS;


const uint8_t READ = 0x01;

//REGISTER
#define REG_CONFIG         0x00 // Config register address
#define REG_SHUNTVOLTAGE   0x01 //shunt voltage register
#define REG_BUSVOLTAGE     0x02 //bus voltage register
#define REG_POWER          0x03 // power register
#define REG_CURRENT        0x04 //current register
#define REG_CALIBRATION    0x05 //calibration register
//
#define CONFIG_RESET       0x8000  // Reset Bit
//
#define	CONFIG_BVOLTAGERANGE_16V 0x0000  // 0-16V Range
#define	CONFIG_BVOLTAGERANGE_32V 0x2000  // 0-32V Range
//
#define	INA219_CONFIG_GAIN_1_40MV				(0x0000)  // Gain 1, 40mV Range
#define	INA219_CONFIG_GAIN_2_80MV				(0x0800)  // Gain 2, 80mV Range
#define	NA219_CONFIG_GAIN_4_160MV				(0x1000) // Gain 4, 160mV Range
#define	INA219_CONFIG_GAIN_8_320MV				(0x1800) // Gain 8, 320mV Range

#define	INA219_CONFIG_BADCRES_9BIT				(0x0000)  // 9-bit bus res = 0..511
#define	INA219_CONFIG_BADCRES_10BIT				(0x0080) // 10-bit bus res = 0..1023
#define	INA219_CONFIG_BADCRES_11BIT				(0x0100) // 11-bit bus res = 0..2047
#define	INA219_CONFIG_BADCRES_12BIT				(0x0180) // 12-bit bus res = 0..4097
#define	INA219_CONFIG_BADCRES_12BIT_2S_1060US 	(0x0480) // 2 x 12-bit bus samples averaged together
#define	INA219_CONFIG_BADCRES_12BIT_4S_2130US	(0x0500) // 4 x 12-bit bus samples averaged together
#define	INA219_CONFIG_BADCRES_12BIT_8S_4260US	(0x0580) // 8 x 12-bit bus samples averaged together
#define	INA219_CONFIG_BADCRES_12BIT_16S_8510US	(0x0600) // 16 x 12-bit bus samples averaged together
#define	INA219_CONFIG_BADCRES_12BIT_32S_17MS	(0x0680) // 32 x 12-bit bus samples averaged together
#define	INA219_CONFIG_BADCRES_12BIT_64S_34MS	(0x0700) // 64 x 12-bit bus samples averaged together
#define	INA219_CONFIG_BADCRES_12BIT_128S_69MS	(0x0780) // 128 x 12-bit bus samples averaged together

#define	INA219_CONFIG_SADCRES_9BIT_1S_84US		(0x0000) // 1 x 9-bit shunt sample
#define	INA219_CONFIG_SADCRES_10BIT_1S_148US	(0x0008) // 1 x 10-bit shunt sample
#define	INA219_CONFIG_SADCRES_11BIT_1S_276US	(0x0010) // 1 x 11-bit shunt sample
#define	INA219_CONFIG_SADCRES_12BIT_1S_532US	(0x0018) // 1 x 12-bit shunt sample
#define	INA219_CONFIG_SADCRES_12BIT_2S_1060US	(0x0048) // 2 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_4S_2130US	(0x0050) // 4 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_8S_4260US	(0x0058) // 8 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_16S_8510US	(0x0060) // 16 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_32S_17MS	(0x0068) // 32 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_64S_34MS	(0x0070) // 64 x 12-bit shunt samples averaged together
#define	INA219_CONFIG_SADCRES_12BIT_128S_69MS	(0x0078) // 128 x 12-bit shunt samples averaged together
//
#define INA219_CONFIG_MODE_MASK					(0x07)
#define	INA219_CONFIG_MODE_POWERDOWN			0x00 /**< power down */
#define	INA219_CONFIG_MODE_SVOLT_TRIGGERED		0x01 /**< shunt voltage triggered */
#define	INA219_CONFIG_MODE_BVOLT_TRIGGERED		0x02 /**< bus voltage triggered */
#define	INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED	0x03 /**< shunt and bus voltage triggered */
#define	INA219_CONFIG_MODE_ADCOFF				0x04 /**< ADC off */
#define	INA219_CONFIG_MODE_SVOLT_CONTINUOUS		0x05 /**< shunt voltage continuous */
#define	INA219_CONFIG_MODE_BVOLT_CONTINUOUS		0x06 /**< bus voltage continuous */
#define	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS 0x07


/**************************************************************************
	@brief  functions for interacting with INA219 current/power monitor IC
**************************************************************************/

typedef struct INA219
{
	I2C_HandleTypeDef *hi2c;
	INA219_ADDRESS	  i2cAddress;
} INA219_t;

uint16_t ina219_calibrationValue;
int16_t ina219_currentDivider_mA;
int16_t ina219_powerMultiplier_mW;

void ina219_init();
uint16_t INA219_ReadBusVoltage();
int16_t INA219_ReadCurrent();
int16_t INA219_ReadCurrent_raw();
uint16_t INA219_ReadShuntVolage();

void INA219_Reset();
void INA219_setCalibration(uint16_t CalibrationData);
uint16_t INA219_getConfig();
void INA219_setConfig(uint16_t Config);
void INA219_setCalibration_32V_2A();
void INA219_setCalibration_32V_1A();
void INA219_setCalibration_16V_400mA();
void INA219_setPowerMode( uint8_t Mode);

uint16_t Read16(uint8_t Register);
void Write16(uint8_t Register, uint16_t Value);



#endif /* SRC_HW_DRIVER_I2C_INA219_H_ */
