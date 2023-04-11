/*
 * SHT31.h
 *
 *  Created on: Mar 29, 2023
 *      Author: Nguyen Trung Thao
 */

#ifndef INC_SHT31_H_
#define INC_SHT31_H_

#define STM32F1

#ifdef STM32F1
#include "stm32f1xx_hal.h"
#elif STM32F4
#include "stm32f4xx_hal.h"
#endif

#define SHT31_LIB_VERSION				V0.1

// ADDR (pin 2) connected to VSS
#define SHT31_ADDRESS_VSS (0x44)
//ADDR (pin 2) connected to VDD
#define SHT31_ADDRESS_VDD (0x45)

// fields readStatus
#define SHT31_STATUS_ALERT_PENDING    (1 << 15)
#define SHT31_STATUS_HEATER_ON        (1 << 13)
#define SHT31_STATUS_HUM_TRACK_ALERT  (1 << 11)
#define SHT31_STATUS_TEMP_TRACK_ALERT (1 << 10)
#define SHT31_STATUS_SYSTEM_RESET     (1 << 4)
#define SHT31_STATUS_COMMAND_STATUS   (1 << 1)
#define SHT31_STATUS_WRITE_CRC_STATUS (1 << 0)

// error codes
#define SHT31_OK                      0x00
#define SHT31_ERR_WRITECMD            0x81
#define SHT31_ERR_READBYTES           0x82
#define SHT31_ERR_HEATER_OFF          0x83
#define SHT31_ERR_NOT_CONNECT         0x84
#define SHT31_ERR_CRC_TEMP            0x85
#define SHT31_ERR_CRC_HUM             0x86
#define SHT31_ERR_CRC_STATUS          0x87
#define SHT31_ERR_HEATER_COOLDOWN     0x88
#define SHT31_ERR_HEATER_ON           0x89


// SUPPORTED COMMANDS - single shot mode only
#define SHT31_READ_STATUS       0xF32D
#define SHT31_CLEAR_STATUS      0x3041
// RST sensor
#define SHT31_SOFT_RESET        0x30A2
#define SHT31_HARD_RESET        0x0006

//measurement without Clock stretching page 10 datasheet
#define SHT31_MEASUREMENT_FAST  0x2416    // page 10 datasheet
#define SHT31_MEASUREMENT_SLOW  0x2400    // no clock stretching

#define SHT31_HEAT_ON           0x306D
#define SHT31_HEAT_OFF          0x3066
#define SHT31_HEATER_TIMEOUT    180000UL  // milliseconds
// time to sensor compete measurement
#define SHT31_TIME_GET_DATA 	15 // milliseconds

typedef struct {
	//i2c handle
	I2C_HandleTypeDef *I2cHandle;
	//raw temperature value
	uint8_t tempratureRAW[3];
	//raw temperature value
	uint8_t humihityRAW[3];


}SHT31_type;
//check sensor
HAL_StatusTypeDef SHT31_init(SHT31_type *sht31, I2C_HandleTypeDef *I2cHandle);
HAL_StatusTypeDef SHT31_IsConnected(SHT31_type *sht31);
HAL_StatusTypeDef SHT31_ReadStatus(uint8_t *statusValue);
HAL_StatusTypeDef SHT31_Crc(uint8_t *data, uint8_t len, uint8_t crcCheck);
HAL_StatusTypeDef SHT31_ReadTemperature(SHT31_type *sht31, double *temperatureValue);
HAL_StatusTypeDef SHT31_ReadHumihity(SHT31_type *sht31, float *humihityValue);
HAL_StatusTypeDef SHT31_WriteCMD(SHT31_type *sht31, uint16_t CMD);
HAL_StatusTypeDef SHT31_ConverteData(SHT31_type *sht31);

#endif /* INC_SHT31_H_ */
