/*
 * SHT31.c
 *
 *  Created on: Mar 29, 2023
 *      Author: nguye
 */

#include "SHT31.h"

HAL_StatusTypeDef SHT31_IsConnected(SHT31_type *sht31){
	return HAL_I2C_IsDeviceReady(sht31->I2cHandle, SHT31_ADDRESS_VSS << 1, 3, HAL_MAX_DELAY);
}

HAL_StatusTypeDef SHT31_init(SHT31_type *sht31, I2C_HandleTypeDef *I2cHandle){
	sht31->I2cHandle = I2cHandle;
	return SHT31_IsConnected(sht31);
}

//HAL_StatusTypeDef SHT31_ReadStatus(uint8_t *statusValue);

HAL_StatusTypeDef SHT31_Crc(uint8_t *data, uint8_t len, uint8_t crcCheck){

	  const uint8_t POLY = 0x31;
	  uint8_t crc = 0xFF;

	  for (uint8_t j = len; j; --j)
	  {
	    crc ^= *data++;

	    for (uint8_t i = 8; i; --i)
	    {
	      crc = (crc & 0x80) ? (crc << 1) ^ POLY : (crc << 1);
	    }
	  }
	  if(crc == crcCheck){
		  return HAL_OK;
	  }
	  else{
		  return HAL_ERROR;
	  }
}

//HAL_StatusTypeDef SHT31_ReadTemperature(SHT31_type *sht31, float *temperatureValue, uint8_t *flagLed){
//	uint16_t data = 0u;
//	if(HAL_I2C_Master_Receive_IT(sht31->I2cHandle, SHT31_ADDRESS_VSS << 1, sht31->I2cHandle, 3) == HAL_OK){
//
//	}
//	return HAL_ERROR;
//}
//HAL_StatusTypeDef SHT31_ReadHumihity(SHT31_type *sht31, float *humihityValue);
//HAL_StatusTypeDef SHT31_WriteCMD(SHT31_type *sht31, uint16_t CMD);
//void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
//	uint16_t data;
////	if(SHT31_Crc(sht31->tempratureRAW, 2, sht31->tempratureRAW[2]) == HAL_OK){
////		data = (sht31->tempratureRAW[0] << 8) |  sht31->tempratureRAW[1];
////		*temperatureValue = data * (175.0f / 65535) - 45;
////	}
//	return HAL_OK;
//}

HAL_StatusTypeDef SHT31_ReadTemperature(SHT31_type *sht31, double *temperatureValue){
	uint16_t data = 0u;
	if(HAL_I2C_Master_Receive(sht31->I2cHandle, SHT31_ADDRESS_VSS << 1, sht31->tempratureRAW, 3, HAL_MAX_DELAY) == HAL_OK){
//	if(HAL_I2C_Master_Receive_IT(sht31->I2cHandle, SHT31_ADDRESS_VSS << 1, sht31->I2cHandle, 3) == HAL_OK){
		if(SHT31_Crc(sht31->tempratureRAW, 2, sht31->tempratureRAW[2]) == HAL_OK){
			data = (sht31->tempratureRAW[0] << 8) |  sht31->tempratureRAW[1];
			*temperatureValue = data * (175.0f / 65535) - 45;
		}
		return HAL_OK;
	}
	return HAL_ERROR;
}
HAL_StatusTypeDef SHT31_ReadHumihity(SHT31_type *sht31, float *humihityValue){
	uint16_t data = 0u;
	if(HAL_I2C_Master_Receive(sht31->I2cHandle, SHT31_ADDRESS_VSS << 1, sht31->humihityRAW, 2, HAL_MAX_DELAY) == HAL_OK){
		data = (sht31->humihityRAW[1] << 8) |  sht31->humihityRAW[0];
		*humihityValue = data * (100.0 / 65535);
			return HAL_OK;
		}
		return HAL_ERROR;
}
HAL_StatusTypeDef SHT31_WriteCMD(SHT31_type *sht31, uint16_t CMD){
	uint8_t cc[2] = {0};
	cc[0] = (uint8_t)(CMD >> 8);
	cc[1] = (uint8_t)CMD;
	return HAL_I2C_Master_Transmit(sht31->I2cHandle, SHT31_ADDRESS_VSS << 1, cc, 2, HAL_MAX_DELAY);
}



