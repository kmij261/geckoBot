#ifndef _PERIPHERAL_I2C_H
#define _PERIPHERAL_I2C_H

#include "sys.h"

#define I2C_TIME_OUT		((uint16_t) 500)	//I2C通讯超时阈值	

typedef enum I2C_Err_Type{
	I2C_ERR_NoError = 0,
	I2C_ERR_Busy,
	I2C_ERR_StartTimeOut,
	I2C_ERR_SendSlaveAddrTimeOut,
	I2C_ERR_SendRegAddrTimeOut,
	I2C_ERR_SendDataTimeOut,
	I2C_ERR_ReadTimeOut,
	
	
}I2cErrType_t;


extern I2cErrType_t i2cError;


void IIC_Init(void);

uint8_t I2C_ByteWrite(I2C_TypeDef* I2Cx, uint8_t ucSlaveAddr, uint8_t ucRegAddr,\
					uint8_t ucData, I2cErrType_t* err);
uint8_t I2C_ByteRead(I2C_TypeDef* I2Cx, uint8_t ucSlaveAddr,\
							uint8_t ucRegAddr, I2cErrType_t* err);
uint8_t I2C_MultiWrite(I2C_TypeDef * I2Cx, uint8_t ucSlaveAddr, uint8_t ucRegAddr,\
						uint8_t ucNumToWrite, uint8_t* pucBuffer, I2cErrType_t* err);
uint8_t I2C_MultiRead(I2C_TypeDef* I2Cx, uint8_t ucSlaveAddr, uint8_t ucRegAddr,\
					uint8_t ucNumToRead, uint8_t* pucBuffer, I2cErrType_t* err);
#endif
