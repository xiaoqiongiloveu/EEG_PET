#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include "main.h"

/* Function Prototypes */
void Soft_I2C_Init(void);
void Soft_I2C_Start(void);
void Soft_I2C_Stop(void);
void Soft_I2C_SendAck(uint8_t ack);
uint8_t Soft_I2C_WaitAck(void);
void Soft_I2C_SendByte(uint8_t data);
uint8_t Soft_I2C_ReadByte(uint8_t ack);
uint8_t Soft_I2C_IsDeviceReady(uint8_t addr);

#endif
