#ifndef __AD5933_H
#define __AD5933_H

#include "main.h"

// AD5933 Device Address (7-bit)
#define AD5933_ADDRESS	0x0D // 0x1A >> 1
// Note: Reference used 0x1A which is likely the 8-bit write address? 
// Or 0x1A is 0001 1010. 7-bit is 0001 101 (0x0D). 
// Default address is 0x0D. 
// Reference code: #define AD5933_ADDRESS 0x1A. 
// Reference comment says "Default 0001101 (0x0D), shifted left is 0x1A".
// HAL expects 8-bit address (shifted). So we can use 0x1A directly as the address parameter?
// HAL_I2C_Master_Transmit takes "DevAddress".
// "The device 7 bits address value in datasheet must be shifted to the left before calling the interface".
// So if I pass 0x1A (0x0D << 1), it is correct.

#define AD5933_ADDR_8BIT (0x0D << 1) // 0x1A

#define	POINTER_COMMAND	0xB0

void AD5933_Init(void);
void AD5933_Sweep(void);
uint8_t AD5933_ReadReg(uint8_t reg);
void AD5933_WriteReg(uint8_t reg, uint8_t val);

#endif
