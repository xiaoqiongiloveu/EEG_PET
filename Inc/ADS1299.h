#ifndef __ADS1299_H
#define __ADS1299_H

#include "main.h"

// Command Definitions
#define ADS1299_WAKEUP  0x02
#define ADS1299_STANDBY 0x04
#define ADS1299_RESET   0x06
#define ADS1299_START   0x08
#define ADS1299_STOP    0x0A
#define ADS1299_RDATAC  0x10
#define ADS1299_SDATAC  0x11
#define ADS1299_RDATA   0x12

// Register Definitions (mapped to user provided code values)
// 0x00 ID
#define ADS1299_CONFIG1    0x01
#define ADS1299_CONFIG2    0x02
#define ADS1299_CONFIG3    0x03
#define ADS1299_LOFF       0x04
#define ADS1299_CH1SET     0x05
#define ADS1299_CH2SET     0x06
#define ADS1299_CH3SET     0x07
#define ADS1299_CH4SET     0x08
#define ADS1299_CH5SET     0x09
#define ADS1299_CH6SET     0x0A
#define ADS1299_CH7SET     0x0B
#define ADS1299_CH8SET     0x0C
#define ADS1299_BIAS_SENSP 0x0D
#define ADS1299_BIAS_SENSN 0x0E
#define ADS1299_LOFF_SENSP 0x0F
#define ADS1299_LOFF_SENSN 0x10
#define ADS1299_LOFF_FLIP  0x11
#define ADS1299_LOFF_STATP 0x12
#define ADS1299_LOFF_STATN 0x13
#define ADS1299_GPIO       0x14
#define ADS1299_MISC1      0x15
#define ADS1299_MISC2      0x16
#define ADS1299_CONFIG4    0x17

// Macros for compatibility with commands_impl.c if needed, or mapping
#define ADS1299_CMD_START ADS1299_START
#define ADS1299_CMD_STOP  ADS1299_STOP
#define ADS1299_CMD_RDATAC ADS1299_RDATAC
#define ADS1299_CMD_SDATAC ADS1299_SDATAC

// Function Prototypes
void ADS1299_Init(void);
void ADS1299_Rest(void);
void ADS1299_Send_Cmd(uint8_t cmd);
void ADS1299_Write_Reg(uint8_t addr, uint8_t value);
void ADS1299_Write_Regs(uint8_t addr, uint8_t *regs, uint8_t len);
void ADS1299_Read_Regs(uint8_t addr, uint8_t *regs, uint8_t len);
void ADS1299_SPI_Rate_Set(uint32_t baudRatePrescaler);

// Compatibility wrappers for commands_impl.c
void ADS1299_Start(void);
void ADS1299_Stop(void);
void ADS1299_Set_Sample_Parameter(uint8_t dr, uint8_t gain, uint8_t input_type);
void ADS1299_WriteRegister(uint8_t reg, uint8_t value); // Alias to Write_Reg
uint8_t ADS1299_ReadRegister(uint8_t reg); // Wrapper around Read_Regs
void ADS1299_DRDY_Handler(void);

#endif
