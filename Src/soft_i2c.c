#include "soft_i2c.h"
#include "delay.h"

/* Pin Definitions - Updated to PB6/PB7 as requested (Swapped) */
#define I2C_SCL_PIN GPIO_PIN_6
#define I2C_SCL_PORT GPIOB
#define I2C_SDA_PIN GPIO_PIN_7
#define I2C_SDA_PORT GPIOB

/* Macros for GPIO Control */
/* Note: GPIOs should be configured as Output Open Drain for I2C */
#define I2C_SCL_H() HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET)
#define I2C_SCL_L() HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET)

#define I2C_SDA_H() HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET)
#define I2C_SDA_L() HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET)

#define I2C_SDA_READ() HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN)

/* Delay Function for I2C Timing */
/* Adjust delay count based on system clock to achieve desired I2C speed (e.g., 100kHz) */
static void I2C_Delay(void) {
    delay_us(10); // 4us delay for ~125kHz (Standard Mode 100kHz is safe target)
}

/**
  * @brief  Initialize Soft I2C (Pins are initialized here)
  *         This function ensures bus is in idle state.
  */
void Soft_I2C_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIOs for QMI8658 (PB6/PB7) */
    GPIO_InitStruct.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    I2C_SCL_H();
    I2C_SDA_H();
}

/**
  * @brief  Generate I2C Start Condition
  */
void Soft_I2C_Start(void) {
    I2C_SDA_H();
    I2C_SCL_H();
    I2C_Delay();
    I2C_SDA_L();
    I2C_Delay();
    I2C_SCL_L(); // Hold SCL Low
	  I2C_Delay();
}

/**
  * @brief  Generate I2C Stop Condition
  */
void Soft_I2C_Stop(void) {
    I2C_SDA_L();
//    I2C_SCL_L();
    I2C_Delay();
    I2C_SCL_H();
    I2C_Delay();
    I2C_SDA_H();
    I2C_Delay();
}

/**
  * @brief  Wait for ACK from Slave
  * @retval 0: ACK received, 1: NACK (No ACK)
  */
uint8_t Soft_I2C_WaitAck(void) {
    uint8_t ack = 0;
    uint16_t timeout = 0;
    
    I2C_SDA_H(); // Release SDA line (Pull-up pulls it high)
    I2C_Delay();
    I2C_SCL_H(); // Clock High
    I2C_Delay();
    
    while (I2C_SDA_READ()) {
        timeout++;
        if (timeout > 250) {
            Soft_I2C_Stop();
            return 1; // NACK/Timeout
        }
    }
    
    I2C_SCL_L(); // Clock Low
		I2C_Delay();
    return 0; // ACK received
}

/**
  * @brief  Send ACK/NACK to Slave
  * @param  ack: 0 for ACK, 1 for NACK
  */
void Soft_I2C_SendAck(uint8_t ack) {
    if (ack) {
        I2C_SDA_H(); // NACK (High)
    } else {
        I2C_SDA_L(); // ACK (Low)
    }
    I2C_Delay();
    I2C_SCL_H();
    I2C_Delay();
    I2C_SCL_L();
    I2C_SDA_H(); // Release SDA
}

/**
  * @brief  Send one byte to I2C Bus
  * @param  data: Byte to send
  */
void Soft_I2C_SendByte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) {
            I2C_SDA_H();
        } else {
            I2C_SDA_L();
        }
        data <<= 1;
        I2C_Delay();
        I2C_SCL_H();
        I2C_Delay();
        I2C_SCL_L();
        I2C_Delay();
    }
}

/**
  * @brief  Read one byte from I2C Bus (No ACK sent)
  * @retval Received byte
  */
uint8_t Soft_I2C_ReceiveByte(void) {
    uint8_t data = 0;
    I2C_SDA_H(); // Ensure SDA is released (Input mode effectively via Open Drain)
    
    for (uint8_t i = 0; i < 8; i++) {
        data <<= 1;
        I2C_SCL_H();
        I2C_Delay();
        if (I2C_SDA_READ()) {
            data |= 0x01;
        }
        I2C_SCL_L();
        I2C_Delay();
    }
    return data;
}

/**
  * @brief  Read one byte from I2C Bus with ACK/NACK
  * @param  ack: 0 to send ACK, 1 to send NACK (usually for last byte)
  * @retval Received byte
  */
uint8_t Soft_I2C_ReadByte(uint8_t ack) {
    uint8_t data = Soft_I2C_ReceiveByte();
    Soft_I2C_SendAck(ack);
    return data;
}
