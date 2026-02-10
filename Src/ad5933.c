#include "ad5933.h"
#include "main.h"
#include "delay.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h> // For abs

/* Pin Definitions - PB10/PB11 for AD5933 (Swapped) */
#define AD5933_SCL_PIN GPIO_PIN_10
#define AD5933_SCL_PORT GPIOB
#define AD5933_SDA_PIN GPIO_PIN_11
#define AD5933_SDA_PORT GPIOB

/* Macros for GPIO Control */
#define SCL_H() HAL_GPIO_WritePin(AD5933_SCL_PORT, AD5933_SCL_PIN, GPIO_PIN_SET)
#define SCL_L() HAL_GPIO_WritePin(AD5933_SCL_PORT, AD5933_SCL_PIN, GPIO_PIN_RESET)

#define SDA_H() HAL_GPIO_WritePin(AD5933_SDA_PORT, AD5933_SDA_PIN, GPIO_PIN_SET)
#define SDA_L() HAL_GPIO_WritePin(AD5933_SDA_PORT, AD5933_SDA_PIN, GPIO_PIN_RESET)

#define SDA_READ() HAL_GPIO_ReadPin(AD5933_SDA_PORT, AD5933_SDA_PIN)

/* Delay Function for I2C Timing */
static void I2C_Delay(void) {
    delay_us(5); 
}

/**
  * @brief  Initialize Soft I2C for AD5933
  */
void AD5933_Soft_I2C_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIOs */
    GPIO_InitStruct.Pin = AD5933_SCL_PIN | AD5933_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Idle State */
    SCL_H();
    SDA_H();
}

/**
  * @brief  Generate I2C Start Condition
  */
void AD5933_Soft_I2C_Start(void) {
    SDA_H();
    SCL_H();
    I2C_Delay();
    SDA_L();
    I2C_Delay();
    SCL_L(); // Hold SCL Low
    I2C_Delay();
}

/**
  * @brief  Generate I2C Stop Condition
  */
void AD5933_Soft_I2C_Stop(void) {
    SDA_L();
    I2C_Delay();
    SCL_H();
    I2C_Delay();
    SDA_H();
    I2C_Delay();
}

/**
  * @brief  Wait for ACK from Slave
  * @retval 0: ACK received, 1: NACK (No ACK)
  */
uint8_t AD5933_Soft_I2C_WaitAck(void) {
    uint8_t ack = 0;
    uint16_t timeout = 0;
    
    SDA_H(); // Release SDA line
    I2C_Delay();
    SCL_H(); // Clock High
    I2C_Delay();
    
    while (SDA_READ()) {
        timeout++;
        if (timeout > 250) {
            AD5933_Soft_I2C_Stop();
            return 1; // NACK/Timeout
        }
    }
    
    SCL_L(); // Clock Low
    I2C_Delay();
    return 0; // ACK received
}

/**
  * @brief  Send ACK/NACK to Slave
  * @param  ack: 0 for ACK, 1 for NACK
  */
void AD5933_Soft_I2C_SendAck(uint8_t ack) {
    if (ack) {
        SDA_H(); // NACK (High)
    } else {
        SDA_L(); // ACK (Low)
    }
    I2C_Delay();
    SCL_H();
    I2C_Delay();
    SCL_L();
    I2C_Delay();
    SDA_H(); // Release SDA
}

/**
  * @brief  Send one byte to I2C Bus
  * @param  data: Byte to send
  */
void AD5933_Soft_I2C_SendByte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) {
            SDA_H();
        } else {
            SDA_L();
        }
        data <<= 1;
        I2C_Delay();
        SCL_H();
        I2C_Delay();
        SCL_L();
        I2C_Delay();
    }
}

/**
  * @brief  Read one byte from I2C Bus with ACK/NACK
  * @param  ack: 0 to send ACK, 1 to send NACK (usually for last byte)
  * @retval Received byte
  */
uint8_t AD5933_Soft_I2C_ReadByte(uint8_t ack) {
    uint8_t data = 0;
    SDA_H(); // Ensure SDA is released
    
    for (uint8_t i = 0; i < 8; i++) {
        data <<= 1;
        SCL_H();
        I2C_Delay();
        if (SDA_READ()) {
            data |= 0x01;
        }
        SCL_L();
        I2C_Delay();
    }
    
    AD5933_Soft_I2C_SendAck(ack);
    return data;
}

extern void USB_Print(const char* format, ...);

// Constants from reference
#define PI 3.1415926536
#define rad_to_degree (180.0 / PI)

// Global variables for results
double Magnitude, Impedance, Phase;
int Re, Im;

// Calibration Data (Calculated for RFB=200k, Range 1, Vdd=3.3V)
// GF = 1 / (Mag * Rcal) = 1 / (1639.8 * 200000) = 3.04915e-9
double GainFactor[] = {3.04915e-9, 3.04915e-9, 3.04915e-9, 3.04915e-9, 3.04915e-9, 3.04915e-9, 3.04915e-9, 3.04915e-9, 3.04915e-9, 3.04915e-9};
double SystemPhase[] = {108.647, 109.279, 109.847, 110.485, 111.151, 111.763, 112.361, 112.953, 113.608, 114.225};

/**
  * @brief  Write a byte to AD5933 register
  * @param  reg: Register address
  * @param  val: Data to write
  */
void AD5933_WriteReg(uint8_t reg, uint8_t val)
{
    AD5933_Soft_I2C_Start();
    AD5933_Soft_I2C_SendByte(AD5933_ADDR_8BIT); // Write Addr
    if (AD5933_Soft_I2C_WaitAck()) {
        AD5933_Soft_I2C_Stop();
        return; // Error
    }
    
    AD5933_Soft_I2C_SendByte(reg);
    if (AD5933_Soft_I2C_WaitAck()) {
        AD5933_Soft_I2C_Stop();
        return;
    }
    
    AD5933_Soft_I2C_SendByte(val);
    if (AD5933_Soft_I2C_WaitAck()) {
        AD5933_Soft_I2C_Stop();
        return;
    }
    
    AD5933_Soft_I2C_Stop();
}

/**
  * @brief  Read a byte from AD5933 register
  * @param  reg: Register address
  * @retval Data read
  */
uint8_t AD5933_ReadReg(uint8_t reg)
{
    uint8_t val = 0;
    
    // 1. Set Address Pointer
    AD5933_WriteReg(POINTER_COMMAND, reg);
    
    // 2. Read Byte
    AD5933_Soft_I2C_Start();
    AD5933_Soft_I2C_SendByte(AD5933_ADDR_8BIT | 0x01); // Read Addr
    if (AD5933_Soft_I2C_WaitAck()) {
        AD5933_Soft_I2C_Stop();
        return 0; // Error
    }
    
    val = AD5933_Soft_I2C_ReadByte(1); // NACK for last byte
    AD5933_Soft_I2C_Stop();
    
    return val;
}

/**
  * @brief  Initialize AD5933
  */
void AD5933_Init(void)
{
    AD5933_Soft_I2C_Init();
    HAL_Delay(100);

    // 1. Start Frequency (30kHz)
    // 0x82, 0x83, 0x84
    AD5933_WriteReg(0x84, 0x45);
    AD5933_WriteReg(0x83, 0xA6);
    AD5933_WriteReg(0x82, 0x0E);

    // 2. Frequency Increment (1kHz)
    // 0x85, 0x86, 0x87
    AD5933_WriteReg(0x87, 0x02);
    AD5933_WriteReg(0x86, 0x7D);
    AD5933_WriteReg(0x85, 0x00);

    // 3. Number of Increments (10)
    // 0x88, 0x89
    AD5933_WriteReg(0x89, 0x0A);
    AD5933_WriteReg(0x88, 0x00);

    // 4. Settling Time Cycles (15)
    // 0x8A, 0x8B
    AD5933_WriteReg(0x8B, 0x0F);
    AD5933_WriteReg(0x8A, 0x00);

    // 5. Control Register - Standby Mode
    // 0x80 = 0xB0
    AD5933_WriteReg(0x80, 0xB0);

    // 6. Internal System Clock
    // 0x81 = 0x00
    AD5933_WriteReg(0x81, 0x00);

    // 7. Initialize with Start Freq (Range 1: 2Vp-p, PGA=x1)
    // 0x80 = 0x11
    AD5933_WriteReg(0x80, 0x11);

    HAL_Delay(5);

    // 8. Start Frequency Sweep
    // 0x80 = 0x21
    AD5933_WriteReg(0x80, 0x21);
    
    // Verify Communication: Read back Start Frequency Register (0x84)
    // We wrote 0x45 to 0x84 previously.
    uint8_t check_val = AD5933_ReadReg(0x84);
    if(check_val == 0x45) {
        USB_Print("[AD5933] Init Success! Reg[0x84] Readback: 0x%02X (Match)\r\n", check_val);
    } else {
        USB_Print("[AD5933] Init Warning: Reg[0x84] Readback: 0x%02X (Expected 0x45). Check Wiring!\r\n", check_val);
    }

    USB_Print("[AD5933] Init Complete. Sweep Started.\r\n");
}

/**
  * @brief  Perform Frequency Sweep
  */
void AD5933_Sweep(void)
{
    unsigned int real_byte_high;
    unsigned int real_byte_low;
    unsigned int imag_byte_high;
    unsigned int imag_byte_low;

    signed short int imag_data;
    signed short int real_data;

    int m = 0;
    uint8_t status_register;

    USB_Print("[AD5933] Starting Sweep Loop...\r\n");
    HAL_Delay(10);

    // Infinite loop in reference, but we should probably limit it or have an exit condition
    // For now, implementing the loop as per reference but breaking when sweep is done.
    
    while(1)
    {
        // Poll Status Register (0x8F)
        // Bit 1: Valid Real/Imag Data
        // Bit 2: Frequency Sweep Complete
        
        status_register = AD5933_ReadReg(0x8F);
        
        // Check for Valid Data (Bit 1)
        if ((status_register & 0x02) == 0x02)
        {
            // Check if Sweep Complete (Bit 2) is NOT set (Wait, reference logic is a bit weird)
            // Reference: if( (AD5933_read(0x8F)| 0xFB )!=  0xFF) -> checks if bit 2 is 0?
            // 0xFB is 1111 1011. ORing with 0xFB makes bit 2 the only variable.
            // If result != 0xFF, it means bit 2 was 0. (Sweep NOT complete).
            // So if sweep is NOT complete, read data.
            
            // Simpler check:
            // if (!(status_register & 0x04)) 
            
            if ((status_register & 0x04) == 0) // Sweep Not Complete
            {
                // Read Real Data
                real_byte_high = AD5933_ReadReg(0x94);
                real_byte_low  = AD5933_ReadReg(0x95);
                // Read Imag Data
                imag_byte_high = AD5933_ReadReg(0x96);
                imag_byte_low  = AD5933_ReadReg(0x97);

                real_data = (signed short int)((real_byte_high << 8) | real_byte_low);
                imag_data = (signed short int)((imag_byte_high << 8) | imag_byte_low);

                Re = (int)real_data;
                Im = (int)imag_data;
                
                Magnitude = sqrt(pow(Re, 2) + pow(Im, 2));
                
                // Avoid division by zero or index out of bounds
                if (m < 10) {
                    Impedance = 1.0 / (Magnitude * GainFactor[m]);
                    
                    // Calculate Phase
                    if(Re>0 && Im>0)
                        Phase = (atan((double)Im / (double)Re) * rad_to_degree) - SystemPhase[m];
                    else if(Re>0 && Im<0)
                        Phase = (360 + atan((double)Im / (double)Re) * rad_to_degree) - SystemPhase[m];
                    else
                        Phase = (180 + atan((double)Im / (double)Re) * rad_to_degree) - SystemPhase[m];
                        
                    USB_Print("[AD5933] Pt %d: Z=%.2f Ohm, P=%.2f Deg (Re=%d, Im=%d)\r\n", 
                              m, Impedance, Phase, Re, Im);
                } else {
                    USB_Print("[AD5933] Pt %d: Z=%.2f (Re=%d, Im=%d) [No Cal Data]\r\n", 
                              m, 1.0/Magnitude, Re, Im);
                }
                
                HAL_Delay(10);
                
                // Increment Frequency
                // 0x80 = 0x31 (Increment Freq)
                // Wait, reference does delay_ms(1800) then increment.
                // We'll reduce delay for testing speed.
                HAL_Delay(100); 
                
                m++;
                AD5933_WriteReg(0x80, 0x31);
            }
            else
            {
                // Sweep Complete
                USB_Print("[AD5933] Sweep Complete.\r\n");
                
                // Power down or reset?
                // Reference just breaks.
                break;
            }
        }
        
        HAL_Delay(1);
    }
}
