/* Audio hardware removed; implementation retained for reference. */
#if 0
#include "wm8978.h"
#include "main.h"

/* External I2C Handle from main.c */
extern I2C_HandleTypeDef hi2c1;

/* WM8978 Register Cache (0~57), total 58 registers */
/* WM8978 write-only, so we shadow registers locally */
static uint16_t WM8978_REGVAL_TBL[58] = {
	0X0000, 0X0000, 0X0000, 0X0000, 0X0050, 0X0000, 0X0140, 0X0000,
	0X0000, 0X0000, 0X0000, 0X00FF, 0X00FF, 0X0000, 0X0100, 0X00FF,
	0X00FF, 0X0000, 0X012C, 0X002C, 0X002C, 0X002C, 0X002C, 0X0000,
	0X0032, 0X0000, 0X0000, 0X0000, 0X0000, 0X0000, 0X0000, 0X0000,
	0X0038, 0X000B, 0X0032, 0X0000, 0X0008, 0X000C, 0X0093, 0X00E9,
	0X0000, 0X0000, 0X0000, 0X0000, 0X0003, 0X0010, 0X0010, 0X0100,
	0X0100, 0X0002, 0X0001, 0X0001, 0X0039, 0X0039, 0X0039, 0X0039,
	0X0001, 0X0001
};

/**
  * @brief  Write WM8978 register
  * @param  reg: Register address (7-bit)
  * @param  val: Register value (9-bit)
  * @retval 0: Success, non-zero: Error
  */
uint8_t WM8978_Write_Reg(uint8_t reg, uint16_t val)
{
    uint8_t data[2];
    
    // WM8978 register write format:
    // Byte 1: {RegAddr[6:0], Val[8]}
    // Byte 2: {Val[7:0]}
    
    data[0] = (reg << 1) | ((val >> 8) & 0x01);
    data[1] = val & 0xFF;

    if (HAL_I2C_Master_Transmit(&hi2c1, (WM8978_ADDR << 1), data, 2, 100) != HAL_OK)
    {
        return 1; // Error
    }

    WM8978_REGVAL_TBL[reg] = val; // Update cache
    return 0;
}

/**
  * @brief  Read WM8978 register (from local cache)
  * @param  reg: Register address
  * @retval Register value
  */
uint16_t WM8978_Read_Reg(uint8_t reg)
{
    return WM8978_REGVAL_TBL[reg];
}

/**
  * @brief  Initialize WM8978
  * @retval 0: Success, 1: Error
  */
uint8_t WM8978_Init(void)
{
    uint8_t res;

    // 1. Reset WM8978
    res = WM8978_Write_Reg(0, 0);
    if (res) return 1; // Error

    // 2. Default Configuration
    // R1: MICEN=1 (Mic Enable), BIASEN=1 (Analog Bias), VMIDSEL=11 (5K Ohm)
    WM8978_Write_Reg(1, 0X1B);
    
    // R2: ROUT1EN=1, LOUT1EN=1, BOOSTENR=1, BOOSTENL=1
    WM8978_Write_Reg(2, 0X1B0);
    
    // R3: LOUT2EN=1, ROUT2EN=1, RMIXEN=1, LMIXEN=1 (Output to Headphone and Speaker)
    WM8978_Write_Reg(3, 0X6C);
    
    // R6: MCLK Source (Keep default, typically MCLK input)
    WM8978_Write_Reg(6, 0);
    
    // R43: INVROUT2=0 (Disable Speaker Inversion)
    WM8978_Write_Reg(43, 0);
    
    // R47: PGABOOSTL=1 (+20dB Mic Boost)
    WM8978_Write_Reg(47, 1 << 8);
    
    // R48: PGABOOSTR=1 (+20dB Mic Boost)
    WM8978_Write_Reg(48, 1 << 8);
    
    // R49: TSDEN=1 (Thermal Shutdown Enable)
    WM8978_Write_Reg(49, 1 << 1);
    
    // R10: SOFTMUTE=0, DACOSR=128x (Best SNR)
    WM8978_Write_Reg(10, 1 << 3);
    
    // R14: ADCOSR=128x
    WM8978_Write_Reg(14, 1 << 3);

    return 0;
}

/**
  * @brief  Configure ADC/DAC Enable
  * @param  dacen: 1=Enable DAC, 0=Disable
  * @param  adcen: 1=Enable ADC, 0=Disable
  */

void WM8978_ADDA_Cfg(uint8_t dacen, uint8_t adcen)
{
    uint16_t regval;
    
    regval = WM8978_Read_Reg(3);
    if (dacen) regval |= 3 << 0; // Enable DACR & DACL
    else       regval &= ~(3 << 0);
    WM8978_Write_Reg(3, regval);
    
    regval = WM8978_Read_Reg(2);
    if (adcen) regval |= 3 << 0; // Enable ADCR & ADCL
    else       regval &= ~(3 << 0);
    WM8978_Write_Reg(2, regval);
}

/**
  * @brief  Configure Input Source
  * @param  micen: 1=Enable Mic, 0=Disable
  * @param  lineinen: 1=Enable LineIn, 0=Disable
  * @param  auxen: 1=Enable AUX, 0=Disable
  */
void WM8978_Input_Cfg(uint8_t micen, uint8_t lineinen, uint8_t auxen)
{
    uint16_t regval;
    
    // R2: INPPGAENR/L
    regval = WM8978_Read_Reg(2);
    if (micen) regval |= 3 << 2;
    else       regval &= ~(3 << 2);
    WM8978_Write_Reg(2, regval);
    
    // R44: Input to PGA mapping
    regval = WM8978_Read_Reg(44);
    if (micen) regval |= (3 << 4) | (3 << 0); // LIN2INPPGA, etc.
    else       regval &= ~((3 << 4) | (3 << 0));
    WM8978_Write_Reg(44, regval);
    
    if (lineinen) WM8978_LINEIN_Gain(5); // 0dB
    else          WM8978_LINEIN_Gain(7); // -15dB (Min Gain) to suppress noise
    
    if (auxen)    WM8978_AUX_Gain(0);    // +6dB (Max Gain)
    else          WM8978_AUX_Gain(7);    // -15dB (Min Gain) to suppress noise
}

/**
  * @brief  Configure Output
  * @param  dacen: 1=Enable DAC Output
  * @param  bpsen: 1=Enable Bypass (Analog Loopback)
  */
void WM8978_Output_Cfg(uint8_t dacen, uint8_t bpsen)
{
    uint16_t regval = 0;
    if (dacen) regval |= 1 << 0;
    if (bpsen) {
        regval |= 1 << 1; // Bypass enable
        regval |= 5 << 2; // 0dB
    }
    WM8978_Write_Reg(50, regval); // L
    WM8978_Write_Reg(51, regval); // R
}

void WM8978_MIC_Gain(uint8_t gain)
{
    gain &= 0X3F;
    WM8978_Write_Reg(45, gain);         // R45
    WM8978_Write_Reg(46, gain | 1 << 8); // R46 + Update
}

void WM8978_LINEIN_Gain(uint8_t gain)
{
    uint16_t regval;
    gain &= 0X07;
    
    regval = WM8978_Read_Reg(47);
    regval &= ~(7 << 4);
    WM8978_Write_Reg(47, regval | (gain << 4));
    
    regval = WM8978_Read_Reg(48);
    regval &= ~(7 << 4);
    WM8978_Write_Reg(48, regval | (gain << 4));
}

void WM8978_AUX_Gain(uint8_t gain)
{
    uint16_t regval;
    gain &= 0X07;
    
    regval = WM8978_Read_Reg(47);
    regval &= ~(7 << 0);
    WM8978_Write_Reg(47, regval | (gain << 0));
    
    regval = WM8978_Read_Reg(48);
    regval &= ~(7 << 0);
    WM8978_Write_Reg(48, regval | (gain << 0));
}

void WM8978_I2S_Cfg(uint8_t fmt, uint8_t len)
{
    fmt &= 0X03;
    len &= 0X03;
    // R4: Audio Interface Control
    // FMT[4:3], WL[6:5]
    WM8978_Write_Reg(4, (fmt << 3) | (len << 5));
}

void WM8978_HPvol_Set(uint8_t voll, uint8_t volr)
{
    voll &= 0X3F;
    volr &= 0X3F;
    if (voll == 0) voll |= 1 << 6; // Mute
    if (volr == 0) volr |= 1 << 6; // Mute
    
    WM8978_Write_Reg(52, voll);          // R52 L
    WM8978_Write_Reg(53, volr | (1 << 8)); // R53 R + Update
}

void WM8978_SPKvol_Set(uint8_t volx)
{
    volx &= 0X3F;
    if (volx == 0) volx |= 1 << 6; // Mute
    
    WM8978_Write_Reg(54, volx);          // R54 L
    WM8978_Write_Reg(55, volx | (1 << 8)); // R55 R + Update
}

void WM8978_3D_Set(uint8_t depth)
{
    depth &= 0XF;
    WM8978_Write_Reg(41, depth); // R41
}

void WM8978_EQ_3D_Dir(uint8_t dir)
{
    uint16_t regval;
    regval = WM8978_Read_Reg(0X12);
    if (dir) regval |= 1 << 8; // DAC path
    else     regval &= ~(1 << 8); // ADC path
    WM8978_Write_Reg(18, regval);
}

void WM8978_EQ1_Set(uint8_t cfreq, uint8_t gain)
{
    uint16_t regval;
    cfreq &= 0X3;
    if (gain > 24) gain = 24;
    gain = 24 - gain;
    regval = WM8978_Read_Reg(18);
    regval &= 0X100; // Keep EQ3DMODE
    regval |= cfreq << 5;
    regval |= gain;
    WM8978_Write_Reg(18, regval);
}

void WM8978_EQ2_Set(uint8_t cfreq, uint8_t gain)
{
    uint16_t regval = 0;
    cfreq &= 0X3;
    if (gain > 24) gain = 24;
    gain = 24 - gain;
    regval |= cfreq << 5;
    regval |= gain;
    WM8978_Write_Reg(19, regval);
}

void WM8978_EQ3_Set(uint8_t cfreq, uint8_t gain)
{
    uint16_t regval = 0;
    cfreq &= 0X3;
    if (gain > 24) gain = 24;
    gain = 24 - gain;
    regval |= cfreq << 5;
    regval |= gain;
    WM8978_Write_Reg(20, regval);
}

void WM8978_EQ4_Set(uint8_t cfreq, uint8_t gain)
{
    uint16_t regval = 0;
    cfreq &= 0X3;
    if (gain > 24) gain = 24;
    gain = 24 - gain;
    regval |= cfreq << 5;
    regval |= gain;
    WM8978_Write_Reg(21, regval);
}

void WM8978_EQ5_Set(uint8_t cfreq, uint8_t gain)
{
    uint16_t regval = 0;
    cfreq &= 0X3;
    if (gain > 24) gain = 24;
    gain = 24 - gain;
    regval |= cfreq << 5;
    regval |= gain;
    WM8978_Write_Reg(22, regval);
}

#endif
