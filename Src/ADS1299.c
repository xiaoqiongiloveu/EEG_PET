#include "ADS1299.h"
#include "circular_buffer.h"
#include <string.h>
#include "main.h"

extern SPI_HandleTypeDef hspi1;
extern CircularBuffer ADS1299_circula_buffer;

#define SPI1_Handler hspi1
#define ADS1299_CS_L  HAL_GPIO_WritePin(ADS1299_CS_GPIO_Port, ADS1299_CS_Pin, GPIO_PIN_RESET)
#define ADS1299_CS_H  HAL_GPIO_WritePin(ADS1299_CS_GPIO_Port, ADS1299_CS_Pin, GPIO_PIN_SET)
#define ADS1299_REST_L HAL_GPIO_WritePin(ADS1299_RESET_GPIO_Port, ADS1299_RESET_Pin, GPIO_PIN_RESET)
#define ADS1299_REST_H HAL_GPIO_WritePin(ADS1299_RESET_GPIO_Port, ADS1299_RESET_Pin, GPIO_PIN_SET)

static uint8_t spi_tx_buf[30];
static uint8_t spi_rx_buf[30];
uint8_t ADS1299_REG[24];
uint8_t ADS1299_REGt[24];
uint8_t pga = 12;

void ADS1299_SPI_RW(uint8_t *tx_data, uint8_t *rx_data, uint8_t len)
{
    HAL_SPI_TransmitReceive(&SPI1_Handler, tx_data, rx_data, len, 10);
}

void ADS1299_SPI_Rate_Set(uint32_t baudRatePrescaler)
{
    SPI1_Handler.Init.BaudRatePrescaler = baudRatePrescaler;
    if (HAL_SPI_Init(&SPI1_Handler) != HAL_OK)
    {
        Error_Handler();
    }
}

void ADS1299_Send_Cmd(uint8_t cmd)
{
    ADS1299_CS_L;
    spi_tx_buf[0] = cmd;
    ADS1299_SPI_RW(spi_tx_buf, spi_rx_buf, 1);
    ADS1299_CS_H;
}

void ADS1299_Write_Reg(uint8_t addr, uint8_t value)
{
    spi_tx_buf[0] = 0x40 + addr;
    spi_tx_buf[1] = 0x00;
    spi_tx_buf[2] = value;

    ADS1299_CS_L;
    ADS1299_SPI_RW(spi_tx_buf, spi_rx_buf, 3);
    ADS1299_CS_H;
}

void ADS1299_Write_Regs(uint8_t addr, uint8_t *regs, uint8_t len)
{
    spi_tx_buf[0] = 0x40 + addr;
    spi_tx_buf[1] = 0x00 + len - 1;
    ADS1299_CS_L;
    ADS1299_SPI_RW(spi_tx_buf, spi_rx_buf, 2);
    ADS1299_SPI_RW(regs, spi_rx_buf, len);
    ADS1299_CS_H;
}

void ADS1299_Read_Regs(uint8_t addr, uint8_t *regs, uint8_t len)
{
    spi_tx_buf[0] = 0x20 + addr;
    spi_tx_buf[1] = 0x00 + len - 1;
    ADS1299_CS_L;
    ADS1299_SPI_RW(spi_tx_buf, spi_rx_buf, 2);
    memset(spi_tx_buf, 0x00, sizeof(spi_tx_buf));
    ADS1299_SPI_RW(spi_tx_buf, regs, len);
    ADS1299_CS_H;
}

void ADS1299_Rest(void)
{
    ADS1299_CS_H;
    ADS1299_REST_H;
    HAL_Delay(200);

    ADS1299_REST_L;
    HAL_Delay(10);
    ADS1299_REST_H;
    HAL_Delay(50);

    ADS1299_Send_Cmd(ADS1299_SDATAC);
    ADS1299_Send_Cmd(ADS1299_STOP);
}

void ADS1299_Init(void)
{
    ADS1299_Rest();
    
    // Check ID to verify communication
    uint8_t id = 0;
    ADS1299_Read_Regs(0x00, &id, 1);
    USB_Print("[ADS1299] Device ID Read: 0x%02X\r\n", id);

    ADS1299_REG[0x00] = 0x1C;
    ADS1299_REG[0x01] = 0x96;
    ADS1299_REG[0x02] = 0xC0;
    ADS1299_REG[0x03] = 0xEC; // 
    ADS1299_REG[0x04] = 0x00;

    ADS1299_REG[0x05] = 0x60; // 
    ADS1299_REG[0x06] = 0x60; //
    ADS1299_REG[0x07] = 0x60; //
    ADS1299_REG[0x08] = 0x60; //
    ADS1299_REG[0x09] = 0x60;
    ADS1299_REG[0x0A] = 0x60;
    ADS1299_REG[0x0B] = 0x60;
    ADS1299_REG[0x0C] = 0x60;

    ADS1299_REG[0x0D] = 0xFF; // FF : all p to RLD bias
    ADS1299_REG[0x0E] = 0xFF; // 
    ADS1299_REG[0x0F] = 0x00; //
    ADS1299_REG[0x10] = 0x00; //
    ADS1299_REG[0x11] = 0x00; // Lead-Off Current Direction
    ADS1299_REG[0x12] = 0x00; // P Lead-Off Off
    ADS1299_REG[0x13] = 0x00; // N Lead-Off Off
    ADS1299_REG[0x14] = 0x00; // 0f
    ADS1299_REG[0x15] = 0x20; // all n connected to SRB1
    ADS1299_REG[0x16] = 0x00; //
    ADS1299_REG[0x17] = 0x00; // Continuous conversion

    ADS1299_Write_Regs(0x01, ADS1299_REG + 1, 23);

    HAL_Delay(10);

    ADS1299_Read_Regs(0x00, ADS1299_REGt, 24);

    ADS1299_REGt[0] &= 0x1F;

    
    HAL_Delay(150); //delay 150ms

    // Removed USB_Print loop to avoid dependency on usbd_cdc_if.h in driver file if not necessary, 
    // or add #include "usbd_cdc_if.h" if USB logging is desired here.
    // For now, I'll comment it out to minimize dependencies, or keep it if I include the header.
    // The user wants it to run, so I'll leave it out or replace with empty loop if needed.
    // Re-checking old code: it had #include "usbd_cdc_if.h". I will include it if it exists in new project.
    // New project has "usbd_cdc_if.h" in Inc.
    
    USB_Print("\r\n=== ADS1299 Registers (Init) ===\r\n");
    for(uint8_t n=0;n<24;n++)
    {
        USB_Print("ADS1299_reg[0x%x] = 0x%x \r\n",n,ADS1299_REGt[n]);
        // HAL_Delay(10); // Don't use HAL_Delay in init if called from interrupt (though Init usually isn't)
        for(volatile int w=0; w<10000; w++); 
    }

    ADS1299_SPI_Rate_Set(SPI_BAUDRATEPRESCALER_16);
}

// === Compatibility Wrappers for commands_impl.c ===

void ADS1299_Start(void) {
//	  memset(spi_tx_buf,0x00,sizeof(spi_tx_buf));
    ADS1299_Send_Cmd(ADS1299_RDATAC);
    ADS1299_Send_Cmd(ADS1299_START);
}

void ADS1299_Stop(void) {
    ADS1299_Send_Cmd(ADS1299_SDATAC);
    ADS1299_Send_Cmd(ADS1299_STOP);
}

void ADS1299_Set_Sample_Parameter(uint8_t dr, uint8_t gain, uint8_t input_type) {
    // 1. Set Data Rate (CONFIG1)
    uint8_t config1 = 0x90 | (dr & 0x07); 
    ADS1299_Write_Reg(ADS1299_CONFIG1, config1);
    
    // 2. Configure All Channels (CHnSET)
    uint8_t ch_settings = 0x00;
    ch_settings |= ((gain & 0x07) << 4);     // Set Gain
    ch_settings |= (input_type & 0x07);      // Set Input Type (MUX)
    
    for(int i = 1; i < 8; i++) {
        ADS1299_Write_Reg(ADS1299_CH1SET + i, ch_settings);
    }
}

void ADS1299_WriteRegister(uint8_t reg, uint8_t value) {
    ADS1299_Write_Reg(reg, value);
}

uint8_t ADS1299_ReadRegister(uint8_t reg) {
    uint8_t val;
    ADS1299_Read_Regs(reg, &val, 1);
    return val;
}

void ADS1299_DRDY_Handler(void) {
    uint32_t ts = HAL_GetTick();
    uint8_t rx[27];
    memset(rx, 0, 27);
    
    ADS1299_CS_L;
    // Just receive 27 bytes (3 status + 24 data)
    ADS1299_SPI_RW(rx, rx, 27); 
    ADS1299_CS_H;
    
    // Ensure we only push if there is enough room for the FULL 31-byte packet.
    // This prevents partial packet pushes which permanently destroy frame alignment.
    uint32_t data_count = circular_buffer_get_data_count(&ADS1299_circula_buffer);
    uint32_t free_space = ADS1299_circula_buffer.size - 1 - data_count;
    
    if (free_space >= 31) {
        for(int i=0; i<27; i++) {
            circular_buffer_push(&ADS1299_circula_buffer, rx[i]);
        }
        circular_buffer_push(&ADS1299_circula_buffer, (ts >> 24) & 0xFF);
        circular_buffer_push(&ADS1299_circula_buffer, (ts >> 16) & 0xFF);
        circular_buffer_push(&ADS1299_circula_buffer, (ts >> 8) & 0xFF);
        circular_buffer_push(&ADS1299_circula_buffer, ts & 0xFF);
    } else {
        // Buffer overflow: drop the entire packet to maintain alignment
        // (Optional: increment an overflow counter here)
    }
}
