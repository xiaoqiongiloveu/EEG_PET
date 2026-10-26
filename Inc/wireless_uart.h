#ifndef WIRELESS_UART_H
#define WIRELESS_UART_H

#include <stdint.h>

void Wireless_UART_Init(void);
void Wireless_UART_Poll(void);
void Wireless_UART_SelectUSB(void);
uint8_t Wireless_UART_IsSelected(void);
uint8_t Transport_Transmit(uint8_t *data, uint16_t len);
int Transport_WaitTxIdle(uint32_t timeout_ms);

extern volatile uint32_t g_wireless_tx_overflow;
extern volatile uint32_t g_wireless_rx_overflow;

#endif
