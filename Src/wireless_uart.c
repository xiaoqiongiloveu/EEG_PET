#include "wireless_uart.h"

#include "binary_protocol.h"
#include "command_system.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "usb_device.h"

#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

#define WIRELESS_BAUD_RATE 1000000U
#define WIRELESS_TX_SIZE 4096U
#define WIRELESS_RX_SIZE 4096U

static UART_HandleTypeDef huart1;
static DMA_HandleTypeDef hdma_usart1_tx;
static DMA_HandleTypeDef hdma_usart1_rx;
static uint8_t tx_buffer[WIRELESS_TX_SIZE];
static uint8_t rx_buffer[WIRELESS_RX_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint16_t tx_dma_length;
static volatile uint8_t tx_active;
static volatile uint32_t rx_wraps;
static uint32_t rx_read;
static volatile uint8_t rx_restart_requested;
static uint8_t initialized;
static volatile uint8_t wireless_selected;
static BP_Parser_t wireless_parser;
static char wireless_command[128];
static uint16_t wireless_command_length;

volatile uint32_t g_wireless_tx_overflow;
volatile uint32_t g_wireless_rx_overflow;

extern void BP_Command_Callback(uint8_t type, uint16_t seq, uint32_t ts,
                                uint8_t *payload, uint16_t len);

static void Wireless_BinaryCommand(uint8_t type, uint16_t seq, uint32_t ts,
                                   uint8_t *payload, uint16_t len)
{
    if (type == BP_TYPE_CMD) {
        wireless_command_length = 0;
        wireless_selected = 1;
        BP_Command_Callback(type, seq, ts, payload, len);
    }
}

static void Wireless_StartTx(void)
{
    uint16_t length;
    if (tx_active || tx_head == tx_tail) return;

    length = (tx_head > tx_tail) ? (tx_head - tx_tail) : (WIRELESS_TX_SIZE - tx_tail);
    tx_active = 1;
    tx_dma_length = length;
    if (HAL_UART_Transmit_DMA(&huart1, &tx_buffer[tx_tail], length) != HAL_OK) {
        tx_active = 0;
    }
}

static uint8_t Wireless_Queue(const uint8_t *data, uint16_t len)
{
    uint32_t primask = __get_PRIMASK();
    uint16_t used;
    uint16_t first;

    __disable_irq();
    used = (tx_head >= tx_tail) ? (tx_head - tx_tail)
                                : (WIRELESS_TX_SIZE - tx_tail + tx_head);
    if (len > WIRELESS_TX_SIZE - 1U - used) {
        g_wireless_tx_overflow++;
        __set_PRIMASK(primask);
        return USBD_BUSY;
    }

    first = WIRELESS_TX_SIZE - tx_head;
    if (first > len) first = len;
    memcpy(&tx_buffer[tx_head], data, first);
    memcpy(tx_buffer, data + first, len - first);
    tx_head = (tx_head + len) % WIRELESS_TX_SIZE;
    Wireless_StartTx();
    __set_PRIMASK(primask);
    return USBD_OK;
}

void Wireless_UART_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* Active-low WiFi power control: keep PA8 low while the module is running. */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = WIRELESS_BAUD_RATE;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    /* AF7 names PA9 as TX and PA10 as RX; SWAP makes PA10 TX and PA9 RX. */
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
    huart1.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();

    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_2;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_2;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    BP_Init(&wireless_parser, Wireless_BinaryCommand);
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    if (HAL_UART_Receive_DMA(&huart1, rx_buffer, WIRELESS_RX_SIZE) != HAL_OK) {
        Error_Handler();
    }
    initialized = 1;
}

void Wireless_UART_SelectUSB(void)
{
    wireless_selected = 0;
}

uint8_t Wireless_UART_IsSelected(void)
{
    return wireless_selected;
}

uint8_t Transport_Transmit(uint8_t *data, uint16_t len)
{
    if (wireless_selected && initialized) return Wireless_Queue(data, len);
    if (hUsbDeviceFS.pClassData == NULL) return USBD_FAIL;
    return CDC_Transmit_FS(data, len);
}

int Transport_WaitTxIdle(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    if (wireless_selected && initialized) {
        while (tx_head != tx_tail || tx_active) {
            if (HAL_GetTick() - start > timeout_ms) return 0;
        }
        return 1;
    }

    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL) return 0;
    while (hcdc->TxState != 0) {
        if (HAL_GetTick() - start > timeout_ms) return 0;
    }
    return 1;
}

static uint32_t Wireless_RxProduced(void)
{
    uint32_t before, after, remaining;
    do {
        before = rx_wraps;
        remaining = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        after = rx_wraps;
    } while (before != after);
    uint32_t produced = after * WIRELESS_RX_SIZE + (WIRELESS_RX_SIZE - remaining);
    if (produced < rx_read) produced += WIRELESS_RX_SIZE;
    return produced;
}

void Wireless_UART_Poll(void)
{
    uint32_t produced;
    if (!initialized) return;

    if (rx_restart_requested) {
        if (HAL_UART_AbortReceive(&huart1) != HAL_OK) return;
        rx_wraps = 0;
        rx_read = 0;
        wireless_command_length = 0;
        BP_Init(&wireless_parser, Wireless_BinaryCommand);
        if (HAL_UART_Receive_DMA(&huart1, rx_buffer, WIRELESS_RX_SIZE) != HAL_OK) return;
        rx_restart_requested = 0;
    }

    produced = Wireless_RxProduced();
    if (g_file_transfer_mode && !wireless_selected) {
        rx_read = produced;
        return;
    }
    if (produced - rx_read > WIRELESS_RX_SIZE) {
        g_wireless_rx_overflow++;
        rx_read = produced - WIRELESS_RX_SIZE;
        wireless_command_length = 0;
        BP_Init(&wireless_parser, Wireless_BinaryCommand);
    }

    while (rx_read != produced) {
        uint8_t byte = rx_buffer[rx_read % WIRELESS_RX_SIZE];
        BP_State_t previous_state = wireless_parser.state;
        rx_read++;
        if (g_file_transfer_mode) {
            handle_file_transfer(&byte, 1);
            continue;
        }

        BP_ProcessByte(&wireless_parser, byte);
        if (previous_state >= BP_STATE_LEN1 ||
            wireless_parser.state >= BP_STATE_LEN1 ||
            (previous_state == BP_STATE_SYNC2 && byte == 0xAA)) {
            if (previous_state == BP_STATE_SYNC2 && byte == 0xAA) {
                wireless_command_length = 0;
            }
            continue;
        }
        if (byte == '\r') continue;
        if (byte == '\n') {
            if (wireless_command_length) {
                wireless_command[wireless_command_length] = '\0';
                wireless_selected = 1;
                process_command_ex(wireless_command);
                wireless_command_length = 0;
            }
        } else if (wireless_command_length < sizeof(wireless_command) - 1U) {
            wireless_command[wireless_command_length++] = (char)byte;
        } else {
            wireless_command_length = 0;
        }
    }
}

void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void DMA1_Channel5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1) return;
    tx_tail = (tx_tail + tx_dma_length) % WIRELESS_TX_SIZE;
    tx_active = 0;
    Wireless_StartTx();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) rx_wraps++;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1) return;
    g_wireless_rx_overflow++;
    rx_restart_requested = 1;
}
