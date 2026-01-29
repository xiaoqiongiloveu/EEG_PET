/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "ADS1299.h"
#include "circular_buffer.h"
#include "soft_i2c.h"
#include "qmi8658_adapter.h"
#include "delay.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "command_system.h"
#include "commands_impl.h"
#include "ad5933.h"
#include "wm8978.h"
#include "wavplay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_sdmmc1;
DMA_HandleTypeDef hdma_sai1_a;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SAI_HandleTypeDef hsai_BlockA1;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN PV */
CircularBuffer ADS1299_circula_buffer;
uint8_t sd_write_buf[512]; // Buffer for SD card writes
UINT bw;                   // Bytes written
FRESULT res;               // FatFs function common result code
uint8_t sd_mounted = 0;    // Flag for SD mount status
uint8_t file_opened = 0;   // Flag for file open status

float Battery_Voltage = 0.0f;
float acc[3], gyro[3];
uint32_t last_qmi_print = 0;
uint8_t wm8978_initialized = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SAI1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */
void Send_EEG_Packet(uint8_t *raw_27_bytes);
void USB_Print(const char* format, ...);
void USB_SendString(const char* str);
void Power_Control_Init(void);
void Check_Power_Off(void);
void Update_Battery_Voltage(void);
void Send_AUX_Packet(float *acc, float *gyro, float vbat);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == ADS1299_DRDY_Pin)
  {
    ADS1299_DRDY_Handler();
  }
}

void SD_Init_Process(void)
{
    // 1. Mount File System
    // USB_Print("[SD] Mounting FS... (ClockDiv: %d)\r\n", hsd1.Init.ClockDiv);
    // Retry mount up to 3 times
    for(int i=0; i<3; i++) {
        res = f_mount(&SDFatFS, SDPath, 1);
        if(res == FR_OK) break;
        // USB_Print("[SD] Mount attempt %d failed: %d. Retrying...\r\n", i+1, res);
        HAL_Delay(100);
    }
    
    if(res != FR_OK)
    {
        // USB_Print("[SD] Mount Failed! Error: %d\r\n", res);
        return;
    }
    // USB_Print("[SD] Mount Success!\r\n");
    sd_mounted = 1;

    // 2. Test Write
    // USB_Print("[SD] Testing Write to '2025.txt'...\r\n");
    res = f_open(&SDFile, "2025.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if(res != FR_OK) {
        // USB_Print("[SD] Open for Write Failed! Error: %d\r\n", res);
        return;
    }
    
    char test_str[] = "20250113\r\n";
    UINT bytes_written;
    res = f_write(&SDFile, test_str, strlen(test_str), &bytes_written);
    if(res != FR_OK) {
        // USB_Print("[SD] Write Failed! Error: %d\r\n", res);
        f_close(&SDFile);
        return;
    }
    // USB_Print("[SD] Written %d bytes.\r\n", bytes_written);
    
    // Explicitly sync the file buffer to physical disk
    res = f_sync(&SDFile);
    if(res != FR_OK) {
        // USB_Print("[SD] Sync Failed! Error: %d\r\n", res);
    }

    f_close(&SDFile);

    // 3. Test Read
    // USB_Print("[SD] Testing Read from '2025.txt'...\r\n");
    res = f_open(&SDFile, "2025.txt", FA_READ);
    if(res != FR_OK) {
        // USB_Print("[SD] Open for Read Failed! Error: %d\r\n", res);
        return;
    }
    
    char read_buf[64] = {0};
    UINT bytes_read;
    res = f_read(&SDFile, read_buf, sizeof(read_buf)-1, &bytes_read);
    if(res != FR_OK) {
        // USB_Print("[SD] Read Failed! Error: %d\r\n", res);
        f_close(&SDFile);
        return;
    }
    // USB_Print("[SD] Read Content: %s\r\n", read_buf);
    f_close(&SDFile);
    
    // USB_Print("[SD] Test Complete.\r\n");
    
    // Prepare data.bin for logging (optional, if we were logging)
    // res = f_open(&SDFile, "data.bin", FA_OPEN_APPEND | FA_WRITE);
    // if(res == FR_OK) file_opened = 1;
}

void User_BSP_Init(void)
{
    // 1. Power Control (Must be first)
    Power_Control_Init();
    
    // 2. Soft I2C for Sensors
    Soft_I2C_Init();
}

void User_Device_Init(void)
{
    // 1. Initialize Circular Buffer (Required for ADS1299)
    circular_buffer_init(&ADS1299_circula_buffer, CIRCULAR_BUFFER_MAX_SIZE);

    // 2. ADS1299 EEG AFE Init
    ADS1299_Init();
    
    // 3. QMI8658 IMU Init
    // USB_Print("[BOOT] Initializing QMI8658...\r\n");
    if (QMI8658_Init() == 0) {
        // USB_Print("[BOOT] QMI8658 Init Success (ID=0x05)\r\n");
    } else {
        // USB_Print("[BOOT] QMI8658 Init Failed!\r\n");
    }
    
    // 4. SD Card Init
    // USB_Print("[BOOT] Initializing SD Card...\r\n");
    SD_Init_Process(); 

    // 5. AD5933 Impedance Converter Scan & Init
    // USB_Print("[I2C] Scanning I2C1 bus...\r\n");
    uint8_t ad5933_found = 0;
    for(uint16_t addr = 0; addr < 128; addr++) {
        if(HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 1, 10) == HAL_OK) {
            // USB_Print("[I2C] Found device at 0x%02X\r\n", addr);
            if (addr == 0x0D) ad5933_found = 1;
        }
    }

    if (ad5933_found) {
        // USB_Print("[BOOT] AD5933 Found! Initializing...\r\n");
        AD5933_Init();
    } else {
        // USB_Print("[BOOT] AD5933 NOT Found (Check connections to PB6/PB7)\r\n");
    }

    // 6. WM8978 Audio Codec Init
    if (WM8978_Init() == 0) {
        wm8978_initialized = 1;
        // USB_Print("[BOOT] WM8978 Init Success!\r\n");
        
        // Basic Config for DAC output (Headphones)
        WM8978_ADDA_Cfg(1, 0);      // Enable DAC, Disable ADC
        WM8978_Write_Reg(11, 0xFF); // Digital Volume Max
        
        // Enable Speaker Boost (Optional)
        uint16_t r49 = WM8978_Read_Reg(49);
        WM8978_Write_Reg(49, r49 | (1 << 2)); // Bit 2 is SPKBOOST

        WM8978_Input_Cfg(0, 0, 0);  // Disable Inputs
        WM8978_Output_Cfg(1, 0);    // Enable DAC Output
        WM8978_HPvol_Set(63, 63);   // Headphone Volume (Max 63)
        WM8978_SPKvol_Set(63);      // Speaker Volume (Max 63)
    } else {
        wm8978_initialized = 0;
        // USB_Print("[BOOT] WM8978 Init Failed!\r\n");
    }
}

void User_App_Init(void)
{
    // Register CLI Commands
    register_all_commands();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 4 */
/* USER CODE END 4 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  Delay_Init();
/* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  // MX_I2C2_Init();
  MX_SAI1_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  // 1. BSP Init (Power, Soft I2C)
  User_BSP_Init();

  // 2. Device Init (ADS1299, QMI8658, AD5933)
  User_Device_Init();

  /* 
   * Note: ADS1299 Auto-start and WM8978 test code were removed 
   * during refactoring to keep main() clean.
   * See User_Device_Init() for sensor details.
   */

  // 3. App Init (CLI, Buffers)
  User_App_Init();
  
  // Play 1.wav after init
  Wav_Play("AUDIO/1.wav");
  while(is_playing) {
      Wav_Process();
  }
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (usb_cmd_ready) {
        usb_cmd_ready = 0;
        process_command_ex(usb_cmd_buffer);
    }
    
    app_run_loop();
    
    // Process WAV double buffer filling
    Wav_Process();
    
//    
    Check_Power_Off();

    // 1. Check for new EEG Data (250Hz)
//    if (circular_buffer_get_data_count(&ADS1299_circula_buffer) >= 27) {
//        uint8_t packet[27];
//        circular_buffer_read(&ADS1299_circula_buffer, packet, 27);
//        Send_EEG_Packet(packet);
//    }

//    // 2. Check for AUX Data (1Hz)
//    if (HAL_GetTick() - last_qmi_print >= 1000) {
//        // Read IMU
//        QMI8658_Read_XYZ(acc, gyro);
//        
//        // Read Battery
//        Update_Battery_Voltage();
//        
//        // Send Packet
//        Send_AUX_Packet(acc, gyro, Battery_Voltage);
//        
//        last_qmi_print = HAL_GetTick();
//    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_48M2CLK
                              |RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_I2S_STANDARD;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  /* FrameInit and SlotInit are not used for SAI_I2S_STANDARD protocol */
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv = 0; // 48MHz / (4+2) = 8MHz (Stable for polling)
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_1_Pin|ADS1299_RESET_Pin|ADS1299_START_Pin|ADS1299_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, G_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, POWER_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_1_Pin ADS1299_RESET_Pin ADS1299_START_Pin ADS1299_CS_Pin
                           G_CS_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|ADS1299_RESET_Pin|ADS1299_START_Pin|ADS1299_CS_Pin
                          |G_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : POWER_CTRL_Pin */
  GPIO_InitStruct.Pin = POWER_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : POWER_KEY_Pin */
  GPIO_InitStruct.Pin = POWER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(POWER_KEY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /*Configure GPIO pin : ADS1299_DRDY_Pin */
  GPIO_InitStruct.Pin = ADS1299_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ADS1299_DRDY_GPIO_Port, &GPIO_InitStruct);

  /* SCL (PB10) 三轴IIC */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SDA (PB11) 三轴IIC */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // 
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Power_Control_Init(void) {
    // Check Power Key (PA1) - Active Low
    // Note: PA1 is configured as INPUT in MX_GPIO_Init
    
    if (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(20); // 20ms debounce
        if (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET) {
            uint32_t press_start = HAL_GetTick();
            while (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET) {
                if (HAL_GetTick() - press_start >= 2000) {
                    // Confirm press for 2 seconds -> Power ON
                    HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_SET); // Latch Power (PA4 High)
                    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); // LED On (Active Low)
                    return;
                }
            }
        }
    }
    // If not pressed for 2 seconds, do not latch power.
    // Power will cut off when button is released.
}

void Check_Power_Off(void) {
    // Power Off Detection
    if (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(20); // 20ms debounce
        if (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET) {
            uint32_t press_start = HAL_GetTick();
            while (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET) {
                if (HAL_GetTick() - press_start >= 2000) {
                    // Confirm press for 2 seconds -> Power OFF
                    
                    // Play 2.wav before power off
                    Wav_Play("AUDIO/2.wav");
                    while(is_playing) {
                        Wav_Process();
                    }
                    
                    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); // LED Off
                    HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_RESET); // Unlatch Power (PA4 Low)
                    while(1); // Wait for power cut
                }
            }
        }
    }
}

void Update_Battery_Voltage(void) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        uint32_t raw = HAL_ADC_GetValue(&hadc1);
        
        // Use user-defined calibration formula
        Battery_Voltage = ((float)raw * 3.3f / 4095.0f) + 1.0f;
        
        // Convert to percentage: 3.6V = 0%, 4.1V = 100%
        float percentage = (Battery_Voltage - 3.6f) / (4.1f - 3.6f) * 100.0f;
        
        if (percentage < 0.0f) percentage = 0.0f;
        if (percentage > 100.0f) percentage = 100.0f;
        
        // Store percentage in Battery_Voltage variable to be sent out
        Battery_Voltage = percentage;
    }
    HAL_ADC_Stop(&hadc1);
}

void Send_AUX_Packet(float *acc, float *gyro, float vbat) {
    // Print AUX data as text
    USB_Print("AUX:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", 
              acc[0], acc[1], acc[2], 
              gyro[0], gyro[1], gyro[2], 
              vbat);
}

void USB_SendString(const char* str) {
    if (str) {
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
        if (hcdc == NULL) return;

        // Wait until USB is free
        uint32_t start = HAL_GetTick();
        while (hcdc->TxState != 0) {
            if (HAL_GetTick() - start > 100) return; // Timeout 100ms
        }
        CDC_Transmit_FS((uint8_t*)str, strlen(str));
    }
}


#include "usbd_cdc.h"

void USB_Print(const char* format, ...) {
    static char buffer[256]; // Use static buffer to ensure memory validity during DMA/Interrupt transfer
    
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc == NULL) return;

    // Wait until USB is free BEFORE writing to the shared static buffer
    // This prevents overwriting data that is currently being sent
    uint32_t start = HAL_GetTick();
    while (hcdc->TxState != 0) {
        if (HAL_GetTick() - start > 100) return; // Timeout 100ms
    }

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
}

void Send_EEG_Packet(uint8_t *raw_27_bytes) {
    float voltages[8];
    // Vref=4.5V, Gain=24. 
    // Scale factor to convert 24-bit integer to uV.
    // 4.5V = 4500000uV. 2^23 - 1 = 8388607.
    float scale = 4500000.0f / 8388607.0f / 24.0f; 

    for(int i=0; i<8; i++) {
        // raw_27_bytes: [Status(3), Ch1(3), Ch2(3)...]
        int ch_offset = 3 + i * 3;
        int32_t val = (raw_27_bytes[ch_offset] << 16) | (raw_27_bytes[ch_offset+1] << 8) | raw_27_bytes[ch_offset+2];
        
        // Sign extension for 24-bit number
        if (val & 0x800000) val |= 0xFF000000;
        
        voltages[i] = (float)val * scale;
    }
    
    // Print as CSV text, matching reference project style
    // Add "EEG:" prefix to distinguish from AUX data
    USB_Print("EEG:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", 
        voltages[0], voltages[1], voltages[2], voltages[3],
        voltages[4], voltages[5], voltages[6], voltages[7]);
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
