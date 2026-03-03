#include "main.h"
#include "fatfs.h"
#include <string.h>

/*
 * Bootloader Reference Logic
 *
 * This code is intended to be used in a SEPARATE STM32CubeMX Project.
 *
 * Project Setup:
 * 1. Create New Project for STM32L452.
 * 2. Enable SDMMC1 (SD 4 bits wide).
 * 3. Enable FATFS (SD Card mode).
 * 4. Configure Clock (HSE/HSI/MSI as needed).
 * 5. Generate Code.
 *
 * Implementation:
 * Copy the following function into your Bootloader's main.c (before main loop).
 * Call it after MX_FATFS_Init().
 */

#define APP_ADDRESS     0x08008000
#define UPDATE_FILENAME "update.bin"

typedef void (*pFunction)(void);
pFunction JumpToApplication;
uint32_t JumpAddress;

void Bootloader_Process(void) {
    FATFS fs;
    FIL fil;
    FRESULT fr;
    UINT br;
    uint8_t buffer[512]; // Small buffer for reading
    uint32_t flash_address = APP_ADDRESS;
    
    // 1. Mount SD Card
    if (f_mount(&fs, "", 1) != FR_OK) {
        // Mount failed, maybe no card? Jump to App directly
        goto jump_to_app;
    }
    
    // 2. Check for Update File
    if (f_open(&fil, UPDATE_FILENAME, FA_READ) == FR_OK) {
        // Update found!
        
        // Unlock Flash
        HAL_FLASH_Unlock();
        
        // Erase App Area (Simple loop, erase page by page until file size)
        // Note: You should implement a proper erase loop based on file size
        // For simplicity, let's assume we erase needed pages as we go, or bulk erase beforehand.
        // Better: Erase based on file size.
        
        FLASH_EraseInitTypeDef EraseInitStruct;
        uint32_t PageError;
        uint32_t fileSize = f_size(&fil);
        uint32_t nbPages = (fileSize + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
        
        // Start Page for 0x08008000 (Page 16, assuming 2KB pages on L452?)
        // STM32L452 has 2KB pages. 0x8000 / 2048 = 16.
        uint32_t startPage = 16;
        
        EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.Banks       = FLASH_BANK_1; // Single bank usually for L452 < 512KB? Check ref manual.
        EraseInitStruct.Page        = startPage;
        EraseInitStruct.NbPages     = nbPages;
        
        if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
            // Erase Error
            HAL_FLASH_Lock();
            f_close(&fil);
            goto jump_to_app;
        }
        
        // Program Flash
        while (1) {
            fr = f_read(&fil, buffer, sizeof(buffer), &br);
            if (fr != FR_OK || br == 0) break;
            
            for (uint32_t i = 0; i < br; i += 8) {
                uint64_t data = 0;
                memcpy(&data, &buffer[i], (br - i) < 8 ? (br - i) : 8); // Handle partial last double-word
                
                if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_address, data) != HAL_OK) {
                    // Error
                    break; 
                }
                flash_address += 8;
            }
        }
        
        HAL_FLASH_Lock();
        f_close(&fil);
        
        // Rename or Delete update file to avoid loop
        f_unlink(UPDATE_FILENAME); 
        // Or rename: f_rename(UPDATE_FILENAME, "update.bak");
    }
    
jump_to_app:
    // 3. Jump to Application
    
    // Check if App exists (Stack Pointer at first address should be valid RAM address)
    if (((*(__IO uint32_t*)APP_ADDRESS) & 0x2FF00000 ) == 0x20000000) {
        
        // Disable all interrupts
        __disable_irq();
        
        // Set Vector Table
        SCB->VTOR = APP_ADDRESS;
        
        // Get Jump Address (Reset Handler)
        JumpAddress = *(__IO uint32_t*) (APP_ADDRESS + 4);
        JumpToApplication = (pFunction) JumpAddress;
        
        // Initialize user application's Stack Pointer
        __set_MSP(*(__IO uint32_t*) APP_ADDRESS);
        
        // Jump
        JumpToApplication();
    }
}
