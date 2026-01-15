#include "wavplay.h"
#include "wm8978.h"
#include "usb_device.h" // For USB_Print
#include <string.h>
#include <stdlib.h>

extern SAI_HandleTypeDef hsai_BlockA1;
extern void USB_Print(const char* format, ...);

__wavctrl wavctrl;
FIL wav_file;
uint8_t wav_dma_buffer[WAV_DMA_BUFFER_SIZE]; // Double Buffer logic handled by HAL
volatile uint8_t wav_transfer_complete = 0;
volatile uint8_t wav_half_transfer = 0;
uint8_t is_playing = 0;

// Callback from HAL_SAI_TxCpltCallback
void Wav_TransferComplete_Callback(void) {
    wav_transfer_complete = 1;
}

// Callback from HAL_SAI_TxHalfCpltCallback
void Wav_HalfTransfer_Callback(void) {
    wav_half_transfer = 1;
}

static uint8_t Wav_ParseHeader(const char* fname) {
    uint32_t br;
    uint8_t res;
    uint8_t header_buf[512];
    ChunkRIFF *riff;
    ChunkFMT *fmt;
    ChunkDATA *data;
    uint32_t offset = 0;

    res = f_open(&wav_file, fname, FA_READ);
    if (res != FR_OK) return 1;

    f_read(&wav_file, header_buf, 512, (UINT*)&br);
    if (br < sizeof(ChunkRIFF)) {
        f_close(&wav_file);
        return 2;
    }

    riff = (ChunkRIFF *)header_buf;
    if (riff->ChunkID != 0X46464952 || riff->Format != 0X45564157) { // "RIFF", "WAVE"
        f_close(&wav_file);
        return 3;
    }

    offset = 12;
    // Simple parser: assumes fmt chunk is first, then maybe fact/LIST, then data
    // A robust parser would loop through chunks. 
    // For simplicity, we check typical offset for "fmt "
    fmt = (ChunkFMT *)(header_buf + offset);
    if (fmt->ChunkID != 0X20746D66) { // "fmt "
        // Try to search for fmt chunk? Or just fail for now.
        // Let's implement a simple search within first 512 bytes
        uint32_t search_ptr = 12;
        int found = 0;
        while(search_ptr < 400) {
             if (*(uint32_t*)(header_buf + search_ptr) == 0X20746D66) {
                 fmt = (ChunkFMT *)(header_buf + search_ptr);
                 found = 1;
                 break;
             }
             search_ptr++;
        }
        if (!found) {
            f_close(&wav_file);
            return 4;
        }
    }
    
    offset = (uint32_t)((uint8_t*)fmt - header_buf) + 8 + fmt->ChunkSize;
    
    // Search for "data" chunk
    uint32_t search_ptr = offset;
    int found_data = 0;
    // We scan forward for "data" (0x61746164)
    while(search_ptr < 500) {
        if (*(uint32_t*)(header_buf + search_ptr) == 0X61746164) {
            data = (ChunkDATA *)(header_buf + search_ptr);
            found_data = 1;
            break;
        }
        search_ptr++;
    }

    if (!found_data) {
        // Data chunk might be beyond 512 bytes? unlikely for standard wavs
        f_close(&wav_file);
        return 5;
    }

    wavctrl.audioformat = fmt->AudioFormat;
    wavctrl.nchannels = fmt->NumOfChannels;
    wavctrl.samplerate = fmt->SampleRate;
    wavctrl.bitrate = fmt->ByteRate * 8;
    wavctrl.blockalign = fmt->BlockAlign;
    wavctrl.bps = fmt->BitsPerSample;
    wavctrl.datasize = data->ChunkSize;
    wavctrl.datastart = search_ptr + 8; // data chunk header + size (8 bytes)

    return 0;
}

uint8_t Wav_Play(const char* fname) {
    uint8_t res;
    uint32_t br;

    USB_Print("[WAV] Parsing %s...\r\n", fname);
    res = Wav_ParseHeader(fname);
    if (res != 0) {
        USB_Print("[WAV] Parse Failed: %d\r\n", res);
        return res;
    }

    USB_Print("[WAV] Info: %d Hz, %d Bit, %d Ch\r\n", wavctrl.samplerate, wavctrl.bps, wavctrl.nchannels);
    USB_Print("[WAV] Data Start: %u, Data Size: %u\r\n", wavctrl.datastart, wavctrl.datasize);

    // Configure WM8978 and SAI based on WAV format
    // WM8978 supports 16/20/24/32 bits. 
    // STM32 SAI DataSize should match.
    // For simplicity, we assume 16-bit or 24-bit.
    
    uint8_t word_len = 0;
    if (wavctrl.bps == 16) word_len = 0;      // 16-bit
    else if (wavctrl.bps == 24) word_len = 2; // 24-bit
    else if (wavctrl.bps == 32) word_len = 3; // 32-bit
    else {
        USB_Print("[WAV] Unsupported Bit Depth: %d\r\n", wavctrl.bps);
        f_close(&wav_file);
        return 6;
    }

    // Stop any ongoing transmission
    HAL_SAI_DMAStop(&hsai_BlockA1);

    // Reconfigure WM8978 I2S Format
    WM8978_I2S_Cfg(2, word_len); // I2S Standard

    // Reconfigure SAI (Sample Rate & Bit Depth) - Using HAL handle directly
    // Note: To change HAL SAI config properly, we should deinit and reinit.
    // But for now, let's try just disabling SAI, updating init struct, and Init.
    HAL_SAI_DeInit(&hsai_BlockA1);
    
    hsai_BlockA1.Init.AudioFrequency = 6000;//wavctrl.samplerate;
    if (wavctrl.bps == 16) {
        hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
    } else if (wavctrl.bps == 24) {
        hsai_BlockA1.Init.DataSize = SAI_DATASIZE_24; // Or 32 if padded
    } else {
        hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16; // Fallback
    }
    
    if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK) {
        USB_Print("[WAV] SAI Init Failed!\r\n");
        f_close(&wav_file);
        return 7;
    }

    // Prepare DMA Buffer
    // Seek to data start
    f_lseek(&wav_file, wavctrl.datastart);
    
    // Fill first half and second half
    f_read(&wav_file, wav_dma_buffer, WAV_DMA_BUFFER_SIZE, (UINT*)&br);
    
    // Start DMA
    is_playing = 1;
    wav_transfer_complete = 0;
    wav_half_transfer = 0;
    
    if (HAL_SAI_Transmit_DMA(&hsai_BlockA1, wav_dma_buffer, WAV_DMA_BUFFER_SIZE / (wavctrl.bps/8)) != HAL_OK) {
        USB_Print("[WAV] DMA Start Failed!\r\n");
        f_close(&wav_file);
        return 8;
    }

    USB_Print("[WAV] Playing...\r\n");
    return 0;
}

void Wav_Stop(void) {
    HAL_SAI_DMAStop(&hsai_BlockA1);
    f_close(&wav_file);
    is_playing = 0;
    USB_Print("[WAV] Stopped.\r\n");
}

// Call this in main loop to handle buffer refilling
void Wav_Process(void) {
    if (!is_playing) return;
	  

    uint32_t br;
	  FRESULT res;
    
    // Handle Half Transfer (Refill First Half)
    if (wav_half_transfer) {
        wav_half_transfer = 0;
        
        uint32_t start_ms = HAL_GetTick();
        res = f_read(&wav_file, &wav_dma_buffer[0], WAV_DMA_BUFFER_SIZE/2, (UINT*)&br);
        uint32_t end_ms = HAL_GetTick();
        
        // Calculate speed every 20 chunks (~1s) to avoid spamming
        static int count = 0;
        if (++count % 20 == 0) {
             float speed = (float)br / (end_ms - start_ms); // KB/s
             USB_Print("[PERF] Read %d bytes in %lu ms (%.1f KB/s)\r\n", br, end_ms - start_ms, speed);
        }

        if (res != FR_OK) {
             USB_Print("[WAV] Read Err 1: %d\r\n", res);
        }
        if (br < WAV_DMA_BUFFER_SIZE/2) {
            USB_Print("[WAV] EOF/Short Read 1: %d\r\n", br);
            // End of file (or close to it)
            // For loop playback, seek to start
            f_lseek(&wav_file, wavctrl.datastart);
            // Fill the rest with 0 or read from start?
            // Simple approach: just read from start to fill buffer
            f_read(&wav_file, &wav_dma_buffer[br], (WAV_DMA_BUFFER_SIZE/2) - br, (UINT*)&br);
        }
    }

    // Handle Full Transfer (Refill Second Half)
    if (wav_transfer_complete) {
        wav_transfer_complete = 0;
        res = f_read(&wav_file, &wav_dma_buffer[WAV_DMA_BUFFER_SIZE/2], WAV_DMA_BUFFER_SIZE/2, (UINT*)&br);
        if (res != FR_OK) {
             USB_Print("[WAV] Read Err 2: %d\r\n", res);
        }
        if (br < WAV_DMA_BUFFER_SIZE/2) {
            USB_Print("[WAV] EOF/Short Read 2: %d\r\n", br);
            // End of file
            f_lseek(&wav_file, wavctrl.datastart);
            f_read(&wav_file, &wav_dma_buffer[(WAV_DMA_BUFFER_SIZE/2) + br], (WAV_DMA_BUFFER_SIZE/2) - br, (UINT*)&br);
        }
    }
}

// Hook HAL Callbacks
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    if (hsai == &hsai_BlockA1) {
        Wav_HalfTransfer_Callback();
    }
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
    if (hsai == &hsai_BlockA1) {
        Wav_TransferComplete_Callback();
    }
}
