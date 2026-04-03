#include "wavplay.h"
#include "wm8978.h"
#include "usb_device.h" // For USB_Print
#include <string.h>
#include <stdlib.h>

extern SAI_HandleTypeDef hsai_BlockA1;
extern void USB_Print(const char* format, ...);

__wavctrl wavctrl;
FIL wav_file;

// Double Buffer for DMA (16-bit Stereo samples)
// If playing 24-bit, we might need to adjust, but let's assume 16-bit for now as base case.
uint8_t wav_dma_buffer[WAV_DMA_BUFFER_SIZE]; 

// Temp Buffer for reading from SD card (Half size of DMA buffer is enough usually, 
// but if expanding mono->stereo, we need read buffer to be smaller)
// Let's define a read buffer size.
#define WAV_READ_BUFFER_SIZE (WAV_DMA_BUFFER_SIZE / 2)
uint8_t wav_read_buffer[WAV_READ_BUFFER_SIZE];

volatile uint8_t wav_transfer_complete = 0;
volatile uint8_t wav_half_transfer = 0;
uint8_t is_playing = 0;
uint8_t wav_loop_mode = 0;

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
    uint32_t chunk_id, chunk_size;
    uint32_t file_offset = 12; // Start after RIFF header
    uint32_t file_size = 0;
    
    // Header Buffers
    ChunkRIFF riff;
    ChunkFMT fmt;
    
    res = f_open(&wav_file, fname, FA_READ);
    if (res != FR_OK) return 1;

    // 1. Read RIFF Header
    f_read(&wav_file, &riff, sizeof(ChunkRIFF), (UINT*)&br);
    if (br < sizeof(ChunkRIFF)) {
        f_close(&wav_file);
        return 2;
    }

    if (riff.ChunkID != 0X46464952 || riff.Format != 0X45564157) { // "RIFF", "WAVE"
        f_close(&wav_file);
        return 3;
    }
    file_size = riff.ChunkSize + 8; // Total file size

    // 2. Iterate through Chunks
    int found_fmt = 0;
    int found_data = 0;

    while (file_offset < file_size) {
        // Read Chunk Header (8 bytes)
        res = f_lseek(&wav_file, file_offset);
        if (res != FR_OK) break;
        
        f_read(&wav_file, &chunk_id, 4, (UINT*)&br);
        if (br < 4) break;
        f_read(&wav_file, &chunk_size, 4, (UINT*)&br);
        if (br < 4) break;

        // Process Chunk
        if (chunk_id == 0X20746D66) { // "fmt "
            f_read(&wav_file, (void*)&fmt.AudioFormat, sizeof(ChunkFMT) - 8, (UINT*)&br); // Read content of fmt starting from AudioFormat
            
            wavctrl.audioformat = fmt.AudioFormat;
            wavctrl.nchannels = fmt.NumOfChannels;
            wavctrl.samplerate = fmt.SampleRate;
            wavctrl.bitrate = fmt.ByteRate * 8;
            wavctrl.blockalign = fmt.BlockAlign;
            wavctrl.bps = fmt.BitsPerSample;
            found_fmt = 1;
        } 
        else if (chunk_id == 0X61746164) { // "data"
            wavctrl.datasize = chunk_size;
            wavctrl.datastart = file_offset + 8; // Data starts after header
            found_data = 1;
            break; // Stop after finding data (assuming fmt is before data)
        }
        
        // Move to next chunk
        file_offset += 8 + chunk_size;
        
        // Handle padding byte if chunk size is odd
        if (chunk_size % 2 != 0) file_offset++;
    }

    if (!found_fmt) {
        f_close(&wav_file);
        return 4; // fmt chunk not found
    }

    if (!found_data) {
        f_close(&wav_file);
        return 5; // data chunk not found
    }
    
    // Support non-PCM format (e.g. 0xFFFE extensible) if simple PCM is valid
    if (wavctrl.audioformat != 1 && wavctrl.audioformat != 0xFFFE) {
         USB_Print("[WAV] Warning: Non-PCM format %d\r\n", wavctrl.audioformat);
         // Don't fail here, try to play anyway as some WAVs are tricky
    }

    return 0;
}

// Helper to expand Mono 16-bit to Stereo 16-bit
// dst_buf size must be 2x src_buf size
static void ExpandMonoToStereo16(uint8_t *dst, uint8_t *src, uint32_t src_bytes) {
    int16_t *s = (int16_t*)src;
    int16_t *d = (int16_t*)dst;
    uint32_t samples = src_bytes / 2;
    
    for(uint32_t i=0; i<samples; i++) {
        d[i*2]     = s[i]; // Left
        d[i*2 + 1] = s[i]; // Right
    }
}

static void FillBuffer(uint8_t *buffer, uint32_t buffer_size) {
    UINT br;
    FRESULT res;
    
    if (wavctrl.nchannels == 1 && wavctrl.bps == 16) {
        // Mono 16-bit -> Read half amount, expand to stereo
        res = f_read(&wav_file, wav_read_buffer, buffer_size / 2, &br);
        if (res != FR_OK || br == 0) {
             if (wav_loop_mode && br == 0) {
                 f_lseek(&wav_file, wavctrl.datastart);
                 f_read(&wav_file, wav_read_buffer, buffer_size / 2, &br);
             }
             if (br == 0) {
                 memset(buffer, 0, buffer_size);
                 Wav_Stop();
                 return;
             }
        }
        ExpandMonoToStereo16(buffer, wav_read_buffer, br);
        // If read less than expected (EOF), zero pad the rest
        if (br < buffer_size / 2) {
             if (wav_loop_mode) {
                 f_lseek(&wav_file, wavctrl.datastart);
                 // Fill the rest
                 UINT br2;
                 f_read(&wav_file, wav_read_buffer, (buffer_size / 2) - br, &br2);
                 ExpandMonoToStereo16(buffer + br*2, wav_read_buffer, br2);
                 if (br + br2 < buffer_size / 2) {
                     memset(buffer + (br+br2)*2, 0, buffer_size - (br+br2)*2);
                 }
             } else {
                 memset(buffer + br*2, 0, buffer_size - br*2);
                 Wav_Stop();
             }
        }
    } else {
        // Already Stereo or other format -> Read directly
        res = f_read(&wav_file, buffer, buffer_size, &br);
        if (res != FR_OK || br == 0) {
             if (wav_loop_mode && br == 0) {
                 f_lseek(&wav_file, wavctrl.datastart);
                 f_read(&wav_file, buffer, buffer_size, &br);
             }
             if (br == 0) {
                 memset(buffer, 0, buffer_size);
                 Wav_Stop();
                 return;
             }
        }
        if (br < buffer_size) {
             if (wav_loop_mode) {
                 f_lseek(&wav_file, wavctrl.datastart);
                 UINT br2;
                 f_read(&wav_file, buffer + br, buffer_size - br, &br2);
                 if (br + br2 < buffer_size) {
                     memset(buffer + br + br2, 0, buffer_size - (br + br2));
                 }
             } else {
                 memset(buffer + br, 0, buffer_size - br);
                 Wav_Stop();
             }
        }
    }
}

uint8_t Wav_Play(const char* fname) {
    uint8_t res;
    // uint32_t br;

    // Force stop any ongoing playback to prevent FATFS handle corruption or DMA deadlock
    Wav_Stop();

    USB_Print("[WAV] Parsing %s...\r\n", fname);
    res = Wav_ParseHeader(fname);
    if (res != 0) {
        USB_Print("[WAV] Parse Failed: %d\r\n", res);
        return res;
    }

    USB_Print("[WAV] Info: %d Hz, %d Bit, %d Ch\r\n", wavctrl.samplerate, wavctrl.bps, wavctrl.nchannels);
    USB_Print("[WAV] Data Start: %u, Data Size: %u\r\n", wavctrl.datastart, wavctrl.datasize);

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

    // Reconfigure SAI
    HAL_SAI_DeInit(&hsai_BlockA1);
    
    // Always use Stereo Mode for hardware consistency
    hsai_BlockA1.Init.AudioFrequency = 6000;//wavctrl.samplerate;
    hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE; 

    if (wavctrl.bps == 16) {
        hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
    } else if (wavctrl.bps == 24) {
        hsai_BlockA1.Init.DataSize = SAI_DATASIZE_24; 
    } else {
        hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16; 
    }
    
    if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK) {
        USB_Print("[WAV] SAI Init Failed!\r\n");
        f_close(&wav_file);
        return 7;
    }

    // Prepare DMA Buffer
    f_lseek(&wav_file, wavctrl.datastart);
    
    // Initial Fill
    // First Half
    FillBuffer(&wav_dma_buffer[0], WAV_DMA_BUFFER_SIZE/2);
    // Second Half
    FillBuffer(&wav_dma_buffer[WAV_DMA_BUFFER_SIZE/2], WAV_DMA_BUFFER_SIZE/2);
    
    // Start DMA
    is_playing = 1;
    wav_transfer_complete = 0;
    wav_half_transfer = 0;
    
    // For DMA, we always send stereo data now (expanded if needed)
    // So the size is always consistent with stereo 16/24/32 bit
    if (HAL_SAI_Transmit_DMA(&hsai_BlockA1, wav_dma_buffer, WAV_DMA_BUFFER_SIZE / (wavctrl.bps/8)) != HAL_OK) {
        USB_Print("[WAV] DMA Start Failed!\r\n");
        f_close(&wav_file);
        return 8;
    }

    USB_Print("[WAV] Playing...\r\n");
    return 0;
}

void Wav_Stop(void) {
    if (is_playing) {
        HAL_SAI_DMAStop(&hsai_BlockA1);
        f_close(&wav_file);
        is_playing = 0;
        USB_Print("[WAV] Stopped.\r\n");
    }
}

// Call this in main loop to handle buffer refilling
void Wav_Process(void) {
    if (!is_playing) return;
	  
    // Handle Half Transfer (Refill First Half)
    if (wav_half_transfer) {
        wav_half_transfer = 0;
        FillBuffer(&wav_dma_buffer[0], WAV_DMA_BUFFER_SIZE/2);
    }

    // Handle Full Transfer (Refill Second Half)
    if (wav_transfer_complete) {
        wav_transfer_complete = 0;
        FillBuffer(&wav_dma_buffer[WAV_DMA_BUFFER_SIZE/2], WAV_DMA_BUFFER_SIZE/2);
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
