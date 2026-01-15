#ifndef __WAVPLAY_H
#define __WAVPLAY_H

#include "main.h"
#include "fatfs.h"

// WAV TX DMA Buffer Size (Increase for high bitrate/sample rate)
// For 48kHz/16bit/Stereo: 48000 * 2 * 2 = 192000 bytes/sec
// Buffer needs to hold enough data to prevent underrun
// Increased to 16KB to allow ~46ms per half-buffer process time (at 44.1kHz)
#define WAV_DMA_BUFFER_SIZE    4096

// RIFF Chunk
typedef __packed struct
{
    uint32_t ChunkID;       // "RIFF" = 0X46464952
    uint32_t ChunkSize;     // File Size - 8
    uint32_t Format;        // "WAVE" = 0X45564157
} ChunkRIFF;

// fmt Chunk
typedef __packed struct
{
    uint32_t ChunkID;       // "fmt " = 0X20746D66
    uint32_t ChunkSize;     // Chunk Size (16 for PCM)
    uint16_t AudioFormat;   // 1 = PCM
    uint16_t NumOfChannels; // 1 = Mono, 2 = Stereo
    uint32_t SampleRate;    // e.g., 44100
    uint32_t ByteRate;      // SampleRate * NumChannels * BitsPerSample / 8
    uint16_t BlockAlign;    // NumChannels * BitsPerSample / 8
    uint16_t BitsPerSample; // 16, 24, 32
} ChunkFMT;

// fact Chunk (Optional)
typedef __packed struct 
{
    uint32_t ChunkID;       // "fact" = 0X74636166
    uint32_t ChunkSize;     // 4
    uint32_t NumOfSamples;  
} ChunkFACT;

// LIST Chunk (Optional)
typedef __packed struct 
{
    uint32_t ChunkID;       // "LIST" = 0X74636166
    uint32_t ChunkSize;     
} ChunkLIST;

// data Chunk
typedef __packed struct 
{
    uint32_t ChunkID;       // "data" = 0X61746164
    uint32_t ChunkSize;     // Data Size
} ChunkDATA;

// WAV Control Structure
typedef struct
{ 
    uint16_t audioformat;   // Audio Format
    uint16_t nchannels;     // Channels
    uint16_t blockalign;    // Block Align
    uint32_t datasize;      // Data Size
    uint32_t bitrate;       // Bitrate
    uint32_t samplerate;    // Sample Rate
    uint16_t bps;           // Bits Per Sample
    uint32_t datastart;     // Data Start Offset
} __wavctrl; 

// Functions
uint8_t Wav_Play(const char* fname);
void Wav_Stop(void);

#endif
