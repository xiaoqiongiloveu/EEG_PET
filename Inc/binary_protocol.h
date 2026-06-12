#ifndef __BINARY_PROTOCOL_H
#define __BINARY_PROTOCOL_H

#include <stdint.h>
#include <string.h>

// --- Constants ---
#define BP_SYNC_WORD        0xAA55

// Frame Types
#define BP_TYPE_EEG         0x01
#define BP_TYPE_IMU         0x02
#define BP_TYPE_IMP         0x03
#define BP_TYPE_STATUS      0x04
#define BP_TYPE_AUX         0x05
#define BP_TYPE_CMD         0x10
#define BP_TYPE_CMD_RESP    0x11
#define BP_TYPE_OTA         0x20

// Command IDs
#define BP_CMD_START_STREAM   0x01
#define BP_CMD_STOP_STREAM    0x02
#define BP_CMD_SET_CONF       0x03
#define BP_CMD_GET_STATE      0x04
#define BP_CMD_LIST_FILES     0x05
#define BP_CMD_GET_FILE       0x06
#define BP_CMD_DELETE_FILE    0x07
#define BP_CMD_OTA_START      0x08
#define BP_CMD_OTA_APPLY      0x09
#define BP_CMD_PLAY_MUSIC     0x0A
#define BP_CMD_ENABLE_DEVICE  0x0B // Replaced BP_CMD_SINGLE with BP_CMD_ENABLE_DEVICE
#define BP_CMD_DISABLE_DEVICE 0x0C // Replaced BP_CMD_QUAD with BP_CMD_DISABLE_DEVICE
#define BP_CMD_AUTO_STREAM    0x0D
#define BP_CMD_TEST_IMP       0x0E // Replaced CHECK_LOSS with TEST_IMP
#define BP_CMD_AUDIO_ROUTE    0x0F
#define BP_CMD_READ_PA9       0x10
#define BP_CMD_CLEAR_DATA     0x11 // Auto clear SD card files
#define BP_CMD_PAUSE          0x12 // Pause online streaming
#define BP_CMD_RESUME         0x13 // Resume online streaming
#define BP_CMD_STOP_IMP       0x14 // Stop impedance test

// Response Status
#define BP_RESP_OK          0x00
#define BP_RESP_ERR         0x01

// Max Payload Size (Adjust based on USB packet size)
#define BP_MAX_PAYLOAD_SIZE 512
#define BP_HEADER_SIZE      9  // Sync(2)+Len(2)+Type(1)+Seq(2)+TS(4)
#define BP_CRC_SIZE         2
#define BP_MAX_FRAME_SIZE   (BP_HEADER_SIZE + BP_MAX_PAYLOAD_SIZE + BP_CRC_SIZE)

// --- Structs ---
#pragma pack(push, 1)
typedef struct {
    uint16_t sync;      // 0xAA55
    uint16_t length;    // Payload Length (Bytes after Length field?? No, let's match Python: Len of Payload only? 
                        // Wait, Python parser uses: total_frame_len = 2+2+payload_len_field+2
                        // So payload_len_field = Type(1) + Seq(2) + TS(4) + Data(N)
                        // This means length field = 7 + N
    uint8_t  type;
    uint16_t seq;
    uint32_t timestamp;
    uint8_t  payload[1]; // Flexible array member placeholder
} BP_Header_t;
#pragma pack(pop)

// Parser State Machine
typedef enum {
    BP_STATE_SYNC1,
    BP_STATE_SYNC2,
    BP_STATE_LEN1,
    BP_STATE_LEN2,
    BP_STATE_PAYLOAD,
    BP_STATE_CRC1,
    BP_STATE_CRC2
} BP_State_t;

typedef struct {
    BP_State_t state;
    uint16_t   payload_len; // Value from Length field (Type+Seq+TS+Data)
    uint16_t   bytes_read;  // Bytes read into buffer so far
    
    // Header fields storage during parsing
    uint8_t    temp_type;
    uint16_t   temp_seq;
    uint32_t   temp_ts;
    uint8_t    header_bytes_read; // For Seq/TS parsing
    
    uint8_t    buffer[BP_MAX_FRAME_SIZE]; // Reassembly buffer (Starting from Type)
    
    // Callback when a full valid frame is found
    void (*frame_callback)(uint8_t type, uint16_t seq, uint32_t ts, uint8_t* payload, uint16_t len);
} BP_Parser_t;

// --- API ---
void BP_Init(BP_Parser_t* parser, void (*cb)(uint8_t, uint16_t, uint32_t, uint8_t*, uint16_t));
void BP_ProcessByte(BP_Parser_t* parser, uint8_t byte);
uint16_t BP_Pack(uint8_t* dest, uint8_t type, uint16_t seq, uint32_t ts, uint8_t* payload, uint16_t len);
uint16_t BP_CRC16(uint8_t* data, uint16_t len);

extern volatile uint8_t g_use_binary_protocol;

#endif
