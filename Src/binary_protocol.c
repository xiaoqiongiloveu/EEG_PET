#include "binary_protocol.h"

/* 全局协议模式标志，0: ASCII 文本模式, 1: Binary 二进制模式 */
volatile uint8_t g_use_binary_protocol = 0; // 0: ASCII, 1: Binary

// CRC16-CCITT (Poly 0x1021) 查表法使用的常量表
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/**
 * @brief  计算二进制数据的 CRC16-CCITT 校验和
 * @param  data: 指向需要计算校验和的数据缓冲区的指针
 * @param  len: 需要计算的数据长度（字节数）
 * @retval 计算得到的 16 位 CRC 校验值
 */
uint16_t BP_CRC16(uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

/**
 * @brief  初始化二进制协议解析器
 * @param  parser: 指向解析器状态结构体的指针
 * @param  cb: 解析出完整有效帧后触发的回调函数，包含参数 (类型, 序列号, 时间戳, 载荷指针, 载荷长度)
 */
void BP_Init(BP_Parser_t* parser, void (*cb)(uint8_t, uint16_t, uint32_t, uint8_t*, uint16_t))
{
    parser->state = BP_STATE_SYNC1;
    parser->frame_callback = cb;
    parser->bytes_read = 0;
}

/**
 * @brief  处理接收到的单个字节数据 (基于状态机的解析流)
 * @param  parser: 指向解析器状态结构体的指针
 * @param  byte: 接收到的单字节数据
 * @note   支持连续传入字节，内部维护解析状态，校验通过后自动调用注册的回调函数
 */
void BP_ProcessByte(BP_Parser_t* parser, uint8_t byte)
{
    // 简单的解析状态机
    switch (parser->state) {
        case BP_STATE_SYNC1:
            if (byte == 0x55) parser->state = BP_STATE_SYNC2; // 等待同步头, 0xAA55 小端模式下为 55 AA
            break;
            
        case BP_STATE_SYNC2:
            if (byte == 0xAA) parser->state = BP_STATE_LEN1;
            else parser->state = BP_STATE_SYNC1; // 同步失败，重置状态
            break;
            
        case BP_STATE_LEN1:
            parser->payload_len = byte;
            parser->state = BP_STATE_LEN2;
            break;
            
        case BP_STATE_LEN2:
            parser->payload_len |= (uint16_t)byte << 8;
            // 校验长度有效性
            if (parser->payload_len > BP_MAX_PAYLOAD_SIZE || parser->payload_len < 7) {
                // 长度无效，重置状态。
                // 最小长度 = 1(Type) + 2(Seq) + 4(TS) = 7 字节
                parser->state = BP_STATE_SYNC1;
            } else {
                parser->state = BP_STATE_PAYLOAD;
                parser->bytes_read = 0;
            }
            break;
            
        case BP_STATE_PAYLOAD:
            parser->buffer[parser->bytes_read++] = byte;
            if (parser->bytes_read >= parser->payload_len) {
                parser->state = BP_STATE_CRC1;
            }
            break;
            
        case BP_STATE_CRC1:
            parser->buffer[parser->bytes_read] = byte; // 存储 CRC 低字节 (为了计算方便，暂时存入 buffer 末尾)
            // 实际上 buffer 此时包含的是 Type+Seq+TS+Payload。
            // 我们将 CRC 追加到 buffer 中，以便一并处理。
            parser->buffer[parser->bytes_read++] = byte;
            parser->state = BP_STATE_CRC2;
            break;
            
        case BP_STATE_CRC2:
        {
            parser->buffer[parser->bytes_read++] = byte; // 存储 CRC 高字节
            
            // 校验 CRC
            // 当前 Buffer 包含: Type(1) + Seq(2) + TS(4) + Data(N) + CRC(2)
            // CRC 计算范围: Type...Data
            uint16_t data_len = parser->payload_len; // Type..Data 的总长度
            uint16_t crc_calc = BP_CRC16(parser->buffer, data_len);
            uint16_t crc_recv = parser->buffer[data_len] | (parser->buffer[data_len+1] << 8);
            
            if (crc_calc == crc_recv) {
                // 帧校验有效！提取各个字段
                uint8_t type = parser->buffer[0];
                uint16_t seq = parser->buffer[1] | (parser->buffer[2] << 8);
                uint32_t ts  = parser->buffer[3] | (parser->buffer[4] << 8) | (parser->buffer[5] << 16) | (parser->buffer[6] << 24);
                uint8_t* payload_ptr = &parser->buffer[7];
                uint16_t payload_size = data_len - 7;
                
                if (parser->frame_callback) {
                    parser->frame_callback(type, seq, ts, payload_ptr, payload_size);
                }
            }
            
            parser->state = BP_STATE_SYNC1; // 重置状态以接收下一帧
            break;
        }
            
        default:
            parser->state = BP_STATE_SYNC1;
            break;
    }
}

/**
 * @brief  将数据打包成符合协议的二进制帧
 * @param  dest: 用于存放打包后完整帧数据的目标缓冲区
 * @param  type: 数据包类型标志 (如 TYPE_EEG, TYPE_IMU 等)
 * @param  seq: 数据包序列号 (用于防丢包检测)
 * @param  ts: 硬件级时间戳 (毫秒级或微秒级)
 * @param  payload: 指向实际需要传输的业务数据 (载荷) 的指针
 * @param  len: 业务数据 (载荷) 的长度
 * @retval 打包后的完整帧长度 (字节数)，包含同步头、包头、载荷和 CRC
 */
uint16_t BP_Pack(uint8_t* dest, uint8_t type, uint16_t seq, uint32_t ts, uint8_t* payload, uint16_t len)
{
    // 完整帧结构: Sync(2) + Len(2) + Type(1) + Seq(2) + TS(4) + Payload(N) + CRC(2)
    // 长度字段 (Len field) 所指范围 = 1 + 2 + 4 + N = 7 + N
    
    uint16_t total_len = 0;
    uint16_t frame_len_field = 7 + len;
    
    // Header (帧头: 包含 Sync 与 Len)
    dest[0] = 0x55;
    dest[1] = 0xAA;
    dest[2] = frame_len_field & 0xFF;
    dest[3] = (frame_len_field >> 8) & 0xFF;
    
    // Body (参与 CRC 计算的主体部分: Type + Seq + TS)
    dest[4] = type;
    dest[5] = seq & 0xFF;
    dest[6] = (seq >> 8) & 0xFF;
    dest[7] = ts & 0xFF;
    dest[8] = (ts >> 8) & 0xFF;
    dest[9] = (ts >> 16) & 0xFF;
    dest[10] = (ts >> 24) & 0xFF;
    
    // Payload (载荷数据)
    if (len > 0 && payload) {
        memcpy(&dest[11], payload, len);
    }
    
    // CRC (对帧头以后的主体及载荷进行校验)
    uint16_t crc = BP_CRC16(&dest[4], frame_len_field);
    uint16_t crc_offset = 4 + frame_len_field;
    dest[crc_offset] = crc & 0xFF;
    dest[crc_offset+1] = (crc >> 8) & 0xFF;
    
    return crc_offset + 2; // 返回总写入的字节数
}
