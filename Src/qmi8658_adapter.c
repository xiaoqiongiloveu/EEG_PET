#include "qmi8658_adapter.h"
#include "soft_i2c.h"
#include "delay.h"
#include "math.h"

/* Global Variables */
uint8_t g_imu_init = 0;
qmi8658_state g_imu;

/* Helper for Printing */
#define printf(...) 

/**
 * @brief   Write a byte to QMI8658 register
 * @param   reg: Register address
 * @param   data: Data to write
 * @retval  0: Success, 1: Failure
 */
static uint8_t QMI8658_Write_Byte(uint8_t reg, uint8_t data)
{
    Soft_I2C_Start();
    Soft_I2C_SendByte((QMI8658_ADDR << 1) | 0x00);
    if (Soft_I2C_WaitAck() != 0)
    {
        Soft_I2C_Stop();
        return 1;
    }
    Soft_I2C_SendByte(reg);
    if (Soft_I2C_WaitAck() != 0)
    {
        Soft_I2C_Stop();
        return 1;
    }
    Soft_I2C_SendByte(data);
    if (Soft_I2C_WaitAck() != 0)
    {
        Soft_I2C_Stop();
        return 1;
    }
    Soft_I2C_Stop();
    
    return 0;
}

/**
 * @brief   Read a byte from QMI8658 register
 * @param   reg: Register address
 * @retval  Read value
 */
static uint8_t QMI8658_Read_Byte(uint8_t reg)
{
    uint8_t temp = 0;
    
    Soft_I2C_Start();
    Soft_I2C_SendByte((QMI8658_ADDR << 1) | 0x00);
    if (Soft_I2C_WaitAck() != 0)
    {
        Soft_I2C_Stop();
        return 0; // Return 0 on error
    }
    Soft_I2C_SendByte(reg);
    if (Soft_I2C_WaitAck() != 0)
    {
        Soft_I2C_Stop();
        return 0;
    }
    
    Soft_I2C_Start();
    Soft_I2C_SendByte((QMI8658_ADDR << 1) | 0x01);
    if (Soft_I2C_WaitAck() != 0)
    {
        Soft_I2C_Stop();
        return 0;
    }
    // Reference: iic_read_byte(0) -> NACK
    // Target: Soft_I2C_ReadByte(1) -> NACK (1=NACK in Target)
    temp = Soft_I2C_ReadByte(1); 
    Soft_I2C_Stop();
    
    return temp;
}

/**
 * @brief   Read N bytes from QMI8658
 * @param   reg: Register address
 * @param   buf: Buffer to store data
 * @param   len: Length to read
 * @retval  0: Success, 1: Failure
 */
static int QMI8658_Read_NBytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    
    Soft_I2C_Start();
    Soft_I2C_SendByte((QMI8658_ADDR << 1) | 0x00);
    if (Soft_I2C_WaitAck() != 0)
    {
        Soft_I2C_Stop();
        return 1;
    }
    Soft_I2C_SendByte(reg);
    if (Soft_I2C_WaitAck() != 0)
    {
        Soft_I2C_Stop();
        return 1;
    }
    
    Soft_I2C_Start();
    Soft_I2C_SendByte((QMI8658_ADDR << 1) | 0x01);
    if (Soft_I2C_WaitAck() != 0)
    {
        Soft_I2C_Stop();
        return 1;
    }
    for (i = 0; i < len; i++)
    {
        // Reference: iic_read_byte((i == (len - 1)) ? 0 : 1); 
        // Ref: 0=NACK, 1=ACK.
        // Target: Soft_I2C_ReadByte(ack);
        // Target: 1=NACK, 0=ACK.
        // So we need to INVERT the logic.
        // If last byte (i == len-1), we want NACK. Ref sends 0. Target should send 1.
        // If not last byte, we want ACK. Ref sends 1. Target should send 0.
        
        if (i == (len - 1)) {
            buf[i] = Soft_I2C_ReadByte(1); // NACK
        } else {
            buf[i] = Soft_I2C_ReadByte(0); // ACK
        }
    }
    Soft_I2C_Stop();
    
    return 0;
}

/**
 * @brief   Check Device ID
 * @retval  0: Success, 1: Failure
 */
static uint8_t QMI8658_Check_WhoAmI(void)
{
    uint8_t qmi8658_chip_id = 0;
    uint8_t qmi8658_revision_id = 0;
    uint8_t i = 0;
    
    while((qmi8658_chip_id != 0x05) && (i < 5))
    {
        qmi8658_chip_id = QMI8658_Read_Byte(Register_WhoAmI);
        if(qmi8658_chip_id == 0x05)
        {
            qmi8658_revision_id = QMI8658_Read_Byte(Register_Revision);
            break;
        }
        i++;
    }
    if((qmi8658_chip_id == 0x05) && (qmi8658_revision_id == 0x7c))
    {
        printf("QMI8658 Found! Chip ID: %#x, Revision: %#x\r\n", qmi8658_chip_id, qmi8658_revision_id);
        return 0;
    }
    else
    {
        return 1;
    }
}

/**
 * @brief   Sensor Calibration
 * @retval  0: Success, 1: Failure
 */
static uint8_t QMI8658_Calibration(void)
{
    uint8_t sta = 0;
    QMI8658_Write_Byte(Register_Ctrl7, 0x00);   /* Disable Gyro/Acc */
    QMI8658_Write_Byte(Register_Ctrl9, 0xA2);   /* On Demand Calibration */
    delay_ms(2000); // Wait for calibration
    sta = QMI8658_Read_Byte(Register_COD_Status);
    if(sta == 0x00)
    {
        return 0;
    } else return 1;
}

/**
 * @brief   Reset Sensor
 */
void QMI8658_Reset(void)
{
    QMI8658_Write_Byte(Register_Reset, 0xB0);
    delay_ms(150);
}

/**
 * @brief   Send Command to Ctrl9
 */
static int QMI8658_Send_Ctl9Cmd(enum Ctrl9Command cmd)
{
    unsigned char status1 = 0x00;
    unsigned short count = 0;
    unsigned char status_reg = Register_StatusInt;
    unsigned char cmd_done = 0x80;
    unsigned char retry = 0;
    int ret1 = 0;
    int ret2 = 0;

#if defined(QMI8658_SYNC_SAMPLE_MODE)
    if (g_imu.cfg.syncsample == 1)
    {
        status_reg = Register_Status1;
        cmd_done = 0x01;
    }
#endif
    while(retry++ < 3)
    {
        QMI8658_Write_Byte(Register_Ctrl9, (unsigned char)cmd);

        QMI8658_Read_NBytes(status_reg, &status1, 1);
        while (((status1 & cmd_done) != cmd_done) && (count++ < 100))
        {
            delay_ms(1);
            QMI8658_Read_NBytes(status_reg, &status1, 1);
        }
        if(count < 100) ret1 = 1; else ret1 = 0;

        QMI8658_Write_Byte(Register_Ctrl9, Ctrl9_Cmd_Ack);
        count = 0;
        QMI8658_Read_NBytes(status_reg, &status1, 1);
        while (((status1 & cmd_done) == cmd_done) && (count++ < 100))
        {
            delay_ms(1);    
            QMI8658_Read_NBytes(status_reg, &status1, 1);
        }
        if(count < 100) ret2 = 1; else ret2 = 0;

        if((ret1==0) || (ret2==0)) continue; else break;
    }

    if(ret1 && ret2) return 0;
    else
    {
        printf("QMI8658_Send_Ctl9Cmd fail cmd=%d\n", cmd);
        return 1;
    }
}

/**
 * @brief   Config Accelerometer
 */
static void QMI8658_Config_Acc(enum qmi8658_accrange range, enum qmi8658_accodr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
    unsigned char ctl_dada;

    switch (range)
    {
        case Qmi8658accrange_2g:  g_imu.ssvt_a = (1 << 14); break;
        case Qmi8658accrange_4g:  g_imu.ssvt_a = (1 << 13); break;
        case Qmi8658accrange_8g:  g_imu.ssvt_a = (1 << 12); break;
        case Qmi8658accrange_16g: g_imu.ssvt_a = (1 << 11); break;
        default: range = Qmi8658accrange_8g; g_imu.ssvt_a = (1 << 12); break;
    }
    if (stEnable == Qmi8658St_Enable)
    {
        ctl_dada = (unsigned char)range | (unsigned char)odr | 0x80;
    } 
    else
    {
        ctl_dada = (unsigned char)range | (unsigned char)odr;
    }
    QMI8658_Write_Byte(Register_Ctrl2, ctl_dada);
    
    QMI8658_Read_NBytes(Register_Ctrl5, &ctl_dada, 1);
    ctl_dada &= 0xf0;
    if (lpfEnable == Qmi8658Lpf_Enable)
    {
        ctl_dada |= A_LSP_MODE_3;
        ctl_dada |= 0x01;
    }
    else
    {
        ctl_dada &= ~0x01;
    }
    QMI8658_Write_Byte(Register_Ctrl5, ctl_dada);
}

/**
 * @brief   Config Gyroscope
 */
static void QMI8658_Config_Gyro(enum qmi8658_gyrrange range, enum qmi8658_gyrodr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
    unsigned char ctl_dada;

    switch (range)
    {
        case Qmi8658gyrrange_16dps:   g_imu.ssvt_g = 2048; break;
        case Qmi8658gyrrange_32dps:   g_imu.ssvt_g = 1024; break;
        case Qmi8658gyrrange_64dps:   g_imu.ssvt_g = 512; break;
        case Qmi8658gyrrange_128dps:  g_imu.ssvt_g = 256; break;
        case Qmi8658gyrrange_256dps:  g_imu.ssvt_g = 128; break;
        case Qmi8658gyrrange_512dps:  g_imu.ssvt_g = 64; break;
        case Qmi8658gyrrange_1024dps: g_imu.ssvt_g = 32; break;
        case Qmi8658gyrrange_2048dps: g_imu.ssvt_g = 16; break;
        default: range = Qmi8658gyrrange_512dps; g_imu.ssvt_g = 64; break;
    }

    if (stEnable == Qmi8658St_Enable)
    {
        ctl_dada = (unsigned char)range | (unsigned char)odr | 0x80;
    }   
    else
    {
        ctl_dada = (unsigned char)range | (unsigned char)odr;
    }
    QMI8658_Write_Byte(Register_Ctrl3, ctl_dada);

    QMI8658_Read_NBytes(Register_Ctrl5, &ctl_dada, 1);
    ctl_dada &= 0x0f;
    if (lpfEnable == Qmi8658Lpf_Enable)
    {
        ctl_dada |= G_LSP_MODE_3;
        ctl_dada |= 0x10;
    }
    else
    {
        ctl_dada &= ~0x10;
    }
    QMI8658_Write_Byte(Register_Ctrl5, ctl_dada);
}

/**
 * @brief   Enable AHB Clock
 */
static void QMI8658_Enable_Ahb_Clock(int enable)
{
    if (enable)
    {
        QMI8658_Write_Byte(Register_Ctrl7, 0x00);
        QMI8658_Write_Byte(Register_Cal1_L, 0x00);
        QMI8658_Send_Ctl9Cmd(Ctrl9_Cmd_AHBClockGating);
    }
    else
    {
        QMI8658_Write_Byte(Register_Ctrl7, 0x00);
        QMI8658_Write_Byte(Register_Cal1_L, 0x01);
        QMI8658_Send_Ctl9Cmd(Ctrl9_Cmd_AHBClockGating);
    }
}

/**
 * @brief   Enable Sensors
 */
static void QMI8658_EnableSensors(unsigned char enableFlags)
{
#if defined(QMI8658_SYNC_SAMPLE_MODE)
    g_imu.cfg.syncsample = 1;
    QMI8658_Enable_Ahb_Clock(0);
    QMI8658_Write_Byte(Register_Ctrl7, enableFlags | 0x80);
#else
    QMI8658_Write_Byte(Register_Ctrl7, enableFlags);    
#endif
    g_imu.cfg.ensensors = enableFlags & 0x03;

    delay_ms(2);
}

/**
 * @brief   Config Registers (Public)
 */
void QMI8658_Config_Reg(unsigned char low_power)
{
    QMI8658_EnableSensors(QMI8658_DISABLE_ALL);
    if (low_power)
    {
        g_imu.cfg.ensensors = QMI8658_ACC_ENABLE;           
        g_imu.cfg.accrange = Qmi8658accrange_8g;
        g_imu.cfg.accodr = Qmi8658accodr_LowPower_21Hz;
        g_imu.cfg.gyrrange = Qmi8658gyrrange_1024dps;
        g_imu.cfg.gyrodr = Qmi8658gyrodr_250Hz;
    }
    else
    {
        g_imu.cfg.ensensors = QMI8658_ACCGYR_ENABLE;    
        g_imu.cfg.accrange = Qmi8658accrange_16g;       
        g_imu.cfg.accodr = Qmi8658accodr_500Hz;         
        g_imu.cfg.gyrrange = Qmi8658gyrrange_128dps;    
        g_imu.cfg.gyrodr = Qmi8658gyrodr_500Hz;         
    }

    if (g_imu.cfg.ensensors & QMI8658_ACC_ENABLE)
    {
        QMI8658_Config_Acc(g_imu.cfg.accrange, g_imu.cfg.accodr, Qmi8658Lpf_Enable, Qmi8658St_Enable);
    }
    if (g_imu.cfg.ensensors & QMI8658_GYR_ENABLE)
    {
        QMI8658_Config_Gyro(g_imu.cfg.gyrrange, g_imu.cfg.gyrodr, Qmi8658Lpf_Enable, Qmi8658St_Enable);
    }
}

/**
 * @brief   Get Temperature (Public)
 */
float QMI8658_Get_Temperature(void)
{
    short temp = 0;
    float temp_f = 0;
    uint8_t buf[2];

    QMI8658_Read_NBytes(Register_Tempearture_L, buf, 2);    
    temp = ((short)buf[1] << 8) | buf[0];
    temp_f = (float)temp / 256.0f;
    return temp_f;
}

/**
 * @brief   Read Sensor Data (Internal Helper)
 */
static void QMI8658_Read_Sensor_Data(float *acc, float *gyro)
{
    unsigned char buf_reg[12];
    short raw_acc_xyz[3];
    short raw_gyro_xyz[3];
    unsigned char axis = 0;
    static int cali_count = 0;
    static float offset_acc[3] = {0.0, 0.0, 0.0};
    static float offset_gyro[3] = {0.0, 0.0, 0.0};
    static float accel_calibration_sum[3] = {0.0f, 0.0f, 0.0f};
    static float gyro_calibration_sum[3] = {0.0f, 0.0f, 0.0f};

    float acc_raw[3];
    float gyro_raw[3];

    QMI8658_Read_NBytes(Register_Ax_L, buf_reg, 12);
    
    raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));

    raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[7] << 8) | (buf_reg[6]));
    raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[9] << 8) | (buf_reg[8]));
    raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[11] << 8) | (buf_reg[10]));

    acc_raw[0] = (float)(raw_acc_xyz[0] * ONE_G) / g_imu.ssvt_a;
    acc_raw[1] = (float)(raw_acc_xyz[1] * ONE_G) / g_imu.ssvt_a;
    acc_raw[2] = (float)(raw_acc_xyz[2] * ONE_G) / g_imu.ssvt_a;

    gyro_raw[0] = (float)(raw_gyro_xyz[0] * M_PI) / (g_imu.ssvt_g * 180);
    gyro_raw[1] = (float)(raw_gyro_xyz[1] * M_PI) / (g_imu.ssvt_g * 180);
    gyro_raw[2] = (float)(raw_gyro_xyz[2] * M_PI) / (g_imu.ssvt_g * 180);

    if (g_imu_init != 1)
    {
        if (cali_count == 0)
        {
            memset((void *)accel_calibration_sum, 0, sizeof(accel_calibration_sum));
            memset((void *)gyro_calibration_sum, 0, sizeof(gyro_calibration_sum));
            cali_count++;
        }
        else if (cali_count < MAX_CALI_COUNT)
        {
            for (axis = 0; axis < 3; axis++)
            {
                if (axis == 2)
                    accel_calibration_sum[axis] += (acc_raw[axis] - ONE_G);
                else
                    accel_calibration_sum[axis] += acc_raw[axis];
                
                gyro_calibration_sum[axis] += gyro_raw[axis];
            }
            cali_count++;
        }
        else if (cali_count == MAX_CALI_COUNT)
        {
            for (axis = 0; axis < 3; axis++)
            {
                offset_gyro[axis] = (0.0f - (gyro_calibration_sum[axis] / (MAX_CALI_COUNT - 1)));
                offset_acc[axis] = (0.0f - (accel_calibration_sum[axis] / (MAX_CALI_COUNT - 1)));
            }
            g_imu_init = 1;
            cali_count++;
        }
    }
    else
    {
        for (axis = 0; axis < 3; axis++)
        {
            acc[axis] = acc_raw[axis] + offset_acc[axis];
            gyro[axis] = gyro_raw[axis] + offset_gyro[axis];
        }
    }
}

/**
 * @brief   Read XYZ (Public)
 */
void QMI8658_Read_XYZ(float *acc, float *gyro)
{
    unsigned char status = 0;
    unsigned char data_ready = 0;
    int retry = 0;

    while (retry++ < 3)
    {
#if defined(QMI8658_SYNC_SAMPLE_MODE)
        QMI8658_Read_NBytes(Register_StatusInt, &status, 1);
        if (status & 0x01) delay_us(12);
        if ((status & 0x01) || (status & 0x03))
        {
            data_ready = 1;
            break;
        }
#else
        QMI8658_Read_NBytes(Register_Status0, &status, 1);
        if (status & 0x03)
        {
            data_ready = 1;
            break;
        }
#endif
    }
    if (data_ready)
    {
        QMI8658_Read_Sensor_Data(acc, gyro);

        g_imu.imu[0] = acc[0];
        g_imu.imu[1] = acc[1];
        g_imu.imu[2] = acc[2];
        g_imu.imu[3] = gyro[0];
        g_imu.imu[4] = gyro[1];
        g_imu.imu[5] = gyro[2];
    }
    else
    {
        acc[0] = g_imu.imu[0];
        acc[1] = g_imu.imu[1];
        acc[2] = g_imu.imu[2];
        gyro[0] = g_imu.imu[3];
        gyro[1] = g_imu.imu[4];
        gyro[2] = g_imu.imu[5];
    }
}

/**
 * @brief   Initialize QMI8658 (Public)
 * @retval  0: Success, 1: Failure
 */
uint8_t QMI8658_Init(void)
{
    Soft_I2C_Init();

    QMI8658_Reset();
    
    if(QMI8658_Check_WhoAmI())
    {
        return 1;
    }
    if(QMI8658_Calibration())
    {
        return 1;
    }
    printf("QMI8658 Calibration Success!\r\n");

    QMI8658_Write_Byte(Register_Ctrl1, 0x60);   /* I2C Address Auto-Inc */
    QMI8658_Write_Byte(Register_Ctrl7, 0x00);   /* Disable Sensors */
    
    QMI8658_Config_Reg(0);                      /* Configure Sensors */
    QMI8658_EnableSensors(g_imu.cfg.ensensors); /* Enable Sensors */

    return 0;
}
