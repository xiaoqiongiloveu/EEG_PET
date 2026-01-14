#ifndef __QMI8658_ADAPTER_H
#define __QMI8658_ADAPTER_H

#include "main.h"
#include <stdio.h>
#include <string.h>

/****************************************************************************************************************/
/* QMI8658A Macro Definitions */

#define QMI8658_ADDR            0X6A                        /* SA0=1 Address=0X6A, SA0=0 Address=0X6B */        
#define ONE_G                   9.807f                      /* Gravity acceleration unit conversion */
#define M_PI                    (3.14159265358979323846f)   /* Pi for unit conversion */
#define MAX_CALI_COUNT          100                         /* Calibration count */

#define QMI8658_DISABLE_ALL     (0x0)                       /* Disable all sensors */
#define QMI8658_ACC_ENABLE      (0x1)                       /* Enable Accelerometer */
#define QMI8658_GYR_ENABLE      (0x2)                       /* Enable Gyroscope */
#define QMI8658_ACCGYR_ENABLE   (QMI8658_ACC_ENABLE | QMI8658_GYR_ENABLE)/* Enable both */

extern uint8_t g_imu_init;      

/****************************************************************************************************************/
/* QMI8658A Register Map */
enum Qmi8658AReg
{
    Register_WhoAmI = 0,
    Register_Revision,
    Register_Ctrl1,
    Register_Ctrl2,
    Register_Ctrl3,
    Register_Reserved,
    Register_Ctrl5,
    Register_Reserved1,
    Register_Ctrl7,
    Register_Ctrl8,
    Register_Ctrl9,
    Register_Cal1_L = 11,
    Register_Cal1_H,
    Register_Cal2_L,
    Register_Cal2_H,
    Register_Cal3_L,
    Register_Cal3_H,
    Register_Cal4_L,
    Register_Cal4_H,
    Register_FifoWmkTh = 19, 
    Register_FifoCtrl = 20,
    Register_FifoCount = 21,
    Register_FifoStatus = 22,
    Register_FifoData = 23,
    Register_StatusInt = 45,
    Register_Status0,
    Register_Status1,
    Register_Timestamp_L = 48,
    Register_Timestamp_M,
    Register_Timestamp_H,
    Register_Tempearture_L = 51,
    Register_Tempearture_H,
    Register_Ax_L = 53,
    Register_Ax_H,
    Register_Ay_L,
    Register_Ay_H,
    Register_Az_L,
    Register_Az_H,
    Register_Gx_L = 59,
    Register_Gx_H,
    Register_Gy_L,
    Register_Gy_H,
    Register_Gz_L,
    Register_Gz_H,
    Register_COD_Status = 70,
    Register_dQW_L = 73,
    Register_dQW_H,
    Register_dQX_L,
    Register_dQX_H,
    Register_dQY_L,
    Register_dQY_H,
    Register_dQZ_L,
    Register_dQZ_H,
    Register_dVX_L,
    Register_dVX_H,
    Register_dVY_L,
    Register_dVY_H,
    Register_dVZ_L,
    Register_dVZ_H,
    Register_TAP_Status = 89,
    Register_Step_Cnt_L = 90,
    Register_Step_Cnt_M = 91,
    Register_Step_Cnt_H = 92,
    Register_Reset = 96
};

/****************************************************************************************************************/
/* Ctrl9 Command Definitions */
enum Ctrl9Command
{
    Ctrl9_Cmd_Ack                   = 0X00,
    Ctrl9_Cmd_RstFifo               = 0X04,
    Ctrl9_Cmd_ReqFifo               = 0X05, /* Get FIFO data from Device */
    Ctrl9_Cmd_WoM_Setting           = 0x08, /* WoM setting */
    Ctrl9_Cmd_AccelHostDeltaOffset  = 0x09, /* Accel Host Delta Offset */
    Ctrl9_Cmd_GyroHostDeltaOffset   = 0x0A, /* Gyro Host Delta Offset */
    Ctrl9_Cmd_CfgTap                = 0x0C, /* Config Tap */
    Ctrl9_Cmd_CfgPedometer          = 0x0D, /* Config Pedometer */
    Ctrl9_Cmd_Motion                = 0x0E, /* Motion */
    Ctrl9_Cmd_RstPedometer          = 0x0F, /* Reset Pedometer */
    Ctrl9_Cmd_CopyUsid              = 0x10, /* Copy USID */
    Ctrl9_Cmd_SetRpu                = 0x11, /* Set RPU */
    Ctrl9_Cmd_AHBClockGating        = 0x12, /* AHB Clock Gating */
    Ctrl9_Cmd_OnDemandCalivration   = 0xA2, /* On Demand Calibration */
    Ctrl9_Cmd_ApplyGyroGains        = 0xAA  /* Apply Gyro Gains */
};

/* Low Pass Filter Config */
enum qmi8658_LpfConfig
{
    Qmi8658Lpf_Disable,
    Qmi8658Lpf_Enable
};

/* Self Test Config */
enum qmi8658_StConfig
{
    Qmi8658St_Disable,
    Qmi8658St_Enable
};

/* LPF Mode Selection */
enum qmi8658_LpfMode
{
    A_LSP_MODE_0 = 0x00 << 1,
    A_LSP_MODE_1 = 0x01 << 1,
    A_LSP_MODE_2 = 0x02 << 1,
    A_LSP_MODE_3 = 0x03 << 1,

    G_LSP_MODE_0 = 0x00 << 5,
    G_LSP_MODE_1 = 0x01 << 5,
    G_LSP_MODE_2 = 0x02 << 5,
    G_LSP_MODE_3 = 0x03 << 5
};

/* Accelerometer Range */
enum qmi8658_accrange
{
    Qmi8658accrange_2g = 0x00 << 4,
    Qmi8658accrange_4g = 0x01 << 4,
    Qmi8658accrange_8g = 0x02 << 4,
    Qmi8658accrange_16g = 0x03 << 4
};

/* Accelerometer ODR */
enum qmi8658_accodr
{
    Qmi8658accodr_8000Hz = 0x00,
    Qmi8658accodr_4000Hz = 0x01,
    Qmi8658accodr_2000Hz = 0x02,
    Qmi8658accodr_1000Hz = 0x03,
    Qmi8658accodr_500Hz = 0x04,
    Qmi8658accodr_250Hz = 0x05,
    Qmi8658accodr_125Hz = 0x06,
    Qmi8658accodr_62_5Hz = 0x07,
    Qmi8658accodr_31_25Hz = 0x08,
    Qmi8658accodr_LowPower_128Hz = 0x0c,
    Qmi8658accodr_LowPower_21Hz = 0x0d,
    Qmi8658accodr_LowPower_11Hz = 0x0e,
    Qmi8658accodr_LowPower_3Hz = 0x0f
};

/* Gyroscope Range */
enum qmi8658_gyrrange
{
    Qmi8658gyrrange_16dps = 0 << 4,
    Qmi8658gyrrange_32dps = 1 << 4,
    Qmi8658gyrrange_64dps = 2 << 4,
    Qmi8658gyrrange_128dps = 3 << 4,
    Qmi8658gyrrange_256dps = 4 << 4,
    Qmi8658gyrrange_512dps = 5 << 4,
    Qmi8658gyrrange_1024dps = 6 << 4,
    Qmi8658gyrrange_2048dps = 7 << 4
};

/* Gyroscope ODR */
enum qmi8658_gyrodr
{
    Qmi8658gyrodr_8000Hz = 0x00,
    Qmi8658gyrodr_4000Hz = 0x01,
    Qmi8658gyrodr_2000Hz = 0x02,
    Qmi8658gyrodr_1000Hz = 0x03,
    Qmi8658gyrodr_500Hz = 0x04,
    Qmi8658gyrodr_250Hz = 0x05,
    Qmi8658gyrodr_125Hz = 0x06,
    Qmi8658gyrodr_62_5Hz = 0x07,
    Qmi8658gyrodr_31_25Hz = 0x08
};

typedef struct
{
    unsigned char ensensors;        /* Enable Sensors Flag */
    enum qmi8658_accrange accrange; /* Accelerometer Range */
    enum qmi8658_accodr accodr;     /* Accelerometer ODR */     
    enum qmi8658_gyrrange gyrrange; /* Gyroscope Range */
    enum qmi8658_gyrodr gyrodr;     /* Gyroscope ODR */     
    unsigned char syncsample;       /* Sync Sample Flag */
} qmi8658_config;

typedef struct
{
    qmi8658_config cfg;             /* Config Structure */
    unsigned short ssvt_a;          /* Acc Sensitivity */
    unsigned short ssvt_g;          /* Gyro Sensitivity */
    float imu[6];                   /* IMU Data (acc x,y,z, gyro x,y,z) */
} qmi8658_state;

/****************************************************************************************************************/
/* Functions */
// Adapting function names to match what main.c likely expects (QMI8658_Init) 
// but implementing them using the reference code's logic.
uint8_t QMI8658_Init(void);                                         /* Initialize QMI8658 */
float QMI8658_Get_Temperature(void);                                /* Get Temperature */
void QMI8658_Reset(void);                                           /* Reset */
void QMI8658_Read_XYZ(float *acc, float *gyro);                     /* Read XYZ Data */

// Helper functions that might be useful to expose
void QMI8658_Config_Reg(unsigned char low_power);

#endif
