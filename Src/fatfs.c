/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  使用 FatFs 中间件配置文件的代码。
  ******************************************************************************
  * @attention
  *
  * 版权所有 (c) 2026 STMicroelectronics。
  * 保留所有权利。
  *
  * 本软件根据随此软件包分发的 LICENSE 文件中的条款进行许可。
  * 如果未随附 LICENSE 文件，则本软件“按原样”提供。
  *
  ******************************************************************************
  */
/* 包含文件 ------------------------------------------------------------------*/
#include "fatfs.h"

uint8_t retSD;    /* 链接 SD 卡返回的返回值 */
char SDPath[4];   /* SD 卡逻辑驱动器路径 */
__attribute__((aligned(4))) FATFS SDFatFS;    /* SD 卡逻辑驱动器的文件系统工作区对象 */
__attribute__((aligned(4))) FIL SDFile;       /* SD 卡的文件对象 */

/* 用户代码 BEGIN Variables */

/* 用户代码 END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: 在这里链接磁盘 I/O 驱动 ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* 用户代码 BEGIN Init */
  /* 可在下方添加额外的用户初始化代码 */
  /* 用户代码 END Init */
}

/* 获取基于毫秒的时间 */
DWORD get_fattime(void)
{
  /* 用户代码 BEGIN get_fattime */
  return 0;
  /* 用户代码 END get_fattime */  
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
