#include "delay.h"
#include "main.h"

static uint32_t fac_us = 0;  
static uint32_t fac_ms = 0;  

/**
  * @brief  
  * @note   
  */
void Delay_Init(void)
{
    
    fac_us = SystemCoreClock / 1000000;
    
  
    fac_ms = (uint32_t)fac_us * 1000;
    
    
    HAL_SYSTICK_Config(SystemCoreClock / 1000);  
}

/**
  * @brief 
  * @param 
  */
void delay_us(uint32_t us)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;  
    
    ticks = us * fac_us;  
    told = SysTick->VAL;  
    
    while(1)
    {
        tnow = SysTick->VAL;
        if(tnow != told)
        {
            if(tnow < told)
            {
                tcnt += told - tnow;  
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if(tcnt >= ticks)
            {
                break;  
            }
        }
    }
}

/**
  * @brief  
  * @param  
  */
void delay_ms(uint32_t ms)
{
    uint32_t i;
    for(i = 0; i < ms; i++)
    {
        delay_us(1000);  
    }
}

/**
  * @brief  
  * @param  
  */
void delay_s(uint32_t s)
{
    uint32_t i;
    for(i = 0; i < s; i++)
    {
        delay_ms(1000);  
    }
}

/**
  * @brief  
  */
void HAL_Delay(uint32_t Delay)
{
    delay_ms(Delay);
}
