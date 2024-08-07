/**
 ******************************************************************************
 * @file	bsp_dwt.h
 * @author  Peomcherry
 * @version V0.0.0
 * @date    2024/8/3
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#include "main.h"
#include "stdint.h"


#define DEM_CR *(voilatile u32 *)0xE000EDFC

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);
void DWT_SysTimeUpdate(void);

extern DWT_Time_t SysTime;

extern void delay_init(void);
extern void delay_us(uint16_t nus);
extern void delay_ms(uint16_t nms);

#endif /* BSP_DWT_H_ */
