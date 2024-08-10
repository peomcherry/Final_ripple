/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Peomcherry
 * @version V0.0.0
 * @date    2024/8/3
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "BSP_DWT.h"
//#include "bsp_delay.h"
#include "main.h"

DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
uint64_t CYCCNT64;
static void DWT_CNT_Update(void);
/**
 * @brief ��ʼ��DWT��Ԫ������CPUƵ��
 * @param CPU_Freq_mHz CPUƵ�ʣ���λΪMHz
 */
void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* ʹ��DWT���� */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT�Ĵ���������0 */
    DWT->CYCCNT = (uint32_t)0u; 

    /* ʹ��Cortex-M DWT CYCCNT�Ĵ��� */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;
}


// ���������ڻ�ȡʱ������ϵͳʱ�䡢��ȡʱ���ߵĺ������Լ�һ���ڲ�ʹ�õļ��������º���
// ÿ������������ϸ��ע�ͣ����������ǵ���;�͹���ԭ��

/**
 * @brief ��ȡ���ϴε��õ����ڵ�ʱ���룩
 * @param cnt_last ָ���ϴμ�����ָ��
 * @return ʱ����λΪ��
 */
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

/**
 * @brief ��ȡ���ϴε��õ����ڵ�ʱ���룬˫���ȣ�
 * @param cnt_last ָ���ϴμ�����ָ��
 * @return ʱ����λΪ�루˫���ȣ�
 */

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

/**
 * @brief ����ϵͳʱ��
 * @note  ����ʵ�ֺ������û�����ֱ�ӵ���
 */

void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

/**
 * @brief ��ȡ��ϵͳ���������ڵ���ʱ�䣨�룩
 * @return ��ʱ�䣬��λΪ��
 */

float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

/**
 * @brief ��ȡ��ϵͳ���������ڵ���ʱ�䣨���룩
 * @return ��ʱ�䣬��λΪ����
 */

float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

/**
 * @brief ��ȡ��ϵͳ���������ڵ���ʱ�䣨΢�룩
 * @return ��ʱ�䣬��λΪ΢��
 */

uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

/**
 * @brief ����CYCCNT�Ĵ�����ֵ���������
 */

static void DWT_CNT_Update(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;

    if (cnt_now < CYCCNT_LAST)
        CYCCNT_RountCount++;//���

    CYCCNT_LAST = cnt_now;
}

/**
 * @brief ����DWT����ʱ����
 * @param Delay ��ʱʱ�䣬��λΪ��
 */

void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
    {
    }
}




static uint8_t fac_us = 0;
static uint32_t fac_ms = 0;

void delay_init(void)
{
    fac_us = SystemCoreClock / 1000000;
    fac_ms = SystemCoreClock / 1000;

}

void delay_us(uint16_t nus)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nus * fac_us;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

void delay_ms(uint16_t nms)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nms * fac_ms;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}
