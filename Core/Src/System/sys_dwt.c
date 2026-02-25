/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_dwt.c
 *  Description  : Data Watch point and Trace (适配STM32H7/Cortex-M7)
 *  LastEditors  : Polaris
 *  Date         : 2023-02-10 13:25:11
 *  LastEditTime : 2024-0x-xx xx:xx:xx
 */

#include "sys_dwt.h"

// 全局DWT数据实例
static DWT_DataTypeDef DWTData = {0};

/**
 * @brief  DWT初始化（适配STM32H7）
 * @param  CPU_Freq_mHz: CPU主频，单位mHz（如480表示480MHz）
 * @retval None
 */
void DWT_Init(uint32_t CPU_Freq_mHz) {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();

    // 使能DWT模块（Cortex-M7内核）
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;                  // 重置CYCCNT计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 使能CYCCNT计数器

    // 初始化主频参数（STM32H7最大480MHz）
    dwt->CPU_FREQ_Hz = CPU_Freq_mHz * 1000000UL;
    dwt->CPU_FREQ_Hz_ms = dwt->CPU_FREQ_Hz / 1000UL;
    dwt->CPU_FREQ_Hz_us = dwt->CPU_FREQ_Hz / 1000000UL;
    
    // 初始化溢出计数和历史值
    dwt->CYCCNT_RountCount = 0UL;
    dwt->CYCCNT_LAST = DWT->CYCCNT;
    dwt->CYCCNT64 = 0ULL;
}

/**
 * @brief  获取DWT数据结构体指针
 * @retval DWT_DataTypeDef*: 数据指针
 */
DWT_DataTypeDef* DWT_GetDWTDataPtr() {
    return &DWTData;
}

/**
 * @brief  获取时间差（秒），并更新上次计数值
 * @param  cnt_last: 上次CYCCNT计数值指针
 * @retval float: 时间差（秒）
 */
float DWT_GetDeltaT(uint32_t *cnt_last) {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    volatile uint32_t cnt_now = DWT->CYCCNT;
    // 计算时间差（兼容溢出）
    float dt = (float)(cnt_now - *cnt_last) / (float)dwt->CPU_FREQ_Hz;
    *cnt_last = cnt_now; // 更新上次计数值

    DWT_CNT_Update();
    return dt;
}

/**
 * @brief  获取时间差（秒），不更新上次计数值
 * @param  cnt_last: 上次CYCCNT计数值指针
 * @retval float: 时间差（秒）
 */
float DWT_GetDeltaTWithOutUpdate(uint32_t *cnt_last) {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    volatile uint32_t cnt_now = DWT->CYCCNT;
    // 修复原代码重复强制转换+未读cnt_now的问题
    float dt = (float)(cnt_now - *cnt_last) / (float)dwt->CPU_FREQ_Hz;

    DWT_CNT_Update();
    return dt;
}

/**
 * @brief  获取高精度时间差（秒），并更新上次计数值
 * @param  cnt_last: 上次CYCCNT计数值指针
 * @retval double: 时间差（秒）
 */
double DWT_GetDeltaT64(uint32_t *cnt_last) {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    volatile uint32_t cnt_now = DWT->CYCCNT;
    // 高精度计算（适配H7高主频）
    double dt = (double)(cnt_now - *cnt_last) / (double)dwt->CPU_FREQ_Hz;
    *cnt_last = cnt_now; // 更新上次计数值

    DWT_CNT_Update();
    return dt;
}

/**
 * @brief  更新系统时间（秒/毫秒/微秒）
 * @retval None
 */
void DWT_SysTimeUpdate() {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    volatile uint32_t cnt_now = DWT->CYCCNT;

    // 更新CYCCNT溢出计数
    DWT_CNT_Update();

    // 计算64位CYCCNT（防32位溢出）
    dwt->CYCCNT64 = (uint64_t)dwt->CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    
    // 分解为秒/毫秒/微秒
    uint64_t total_us = dwt->CYCCNT64 / dwt->CPU_FREQ_Hz_us; // 总微秒数
    dwt->SysTime.s = (uint32_t)(total_us / 1000000UL);       // 秒
    dwt->SysTime.ms = (uint32_t)((total_us % 1000000UL) / 1000UL); // 毫秒
    dwt->SysTime.us = (uint16_t)(total_us % 1000UL);         // 微秒
    dwt->SysTime.ms_tick = dwt->SysTime.s * 1000UL + dwt->SysTime.ms; // 总毫秒数
}

/**
 * @brief  获取系统时间（秒）
 * @retval float: 系统时间（秒）
 */
float DWT_GetTimeline_s() {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    DWT_SysTimeUpdate();

    float timeline = (float)dwt->SysTime.s + 
                     (float)dwt->SysTime.ms * 0.001f + 
                     (float)dwt->SysTime.us * 0.000001f;
    return timeline;
}

/**
 * @brief  获取系统时间（毫秒）
 * @retval float: 系统时间（毫秒）
 */
float DWT_GetTimeline_ms() {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    DWT_SysTimeUpdate();

    float timeline = (float)dwt->SysTime.s * 1000.f + 
                     (float)dwt->SysTime.ms + 
                     (float)dwt->SysTime.us * 0.001f;
    return timeline;
}

/**
 * @brief  获取系统时间（微秒）
 * @retval uint64_t: 系统时间（微秒）
 */
uint64_t DWT_GetTimeline_us() {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    DWT_SysTimeUpdate();

    uint64_t timeline = (uint64_t)dwt->SysTime.s * 1000000ULL + 
                        (uint64_t)dwt->SysTime.ms * 1000ULL + 
                        (uint64_t)dwt->SysTime.us;
    return timeline;
}

/**
 * @brief  更新CYCCNT溢出计数（简化pending逻辑，提升H7可靠性）
 * @retval None
 */
void DWT_CNT_Update() {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    volatile uint32_t cnt_now = DWT->CYCCNT;

    // 检测CYCCNT溢出（32位计数器回绕）
    if (cnt_now < dwt->CYCCNT_LAST) {
        dwt->CYCCNT_RountCount++;
    }
    dwt->CYCCNT_LAST = cnt_now;
}

/**
 * @brief  精准延时（秒）
 * @param  Delay: 延时时间（秒）
 * @retval None
 */
void DWT_Delay(float Delay) {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    uint32_t tickstart = DWT->CYCCNT;
    uint32_t delay_cycles = (uint32_t)(Delay * (float)dwt->CPU_FREQ_Hz);

    while ((DWT->CYCCNT - tickstart) < delay_cycles) {
        // 空循环等待
    }
}

/**
 * @brief  精准延时（毫秒）
 * @param  Delay: 延时时间（毫秒）
 * @retval None
 */
void DWT_Delayms(float Delay) {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    uint32_t tickstart = DWT->CYCCNT;
    uint32_t delay_cycles = (uint32_t)(Delay * (float)dwt->CPU_FREQ_Hz_ms);

    while ((DWT->CYCCNT - tickstart) < delay_cycles) {
        // 空循环等待
    }
}

/**
 * @brief  精准延时（微秒）
 * @param  Delay: 延时时间（微秒）
 * @retval None
 */
void DWT_Delayus(float Delay) {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    uint32_t tickstart = DWT->CYCCNT;
    uint32_t delay_cycles = (uint32_t)(Delay * (float)dwt->CPU_FREQ_Hz_us);

    while ((DWT->CYCCNT - tickstart) < delay_cycles) {
        // 空循环等待
    }
}