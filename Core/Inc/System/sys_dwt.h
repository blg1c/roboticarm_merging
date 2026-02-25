/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_dwt.h
 *  Description  : Data Watch point and Trace 
 *  LastEditors  : Polaris
 *  Date         : 2023-02-10 13:25:31
 *  LastEditTime : 2024-0x-xx xx:xx:xx
 */

#ifndef DWT_LIB_H
#define DWT_LIB_H

#ifdef __cplusplus
extern "C" {
#endif


//#include "core_cm7.h"
#include "main.h"

// 寄存器宏定义兼容（防止CMSIS库未定义）
#ifndef CoreDebug_DEMCR_TRCENA_Msk
#define CoreDebug_DEMCR_TRCENA_Msk    (1UL << 24)
#endif

#ifndef DWT_CTRL_CYCCNTENA_Msk
#define DWT_CTRL_CYCCNTENA_Msk        (1UL << 0)
#endif

// 时间类型定义
typedef struct {
    uint32_t s;          // 秒
    uint32_t ms;         // 毫秒
    uint16_t us;         // 微秒
    uint32_t ms_tick;    // 总毫秒数
} DWT_TimeTypeDef;

// DWT核心数据结构
typedef struct {
    DWT_TimeTypeDef SysTime;          // 系统时间
    uint32_t CPU_FREQ_Hz;             // CPU主频(Hz)
    uint32_t CPU_FREQ_Hz_ms;          // CPU主频(Hz/ms)
    uint32_t CPU_FREQ_Hz_us;          // CPU主频(Hz/us)
    uint32_t CYCCNT_RountCount;       // CYCCNT溢出次数
    uint32_t CYCCNT_LAST;             // 上一次CYCCNT值
    uint64_t CYCCNT64;                // 64位CYCCNT（防溢出）
} DWT_DataTypeDef;

// 函数声明
void DWT_Init(uint32_t CPU_Freq_mHz);                  // 初始化（入参：CPU主频，单位mHz，如480表示480MHz）
DWT_DataTypeDef* DWT_GetDWTDataPtr(void);              // 获取DWT数据指针
float DWT_GetDeltaT(uint32_t *cnt_last);               // 获取时间差（秒，更新cnt_last）
double DWT_GetDeltaT64(uint32_t *cnt_last);            // 获取高精度时间差（秒，更新cnt_last）
float DWT_GetDeltaTWithOutUpdate(uint32_t *cnt_last);  // 获取时间差（秒，不更新cnt_last）
float DWT_GetTimeline_s(void);                         // 获取系统时间（秒）
float DWT_GetTimeline_ms(void);                        // 获取系统时间（毫秒）
uint64_t DWT_GetTimeline_us(void);                     // 获取系统时间（微秒）
void DWT_Delay(float Delay);                           // 延时（秒）
void DWT_Delayms(float Delay);                         // 延时（毫秒）
void DWT_Delayus(float Delay);                         // 延时（微秒）
void DWT_SysTimeUpdate(void);                          // 更新系统时间
void DWT_CNT_Update(void);                             // 更新CYCCNT溢出计数

#ifdef __cplusplus
}
#endif

#endif // DWT_LIB_H