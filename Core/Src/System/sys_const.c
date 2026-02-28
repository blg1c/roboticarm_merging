/*
 * Project      : Polaris Robot 
 * FilePath     : sys_const.c
 * Description  : System global constants memory definitions
 */

#include "sys_const.h"

/* * 注意：宏定义 (#define) 不分配内存空间，因此在 .h 文件中声明即可，不需要在 .c 中定义。
 * 这里只放置需要占用内存空间的 const 变量实体。
 */

/* ========================================================================= */
/* Modules 层 (模块业务)                         */
/* ========================================================================= */

/* 物理极限参数 (Robot State Limits) */
const float RS_P_MIN  = -12.57f;  // 位置最小值(rad)
const float RS_P_MAX  =  12.57f;  // 位置最大值(rad)
const float RS_V_MIN  = -50.0f;   // 速度最小值(rad/s)
const float RS_V_MAX  =  50.0f;   // 速度最大值(rad/s)
const float RS_T_MIN  = -36.0f;   // 力矩最小值(Nm)
const float RS_T_MAX  =  36.0f;   // 力矩最大值(Nm)

const float RS_KP_MIN =  0.0f;    // Kp(刚度)最小值
const float RS_KP_MAX =  5000.0f; // Kp(刚度)最大值

const float RS_KD_MIN =  0.0f;    // Kd(阻尼)最小值
const float RS_KD_MAX =  100.0f;  // Kd(阻尼)最大值