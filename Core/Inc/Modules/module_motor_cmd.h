/*
 *  Project      : Polaris Robot
 *  FilePath     : module_motor_cmd.h
 *  Description  : 电机控制指令通用定义（module层）
 *  LastEditors  : Polaris
 *  Date         : 2024-01-19
 *  LastEditTime : 2024-01-19
 */
#ifndef __MODULE_MOTOR_CMD_H__
#define __MODULE_MOTOR_CMD_H__

#ifdef __cplusplus
extern "C" {
#endif

//#include "util_can.h"  // 依赖FDCAN_Channel_e枚举
//#include "comm_receive.h"

#include "stdint.h"
#include "sys_const.h"

/* FDCAN通道枚举（仅FDCAN1 - 所有电机共用通道） */
typedef enum {
    FDCAN1_CH = 0,   // FDCAN1 - DM电机共用通道
		FDCAN2_CH,
    FDCAN_CH_MAX              // 通道总数
} FDCAN_Channel_e;

/* 电机控制模式枚举（对应不同指令格式） */
typedef enum {
    MOTOR_MODE_MIT = 1,     // MIT模式：位置+速度+KP+KD+前馈扭矩
    MOTOR_MODE_POSITION = 2,// 位置模式：目标位置+输出扭矩
    MOTOR_MODE_TORQUE = 3   // 扭矩模式：仅目标扭矩
} Motor_ControlMode_e;

/* 电机控制指令结构体（按控制模式封装目标值） */
typedef struct {
    FDCAN_Channel_e channel;        // 目标CAN通道（固定为FDCAN1_CH）
    Motor_ControlMode_e mode;       // 控制模式
    float target_pos;               // 位置目标值(rad)
    float target_vel;               // 速度目标值(rad/s)
    float target_torque;            // 扭矩目标值(Nm)
    float kp;                       // 位置比例系数(0~500)
    float kd;                       // 速度阻尼系数(0~5)
} Motor_ControlCmd_t;


void Motor_CtrlCmds_Init(void);

// 获取指定索引的电机控制命令指针（核心函数，类似Motor_GetMotorDataPtr）
Motor_ControlCmd_t* Motor_GetCtrlCmdPtr(uint8_t cmd_idx);


#ifdef __cplusplus
}
#endif

#endif /* __MODULE_MOTOR_CMD_H__ */