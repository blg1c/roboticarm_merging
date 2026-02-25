/*
 *  Project      : Polaris Robot
 *  FilePath     : module_right_arm.h
 *  Description  : 右机械臂DM电机双环PID位置控制模块
 *  LastEditors  : Polaris
 *  Date         : 2024-0x-0x
 *  LastEditTime : 2024-0x-0x
 */
#ifndef __MODULE_RIGHT_ARM_H__
#define __MODULE_RIGHT_ARM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "periph_motor.h"
#include "alg_pid.h"
#include "util_can.h"

/* 电机双环PID控制结构体 */
typedef struct {
    Motor_MotorType_e motor_type;  // 电机类型
    uint8_t motor_id;              // 电机ID（6-10号为右臂）
    PID_PIDTypeDef pos_pid;        // 位置环PID
    PID_PIDParamTypeDef pos_pid_param; // 位置环PID参数
    PID_PIDTypeDef speed_pid;      // 速度环PID（内环）
    PID_PIDParamTypeDef speed_pid_param; // 速度环PID参数
    float target_pos;              // 目标位置(rad)
    float target_speed;            // 目标速度(rad/s)
    float feedforward_torque;      // 前馈扭矩(Nm)
    float output_torque;           // 输出控制扭矩(Nm)
    Motor_MotorTypeDef* motor_hdl; // 电机句柄
    FDCAN_HandleTypeDef* can_hdl;  // CAN句柄
} RightArm_DoubleLoopPIDTypeDef;

/* 全局宏定义 */
#define RIGHT_ARM_DOUBLE_LOOP_PID_MOTOR_NUM 5  // 右臂支持电机数量（6-10号）

/* 全局变量 */
extern RightArm_DoubleLoopPIDTypeDef RightArm_DoubleLoopPID[RIGHT_ARM_DOUBLE_LOOP_PID_MOTOR_NUM];


/* 函数声明 */
/**
 * @brief  初始化指定右臂电机的双环PID控制器
 * @param  motor_type: 电机类型
 * @param  motor_id: 电机ID（6-10）
 * @param  pos_kp/ki/kd: 位置环PID参数
 * @param  speed_kp/ki/kd: 速度环PID参数
 * @param  feedforward_torque: 前馈扭矩
 * @retval 0-成功 1-失败
 */
uint8_t RightArm_DoubleLoopPID_Init(Motor_MotorType_e motor_type, uint8_t motor_id,
                                    float pos_kp, float pos_ki, float pos_kd,
                                    float speed_kp, float speed_ki, float speed_kd,
                                    float feedforward_torque);

/**
 * @brief  设置右臂电机目标位置
 * @param  motor_type: 电机类型
 * @param  motor_id: 电机ID（6-10）
 * @param  target_pos: 目标位置(rad)
 * @retval 0-成功 1-失败
 */
uint8_t RightArm_DoubleLoopPID_SetTargetPos(Motor_MotorType_e motor_type, uint8_t motor_id, float target_pos);

/**
 * @brief  双环PID计算（位置环外环+速度环内环）
 * @param  pid_hdl: 双环PID控制器句柄
 * @retval 输出控制扭矩(Nm)
 */
float RightArm_DoubleLoopPID_Calc(RightArm_DoubleLoopPIDTypeDef* pid_hdl);

/**
 * @brief  位置闭环控制循环（读取电机状态+PID计算+发送指令）
 * @param  motor_type: 电机类型
 * @param  motor_id: 电机ID（6-10）
 * @retval 0-成功 1-失败
 */
uint8_t RightArm_DoubleLoopPID_ControlLoop(Motor_MotorType_e motor_type, uint8_t motor_id);

/**
 * @brief  初始化所有右臂电机的双环PID控制器（默认参数）
 * @retval 0-成功 1-失败
 */
uint8_t RightArm_DoubleLoopPID_InitAll(void);


void RightArm_Output(void);

#ifdef __cplusplus
}
#endif

#endif /* __MODULE_RIGHT_ARM_H__ */