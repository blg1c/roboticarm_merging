/*
 * Project      : Polaris Robot
 * FilePath     : module_body.h
 * Description  : 机器狗躯干/腿部电机控制模块
 */
#ifndef __MODULE_BODY_H__
#define __MODULE_BODY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "periph_motor.h"
#include "alg_pid.h"
#include "util_can.h"

/* 1. 身体电机总数与 ID 范围 */
#define BODY_DOUBLE_LOOP_PID_MOTOR_NUM 12  
#define BODY_MOTOR_ID_MIN 11
#define BODY_MOTOR_ID_MAX 22

/* 2. 物理极限参数 (Robot State Limits) */
static const float RS_P_MIN = -12.57f;  // 位置最小值(rad)
static const float RS_P_MAX = 12.57f;   // 位置最大值(rad)
static const float RS_V_MIN = -50.0f;   // 速度最小值(rad/s)
static const float RS_V_MAX = 50.0f;    // 速度最大值(rad/s)
static const float RS_T_MIN = -36.0f;   // 力矩最小值(Nm)
static const float RS_T_MAX = 36.0f;    // 力矩最大值(Nm)

static const float RS_KP_MIN = 0.0f;     // Kp(刚度)最小值
static const float RS_KP_MAX = 5000.0f;  // Kp(刚度)最大值

static const float RS_KD_MIN = 0.0f;     // Kd(阻尼)最小值
static const float RS_KD_MAX = 100.0f;   // Kd(阻尼)最大值

/* 3. 身体电机控制结构体 */
typedef struct {
    Motor_MotorType_e motor_type;        // 电机类型
    uint8_t motor_id;                    // 电机ID
    PID_PIDTypeDef pos_pid;              // 位置环PID
    PID_PIDParamTypeDef pos_pid_param;   // 位置环PID参数
    PID_PIDTypeDef speed_pid;            // 速度环PID（内环）
    PID_PIDParamTypeDef speed_pid_param; // 速度环PID参数
    float target_pos;                    // 目标位置(rad)
    float target_speed;                  // 目标速度(rad/s)
    float feedforward_torque;            // 前馈扭矩(Nm)
    float output_torque;                 // 输出控制扭矩(Nm)
    Motor_MotorTypeDef* motor_hdl;       // 电机句柄
    FDCAN_HandleTypeDef* can_hdl;        // CAN句柄
} Body_DoubleLoopPIDTypeDef;

/* 全局变量声明 */
extern Body_DoubleLoopPIDTypeDef Body_DoubleLoopPID[BODY_DOUBLE_LOOP_PID_MOTOR_NUM];

/* 函数声明 */
uint8_t Body_DoubleLoopPID_Init(Motor_MotorType_e motor_type, uint8_t motor_id,
                                float pos_kp, float pos_ki, float pos_kd,
                                float speed_kp, float speed_ki, float speed_kd,
                                float feedforward_torque);

uint8_t Body_DoubleLoopPID_SetTargetPos(Motor_MotorType_e motor_type, uint8_t motor_id, float target_pos);
float Body_DoubleLoopPID_Calc(Body_DoubleLoopPIDTypeDef* pid_hdl);
uint8_t Body_DoubleLoopPID_ControlLoop(Motor_MotorType_e motor_type, uint8_t motor_id);
uint8_t Body_DoubleLoopPID_InitAll(void);
void Body_Output(void);

#ifdef __cplusplus
}
#endif

#endif /* __MODULE_BODY_H__ */