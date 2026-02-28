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
#include "sys_const.h"


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