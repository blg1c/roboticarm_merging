/*
 *  Project      : Polaris Robot
 *  FilePath     : module_right_arm.c
 *  Description  : 右机械臂DM电机双环PID位置控制模块实现
 *  LastEditors  : Polaris
 *  Date         : 2024-0x-0x
 *  LastEditTime : 2024-0x-0x
 */
#include "module_right_arm.h"
#include "string.h"
#include "comm_receive.h"

/* 全局变量 */
RightArm_DoubleLoopPIDTypeDef RightArm_DoubleLoopPID[RIGHT_ARM_DOUBLE_LOOP_PID_MOTOR_NUM] = {0};



/**
 * @brief  根据电机类型和ID获取双环PID控制器句柄
 * @param  motor_type: 电机类型
 * @param  motor_id: 电机ID（6-10）
 * @retval 控制器句柄NULL表示无效
 */
static RightArm_DoubleLoopPIDTypeDef* RightArm_DoubleLoopPID_GetHandle(Motor_MotorType_e motor_type, uint8_t motor_id) {
    for (uint8_t i = 0; i < RIGHT_ARM_DOUBLE_LOOP_PID_MOTOR_NUM; i++) {
        if (RightArm_DoubleLoopPID[i].motor_type == motor_type && RightArm_DoubleLoopPID[i].motor_id == motor_id) {
            return &RightArm_DoubleLoopPID[i];
        }
    }
    return NULL;
}

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
                                    float feedforward_torque) {
    // 1. 校验电机类型和ID有效性（6-10为右臂）
    if (motor_type >= Motor_TYPE_MAX || motor_id < 6 || motor_id > 10) {
        return 1;
    }

    // 2. 获取对应电机CAN句柄
    Motor_MotorTypeDef* motor_hdl = Motor_GetMotorHandle(motor_type, motor_id);
    if (motor_hdl == NULL || !motor_hdl->is_online) {
        return 1;
    }
    Motor_MotorGroupTypeDef* group_hdl = Motor_GetMotorGroupHandle(motor_id - 1);
    if (group_hdl == NULL || group_hdl->can_handle == NULL) {
        return 1;
    }

    // 3. 获取/分配双环PID控制器句柄
    RightArm_DoubleLoopPIDTypeDef* pid_hdl = RightArm_DoubleLoopPID_GetHandle(motor_type, motor_id);
    if (pid_hdl == NULL) {
        for (uint8_t i = 0; i < RIGHT_ARM_DOUBLE_LOOP_PID_MOTOR_NUM; i++) {
            if (RightArm_DoubleLoopPID[i].motor_id == 0) {
                pid_hdl = &RightArm_DoubleLoopPID[i];
                break;
            }
        }
        if (pid_hdl == NULL) {
            return 1; // 控制器资源耗尽
        }
    }

    // 4. 初始化控制器基础信息
    memset(pid_hdl, 0, sizeof(RightArm_DoubleLoopPIDTypeDef));
    pid_hdl->motor_type = motor_type;
    pid_hdl->motor_id = motor_id;
    pid_hdl->motor_hdl = motor_hdl;
    pid_hdl->can_hdl = (FDCAN_HandleTypeDef*)group_hdl->can_handle;
    pid_hdl->feedforward_torque = feedforward_torque;
    pid_hdl->target_pos = motor_hdl->encoder.angle; // 初始目标位置为当前位置

    // 5. 设置PID控制器积分/输出限幅
    float pos_sum_max = 0.0f, pos_output_max = 0.0f;
    float speed_sum_max = 0.0f, speed_output_max = 0.0f;
    switch (motor_type) {
        case Motor_TYPE_DM4310:
            pos_sum_max = 10.0f;    // 位置环积分限幅
            pos_output_max = 30.0f; // 位置环输出限幅(速度目标), rad/s
            speed_sum_max = 5.0f;   // 速度环积分限幅
            speed_output_max = 28.0f;// 速度环输出限幅(扭矩), Nm
            break;
        case Motor_TYPE_DM8006:
            pos_sum_max = 15.0f;
            pos_output_max = 45.0f;
            speed_sum_max = 10.0f;
            speed_output_max = 54.0f;
            break;
        case Motor_TYPE_DM8009:
            pos_sum_max = 8.0f;
            pos_output_max = 30.0f;
            speed_sum_max = 3.0f;
            speed_output_max = 10.0f;
            break;
        default:
            return 1;
    }

    // 6. 初始化位置环PID参数
    PID_ClearPID(&pid_hdl->pos_pid);
    PID_InitPIDParam(&pid_hdl->pos_pid_param, 
                     pos_kp, pos_ki, pos_kd,
                     pos_sum_max, pos_output_max,
                     0.0f, 0.0f,
                     0.0f, 0.0f,
                     0.0f, 0.0f,
                     PID_POSITION);

    // 7. 初始化速度环PID参数
    PID_ClearPID(&pid_hdl->speed_pid);
    PID_InitPIDParam(&pid_hdl->speed_pid_param, 
                     speed_kp, speed_ki, speed_kd,
                     speed_sum_max, speed_output_max,
                     100.0f, 100.0f,
                     0.0f, 0.0f,
                     0.0f, 0.0f,
                     PID_POSITION);

    return 0;
}

/**
 * @brief  设置右臂电机目标位置
 * @param  motor_type: 电机类型
 * @param  motor_id: 电机ID（6-10）
 * @param  target_pos: 目标位置(rad)
 * @retval 0-成功 1-失败
 */
uint8_t RightArm_DoubleLoopPID_SetTargetPos(Motor_MotorType_e motor_type, uint8_t motor_id, float target_pos) {
    RightArm_DoubleLoopPIDTypeDef* pid_hdl = RightArm_DoubleLoopPID_GetHandle(motor_type, motor_id);
    if (pid_hdl == NULL || !pid_hdl->motor_hdl->is_online) {
        return 1;
    }
    pid_hdl->target_pos = target_pos;
    return 0;
}

/**
 * @brief  双环PID计算（位置环外环+速度环内环）
 * @param  pid_hdl: 双环PID控制器句柄
 * @retval 输出控制扭矩(Nm)
 */
float RightArm_DoubleLoopPID_Calc(RightArm_DoubleLoopPIDTypeDef* pid_hdl) {
    if (pid_hdl == NULL || !pid_hdl->motor_hdl->is_online) {
        return 0.0f;
    }

    // 1. 获取电机当前状态（位置/速度）
    float curr_pos = pid_hdl->motor_hdl->encoder.angle;   // 当前位置(rad)
    float curr_speed = pid_hdl->motor_hdl->encoder.speed; // 当前速度(rad/s)

    // 2. 位置环PID计算（输出为速度环目标）
    PID_SetPIDRef(&pid_hdl->pos_pid, pid_hdl->target_pos);
    PID_SetPIDFdb(&pid_hdl->pos_pid, curr_pos);
    PID_CalcPID(&pid_hdl->pos_pid, &pid_hdl->pos_pid_param);
    pid_hdl->target_speed = PID_GetPIDOutput(&pid_hdl->pos_pid);

    // 3. 速度环PID计算（输出为前馈扭矩）
    PID_SetPIDRef(&pid_hdl->speed_pid, pid_hdl->target_speed);
    PID_SetPIDFdb(&pid_hdl->speed_pid, curr_speed);
    PID_CalcPID(&pid_hdl->speed_pid, &pid_hdl->speed_pid_param);
    float speed_pid_out = PID_GetPIDOutput(&pid_hdl->speed_pid);

    // 4. 扭矩输出计算（速度环输出+固定前馈扭矩）
    pid_hdl->output_torque = speed_pid_out + pid_hdl->feedforward_torque;

    // 5. 扭矩限幅（按电机类型）
    switch (pid_hdl->motor_type) {
        case Motor_TYPE_DM4310:
            LimitMax(pid_hdl->output_torque, 28.0f);
            break;
        case Motor_TYPE_DM8006:
            LimitMax(pid_hdl->output_torque, 54.0f);
            break;
        case Motor_TYPE_DM8009:
            LimitMax(pid_hdl->output_torque, 10.0f);
            break;
        default:
            break;
    }

    return pid_hdl->output_torque;
}

/**
 * @brief  位置闭环控制循环（读取电机状态+PID计算+发送指令）
 * @param  motor_type: 电机类型
 * @param  motor_id: 电机ID（6-10）
 * @retval 0-成功 1-失败
 */
uint8_t RightArm_DoubleLoopPID_ControlLoop(Motor_MotorType_e motor_type, uint8_t motor_id) {
    RightArm_DoubleLoopPIDTypeDef* pid_hdl = RightArm_DoubleLoopPID_GetHandle(motor_type, motor_id);
    if (pid_hdl == NULL || !pid_hdl->motor_hdl->is_online) {
        return 1;
    }

    // 1. 双环PID计算
    float output_torque = RightArm_DoubleLoopPID_Calc(pid_hdl);

    // 2. 发送位置控制指令（位置模式：目标位置+扭矩）
    uint8_t ret = FDCAN_SendPosCmd(pid_hdl->can_hdl, motor_id, pid_hdl->target_pos, output_torque);
    /*if (ret != 0) {
        pid_hdl->motor_hdl->watchdog++; // 通讯故障，喂狗计数
        return 1;
    }*/

    return 0;
}


/**
 * @brief 右臂电机输出控制函数（核心逻辑：按上位机控制模式分发指令）
 * @note  6-10号电机为右臂，仅位置模式执行双环PID，MIT/扭矩模式直接透传指令
 */
void RightArm_Output() {
    // 遍历左臂所有电机（6-10号）
    for (uint8_t motor_id = 6; motor_id <= 10; motor_id++) {
        // 1. 获取当前电机的控制指令和电机类型
        Motor_ControlCmd_t* cmd = Motor_GetCtrlCmdPtr(motor_id - 1);  
        Motor_MotorType_e motor_type = FDCAN_GetMotorTypeById(motor_id);
        if (motor_type == Motor_TYPE_MAX) {
            continue;  // 无效电机ID，跳过
        }

        // 2. 获取电机CAN句柄（复用现有PID结构体的CAN句柄，保证一致性）
        RightArm_DoubleLoopPIDTypeDef* pid_hdl = RightArm_DoubleLoopPID_GetHandle(motor_type, motor_id);
        if (pid_hdl == NULL || pid_hdl->can_hdl == NULL || !pid_hdl->motor_hdl->is_online) {
            continue;  // 控制器未初始化/CAN句柄无效/电机离线，跳过
        }

        // 3. 按控制模式分发指令
        switch (cmd->mode) {
            case MOTOR_MODE_POSITION:
                // 位置模式：执行双环PID闭环（含反馈读取+PID计算+指令发送）
                RightArm_DoubleLoopPID_SetTargetPos(motor_type, motor_id, cmd->target_pos);
                RightArm_DoubleLoopPID_ControlLoop(motor_type, motor_id);
                break;

            case MOTOR_MODE_MIT:
                // MIT模式：直接发送MIT指令（透传上位机参数）
                FDCAN_SendMITCmd(
                    pid_hdl->can_hdl,
                    motor_id,
                    cmd->target_pos,
                    cmd->target_vel,
                    cmd->kp,
                    cmd->kd,
                    cmd->target_torque
                );
                break;

            case MOTOR_MODE_TORQUE:
                // 扭矩模式：直接发送扭矩指令（透传上位机扭矩参数）
                FDCAN_SendTorqueCmd(
                    pid_hdl->can_hdl,
                    motor_id,
                    cmd->target_torque
                );
                break;

            default:
                // 无效模式：清空输出（可选：也可保持上次指令/报错）
                FDCAN_SendTorqueCmd(pid_hdl->can_hdl, motor_id, 0.0f);
                break;
        }
    }
}

/**
 * @brief  初始化所有右臂电机的双环PID控制器（默认参数）
 * @retval 0-成功 1-失败
 */

uint8_t RightArm_DoubleLoopPID_InitAll(void) {
    uint8_t ret = 0;
    // 右臂DM4310电机：9,10号
    ret |= RightArm_DoubleLoopPID_Init(Motor_TYPE_DM4310, 9, 80.0f, 0.5f, 5.0f, 10.0f, 0.2f, 0.8f, 0.0f);
    ret |= RightArm_DoubleLoopPID_Init(Motor_TYPE_DM4310, 10, 80.0f, 0.5f, 5.0f, 10.0f, 0.2f, 0.8f, 0.0f);

    // 右臂DM8006电机：6,8号
    ret |= RightArm_DoubleLoopPID_Init(Motor_TYPE_DM8006, 6, 70.0f, 0.3f, 4.0f, 8.0f, 0.1f, 0.6f, 0.0f);
    ret |= RightArm_DoubleLoopPID_Init(Motor_TYPE_DM8006, 8, 70.0f, 0.3f, 4.0f, 8.0f, 0.1f, 0.6f, 0.0f);

    // 右臂DM8009电机：7号
    ret |= RightArm_DoubleLoopPID_Init(Motor_TYPE_DM8009, 7, 90.0f, 0.4f, 6.0f, 12.0f, 0.15f, 0.9f, 0.0f);

    return (ret == 0) ? 0 : 1;
}