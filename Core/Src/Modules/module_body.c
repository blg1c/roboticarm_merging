/*
 * Project      : Polaris Robot
 * FilePath     : module_body.c
 * Description  : 机器狗躯干/腿部电机控制模块实现
 */
#include "module_body.h"
#include "string.h"
#include "sys_dwt.h"
#include "comm_receive.h"

// 内部宏函数：用于上下限幅
#define CLAMP(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

Body_DoubleLoopPIDTypeDef Body_DoubleLoopPID[BODY_DOUBLE_LOOP_PID_MOTOR_NUM] = {0};

/**
 * @brief 获取控制器句柄
 */
static Body_DoubleLoopPIDTypeDef* Body_DoubleLoopPID_GetHandle(Motor_MotorType_e motor_type, uint8_t motor_id) {
    for (uint8_t i = 0; i < BODY_DOUBLE_LOOP_PID_MOTOR_NUM; i++) {
        if (Body_DoubleLoopPID[i].motor_type == motor_type && Body_DoubleLoopPID[i].motor_id == motor_id) {
            return &Body_DoubleLoopPID[i];
        }
    }
    return NULL;
}

/**
 * @brief 初始化指定躯干电机的双环PID控制器
 */
uint8_t Body_DoubleLoopPID_Init(Motor_MotorType_e motor_type, uint8_t motor_id,
                                float pos_kp, float pos_ki, float pos_kd,
                                float speed_kp, float speed_ki, float speed_kd,
                                float feedforward_torque) {
    // 1. 校验有效性
    if (motor_type >= Motor_TYPE_MAX || motor_id < BODY_MOTOR_ID_MIN || motor_id > BODY_MOTOR_ID_MAX) {
        return 1;
    }

    // 2. 获取电机底层句柄
    Motor_MotorTypeDef* motor_hdl = Motor_GetMotorHandle(motor_type, motor_id);
    if (motor_hdl == NULL || !motor_hdl->is_online) return 1;
    
    // 3. 获取CAN组句柄
    Motor_MotorGroupTypeDef* group_hdl = Motor_GetMotorGroupHandle(motor_id - 1); 
    if (group_hdl == NULL || group_hdl->can_handle == NULL) return 1;

    // 4. 分配或获取PID句柄
    Body_DoubleLoopPIDTypeDef* pid_hdl = Body_DoubleLoopPID_GetHandle(motor_type, motor_id);
    if (pid_hdl == NULL) {
        for (uint8_t i = 0; i < BODY_DOUBLE_LOOP_PID_MOTOR_NUM; i++) {
            if (Body_DoubleLoopPID[i].motor_id == 0) { // 找个空位
                pid_hdl = &Body_DoubleLoopPID[i];
                break;
            }
        }
        if (pid_hdl == NULL) return 1; 
    }

    // 5. 参数赋值
    memset(pid_hdl, 0, sizeof(Body_DoubleLoopPIDTypeDef));
    pid_hdl->motor_type = motor_type;
    pid_hdl->motor_id = motor_id;
    pid_hdl->motor_hdl = motor_hdl;
    pid_hdl->can_hdl = (FDCAN_HandleTypeDef*)group_hdl->can_handle;
    pid_hdl->feedforward_torque = feedforward_torque;
    pid_hdl->target_pos = motor_hdl->encoder.angle; 

    // 6. 配置限幅 (严格使用外部声明的限幅参数)
    float pos_sum_max = 10.0f;           
    float pos_output_max = RS_V_MAX;     // 严格遵循外部定义的最大速度 50.0 rad/s
    
    float speed_sum_max = 10.0f;         
    float speed_output_max = RS_T_MAX;   // 严格遵循外部定义的最大力矩 36.0 Nm

    PID_ClearPID(&pid_hdl->pos_pid);
    PID_InitPIDParam(&pid_hdl->pos_pid_param, pos_kp, pos_ki, pos_kd, pos_sum_max, pos_output_max, 0,0,0,0,0,0, PID_POSITION);

    PID_ClearPID(&pid_hdl->speed_pid);
    PID_InitPIDParam(&pid_hdl->speed_pid_param, speed_kp, speed_ki, speed_kd, speed_sum_max, speed_output_max, 100,100,0,0,0,0, PID_POSITION);

    return 0;
}

/**
 * @brief 设置目标位置
 */
uint8_t Body_DoubleLoopPID_SetTargetPos(Motor_MotorType_e motor_type, uint8_t motor_id, float target_pos) {
    Body_DoubleLoopPIDTypeDef* pid_hdl = Body_DoubleLoopPID_GetHandle(motor_type, motor_id);
    if (pid_hdl == NULL) return 1;

    // 目标位置限幅
    pid_hdl->target_pos = CLAMP(target_pos, RS_P_MIN, RS_P_MAX);
    return 0;
}

/**
 * @brief 串级 PID 核心计算
 */
float Body_DoubleLoopPID_Calc(Body_DoubleLoopPIDTypeDef* pid_hdl) {
    if (pid_hdl == NULL || !pid_hdl->motor_hdl->is_online) return 0.0f;

    float curr_pos = pid_hdl->motor_hdl->encoder.angle;
    float curr_speed = pid_hdl->motor_hdl->encoder.speed;

    // 1. 位置环计算
    PID_SetPIDRef(&pid_hdl->pos_pid, pid_hdl->target_pos);
    PID_SetPIDFdb(&pid_hdl->pos_pid, curr_pos);
    PID_CalcPID(&pid_hdl->pos_pid, &pid_hdl->pos_pid_param);
    pid_hdl->target_speed = PID_GetPIDOutput(&pid_hdl->pos_pid);
    
    // 二次保险：限制目标速度
    pid_hdl->target_speed = CLAMP(pid_hdl->target_speed, RS_V_MIN, RS_V_MAX);

    // 2. 速度环计算
    PID_SetPIDRef(&pid_hdl->speed_pid, pid_hdl->target_speed);
    PID_SetPIDFdb(&pid_hdl->speed_pid, curr_speed);
    PID_CalcPID(&pid_hdl->speed_pid, &pid_hdl->speed_pid_param);
    float speed_pid_out = PID_GetPIDOutput(&pid_hdl->speed_pid);

    // 3. 最终输出力矩加上前馈
    pid_hdl->output_torque = speed_pid_out + pid_hdl->feedforward_torque;

    // 二次保险：限制最终输出力矩
    pid_hdl->output_torque = CLAMP(pid_hdl->output_torque, RS_T_MIN, RS_T_MAX);

    return pid_hdl->output_torque;
}

/**
 * @brief PID 闭环执行函数，发送给下层
 */
uint8_t Body_DoubleLoopPID_ControlLoop(Motor_MotorType_e motor_type, uint8_t motor_id) {
    Body_DoubleLoopPIDTypeDef* pid_hdl = Body_DoubleLoopPID_GetHandle(motor_type, motor_id);
    if (pid_hdl == NULL || pid_hdl->can_hdl == NULL || !pid_hdl->motor_hdl->is_online) {
        return 1;
    }

    float final_torque = Body_DoubleLoopPID_Calc(pid_hdl);
    
    // 发送位置指令及计算出的扭矩
    FDCAN_SendPosCmd(pid_hdl->can_hdl, motor_id, pid_hdl->target_pos, final_torque);

    return 0;
}

/**
 * @brief 输出总控，根据上位机模式路由指令
 */
void Body_Output(void) {
    // 遍历所有躯干电机 (ID: 11 ~ 22)
    for (uint8_t motor_id = BODY_MOTOR_ID_MIN; motor_id <= BODY_MOTOR_ID_MAX; motor_id++) {
        
        // 映射索引，获取控制命令
        Motor_ControlCmd_t* cmd = Motor_GetCtrlCmdPtr(motor_id - 1); 
        Motor_MotorType_e motor_type = FDCAN_GetMotorTypeById(motor_id - 1);
        
        if (motor_type == Motor_TYPE_MAX) continue;

        Body_DoubleLoopPIDTypeDef* pid_hdl = Body_DoubleLoopPID_GetHandle(motor_type, motor_id);
        if (pid_hdl == NULL || pid_hdl->can_hdl == NULL || !pid_hdl->motor_hdl->is_online) continue;

        // 根据控制模式派发任务
        switch (cmd->mode) {
            case MOTOR_MODE_POSITION:
                Body_DoubleLoopPID_SetTargetPos(motor_type, motor_id, cmd->target_pos);
                Body_DoubleLoopPID_ControlLoop(motor_type, motor_id);
                break;
                
            case MOTOR_MODE_MIT:
            {
                // 对 MIT 模式的参数进行保护性限幅，确保遵循 Robstride 的限制
                float safe_pos    = CLAMP(cmd->target_pos, RS_P_MIN, RS_P_MAX);
                float safe_vel    = CLAMP(cmd->target_vel, RS_V_MIN, RS_V_MAX);
                float safe_torque = CLAMP(cmd->target_torque, RS_T_MIN, RS_T_MAX);
                float safe_kp     = CLAMP(cmd->kp, RS_KP_MIN, RS_KP_MAX);
                float safe_kd     = CLAMP(cmd->kd, RS_KD_MIN, RS_KD_MAX);

                FDCAN_SendMITCmd_RS(pid_hdl->can_hdl, motor_id, safe_pos, safe_vel, safe_kp, safe_kd, safe_torque);
                break;
            }
                
            case MOTOR_MODE_TORQUE:
            {
                float safe_torque = CLAMP(cmd->target_torque, RS_T_MIN, RS_T_MAX);
                FDCAN_SendTorqueCmd_RS(pid_hdl->can_hdl, motor_id, safe_torque);
                break;
            }
                
            default:
                FDCAN_SendTorqueCmd_RS(pid_hdl->can_hdl, motor_id, 0.0f);
                break;
        }
    }
}

/**
 * @brief 初始化所有躯干/腿部电机
 * @retval 0-成功 1-失败
 */
uint8_t Body_DoubleLoopPID_InitAll(void) {
    uint8_t ret = 0;
    
    // 初始化所有12个身体电机
    for(uint8_t id = BODY_MOTOR_ID_MIN; id <= BODY_MOTOR_ID_MAX; id++) {
        
        // 初始化上位机(MCU)侧的双环PID参数。
        // 注意：如果实际控制中全程使用 MIT 模式，这里的参数实际上不会参与运算；
        // 但为了防止上位机切入 MOTOR_MODE_POSITION (主控运算模式)，仍需赋一套基础防抖参数。
        ret |= Body_DoubleLoopPID_Init(
            Motor_TYPE_RS06, // 电机类型
            id, 
            15.0f, 0.0f, 0.5f,    // 位置环 PID 初始参数
            2.0f,  0.0f, 0.1f,    // 速度环 PID 初始参数
            0.0f                  // 前馈扭矩
        );
    }
    
    return (ret == 0) ? 0 : 1;
}