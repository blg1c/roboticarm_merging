/*
 *  Project      : Polaris Robot 
 *  FilePath     : module_motor_cmd.c
 *  Description  : 电机控制命令数组初始化模块
 *  LastEditors  : Polaris
 *  Date         : 202X-XX-XX XX:XX:XX
 *  LastEditTime : 202X-XX-XX XX:XX:XX
 */

#include "module_motor_cmd.h"
#include <string.h>  // 用于memset初始化内存

// 全局电机控制命令数组（原comm_receive.c中的定义迁移至此）
Motor_ControlCmd_t motor_ctrl_cmds[MOTOR_CTRL_CMD_NUM];

/**
 * @brief 电机控制命令数组初始化
 */
void Motor_CtrlCmds_Init(void) {
    // 1. 先将数组所有字节置0，清空默认值
    memset(motor_ctrl_cmds, 0, sizeof(Motor_ControlCmd_t) * MOTOR_CTRL_CMD_NUM);

    // 2. 可选：设置默认值（根据业务需求补充）
    for (uint8_t i = 0; i < MOTOR_CTRL_CMD_NUM; i++) {
        motor_ctrl_cmds[i].channel = FDCAN1_CH;  // 默认CAN通道
        // 其他参数（target_pos/vel/torque/kp/kd）默认0即可
    }
}

/**
 * @brief 获取电机控制命令指针（迁移原comm_receive.c中的函数）
 */
Motor_ControlCmd_t* Motor_GetCtrlCmdPtr(uint8_t cmd_idx) {
    if (cmd_idx >= MOTOR_CTRL_CMD_NUM) {
        return NULL;
    }
    return &motor_ctrl_cmds[cmd_idx];
}