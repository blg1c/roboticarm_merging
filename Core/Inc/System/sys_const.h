#ifndef SYS_CONST_H
#define SYS_CONST_H

#ifdef __cplusplus
extern "C" {
#endif 

#include <stdint.h>

/* ========================================================================= */
/* Util 层 (基础工具库)                          */
/* ========================================================================= */
// CAN 通信相关
#define CONST_FDCAN_RX_BUFF_LEN             16U     // 接收缓冲区长度
#define CONST_FDCAN_MAX_DATA_LEN            8U      // CAN最大数据长度
#define FDCAN_DLC_SHIFT                     16U     // FDCAN DLC偏移位数

// 数学常量
#define PI                                  3.1415926535f // 圆周率

// 按键与事件触发 ID
#define RISE_TRIGGER                        0xAA
#define DOWN_TRIGGER                        0xBB
#define KEY_FUNC_EVENT_ID                   0xA1
#define KEY_BACK_EVENT_ID                   0x12
#define BIG_KEY1_EVENT_ID                   0x64
#define BIG_KEY2_EVENT_ID                   0x66


/* ========================================================================= */
/* Periph 层 (外设硬件相关)                         */
/* ========================================================================= */
#define MOTOR_GROUP_NUM                     22      // 最大电机组数量
#define DM_MOTOR_GROUP_NUM                  10
#define RS_MOTOR_GROUP_NUM                  12


/* ========================================================================= */
/* Modules 层 (模块业务)                         */
/* ========================================================================= */
// 1. 身体控制相关
#define BODY_DOUBLE_LOOP_PID_MOTOR_NUM      12      // 身体双环PID电机数
#define BODY_MOTOR_ID_MIN                   11      // 身体电机ID最小范围
#define BODY_MOTOR_ID_MAX                   22      // 身体电机ID最大范围
#define MOTOR_CTRL_CMD_NUM                  22      // 电机控制命令数组长度

// 2. 机械臂控制相关
#define LEFT_ARM_DOUBLE_LOOP_PID_MOTOR_NUM  5       // 左臂支持电机数量（1-5号）
#define RIGHT_ARM_DOUBLE_LOOP_PID_MOTOR_NUM 5       // 右臂支持电机数量（6-10号）

// 3. 物理极限参数 (Robot State Limits) - 外部变量声明
extern const float RS_P_MIN;     // 位置最小值(rad)
extern const float RS_P_MAX;     // 位置最大值(rad)
extern const float RS_V_MIN;     // 速度最小值(rad/s)
extern const float RS_V_MAX;     // 速度最大值(rad/s)
extern const float RS_T_MIN;     // 力矩最小值(Nm)
extern const float RS_T_MAX;     // 力矩最大值(Nm)
extern const float RS_KP_MIN;    // Kp(刚度)最小值
extern const float RS_KP_MAX;    // Kp(刚度)最大值
extern const float RS_KD_MIN;    // Kd(阻尼)最小值
extern const float RS_KD_MAX;    // Kd(阻尼)最大值


/* ========================================================================= */
/* Communicate 层 (通信协议相关)                     */
/* ========================================================================= */
// 缓冲区长度定义
#define COMM_SEND_BUFF_LEN                  64
#define COMM_RECEIVE_BUFF_LEN               64
#define Const_Comm_Transmit_BUFF_SIZE       3
#define Const_Comm_Receive_BUFF_SIZE        9

// 控制对象宏（对应帧格式）
#define CONTROL_OBJ_BODY                    0x00    // body
#define CONTROL_OBJ_L_ARM                   0x01    // 左臂
#define CONTROL_OBJ_R_ARM                   0x02    // 右臂

// 方向定义
#define LEFT                                0
#define RIGHT                               1


#ifdef __cplusplus
}
#endif

#endif // SYS_CONST_H