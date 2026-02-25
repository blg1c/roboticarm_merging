/*
 *  Project      : Polaris Robot
 *
 *  FilePath     : comm_common.h
 *  Description  : This file contains Bus communication control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:03:07
 *  LastEditTime : 2023-10-04 00:33:47
 */

#ifndef COMM_COMMON_H
#define COMM_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "periph_motor.h"

// 缓冲区长度定义
#define COMM_SEND_BUFF_LEN     64
#define COMM_RECEIVE_BUFF_LEN  64

// 控制对象宏（对应帧格式）
#define CONTROL_OBJ_BODY   0x00  // body
#define CONTROL_OBJ_L_ARM    0x01  // 左臂
#define CONTROL_OBJ_R_ARM    0x02  // 右臂

// 通信状态枚举
typedef enum {
    Comm_STATE_NULL      = 0,  // 初始状态
    Comm_STATE_CONNECTED = 1,  // 已连接
    Comm_STATE_LOST      = 2,  // 连接丢失
    Comm_STATE_ERROR     = 3,  // 错误状态
    Comm_STATE_PENDING   = 4   // 挂起状态
} Comm_CommStateEnum;

// 通信数据结构体
typedef struct {
    Comm_CommStateEnum state;          // 通信状态
    uint8_t usb_watchBuff[COMM_RECEIVE_BUFF_LEN];  // USB接收监听缓冲区
    uint8_t usb_sendBuff[COMM_SEND_BUFF_LEN];      // USB发送缓冲区
    uint32_t last_rx_tick;             // 最后一次接收时间戳(us)
    float rx_dt;                       // 接收时间差(s)
    uint32_t last_tx_tick;             // 最后一次发送时间戳(us)
    float tx_dt;                       // 发送时间差(s)
    uint32_t rx_len;                   // 本次接收数据长度
} Comm_DataTypeDef;

// 外部变量声明
extern Comm_DataTypeDef Comm_Data;
extern Motor_ControlCmd_t motor_ctrl_cmds[10];
extern int ARM_P;

// 指令函数指针结构体（需与comm_transmit/comm_receive中定义一致）
typedef struct {
    uint32_t (*bus_func)(uint8_t* buff);
} Comm_CmdFuncDef;

// 外部指令表声明（需在comm_transmit/comm_receive中定义）
//extern Comm_CmdFuncDef CommCmd_Send[];
//extern Comm_CmdFuncDef CommCmd_Receive[];
extern uint32_t Const_Comm_Transmit_BUFF_SIZE;
extern uint32_t Const_Comm_OFFLINE_TIME;

// 函数声明
void Comm_InitComm(void);
Comm_DataTypeDef* Comm_GetBusDataPtr(void);
uint8_t Comm_IsCommOffline(void);
uint8_t Comm_IsGimCommOffline(void);
void Comm_SendBlockError(void);
void Comm_ResetCommData(void);
//void Comm_SendCommData(void);
void Comm_Send_Body_CommData(void);
void Comm_Send_L_Arm_CommData(void);
void Comm_Send_R_Arm_CommData(void);
void Comm_DecodeData(uint8_t buff[], uint32_t rxdatalen);
void Comm_GimDecode(uint32_t stdid, uint8_t rxdata[]);
void Communicate_RXCallback_USB(uint8_t* buff, uint32_t len); // USB接收回调

#ifdef __cplusplus
}
#endif

#endif // COMM_COMMON_H