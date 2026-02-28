/*
 *  Project      : Polaris Robot
 *  FilePath     : util_can.h
 *  Description  : FDCAN(高速CAN)通信底层驱动（适配DM8006/8009/4310）
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16
 *  LastEditTime : 2024-01-19
 */
#ifndef __UTIL_CAN_H__
#define __UTIL_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"
#include "math.h"
#include "fdcan.h"
#include "module_motor_cmd.h"  // 引入module层电机指令定义
#include "sys_const.h"


/* 函数声明 */
void FDCAN_ErrorHandler(uint32_t err_code);
void FDCAN_InitTxHeader(FDCAN_TxHeaderTypeDef *pheader, uint32_t std_id, uint32_t dlc);
void FDCAN_InitTxHeader_Extended(FDCAN_TxHeaderTypeDef *pheader, uint32_t ext_id, uint32_t dlc);
uint8_t FDCAN_InitFilterAndStart(FDCAN_HandleTypeDef* phfdcan, FDCAN_Channel_e ch);
uint8_t FDCAN_SendMITCmd(FDCAN_HandleTypeDef* phfdcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torque);
uint8_t FDCAN_SendMITCmd_RS(FDCAN_HandleTypeDef* phfdcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torque);
uint8_t FDCAN_SendPosCmd(FDCAN_HandleTypeDef* phfdcan, uint16_t motor_id, float pos, float torque);
uint8_t FDCAN_SendPosCmd_RS(FDCAN_HandleTypeDef* phfdcan, uint16_t motor_id, float pos, float torque);
uint8_t FDCAN_SendTorqueCmd(FDCAN_HandleTypeDef* phfdcan, uint16_t motor_id, float torque);
uint8_t FDCAN_SendTorqueCmd_RS(FDCAN_HandleTypeDef* phfdcan, uint16_t motor_id, float torque);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* phfdcan, uint32_t RxFifo0ITs);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* phfdcan, uint32_t RxFifo1ITs);
void FDCAN_RxMessageCallback(FDCAN_HandleTypeDef* phfdcan, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[], FDCAN_Channel_e ch);
void DM_Motor_DecodeFeedback(FDCAN_HandleTypeDef* phfdcan, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[], FDCAN_Channel_e ch);
void RS_Motor_DecodeFeedback(FDCAN_HandleTypeDef* phfdcan, uint32_t ext_id, uint8_t* rx_data);
FDCAN_Channel_e FDCAN_GetChannelByHandle(FDCAN_HandleTypeDef* phfdcan);



#ifdef __cplusplus
}
#endif

#endif /* __UTIL_CAN_H__ */