/*
 *  Project      : Polaris Robot
 *  FilePath     : periph_motor.c
 *  Description  : 电机驱动实现（仅保留DM系列电机）
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16
 *  LastEditTime : 2024-01-19
 */
#include "periph_motor.h"
#include "util_can.h"
#include "sys_dwt.h"
#include <string.h>
#include "fdcan.h"
#include "lib_buff.h"
#include "sys_const.h"

/* 全局变量实例化 */
Motor_MotorGroupTypeDef *Motor_groupHandle[MOTOR_GROUP_NUM] = {NULL};

// DM4310（4个电机+4个电机组）
Motor_MotorGroupTypeDef Motor_DM4310_Group1;
Motor_MotorGroupTypeDef Motor_DM4310_Group2;
Motor_MotorGroupTypeDef Motor_DM4310_Group3;
Motor_MotorGroupTypeDef Motor_DM4310_Group4;
Motor_MotorTypeDef Motor_DM4310_Motor1;
Motor_MotorTypeDef Motor_DM4310_Motor2;
Motor_MotorTypeDef Motor_DM4310_Motor3;
Motor_MotorTypeDef Motor_DM4310_Motor4;

// DM8006（4个电机+4个电机组）
Motor_MotorGroupTypeDef Motor_DM8006_Group1;
Motor_MotorGroupTypeDef Motor_DM8006_Group2;
Motor_MotorGroupTypeDef Motor_DM8006_Group3;
Motor_MotorGroupTypeDef Motor_DM8006_Group4;
Motor_MotorTypeDef Motor_DM8006_Motor1;
Motor_MotorTypeDef Motor_DM8006_Motor2;
Motor_MotorTypeDef Motor_DM8006_Motor3;
Motor_MotorTypeDef Motor_DM8006_Motor4;

// DM8009（2个电机+2个电机组）
Motor_MotorGroupTypeDef Motor_DM8009_Group1;
Motor_MotorGroupTypeDef Motor_DM8009_Group2;
Motor_MotorTypeDef Motor_DM8009_Motor1;
Motor_MotorTypeDef Motor_DM8009_Motor2;

// Robstride06（12个电机+12个电机组）
Motor_MotorGroupTypeDef Motor_Robstride06_Group1;
Motor_MotorGroupTypeDef Motor_Robstride06_Group2;
Motor_MotorGroupTypeDef Motor_Robstride06_Group3;
Motor_MotorGroupTypeDef Motor_Robstride06_Group4;
Motor_MotorGroupTypeDef Motor_Robstride06_Group5;
Motor_MotorGroupTypeDef Motor_Robstride06_Group6;
Motor_MotorGroupTypeDef Motor_Robstride06_Group7;
Motor_MotorGroupTypeDef Motor_Robstride06_Group8;
Motor_MotorGroupTypeDef Motor_Robstride06_Group9;
Motor_MotorGroupTypeDef Motor_Robstride06_Group10;
Motor_MotorGroupTypeDef Motor_Robstride06_Group11;
Motor_MotorGroupTypeDef Motor_Robstride06_Group12;

Motor_MotorTypeDef Motor_Robstride06_Motor1;
Motor_MotorTypeDef Motor_Robstride06_Motor2;
Motor_MotorTypeDef Motor_Robstride06_Motor3;
Motor_MotorTypeDef Motor_Robstride06_Motor4;
Motor_MotorTypeDef Motor_Robstride06_Motor5;
Motor_MotorTypeDef Motor_Robstride06_Motor6;
Motor_MotorTypeDef Motor_Robstride06_Motor7;
Motor_MotorTypeDef Motor_Robstride06_Motor8;
Motor_MotorTypeDef Motor_Robstride06_Motor9;
Motor_MotorTypeDef Motor_Robstride06_Motor10;
Motor_MotorTypeDef Motor_Robstride06_Motor11;
Motor_MotorTypeDef Motor_Robstride06_Motor12;

/* DM4310编码器回调函数 */
void DM4310_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    if (pmotor == NULL || len != 8) return;
    
    // 解析电机数据(rad) - 适配DM4310参数范围
    pmotor->encoder.angle = uint_to_float((rxbuff[1]<<8)|rxbuff[2], -12.5f, 12.5f, 16);
    pmotor->encoder.speed = uint_to_float(((rxbuff[3]<<4)|(rxbuff[4]>>4)), -30, 30, 12);
    pmotor->encoder.torque = uint_to_float(((rxbuff[4]&0xF)<<8)|rxbuff[5], -28, 28, 12);
    pmotor->encoder.temp = rxbuff[7];
    pmotor->encoder.err = (rxbuff[0] >> 4) & 0x0F;
    
    // 更新状态
    pmotor->init = 1;
    pmotor->is_online = (pmotor->encoder.err == 0) ? 1 : 0;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
}

/* DM8006编码器回调函数 */
void DM8006_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    if (pmotor == NULL || len != 8) return;
    
    // 解析电机数据(rad) - 适配DM8006参数范围
    pmotor->encoder.angle = uint_to_float((rxbuff[1]<<8)|rxbuff[2], -12.5f, 12.5f, 16);
    pmotor->encoder.speed = uint_to_float(((rxbuff[3]<<4)|(rxbuff[4]>>4)), -45, 45, 12);
    pmotor->encoder.torque = uint_to_float(((rxbuff[4]&0xF)<<8)|rxbuff[5], -54, 54, 12);
    pmotor->encoder.temp = rxbuff[7];
    pmotor->encoder.err = (rxbuff[0] >> 4) & 0x0F;
    
    // 更新状态
    pmotor->init = 1;
    pmotor->is_online = (pmotor->encoder.err == 0) ? 1 : 0;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
}

/* DM8009编码器回调函数 */
void DM8009_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    if (pmotor == NULL || len != 8) return;
    
    // 解析电机数据(rad) - 适配DM8009参数范围
    pmotor->encoder.angle = uint_to_float((rxbuff[1]<<8)|rxbuff[2], -12.5f, 12.5f, 16);
    pmotor->encoder.speed = uint_to_float(((rxbuff[3]<<4)|(rxbuff[4]>>4)), -30, 30, 12);
    pmotor->encoder.torque = uint_to_float(((rxbuff[4]&0xF)<<8)|rxbuff[5], -10, 10, 12);
    pmotor->encoder.temp = rxbuff[7];
    pmotor->encoder.err = (rxbuff[0] >> 4) & 0x0F;
    
    // 更新状态
    pmotor->init = 1;
    pmotor->is_online = (pmotor->encoder.err == 0) ? 1 : 0;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
}


/* Robstride 06 编码器回调函数 (基于 MIT 通信协议) */
void RS06_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    if (pmotor == NULL || len != 8) return;
    
    // 解析电机数据 - 适配 Robstride 06 (RS06) 参数范围
    // 位置范围：通常为 -12.5 ~ 12.5 rad (约 -4*PI 到 4*PI)
    pmotor->encoder.angle = uint_to_float((rxbuff[1]<<8)|rxbuff[2], -12.5f, 12.5f, 16);
    
    // 速度范围：Robstride 06 常规最大转速配置为 30 rad/s 或 45 rad/s，这里默认使用 -30 ~ 30
    pmotor->encoder.speed = uint_to_float(((rxbuff[3]<<4)|(rxbuff[4]>>4)), -30.0f, 30.0f, 12);
    
    // 扭矩范围：根据 RS06 硬件手册，其峰值扭矩为 36 N.m，因此映射范围为 -36.0 ~ 36.0
    pmotor->encoder.torque = uint_to_float(((rxbuff[4]&0xF)<<8)|rxbuff[5], -36.0f, 36.0f, 12);
    
    // 解析温度与错误码
    pmotor->encoder.temp = rxbuff[7];
    pmotor->encoder.err = (rxbuff[0] >> 4) & 0x0F;
    
    // 更新状态
    pmotor->init = 1;
    pmotor->is_online = (pmotor->encoder.err == 0) ? 1 : 0;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
}





/* 初始化电机组 */
void Motor_InitMotorGroup(Motor_MotorGroupTypeDef *pgroup, Motor_MotorType_e type, uint8_t motor_num, void *can_handle, uint8_t *tx_id, uint8_t rx_id) {
    UNUSED(rx_id);
    if (pgroup == NULL) return;
    
    // 清空结构体
    memset(pgroup, 0, sizeof(Motor_MotorGroupTypeDef));
    
    // 配置电机组参数（仅支持DM系列）
    if (type >= Motor_TYPE_MAX) return;
    pgroup->type = type;
    pgroup->motor_num = motor_num;
    pgroup->can_handle = can_handle;
    pgroup->tx_id = tx_id;
}

/* 初始化单个电机 */
void Motor_InitMotor(Motor_MotorTypeDef *pmotor, Motor_MotorType_e type, uint8_t id, float reduce_ratio, EncoderCallback encodercallback) { //void (*EncoderCallback)(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len)
    if (pmotor == NULL) return;
    
    // 清空结构体
    memset(pmotor, 0, sizeof(Motor_MotorTypeDef));
    
    // 配置电机参数（仅支持DM系列）
    if (type >= Motor_TYPE_MAX) return;
    pmotor->type = type;
    pmotor->id = id;
    pmotor->reduce_ratio = reduce_ratio;
    pmotor->EncoderCallback = encodercallback;
    pmotor->last_update_tick = DWT_GetTimeline_ms();
}

/* 设置电机控制模式并发送指令 */

/*int a;

void Motor_SetControlMode(Motor_MotorGroupTypeDef* pgroup, Motor_ControlMode_e mode, Motor_ControlCmd_t cmd) {
    if (pgroup == NULL || pgroup->can_handle == NULL) return;
    
    // 仅处理DM系列电机
    if (pgroup->type >= Motor_TYPE_MAX) return;
    
    FDCAN_HandleTypeDef* phfdcan = (FDCAN_HandleTypeDef*)pgroup->can_handle;
    uint16_t motor_id = pgroup->motor_handle[0]->id;

    // 按模式发送指令
    switch (mode) {
        case MOTOR_MODE_MIT:
            a=FDCAN_SendMITCmd(phfdcan, motor_id, cmd.target_pos, cmd.target_vel, cmd.kp, cmd.kd, cmd.target_torque);
            break;
        case MOTOR_MODE_POSITION:
            FDCAN_SendPosCmd(phfdcan, motor_id, cmd.target_pos, cmd.target_vel);
            break;
        case MOTOR_MODE_TORQUE:
            FDCAN_SendTorqueCmd(phfdcan, motor_id, cmd.target_torque);
            break;
        default: break;
    }
}*/




/* 初始化所有电机 */
void Motor_InitAllMotors(void) {
 
    // 电机组1（ID1）
    Motor_groupHandle[0] = &Motor_DM8006_Group1;
    Motor_InitMotorGroup(&Motor_DM8006_Group1, Motor_TYPE_DM8006, 1, &hfdcan1, NULL, 0x01);
    Motor_InitMotor(&Motor_DM8006_Motor1, Motor_TYPE_DM8006, 0x01, 19.0f, DM8006_encoder_callback);
    Motor_DM8006_Group1.motor_handle[0] = &Motor_DM8006_Motor1;

    // 电机组2（ID2）
    Motor_groupHandle[1] = &Motor_DM8009_Group1;
    Motor_InitMotorGroup(&Motor_DM8009_Group1, Motor_TYPE_DM8009, 1, &hfdcan1, NULL, 0x02);
    Motor_InitMotor(&Motor_DM8009_Motor1, Motor_TYPE_DM8009, 0x02, 19.0f, DM8009_encoder_callback);
    Motor_DM8009_Group1.motor_handle[0] = &Motor_DM8009_Motor1;

    // 电机组3（ID3）
    Motor_groupHandle[2] = &Motor_DM8006_Group2;
    Motor_InitMotorGroup(&Motor_DM8006_Group2, Motor_TYPE_DM8006, 1, &hfdcan1, NULL, 0x03);
    Motor_InitMotor(&Motor_DM8006_Motor2, Motor_TYPE_DM8006, 0x03, 19.0f, DM8006_encoder_callback);
    Motor_DM8006_Group2.motor_handle[0] = &Motor_DM8006_Motor2;

    // 电机组4（ID4）
    Motor_groupHandle[3] = &Motor_DM4310_Group1;
    Motor_InitMotorGroup(&Motor_DM4310_Group1, Motor_TYPE_DM4310, 1, &hfdcan1, NULL, 0x04);
    Motor_InitMotor(&Motor_DM4310_Motor1, Motor_TYPE_DM4310, 0x04, 19.0f, DM4310_encoder_callback);
    Motor_DM4310_Group1.motor_handle[0] = &Motor_DM4310_Motor1;

    // 电机组5（ID5）
    Motor_groupHandle[4] = &Motor_DM4310_Group2;
    Motor_InitMotorGroup(&Motor_DM4310_Group2, Motor_TYPE_DM4310, 1, &hfdcan1, NULL, 0x05);
    Motor_InitMotor(&Motor_DM4310_Motor2, Motor_TYPE_DM4310, 0x05, 19.0f, DM4310_encoder_callback);
    Motor_DM4310_Group2.motor_handle[0] = &Motor_DM4310_Motor2;

    // 电机组6（ID6）
    Motor_groupHandle[5] = &Motor_DM8006_Group3;
    Motor_InitMotorGroup(&Motor_DM8006_Group3, Motor_TYPE_DM8006, 1, &hfdcan1, NULL, 0x06);
    Motor_InitMotor(&Motor_DM8006_Motor3, Motor_TYPE_DM8006, 0x06, 19.0f, DM8006_encoder_callback);
    Motor_DM8006_Group3.motor_handle[0] = &Motor_DM8006_Motor3;

    // 电机组7（ID7）
    Motor_groupHandle[6] = &Motor_DM8009_Group2;
    Motor_InitMotorGroup(&Motor_DM8009_Group2, Motor_TYPE_DM8009, 1, &hfdcan1, NULL, 0x07);
    Motor_InitMotor(&Motor_DM8009_Motor2, Motor_TYPE_DM8009, 0x07, 19.0f, DM8009_encoder_callback);
    Motor_DM8009_Group2.motor_handle[0] = &Motor_DM8009_Motor2;

    // 电机组8（ID8）
    Motor_groupHandle[7] = &Motor_DM8006_Group4;
    Motor_InitMotorGroup(&Motor_DM8006_Group4, Motor_TYPE_DM8006, 1, &hfdcan1, NULL, 0x08);
    Motor_InitMotor(&Motor_DM8006_Motor4, Motor_TYPE_DM8006, 0x08, 19.0f, DM8006_encoder_callback);
    Motor_DM8006_Group4.motor_handle[0] = &Motor_DM8006_Motor4;

    // 电机组9（ID9）
    Motor_groupHandle[8] = &Motor_DM4310_Group3;
    Motor_InitMotorGroup(&Motor_DM4310_Group3, Motor_TYPE_DM4310, 1, &hfdcan1, NULL, 0x09);
    Motor_InitMotor(&Motor_DM4310_Motor3, Motor_TYPE_DM4310, 0x09, 19.0f, DM4310_encoder_callback);
    Motor_DM4310_Group3.motor_handle[0] = &Motor_DM4310_Motor3;

    // 电机组10（ID10）
    Motor_groupHandle[9] = &Motor_DM4310_Group4;
    Motor_InitMotorGroup(&Motor_DM4310_Group4, Motor_TYPE_DM4310, 1, &hfdcan1, NULL, 0x0A);
    Motor_InitMotor(&Motor_DM4310_Motor4, Motor_TYPE_DM4310, 0x0A, 19.0f, DM4310_encoder_callback);
    Motor_DM4310_Group4.motor_handle[0] = &Motor_DM4310_Motor4;
	
	// RS06 电机 1 (ID: 0x0B / 11)
    Motor_groupHandle[10] = &Motor_Robstride06_Group1;
    Motor_InitMotorGroup(&Motor_Robstride06_Group1, Motor_TYPE_RS06, 1, &hfdcan2, NULL, 0x0B);
    Motor_InitMotor(&Motor_Robstride06_Motor1, Motor_TYPE_RS06, 0x0B, 1.0f, RS06_encoder_callback);
    Motor_Robstride06_Group1.motor_handle[0] = &Motor_Robstride06_Motor1;

    // RS06 电机 2 (ID: 0x0C / 12)
    Motor_groupHandle[11] = &Motor_Robstride06_Group2;
    Motor_InitMotorGroup(&Motor_Robstride06_Group2, Motor_TYPE_RS06, 1, &hfdcan2, NULL, 0x0C);
    Motor_InitMotor(&Motor_Robstride06_Motor2, Motor_TYPE_RS06, 0x0C, 1.0f, RS06_encoder_callback);
    Motor_Robstride06_Group2.motor_handle[0] = &Motor_Robstride06_Motor2;

    // RS06 电机 3 (ID: 0x0D / 13)
    Motor_groupHandle[12] = &Motor_Robstride06_Group3;
    Motor_InitMotorGroup(&Motor_Robstride06_Group3, Motor_TYPE_RS06, 1, &hfdcan2, NULL, 0x0D);
    Motor_InitMotor(&Motor_Robstride06_Motor3, Motor_TYPE_RS06, 0x0D, 1.0f, RS06_encoder_callback);
    Motor_Robstride06_Group3.motor_handle[0] = &Motor_Robstride06_Motor3;

    // RS06 电机 4 (ID: 0x0E / 14)
    Motor_groupHandle[13] = &Motor_Robstride06_Group4;
    Motor_InitMotorGroup(&Motor_Robstride06_Group4, Motor_TYPE_RS06, 1, &hfdcan2, NULL, 0x0E);
    Motor_InitMotor(&Motor_Robstride06_Motor4, Motor_TYPE_RS06, 0x0E, 1.0f, RS06_encoder_callback);
    Motor_Robstride06_Group4.motor_handle[0] = &Motor_Robstride06_Motor4;

    // RS06 电机 5 (ID: 0x0F / 15)
    Motor_groupHandle[14] = &Motor_Robstride06_Group5;
    Motor_InitMotorGroup(&Motor_Robstride06_Group5, Motor_TYPE_RS06, 1, &hfdcan2, NULL, 0x0F);
    Motor_InitMotor(&Motor_Robstride06_Motor5, Motor_TYPE_RS06, 0x0F, 1.0f, RS06_encoder_callback);
    Motor_Robstride06_Group5.motor_handle[0] = &Motor_Robstride06_Motor5;

    // RS06 电机 6 (ID: 0x10 / 16)
    Motor_groupHandle[15] = &Motor_Robstride06_Group6;
    Motor_InitMotorGroup(&Motor_Robstride06_Group6, Motor_TYPE_RS06, 1, &hfdcan2, NULL, 0x10);
    Motor_InitMotor(&Motor_Robstride06_Motor6, Motor_TYPE_RS06, 0x10, 1.0f, RS06_encoder_callback);
    Motor_Robstride06_Group6.motor_handle[0] = &Motor_Robstride06_Motor6;

    // RS06 电机 7 (ID: 0x11 / 17)
    Motor_groupHandle[16] = &Motor_Robstride06_Group7;
    Motor_InitMotorGroup(&Motor_Robstride06_Group7, Motor_TYPE_RS06, 1, &hfdcan2, NULL, 0x11);
    Motor_InitMotor(&Motor_Robstride06_Motor7, Motor_TYPE_RS06, 0x11, 1.0f, RS06_encoder_callback);
    Motor_Robstride06_Group7.motor_handle[0] = &Motor_Robstride06_Motor7;

    // RS06 电机 8 (ID: 0x12 / 18)
    Motor_groupHandle[17] = &Motor_Robstride06_Group8;
    Motor_InitMotorGroup(&Motor_Robstride06_Group8, Motor_TYPE_RS06, 1, &hfdcan2, NULL, 0x12);
    Motor_InitMotor(&Motor_Robstride06_Motor8, Motor_TYPE_RS06, 0x12, 1.0f, RS06_encoder_callback);
    Motor_Robstride06_Group8.motor_handle[0] = &Motor_Robstride06_Motor8;

    // RS06 电机 9 (ID: 0x13 / 19)
    Motor_groupHandle[18] = &Motor_Robstride06_Group9;
    Motor_InitMotorGroup(&Motor_Robstride06_Group9, Motor_TYPE_RS06, 1, &hfdcan2, NULL, 0x13);
    Motor_InitMotor(&Motor_Robstride06_Motor9, Motor_TYPE_RS06, 0x13, 1.0f, RS06_encoder_callback);
    Motor_Robstride06_Group9.motor_handle[0] = &Motor_Robstride06_Motor9;

    // RS06 电机 10 (ID: 0x14 / 20)
    Motor_groupHandle[19] = &Motor_Robstride06_Group10;
    Motor_InitMotorGroup(&Motor_Robstride06_Group10, Motor_TYPE_RS06, 1, &hfdcan2, NULL, 0x14);
    Motor_InitMotor(&Motor_Robstride06_Motor10, Motor_TYPE_RS06, 0x14, 1.0f, RS06_encoder_callback);
    Motor_Robstride06_Group10.motor_handle[0] = &Motor_Robstride06_Motor10;

    // RS06 电机 11 (ID: 0x15 / 21)
    Motor_groupHandle[20] = &Motor_Robstride06_Group11;
    Motor_InitMotorGroup(&Motor_Robstride06_Group11, Motor_TYPE_RS06, 1, &hfdcan2, NULL, 0x15);
    Motor_InitMotor(&Motor_Robstride06_Motor11, Motor_TYPE_RS06, 0x15, 1.0f, RS06_encoder_callback);
    Motor_Robstride06_Group11.motor_handle[0] = &Motor_Robstride06_Motor11;

    // RS06 电机 12 (ID: 0x16 / 22)
    Motor_groupHandle[21] = &Motor_Robstride06_Group12;
    Motor_InitMotorGroup(&Motor_Robstride06_Group12, Motor_TYPE_RS06, 1, &hfdcan2, NULL, 0x16);
    Motor_InitMotor(&Motor_Robstride06_Motor12, Motor_TYPE_RS06, 0x16, 1.0f, RS06_encoder_callback);
    Motor_Robstride06_Group12.motor_handle[0] = &Motor_Robstride06_Motor12;
}



/**
 * @brief  获取指定索引的电机组句柄
 * @param  group_idx 电机组索引(0~9对应10个电机组)
 * @retval 电机组句柄(NULL表示索引无效)
 */
Motor_MotorGroupTypeDef* Motor_GetMotorGroupHandle(uint8_t group_idx) {
    if (group_idx >= MOTOR_GROUP_NUM) {
        return NULL;
    }
    return Motor_groupHandle[group_idx];
}

/**
 * @brief  根据电机类型和ID获取电机句柄
 * @param  type 电机类型(DW4310/DM8006/DM8009)
 * @param  motor_id 电机ID(0x01~0x0A)
 * @retval 电机句柄(NULL表示未找到)
 */
Motor_MotorTypeDef* Motor_GetMotorHandle(Motor_MotorType_e type, uint8_t motor_id) {
    // 遍历所有电机组匹配类型和ID
    for (uint8_t i = 0; i < MOTOR_GROUP_NUM; i++) {
        if (Motor_groupHandle[i] == NULL) continue;
        if (Motor_groupHandle[i]->type == type && 
            Motor_groupHandle[i]->motor_handle[0]->id == motor_id) {
            return Motor_groupHandle[i]->motor_handle[0];
        }
    }
    return NULL;
}

/**
 * @brief  获取指定索引的电机数据指针
 * @param  group_idx 电机组索引(0~21对应21个电机组)
 * @retval 电机数据结构体指针(NULL表示索引无效)
 */
Motor_MotorTypeDef* Motor_GetMotorDataPtr(uint8_t group_idx) {
    if (group_idx >= MOTOR_GROUP_NUM || Motor_groupHandle[group_idx] == NULL) {
        return NULL;
    }
    return Motor_groupHandle[group_idx]->motor_handle[0];
}



/* 根据电机ID判断电机类型 */
Motor_MotorType_e FDCAN_GetMotorTypeById(uint8_t motor_id) {
    // ID分配规则（更新后）：
    // DM8006：1，3，6，8
    // DM8009：2，7
    // DM4310：4，5，9，10
	// RS：11-22
    switch(motor_id) {
        case 1:
        case 3:
        case 6:
        case 8:
            return Motor_TYPE_DM8006; // 匹配DM8006的ID
        case 2:
        case 7:
            return Motor_TYPE_DM8009; // 匹配DM8009的ID
        case 4:
        case 5:
        case 9:
        case 10:
            return Motor_TYPE_DM4310; // 匹配DM4310的ID
		case 11: case 12: case 13: case 14:
        case 15: case 16: case 17: case 18:
        case 19: case 20: case 21: case 22:
            return Motor_TYPE_RS06;   // 匹配RS06的ID
        default:
            return Motor_TYPE_MAX;    // 无效ID返回最大值（边界判断用）
    }
}


void Motor_DM_Basic_Output(Motor_MotorGroupTypeDef *pgroup , Motor_DMBasicCtrlEnum basic)
 {
	 
	 FDCAN_TxHeaderTypeDef tx_header;
	 
    if (pgroup == NULL) return;
    uint8_t txbuff[8];
    txbuff[0] = 0xFF;
    txbuff[1] = 0xFF;
    txbuff[2] = 0xFF;
    txbuff[3] = 0xFF;
    txbuff[4] = 0xFF;
    txbuff[5] = 0xFF;
    txbuff[6] = 0xFF;
	switch (basic)
	{
	case Motor_Enable:
	    txbuff[7] = 0xFC;
		break;

	case Motor_Disable:
	    txbuff[7] = 0xFD;
		break;

	case Motor_SaveInitpos:
        txbuff[7] = 0xFE;
		break;

	case Motor_Clearerr :
        txbuff[7] = 0xFB;
        break;
    }

  	HAL_Delay(500);
    //Can_SendMessage(pgroup->can_handle, &(pgroup->can_header), txbuff);
		FDCAN_InitTxHeader(&tx_header, pgroup->motor_handle[0]->id, 8);
		HAL_FDCAN_AddMessageToTxFifoQ(pgroup->can_handle, &tx_header, txbuff);
			
 }

void Motor_DM_Basic_Ctrl(Motor_DMBasicCtrlEnum basic)
{
    Motor_DM_Basic_Output(&Motor_DM4310_Group1,basic);
    Motor_DM_Basic_Output(&Motor_DM4310_Group2,basic);
    Motor_DM_Basic_Output(&Motor_DM4310_Group3,basic);
		Motor_DM_Basic_Output(&Motor_DM4310_Group4,basic);
    Motor_DM_Basic_Output(&Motor_DM8006_Group1,basic);
    Motor_DM_Basic_Output(&Motor_DM8006_Group2,basic);
		Motor_DM_Basic_Output(&Motor_DM8006_Group3,basic);
    Motor_DM_Basic_Output(&Motor_DM8006_Group4,basic);
    Motor_DM_Basic_Output(&Motor_DM8009_Group1,basic);
		Motor_DM_Basic_Output(&Motor_DM8009_Group2,basic);
}


void Motor_RS_Basic_Output(Motor_MotorGroupTypeDef *pgroup, Motor_DMBasicCtrlEnum basic)
{
    FDCAN_TxHeaderTypeDef tx_header;
    
    if (pgroup == NULL || pgroup->motor_handle[0] == NULL) return;
    
    // 根据RS电机协议，8字节数据区全部为 0x00
    uint8_t txbuff[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    uint8_t comm_type = 0;
    uint8_t master_id = 0xFD; // 默认主站 ID 
    uint8_t target_id = pgroup->motor_handle[0]->id;
    
    // 确定通信类型 (Communication Type)
    switch (basic)
    {
    case Motor_Enable:
        comm_type = 0x03; // 通信类型 3：电机使能运行
        break;
    case Motor_Disable:
        comm_type = 0x04; // 通信类型 4：电机停止运行 (RS06标准失能指令)
        break;
    default:
        return;
    }

    // 拼接 29 位扩展 ID
    // ID 格式: [通信类型 bit28~24] | [0x00 bit23~16] | [主站ID bit15~8] | [目标电机ID bit7~0]
    uint32_t ext_id = (comm_type << 24) | (master_id << 8) | target_id;

    // 建议：对于RS电机，这里适当加1~2ms延时防止CAN总线邮箱堵塞即可
    HAL_Delay(2); 

    // 初始化 CAN 发送头 (复用你原有的函数)
    FDCAN_InitTxHeader(&tx_header, ext_id, 8);
    
    // 【关键修改】RS电机使用的是29位扩展帧，必须覆盖默认的标准帧设置
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.Identifier = ext_id;

    HAL_FDCAN_AddMessageToTxFifoQ((FDCAN_HandleTypeDef *)pgroup->can_handle, &tx_header, txbuff);
}

/**
  * @brief  一键控制所有 12 个 RS06 电机
  * @param  basic: 控制指令枚举 (如 Motor_Enable)
  */
void Motor_RS_Basic_Ctrl(Motor_DMBasicCtrlEnum basic)
{
    // 依次向 1~12 号 RS 电机发送控制帧
    Motor_RS_Basic_Output(&Motor_Robstride06_Group1, basic);
    Motor_RS_Basic_Output(&Motor_Robstride06_Group2, basic);
    Motor_RS_Basic_Output(&Motor_Robstride06_Group3, basic);
    Motor_RS_Basic_Output(&Motor_Robstride06_Group4, basic);
    Motor_RS_Basic_Output(&Motor_Robstride06_Group5, basic);
    Motor_RS_Basic_Output(&Motor_Robstride06_Group6, basic);
    Motor_RS_Basic_Output(&Motor_Robstride06_Group7, basic);
    Motor_RS_Basic_Output(&Motor_Robstride06_Group8, basic);
    Motor_RS_Basic_Output(&Motor_Robstride06_Group9, basic);
    Motor_RS_Basic_Output(&Motor_Robstride06_Group10, basic);
    Motor_RS_Basic_Output(&Motor_Robstride06_Group11, basic);
    Motor_RS_Basic_Output(&Motor_Robstride06_Group12, basic);
    
    // 统一在这里进行较长延时，确保电机有足够时间响应使能状态
    if(basic == Motor_Enable) {
        HAL_Delay(200); 
    }
}
