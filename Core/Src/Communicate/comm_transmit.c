/*
 *  Project      : Polaris Robot
 *  FilePath     : comm_transmit.c
 *  Description  : 电机数据打包（适配原有comm_common发送逻辑）
 *  LastEditors  : Polaris
 *  Date         : 2024-0X-0X
 */
#include "comm_common.h"
#include "comm_transmit.h"
#include "periph_motor.h"
#include "string.h"
#include "lib_buff.h"
#include "sys_const.h"


static uint32_t _pack_L_Arm_Motor_Data_(uint8_t *buff);
static uint32_t _pack_R_Arm_Motor_Data_(uint8_t *buff);
static uint32_t _pack_Body_Motor_Data_(uint8_t *buff);


// 发送函数表（与原有comm_common的CommCmd_Send匹配）
Comm_SendEntry CommCmd_Send[Const_Comm_Transmit_BUFF_SIZE] = {
    
		{_pack_Body_Motor_Data_},			//索引0： body数据
		{_pack_L_Arm_Motor_Data_},  // 索引1：左臂数据
    {_pack_R_Arm_Motor_Data_},   // 索引2：右臂数据
		
};


static uint32_t _pack_Body_Motor_Data_(uint8_t *buff) {
    uint32_t len = 0;
    

    //  遍历11-22号电机（核心：对齐Remote_GetRemoteDataPtr风格）
    for (uint8_t motor_id = 11; motor_id <= 22; motor_id++) {
        // 一行获取电机指针（和Remote_GetRemoteDataPtr写法一致）
        Motor_MotorTypeDef* motor = Motor_GetMotorDataPtr(motor_id - 1); // 索引0对应ID1

        // 3. 填充位置/速度/力矩（离线填0）
        if (motor != NULL && motor->is_online) {
            float2buff(motor->encoder.angle, buff + len);   // 位置(4字节)
            len += 4;
            float2buff(motor->encoder.speed, buff + len);   // 速度(4字节)
            len += 4;
            float2buff(motor->encoder.torque, buff + len);  // 力矩(4字节)
            len += 4;
        } else {
            memset(buff + len, 0, 12); // 离线填0
            len += 12;
        }
    }
    return len;
}


/**
 * @brief 打包左臂电机数据（1-5号）
 * @param buff 输出缓冲区
 * @return 打包字节数
 */
static uint32_t _pack_L_Arm_Motor_Data_(uint8_t *buff) {
    uint32_t len = 0;

    // 遍历1-5号电机（核心：对齐Remote_GetRemoteDataPtr风格）
    for (uint8_t motor_id = 1; motor_id <= 5; motor_id++) {
        // 一行获取电机指针（和Remote_GetRemoteDataPtr写法一致）
        Motor_MotorTypeDef* motor = Motor_GetMotorDataPtr(motor_id - 1); // 索引0对应ID1

        // 填充位置/速度/力矩（离线填0）
        if (motor != NULL && motor->is_online) {
            float2buff(motor->encoder.angle, buff + len);   // 位置(4字节)
            len += 4;
            float2buff(motor->encoder.speed, buff + len);   // 速度(4字节)
            len += 4;
            float2buff(motor->encoder.torque, buff + len);  // 力矩(4字节)
            len += 4;
        } else {
            memset(buff + len, 0, 12); // 离线填0
            len += 12;
        }
    }
    return len;
}

/**
 * @brief 打包右臂电机数据（6-10号）
 * @param buff 输出缓冲区
 * @return 打包字节数
 */
static uint32_t _pack_R_Arm_Motor_Data_(uint8_t *buff) {
    uint32_t len = 0;
  
    // 遍历6-10号电机
    for (uint8_t motor_id = 6; motor_id <= 10; motor_id++) {
        // 一行获取电机指针（风格统一）
        Motor_MotorTypeDef* motor = Motor_GetMotorDataPtr(motor_id - 1); // 索引5对应ID6

        //填充位置/速度/力矩（离线填0）
        if (motor != NULL && motor->is_online) {
            float2buff(motor->encoder.angle, buff + len);   // 位置(4字节)
            len += 4;
            float2buff(motor->encoder.speed, buff + len);   // 速度(4字节)
            len += 4;
            float2buff(motor->encoder.torque, buff + len);  // 力矩(4字节)
            len += 4;
        } else {
            memset(buff + len, 0, 12); // 离线填0
            len += 12;
        }
    }
    return len;
}