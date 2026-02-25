/*
 *  Project      : Polaris Robot
 *  FilePath     : periph_motor.h
 *  Description  : 电机外设驱动（仅保留DM系列电机）
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16
 *  LastEditTime : 2024-01-19
 */
#ifndef __PERIPH_MOTOR_H__
#define __PERIPH_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "util_can.h"
#include "module_motor_cmd.h"

typedef enum
{
  Motor_Enable = 1,
  Motor_Disable, 
  Motor_SaveInitpos,
  Motor_Clearerr 
} Motor_DMBasicCtrlEnum;


/* 电机类型枚举 - 仅保留DM系列 */
typedef enum {
    Motor_TYPE_DM4310 = 0,   // DM4310电机
    Motor_TYPE_DM8006,       // DM8006电机
    Motor_TYPE_DM8009,       // DM8009电机
		Motor_TYPE_RS06,         // RS06电机
    Motor_TYPE_MAX           // 电机类型总数
} Motor_MotorType_e;

/* 电机编码器结构体 */
typedef struct {
    float angle;        // 角度(rad)
    float speed;        // 速度(rad/s)
    float torque;       // 力矩(Nm)
    uint8_t temp;       // 温度(℃)
    uint8_t err;        // 错误码
} Motor_EncoderTypeDef;

/* 电机结构体 */
typedef struct _motor_type {
    Motor_MotorType_e type;          // 电机类型
    uint8_t id;                      // 电机ID
    float reduce_ratio;              // 减速比
    Motor_EncoderTypeDef encoder;    // 编码器数据
    uint8_t init;                    // 初始化标志
    uint8_t is_online;               // 在线标志
    uint32_t watchdog;               // 看门狗计数
    uint32_t last_update_tick;       // 最后更新时间
    float update_dt;                 // 更新间隔(s)
    void (*EncoderCallback)(struct _motor_type*, uint8_t rxbuff[], uint32_t len); // 编码器回调 //pmotor
} Motor_MotorTypeDef;

typedef void (*EncoderCallback)(Motor_MotorTypeDef*, uint8_t[], uint32_t);


/* 电机组结构体 */
typedef struct {
    Motor_MotorType_e type;                  // 电机组类型
    uint8_t motor_num;                       // 电机数量（固定为1）
    void *can_handle;                        // FDCAN句柄（固定为hfdcan1）
    uint8_t *tx_id;                          // 发送ID数组
    Motor_MotorTypeDef *motor_handle[1];     // 电机句柄数组（仅1个电机）
} Motor_MotorGroupTypeDef;


/* 获取电机句柄相关声明 */
Motor_MotorGroupTypeDef* Motor_GetMotorGroupHandle(uint8_t group_idx);
Motor_MotorTypeDef* Motor_GetMotorHandle(Motor_MotorType_e type, uint8_t motor_id);
Motor_MotorTypeDef* Motor_GetMotorDataPtr(uint8_t group_idx); 
// 新增：根据电机ID判断电机类型
Motor_MotorType_e FDCAN_GetMotorTypeById(uint8_t motor_id);

/* 全局宏定义 */
#define MOTOR_GROUP_NUM 22  // 最大电机组数量
#define DM_MOTOR_GROUP_NUM 10
#define RS_MOTOR_GROUP_NUM 12

/* 全局变量声明  */
extern Motor_MotorGroupTypeDef *Motor_groupHandle[MOTOR_GROUP_NUM];

// DM4310（4个）
extern Motor_MotorGroupTypeDef Motor_DM4310_Group1;
extern Motor_MotorGroupTypeDef Motor_DM4310_Group2;
extern Motor_MotorGroupTypeDef Motor_DM4310_Group3;
extern Motor_MotorGroupTypeDef Motor_DM4310_Group4;
extern Motor_MotorTypeDef Motor_DM4310_Motor1;
extern Motor_MotorTypeDef Motor_DM4310_Motor2;
extern Motor_MotorTypeDef Motor_DM4310_Motor3;
extern Motor_MotorTypeDef Motor_DM4310_Motor4;

// DM8006（4个）
extern Motor_MotorGroupTypeDef Motor_DM8006_Group1;
extern Motor_MotorGroupTypeDef Motor_DM8006_Group2;
extern Motor_MotorGroupTypeDef Motor_DM8006_Group3;
extern Motor_MotorGroupTypeDef Motor_DM8006_Group4;
extern Motor_MotorTypeDef Motor_DM8006_Motor1;
extern Motor_MotorTypeDef Motor_DM8006_Motor2;
extern Motor_MotorTypeDef Motor_DM8006_Motor3;
extern Motor_MotorTypeDef Motor_DM8006_Motor4;

// DM8009（2个）
extern Motor_MotorGroupTypeDef Motor_DM8009_Group1;
extern Motor_MotorGroupTypeDef Motor_DM8009_Group2;
extern Motor_MotorTypeDef Motor_DM8009_Motor1;
extern Motor_MotorTypeDef Motor_DM8009_Motor2;

//RS06 (12个)
extern Motor_MotorGroupTypeDef Motor_Robstride06_Group1;
extern Motor_MotorGroupTypeDef Motor_Robstride06_Group2;
extern Motor_MotorGroupTypeDef Motor_Robstride06_Group3;
extern Motor_MotorGroupTypeDef Motor_Robstride06_Group4;
extern Motor_MotorGroupTypeDef Motor_Robstride06_Group5;
extern Motor_MotorGroupTypeDef Motor_Robstride06_Group6;
extern Motor_MotorGroupTypeDef Motor_Robstride06_Group7;
extern Motor_MotorGroupTypeDef Motor_Robstride06_Group8;
extern Motor_MotorGroupTypeDef Motor_Robstride06_Group9;
extern Motor_MotorGroupTypeDef Motor_Robstride06_Group10;
extern Motor_MotorGroupTypeDef Motor_Robstride06_Group11;
extern Motor_MotorGroupTypeDef Motor_Robstride06_Group12;

extern Motor_MotorTypeDef Motor_Robstride06_Motor1;
extern Motor_MotorTypeDef Motor_Robstride06_Motor2;
extern Motor_MotorTypeDef Motor_Robstride06_Motor3;
extern Motor_MotorTypeDef Motor_Robstride06_Motor4;
extern Motor_MotorTypeDef Motor_Robstride06_Motor5;
extern Motor_MotorTypeDef Motor_Robstride06_Motor6;
extern Motor_MotorTypeDef Motor_Robstride06_Motor7;
extern Motor_MotorTypeDef Motor_Robstride06_Motor8;
extern Motor_MotorTypeDef Motor_Robstride06_Motor9;
extern Motor_MotorTypeDef Motor_Robstride06_Motor10;
extern Motor_MotorTypeDef Motor_Robstride06_Motor11;
extern Motor_MotorTypeDef Motor_Robstride06_Motor12;

/* 函数声明 */
void Motor_InitAllMotors(void);
void Motor_InitMotorGroup(Motor_MotorGroupTypeDef *pgroup, Motor_MotorType_e type, uint8_t motor_num, void *can_handle, uint8_t *tx_id, uint8_t rx_id);
void Motor_InitMotor(Motor_MotorTypeDef *pmotor, Motor_MotorType_e type, uint8_t id, float reduce_ratio, void (*EncoderCallback)(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len));
void Motor_SetControlMode(Motor_MotorGroupTypeDef* pgroup, Motor_ControlMode_e mode, Motor_ControlCmd_t cmd);
void DM4310_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void DM8006_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void DM8009_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void RS06_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void Motor_DM_Basic_Ctrl(Motor_DMBasicCtrlEnum basic);
void Motor_RS_Basic_Ctrl(Motor_DMBasicCtrlEnum basic);


#ifdef __cplusplus
}
#endif

#endif /* __PERIPH_MOTOR_H__ */
