/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : comm_receive.c
 *  Description  : This file is for receive communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:10:30
 *  LastEditTime : 2023-10-04 00:47:42
 */


#include "comm_common.h"
#include "comm_receive.h"
#include "alg_filter.h"
#include "stdlib.h"
#include "math.h"
#include "lib_buff.h"
#include "periph_motor.h"


// Platform data receive function
// 参数: buff 接收到的数据缓冲区指针
// 返回值: uint32_t 实际处理的数据长度
static uint32_t _set_T_L_Roboticarm_Data_(uint8_t *buff); 
static uint32_t _set_P_L_Roboticarm_Data_(uint8_t *buff); 
static uint32_t _set_M_L_Roboticarm_Data_(uint8_t *buff); 
static uint32_t _set_T_R_Roboticarm_Data_(uint8_t *buff); 
static uint32_t _set_P_R_Roboticarm_Data_(uint8_t *buff); 
static uint32_t _set_M_R_Roboticarm_Data_(uint8_t *buff); 
static uint32_t _set_T_Body_Data_(uint8_t *buff);
static uint32_t _set_P_Body_Data_(uint8_t *buff);
static uint32_t _set_M_Body_Data_(uint8_t *buff);



Filter_WindowTypeDef avg_filter;


//extern Motor_ControlCmd_t motor_ctrl_cmds[MOTOR_CTRL_CMD_NUM];

// Communication receive function table
// 每一个通信指令对应一个处理函数
// CommCmd_Receive 数组中的每一个元素都是一个 Comm_ReceiveEntry 结构体
// 该结构体包含一个函数指针 bus_func，指向具体的处理函数
Comm_ReceiveEntry CommCmd_Receive[Const_Comm_Receive_BUFF_SIZE] = {
  	{&_set_T_L_Roboticarm_Data_    },
		{&_set_P_L_Roboticarm_Data_    },
		{&_set_M_L_Roboticarm_Data_    },
		{&_set_T_R_Roboticarm_Data_    },
		{&_set_P_R_Roboticarm_Data_    },
		{&_set_M_R_Roboticarm_Data_    },
		{&_set_T_Body_Data_    },
		{&_set_P_Body_Data_    },
		{&_set_M_Body_Data_    },
};



static uint32_t _set_T_L_Roboticarm_Data_(uint8_t *buff) {

 
	ARM_P=LEFT;
	for(uint8_t i = 0; i < 5; i++)
    {
			
				Motor_ControlCmd_t* cmd = Motor_GetCtrlCmdPtr(i);
        cmd->channel = FDCAN1_CH;
        cmd->mode = MOTOR_MODE_TORQUE;               // 力矩模式（仅目标力矩）
        cmd->target_torque = buff2float(buff);
				buff=buff+4;
		}

  return 20; // 这个返回值用于告诉调用者实际处理了多少数据,方便后续的数据处理和管理

}


static uint32_t _set_P_L_Roboticarm_Data_(uint8_t *buff) {

 
	ARM_P=LEFT;
	for(uint8_t i = 0; i < 5; i++)
    {
				Motor_ControlCmd_t* cmd = Motor_GetCtrlCmdPtr(i);
        cmd->channel = FDCAN1_CH;
        cmd->mode = MOTOR_MODE_POSITION;               // 位置模式（目标位置+前馈力矩）
        cmd->target_pos = buff2float(buff);
				cmd->target_torque = buff2float(buff+4);
				buff=buff+8;
		}

  return 40; // 这个返回值用于告诉调用者实际处理了多少数据,方便后续的数据处理和管理

}


static uint32_t _set_M_L_Roboticarm_Data_(uint8_t *buff) {

 
	ARM_P=LEFT;
	for(uint8_t i = 0; i < 5; i++)
    {
				Motor_ControlCmd_t* cmd = Motor_GetCtrlCmdPtr(i);
        cmd->channel = FDCAN1_CH;
        cmd->mode = MOTOR_MODE_MIT;                  // MIT模式（位置+速度+力矩）
        cmd->target_pos = buff2float(buff);         // 每个电机目标角度
        cmd->target_vel = buff2float(buff+4);                      // 初始目标速度
        cmd->target_torque = buff2float(buff+8);               // 初始目标力矩
        cmd->kp = buff2float(buff+12);                           // 位置增益（0~500，根据电机调整）
        cmd->kd = buff2float(buff+16);  			// 速度增益（0~5，根据电机调整）
				buff=buff+20;
    }

  return 100; // 这个返回值用于告诉调用者实际处理了多少数据,方便后续的数据处理和管理

}

static uint32_t _set_T_R_Roboticarm_Data_(uint8_t *buff) {

 
	ARM_P=RIGHT;
	for(uint8_t i = 5; i < 10; i++)
    {
				Motor_ControlCmd_t* cmd = Motor_GetCtrlCmdPtr(i);
        cmd->channel = FDCAN1_CH;
        cmd->mode = MOTOR_MODE_TORQUE;               // 力矩模式（仅目标力矩）
        cmd->target_torque = buff2float(buff);
				buff=buff+4;
		}

  return 4; // 这个返回值用于告诉调用者实际处理了多少数据,方便后续的数据处理和管理

}


static uint32_t _set_P_R_Roboticarm_Data_(uint8_t *buff) {

 
	ARM_P=RIGHT;
	for(uint8_t i = 5; i < 10; i++)
    {
				Motor_ControlCmd_t* cmd = Motor_GetCtrlCmdPtr(i);
        cmd->channel = FDCAN1_CH;
        cmd->mode = MOTOR_MODE_POSITION;               // 位置模式（目标位置+前馈力矩）
        cmd->target_pos = buff2float(buff);
				cmd->target_torque = buff2float(buff+4);
				buff=buff+8;
		}

  return 40; // 这个返回值用于告诉调用者实际处理了多少数据,方便后续的数据处理和管理
  

}


static uint32_t _set_M_R_Roboticarm_Data_(uint8_t *buff) {

 
	ARM_P=RIGHT;
	for(uint8_t i = 5; i < 10; i++)
    {
				Motor_ControlCmd_t* cmd = Motor_GetCtrlCmdPtr(i);
        cmd->channel = FDCAN1_CH;
        cmd->mode = MOTOR_MODE_MIT;                  // MIT模式（位置+速度+力矩）
        cmd->target_pos = buff2float(buff);         // 每个电机目标角度
        cmd->target_vel = buff2float(buff+4);                      // 初始目标速度
        cmd->target_torque = buff2float(buff+8);               // 初始目标力矩
        cmd->kp = buff2float(buff+12);                           // 位置增益（0~500，根据电机调整）
        cmd->kd = buff2float(buff+16);  			// 速度增益（0~5，根据电机调整）
				buff=buff+20;
    }

  return 100; // 这个返回值用于告诉调用者实际处理了多少数据,方便后续的数据处理和管理

  
}


static uint32_t _set_T_Body_Data_(uint8_t *buff) {

 
	for(uint8_t i = 10; i < 22; i++)
    {
			
				Motor_ControlCmd_t* cmd = Motor_GetCtrlCmdPtr(i);
        cmd->channel = FDCAN2_CH;
        cmd->mode = MOTOR_MODE_TORQUE;               // 力矩模式（仅目标力矩）
        cmd->target_torque = buff2float(buff);
				buff=buff+4;
		}

  return 48; // 这个返回值用于告诉调用者实际处理了多少数据,方便后续的数据处理和管理

}


static uint32_t _set_P_Body_Data_(uint8_t *buff) {

 
	for(uint8_t i = 10; i < 22; i++)
    {
				Motor_ControlCmd_t* cmd = Motor_GetCtrlCmdPtr(i);
        cmd->channel = FDCAN2_CH;
        cmd->mode = MOTOR_MODE_POSITION;               // 位置模式（目标位置+前馈力矩）
        cmd->target_pos = buff2float(buff);
				cmd->target_torque = buff2float(buff+4);
				buff=buff+8;
		}

  return 96; // 这个返回值用于告诉调用者实际处理了多少数据,方便后续的数据处理和管理

}


static uint32_t _set_M_Body_Data_(uint8_t *buff) {

 
	for(uint8_t i = 10; i < 22; i++)
    {
				Motor_ControlCmd_t* cmd = Motor_GetCtrlCmdPtr(i);
        cmd->channel = FDCAN2_CH;
        cmd->mode = MOTOR_MODE_MIT;                  // MIT模式（位置+速度+力矩）
        cmd->target_pos = buff2float(buff);         // 每个电机目标角度
        cmd->target_vel = buff2float(buff+4);                      // 初始目标速度
        cmd->target_torque = buff2float(buff+8);               // 初始目标力矩
        cmd->kp = buff2float(buff+12);                           // 位置增益（0~500，根据电机调整）
        cmd->kd = buff2float(buff+16);  			// 速度增益（0~5，根据电机调整）
				buff=buff+20;
    }

  return 240; // 这个返回值用于告诉调用者实际处理了多少数据,方便后续的数据处理和管理

}



void init_average_filter(void) {
    for (int i = 0; i < MAX_LENGTH; i++) {
        avg_filter.val[i] = 0.0f;
    }
    avg_filter.sum = 0.0f;
}
