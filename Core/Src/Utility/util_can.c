/*
 *  Project      : Polaris Robot
 *  FilePath     : util_can.c
 *  Description  : FDCAN通信驱动实现
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16
 *  LastEditTime : 2024-01-19
 */
#include "util_can.h"
#include "periph_motor.h"
#include "sys_dwt.h"
#include "lib_buff.h"
#include "sys_const.h"
#include <string.h>

/* 私有全局变量 */
static FDCAN_HandleTypeDef* g_fdcan_handle_map[FDCAN_CH_MAX] = {NULL};
// 电机参数范围（按类型区分：索引0=4310,1=8006,2=8009）
static const float DM_P_MIN[] = {-12.5f, -12.5f, -12.5f};  // 位置最小值(rad)
static const float DM_P_MAX[] = {12.5f, 12.5f, 12.5f};     // 位置最大值(rad)
static const float DM_V_MIN[] = {-30.0f, -45.0f, -30.0f};  // 速度最小值(rad/s)
static const float DM_V_MAX[] = {30.0f, 45.0f, 30.0f};     // 速度最大值(rad/s)
static const float DM_T_MIN[] = {-28.0f, -54.0f, -10.0f};  // 力矩最小值(Nm)
static const float DM_T_MAX[] = {28.0f, 54.0f, 10.0f};     // 力矩最大值(Nm)


/* 私有函数声明 */
static uint8_t FDCAN_CheckChannelValid(FDCAN_Channel_e ch);


/* 错误处理函数 */
void FDCAN_ErrorHandler(uint32_t err_code) {
    // 错误等级扩展LED闪烁/日志提示
    switch (err_code) {
        case 1: // 初始化失败
            while(1) { HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0); HAL_Delay(200); }
        case 2: // 发送失败
            while(1) { HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0); HAL_Delay(500); }
        case 3: // 接收失败
            while(1) { HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0); HAL_Delay(1000); }
        default: break;
    }
    while (1); // 死循环挂起
}

/* 初始化发送头结构体 */
void FDCAN_InitTxHeader(FDCAN_TxHeaderTypeDef *pheader, uint32_t std_id, uint32_t dlc) {
    if (pheader == NULL || dlc > CONST_FDCAN_MAX_DATA_LEN) {
        FDCAN_ErrorHandler(1);
        return;
    }
    // 清空发送头结构体
    memset(pheader, 0, sizeof(FDCAN_TxHeaderTypeDef));
    
    // 标准CAN协议配置（关闭FD特性）
    pheader->Identifier = std_id;                    // 标准ID
    pheader->IdType = FDCAN_STANDARD_ID;             // 标准ID模式
    pheader->TxFrameType = FDCAN_DATA_FRAME;         // 数据帧
    pheader->DataLength = FDCAN_DLC_BYTES_8;    // DLC配置（左移16位）
    pheader->BitRateSwitch = FDCAN_BRS_OFF;          // 关闭比特率切换
    pheader->FDFormat = FDCAN_CLASSIC_CAN;           // 强制标准CAN格式
    pheader->ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 错误状态指示
    pheader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;// 禁用发送事件FIFO
    pheader->MessageMarker = 0;                      // 消息标记
}

/* 初始化 FDCAN 扩展帧发送头结构体 (供 Robstride 等使用) */
void FDCAN_InitTxHeader_Extended(FDCAN_TxHeaderTypeDef *pheader, uint32_t ext_id, uint32_t dlc) {
    if (pheader == NULL) {
        // 可以在这里调用你的 ErrorHandler
        return;
    }
    
    // 清空发送头结构体
    memset(pheader, 0, sizeof(FDCAN_TxHeaderTypeDef));
    
    // 标准 CAN 协议配置（关闭 FD 特性，完全兼容 Robstride 需求）
    pheader->Identifier = ext_id;                    // 传入 29 位扩展 ID
    pheader->IdType = FDCAN_EXTENDED_ID;             // 【核心区别】配置为扩展帧模式
    pheader->TxFrameType = FDCAN_DATA_FRAME;         // 数据帧
    pheader->DataLength = dlc;                       // 动态传入的数据长度宏 (如 FDCAN_DLC_BYTES_8)
    pheader->BitRateSwitch = FDCAN_BRS_OFF;          // 关闭比特率切换
    pheader->FDFormat = FDCAN_CLASSIC_CAN;           // 强制标准 CAN 格式
    pheader->ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 错误状态指示
    pheader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;// 禁用发送事件 FIFO
    pheader->MessageMarker = 0;                      // 消息标记
}

/* 初始化滤波器并启动FDCAN */
uint8_t FDCAN_InitFilterAndStart(FDCAN_HandleTypeDef* phfdcan, FDCAN_Channel_e ch) {
    FDCAN_FilterTypeDef s_filter_config;
    uint32_t hal_ret;

    // 参数合法性检查
    if (phfdcan == NULL || !FDCAN_CheckChannelValid(ch)) return 1;
    g_fdcan_handle_map[ch] = phfdcan; // 存储句柄-通道映射

    /* 滤波器配置 */
    s_filter_config.IdType = FDCAN_STANDARD_ID;             // 标准CAN标准ID
    s_filter_config.FilterType = FDCAN_FILTER_MASK;         // 掩码过滤模式
    s_filter_config.FilterID1 = 0x000;                      // 过滤ID（全匹配）
    s_filter_config.FilterID2 = 0x000;                      // 掩码0=全接收
    s_filter_config.FilterIndex = 0;                        // 滤波器索引0
    s_filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  // 映射到FIFO0

    // 配置滤波器
    hal_ret = HAL_FDCAN_ConfigFilter(phfdcan, &s_filter_config);
    if (hal_ret != HAL_OK) return 1;

    // 启动FDCAN
    hal_ret = HAL_FDCAN_Start(phfdcan);
    if (hal_ret != HAL_OK) return 1;

    // 启用接收中断（所有电机统一用FIFO0）
    hal_ret = HAL_FDCAN_ActivateNotification(phfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    return (hal_ret == HAL_OK) ? 0 : 1;
}

/* MIT模式指令发送（帧格式：ID+P(16bit)+V(12bit)+KP(12bit)+KD(12bit)+T(12bit)） */
uint8_t FDCAN_SendMITCmd(FDCAN_HandleTypeDef* phfdcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torque) {
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    //uint32_t tx_mailbox;
    FDCAN_Channel_e ch = FDCAN_GetChannelByHandle(phfdcan);
    Motor_MotorType_e motor_type = FDCAN_GetMotorTypeById(motor_id);
    uint8_t type_idx = 0;

    // 确定电机类型索引
    switch(motor_type) {
        case Motor_TYPE_DM4310: type_idx = 0; break;
        case Motor_TYPE_DM8006: type_idx = 1; break;
        case Motor_TYPE_DM8009: type_idx = 2; break;
        default: return 2;
    }

    // 限制参数在合理范围
    pos = (pos < DM_P_MIN[type_idx]) ? DM_P_MIN[type_idx] : (pos > DM_P_MAX[type_idx]) ? DM_P_MAX[type_idx] : pos;
    vel = (vel < DM_V_MIN[type_idx]) ? DM_V_MIN[type_idx] : (vel > DM_V_MAX[type_idx]) ? DM_V_MAX[type_idx] : vel;
    torque = (torque < DM_T_MIN[type_idx]) ? DM_T_MIN[type_idx] : (torque > DM_T_MAX[type_idx]) ? DM_T_MAX[type_idx] : torque;
    kp = (kp < 0) ? 0 : (kp > 500) ? 500 : kp;
    kd = (kd < 0) ? 0 : (kd > 5) ? 5 : kd;

    // 浮点转无符号整数（按位宽映射）
    uint16_t pos_uint = float_to_uint(pos, DM_P_MIN[type_idx], DM_P_MAX[type_idx], 16);
    uint16_t vel_uint = float_to_uint(vel, DM_V_MIN[type_idx], DM_V_MAX[type_idx], 12);
    uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
    uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
    uint16_t t_uint = float_to_uint(torque, DM_T_MIN[type_idx], DM_T_MAX[type_idx], 12);

    // 按协议格式封装8字节数据
    tx_data[0] = (pos_uint >> 8) & 0xFF;
    tx_data[1] = pos_uint & 0xFF;
    tx_data[2] = (vel_uint >> 4) & 0xFF;
    tx_data[3] = ((vel_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F);
    tx_data[4] = kp_uint & 0xFF;
    tx_data[5] = (kd_uint >> 4) & 0xFF;
    tx_data[6] = ((kd_uint & 0x0F) << 4) | ((t_uint >> 8) & 0x0F);
    tx_data[7] = t_uint & 0xFF;

    // 初始化发送头并发送
    FDCAN_InitTxHeader(&tx_header, motor_id, 8);
    return (HAL_FDCAN_AddMessageToTxFifoQ(phfdcan, &tx_header, tx_data) == HAL_OK) ? 0 : 2;
}

/* 位置模式指令发送（复用MIT模式，KP/KD/V设为0） */
uint8_t FDCAN_SendPosCmd(FDCAN_HandleTypeDef* phfdcan, uint16_t motor_id, float pos, float torque) {
    return FDCAN_SendMITCmd(phfdcan, motor_id, pos, 0, 0, 0, torque);
}

/* 力矩模式指令发送（复用MIT模式，KP/KD设为0） */
uint8_t FDCAN_SendTorqueCmd(FDCAN_HandleTypeDef* phfdcan, uint16_t motor_id, float torque) {
    return FDCAN_SendMITCmd(phfdcan, motor_id, 0, 0, 0, 0, torque);
}

// RS06 MIT模式指令发送
uint8_t FDCAN_SendMITCmd_RS(FDCAN_HandleTypeDef* phfdcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torque) {
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};

    // 1. 限制参数在合理范围
    pos    = (pos < RS_P_MIN) ? RS_P_MIN : (pos > RS_P_MAX) ? RS_P_MAX : pos;
    vel    = (vel < RS_V_MIN) ? RS_V_MIN : (vel > RS_V_MAX) ? RS_V_MAX : vel;
    torque = (torque < RS_T_MIN) ? RS_T_MIN : (torque > RS_T_MAX) ? RS_T_MAX : torque;
    kp     = (kp < RS_KP_MIN) ? RS_KP_MIN : (kp > RS_KP_MAX) ? RS_KP_MAX : kp;
    kd     = (kd < RS_KD_MIN) ? RS_KD_MIN : (kd > RS_KD_MAX) ? RS_KD_MAX : kd;

    // 2. 浮点转无符号整数（注意：Robstride协议全部是 16 位宽！）
    uint16_t pos_uint = float_to_uint(pos, RS_P_MIN, RS_P_MAX, 16);
    uint16_t vel_uint = float_to_uint(vel, RS_V_MIN, RS_V_MAX, 16);
    uint16_t kp_uint  = float_to_uint(kp, RS_KP_MIN, RS_KP_MAX, 16);
    uint16_t kd_uint  = float_to_uint(kd, RS_KD_MIN, RS_KD_MAX, 16);
    uint16_t t_uint   = float_to_uint(torque, RS_T_MIN, RS_T_MAX, 16);

    // 3. 封装 8 字节数据域 (高字节在前 Big-Endian，完美2字节对齐)
    tx_data[0] = (pos_uint >> 8) & 0xFF;  // P 高八位
    tx_data[1] = pos_uint & 0xFF;         // P 低八位
    tx_data[2] = (vel_uint >> 8) & 0xFF;  // V 高八位
    tx_data[3] = vel_uint & 0xFF;         // V 低八位
    tx_data[4] = (kp_uint >> 8) & 0xFF;   // Kp 高八位
    tx_data[5] = kp_uint & 0xFF;          // Kp 低八位
    tx_data[6] = (kd_uint >> 8) & 0xFF;   // Kd 高八位
    tx_data[7] = kd_uint & 0xFF;          // Kd 低八位

    // 4. 组装 29 位扩展 ID
    // 0x01 是运控模式的指令码 (Communication_Type_MotionControl)
    uint32_t ext_id = ((uint32_t)0x01 << 24) | ((uint32_t)t_uint << 8) | ((uint32_t)motor_id);

    // 5. 调用上一轮为你写的扩展帧初始化函数准备头
    // (如果你把 FDCAN_InitTxHeader_Extended 写在 util_can 里面，这里直接调用)
    FDCAN_InitTxHeader_Extended(&tx_header, ext_id, FDCAN_DLC_BYTES_8);

    // 6. 推入发送队列
    return (HAL_FDCAN_AddMessageToTxFifoQ(phfdcan, &tx_header, tx_data) == HAL_OK) ? 0 : 2;
}


/* RS位置模式指令发送（复用MIT模式，KP/KD/V设为0） */
uint8_t FDCAN_SendPosCmd_RS(FDCAN_HandleTypeDef* phfdcan, uint16_t motor_id, float pos, float torque) {
    return FDCAN_SendMITCmd_RS(phfdcan, motor_id, pos, 0, 0, 0, torque);
}

/* RS力矩模式指令发送（复用MIT模式，KP/KD设为0） */
uint8_t FDCAN_SendTorqueCmd_RS(FDCAN_HandleTypeDef* phfdcan, uint16_t motor_id, float torque) {
    return FDCAN_SendMITCmd_RS(phfdcan, motor_id, 0, 0, 0, 0, torque);
}	

/* FIFO0接收回调（DM电机统一处理） */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* phfdcan, uint32_t RxFifo0ITs) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[CONST_FDCAN_MAX_DATA_LEN] = {0};

    // 读取FIFO0数据
    if (HAL_FDCAN_GetRxMessage(phfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        FDCAN_ErrorHandler(3);
        return;
    }

    // 转发到统一接收回调（固定通道为FDCAN1_CH）
    FDCAN_RxMessageCallback(phfdcan, &rx_header, rx_data, FDCAN1_CH);
}


/* FIFO1接收回调（RS电机统一处理） */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* phfdcan, uint32_t RxFifo1ITs) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[CONST_FDCAN_MAX_DATA_LEN] = {0};

    //安全检查：确认这个中断确实是 FDCAN2 触发的
    if (phfdcan->Instance == FDCAN2) {
        //从 FIFO1 读取数据
        if (HAL_FDCAN_GetRxMessage(phfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK) {
            FDCAN_ErrorHandler(3); // FIFO1 读取错误
            return;
        }

        // 转发到统一接收回调（固定通道为FDCAN2_CH）
        FDCAN_RxMessageCallback(phfdcan, &rx_header, rx_data, FDCAN2_CH);
    }
}


/* DM电机统一接收消息解析回调 */
void FDCAN_RxMessageCallback(FDCAN_HandleTypeDef* phfdcan, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[], FDCAN_Channel_e ch) {
    UNUSED(phfdcan);
    //UNUSED(ch);

	
		//1.处理DM电机
	if (ch == FDCAN1_CH)
	{
    // 解析电机数据（协议格式）
    uint8_t motor_id = rx_data[0] & 0x0F;          // 电机ID（低4位）
    uint8_t err = (rx_data[0] >> 4) & 0x0F;        // 错误码（高4位）
    Motor_MotorType_e motor_type = FDCAN_GetMotorTypeById(motor_id);
    uint8_t type_idx = 0;

    // 确定电机类型索引
    switch(motor_type) {
        case Motor_TYPE_DM4310: type_idx = 0; break;
        case Motor_TYPE_DM8006: type_idx = 1; break;
        case Motor_TYPE_DM8009: type_idx = 2; break;
        default: return;
    }

    // 解析位置、速度、力矩
    float pos = uint_to_float((rx_data[1] << 8) | rx_data[2], DM_P_MIN[type_idx], DM_P_MAX[type_idx], 16); // 位置
    float vel = uint_to_float(((rx_data[3] << 4) | (rx_data[4] >> 4)), DM_V_MIN[type_idx], DM_V_MAX[type_idx], 12); // 速度
    float torque = uint_to_float(((rx_data[4] & 0x0F) << 8) | rx_data[5], DM_T_MIN[type_idx], DM_T_MAX[type_idx], 12); // 力矩
    uint8_t temp_mos = rx_data[6];                 // MOS温度
    uint8_t temp_rotor = rx_data[7];               // 转子温度

    // 更新电机状态（遍历所有电机组）
    for (int i = 0; i < DM_MOTOR_GROUP_NUM; i++) {
        if (Motor_groupHandle[i] && Motor_groupHandle[i]->can_handle == phfdcan) {
            for (int j = 0; j < Motor_groupHandle[i]->motor_num; j++) {
                if (Motor_groupHandle[i]->motor_handle[j]->id == motor_id) {
                    Motor_MotorTypeDef* motor = Motor_groupHandle[i]->motor_handle[j];
                    // 更新编码器数据
                    motor->encoder.angle = pos;
                    motor->encoder.speed = vel;
                    motor->encoder.torque = torque;
                    motor->encoder.temp = temp_rotor;
                    motor->encoder.err = err;
                    // 更新状态标志
                    motor->is_online = (err == 0) ? 1 : 0; // 0=无故障
                    motor->watchdog = 0;                   // 喂看门狗
                    motor->last_update_tick = DWT_GetTimeline_ms(); // 更新时间戳
                    motor->update_dt = DWT_GetDeltaT(&motor->last_update_tick); // 更新间隔
                    break;
                }
            }
        }
    }
	}
	
	else if(ch == FDCAN2_CH)
	{
		uint32_t ext_id = rx_header->Identifier;

        // A. 解析 29 位 CAN ID 里的状态信息
        uint8_t cmd_type = (ext_id >> 24) & 0x1F;
        uint8_t error    = (ext_id >> 16) & 0x3F;
        uint8_t motor_id = (ext_id >> 8) & 0xFF;

        // 0x02 是电机状态反馈指令码
        if (cmd_type != 0x02) return; 

        // B. 解析 8 字节数据域 (16 bit，高字节在前)
        uint16_t pos_uint  = ((uint16_t)rx_data[0] << 8) | rx_data[1];
        uint16_t vel_uint  = ((uint16_t)rx_data[2] << 8) | rx_data[3];
        uint16_t t_uint    = ((uint16_t)rx_data[4] << 8) | rx_data[5];
        uint16_t temp_uint = ((uint16_t)rx_data[6] << 8) | rx_data[7];

        // C. 还原成物理量 (这里假设 type_idx = 0 为基础的 RS06 型号)
        float pos    = uint_to_float(pos_uint, RS_P_MIN, RS_P_MAX, 16);
        float vel    = uint_to_float(vel_uint, RS_V_MIN, RS_V_MAX, 16);
        float torque = uint_to_float(t_uint,   RS_T_MIN, RS_T_MAX, 16);
        float temp   = (float)temp_uint / 10.0f; 

        // D. 核心修改：遍历电机组，寻找对应的 RS 电机对象并更新！
        for (int i = 0; i < RS_MOTOR_GROUP_NUM; i++) {
            if (Motor_groupHandle[i] && Motor_groupHandle[i]->can_handle == phfdcan) {
                for (int j = 0; j < Motor_groupHandle[i]->motor_num; j++) {
                    
                    // 校验：不仅要 ID 对得上，还得确保它是 RS 电机 (假设你在枚举里加了 Motor_TYPE_RS06)
                    if (Motor_groupHandle[i]->motor_handle[j]->id == motor_id && 
                        Motor_groupHandle[i]->motor_handle[j]->type == Motor_TYPE_RS06) {
                        
                        Motor_MotorTypeDef* motor = Motor_groupHandle[i]->motor_handle[j];
                        
                        // 完美复用 encoder 结构体存储解析数据
                        motor->encoder.angle  = pos;
                        motor->encoder.speed  = vel;
                        motor->encoder.torque = torque;
                        motor->encoder.temp   = temp;
                        motor->encoder.err    = error;
                        
                        // 完美复用状态标志位与时间戳逻辑
                        motor->is_online = (error == 0) ? 1 : 0; 
                        motor->watchdog  = 0;                   
                        motor->last_update_tick = DWT_GetTimeline_ms(); 
                        motor->update_dt = DWT_GetDeltaT(&motor->last_update_tick); 
                        
                        return; // 找到并更新完毕，直接退出
										}
									}

								}
							}
						}
					}

/* 获取句柄对应通道 */
FDCAN_Channel_e FDCAN_GetChannelByHandle(FDCAN_HandleTypeDef* phfdcan) {
    if (phfdcan == NULL) {
        return FDCAN_CH_MAX; // 防御性编程，防止空指针
    }

    if (phfdcan->Instance == FDCAN1) {
        return FDCAN1_CH;    // 如果底层硬件是 FDCAN1，返回通道1
    } 
    else if (phfdcan->Instance == FDCAN2) {
        return FDCAN2_CH;    // 如果底层硬件是 FDCAN2，返回通道2
    }
    
    return FDCAN_CH_MAX;     // 无效通道
}

/* 检查通道有效性 */
static uint8_t FDCAN_CheckChannelValid(FDCAN_Channel_e ch) {
    return (ch >= FDCAN1_CH && ch < FDCAN_CH_MAX) ? 1 : 0;
}



