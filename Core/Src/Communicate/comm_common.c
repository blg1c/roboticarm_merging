/*
 *  Project      : Polaris Robot
 *
 *  FilePath     : comm_common.c
 *  Description  : This file contains Bus communication control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:02:53
 *  LastEditTime : 2023-10-04 01:05:20
 */

#include "comm_common.h"
#include "comm_receive.h"
#include "comm_transmit.h"
#include "string.h"
#include "stdlib.h"
#include "sys_dwt.h"
//#include "sys_const.h"
#include "alg_crc.h"
#include "periph_motor.h"
#include "sys_const.h"

// 1. 移除UART依赖，引入USB工具头文件
#include "util_usb.h"

// 全局通信数据结构
Comm_DataTypeDef Comm_Data;
Comm_DataTypeDef* comm = NULL;  // 初始化避免野指针

int ARM_P=0;

uint32_t Const_Comm_OFFLINE_TIME = 0.5;





// 获取通信数据指针
Comm_DataTypeDef* Comm_GetBusDataPtr() {
  return &Comm_Data;
}

// 2. 修改初始化函数：完全替换UART为USB初始化
void Comm_InitComm() {
  Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
  Comm_ResetCommData();
  
  // 初始化USB接收（替换原UART DMA初始化）
  Usb_InitUsbReceive(buscomm->usb_watchBuff, COMM_RECEIVE_BUFF_LEN);
  
  // 初始化全局comm指针
  comm = buscomm;
}

// 检查通信是否离线
uint8_t Comm_IsCommOffline() {
  Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
  if (buscomm == NULL) return 1;
  
  if (DWT_GetDeltaTWithOutUpdate(&buscomm->last_rx_tick) > Const_Comm_OFFLINE_TIME) {
      buscomm->state = Comm_STATE_LOST;
  }
  return (buscomm->state != Comm_STATE_CONNECTED);
}


// 发送阻塞错误（预留实现）
void Comm_SendBlockError() {
  // 可添加USB错误帧发送逻辑
}

// 重置通信数据
void Comm_ResetCommData() {
  Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
  if (buscomm == NULL) return;
  
  // 初始化时间戳
	buscomm->last_rx_tick = DWT_GetTimeline_us();
	buscomm->last_tx_tick = DWT_GetTimeline_us();
  
  // 初始化状态和长度
  buscomm->state = Comm_STATE_NULL;
  buscomm->rx_len = 0;
  
  // 清空缓冲区
  memset(buscomm->usb_watchBuff, 0, COMM_RECEIVE_BUFF_LEN);
  memset(buscomm->usb_sendBuff, 0, COMM_SEND_BUFF_LEN);
  
  // 更新时间差
  buscomm->rx_dt = DWT_GetDeltaT(&buscomm->last_rx_tick);
  buscomm->tx_dt = DWT_GetDeltaT(&buscomm->last_tx_tick);
}


/*
// 3. 修改发送函数：替换UART发送为USB发送
void Comm_SendCommData() {
  Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
  if (buscomm == NULL) return;

  uint32_t len = 5;

  // 填充帧头
  buscomm->usb_sendBuff[0] = 0X5A;
  buscomm->usb_sendBuff[1] = 0XA5;
  
	
  // 调用发送指令函数填充数据
  for (int i = 0; i < Const_Comm_Transmit_BUFF_SIZE; i++) {
      if (CommCmd_Send[i].bus_func != NULL) {
          len += CommCmd_Send[i].bus_func(buscomm->usb_sendBuff + len);
      }
  }
  
  // 填充长度字段
  buscomm->usb_sendBuff[2] = (uint8_t)(len & 0xff);
  buscomm->usb_sendBuff[3] = (uint8_t)((len & 0xff00) >> 8);
  
  // 填充帧尾
  buscomm->usb_sendBuff[len] = 0X7A;
  buscomm->usb_sendBuff[len + 1] = 0XA7;

  // 替换UART发送为USB发送
  Usb_SendMessage(buscomm->usb_sendBuff, len + 2);
  
  // 更新状态和时间戳
  buscomm->state = Comm_STATE_CONNECTED;
  buscomm->last_tx_tick = DWT_GetTimeline_us();
  buscomm->tx_dt = DWT_GetDeltaT(&buscomm->last_tx_tick);
}
*/


void Comm_Send_Body_CommData() {
  Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
  if (buscomm == NULL) return;

  uint32_t len = 5;

  // 填充帧头
  buscomm->usb_sendBuff[0] = 0X5A;
  buscomm->usb_sendBuff[1] = 0XA5;
  
	
  // 调用发送指令函数填充数据
      if (CommCmd_Send[0].bus_func != NULL) {
          len += CommCmd_Send[0].bus_func(buscomm->usb_sendBuff + len);
      }
  
  // 填充长度字段
  buscomm->usb_sendBuff[2] = (uint8_t)(len & 0xff);
  buscomm->usb_sendBuff[3] = (uint8_t)((len & 0xff00) >> 8);
  
	//填充控制对象
	buscomm->usb_sendBuff[4] = CONTROL_OBJ_BODY;
			
  // 填充帧尾
  buscomm->usb_sendBuff[len] = 0X7A;
  buscomm->usb_sendBuff[len + 1] = 0XA7;

  // 替换UART发送为USB发送
  Usb_SendMessage(buscomm->usb_sendBuff, len + 2);
  
  // 更新状态和时间戳
  buscomm->state = Comm_STATE_CONNECTED;
  buscomm->last_tx_tick = DWT_GetTimeline_us();
  buscomm->tx_dt = DWT_GetDeltaT(&buscomm->last_tx_tick);
}

void Comm_Send_L_Arm_CommData() {
  Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
  if (buscomm == NULL) return;

  uint32_t len = 5;

  // 填充帧头
  buscomm->usb_sendBuff[0] = 0X5A;
  buscomm->usb_sendBuff[1] = 0XA5;
  
	
  // 调用发送指令函数填充数据
      if (CommCmd_Send[1].bus_func != NULL) {
          len += CommCmd_Send[1].bus_func(buscomm->usb_sendBuff + len);
      }
  
			
  // 填充长度字段
  buscomm->usb_sendBuff[2] = (uint8_t)(len & 0xff);
  buscomm->usb_sendBuff[3] = (uint8_t)((len & 0xff00) >> 8);
  
	//填充控制对象
	buscomm->usb_sendBuff[4] = CONTROL_OBJ_L_ARM;
	
  // 填充帧尾
  buscomm->usb_sendBuff[len] = 0X7A;
  buscomm->usb_sendBuff[len + 1] = 0XA7;

  // 替换UART发送为USB发送
  Usb_SendMessage(buscomm->usb_sendBuff, len + 2);
  
  // 更新状态和时间戳
  buscomm->state = Comm_STATE_CONNECTED;
  buscomm->last_tx_tick = DWT_GetTimeline_us();
  buscomm->tx_dt = DWT_GetDeltaT(&buscomm->last_tx_tick);
}

void Comm_Send_R_Arm_CommData() {
  Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
  if (buscomm == NULL) return;

  uint32_t len = 5;

  // 填充帧头
  buscomm->usb_sendBuff[0] = 0X5A;
  buscomm->usb_sendBuff[1] = 0XA5;
  
	
  // 调用发送指令函数填充数据
      if (CommCmd_Send[2].bus_func != NULL) {
          len += CommCmd_Send[2].bus_func(buscomm->usb_sendBuff + len);
      }
  
  // 填充长度字段
  buscomm->usb_sendBuff[2] = (uint8_t)(len & 0xff);
  buscomm->usb_sendBuff[3] = (uint8_t)((len & 0xff00) >> 8);
		
	//填充控制对象
	buscomm->usb_sendBuff[4] = CONTROL_OBJ_R_ARM;
  
  // 填充帧尾
  buscomm->usb_sendBuff[len] = 0X7A;
  buscomm->usb_sendBuff[len + 1] = 0XA7;

  // 替换UART发送为USB发送
  Usb_SendMessage(buscomm->usb_sendBuff, len + 2);
  
  // 更新状态和时间戳
  buscomm->state = Comm_STATE_CONNECTED;
  buscomm->last_tx_tick = DWT_GetTimeline_us();
  buscomm->tx_dt = DWT_GetDeltaT(&buscomm->last_tx_tick);
}



// 数据合并与校验（私有函数）
static uint8_t Comm_MergeAndverify(uint8_t buff[], uint32_t rxdatalen) {
  if (buff == NULL || rxdatalen < 6U) { // 最小帧长度：头(2)+长度(2)+尾(2)
    return 0;
  }

  Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
  if (buscomm == NULL) return 0;

  // 校验帧头
  if ((buff[0] != 0x5A) || (buff[1] != 0xA5)) {
    return 0;
  }

  // 校验帧尾
  if ((buff[rxdatalen - 2] != 0x7A) || (buff[rxdatalen - 1] != 0xA7)) {
    return 0;
  }

  // 校验帧长度
  uint16_t frame_len = (uint16_t)(buff[2] | (buff[3] << 8));
  if (frame_len != (uint16_t)(rxdatalen) - 4) { // 总长度 - 头(2) - 尾(2)
    return 0;
  }

  // 限制拷贝长度，防止缓冲区溢出
  uint32_t copy_len = rxdatalen;
  if (copy_len > COMM_RECEIVE_BUFF_LEN) {
    copy_len = COMM_RECEIVE_BUFF_LEN;
  }

  // 拷贝数据到监听缓冲区
  if (buscomm->usb_watchBuff != buff) {
    memcpy(buscomm->usb_watchBuff, buff, copy_len);
  }
  
  return 1;
}

// 数据解码
void Comm_DecodeData(uint8_t buff[], uint32_t rxdatalen) {
  Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
  if (buscomm == NULL || buff == NULL || rxdatalen == 0) {
    return;
  }

  // 数据校验失败直接返回
  if (Comm_MergeAndverify(buff, rxdatalen) == 0){
    return;
  }

  // 更新接收时间戳和时间差
  buscomm->last_rx_tick = DWT_GetTimeline_us();
  buscomm->rx_dt = DWT_GetDeltaT(&buscomm->last_rx_tick);
  
  uint32_t len = 6;

  // 跳过无效指令（指令类型为0）
  if(buscomm->usb_watchBuff[5] == 0) return;

  // 解析指令（按指令类型分支）
  switch(buscomm->usb_watchBuff[5])
  {
		case 0: //身体
			switch(buscomm->usb_watchBuff[4])
      {
        case 0:
          if (CommCmd_Receive[6].bus_func != NULL)
            len += CommCmd_Receive[6].bus_func(buscomm->usb_watchBuff + len);
          break;
        case 1:
          if (CommCmd_Receive[7].bus_func != NULL)
            len += CommCmd_Receive[7].bus_func(buscomm->usb_watchBuff + len);
          break;
        case 2:
          if (CommCmd_Receive[8].bus_func != NULL)
            len += CommCmd_Receive[8].bus_func(buscomm->usb_watchBuff + len);
          break;
        default:
          break;
      }
      break;
			
    case 1: // 左臂
      switch(buscomm->usb_watchBuff[4])
      {
        case 0:
          if (CommCmd_Receive[0].bus_func != NULL)
            len += CommCmd_Receive[0].bus_func(buscomm->usb_watchBuff + len);
          break;
        case 1:
          if (CommCmd_Receive[1].bus_func != NULL)
            len += CommCmd_Receive[1].bus_func(buscomm->usb_watchBuff + len);
          break;
        case 2:
          if (CommCmd_Receive[2].bus_func != NULL)
            len += CommCmd_Receive[2].bus_func(buscomm->usb_watchBuff + len);
          break;
        default:
          break;
      }
      break;

    case 2: // 右臂
      switch(buscomm->usb_watchBuff[4])
      {
        case 0:
          if (CommCmd_Receive[3].bus_func != NULL)
            len += CommCmd_Receive[3].bus_func(buscomm->usb_watchBuff + len);
          break;
        case 1:
          if (CommCmd_Receive[4].bus_func != NULL)
            len += CommCmd_Receive[4].bus_func(buscomm->usb_watchBuff + len);
          break;
        case 2:
          if (CommCmd_Receive[5].bus_func != NULL)
            len += CommCmd_Receive[5].bus_func(buscomm->usb_watchBuff + len);
          break;
        default:
          break;
      }
      break;

    default:
      break;
  }

  // 更新通信状态为已连接
  buscomm->state = Comm_STATE_CONNECTED;
}

// 4. USB接收回调（替换原UART的Communicate_RXCallback）
void Communicate_RXCallback_USB(uint8_t* buff, uint32_t rxdatalen) {
  if (buff == NULL || rxdatalen == 0) return;
  
  Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
  if (buscomm == NULL) return;

  // 记录接收长度
  buscomm->rx_len = rxdatalen;

  // 复用原解析逻辑
  Comm_DecodeData(buff, rxdatalen);

  // 重启USB接收（必须调用，否则无法接收下一包数据）
  USBD_CDC_SetRxBuffer(Const_Comm_USB_HANDLER, buscomm->usb_watchBuff);
  USBD_CDC_ReceivePacket(Const_Comm_USB_HANDLER);
}
