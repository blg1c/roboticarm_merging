/*
 *  Project      : Polaris Robot
 *
 *  FilePath     : util_usb.c
 *  Description  : This file contains the functions of USB
 *  LastEditors  : Polaris
 *  Date         : 2023-10-04
 *  LastEditTime : 2023-10-04
 */

#include "util_usb.h"
#include "comm_common.h"
#include "sys_dwt.h"
#include "usbd_cdc.h"  // 显式引入CDC驱动头文件
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

// USB接收缓冲区（与原UART接收缓冲区大小一致）
static uint8_t* usb_rx_buff = NULL;
static uint32_t usb_rx_buff_size = 0;

// USB CDC接收回调（底层USB中断触发）
void CDC_ReceiveCallback(uint8_t* Buf, uint32_t Len) {
    if (Buf == NULL || Len == 0) return;
    Usb_RxIdleCallback(Buf, Len);
}

// USB接收回调转发（对齐原UART逻辑）
void Usb_RxIdleCallback(uint8_t* buff, uint32_t len) {
    Communicate_RXCallback_USB(buff, len);  // 替换原UART的回调入口
}

// USB发送数据（复用原UART的发送逻辑，替换为USB接口）
void Usb_SendMessage(uint8_t txdata[], uint16_t size) {
    if (txdata == NULL || size == 0) return;
    // 阻塞发送（STM32 CDC标准接口，失败时重试一次）
    while (CDC_Transmit_HS(txdata, size) != USBD_OK) {
        DWT_Delayus(10); // 短延时后重试
    }
}

// USB接收初始化
void Usb_InitUsbReceive(uint8_t* rx_buff, uint32_t buff_size) {
    if (rx_buff == NULL || buff_size == 0) return;
    usb_rx_buff = rx_buff;
    usb_rx_buff_size = buff_size;
    // 注册USB CDC接收缓冲区+开启接收包监听
    USBD_CDC_SetRxBuffer(Const_Comm_USB_HANDLER, usb_rx_buff);
    USBD_CDC_ReceivePacket(Const_Comm_USB_HANDLER);
}

// 获取USB当前接收数据长度（对齐原UART DMA的计数逻辑）
uint16_t Usb_CurrentDataCounter(void) {
    // 若USB无DMA，返回已接收长度（STM32 CDC标准接口）
    return (uint16_t)USBD_LL_GetRxDataSize(Const_Comm_USB_HANDLER, CDC_OUT_EP);
}