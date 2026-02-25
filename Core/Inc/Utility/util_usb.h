/* util_usb.h 修正后 */
#ifndef USB_UTIL_H
#define USB_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc.h"  
#include "usbd_def.h"  

// 修正：改为hUsbDeviceHS（与实际定义一致）
extern USBD_HandleTypeDef hUsbDeviceHS;
// 同步修改宏定义的句柄
#define Const_Comm_USB_HANDLER &hUsbDeviceHS  

// 保留原有函数声明
void Usb_RxIdleCallback(uint8_t* buff, uint32_t len);
void Usb_SendMessage(uint8_t txdata[], uint16_t size);
void Usb_InitUsbReceive(uint8_t* rx_buff, uint32_t buff_size);
uint16_t Usb_CurrentDataCounter(void);

#ifdef __cplusplus
}
#endif

#endif // USB_UTIL_H