/*
 *  Project      :LeftArm
 * 
 *  file         : app_left.c
 *  Description  : This file contains LeftArm task function
 *  LastEditors  : lkn
 *  Date         : 2024-7-2
 *  LastEditTime : 2024-7-3
 */
 
#include "module_left_arm.h"
#include "app_left.h"
#include "sys_dwt.h"
#include "comm_common.h"
 /**
  * @brief          LeftArm task
  * @param          NULL
  * @retval         NULL
  */
	
	
void LeftArm_Task(void const * argument) {

  

  for(;;) {

   
		
	 //LeftArm_Output();
	FDCAN_SendMITCmd(&hfdcan1, 0x01, 0, 0, 0, 0, 5);
	
//Comm_Send_L_Arm_CommData();
	 
   osDelay(2);
  }
}
