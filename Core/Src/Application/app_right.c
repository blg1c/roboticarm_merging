/*
 *  Project      :RightArm
 * 
 *  file         : app_right.c
 *  Description  : This file contains RightArm task function
 *  LastEditors  : lkn
 *  Date         : 2024-7-2
 *  LastEditTime : 2024-7-3
 */
 
#include "module_right_arm.h"
#include "app_right.h"
#include "sys_dwt.h"
#include "comm_common.h"

 /**
  * @brief          LeftArm task
  * @param          NULL
  * @retval         NULL
  */
	
	
void RightArm_Task(void const * argument) {

  
  osDelay(500);

  for(;;) {

		
	 RightArm_Output();
		
	 //Comm_Send_R_Arm_CommData();
		
   osDelay(2);
  }
}
