/*
 * Project      : RobotBody
 * * file         : app_body.c
 * Description  : This file contains Body task function
 * LastEditors  : User
 * Date         : 2024
 * LastEditTime : 2024
 */
 
#include "app_body.h"
#include "sys_dwt.h"
#include "comm_common.h"
#include "module_body.h"



 /**
  * @brief          Body task
  * @param          argument: unused
  * @retval         NULL
  */
void Body_Task(void const * argument) {

  for(;;) {
     
     Body_Output();
        
     Comm_Send_Body_CommData();
     
     osDelay(2);
  }
}