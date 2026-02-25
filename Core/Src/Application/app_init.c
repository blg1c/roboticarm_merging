/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_init.c
 *  Description  : All initialization threads
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:42:52
 *  LastEditTime : 2023-10-28 18:15:38
 */
 
 #include "app_init.h"
 #include "sys_dwt.h"
 #include "util_can.h"
 #include "module_left_arm.h"
 #include "module_right_arm.h"
 #include "module_motor_cmd.h"
 #include "periph_motor.h"
 #include "FreeRTOS.h"
 #include "cmsis_os.h"
 #include "comm_common.h"
 
void Init_InitAll() {

	// system init
	DWT_Init(480);

	// uart init
	Comm_InitComm();
	// util init
	FDCAN_InitFilterAndStart(&hfdcan1, FDCAN1_CH);
	FDCAN_InitFilterAndStart(&hfdcan2, FDCAN2_CH);
	// periph init
	Motor_InitAllMotors();
	HAL_Delay(500);
	Motor_DM_Basic_Ctrl(Motor_Enable);  
	Motor_RS_Basic_Ctrl(Motor_Enable);
	// 电机使能
	
	
	
	// Modules init
	Motor_CtrlCmds_Init();
	
	LeftArm_DoubleLoopPID_InitAll();
	RightArm_DoubleLoopPID_InitAll();
} 



void Init_Task(){

	for(;;){
		osDelay(1000);
	}

}
