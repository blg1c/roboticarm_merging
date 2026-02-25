/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for LeftArmTask */
osThreadId_t LeftArmTaskHandle;
const osThreadAttr_t LeftArmTask_attributes = {
  .name = "LeftArmTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for RightArmTask */
osThreadId_t RightArmTaskHandle;
const osThreadAttr_t RightArmTask_attributes = {
  .name = "RightArmTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for BodyTask */
osThreadId_t BodyTaskHandle;
const osThreadAttr_t BodyTask_attributes = {
  .name = "BodyTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LeftArm_Task(void *argument);
void RightArm_Task(void *argument);
void Body_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LeftArmTask */
  LeftArmTaskHandle = osThreadNew(LeftArm_Task, NULL, &LeftArmTask_attributes);

  /* creation of RightArmTask */
  RightArmTaskHandle = osThreadNew(RightArm_Task, NULL, &RightArmTask_attributes);

  /* creation of BodyTask */
  BodyTaskHandle = osThreadNew(Body_Task, NULL, &BodyTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_LeftArm_Task */
/**
  * @brief  Function implementing the LeftArmTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LeftArm_Task */
__weak void LeftArm_Task(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN LeftArm_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LeftArm_Task */
}

/* USER CODE BEGIN Header_RightArm_Task */
/**
* @brief Function implementing the RightArmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RightArm_Task */
__weak void RightArm_Task(void *argument)
{
  /* USER CODE BEGIN RightArm_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RightArm_Task */
}

/* USER CODE BEGIN Header_Body_Task */
/**
* @brief Function implementing the BodyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Body_Task */
__weak void Body_Task(void *argument)
{
  /* USER CODE BEGIN Body_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Body_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

