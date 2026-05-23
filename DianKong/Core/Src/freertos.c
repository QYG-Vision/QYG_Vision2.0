/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "bsp_buzzer.h"
#include "master_node.h"
#include "Flysky_bsp.h"
#include "auto_mode.h"
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
extern _sentry_data_t             sentry_RM2026;
extern sentry_remote              sentry_Remote;
extern JiuXian_Logic_info_t       Soul_of_JiuXian;
/* USER CODE END Variables */
/* Definitions for ctrlTask */
osThreadId_t ctrlTaskHandle;
const osThreadAttr_t ctrlTask_attributes = {
  .name = "ctrlTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};
/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
  .name = "DebugTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};
/* Definitions for Master_task */
osThreadId_t Master_taskHandle;
const osThreadAttr_t Master_task_attributes = {
  .name = "Master_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};
/* Definitions for sentry_buzzer */
osThreadId_t sentry_buzzerHandle;
const osThreadAttr_t sentry_buzzer_attributes = {
  .name = "sentry_buzzer",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ctrl_node(void *argument);
void Serial_node(void *argument);
void Master_node(void *argument);
void buzzer_task(void *argument);

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
  /* creation of ctrlTask */
  ctrlTaskHandle = osThreadNew(ctrl_node, NULL, &ctrlTask_attributes);

  /* creation of DebugTask */
  DebugTaskHandle = osThreadNew(Serial_node, NULL, &DebugTask_attributes);

  /* creation of Master_task */
  Master_taskHandle = osThreadNew(Master_node, (void*) Master_node, &Master_task_attributes);

  /* creation of sentry_buzzer */
  sentry_buzzerHandle = osThreadNew(buzzer_task, NULL, &sentry_buzzer_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ctrl_node */
/**
  * @brief  Function implementing the ctrlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ctrl_node */
__weak void ctrl_node(void *argument)
{
  /* USER CODE BEGIN ctrl_node */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ctrl_node */
}

/* USER CODE BEGIN Header_Serial_node */
/**
* @brief Function implementing the DebugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Serial_node */
__weak void Serial_node(void *argument)
{
  /* USER CODE BEGIN Serial_node */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Serial_node */
}

/* USER CODE BEGIN Header_Master_node */
/**
* @brief Function implementing the Master_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Master_node */
__weak void Master_node(void *argument)
{
  /* USER CODE BEGIN Master_node */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Master_node */
}

/* USER CODE BEGIN Header_buzzer_task */
/**
* @brief Function implementing the sentry_buzzer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_buzzer_task */
__weak void buzzer_task(void *argument)
{
  /* USER CODE BEGIN buzzer_task */
	sentry_open();
  /* Infinite loop */
  for(;;)
  {
    // if(sentry_Remote.ctrl.mode_switch == AUTOMATION_MODE)
    // {
    //       if(sentry_RM2026.shoot_vel != 0 && sentry_RM2026.pluck_vel != 0)
    //       {
    //               attack_status(29,800);
    //       }else{
    //               normal_status(29,1400);
    //       }
    //       osDelay(0);
    // }
    // else{
    //       osDelay(10);
    // }
    if (Soul_of_JiuXian.Mode == AUTO_SCAN)
    {
       normal_status(29,1400);
       taskYIELD();
    }else if (Soul_of_JiuXian.Mode == STRICKER)
    {
       attack_status(29,800);
       taskYIELD();
    }else {
       osDelay(10);
    }
  }
  /* USER CODE END buzzer_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

