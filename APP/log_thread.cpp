/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "elog.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 256 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for elog */
osThreadId_t elogHandle;
uint32_t elogBuffer[ 512 ];
osStaticThreadDef_t elogControlBlock;
const osThreadAttr_t elog_attributes = {
  .name = "elog",
  .stack_mem = &elogBuffer[0],
  .stack_size = sizeof(elogBuffer),
  .cb_mem = &elogControlBlock,
  .cb_size = sizeof(elogControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for elog_lock */
osSemaphoreId_t elog_lockHandle;
const osSemaphoreAttr_t elog_lock_attributes = {
  .name = "elog_lock"
};
/* Definitions for elog_async */
osSemaphoreId_t elog_asyncHandle;
const osSemaphoreAttr_t elog_async_attributes = {
  .name = "elog_async"
};
/* Definitions for elog_dma_lock */
osSemaphoreId_t elog_dma_lockHandle;
const osSemaphoreAttr_t elog_dma_lock_attributes = {
  .name = "elog_dma_lock"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void elog_entry(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  elog_init();

  elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL & ~ELOG_FMT_P_INFO);
  elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
  elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~(ELOG_FMT_FUNC | ELOG_FMT_P_INFO));
  elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL & ~(ELOG_FMT_FUNC | ELOG_FMT_P_INFO));

  elog_start();

  elog_lockHandle = osSemaphoreNew(1, 1, &elog_lock_attributes);

  /* creation of elog_async */
  elog_asyncHandle = osSemaphoreNew(1, 1, &elog_async_attributes);

  /* creation of elog_dma_lock */
  elog_dma_lockHandle = osSemaphoreNew(1, 1, &elog_dma_lock_attributes);

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  elogHandle = osThreadNew(elog_entry, NULL, &elog_attributes);
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  static const char* TAG = "main";
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
    elog_i(TAG, "HelloWorld");
    elog_w(TAG, "HelloWorld");
    elog_e(TAG, "HelloWorld");
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == huart2.Instance) {
        extern osSemaphoreId_t elog_dma_lockHandle;
        osSemaphoreRelease(elog_dma_lockHandle);
    }
}
/* USER CODE END Application */

