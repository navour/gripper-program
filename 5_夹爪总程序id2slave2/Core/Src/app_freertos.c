/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
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
#include "pressure_history.h"
#include "AS5048A.h"
#include "bsp_fdcan.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include <string.h>
#include <stdio.h>
#include "pressure_sensor.h"
#include "Uart1_Print.h"
#include "dm_modbus_bridge.h"
#include "usart.h" 
#include "force_estimator.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRESSURE_SPI_DIAG_MINIMAL 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for MOTORTask */
osThreadId_t MOTORTaskHandle;
const osThreadAttr_t MOTORTask_attributes = {
  .name = "MOTORTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for AS5048ATask */
osThreadId_t AS5048ATaskHandle;
const osThreadAttr_t AS5048ATask_attributes = {
  .name = "AS5048ATask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for PRESSURETask */
osThreadId_t PRESSURETaskHandle;
const osThreadAttr_t PRESSURETask_attributes = {
  .name = "PRESSURETask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* Definitions for UART1PrintTask */
osThreadId_t UART1PrintTaskHandle;
const osThreadAttr_t UART1PrintTask_attributes = {
  .name = "UART1PrintTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 768 * 4
};
/* Definitions for ModbusTask */
osThreadId_t ModbusTaskHandle;
const osThreadAttr_t ModbusTask_attributes = {
  .name = "ModbusTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for SpiRxSem */
osSemaphoreId_t SpiRxSemHandle;
const osSemaphoreAttr_t SpiRxSem_attributes = {
  .name = "SpiRxSem"
};


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMOTORTask(void *argument);
void StartLEDTask(void *argument);
void StartAS5048ATask(void *argument);
void StartPRESSURETask(void *argument);
void StartUART1PrintTask(void *argument);
void StartModbusTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  PressureHistory_Init(2u, 2u);

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SpiRxSem */
  SpiRxSemHandle = osSemaphoreNew(1, 0, &SpiRxSem_attributes);


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
  /* creation of MOTORTask */
#if !PRESSURE_SPI_DIAG_MINIMAL
  MOTORTaskHandle = osThreadNew(StartMOTORTask, NULL, &MOTORTask_attributes);
#endif

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

  /* creation of AS5048ATask */
  AS5048ATaskHandle = osThreadNew(StartAS5048ATask, NULL, &AS5048ATask_attributes);

  /* creation of PRESSURETask */
  PRESSURETaskHandle = osThreadNew(StartPRESSURETask, NULL, &PRESSURETask_attributes);

  /* creation of UART1PrintTask */
  UART1PrintTaskHandle = osThreadNew(StartUART1PrintTask, NULL, &UART1PrintTask_attributes);

  /* creation of ModbusTask */
#if !PRESSURE_SPI_DIAG_MINIMAL
  ModbusTaskHandle = osThreadNew(StartModbusTask, NULL, &ModbusTask_attributes);
#endif

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMOTORTask */
/**
  * @brief  Function implementing the MOTORTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMOTORTask */
void StartMOTORTask(void *argument)
{
  /* USER CODE BEGIN StartMOTORTask */
  osDelay(500);
	
  motor[Motor1].ctrl.mode   = mit_mode;
  motor[Motor1].ctrl.kp_set = 0.0f;
  motor[Motor1].ctrl.kd_set = 0.0f;
  motor[Motor1].ctrl.vel_set = 0.0f;
  motor[Motor1].ctrl.tor_set = 0.0f;
  motor[Motor1].ctrl.pos_set = 0.0f;


  /* Infinite loop */
  for(;;)
  {
      dm_motor_ctrl_send(&hfdcan1, &motor[Motor1]);
      osDelay(2);
  }
  /* USER CODE END StartMOTORTask */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for(;;)
  {    
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    osDelay(500);

  }
  /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_StartAS5048ATask */
/**
* @brief Function implementing the AS5048ATask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAS5048ATask */
void StartAS5048ATask(void *argument)
{
  /* USER CODE BEGIN StartAS5048ATask */

	AS5048A_QH_Init();
  /* Infinite loop */
  for(;;)
  {
	  AS5048A_Update_All();

    if (!error_flag)
    {
      Uart1_Print_UpdateAS5048A(true, angle_val, angle_deg, mag_val);
    }
    else
    {
      Uart1_Print_UpdateAS5048A(false, angle_val, angle_deg, mag_val);
    }
	
    osDelay(50);
  }
  /* USER CODE END StartAS5048ATask */
}

/* USER CODE BEGIN Header_StartPRESSURETask */
/**
* @brief Function implementing the PRESSURETask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPRESSURETask */
void StartPRESSURETask(void *argument)
{
  /* USER CODE BEGIN StartPRESSURETask */
	PressureSensor_Init();
	osDelay(100);
  /* Infinite loop */
  for(;;)
  {
	  PressureSensor_UpdateAndPrint();
      osDelay(50);
}
  /* USER CODE END StartPRESSURETask */
}

/* USER CODE BEGIN Header_StartUART1PrintTask */
/**
* @brief Function implementing the UART1PrintTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART1PrintTask */
void StartUART1PrintTask(void *argument)
{
  /* USER CODE BEGIN StartUART1PrintTask */
  /* Infinite loop */
  for(;;)
  {
	UART1Print(argument);  
    osDelay(1);
  }
  /* USER CODE END StartUART1PrintTask */
}

/* USER CODE BEGIN Header_StartModbusTask */
/**
* @brief Function implementing the ModbusTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartModbusTask */
void StartModbusTask(void *argument)
{
  /* USER CODE BEGIN StartModbusTask */
	 osDelay(20); 
	 ForceEstimator_Init();
  // 初始化桥接，并启动UART2的ReceiveToIdle
	DM_Bridge_Init(&huart2, &hfdcan1, 1);  // motor_id=1
	DM_Bridge_Start();

  /* Infinite loop */
  for(;;)
  {
	DM_Bridge_Poll();
    osDelay(1);
  }
  /* USER CODE END StartModbusTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
