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
#include "imu_task.h"
#include "usart_task.h"
#include "remote_task.h"
#include "can_task.h"
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
/* 新增静态任务，不挤占现有RTOS heap；栈单位为32位word。 */
static StaticTask_t trajectory_tcb, plot_tcb;
static StackType_t trajectory_stack[256], plot_stack[512];
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId imutaskHandle;
osThreadId usarttaskHandle;
osThreadId remoteTaskHandle;
osThreadId ledtaskHandle;
osThreadId buzzerTaskHandle;
osThreadId controlTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void trajectory_task(void const * argument);
extern void plot_task(void const * argument);
static void trajectory_entry(void * argument) { trajectory_task(argument); }
static void plot_entry(void * argument) { plot_task(argument); }
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void imu_task(void const * argument);
extern void usart_task(void const * argument);
extern void remote_task(void const * argument);
extern void led_task(void const * argument);
extern void buzzer_task(void const * argument);
extern void control_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of imutask */
  osThreadDef(imutask, imu_task, osPriorityHigh, 0, 512);
  imutaskHandle = osThreadCreate(osThread(imutask), NULL);

  /* definition and creation of usarttask */
  osThreadDef(usarttask, usart_task, osPriorityLow, 0, 512);
  usarttaskHandle = osThreadCreate(osThread(usarttask), NULL);

  /* definition and creation of remoteTask */
  osThreadDef(remoteTask, remote_task, osPriorityNormal, 0, 512);
  remoteTaskHandle = osThreadCreate(osThread(remoteTask), NULL);

  /* definition and creation of ledtask */
  osThreadDef(ledtask, led_task, osPriorityLow, 0, 512);
  ledtaskHandle = osThreadCreate(osThread(ledtask), NULL);

  /* definition and creation of buzzerTask */
  osThreadDef(buzzerTask, buzzer_task, osPriorityIdle, 0, 512);
  buzzerTaskHandle = osThreadCreate(osThread(buzzerTask), NULL);

  /* definition and creation of controlTask */
  osThreadDef(controlTask, control_task, osPriorityRealtime, 0, 512);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 原生优先级：轨迹4、绘图1、现有control为6。 */
  TaskHandle_t trajectory_handle = xTaskCreateStatic(
    trajectory_entry, "trajectory", 256, NULL, 4, trajectory_stack, &trajectory_tcb);
  TaskHandle_t plot_handle = xTaskCreateStatic(
    plot_entry, "tracking_plot", 512, NULL, 1, plot_stack, &plot_tcb);
  configASSERT(trajectory_handle != NULL);
  configASSERT(plot_handle != NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */
