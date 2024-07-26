/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <BMI088driver.h>
#include "MahonyAHRS.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "pid.h"
#include "usart.h"
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
//typedef struct {
//    float Kp;
//    float Ki;
//    float Kd;
//    float last_error;
//    float integral;
//} PID;
char receive;
int it_state = 0;
const float default_speed = 25;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId proc_imuHandle;
osThreadId base_ctrlHandle;
osThreadId lora_receiveHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (!it_state) {
    it_state = 1;
    HAL_UART_Receive_IT(&huart1, &receive, 8);
  }
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);

void Start_proc_imu(void const *argument);

void Start_base_ctrl(void const *argument);

void Start_lora_receive(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
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

  /* definition and creation of proc_imu */
  osThreadDef(proc_imu, Start_proc_imu, osPriorityIdle, 0, 128);
  proc_imuHandle = osThreadCreate(osThread(proc_imu), NULL);

  /* definition and creation of base_ctrl */
  osThreadDef(base_ctrl, Start_base_ctrl, osPriorityIdle, 0, 128);
  base_ctrlHandle = osThreadCreate(osThread(base_ctrl), NULL);

  /* definition and creation of lora_receive */
  osThreadDef(lora_receive, Start_lora_receive, osPriorityIdle, 0, 128);
  lora_receiveHandle = osThreadCreate(osThread(lora_receive), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_proc_imu */
/**
* @brief Function implementing the proc_imu thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_proc_imu */
void Start_proc_imu(void const *argument) {
  /* USER CODE BEGIN Start_proc_imu */
  /* Infinite loop */

  for (;;) {
    BMI088_read(gyro, accel, &temp);
    MahonyAHRSupdateIMU(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
    get_angle(&yaw, &pitch, &roll);
    osDelay(1);
  }
  /* USER CODE END Start_proc_imu */
}

/* USER CODE BEGIN Header_Start_base_ctrl */
/**
* @brief Function implementing the base_ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_base_ctrl */
void Start_base_ctrl(void const *argument) {
  /* USER CODE BEGIN Start_base_ctrl */
  /* Infinite loop */

  // 直立
  float ver_kp = 80;
  float ver_kd = 300;
  // 速度
  float vel_kp = -0.310;

  float vel_ki = vel_kp / 100;

//  pid_Init(&vel_pid, 0, 0, 0, 0);
//  pid_Init(&ver_pid, 0, 0, 0, 0);
  pid_Init(&vel_pid, vel_kp, vel_ki, 0, 0);  // 速度环初始化 14 0.07
  pid_Init(&ver_pid, ver_kp, 0, ver_kd, 0);  //  直立环初始化 78 72
  pid_Init(&turn_pid, -10, 0, -70, 0);  //  转向环初始化

  target_speed = 0;
  target_turn = 0;

  HAL_UART_Receive_IT(&huart1, &receive, 8);
  for (;;) {
    if (it_state) {
      if (receive == 'w') {
        target_speed = -default_speed;
      } else if (receive == 's') {
        target_speed = default_speed;
      } else if (receive == 'a') {
        target_turn = -50;
      } else if (receive == 'd') {
        target_turn = 50;
      } else if (receive == 'q') {
        target_speed = 0;
        target_turn = 0;
      } else if (receive == 'p') {
        target_speed = 0;
        target_turn = 0;
        stop = 1;
        vTaskSuspend(NULL);
      }
      it_state = 0;
    }

    gyro_x = gyro[0];
    gyro_y = gyro[1];
    gyro_z = gyro[2];
    contral(&volacity_out, &vertical_out, &turn_out);
    Al_out = vertical_out;
    Motor_1 = Al_out + turn_out;
    Motor_2 = Al_out - turn_out;
    Motor_1 > 2000 ? 2000 : (Motor_1 < (-2000) ? (-2000) : Motor_1);
    Motor_2 > 2000 ? 2000 : (Motor_2 < (-2000) ? (-2000) : Motor_2);
    if (Motor_1 < 0) {
      Can_out[1] = 0;
      Can_out[0] = -Motor_1;
    } else {
      Can_out[1] = Motor_1;
      Can_out[0] = 0;
    }
    if (Motor_2 < 0) {
      Can_out[3] = -Motor_2;
      Can_out[2] = 0;
    } else {
      Can_out[3] = 0;
      Can_out[2] = Motor_2;
    }
    if (!stop){
      CAN_cmd_chassis(Can_out[0], Can_out[1], Can_out[2], Can_out[3]);
    }

    osDelay(10);
  }
  /* USER CODE END Start_base_ctrl */
}

/* USER CODE BEGIN Header_Start_lora_receive */
/**
* @brief Function implementing the lora_receive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_lora_receive */
void Start_lora_receive(void const *argument) {
  /* USER CODE BEGIN Start_lora_receive */
  /* Infinite loop */
  for (;;) {

    osDelay(1);
  }
  /* USER CODE END Start_lora_receive */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
