/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "supervise.h"
#include "robot.h"
#include "chassis.h"
#include "mpu6050.h"
#include "shoot.h"
#include "pid.h"
#include "usart.h"
#include "la_motor.h" //  
#include "hc_sr04.h" // ???
#include "dynamixel.h" //
#include "controller.h" //
#include "math_controller.h"
#include "can_motor.h"//
#include "climb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int FPS = 0;
float v, w;

extern uint8_t lamotor_rx_buffer[32];
extern uint8_t lamotor_tx_buffer[32];
extern uint8_t uart7_rx_callback;

uint8_t climb_flag = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t test = 0x00; 

bounce_wheel_t wheel1;

extern encoder_t motor_left_encoder, motor_right_encoder;

/* USER CODE END Variables */
osThreadId ClimbTaskHandle;
osThreadId GripperTaskHandle;
osThreadId ShootTaskHandle;
osThreadId ChassisTaskHandle;
osTimerId SuperviseTimerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartClimbTask(void const * argument);
void StartGripperTask(void const * argument);
void StartShootTask(void const * argument);
void StartChassisTask(void const * argument);
void SuperviseTimerCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

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

  /* Create the timer(s) */
  /* definition and creation of SuperviseTimer */
  osTimerDef(SuperviseTimer, SuperviseTimerCallback);
  SuperviseTimerHandle = osTimerCreate(osTimer(SuperviseTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	osTimerStart(SuperviseTimerHandle, 10);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ClimbTask */
  osThreadDef(ClimbTask, StartClimbTask, osPriorityRealtime, 0, 256);
  ClimbTaskHandle = osThreadCreate(osThread(ClimbTask), NULL);

  /* definition and creation of GripperTask */
  osThreadDef(GripperTask, StartGripperTask, osPriorityRealtime, 0, 256);
  GripperTaskHandle = osThreadCreate(osThread(GripperTask), NULL);

  /* definition and creation of ShootTask */
  osThreadDef(ShootTask, StartShootTask, osPriorityNormal, 0, 256);
  ShootTaskHandle = osThreadCreate(osThread(ShootTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, StartChassisTask, osPriorityRealtime, 0, 128);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartClimbTask */
/**
  * @brief  Function implementing the ClimbTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartClimbTask */
extern uint8_t support_link_init_status;
uint8_t support_link_status = 0;
extern uint8_t climb_status;
void StartClimbTask(void const * argument)
{
  /* USER CODE BEGIN StartClimbTask */
	SupportLinkInit();
  /* Infinite loop */
  for(;;)
  {
//		if(support_link_init_status == 0)
//			SelfCalibration();
//		else
//		SupportLinkControl(1);
		switch(climb_status)
		{
			case READY:
				SupportLinkControl(-1);
				break;
			
			case CLIMBING:
				SupportLinkControl(1);
				break;
			
			case PUTDOWN:
				SupportLinkControl(1);
				break;
			
			case RETRIEVING:
				SupportLinkControl(0);
				break;
			
			case CALIB:
				SupportLinkControl(2);
				break;
			
			default:
				break;
		}
		
    osDelay(1);
  }
  /* USER CODE END StartClimbTask */
}

/* USER CODE BEGIN Header_StartGripperTask */
/**
* @brief Function implementing the GripperTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGripperTask */
extern const uint16_t servo1_set_encoder;
extern const uint16_t servo1_reset_encoder;
extern const uint16_t servo2_set_encoder;
extern const uint16_t servo2_reset_encoder;
float servo1_pwm = 0;
float servo2_pwm = 0;
uint16_t test_spd = 50;
uint16_t delay_time = 10;

uint16_t test_pos = 2000;
int16_t test_servo_pos = 2150;

uint8_t reset = 0;
void StartGripperTask(void const * argument)
{
  /* USER CODE BEGIN StartGripperTask */
  /* Infinite loop */
  for(;;)
  {

		LAMotorSetPosition(1,test_pos);//test_pos
		osDelay(50);
		LAMotorGetState(1);
		osDelay(50);
		
		DynamixelReadMsg(0x01);
		osDelay(20);

//   	DynamixelReset(0x01);
		osDelay(10);
//	  DynamixelSetMultiTermMode(0x02);
  		DynamixelSetSingleTermMode(0x01);
  		osDelay(10);
//		DynamixelWriteSpeed(0x02,test_spd); 
//    DynamixelWritePositionAndSpeed(0x01, test_servo_pos, test_spd);
 			DynamixelWritePositionAndSpeed(0x01, servo1_pwm*(servo1_set_encoder - servo1_reset_encoder) + servo1_reset_encoder, test_spd);
//		osDelay(10);
//		DynamixelWritePositionAndSpeed(0x02, servo2_pwm*(servo2_set_encoder - servo2_reset_encoder) + servo2_reset_encoder, test_spd);
		
		osDelay(10);
  }
  /* USER CODE END StartGripperTask */
}

/* USER CODE BEGIN Header_StartShootTask */
/**
* @brief Function implementing the ShootTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartShootTask */
void StartShootTask(void const * argument)
{
  /* USER CODE BEGIN StartShootTask */

  /* Infinite loop */
  for(;;)
  {
		osDelay(10);
  }
  /* USER CODE END StartShootTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
float test_output = 0;
float test_input = 0;

extern flpid_t wheel_left, wheel_right;
extern flcpid2d_t supportlink_ctrl;
void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* Infinite loop */
	ChassisInit();
  for(;;)
  {
		SelectMode();
		switch(climb_status)
		{
			case READY:
				SpeedControl(v, w, 1);
				break;
			
			case CLIMBING:
				SpeedControl(CLIMBING_SPEED, 0, 1);
				break;
			
			case PUTDOWN:
				SpeedControl(0, 0, 1);
				break;
			
			case RETRIEVING:
				SpeedControl(0, 0, 0);
				break;
			
			default:
				break;
		}
		CanTransmit_1234(&hcan1, (int16_t)wheel_left.output, (int16_t)wheel_right.output, (int16_t)supportlink_ctrl.output, 0);
		
//		HCSR04GetDistance();

    osDelay(1);
  }
  /* USER CODE END StartChassisTask */
}

/* SuperviseTimerCallback function */
void SuperviseTimerCallback(void const * argument)
{
  /* USER CODE BEGIN SuperviseTimerCallback */
  SuperviseTaskHandle();
  /* USER CODE END SuperviseTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
