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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// every uart port have a portvector queue...
typedef struct Portvector
{
  uint8_t servonum;
  uint16_t angle[18]; // 7500 +/-3500 (3000 - 12000)
  uint8_t active[18]; // active or not
  uint16_t rangle[18]; // read angle
}
pvector;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CIRC_BUF_SZ (64) // must be power of two
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static uint8_t rx_dma_circ_buf[CIRC_BUF_SZ];
static UART_HandleTypeDef *huart_cobs;
// need variables
#define DMA_WRITE_PTR ( (CIRC_BUF_SZ - __HAL_DMA_GET_COUNTER(huart_cobs->hdmarx)) & (CIRC_BUF_SZ - 1) )
static uint32_t rd_ptr;
static uint8_t dma_char;
/* USER CODE END Variables */
osThreadId idleTaskHandle;
osThreadId LED1TaskHandle;
osThreadId LED2TaskHandle;
osThreadId J1uartportHandle;
osThreadId J2uartportHandle;
osThreadId J3uartportHandle;
osThreadId J4uartportHandle;
osThreadId J5uartportHandle;
osThreadId J6uartportHandle;
osThreadId ImuTaskHandle;
osThreadId ADCTaskHandle;
osMessageQId J1vectorqueueHandle;
osMessageQId J2vectorqueueHandle;
osMessageQId J3vectorqueueHandle;
osMessageQId J4vectorqueueHandle;
osMessageQId J5vectorqueueHandle;
osMessageQId J6vectorqueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void msgrx_init(UART_HandleTypeDef *huart);
static bool msgrx_circ_buf_is_empty(void);
static uint8_t msgrx_circ_buf_get(void);
/* USER CODE END FunctionPrototypes */

void StartidleTask(void const * argument);
void StartLED1Task(void const * argument);
void StartLED2Task(void const * argument);
void StartJ1uartport(void const * argument);
void StartJ2uartport(void const * argument);
void StartJ3uartport(void const * argument);
void StartJ4uartport(void const * argument);
void StartJ5uartport(void const * argument);
void StartJ6uartport(void const * argument);
void StartImuTask(void const * argument);
void StartADCTask(void const * argument);

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

  /* Create the queue(s) */
  /* definition and creation of J1vectorqueue */
  osMessageQDef(J1vectorqueue, 1, pvector);
  J1vectorqueueHandle = osMessageCreate(osMessageQ(J1vectorqueue), NULL);

  /* definition and creation of J2vectorqueue */
  osMessageQDef(J2vectorqueue, 1, pvector);
  J2vectorqueueHandle = osMessageCreate(osMessageQ(J2vectorqueue), NULL);

  /* definition and creation of J3vectorqueue */
  osMessageQDef(J3vectorqueue, 1, pvector);
  J3vectorqueueHandle = osMessageCreate(osMessageQ(J3vectorqueue), NULL);

  /* definition and creation of J4vectorqueue */
  osMessageQDef(J4vectorqueue, 1, pvector);
  J4vectorqueueHandle = osMessageCreate(osMessageQ(J4vectorqueue), NULL);

  /* definition and creation of J5vectorqueue */
  osMessageQDef(J5vectorqueue, 1, pvector);
  J5vectorqueueHandle = osMessageCreate(osMessageQ(J5vectorqueue), NULL);

  /* definition and creation of J6vectorqueue */
  osMessageQDef(J6vectorqueue, 1, pvector);
  J6vectorqueueHandle = osMessageCreate(osMessageQ(J6vectorqueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of idleTask */
  osThreadDef(idleTask, StartidleTask, osPriorityIdle, 0, 128);
  idleTaskHandle = osThreadCreate(osThread(idleTask), NULL);

  /* definition and creation of LED1Task */
  osThreadDef(LED1Task, StartLED1Task, osPriorityIdle, 0, 128);
  LED1TaskHandle = osThreadCreate(osThread(LED1Task), NULL);

  /* definition and creation of LED2Task */
  osThreadDef(LED2Task, StartLED2Task, osPriorityIdle, 0, 128);
  LED2TaskHandle = osThreadCreate(osThread(LED2Task), NULL);

  /* definition and creation of J1uartport */
  osThreadDef(J1uartport, StartJ1uartport, osPriorityIdle, 0, 128);
  J1uartportHandle = osThreadCreate(osThread(J1uartport), NULL);

  /* definition and creation of J2uartport */
  osThreadDef(J2uartport, StartJ2uartport, osPriorityIdle, 0, 128);
  J2uartportHandle = osThreadCreate(osThread(J2uartport), NULL);

  /* definition and creation of J3uartport */
  osThreadDef(J3uartport, StartJ3uartport, osPriorityIdle, 0, 128);
  J3uartportHandle = osThreadCreate(osThread(J3uartport), NULL);

  /* definition and creation of J4uartport */
  osThreadDef(J4uartport, StartJ4uartport, osPriorityIdle, 0, 128);
  J4uartportHandle = osThreadCreate(osThread(J4uartport), NULL);

  /* definition and creation of J5uartport */
  osThreadDef(J5uartport, StartJ5uartport, osPriorityIdle, 0, 128);
  J5uartportHandle = osThreadCreate(osThread(J5uartport), NULL);

  /* definition and creation of J6uartport */
  osThreadDef(J6uartport, StartJ6uartport, osPriorityIdle, 0, 128);
  J6uartportHandle = osThreadCreate(osThread(J6uartport), NULL);

  /* definition and creation of ImuTask */
  osThreadDef(ImuTask, StartImuTask, osPriorityIdle, 0, 128);
  ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);

  /* definition and creation of ADCTask */
  osThreadDef(ADCTask, StartADCTask, osPriorityIdle, 0, 128);
  ADCTaskHandle = osThreadCreate(osThread(ADCTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartidleTask */
/**
  * @brief  Function implementing the idleTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartidleTask */
__weak void StartidleTask(void const * argument)
{
  /* USER CODE BEGIN StartidleTask */
  /* Infinite loop */
  for(;;)
  {
    while (msgrx_circ_buf_is_empty()) {
    }
    osDelay(100);
    while (!msgrx_circ_buf_is_empty()) {
      dma_char = msgrx_circ_buf_get();
    }
    osDelay(100);
  }
  /* USER CODE END StartidleTask */
}

/* USER CODE BEGIN Header_StartLED1Task */
/**
* @brief Function implementing the LED1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED1Task */
__weak void StartLED1Task(void const * argument)
{
  /* USER CODE BEGIN StartLED1Task */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
    osDelay(300);
  }
  /* USER CODE END StartLED1Task */
}

/* USER CODE BEGIN Header_StartLED2Task */
/**
* @brief Function implementing the LED2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED2Task */
__weak void StartLED2Task(void const * argument)
{
  /* USER CODE BEGIN StartLED2Task */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
	osDelay(600);
  }
  /* USER CODE END StartLED2Task */
}

/* USER CODE BEGIN Header_StartJ1uartport */
/**
* @brief Function implementing the J1uartport thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartJ1uartport */
__weak void StartJ1uartport(void const * argument)
{
  /* USER CODE BEGIN StartJ1uartport */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartJ1uartport */
}

/* USER CODE BEGIN Header_StartJ2uartport */
/**
* @brief Function implementing the J2uartport thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartJ2uartport */
__weak void StartJ2uartport(void const * argument)
{
  /* USER CODE BEGIN StartJ2uartport */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartJ2uartport */
}

/* USER CODE BEGIN Header_StartJ3uartport */
/**
* @brief Function implementing the J3uartport thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartJ3uartport */
__weak void StartJ3uartport(void const * argument)
{
  /* USER CODE BEGIN StartJ3uartport */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartJ3uartport */
}

/* USER CODE BEGIN Header_StartJ4uartport */
/**
* @brief Function implementing the J4uartport thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartJ4uartport */
__weak void StartJ4uartport(void const * argument)
{
  /* USER CODE BEGIN StartJ4uartport */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartJ4uartport */
}

/* USER CODE BEGIN Header_StartJ5uartport */
/**
* @brief Function implementing the J5uartport thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartJ5uartport */
__weak void StartJ5uartport(void const * argument)
{
  /* USER CODE BEGIN StartJ5uartport */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartJ5uartport */
}

/* USER CODE BEGIN Header_StartJ6uartport */
/**
* @brief Function implementing the J6uartport thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartJ6uartport */
__weak void StartJ6uartport(void const * argument)
{
  /* USER CODE BEGIN StartJ6uartport */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartJ6uartport */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
__weak void StartImuTask(void const * argument)
{
  /* USER CODE BEGIN StartImuTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartADCTask */
/**
* @brief Function implementing the ADCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADCTask */
__weak void StartADCTask(void const * argument)
{
  /* USER CODE BEGIN StartADCTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartADCTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void msgrx_init(UART_HandleTypeDef *huart)
{
    huart_cobs = huart;
    HAL_UART_Receive_DMA(huart_cobs, rx_dma_circ_buf, CIRC_BUF_SZ);
    rd_ptr = 0;
}
static bool msgrx_circ_buf_is_empty(void) {
    if(rd_ptr == DMA_WRITE_PTR) {
        return true;
    }
    return false;
}
static uint8_t msgrx_circ_buf_get(void) {
    uint8_t c = 0;
    if(rd_ptr != DMA_WRITE_PTR) {
        c = rx_dma_circ_buf[rd_ptr++];
        rd_ptr &= (CIRC_BUF_SZ - 1);
    }
    return c;
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
