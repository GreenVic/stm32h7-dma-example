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
#include "usart.h"
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
// this must be 16 due to the dma and usart hardware restriction? might be wrong
#define RX_CIRC_BUF_SZ (256)
#define UART8BYTES (sizeof(pvector)+2)

// #define [header name] [code] // [extra receive bytes] [send back bytes] [kind]
#define SCHEADER 0xF8  // 0 4 servo_idmode_scan
#define RXHEADER 0xF7  // 0 74 Current servo vector
#define RCHEADER 0xF6  // 0 pc_rx_buff[1] + 5 Remocon data
#define IMHEADER 0xF5  // 0 20 IMU data
#define PMHEADER 0xF4  // 0 110 servo's flag
#define SPMHEADER 0xF3 // 0 66 servo's 64 parameter bytes
#define BAHEADER 0xF2  // 5 8 balance
#define TVHEADER 0xF1  // 72 74 Trim vector
#define TMHEADER 0xF0  // 0 30 get time info
#define COHEADER 0xEF  // 0 78 get control data
#define GAHEADER 0xEE  // 28 30 gain parameters
#define RFHEADER 0xED  // 0 3 rom_to_flash command
#define FRHEADER 0xEC  // 0 3 flash_to_rom command
#define ADHEADER 0xEB  // 0 4 ADC command
// s is the number of attached sensors (summary: s * 9 + 1 < sendbytes < s * 38 + 1)
#define JSHEADER 0xEA  // 0 s * (9 + [sum of the following bytes]) + 1 jointbase_sensorboard command
#define JS_SW 0x01     // 0 1 jointbase_sensorboard subcommand switch bit
#define JS_ADC 0x02    // 0 8 jointbase_sensorboard subcommand adc bit
#define JS_PS 0x04     // 0 8 jointbase_sensorboard subcommand proximity sensor bit
#define JS_GYRO 0x08   // 0 6 jointbase_sensorboard subcommand gyro bit
#define JS_ACC 0x10    // 0 6 jointbase_sensorboard subcommand acc bit
#define TSHEADER 0xE8  // 65 80
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RX_DMA_WRITE_IDX ( (RX_CIRC_BUF_SZ - ((DMA_Stream_TypeDef *)huart_cobs->hdmarx->Instance)->NDTR) & (RX_CIRC_BUF_SZ - 1) )
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static uint8_t rx_dma_circ_buf[RX_CIRC_BUF_SZ];
static UART_HandleTypeDef *huart_cobs;
static volatile uint32_t rd_idx;
static uint8_t dma_char;

/*!
 * @brief rx header and data
 * The first two bytes data from PC is the communication header.
 * After that, fixed number of bytes will be sent.
 * if header_received is true, it means the microprocessor is receiving data associated with the header.
 * if header_received is false, it means the microprocessor is receiving header.
 */

static uint8_t pc_rx_buff[UART8BYTES] = {0};
static uint32_t pc_rx_buff_idx = 0;
typedef enum RXCommState
{
  RX_HEADER_RECEIVING = 0,
  RX_DATA_RECEIVING,
  RX_READY // data prepared
} rx_comm_state_t;
static rx_comm_state_t rx_comm_state = RX_HEADER_RECEIVING;

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
static void process_pc_rx_buff(void);
static uint32_t get_rx_data_length_from_rx_header(void);
static void process_pc_rx_1byte(uint8_t c);
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
    // ITM_SendChar('a');
    // ITM_SendChar('\r');
    // ITM_SendChar('\n');
    //! @todo implement timeout feature.
    while (!msgrx_circ_buf_is_empty()) {
      dma_char = msgrx_circ_buf_get();
      process_pc_rx_1byte(dma_char);
      if (rx_comm_state == RX_READY) {
        process_pc_rx_buff();
      }
    }
    osDelay(1);
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
    HAL_UART_Receive_DMA(huart_cobs, rx_dma_circ_buf, RX_CIRC_BUF_SZ);
    rd_idx = 0;
}
static bool msgrx_circ_buf_is_empty(void) {
    return rd_idx == RX_DMA_WRITE_IDX;
}
static uint8_t msgrx_circ_buf_get(void) {
    uint8_t c = 0;
    if(!msgrx_circ_buf_is_empty()) {
        c = rx_dma_circ_buf[rd_idx++];
        rd_idx &= (RX_CIRC_BUF_SZ - 1);
    }
    return c;
}

static void process_pc_rx_1byte(uint8_t c)
{
  switch(rx_comm_state)
  {
  case RX_HEADER_RECEIVING:
    pc_rx_buff[pc_rx_buff_idx++] = c;
    if (pc_rx_buff_idx == 2) {
      rx_comm_state = RX_DATA_RECEIVING;
      if (get_rx_data_length_from_rx_header() == 2) {
        rx_comm_state = RX_READY;
      }
    }
    break;
  case RX_DATA_RECEIVING:
    pc_rx_buff[pc_rx_buff_idx++] = c;
    if (pc_rx_buff_idx == get_rx_data_length_from_rx_header()) {
      rx_comm_state = RX_READY;
      pc_rx_buff_idx = 0;
    }
    break;
  case RX_READY:
    // should not enter here
    break;
  default:
    // should not enter here
    break;
  }
}

// call when rx_header[2] is ready (rx_comm_state == RX_DATA_RECEIVING)
static uint32_t get_rx_data_length_from_rx_header(void)
{
  switch (pc_rx_buff[0])
  {
  case BAHEADER:
    return 7;
  case TVHEADER:
     return 74;
  case GAHEADER:
    return 30;
  case TSHEADER: // m-hattori test
    return 67;
  default:
    return 2;
  }
}

static void process_pc_rx_buff(void)
{
  uint8_t tx_buff[256] = {0};
  for (int i = 0; i < 256; i++) {
	  tx_buff[i] = i + 1;
  }
  tx_buff[0] = 1;
  tx_buff[1] = 2;
  tx_buff[2] = 3;
  tx_buff[3] = 4;
  switch(pc_rx_buff[0])
  {
  case SCHEADER:
    HAL_UART_Transmit(&huart8, tx_buff, 4, HAL_MAX_DELAY);
    break;
  case RXHEADER:
    HAL_UART_Transmit(&huart8, tx_buff, 74, HAL_MAX_DELAY);
    break;
  case RCHEADER:
    HAL_UART_Transmit(&huart8, tx_buff, pc_rx_buff[1] + 5, HAL_MAX_DELAY);
    break;
  case IMHEADER:
    tx_buff[0] = 20;
    HAL_UART_Transmit(&huart8, tx_buff, 20, HAL_MAX_DELAY);
    break;
  case PMHEADER:
    HAL_UART_Transmit(&huart8, tx_buff, 110, HAL_MAX_DELAY);
    break;
  case SPMHEADER:
    HAL_UART_Transmit(&huart8, tx_buff, 66, HAL_MAX_DELAY);
    break;
  case BAHEADER:
    HAL_UART_Transmit(&huart8, tx_buff, 8, HAL_MAX_DELAY);
    break;
  case TVHEADER:
	 tx_buff[0] = 74;
	 tx_buff[73] = 74;
    HAL_UART_Transmit(&huart8, tx_buff, 74, HAL_MAX_DELAY);
    break;
  case TMHEADER:
    HAL_UART_Transmit(&huart8, tx_buff, 30, HAL_MAX_DELAY);
    break;
  case COHEADER:
    HAL_UART_Transmit(&huart8, tx_buff, 78, HAL_MAX_DELAY);
    break;
  case GAHEADER:
    HAL_UART_Transmit(&huart8, tx_buff, 30, HAL_MAX_DELAY);
    break;
  case RFHEADER:
    HAL_UART_Transmit(&huart8, tx_buff, 3, HAL_MAX_DELAY);
    break;
  case FRHEADER:
    HAL_UART_Transmit(&huart8, tx_buff, 3, HAL_MAX_DELAY);
    break;
  case ADHEADER:
    HAL_UART_Transmit(&huart8, tx_buff, 4, HAL_MAX_DELAY);
    break;
  case JSHEADER:
    HAL_UART_Transmit(&huart8, tx_buff, 77, HAL_MAX_DELAY);
    break;
  case TSHEADER: // m-hattori test
    for (size_t i = 0; i < 80; i++) {
        tx_buff[i] = i + 1;
    }
    HAL_UART_Transmit(&huart8, tx_buff, 80, HAL_MAX_DELAY);
    break;
  default:
    break;
  }
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
