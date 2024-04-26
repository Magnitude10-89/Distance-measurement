/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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

/* USER CODE BEGIN PV */
uint16_t msHcCount_L = 0; // Left ultrasonic measurement counter
uint16_t msHcCount_R = 0; // Right ultrasonic measurement counter
uint16_t Test_length = 0; //Using to measure 2 sensors seperately
float  L_data, R_data;    // Distance measurements for left and right
int    L_int, R_int;      // Integer part of the measurements
float  L_dec, R_dec;      // Decimal part after the point
float  data = 0.0f;        // Initialization of data variable

// Delay for a specified number of microseconds
void delay_us(uint32_t udelay)
{
  __IO uint32_t Delay = udelay * 84 ; // Calculation based on SystemCoreClock (SystemCoreClock / 8U / 1000000U)
  // Reference: from stm32f1xx_hal_rcc.c -- static void RCC_Delay(uint32_t mdelay)
  do
  {
    __NOP();
  }
  while (Delay --);
}

// Function to open timer for ultrasonic measurement
static void OpenTimerForHc()  
{
   __HAL_TIM_SET_COUNTER(&htim4, 0); // Initialize the counter to zero
   msHcCount_L = 0; // Reset left counter
   msHcCount_R = 0; // Reset right counter
   HAL_TIM_Base_Start_IT(&htim4); // Start the base timer
}

// Function to close timer for ultrasonic measurement
static void CloseTimerForHc()    
{
   HAL_TIM_Base_Stop_IT(&htim4); // Stops the base timer
}

// Function to get echo timer value for left ultrasonic module
uint32_t GetEchoTimer_L(void) 
{
   uint32_t t = 0;
   t = msHcCount_L * 1000; // Convert units to milliseconds
   t += __HAL_TIM_GET_COUNTER(&htim4);
   TIM4->CNT = 0;
   HAL_Delay(50); // Delay for 50ms
   return t;
}

// Callback function for timer period elapsed - interrupts for counting time
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == (&htim4))
  {
    msHcCount_L++; // Measuring time for left
    msHcCount_R++; // Measuring time for right
  }
}

// Function to get echo timer value for right ultrasonic module
uint32_t GetEchoTimer_R(void) 
{
   uint32_t t = 0;
   t = msHcCount_R * 1000; // Convert units to milliseconds
   t += __HAL_TIM_GET_COUNTER(&htim4);
   TIM4->CNT = 0;
   HAL_Delay(50); // Delay for 50ms
   return t;
}

uint32_t Test_L(void) 
{
   uint32_t t = 0;
   t = Test_length * 1000; // Convert units to milliseconds
   t += __HAL_TIM_GET_COUNTER(&htim4);
   TIM4->CNT = 0;//set counter back to 0
   HAL_Delay(50); // Delay for 50ms
   return t;
}
// Function to measure distance using left ultrasonic module
float Hcsr04GetLength_L(void) 
{
   uint32_t t = 0;

   float lengthTemp_L = 0;

 
   {
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // Send start signal  trig2:PB8  echo1:PB6
       delay_us(20);
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // End start signal

       while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0); // Wait for initial low voltage level
       OpenTimerForHc() ; // Start timing when high voltage appears
//       i = i + 1;
       while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 1); // Wait for high voltage level to disappear
       CloseTimerForHc(); // Stop timing
       t = GetEchoTimer_L(); // Process time to calculate distance
       lengthTemp_L = (t / 58.0); // Convert to distance in cm
//     
        
    }

    return lengthTemp_L;	
}



// Function to measure distance using right ultrasonic module
float Hcsr04GetLength_R(void) 
{
   uint32_t t = 0;

   float lengthTemp_R = 0;

   {
		   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 , GPIO_PIN_SET); // Send start signal  trig2:PB8  echo1:PB6
       delay_us(20);
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 , GPIO_PIN_RESET); // End start signal

       while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0); // Wait for initial low voltage level
       OpenTimerForHc() ; // Start timing when high voltage appears
//   i = i + 1;
       while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 1); // Wait for high voltage level to disappear
       CloseTimerForHc(); // Stop timing
       t = GetEchoTimer_R(); // Process time to calculate distance
       lengthTemp_R = (t / 58.0); // Convert to distance in cm
    }

    return lengthTemp_R; // Return measured distance
}
float test_length=0;
void Test(void)
{
	uint32_t t = 0;


   {
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8| GPIO_PIN_5, GPIO_PIN_SET); // Send start signal trig1:PB5 trig2:PB8  echo1:PB6 echo2:PB7
       delay_us(20);
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8| GPIO_PIN_5, GPIO_PIN_RESET); // End start signal

       while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0); // Wait for initial low voltage level
       OpenTimerForHc() ; // Start timing when high voltage appears

       while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 1); // Wait for high voltage level to disappear
       CloseTimerForHc(); // Stop timing
       t = Test_L(); // Process time to calculate distance
       test_length = (t / 29.0); // Convert to distance in cm
    }

   
}

// Function to get distance values from ultrasonic modules
void get_hc_da(void)
{

//	  R_data=Hcsr04GetLength_R();
//	  L_data=Hcsr04GetLength_L();
	Test();
  HAL_Delay(300);


   // return lengthTemp_R; // Return distance value
}   
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Start base timer for ultrasonic measurement
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   get_hc_da();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Main loop where the ultrasonic data is collected and printed
//		R_data= Hcsr04GetLength_R(); 
//		HAL_Delay(300);
//		L_data=Hcsr04GetLength_L(); 
//		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);  //send start signal	  
//      delay_us(20);
//      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET); //End start signal
//     // delay_us(10);
//			HAL_Delay(300);//echo
//    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number */
  /* Ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
