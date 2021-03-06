/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "sdmmc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HM10_BleModule.h"
#include "LF_AppMain.h"
#include "Encoders_Module.h"

#include <stdio.h>

#include "ranging_vl53l0x.h"
#include "vl53l0x_api.h"

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
//FRESULT FatFsResult;
//FATFS SdFatFs;
//FIL SdCardFile;
//
//uint8_t bytes;
//char data[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	// Check if UART2 trigger the Callback
	if(huart->Instance == USART2)
	{
		// Start listening again
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, HM10BLE_App.ReceiveBuffer, ReceiveBufferSize);
		HM10BLE_RxEventCallback(Size); //Application/Src/HM10_BleModule.c
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	// Check if UART2 triggered the Callback
	if(huart->Instance == USART2)
	{
		HM10BLE_TxCmpltEventCallback(); //Application/Src/HM10_BleModule.c
	}
}

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDMMC1_SD_Init();
  MX_ADC1_Init();
  MX_TIM15_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_I2C2_Init();
  MX_TIM12_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  //Activate 100usTimer
  LL_TIM_EnableIT_CC1(TIM2);
  LL_TIM_EnableCounter(TIM2);

/* MPU-6050 Detected at 0x69 &i2c1 */

  LF_App_MainConfig(); //Application/Src/LF_AppMain

//  static VL53L0X_Dev_t device;
//
//  static volatile uint16_t res = 0;
//
//  device.I2cHandle=&hi2c2;
//  device.I2cDevAddr=0x52;
//  device.Present=0;
//  device.Id=0;
//
//  initSensor( &device );

//  static uint8_t data_ready;
//  static VL53L0X_RangingMeasurementData_t result;
//  static VL53L0X_Error Status;

static uint32_t SavedTimeLocalTest =0 ;

//
  // FatFS mount init
  //
//  FatFsResult = f_mount(&SdFatFs, "", 1);
//
//  //
//  // FatFS mount init error check
//  //
//  if(FatFsResult != FR_OK)
//  {
//  	  bytes = sprintf(data, "FatFS mount error.\n\r");
//  	  HAL_UART_Transmit(&huart2, (uint8_t*)data, bytes, 1000);
//  }
//  else
//  {
//  	  bytes = sprintf(data, "FatFS mounted.\n\r");
//  	  HAL_UART_Transmit(&huart2, (uint8_t*)data, bytes, 1000);
//
//  	  //
//  	  // Open file on SD for writing
//  	  //
//  	  FatFsResult = f_open(&SdCardFile, "test.txt", FA_WRITE|FA_CREATE_ALWAYS);
//
//  	  //
//  	  // File open error check
//  	  //
//  	  if(FatFsResult != FR_OK)
//  	  {
//  		  bytes = sprintf(data, "No test.txt file. Can't create.\n\r");
//  		  HAL_UART_Transmit(&huart2, (uint8_t*)data, bytes, 1000);
//  	  }
//  	  else
//  	  {
//  		  bytes = sprintf(data, "File opened.\n\r");
//  		  HAL_UART_Transmit(&huart2, (uint8_t*)data, bytes, 1000);
//
//  		  //
//		  //	Print something to this file
//		  //
//		  for(uint8_t i = 0; i < 10; i++)
//		  {
//			  f_printf(&SdCardFile, "Line number %d.\n", i);
//		  }
//
//		  //
//		  // Close file
//		  //
//		  FatFsResult = f_close(&SdCardFile);
//
//		  bytes = sprintf(data, "File closed.\n\r");
//		  HAL_UART_Transmit(&huart2, (uint8_t*)data, bytes, 1000);
//
//
//	  	  //
//	  	  // Open file on SD for writing
//	  	  //
//	  	  FatFsResult = f_open(&SdCardFile, "test.txt", FA_READ);
//
//	  	  //
//	  	  // File open error check
//	  	  //
//	  	  if(FatFsResult != FR_OK)
//	  	  {
//	  		  bytes = sprintf(data, "No test.txt file. Can't open. \n\r");
//	  		  HAL_UART_Transmit(&huart2, (uint8_t*)data, bytes, 1000);
//	  	  }
//	  	  else
//	  	  {
//	  		  UINT len;
//	  		  do
//	  		  {
//	  			  len = 0;
//		  		  f_read(&SdCardFile, data, 10, &len);
//		  		  HAL_UART_Transmit(&huart2, (uint8_t*)data, len, 1000);
//	  		  }while(len > 0);
//
//			  //
//			  // Close file
//			  //
//			  FatFsResult = f_close(&SdCardFile);
//	  	  }
//  	  }
//  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(SavedTimeLocalTest + 1000 < HAL_GetTick() )
		  {
			  SavedTimeLocalTest = HAL_GetTick() ;

//		        Status=VL53L0X_GetMeasurementDataReady(&device, &data_ready);
//
//		        if( Status == VL53L0X_ERROR_NONE )
//		        {
//		            Status = VL53L0X_GetRangingMeasurementData(&device, &result);
//
//		            HAL_UART_Transmit(&huart2, "SFFS", 20, 100); result.RangeMilliMeter;
//
//		            if (Status == VL53L0X_ERROR_NONE)
//		            {
//		                Status = VL53L0X_ClearInterruptMask(&device,0);
//		            }
//		        }
		  }

	  LF_App_MainTask(); //Application/Src/LF_AppMain
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 16;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static VL53L0X_Error initSensor( VL53L0X_Dev_t * device )
{
    VL53L0X_Error Status=VL53L0X_ERROR_NONE;


    static uint32_t refSpadCount     = 0;
    static uint8_t  isApertureSpads  = 0;
    static uint8_t  VhvSettings      = 0;
    static uint8_t  PhaseCal         = 0;

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status=VL53L0X_SetDeviceAddress( device, 0x51 );
        device->I2cDevAddr=0x51;
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status=VL53L0X_DataInit( device );
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status=VL53L0X_StaticInit( device );
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_PerformRefSpadManagement( device, &refSpadCount, &isApertureSpads);
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_PerformRefCalibration( device, &VhvSettings, &PhaseCal);
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status=VL53L0X_SetReferenceSpads( device, refSpadCount, isApertureSpads);
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status=VL53L0X_SetRefCalibration( device, VhvSettings, PhaseCal);
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status=VL53L0X_SetDeviceMode( device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_SetLimitCheckValue( device, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536) );
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_SetLimitCheckValue( device, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32*65536) );
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status =VL53L0X_SetMeasurementTimingBudgetMicroSeconds( device,	20000 );
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status=VL53L0X_StartMeasurement( device );
    }

    return Status;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
