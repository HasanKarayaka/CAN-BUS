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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define  CAN_IDE_32 0b00000100 // bu ıde bitini ekleme sebebimiz bazı durumlarda hem stdr ıd icin hem extdıd icin calıacaktır bunu engellemek ıcın tanımlıyoruz

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
CAN_HandleTypeDef hcan2;
//CAN_TxHeaderTypeDef Gonderilen_veri;
//CAN_RxHeaderTypeDef Alinan_veri;
//CAN_FilterTypeDef exFilterconfig;
//
//uint8_t count[8]; //gönderilecek değer
//
//uint8_t rcount[8]; //alinacak değer
//uint32_t bilgi[5]; //alim işlemi sirasinda bizden istenen mailbax

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef Gonderilen_veri;
CAN_RxHeaderTypeDef Alinan_veri;
CAN_FilterTypeDef exFilterconfig;

uint8_t count[8]; //gönderilecek değer

uint8_t rcount[8]; //alinacak değer
uint32_t bilgi[5]; //alim işlemi sirasinda bizden istenen mailbax
uint32_t gbilgi;

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
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan2);


  Gonderilen_veri.DLC = 1; //gonderilecek mesaj kac bytelik?
  Gonderilen_veri.IDE = CAN_ID_EXT; // stn mi rxt mi göndereceksin
  Gonderilen_veri.RTR = CAN_RTR_DATA; // o sa mesaj geitrdim diyordu 1 se istek yapiyor
  Gonderilen_veri.ExtId = 0x1FBF9000; // 29 bitlik deger göndericez ve ıd degerimiz bu

  exFilterconfig.FilterActivation = CAN_FILTER_ENABLE; //fiter ative ediyoruz
  exFilterconfig.FilterBank = 0;
  exFilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  exFilterconfig.FilterIdHigh = 0x1FBF9000 >> 13; // 0x1FBF9000 --- 0x1FBF9FFF arasındaki tüm mesajları al stıd bu degerı 5 bıt solaotelerken extıd da 13 saga otelerız
  exFilterconfig.FilterIdLow = 0x1FBF9000 | CAN_IDE_32 << 3; // 32 bit bellegin 29 bitini kullanıyoruz bu yüzden 3 bit sola cekiyorz ki 32 bitin tamamını kulanalım
  exFilterconfig.FilterMaskIdHigh = 0xFFFFF000 >> 13; // f degerıne karsilik gelen sxtd li mesajları alır 0 a denk gelenleri gecirir
  exFilterconfig.FilterMaskIdLow = 0x1FBF9000 | CAN_IDE_32 << 3;
  exFilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  exFilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
// burda sadece 0x1FBF9000 bu ıd degerınden verı alsın ıstıyoruz al sınırları ıkısnı de ıstedıgımız ıd sectık ve makshıgh ı da ıd degerımızı ıcın ayarladık

  HAL_CAN_ConfigFilter(&hcan2, &exFilterconfig);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_CAN_AddTxMessage(&hcan2, &Gonderilen_veri, &count, bilgi);




	  HAL_CAN_AddTxMessage(&hcan2, &Gonderilen_veri, count, &gbilgi);
	  //HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &Alinan_veri, rcount);

	  bilgi[0] = Alinan_veri.DLC;
	  bilgi[1] = Alinan_veri.ExtId;
	  bilgi[2] = Alinan_veri.IDE; //ext id ise sonu 4 gelir
	  bilgi[3] = Alinan_veri.RTR; // Data frame ise sonu 0

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 12;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
