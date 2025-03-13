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
#include "TLV320AIC3104.h"
#include "Audio_IO.h"
#include "Audio_FIR.h"
#include "Audio_QIIR.h"
#include "Audio_Nonlinearity.h"
#include "AntiAlias10k.h"
#include "MesaBoogie_Tight1_SM57.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_OF_POT 4
#define ADC16_RANGE 65535.0f
#define ADC12_RANGE 4095.0f
#define ADC16_TO_1BASE 0.000015259f
#define ADC16_TO_5BASE 0.000076295f
#define ADC16_TO_30BASE 0.0004577706f
#define AUDIO_BUFF_SIZE16 256

#define INT16_TO_FLOAT (1.0f/32767.0f)
#define INT24_TO_FLOAT (1.0f/8388607.0f)
#define UINT32_TO_FLOAT (1.0f/4294967295.0f)
#define UINT16_TO_FLOAT (1.0f/65535.0f)
#define INT32_TO_FLOAT (1.0f/2147483647.0f)
#define UINT24_TO_FLOAT (1.0f/16777215.0f)
#define FLOAT_TO_INT16 32767
#define FLOAT_TO_INT24 8388607
#define FLOAT_TO_UINT32 4294967295.0f
#define FLOAT_TO_INT32 2147483647.0f
#define FLOAT_TO_UINT24 16777215.0f
#define FLOAT_TO_UINT16 65535.0f
#define CAN_PROCEED_STREAM 1U
#define DONT_PROCEED_STREAM 0U
#define ACTUAL_FS 47753.0f
#define ACTUAL_TS 1/ACTUAL_FS
#define IR_LEN 		1024
#define FX_ACTIVE	1
#define FX_BYPASS	0
#define CRUNCH_MODE 1
#define LEAD_MODE 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

float Global_Lvl_Inp = 0.8f;
float Global_Lvl_Out = 0.6f;
AIC320_CTL_OBJ AIC320_codec;
FIR_Struct AntiAliasing_LowPass;
QIIR_Struct Pregain_IIR0;
QIIR_Struct Pregain_IIR1;
QIIR_Struct Pregain_IIR2;
QIIR_Struct Pregain_IIR3;
QIIR_Struct Pregain_IIR4;
QIIR_Struct Pregain_IIR5;
QIIR_Struct Pregain_IIR6;
Drive_Struct GainStage1;
QIIR_Struct MidGain_IIR0;
QIIR_Struct MidGain_IIR1;
QIIR_Struct MidGain_IIR2;
QIIR_Struct MidGain_IIR3;
Drive_Struct GainStage2;
QIIR_Struct PostGain_IIR0;
QIIR_Struct PostGain_IIR1;
QIIR_Struct PostGain_IIR2;
QIIR_Struct PostGain_IIR3;
QIIR_Struct PostGain_IIR4;
QIIR_Struct PostGain_HP;
FIR_Struct Cabinet_IR;


float AntiAlias_LowPass_Buff[ANTI_ALIAS10_SIZE] = {0};
float Cabinet_IR_Buff[IR_LEN] = {0};

typedef enum knob_type{KNOB_OFF, KNOB_GAIN, KNOB_Q, KNOB_FREQ, KNOB_FREQ_GAIN, KNOB_VOL_OUT} Knob_Type;
Knob_Type Current_Knob_Type = KNOB_OFF;

volatile uint16_t POT_DATA[NUM_OF_POT] __attribute__((section(".nocache")));;
float POT_1BASE[NUM_OF_POT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float POT_5BASE[NUM_OF_POT] = {0, 0, 0, 0};
volatile float POT_30BASE[NUM_OF_POT] = {0, 0, 0, 0};
float Control_Knob_Val[NUM_OF_POT] = {0, 0, 0, 0};

uint8_t  fx_switch = FX_BYPASS;
uint8_t  Gain_Mode = LEAD_MODE;
uint8_t Button_Assign_To_Process_ID = 0;
const uint8_t Max_Btn_Assignment = 14;
Run_Status Current_Run_State = ON_RUN;
Run_Status Change_Request_Status = ON_RUN;

volatile uint8_t Stream_Status = 0;
uint8_t I2S_Startup_Sts = 0;

volatile int16_t Audio_ADC_Buff[AUDIO_BUFF_SIZE16]  __attribute__((section(".nocache")));;;
volatile int16_t Audio_DAC_Buff[AUDIO_BUFF_SIZE16]  __attribute__((section(".nocache")));;;
static volatile int16_t *Audio_Inp_Ptr;
static volatile int16_t *Audio_Out_Ptr = &Audio_DAC_Buff[0];

//*************ON THE GO ADJUSTABLE PARM***********************//
float 			Run_Gain_Max = 100.0f;
float 			Run_Gain_Min = 10.0f;
volatile float 	Run_Gain = 0.0f;
float			Run_Gain_Scale = 0.0f;
float			Run_Gain_Conv_Factor = 0.0f;

float  			Run_Vol_Max = 0.9f;
float  			Run_Vol_Min = 0.0f;
volatile float	Run_Vol = 0.0f;
float 			Run_Vol_Scale = 0.0f;
float 			Run_Vol_Conv_Factor = 0.0f;

float			Run_High_Max = 16000.0f;
float			Run_High_Min = 4000.0f;
volatile float 	Run_High = 0.0f;
float			Run_High_Scale = 0.0f;
float			Run_High_Conv_Factor = 0.0f;

float			Run_LowCut_Scale = 270.0f;
float			Run_LowCut_Min = 30.0f;
volatile float	Run_Low = 0.0f;
float			Run_LowCut_Max = 0.0f;
float			Run_LowCut_Conv_Factor = 0.0f;

//***********WATCH, TEST, AND DEBUG HERE!******************//
float wGain = 0;
float wQ = 0;
float wF0 = 0;
float wVolout = 0;
float wGainB = 0;
float wQB = 0;
float wF0B = 0;
float wVoloutB = 0;
float in_floated = 0.0f;
float out_floated = 0.0f;
float tinx = 0;
float tox = 0;
int16_t i1 = 0;
int16_t i2 = 0;
int16_t i2_neg = 0;
int16_t o1 = 0;
int16_t o2 = 0;


int32_t inp_i16_0 = 0;
int32_t inp_i16_1 = 0;
int32_t inp_i16_2 = 0;
int32_t inp_i16_3 = 0;
int32_t inp_i16_s = 0;

int32_t out_i16_0 = 0;
int32_t out_i16_1 = 0;
int32_t out_i16_2 = 0;
int32_t out_i16_3 = 0;
int32_t out_i16_s = 0;


int8_t i2_tmp_int = 0;
int8_t o2_tmp_int = 0;
int8_t i2_tmps_int = 0;
int8_t o2_tmps_int = 0;

float dsp1 = 0.0f;
float dsp2 = 0.0f;
float dsp3 = 0.0f;

int32_t i1_int_combined = 0;
uint32_t i1_uint_combined = 0;
uint32_t i2s = 0;
int32_t o1_int_combined = 0;
uint32_t o_uint_combined = 0;
uint32_t o2s = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

void Process_Audio();
void Check_Current_Knob_Assignment();
void Check_State_Change_Request();
void Init_AntiAlias10_Filter();
void Init_Cabinet_IR();
void ProcessAll_PreGain1();
void ProcessAll_PostGain1();
void ProcessAll_PostGain2();
void AdjustParm();

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
	 HAL_GPIO_WritePin(GPIOD, YELLOW_LED_Pin, GPIO_PIN_SET);
	 Run_Gain = Run_Gain_Min;
	 Run_Gain_Scale = Run_Gain_Max - Run_Gain_Min;
	 Run_Gain_Conv_Factor = Run_Gain_Scale/ADC12_RANGE;

	 Run_Vol = Run_Vol_Min;
	 Run_Vol_Scale = Run_Vol_Max - Run_Vol_Min;
	 Run_Vol_Conv_Factor = Run_Vol_Scale/ADC12_RANGE;

	 Run_High = Run_High_Min;
	 Run_High_Scale = Run_High_Max - Run_High_Min;
	 Run_High_Conv_Factor = Run_High_Scale/ADC12_RANGE;

	 Run_Low = Run_LowCut_Min;
	 Run_LowCut_Max = Run_LowCut_Scale - Run_LowCut_Min;
	 Run_LowCut_Conv_Factor = Run_LowCut_Max/ADC12_RANGE;

	 //Initialize audio processing components:

	Init_FIR_Filter(&AntiAliasing_LowPass, ANTI_ALIAS10_SIZE, &AntiAlias_Coefs[0], &AntiAlias_LowPass_Buff[0], FIR_OFF, 1);

	Init_QIIR_Filter(&Pregain_IIR0, HIGHPASS, 623.0f, 0.6f , 1.0f, ACTUAL_FS, QIIR_ON, 2, CTRL_OFF);
	Update_QIIR_Filter(&Pregain_IIR0);

	Init_QIIR_Filter(&Pregain_IIR1, BELL, 150.0f, 0.74f , 1.51f, ACTUAL_FS, QIIR_ON, 3, CTRL_OFF);
	Update_QIIR_Filter(&Pregain_IIR1);

	Init_QIIR_Filter(&Pregain_IIR2, BELL, 470.0f, 0.9f , 1.1f, ACTUAL_FS, QIIR_OFF, 4, CTRL_OFF);
	Update_QIIR_Filter(&Pregain_IIR2);

	Init_QIIR_Filter(&Pregain_IIR3, BELL, 2700.0f, 0.96f , 1.20f, ACTUAL_FS, QIIR_OFF, 5, CTRL_OFF);
	Update_QIIR_Filter(&Pregain_IIR3);

	Init_QIIR_Filter(&Pregain_IIR4, BELL, 1500.0f, 0.51f , 2.31f, ACTUAL_FS, QIIR_ON, 6, CTRL_OFF);
	Update_QIIR_Filter(&Pregain_IIR4);

	Init_QIIR_Filter(&Pregain_IIR5, BELL, 680.0f, 0.94f , 1.4f, ACTUAL_FS, QIIR_ON, 7, CTRL_OFF);
	Update_QIIR_Filter(&Pregain_IIR5);

	Init_QIIR_Filter(&Pregain_IIR6, LOWPASS, 5000.0f, 0.8f , 1.0f, ACTUAL_FS, QIIR_ON, 8, CTRL_OFF);
	Update_QIIR_Filter(&Pregain_IIR6);


	Init_Drive(&GainStage1, 28.0f, 0.3f, 0.1f, 0.6f, 8.0f, 6.0f, DRIVE_ON, 9, CTRL_OFF);


	Init_QIIR_Filter(&MidGain_IIR0, LO_SHELF , 2000.0f, 0.8f, 0.9f, ACTUAL_FS, QIIR_OFF, 10, CTRL_OFF);
	Update_QIIR_Filter(&MidGain_IIR0);

	Init_QIIR_Filter(&MidGain_IIR1, LOWPASS , 8000.0f, 0.6f, 1.0f, ACTUAL_FS, QIIR_OFF, 11, CTRL_OFF);
	Update_QIIR_Filter(&MidGain_IIR1);

	Init_QIIR_Filter(&MidGain_IIR2, BELL , 720.0f, 0.8f, 0.9f, ACTUAL_FS, QIIR_OFF, 12, CTRL_OFF);
	Update_QIIR_Filter(&MidGain_IIR2);

	Init_QIIR_Filter(&MidGain_IIR3, BELL , 1800.0f, 0.5f, 1.3f, ACTUAL_FS, QIIR_OFF, 13, CTRL_OFF);
	Update_QIIR_Filter(&MidGain_IIR3);


	Init_Drive(&GainStage2, 18.0f, 0.05f, 0.6f, 0.8f, 20.0f, 10.0f, DRIVE_ON, 14, CTRL_OFF);


	Init_QIIR_Filter(&PostGain_IIR0, BELL, 90.0f, 0.96f , 1.652f, ACTUAL_FS, QIIR_ON, 15, CTRL_OFF);
	Update_QIIR_Filter(&PostGain_IIR0);

	Init_QIIR_Filter(&PostGain_IIR1, BELL, 2300.0f, 0.71f , 0.673f, ACTUAL_FS, QIIR_ON, 16, CTRL_OFF);
	Update_QIIR_Filter(&PostGain_IIR1);

	Init_QIIR_Filter(&PostGain_IIR2, LOWPASS, 8000.0f, 0.66f , 1.0f, ACTUAL_FS, QIIR_ON, 17, CTRL_OFF);
	Update_QIIR_Filter(&PostGain_IIR2);

	Init_QIIR_Filter(&PostGain_IIR3, BELL, 350.0f, 0.94f , 0.86f, ACTUAL_FS, QIIR_OFF, 17, CTRL_OFF);
	Update_QIIR_Filter(&PostGain_IIR3);

	Init_QIIR_Filter(&PostGain_IIR4, BELL, 4000.0f, 0.96f , 1.4f, ACTUAL_FS, QIIR_ON, 17, CTRL_OFF);
	Update_QIIR_Filter(&PostGain_IIR4);

	Init_QIIR_Filter(&PostGain_HP, HIGHPASS, 80.0f, 0.7f , 1.0f, ACTUAL_FS, QIIR_ON, 8, CTRL_OFF);
	Update_QIIR_Filter(&PostGain_HP);

	Init_FIR_Filter(&Cabinet_IR, IR_LEN, &Messa0_IR_Coefs[0], &Cabinet_IR_Buff[0], FIR_ON, 18);

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_I2C1_Init();
  MX_I2S1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) POT_DATA, NUM_OF_POT);

  HAL_TIM_Base_Start(&htim8);
  //HAL_TIM_Base_Start_IT(&htim17);



  AIC320_codec.I2C_Handler = &hi2c2;
  AIC320_Init(&AIC320_codec);
  HAL_Delay(1000);
  I2S_Startup_Sts = HAL_I2SEx_TransmitReceive_DMA(&hi2s1,(uint16_t*)Audio_DAC_Buff, (uint16_t*)Audio_ADC_Buff,
		                        AUDIO_BUFF_SIZE16/2);

  HAL_GPIO_WritePin(GPIOD, YELLOW_LED_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (Stream_Status == CAN_PROCEED_STREAM){
		Process_Audio();
	}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 40;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 5;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 6144;
  PeriphClkInitStruct.PLL3.PLL3M = 2;
  PeriphClkInitStruct.PLL3.PLL3N = 13;
  PeriphClkInitStruct.PLL3.PLL3P = 1;
  PeriphClkInitStruct.PLL3.PLL3Q = 3;
  PeriphClkInitStruct.PLL3.PLL3R = 4;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_PLL3;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T8_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x009098E4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x009098E4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_FULLDUPLEX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s1.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s1.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  hi2s1.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 24000-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 2000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CODEC_NRST_Pin|YELLOW_LED_Pin|RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT_SIGN_GPIO_Port, OUT_SIGN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN_1_Pin BTN_2_Pin */
  GPIO_InitStruct.Pin = BTN_1_Pin|BTN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_3_Pin */
  GPIO_InitStruct.Pin = BTN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CODEC_NRST_Pin YELLOW_LED_Pin RED_LED_Pin */
  GPIO_InitStruct.Pin = CODEC_NRST_Pin|YELLOW_LED_Pin|RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT_SIGN_Pin */
  GPIO_InitStruct.Pin = OUT_SIGN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT_SIGN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	Run_Vol		= Run_Vol_Min + (POT_DATA[0] * Run_Vol_Conv_Factor);
	if (Run_Vol < 0.0003f){
		Run_Vol = 0;
	}

	Run_Gain 	= Run_Gain_Min + (POT_DATA[1] * Run_Gain_Conv_Factor);
	Run_Low		= Run_LowCut_Min + Run_LowCut_Max - (POT_DATA[2] * Run_LowCut_Conv_Factor);
	Run_High	= Run_High_Min + (POT_DATA[3] * Run_High_Conv_Factor);

}


void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){
	Audio_Inp_Ptr = &Audio_ADC_Buff[AUDIO_BUFF_SIZE16/2];
	Audio_Out_Ptr = &Audio_DAC_Buff[AUDIO_BUFF_SIZE16/2];
	Stream_Status = CAN_PROCEED_STREAM;
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
	Audio_Inp_Ptr = &Audio_ADC_Buff[0];
	Audio_Out_Ptr = &Audio_DAC_Buff[0];
	Stream_Status = CAN_PROCEED_STREAM;
}

void Process_Audio(){
	static float Left_Stream_Inp = 0.0f;
	static int32_t Left_Stream_Inp_Int = 0;

	static float Left_Stream_Out = 0.0f;
	static int32_t Left_Stream_Out_Int = 0;

	//static float Right_Stream_Inp;
	//static float Right_Stream_Out;

	Global_Lvl_Out = Run_Vol;
	GainStage1.Inp_Lvl = Run_Gain;

	PostGain_IIR2.F0 = Run_High;
	Update_QIIR_Filter(&PostGain_IIR2);

	PostGain_HP.F0 = Run_Low;
	Update_QIIR_Filter(&PostGain_HP);

	for (int n = 0; n < (AUDIO_BUFF_SIZE16/2) - 3 ; n += 4){

		inp_i16_0 = ((int32_t)Audio_Inp_Ptr[n]<<8) & 0x0000FF00 ;
		inp_i16_1 = ((int32_t)Audio_Inp_Ptr[n]<<8) & 0x00FF0000;
		inp_i16_2 = ((int32_t)Audio_Inp_Ptr[n+1]<<24) & 0xFF000000;


		i1_int_combined  =  (  ((int32_t)inp_i16_0)  |  ((int32_t)inp_i16_1)   |  ((int32_t)inp_i16_2)  );
		Left_Stream_Inp_Int = i1_int_combined;


		Left_Stream_Inp =  (float) (INT32_TO_FLOAT * Left_Stream_Inp_Int);


		if (fx_switch == FX_ACTIVE){
			//Do some magic here :
			Left_Stream_Out = Left_Stream_Inp;
			Left_Stream_Out = Process_FIR_Filter(&AntiAliasing_LowPass, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			//Process IIR0
			Left_Stream_Out = Process_QIIR_Filter(&Pregain_IIR0, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			//Process IIR1
			Left_Stream_Out = Process_QIIR_Filter(&Pregain_IIR1, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			//Process IIR2
			Left_Stream_Out = Process_QIIR_Filter(&Pregain_IIR2, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			//Process IIR3
			Left_Stream_Out = Process_QIIR_Filter(&Pregain_IIR3, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			//Process IIR4
			Left_Stream_Out = Process_QIIR_Filter(&Pregain_IIR4, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			//Process IIR5
			Left_Stream_Out = Process_QIIR_Filter(&Pregain_IIR5, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			//Process IIR6
			Left_Stream_Out = Process_QIIR_Filter(&Pregain_IIR6, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			//Gain stage 1
			Update_Drive(&GainStage1);
			Left_Stream_Out = Process_Drive(&GainStage1, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			Left_Stream_Out = Process_QIIR_Filter(&MidGain_IIR0, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			//Process Midgain IIR1
			Left_Stream_Out = Process_QIIR_Filter(&MidGain_IIR1, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			//Process Midgain IIR2
			Left_Stream_Out = Process_QIIR_Filter(&MidGain_IIR2, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			//Process Midgain IIR3
			Left_Stream_Out = Process_QIIR_Filter(&MidGain_IIR3, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			if (Gain_Mode != CRUNCH_MODE){
				//Gain stage 2
				Update_Drive(&GainStage2);
				Left_Stream_Out = Process_Drive(&GainStage2, Left_Stream_Inp);
				Left_Stream_Inp = Left_Stream_Out;
			}

			Left_Stream_Out = Process_QIIR_Filter(&PostGain_IIR0, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			Left_Stream_Out = Process_QIIR_Filter(&PostGain_IIR1, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			Left_Stream_Out = Process_QIIR_Filter(&PostGain_IIR2, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			Left_Stream_Out = Process_QIIR_Filter(&PostGain_IIR3, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			Left_Stream_Out = Process_QIIR_Filter(&PostGain_IIR4, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			Left_Stream_Out = Process_QIIR_Filter(&PostGain_HP, Left_Stream_Inp);
			Left_Stream_Inp = Left_Stream_Out;

			Left_Stream_Out = Process_FIR_Filter(&Cabinet_IR, Left_Stream_Inp);
		}
		else{
			Left_Stream_Out = Left_Stream_Inp;
		};

		Left_Stream_Out = Left_Stream_Out * Global_Lvl_Out;

		if (Left_Stream_Out > 0.085){
			HAL_GPIO_WritePin(GPIOD, YELLOW_LED_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOD, YELLOW_LED_Pin, GPIO_PIN_RESET);
		}


		Left_Stream_Out_Int =  ( Left_Stream_Out * FLOAT_TO_INT32);

		out_i16_2 = (Left_Stream_Out_Int>>24) 	& 0x000000FF;
		out_i16_1= (Left_Stream_Out_Int>>8)	& 0x000000FF;
		out_i16_0= (Left_Stream_Out_Int>>8)	& 0x0000FF00;


		Audio_Out_Ptr[n] = out_i16_1 | out_i16_0;
		Audio_Out_Ptr[n+1] = out_i16_2;

		Audio_Out_Ptr[n+2] = Audio_Inp_Ptr[n+2];
		Audio_Out_Ptr[n+3] = Audio_Inp_Ptr[n+3];

	}
	Stream_Status = DONT_PROCEED_STREAM;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch (GPIO_Pin){
	case BTN_1_Pin:
		if (fx_switch == FX_ACTIVE){
			fx_switch = FX_BYPASS;
		}
		else{
			fx_switch = FX_ACTIVE;
		}
		break;


	case BTN_2_Pin:

		if (Gain_Mode == LEAD_MODE){
			Gain_Mode = CRUNCH_MODE;
		}
		else
		{
			Gain_Mode = LEAD_MODE;
		}
		break;

	case BTN_3_Pin:
	default:
		break;
	}
}

void Check_State_Change_Request(){
	if ((Change_Request_Status == DONE_APPLY) || (Change_Request_Status == DONE_CANCEL)){
		Current_Run_State = ON_RUN;
		Button_Assign_To_Process_ID = 0;
		Change_Request_Status = ON_RUN;
	}
}





/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
