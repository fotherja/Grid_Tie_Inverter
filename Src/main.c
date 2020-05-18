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
	// Changes -> Increased rate of PLL correction
	// Changed DMA to half-words
	// 
	//
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "support.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define					ADC_OFFSET								2048
#define         SINE_STEPS          			256                         							// Number of steps to build our sinewave in

#define					PLL_ADJ_RATE_LIMIT				500.0f																		// Limit alterations in our local oscillator freq

#define					PID_Kp										1.0f
#define					PID_Ki										1.0f
#define					PID_Kd										0.0f
#define					PID_PERIOD								0.0001f
#define					PID_Min									 -1024.0f
#define					PID_Max										1024.0f

#define					PID_LOOP_PERIOD						16800																			// Timer9 ticks for PID iterations (100us)
#define					SINE_STEP_PERIOD_BASE			13125																			// Timer9 ticks for Sine lookup index increments for 50Hz
#define					SINE_STEP_PERIOD_BASE_B		40000
#define					PROTECT_LOOP_PERIOD				33600																			// Timer9 ticks for protection routine (200us)

#define					ADC_BUFFER_LENGTH					11																				// SHOULD BE AN ODD NUMBER

#define					RMS_UPPER_LIMIT						500000																		// Actually, we don't bother with the root...
#define					RMS_LOWER_LIMIT						300000
#define					FREQ_UPPER_LIMIT					131
#define					FREQ_LOWER_LIMIT				 -131

#define					STARTUP_MASK_CNT					5000																			// Allow 100ms to sync phase with the mains at startup 
#define					RUNNING_MASK_CNT					3000																			// Once running, if out of phase for > 60ms, we shut off
#define					RESTART_MASK_CNT			 	 -1000																			// Monitor mains Vrms for 1sec before considering re-engaging 
#define					GRID_BAD_FAIL_RATE				11																				//
#define					GRID_OK										0

#define					START_UP_CURRENT					0.25f																			// At startup we gradually increase our output current from this
#define					TARGET_OUTPUT_CURRENT			3.0f																			// To this
#define					CURRENT_RAMP_RATE					0.00004f																		// Every PROTECT_LOOP_PERIOD this increments into I output demand

#define 				CONSTRAIN(x,lower,upper)	((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))
#define					MAX(x, y) 								(((x) > (y)) ? (x) : (y))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */
float    		Sine_LookupF[256]          		=     {0.00,6.28,12.56,18.83,25.09,31.34,37.56,43.77,49.94,56.09,62.20,68.28,74.31,80.30,86.24,92.13,
																								97.97,103.74,109.45,115.10,120.68,126.18,131.61,136.96,142.23,147.41,152.50,157.50,162.40,167.21,171.92,176.52,
																								181.02,185.41,189.68,193.85,197.89,201.82,205.62,209.30,212.86,216.28,219.58,222.74,225.77,228.67,231.42,234.04,
																								236.51,238.85,241.04,243.08,244.98,246.73,248.33,249.78,251.08,252.23,253.23,254.07,254.77,255.31,255.69,255.92,
																								256.00,255.92,255.69,255.31,254.77,254.07,253.23,252.23,251.08,249.78,248.33,246.73,244.98,243.08,241.04,238.85,
																								236.51,234.04,231.42,228.67,225.77,222.74,219.58,216.28,212.86,209.30,205.62,201.82,197.89,193.85,189.68,185.41,
																								181.02,176.52,171.92,167.21,162.40,157.50,152.50,147.41,142.23,136.96,131.61,126.18,120.68,115.10,109.45,103.74,
																								97.97,92.13,86.24,80.30,74.31,68.28,62.20,56.09,49.94,43.77,37.56,31.34,25.09,18.83,12.56,6.28,	
																								0.00,-6.28,-12.56,-18.83,-25.09,-31.34,-37.56,-43.77,-49.94,-56.09,-62.20,-68.28,-74.31,-80.30,-86.24,-92.13,
																								-97.97,-103.74,-109.45,-115.10,-120.68,-126.18,-131.61,-136.96,-142.23,-147.41,-152.50,-157.50,-162.40,-167.21,-171.92,-176.52,
																								-181.02,-185.41,-189.68,-193.85,-197.89,-201.82,-205.62,-209.30,-212.86,-216.28,-219.58,-222.74,-225.77,-228.67,-231.42,-234.04,
																								-236.51,-238.85,-241.04,-243.08,-244.98,-246.73,-248.33,-249.78,-251.08,-252.23,-253.23,-254.07,-254.77,-255.31,-255.69,-255.92,
																								-256.00,-255.92,-255.69,-255.31,-254.77,-254.07,-253.23,-252.23,-251.08,-249.78,-248.33,-246.73,-244.98,-243.08,-241.04,-238.85,
																								-236.51,-234.04,-231.42,-228.67,-225.77,-222.74,-219.58,-216.28,-212.86,-209.30,-205.62,-201.82,-197.89,-193.85,-189.68,-185.41,
																								-181.02,-176.52,-171.92,-167.21,-162.40,-157.50,-152.50,-147.41,-142.23,-136.96,-131.61,-126.18,-120.68,-115.10,-109.45,-103.74,
																								-97.97,-92.13,-86.24,-80.30,-74.31,-68.28,-62.20,-56.09,-49.94,-43.77,-37.56,-31.34,-25.09,-18.83,-12.56,-6.28};
										

int32_t Debug_PLL_Out, Debug_PLL_iterm, Debug_PLL_In;																								
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM9_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */
void DRV_Config_Calibration(void);
void DRV_Config_Six_Wire(void);
void DRV_Config_Three_Wire(void);
void DRV_Enable(void);
void DRV_Disable(void);
																								
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
	static int16_t 		ADC_Buffer_A[ADC_BUFFER_LENGTH];																// The DMA keeps these buffers filled with fresh ADC readings
	static int16_t 		ADC_Buffer_B[ADC_BUFFER_LENGTH];
	static int16_t		ADC_Line_V;																								
																								
	static PIDControl PID_I;																													// PIDControl struct instance	
	static PIDControl	PLL_PID;
															
	static int16_t		OFFSET_A						= 0;				
	static int16_t		OFFSET_B						= 0;
		
	static uint32_t 	Mains_RMS;	
	static float			I_Output_Demand 		= START_UP_CURRENT;																			
	static int16_t		Measured_I;																											// Stores our most up to date output current reading 																								
	
	static int32_t 		Mains_Good_Bad_Counter;
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM9_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
	//-------------------------------------------------------------------------------------------------------------------------------
	//###############################################################################################################################
	//-------------------------------------------------------------------------------------------------------------------------------		
	// 1) Start the 3 ADC engines
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buffer_A, ADC_BUFFER_LENGTH);
		HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC_Buffer_B, ADC_BUFFER_LENGTH);
		HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&ADC_Line_V, 1);	
		
	// 2) Configure our PID parameters
		PIDInit(&PID_I, PID_Kp, PID_Ki, PID_Kd, PID_PERIOD, PID_Min, PID_Max, AUTOMATIC, DIRECT);
		PIDInit(&PLL_PID, 2.0e-3, 5.0e-5, 0.0, 7.8e-5, -PLL_ADJ_RATE_LIMIT, PLL_ADJ_RATE_LIMIT, AUTOMATIC, DIRECT);
		PIDSetpointSet(&PLL_PID, 0.0);
	
	// 3) Start our PWM driver which uses Timer1 to power our H-bridge. Both channels start with Duty = 0
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);																			
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	
	// 4) Enable & Configure the DRV8301 over SPI for shunt calibration
		DRV_Enable();
		DRV_Config_Calibration();

	// 5) Read the shorted current amplifier to obtain their respective offsets 
		for(int8_t i = 0; i < 10; i++)	{
			HAL_Delay(100);
			OFFSET_A += Get_Current_Median(ADC_Buffer_A, ADC_BUFFER_LENGTH, 0.0);
			OFFSET_B += Get_Current_Median(ADC_Buffer_B, ADC_BUFFER_LENGTH, 0.0);
		}
		
		OFFSET_A /= -10;																																// We just summed up 10 readings so /10 to get avg
		OFFSET_B /= -10;
		
		OFFSET_A = CONSTRAIN(OFFSET_A, 1948, 2148);																			// Just for safety, constrain the allowable offset
		OFFSET_B = CONSTRAIN(OFFSET_B, 1948, 2148);
		
		DRV_Config_Six_Wire();
		HAL_Delay(10);
		
	// 6) Ensure the mains is as we expect in terms of voltage before joining at a Zero-Crossing-Point
		Mains_Good_Bad_Counter = RESTART_MASK_CNT;
		while(Mains_Good_Bad_Counter < GRID_OK)		{
			HAL_Delay(1);
			Mains_RMS = Integrate_Mains_RMS(ADC_Line_V - ADC_OFFSET);											// Update our Mains RMS measurement
			
			if(Mains_RMS > RMS_UPPER_LIMIT || Mains_RMS < RMS_LOWER_LIMIT)
				Mains_Good_Bad_Counter = RESTART_MASK_CNT;																	// If out of range, reset the count
			else
				Mains_Good_Bad_Counter++;
		}
		
		Mains_Good_Bad_Counter = STARTUP_MASK_CNT;
		Await_ZCP(&ADC_Line_V);																													// This blocks until a zero crossing point
		DRV_Config_Three_Wire();																												
	//------------------------------------------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {				
		// Keeps our local oscillator synchronised to the grid by implimenting a PLL - This works well!
		static uint16_t	TimeStamp_Sine = 0, Sine_Index = 0, Time_Diff;
		Time_Diff = (uint16_t)htim9.Instance->CNT - TimeStamp_Sine;
		if(Time_Diff > SINE_STEP_PERIOD_BASE && Time_Diff < SINE_STEP_PERIOD_BASE_B) {   																				   
			TimeStamp_Sine += SINE_STEP_PERIOD_BASE; 				
			
			int32_t Delayed_Line_V_Readings = Signal_Delay(ADC_Line_V - ADC_OFFSET);			
			PLL_PID.input = Sine_LookupF[Sine_Index] * (float)Delayed_Line_V_Readings;		
			PIDCompute(&PLL_PID);							
			
			TimeStamp_Sine -= (int32_t)PLL_PID.output;
			
			if(++Sine_Index >= SINE_STEPS)	{Sine_Index = 0;}
			
			Debug_PLL_iterm = (int32_t)PLL_PID.iTerm;
			Debug_PLL_Out = (int32_t)PLL_PID.output;
			Debug_PLL_In = (int32_t)PLL_PID.input;
		}		
		
		
		// Run the PID if its period has elapsed - I believe this is where our problems lie!
		static uint16_t	TimeStamp_PID = 0;		
		Time_Diff = htim9.Instance->CNT - TimeStamp_PID;
		if(Time_Diff > PID_LOOP_PERIOD) {   																				   
			TimeStamp_PID += PID_LOOP_PERIOD;
			
			static int16_t Duty_Cycle = 0;
			
			if(Duty_Cycle >= 0)	
				Measured_I = -Get_Current_Median(ADC_Buffer_A, ADC_BUFFER_LENGTH, OFFSET_A);				
			else	
				Measured_I = Get_Current_Median(ADC_Buffer_B, ADC_BUFFER_LENGTH, OFFSET_B); 
			
			// Write the measured current to the DAC so we can debug it.
			uint32_t DAC_Data = (uint32_t)(Measured_I + 2048);
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_Data);
			
			//PIDSetpointSet(&PID_I, Sine_LookupF[Sine_Index] * I_Output_Demand);						// This is definitely what we want. Our output current in phase with the grid voltage
			//PIDInputSet(&PID_I, (float)Measured_I);																				// I don't trust our current readings very much...
			//PIDCompute(&PID_I);
				
			Duty_Cycle = (int16_t)(Sine_LookupF[Sine_Index]*1.6f); // + (int16_t)PIDOutputGet(&PID_I);				// I don't feel this needs to happen. The PID should be able to handle everything itself.
				
			if(Duty_Cycle >= 0)	{																													
				htim1.Instance->CCR1 = Duty_Cycle;
				htim1.Instance->CCR2 = 0;							
			}
			else	{
				htim1.Instance->CCR1 = 0;
				htim1.Instance->CCR2 = -Duty_Cycle;
			}
		}

		/*
		// Keep tabs on mains RMS and Frequency. Cut out/in when things go out/in range - This works well!
		static uint16_t	TimeStamp_Protect = 0;		
		Time_Diff = htim9.Instance->CNT - TimeStamp_Protect;
		if(Time_Diff > PROTECT_LOOP_PERIOD) {   																				   
			TimeStamp_Protect += PROTECT_LOOP_PERIOD;		
			
			Mains_RMS = Integrate_Mains_RMS(ADC_Line_V - ADC_OFFSET);											// Update our mains RMS measurement

			//if(Freq_Offset > FREQ_UPPER_LIMIT || Freq_Offset < FREQ_LOWER_LIMIT) 
			//	Mains_Good_Bad_Counter -= GRID_BAD_FAIL_RATE;
			if(Mains_RMS > RMS_UPPER_LIMIT || Mains_RMS < RMS_LOWER_LIMIT)
				Mains_Good_Bad_Counter -= GRID_BAD_FAIL_RATE;
			
			if(Mains_Good_Bad_Counter < RUNNING_MASK_CNT)	
				Mains_Good_Bad_Counter++;
			else
				Mains_Good_Bad_Counter--;
			
			if(Mains_Good_Bad_Counter < GRID_OK)	{
				DRV_Disable();																															// This puts the H-bridge into a high impedance state
				Sine_Index = 0;
				PID_I.iTerm = 0;
				PID_I.lastInput = 0;
				I_Output_Demand 	= START_UP_CURRENT;
				Mains_Good_Bad_Counter = RESTART_MASK_CNT;

				while(Mains_Good_Bad_Counter < GRID_OK)	{																		// Remain paused until the mains RMS goes back in range
					HAL_Delay(1);
					Mains_RMS = Integrate_Mains_RMS(ADC_Line_V - ADC_OFFSET);				
					
					if(Mains_RMS > RMS_UPPER_LIMIT || Mains_RMS < RMS_LOWER_LIMIT)
						Mains_Good_Bad_Counter = RESTART_MASK_CNT;
					else
						Mains_Good_Bad_Counter++;				
				}

				Mains_Good_Bad_Counter = STARTUP_MASK_CNT;
				DRV_Enable();
				DRV_Config_Six_Wire();				
				Await_ZCP(&ADC_Line_V);
				DRV_Config_Three_Wire();	
			}
			else if(I_Output_Demand < TARGET_OUTPUT_CURRENT)
				I_Output_Demand += CURRENT_RAMP_RATE;		
		}		
		*/
		
	//-------------------------------------------------------------------------------------------------------------------------------
	//###############################################################################################################################
	//-------------------------------------------------------------------------------------------------------------------------------			
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */
  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */
  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */
  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */
  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */
  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1023;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */
  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */
  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */
	HAL_TIM_Base_Start(&htim9);
  /* USER CODE END TIM9_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DRV_Enable_GPIO_Port, DRV_Enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV_Enable_Pin */
  GPIO_InitStruct.Pin = DRV_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DRV_Enable_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void DRV_Config_Calibration(void)
{
	uint8_t DRV_dataTxA[2] = {0x10, 0x00};																						// CR1: 6 wire mode  		
	uint8_t DRV_dataTxB[2] = {0x18, 0x38};																						// CR2: Current shunt amp gain = 40 & enable calibration mode
	
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);										
	HAL_SPI_Transmit(&hspi1, DRV_dataTxA, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);											
	HAL_SPI_Transmit(&hspi1, DRV_dataTxB, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}

void DRV_Config_Six_Wire(void)
{
	uint8_t DRV_dataTxA[2] = {0x10, 0x00};																						// CR1: 6 wire mode  
	uint8_t DRV_dataTxC[2] = {0x18, 0x08};																						// CR2: Current shunt amp gain = 40 & disable calibration more 
	
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);												
	HAL_SPI_Transmit(&hspi1, DRV_dataTxA, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);													
	HAL_SPI_Transmit(&hspi1, DRV_dataTxC, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}

void DRV_Config_Three_Wire(void)
{
	uint8_t DRV_dataTxD[2] = {0x10, 0x08};																						// CR1: 3 wire mode
	
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);											
	HAL_SPI_Transmit(&hspi1, DRV_dataTxD, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}

void DRV_Disable(void)
{
	HAL_GPIO_WritePin(DRV_Enable_GPIO_Port, DRV_Enable_Pin, GPIO_PIN_RESET);					// Puts all MOSFETs into high impedance mode. All registers reset
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
}

void DRV_Enable(void)
{
	HAL_Delay(20);
	HAL_GPIO_WritePin(DRV_Enable_GPIO_Port, DRV_Enable_Pin, GPIO_PIN_SET);						// Enable the DRV8301, it boots up in its reset state.
	HAL_Delay(20);
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
