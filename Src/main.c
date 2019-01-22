/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TRUE 				1
#define FALSE 			0
#define VREF				2.91		/* Voltaje de referencia del ADC */
#define ADC_12BIT		4096		/* Cuenta Maxima del ADC de 12 bits */
#define P_HUM25			0.7275	/* 25% de 3V */
#define	P_HUM75			2.1825	/* 75% de 3V */
#define TIMEOUT_EOC	5			/*espera 5 milisegundos por End Of Convertion y ADC no ha terminado, ha habido un error*/
#define RIEGO_ON		5			/* riega por 5 segundos */
#define RIEGO_OFF		10		/*Riego apagado por 10 segundos*/
#define	OFF					0
#define	ON					1
#define TIMEOUT_EOC	5
#define PWM_MAXPOS	80		/* Corresponde a 180º en el plano cartesiano */
#define PWM_MIN			330		/* Corresponde a 0º en el plano cartesiano */
#define MODO_SENSOR				1
#define MODO_TEMPORIZADOR	2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;			/* Timers para las interrupciones usadas en el PWM */
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

int						task_Pending;	/* Flag que indica que una conversión se ha lanzado y hay que recoger el valor */
double 				ADC_Read_fValue;	/* Variable donde se almacena el valor correspondiente a la lectura del ADC*/
int 					cuenta;	/* numero de ticks transcurrido desde el inicio de un ciclo de riego */
int						estado= OFF;					/* estado del prceso de riego */
int 					Modo= MODO_TEMPORIZADOR;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

double Binary_to_Float(uint16_t valor);
double Leer_ADC(void);
void Tick_Iam_alive(void);
void Start_ADC_Conversion(void);
void Medidor_Humedad_Riego(double fADC);
void Leer_Boton_ChangeLedPIN13(void);
void Temporizacion_Riego(void);
void Cambio_Modo(void);
void Inicia_Estado_Temp(void);

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
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2); 
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 

	task_Pending= FALSE;	/* FALSE= 0, TRUE<> 0*/
  Inicia_Estado_Temp();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN 3 */
		Cambio_Modo();
		/* Leer estado del boton de usuario */
		Leer_Boton_ChangeLedPIN13();

		if (task_Pending) {
			if (Modo== MODO_TEMPORIZADOR){
				Temporizacion_Riego();
			}
			else{
				ADC_Read_fValue = Leer_ADC();
				Medidor_Humedad_Riego(ADC_Read_fValue);
			}
			task_Pending= FALSE;
		}
		
		/* USER CODE END 3 */
  }
  
	/* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
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
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 800;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * Realiza el posicionamiento de un servo 
	* @param  Pos: Número de gardo con respecto al plano cartesiano 0º a 180º
  * @retval regresa el valor leido convertido en double
  */
void Inicia_Estado_Temp(void){	
// Inicialización en no riego
	estado= OFF;
	cuenta= 0;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MAXPOS);
	HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);
}



/**
  * Realiza la inicialización del conversor ADC y espera por el valor 
  * @param  None
  * @retval regresa el valor leido convertido en double
  */
double Leer_ADC(void){
	
	HAL_ADC_Start (&hadc1); /* arranco conversion*/
	if (HAL_ADC_PollForConversion (&hadc1, TIMEOUT_EOC)== HAL_OK)
		return Binary_to_Float(HAL_ADC_GetValue (&hadc1)); 
	return -1.; 				/* else conversion fallo-> valor retornado HAL_TIMEOUT*/
}

/**
  * Cambia el estado del sistema de riego en base a la medición del % de humedad  
  * @param  fADC, contiene la medición de la humedad y sobre la que se toma la desición
  * @retval None
  */
void Medidor_Humedad_Riego(double fADC){
	
	if(fADC > P_HUM75){
		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MAXPOS);
	} 
	if(fADC < P_HUM25){
		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MIN);
		
	}
}

double Binary_to_Float(uint16_t valor){

	return VREF * valor/ADC_12BIT;
}

/**
  * Cambio continuamente el estado del Led Rojo Pin_14 para indicar que estamos vivos 
  * @param  None
  * @retval None
  */
void Tick_Iam_alive(void){

	static GPIO_PinState rember_Last= GPIO_PIN_RESET;
	
        /* Toggle LED */
			if (rember_Last == GPIO_PIN_RESET){
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
				rember_Last= GPIO_PIN_SET;
			}
			else { /* if (rember_Last == GPIO_PIN_SET) */
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
				rember_Last= GPIO_PIN_RESET;
			}
}

/**
  * Indica al ADC que comience la conversion
  * @param  None
  * @retval None
  */

void Start_ADC_Conversion(void){
	HAL_ADC_Start (&hadc1); /* Start conversation */
}

/**
  * @Interrupción: Funcion Callback no bloqueante- llamada por la interupción
  * @param  htim: TIM handle
  * @retval None
  */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	
    if (htim->Instance == htim2.Instance)
    {
			Tick_Iam_alive();
			task_Pending= TRUE; /* Indica que hay una tarea pendiente */
    }
}

/**
  * @Leer estado del Botton externo PA6 
	* @cambio de medidor de humedad a temporizador
  * @param  NONE
  * @retval Modo
  */

void Cambio_Modo(void){
	if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)){
		if(Modo== MODO_SENSOR){ 
			Modo= MODO_TEMPORIZADOR;
			Inicia_Estado_Temp();
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		}
		else{
			Modo= MODO_SENSOR;
			Inicia_Estado_Temp();
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);	// Modo SENSOR Pin ON  Led proto Rojo
		}
		HAL_Delay(1000);
	}
}

/**
  * @Leer estado del Botton Usuario, tarjeta STM32F411, 
	* @Enciente o apaga PIN13 dependiendo del estado
  * @param  NONE
  * @retval None
  */

void Leer_Boton_ChangeLedPIN13(void)
	{
		if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET); // Boton presionado se enciende luz naranja
			if( estado== ON){
				estado= OFF;
				cuenta= 0;
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MAXPOS);
				HAL_Delay(1000);
			}
			else{
				estado= ON;
				cuenta= 0;
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MIN);
				HAL_Delay(1000);
			}
		} else {
						HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
		}
	}
	
	/**
  * @Abre o cierrar la valvula en función del tiempo 5 segundo de riego y 10 segundo sin riego
	* @La salida PIN_5 se utiliza para controlar una valvula
  * @param  NONE
  * @retval None
  */
	void Temporizacion_Riego(void){
			cuenta++;
			if (estado==ON && cuenta >= RIEGO_ON){
				cuenta= 0;
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MAXPOS);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
				
				// se suspende el riego
				estado= OFF;
			}
			if (estado==OFF && cuenta >= RIEGO_OFF){
				cuenta= 0;
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MIN);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
				estado= ON;
			}
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
