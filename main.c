/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stdlib.h"
#include "math.h"
#include "stdio.h"
#include "string.h"

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

#define ARM_MATH_CM4
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- CONFIGURACIÓN DEL GRÁFICO ---
#define GRAPH_X      10   // Margen izquierdo
#define GRAPH_Y      230  // Posición Y de la base (0 es arriba, 240 es abajo)
#define GRAPH_W      300  // Ancho del gráfico
#define GRAPH_H      180
#define ADC_MAX      4095 // Resolución simulada

#define BUFFER_RAW_LEN   512
#define BUFFER_GRAF_LEN  300

#define TOPEFFT          15000.0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/*--------------ADC---------------*/
uint8_t counterData = 0;
uint16_t adcRawData = 0;
volatile uint8_t conversionCompleta = 0;
uint16_t adcRawDataDma[BUFFER_RAW_LEN] = {0};  //Array en el cual vamos a almacenar los valores transferidos por la DMA desde el ADC
uint16_t adcRawDataDmaCOPY[BUFFER_RAW_LEN] = {0};
uint16_t bufferAdcParaleloProcs[BUFFER_RAW_LEN] = {0};

uint16_t bufferAdcParaleloGraf[BUFFER_GRAF_LEN] = {0};  //Estos son para graficar la señal pura. NO la FFT.
uint16_t bufferAdcPrevio[BUFFER_GRAF_LEN] = {0};        // Pero que debe tener el mismo tamaño para graficar la FFT
                                                        // porque este tamaño lo definen los pixeles de la pantalla.
uint16_t bufferFFTgraf[BUFFER_GRAF_LEN] = {0};
float bufferFFTMag[BUFFER_RAW_LEN/2] = {0};          //Necesitamos un buffer con los valores tal cual preferiblemente.

char bufferText[64];
volatile uint8_t g_dato_recibido = 0;

volatile char byteCaracterRec; // Variable temporal para 1 byte
volatile uint8_t plantillaInstrRec[64]; // Buffer para guardar el comando
volatile uint8_t posicionEnPlantInstrRec = 0; // Índice de dónde estamos en el buffer
volatile uint8_t banderaInstrCmplt = 0; // Bandera de "comando listo"

volatile uint8_t banderaChar = 0;

char *instruccion;  //Apuntadores que vamos a usar para leer los comandos
char *valor_instr;

//volatile uint8_t contadorInicioProcess = 0;

volatile uint16_t contador_EMA = 0;
uint32_t prev = 0;
uint32_t current = 0;
float alpha = 0.1;

volatile float valorFiltrado = 0;

volatile float escala_graf = 0;  //Escala del grafico en ms.
volatile uint32_t freq_muestreo = 10000;
volatile uint8_t multiplicador_freq = 1;
volatile uint32_t periodo_timer = 1;

uint32_t duty_cycle = 0;
uint8_t caso_led_pwm = 0;


float ultimoAnterior = 0;

arm_rfft_fast_instance_f32 fftHandler;
float fftBufIn[BUFFER_RAW_LEN];
float fftBufOut[BUFFER_RAW_LEN];

volatile float frecuencia_max_graf = 0;

float maxFFTValue = 0;
uint32_t maxFFTIndex = 0;

volatile uint8_t banderaEncoder = 1;

typedef enum{

	inicializar_pantalla,
	tomar_datos_adc,
	procesar_datos,
	calcular_fft,
	graficar,
	graficar_fft,
	error
} EstadoOsciloscopio;

typedef enum{

	modo_graficacion_SR,
	modo_graficacion_FFT,
} ModoOsciloscopio;

volatile ModoOsciloscopio modoActual = modo_graficacion_SR;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void escalaTiempoGraf(void);
void generacionInstruccion(void);
void freqRangeDraw(void);
void controlFreqRange(void);
void time_scale(uint32_t freq_muestreo);
long map(long x, long in_min, long in_max, long out_min, long out_max);
float EMA_Filter(float current, float prev, float alpha);
void zoomPhotoEncoder(void);
void actualizar_PWM_Segun_Grafico(void);
void max_fft_freq(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // PB13
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); // PB14
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); // PB15

//  HAL_TIM_Base_Start(&htim2);

  EstadoOsciloscopio estadoActual = inicializar_pantalla;

  HAL_UART_Receive_IT(&huart2, (uint8_t *)&byteCaracterRec, 1);

//  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcRawDataDma, BUFFER_RAW_LEN);

  arm_rfft_fast_init_f32(&fftHandler, 512);  //Aqui se debe tambien tener en cuenta el tamaño del array para la FFT.

//  uint32_t SAMPLE_RATE_HZ = freq_muestreo;
//  uint16_t FFT_LEN = BUFFER_RAW_LEN;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  generacionInstruccion();

	  zoomPhotoEncoder();

	  actualizar_PWM_Segun_Grafico();

	  switch(estadoActual)
	  {
	  case inicializar_pantalla:

		  // 1. Encender la luz de fondo (Backlight) con PWM
		  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

		  // 2. Inicializar la Pantalla (Librería Matej Artnak)
		  ILI9341_Init();

		  // 3. Rotación Horizontal
		  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);

		  // 4. Fondo Negro
		  ILI9341_Fill_Screen(BLACK);

		  // 5. Dibujar ejes básicos (Opcional)
		  ILI9341_Draw_Hollow_Rectangle_Coord(GRAPH_X-1, GRAPH_Y - GRAPH_H -1, GRAPH_X + GRAPH_W +1, GRAPH_Y +1, WHITE);
		  ILI9341_Draw_Text("EEG SIGNAL - FELIPE PARRA", 10, 10, WHITE, 1, BLACK);

		  estadoActual = tomar_datos_adc;

		  break;
	  case tomar_datos_adc:

		  if(conversionCompleta)
		  {
			  memcpy(adcRawDataDmaCOPY, adcRawDataDma, BUFFER_RAW_LEN * sizeof(uint16_t));  //Tomamos una foto al buffer de la DMA

			    //Ponerlo afuera, no funciona

			  //Asi demos todo el tiempo del switch para que se grafique y se hagan calculos sin estar corriendo

			  conversionCompleta = 0;
			  estadoActual = procesar_datos;
		  }

		  break;

	  case procesar_datos:

		  float auxPrevio = ultimoAnterior;
		  float auxDataFiltered = 0;

		  float valor_escalado = 0;

		  for(int i = 0; i < BUFFER_RAW_LEN; i++)
		  {
			  // Aplicamos el filtro usando el valor previo (sea del buffer anterior o del ciclo anterior)
			  auxDataFiltered = EMA_Filter((float)adcRawDataDmaCOPY[i], auxPrevio, alpha);

			  // Guardamos el valor filtrado actual para la siguiente vuelta del loop
			  auxPrevio = auxDataFiltered;

			  // Mapeo a coordenadas de pantalla (Y-Axis)
			  // Casteamos a int al final
			  valor_escalado = (((4095.0 - auxDataFiltered) * (180.0 / 4095.0)) + 50);


			  bufferAdcParaleloProcs[i] = (int)valor_escalado;
			  fftBufIn[i] = valor_escalado;
		  }

		  // GUARDAMOS EL ESTADO PARA EL SIGUIENTE BUFFER
		  // Importante: Guardamos el último valor FLOTANTE filtrado.
		  ultimoAnterior = auxPrevio;

		  switch(modoActual)
		  {
		  case modo_graficacion_SR:

			  //Debemos recortar el array grande, y volverlo de 300 pixeles. Normal. Para poder graficar en el max de la pantalla
			  memcpy(bufferAdcParaleloGraf, bufferAdcParaleloProcs, BUFFER_GRAF_LEN * sizeof(uint16_t));  //Destino, origen. tamaño de los datos

			  ILI9341_Draw_Filled_Rectangle_Coord(10, 40, 150, 49, BLACK);  //Da exacto con los 130 px al parecer
			  ILI9341_Draw_Text("Y-Axis 3.3V", 10, 40, WHITE, 1, BLACK);

			  estadoActual = graficar;
			  break;

		  case modo_graficacion_FFT:

			  estadoActual = calcular_fft;
			  break;
		  }  //Termina el switch case de eleccion de modo


		  break;

	  case calcular_fft:  //Todo este proceso puede ser una aplicacion totalmente independiente del graficado normal
						  // para evitar cargar mucho el procesamiento y que se retrasen ambas aplicaciones(deteccion y fft)

		  for(uint16_t o = 0; o < 300; o += 1)
		  {
			  bufferFFTgraf[o] = 230;
		  }

		  uint16_t indice_array_fft = 0;

		  float valor_escalado_fft = 0;

		  arm_rfft_fast_f32(&fftHandler, fftBufIn, fftBufOut, 0);  //Creo que el cero es para ver si es la inversa o la directa.

		  for(uint16_t indexfft = 0; indexfft < BUFFER_RAW_LEN; indexfft += 2)  //Para sacar por parejas los valores reales e imaginarios.
		  {
			  float curVal = sqrtf((fftBufOut[indexfft] * fftBufOut[indexfft]) + (fftBufOut[indexfft + 1] * fftBufOut[indexfft + 1]));  //Valor real

			  if(curVal > TOPEFFT)
			  {
				  curVal = TOPEFFT;
			  }
			  //Debe ser convertido a la escala del grafico en la pantalla antes de agregarse, igual que en el modo anterior
			  //pero... cual es nuestro tope?

			  if(indice_array_fft != 0)
			  {
				  valor_escalado_fft = (((TOPEFFT - curVal) * (180.0 / TOPEFFT)) + 50);

				  bufferFFTgraf[indice_array_fft] = (int)valor_escalado_fft;

				  bufferFFTMag[indice_array_fft] = curVal;

			  }else
			  {
				  bufferFFTgraf[indice_array_fft] = 230;
			  }

			  indice_array_fft ++;
		  }

		  max_fft_freq();  //Calculamos la frecuencia max que se grafica, y la escribimos sobre el eje Y.

		  estadoActual = graficar_fft;

		  break;

	  case graficar:

		  freqRangeDraw();

		  // 1. BORRAR señal anterior
		  for (int i = 1; i < GRAPH_W; i++) {
			  ILI9341_Draw_Line(
				  GRAPH_X + (i - 1), bufferAdcPrevio[i - 1],
				  GRAPH_X + i,       bufferAdcPrevio[i],
				  BLACK
			  );
		  }

		  // 2. DIBUJAR señal actual
		  for (int i = 1; i < GRAPH_W; i++) {
			  ILI9341_Draw_Line(
				  GRAPH_X + (i - 1), bufferAdcParaleloGraf[i - 1],
				  GRAPH_X + i,       bufferAdcParaleloGraf[i],
				  YELLOW
			  );
		  }

		  //GUARDAR señal actual como previa
		  memcpy(bufferAdcPrevio, bufferAdcParaleloGraf,
				 BUFFER_GRAF_LEN * sizeof(uint16_t));

  //		  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcRawDataDma, BUFFER_RAW_LEN);
		  // No es necesario parar y reiniciar la
		  // transimision de datos de la DMA al buffer solo si estamos haciendo una copia del buffer, tomamos un foto, y esa
		  //la dibujamos. Esto soluciona inmediatamente el efecto de que la grafica siempre empezara en un valor alto, dado
		  //que con el tiempo de espera cuando se paraba a la DMA, el capacitor tenia mas tiempo de cargarse.

		  estadoActual = tomar_datos_adc;

		  //Si el metodo es el mismo, y estoy dejando de utilizar uno de los buffers, podría dejar un solo
		  //caso de graficación y según el modo depositar diferentes conjuntos de datos, al final se grafica
		  // de la misma forma. La forma en la que llegan a ese buffer los datos es lo importante.

		  HAL_TIM_Base_Start(&htim2);
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcRawDataDma, BUFFER_RAW_LEN);

		  break;

	  case graficar_fft:

		  freqRangeDraw();

		  // 1. BORRAR señal anterior
		  for (int i = 1; i < GRAPH_W; i++) {
			  ILI9341_Draw_Line(
					  GRAPH_X + (i - 1), bufferAdcPrevio[i - 1],
					  GRAPH_X + i,       bufferAdcPrevio[i],
					  BLACK
			  );
		  }

		  // 2. DIBUJAR señal actual
		  for (int i = 1; i < GRAPH_W; i++) {
			  ILI9341_Draw_Line(
					  GRAPH_X + (i - 1), bufferFFTgraf[i - 1],
					  GRAPH_X + i,       bufferFFTgraf[i],
					  RED
			  );
		  }

		  //GUARDAR señal actual como previa
		  memcpy(bufferAdcPrevio, bufferFFTgraf,
				  BUFFER_GRAF_LEN * sizeof(uint16_t));

		  estadoActual = tomar_datos_adc;

		  HAL_TIM_Base_Start(&htim2);
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcRawDataDma, BUFFER_RAW_LEN);

		  break;

	  case error:
		  break;

	  default:
		  break;
	  }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim1.Init.Prescaler = 10 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 500;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  htim2.Init.Prescaler = 1 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 85;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void time_scale(uint32_t freq_muestreo)
{
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_Base_Stop(&htim2);

	if(freq_muestreo < 100)
	{
		freq_muestreo = 100;
	}

	periodo_timer = (int)(100000000 / freq_muestreo) - 1;

	__HAL_TIM_SET_AUTORELOAD(&htim2, periodo_timer);

	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcRawDataDma, BUFFER_RAW_LEN);
}

void escalaTiempoGraf(void)
{
	escala_graf = (((100000000.0 / (float)freq_muestreo) - 1.0) * ((1.0 / 100000000.0) * 1000.0)) * 300.0;  //Conversion ajustada, escala del grafico, se asume el timer va a 100 MHz.

	sprintf((char *)bufferText, "X-AXIS %.3f MS", escala_graf);
	ILI9341_Draw_Filled_Rectangle_Coord(180, 40, 310, 49, BLACK);
	ILI9341_Draw_Text(bufferText, 150, 40, WHITE, 1, BLACK);
}

/*
 * Filtro Exponencial (Low Pass Filter)
 * current: El valor nuevo del ADC
 * prev: El valor filtrado anterior (memoria)
 * alpha: Factor de suavizado (0.0 a 1.0).
 * 0.1 = Muy suave (lento), 0.9 = Poco filtrado (rápido/ruidoso)
 */
float EMA_Filter(float current, float prev, float alpha)
{
    return (alpha * current) + ((1.0f - alpha) * prev);
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void freqRangeDraw(void)
{
	float freq_aux = 0;

	if(freq_muestreo > 999)
	{
		freq_aux = (float)freq_muestreo / 1000.0;
		sprintf((char *)bufferText, "FREQ-M = %.1f KHZ", freq_aux);

	}else if(freq_muestreo > 999999)
	{
		freq_aux = (float)freq_muestreo / 1000000.0;
		sprintf((char *)bufferText, "FREQ-M = %.3f MHZ", freq_aux);

	}else if(freq_muestreo < 1000)
	{
		freq_aux = (float)freq_muestreo;
		snprintf((char *)bufferText, sizeof(bufferText), "FREQ-M = %.1f Hz", freq_aux);
	}

	ILI9341_Draw_Filled_Rectangle_Coord(70, 20, 130, 30, BLACK);
	ILI9341_Draw_Text(bufferText, 10, 20, YELLOW, 1, BLACK);
}


void max_fft_freq(void)
{
	float valorMaxFound = 0;

	escala_graf = (((100000000.0 / (float)freq_muestreo) - 1.0) * ((1.0 / 100000000.0) * 1000.0)) * 512.0;

	frecuencia_max_graf = 512.0 / (2.0 * (escala_graf / 1000.0));  //Importante el 2, limite de Nyquist, max freq

	snprintf((char *)bufferText, sizeof(bufferText), "RANGE-FREQ = %.1f Hz", frecuencia_max_graf);

	arm_max_f32(bufferFFTMag, BUFFER_RAW_LEN/2 , &maxFFTValue, &maxFFTIndex);

	valorMaxFound = (maxFFTIndex * (float)freq_muestreo) / 512;

	ILI9341_Draw_Filled_Rectangle_Coord(10, 40, 142, 49, BLACK);
	ILI9341_Draw_Text(bufferText, 10, 40, RED, 1, BLACK);

	snprintf((char *)bufferText, sizeof(bufferText), "FREQ-SGN = %.1f HZ", valorMaxFound);

	ILI9341_Draw_Filled_Rectangle_Coord(150, 20, 240, 30, BLACK);
	ILI9341_Draw_Text(bufferText, 150, 20, RED, 1, BLACK);
}


void controlFreqRange(void)
{
	if(banderaChar)
	{
		banderaChar = 0;

		switch(g_dato_recibido)  //Bloque que identifica el caracter recibido de la UART.
		{
		case '+':

			if(freq_muestreo < 20000000)
			{
				freq_muestreo = freq_muestreo + (100 * multiplicador_freq);
			}

			snprintf((char *)bufferText, 64, "\n\r FrecuenciaM = %lu Hz \n\r", freq_muestreo);
			HAL_UART_Transmit(&huart2, (uint8_t *)bufferText, strlen(bufferText), 100);

			freqRangeDraw();
			time_scale(freq_muestreo);

			escalaTiempoGraf();

			break;
		case '-':

			if(freq_muestreo > 110)
			{
				freq_muestreo = freq_muestreo - (100 * multiplicador_freq);
			}

			snprintf((char *)bufferText, 64, "\n\r FrecuenciaM = %lu Hz \n\r", freq_muestreo);
			HAL_UART_Transmit(&huart2, (uint8_t *)bufferText, strlen(bufferText), 100);

			freqRangeDraw();
			time_scale(freq_muestreo);

			escalaTiempoGraf();

			break;

		case '*':

			modoActual = (modoActual + 1) % 2;

			if(modoActual)
			{
				HAL_UART_Transmit(&huart2, (uint8_t *)"Modo FFT\n\r", strlen("Modo FFT\n\r"), 100);
			}else
			{
				HAL_UART_Transmit(&huart2, (uint8_t *)"Modo Señal\n\r", strlen("Modo Señal\n\r"), 100);
			}

			if(modoActual > 1)
			{
				modoActual = 0;
			}

			break;

		default:
			break;
		}

		g_dato_recibido = '\0';  //Limpiamos el dato de tal forma que en el siguiente ciclo no ejecute ninguna interrupcion o se vuelva "rancio"
	}
}

void generacionInstruccion(void)
{
	controlFreqRange();

	if(banderaInstrCmplt)  //Recibimos la bandera de que se completó el mensaje al oprimir enter
	{
		banderaInstrCmplt = 0;

		instruccion = strtok((char *)plantillaInstrRec, "=");  //Separamos el mensaje a partir del simbolo '='
		valor_instr = strtok(NULL, "=");

		/*------------------------------Comandos para fijar un valor--------------------------------*/

		if(instruccion != NULL && valor_instr != NULL)  //Verificamos que ninguna de las partes sea nula
		{
			if(strcmp(instruccion, "FrecuenciaM") == 0)  //strcmp devuelve cero si las cadenas son identicas
			{
				freq_muestreo = (uint16_t)atoi(valor_instr);  //ASCII to Integer Puntos que van a determinar cantidad de muestras igualmente espaciadas

				freqRangeDraw();
				time_scale(freq_muestreo);
				escalaTiempoGraf();

				HAL_UART_Transmit(&huart2, (uint8_t *)"Listo!\n\r", strlen("Listo!\n\r"), 100);

			}else if(strcmp(instruccion, "AlphaEMI") == 0)  //Usado para fijar el voltaje que proviene del PWM y que cae sobre la resistencia que está en serie con el colector
			{
				alpha = (float)atof(valor_instr);

				HAL_UART_Transmit(&huart2, (uint8_t *)"Listo!\n\r", strlen("Listo!\n\r"), 100);

			}else if(strcmp(instruccion, "FactorM") == 0)
			{
				multiplicador_freq = (uint16_t)atoi(valor_instr);

				HAL_UART_Transmit(&huart2, (uint8_t *)"Listo!\n\r", strlen("Listo!\n\r"), 100);

			}else  //Sino corresponde a ninguna de las opciones el mensaje escrito se muestra error
			{
				HAL_UART_Transmit(&huart2, (uint8_t *)"Error\n\r", strlen("Error\n\r"), 100);
			}  //Fin interpretacion de comandos de indicacion

			HAL_UART_Receive_IT(&huart2, (uint8_t *)&byteCaracterRec, 1);
		}
	}
}

void zoomPhotoEncoder(void)
{
	if(banderaEncoder)
	{
		banderaEncoder = 0;

		time_scale(freq_muestreo);

		switch(modoActual)
		{
		case modo_graficacion_SR:

			escalaTiempoGraf();  //Si hubo algun cambio, graficamos el nuevo intervalo de tiempo.

			break;
		case modo_graficacion_FFT:

			break;

		default:
			break;
		}
	}
}

void actualizar_PWM_Segun_Grafico(void)
{
    duty_cycle = (int)((float)adcRawDataDmaCOPY[160] * (1000.0 / 4095.0));

    caso_led_pwm = 0;

    if((int)escala_graf > 100)
    {
        caso_led_pwm = 1; // Lento

    }else if(((int)escala_graf <= 100) && ((int)escala_graf > 33))
    {
        caso_led_pwm = 2; // Medio

    }else
    {
        caso_led_pwm = 3; // Rápido
    }

    switch(caso_led_pwm)
    {
    case 1: // PB15 Activo (TIM1_CH3N)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);          // Apaga PB13
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);          // Apaga PB14
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_cycle); // Enciende PB15 con brillo variable
        break;

    case 2: // PB14 Activo (TIM1_CH2N)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);          // Apaga PB13
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_cycle); // Enciende PB14 con brillo variable
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);          // Apaga PB15
        break;

    case 3: // PB13 Activo (TIM1_CH1N)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle); // Enciende PB13 con brillo variable
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);          // Apaga PB14
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);          // Apaga PB15
        break;
    }
}

/*-------------------------INTERRUPCIONES----------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  //Ejecuta la instruccion de la interrupcion recibida del EXTI
{
	//Debemos tener en cuenta tambien el valor del PINC8 para conocer según el desfase del encoder
	 // la dirección de giro de este.

	if(GPIO_Pin == GPIO_PIN_5)
	{
//		banderaSWEncoder = 1;

		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5))
		{
			HAL_TIM_Base_Stop(&htim2);

		}else
		{
			HAL_TIM_Base_Start(&htim2);
		}
	}

	if(GPIO_Pin == GPIO_PIN_8)  //La entrada del pin que escogimos como entrada del CLK
	{
		banderaEncoder = 1;  //Activamos la bandera del EXTI

		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) & HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))
		{
			if(freq_muestreo < 25000000)
			{
				freq_muestreo = freq_muestreo + (100 * multiplicador_freq);
			}

		}else
		{
			if(freq_muestreo > 100)
			{
				freq_muestreo = freq_muestreo - (100 * multiplicador_freq);
			}
		}

	}  //Finaliza la modificación del valor debido a la interrupcion del encoder
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc)
{
	if(hadc->Instance == ADC1)
	{
		HAL_ADC_Stop_DMA(&hadc1);
		HAL_TIM_Base_Stop(&htim2);

		conversionCompleta = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		if (byteCaracterRec == '+' || byteCaracterRec == '-' || byteCaracterRec == '*')
		{
			g_dato_recibido = byteCaracterRec;
			banderaChar = 1;
		}
		else if (byteCaracterRec == '\r')
		{
			plantillaInstrRec[posicionEnPlantInstrRec] = '\0';
			banderaInstrCmplt = 1;
			posicionEnPlantInstrRec = 0;
		}
		// CASO 3: Backspace (\b) -> Borrar ultimo caracter
		else if (byteCaracterRec == '\b' || byteCaracterRec == 64)
		{
			if (posicionEnPlantInstrRec > 0)
			{
				posicionEnPlantInstrRec--;
				// Opcional: Enviar espacio atrás para borrar visualmente en terminal
			}
		}
		else if (byteCaracterRec != '\n')
		{
			if (posicionEnPlantInstrRec < (64 - 1))
			{
				plantillaInstrRec[posicionEnPlantInstrRec] = byteCaracterRec;
				posicionEnPlantInstrRec++;
			}
		}

		//Siempre rearmar la interrupción
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&byteCaracterRec, 1);
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
