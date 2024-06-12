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
#include "string.h"
#include <time.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "asm_func.h"
#include "data.h"
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

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
void zeros(uint32_t *vec, uint32_t longitud);
void mul_esc_32(uint32_t *vectorIn, uint32_t *vectorOut, uint32_t longitud, uint16_t escalar);
void mul_esc_16(uint16_t *vectorIn, uint16_t *vectorOut, uint32_t longitud, uint16_t escalar);
void mul_esc_16_sat(uint16_t *vectorIn, uint16_t *vectorOut, uint32_t longitud, uint16_t escalar);
uint32_t bitfield_toggle(uint32_t *dato, uint32_t ancho, uint32_t inicio);

/* AdM - TP2 */
void pack32to16(int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);
uint32_t max(int32_t *vectorIn, uint32_t longitud);
void downSample(int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);
void invertir(uint16_t * vector, uint32_t longitud);

/* AdM - TP3*/
uint32_t c_potencia (int16_t * vecIn, uint32_t longitud);
void c_medDif(int8_t * e, int8_t *x, int8_t *y, uint16_t longitud);
void c_eco (int16_t * signal, int16_t *eco, uint32_t longitud);

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
	DWT->CTRL |= 1 << DWT_CTRL_CYCCNTENA_Pos;
	uint32_t ciclos_C, ciclos_ASM, ciclos_DSP;

	/* ----------- Funcion asm_sum Start ----------- */
	uint32_t a, b, c;
	a = 10;
	b = 20;
	c = asm_sum(a, b);
	/* ----------- Funcion asm_sum End ----------- */

	/* ----------- Funcion asm_svc Start ----------- */

	uint32_t x;
	x = __get_CONTROL();		// leo el registro  de control usando CMSIS Macro

	x |= 1;						// seteo b0 = 1.

	__set_CONTROL(x);			// escribo registro de control usando CMSIS Macro
								// Modo no privlegiado

	x &= ~1;					// pongo un 0 en b0

	__set_CONTROL(x);			// escribo registro de control usando CMSIS Macro
								// como estoy en modo no privlegiado se ignora la escritura

	asm_svc();					//invoco a la excepcion supervosr call para cambiar a modo
								//privilegiado

	/* ----------- Funcion asm_svc End ----------- */

	/* ----------- Funcion asm_stack Start ----------- */

	uint32_t d, e, f;
	d = 10;
	e = 1234;
	f = 5678;
	asm_stack(a, b, c, d, e, f);

	/* ----------- Función asm_stack End ----------- */



	/* ----------- Funcion asm_stack Start ----------- */

	uint32_t lon = 10;
	uint32_t vector[lon];
	uint32_t vector2[lon];
	zeros(vector, lon);
	asm_zeros(vector2, lon);

	/* ----------- Función asm_stack End ----------- */

	/* ----------- Función bitfield_clear ----------- */

	uint32_t dato, resultado, inicio, ancho;
	dato = 1023;
	inicio = 3;
	ancho = 5;
	resultado = asm_bitfield_clear(dato, ancho, inicio);

	/* ----------- Función bitfield_clear ----------- */




	/* ----------- Funcion asm_pack32to16 Start ----------- */

    /*
	uint32_t pack_lon = 10;
	int32_t asm_pack_vectorIn[pack_lon];
	int16_t asm_pack_vectorOut[pack_lon];
	int32_t pack_vectorIn[pack_lon];
	int16_t pack_vectorOut[pack_lon];

	asm_pack32to16(asm_pack_vectorIn, asm_pack_vectorOut, pack_lon);
	pack32to16(pack_vectorIn, pack_vectorOut, pack_lon);
	*/
	int32_t vectorIn[] = { 123456, -123456, 200000, -200000 };
	int16_t vectorOut[4];
	uint32_t longitud = 4;

	asm_pack32to16(vectorIn, vectorOut, longitud);

	/* ----------- Función asm_pack32to16 End ----------- */




	/* ----------- Funcion asm_max Start ----------- */

	uint32_t max_result;
	uint32_t asm_max_result;
	uint32_t max_lon = 5;
	int32_t asm_max_vectorIn[]= {100000, -522, 35812, -7999, -2588888};
	int32_t max_vectorIn[] = {100000, -522, 35812, -7999, -2588888};

	max_result = max(max_vectorIn, max_lon);
	asm_max_result = asm_max(asm_max_vectorIn, max_lon);

	/* ----------- Función asm_max End ----------- */




	/* ----------- Funcion asm_downSample Start ----------- */

	uint32_t down_long = 10;
	int32_t down_vectorIn[down_long];
	int32_t down_vectorOut[down_long];
	int32_t asm_down_vectorIn[down_long];
	int32_t asm_down_vectorOut[down_long];
	uint32_t down_N = 3;
	asm_downSample(asm_down_vectorIn, asm_down_vectorOut, down_long, down_N);
	downSample(down_vectorIn, down_vectorOut, down_long, down_N);

	/* ----------- Función asm_downSample End ----------- */




	/* ----------- Funcion asm_invertir Start ----------- */

	uint32_t invrt_long = 9;
	uint16_t invrt_vectorIn[] = {0,1,2,3,4,5,6,7,8};
	uint16_t asm_invrt_vectorIn[] = {0,1,2,3,4,5,6,7,8};
	invertir(invrt_vectorIn, invrt_long);
	asm_invertir(asm_invrt_vectorIn, invrt_long);

	/* ----------- Función asm_invertir End ----------- */


	/* ----------- Funcion c_potencia Start ----------- */

	uint32_t power, asm_power, asm_DSP_power;
	power = asm_power = asm_DSP_power = 0;
	power = c_potencia(c_potencia_vecIn, c_potencia_POTENCIA_LEN);
	asm_power = asm_potencia(c_potencia_vecIn, c_potencia_POTENCIA_LEN);
	asm_DSP_power = asm_potencia_DSP(c_potencia_vecIn, c_potencia_POTENCIA_LEN);


	/* ----------- Función c_potencia End ----------- */


	/* ----------- Funcion c_medDif Start ----------- */

	c_medDif(c_medDif_e, c_medDif_x, c_medDif_y, c_medDif_longitud);
	asm_medDif(asm_medDif_e, c_medDif_x, c_medDif_y, c_medDif_longitud);
	asm_medDif_DSP(asm_DSP_medDif_e, c_medDif_x, c_medDif_y, c_medDif_longitud);

	/* ----------- Función c_medDif End ----------- */


	/* ----------- Funcion c_eco Start ----------- */

	/* Initialize the random number generator with a seed based on the current time */
	srand(time(NULL));

	// Llena el array con números aleatorios en el rango de int16_t
	for (int i = 0; i < c_eco_longitud; i++) {

		c_eco_signalIn[i] = (int16_t)(rand() % (INT16_MAX - INT16_MIN + 1) + INT16_MIN);

	}

	DWT->CYCCNT = 0;
	c_eco(c_eco_signalIn, c_eco_signalOut, c_eco_longitud);
	ciclos_C = DWT->CYCCNT; // 301.643

	DWT->CYCCNT = 0;
	asm_eco(c_eco_signalIn, asm_eco_signalOut, c_eco_longitud);
	ciclos_ASM = DWT->CYCCNT; // 72.578

	DWT->CYCCNT = 0;
	asm_eco_DSP(c_eco_signalIn, asm_DSP_eco_signalOut, c_eco_longitud);
	ciclos_DSP = DWT->CYCCNT;	// 49.335

	/* ----------- Función c_eco End ----------- */

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * @Func:	zeros(uint32_t *vec, uint32_t longitud)
 * @Brief:	fill vector to zero
 *
 * @Parameters:
 * 		uint32_t *vec: vector
 * 		uint32_t longitud
 *
 * @Return:
 * 		None
 */
void zeros(uint32_t *vec, uint32_t longitud) {
	// Notar que se usa > y no ≥ por el tipo uint
	for (uint8_t i = longitud; i > 0; i--) {
		vec[i - 1] = 0;
	}

	return;

}

/*
 * @Func:	mul_esc_32(uint32_t *vectorIn, uint32_t *vectorOut, uint32_t longitud, uint16_t escalar)
 * @Brief:	multiply a 32 bits vector output by scalar factor
 *
 * @Parameters:
 * 		uint32_t *vectorIn: input vector
 * 		uint32_t *vectorOut: output vector
 * 		uint32_t longitud: lenght both vectors
 * 		uint16_t escalar: scalar factor multiplier
 *
 * @Return:
 * 		None
 */
void mul_esc_32(uint32_t *vectorIn, uint32_t *vectorOut, uint32_t longitud, uint16_t escalar){


	return;

}

/*
 * @Func:	mul_esc_16(uint16_t *vectorIn, uint16_t *vectorOut, uint32_t longitud, uint16_t escalar)
 * @Brief:	multiply a 16 bits vector output by scalar factor
 *
 * @Parameters:
 * 		uint16_t *vectorIn: input vector
 * 		uint16_t *vectorOut: output vector
 * 		uint32_t longitud: lenght both vectors
 * 		uint16_t escalar: scalar factor multiplier
 *
 * @Return:
 * 		None
 */
void mul_esc_16(uint16_t *vectorIn, uint16_t *vectorOut, uint32_t longitud, uint16_t escalar){


	return;

}

/*
 * @Func:	mul_esc_16(uint16_t *vectorIn, uint16_t *vectorOut, uint32_t longitud, uint16_t escalar)
 * @Brief:	multiply a 16 bits vector output by scalar factor and implements saturation maths
 *
 * @Parameters:
 * 		uint16_t *vectorIn: input vector
 * 		uint16_t *vectorOut: saturated output vector
 * 		uint32_t longitud: lenght both vectors
 * 		uint16_t escalar: scalar factor multiplier
 *
 * @Return:
 * 		None
 */
void mul_esc_16_sat(uint16_t *vectorIn, uint16_t *vectorOut, uint32_t longitud, uint16_t escalar){


	return;

}


/*
 * @Func:	bitfield_toggle(uint32_t *dato, uint32_t ancho, uint32_t inicio)
 * @Brief:	togle a bitfield in a data
 *
 * @Parameters:
 * 		uint32_t *dato: pointer to date to be procesing
 * 		uint32_t ancho: bitfield weight
 * 		uint32_t inicio: bitfield start
 *
 * @Return:
 * 		None
 */
uint32_t bitfield_toggle(uint32_t *dato, uint32_t ancho, uint32_t inicio){

	uint32_t result = 0;

	return (result);

}

/*
 * @Func:	pack32to16(int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud)
 *
 * @Brief:	Realizar una función que reciba un vector de
 * 			números signados de 32 bits y los
 * 			“empaquete” en otro vector de 16 bits, guardando
 * 			la parte alta (MSB).
 * 			Sugerencia: utilice números mayores a 100.000 y
 * 			menores a -100.000 en el vector de pruebas para
 * 			poder ver los resultados.
 *
 * @Parameters:
 * 		int32_t *vectorIn:
 * 		int16_t *vectorOut:
 * 		uint32_t longitud:
 *
 * @Return:
 * 		None
 */
void pack32to16(int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud){


	return;

}

/*
 * @Func:	max(int32_t *vectorIn, uint32_t longitud)
 *
 * @Brief:	Realizar una función que reciba un vector de
 * 			números signados de 32 bits y devuelva la
 * 			posición del máximo del vector.
 *
 * @Parameters:
 * 		int32_t *vectorIn:
 * 		uint32_t longitud:
 *
 * @Return:
 * 		uint32_t
 *
 * @Created on: May 25, 2024
 * @Author: Daniel David Albarracin
 * @SIU: e2207
 *
 */
uint32_t max(int32_t *vectorIn, uint32_t longitud) {

	uint32_t index = 0;

	uint32_t posMax = 0;
	int32_t MaxValue = 0;

	MaxValue = vectorIn[longitud - 1];

	for (index = (longitud - 1); index > 0; index--) {
		if (vectorIn[index - 1] > MaxValue) {
			posMax = (index - 1);
			MaxValue = vectorIn[index - 1];
		}
	}

	return (posMax);

}

/*
 * @Func:	downSample(int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N)
 *
 * @Brief:	Realizar una función que reciba un vector de muestras
 * 			signadas de 32 bits y descarte una de cada N muestras.
 * 			Si se impone en la aplicación que longitud siempre
 * 			es múltiplo de N, ¿cómo determinaría la longitud de
 * 			vectorOut?
 *
 * @Parameters:
 * 		int32_t *vectorIn:
 * 		int16_t vectorOut:
 * 		uint32_t longitud:
 * 		uint32_t N:
 *
 * @Return:
 * 		None
 */
void downSample(int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N){

	uint32_t outIndex = 0;
	for (uint32_t i = 0; i < longitud; i++) {
		if ((i + 1) %N != 0) {
			vectorOut[outIndex++] = vectorIn[i];
	    }
	}

	return;

}

/*
 * @Func:	invertir(uint16_t * vector, uint32_t longitud)
 *
 * @Brief:	Realizar una función que reciba un vector de
 * 			muestras no signadas de 16 bits e invierta su orden.
 *
 * @Parameters:
 * 		uint16_t *vector:
 * 		uint32_t longitud:
 *
 * @Return:
 * 		None
 *
 * @Created on: May 25, 2024
 * @Author: Daniel David Albarracin
 * @SIU: e2207
 *
 */
void invertir(uint16_t *vector, uint32_t longitud) {

	uint16_t firstValue, lastValue;
	uint16_t index;

	for (index = 0; index < (longitud / 2); index++) {
		firstValue = vector[index];
		lastValue = vector[(longitud - 1) - index];
		vector[index] = lastValue;
		vector[(longitud - 1) - index] = firstValue;
	}

	return;

}


/*
 * @Func:	c_potencia (int16_t * vecIn, uint32_t longitud)
 *
 * @Brief:	Implementar una función que calcule la potencia
 * 			de una señal discreta x[n] con datos signados de
 * 			16 bits
 *
 * @Parameters:
 * 		int16_t *vector:
 * 		uint32_t longitud:
 *
 * @Return:
 * 		uint32_t power
 *
 * @Created on: June 06, 2024
 * @Author: All
 * @SIU: e2201 - e2203 - e2207
 *
 */
uint32_t c_potencia (int16_t * vecIn, uint32_t longitud) {

	uint32_t power = 0;

	if ((vecIn != NULL) & (longitud != 0)){

		for (int32_t i = 0; i < longitud; i++) {

			power += (vecIn [i] * vecIn [i]);

		}

		power /= longitud;

	}

	return power;

}

/*
 * @Func:	c_medDif(int8_t * e, int8_t *x, int8_t *y, uint16_t longitud)
 *
 * @Brief:	Implementar una función que calcule el vector de diferencia
 * 			media entre dos señales discretas x[n] e y[n] con datos
 * 			signados de 8 bits y de igual cantidad de elementos
 *
 * @Parameters:
 * 		int8_t *e: mean difference vector
 * 		int8_t *x: x signal
 * 		int8_t *y: y signal
 * 		uint16_t longitud: vector length
 *
 * @Return:
 * 		None
 *
 * @Created on: Jun 06, 2024
 * @Author: All
 * @SIU: e2201 - e2203 - e2207
 *
 */
void c_medDif(int8_t * e, int8_t *x, int8_t *y, uint16_t longitud) {

	if ((e != NULL) & (x != NULL) & (y != NULL) & (longitud != 0)){

		for (int32_t i = 0; i < longitud; i++) {

			e[i] = (x[i] - y [i]) / 2;

		}
	}

}


/*
 * @Func:	c_eco(int16_t * signal, int16_t *eco, uint32_t longitud)
 *
 * @Brief:	Realizar una función que recibe un vector de 4096
 * 			valores de 16 bits signados, que corresponden a muestras
 * 			de audio tomadas a una tasa de muestreo de 44.100
 * 			muestras/s. La función debe introducir un “eco” que
 * 			consiste en adicionar a la señal original, la propia
 * 			señal original dividida por dos y atrasada en 20 mseg
 *
 * @Parameters:
 * 		int16_t *signal: signal vector
 * 		int16_t *eco: eco signal
 * 		uint32_t longitud: vector length
 *
 * @Return:
 * 		None
 *
 * @Created on: Jun 06, 2024
 * @Author: All
 * @SIU: e2201 - e2203 - e2207
 *
 */
void c_eco(int16_t * signal, int16_t *eco, uint32_t longitud) {

	// offset[index] = ((offset[ms] * fs[Hz]) / 1000) - 1
	uint32_t offset = ((20 * 44100) / 1000) - 1;	// Initial Eco Sample = 881

	int16_t sample = 0;

	if ((signal != NULL) & (longitud != 0)){

		memset(eco, '\0', (longitud * sizeof(int16_t)));

		for (uint32_t i = 0; i < longitud; i++) {

			sample = signal [i];

			if (i >= offset) {

				sample += (signal[i - offset] / 2);

			}

			eco[i] = sample;

		}

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
