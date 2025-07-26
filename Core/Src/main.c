/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ff.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "trinityfunctions.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t IMU_tag_buff[15] = {}; //temp, acc, gyro
uint8_t Bar_pt_buff[6] = {};
float gyro[3] = {0};
float gyro_offset[3] = {};
float acc[3] = {};
float acc_offset[3] = {};

float body_quat[4] = {1,0,0,0};
float target_quat[4] = {1,0,0,0};
float target_gyro[3] = {};
float relative_quat[4]; //quaternion that represents the relative rotation between the body and target quaternions. Specifically, relative = quat_multiply(body,conj(target))
float axang[4] = {};

float thetas[3] = {};
float gammas[3] = {};
float Forces[3] = {12,12,12};
float ReqTorque[3] = {};

float alt = 0;
#define ADC_CHANNEL_COUNT 4
float voltages[ADC_CHANNEL_COUNT];        // Converted voltages
long counter = 0;

uint8_t TxBufA_with_cmd[4+256] = {0x02}; //tx flash buffers: 1*(cmd) + 3*(mem addr) + 256*(data)
uint8_t TxBufB_with_cmd[4+256] = {0x02};
uint8_t* TxBufA = TxBufA_with_cmd + 4;
uint8_t* TxBufB = TxBufB_with_cmd + 4;

uint8_t FastRxBufA_with_cmd[5+256] = {}; //FAST READ ONLY rx flash buffers: 1*(cmd) + 3*(mem addr) + 1*(dummy) + any_amount*(data) , in this case 256 data
uint8_t FastRxBufB_with_cmd[5+256] = {};
uint8_t* FastRxBufA = FastRxBufA_with_cmd + 5;
uint8_t* FastRxBufB = FastRxBufB_with_cmd + 5;

uint8_t RxBufA_with_cmd[4+256] = {}; //standard read rx flash buffers: 1*(cmd) + 3*(mem addr) + any_amount*(data) , in this case 256 data
uint8_t RxBufB_with_cmd[4+256] = {};
uint8_t* RxBufA = RxBufA_with_cmd + 4;
uint8_t* RxBufB = RxBufB_with_cmd + 4;

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(2000);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_Base_Start(&htim2); //timer to count microseconds

  uint8_t txBuf[3] = {(0x09 | 0x80),0x00,0x00}; // 0F for bar, 75 for imu WHO_AM_I
  uint8_t txBufbar[3] = {(0x2b | 0x80 | 0x40),0x00,0x00}; // 0F for bar, 75 for imu WHO_AM_I
  uint8_t rx_data[3] = {0};

  uint8_t who_am_i = 0;

  uint16_t adc_dma_buf[ADC_CHANNEL_COUNT];  // Raw ADC data
  HAL_ADC_Start_DMA(&hadc1, adc_dma_buf, ADC_CHANNEL_COUNT);

  int16_t temp_raw = 0;
  float temperature;
  float temperature_bar;

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);


  uint8_t txBufWHO[2] = { 0x0F | 0x80, 0x00 };  // 0x0F | 0x80 (read WHO_AM_I)
  uint8_t rxBufWHO[2] = { 0 };
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi3, txBufWHO, rxBufWHO, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

  uint8_t txBufWHOflash[4] = { 0x9F };  // 0x0F | 0x80
  uint8_t rxBufWHOflash[4] = { 0 };
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, txBufWHOflash, rxBufWHOflash, 4, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  //rxBufWHOflash should contain 1Fh 89h 01h. The first byte received is a dummy, and it contains 0xff because the MISO line is pulled high through a resistor (because the SD on the same line needed it)

  initIMU(&hspi3, IMU_tag_buff); //passes also the buffer address for later use
  initBar(&hspi3, Bar_pt_buff);

  IMU_Calibration(&hspi3, gyro_offset, acc_offset, 5000); //SPI_HandleTypeDef *hspi, float *gyro_offset, float *acc0, int n_cycles

  //float barvar = get_barometer_variance(&hspi3, 10000); //got ~0.02

  sector_erase(&hspi1, 0);//blocking
  //uint8_t flash_status = is_flash_busy(&hspi1);
  //TxBufA[16] = 11;
  for (int i=0;i<100;i++) {
	  TxBufA[i] = i;
  }
  if (is_SPI1_done()) {
	  flash_program(TxBufA_with_cmd, &hspi1);//transmits 256 bytes of data;
  }
  HAL_Delay(1000);
  //flash_status = is_flash_busy(&hspi1);

  if (!is_flash_busy(&hspi1)) {
      read_flash(RxBufA_with_cmd ,20, &hspi1);
  }
  HAL_Delay(1000);

  //flash_status = is_flash_busy(&hspi1);
  if (!is_flash_busy(&hspi1)) {
	  fast_read_flash(FastRxBufA_with_cmd ,20, &hspi1);
  }

  //blog post made by the maker of the library: https://01001000.xyz/2020-08-09-Tutorial-STM32CubeIDE-SD-card/
  //library used for SD CARD: https://github.com/kiwih/cubeide-sd-card
  //Video used for SD CARD: https://youtu.be/spVIZO-jbxE?si=KLvULVrA2ofx23bV

  /*
  FATFS fs;       // File system object
  FIL file;       // File object
  FRESULT res;    // FatFS result type
  UINT bw;        // Bytes written

  HAL_Delay(1000);
  // Mount the filesystem
  res = f_mount(&fs, "", 1);
  if (res != FR_OK) {
      // Handle error (e.g., no card present)
      Error_Handler();
  }

  // Open or create file
  res = f_open(&file, "data.txt", FA_CREATE_ALWAYS | FA_WRITE);
  if (res != FR_OK) {
      // Handle file open error
      Error_Handler();
  }

  // Write text to file
  char text[] = "Hello from STM32!\r\n";
  res = f_write(&file, text, strlen(text), &bw);
  if (res != FR_OK || bw == 0) {
      // Handle write error
      Error_Handler();
  }

  // Close the file
  f_close(&file);

  // Optional: unmount the filesystem
  f_mount(NULL, "", 1);
  */

  float press_bar = 0;
  int32_t press_raw = 0;
  float corrected_press = 0;

  uint32_t counter = 0;
  uint32_t tick = 0;
  uint32_t tock = 0;
  uint32_t tic=0;
  uint32_t toc=0;
  uint32_t elaps = 0;
  uint32_t micro_elaps = 0;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	counter += 1;
	if (counter>=10000) {HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);}

	tic = HAL_GetTick();
	tick = my_micros(&htim2);

	typedef enum {
	    //SPI3_IDLE = 0,
	    SPI3_IMU_READING,
	    SPI3_BAR_READING
	} SPI3_LastReading;
	static SPI3_LastReading spi3_LastReading = SPI3_BAR_READING;

	typedef enum {
	    FILLING_BUF_A,
	    FILLING_BUF_B
	} BufferState;
	static BufferState bufferstate = FILLING_BUF_A;
	static uint8_t loggingdata = 0;

	if (is_ADC_done()) {
		ADC_to_voltages(adc_dma_buf,voltages);
	}
	HAL_ADC_Start_DMA(&hadc1,adc_dma_buf, 4);

	if (is_SPI3_done()) {
		read_IMU_Bar(&hspi3, IMU_tag_buff, Bar_pt_buff);
	}

	if (0) {
		IMU_readTempAccGyro(&hspi3, IMU_tag_buff);
	}

	if (0) {
		BAR_readPressureTemp(&hspi3, Bar_pt_buff);
		//HAL_Delay(100);
	}
/*
	switch (spi3_LastReading) { //alternates between IMU and Bar reading

		case SPI3_IMU_READING:
			if (is_SPI3_done()) {
				//set_SPI3_availability(0);
				BAR_readPressureTemp(&hspi3, Bar_pt_buff);
				spi3_LastReading = SPI3_BAR_READING;
			}
			break;

		case SPI3_BAR_READING:
			if (is_SPI3_done()) {
				//set_SPI3_availability(0);
				//uint8_t local_buff[2];
				//uint8_t mycmd[2] = {(0x39 | 0x80)};
				//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
				//HAL_SPI_TransmitReceive_DMA(&hspi3, mycmd, local_buff, 2);
				//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
				IMU_readTempAccGyro(&hspi3, IMU_tag_buff);
				spi3_LastReading = SPI3_IMU_READING;
			}
			break;
	}*/
	static uint8_t all_engines = 0;
	if (!all_engines) {
		static uint8_t engine_state[3] = {};
		for (int i=0;i<3;i++) {
			if (voltages[i]>=0.4) {engine_state[i] = 1;};
		}
		if ((engine_state[0]+engine_state[1]+engine_state[2])==3) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
			all_engines = 1;
		}
	}

	get_gyro(IMU_tag_buff, gyro_offset, gyro);//processes gyro data by offsetting, and by bringing it to the right reference frame (IMU to rocket frame)
	gyro2quat_integration(gyro, body_quat, micro_elaps); //gets the body quaternion
	gyro2quat_integration(target_gyro, target_quat, micro_elaps); //gets the target quaternion
	get_relative_quat(body_quat, target_quat, relative_quat); //finds the relative quat between body and target
	quat2axang(relative_quat, axang);

	updateReqTorque(axang, gyro, target_gyro, body_quat, target_quat, ReqTorque, micro_elaps);
	//updateReqTorque(axang, gyro, ReqTorque, micro_elaps);
	//get_parabVertex_angles(thetas, gammas, Forces, ReqTorque);
	get_angles_RMS_and_paraboloid(thetas, gammas, Forces, ReqTorque);
	writeServos(thetas, gammas, &htim1, &htim3);

	acc[0] = ((int16_t)(IMU_tag_buff[3]<<8 | IMU_tag_buff[4])) /2047.0 * 1;//x of IMU
	acc[1] = ((int16_t)(IMU_tag_buff[5]<<8 | IMU_tag_buff[6])) /2047.0 * 1;//y of IMU
	acc[2] = ((int16_t)(IMU_tag_buff[7]<<8 | IMU_tag_buff[8])) /2047.0 * 1;//z of IMU

	if (0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // CS LOW, C1 for bar, C0 for imu
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		//uint8_t isok = (HAL_SPI_TransmitReceive(&hspi3, txBufbar, rx_data, 3, HAL_MAX_DELAY)==HAL_OK); // Read data
		HAL_SPI_TransmitReceive_DMA(&hspi3, txBufbar, rx_data, 3);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); // CS HIGH
		temp_raw = (int16_t)((rx_data[2] << 8) | rx_data[1]);
		temperature_bar = temp_raw / 480.0 + 42.5;
	}

	if (0) {
		uint8_t TXpress[4] = {(0x80 | 0x28 | 0x40)};
		uint8_t RXpress[4];
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // CS LOW, C1 for bar, C0 for imu
		uint8_t isok = (HAL_SPI_TransmitReceive(&hspi3, TXpress, RXpress, 4, HAL_MAX_DELAY)==HAL_OK); // Read data
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); // CS HIGH
		press_raw = (int32_t)((RXpress[3] << 16) | RXpress[2] << 8 | RXpress[1]);
		press_bar = press_raw / 4096.0;
	}

	if (0) { //THIS SOMEHOW WORKS, BUT THE ONE IN THE LIBRARY DOESN'T
		//TURNS OUT THE READING OF 6 REGISTERS IS WRONG??? READING JUST THE TEMPERATURE WORKS
		//HAL_Delay(100);
		set_SPI3_availability(0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);//CS LOW
		temp_raw = (int16_t)((Bar_pt_buff[2] << 8) | Bar_pt_buff[1]);
		temperature_bar = temp_raw / 480.0 + 42.5;

		//static uint8_t txxBuffer[6] = {(0x80 | 0x40 | 28)}; //Read + Increment address + address
		if(HAL_SPI_TransmitReceive_DMA(&hspi3, txBufbar, Bar_pt_buff, 3) == HAL_OK ) { //5 bytes of data, one of address
			//return 1;
		} else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
			//return 0;
		}

	}

	if (0) { //THIS WORKS, TRY TO FIGURE OUT WHY ALSO READING THE PRESSURE IN A SINGLE BUFFER DOESN'T (sometimes it did work...)
		BAR_readTemp(&hspi3, Bar_pt_buff);
		temp_raw = (int16_t)((Bar_pt_buff[2] << 8) | Bar_pt_buff[1]);
		temperature_bar = temp_raw / 480.0 + 42.5;
	}
	//Barometer computing
	press_raw = (int32_t)((Bar_pt_buff[3] << 16) | Bar_pt_buff[2] << 8 | Bar_pt_buff[1]);
	press_bar = press_raw / 4096.0;
	temp_raw = (int16_t)((Bar_pt_buff[5] << 8) | Bar_pt_buff[4]);
	temperature_bar = temp_raw / 480.0 + 42.5;
	//IMU temp
	temp_raw = (int16_t)((IMU_tag_buff[1] << 8) | IMU_tag_buff[2]);//IMU temp bytes
	temperature = temp_raw / 128.0 + 25.0; //IMU temperature conversion
	corrected_press = (temperature + 273.15) / (temperature_bar + 273.15) * press_bar; //Barometer temperature readings are trash. Use pv=nrt to correct, using IMU's temperature

	static const float T0 = 306; //T0 is the temperature at 0 meters level, in Kelvin
	static const float RR = 287.05; //Gas constant for dry air [J / (kg * K) ]
	static const float p0 = 1018; //Today's pressure at 0m level [hPa]
	static const float g0 = 9.80665; //No explanation needed, come on
	alt = -T0*RR / (g0 * p0) * (press_bar - p0); //press to altitude formula. approximated even more through Taylor expansion, but it's fine for my altitude range

	if (loggingdata == 1) {
		static uint fill_idx = 0; //to know at what index of the buffer we are at
		static const uint buffer_size = sizeof(TxBufA) / sizeof(TxBufA[0]);//assume BufA and BufB are of the same size. the division is added for "the concept", but the denominator has value 1
		static const uint packet_size = 16;//placeholder for now
		static const uint max_start_idx = buffer_size - packet_size; //max_idx at which we can start writing

		if (fill_idx > max_start_idx) {
			bufferstate ^= 1; //flips buffer state with XOR gate
			fill_idx = 0;
		}

		uint8_t *Buffer = (bufferstate == FILLING_BUF_A) ? TxBufA : TxBufB; //sets what buffer we are about to fill

		//ALL THE SIZES COMBINED NEED TO ADD UP TO packet_size

		//time and pacing data
		memcpy(Buffer + fill_idx, counter, sizeof(counter));
		fill_idx += sizeof(counter);

		memcpy(Buffer + fill_idx, tic, sizeof(tic));
		fill_idx += sizeof(tic);

		memcpy(Buffer + fill_idx, micro_elaps, sizeof(micro_elaps));
		fill_idx += sizeof(micro_elaps);

	}

	toc = HAL_GetTick();
	tock = my_micros(&htim2);

	elaps = toc-tic;
	micro_elaps = tock-tick;
	counter ++;

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB14 PB15
                           PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
