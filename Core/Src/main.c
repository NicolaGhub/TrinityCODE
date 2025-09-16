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
float gyro[3] = {0};//body frame
float gyro_offset[3] = {};
float acc_raw[3] = {};
float acc[3] = {0,0,9.80665};//body frame. initialized like this since it goes through a complementary filter
float acc_earth[3] = {};//earth frame
float vel_earth[3] = {};//earth frame
float pos_earth[3] = {};//earth frame
float acc_offset[4] = {};//offset x,y,z and norm

float body_quat[4] = {1,0,0,0};
float target_quat[4] = {1,0,0,0};
float target_gyro[3] = {};
float relative_quat[4]; //quaternion that represents the relative rotation between the body and target quaternions. Specifically, relative = quat_multiply(body,conj(target))
float axang[4] = {};

float thetas[3] = {};
float gammas[3] = {};
float Forces[3] = {};//{12,12,12};
float ReqTorque[3] = {};
float diff = 0;

float alt = 0;//bar alt
float filtered_alt = 0;
#define ADC_CHANNEL_COUNT 4
float voltages[ADC_CHANNEL_COUNT] = {};        // Converted voltages
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

typedef enum {
	NOT_LOGGING,
	LOGGING_FLASH,
	LOGGING_SD
} LoggingState;
LoggingState loggingstate = NOT_LOGGING; //d

typedef enum {
	IDLE,
	VECTORING,
	ASCENT,
	DESCENT
} RocketState;
RocketState rocketstate = IDLE;

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
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);//chute

  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_Base_Start(&htim2); //timer to count microseconds

  uint16_t adc_dma_buf[ADC_CHANNEL_COUNT] = {};  // Raw ADC data
  HAL_ADC_Start_DMA(&hadc1, adc_dma_buf, ADC_CHANNEL_COUNT);

  int16_t temp_raw = 0;
  float temperature;
  float temperature_bar;

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);  //chip select Barometer
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);  //chip select IMU
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); //chip select NOR Flash
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); //chip select SD

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //output to pad pb12
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); //output to pad pb13

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); //YELLOW LED ON TO SIGNAL HOLDING

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1900);//close chute

  /*
██╗  ██╗ ██████╗ ██╗     ██████╗ ██╗███╗   ██╗ ██████╗
██║  ██║██╔═══██╗██║     ██╔══██╗██║████╗  ██║██╔════╝
███████║██║   ██║██║     ██║  ██║██║██╔██╗ ██║██║  ███╗
██╔══██║██║   ██║██║     ██║  ██║██║██║╚██╗██║██║   ██║
██║  ██║╚██████╔╝███████╗██████╔╝██║██║ ╚████║╚██████╔╝
╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚═════╝ ╚═╝╚═╝  ╚═══╝ ╚═════╝
*/
  //blocking the execution unless the state isn't "NONE" anymore
  UmbilicalState umb_input = NONE;
  umb_input = TEST; //bypass for debugging
  while (umb_input == NONE) {
	  UmbilicalState temp_umb_input = read_umbilical();
	  HAL_Delay(500);
	  if (temp_umb_input == read_umbilical()) {
		  umb_input = temp_umb_input; //double reading to be sure the command is correctly read
	  }
  }

  //if (umb_input == TEST) {umb_input = LAUNCH;} //bypass since one gpio got unsoldered (faulty)

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); //YELLOW LED OFF

  initIMU(&hspi3, IMU_tag_buff); //passes also the buffer address for later use
  initBar(&hspi3, Bar_pt_buff);

  //float barvar = get_barometer_variance(&hspi3, 10000); //got ~0.02

  if (umb_input == LAUNCH) {
	  block_erase(&hspi1, 0);//blocking, erases 64kb of data starting from address 0x00
	  loggingstate = LOGGING_FLASH;
	  //takes about 12 secs
	  for (int nblock = 0; nblock < 50; nblock++) { //flash erase cycles
		  block_erase(&hspi1, nblock*0x10000);//delay incorporated in the function (250ms)
		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2); //BLUE LED TOGGLE
	  }
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); //BLUE LED OFF
  }

  if (umb_input == LOG) {
		loggingstate=LOGGING_SD;
		set_flash_add(0xFFFFFF);//reads up all the flash
  }

  //two calibrations. The first is just to get the MEMS to temperature or operating contitions
  IMU_Calibration(&hspi3, gyro_offset, acc_offset, 10000); //SPI_HandleTypeDef *hspi, float *gyro_offset, float *acc0, int n_cycles
  IMU_Calibration(&hspi3, gyro_offset, acc_offset, 10000);

  //NOR FLASH TRY CODE
  /*
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
  }*/



  //SD INITIALIZATION
  //blog post made by the maker of the library: https://01001000.xyz/2020-08-09-Tutorial-STM32CubeIDE-SD-card/
  //library used for SD CARD: https://github.com/kiwih/cubeide-sd-card
  //Video used for SD CARD: https://youtu.be/spVIZO-jbxE?si=KLvULVrA2ofx23bV
  FATFS fs;       // File system object
  FIL file;       // File object
  FIL file2;
  FRESULT res;    // FatFS result type
  UINT bw;        // Bytes written
  UINT br;        // Bytes read
  HAL_Delay(1000);
  // Mount the filesystem
  res = f_mount(&fs, "", 1);
  if (res != FR_OK) {
      // Handle error (e.g., no card present)
      Error_Handler();
  }
  // Open or create file
  if (umb_input == LAUNCH) {
	  res = f_open(&file, "data.txt", FA_CREATE_ALWAYS | FA_WRITE);
	  if (res != FR_OK) {
		  // Handle file open error
		  Error_Handler();
	  }
  } else if (umb_input == LOG) {
	  res = f_open(&file, "log.txt", FA_CREATE_ALWAYS | FA_WRITE);
	  if (res != FR_OK) {
		  // Handle file open error
		  Error_Handler();
	  }
  } else if (umb_input == TEST) {
	  res = f_open(&file, "data.txt", FA_READ);
	  if (res != FR_OK) {
		  // Handle file open error
		  Error_Handler();
	  }
	  res = f_open(&file2, "re_iter.txt", FA_CREATE_ALWAYS | FA_WRITE); //file that contains the re-elaborated data
	  if (res != FR_OK) {
		  // Handle file open error
		  Error_Handler();
	  }
  }

/*
  // Write text to file
  char text[] = "Hello from STM32!\r\n";
  uint8_t datatry[6] = {0,1,0,2,3,4};
  //res = f_write(&file, text, strlen(text), &bw);
  res = f_write(&file, datatry, sizeof(datatry), &bw);
  if (res != FR_OK || bw == 0) {
      // Handle write error
      Error_Handler();
  }

  //equivalent of flushing. Secures the data written but keeps the file open
  f_sync(&file);*/


  // Close the file
  //f_close(&file);

  // Optional: unmount the filesystem
  //f_mount(NULL, "", 1);


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
	uint16_t write_faults = 0;

	uint32_t t0 = HAL_GetTick();

	if (umb_input == LAUNCH) { //creates the setup line in the flash
		//good to go
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //output to pad pb12
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); //output to pad pb13 GOOD TO GO FOR LAUNCH
		//indicator led
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); //RED LED ON

		//first page of flash programmed with setup data:
		uint8_t TxBufTEMP_with_cmd[4+256] = {0x02}; //tx flash buffer: 1*(cmd) + 3*(mem addr) + 256*(data)
		uint8_t* TxBufTEMP = TxBufTEMP_with_cmd + 4;
		uint32_t fill_idxTEMP = 0;

		//t0, gyro_offset, acc_offset
		memcpy(TxBufTEMP + fill_idxTEMP, &t0, sizeof(t0)); //to , from , how many
		fill_idxTEMP += sizeof(t0);

		memcpy(TxBufTEMP + fill_idxTEMP, gyro_offset, sizeof(gyro_offset));
		fill_idxTEMP += sizeof(gyro_offset);

		memcpy(TxBufTEMP + fill_idxTEMP, acc_offset, sizeof(acc_offset));
		fill_idxTEMP += sizeof(acc_offset);

		flash_program(TxBufTEMP_with_cmd, &hspi1);
	}

	if (umb_input == TEST) { //decodes the setup line from the SD to the corresponding variables
		uint8_t transfer_buffer[64]; //buffer that stores the read bytes
		res = f_read(&file, transfer_buffer, 64, &br);
		uint8_t idx = 0;
		memcpy(&t0, transfer_buffer, sizeof(t0));//to , from , how many
		idx += sizeof(t0);
		memcpy(gyro_offset, transfer_buffer + idx, sizeof(gyro_offset));
		idx += sizeof(gyro_offset);
		memcpy(acc_offset, transfer_buffer + idx, sizeof(acc_offset));
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	tic = HAL_GetTick(); //millis
	tick = my_micros(&htim2); //micros

	if ((tic-t0>1500) && (umb_input == LAUNCH)) {
		//disable launch command after 1.5 sec, to ensure that ignition doesn't happen after too much time.
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //output to pad pb12
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); //output to pad pb13
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); //RED LED OFF
	}

	typedef enum {
	    FILLING_BUF_A,
	    FILLING_BUF_B
	} BufferState;
	static BufferState bufferstate = FILLING_BUF_A;

	//READINGS
	if (umb_input == LAUNCH) { //reads only if the command was read as LAUNCH (avoids overwriting of the buffers during reiteration of the data if umb_input==TEST)
		if (is_ADC_done()) {
			ADC_to_voltages(adc_dma_buf,voltages);
			HAL_ADC_Start_DMA(&hadc1,adc_dma_buf, 4);
		}
		if (is_SPI3_done()) {
			read_IMU_Bar(&hspi3, IMU_tag_buff, Bar_pt_buff);
		}
	}

	if ((umb_input == TEST)&&(counter<=19000)) { //copies some of the needed values stored in the file, to their corresponding variable

		//res = f_open(&file, "data.txt", FA_READ);

		static uint32_t offset = 256; //skips the first page since it has only setup data

		res = f_lseek(&file, offset); //positions the read pointer at the offset

		#define RECORD_SIZE 100 //should be the same as the "packet_size" used to record the data in question
		uint8_t transfer_buffer[RECORD_SIZE]; //buffer that stores the read bytes
		res = f_read(&file, transfer_buffer, RECORD_SIZE, &br);

		uint32_t copy_idx = 0;
		//memcpy(&counter, transfer_buffer + copy_idx, sizeof(counter));//to , from , how many
		copy_idx += sizeof(counter);
		memcpy(&tic, transfer_buffer + copy_idx, sizeof(tic));
		copy_idx += sizeof(tic);
		memcpy(&micro_elaps, transfer_buffer + copy_idx, sizeof(micro_elaps));
		copy_idx += sizeof(micro_elaps);
		memcpy(IMU_tag_buff+1, transfer_buffer + copy_idx, sizeof(IMU_tag_buff)-1); //the first byte is empty
		copy_idx += sizeof(IMU_tag_buff)-1;
		memcpy(target_gyro, transfer_buffer + copy_idx, sizeof(target_gyro));
		copy_idx += sizeof(target_gyro);
		//memcpy(thetas, transfer_buffer + copy_idx, sizeof(thetas));//12
		copy_idx += sizeof(thetas);
		//memcpy(gammas, transfer_buffer + copy_idx, sizeof(gammas));//12
		copy_idx += sizeof(gammas);
		//memcpy(ReqTorque, transfer_buffer + copy_idx, sizeof(ReqTorque));//12
		copy_idx += sizeof(ReqTorque);
		memcpy(voltages, transfer_buffer + copy_idx, sizeof(voltages));//16
		copy_idx += sizeof(voltages);
		//memcpy(&press_bar, transfer_buffer + copy_idx, sizeof(press_bar));//4
		copy_idx += sizeof(press_bar);
		//memcpy(&filtered_alt, transfer_buffer + copy_idx, sizeof(filtered_alt));//4
		copy_idx += sizeof(filtered_alt);
		//memcpy(&write_faults, transfer_buffer + copy_idx, sizeof(write_faults));//2
		copy_idx += sizeof(write_faults);

		if (counter%2 == 0) {
			offset += 100;
		} else {
			offset += 156;
		}

		if (counter%100 == 0) {

		}
	}

	//all engine activated at least once:
	/*static uint8_t all_engines = 0;
	if (!all_engines) {
		static uint8_t engine_state[3] = {};
		for (int i=0;i<3;i++) {
			if (voltages[i]>=0.4) {engine_state[i] = 1;};
		}
		if ((engine_state[0]+engine_state[1]+engine_state[2])==3) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); //YELLOW LED ON
			all_engines = 1;
		}
	}*/

	//all engines activated LED:
	uint8_t engine_state[3] = {};
	for (int i=0;i<3;i++) {
		if (voltages[i]>=0.4) {
			engine_state[i] = 1;
		} else {engine_state[i] = 0;}
	}
	if ((engine_state[0] + engine_state[1] + engine_state[2]) == 3) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); //YELLOW LED ON
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); //YELLOW LED OFF
	}

	//Voltage -> Force
	for (int i=0;i<3;i++) {
		if (voltages[i]>=0.4) {
			Forces[i] = 9;//9 Newtons if D9 engine. 12 Newtons if E12 engine. E12 were used in the static tests
		} else {
			Forces[i] = 0;
		}
	}

	get_gyro(IMU_tag_buff, gyro_offset, gyro);//processes gyro data by offsetting, and by bringing it to the right reference frame (IMU to rocket frame)
	get_acc(IMU_tag_buff, acc_offset, acc, acc_raw);

	gyro2quat_integration(gyro, body_quat, micro_elaps); //gets the body quaternion
	gyro2quat_integration(target_gyro, target_quat, micro_elaps); //gets the target quaternion
	get_relative_quat(body_quat, target_quat, relative_quat); //finds the relative quat between body and target
	quat2axang(relative_quat, axang);

	get_earth_acc(acc, body_quat, acc_earth);//gets the acceleration vector in the body frame, with acceleration of gravity removed
	get_vel_pos(pos_earth, vel_earth, acc_earth, micro_elaps);

	updateReqTorque(axang, gyro, target_gyro, body_quat, target_quat, ReqTorque, micro_elaps);
	//get_parabVertex_angles(thetas, gammas, Forces, ReqTorque);
	get_angles_RMS_and_paraboloid(thetas, gammas, Forces, ReqTorque, &diff);
	writeServos(thetas, gammas, &htim1, &htim3); //refers just to the 6 thrust vectoring servos, not to the parachute deploying servo


	//Barometer computing
	press_bar = get_press(Bar_pt_buff);
	temperature_bar = get_bar_temp(Bar_pt_buff);
	//IMU temp
	temp_raw = (int16_t)((IMU_tag_buff[1] << 8) | IMU_tag_buff[2]);//IMU temp bytes
	temperature = temp_raw / 128.0 + 25.0; //IMU temperature conversion
	corrected_press = (temperature + 273.15) / (temperature_bar + 273.15) * press_bar; //Barometer temperature readings are trash. Use pv=nrt to correct, using IMU's temperature

	alt = get_bar_alt(press_bar);
	filtered_alt = filter_altitude(alt, acc_earth[2], micro_elaps);//KALMAN. Accelerometer is used as the control input and for the estimate state. barometer is used as the measurement


	if ((umb_input == TEST)&&(counter<=19000)) {
		f_write(&file2, &diff, sizeof(diff), &bw);
	}
	if ((umb_input == TEST)&&(counter>19000)) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //BLUE LED ON
		f_sync(&file2);
	}


	//if (counter>10&&counter<20) {loggingstate=LOGGING_SD;}
	//if (counter>20) {loggingstate=NOT_LOGGING;}

	if (umb_input == LAUNCH) { //logging logic
		if (tic-t0>10000) {loggingstate=LOGGING_SD;}
	}

	if (loggingstate == LOGGING_FLASH) {//logs to flash
		static unsigned int fill_idx = 0; //to know at what index of the buffer we are at
		static const unsigned int buffer_size = 256; //sizeof(TxBufA) / sizeof(TxBufA[0]);//DIVISION DOESNT WORK since txbufa is a pointer//assume BufA and BufB are of the same size. the division is added for "the concept", but the denominator has value 1
		static const unsigned int packet_size = 100;//placeholder for now. consider calculating once, after the first data packet is created
		static const unsigned int max_start_idx = buffer_size - packet_size; //max_idx at which we can start writing

		if (fill_idx > max_start_idx) {//if the buffer is full or if it would overflow when writing

			//if the execution enters here, then the last used buffer is good to go to be written.
			//Selects the last used buffer, before we change the buffer state
			uint8_t *WriteBuf = (bufferstate == FILLING_BUF_A) ? TxBufA_with_cmd : TxBufB_with_cmd;
			if(is_SPI1_done()) {
				if (!is_flash_busy(&hspi1)) {
					static uint32_t time_of_program = 0;
					while( my_micros(&htim2)-time_of_program < 650) {}
					flash_program(WriteBuf, &hspi1);
					time_of_program = my_micros(&htim2);
					//flash_program is currently non blocking, but takes 0.6ms. MAKE SURE THIS TIME HAS ELAPSED BEFORE ATTEMPTING A WRITE OPERATION
					//The check can be done with a variable that checks how much time has passed, but this wouln't resolve the data overflow, and some of it would be loss.
					//To really resolve overflow, reduce the number of bytes that get written to the flash at each loop.
					//HAL_Delay(2);
				} else write_faults += 1; //signals that an attempt to page_program was done but the previous page_program wasn't completed
			} else write_faults += 1; //signals that one buffer was skipped

			bufferstate ^= 1; //flips buffer state with XOR gate (FILLING_BUF_A <----> FILLING_BUF_B)
			fill_idx = 0;
		}

		uint8_t *Buffer = (bufferstate == FILLING_BUF_A) ? TxBufA : TxBufB; //sets what buffer we are about to fill

		//ALL THE SIZES COMBINED NEED TO ADD UP TO packet_size

		//time and pacing data
		memcpy(Buffer + fill_idx, &counter, sizeof(counter)); //to , from , how many
		fill_idx += sizeof(counter);

		memcpy(Buffer + fill_idx, &tic, sizeof(tic));
		fill_idx += sizeof(tic);

		memcpy(Buffer + fill_idx, &micro_elaps, sizeof(micro_elaps));
		fill_idx += sizeof(micro_elaps);//4 bytes

		//memcpy(Buffer + fill_idx, gyro_offset, sizeof(gyro_offset));//12 bytes
		//fill_idx += sizeof(gyro_offset);
		//memcpy(Buffer + fill_idx, acc_offset, sizeof(acc_offset));
		//fill_idx += sizeof(acc_offset);

		//memcpy(Buffer + fill_idx, gyro, sizeof(gyro));
		//fill_idx += sizeof(gyro);
		//memcpy(Buffer + fill_idx, acc_raw, sizeof(acc_raw));
		//fill_idx += sizeof(acc_raw);
		//memcpy(Buffer + fill_idx, acc, sizeof(acc));
		//fill_idx += sizeof(acc);

		memcpy(Buffer + fill_idx, IMU_tag_buff+1, sizeof(IMU_tag_buff)-1); //the first byte is empty
		fill_idx += sizeof(IMU_tag_buff)-1;

		//memcpy(Buffer + fill_idx, acc_earth, sizeof(acc_earth));
		//fill_idx += sizeof(acc_earth);
		//memcpy(Buffer + fill_idx, vel_earth, sizeof(vel_earth));
		//fill_idx += sizeof(vel_earth);
		//memcpy(Buffer + fill_idx, pos_earth, sizeof(pos_earth));
		//fill_idx += sizeof(pos_earth);//9*12

		//memcpy(Buffer + fill_idx, body_quat, sizeof(body_quat));//16 bytes
		//fill_idx += sizeof(body_quat);
		//memcpy(Buffer + fill_idx, target_quat, sizeof(target_quat));//16
		//fill_idx += sizeof(target_quat);
		memcpy(Buffer + fill_idx, target_gyro, sizeof(target_gyro));//12
		fill_idx += sizeof(target_gyro);
		//memcpy(Buffer + fill_idx, relative_quat, sizeof(relative_quat));//16
		//fill_idx += sizeof(relative_quat);
		//memcpy(Buffer + fill_idx, axang, sizeof(axang));//16
		//fill_idx += sizeof(axang);

		memcpy(Buffer + fill_idx, thetas, sizeof(thetas));//12
		fill_idx += sizeof(thetas);
		memcpy(Buffer + fill_idx, gammas, sizeof(gammas));//12
		fill_idx += sizeof(gammas);
		//memcpy(Buffer + fill_idx, Forces, sizeof(Forces));//12
		//fill_idx += sizeof(Forces);
		memcpy(Buffer + fill_idx, ReqTorque, sizeof(ReqTorque));//12
		fill_idx += sizeof(ReqTorque);

		memcpy(Buffer + fill_idx, voltages, sizeof(voltages));//16
		fill_idx += sizeof(voltages);
		//memcpy(Buffer + fill_idx, &alt, sizeof(alt));//4
		//fill_idx += sizeof(alt);
		memcpy(Buffer + fill_idx, &press_bar, sizeof(press_bar));//4
		fill_idx += sizeof(press_bar);
		memcpy(Buffer + fill_idx, &filtered_alt, sizeof(filtered_alt));//4
		fill_idx += sizeof(filtered_alt);

		memcpy(Buffer + fill_idx, &write_faults, sizeof(write_faults));//2
		fill_idx += sizeof(write_faults);

	}

	if (loggingstate == LOGGING_SD) {

		static uint16_t SD_faults = 0; //consider making this global so it can be logged

		uint32_t last_addr = get_flash_add() + 255;//returns the last flash address at which we wrote a byte.
		//get_flash_add gives the start of the page. +255 brings to the last byte, since every page_program writes 256 bytes

		static uint32_t read_addr = 0; //address at which we want to read
		uint32_t nbytes = 256;//Reading the flash is not limited to reading 256 bytes, so consider increasing this number. You would have to edit the function fast_read, and SDbuf_with_cmd
		uint8_t SDbuf_with_cmd[256+5] = {};//fast read is +5, read is +4 //REMEMBER TO PLACE nbytes here correctly in the size
		uint8_t *SDbuf = SDbuf_with_cmd + 5;

		if (read_addr <= last_addr) { //if we haven't read everything that was written yet

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); //BLUE LED OFF

			if (is_SPI1_done()) { //READ FROM FLASH
				if (!is_flash_busy(&hspi1)) { //put here for safety, but the RDY bit is not set to 0 when reading
					fast_read_flash(SDbuf_with_cmd, nbytes, read_addr, &hspi1);
					read_addr += nbytes;//increments read address by the number of bytes read
				} else SD_faults+=1;
			} else SD_faults+=1;

			if (is_SPI1_done()) { //WRITE TO SD
				if (!is_flash_busy(&hspi1)) {
					f_write(&file, SDbuf, nbytes, &bw); //fwrite doesnt set the spi1_done variable to 0, but it should be blocking so there shouldnt be any issue

					if (counter%10 == 0) {//syncs every 10 loops
						f_sync(&file);
					}

				} else SD_faults+=1;
			} else SD_faults+=1;
		} else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //BLUE LED ON
			f_sync(&file);
		}

	}

	toc = HAL_GetTick();
	tock = my_micros(&htim2);

	elaps = toc-tic; //stops with breakpoints
	micro_elaps = tock-tick; //doesn't stop with breakpoints, keeps counting
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
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

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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
