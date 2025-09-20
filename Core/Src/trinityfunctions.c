/*
 * trinityfunctions.c
 *
 *  Created on: Jun 6, 2025
 *      Author: Admin
 */
#ifndef IS_TRINITY_LIB_DEFINED
#define IS_TRINITY_LIB_DEFINED

#include "trinityfunctions.h"
#include <stdint.h>
#include "main.h"
#include <math.h>

#define PI 3.1415926535897932384626433832795

static uint8_t *bar_buff_addr;
static uint8_t *imu_buff_addr;
static SPI_HandleTypeDef *hspi_local;
static volatile uint8_t spi3_done = 1;
static volatile uint8_t spi1_done = 1;
static volatile uint8_t read_also_bar = 0;
static volatile uint8_t adc_done  = 1;

static const float 	K = 0.05,//angle minimazation coefficient
					KP = 20.0f,//PID gains
					KI = 40.0f,
					KD = 10.0f,
					R_gain_x = 1.0f,//gains of req torque vector
					R_gain_y = 1.0f,
					R_gain_z = 4.0f,
					Ix = 0.06695f,//0.06959f,//inertia
					Iy = 0.06694f,//0.06709f,//inertia
					Iz = 0.0008214f,//0.0007898f,//inertia
					lx = 0.021234f,//components of vector from CM to Motor Force applied by motor number 1. The others will then be calculated by rotating it by 120 & 240deg
					ly = -0.014f,
					lz = -0.2975f;//-0.2475f;

void initIMU(SPI_HandleTypeDef *hspi, uint8_t *tagBuff) { //+-2000 deg/s, +-16g
	spi3_done = 0;
	imu_buff_addr = tagBuff; //sets buffer address for later use
	hspi_local = hspi;
	//power on gyro and accelerometer
	uint8_t cmd0[2] = {
		(0x1F & 0x7F),   // register 0x1F, and "& 0x7F" is used to set write operation (MSB=0), probably not needed
		0b00001111       // 0b00001111: ACCEL_MODE=11, GYRO_MODE=11
	  	};
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd0, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
/*
	uint8_t cmd3[2] = {
		(0x39 & 0x7F), //This bit automatically sets to 1 when a Data Ready interrupt is generated. The bit clears to 0 after the register has been read.
		0b00000001
		};
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd3, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	uint8_t cmd4[2] = {
		(0x04 & 0x7F), //Data Ready Interrupt Clear Option (latched mode)
		0b00110000
		};
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd4, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
*/

/*
	uint8_t cmd1[2] = { // interrupt configuration
		(0x06 & 0x7F),
		0b00000011
		};
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd1, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	uint8_t cmd2[2] = { //Interrupt source set to data ready
		(0x2b & 0x7F),   //"& 0x7F" is used to set write operation (MSB=0), probably not needed
		0b00001000       // data ready
		  };
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd2, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
*/
	spi3_done = 1;
}

void initBar(SPI_HandleTypeDef *hspi, uint8_t *ptBuff) {
	spi3_done = 0;
	bar_buff_addr = ptBuff; //sets buffer address for later use
	hspi_local = hspi;
	uint8_t cmd[2] = { //CTRL_REG1
	    (0x20 & 0x7F),   // register 0x20h, write operation (MSB=0)
	    0b11000000 //interrupt is generation set to off
	};
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

	/*//control register 2
	uint8_t cmd2[2] = { //CTRL_REG1
	    (0x21 & 0x7F),   // register 0x21h, write operation (MSB=0)
	    0b00000000 //interrupt is generation set to off
	};
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd2, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(5);*/


/*
	uint8_t cmd2[2] = { //interrupt set to data ready
		(0x23 & 0x7F),   // register 0x23h, write operation (MSB=0)
		0b00000000
	};
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd2, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
*/
	spi3_done = 1;
}

uint8_t is_SPI3_done() {
	return spi3_done;
}

void set_SPI3_availability(uint8_t var) {
	spi3_done = var;
}

uint8_t is_SPI1_done() {
	return spi1_done;
}

void set_SPI1_availability(uint8_t var) {
	spi1_done = var;
}

uint8_t is_ADC_done() {
	return adc_done;
}

uint8_t IMU_readTempAccGyro(SPI_HandleTypeDef *hspi, uint8_t *tagBuff) {
	spi3_done = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);//CS LOW

	static uint8_t txBuffer[15] = {(0x80 | 0x09)}; //09 is the temp address
	if(HAL_SPI_TransmitReceive_DMA(hspi, txBuffer, tagBuff, 15) == HAL_OK ) { //14 bytes of data, one of address
		return 1;
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		spi3_done = 1;
		return 0;
	}
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);//CS HIGH
	//imu_dma_done = 1;
}

void BAR_readTemp(SPI_HandleTypeDef *hspi, uint8_t *tempBuff) {
	set_SPI3_availability(0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);//CS LOW
	uint8_t txBuffer[3] = {(0x2b | 0x80 | 0x40),0x00,0x00};

	//static uint8_t txxBuffer[6] = {(0x80 | 0x40 | 28)}; //Read + Increment address + address
	if(HAL_SPI_TransmitReceive_DMA(hspi, txBuffer, tempBuff, 3) == HAL_OK ) {
		//return 1;
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		//return 0;
	}
}

/*
void BAR_readPress(SPI_HandleTypeDef *hspi, uint8_t *pressBuff) {
	set_SPI3_availability(0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);//CS LOW
	uint8_t txBuffer[3] = {(0x2b | 0x80 | 0x40),0x00,0x00};

	//static uint8_t txxBuffer[6] = {(0x80 | 0x40 | 28)}; //Read + Increment address + address
	if(HAL_SPI_TransmitReceive_DMA(hspi, txBuffer, tempBuff, 3) == HAL_OK ) {
		//return 1;
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		//return 0;
	}
}*/

uint8_t BAR_readPressureTemp(SPI_HandleTypeDef *hspi, uint8_t *ptBuff) {
	spi3_done = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);//CS LOW

	static uint8_t txBuffer[6] = {(0x80 | 0x40 | 0x28)}; //Read + Increment address + address
	if(HAL_SPI_TransmitReceive_DMA(hspi, txBuffer, ptBuff, 6) == HAL_OK ) { //5 bytes of data, one of address
		return 1;
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		spi3_done = 1;
		//uint32_t err = hspi->ErrorCode;
		return 0;
	}
}

void read_IMU_Bar(SPI_HandleTypeDef *hspi, uint8_t *tagBuff, uint8_t *ptBuff) {
	read_also_bar = 1;
	bar_buff_addr = ptBuff;
	hspi_local = hspi;
	IMU_readTempAccGyro(hspi,tagBuff);
	//BAR_readPressureTemp(hspi,ptBuff);
}

void ADC_to_voltages(uint16_t *ADCbuff, float *voltages) {
	static const float adc_scaler = 3.3f / 4095.0f;
	voltages[0] = ADCbuff[0] * adc_scaler;
	voltages[1] = ADCbuff[1] * adc_scaler;
	voltages[2] = ADCbuff[2] * adc_scaler;
	voltages[3] = ADCbuff[3] * adc_scaler;
	adc_done = 0; //tells me if a new data has been available since the last voltage conversion
}

UmbilicalState read_umbilical() {
	UmbilicalState state = 0;
	//state = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) ? (state | 0b0001) : state;
	//state = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) ? (state | 0b0010) : state;
	state = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)) ? (state | 0b01) : state; //actually used as input
	state = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)) ? (state | 0b10) : state; //actually used as input
	return state;
}

void write_umbilical(uint8_t pin12, uint8_t pin13) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, pin12 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, pin13 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*
    ███╗   ██╗ ██████╗ ██████╗     ███████╗██╗      █████╗ ███████╗██╗  ██╗
	████╗  ██║██╔═══██╗██╔══██╗    ██╔════╝██║     ██╔══██╗██╔════╝██║  ██║
	██╔██╗ ██║██║   ██║██████╔╝    █████╗  ██║     ███████║███████╗███████║
	██║╚██╗██║██║   ██║██╔══██╗    ██╔══╝  ██║     ██╔══██║╚════██║██╔══██║
	██║ ╚████║╚██████╔╝██║  ██║    ██║     ███████╗██║  ██║███████║██║  ██║
	╚═╝  ╚═══╝ ╚═════╝ ╚═╝  ╚═╝    ╚═╝     ╚══════╝╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝

ANSI shadow font*/
void flash_WriteEnable(SPI_HandleTypeDef *hspi) {
	static uint8_t write_enable = 0x06;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &write_enable, 1, HAL_MAX_DELAY); // REMEMBER THAT HAL_SPI_Transmit(...) does NOT call HAL_SPI_TxRxCpltCallback, but only HAL_SPI_TxCpltCallback
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
}

uint32_t current_address = 0x000000; //available addresses are 000000h to FFFFFFh --> 2^24 bytes ~= 16Mbyte
void flash_program(uint8_t* Buf, SPI_HandleTypeDef *hspi) {
	//this function automatically changes page every time it is called, no matter how many bytes are written (max 256)
	//The buffer that needs to be passed needs to contain the page program command at its first byte
	//the next threee bytes are reserved to the address at which we want to write.

	//Note that the NOR flash can only program pages that have previously been erased, since it can only flip bits
	//from 1 to 0 and not the other way. The block/sector erase function sets the selected memory bits to 1.
	spi1_done = 0;

	static uint32_t n_cycles = 0;

	current_address = 0x00+ 256*n_cycles; //sets the address to be increased of 256 bytes after the last page program sequence
	if (current_address > (0xFFFFFF - 255)) {//overflow management
		current_address = 0xFFFFFF;
		return;
	}

	Buf[1] = current_address >> 16; //since Buf contains uint8_t, Buf[1] should truncate the shifted address
	Buf[2] = current_address >> 8;
	Buf[3] = current_address;
	flash_WriteEnable(hspi); //blocking
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(hspi, Buf, 260);

	n_cycles += 1;
}

uint32_t get_flash_add() {
	return current_address; //returns the last flash address at which we wrote a page
}

void set_flash_add(uint32_t address) {
	//sets the last address at which the readings will stop at
	current_address = address;
}

void fast_read_flash(uint8_t *RxBuf ,uint32_t data_byte_quantity, uint32_t address, SPI_HandleTypeDef *hspi) {
	spi1_done = 0;
	uint8_t addr_pieces[3] = {};
	addr_pieces[0] = address >> 16;
	addr_pieces[1] = address >>  8;
	addr_pieces[2] = address      ;

	uint8_t cmd[256 + 5] = {0x0B,addr_pieces[0],addr_pieces[1],addr_pieces[2]}; // [0Bh] [A23 - A16] [A15 - A8] [A7 - A0] [dummy] [(D7 - D0)] [continuous...
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	//HAL_SPI_TransmitReceive_DMA(hspi, cmd, RxBuf, data_byte_quantity + 5);
	HAL_SPI_TransmitReceive(hspi, cmd, RxBuf, data_byte_quantity + 5, HAL_MAX_DELAY);//blocking
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	spi1_done = 1;
}

void read_flash(uint8_t *RxBuf ,int data_byte_quantity, SPI_HandleTypeDef *hspi) {
	spi1_done = 0;
//consider making this blocking (just like fast read), otherwise data might not have fiinished being read when writing to the SD right away
	uint8_t cmd[256 + 4] = {0x03,0x00,0x00,0x00}; // [0Bh] [A23 - A16] [A15 - A8] [A7 - A0] [(D7 - D0)] [continuous...
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(hspi, cmd, RxBuf, data_byte_quantity + 4);
}

uint8_t is_flash_busy(SPI_HandleTypeDef *hspi) {
	spi1_done = 0;
	static uint8_t cmd[2] = { 0x05 }; //status register 1
	uint8_t rxbuf[2] = {};
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, cmd, rxbuf, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	spi1_done = 1;
	return (rxbuf[1] & 0b00000001);
	//When the RDY/BSY bit = 1, the device is busy in program/erase/write status register progress; when the RDY/BSY
	//bit = 0, the device is not in program/erase/write status register progress.

}

void sector_erase(SPI_HandleTypeDef *hspi, uint32_t address) { //erases 4kb, from xxx000h to xxxFFFh. It takes about 70ms to complete
	uint8_t cmd[4] = {0x20, 0x00, 0x00, 0x00}; // [20h] [A23 - A16] [A15 - A8] [A7 - A0]
	cmd[1] = address>>16;
	cmd[2] = address>>8;
	cmd[3] = address;
	flash_WriteEnable(hspi); //blocking
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_Delay(70); //datasheet said it takes about 70ms for the operation to complete
}

void block_erase(SPI_HandleTypeDef *hspi, uint32_t address) { //erases 64kb, from xx0000h to xxFFFFh. It takes about 250ms to complete
	uint8_t cmd[4] = {0xD8, 0x00, 0x00, 0x00}; // [20h] [A23 - A16] [A15 - A8] [A7 - A0]
	cmd[1] = address>>16;
	cmd[2] = address>>8;
	cmd[3] = address;
	while (is_flash_busy(hspi)) {
		HAL_Delay(10);
	}
	flash_WriteEnable(hspi); //blocking
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	//HAL_Delay(250); //datasheet said it takes about 250ms for the operation to complete
}

//--------------------------------------------------
//SD
//import sd lib?

//f_write(&file, text, strlen(text), &bw);




//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
/*
 ██████╗ █████╗ ██╗     ██╗     ██████╗  █████╗  ██████╗██╗  ██╗███████╗
██╔════╝██╔══██╗██║     ██║     ██╔══██╗██╔══██╗██╔════╝██║ ██╔╝██╔════╝
██║     ███████║██║     ██║     ██████╔╝███████║██║     █████╔╝ ███████╗
██║     ██╔══██║██║     ██║     ██╔══██╗██╔══██║██║     ██╔═██╗ ╚════██║
╚██████╗██║  ██║███████╗███████╗██████╔╝██║  ██║╚██████╗██║  ██╗███████║
 ╚═════╝╚═╝  ╚═╝╚══════╝╚══════╝╚═════╝ ╚═╝  ╚═╝ ╚═════╝╚═╝  ╚═╝╚══════╝
*/


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI3) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // CS HIGH solo quando DMA è finita
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

        //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        if (read_also_bar==1) {
        	read_also_bar=0;
        	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
        	BAR_readPressureTemp(hspi_local,bar_buff_addr);
        } else {
        	spi3_done = 1;
        }

    } else if(hspi->Instance == SPI1) {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // CS HIGH solo quando DMA è finita
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);

    	spi1_done = 1;
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		spi1_done = 1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_9) {
    	//IMU_readTempAccGyro(hspi_local, imu_buff_addr);
    	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);//, GPIO_PIN_SET);
    }

    if (GPIO_Pin == GPIO_PIN_14) {
        //BAR_readPressureTemp(hspi_local, bar_buff_addr);
        //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);//, GPIO_PIN_SET);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_done = 1;
    }
}

/*
██╗   ██╗████████╗██╗██╗     ██╗████████╗██╗   ██╗
██║   ██║╚══██╔══╝██║██║     ██║╚══██╔══╝╚██╗ ██╔╝
██║   ██║   ██║   ██║██║     ██║   ██║    ╚████╔╝
██║   ██║   ██║   ██║██║     ██║   ██║     ╚██╔╝
╚██████╔╝   ██║   ██║███████╗██║   ██║      ██║
 ╚═════╝    ╚═╝   ╚═╝╚══════╝╚═╝   ╚═╝      ╚═╝
 */

uint32_t my_micros(TIM_HandleTypeDef *htim)
{
	return __HAL_TIM_GET_COUNTER(htim);
}

//A union is like a struct, but the variables are stored starting at the same memory location
union float_and_uint32_t_union {
	uint32_t conv_bits;
	float conv_float;
};
union float_and_uint32_t_union data = {};

uint32_t float_to_bits(float var) {
    data.conv_float = var;
    return data.conv_bits;
    //alternative:
    //uint32_t binary;
    //memcpy(&var,&binary,sizeof(var)) //so var has to be passed to the function by its address
    //return binary
}

float bits_to_float(uint32_t var) {
    data.conv_bits = var;
    return data.conv_float;
}

float round2(float val) {
	return (roundf(val * 100) / 100);
	//return val;
}


//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
/*
███╗   ███╗ █████╗ ████████╗██╗  ██╗     █████╗ ███╗   ██╗██████╗      ██████╗ ██████╗ ███╗   ██╗████████╗██████╗  ██████╗ ██╗
████╗ ████║██╔══██╗╚══██╔══╝██║  ██║    ██╔══██╗████╗  ██║██╔══██╗    ██╔════╝██╔═══██╗████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██║
██╔████╔██║███████║   ██║   ███████║    ███████║██╔██╗ ██║██║  ██║    ██║     ██║   ██║██╔██╗ ██║   ██║   ██████╔╝██║   ██║██║
██║╚██╔╝██║██╔══██║   ██║   ██╔══██║    ██╔══██║██║╚██╗██║██║  ██║    ██║     ██║   ██║██║╚██╗██║   ██║   ██╔══██╗██║   ██║██║
██║ ╚═╝ ██║██║  ██║   ██║   ██║  ██║    ██║  ██║██║ ╚████║██████╔╝    ╚██████╗╚██████╔╝██║ ╚████║   ██║   ██║  ██║╚██████╔╝███████╗
╚═╝     ╚═╝╚═╝  ╚═╝   ╚═╝   ╚═╝  ╚═╝    ╚═╝  ╚═╝╚═╝  ╚═══╝╚═════╝      ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝
*/

/*
Quat makeQuat(float w, float x, float y, float z) {
	return (Quat){ .sto = {w, x, y, z} };
}

Quat Quat_conj(Quat q) {
	Quat result;
	result.sto[0] =  q.sto[0];
	result.sto[1] = -q.sto[1];
	result.sto[2] = -q.sto[2];
	result.sto[3] = -q.sto[3];
	return result;
}

Quat Quat_multiply(Quat q1, Quat q2) {
	Quat result;
	result.sto[0] = q2.sto[0]*q1.sto[0] - q2.sto[1]*q1.sto[1] - q2.sto[2]*q1.sto[2] - q2.sto[3]*q1.sto[3];
	result.sto[1] = q2.sto[0]*q1.sto[1] + q2.sto[1]*q1.sto[0] - q2.sto[2]*q1.sto[3] + q2.sto[3]*q1.sto[2];
	result.sto[2] = q2.sto[0]*q1.sto[2] + q2.sto[1]*q1.sto[3] + q2.sto[2]*q1.sto[0] - q2.sto[3]*q1.sto[1];
	result.sto[3] = q2.sto[0]*q1.sto[3] - q2.sto[1]*q1.sto[2] + q2.sto[2]*q1.sto[1] + q2.sto[3]*q1.sto[0];
	return result;
}
*/

void IMU_Calibration(SPI_HandleTypeDef *hspi, float *gyro_offset, float *acc0, int n_cycles) {
	uint8_t rxbuf[7] = {};
	float gyro_cal[3] = {};
	uint8_t cmd[7] = {(0x80 | 0x11)}; //0x11 is the first gyro address
	for (int i=0; i<n_cycles; i++) {
		//read gyro (blocking reading)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // CS LOW, C1 for bar, C0 for imu
		HAL_SPI_TransmitReceive(hspi, cmd, rxbuf, 7, HAL_MAX_DELAY); // Read data
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // CS HIGH
		gyro_cal[0] += ((int16_t)(rxbuf[1]<<8 | rxbuf[2])) / 16.4f;// /32767.0f *2000.0f;//x of IMU
		gyro_cal[1] += ((int16_t)(rxbuf[3]<<8 | rxbuf[4])) / 16.4f;// / 16.4f;// /32767.0f *2000.0f;//y of IMU
		gyro_cal[2] += ((int16_t)(rxbuf[5]<<8 | rxbuf[6])) / 16.4f;// /32767.0f *2000.0f;//z of IMU
	}
	gyro_offset[0] = gyro_cal[0]/n_cycles;
	gyro_offset[1] = gyro_cal[1]/n_cycles;
	gyro_offset[2] = gyro_cal[2]/n_cycles;

	float acc_cal[3] = {};
	uint8_t cmd2[7] = {(0x80 | 0x0b)}; //0x0b is the first acc address
	for (int i=0; i<n_cycles; i++) {
		//read gyro (blocking reading)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // CS LOW, C1 for bar, C0 for imu
		HAL_SPI_TransmitReceive(hspi, cmd2, rxbuf, 7, HAL_MAX_DELAY); // Read data
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // CS HIGH
		acc_cal[0] += ((int16_t)(rxbuf[1]<<8 | rxbuf[2])) /2048.0f * 1;//x of IMU
		acc_cal[1] += ((int16_t)(rxbuf[3]<<8 | rxbuf[4])) /2048.0f * 1;//y of IMU
		acc_cal[2] += ((int16_t)(rxbuf[5]<<8 | rxbuf[6])) /2048.0f * 1;//z of IMU
	}
	acc0[0] = acc_cal[0]/n_cycles + 1;//assumes the right vector is [0;0;1]
	acc0[1] = acc_cal[1]/n_cycles;
	acc0[2] = acc_cal[2]/n_cycles;
	acc0[3] = sqrt( pow(acc_cal[0],2) + pow(acc_cal[1],2) + pow(acc_cal[2],2))/n_cycles;
}

void get_gyro(uint8_t *IMU_tag_buff, float *gyro_offset, float *gyro) {
	float gyro_raw[3];
	gyro_raw[0] = ((int16_t)(IMU_tag_buff[9] <<8 | IMU_tag_buff[10])) / 16.4f;//32767.0f *2000.0f;//x of IMU
	gyro_raw[1] = ((int16_t)(IMU_tag_buff[11]<<8 | IMU_tag_buff[12])) / 16.4f;//32767.0f *2000.0f;//y of IMU
	gyro_raw[2] = ((int16_t)(IMU_tag_buff[13]<<8 | IMU_tag_buff[14])) / 16.4f;//32767.0f *2000.0f;//z of IMU
	//Apply offsets calculated through calibration.
	//OFFSETS REFER TO THE IMU REFERENCE FRAME
	gyro_raw[0] -= gyro_offset[0];
	gyro_raw[1] -= gyro_offset[1];
	gyro_raw[2] -= gyro_offset[2];
	//Rotate the IMU reference frame to the Rocket Reference frame
	gyro[0] =   gyro_raw[2];//+X
	gyro[1] =   gyro_raw[1];//+Y
	gyro[2] = - gyro_raw[0];//+Z

	//Degrees to Rad
	gyro[0] *= PI/180.0f;
	gyro[1] *= PI/180.0f;
	gyro[2] *= PI/180.0f;
}



void get_acc(uint8_t *IMU_tag_buff, float *acc0, float *acc, float *acc_raw) {
	//float acc_raw[3];
	acc_raw[0] = ((int16_t)(IMU_tag_buff[3]<<8 | IMU_tag_buff[4])) /2048.0 * 1;;//x of IMU
	acc_raw[1] = ((int16_t)(IMU_tag_buff[5]<<8 | IMU_tag_buff[6])) /2048.0 * 1;;//y of IMU
	acc_raw[2] = ((int16_t)(IMU_tag_buff[7]<<8 | IMU_tag_buff[8])) /2048.0 * 1;;//z of IMU

	//Apply offsets calculated through calibration.
	//OFFSETS REFER TO THE IMU REFERENCE FRAME
	acc_raw[0] -= acc0[0];
	acc_raw[1] -= acc0[1];
	acc_raw[2] -= acc0[2];
	//acc_raw[0] /= acc0[3];
	//acc_raw[1] /= acc0[3];
	//acc_raw[2] /= acc0[3];

	float acc_new[3];
	//Rotate the IMU reference frame to the Rocket Reference frame
	acc_new[0] =   acc_raw[2];//+X
	acc_new[1] =   acc_raw[1];//+Y
	acc_new[2] = - acc_raw[0];//+Z

	// [g] to [m/s^2]
	acc_new[0] *= 9.80665;//g
	acc_new[1] *= 9.80665;
	acc_new[2] *= 9.80665;

	//filter:
	float k = 0.9;
	acc[0] = k*acc[0] + (1-k) * acc_new[0];
	acc[1] = k*acc[1] + (1-k) * acc_new[1];
	acc[2] = k*acc[2] + (1-k) * acc_new[2];

	//acc[0] = round2(acc[0]);
	//acc[1] = round2(acc[1]);
	//acc[2] = round2(acc[2]);
}

void get_vel_pos(float *pos_earth, float *vel_earth, float *acc_earth, uint32_t micro_elaps) {
	double dt = micro_elaps/1e6;
	/*
	vel_earth[0] += dt * round2(acc_earth[0]);
	vel_earth[1] += dt * round2(acc_earth[1]);
	vel_earth[2] += dt * round2(acc_earth[2]);
	pos_earth[0] += dt * round2(vel_earth[0]) + 0.5*round2(acc_earth[0])*pow(dt,2);
	pos_earth[1] += dt * round2(vel_earth[1]) + 0.5*round2(acc_earth[1])*pow(dt,2);
	pos_earth[2] += dt * round2(vel_earth[2]) + 0.5*round2(acc_earth[2])*pow(dt,2);*/

	float a_thr = 0.05;
	if ((-a_thr < acc_earth[0]) && (acc_earth[0] < a_thr)) acc_earth[0] = 0;
	if ((-a_thr < acc_earth[1]) && (acc_earth[1] < a_thr)) acc_earth[1] = 0;
	if ((-a_thr < acc_earth[2]) && (acc_earth[2] < a_thr)) acc_earth[2] = 0;

	vel_earth[0] += dt * (acc_earth[0]);
	vel_earth[1] += dt * (acc_earth[1]);
	vel_earth[2] += dt * (acc_earth[2]);
	pos_earth[0] += dt * (vel_earth[0]) + 0.5*(acc_earth[0])*pow(dt,2);
	pos_earth[1] += dt * (vel_earth[1]) + 0.5*(acc_earth[1])*pow(dt,2);
	pos_earth[2] += dt * (vel_earth[2]) + 0.5*(acc_earth[2])*pow(dt,2);
}

void get_earth_acc(float *vec, float *q, float *result) {
	//q x vec x q*, same as target2earth, but removes g at the end
	float vec2quat[4] = {0, vec[0], vec[1], vec[2]};
	float conjq[4];
	quat_conjugate(q,conjq);
	float q_times_0vec[4];
	quat_multiply(q, vec2quat, q_times_0vec);
	float q_0vec_times_conjq[4];
	quat_multiply(q_times_0vec, conjq, q_0vec_times_conjq);

	result[0] = q_0vec_times_conjq[1];
	result[1] = q_0vec_times_conjq[2];
	result[2] = q_0vec_times_conjq[3] - 9.80665;
}

float get_press(uint8_t *Bar_pt_buff) {

	uint32_t press_raw = (int32_t)((Bar_pt_buff[3] << 16) | Bar_pt_buff[2] << 8 | Bar_pt_buff[1]);
	float press_bar = press_raw / 4096.0;

	return press_bar;
}

float get_bar_temp(uint8_t *Bar_pt_buff) {
	float temp_raw = (int16_t)((Bar_pt_buff[5] << 8) | Bar_pt_buff[4]);
	float temp = temp_raw / 480.0 + 42.5;
	return temp;
}

float get_bar_alt(float press) {

	//uint32_t press_raw = (int32_t)((Bar_pt_buff[3] << 16) | Bar_pt_buff[2] << 8 | Bar_pt_buff[1]);
	//float press_bar = press_raw / 4096.0;

	static const float T0 = 298; //T0 is the temperature at 0 meters level, in Kelvin
	static const float RR = 287.05; //Gas constant for dry air [J / (kg * K) ]
	static const float p0 = 1023; //Today's pressure at 0m level [hPa]
	static const float g0 = 9.80665; //No explanation needed, come on
	//WARNING!!!! :
	//press to altitude formula. Approximated through a Taylor expansion, but it's fine for my altitude range (up until ~5km)
	float alt = -T0*RR / (g0 * p0) * (press - p0);
	return alt;
}

void quat_normalize(float *q) {
	float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	if (norm > 1e-6) {
		q[0] = q[0]/norm;
		q[1] = q[1]/norm;
		q[2] = q[2]/norm;
		q[3] = q[3]/norm;
	}
}
void quat_multiply(float *q1, float *q2, float *result) {
	result[0] = q2[0]*q1[0] - q2[1]*q1[1] - q2[2]*q1[2] - q2[3]*q1[3];
	result[1] = q2[0]*q1[1] + q2[1]*q1[0] - q2[2]*q1[3] + q2[3]*q1[2];
	result[2] = q2[0]*q1[2] + q2[1]*q1[3] + q2[2]*q1[0] - q2[3]*q1[1];
	result[3] = q2[0]*q1[3] - q2[1]*q1[2] + q2[2]*q1[1] + q2[3]*q1[0];
}
void quat_conjugate(float *q, float *result) {
	result[0] =  q[0];
	result[1] = -q[1];
	result[2] = -q[2];
	result[3] = -q[3];
}
void earth2body(float *q, float *vec, float *result) {
	//performs the passive rotation conj(q)*(0, vec)*q. It brings a vector from earth to body frame,
	//through the quaternion q that instead represents the active rotation from body earth
	float vec2quat[4] = {0, vec[0], vec[1], vec[2]};
	float conjq[4];
	quat_conjugate(q,conjq);
	float conjq_times_0vec[4];
	quat_multiply(conjq, vec2quat, conjq_times_0vec);
	float conjq_0vec_times_q[4];
	quat_multiply(conjq_times_0vec, q, conjq_0vec_times_q);
	result[0] = conjq_0vec_times_q[1];
	result[1] = conjq_0vec_times_q[2];
	result[2] = conjq_0vec_times_q[3];
}

void body2earth(float *q, float *vec, float *result) {
	//q x vec x q*, same as target2earth
	float vec2quat[4] = {0, vec[0], vec[1], vec[2]};
	float conjq[4];
	quat_conjugate(q,conjq);
	float q_times_0vec[4];
	quat_multiply(q, vec2quat, q_times_0vec);
	float q_0vec_times_conjq[4];
	quat_multiply(q_times_0vec, conjq, q_0vec_times_conjq);
	result[0] = q_0vec_times_conjq[1];
	result[1] = q_0vec_times_conjq[2];
	result[2] = q_0vec_times_conjq[3];
}

void target2earth(float *q, float *vec, float *result) {
	//performs the active rotation q*(0, vec)*conj(q)
	float vec2quat[4] = {0, vec[0], vec[1], vec[2]};
	float conjq[4];
	quat_conjugate(q,conjq);
	float q_times_0vec[4];
	quat_multiply(q, vec2quat, q_times_0vec);
	float q_0vec_times_conjq[4];
	quat_multiply(q_times_0vec, conjq, q_0vec_times_conjq);
	result[0] = q_0vec_times_conjq[1];
	result[1] = q_0vec_times_conjq[2];
	result[2] = q_0vec_times_conjq[3];
}

float my_cos(float angle){
	return (1-(angle*angle)/2);
}
float my_sin(float angle){
	return ((angle));
}

float saturate(float var, float min_val, float max_val) {
	if (var<min_val) var = min_val;
	if (var>max_val) var = max_val;
	return var;
}

float theta2servo(float angle){ //degrees to degrees
	return(-0.017369798316367473*angle*angle + 3.1495686957697946*angle + 107.24902765456899);
}
float gamma2servo(float angle){ //degrees to degrees
	return(0.006130875311898612*angle*angle -2.408525444459922*angle + 87.30219354006888);
}

void gyro2quat_integration(float *gyro, float *quat, uint32_t micro_elaps) { //the elaps time must me in microseconds
	float mult[4] = {0,0,0,0};
	mult[0] = 0.5*(0*quat[0] - gyro[0]*quat[1] - gyro[1]*quat[2] - gyro[2]*quat[3]);
	mult[1] = 0.5*(0*quat[1] + gyro[0]*quat[0] - gyro[1]*quat[3] + gyro[2]*quat[2]);
	mult[2] = 0.5*(0*quat[2] + gyro[0]*quat[3] + gyro[1]*quat[0] - gyro[2]*quat[1]);
	mult[3] = 0.5*(0*quat[3] - gyro[0]*quat[2] + gyro[1]*quat[1] + gyro[2]*quat[0]);

	quat[0] += mult[0] * (micro_elaps/1e6);
	quat[1] += mult[1] * (micro_elaps/1e6);
	quat[2] += mult[2] * (micro_elaps/1e6);
	quat[3] += mult[3] * (micro_elaps/1e6);

	quat_normalize(quat);
}
void get_relative_quat(float *body_quat, float *target_quat, float *relative_quat) {
	float target_conj[4];
	quat_conjugate(target_quat, target_conj);//(q, result)
	quat_multiply(body_quat, target_conj, relative_quat);//(q1, q2, result). Now we have the relative_quat
}
void quat2axang(float *quat, float *axang) {
	//axang = [x y z theta]]; //x,y,z,theta
	static const float epsilon = 1e-6; //value of quat[1]^2+quat[2]^2+quat[3]^2 under which the axang is considered to be [0 0 1 0]. should limit "nan" in operations
	axang[0] = 0;
	axang[1] = 0;
	axang[2] = 0;
	axang[3] = 0;
	float norm = quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3];
	if (norm>1) {
	  quat[0]=quat[0]/sqrt(norm);
	  quat[1]=quat[1]/sqrt(norm);
	  quat[2]=quat[2]/sqrt(norm);
	  quat[3]=quat[3]/sqrt(norm);
	}
	if ((quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]) <= epsilon) {
	  axang[2] = 1;
	  return;
	}
	axang[3] = 2*acos(quat[0]); // rotation angle theta in range (0, 2*pi)
	float n_inv = 1/sqrt(1 - quat[0]*quat[0]);
	// unit vector u:
	axang[0] = quat[1]*n_inv;
	axang[1] = quat[2]*n_inv;
	axang[2] = quat[3]*n_inv;
}
void updateReqTorque(float *axang, float *gyro, float *target_gyro, float *body_quat, float *target_quat, float *ReqTorque, uint32_t micro_elaps) {

	static float integral_x = 0.0f,
				 integral_y = 0.0f,
				 integral_z = 0.0f;

	//PROPORTIONAL PART
	float pvec_earth[3] = {//this vector represents the (axis vector)*angle, but in the earth frame
		axang[3]*axang[0],
		axang[3]*axang[1],
		axang[3]*axang[2]
	};
	//so we bring it in the body frame
	float pvec_body[3];
	earth2body(body_quat, pvec_earth, pvec_body);
	//and then we get the components
	float prop_x = pvec_body[0];
	float prop_y = pvec_body[1];
	float prop_z = pvec_body[2];

	//INTEGRAL PART
	integral_x += prop_x*(micro_elaps/1e6);
	integral_y += prop_y*(micro_elaps/1e6);
	integral_z += prop_z*(micro_elaps/1e6);

	//Anti windup
	const float intsat = 0.1;
	if (integral_x < -intsat)            integral_x = -intsat;
	if (integral_x >  intsat)            integral_x =  intsat;
	if (integral_y < -intsat)            integral_y = -intsat;
	if (integral_y >  intsat)            integral_y =  intsat;
	if (integral_z < -intsat)            integral_z = -intsat;
	if (integral_z >  intsat)            integral_z =  intsat;

	//DERIVATIVE PART
	//currently the target_gyro is expressed in the target frame, so we bring it to the body frame by 2 rotations:
	float target_gyro_earth[3];
	target2earth(target_quat, target_gyro, target_gyro_earth);//active rotation. vec Target-->Earth
	float target_gyro_body[3];
	earth2body(body_quat, target_gyro_earth, target_gyro_body);//passive rotation. vec Earth-->Body
	//now we have the angluar rotation we would like to have during a manouver, expressed in the body frame. We want to match it
	//Notice that since to get alpha we do (... -KD*w), we are trying to "angular accelerate" opposize to gyro[], and in the same direction of target_gyro[]
	float wx = gyro[0] - target_gyro_body[0],
			wy = gyro[1] - target_gyro_body[1],
			wz = gyro[2] - target_gyro_body[2];
	//GET ANGULAR ACCELERATION
	float alphax,alphay,alphaz;//required angular acceleration
	alphax =  -KP*prop_x - KI*integral_x - KD*wx;
	alphay =  -KP*prop_y - KI*integral_y - KD*wy;
	alphaz =  -KP*prop_z - KI*integral_z - KD*wz;

	//GET REQUIRED TORQUE THROUGH EULER's EQUATION. I@alpha + w x (I@w) = Torque
	ReqTorque[0] = Ix*alphax - Iy*wy*wz + Iz*wy*wz;
	ReqTorque[1] = Iy*alphay + Ix*wx*wz - Iz*wx*wz;
	ReqTorque[2] = Iz*alphaz - Ix*wx*wy + Iy*wx*wy;

	//APPLY REQUIRED TORQUE GAINS
	ReqTorque[0] *=  R_gain_x;
	ReqTorque[1] *=  R_gain_y;
	ReqTorque[2] *=  R_gain_z;
}
void get_parabVertex_angles(float *thetas, float *gammas, float *Forces, float *ReqTorque) {
	//assignments
	float th1 = thetas[0];
	float th2 = thetas[1];
	float th3 = thetas[2];
	float ga1 = gammas[0];
	float ga2 = gammas[1];
	float ga3 = gammas[2];
	float F1  = Forces[0];
	float F2  = Forces[1];
	float F3  = Forces[2];
	float reqx = ReqTorque[0];
	float reqy = ReqTorque[1];
	float reqz = ReqTorque[2];

	//Precompute common terms
	static const float sin_mu1 = sin(0.0);
	static const float sin_mu2 = sin(2.0/3.0*PI);
	static const float sin_mu3 = sin(4.0/3.0*PI);
	static const float cos_mu1 = cos(0.0);
	static const float cos_mu2 = cos(2.0/3.0*PI);
	static const float cos_mu3 = cos(4.0/3.0*PI);

	float sq_ga1 = ga1 * ga1;
	float sq_ga2 = ga2 * ga2;
	float sq_ga3 = ga3 * ga3;

	float sq_th1 = th1 * th1;
	float sq_th2 = th2 * th2;
	float sq_th3 = th3 * th3;

	float sq_ga1_2 = sq_ga1 - 2;
	float sq_ga2_2 = sq_ga2 - 2;
	float sq_ga3_2 = sq_ga3 - 2;
	float sq_th1_2 = sq_th1 - 2;
	float sq_th2_2 = sq_th2 - 2;
	float sq_th3_2 = sq_th3 - 2;

	float F1_lx = F1 * lx;
	float F1_ly = F1 * ly;
	float F1_lz = F1 * lz;

	float F2_lx = F2 * lx;
	float F2_ly = F2 * ly;
	float F2_lz = F2 * lz;

	float F3_lx = F3 * lx;
	float F3_ly = F3 * ly;
	float F3_lz = F3 * lz;

	float sq_F1 = F1*F1;
	float sq_F2 = F2*F2;
	float sq_F3 = F3*F3;
	float sq_lx = lx*lx;
	float sq_ly = ly*ly;
	float sq_lz = lz*lz;

	static const float outer_lower = -0.104; //-6 degrees
	static const float outer_upper =  0.139; //8 degrees
	static const float inner_lower = -0.122; //-7 degrees
	static const float inner_upper =  0.122; //7 degrees


	th1 = (2*F1_lx*sq_ga1_2*(- F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1*ga1*ly + 2*F2_lx*th2 + 2*F3_lx*th3) - F1_lz*cos_mu1*sq_ga1_2*(4*reqx + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F1*sq_ga1_2*(ly*cos_mu1 + lx*sin_mu1) + 4*F1*ga1*lz*sin_mu1 - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) + F1_lz*sin_mu1*sq_ga1_2*(2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F1*sq_ga1_2*(lx*cos_mu1 - ly*sin_mu1) + 4*F1*ga1*lz*cos_mu1 - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)))/(8*K + 2*sq_F1*sq_lx*pow(sq_ga1_2,2) + 2*sq_F1*sq_lz*pow(sq_ga1_2,2) - F1*sq_ga1_2*(ly*cos_mu1 + lx*sin_mu1)*(4*reqx + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F1*sq_ga1_2*(ly*cos_mu1 + lx*sin_mu1) + 4*F1*ga1*lz*sin_mu1 - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) - F1*sq_ga1_2*(lx*cos_mu1 - ly*sin_mu1)*(2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F1*sq_ga1_2*(lx*cos_mu1 - ly*sin_mu1) + 4*F1*ga1*lz*cos_mu1 - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)));
	if (th1 < outer_lower)            th1 = outer_lower;
	if (th1 > outer_upper)            th1 = outer_upper;
	sq_th1 = th1 * th1;
	sq_th1_2 = sq_th1 - 2;

	th2 = (2*F2_lx*sq_ga2_2*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F2*ga2*ly + 2*F1_lx*th1 + 2*F3_lx*th3) - F2_lz*cos_mu2*sq_ga2_2*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F2*sq_ga2_2*(ly*cos_mu2 + lx*sin_mu2) + 4*F2*ga2*lz*sin_mu2 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) + F2_lz*sin_mu2*sq_ga2_2*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F2*sq_ga2_2*(lx*cos_mu2 - ly*sin_mu2) + 4*F2*ga2*lz*cos_mu2 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)))/(8*K + 2*sq_F2*sq_lx*pow(sq_ga2_2,2) + 2*sq_F2*sq_lz*pow(sq_ga2_2,2) - F2*sq_ga2_2*(ly*cos_mu2 + lx*sin_mu2)*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F2*sq_ga2_2*(ly*cos_mu2 + lx*sin_mu2) + 4*F2*ga2*lz*sin_mu2 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) - F2*sq_ga2_2*(lx*cos_mu2 - ly*sin_mu2)*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F2*sq_ga2_2*(lx*cos_mu2 - ly*sin_mu2) + 4*F2*ga2*lz*cos_mu2 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)));
	if (th2 < outer_lower)            th2 = outer_lower;
	if (th2 > outer_upper)            th2 = outer_upper;
	sq_th2 = th2 * th2;
	sq_th2_2 = sq_th2 - 2;

	th3 = (2*F3_lx*sq_ga3_2*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 + 2*reqz + 2*F3*ga3*ly + 2*F1_lx*th1 + 2*F2_lx*th2) - F3_lz*cos_mu3*sq_ga3_2*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3*sq_ga3_2*(ly*cos_mu3 + lx*sin_mu3) + 4*F3*ga3*lz*sin_mu3 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2)) + F3_lz*sin_mu3*sq_ga3_2*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) + 2*F3*sq_ga3_2*(lx*cos_mu3 - ly*sin_mu3) + 4*F3*ga3*lz*cos_mu3 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2)))/(8*K + 2*sq_F3*sq_lx*pow(sq_ga3_2,2) + 2*sq_F3*sq_lz*pow(sq_ga3_2,2) - F3*sq_ga3_2*(ly*cos_mu3 + lx*sin_mu3)*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3*sq_ga3_2*(ly*cos_mu3 + lx*sin_mu3) + 4*F3*ga3*lz*sin_mu3 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2)) - F3*sq_ga3_2*(lx*cos_mu3 - ly*sin_mu3)*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) + 2*F3*sq_ga3_2*(lx*cos_mu3 - ly*sin_mu3) + 4*F3*ga3*lz*cos_mu3 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2)));
	if (th3 < outer_lower)            th3 = outer_lower;
	if (th3 > outer_upper)            th3 = outer_upper;
	sq_th3 = th3 * th3;
	sq_th3_2 = sq_th3 - 2;

	ga1 = -(4*F1_ly*(- F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3) + 2*F1_lz*sin_mu1*(4*reqx + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F1*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - 4*F1_lz*th1*cos_mu1 - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) + 2*F1_lz*cos_mu1*(2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F1*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) + 4*F1_lz*th1*sin_mu1 - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)))/(8*K - (F1*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - 2*F1_lz*th1*cos_mu1)*(4*reqx + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F1*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - 4*F1_lz*th1*cos_mu1 - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) - (F1*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) + 2*F1_lz*th1*sin_mu1)*(2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F1*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) + 4*F1_lz*th1*sin_mu1 - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)) + 8*sq_F1*sq_ly + 8*sq_F1*sq_lz - 4*F1_lx*th1*(- F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3));
	if (ga1 < inner_lower)            ga1 = inner_lower;
	if (ga1 > inner_upper)            ga1 = inner_upper;
	sq_ga1 = ga1 * ga1;
	sq_ga1_2 = sq_ga1 - 2;

	ga2 = -(4*F2_ly*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3) + 2*F2_lz*sin_mu2*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - 4*F2_lz*th2*cos_mu2 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) + 2*F2_lz*cos_mu2*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) + 4*F2_lz*th2*sin_mu2 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)))/(8*K - (F2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - 2*F2_lz*th2*cos_mu2)*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - 4*F2_lz*th2*cos_mu2 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) - (F2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) + 2*F2_lz*th2*sin_mu2)*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) + 4*F2_lz*th2*sin_mu2 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)) + 8*sq_F2*sq_ly + 8*sq_F2*sq_lz - 4*F2_lx*th2*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3));
	if (ga2 < inner_lower)            ga2 = inner_lower;
	if (ga2 > inner_upper)            ga2 = inner_upper;
	sq_ga2 = ga2 * ga2;
	sq_ga2_2 = sq_ga2 - 2;

	ga3 = -(4*F3_ly*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3) + 2*F3_lz*sin_mu3*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3) - 4*F3_lz*th3*cos_mu3 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2)) + 2*F3_lz*cos_mu3*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) + 2*F3*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3) + 4*F3_lz*th3*sin_mu3 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2)))/(8*K - (F3*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3) - 2*F3_lz*th3*cos_mu3)*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3) - 4*F3_lz*th3*cos_mu3 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2)) - (F3*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3) + 2*F3_lz*th3*sin_mu3)*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) + 2*F3*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3) + 4*F3_lz*th3*sin_mu3 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2)) + 8*sq_F3*sq_ly + 8*sq_F3*sq_lz - 4*F3_lx*th3*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3));
	if (ga3 < inner_lower)            ga3 = inner_lower;
	if (ga3 > inner_upper)            ga3 = inner_upper;

	/*
	th1 = (2*F1_lx*sq_ga1_2*(- F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1_ly*ga1 + 2*F2_lx*th2 + 2*F3_lx*th3) - F1_lz*cos_mu1*sq_ga1_2*(4*reqx + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F1*sq_ga1_2*(ly*cos_mu1 + lx*sin_mu1) + 4*F1_lz*ga1*sin_mu1 - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) + F1_lz*sin_mu1*sq_ga1_2*(2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F1*sq_ga1_2*(lx*cos_mu1 - ly*sin_mu1) + 4*F1_lz*ga1*cos_mu1 - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)))/(8*K + 2*sq_F1*sq_lx*sq_ga1_2*sq_ga1_2 + 2*sq_F1*sq_lz*sq_ga1_2*sq_ga1_2 - F1*sq_ga1_2*(ly*cos_mu1 + lx*sin_mu1)*(4*reqx + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F1*sq_ga1_2*(ly*cos_mu1 + lx*sin_mu1) + 4*F1_lz*ga1*sin_mu1 - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) - F1*sq_ga1_2*(lx*cos_mu1 - ly*sin_mu1)*(2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F1*sq_ga1_2*(lx*cos_mu1 - ly*sin_mu1) + 4*F1_lz*ga1*cos_mu1 - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)));
	th2 = (2*F2_lx*sq_ga2_2*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F2_ly*ga2 + 2*F1_lx*th1 + 2*F3_lx*th3) - F2_lz*cos_mu2*sq_ga2_2*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F2*sq_ga2_2*(ly*cos_mu2 + lx*sin_mu2) + 4*F2_lz*ga2*sin_mu2 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) + F2_lz*sin_mu2*sq_ga2_2*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F2*sq_ga2_2*(lx*cos_mu2 - ly*sin_mu2) + 4*F2_lz*ga2*cos_mu2 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)))/(8*K + 2*sq_F2*sq_lx*sq_ga2_2*sq_ga2_2 + 2*sq_F2*sq_lz*sq_ga2_2*sq_ga2_2 - F2*sq_ga2_2*(ly*cos_mu2 + lx*sin_mu2)*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F2*sq_ga2_2*(ly*cos_mu2 + lx*sin_mu2) + 4*F2_lz*ga2*sin_mu2 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) - F2*sq_ga2_2*(lx*cos_mu2 - ly*sin_mu2)*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F2*sq_ga2_2*(lx*cos_mu2 - ly*sin_mu2) + 4*F2_lz*ga2*cos_mu2 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)));
	th3 = (2*F3_lx*sq_ga3_2*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 + 2*reqz + 2*F3_ly*ga3 + 2*F1_lx*th1 + 2*F2_lx*th2) - F3_lz*cos_mu3*sq_ga3_2*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3*sq_ga3_2*(ly*cos_mu3 + lx*sin_mu3) + 4*F3_lz*ga3*sin_mu3 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2)) + F3_lz*sin_mu3*sq_ga3_2*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) + 2*F3*sq_ga3_2*(lx*cos_mu3 - ly*sin_mu3) + 4*F3_lz*ga3*cos_mu3 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2)))/(8*K + 2*sq_F3*sq_lx*sq_ga3_2*sq_ga3_2 + 2*sq_F3*sq_lz*sq_ga3_2*sq_ga3_2 - F3*sq_ga3_2*(ly*cos_mu3 + lx*sin_mu3)*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3*sq_ga3_2*(ly*cos_mu3 + lx*sin_mu3) + 4*F3_lz*ga3*sin_mu3 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2)) - F3*sq_ga3_2*(lx*cos_mu3 - ly*sin_mu3)*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) + 2*F3*sq_ga3_2*(lx*cos_mu3 - ly*sin_mu3) + 4*F3_lz*ga3*cos_mu3 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2)));
	ga1 = -(4*F1_ly*(- F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3) + 2*F1_lz*sin_mu1*(4*reqx + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F1*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - 4*F1_lz*th1*cos_mu1 - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) + 2*F1_lz*cos_mu1*(2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F1*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) + 4*F1_lz*th1*sin_mu1 - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)))/(8*K - (F1*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - 2*F1_lz*th1*cos_mu1)*(4*reqx + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F1*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - 4*F1_lz*th1*cos_mu1 - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) - (F1*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) + 2*F1_lz*th1*sin_mu1)*(2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F1*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) + 4*F1_lz*th1*sin_mu1 - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)) + 8*sq_F1*sq_ly + 8*sq_F1*sq_lz - 4*F1_lx*th1*(- F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3));
	ga2 = -(4*F2_ly*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3) + 2*F2_lz*sin_mu2*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - 4*F2_lz*th2*cos_mu2 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) + 2*F2_lz*cos_mu2*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) + 4*F2_lz*th2*sin_mu2 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)))/(8*K - (F2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - 2*F2_lz*th2*cos_mu2)*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - 4*F2_lz*th2*cos_mu2 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) - (F2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) + 2*F2_lz*th2*sin_mu2)*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) + 4*F2_lz*th2*sin_mu2 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)) + 8*sq_F2*sq_ly + 8*sq_F2*sq_lz - 4*F2_lx*th2*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3));
	ga3 = -(4*F3_ly*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3) + 2*F3_lz*sin_mu3*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3) - 4*F3_lz*th3*cos_mu3 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2)) + 2*F3_lz*cos_mu3*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) + 2*F3*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3) + 4*F3_lz*th3*sin_mu3 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2)))/(8*K - (F3*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3) - 2*F3_lz*th3*cos_mu3)*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3) - 4*F3_lz*th3*cos_mu3 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2)) - (F3*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3) + 2*F3_lz*th3*sin_mu3)*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) + 2*F3*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3) + 4*F3_lz*th3*sin_mu3 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2)) + 8*sq_F3*sq_ly + 8*sq_F3*sq_lz - 4*F3_lx*th3*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3));
*/


	thetas[0] = th1;
	thetas[1] = th2;
	thetas[2] = th3;
	gammas[0] = ga1;
	gammas[1] = ga2;
	gammas[2] = ga3;
}

void get_angles_RMS_and_paraboloid(float *thetas, float *gammas, float *Forces, float *ReqTorque, float* diff_pointer) {
	//assignments
	float th1 = thetas[0];
	float th2 = thetas[1];
	float th3 = thetas[2];
	float ga1 = gammas[0];
	float ga2 = gammas[1];
	float ga3 = gammas[2];
	float F1  = Forces[0];
	float F2  = Forces[1];
	float F3  = Forces[2];
	float reqx = ReqTorque[0];
	float reqy = ReqTorque[1];
	float reqz = ReqTorque[2];

	//Precompute common terms
	//Precompute common terms
	static const float sin_mu1 = sin(0.0);
	static const float sin_mu2 = sin(2.0/3.0*PI);
	static const float sin_mu3 = sin(4.0/3.0*PI);
	static const float cos_mu1 = cos(0.0);
	static const float cos_mu2 = cos(2.0/3.0*PI);
	static const float cos_mu3 = cos(4.0/3.0*PI);

	float sq_ga1 = ga1 * ga1;
	float sq_ga2 = ga2 * ga2;
	float sq_ga3 = ga3 * ga3;

	float sq_th1 = th1 * th1;
	float sq_th2 = th2 * th2;
	float sq_th3 = th3 * th3;

	float sq_ga1_2 = sq_ga1 - 2;
	float sq_ga2_2 = sq_ga2 - 2;
	float sq_ga3_2 = sq_ga3 - 2;
	float sq_th1_2 = sq_th1 - 2;
	float sq_th2_2 = sq_th2 - 2;
	float sq_th3_2 = sq_th3 - 2;

	float sq_th1_2_1 = sq_th1/2.0 - 1;
	float sq_th2_2_1 = sq_th2/2.0 - 1;
	float sq_th3_2_1 = sq_th3/2.0 - 1;
	float sq_ga1_2_1 = sq_ga1/2.0 - 1;
	float sq_ga2_2_1 = sq_ga2/2.0 - 1;
	float sq_ga3_2_1 = sq_ga3/2.0 - 1;

	float F1_lx = F1 * lx;
	float F1_ly = F1 * ly;
	float F1_lz = F1 * lz;

	float F2_lx = F2 * lx;
	float F2_ly = F2 * ly;
	float F2_lz = F2 * lz;

	float F3_lx = F3 * lx;
	float F3_ly = F3 * ly;
	float F3_lz = F3 * lz;

	float sq_F1 = F1*F1;
	float sq_F2 = F2*F2;
	float sq_F3 = F3*F3;
	float sq_lx = lx*lx; //not a const since i will consider varying these parameters due to CM moving (should probably only apply to lz anyways)
	float sq_ly = ly*ly;
	float sq_lz = lz*lz;
	//end of precomputation

	//Limits of motor angle of inclination, for each axis of freedom
	static const float outer_lower = -0.104; //-6 degrees
	static const float outer_upper =  0.139; //8 degrees
	static const float inner_lower = -0.122; //-7 degrees
	static const float inner_upper =  0.122; //7 degrees


	static const float method_threshold = 1e-3;//Cost function threshold under which we switch from Gradient Descent to Newton's Method. The cost function is diff, see below
	static float V[6] = {};//v(t)​ is the accumulated moving average of squared gradients
	static const float b = 0.9; //parameter to calculate the moving average of the square gradients, v(t)
	static float dx[6] = {}; //difference between last update and current update. Used for momentum

	//First we calculate the cost function: (ReqTorque - ProvidedTorque)^2, to decide between what numerical method to use.
	//Newton's method is used when close to a solution since we are sure it won't  diverge if close. Otherwise grad descent, since it can't diverge, or converge to maximums of the cost function
	float diff = pow(reqx + F1_lz*(ga1*sin_mu1 + th1*cos_mu1*sq_ga1_2_1) + F2_lz*(ga2*sin_mu2 + th2*cos_mu2*sq_ga2_2_1) + F3_lz*(ga3*sin_mu3 + th3*cos_mu3*sq_ga3_2_1) - F1*sq_ga1_2_1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2_1*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2_1*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3),2) + pow(reqy - F1_lz*(ga1*cos_mu1 - th1*sin_mu1*sq_ga1_2_1) - F2_lz*(ga2*cos_mu2 - th2*sin_mu2*sq_ga2_2_1) - F3_lz*(ga3*cos_mu3 - th3*sin_mu3*sq_ga3_2_1) + F1*sq_ga1_2_1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1) + F2*sq_ga2_2_1*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2) + F3*sq_ga3_2_1*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3),2) + pow(reqz - F1*(ga1*sin_mu1 + th1*cos_mu1*sq_ga1_2_1)*(lx*cos_mu1 - ly*sin_mu1) + F1*(ga1*cos_mu1 - th1*sin_mu1*sq_ga1_2_1)*(ly*cos_mu1 + lx*sin_mu1) - F2*(ga2*sin_mu2 + th2*cos_mu2*sq_ga2_2_1)*(lx*cos_mu2 - ly*sin_mu2) + F2*(ga2*cos_mu2 - th2*sin_mu2*sq_ga2_2_1)*(ly*cos_mu2 + lx*sin_mu2) - F3*(ga3*sin_mu3 + th3*cos_mu3*sq_ga3_2_1)*(lx*cos_mu3 - ly*sin_mu3) + F3*(ga3*cos_mu3 - th3*sin_mu3*sq_ga3_2_1)*(ly*cos_mu3 + lx*sin_mu3),2);

	float old_angles[6] = { //used for momentum
			thetas[0],
			gammas[0],
			thetas[1],
			gammas[1],
			thetas[2],
			gammas[2]
	};

	if (diff < method_threshold) { //DOES NEWTON's METHOD --- PARABOLA VERTEX METHOD. The threshold was found heuristically through simulations

		//setup RMS prop with Momentum:
		/* calculates V at every step since in the case we had to switch to grad descent, we need the current values of V
		* to have a ready response and not have to wait for it to converge (since V is a moving average).
		* The same thing will be done later for dx, to have a ready-to-go momentum.
		*/
		float grad_th1 =                                                          2*K*th1 - F1*sq_ga1_2*(ly*th1*cos_mu1 - lz*cos_mu1 + lx*th1*sin_mu1)*(reqx + F1_lz*((th1*cos_mu1*sq_ga1)/2 + sin_mu1*ga1 - th1*cos_mu1) + F2_lz*((th2*cos_mu2*sq_ga2)/2 + sin_mu2*ga2 - th2*cos_mu2) + F3_lz*((th3*cos_mu3*sq_ga3)/2 + sin_mu3*ga3 - th3*cos_mu3) - F1*sq_ga1_2_1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2_1*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2_1*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3)) + F1*sq_ga1_2*(lz*sin_mu1 + lx*th1*cos_mu1 - ly*th1*sin_mu1)*(reqy - F1_lz*(ga1*cos_mu1 + th1*sin_mu1 - (sq_ga1*th1*sin_mu1)/2) - F2_lz*(ga2*cos_mu2 + th2*sin_mu2 - (sq_ga2*th2*sin_mu2)/2) - F3_lz*(ga3*cos_mu3 + th3*sin_mu3 - (sq_ga3*th3*sin_mu3)/2) + F1*sq_ga1_2_1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1) + F2*sq_ga2_2_1*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2) + F3*sq_ga3_2_1*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3)) - F1_lx*sq_ga1_2*(reqz + F1*ga1*ly + F2*ga2*ly + F3*ga3*ly + F1_lx*th1 + F2_lx*th2 + F3_lx*th3 - (F1*sq_ga1*lx*th1)/2 - (F2*sq_ga2*lx*th2)/2 - (F3*sq_ga3*lx*th3)/2);
		float grad_ga1 = 2*K*ga1 - 2*(F1_lz*(cos_mu1 - ga1*th1*sin_mu1) - F1*ga1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1))*(reqy - F1_lz*(ga1*cos_mu1 + th1*sin_mu1 - (sq_ga1*th1*sin_mu1)/2) - F2_lz*(ga2*cos_mu2 + th2*sin_mu2 - (sq_ga2*th2*sin_mu2)/2) - F3_lz*(ga3*cos_mu3 + th3*sin_mu3 - (sq_ga3*th3*sin_mu3)/2) + F1*sq_ga1_2_1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1) + F2*sq_ga2_2_1*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2) + F3*sq_ga3_2_1*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3)) + 2*(F1_lz*(sin_mu1 + ga1*th1*cos_mu1) - F1*ga1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1))*(reqx + F1_lz*((th1*cos_mu1*sq_ga1)/2 + sin_mu1*ga1 - th1*cos_mu1) + F2_lz*((th2*cos_mu2*sq_ga2)/2 + sin_mu2*ga2 - th2*cos_mu2) + F3_lz*((th3*cos_mu3*sq_ga3)/2 + sin_mu3*ga3 - th3*cos_mu3) - F1*sq_ga1_2_1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2_1*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2_1*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3)) + 2*F1*(ly - ga1*lx*th1)*(reqz + F1*ga1*ly + F2*ga2*ly + F3*ga3*ly + F1_lx*th1 + F2_lx*th2 + F3_lx*th3 - (F1*sq_ga1*lx*th1)/2 - (F2*sq_ga2*lx*th2)/2 - (F3*sq_ga3*lx*th3)/2);
		float grad_th2 =                                                          2*K*th2 - F2*sq_ga2_2*(ly*th2*cos_mu2 - lz*cos_mu2 + lx*th2*sin_mu2)*(reqx + F1_lz*((th1*cos_mu1*sq_ga1)/2 + sin_mu1*ga1 - th1*cos_mu1) + F2_lz*((th2*cos_mu2*sq_ga2)/2 + sin_mu2*ga2 - th2*cos_mu2) + F3_lz*((th3*cos_mu3*sq_ga3)/2 + sin_mu3*ga3 - th3*cos_mu3) - F1*sq_ga1_2_1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2_1*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2_1*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3)) + F2*sq_ga2_2*(lz*sin_mu2 + lx*th2*cos_mu2 - ly*th2*sin_mu2)*(reqy - F1_lz*(ga1*cos_mu1 + th1*sin_mu1 - (sq_ga1*th1*sin_mu1)/2) - F2_lz*(ga2*cos_mu2 + th2*sin_mu2 - (sq_ga2*th2*sin_mu2)/2) - F3_lz*(ga3*cos_mu3 + th3*sin_mu3 - (sq_ga3*th3*sin_mu3)/2) + F1*sq_ga1_2_1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1) + F2*sq_ga2_2_1*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2) + F3*sq_ga3_2_1*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3)) - F2_lx*sq_ga2_2*(reqz + F1*ga1*ly + F2*ga2*ly + F3*ga3*ly + F1_lx*th1 + F2_lx*th2 + F3_lx*th3 - (F1*sq_ga1*lx*th1)/2 - (F2*sq_ga2*lx*th2)/2 - (F3*sq_ga3*lx*th3)/2);
		float grad_ga2 = 2*K*ga2 - 2*(F2_lz*(cos_mu2 - ga2*th2*sin_mu2) - F2*ga2*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2))*(reqy - F1_lz*(ga1*cos_mu1 + th1*sin_mu1 - (sq_ga1*th1*sin_mu1)/2) - F2_lz*(ga2*cos_mu2 + th2*sin_mu2 - (sq_ga2*th2*sin_mu2)/2) - F3_lz*(ga3*cos_mu3 + th3*sin_mu3 - (sq_ga3*th3*sin_mu3)/2) + F1*sq_ga1_2_1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1) + F2*sq_ga2_2_1*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2) + F3*sq_ga3_2_1*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3)) + 2*(F2_lz*(sin_mu2 + ga2*th2*cos_mu2) - F2*ga2*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2))*(reqx + F1_lz*((th1*cos_mu1*sq_ga1)/2 + sin_mu1*ga1 - th1*cos_mu1) + F2_lz*((th2*cos_mu2*sq_ga2)/2 + sin_mu2*ga2 - th2*cos_mu2) + F3_lz*((th3*cos_mu3*sq_ga3)/2 + sin_mu3*ga3 - th3*cos_mu3) - F1*sq_ga1_2_1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2_1*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2_1*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3)) + 2*F2*(ly - ga2*lx*th2)*(reqz + F1*ga1*ly + F2*ga2*ly + F3*ga3*ly + F1_lx*th1 + F2_lx*th2 + F3_lx*th3 - (F1*sq_ga1*lx*th1)/2 - (F2*sq_ga2*lx*th2)/2 - (F3*sq_ga3*lx*th3)/2);
		float grad_th3 =                                                          2*K*th3 - F3*sq_ga3_2*(ly*th3*cos_mu3 - lz*cos_mu3 + lx*th3*sin_mu3)*(reqx + F1_lz*((th1*cos_mu1*sq_ga1)/2 + sin_mu1*ga1 - th1*cos_mu1) + F2_lz*((th2*cos_mu2*sq_ga2)/2 + sin_mu2*ga2 - th2*cos_mu2) + F3_lz*((th3*cos_mu3*sq_ga3)/2 + sin_mu3*ga3 - th3*cos_mu3) - F1*sq_ga1_2_1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2_1*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2_1*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3)) + F3*sq_ga3_2*(lz*sin_mu3 + lx*th3*cos_mu3 - ly*th3*sin_mu3)*(reqy - F1_lz*(ga1*cos_mu1 + th1*sin_mu1 - (sq_ga1*th1*sin_mu1)/2) - F2_lz*(ga2*cos_mu2 + th2*sin_mu2 - (sq_ga2*th2*sin_mu2)/2) - F3_lz*(ga3*cos_mu3 + th3*sin_mu3 - (sq_ga3*th3*sin_mu3)/2) + F1*sq_ga1_2_1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1) + F2*sq_ga2_2_1*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2) + F3*sq_ga3_2_1*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3)) - F3_lx*sq_ga3_2*(reqz + F1*ga1*ly + F2*ga2*ly + F3*ga3*ly + F1_lx*th1 + F2_lx*th2 + F3_lx*th3 - (F1*sq_ga1*lx*th1)/2 - (F2*sq_ga2*lx*th2)/2 - (F3*sq_ga3*lx*th3)/2);
		float grad_ga3 = 2*K*ga3 - 2*(F3_lz*(cos_mu3 - ga3*th3*sin_mu3) - F3*ga3*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3))*(reqy - F1_lz*(ga1*cos_mu1 + th1*sin_mu1 - (sq_ga1*th1*sin_mu1)/2) - F2_lz*(ga2*cos_mu2 + th2*sin_mu2 - (sq_ga2*th2*sin_mu2)/2) - F3_lz*(ga3*cos_mu3 + th3*sin_mu3 - (sq_ga3*th3*sin_mu3)/2) + F1*sq_ga1_2_1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1) + F2*sq_ga2_2_1*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2) + F3*sq_ga3_2_1*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3)) + 2*(F3_lz*(sin_mu3 + ga3*th3*cos_mu3) - F3*ga3*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3))*(reqx + F1_lz*((th1*cos_mu1*sq_ga1)/2 + sin_mu1*ga1 - th1*cos_mu1) + F2_lz*((th2*cos_mu2*sq_ga2)/2 + sin_mu2*ga2 - th2*cos_mu2) + F3_lz*((th3*cos_mu3*sq_ga3)/2 + sin_mu3*ga3 - th3*cos_mu3) - F1*sq_ga1_2_1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2_1*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2_1*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3)) + 2*F3*(ly - ga3*lx*th3)*(reqz + F1*ga1*ly + F2*ga2*ly + F3*ga3*ly + F1_lx*th1 + F2_lx*th2 + F3_lx*th3 - (F1*sq_ga1*lx*th1)/2 - (F2*sq_ga2*lx*th2)/2 - (F3*sq_ga3*lx*th3)/2);

		V[0] = b*V[0] + (1-b)*pow(grad_th1,2);
		V[1] = b*V[1] + (1-b)*pow(grad_ga1,2);
		V[2] = b*V[2] + (1-b)*pow(grad_th2,2);
		V[3] = b*V[3] + (1-b)*pow(grad_ga2,2);
		V[4] = b*V[4] + (1-b)*pow(grad_th3,2);
		V[5] = b*V[5] + (1-b)*pow(grad_ga3,2);

		th1 = (2*F1_lx*sq_ga1_2*(- F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1*ga1*ly + 2*F2_lx*th2 + 2*F3_lx*th3) - F1_lz*cos_mu1*sq_ga1_2*(4*reqx + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F1*sq_ga1_2*(ly*cos_mu1 + lx*sin_mu1) + 4*F1*ga1*lz*sin_mu1 - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) + F1_lz*sin_mu1*sq_ga1_2*(2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F1*sq_ga1_2*(lx*cos_mu1 - ly*sin_mu1) + 4*F1*ga1*lz*cos_mu1 - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)))/(8*K + 2*sq_F1*sq_lx*pow(sq_ga1_2,2) + 2*sq_F1*sq_lz*pow(sq_ga1_2,2) - F1*sq_ga1_2*(ly*cos_mu1 + lx*sin_mu1)*(4*reqx + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F1*sq_ga1_2*(ly*cos_mu1 + lx*sin_mu1) + 4*F1*ga1*lz*sin_mu1 - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) - F1*sq_ga1_2*(lx*cos_mu1 - ly*sin_mu1)*(2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F1*sq_ga1_2*(lx*cos_mu1 - ly*sin_mu1) + 4*F1*ga1*lz*cos_mu1 - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)));
		th1 = saturate(th1, outer_lower, outer_upper);
		sq_th1 = th1 * th1;//recomputes frequent terms
		sq_th1_2 = sq_th1 - 2;
		sq_th1_2_1 = sq_th1/2.0 - 1;

		ga1 = -(4*F1_ly*(- F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3) + 2*F1_lz*sin_mu1*(4*reqx + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F1*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - 4*F1_lz*th1*cos_mu1 - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) + 2*F1_lz*cos_mu1*(2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F1*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) + 4*F1_lz*th1*sin_mu1 - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)))/(8*K - (F1*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - 2*F1_lz*th1*cos_mu1)*(4*reqx + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F1*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - 4*F1_lz*th1*cos_mu1 - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) - (F1*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) + 2*F1_lz*th1*sin_mu1)*(2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F1*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) + 4*F1_lz*th1*sin_mu1 - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)) + 8*sq_F1*sq_ly + 8*sq_F1*sq_lz - 4*F1_lx*th1*(- F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3));
		ga1 = saturate(ga1, inner_lower, inner_upper);
		sq_ga1 = ga1 * ga1;
		sq_ga1_2 = sq_ga1 - 2;
		sq_ga1_2_1 = sq_ga1/2.0 - 1;

		th2 = (2*F2_lx*sq_ga2_2*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F2*ga2*ly + 2*F1_lx*th1 + 2*F3_lx*th3) - F2_lz*cos_mu2*sq_ga2_2*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F2*sq_ga2_2*(ly*cos_mu2 + lx*sin_mu2) + 4*F2*ga2*lz*sin_mu2 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) + F2_lz*sin_mu2*sq_ga2_2*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F2*sq_ga2_2*(lx*cos_mu2 - ly*sin_mu2) + 4*F2*ga2*lz*cos_mu2 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)))/(8*K + 2*sq_F2*sq_lx*pow(sq_ga2_2,2) + 2*sq_F2*sq_lz*pow(sq_ga2_2,2) - F2*sq_ga2_2*(ly*cos_mu2 + lx*sin_mu2)*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F2*sq_ga2_2*(ly*cos_mu2 + lx*sin_mu2) + 4*F2*ga2*lz*sin_mu2 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) - F2*sq_ga2_2*(lx*cos_mu2 - ly*sin_mu2)*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F2*sq_ga2_2*(lx*cos_mu2 - ly*sin_mu2) + 4*F2*ga2*lz*cos_mu2 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)));
		th2 = saturate(th2, outer_lower, outer_upper);
		sq_th2 = th2 * th2;
		sq_th2_2 = sq_th2 - 2;
		sq_th2_2_1 = sq_th2/2.0 - 1;

		ga2 = -(4*F2_ly*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3) + 2*F2_lz*sin_mu2*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - 4*F2_lz*th2*cos_mu2 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) + 2*F2_lz*cos_mu2*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) + 4*F2_lz*th2*sin_mu2 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)))/(8*K - (F2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - 2*F2_lz*th2*cos_mu2)*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F3_lz*(th3*cos_mu3*sq_ga3 + 2*sin_mu3*ga3 - 2*th3*cos_mu3) + 2*F2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2) - 4*F2_lz*th2*cos_mu2 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3)) - (F2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) + 2*F2_lz*th2*sin_mu2)*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F3_lz*(- th3*sin_mu3*sq_ga3 + 2*cos_mu3*ga3 + 2*th3*sin_mu3) + 2*F2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2) + 4*F2_lz*th2*sin_mu2 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F3*sq_ga3_2*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3)) + 8*sq_F2*sq_ly + 8*sq_F2*sq_lz - 4*F2_lx*th2*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F3_lx*th3*sq_ga3 + 2*F3_ly*ga3 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3));
		ga2 = saturate(ga2, inner_lower, inner_upper);
		sq_ga2 = ga2 * ga2;
		sq_ga2_2 = sq_ga2 - 2;
		sq_ga2_2_1 = sq_ga2/2.0 - 1;

		th3 = (2*F3_lx*sq_ga3_2*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 + 2*reqz + 2*F3*ga3*ly + 2*F1_lx*th1 + 2*F2_lx*th2) - F3_lz*cos_mu3*sq_ga3_2*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3*sq_ga3_2*(ly*cos_mu3 + lx*sin_mu3) + 4*F3*ga3*lz*sin_mu3 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2)) + F3_lz*sin_mu3*sq_ga3_2*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) + 2*F3*sq_ga3_2*(lx*cos_mu3 - ly*sin_mu3) + 4*F3*ga3*lz*cos_mu3 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2)))/(8*K + 2*sq_F3*sq_lx*pow(sq_ga3_2,2) + 2*sq_F3*sq_lz*pow(sq_ga3_2,2) - F3*sq_ga3_2*(ly*cos_mu3 + lx*sin_mu3)*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3*sq_ga3_2*(ly*cos_mu3 + lx*sin_mu3) + 4*F3*ga3*lz*sin_mu3 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2)) - F3*sq_ga3_2*(lx*cos_mu3 - ly*sin_mu3)*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) + 2*F3*sq_ga3_2*(lx*cos_mu3 - ly*sin_mu3) + 4*F3*ga3*lz*cos_mu3 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2)));
		th3 = saturate(th3, outer_lower, outer_upper);
		sq_th3 = th3 * th3;
		sq_th3_2 = sq_th3 - 2;
		sq_th3_2_1 = sq_th3/2.0 - 1;

		ga3 = -(4*F3_ly*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3) + 2*F3_lz*sin_mu3*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3) - 4*F3_lz*th3*cos_mu3 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2)) + 2*F3_lz*cos_mu3*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) + 2*F3*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3) + 4*F3_lz*th3*sin_mu3 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2)))/(8*K - (F3*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3) - 2*F3_lz*th3*cos_mu3)*(4*reqx + 2*F1_lz*(th1*cos_mu1*sq_ga1 + 2*sin_mu1*ga1 - 2*th1*cos_mu1) + 2*F2_lz*(th2*cos_mu2*sq_ga2 + 2*sin_mu2*ga2 - 2*th2*cos_mu2) + 2*F3*sq_th3_2*(ly*cos_mu3 + lx*sin_mu3) - 4*F3_lz*th3*cos_mu3 - F1*sq_ga1_2*sq_th1_2*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(ly*cos_mu2 + lx*sin_mu2)) - (F3*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3) + 2*F3_lz*th3*sin_mu3)*(2*F1_lz*(- th1*sin_mu1*sq_ga1 + 2*cos_mu1*ga1 + 2*th1*sin_mu1) - 4*reqy + 2*F2_lz*(- th2*sin_mu2*sq_ga2 + 2*cos_mu2*ga2 + 2*th2*sin_mu2) + 2*F3*sq_th3_2*(lx*cos_mu3 - ly*sin_mu3) + 4*F3_lz*th3*sin_mu3 - F1*sq_ga1_2*sq_th1_2*(lx*cos_mu1 - ly*sin_mu1) - F2*sq_ga2_2*sq_th2_2*(lx*cos_mu2 - ly*sin_mu2)) + 8*sq_F3*sq_ly + 8*sq_F3*sq_lz - 4*F3_lx*th3*(- F1_lx*th1*sq_ga1 + 2*F1_ly*ga1 - F2_lx*th2*sq_ga2 + 2*F2_ly*ga2 + 2*reqz + 2*F1_lx*th1 + 2*F2_lx*th2 + 2*F3_lx*th3));
		ga3 = saturate(ga3, inner_lower, inner_upper);



	} else { //DOES GRADIENT DESCENT WITH RMS PROP AND MOMENTUM

		static const float step = 1e-3; //gradient descent learning rate
		static const float eps = 1e-3; //used to avoid division by zero in RMS prop
		static const float decay = 0.5; //momentum decay. How much "velocity" from the previous step is carried to the current step

		float grad_th1 =                                                          2*K*th1 - F1*sq_ga1_2*(ly*th1*cos_mu1 - lz*cos_mu1 + lx*th1*sin_mu1)*(reqx + F1_lz*((th1*cos_mu1*sq_ga1)/2 + sin_mu1*ga1 - th1*cos_mu1) + F2_lz*((th2*cos_mu2*sq_ga2)/2 + sin_mu2*ga2 - th2*cos_mu2) + F3_lz*((th3*cos_mu3*sq_ga3)/2 + sin_mu3*ga3 - th3*cos_mu3) - F1*sq_ga1_2_1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2_1*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2_1*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3)) + F1*sq_ga1_2*(lz*sin_mu1 + lx*th1*cos_mu1 - ly*th1*sin_mu1)*(reqy - F1_lz*(ga1*cos_mu1 + th1*sin_mu1 - (sq_ga1*th1*sin_mu1)/2) - F2_lz*(ga2*cos_mu2 + th2*sin_mu2 - (sq_ga2*th2*sin_mu2)/2) - F3_lz*(ga3*cos_mu3 + th3*sin_mu3 - (sq_ga3*th3*sin_mu3)/2) + F1*sq_ga1_2_1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1) + F2*sq_ga2_2_1*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2) + F3*sq_ga3_2_1*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3)) - F1_lx*sq_ga1_2*(reqz + F1*ga1*ly + F2*ga2*ly + F3*ga3*ly + F1_lx*th1 + F2_lx*th2 + F3_lx*th3 - (F1*sq_ga1*lx*th1)/2 - (F2*sq_ga2*lx*th2)/2 - (F3*sq_ga3*lx*th3)/2);
		float grad_ga1 = 2*K*ga1 - 2*(F1_lz*(cos_mu1 - ga1*th1*sin_mu1) - F1*ga1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1))*(reqy - F1_lz*(ga1*cos_mu1 + th1*sin_mu1 - (sq_ga1*th1*sin_mu1)/2) - F2_lz*(ga2*cos_mu2 + th2*sin_mu2 - (sq_ga2*th2*sin_mu2)/2) - F3_lz*(ga3*cos_mu3 + th3*sin_mu3 - (sq_ga3*th3*sin_mu3)/2) + F1*sq_ga1_2_1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1) + F2*sq_ga2_2_1*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2) + F3*sq_ga3_2_1*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3)) + 2*(F1_lz*(sin_mu1 + ga1*th1*cos_mu1) - F1*ga1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1))*(reqx + F1_lz*((th1*cos_mu1*sq_ga1)/2 + sin_mu1*ga1 - th1*cos_mu1) + F2_lz*((th2*cos_mu2*sq_ga2)/2 + sin_mu2*ga2 - th2*cos_mu2) + F3_lz*((th3*cos_mu3*sq_ga3)/2 + sin_mu3*ga3 - th3*cos_mu3) - F1*sq_ga1_2_1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2_1*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2_1*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3)) + 2*F1*(ly - ga1*lx*th1)*(reqz + F1*ga1*ly + F2*ga2*ly + F3*ga3*ly + F1_lx*th1 + F2_lx*th2 + F3_lx*th3 - (F1*sq_ga1*lx*th1)/2 - (F2*sq_ga2*lx*th2)/2 - (F3*sq_ga3*lx*th3)/2);
		float grad_th2 =                                                          2*K*th2 - F2*sq_ga2_2*(ly*th2*cos_mu2 - lz*cos_mu2 + lx*th2*sin_mu2)*(reqx + F1_lz*((th1*cos_mu1*sq_ga1)/2 + sin_mu1*ga1 - th1*cos_mu1) + F2_lz*((th2*cos_mu2*sq_ga2)/2 + sin_mu2*ga2 - th2*cos_mu2) + F3_lz*((th3*cos_mu3*sq_ga3)/2 + sin_mu3*ga3 - th3*cos_mu3) - F1*sq_ga1_2_1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2_1*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2_1*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3)) + F2*sq_ga2_2*(lz*sin_mu2 + lx*th2*cos_mu2 - ly*th2*sin_mu2)*(reqy - F1_lz*(ga1*cos_mu1 + th1*sin_mu1 - (sq_ga1*th1*sin_mu1)/2) - F2_lz*(ga2*cos_mu2 + th2*sin_mu2 - (sq_ga2*th2*sin_mu2)/2) - F3_lz*(ga3*cos_mu3 + th3*sin_mu3 - (sq_ga3*th3*sin_mu3)/2) + F1*sq_ga1_2_1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1) + F2*sq_ga2_2_1*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2) + F3*sq_ga3_2_1*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3)) - F2_lx*sq_ga2_2*(reqz + F1*ga1*ly + F2*ga2*ly + F3*ga3*ly + F1_lx*th1 + F2_lx*th2 + F3_lx*th3 - (F1*sq_ga1*lx*th1)/2 - (F2*sq_ga2*lx*th2)/2 - (F3*sq_ga3*lx*th3)/2);
		float grad_ga2 = 2*K*ga2 - 2*(F2_lz*(cos_mu2 - ga2*th2*sin_mu2) - F2*ga2*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2))*(reqy - F1_lz*(ga1*cos_mu1 + th1*sin_mu1 - (sq_ga1*th1*sin_mu1)/2) - F2_lz*(ga2*cos_mu2 + th2*sin_mu2 - (sq_ga2*th2*sin_mu2)/2) - F3_lz*(ga3*cos_mu3 + th3*sin_mu3 - (sq_ga3*th3*sin_mu3)/2) + F1*sq_ga1_2_1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1) + F2*sq_ga2_2_1*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2) + F3*sq_ga3_2_1*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3)) + 2*(F2_lz*(sin_mu2 + ga2*th2*cos_mu2) - F2*ga2*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2))*(reqx + F1_lz*((th1*cos_mu1*sq_ga1)/2 + sin_mu1*ga1 - th1*cos_mu1) + F2_lz*((th2*cos_mu2*sq_ga2)/2 + sin_mu2*ga2 - th2*cos_mu2) + F3_lz*((th3*cos_mu3*sq_ga3)/2 + sin_mu3*ga3 - th3*cos_mu3) - F1*sq_ga1_2_1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2_1*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2_1*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3)) + 2*F2*(ly - ga2*lx*th2)*(reqz + F1*ga1*ly + F2*ga2*ly + F3*ga3*ly + F1_lx*th1 + F2_lx*th2 + F3_lx*th3 - (F1*sq_ga1*lx*th1)/2 - (F2*sq_ga2*lx*th2)/2 - (F3*sq_ga3*lx*th3)/2);
		float grad_th3 =                                                          2*K*th3 - F3*sq_ga3_2*(ly*th3*cos_mu3 - lz*cos_mu3 + lx*th3*sin_mu3)*(reqx + F1_lz*((th1*cos_mu1*sq_ga1)/2 + sin_mu1*ga1 - th1*cos_mu1) + F2_lz*((th2*cos_mu2*sq_ga2)/2 + sin_mu2*ga2 - th2*cos_mu2) + F3_lz*((th3*cos_mu3*sq_ga3)/2 + sin_mu3*ga3 - th3*cos_mu3) - F1*sq_ga1_2_1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2_1*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2_1*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3)) + F3*sq_ga3_2*(lz*sin_mu3 + lx*th3*cos_mu3 - ly*th3*sin_mu3)*(reqy - F1_lz*(ga1*cos_mu1 + th1*sin_mu1 - (sq_ga1*th1*sin_mu1)/2) - F2_lz*(ga2*cos_mu2 + th2*sin_mu2 - (sq_ga2*th2*sin_mu2)/2) - F3_lz*(ga3*cos_mu3 + th3*sin_mu3 - (sq_ga3*th3*sin_mu3)/2) + F1*sq_ga1_2_1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1) + F2*sq_ga2_2_1*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2) + F3*sq_ga3_2_1*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3)) - F3_lx*sq_ga3_2*(reqz + F1*ga1*ly + F2*ga2*ly + F3*ga3*ly + F1_lx*th1 + F2_lx*th2 + F3_lx*th3 - (F1*sq_ga1*lx*th1)/2 - (F2*sq_ga2*lx*th2)/2 - (F3*sq_ga3*lx*th3)/2);
		float grad_ga3 = 2*K*ga3 - 2*(F3_lz*(cos_mu3 - ga3*th3*sin_mu3) - F3*ga3*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3))*(reqy - F1_lz*(ga1*cos_mu1 + th1*sin_mu1 - (sq_ga1*th1*sin_mu1)/2) - F2_lz*(ga2*cos_mu2 + th2*sin_mu2 - (sq_ga2*th2*sin_mu2)/2) - F3_lz*(ga3*cos_mu3 + th3*sin_mu3 - (sq_ga3*th3*sin_mu3)/2) + F1*sq_ga1_2_1*sq_th1_2_1*(lx*cos_mu1 - ly*sin_mu1) + F2*sq_ga2_2_1*sq_th2_2_1*(lx*cos_mu2 - ly*sin_mu2) + F3*sq_ga3_2_1*sq_th3_2_1*(lx*cos_mu3 - ly*sin_mu3)) + 2*(F3_lz*(sin_mu3 + ga3*th3*cos_mu3) - F3*ga3*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3))*(reqx + F1_lz*((th1*cos_mu1*sq_ga1)/2 + sin_mu1*ga1 - th1*cos_mu1) + F2_lz*((th2*cos_mu2*sq_ga2)/2 + sin_mu2*ga2 - th2*cos_mu2) + F3_lz*((th3*cos_mu3*sq_ga3)/2 + sin_mu3*ga3 - th3*cos_mu3) - F1*sq_ga1_2_1*sq_th1_2_1*(ly*cos_mu1 + lx*sin_mu1) - F2*sq_ga2_2_1*sq_th2_2_1*(ly*cos_mu2 + lx*sin_mu2) - F3*sq_ga3_2_1*sq_th3_2_1*(ly*cos_mu3 + lx*sin_mu3)) + 2*F3*(ly - ga3*lx*th3)*(reqz + F1*ga1*ly + F2*ga2*ly + F3*ga3*ly + F1_lx*th1 + F2_lx*th2 + F3_lx*th3 - (F1*sq_ga1*lx*th1)/2 - (F2*sq_ga2*lx*th2)/2 - (F3*sq_ga3*lx*th3)/2);

		V[0] = b*V[0] + (1-b)*pow(grad_th1,2);
		V[1] = b*V[1] + (1-b)*pow(grad_ga1,2);
		V[2] = b*V[2] + (1-b)*pow(grad_th2,2);
		V[3] = b*V[3] + (1-b)*pow(grad_ga2,2);
		V[4] = b*V[4] + (1-b)*pow(grad_th3,2);
		V[5] = b*V[5] + (1-b)*pow(grad_ga3,2);

		th1 = th1 + dx[0]*decay - step*grad_th1/(sqrt(V[0])+eps);
		ga1 = ga1 + dx[1]*decay - step*grad_ga1/(sqrt(V[1])+eps);
		th2 = th2 + dx[2]*decay - step*grad_th2/(sqrt(V[2])+eps);
		ga2 = ga2 + dx[3]*decay - step*grad_ga2/(sqrt(V[3])+eps);
		th3 = th3 + dx[4]*decay - step*grad_th3/(sqrt(V[4])+eps);
		ga3 = ga3 + dx[5]*decay - step*grad_ga3/(sqrt(V[5])+eps);

		th1 = saturate(th1, outer_lower, outer_upper);
		th2 = saturate(th2, outer_lower, outer_upper);
		th3 = saturate(th3, outer_lower, outer_upper);
		ga1 = saturate(ga1, inner_lower, inner_upper);
		ga2 = saturate(ga2, inner_lower, inner_upper);
		ga3 = saturate(ga3, inner_lower, inner_upper);
	}

	dx[0] = th1 - old_angles[0];
	dx[1] =	ga1 - old_angles[1];
	dx[2] =	th2 - old_angles[2];
	dx[3] =	ga2 - old_angles[3];
	dx[4] =	th3 - old_angles[4];
	dx[5] =	ga3 - old_angles[5];

	thetas[0] = th1;
	thetas[1] = th2;
	thetas[2] = th3;
	gammas[0] = ga1;
	gammas[1] = ga2;
	gammas[2] = ga3;

	*diff_pointer = diff;
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void writeServos(float *thetas, float *gammas, TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim3) {
	//assignments
	float th1 = thetas[0];
	float th2 = thetas[1];
	float th3 = thetas[2];
	float ga1 = gammas[0];
	float ga2 = gammas[1];
	float ga3 = gammas[2];

	//theta refers to outer axis of rotation of the rocket motor
	//gamma refers to inner axis
	//they are the angle at wich the motor is rotated from its vertical orientation

	//RAD TO DEG
	//we first convert them to degrees
	float th1_temp = th1/PI*180.0f;
	float th2_temp = th2/PI*180.0f;
	float th3_temp = th3/PI*180.0f;
	float ga1_temp = ga1/PI*180.0f;
	float ga2_temp = ga2/PI*180.0f;
	float ga3_temp = ga3/PI*180.0f;

	//MOTOR ANGLE TO SERVO ANGLE
	//then we map them to their servo angle postion, obtained from a polyfit of the exact mechanism trig function
	th1_temp = theta2servo(th1_temp);
	th2_temp = theta2servo(th2_temp);
	th3_temp = theta2servo(th3_temp);
	ga1_temp = gamma2servo(ga1_temp);
	ga2_temp = gamma2servo(ga2_temp);
	ga3_temp = gamma2servo(ga3_temp);

	//OLD
	//SERVO ANGLE TO PULSE DURATION
	//By calibrating the servos I got:
	//microseconds pulse duration associated to 0 degrees and to 90 degrees position, respectively
	//				|  0 deg| 90 deg|
	//-------------------------------
	//outer 1(th1)	|	583 |1561	| micro-seconds pulse duration
	//inner 1(ga1)	|	622 |1617	|
	//outer 2(th2)	|	715 |1785	|
	//inner 2(ga2)	|	550 |1525	|
	//outer 3(th3)	|	649 |1669	|
	//inner 3(ga3)	|	649 |1669	|

	//th1_temp = mapFloat(th1_temp, 0.0f, 90.0f, 583.0f, 1561.0f);
	//ga1_temp = mapFloat(ga1_temp, 0.0f, 90.0f, 622.0f, 1617.0f);
	//th2_temp = mapFloat(th2_temp, 0.0f, 90.0f, 715.0f, 1785.0f);
	//ga2_temp = mapFloat(ga2_temp, 0.0f, 90.0f, 550.0f, 1525.0f);
	//th3_temp = mapFloat(th3_temp, 0.0f, 90.0f, 649.0f, 1669.0f);
	//ga3_temp = mapFloat(ga3_temp, 0.0f, 90.0f, 649.0f, 1669.0f);

	//NEW
	//redone with new servos bought after the first static test on gyroscopic mount
	//outer 1(th1)		600 1550
	//inner 1(ga1)		680 1660
	//outer 2(th2)		650 1660
	//inner 2(ga2)		580 1550
	//outer 3(th3)		730 1680
	//inner 3(ga3)		550 1520
	th1_temp = mapFloat(th1_temp, 0.0f, 90.0f, 600.0f, 1550.0f);
	ga1_temp = mapFloat(ga1_temp, 0.0f, 90.0f, 680.0f, 1660.0f);
	th2_temp = mapFloat(th2_temp, 0.0f, 90.0f, 650.0f, 1660.0f);
	ga2_temp = mapFloat(ga2_temp, 0.0f, 90.0f, 580.0f, 1550.0f);
	th3_temp = mapFloat(th3_temp, 0.0f, 90.0f, 730.0f, 1680.0f);
	ga3_temp = mapFloat(ga3_temp, 0.0f, 90.0f, 550.0f, 1520.0f);

	//OLD
	//SERVOS MUST BE CONNECTED BOTTOM TO TOP (1 below, 2 middle, 3 top), SO THAT THE TIMER+CHANNEL CORRESPONDS TO THE RIGHT SERVO
	//__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_1, th1_temp);
	//__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_2, ga1_temp);
	//__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_3, th2_temp);
	//__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_4, ga2_temp);
	//__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, th3_temp);
	//__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_2, ga3_temp);

	//NEW
	//ordered bottom to top
	__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_1, ga3_temp);
	__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_2, th3_temp);
	__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_3, ga2_temp);
	__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_4, th2_temp);
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, ga1_temp);
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_2, th1_temp);
}

/*
██╗  ██╗ █████╗ ██╗     ███╗   ███╗ █████╗ ███╗   ██╗
██║ ██╔╝██╔══██╗██║     ████╗ ████║██╔══██╗████╗  ██║
█████╔╝ ███████║██║     ██╔████╔██║███████║██╔██╗ ██║
██╔═██╗ ██╔══██║██║     ██║╚██╔╝██║██╔══██║██║╚██╗██║
██║  ██╗██║  ██║███████╗██║ ╚═╝ ██║██║  ██║██║ ╚████║
╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝╚═╝     ╚═╝╚═╝  ╚═╝╚═╝  ╚═══╝
*/

float get_accelerometer_variance(SPI_HandleTypeDef *hspi, float *local_acc0, int n_cycles) {
	//this function gives the variance of the z axis accelerometer. The reading on the sensor is not optimized for
	//reading just that, and instead reads everything (could be changed)
	uint8_t local_tagBuff[15] = {};
	float local_acc[3] = {};
	float local_acc_raw[3] = {};
	float acc_values[n_cycles];
	float sum = 0;

	for (int i=0; i<n_cycles; i++) {
		//reads accelerometer
		uint8_t has_read = 0;
		while (has_read == 0) { //checks if reading has happened in this current "for cycle"
			if (is_SPI3_done()) { //if SPI3 is available (dma done) it tries to read. Otherwise it retries
				IMU_readTempAccGyro(hspi, local_tagBuff);
				get_acc(local_tagBuff, local_acc0, local_acc, local_acc_raw);
				acc_values[i] = local_acc[3];
				has_read = 1; //changes flag
			}
		}

		sum += local_acc[3];
	}

	float average = sum/n_cycles;

	sum = 0; //resets sum to reuse it for other purposes
	for (int i=0; i<n_cycles; i++) {
		sum += pow(acc_values[i]-average,2);
	}
	//return the variance
	return (sum/n_cycles);

}

float get_barometer_variance(SPI_HandleTypeDef *hspi, int n_cycles) {
	//The variance will be calculate directly from altitude values rather than in pressure values
	//This is done to make the construction of the Kalman filter easier
	uint8_t ptBuff[6] = {};
	float new_alt;
	float alt_values[n_cycles];
	float sum = 0;

	BAR_readPressureTemp(hspi, ptBuff);

	for (int i=0; i<n_cycles; i++) {
		//reads barometer
		uint8_t has_read = 0;
		while (has_read == 0) { //checks if reading has happened in this current "for cycle"
			if (is_SPI3_done()) { //if SPI3 is available (dma done) it tries to read. Otherwise it retries
				BAR_readPressureTemp(hspi, ptBuff);
				float pressure = get_press(ptBuff);
				new_alt = get_bar_alt(pressure);
				alt_values[i] = new_alt;
				has_read = 1; //changes flag
			}
		}

		sum += new_alt;
	}

	float average = sum/n_cycles;

	sum = 0; //resets sum to reuse it for other purposes
	for (int i=0; i<n_cycles; i++) {
		sum += pow(alt_values[i]-average,2);
	}
	//return the variance
	return (sum/n_cycles);
}

float filter_altitude(float bar_alt, float az_earth, uint32_t micro_elaps) {
	static float p0  = 1;//initial estimate covariance
	static float p1  = 0.1;
	static float p10 = 0; //non diagonal term of P, assuming P is simmetric
	const float q0 = 1e-4; //model noise
	const float q1 = 1e-4;
	const float r = 1000;//measurement noise. measured 0.02 for the altitude gotten from the barometer

	static float x0;
	static uint8_t initialized = 0;
	if (!initialized) {
		x0 = bar_alt;
		initialized = 1;
	}
	static float x1 = 0;

	float dt = micro_elaps / 1e6;
	float u = az_earth;
	float z = bar_alt;

	float x0_est = x0 + x1*dt + u*0.5*dt*dt;
	float x1_est = x1 + u*dt;

	float p0_new  = p0  + q0 + dt*p10 + dt*(p10 + dt*p1);
	float p10_new = p10 + dt*p1;
	float p1_new  = p1  + q1;
	p0  = p0_new;
	p10 = p10_new;
	p1  = p1_new;

	static float k0;
	static float k1;
	k0 = p0  / (p0 + r);
	k1 = p10 / (p0 + r);

	x0 = x0_est - k0*(x0_est - z);
	x1 = x1_est - k1*(x0_est - z);

	p0_new  = p0  * (1-k0);
	p10_new = p10 * (1-k0);
	p1_new  = p1 - k1*p10;
	p0  = p0_new;
	p10 = p10_new;
	p1  = p1_new;

	return x0;
}

#define window_length 5
void moving_average_gyro(float *gyro) {

	static float buff0[window_length] = {};
	static float buff1[window_length] = {};
	static float buff2[window_length] = {};
	static uint8_t counter = 0;
	static float sum[3] = {};

	sum[0] -= buff0[counter];
	sum[1] -= buff1[counter];
	sum[2] -= buff2[counter];

	buff0[counter] = gyro[0];
	buff1[counter] = gyro[1];
	buff2[counter] = gyro[2];

	sum[0] += buff0[counter];
	sum[1] += buff1[counter];
	sum[2] += buff2[counter];

	counter+=1;
	if (counter >= window_length) counter = 0;

	gyro[0] = sum[0] / window_length;
	gyro[1] = sum[1] / window_length;
	gyro[2] = sum[2] / window_length;
}

void moving_average_acc(float *acc) {

	static float buff0[window_length] = {};
	static float buff1[window_length] = {};
	static float buff2[window_length] = {};
	static uint8_t counter = 0;
	static float sum[3] = {};

	sum[0] -= buff0[counter];
	sum[1] -= buff1[counter];
	sum[2] -= buff2[counter];

	buff0[counter] = acc[0];
	buff1[counter] = acc[1];
	buff2[counter] = acc[2];

	sum[0] += buff0[counter];
	sum[1] += buff1[counter];
	sum[2] += buff2[counter];

	counter+=1;
	if (counter >= window_length) counter = 0;

	acc[0] = sum[0] / window_length;
	acc[1] = sum[1] / window_length;
	acc[2] = sum[2] / window_length;
}

#endif
