/*
 * trinityfunctions.h
 *
 *  Created on: Jun 6, 2025
 *      Author: Admin
 */

#ifndef INC_TRINITYFUNCTIONS_H_
#define INC_TRINITYFUNCTIONS_H_

#include <stdint.h>
#include <main.h>

//get micros tick
uint32_t my_micros(TIM_HandleTypeDef *htim);

//Initializations
void initIMU(SPI_HandleTypeDef *hspi, uint8_t *tagBuff);
void initBar(SPI_HandleTypeDef *hspi, uint8_t *ptBuff);
void initSD();
void initFlash();

//----------------------------------------------------------------------------------------------
//write servo angles and impose angle limits
void WriteOuter1(TIM_HandleTypeDef *htim, float theta2servo);
void WriteInner1(TIM_HandleTypeDef *htim, float gamma2servo);

void WriteOuter2(TIM_HandleTypeDef *htim, float theta2servo);
void WriteInner2(TIM_HandleTypeDef *htim, float gamma2servo);

void WriteOuter3(TIM_HandleTypeDef *htim, float theta2servo);
void WriteInner3(TIM_HandleTypeDef *htim, float gamma2servo);

void  WriteChute(TIM_HandleTypeDef *htim, float alpha);
//----------------------------------------------------------------------------------------------

//SPI availability
uint8_t is_SPI3_done();
void set_SPI3_availability(uint8_t var);
uint8_t is_SPI1_done();
void set_SPI1_availability(uint8_t var);

//SPI3 readings
void read_IMU_Bar(SPI_HandleTypeDef *hspi, uint8_t *tagBuff, uint8_t *ptBuff);

//IMU reading
//void IMU_readGyro(uint8_t *gBuff, int size);
//void IMU_readAcc(uint8_t *aBuff, int size);
//void IMU_readAccGyro(uint8_t *agBuff, int size);
uint8_t IMU_readTempAccGyro(SPI_HandleTypeDef *hspi, uint8_t *tagBuff);
void IMU_Calibration(SPI_HandleTypeDef *hspi, float *gyro_offset, float *acc0, int n_cylces);

//Barometer reading
uint8_t BAR_readPressureTemp(SPI_HandleTypeDef *hspi, uint8_t *ptBuff);
void BAR_readTemp(SPI_HandleTypeDef *hspi, uint8_t *tempBuff);
void BAR_readPress(SPI_HandleTypeDef *hspi, uint8_t *pressBuff);

//ADC readings
uint8_t is_ADC_done();
void ADC_to_voltages(uint16_t *ADCbuff, float *voltages);

//GPIO readings
//void readUmbilical(uint8_t *umbilicalstatus);
uint8_t read_umbilical();

//Flash Memory
uint8_t is_flash_busy(SPI_HandleTypeDef *hspi);
void flash_WriteEnable(SPI_HandleTypeDef *hspi);
void sector_erase(SPI_HandleTypeDef *hspi, uint32_t address);
void block_erase(SPI_HandleTypeDef *hspi, uint32_t address);
void flash_program(uint8_t* Buf, SPI_HandleTypeDef *hspi);
uint32_t get_flash_add();
void fast_read_flash(uint8_t *RxBuf ,uint32_t data_byte_quantity, uint32_t address, SPI_HandleTypeDef *hspi);
void read_flash(uint8_t *RxBuf ,int data_byte_quantity, SPI_HandleTypeDef *hspi);

uint32_t float_to_bits(float var);
float bits_to_float(uint32_t var);
float round2(float val);
//----------------------------------------------------------------------------------------------
//MATHEMATICAL
/*
typedef struct{
	float sto[4];
} Quat;
Quat Quat_conj(Quat q);
Quat Quat_multiply(Quat q1, Quat q2);
*/
float my_cos(float angle);
float my_sin(float angle);
float saturate(float var, float min, float max);

void quat_normalize(float* q);
void quat_multiply(float *q1, float *q2, float *result);
void quat_conjugate(float *q, float *result);
void earth2body(float *q, float *vec, float *result);
void body2earth(float *q, float *vec, float *result);
void target2earth(float *q, float *vec, float *result);

float theta2servo(float angle); // outer axis of gimbal
float gamma2servo(float angle); // inner axis of gimbal
//CONTROL
void gyro2quat_integration(float *gyro, float *quat, uint32_t micro_elaps);//integrates angular velocity to get a quaternion of earth's frame wrt rocket's frame
void get_relative_quat(float *body_quat, float *target_quat, float *relative_quat);
void quat2axang(float *quat, float *axang);
void updateReqTorque(float *axang, float *gyro, float *target_gyro, float *body_quat, float *target_quat, float *ReqTorque, uint32_t micro_elaps);

void get_parabVertex_angles(float *thetas, float *gammas, float *Forces, float *ReqTorque);
void get_angles_RMS_and_paraboloid(float *thetas, float *gammas, float *Forces, float *ReqTorque);

void get_gyro(uint8_t *IMU_tag_buff, float *gyro_offset, float *gyro);//Incorporate gyro reading with calibration bias
void get_acc(uint8_t *IMU_tag_buff, float *acc0, float *acc, float *acc_raw);
void get_vel_pos(float *pos_earth, float *vel_earth, float *acc_earth, uint32_t micro_elaps);
void get_earth_acc(float *vec, float *q, float *result);
float get_press(uint8_t *Bar_pt_buff);
float get_bar_alt(uint8_t *Bar_pt_buff);
void inject_gyro(float *gyro);
void inject_servo_angles();

void writeServos(float *thetas, float *gammas, TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim3);

//KALMAN
float get_accelerometer_variance(SPI_HandleTypeDef *hspi, float *local_acc0, int n_cycles);
float get_barometer_variance(SPI_HandleTypeDef *hspi, int n_cycles);
float filter_altitude(float bar_alt, float az_earth, uint32_t micro_elaps);

#endif /* INC_TRINITYFUNCTIONS_H_ */
