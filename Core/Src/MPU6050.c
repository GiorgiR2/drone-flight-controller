/*
 * MPU_6050.c
 *
 *  Created on: Jan 3, 2024
 *      Author: giorgir
 */

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
//#include <stdlib.h>

extern I2C_HandleTypeDef hi2c1;
#define MPU_I2C &hi2c1

extern UART_HandleTypeDef huart2;
#define FEEDBACK_UART &huart2

#define TIMEOUT 100

#define GY521_ADDRESS 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define DLPF_CFG_REG 0x1A

void MPU6050_Init(void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I (check if device is responding)
	HAL_I2C_Mem_Read(MPU_I2C, GY521_ADDRESS, WHO_AM_I_REG, 1, &check, 1, 1000);

	if(check == 104) // if the device is present
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(MPU_I2C, GY521_ADDRESS, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		// Gyroscope Output Rate = 8khz
		// Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
		Data = 0x07;
		HAL_I2C_Mem_Write(MPU_I2C, GY521_ADDRESS, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		//(by 0x00) Full scale range; ± 2g; ± 250 °/s
		Data = 0x00; // 0x8 -> gyro / 65.5; accel /
		HAL_I2C_Mem_Write(MPU_I2C, GY521_ADDRESS, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);
		HAL_I2C_Mem_Write(MPU_I2C, GY521_ADDRESS, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

		Data = 0x05; // FIlter; Cutoff frequency of 10hz
		HAL_I2C_Mem_Write(MPU_I2C, GY521_ADDRESS, 0x1A, 1, &Data, 1, 1000);

		HAL_UART_Transmit(FEEDBACK_UART, (const uint8_t*)"MPU6050 Connected...\n\r", 22, TIMEOUT);
	}
	else{
		HAL_Delay(150);
		HAL_UART_Transmit(FEEDBACK_UART, (const uint8_t*)"retrying initialization\n\r", 25, TIMEOUT);
		MPU6050_Init();
	}
}

void MPU6050_read_accel(void)
{
	uint8_t Rec_Data[6];
	int16_t accel_X_raw;
	int16_t accel_Y_raw;
	int16_t accel_Z_raw;
	float accel_X = 0;
	float accel_Y = 0;
	float accel_Z = 0;

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
	HAL_I2C_Mem_Read (MPU_I2C, GY521_ADDRESS, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	accel_X_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	accel_Y_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	accel_Z_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	// get the float g
	accel_X = accel_X_raw / 16384.0;
	accel_Y = accel_Y_raw / 16384.0;
	accel_Z = accel_Z_raw / 16384.0;

	uint8_t pbuf[30];
	sprintf((char*)pbuf, "X %.03f ", accel_X);
	HAL_UART_Transmit(FEEDBACK_UART, pbuf, strlen((char*)pbuf), TIMEOUT);
	sprintf((char*)pbuf, "Y %.03f ", accel_Y);
	HAL_UART_Transmit(FEEDBACK_UART, pbuf, strlen((char*)pbuf), TIMEOUT);
	sprintf((char*)pbuf, "Z %.03f \n\r", accel_Z);
	HAL_UART_Transmit(FEEDBACK_UART, pbuf, strlen((char*)pbuf), TIMEOUT);
}

void MPU6050_read_gyro(void)
{
	uint8_t Rec_Data[6];
	int16_t gyro_X_raw;
	int16_t gyro_Y_raw;
	int16_t gyro_Z_raw;
	float gyro_X;
	float gyro_Y;
	float gyro_Z;

	// Read 6 BYTES of data starting from GYRO_XOUT_H register
	HAL_I2C_Mem_Read(MPU_I2C, GY521_ADDRESS, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	gyro_X_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	gyro_Y_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	gyro_Z_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	// get the float g
	gyro_X = gyro_X_raw / 131.0;
	gyro_Y = gyro_Y_raw / 131.0;
	gyro_Z = gyro_Z_raw / 131.0;

	uint8_t pbuf[30];
	sprintf((char*)pbuf, "X %.03f ", gyro_X);
	HAL_UART_Transmit(FEEDBACK_UART, pbuf, strlen((char*)pbuf), TIMEOUT);
	sprintf((char*)pbuf, "Y %.03f ", gyro_Y);
	HAL_UART_Transmit(FEEDBACK_UART, pbuf, strlen((char*)pbuf), TIMEOUT);
	sprintf((char*)pbuf, "Z %.03f \n\r", gyro_Z);
	HAL_UART_Transmit(FEEDBACK_UART, pbuf, strlen((char*)pbuf), TIMEOUT);
}

int16_t get_x_a()
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(MPU_I2C, GY521_ADDRESS, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	return (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
}

int16_t get_y_a()
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(MPU_I2C, GY521_ADDRESS, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	return (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
}

int16_t get_z_a()
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(MPU_I2C, GY521_ADDRESS, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	return (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
}

int16_t get_x_g()
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(MPU_I2C, GY521_ADDRESS, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	return (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
}

int16_t get_y_g()
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(MPU_I2C, GY521_ADDRESS, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	return (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
}

int16_t get_z_g()
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(MPU_I2C, GY521_ADDRESS, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	return (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
}
