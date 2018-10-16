#ifndef DVC_ADI16465_H_201804271518
#define DVC_ADI16465_H_201804271518
/**
 * @file dvc_ADI16465.c
 * @author Qiaoyy@xlnav.com
 * @date 2018-04-27
 * @brief  ADI 16465 Çý¶¯³ÌÐò
 * Copyright (c) 2018 XiLang Co. Ltd.
 * All rights reserved.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "Global.h"
/* Exported macro ------------------------------------------------------------*/
#define ADI_USE_SPI

// 16465 ¼Ä´æÆ÷±í
//  Reserved  0x00 to 0x01 // Reserved N/A 
#define  ADI_16465_DIAG_STAT  0x0203 // Output, system error flags R 
#define  ADI_16465_X_GYRO_LOW  0x0405 // Output, x-axis gyroscope, low word R 
#define  ADI_16465_X_GYRO_OUT  0x0607 // Output, x-axis gyroscope, high word R 
#define  ADI_16465_Y_GYRO_LOW  0x0809 // Output, y-axis gyroscope, low word R 
#define  ADI_16465_Y_GYRO_OUT  0x0A0B // Output, y-axis gyroscope, high word R 
#define  ADI_16465_Z_GYRO_LOW  0x0C0D // Output, z-axis gyroscope, low word R 
#define  ADI_16465_Z_GYRO_OUT  0x0E0F // Output, z-axis gyroscope, high word R 
#define  ADI_16465_X_ACCL_LOW  0x1011 // Output, x-axis accelerometer, low word R 
#define  ADI_16465_X_ACCL_OUT  0x1213 // Output, x-axis accelerometer, high word R 
#define  ADI_16465_Y_ACCL_LOW  0x1415 // Output, y-axis accelerometer, low word R 
#define  ADI_16465_Y_ACCL_OUT  0x1617 // Output, y-axis accelerometer, high word R 
#define  ADI_16465_Z_ACCL_LOW  0x1819 // Output, z-axis accelerometer, low word R 
#define  ADI_16465_Z_ACCL_OUT  0x1A1B // Output, z-axis accelerometer, high word R 
#define  ADI_16465_TEMP_OUT  0x1C1D // Output, temperature R 
#define  ADI_16465_TIME_STAMP  0x1E1F // Output, time stamp R 
//  ADI_16465_Reserved  0x2021 // Reserved N/A 
#define  ADI_16465_DATA_CNTR  0x2223 // New data counter R 
#define  ADI_16465_X_DELTANG_LOW  0x2425 // Output, x-axis delta angle, low word R 
#define  ADI_16465_X_DELTANG_OUT  0x2627 // Output, x-axis delta angle, high word R 
#define  ADI_16465_Y_DELTANG_LOW  0x2829 // Output, y-axis delta angle, low word R 
#define  ADI_16465_Y_DELTANG_OUT  0x2A2B // Output, y-axis delta angle, high word R 
#define  ADI_16465_Z_DELTANG_LOW  0x2C2D // Output, z-axis delta angle, low word R 
#define  ADI_16465_Z_DELTANG_OUT  0x2E2F // Output, z-axis delta angle, high word R 
#define  ADI_16465_X_DELTVEL_LOW  0x3031 // Output, x-axis delta velocity, low word R 
#define  ADI_16465_X_DELTVEL_OUT  0x3233 // Output, x-axis delta velocity, high word R 
#define  ADI_16465_Y_DELTVEL_LOW  0x3435 // Output, y-axis delta velocity, low word R 
#define  ADI_16465_Y_DELTVEL_OUT  0x3637 // Output, y-axis delta velocity, high word R 
#define  ADI_16465_Z_DELTVEL_LOW  0x3839 // Output, z-axis delta velocity, low word R 
#define  ADI_16465_Z_DELTVEL_OUT  0x3A3B // Output, z-axis delta velocity, high word R 
//  Reserved  0x3C to 0x3F // Reserved N/A 
#define  ADI_16465_XG_BIAS_LOW  0x4041 // Calibration, offset, gyroscope, x-axis, low word R/W 
#define  ADI_16465_XG_BIAS_HIGH  0x4243 // Calibration, offset, gyroscope, x-axis, high word R/W 
#define  ADI_16465_YG_BIAS_LOW  0x4445 // Calibration, offset, gyroscope, y-axis, low word R/W 
#define  ADI_16465_YG_BIAS_HIGH  0x4647 // Calibration, offset, gyroscope, y-axis, high word R/W 
#define  ADI_16465_ZG_BIAS_LOW  0x4849 // Calibration, offset, gyroscope, z-axis, low word R/W 
#define  ADI_16465_ZG_BIAS_HIGH  0x4A4B // Calibration, offset, gyroscope, z-axis, high word R/W 
#define  ADI_16465_XA_BIAS_LOW  0x4C4D // Calibration, offset, accelerometer, x-axis, low word R/W 
#define  ADI_16465_XA_BIAS_HIGH  0x4E4F // Calibration, offset, accelerometer, x-axis, high word R/W 
#define  ADI_16465_YA_BIAS_LOW  0x5051 // Calibration, offset, accelerometer, y-axis, low word R/W 
#define  ADI_16465_YA_BIAS_HIGH  0x5253 // Calibration, offset, accelerometer, y-axis, high word R/W 
#define  ADI_16465_ZA_BIAS_LOW  0x5455 // Calibration, offset, accelerometer, z-axis, low word R/W 
#define  ADI_16465_ZA_BIAS_HIGH  0x5657 // Calibration, offset, accelerometer, z-axis, high word R/W 
//  Reserved  0x585B // Reserved N/A 
#define  ADI_16465_FILT_CTRL  0x5C5D // Control, Bartlett window FIR filter R/W 
//  Reserved  0x5E5F // Reserved N/A 
#define  ADI_16465_MSC_CTRL  0x6061 // Control, input/output and other miscellaneous options R/W 
#define  ADI_16465_UP_SCALE  0x6263 // Control, scale factor for input clock, pulse per second (PPS) R/W  0x07D0
#define  ADI_16465_DEC_RATE  0x6465 // Control, decimation filter (output data rate) R/W 
#define  ADI_16465_NULL_CNFG  0x6667 // Control, bias estimation period R/W 
#define  ADI_16465_GLOB_CMD  0x6869 // Control, global commands W 
//  Reserved  0x6A 6B // Reserved N/A 
#define  ADI_16465_FIRM_REV  0x6C6D // Identification, firmware revision R 
#define  ADI_16465_FIRM_DM  0x6E6F // Identification, date code, day and month R 
#define  ADI_16465_FIRM_Y  0x7071 // Identification, date code, year R 
#define  ADI_16465_PROD_ID  0x7273 // Identification, part number R 0x4056
#define  ADI_16465_SERIAL_NUM  0x7475 // Identification, serial number R  
#define  ADI_16465_USER_SCR1  0x7677 // User Scratch Register 1 R/W 
#define  ADI_16465_USER_SCR2  0x7879 // User Scratch Register 2 R/W 
#define  ADI_16465_USER_SCR3  0x7A7B // User Scratch Register 3 R/W 
#define  ADI_16465_FLSHCNT_LOW  0x7C7D // Output, flash memory write cycle counter, lower word R 
#define  ADI_16465_FLSHCNT_HIGH  0x7E7E // Output, flash memory write cycle counter, upper word R 

/* Exported types ------------------------------------------------------------*/ 
typedef struct
{
	uint16_t usBuffer[14];
	//int16_t usBuffer[14];
	float fGyrX;
	float fGyrY;
	float fGyrZ;
	float fAccX;
	float fAccY;
	float fAccZ;
	float fTempretrue;
	unsigned char ucFrameNumber;
	uint16_t nProductID;
#ifdef ADI_USE_SPI
	SPI_HandleTypeDef *phspi;
	GPIO_TypeDef* NSS_GPIOx;
	uint16_t NSS_GPIO_Pin;
#endif	
	

} ADI_16465_HandleTypeDef;
/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int ADI_16465_ReadImu(ADI_16465_HandleTypeDef* pImu);
int ADI_16465_ReadDeviceNumber(ADI_16465_HandleTypeDef* pImu);
int ADI_16465_ParseImu(ADI_16465_HandleTypeDef* pImu);
int ADI_16465_Readreg(ADI_16465_HandleTypeDef* pImu);
int ADI_16465_LowFilter(ADI_16465_HandleTypeDef* pImu);


#endif
/************************ (C) COPYRIGHT XiLang CO. Ltd. *******END OF FILE*****/
